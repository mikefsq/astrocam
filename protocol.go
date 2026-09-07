package astrocam

// Vendor-neutral control plane: the register-bus selector every vendor's Regmap honours, the
// FPGA frame-length write, and the Camera-level operations that reach the bridge through the
// vendor's command table (Vendor.Cmds) rather than through a vendor's own request codes.
// ZWO's dialect is protocol_zwo.go, PlayerOne's protocol_poa.go.

import (
	"fmt"
	"strings"
)

// RegBus selects which vendor request a sensor's WriteReg/ReadReg map to: BusSony the Sony I2C
// path, BusCamera the generic camera-register path. FPGA access is a separate space.
type RegBus uint8

const (
	BusSony   RegBus = iota // WriteSONYREG / ReadSONYREG (0xB6 / 0xB7); the default
	BusCamera               // WriteCameraRegister (0xA6); non-Sony dies
)

// VMAX FPGA registers: a strobe write to FPGA reg 1, then the 24-bit frame length
// little-endian across regs 0x10/0x11/0x12.
const (
	fpgaVMAXStrobe = 0x01
	fpgaVMAX0      = 0x10
	fpgaVMAX1      = 0x11
	fpgaVMAX2      = 0x12
)

// SetVMAX programs the frame length (VMAX) into the camera FPGA: clamp to 24 bits, strobe FPGA
// reg 1, write VMAX little-endian to regs 0x10/0x11/0x12, then release the strobe (on error
// paths too, see FPGAWrite16; the first error wins).
func SetVMAX(rm Regmap, vmax uint32) (err error) {
	if vmax > 0xffffff {
		vmax = 0xffffff
	}
	if err = rm.WriteFPGAReg(fpgaVMAXStrobe, 1); err != nil {
		return err
	}
	defer func() {
		if rerr := rm.WriteFPGAReg(fpgaVMAXStrobe, 0); err == nil {
			err = rerr
		}
	}()
	for _, w := range []RegVal{
		{Reg: fpgaVMAX0, Val: uint16(vmax) & 0xff},
		{Reg: fpgaVMAX1, Val: uint16(vmax>>8) & 0xff},
		{Reg: fpgaVMAX2, Val: uint16(vmax>>16) & 0xff},
	} {
		if err = rm.WriteFPGAReg(w.Reg, w.Val); err != nil {
			return err
		}
	}
	return nil
}

// vendorCmd issues a SendCMD-style FX3 vendor command through the vendor's command table. The
// wValue comes from the table too, since PlayerOne selects stream start from stop with it rather
// than with a second request code.
func (c *Camera) vendorCmd(op FX3Op) error {
	cmd := c.vend.Cmds.cmd(op)
	if !cmd.decoded() {
		return fmt.Errorf("astrocam: FX3 %s not decoded for vendor %s", op, c.vend.Name)
	}
	return c.t.ControlOut(cmd.Req, cmd.WValue, 0, nil)
}

// vendorIn issues a vendor IN request from the command table (code 0 = not decoded).
func (c *Camera) vendorIn(code uint8, what string, wValue, wIndex uint16, data []byte) (int, error) {
	if code == 0 {
		return 0, fmt.Errorf("astrocam: FX3 %s not decoded for vendor %s", what, c.vend.Name)
	}
	return c.t.ControlIn(code, wValue, wIndex, data)
}

// ReadSPIFlash reads n bytes from the camera's SPI flash starting at addr, in 2 KiB vendor-IN
// blocks (wIndex tracks addr>>8). SPI flash and the sensor's 32-bit GPIF data bus share FX3
// pins, so the read is bracketed by EnableGPIF32DQ(false)/(true); the camera must be Init'd first.
func (c *Camera) ReadSPIFlash(addr uint32, n int) (out []byte, err error) {
	gpif, flash := c.vend.Cmds.EnableGPIF32DQ, c.vend.Cmds.ReadSPIFlash
	if flash == 0 {
		return nil, fmt.Errorf("astrocam: SPI flash read not decoded for vendor %s", c.vend.Name)
	}
	// The GPIF toggle is ZWO's: its flash shares the FX3 pins with the data bus, so the bus has
	// to be dropped for the read and put back afterwards. PlayerOne's does not, and a vendor that
	// declares no toggle simply reads.
	if gpif != 0 {
		if err := c.t.ControlOut(gpif, 0, 0, nil); err != nil {
			return nil, err
		}
		defer func() {
			// A data bus left disabled leaves the next readout dead, so a failed re-enable is
			// surfaced (a read error from the body wins if both fail).
			if rerr := c.t.ControlOut(gpif, 1, 0, nil); err == nil && rerr != nil {
				err = fmt.Errorf("astrocam: re-enable GPIF32DQ after flash read: %w", rerr)
			}
		}()
	}
	const block = 2048
	out = make([]byte, 0, n)
	for len(out) < n {
		want := n - len(out)
		if want > block {
			want = block
		}
		buf := make([]byte, want)
		got, err := c.t.ControlIn(flash, 0, uint16(addr>>8), buf)
		if err != nil {
			return out, err
		}
		out = append(out, buf[:got]...)
		if got < want {
			break
		}
		addr += uint32(got)
	}
	return out, nil
}

// FirmwareVersion reads the camera firmware version, little-endian over the vendor's reply
// length (2 bytes on ZWO, 1 on PlayerOne).
func (c *Camera) FirmwareVersion() (uint16, error) {
	buf := make([]byte, c.vend.Cmds.firmwareBytes())
	if _, err := c.vendorIn(c.vend.Cmds.FirmwareVersion, "FirmwareVersion", 0, 0, buf); err != nil {
		return 0, err
	}
	var v uint16
	for i := len(buf) - 1; i >= 0; i-- {
		v = v<<8 | uint16(buf[i])
	}
	return v, nil
}

// Serial is a camera's factory serial, rendered the way its vendor writes it. ZWO burns 8 raw
// bytes (the ASI_ID) that its SDK shows as hex; PlayerOne burns 20 printable ASCII characters
// (e.g. "CAMGF252416072209000"). Neither camera exposes a USB serial-number descriptor, so this
// is the only stable per-unit identifier.
type Serial string

func (s Serial) String() string { return string(s) }

// decodeSerial renders raw serial bytes per the vendor's convention: printable text as itself
// (trailing NULs and padding trimmed), raw bytes as lowercase hex.
func decodeSerial(raw []byte, ascii bool) Serial {
	if ascii {
		return Serial(strings.TrimRight(string(raw), "\x00 "))
	}
	const hex = "0123456789abcdef"
	b := make([]byte, 0, len(raw)*2)
	for _, c := range raw {
		b = append(b, hex[c>>4], hex[c&0xf])
	}
	return Serial(b)
}

// SerialNumber reads the factory serial: a single vendor control-IN transfer (bRequest 0xC8 on
// ZWO, 0xA3 on PlayerOne; wValue 0, wIndex 0) of the vendor's reply length.
func (c *Camera) SerialNumber() (Serial, error) {
	raw := make([]byte, c.vend.Cmds.serialBytes())
	if _, err := c.vendorIn(c.vend.Cmds.SerialNumber, "SerialNumber", 0, 0, raw); err != nil {
		return "", err
	}
	return decodeSerial(raw, c.vend.Cmds.SerialASCII), nil
}
