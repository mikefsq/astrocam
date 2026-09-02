package astrocam

import (
	"fmt"
	"strings"
	"sync"
)

// ZWO vendor request codes. Writes: OUT 0x40, wValue=reg, wIndex=val, no data stage.
// Reads: IN 0xC0, wValue=reg, 1-byte data.
//
//	WriteSONYREG         OUT 0x40 bReq 0xB6  wValue=reg wIndex=val8
//	ReadSONYREG          IN  0xC0 bReq 0xB7  wValue=reg  -> 1B
//	WriteCameraRegister  OUT 0x40 bReq 0xA6  wValue=reg wIndex=val16
//	WriteFPGAREG         OUT 0x40 bReq 0xBD  wValue=reg wIndex=val16
//	ReadFPGAREG          IN  0xC0 bReq 0xBC  wValue=reg  -> 1B
const (
	reqWriteSonyReg = 0xB6 // WriteSONYREG: sensor I2C register write (8-bit value)
	reqReadSonyReg  = 0xB7 // ReadSONYREG: sensor I2C register read (1-byte IN)
	reqWriteCamReg  = 0xA6 // WriteCameraRegister: generic 16-bit register write
	reqReadCamReg   = 0xA7 // generic 16-bit register read
	reqWriteFPGAReg = 0xBD // WriteFPGAREG: camera-FPGA register write (16-bit)
	reqReadFPGAReg  = 0xBC // ReadFPGAREG: camera-FPGA register read (1-byte IN)
	reqFirmwareVer  = 0xAD // read firmware version
	reqSerialNumber = 0xC8 // GetSerialNumber: read the 8-byte factory serial (ASI_ID)
	reqReadSPIFlash = 0xC3 // ReadSPIFlash: read camera config/calibration from SPI flash (IN)

	// SendCMD opcodes (vendor OUT, no register payload). The stream stop/start/flush trio lives
	// in capture.go (cmdStreamStop/Start/Flush).
	cmdST4N           = 0xB0 // ST4 guide on
	cmdST4F           = 0xB1 // ST4 guide off
	cmdEnableGPIF32DQ = 0xBE // EnableGPIF32DQ: enable the FPGA->FX3 32-bit data bus
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

// zwoRegmap implements Regmap over a Transport using the ZWO control-transfer protocol. bus
// picks the sensor-register request (Sony vs generic camera).
type zwoRegmap struct {
	t      Transport
	bus    RegBus
	modeMu sync.RWMutex
	mode   ReadoutMode // live readout context (USB speed, output depth, FPS%), set by Camera
}

// ReadoutMode implements modeReader (read under the mode lock).
func (r *zwoRegmap) ReadoutMode() ReadoutMode {
	r.modeMu.RLock()
	defer r.modeMu.RUnlock()
	return r.mode
}

// updateMode implements modeCarrier (mutate under the mode lock).
func (r *zwoRegmap) updateMode(f func(*ReadoutMode)) {
	r.modeMu.Lock()
	defer r.modeMu.Unlock()
	f(&r.mode)
}

// VID reports the ZWO vendor id (selects the ZWO gain/offset encoding).
func (r *zwoRegmap) VID() uint16 { return ZWO.VID }

// ZWO is the vendor descriptor for ZWO ASI cameras (USB VID 0x03C3). Registered at init.
var ZWO = &Vendor{
	VID:  0x03C3,
	Name: "ZWO",
	Cmds: FX3Cmds{
		StreamStop:      FX3Cmd{Req: cmdStreamStop},
		StreamStart:     FX3Cmd{Req: cmdStreamStart},
		Flush:           FX3Cmd{Req: cmdFlush},
		EnableGPIF32DQ:  cmdEnableGPIF32DQ,
		ReadSPIFlash:    reqReadSPIFlash,
		FirmwareVersion: reqFirmwareVer,
		SerialNumber:    reqSerialNumber,
		ST4:             FX3ST4{On: cmdST4N, Off: cmdST4F},
		ReadTemp:        reqReadTemp,
		TempC:           zwoTempC,
		ReadHumidity:    reqReadHumidity,

		ReadHumidityWValue: humidityWValue,
	},
	frameStart: zwoFrameStart,
	// Nothing follows a ZWO frame. Traced on an ASI6200MC free-running 2328x2280, 2344x1500 and
	// 1840x1146: every frame is exactly width×height×bpp, ending on a short transfer with no bytes
	// beyond the pixels.
	frameTrailer: 0,
	// ZWO's map is hardware-characterised: in a 20 s dark on a 6200MM, 95.6% of the frame's hot
	// pixels are in it, and its own pixels sit a median 2166 ADU above the pedestal.
	defectMapTrusted: true,
	newRegmap: func(t Transport, bus RegBus, mode ReadoutMode) Regmap {
		return &zwoRegmap{t: t, bus: bus, mode: mode}
	},
	fpgaRun:    zwoFPGARun,
	newThermal: zwoThermal,
}

// zwoFrameStart locates a frame boundary inside a buffer taken from the free-run byte stream, as
// a byte offset, or -1 when there is none to find. 0 means the buffer already begins on a frame.
//
// A DDR frame opens with the FX3 header word 0x00005A7E, so a buffer that starts on one shows it
// at offset 0. Anywhere else, a boundary is the previous frame's footer immediately followed by
// the next frame's header (fx3MarkerOffset), which needs both words and so cannot be tripped by a
// lone 0x7E 0x5A pair in sensor noise.
func zwoFrameStart(buf []byte) int {
	if len(buf) >= 2 && buf[0] == 0x7E && buf[1] == 0x5A {
		return 0
	}
	return fx3MarkerOffset(buf)
}

func init() { RegisterVendor(ZWO) }

func (r *zwoRegmap) writeReq() uint8 {
	if r.bus == BusCamera {
		return reqWriteCamReg
	}
	return reqWriteSonyReg
}

func (r *zwoRegmap) readReq() uint8 {
	if r.bus == BusCamera {
		return reqReadCamReg
	}
	return reqReadSonyReg
}

func (r *zwoRegmap) WriteReg(reg, val uint16) error {
	return r.t.ControlOut(r.writeReq(), reg, val, nil)
}

func (r *zwoRegmap) ReadReg(reg uint16) (uint16, error) {
	buf := make([]byte, 1) // ReadSONYREG returns 8-bit
	got, err := r.t.ControlIn(r.readReq(), reg, 0, buf)
	if err != nil {
		return 0, fmt.Errorf("read reg 0x%x: %w", reg, err)
	}
	if got < 1 {
		return 0, fmt.Errorf("read reg 0x%x: empty control-IN", reg)
	}
	return uint16(buf[0]), nil
}

// WriteRegBits is a read-modify-write of bits [lo:hi] (inclusive) of reg.
func (r *zwoRegmap) WriteRegBits(reg uint16, lo, hi uint8, val uint16) error {
	cur, err := r.ReadReg(reg)
	if err != nil {
		return err
	}
	mask := uint16(((1 << (hi - lo + 1)) - 1) << lo)
	cur = (cur &^ mask) | ((val << lo) & mask)
	return r.WriteReg(reg, cur)
}

// WriteFPGAReg writes a camera-FPGA register (request 0xBD).
func (r *zwoRegmap) WriteFPGAReg(reg, val uint16) error {
	return r.t.ControlOut(reqWriteFPGAReg, reg, val, nil)
}

// ReadFPGAReg reads a camera-FPGA register (request 0xBC).
func (r *zwoRegmap) ReadFPGAReg(reg uint16) (uint16, error) {
	buf := make([]byte, 1)
	got, err := r.t.ControlIn(reqReadFPGAReg, reg, 0, buf)
	if err != nil {
		return 0, fmt.Errorf("read FPGA reg 0x%x: %w", reg, err)
	}
	if got < 1 {
		return 0, fmt.Errorf("read FPGA reg 0x%x: empty control-IN", reg)
	}
	return uint16(buf[0]), nil
}

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

// FlashHPCMapAddr is the flash address of the factory hot/dead-pixel correction map blob.
// Layout: 2 KiB header with magic "ASID" (defect map; "ASIG" = gain map) and a big-endian
// uint32 payload length at offset 4, followed by a compressed 1-bit-per-pixel defect bitmap.
const FlashHPCMapAddr = 0x40000

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
