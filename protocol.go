package astrocam

import (
	"fmt"
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

	// SendCMD opcodes (vendor OUT, no register payload).
	cmdTEC            = 0xB2 // TEC drive
	cmdReset          = 0xAF // pipeline reset (drop recovery)
	cmdST4N           = 0xB0 // ST4 guide on
	cmdST4F           = 0xB1 // ST4 guide off
	cmdFPGABankSelect = 0xAE // FPGA bank select, issued at the end of InitCamera
	cmdEnableGPIF32DQ = 0xBE // EnableGPIF32DQ: enable the FPGA->FX3 32-bit data bus
)

// RegBus selects which vendor request a sensor's WriteReg/ReadReg map to: BusSony
// the Sony I2C path, BusCamera the generic camera-register path. FPGA access is a
// separate space (see WriteFPGAReg).
type RegBus uint8

const (
	BusSony   RegBus = iota // WriteSONYREG / ReadSONYREG (0xB6 / 0xB7) — default
	BusCamera               // WriteCameraRegister (0xA6) — non-Sony dies
)

// VMAX FPGA registers: a strobe write to FPGA reg 1, then the 24-bit frame length
// little-endian across regs 0x10/0x11/0x12.
const (
	fpgaVMAXStrobe = 0x01
	fpgaVMAX0      = 0x10
	fpgaVMAX1      = 0x11
	fpgaVMAX2      = 0x12
)

// zwoRegmap implements Regmap over a Transport using the ZWO control-transfer
// protocol. bus picks the sensor-register request (Sony vs generic camera).
type zwoRegmap struct {
	t      Transport
	bus    RegBus
	modeMu sync.RWMutex
	mode   ReadoutMode // live readout context (USB speed, output depth, FPS%) — set by Camera
}

// ReadoutMode implements modeReader: returns the live runtime context the Camera set.
// Guarded: capture goroutines read it (FrameBytes, HMAX/SHS math) while camera methods
// mutate it via updateMode.
func (r *zwoRegmap) ReadoutMode() ReadoutMode {
	r.modeMu.RLock()
	defer r.modeMu.RUnlock()
	return r.mode
}

// updateMode implements modeCarrier: the Camera mutates the live ReadoutMode (FPS%,
// output depth, bin) under the mode lock.
func (r *zwoRegmap) updateMode(f func(*ReadoutMode)) {
	r.modeMu.Lock()
	defer r.modeMu.Unlock()
	f(&r.mode)
}

// VID reports the ZWO vendor id (selects the ZWO gain/offset encoding).
func (r *zwoRegmap) VID() uint16 { return ZWO.VID }

// ZWO is the vendor descriptor for ZWO ASI cameras (USB VID 0x03C3). Its Regmap dialect
// is the control-transfer protocol in this file. Registered at init.
var ZWO = &Vendor{
	VID:  0x03C3,
	Name: "ZWO",
	newRegmap: func(t Transport, bus RegBus, mode ReadoutMode) Regmap {
		return &zwoRegmap{t: t, bus: bus, mode: mode}
	},
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

// SetVMAX programs the frame length (VMAX) into the camera FPGA: clamp to 24 bits,
// strobe FPGA reg 1, write VMAX little-endian to regs 0x10/0x11/0x12, then release the
// strobe. VMAX lives in the FPGA, not the sensor.
func SetVMAX(rm Regmap, vmax uint32) (err error) {
	if vmax > 0xffffff {
		vmax = 0xffffff
	}
	if err = rm.WriteFPGAReg(fpgaVMAXStrobe, 1); err != nil {
		return err
	}
	// Release the commit strobe (FPGA reg 1: 1 -> 0) after the VMAX bytes — on the error
	// paths too (a held strobe gates every later FPGA group commit); the first error wins.
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

// --- camera/FPGA registers (ZWO's own, not the sensor's) ---

func (c *Camera) writeCamReg(reg, val uint16) error {
	return c.t.ControlOut(reqWriteCamReg, reg, val, nil)
}

func (c *Camera) writeFPGAReg(reg, val uint16) error {
	return c.t.ControlOut(reqWriteFPGAReg, reg, val, nil)
}

func (c *Camera) vendorCmd(op uint8) error { return c.t.ControlOut(op, 0, 0, nil) }

// FlashHPCMapAddr is the flash address of the factory hot/dead-pixel correction map blob.
// Layout: 2 KiB header with magic "ASID" (defect map; "ASIG" = gain map) and a big-endian
// uint32 payload length at offset 4, followed by a compressed 1-bit-per-pixel defect bitmap.
const FlashHPCMapAddr = 0x40000

// ReadSPIFlash reads n bytes from the camera's SPI flash starting at addr, in 2 KiB vendor-IN
// blocks (wIndex tracks addr>>8). SPI flash and the sensor's 32-bit GPIF data bus share FX3
// pins, so the read is bracketed by EnableGPIF32DQ(false)/(true); the camera must be Init'd first.
func (c *Camera) ReadSPIFlash(addr uint32, n int) (out []byte, err error) {
	if err := c.t.ControlOut(cmdEnableGPIF32DQ, 0, 0, nil); err != nil {
		return nil, err
	}
	defer func() {
		// The data bus MUST come back up: left disabled, the next readout is silently dead.
		// Surface a failed re-enable (a read error from the body wins if both fail).
		if rerr := c.t.ControlOut(cmdEnableGPIF32DQ, 1, 0, nil); err == nil && rerr != nil {
			err = fmt.Errorf("asicam: re-enable GPIF32DQ after flash read: %w", rerr)
		}
	}()
	const block = 2048
	out = make([]byte, 0, n)
	for len(out) < n {
		want := n - len(out)
		if want > block {
			want = block
		}
		buf := make([]byte, want)
		got, err := c.t.ControlIn(reqReadSPIFlash, 0, uint16(addr>>8), buf)
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

// FirmwareVersion reads the camera firmware version.
func (c *Camera) FirmwareVersion() (uint16, error) {
	buf := make([]byte, 2)
	if _, err := c.t.ControlIn(reqFirmwareVer, 0, 0, buf); err != nil {
		return 0, err
	}
	return uint16(buf[0]) | uint16(buf[1])<<8, nil
}

// Serial is the ZWO factory device ID (ASI_ID): 8 raw bytes burned at manufacture. USB
// exposes no serial-number descriptor, so this is the only stable per-unit identifier.
type Serial [8]byte

// String renders the serial as 8 bytes of lowercase hex.
func (s Serial) String() string {
	const hex = "0123456789abcdef"
	var b [16]byte
	for i, c := range s {
		b[i*2], b[i*2+1] = hex[c>>4], hex[c&0xf]
	}
	return string(b[:])
}

// SerialNumber reads the factory serial: a single vendor control-IN transfer (bRequest
// 0xC8, wValue 0, wIndex 0) returning the 8 raw ASI_ID bytes.
func (c *Camera) SerialNumber() (Serial, error) {
	var s Serial
	if _, err := c.t.ControlIn(reqSerialNumber, 0, 0, s[:]); err != nil {
		return Serial{}, err
	}
	return s, nil
}
