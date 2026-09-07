package astrocam

// ZWO control protocol: the Regmap dialect, the vendor request codes and the ZWO vendor
// descriptor. Everything here is ZWO-only; the vendor-neutral halves of the control plane
// (RegBus, SetVMAX, vendorCmd/vendorIn, ReadSPIFlash, FirmwareVersion, SerialNumber) live in
// protocol.go, and PlayerOne's mirror of this file is protocol_poa.go.

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

	// SendCMD opcodes (vendor OUT, no register payload). They seed ZWO.Cmds.
	cmdStreamStop     = 0xAA // stop/prepare before (re)arming
	cmdStreamStart    = 0xA9 // begin streaming
	cmdFlush          = 0xAF // pipeline flush / drop recovery
	cmdST4N           = 0xB0 // ST4 guide on
	cmdST4F           = 0xB1 // ST4 guide off
	cmdEnableGPIF32DQ = 0xBE // EnableGPIF32DQ: enable the FPGA->FX3 32-bit data bus
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

// FlashHPCMapAddr is the flash address of the factory hot/dead-pixel correction map blob.
// Layout: 2 KiB header with magic "ASID" (defect map; "ASIG" = gain map) and a big-endian
// uint32 payload length at offset 4, followed by a compressed 1-bit-per-pixel defect bitmap.
const FlashHPCMapAddr = 0x40000
