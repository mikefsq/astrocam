package astrocam

import (
	"fmt"
	"sync"
)

// PlayerOne control-transfer opcodes. Same FX3 bridge as ZWO, different host dialect. The
// argument order is the reverse of ZWO's: PlayerOne puts the value in wValue and the register in
// wIndex.
//
//	sensor write (reg,val)       OUT bReq 0xB0  wValue=val wIndex=reg
//	sensor read (reg)            IN  bReq 0xB2  wValue=0   wIndex=reg
//	protected write (reg,val)    OUT bReq 0xB3  wValue=val wIndex=(reg+0xABCD)&0xffff
//	FPGA write (reg,val)         OUT bReq 0xC0  wValue=val wIndex=reg
//	FPGA read (reg)              IN  bReq 0xC2  wValue=0   wIndex=reg
const (
	poaReqImgSenWrite = 0xB0 // sensor register write (8-bit value)
	poaReqImgSenRead  = 0xB2 // sensor register read (IN)
	poaReqImgSenCryp  = 0xB3 // obfuscated write for protected registers (see poaCrypRegBias)
	poaReqFpgaWrite   = 0xC0 // camera-FPGA register write (8-bit value)
	poaReqFpgaBurst   = 0xC1 // camera-FPGA burst write: wIndex = first register, payload in data
	poaReqFpgaRead    = 0xC2 // camera-FPGA register read (IN)

	// poaCrypRegBias is the additive address obfuscation applied before CrypWrite:
	// wire_reg = (reg + 0xABCD) & 0xffff. A fixed offset, not encryption.
	poaCrypRegBias uint16 = 0xABCD
)

// PlayerOne FX3 bridge vendor requests. They ride the same control-transfer shape as the
// register dialect above — a plain vendor/device transfer with bmRequestType 0xC0 for IN and
// 0x40 for OUT, the same as ZWO's — so only the codes and the argument placement differ. The
// bridge answers further requests, including flash writes and a device reset, that this driver
// does not issue.
const (
	poaReqStream   = 0xA0 // stream start/stop: one code, wValue 1 = start, 0 = stop
	poaReqFwVer    = 0xA2 // FX3 firmware version: 1-byte reply
	poaReqSerial   = 0xA3 // factory serial: 20 printable ASCII bytes
	poaReqST4      = 0xA6 // ST4 guide pulse: wValue = line state, wIndex = direction (<= 4)
	poaReqReadTemp = 0xA8 // sensor temperature: signed 16-bit tenths of a degree
	// Flash page read: wIndex = the 256-byte page number (address >> 8), data IN. Unlike ZWO's
	// equivalent this needs no GPIF toggle around it — the flash does not share the data bus.
	poaReqFlashRead = 0xD1

	poaSerialBytes   = 20 // serial transfer length
	poaReadTempBytes = 8  // the temperature read asks for 8 bytes and uses the first halfword
)

// poaTempC decodes a temperature reply. The SDK requests 8 bytes, then uses only the leading
// halfword: sign-extend it and divide by 10, so the unit is tenths of a degree Celsius. ZWO packs the same reading as 12-bit sixteenths, hence the per-vendor conversion.
func poaTempC(b []byte) float64 {
	if len(b) < 2 {
		return 0
	}
	return float64(int16(uint16(b[0])|uint16(b[1])<<8)) / 10
}

// poaProtectedRegs are the registers written via CrypWrite instead of a plain sensor write. The
// only protected register on either Sony die is the gain-setup 0x67f. Read-only after init.
var poaProtectedRegs = map[uint16]bool{0x67f: true}

// poaRegmap implements Regmap over a Transport using PlayerOne's control-transfer protocol. The
// Sony dies and register semantics are identical to ZWO's; only the wire dialect differs.
// protected routes the gain-setup register through CrypWrite.
type poaRegmap struct {
	t         Transport
	bus       RegBus // PlayerOne's Sony dies all use the ImgSen path; kept for interface parity
	modeMu    sync.RWMutex
	mode      ReadoutMode
	protected map[uint16]bool
}

// ReadoutMode implements modeReader (read under the mode lock).
func (r *poaRegmap) ReadoutMode() ReadoutMode {
	r.modeMu.RLock()
	defer r.modeMu.RUnlock()
	return r.mode
}

// updateMode implements modeCarrier (mutate under the mode lock).
func (r *poaRegmap) updateMode(f func(*ReadoutMode)) {
	r.modeMu.Lock()
	defer r.modeMu.Unlock()
	f(&r.mode)
}

// VID reports the PlayerOne vendor id (selects the PlayerOne gain/offset encoding).
func (r *poaRegmap) VID() uint16 { return POA.VID }

// WriteReg writes a sensor register: plain write 0xB0, or CrypWrite (0xB3, address +0xABCD,
// value unchanged) for a protected register.
func (r *poaRegmap) WriteReg(reg, val uint16) error {
	if r.protected[reg] {
		return r.t.ControlOut(poaReqImgSenCryp, val, reg+poaCrypRegBias, nil)
	}
	return r.t.ControlOut(poaReqImgSenWrite, val, reg, nil)
}

// ReadReg reads a sensor register (8-bit; reg in wIndex, wValue 0).
func (r *poaRegmap) ReadReg(reg uint16) (uint16, error) {
	buf := make([]byte, 1)
	got, err := r.t.ControlIn(poaReqImgSenRead, 0, reg, buf)
	if err != nil {
		return 0, fmt.Errorf("read reg 0x%x: %w", reg, err)
	}
	if got < 1 {
		return 0, fmt.Errorf("read reg 0x%x: empty control-IN", reg)
	}
	return uint16(buf[0]), nil
}

// WriteRegBits is a read-modify-write of bits [lo:hi] (inclusive) of reg.
func (r *poaRegmap) WriteRegBits(reg uint16, lo, hi uint8, val uint16) error {
	cur, err := r.ReadReg(reg)
	if err != nil {
		return err
	}
	mask := uint16(((1 << (hi - lo + 1)) - 1) << lo)
	cur = (cur &^ mask) | ((val << lo) & mask)
	return r.WriteReg(reg, cur)
}

// WriteFPGAReg writes a camera-FPGA register (request 0xC0). val is transmitted as given:
// FPGA regs are byte-wide and every in-tree caller pre-masks to the low byte; what an over-byte
// wValue does on the POA wire is unverified.
func (r *poaRegmap) WriteFPGAReg(reg, val uint16) error {
	return r.t.ControlOut(poaReqFpgaWrite, val, reg, nil)
}

// WriteFPGABurst loads consecutive camera-FPGA registers from one transfer (request 0xC1,
// wIndex = the first register). PlayerOne's geometry, drive, crop, exposure and bandwidth
// registers are burst-only; callers go through POAFPGABurst, which adds the group latch.
func (r *poaRegmap) WriteFPGABurst(reg uint16, data []byte) error {
	if len(data) == 0 {
		return fmt.Errorf("write FPGA burst 0x%x: empty payload", reg)
	}
	return r.t.ControlOut(poaReqFpgaBurst, 0, reg, data)
}

// ReadFPGAReg reads a camera-FPGA register (request 0xC2).
func (r *poaRegmap) ReadFPGAReg(reg uint16) (uint16, error) {
	buf := make([]byte, 1)
	got, err := r.t.ControlIn(poaReqFpgaRead, 0, reg, buf)
	if err != nil {
		return 0, fmt.Errorf("read FPGA reg 0x%x: %w", reg, err)
	}
	if got < 1 {
		return 0, fmt.Errorf("read FPGA reg 0x%x: empty control-IN", reg)
	}
	return uint16(buf[0]), nil
}

// POA is the vendor descriptor for PlayerOne Astronomy cameras (USB VID 0xA0A0). Registered at
// init; the product rows (PID -> sensor) are registered by the sensors package.
//
// Three entries stay zero because PlayerOne HAS no counterpart, not because one is still
// undecoded: Flush (its SDK recovers the pipeline host-side with a libusb bulk clear/reset, so
// Camera.Init skips the flush and relies on the endpoint reset), EnableGPIF32DQ (its SPI flash
// does not borrow the FX3 data-bus pins, so nothing has to be gated) and ReadHumidity.
//
// EnableGPIF32DQ being zero also means Camera.Init skips readCalibrationBlob, which on ZWO is
// what puts the FPGA data path into right-aligned RAW16. PlayerOne needs no equivalent: its
// frames come back right-aligned, pixel-for-pixel against the SDK's on a Xena 585M.
var POA = &Vendor{
	VID:  0xA0A0,
	Name: "PlayerOne",
	Cmds: FX3Cmds{
		StreamStart: FX3Cmd{Req: poaReqStream, WValue: 1},
		StreamStop:  FX3Cmd{Req: poaReqStream, WValue: 0},

		FirmwareVersion: poaReqFwVer,
		FirmwareBytes:   1,
		SerialNumber:    poaReqSerial,
		SerialBytes:     poaSerialBytes,
		SerialASCII:     true,

		ST4: FX3ST4{On: poaReqST4, Off: poaReqST4, DirInWIndex: true},

		ReadSPIFlash:  poaReqFlashRead,
		ReadTemp:      poaReqReadTemp,
		ReadTempBytes: poaReadTempBytes,
		TempC:         poaTempC,
	},
	newRegmap: func(t Transport, bus RegBus, mode ReadoutMode) Regmap {
		return &poaRegmap{t: t, bus: bus, mode: mode, protected: poaProtectedRegs}
	},
	fpgaRun: func(rm Regmap, start bool) error {
		if start {
			return POAFPGAStart(rm)
		}
		return POAFPGAStop(rm)
	},
	deviceBins: true,
	// USBBandWidthLimit is 35..100 default 90 on both links, not ZWO's 40..100 with a flat-out
	// USB3 default. The default matters for more than throughput: it is the value the drive
	// register and the GPIF bandwidth are both computed from, so a driver running at 100 where
	// the vendor runs at 90 disagrees with it on every frame's timing.
	fpsMin: 35, fpsDefUSB2: 90, fpsDefUSB3: 90,
	frameMarker: repairPOAFrameMarker,
	frameStart:  poaFrameStart,
	// PlayerOne's FX3 sends sixteen bytes after each frame. Found by content on a Xena 585M: a
	// reader that consumed them BEFORE the frame rotated every frame left by sixteen bytes, which
	// places them after the pixels and not before. Unverified on other PlayerOne bodies.
	frameTrailer:  16,
	loadDefectMap: loadDefectMapPOA,
	// NOT trusted, deliberately. The "HPC:" blob parses perfectly — the declared count matches the
	// entries exactly, the sentinels number height+1, the row indices run 0..2179 complete and
	// ascending — but the pixels it names are NOT defective on the unit tested. In a 15 s dark at
	// gain 300 the mapped pixels average 2137.7 DN against 2136.5 for ordinary ones, and only 2 of
	// the frame's 800 genuinely hot pixels are in the map; no translation or flip improves it.
	// Applying it would replace 9086 good pixels per frame with neighbour averages.
	defectMapTrusted: false,
	// POA_WB_R/G/B, range +-1200 as the config advertises.
	setWhiteBalance: POAFPGAWhiteBalance,
	wbLimit:         1200,
	// POASetImageSize rounds a width that is not a multiple of 4 and an odd height.
	roiStepW: 4,
	roiStepH: 2,
	newThermal: func(c *Camera) Thermal {
		return &poaThermal{t: c.t, rm: c.rm, cmds: c.vend.Cmds, writes: &c.coolW}
	},
}

func init() { RegisterVendor(POA) }

// poaFrameMarker is the fixed header PlayerOne's firmware writes over the frame data: two magic
// bytes, a sequence counter, a fourth magic byte, then eight zeros — twelve in all. It is a byte
// count, not a pixel count, so the same twelve bytes eat six pixels of a 16-bit frame and twelve
// of an 8-bit one.
//
// Most frames carry exactly one, at offset zero. A DIE-BINNED frame carries two: sequence 0 at the
// start and sequence 1 partway in, which fits the readout making two passes over the window. The
// second one's position is stable run to run but is not a simple function of the geometry — four
// window sizes gave 3978240, 968704, 602112 and 147456 bytes in, all multiples of 1024, with no
// constant ratio to the frame or to the row length. Rather than bake in an offset that is not
// understood, the repair scans.
var (
	poaFrameMarkerHead = []byte{0x77, 0xee} // bytes 0-1; byte 2 is a sequence counter
	poaFrameMarkerTag  = byte(0x0c)         // byte 3
)

const (
	poaFrameMarkerLen  = 12
	poaFrameMarkerZero = 8 // trailing zero bytes, counted from byte 4
)

// poaMarkerAt reports whether a full marker signature starts at i.
func poaMarkerAt(buf []byte, i int) bool {
	if i+poaFrameMarkerLen > len(buf) {
		return false
	}
	if buf[i] != poaFrameMarkerHead[0] || buf[i+1] != poaFrameMarkerHead[1] || buf[i+3] != poaFrameMarkerTag {
		return false
	}
	// The eight trailing zeros are what make this safe to scan for. Real pixel data would have to
	// hold eight consecutive zero bytes at exactly this alignment, which cannot happen with any
	// black level above zero — and where it could, the replacement is a neighbouring pixel that
	// is also near zero, so the repair changes nothing that matters.
	for j := i + 4; j < i+4+poaFrameMarkerZero; j++ {
		if buf[j] != 0 {
			return false
		}
	}
	return true
}

// poaFrameStart locates the first frame marker in buf and returns its byte offset, or -1 when
// there is none. 0 means the buffer already begins on a frame.
//
// It is only sound for a frame carrying ONE marker, which is every frame the sensor or the FPGA
// has not binned. A die-binned frame carries a second marker partway in (the readout makes two
// passes over the window), and a buffer that starts mid-frame would find that one first and align
// to the middle of a frame rather than its start. The caller therefore aligns at bin 1 only.
func poaFrameStart(buf []byte) int {
	for i := 0; i+poaFrameMarkerLen <= len(buf); i++ {
		if poaMarkerAt(buf, i) {
			return i
		}
	}
	return -1
}

// repairPOAFrameMarker overwrites every marker with pixels from the same columns one step down,
// the same repair the ZWO DDR markers get. It is the whole remaining difference against the
// vendor's RAW8 output: a 320x240 frame differed in three pixels and all three were the header.
func repairPOAFrameMarker(buf []byte, bpp, width, rows int) {
	if bpp < 1 || width <= 0 || rows <= 0 || len(buf) < poaFrameMarkerLen {
		return
	}
	src := rows * width * bpp
	if src < poaFrameMarkerLen {
		return
	}
	for i := 0; i+poaFrameMarkerLen <= len(buf); i++ {
		if !poaMarkerAt(buf, i) {
			continue
		}
		// Reach a row down for replacements, or a row up when that would run off the end.
		from := i + src
		if from+poaFrameMarkerLen > len(buf) {
			from = i - src
		}
		if from < 0 || from+poaFrameMarkerLen > len(buf) {
			continue // nowhere clean to copy from; leave it rather than invent pixels
		}
		copy(buf[i:i+poaFrameMarkerLen], buf[from:from+poaFrameMarkerLen])
		i += poaFrameMarkerLen - 1
	}
}
