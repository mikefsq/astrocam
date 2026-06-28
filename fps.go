package astrocam

// This file is the shared readout/line-time engine, reused by every sensor profile. Each
// profile supplies data (pixel clock, HMAX floor, blanking); the runtime supplies the live
// ReadoutMode.

// ReadoutMode is the runtime readout context the FPS/line-time math needs (USB link speed,
// output depth, frame-rate setting). Not sensor-die facts; supplied by the Camera.
type ReadoutMode struct {
	USB3       bool // negotiated link speed (Model.USB3) — selects the bandwidth budget
	BytesPerPx int  // output bytes/pixel: 2 = RAW16, 1 = RAW8
	FPSPercent int  // requested frame-rate percentage, clamped to 40..100 by the FPS-percent throttle
	Bin        int  // symmetric binning factor; 0 normalized to 1
	// SoftBin is the HOST-side bin factor applied AFTER readout (1 = none). RAW16 has no
	// hardware binned readout mode on these sensors, so the driver reads the full 16-bit frame
	// and averages bin×bin on the CPU. When SoftBin>1 the sensor runs bin-1 full-res (Bin=1,
	// Width/Height = the full readout dims) and the driver downsamples after the read; the
	// output is SoftBin² smaller.
	SoftBin int
	// Width/Height are the live readout dimensions (the ROI after binning). VMAX = Height +
	// VBlankAdd sets the free-run frame period, and the HMAX (line-time) throttle scales with
	// Width, so a sub-frame ROI streams faster. 0 means "use the full-frame dimension".
	Width  int
	Height int
	// HighSpeed selects the sensor's 10-bit high-speed readout (ASI_HIGH_SPEED_MODE): a
	// shorter ADC ramp at a doubled pixel clock (~2× fps) for RAW8. The profile reformats to
	// 10-bit and switches clock/HMAX-floor when set.
	HighSpeed bool
}

func (m ReadoutMode) norm() ReadoutMode {
	if m.BytesPerPx == 0 {
		m.BytesPerPx = 2 // RAW16 default (2 bytes/pixel for 16-bit output)
	}
	if m.Bin < 1 {
		m.Bin = 1 // full resolution
	}
	if m.FPSPercent == 0 {
		m.FPSPercent = 100 // max speed / least throttle
	}
	// The FPS-percent throttle clamps the requested percent to [40, 100].
	if m.FPSPercent < 40 { // 0x28
		m.FPSPercent = 40
	}
	if m.FPSPercent > 100 { // 0x64
		m.FPSPercent = 100
	}
	return m
}

// USB-bandwidth budgets the FPS-percent throttle uses. bwUSB2 is identical across every
// camera; bwUSB3 varies per camera (290 0x4dac0098, 455 0x4db9f76c, 174 0x4db79512), so
// when it matters pass the sensor's own via HMAXBW.
const (
	bwUSB2 = 43272000.0  // float32(0x4c2511d0); ~43 MB/s (USB2 HighSpeed) — universal
	bwUSB3 = 360715008.0 // float32(0x4dac0098); ~360 MB/s (USB3) — IMX290's; per-camera
)

// HMAX computes the per-line readout period, which doubles as the line-time numerator
// (lineTime = HMAX*1000/clock), so throttling the readout to fit the USB bus also sets the
// exposure time scale. Per-sensor inputs: pixel clock, HMAX floor (REG_FRAME_LENGTH_PKG_MIN),
// VMAX vblank add. Geometry and mode are runtime.
//
//	cand = 1e6 / (bw / (bytesPerPx*H*W)) / (H+vblankAdd) * clock / 1000   (truncated)
//	HMAX = clamp( max(cand, floor) * 100/fpsPercent , .. , 0xffff )
//
// FPSPercent is the ASI bandwidth-overload (40..100): lower = more USB throttle = larger HMAX.
func HMAX(w, h, clock, floor, vblankAdd int, m ReadoutMode) uint16 {
	return HMAXBW(w, h, clock, floor, vblankAdd, bwUSB2, bwUSB3, m)
}

// HMAXBW is HMAX with the sensor's own USB-bandwidth constants. Use it when the camera's
// bwUSB3 differs from the package default (it varies per camera); bw2 is universal.
func HMAXBW(w, h, clock, floor, vblankAdd int, bw2, bw3 float64, m ReadoutMode) uint16 {
	m = m.norm()
	bw := bw3
	if !m.USB3 {
		bw = bw2
	}
	x := 1e6 / (bw / (float64(m.BytesPerPx) * float64(h) * float64(w)))
	x = x / float64(h+vblankAdd) // H + vblankAdd
	x = x * float64(clock)       // × clock
	x = x / 1000.0               // ÷ 1000.0
	cand := int(x)               // truncate
	if cand < floor {            // max(cand, floor)
		cand = floor
	}
	hm := cand * 100 / m.FPSPercent // ×100 ÷ fpsPercent
	if hm > 0xffff {
		hm = 0xffff
	}
	return uint16(hm)
}

// LineTimeNs is the readout line time in ns: HMAX*1e6/clock.
func LineTimeNs(w, h, clock, floor, vblankAdd int, m ReadoutMode) uint64 {
	if clock == 0 {
		return 0
	}
	return uint64(HMAX(w, h, clock, floor, vblankAdd, m)) * 1_000_000 / uint64(clock)
}

// modeReader is the optional Regmap capability that carries the live ReadoutMode into the
// shared exposure/HMAX bodies. A plain Regmap (e.g. a test fake) falls back to defaults.
type modeReader interface{ ReadoutMode() ReadoutMode }

// ModeOf returns the live ReadoutMode carried by rm, or normalized defaults.
func ModeOf(rm Regmap) ReadoutMode {
	if p, ok := rm.(modeReader); ok {
		return p.ReadoutMode().norm()
	}
	return ReadoutMode{}.norm()
}

// FPGA register numbers (the wValue passed to WriteFPGAREG 0xBD). Shared by every
// camera — not per-sensor.
const (
	fpgaStrobe  = 0x01 // reg-1 commit strobe (1 before a group, 0 after); all setters
	fpgaHBLK0   = 0x02 // SetFPGAHBLK  lo
	fpgaHBLK1   = 0x03 // SetFPGAHBLK  hi
	fpgaWidth0  = 0x04 // SetFPGAWidth lo
	fpgaWidth1  = 0x05 // SetFPGAWidth hi
	fpgaVBLK0   = 0x06 // SetFPGAVBLK  lo
	fpgaVBLK1   = 0x07 // SetFPGAVBLK  hi
	fpgaHeight0 = 0x08 // SetFPGAHeight lo
	fpgaHeight1 = 0x09 // SetFPGAHeight hi
	fpgaHMAX0   = 0x13 // SetFPGAHMAX  lo
	fpgaHMAX1   = 0x14 // SetFPGAHMAX  hi
)

// FPGAWrite16 writes a 16-bit value little-endian to an FPGA register pair, bracketed
// by the FX3 reg-1 commit strobe (1 then 0) — the form every SetFPGA{HBLK,VBLK,Width,
// Height,HMAX} setter takes.
func FPGAWrite16(rm Regmap, loReg, hiReg, val uint16) error {
	if err := rm.WriteFPGAReg(fpgaStrobe, 1); err != nil {
		return err
	}
	if err := rm.WriteFPGAReg(loReg, val&0xff); err != nil {
		return err
	}
	if err := rm.WriteFPGAReg(hiReg, (val>>8)&0xff); err != nil {
		return err
	}
	return rm.WriteFPGAReg(fpgaStrobe, 0)
}

// SetFPGAHBLK / SetFPGAVBLK program the FX3 optical-black crop: the count of leading blank /
// optical-black columns (HBLK → FPGA 0x02/0x03) and rows (VBLK → 0x06/0x07) windowed out
// before the active image. Values are sensor-specific (from the profile).
func SetFPGAHBLK(rm Regmap, hblk uint16) error { return FPGAWrite16(rm, fpgaHBLK0, fpgaHBLK1, hblk) }
func SetFPGAVBLK(rm Regmap, vblk uint16) error { return FPGAWrite16(rm, fpgaVBLK0, fpgaVBLK1, vblk) }

// SetFPGAOutputWidth programs the FX3 output bit width: FPGA reg 0xa bit0 = ADC enable
// (always 1), bit4 = output width (1 = 16-bit RAW16, 0 = 8-bit RAW8).
func SetFPGAOutputWidth(rm Regmap, raw16 bool) error {
	v := uint16(0x01) // bit0 = ADC
	if raw16 {
		v |= 0x10 // bit4 = 1 → 16-bit output
	}
	return FPGAWriteBits(rm, 0x0a, 0x11, v)
}

// SetFPGABinDataLen programs the per-frame DMA word count: a 32-bit little-endian value to
// FPGA regs 0x40..0x43, bracketed by the reg-1 commit strobe (1 then 0). dataWords =
// frameBytes/4. IMX455/IMX571 program this so the FX3 frames the exact (binned / sub-frame)
// transfer; the STARVIS 290 does not use it.
func SetFPGABinDataLen(rm Regmap, dataWords uint32) error {
	if err := rm.WriteFPGAReg(fpgaStrobe, 1); err != nil {
		return err
	}
	for i, reg := range []uint16{0x40, 0x41, 0x42, 0x43} {
		if err := rm.WriteFPGAReg(reg, uint16(dataWords>>(8*uint(i)))&0xff); err != nil {
			return err
		}
	}
	return rm.WriteFPGAReg(fpgaStrobe, 0)
}

// ProgramFrameGeometry writes the FPGA frame dimensions the FX3 needs to package the bulk
// stream: HBLK, width, VBLK, height. hblk/vblk are sensor-specific blanking values.
func ProgramFrameGeometry(rm Regmap, w, h, hblk, vblk int) error {
	for _, g := range []struct {
		lo, hi uint16
		val    int
	}{
		{fpgaHBLK0, fpgaHBLK1, hblk},
		{fpgaWidth0, fpgaWidth1, w},
		{fpgaVBLK0, fpgaVBLK1, vblk},
		{fpgaHeight0, fpgaHeight1, h},
	} {
		if err := FPGAWrite16(rm, g.lo, g.hi, uint16(g.val)); err != nil {
			return err
		}
	}
	return nil
}

// SetFPGABit read-modify-writes a single mask of an FPGA mode register (ReadFPGAREG then
// WriteFPGAREG). on sets the bits, !on clears them.
func SetFPGABit(rm Regmap, reg, bits uint16, on bool) error {
	v, err := rm.ReadFPGAReg(reg)
	if err != nil {
		return err
	}
	if on {
		v |= bits
	} else {
		v &^= bits
	}
	return rm.WriteFPGAReg(reg, v&0xff)
}

// FPGASetBits / FPGAClearBits / FPGAWriteBits are generic FPGA mode-register RMW helpers
// (ReadFPGAREG then WriteFPGAREG): set a mask, clear a mask, or write a masked field.
func FPGASetBits(rm Regmap, reg, bits uint16) error {
	v, err := rm.ReadFPGAReg(reg)
	if err != nil {
		return err
	}
	return rm.WriteFPGAReg(reg, (v|bits)&0xff)
}

func FPGAClearBits(rm Regmap, reg, bits uint16) error {
	v, err := rm.ReadFPGAReg(reg)
	if err != nil {
		return err
	}
	return rm.WriteFPGAReg(reg, (v&^bits)&0xff)
}

func FPGAWriteBits(rm Regmap, reg, mask, val uint16) error {
	v, err := rm.ReadFPGAReg(reg)
	if err != nil {
		return err
	}
	return rm.WriteFPGAReg(reg, ((v&^mask)|val)&0xff)
}

// ProgramHMAX computes the readout line period for the window from the live ReadoutMode
// and writes it to the FPGA (SetFPGAHMAX 0x13/0x14, strobed). clock/floor/vblankAdd are
// the sensor's; the bandwidth/FPS/depth come from rm's ReadoutMode.
func ProgramHMAX(rm Regmap, w, h, clock, floor, vblankAdd int) error {
	hm := HMAX(w, h, clock, floor, vblankAdd, ModeOf(rm))
	return FPGAWrite16(rm, fpgaHMAX0, fpgaHMAX1, hm)
}

// WriteFPGAHMAX writes a constant HMAX line period to the FPGA (0x13/0x14, strobed). Used by
// sensors whose HMAX is a baked constant (e.g. IMX178's ctor HMAX=0x1a4) rather than the
// bandwidth-derived throttle ProgramHMAX computes.
func WriteFPGAHMAX(rm Regmap, hmax uint16) error {
	return FPGAWrite16(rm, fpgaHMAX0, fpgaHMAX1, hmax)
}
