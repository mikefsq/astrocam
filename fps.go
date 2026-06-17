package astrocam

// This file is the shared readout/line-time engine. It is reused by every sensor profile.
// Each sensor profile supplies data (pixel clock, HMAX floor, blanking) and the runtime
// supplies the live ReadoutMode.

// ReadoutMode is the runtime readout context the FPS/line-time math needs. These are
// NOT sensor-die facts: they come from the live USB link, the selected output depth,
// and the user's frame-rate setting. The Camera supplies them (Camera.readoutMode)
type ReadoutMode struct {
	USB3       bool // negotiated link speed (Model.USB3) — selects the bandwidth budget
	BytesPerPx int  // output bytes/pixel: 2 = RAW16, 1 = RAW8
	FPSPercent int  // requested frame-rate percentage, clamped to 40..100 by the FPS-percent throttle
	Bin        int  // symmetric binning factor; 0 normalized to 1
	// SoftBin is the HOST-side bin factor applied AFTER readout (1 = none). RAW16 has no
	// hardware binned readout mode on these sensors — the bin register tables are 12-bit
	// (the RAW8/hardware-bin path). The SDK reads the full 16-bit frame and averages
	// bin×bin on the CPU. When SoftBin>1 the
	// sensor runs bin-1 full-res (Bin=1, Width/Height = the FULL readout dims) and the
	// driver downsamples after the read. FrameBytes is the FULL (wire) size; the output is
	// SoftBin² smaller.
	SoftBin int
	// Width/Height are the live readout dimensions (the ROI after binning). VMAX = Height +
	// VBlankAdd sets the free-run frame period, and the HMAX (line-time) throttle scales with
	// Width, so a sub-frame ROI streams FASTER — this is the SDK's per-ROI frame-rate scaling.
	// 0 means "use the sensor's full-frame dimension".
	Width  int
	Height int
	// HighSpeed selects the sensor's 10-bit high-speed readout (the high-speed flag /
	// ASI_HIGH_SPEED_MODE): a shorter ADC ramp at a doubled pixel clock (~2× fps) for RAW8.
	// The sensor profile reformats to 10-bit and switches clock/HMAX-floor when this is set.
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

// USB-bandwidth budgets the FPS-percent throttle selects on MAX_DATASIZE (ne -> const_A else const_B).
// bwUSB2 is identical across every camera (290/455/174 all 0x4c2511d0); bwUSB3 VARIES per
// camera (290 0x4dac0098, 455 0x4db9f76c, 174 0x4db79512), so when it matters pass the
// sensor's own via HMAXBW.
const (
	bwUSB2 = 43272000.0  // const_A = float32(0x4c2511d0); ~43 MB/s (USB2 HighSpeed) — universal
	bwUSB3 = 360715008.0 // const_B = float32(0x4dac0098); ~360 MB/s (USB3) — IMX290's; per-camera
)

// HMAX reproduces the FPS-percent throttle's per-line readout period. The SDK stores it as the HMAX
// field and ALSO uses it as the line-time numerator (lineTime =
// HMAX*1000/clock), so throttling the readout to fit the USB bus is
// what sets the exposure time scale. Per-sensor inputs are the pixel clock, the HMAX
// floor (REG_FRAME_LENGTH_PKG_MIN) and the VMAX vblank add; geometry and mode are runtime.
//
// The constants below follow the FPS-percent throttle's non-special (default-mode) path:
//
//	cand = 1e6 / (bw / (bytesPerPx*H*W)) / (H+vblankAdd) * clock / 1000   (truncated)
//	HMAX = clamp( max(cand, floor) * 100/fpsPercent , .. , 0xffff )
//
// FPSPercent is the ASI bandwidth-overload (40..100): lower = more USB throttle = larger
// HMAX. USB2 cameras run it low — the IMX174 wire (4337) is reproduced at FPSPercent=40.
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
	x := 1e6 / (bw / (float64(m.BytesPerPx) * float64(h) * float64(w))) // 1e6 = const 0x49742400
	x = x / float64(h+vblankAdd)                                        // H + vblankAdd
	x = x * float64(clock)                                              // × clock
	x = x / 1000.0                                                      // ÷ 1000.0
	cand := int(x)                                                      // fcvtzs (truncate)
	if cand < floor {                                                   // max(cand, floor)
		cand = floor
	}
	hm := cand * 100 / m.FPSPercent // ×100 ÷ fpsPercent (the ASI bandwidth-overload)
	if hm > 0xffff {
		hm = 0xffff
	}
	return uint16(hm)
}

// LineTimeNs is the readout line time in ns: HMAX*1e6/clock, the runtime-computed
// equivalent of HMAX*1000/clock. Sensor exposure math calls this
// rather than storing a baked line time.
func LineTimeNs(w, h, clock, floor, vblankAdd int, m ReadoutMode) uint64 {
	if clock == 0 {
		return 0
	}
	return uint64(HMAX(w, h, clock, floor, vblankAdd, m)) * 1_000_000 / uint64(clock)
}

// modeReader is the optional Regmap capability the Camera's regmap implements to carry
// the live ReadoutMode into the shared exposure/HMAX bodies — without threading it
// through every Sensor op signature and without a global. A plain Regmap (e.g. a test
// fake) falls back to ReadoutMode defaults.
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
// optical-black columns (HBLK → FPGA 0x02/0x03) and rows (VBLK → 0x06/0x07) the sensor emits
// ahead of the active image, so the readout windows past them and the OB margin never reaches
// the host frame. Values are sensor-specific (from the profile).
func SetFPGAHBLK(rm Regmap, hblk uint16) error { return FPGAWrite16(rm, fpgaHBLK0, fpgaHBLK1, hblk) }
func SetFPGAVBLK(rm Regmap, vblk uint16) error { return FPGAWrite16(rm, fpgaVBLK0, fpgaVBLK1, vblk) }

// SetFPGAOutputWidth programs the FX3 output bit width: FPGA reg 0xa bit0 = ADC enable
// (always 1), bit4 = output width (1 = 16-bit RAW16, 0 = 8-bit RAW8). This is the live
// form of what InitFPGA sets at bringup from the output depth.
func SetFPGAOutputWidth(rm Regmap, raw16 bool) error {
	v := uint16(0x01) // bit0 = ADC
	if raw16 {
		v |= 0x10 // bit4 = 1 → 16-bit output
	}
	return FPGAWriteBits(rm, 0x0a, 0x11, v)
}

// SetFPGABinDataLen programs the per-frame DMA word count: a 32-bit little-endian value to
// FPGA regs 0x40..0x43, bracketed by the reg-1 commit strobe (1 then 0) — each byte a
// WriteFPGAREG (bReq 0xBD, wValue=reg, wIndex=byte). dataWords = frameBytes/4 (output_area ·
// bytesPerPx, in 32-bit DMA words). The IMX455/IMX571 Cam_SetResolution program this so the
// FX3 frames the exact (binned / sub-frame) transfer; the STARVIS 290 does not use it.
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

// ProgramFrameGeometry writes the FPGA frame dimensions the FX3 needs to package the
// bulk stream (Cam_SetResolution): HBLK, width, VBLK, height. hblk/vblk are
// sensor-specific blanking values from the profile; width/height are the window.
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

// SetFPGABit read-modify-writes a single mask of an FPGA mode register (ReadFPGAREG
// then WriteFPGAREG) — the form the FX3 reg0/reg0xa mode setters take. on sets
// the bits, !on clears them.
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

// FPGASetBits / FPGAClearBits / FPGAWriteBits are the generic FPGA mode-register RMW
// helpers (ReadFPGAREG then WriteFPGAREG) used across the sensor profiles — set a mask,
// clear a mask, or write a masked field. (SetFPGABit is the on/off form of the first two.)
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
// sensors whose HMAX is a baked constant (e.g. IMX178's ctor HMAX=0x1a4) rather than
// the bandwidth-derived throttle ProgramHMAX computes — the SDK programs the FPGA register via
// SetFPGAHMAX only inside the FPS-percent throttle, so this writes the post-init default directly.
func WriteFPGAHMAX(rm Regmap, hmax uint16) error {
	return FPGAWrite16(rm, fpgaHMAX0, fpgaHMAX1, hmax)
}
