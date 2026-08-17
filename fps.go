package astrocam

// Shared readout/line-time engine, reused by every sensor profile. Each profile supplies data
// (pixel clock, HMAX floor, blanking); the runtime supplies the live ReadoutMode.

// ReadoutMode is the runtime readout context the FPS/line-time math needs (USB link speed,
// output depth, frame-rate setting). Supplied by the Camera, not the sensor profile.
type ReadoutMode struct {
	USB3       bool // negotiated link speed (Model.USB3); selects the bandwidth budget
	BytesPerPx int  // output bytes/pixel: 2 = RAW16, 1 = RAW8
	FPSPercent int  // requested frame-rate percentage, clamped to 40..100
	Bin        int  // the sensor-side (hardware) binning factor; 0 normalized to 1
	// SoftBin is the host-side bin factor applied after readout (1 = none). By default the whole
	// factor is host-side (the sensor runs bin 1 over the bin-scaled region, Width/Height = its
	// dims); with hardware bin on, Bin is the sensor's factor and SoftBin the remainder. The
	// read bins SoftBin×SoftBin blocks (RAW16 mean, RAW8 clipped sum); the output is SoftBin²
	// smaller.
	SoftBin int
	// Width/Height are the live readout dimensions (the ROI after binning). VMAX = Height +
	// VBlankAdd sets the free-run frame period, and the HMAX (line-time) throttle scales with
	// Width, so a sub-frame ROI streams faster. 0 means the full-frame dimension.
	Width  int
	Height int
	// HighSpeed selects the sensor's 10-bit high-speed readout (ASI_HIGH_SPEED_MODE): a shorter
	// ADC ramp at a doubled pixel clock (~2× fps) for RAW8. The profile reformats to 10-bit and
	// switches clock/HMAX-floor when set.
	HighSpeed bool
}

func (m ReadoutMode) norm() ReadoutMode {
	if m.BytesPerPx == 0 {
		m.BytesPerPx = 2 // RAW16 default
	}
	if m.Bin < 1 {
		m.Bin = 1
	}
	if m.FPSPercent == 0 {
		m.FPSPercent = 100
	}
	// FPSPercent clamps to [40, 100] (0x28..0x64).
	if m.FPSPercent < 40 {
		m.FPSPercent = 40
	}
	if m.FPSPercent > 100 {
		m.FPSPercent = 100
	}
	return m
}

// USB-bandwidth budgets the FPS-percent throttle uses. bwUSB2 is identical across every camera;
// bwUSB3 varies per camera (290 0x4dac0098, 455 0x4db9f76c, 174 0x4db79512), so when it matters
// pass the sensor's own via HMAXBW.
const (
	bwUSB2 = 43272000.0  // float32(0x4c2511d0); ~43 MB/s (USB2 HighSpeed); universal
	bwUSB3 = 360715008.0 // float32(0x4dac0098); ~360 MB/s (USB3); IMX290's, per-camera
)

// HMAX computes the per-line readout period, which is also the line-time numerator (lineTime =
// HMAX*1000/clock), so throttling the readout to the USB bus also sets the exposure time scale.
// Per-sensor inputs: pixel clock, HMAX floor (REG_FRAME_LENGTH_PKG_MIN), VMAX vblank add.
// Geometry and mode are runtime. This is the SDK's SetFPSPerc formula: the bandwidth is the
// link-switched MAX_DATASIZE global (USB3→360715, USB2→43272; ×10×100 = bwUSB3/bwUSB2).
//
//	cand = 1e6 / (bw / (bytesPerPx*H*W)) / (H+vblankAdd) * clock / 1000   (truncated)
//	HMAX = clamp( max(cand, floor) * 100/fpsPercent , .. , 0xffff )
//
// FPSPercent is the ASI bandwidth-overload (40..100): lower = more USB throttle = larger HMAX.
// On USB3 the floor usually dominates (462 full-frame candidate ≈196 < its 261 floor); on USB2
// the candidate dominates (462 full-frame 1634 → HMAX 4085 at pct=40, wire-confirmed) and pins
// the sensor line rate to the link budget so the FX3 GPIF never outruns the pipe.
func HMAX(w, h, clock, floor, vblankAdd int, m ReadoutMode) uint16 {
	return HMAXBW(w, h, clock, floor, vblankAdd, bwUSB2, bwUSB3, m)
}

// HMAXBW is HMAX with the sensor's own USB-bandwidth constants. Use it when the camera's bwUSB3
// differs from the package default; bw2 is universal.
func HMAXBW(w, h, clock, floor, vblankAdd int, bw2, bw3 float64, m ReadoutMode) uint16 {
	m = m.norm()
	bw := bw3
	if !m.USB3 {
		bw = bw2
	}
	x := 1e6 / (bw / (float64(m.BytesPerPx) * float64(h) * float64(w)))
	x = x / float64(h+vblankAdd)
	x = x * float64(clock)
	x = x / 1000.0
	cand := int(x) // truncate
	if cand < floor {
		cand = floor
	}
	hm := cand * 100 / m.FPSPercent
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
// camera, not per-sensor.
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

// FPGAWrite16 writes a 16-bit value little-endian to an FPGA register pair, bracketed by the
// reg-1 commit strobe (1 then 0), the form every SetFPGA{HBLK,VBLK,Width,Height,HMAX} setter
// takes. The strobe is released even when a data write errors (a held strobe gates every later
// FPGA group commit); the first error wins.
func FPGAWrite16(rm Regmap, loReg, hiReg, val uint16) (err error) {
	if err = rm.WriteFPGAReg(fpgaStrobe, 1); err != nil {
		return err
	}
	defer func() {
		if rerr := rm.WriteFPGAReg(fpgaStrobe, 0); err == nil {
			err = rerr
		}
	}()
	if err = rm.WriteFPGAReg(loReg, val&0xff); err != nil {
		return err
	}
	return rm.WriteFPGAReg(hiReg, (val>>8)&0xff)
}

// SetFPGAHBLK / SetFPGAVBLK program the FX3 optical-black crop: the count of leading blank /
// optical-black columns (HBLK → FPGA 0x02/0x03) and rows (VBLK → 0x06/0x07) windowed out
// before the active image. Values are sensor-specific (from the profile).
func SetFPGAHBLK(rm Regmap, hblk uint16) error { return FPGAWrite16(rm, fpgaHBLK0, fpgaHBLK1, hblk) }
func SetFPGAVBLK(rm Regmap, vblk uint16) error { return FPGAWrite16(rm, fpgaVBLK0, fpgaVBLK1, vblk) }

// SetFPGAOutputWidth programs the FX3 output bit width: FPGA reg 0xa bit4 = output width
// (1 = 16-bit RAW16, 0 = 8-bit RAW8). Bit0 (ADC_BIT, the readout ADC width) belongs to the
// sensor profile (its InitFPGA/SetROI choose it per readout mode) and is left untouched.
func SetFPGAOutputWidth(rm Regmap, raw16 bool) error {
	v := uint16(0)
	if raw16 {
		v = 0x10 // bit4 = 1 → 16-bit output
	}
	return FPGAWriteBits(rm, 0x0a, 0x10, v)
}

// SetFPGABinDataLen programs the per-frame DMA word count: a 32-bit little-endian value to FPGA
// regs 0x40..0x43, bracketed by the reg-1 commit strobe (1 then 0). dataWords = frameBytes/4.
// IMX455/IMX571 program it so the FX3 frames the exact (binned / sub-frame) transfer; the STARVIS
// 290 does not use it.
func SetFPGABinDataLen(rm Regmap, dataWords uint32) (err error) {
	if err = rm.WriteFPGAReg(fpgaStrobe, 1); err != nil {
		return err
	}
	defer func() { // release the strobe on the error paths too; the first error wins
		if rerr := rm.WriteFPGAReg(fpgaStrobe, 0); err == nil {
			err = rerr
		}
	}()
	for i, reg := range []uint16{0x40, 0x41, 0x42, 0x43} {
		if err = rm.WriteFPGAReg(reg, uint16(dataWords>>(8*uint(i)))&0xff); err != nil {
			return err
		}
	}
	return nil
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

// SetFPGABit read-modify-writes a mask of an FPGA mode register: on sets the bits, !on clears
// them.
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

// FPGASetBits / FPGAClearBits / FPGAWriteBits are FPGA mode-register RMW helpers: set a mask,
// clear a mask, or write a masked field.
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

// ProgramHMAX computes the readout line period for the window from the live ReadoutMode and
// writes it to the FPGA (SetFPGAHMAX 0x13/0x14, strobed). clock/floor/vblankAdd are the sensor's.
func ProgramHMAX(rm Regmap, w, h, clock, floor, vblankAdd int) error {
	hm := HMAX(w, h, clock, floor, vblankAdd, ModeOf(rm))
	return FPGAWrite16(rm, fpgaHMAX0, fpgaHMAX1, hm)
}

// WriteFPGAHMAX writes a constant HMAX line period to the FPGA (0x13/0x14, strobed), for sensors
// whose HMAX is a baked constant (e.g. IMX178's ctor HMAX=0x1a4).
func WriteFPGAHMAX(rm Regmap, hmax uint16) error {
	return FPGAWrite16(rm, fpgaHMAX0, fpgaHMAX1, hmax)
}
