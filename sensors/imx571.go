// UNVERIFIED on ZWO hardware.
// SECOND-SOURCE CONFIRMED (PlayerOne): same conv-gain reg 0x2f, gain-setup 0x67f, offset
// regs + mirror, and code formula 4095·(1−10^(−g/200)) (which also confirms the 4094→4095 fix below).
// PlayerOne shares this profile — SetGain/SetOffset/GainCaps/OffsetCaps dispatch on the regmap's VID
// (ZWO 0x03C3 vs PlayerOne 0xA0A0); see imx571SetGainPOA.
package sensors

import . "asicam"

import (
	"fmt"
	"math"
	"time"
)

// Sony IMX571 — APS-C BSI CMOS (ZWO ASI2600 family, 26 MP). Unlike the IMX462
// "Type 1/2.8" family, the ASI2600's Sony die is driven over the camera FPGA's
// indexed register interface, so the WriteSONYREG addresses are small FPGA-bank
// indices (0x18/0x19 shutter, 0x30..0x33 gain, 0x2f/0x40 conversion gain,
// 0xa7..0xa9 ROI-X, 0x08/0x09 ROI-Y, 0x0a/0x0b window-HEIGHT, 0x1dd/0x1de
// window-WIDTH) rather than raw 0x30xx Sony page addresses. Register 0x07 acts
// as the coupled-group apply.
//
// Register map summary:
//
//	InitCamera          (FPGA-bank sensor setup writes + reset/standby)
//	Cam_SetResolution   (window H 0x0a/0x0b, W 0x1dd/0x1de)
//	SetStartPos         (ROI X 0xa7/0xa8/0xa9, Y 0x08/0x09)
//	SetGain             (setup 0x67f, code 0x30..0x33, conv 0x2f/0x40)
//	SetExp              (shutter SHS 0x18/0x19, VMAX via FPGA)
const (
	// SetGain — gain code 4094*(1 - 10^(-gain/200)) split across two
	// 16-bit copies (0x30/0x31 and 0x32/0x33); 0x2f/0x40 pick conversion gain.
	imx571RegGainSetup = 0x67f // written at the start of a gain update (0 / 0x11)
	imx571RegGainAL    = 0x30  // analog gain code, low byte  (copy 1)
	imx571RegGainAH    = 0x31  // analog gain code, high byte (copy 1)
	imx571RegGainBL    = 0x32  // analog gain code, low byte  (copy 2)
	imx571RegGainBH    = 0x33  // analog gain code, high byte (copy 2)
	imx571RegConvLow   = 0x2f  // LCG/HCG conversion-gain select
	imx571RegConvHi    = 0x40  // extra-high conversion gain bits (>460 branch)

	// SetStartPos — X aligned to 16, written nibble-shifted; Y direct.
	imx571RegApply    = 0x07 // coupled-group apply (written 1 by ROI / res ops)
	imx571RegStartXEn = 0xa7 // ROI-X enable / mode flag
	imx571RegStartXL  = 0xa8 // X start bits [4:12]
	imx571RegStartXH  = 0xa9 // X start bits [12:20]
	imx571RegStartYL  = 0x08 // (Y start + 0x19/0x1b) low byte
	imx571RegStartYH  = 0x09 // (Y start + 0x19/0x1b) high byte

	// Cam_SetResolution — output window in OUTPUT (binned) pixels.
	// SetResolution takes width=arg1, height=arg2;
	// Cam_SetResolution writes the output rows (HEIGHT) to 0x0a/0x0b and
	// the output width (×4-aligned +0x18) to 0x1dd/0x1de — so 0x0a = HEIGHT, not width.
	imx571RegWinMode = 0x1d8 // window mode: 4 full / 0 binned
	imx571RegHeightL = 0x0a  // window HEIGHT low  (effHeight, +2 when binned)
	imx571RegHeightH = 0x0b  // window HEIGHT high
	imx571RegWidthL  = 0x1dd // window WIDTH low  ((effWidth ×4-aligned)+0x18, &0xfc)
	imx571RegWidthH  = 0x1de // window WIDTH high

	// SetExp — shutter SHS, written as two bytes (FPGA holds VMAX).
	imx571RegSHS0 = 0x18 // SHS low byte
	imx571RegSHS1 = 0x19 // SHS high byte

	imx571GainMax    = 0x2bc // 700 (0.1 dB units); SetGain clamp
	imx571GainMin    = -0x19 // -25: clamp lo
	imx571GainHCGAt  = 0x64  // 100: at/above this the high-conv-gain path (0x2f=1)
	imx571GainHCGHi  = 0x1cc // 460: above this the extra 0x2f/0x40 packing
	imx571GainCodeFS = 4095  // full-scale gain code = 4095.0

	imx571StartXAlign = 16   // X start aligned to 16 (mask 0x7ffffff0)
	imx571StartYOff   = 0x19 // Y start += 0x19 before the 0x08/0x09 write

	imx571ExpMinUs  = 0x20       // 32 µs: SetExp clamp lo
	imx571ExpMaxUs  = 0x77359400 // 2,000,000,000 µs: clamp hi
	imx571LongExpUs = 0xf4240    // 1,000,000 µs: > 1 s takes the streaming-toggle path
	imx571HeightFP  = 0x447a0000 // 1000.0f: line-time scale (CalcFrameTime)

	// SetExp derives the per-line readout time and frame length per READOUT MODE:
	//
	//	lineTime_ns = V * 1e6 / clockHz       (V*1000 / clock, in µs → ×1000 ns)
	//	VMAX        = vblank + effHeight       (vblank + effHeight)
	//	SHS         = VMAX - 1 - lines         (clamp >=1; 17-bit cap 0x1fffe; NO halving)
	//
	// V (the HMAX timing base) and the vblank are set
	// PER BIN by InitSensorMode — see imx571SelectMode. clockHz = clock = 0x4e20
	// (20000). effHeight = height/bin, except bin4 = height/2 (the
	// output-rows<<1 quirk, bin4 reusing bin-2 geometry). Unlike the IMX455 there is
	// NO SHS halving. (lineTime and DefVMAX are computed per bin: bin 1 = 48+4168 = 4216.)
	imx571ClockHz   = 20000 // line-time clock divisor (0x4e20)
	imx571SHSOffset = -1    // SHS = VMAX - 1 - lines

	imx571FullWidth  = 6224 // output width at full-frame bin 1 (= MaxWidth)
	imx571FullHeight = 4168 // output rows at full-frame bin 1 (= MaxHeight)
)

// imx571InitTail are the explicit InitCamera FPGA-bank writes that follow the bulk
// per-mode sensor table (the reg_full_16bit table dumped below and applied as the
// bin-1 default, bringing the 571 to two-stage init parity with the IMX455).
// InitCamera is bracketed by the model lookup (id 0x260a), an FPGA reset
// (cmd 0xaf) and a 0x1ee standby toggle the Transport performs.
var imx571InitTail = []RegVal{
	{Reg: 0x03, Val: 0x10},
	{Reg: 0x07, Val: 0x01},  // coupled-group apply
	{Reg: 0xa7, Val: 0x01},  // ROI-X enable
	{Reg: 0x1d8, Val: 0x04}, // window mode
	{Reg: 0x48, Val: 0x0f},
	{Reg: 0x51, Val: 0x08},
}

// imx571Init is the streaming default: the bin-1 16-bit mode table (InitSensorMode default)
// followed by the explicit tail — the two-stage init, mirroring imx455Init.
var imx571Init = append(append([]RegVal{}, imx571ModeFull16...), imx571InitTail...)

var IMX571 = Sensor{
	Name: "IMX571", // ASI2600MC Pro; Sony IMX571 APS-C BSI (color, RGGB)
	Info: CameraInfo{
		MaxWidth:  6224,
		MaxHeight: 4168,
		PixelUm:   3.76,
		BitDepth:  16,
		Bayer:     "RGGB",            // MC = color
		Bins:      []int{1, 2, 3, 4}, // 1/2/3/4× decoded (per-mode tables + V/vblank); UNVERIFIED
	},
	// ASI Brightness / black level. Caps: 0..240, def 1.
	OffsetMax:   240,
	OffsetDef:   1,
	Init:        imx571Init,
	SetGain:     imx571SetGain,
	GainCaps:    imx571GainCaps,
	SetExposure: imx571SetExposure,
	SetOffset:   imx571SetOffset,
	OffsetCaps:  imx571OffsetCaps,
	SetROI:      imx571SetROI,
}

// imx571GainCaps / imx571OffsetCaps return the advertised range per vendor — the dual of the
// dispatched SetGain/SetOffset. ZWO: gain −25..700 (0.1 dB), offset 0..240 def 1. PlayerOne: gain
// 0..550, offset 0..2000 def 20 (the same unified PlayerOne scale as the 455).
func imx571GainCaps(vid uint16) (min, max int) {
	switch vid {
	case ZWO.VID:
		return imx571GainMin, imx571GainMax // -25..700
	case POA.VID:
		return 0, 550
	default:
		return 0, 0
	}
}

func imx571OffsetCaps(vid uint16) (min, max, def int) {
	switch vid {
	case ZWO.VID:
		return 0, 240, 1
	case POA.VID:
		return 0, 2000, 20
	default:
		return 0, 0, 0
	}
}

// imx571SetOffset — SetBrightness (ASI Brightness / black level):
// value = offset·10 at bin 1 (binned applies a float scale, not reproduced), written
// 16-bit little-endian to sensor 0x42/0x43 and mirrored to 0x44/0x45.
// imx571SetOffset selects the black-level encoding from the regmap's VID — PlayerOne offset·8,
// ZWO offset·10; an unrecognized vendor is an error.
func imx571SetOffset(rm Regmap, offset int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx571SetOffsetZWO(rm, offset)
	case POA.VID:
		return imx571SetOffsetPOA(rm, offset)
	default:
		return fmt.Errorf("asicam: imx571 offset: unsupported vendor VID 0x%04x", rm.VID())
	}
}

// imx571SetOffsetPOA is PlayerOne's IMX571 black level: offset·8 (vs ZWO's ·10), same
// 0x42/0x43 mirror 0x44/0x45 block.
func imx571SetOffsetPOA(rm Regmap, offset int) error {
	v := uint16(offset * 8)
	for _, rv := range []RegVal{
		{Reg: 0x42, Val: v & 0xff}, {Reg: 0x43, Val: (v >> 8) & 0xff},
		{Reg: 0x44, Val: v & 0xff}, {Reg: 0x45, Val: (v >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return nil
}

func imx571SetOffsetZWO(rm Regmap, offset int) error {
	v := uint16(offset * 10)
	for _, rv := range []RegVal{
		{Reg: 0x42, Val: v & 0xff}, {Reg: 0x43, Val: (v >> 8) & 0xff},
		{Reg: 0x44, Val: v & 0xff}, {Reg: 0x45, Val: (v >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return nil
}

// imx571Mode is one readout mode: its sensor register table plus the timing base V
// (line-time) and vblank that InitSensorMode writes to the timing-base/vblank fields.
type imx571Mode struct {
	table     []RegVal
	v, vblank int
}

// imx571SelectMode maps the binning factor to the readout mode (InitSensorMode branch
// table; bin 4 reuses the bin-2 table+V like the IMX455). All V/vblank values are the normal
// (non-strap) constants; the strap halves V but can't be read without HW.
// The 12-bit binned tables are the only binned variants the die offers; output stays 2 B/px.
func imx571SelectMode(bin int) imx571Mode {
	switch bin {
	case 2, 4:
		return imx571Mode{imx571ModeBin2w12, 490, 28}
	case 3:
		return imx571Mode{imx571ModeBin3w12, 250, 24}
	default: // bin 1, 16-bit
		return imx571Mode{imx571ModeFull16, 1350, 48} // vblank initial 48
	}
}

// imx571SetGain — SetGain. Clamps to [-25, 700], then
// writes 0x67f (0, or 0x11 when the requested gain is negative — the SDK shifts a
// negative gain up by 25 and flags it). The analog code is
//
//	code = 4095 * (1 - 10^(-gain/200))
//
// (base 4095 − 4095·10^(gain/(10·−20))), written little-endian
// to BOTH 0x30/0x31 and 0x32/0x33. Conversion gain (the IMX571 HCG switch) is a clean
// 0x2f = 0 below gain 100 / 1 from 100..700; the analog code RESETS at that boundary
// (gain re-based to gain−100 before the code math). Above 460 a third segment quantises
// to the 60-step HCG grid, sets the 0x40 stage nibble, and re-bases again.
// imx571SetGain selects the vendor's gain encoding from the regmap's VID (same die, different
// per-vendor band structure); an unrecognized vendor is an error (no implicit default).
func imx571SetGain(rm Regmap, gain int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx571SetGainZWO(rm, gain)
	case POA.VID:
		return imx571SetGainPOA(rm, gain)
	default:
		return fmt.Errorf("asicam: imx571 gain: unsupported vendor VID 0x%04x", rm.VID())
	}
}

// imx571SetGainPOA is PlayerOne's IMX571 gain encoding with the gain-threshold M = 125.
// Simpler than the 455: four bands, conv-gain register 0x2f (not 0x2d),
// no 0x3a4/5/6 high-config, code → reg 0x30 block.
// 0x67f routes via CrypWrite in poaRegmap. UNVERIFIED — no PlayerOne hardware.
//
//	gain      rebased g   0x2f         0x67f
//	0..4      g+30        0            0x22
//	5..29     g-5         0            0x11
//	30..124   g-30        0            0
//	125..229  g-125       1            0
//	230+      g-125       0x11         0
//	code = trunc(4095·(1-10^(-g/200))) clamped 4095 -> [lo,hi,lo,hi] block 0x30/0x31/0x32/0x33.
func imx571SetGainPOA(rm Regmap, gain int) error {
	if gain < 0 {
		gain = 0
	}
	const m = 125 // gain threshold M

	var conv, setup, g uint16 = 0, 0, 0
	switch {
	case gain <= 4:
		setup, g = 0x22, uint16(gain+30)
	case gain <= 29:
		setup, g = 0x11, uint16(gain-5)
	case gain < m: // 30..124
		g = uint16(gain - 30)
	default: // gain >= 125: conv 1 -> 0x11 once (gain-M) exceeds 104
		g = uint16(gain - m)
		if gain-m > 104 {
			conv = 0x11
		} else {
			conv = 1
		}
	}

	code := int(4095.0 * (1.0 - math.Pow(10, float64(g)/-200.0)))
	if code > 4095 {
		code = 4095
	}
	codeU := uint16(code)

	for _, rv := range []RegVal{
		{Reg: 0x2f, Val: conv},
		{Reg: 0x67f, Val: setup},
		{Reg: 0x30, Val: codeU & 0xff}, {Reg: 0x31, Val: (codeU >> 8) & 0xff},
		{Reg: 0x32, Val: codeU & 0xff}, {Reg: 0x33, Val: (codeU >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return nil
}

func imx571SetGainZWO(rm Regmap, gain int) error {
	if gain > imx571GainMax {
		gain = imx571GainMax
	}
	if gain < imx571GainMin {
		gain = imx571GainMin
	}
	setup := uint16(0)
	conv := uint16(0)   // 0x2f
	convHi := uint16(0) // 0x40
	switch {
	case gain < 0:
		// Negative gain: SDK shifts up by 25 and sets the 0x67f flag.
		gain += -imx571GainMin
		setup = 0x11
	case gain < imx571GainHCGAt:
		// Low conversion gain, code from raw gain.
	case gain <= imx571GainHCGHi:
		conv = 1
		gain -= imx571GainHCGAt
	default:
		// Extra-high segment (gain > 460): coarse 60-step stage, identical in
		// form to the IMX455 top band. The quotient input is BYTE-WRAPPED — (gain+52)
		// & 0xff before the *137>>13 reciprocal-divide — while the remainder
		// uses the full gain+52. Omitting the &0xff
		// over-counts the stage for gain > 460 and drives the re-based code negative.
		v := (gain + 0x34) & 0xff
		q := (v * 0x89) >> 13
		if ((gain+0x34)-q*0x3c)&0xff != 0 {
			q++
		}
		convHi = uint16(q) << 4 // 0x40 bits[4:7]
		conv = 1
		gain = gain - q*0x3c - 0x64
	}
	if err := rm.WriteReg(imx571RegGainSetup, setup); err != nil {
		return err
	}
	// code = 4094 * (1 - 10^(gain/(10*-20)))
	lin := math.Pow(10, float64(gain)/10.0/-20.0)
	code := uint16(int32(float64(imx571GainCodeFS) - float64(imx571GainCodeFS)*lin))
	for _, rv := range []RegVal{
		{Reg: imx571RegGainAL, Val: code & 0xff},
		{Reg: imx571RegGainAH, Val: (code >> 8) & 0xff},
		{Reg: imx571RegGainBL, Val: code & 0xff},
		{Reg: imx571RegGainBH, Val: (code >> 8) & 0xff},
		{Reg: imx571RegConvLow, Val: conv},
		{Reg: imx571RegConvHi, Val: convHi & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return nil
}

// imx571SetExposure — SetExp (CalcFrameTime, SetCMOSClk).
// Indexed full-frame rolling-shutter model:
//
//	lines = exposure / lineTime,  lineTime_ns = HMAX*1000/clock (0x546*1000/0x4e20)
//	VMAX  = max(DefaultVMAX, lines + offset + 1)   // FPGA frame length (SetFPGAVMAX)
//	SHS   = VMAX + offset - lines                  // offset = -1, clamp >= 1
//	write SHS little-endian to the indexed regs 0x18/0x19, latched by 0x07
//
// VMAX is programmed via the camera FPGA (SetVMAX → regs 0x10/0x11/0x12); the 16-bit
// SHS is written to the small indexed sensor regs 0x18/0x19. Unlike the IMX455,
// SetExp here does not halve SHS for binning — the ×2/×4 height is already folded
// into VMAX (the branch only clamps SHS to >=1). The base-mode line time is
// used; other readout modes (USB2, high-speed) re-derive the clock/HMAX in
// SetCMOSClk (calibration seam — see ShutterModel).
func imx571SetExposure(rm Regmap, d time.Duration) error {
	bin := ModeOf(rm).Bin
	if bin < 1 {
		bin = 1
	}
	mode := imx571SelectMode(bin)
	// lineTime_ns = V * 1e6 / clockHz (V from the binned readout mode); bin 1 → 1350·50 = 67500.
	lineNs := uint64(mode.v) * 1_000_000 / imx571ClockHz
	// effHeight = output rows = MaxHeight/bin, except bin 4 = MaxHeight/2 (output rows<<1).
	effH := imx571FullHeight / bin
	if bin == 4 {
		effH = imx571FullHeight / 2
	}
	defVMAX := int64(mode.vblank + effH) // vblank + effHeight

	lines := ExposureLines(d, lineNs, imx571ExpMinUs, imx571ExpMaxUs)
	vmax := uint32(defVMAX)
	if defVMAX < int64(lines)+imx571SHSOffset+1 {
		vmax = uint32(int64(lines) + imx571SHSOffset + 1)
	}
	if err := SetVMAX(rm, vmax); err != nil {
		return err
	}
	shs := int64(vmax) + imx571SHSOffset - int64(lines) // SHS = VMAX - 1 - lines; NO halving
	if shs < 1 {
		shs = 1
	}
	return WriteRegLE(rm, imx571RegApply, []uint16{imx571RegSHS0, imx571RegSHS1}, uint32(shs))
}

// imx571SetROI — SetStartPos (X→0xa7/0xa8/0xa9 with X
// aligned to 16 and written as bits[4:12]/[12:20]; Y+0x19→0x08/0x09) plus the
// window size from Cam_SetResolution (W→0x0a/0x0b, H→0x1dd/0x1de, height
// 4-aligned). The coupled group is applied via 0x07.
func imx571SetROI(rm Regmap, x, y, w, h, bin int) error {
	if bin < 1 {
		bin = 1
	}
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	// (x,y,w,h) are BINNED OUTPUT pixels. Apply this bin's readout-mode table (InitSensorMode),
	// then the ROI. START is SENSOR pixels (= binned·bin); window mode 0x1d8 = 4 full
	// / 0 binned; HEIGHT(0x0a) takes the raw output height +2 when binned, WIDTH(0x1dd) the
	// ×4-aligned output width +0x18.
	mode := imx571SelectMode(bin)
	if err := rm.WriteReg(imx571RegApply, 1); err != nil {
		return err
	}
	for _, rv := range mode.table {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}

	sx := x * bin
	sy := y * bin
	sx &^= imx571StartXAlign - 1 // align X to 16
	if bin == 2 || bin == 4 {
		sy &^= 3 // align Y to 4 for binned
	} else {
		sy &^= 1 // align Y to 2
	}
	ux := uint16(sx)
	uy := uint16(sy + imx571StartYOff)

	winMode := uint16(4)
	heightAdj := h
	if bin > 1 {
		winMode = 0       // binned window mode
		heightAdj = h + 2 // +2 for binned
	}
	uh := uint16(heightAdj)       // HEIGHT → 0x0a/0x0b, raw output height (+2 binned)
	wAligned := (w + 3) &^ 3      // WIDTH rounded up to ×4
	uw := uint16(wAligned + 0x18) // WIDTH → 0x1dd/0x1de, +0x18, low &0xfc

	for _, rv := range []RegVal{
		// ROI start (sensor pixels).
		{Reg: imx571RegStartXEn, Val: 0x01},
		{Reg: imx571RegStartXL, Val: (ux >> 4) & 0xff},
		{Reg: imx571RegStartXH, Val: (ux >> 12) & 0xff},
		{Reg: imx571RegStartYL, Val: uy & 0xff},
		{Reg: imx571RegStartYH, Val: (uy >> 8) & 0xff},
		// Window size (output pixels).
		{Reg: imx571RegWinMode, Val: winMode},
		{Reg: imx571RegHeightL, Val: uh & 0xff},
		{Reg: imx571RegHeightH, Val: (uh >> 8) & 0xff},
		{Reg: imx571RegWidthL, Val: uw & 0xfc},
		{Reg: imx571RegWidthH, Val: (uw >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	if err := rm.WriteReg(imx571RegApply, 0); err != nil {
		return err
	}
	// FPGA frame geometry (Cam_SetResolution: SetFPGAHeight/SetFPGAWidth) =
	// the OUTPUT dims — the FX3 transfer size.
	if err := FPGAWrite16(rm, 0x04, 0x05, uint16(w)); err != nil {
		return err
	}
	if err := FPGAWrite16(rm, 0x08, 0x09, uint16(h)); err != nil {
		return err
	}
	// SetFPGABinDataLen (Cam_SetResolution): per-frame DMA word count = output_area·bpp/4
	// (FPGA 0x40..0x43).
	bpp := ModeOf(rm).BytesPerPx
	return SetFPGABinDataLen(rm, uint32((w*h*bpp+3)/4))
}

// IMX571 per-mode register tables (InitSensorMode). Verbatim dumps of the reg_*
// tables (each entry {reg16,val16}, 53 entries); byte-identical in the MM_Pro variant.
var imx571ModeFull16 = []RegVal{ // reg_full_16bit
	{Reg: 0x0001, Val: 0x00}, {Reg: 0x0002, Val: 0x80}, {Reg: 0x002a, Val: 0x0a}, {Reg: 0x0324, Val: 0x01},
	{Reg: 0x0325, Val: 0x0f}, {Reg: 0x0069, Val: 0x30}, {Reg: 0x006c, Val: 0xe6}, {Reg: 0x006d, Val: 0x00},
	{Reg: 0x00d6, Val: 0x58}, {Reg: 0x00d8, Val: 0x58}, {Reg: 0x00db, Val: 0x60}, {Reg: 0x00dd, Val: 0x60},
	{Reg: 0x00df, Val: 0x01}, {Reg: 0x00e2, Val: 0x22}, {Reg: 0x02d3, Val: 0x00}, {Reg: 0x050f, Val: 0x5a},
	{Reg: 0x0512, Val: 0x70}, {Reg: 0x0513, Val: 0x70}, {Reg: 0x0514, Val: 0x70}, {Reg: 0x0515, Val: 0x01},
	{Reg: 0x0517, Val: 0x5a}, {Reg: 0x051f, Val: 0x69}, {Reg: 0x0553, Val: 0x85}, {Reg: 0x0574, Val: 0x02},
	{Reg: 0x0575, Val: 0x02}, {Reg: 0x0576, Val: 0x02}, {Reg: 0x0577, Val: 0x02}, {Reg: 0x0581, Val: 0x00},
	{Reg: 0x0582, Val: 0x16}, {Reg: 0x0583, Val: 0x16}, {Reg: 0x0584, Val: 0x16}, {Reg: 0x0585, Val: 0x16},
	{Reg: 0x059a, Val: 0x00}, {Reg: 0x0603, Val: 0x50}, {Reg: 0x0605, Val: 0x50}, {Reg: 0x062a, Val: 0x8b},
	{Reg: 0x062c, Val: 0x34}, {Reg: 0x0630, Val: 0x89}, {Reg: 0x0632, Val: 0x34}, {Reg: 0x0646, Val: 0x85},
	{Reg: 0x064a, Val: 0x85}, {Reg: 0x066d, Val: 0x11}, {Reg: 0x0670, Val: 0x11}, {Reg: 0x0673, Val: 0x11},
	{Reg: 0x0676, Val: 0x11}, {Reg: 0x067e, Val: 0x00}, {Reg: 0x068a, Val: 0x22}, {Reg: 0x0a2f, Val: 0x8f},
	{Reg: 0x0a30, Val: 0x01}, {Reg: 0x0a31, Val: 0x8f}, {Reg: 0x0a32, Val: 0x01}, {Reg: 0x0a36, Val: 0x8f},
	{Reg: 0x0a37, Val: 0x01},
}
var imx571ModeBin2w12 = []RegVal{ // reg_bin2w_12bit
	{Reg: 0x0001, Val: 0x05}, {Reg: 0x0002, Val: 0x54}, {Reg: 0x002a, Val: 0x04}, {Reg: 0x0069, Val: 0x00},
	{Reg: 0x006c, Val: 0x22}, {Reg: 0x006d, Val: 0x01}, {Reg: 0x00d6, Val: 0x53}, {Reg: 0x00d8, Val: 0x53},
	{Reg: 0x00db, Val: 0x5a}, {Reg: 0x00dd, Val: 0x5a}, {Reg: 0x00df, Val: 0x00}, {Reg: 0x00e2, Val: 0x88},
	{Reg: 0x02d3, Val: 0x00}, {Reg: 0x0324, Val: 0x01}, {Reg: 0x0325, Val: 0x0f}, {Reg: 0x050f, Val: 0x59},
	{Reg: 0x0512, Val: 0xbf}, {Reg: 0x0513, Val: 0xbf}, {Reg: 0x0514, Val: 0xbf}, {Reg: 0x0515, Val: 0x00},
	{Reg: 0x0517, Val: 0x50}, {Reg: 0x051f, Val: 0x5f}, {Reg: 0x0553, Val: 0x7b}, {Reg: 0x0574, Val: 0x0f},
	{Reg: 0x0575, Val: 0x0f}, {Reg: 0x0576, Val: 0x0f}, {Reg: 0x0577, Val: 0x0f}, {Reg: 0x0581, Val: 0x04},
	{Reg: 0x0582, Val: 0x24}, {Reg: 0x0583, Val: 0x24}, {Reg: 0x0584, Val: 0x24}, {Reg: 0x0585, Val: 0x24},
	{Reg: 0x059a, Val: 0x04}, {Reg: 0x0603, Val: 0x4b}, {Reg: 0x0605, Val: 0x4b}, {Reg: 0x062a, Val: 0x81},
	{Reg: 0x062c, Val: 0x52}, {Reg: 0x0630, Val: 0x7f}, {Reg: 0x0632, Val: 0x52}, {Reg: 0x0646, Val: 0x7b},
	{Reg: 0x064a, Val: 0x7b}, {Reg: 0x066d, Val: 0x00}, {Reg: 0x0670, Val: 0x00}, {Reg: 0x0673, Val: 0x00},
	{Reg: 0x0676, Val: 0x00}, {Reg: 0x067e, Val: 0x04}, {Reg: 0x068a, Val: 0x88}, {Reg: 0x0a2f, Val: 0x95},
	{Reg: 0x0a30, Val: 0x00}, {Reg: 0x0a31, Val: 0x95}, {Reg: 0x0a32, Val: 0x00}, {Reg: 0x0a36, Val: 0x95},
	{Reg: 0x0a37, Val: 0x00},
}
var imx571ModeBin3w12 = []RegVal{ // reg_bin3w_12bit
	{Reg: 0x0001, Val: 0x07}, {Reg: 0x0002, Val: 0x54}, {Reg: 0x002a, Val: 0x04}, {Reg: 0x0069, Val: 0x00},
	{Reg: 0x006c, Val: 0x22}, {Reg: 0x006d, Val: 0x01}, {Reg: 0x00d6, Val: 0x53}, {Reg: 0x00d8, Val: 0x53},
	{Reg: 0x00db, Val: 0x5a}, {Reg: 0x00dd, Val: 0x5a}, {Reg: 0x00df, Val: 0x00}, {Reg: 0x00e2, Val: 0x88},
	{Reg: 0x02d3, Val: 0x00}, {Reg: 0x0324, Val: 0x01}, {Reg: 0x0325, Val: 0x0f}, {Reg: 0x050f, Val: 0x59},
	{Reg: 0x0512, Val: 0xbf}, {Reg: 0x0513, Val: 0xbf}, {Reg: 0x0514, Val: 0xbf}, {Reg: 0x0515, Val: 0x00},
	{Reg: 0x0517, Val: 0x50}, {Reg: 0x051f, Val: 0x5f}, {Reg: 0x0553, Val: 0x7b}, {Reg: 0x0574, Val: 0x0f},
	{Reg: 0x0575, Val: 0x0f}, {Reg: 0x0576, Val: 0x0f}, {Reg: 0x0577, Val: 0x0f}, {Reg: 0x0581, Val: 0x04},
	{Reg: 0x0582, Val: 0x24}, {Reg: 0x0583, Val: 0x24}, {Reg: 0x0584, Val: 0x24}, {Reg: 0x0585, Val: 0x24},
	{Reg: 0x059a, Val: 0x04}, {Reg: 0x0603, Val: 0x4b}, {Reg: 0x0605, Val: 0x4b}, {Reg: 0x062a, Val: 0x81},
	{Reg: 0x062c, Val: 0x52}, {Reg: 0x0630, Val: 0x7f}, {Reg: 0x0632, Val: 0x52}, {Reg: 0x0646, Val: 0x7b},
	{Reg: 0x064a, Val: 0x7b}, {Reg: 0x066d, Val: 0x00}, {Reg: 0x0670, Val: 0x00}, {Reg: 0x0673, Val: 0x00},
	{Reg: 0x0676, Val: 0x00}, {Reg: 0x067e, Val: 0x04}, {Reg: 0x068a, Val: 0x88}, {Reg: 0x0a2f, Val: 0x95},
	{Reg: 0x0a30, Val: 0x00}, {Reg: 0x0a31, Val: 0x95}, {Reg: 0x0a32, Val: 0x00}, {Reg: 0x0a36, Val: 0x95},
	{Reg: 0x0a37, Val: 0x00},
}
