// WIP. The IMX462 is a STARVIS Type-1/2.8 sibling of the IMX290 — the
// register map is byte-identical to the IMX290MM Mini except three deltas: the HCG threshold
// (gain 80 vs 60), the init reglist values, and init tail 0x305f (0x01 vs 0x00). The structure
// (gain via 0x3009 conv-gain + 0x3014 code, SHS 0x3020-22, offset 0x300a/0b, STARVIS ROI, the
// host-timed trigger worker) mirrors the hardware-validated imx290. The user HAS an ASI462 →
// validate with gosnap (gain sweep, exposure, ROI). Cross-checkable vs PlayerOne POAImx462.
//
// Op map:
//
//	InitCamera        (reglist 73 entries + explicit tail + FPGA bringup)
//	SetCMOSClk        (clock 9281 kHz; HMAX floor 203, same as 290)
//	Cam_SetResolution (mode 0x3006, window W 0x3042/0x3043 H 0x303e/0x303f, FPGA HBLK/VBLK 9/W/H)
//	SetStartPos       (ROI X 0x3040/0x3041, Y 0x303c/0x303d; X align 4, Y align 2)
//	SetGain           (clamp 0..600, HCG bit 0x10 in 0x3009 ABOVE gain 80, code to 0x3014)
//	SetExp            (SHS 24-bit 0x3020..0x3022, VMAX via SetFPGAVMAX; +18/+17 math)
//	SetBrightness     (offset -> 0x300a/0x300b, 16-bit LE, no scaling)
//	the capture worker (sensor stream gate: standby 0x3000 = 1 stop / 0 start)
package sensors

import . "asicam"

import (
	"fmt"
	"time"
)

const (
	imx462RegLatch    = 0x3001 // write 1 before / 0 after a coupled register group
	imx462RegStandby  = 0x3000 // 1 = sensor standby, 0 = streaming (the capture worker gate)
	imx462RegGainMode = 0x3009 // conversion-gain select; bit 0x10 = high conversion gain
	imx462RegGainCode = 0x3014 // analog gain code (single byte)

	imx462RegStartXL = 0x3040
	imx462RegStartXH = 0x3041
	imx462RegStartYL = 0x303c
	imx462RegStartYH = 0x303d

	imx462RegWidthL  = 0x3042
	imx462RegWidthH  = 0x3043
	imx462RegHeightL = 0x303e
	imx462RegHeightH = 0x303f
	imx462RegMode    = 0x3006 // WINMODE/VMODE byte; 0x22 for the 2× readout, else 0x00

	imx462RegSHS0 = 0x3020
	imx462RegSHS1 = 0x3021
	imx462RegSHS2 = 0x3022

	imx462GainMax   = 600           // 60.0 dB ceiling, ASI 0.1 dB units (SetGain clamp)
	imx462GainHCGAt = 80            // 8.0 dB: ABOVE this, high conversion gain (0x3009 bit 0x10)
	imx462ExpMinUs  = 32            // µs floor (mirrors the STARVIS SetExp clamp)
	imx462ExpMaxUs  = 2_000_000_000 // 2000 s ceiling
	imx462LongExpUs = 1_000_000     // ≥ 1 s enters FPGA trigger mode (reg0 bit7)

	// Die/mode readout facts the shared engine (fps.go / shutter.go) is parameterized by.
	imx462FullWidth   = 1936 // SDK MaxWidth — HARDWARE-CONFIRMED (ASI462MC reports 1936×1096, same array as the 290)
	imx462FullHeight  = 1096
	imx462ClkKHz      = 18562 // RAW16 pixel clock (INCK/2). RAW8 would be 37124; the prior 9281 is the special-mode value, wrong for the normal RAW16 readout
	imx462HMAXFloor   = 261   // 12-bit normal-mode line-time floor for clock 18562. The prior 203 is the 9281-clock special-mode value — it under-times the 12-bit single-slope ADC ramp, over-driving the sensor past its rated 63.9 fps full-res and bending highlight linearity.
	imx462ClkKHzHS    = 37124 // 10-bit high-speed pixel clock: 2× the 12-bit clock — the 10-bit ADC ramp is ~4× shorter, so it sustains the faster clock cleanly
	imx462HMAXFloorHS = 245   // high-speed line-time floor (clock 37124) → ~136 fps full-res RAW8
	imx462VBlankAdd   = 18    // VMAX = height + 0x12
	imx462SHSOffset   = 17    // SHS  = (height + 0x11) − exposureLines
	imx462HBLK        = 0     // SetFPGAHBLK(0)
	imx462VBLK        = 9     // SetFPGAVBLK(9)
)

// imx462Init is InitCamera's sensor-write sequence: the 73-entry reglist
// ([reg:u16le][val:u16le], reg 0xffff = delay ms), then the explicit
// sensor-reg tail the function issues after the table loop. The FPGA-side
// bringup is in imx462InitFPGA; SendCMD(0xAF/0xAE) is Camera-level.
var imx462Init = []RegVal{
	// --- reglist table (verbatim, file order) ---
	{Reg: 0x3003, Val: 0x01}, {Reg: 0xffff, Val: 20}, // delay 20 ms
	{Reg: 0x3000, Val: 0x01}, {Reg: 0x3002, Val: 0x01}, {Reg: 0x3009, Val: 0x01}, {Reg: 0x300e, Val: 0x00},
	{Reg: 0x300f, Val: 0x00}, {Reg: 0x3010, Val: 0x21}, {Reg: 0x3011, Val: 0x02}, {Reg: 0x3012, Val: 0x64},
	{Reg: 0x3014, Val: 0x14}, {Reg: 0x3016, Val: 0x09}, {Reg: 0x3020, Val: 0x26}, {Reg: 0x3021, Val: 0x06},
	{Reg: 0x3046, Val: 0xf1}, {Reg: 0x304d, Val: 0x00}, {Reg: 0x305c, Val: 0x18}, {Reg: 0x305e, Val: 0x20},
	{Reg: 0x305f, Val: 0x01}, {Reg: 0x3070, Val: 0x02}, {Reg: 0x3071, Val: 0x11}, {Reg: 0x309b, Val: 0x10},
	{Reg: 0x309c, Val: 0x21}, {Reg: 0x309e, Val: 0x4a}, {Reg: 0x309f, Val: 0x4a}, {Reg: 0x30a2, Val: 0x02},
	{Reg: 0x30a6, Val: 0x20}, {Reg: 0x30a8, Val: 0x20}, {Reg: 0x30aa, Val: 0x20}, {Reg: 0x30ac, Val: 0x20},
	{Reg: 0x30b0, Val: 0x43}, {Reg: 0x3119, Val: 0x9e}, {Reg: 0x311c, Val: 0x1e}, {Reg: 0x311e, Val: 0x08},
	{Reg: 0x3128, Val: 0x05}, {Reg: 0x313d, Val: 0x83}, {Reg: 0x3150, Val: 0x03}, {Reg: 0x315e, Val: 0x1a},
	{Reg: 0x3164, Val: 0x1a}, {Reg: 0x317c, Val: 0x00}, {Reg: 0x317e, Val: 0x00}, {Reg: 0x3257, Val: 0x03},
	{Reg: 0x3264, Val: 0x1a}, {Reg: 0x3265, Val: 0x2b}, {Reg: 0x3266, Val: 0x00}, {Reg: 0x326b, Val: 0x10},
	{Reg: 0x3274, Val: 0x2b}, {Reg: 0x3275, Val: 0xa0}, {Reg: 0x3276, Val: 0x02}, {Reg: 0x32aa, Val: 0x0d},
	{Reg: 0x32b8, Val: 0x50}, {Reg: 0x32b9, Val: 0x10}, {Reg: 0x32ba, Val: 0x00}, {Reg: 0x32bb, Val: 0x04},
	{Reg: 0x32c8, Val: 0x50}, {Reg: 0x32c9, Val: 0x10}, {Reg: 0x32ca, Val: 0x00}, {Reg: 0x32cb, Val: 0x04},
	{Reg: 0x332c, Val: 0xd3}, {Reg: 0x332d, Val: 0x10}, {Reg: 0x332e, Val: 0x0d}, {Reg: 0x3358, Val: 0x06},
	{Reg: 0x3359, Val: 0xe1}, {Reg: 0x335a, Val: 0x11}, {Reg: 0x3360, Val: 0x1e}, {Reg: 0x3361, Val: 0x61},
	{Reg: 0x3362, Val: 0x10}, {Reg: 0x33b0, Val: 0x50}, {Reg: 0x33b2, Val: 0x1a}, {Reg: 0x33b3, Val: 0x04},
	{Reg: 0x3410, Val: 0x1a}, {Reg: 0x3480, Val: 0x49}, {Reg: 0x3000, Val: 0x01},
	// --- explicit sensor-reg tail (InitCamera), overrides the reglist ---
	{Reg: 0x305c, Val: 0x20}, // INCKSEL
	{Reg: 0x305d, Val: 0x00},
	{Reg: 0x305e, Val: 0x20},
	{Reg: 0x305f, Val: 0x01}, // (462: 0x01, vs 290's 0x00)
	{Reg: 0x3046, Val: 0xf1}, // output interface
	{Reg: 0x3005, Val: 0x01}, // ADBIT = 12-bit
	{Reg: 0x303a, Val: 0x08},
	{Reg: 0x3007, Val: 0x40}, // WINMODE
	// (FPGAReset + SendCMD(0xAF) here in the SDK — see imx462InitFPGA / Camera.Init)
	{Reg: 0x3002, Val: 0x01}, // XMSTA master stop during init
	{Reg: 0x304b, Val: 0x00},
}

var IMX462 = Sensor{
	Name:     "IMX462", // ASI462 / Ceres 462M; Sony IMX462 STARVIS (mono die; MC adds a CFA)
	GainMax:  imx462GainMax,
	ExpMinUs: imx462ExpMinUs,
	ExpMaxUs: imx462ExpMaxUs,
	// ASI Brightness / black level. Default 100 to match the SDK: ASIInitCamera applies
	// Brightness=100 on the ASI462 (wire-confirmed SetBrightness 100-->100, and a back-to-back
	// long-exposure compare showed a constant ~1100 DN black-level offset vs gosnap@30 that
	// vanishes at matched offset 100). The offset→DN mapping itself matches the SDK byte-for-byte
	// (floor 41/71/141 in 12-bit at offsets 0/30/100); only the init DEFAULT was wrong (was 30).
	OffsetMax: 240, OffsetDef: 100,
	Info: CameraInfo{
		MaxWidth:  imx462FullWidth, // IMX462 active pixels (datasheet) — UNVERIFIED, confirm on HW
		MaxHeight: imx462FullHeight,
		PixelUm:   2.9,         // 2.9 µm pixel pitch (datasheet)
		BitDepth:  12,          // 12-bit ADC (ADBIT=12; tail 0x3005=0x01)
		Bayer:     "RGGB",      // CFA (color/MC model only); surfaced when Model.Color
		Bins:      []int{1, 2}, // 1× and 2× (mode 0x3006=0x22, window·bin) — UNVERIFIED
	},
	Init:        imx462Init,
	InitFPGA:    imx462InitFPGA,
	SetGain:     imx462SetGain,
	SetExposure: imx462SetExposure,
	SetOffset:   imx462SetOffset,
	SetROI:      imx462SetROI,
	StreamStop:  func(rm Regmap) error { return rm.WriteReg(imx462RegStandby, 1) }, // standby on (the capture worker)
	StreamStart: func(rm Regmap) error { return rm.WriteReg(imx462RegStandby, 0) }, // standby off
	Worker:      imx462Worker,
}

// imx462Worker — the capture worker, the host-timed single-shot capture. Same skeleton
// as the imx290 (STARVIS standby-0x3000 gate; ≥1 s uses FPGA trigger MODE/SIGNAL). UNVERIFIED.
func imx462Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()
	arm := func(full bool) error {
		if full {
			if err := ctl.VendorCmd(0xAA); err != nil {
				return err
			}
			if err := SetFPGABit(rm, 0x00, 0x10, true); err != nil { // FPGAStop: reg0 bit4
				return err
			}
		}
		if err := rm.WriteReg(imx462RegStandby, 1); err != nil { // standby (stop)
			return err
		}
		if full {
			if err := ctl.VendorCmd(0xA9); err != nil {
				return err
			}
		}
		if err := rm.WriteReg(imx462RegStandby, 0); err != nil { // stream (start)
			return err
		}
		time.Sleep(50 * time.Millisecond)
		// FPGAStart: clear bit4 (readout stop) AND bit6 (WaitMode). The wire capture shows the SDK
		// writes reg0 = 0x21 absolutely for a 100 ms capture — clearing the WaitMode bit that SetExposure
		// (ApplyExposure) sets. gosnap's RMW left bit6 set (reg0 = 0x61), parking the FPGA in wait mode
		// so no free-running frame ever arrived → flat field / read timeout.
		return SetFPGABit(rm, 0x00, 0x50, false) // FPGAStart → reg0 0x21
	}
	// EnableFPGATriggerSignal (reg 0x0b bit0): only the ≥1 s trigger band uses it. The SDK's 100 ms
	// capture (wire-confirmed) never touches reg 0x0b — the sensor free-runs (SHS-timed) and the FPGA
	// just starts and reads one frame. Driving it in normal mode was part of the flat-frame bug.
	trigger := func(on bool) error { return SetFPGABit(rm, 0x0b, 0x01, on) }
	longExp := exposure >= imx462LongExpUs*time.Microsecond // ≥ 1 s — matches SetExposure's trigger band

	if err := arm(true); err != nil {
		return 0, err
	}
	_ = ctl.ResetEndpoint()
	if longExp {
		if err := trigger(true); err != nil {
			return 0, err
		}
	}
	if !longExp {
		// Free-run (<1 s): the integration is SHS/VMAX-timed; this is just a pre-wait before the
		// blocking StreamFrame, not the integration itself.
		if w := exposure - 200*time.Millisecond; w > 0 {
			time.Sleep(w)
		}
	} else {
		// Trigger mode (≥1 s): the trigger-signal hold IS the integration, so wait the FULL
		// exposure (not exp−200 ms — that under-integrates the exactly-1 s boundary by 200 ms).
		for start := time.Now(); time.Since(start) < exposure; {
			time.Sleep(100 * time.Millisecond)
		}
	}
	if longExp {
		if err := trigger(false); err != nil {
			return 0, err
		}
	}
	// Windowed pump (not one-shot BulkRead): the FX3 holds the frame's final partial 1-MiB DMA
	// buffer until FPGABufReload, so a plain bulk read returns ~4 MiB and stops ~49 KB short. The
	// SDK's startAsyncXfer keeps cycling EP 0x81 and flushes the tail — StreamFrame mirrors it.
	target := ctl.FrameBytes()
	if target > len(buf) {
		target = len(buf)
	}
	return ctl.StreamFrame(buf[:target], 500*time.Millisecond, exposure+5*time.Second)
}

// imx462InitFPGA — the FPGA-side bringup after the Sony init writes (InitCamera);
// identical to the 290.
func imx462InitFPGA(rm Regmap, subtype int) error {
	_ = subtype
	if err := FPGAClearBits(rm, 0x00, 0x01); err != nil { // FPGAReset
		return err
	}
	time.Sleep(20 * time.Millisecond)
	if err := FPGASetBits(rm, 0x00, 0x20); err != nil { // SetFPGAAsMaster(1): reg0 bit5
		return err
	}
	if err := FPGASetBits(rm, 0x00, 0x10); err != nil { // FPGAStop: reg0 bit4
		return err
	}
	if err := FPGASetBits(rm, 0x0a, 0x40); err != nil { // EnableFPGADDR(0): reg0xa bit6
		return err
	}
	adcOut := uint16(0x01) // bit0 = adc
	if ModeOf(rm).BytesPerPx >= 2 {
		adcOut |= 0x10 // bit4 = 1: 16-bit output (RAW16)
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x11, adcOut); err != nil {
		return err
	}
	if err := rm.WriteFPGAReg(0x01, 1); err != nil { // SetFPGAGain strobe
		return err
	}
	for _, r := range []uint16{0x0c, 0x0d, 0x0e, 0x0f} {
		if err := rm.WriteFPGAReg(r, 0x80); err != nil {
			return err
		}
	}
	if err := rm.WriteFPGAReg(0x01, 0); err != nil {
		return err
	}
	return rm.WriteFPGAReg(0x1a, 0x04)
}

// imx462SetGain — SetGain. Clamp [0,600], Sony analog-gain code (gain/3
// with HCG rebase ABOVE gain 80) via the shared SonyAnalogGain; sets/clears the 0x3009 HCG bit
// (0x10) and writes the code byte to 0x3014, latched by 0x3001. Identical to the 290 except the
// HCG threshold (80 vs 60).
func imx462SetGain(rm Regmap, gain int) error {
	if gain > imx462GainMax {
		gain = imx462GainMax
	}
	if gain < 0 {
		gain = 0
	}
	code, hcg := SonyAnalogGain(gain, imx462GainHCGAt)
	if err := rm.WriteReg(imx462RegLatch, 1); err != nil {
		return err
	}
	mode, err := rm.ReadReg(imx462RegGainMode)
	if err != nil {
		return err
	}
	if hcg {
		mode |= 0x10
	} else {
		mode &= 0x0f
	}
	// FRSEL (0x3009 bit0) is the pixel-clock / frame-rate select — 1 = 12-bit normal,
	// 0 = 10-bit high-speed (doubles the sensor readout clock to 37124). The reglist leaves
	// it 1; high-speed mode must clear it. Wire-confirmed (SDK high-speed ends on 0x3009=0x10
	// vs normal 0x11) — this is the register that actually engages the 2× clock, gain-shared
	// in 0x3009.
	if ModeOf(rm).HighSpeed {
		mode &^= 0x01
	}
	if err := rm.WriteReg(imx462RegGainMode, mode); err != nil {
		return err
	}
	if err := rm.WriteReg(imx462RegGainCode, code&0xff); err != nil {
		return err
	}
	return rm.WriteReg(imx462RegLatch, 0)
}

var imx462Shutter = ShutterModel{
	SHS0: imx462RegSHS0, SHS1: imx462RegSHS1, SHS2: imx462RegSHS2,
	SHSOffset:          imx462SHSOffset,
	MinExpUs:           imx462ExpMinUs,
	MaxExpUs:           imx462ExpMaxUs,
	Clock:              imx462ClkKHz,
	FloorHMAX:          imx462HMAXFloor,
	HighSpeedClock:     imx462ClkKHzHS,
	HighSpeedFloorHMAX: imx462HMAXFloorHS,
	VBlankAdd:          imx462VBlankAdd,
	DefaultWidth:       imx462FullWidth,
	DefaultHeight:      imx462FullHeight,
}

// imx462ClockFloor returns the pixel clock + HMAX floor for the live readout mode: the
// 10-bit high-speed pair (37124/245) when mode.HighSpeed, else 12-bit normal (18562/261).
func imx462ClockFloor(rm Regmap) (clock, floor int) {
	if ModeOf(rm).HighSpeed {
		return imx462ClkKHzHS, imx462HMAXFloorHS
	}
	return imx462ClkKHz, imx462HMAXFloor
}

// imx462SetExposure — STARVIS VMAX/SHS (ApplyExposure) + FPGA trigger MODE (reg0 bit7) for ≥1 s,
// same as the 290 (SetExp).
//
// HMAX-stretch: the SDK throttles the readout line clock (the FPS-percent throttle → SetFPGAHMAX,
// HMAX) so a long exposure fits inside one default-length frame, then carves
// the integration with SHS — VMAX stays at height+18, SHS = (height+17)−exposureLines.
// gosnap's ApplyExposure computes its line time from the SAME HMAX formula but never
// WROTE that HMAX to the FPGA, so the FPGA ran at the stale fast line rate (HMAX floor):
// the exposure then overflowed one frame and collapsed to the VMAX-stretch + SHS=1 path.
// Program the computed HMAX here first so the FPGA line rate and the SHS math agree.
// With clock 18562 + the USB2 FPSPercent=40 this reproduces the SDK exactly: HMAX 4085,
// VMAX 1114, SHS 659 for a 100 ms exposure.
func imx462SetExposure(rm Regmap, d time.Duration) error {
	trigger := d >= imx462LongExpUs*time.Microsecond
	if err := SetFPGABit(rm, 0x00, 0x80, trigger); err != nil {
		return err
	}
	// HMAX (line time) from the live ROI dimensions when set, so a sub-frame ROI gets the
	// shorter line period it can sustain (the bandwidth throttle scales with width); falls
	// back to full-frame. Matches the ROI HMAX that SetROI's ProgramHMAX programmed.
	hw, hh := imx462FullWidth, imx462FullHeight
	if rd := ModeOf(rm); rd.Width > 0 {
		hw, hh = rd.Width, rd.Height
	}
	clk, floor := imx462ClockFloor(rm)
	hmax := HMAX(hw, hh, clk, floor, imx462VBlankAdd, ModeOf(rm))
	if err := WriteFPGAHMAX(rm, hmax); err != nil {
		return err
	}
	return ApplyExposure(rm, imx462Shutter, imx462RegLatch, d)
}

// imx462SetOffset — SetBrightness: offset 16-bit LE to 0x300a/0x300b, no scaling.
func imx462SetOffset(rm Regmap, offset int) error {
	v := uint16(offset)
	if err := rm.WriteReg(0x300b, (v>>8)&0xff); err != nil {
		return err
	}
	return rm.WriteReg(0x300a, v&0xff)
}

// imx462SetROI — SetStartPos (X→0x3040/41 align 4, Y→0x303c/3d align 2) + Cam_SetResolution
// (mode 0x3006, window→0x3042/3,0x303e/f, FPGA geometry). Identical to the 290.
func imx462SetROI(rm Regmap, x, y, w, h, bin int) error {
	if bin < 1 {
		bin = 1
	}
	if bin > 2 {
		return fmt.Errorf("imx462: bin %d not supported (HW does 1× and 2×)", bin)
	}
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	sx, sy := x*bin, y*bin
	sx &^= 3 // align X to 4
	sy &^= 1 // align Y to 2
	ux, uy := uint16(sx), uint16(sy)
	sw, sh := uint16(w*bin), uint16(h*bin)

	mode := uint16(0x00)
	if bin == 2 {
		mode = 0x22
	}

	if err := rm.WriteReg(imx462RegLatch, 1); err != nil {
		return err
	}
	for _, rv := range []RegVal{
		{Reg: imx462RegStartXL, Val: ux & 0xff}, {Reg: imx462RegStartXH, Val: (ux >> 8) & 0xff},
		{Reg: imx462RegStartYL, Val: uy & 0xff}, {Reg: imx462RegStartYH, Val: (uy >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	if err := rm.WriteReg(imx462RegLatch, 0); err != nil {
		return err
	}

	if err := rm.WriteReg(imx462RegMode, mode); err != nil {
		return err
	}
	for _, rv := range []RegVal{
		{Reg: imx462RegWidthL, Val: sw & 0xff}, {Reg: imx462RegWidthH, Val: (sw >> 8) & 0xff},
		{Reg: imx462RegHeightL, Val: sh & 0xff}, {Reg: imx462RegHeightH, Val: (sh >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}

	// SetOutput16Bits (shared with the 290 Mini): set the
	// sensor output bit-format. The function branches on the HIGH-SPEED mode flag: when it is
	// 0 (normal) OR b==1 (RAW16), it uses the 12-bit ADBIT block (0x3005=1, OUTFMT 0x3046=0xf1,
	// 0x3129=0, 0x317c=0, 0x31ec=0x0e) with FPGA ADC_BIT=1. The 10-bit reformat (high-speed
	// AND RAW8: 0x3046=0xf0, 0x3005=0, 0x3129=0x1d, 0x317c=0x12, ADC_BIT=0) is the high-speed path —
	// a shorter ADC ramp clocked 2× faster (imx462ClockFloor switches clock 18562→37124, floor
	// 261→245). Using it in normal mode reconfigures the sensor to a readout the 12-bit clock can't
	// stream (the RAW8 0-byte hang); using the 12-bit block at the 2× clock over-drives the ADC.
	out16 := []RegVal{{Reg: 0x3046, Val: 0xf1}, {Reg: 0x3005, Val: 0x01}, {Reg: 0x3129, Val: 0x00}, {Reg: 0x317c, Val: 0x00}, {Reg: 0x31ec, Val: 0x0e}}
	adcBit := uint16(0x01) // FPGA reg 0x0a bit0 (SetFPGAADCWidthOutputWidth's ADC_BIT): 1 = 12-bit
	if ModeOf(rm).HighSpeed && ModeOf(rm).BytesPerPx < 2 {
		out16 = []RegVal{{Reg: 0x3046, Val: 0xf0}, {Reg: 0x3005, Val: 0x00}, {Reg: 0x3129, Val: 0x1d}, {Reg: 0x317c, Val: 0x12}}
		adcBit = 0x00 // 10-bit high-speed
	}
	for _, rv := range out16 {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x01, adcBit); err != nil { // ADC_BIT bit0 = mode bit depth
		return err
	}

	if err := ProgramFrameGeometry(rm, w, h, imx462HBLK, imx462VBLK); err != nil {
		return err
	}
	clk, floor := imx462ClockFloor(rm)
	return ProgramHMAX(rm, w, h, clk, floor, imx462VBlankAdd)
}
