// Captures full 1936×1096 16-bit frames. At 0.2 s the driver writes HMAX 0x0662 / VMAX
// 0x0008e0 / SHS 1 / gain 0. The ≥1 s FPGA trigger-mode band (reg0 bit7) is required —
// without it the free-running sensor delivers a partial frame caught mid-readout.
//
// Unverified: absolute exposure-time accuracy; the HMAX floor (not exercised at full-frame,
// which is bandwidth-limited); the high-speed and binned readout modes. Every captured frame
// carries a 4-byte "7e 5a 01 00" start marker that capture.go does not strip (it searches for
// 0xBB00AA11) — likely this camera's frame header.
package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
	"time"
)

// Sony IMX290 — 1/2.8" STARVIS rolling-shutter CMOS (ZWO ASI290 family). FX3 FPGA register
// numbers come from the shared bridge.
//
// Op map:
//
//	InitCamera        (reglist table loop + explicit Sony writes + FPGA bringup)
//	SetCMOSClk        (clock constants -> pixel clock/REG_FRAME_LENGTH_PKG_MIN)
//	Cam_SetResolution (mode 0x3006, window W 0x3042/0x3043 H 0x303e/0x303f, FPGA HBLK/VBLK/W/H)
//	SetStartPos       (ROI X 0x3040/0x3041, Y 0x303c/0x303d; X align 4, Y align 2)
//	SetGain           (clamp 0..600, HCG bit 0x10 in 0x3009 above gain 60, code to 0x3014)
//	SetExp            (SHS 24-bit 0x3020..0x3022, VMAX via FPGA SetFPGAVMAX)
//	capture worker    (sensor stream gate: standby 0x3000 = 1 stop / 0 start)
const (
	imx290RegLatch    = 0x3001 // write 1 before / 0 after a coupled register group
	imx290RegStandby  = 0x3000 // 1 = sensor standby, 0 = streaming
	imx290RegGainMode = 0x3009 // conversion-gain select; bit 0x10 = high conversion gain
	imx290RegGainCode = 0x3014 // analog gain code (single byte)

	// SetStartPos — X aligned to 4, Y to 2 before writing (WINPH/WINPV).
	imx290RegStartXL = 0x3040
	imx290RegStartXH = 0x3041
	imx290RegStartYL = 0x303c
	imx290RegStartYH = 0x303d

	// Cam_SetResolution — output window size × binning (WINWH/WINWV).
	imx290RegWidthL  = 0x3042
	imx290RegWidthH  = 0x3043
	imx290RegHeightL = 0x303e
	imx290RegHeightH = 0x303f
	imx290RegMode    = 0x3006 // WINMODE/VMODE byte; 0x22 for the 2× readout, else 0x00

	// SetExp — shutter (SHS), 24-bit little-endian.
	imx290RegSHS0 = 0x3020
	imx290RegSHS1 = 0x3021
	imx290RegSHS2 = 0x3022

	imx290GainMax   = 600           // 60.0 dB ceiling, in ASI 0.1 dB units (SetGain clamp)
	imx290GainHCGAt = 60            // 6.0 dB: above this, high conversion gain (0x3009 bit 0x10)
	imx290ExpMinUs  = 32            // µs floor (SetExp: exp <= 31 -> 32)
	imx290ExpMaxUs  = 2_000_000_000 // 2000 s ceiling (SetExp)
	imx290LongExpUs = 1_000_000     // ≥ 1 s enters FPGA trigger mode (reg0 bit7); see imx290SetExposure

	// Sensor-die readout constants the shared engine (fps.go / shutter.go) is parameterized by;
	// NOT runtime state (USB speed, FPS, output depth live in ReadoutMode from the Camera).
	// Clock+FloorHMAX feed the HMAX formula; VBlankAdd+SHSOffset the STARVIS VMAX/SHS math;
	// HBLK/VBLK the FPGA frame geometry. Line time is computed from these.
	imx290FullWidth  = 1936 // MaxWidth (full-frame exposure/HMAX assumption)
	imx290FullHeight = 1096 // MaxHeight
	// Pixel clock is mode-dependent: normal RAW16 = 18562, 10-bit high-speed = 37124, bin-2 =
	// 9281. RAW16 imaging runs Bin=1/SoftBin, so only normal/high-speed are modelled (bin-2 9281
	// isn't hit). Normal clock 18562: at 0.2 s HMAX = 0x0662. HMAX floor and high-speed pair not
	// yet exercised.
	imx290ClkKHz      = 18562 // normal RAW16 pixel clock
	imx290HMAXFloor   = 261   // line-time floor for 18562 (UNVERIFIED — not exercised at full-frame)
	imx290ClkKHzHS    = 37124 // 10-bit high-speed pixel clock — used only when ModeOf(rm).HighSpeed (UNVERIFIED)
	imx290HMAXFloorHS = 245   // high-speed line-time floor for 37124 (UNVERIFIED)
	imx290VBlankAdd   = 18    // VMAX = height + 18 (0x12) (SetExp)
	imx290SHSOffset   = 17    // SHS = (height + 17 (0x11)) - exposureLines (SetExp)
	imx290HBLK        = 0     // SetFPGAHBLK(0)  (Cam_SetResolution)
	imx290VBLK        = 9     // SetFPGAVBLK(9)  (Cam_SetResolution) — sensor-specific blanking
)

// imx290Init is the InitCamera sensor-write sequence: the 47-entry reglist table (reg/val16
// pairs; reg 0xffff = InitDelayReg, delay = val ms), then the explicit WriteSONYREG tail issued
// after the table loop.
//
// FPGA-side bringup (FPGAReset, 20 ms delay, SetFPGAAsMaster/FPGAStop/EnableFPGADDR/
// SetFPGAADCWidthOutputWidth/SetFPGAGain) is in imx290InitFPGA. The mid-init SendCMD(0xAF) and
// trailing SendCMD(0xAE) bank-select are Camera-level (Camera.Init around InitFPGA).
var imx290Init = []RegVal{
	// --- reglist table: verbatim reg/val16 pairs, 47 entries in file order. Opaque Sony
	// PLL/analog settings. reg 0xffff = InitDelayReg sentinel (delay = val ms). ---
	{Reg: 0x3003, Val: 0x01}, {Reg: 0xffff, Val: 20}, // delay 20 ms
	{Reg: 0x300e, Val: 0x00}, {Reg: 0x300f, Val: 0x00}, {Reg: 0x3010, Val: 0x21}, {Reg: 0x3012, Val: 0x64},
	{Reg: 0x3016, Val: 0x09}, {Reg: 0x305c, Val: 0x18}, {Reg: 0x305e, Val: 0x20}, {Reg: 0x315e, Val: 0x1a},
	{Reg: 0x3164, Val: 0x1a}, {Reg: 0x3070, Val: 0x02}, {Reg: 0x3071, Val: 0x11}, {Reg: 0x309b, Val: 0x10},
	{Reg: 0x30a2, Val: 0x02}, {Reg: 0x30a6, Val: 0x20}, {Reg: 0x30a8, Val: 0x20}, {Reg: 0x30aa, Val: 0x20},
	{Reg: 0x30ac, Val: 0x20}, {Reg: 0x30b0, Val: 0x43}, {Reg: 0x3119, Val: 0x9e}, {Reg: 0x311c, Val: 0x1e},
	{Reg: 0x311e, Val: 0x08}, {Reg: 0x3128, Val: 0x05}, {Reg: 0x313d, Val: 0x83}, {Reg: 0x3150, Val: 0x03},
	{Reg: 0x317e, Val: 0x00}, {Reg: 0x32b8, Val: 0x50}, {Reg: 0x32b9, Val: 0x10}, {Reg: 0x32ba, Val: 0x00},
	{Reg: 0x32bb, Val: 0x04}, {Reg: 0x32c8, Val: 0x50}, {Reg: 0x32c9, Val: 0x10}, {Reg: 0x32ca, Val: 0x00},
	{Reg: 0x32cb, Val: 0x04}, {Reg: 0x332c, Val: 0xd3}, {Reg: 0x332d, Val: 0x10}, {Reg: 0x332e, Val: 0x0d},
	{Reg: 0x3358, Val: 0x06}, {Reg: 0x3359, Val: 0xe1}, {Reg: 0x335a, Val: 0x11}, {Reg: 0x3360, Val: 0x1e},
	{Reg: 0x3361, Val: 0x61}, {Reg: 0x3362, Val: 0x10}, {Reg: 0x33b0, Val: 0x50}, {Reg: 0x33b2, Val: 0x1a},
	{Reg: 0x33b3, Val: 0x04},
	// --- explicit WriteSONYREG tail: the immediate (reg,val) pairs InitCamera issues
	// after the table loop, in order ---
	{Reg: 0x305c, Val: 0x20}, // INCKSEL
	{Reg: 0x305d, Val: 0x00},
	{Reg: 0x305e, Val: 0x20},
	{Reg: 0x305f, Val: 0x00},
	{Reg: 0x3046, Val: 0xf1}, // output interface
	{Reg: 0x3005, Val: 0x01}, // ADBIT = 12-bit
	{Reg: 0x303a, Val: 0x08},
	{Reg: 0x3007, Val: 0x40}, // WINMODE
	// (FPGAReset + 20 ms + SendCMD(0xAF) happen here — see imx290InitFPGA / Camera.Init)
	{Reg: 0x3002, Val: 0x01}, // XMSTA master stop during init
	{Reg: 0x304b, Val: 0x00},
}

var IMX290 = Sensor{
	Name:     "IMX290", // ASI290 family; Sony IMX290 STARVIS (mono die; MC adds a CFA)
	GainMax:  imx290GainMax,
	ExpMinUs: imx290ExpMinUs,
	ExpMaxUs: imx290ExpMaxUs,
	// ASI Brightness / black level. Caps 0..240, def 1.
	OffsetMax: 240, OffsetDef: 1,
	// Geometry/spec: IMX290 datasheet = ZWO SDK per-model CameraInfo (ASIGetCameraProperty).
	Info: CameraInfo{
		MaxWidth:  1936,        // IMX290 active pixels (datasheet / SDK MaxWidth)
		MaxHeight: 1096,        // IMX290 active pixels (datasheet / SDK MaxHeight); = output rows
		PixelUm:   2.9,         // 2.9 µm pixel pitch (datasheet / SDK PixelSize)
		BitDepth:  12,          // 12-bit ADC (ADBIT=12; init 0x3005=0x01, reglist tail)
		Bayer:     "RGGB",      // IMX290 CFA (color/MC model only); surfaced when Model.Color
		Bins:      []int{1, 2}, // 1× and 2× decoded (2× = mode 0x3006=0x22, window·bin); UNVERIFIED
	},
	Init:        imx290Init,
	InitFPGA:    imx290InitFPGA,
	SetGain:     imx290SetGain,
	SetExposure: imx290SetExposure,
	SetOffset:   imx290SetOffset,
	SetROI:      imx290SetROI,
	StreamStop:  func(rm Regmap) error { return rm.WriteReg(imx290RegStandby, 1) }, // standby on
	StreamStart: func(rm Regmap) error { return rm.WriteReg(imx290RegStandby, 0) }, // standby off
	Worker:      imx290Worker,
}

// imx290Worker is the host-timed single-shot capture. Same skeleton as the IMX174, but the
// STARVIS sensor gate is the 0x3000 standby register (1 = stop, 0 = stream); no 0x212/0x22e
// settle. For >1 s exposures SetExp arms trigger mode (reg0 bit7); this worker drives the
// trigger signal (EnableFPGATriggerSignal, FPGA reg 0x0b bit0) whose 1->0 edge ends the
// integration and releases the frame.
//
//	arm:    SendCMD(0xAA)·FPGAStop·0x3000=1·SendCMD(0xA9)·0x3000=0·usleep(50ms)·FPGAStart·ResetEndpoint
//	expose: EnableFPGATriggerSignal(1) · host-time (≤1 s: exposure−200 ms; >1 s: 100 ms poll)
//	fire:   EnableFPGATriggerSignal(0) · BulkRead
func imx290Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
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
		if err := rm.WriteReg(imx290RegStandby, 1); err != nil { // standby (stop)
			return err
		}
		if full {
			if err := ctl.VendorCmd(0xA9); err != nil {
				return err
			}
		}
		if err := rm.WriteReg(imx290RegStandby, 0); err != nil { // stream (start)
			return err
		}
		time.Sleep(50 * time.Millisecond)        // usleep(0xc350)
		return SetFPGABit(rm, 0x00, 0x10, false) // FPGAStart: reg0 bit4 clear
	}
	trigger := func(on bool) error { return SetFPGABit(rm, 0x0b, 0x01, on) } // EnableFPGATriggerSignal

	if err := arm(true); err != nil {
		return 0, err
	}
	_ = ctl.ResetEndpoint()
	if err := trigger(true); err != nil {
		return 0, err
	}
	if exposure < imx290LongExpUs*time.Microsecond {
		// Free-run (<1 s, reg0 bit7 clear): hold briefly, then re-arm and read the next free-run
		// frame. The standby toggle is safe here — no triggered charge to lose.
		if w := exposure - 200*time.Millisecond; w > 0 {
			time.Sleep(w)
		}
		_ = ctl.ResetEndpoint()
		if err := arm(false); err != nil {
			return 0, err
		}
	} else {
		// Trigger mode (≥1 s, reg0 bit7 set): host-time the integration by holding the trigger
		// signal high for the full exposure, then read the triggered frame directly. Do NOT
		// standby-toggle here — that clears the integrated charge and yields a dark frame.
		for start := time.Now(); time.Since(start) < exposure; {
			if ctl.Aborted() {
				return 0, errExposureAborted // StopExposure ran: bail
			}
			time.Sleep(100 * time.Millisecond)
		}
	}
	if err := trigger(false); err != nil {
		return 0, err
	}
	return ctl.BulkRead(buf, exposure+3*time.Second)
}

// imx290InitFPGA is the FPGA-side bringup InitCamera performs after the Sony init writes,
// using the FX3 register numbers. FPGASetBits/FPGAClearBits/FPGAWriteBits (imx174.go) do the
// read-modify-write of the FPGA mode registers.
//
//	FPGAReset                          reg0 bit0 -> 0
//	(20 ms delay; SendCMD(0xAF) — Camera-level)
//	SetFPGAAsMaster(1)                 reg0 bit5 = 1
//	FPGAStop                           reg0 bit4 = 1
//	EnableFPGADDR(0)                   reg0xa bit6 = 1 (DDR disabled)
//	SetFPGAADCWidthOutputWidth(1, 0)   reg0xa bit0 = 1 (adc), bit4 = 0 (output) — x2
//	SetFPGAGain(0x80,0x80,0x80,0x80)   FPGA 0x0c-0x0f, strobed by reg 1
//	WriteFPGAREG(0x1a, 0x04)           direct FPGA register write (InitCamera)
//
// InitCamera passes outputWidth = 0 (8-bit); the SDK raises reg0xa bit4 to 1 later for RAW16.
// Set here from the live ReadoutMode: with bit4 = 0 the ASI290 streams a complete half-size
// RAW8 frame (1936×1096×1 = 2121856 B) and the RAW16 read reports a short frame; bit4 = 1 gives
// full RAW16. SetCMOSClk()'s sensor 0x3009 clock-select write is a remaining seam (SetGain sets
// 0x3009).
func imx290InitFPGA(rm Regmap, subtype int) error {
	_ = subtype                                           // 290MM_Mini has no <0x12 / >=0x12 split in this path
	if err := FPGAClearBits(rm, 0x00, 0x01); err != nil { // FPGAReset
		return err
	}
	time.Sleep(20 * time.Millisecond)                   // usleep(0x4e20)
	if err := FPGASetBits(rm, 0x00, 0x20); err != nil { // SetFPGAAsMaster(1): reg0 bit5
		return err
	}
	if err := FPGASetBits(rm, 0x00, 0x10); err != nil { // FPGAStop: reg0 bit4
		return err
	}
	if err := FPGASetBits(rm, 0x0a, 0x40); err != nil { // EnableFPGADDR(0): reg0xa bit6
		return err
	}
	// SetFPGAADCWidthOutputWidth(adc=1, outputWidth): reg0xa bit0 = adc (1), bit4 = output
	// width. InitCamera passes outputWidth = 0 (8-bit); bit4 raised for RAW16 from the live
	// ReadoutMode so the FPGA streams 16-bit.
	adcOut := uint16(0x01) // bit0 = adc
	if ModeOf(rm).BytesPerPx >= 2 {
		adcOut |= 0x10 // bit4 = 1: 16-bit output (RAW16)
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x11, adcOut); err != nil {
		return err
	}
	// SetFPGAGain(0x80×4): FPGA 0x0c-0x0f, committed by the reg-1 strobe (1 before, 0 after).
	if err := rm.WriteFPGAReg(0x01, 1); err != nil {
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
	return rm.WriteFPGAReg(0x1a, 0x04) // InitCamera
}

// imx290SetGain — SetGain. Clamps the ASI gain to [0, GainMax], converts it to the Sony
// analog-gain code (0.1 dB -> 0.3 dB step, i.e. gain/3, with the high-conversion-gain rebase
// above GainHCGAt) via SonyAnalogGain, sets/clears the 0x3009 HCG bit (0x10) and writes the
// code byte to 0x3014. Latched by 0x3001.
func imx290SetGain(rm Regmap, gain int) error {
	if gain > imx290GainMax {
		gain = imx290GainMax
	}
	if gain < 0 {
		gain = 0
	}
	code, hcg := SonyAnalogGain(gain, imx290GainHCGAt)
	if err := rm.WriteReg(imx290RegLatch, 1); err != nil {
		return err
	}
	mode, err := rm.ReadReg(imx290RegGainMode)
	if err != nil {
		return err
	}
	if hcg {
		mode |= 0x10 // high conversion gain
	} else {
		mode &= 0x0f // keep low nibble, clear HCG
	}
	if err := rm.WriteReg(imx290RegGainMode, mode); err != nil {
		return err
	}
	if err := rm.WriteReg(imx290RegGainCode, code&0xff); err != nil {
		return err
	}
	return rm.WriteReg(imx290RegLatch, 0)
}

// imx290Shutter is the populated STARVIS template (SetExp). All values are sensor-die
// constants; ApplyExposure (shutter.go) computes the line time from the HMAX formula + the live
// ReadoutMode and applies: VMAX = height + VBlankAdd, SHS = clamp(height + SHSOffset - lines, 1,
// height + SHSOffset - 1), with VMAX stretched (SHS = 1) when the exposure exceeds one frame.
var imx290Shutter = ShutterModel{
	SHS0: imx290RegSHS0, SHS1: imx290RegSHS1, SHS2: imx290RegSHS2,
	SHSOffset:          imx290SHSOffset,
	MinExpUs:           imx290ExpMinUs,
	MaxExpUs:           imx290ExpMaxUs,
	Clock:              imx290ClkKHz,
	FloorHMAX:          imx290HMAXFloor,
	HighSpeedClock:     imx290ClkKHzHS,
	HighSpeedFloorHMAX: imx290HMAXFloorHS,
	VBlankAdd:          imx290VBlankAdd,
	DefaultWidth:       imx290FullWidth,
	DefaultHeight:      imx290FullHeight,
}

// imx290ClockFloor returns the pixel clock + HMAX floor for the live readout mode: the 10-bit
// high-speed pair (37124/245) when mode.HighSpeed, else normal RAW16 (18562/261).
func imx290ClockFloor(rm Regmap) (clock, floor int) {
	if ModeOf(rm).HighSpeed {
		return imx290ClkKHzHS, imx290HMAXFloorHS
	}
	return imx290ClkKHz, imx290HMAXFloor
}

// imx290SetExposure programs the STARVIS VMAX/SHS (ApplyExposure) and, for exposures ≥ 1 s,
// engages FPGA trigger mode (reg0 bit7, EnableFPGATriggerMode), making the FPGA hold the frame
// until the worker's trigger-signal 1→0 edge so a long exposure reads out as one complete frame
// from row 0. Without it, the VMAX-extended sensor free-runs and the worker fires mid-frame,
// delivering a partial frame (e.g. 1027 of 1096 rows at 2 s). Below 1 s the bit is cleared
// (short free-run path). The SDK also calls SelectExtTrigExp here; not reproduced.
func imx290SetExposure(rm Regmap, d time.Duration) error {
	trigger := d >= imx290LongExpUs*time.Microsecond            // ≥ 1 s
	if err := SetFPGABit(rm, 0x00, 0x80, trigger); err != nil { // EnableFPGATriggerMode (bit7); WaitMode (bit6) is set by ApplyExposure
		return err
	}
	// Program the computed HMAX so the FPGA line rate agrees with the SHS math: the pixel clock
	// drives the line time, so a stale fast HMAX would overflow one frame and collapse to the
	// VMAX-stretch / SHS=1 path. Use the live ROI dims when set, else full-frame.
	hw, hh := imx290FullWidth, imx290FullHeight
	if rd := ModeOf(rm); rd.Width > 0 {
		hw, hh = rd.Width, rd.Height
	}
	clk, floor := imx290ClockFloor(rm)
	if err := WriteFPGAHMAX(rm, HMAX(hw, hh, clk, floor, imx290VBlankAdd, ModeOf(rm))); err != nil {
		return err
	}
	// ≥1 s host-times the integration via the trigger-mode path in imx290Worker: hold the
	// trigger signal (reg0b bit0) high for the exposure, then read the triggered frame directly
	// — no standby toggle (that clears the integrated charge → dark frame). Below 1 s the worker
	// free-runs the next frame instead.
	return ApplyExposure(rm, imx290Shutter, imx290RegLatch, d)
}

// imx290SetROI programs the readout window: SetStartPos (ROI start X -> 0x3040/0x3041,
// Y -> 0x303c/0x303d; X aligned to 4, Y to 2) plus Cam_SetResolution (mode byte 0x3006,
// window W -> 0x3042/0x3043, H -> 0x303e/0x303f, and the FPGA frame geometry). Both sensor
// groups are bracketed by the 0x3001 latch. The FPGA geometry uses the FX3 setters'
// register map:
//
//	SetFPGAHBLK(0)   -> 0x02/0x03 (strobed)   horizontal blanking = 0
//	SetFPGAVBLK(9)   -> 0x06/0x07 (strobed)   vertical blanking = 9 (IMX290 value; sensor-specific)
//	SetFPGAWidth(w)  -> 0x04/0x05 (strobed)
//	SetFPGAHeight(h) -> 0x08/0x09 (strobed)
//
// Finally SetFPGAHMAX throttles the readout to USB bandwidth (FPGA 0x13/0x14, strobed);
// ProgramHMAX computes the value from the window geometry + the live ReadoutMode (USB speed /
// output depth / FPS%).
// imx290SetOffset — SetBrightness (ASI Brightness / black level): offset written 16-bit
// little-endian to sensor 0x300a (low) / 0x300b (high), no scaling.
func imx290SetOffset(rm Regmap, offset int) error {
	v := uint16(offset)
	if err := rm.WriteReg(0x300b, (v>>8)&0xff); err != nil {
		return err
	}
	return rm.WriteReg(0x300a, v&0xff)
}

func imx290SetROI(rm Regmap, x, y, w, h, bin int) error {
	if bin < 1 {
		bin = 1
	}
	if bin > 2 {
		return fmt.Errorf("imx290: bin %d not supported (HW does 1× and 2×)", bin)
	}
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	// (x,y,w,h) are binned output pixels. STARVIS binning (Cam_SetResolution): the sensor window
	// regs take the physical region (output·bin), the mode byte 0x3006 selects the 2× readout
	// (0x22, else 0x00); the FPGA frame geometry + HMAX take the output dims (what the FX3
	// transfers). Start is sensor pixels = output·bin (SetStartPos aligns X→4 / Y→2). SetExp is
	// unchanged — VMAX uses the full sensor height (invariant under binning) and the line time
	// follows the throttle on the output frame.
	sx, sy := x*bin, y*bin
	sx &^= 3 // align X to 4 (#0x7ffffffc)
	sy &^= 1 // align Y to 2 (#0x7ffffffe)
	ux, uy := uint16(sx), uint16(sy)
	sw, sh := uint16(w*bin), uint16(h*bin) // sensor physical window = output·bin

	mode := uint16(0x00)
	if bin == 2 {
		mode = 0x22
	}

	// SetStartPos: ROI offset (latched group).
	if err := rm.WriteReg(imx290RegLatch, 1); err != nil {
		return err
	}
	for _, rv := range []RegVal{
		{Reg: imx290RegStartXL, Val: ux & 0xff}, {Reg: imx290RegStartXH, Val: (ux >> 8) & 0xff},
		{Reg: imx290RegStartYL, Val: uy & 0xff}, {Reg: imx290RegStartYH, Val: (uy >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	if err := rm.WriteReg(imx290RegLatch, 0); err != nil {
		return err
	}

	// Cam_SetResolution: mode byte + sensor window (physical region = output·bin → 0x3042/3,
	// 0x303e/f).
	if err := rm.WriteReg(imx290RegMode, mode); err != nil {
		return err
	}
	for _, rv := range []RegVal{
		{Reg: imx290RegWidthL, Val: sw & 0xff}, {Reg: imx290RegWidthH, Val: (sw >> 8) & 0xff},
		{Reg: imx290RegHeightL, Val: sh & 0xff}, {Reg: imx290RegHeightH, Val: (sh >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}

	// FPGA frame geometry + SetFPGAHMAX on the output dims (SetFPGAWidth/Height take output
	// dims). ProgramHMAX recomputes HMAX from the output window + live ReadoutMode (USB speed /
	// depth / FPSPercent); a binned frame is fewer output pixels. HBLK/VBLK are sensor blanking.
	if err := ProgramFrameGeometry(rm, w, h, imx290HBLK, imx290VBLK); err != nil {
		return err
	}
	clk, floor := imx290ClockFloor(rm)
	return ProgramHMAX(rm, w, h, clk, floor, imx290VBlankAdd)
}
