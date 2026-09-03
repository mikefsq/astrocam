// Sony IMX290: 1/2.8" STARVIS rolling-shutter CMOS, 1936×1096 (ZWO ASI290 family).
// Wire-verified against the SDK on an ASI290MM Mini (USB2): RAW16 and RAW8 full frames, a
// 640×480 ROI and RAW16 bin 2 (host bin) match the SDK's plane means; the >= 1 s FPGA
// trigger-mode band (reg0 bit7) at 1/2/5/10 s; streaming 10.2/20.4 fps (RAW16/RAW8, FPS% 100)
// and 4.1 fps (FPS% 40) equal the SDK's; the 10-bit high-speed RAW8 frame matches the SDK's
// (single-shot and 1 s trigger band); bin 2 at both depths is host-binned like the SDK's
// (SoftBinRAW8) and matches its level and rate. The FX3 brackets each frame with the
// 0x5A7E/0x3CF0 marker words (FX3DMAMarkers); the SDK's frames carry real pixels there.
// Profile entry points:
//
//	imx290Init, imx290InitFPGA  sensor and FPGA bringup
//	imx290WriteClockSel         pixel-clock select
//	imx290SetROI                window start, mode 0x3006, window, output width, geometry, HMAX
//	imx290SetGain               analog gain
//	imx290SetExposure           frame length and shutter position
//	imx290SetOffset             black level
//	imx290Worker                the single-shot capture worker

package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
	"time"
)

const (
	imx290RegLatch    = 0x3001 // write 1 before / 0 after a coupled register group
	imx290RegStandby  = 0x3000 // 1 = sensor standby, 0 = streaming
	imx290RegGainMode = 0x3009 // bit 0x10 = high conversion gain; low bits = FRSEL clock select
	imx290RegGainCode = 0x3014 // analog gain code (single byte)

	// Window start: X aligned to 4, Y to 2 before the write (WINPH/WINPV).
	imx290RegStartXL = 0x3040
	imx290RegStartXH = 0x3041
	imx290RegStartYL = 0x303c
	imx290RegStartYH = 0x303d

	// Window setup: output window size × binning (WINWH/WINWV).
	imx290RegWidthL  = 0x3042
	imx290RegWidthH  = 0x3043
	imx290RegHeightL = 0x303e
	imx290RegHeightH = 0x303f
	imx290RegMode    = 0x3006 // WINMODE/VMODE byte; 0x22 for the 2× readout, else 0x00

	// SetExp: shutter (SHS), 24-bit little-endian.
	imx290RegSHS0 = 0x3020
	imx290RegSHS1 = 0x3021
	imx290RegSHS2 = 0x3022

	imx290GainMax   = 600           // 60.0 dB, ASI 0.1 dB units (SetGain clamp)
	imx290GainHCGAt = 60            // above this, high conversion gain (0x3009 bit 0x10)
	imx290ExpMinUs  = 32            // SetExp floor (exp <= 31 -> 32)
	imx290ExpMaxUs  = 2_000_000_000 // 2000 s ceiling (SetExp)
	imx290LongExpUs = 1_000_000     // >= 1 s enters FPGA trigger mode (reg0 bit7)
	// imx290ReadAttempts caps the worker's frame reads (zero and short alike) before the
	// capture fails: two ResetDevice cycles' worth of zero reads plus the short-frame retries.
	// imx290TrigReadTO bounds the frame read in the trigger band. The worker host-holds the whole
	// integration there, so the frame is exposed and buffered before the read starts and only the
	// wire transfer remains (a 1936×1096 RAW16 frame is 4 MB, a fraction of a second on either
	// link). Scaling this with the exposure instead would make a dead pipe cost
	// imx290ReadAttempts × the exposure before the capture fails.
	imx290TrigReadTO = 3 * time.Second

	imx290ReadAttempts = 12

	// Readout constants for the shared engine (fps.go / shutter.go); runtime state (USB speed,
	// FPS, output depth) lives in ReadoutMode. Clock + floor feed the HMAX formula, VBlankAdd +
	// SHSOffset the STARVIS VMAX/SHS math, HBLK/VBLK the FPGA frame geometry.
	imx290FullWidth  = 1936
	imx290FullHeight = 1096
	// Pixel clock by mode (the clock select switch): normal RAW16 18562, 10-bit high-speed 37124,
	// hardware bin 2 9281 (not modelled; RAW16 bin runs the normal clock). At 0.2 s full-frame
	// USB2 the normal clock gives HMAX 0x0662.
	imx290ClkKHz = 18562
	// HMAX floor = REG_FRAME_LENGTH_PKG_MIN, written per clock by the clock select
	// (measured): 18562→203, 37124→196, 9281→145. HMAX = max(bandwidth
	// candidate, floor)·100/FPSPercent: on USB2 the candidate dominates, on USB3 and small ROIs
	// the floor pins HMAX. Floor-dominated readout is not hardware-verified.
	imx290HMAXFloor   = 203
	imx290ClkKHzHS    = 37124 // 10-bit high-speed pixel clock
	imx290HMAXFloorHS = 196
	imx290VBlankAdd   = 18 // VMAX = height + 0x12 (SetExp)
	imx290SHSOffset   = 17 // SHS = (height + 0x11) - exposureLines (SetExp)
	imx290HBLK        = 0  // SetFPGAHBLK(0)
	imx290VBLK        = 9  // SetFPGAVBLK(9), sensor-specific blanking
)

// imx290Init is the sensor-write half of camera bringup: the 47-entry reglist (reg 0xffff =
// InitDelayReg, delay = val ms) then the explicit WriteSONYREG tail. FPGA bringup is in
// imx290InitFPGA; the SDK's SendCMD(0xAF/0xAE) are not replayed (Camera.Init sends 0xAF once).
var imx290Init = []RegVal{
	// --- reglist table (47 entries, file order) ---
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
	// --- explicit WriteSONYREG tail, in order ---
	{Reg: 0x305c, Val: 0x20}, // INCKSEL
	{Reg: 0x305d, Val: 0x00},
	{Reg: 0x305e, Val: 0x20},
	{Reg: 0x305f, Val: 0x00},
	{Reg: 0x3046, Val: 0xf1}, // output interface
	{Reg: 0x3005, Val: 0x01}, // ADBIT = 12-bit
	{Reg: 0x303a, Val: 0x08},
	{Reg: 0x3007, Val: 0x40}, // WINMODE
	// (FPGAReset + 20 ms here; see imx290InitFPGA)
	{Reg: 0x3002, Val: 0x01}, // XMSTA master stop during init
	{Reg: 0x304b, Val: 0x00},
}

// IMX290 is the Sony IMX290 STARVIS profile (ZWO ASI290 family).
var IMX290 = Sensor{
	Name:          "IMX290", // mono die; MC adds a CFA
	TriggerBandUs: imx290LongExpUs,
	GainMax:       imx290GainMax,
	ExpMinUs:      imx290ExpMinUs,
	ExpMaxUs:      imx290ExpMaxUs,
	// ASI Brightness / black level: range 0..240 from ASIGetControlCaps, and the default the
	// SDK's bringup programs rather than the 1 its caps declare — the same poison-and-read
	// check on regs 0x300a/0x300b gives 75. With it our default frames and the SDK's agree to
	// 0.02 % (mean 1321.9 against 1321.6 at 4 ms, gain 0).
	OffsetMax: 240, OffsetDef: 75,
	Info: CameraInfo{
		MaxWidth:  1936,        // SDK MaxWidth
		MaxHeight: 1096,        // SDK MaxHeight (= output rows)
		PixelUm:   2.9,         // µm pitch
		BitDepth:  12,          // 12-bit ADC (ADBIT=12; init tail 0x3005=0x01)
		Bayer:     "RGGB",      // CFA (color variant); surfaced when Model.Color
		Bins:      []int{1, 2}, // host-binned at every depth, as the SDK does (no HWBins)
	},
	Init:        imx290Init,
	InitFPGA:    imx290InitFPGA,
	SetGain:     imx290SetGain,
	SetExposure: imx290SetExposure,
	SetOffset:   imx290SetOffset,
	GetOffset:   imx290GetOffset,
	SetROI:      imx290SetROI,
	StreamStop:  func(rm Regmap) error { return rm.WriteReg(imx290RegStandby, 1) }, // standby on
	StreamStart: func(rm Regmap) error { return rm.WriteReg(imx290RegStandby, 0) }, // standby off
	Worker:      imx290Worker,
	// FX3 marker words 0x5A7E/0x3CF0 in the first/last DMA word (wire-confirmed against the SDK
	// on an ASI290MM Mini).
	FX3DMAMarkers: true,
	// No HWBins: the die's 2× readout (mode 0x3006=0x22, which SetROI still programs for a
	// sensor bin 2) is one the SDK never selects (no ASI_HARDWARE_BIN control, and the window write
	// programs it only under the hard-bin flag), and its frames differ from the SDK's host bin,
	// so SetHardwareBin(true) host-bins here too, as the SDK does.
	ROIStartAlign: func(int) (int, int) { return 4, 2 }, // window-start masks
}

// imx290Worker is the host-timed single-shot capture worker. The sensor gate is the 0x3000
// standby register. At >= 1 s SetExp arms trigger mode (reg0 bit7); the worker drives the
// trigger signal (EnableFPGATriggerSignal, FPGA reg 0x0b bit0), whose 1->0 edge ends the
// integration and releases the frame.
//
//	arm:    SendCMD(0xAA)·FPGAStop·0x3000=1·SendCMD(0xA9)·0x3000=0·50ms·FPGAStart·ResetEndpoint
//	expose: EnableFPGATriggerSignal(1)·host-time (< 1 s: exposure-200 ms, then re-arm;
//	        >= 1 s: 100 ms poll for the full exposure)
//	fire:   EnableFPGATriggerSignal(0)·BulkRead with the retry ladder
//	stop:   FPGAStop·0x3000=1·SendCMD(0xAA)·ResetEndPoint (the SDK's exit)
func imx290Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()
	arm := func(full bool) error {
		if full {
			if err := ctl.VendorCmd(FX3StreamStop); err != nil {
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
			if err := ctl.VendorCmd(FX3StreamStart); err != nil {
				return err
			}
		}
		if err := rm.WriteReg(imx290RegStandby, 0); err != nil { // stream (start)
			return err
		}
		time.Sleep(50 * time.Millisecond)        // usleep(0xc350)
		return SetFPGABit(rm, 0x00, 0x10, false) // FPGAStart: reg0 bit4 clear
	}
	trigger := func(on bool) error { return SetFPGABit(rm, 0x0b, 0x01, on) } // trigger signal

	if err := arm(true); err != nil {
		return 0, err
	}
	// Halt the readout on every return, as the SDK does on its way out:
	// StopSensorStreaming (FPGAStop + standby 0x3000=1), SendCMD(0xAA), ResetEndPoint. A sensor
	// left free-running with no reader backs up the FX3 GPIF until the firmware crashes.
	// Best-effort: a failed stop must not fail a good frame. This exit sequence is not
	// hardware-verified.
	defer func() {
		_ = SetFPGABit(rm, 0x00, 0x10, true) // FPGAStop: reg0 bit4
		_ = rm.WriteReg(imx290RegStandby, 1) // standby (sensor stop)
		_ = ctl.VendorCmd(FX3StreamStop)
		_ = ctl.ResetEndpoint()
	}()
	_ = ctl.ResetEndpoint()
	trigMode := exposure >= imx290LongExpUs*time.Microsecond // reg0 bit7 armed by SetExp
	// integrate runs one trigger cycle: assert the trigger signal, host-time the exposure, drop
	// it. Free-run (< 1 s, reg0 bit7 clear): hold briefly, then re-arm and read the next free-run
	// frame; the standby toggle is safe there (no triggered charge to lose). Trigger mode
	// (>= 1 s, bit7 set): hold the signal high for the full exposure and read the triggered frame
	// directly; a standby toggle here clears the integrated charge and yields a dark frame.
	integrate := func() error {
		if err := trigger(true); err != nil {
			return err
		}
		if !trigMode {
			if w := exposure - 200*time.Millisecond; w > 0 {
				time.Sleep(w)
			}
			_ = ctl.ResetEndpoint()
			if err := arm(false); err != nil {
				return err
			}
		} else {
			for start := time.Now(); time.Since(start) < exposure; {
				if ctl.Aborted() {
					// StopExposure ran: drop the trigger signal on the way out. Left asserted,
					// the next trigger(true) is a no-edge write and the FPGA never gates the
					// integration. (The SDK never host-aborts mid-integration.)
					_ = trigger(false)
					return errExposureAborted
				}
				time.Sleep(100 * time.Millisecond)
			}
		}
		return trigger(false)
	}
	if err := integrate(); err != nil {
		return 0, err
	}
	// Read with the SDK's own retry ladder, which
	// covers the tiny-ROI intermittent short read:
	//
	//   trigger-mode short   -> checked first: if FPGA reg 0x23 bit2 says the triggered frame is
	//                           still buffered, pulse FPGABufReload (reg 0x18 bit0) and re-read
	//                           without re-integrating; max 3 reloads
	//   zero read            -> counted separately, no pipe reset between attempts; the 4th
	//                           escalates to ResetDevice + the full re-arm (ResetDevice, 50 ms,
	//                           StopSensorStreaming = FPGAStop+standby 1, 0xAA, 10 ms, 0xA9,
	//                           StartSensorStreaming = standby 0 + 50 ms + FPGAStart) and, in
	//                           trigger mode, a fresh trigger cycle (the re-arm leaves the FPGA
	//                           in wait mode, so without it the reads wait for a frame that
	//                           never arrives)
	//   short frame          -> retry counter; snap mode gives up past 2 (EXP_FAILED), else
	//                           ResetEndPoint(0x81) and a fresh read
	//
	// Zero reads also count toward imx290ReadAttempts, so a dead pipe fails the capture instead
	// of cycling ResetDevice forever (Linux returns (0, nil) on a read deadline). The SDK's 20 s
	// wall-clock window and ~1 s per-band read timeouts are covered by the bounded exposure+3 s
	// per read × the caps.
	want := ctl.FrameBytes()
	if want > len(buf) {
		want = len(buf)
	}
	readTO := exposure + 3*time.Second // free-run: the frame is still being integrated
	if trigMode {
		readTO = imx290TrigReadTO
	}
	zeros, reloads, retries := 0, 0, 0
	for attempt := 1; ; attempt++ {
		n, err := ctl.BulkRead(buf[:want], readTO)
		if err != nil {
			return n, err
		}
		if n >= want {
			return n, nil
		}
		if ctl.Aborted() {
			return n, errExposureAborted // AbortRead: clean abort, not a stall
		}
		if attempt >= imx290ReadAttempts {
			return n, fmt.Errorf("imx290: short frame %d/%d after %d reads", n, want, attempt)
		}
		ctl.NoteStall()
		if n == 0 {
			zeros++
			if zeros < 4 {
				continue // zero returns are re-read without touching the pipe
			}
			_ = ctl.ResetDevice()
			time.Sleep(50 * time.Millisecond)
			if err := arm(true); err != nil {
				return 0, err
			}
			_ = ctl.ResetEndpoint()
			if trigMode {
				if err := integrate(); err != nil {
					return 0, err
				}
				reloads = 0 // a fresh integration is a fresh buffer
			}
			zeros = 0
			continue
		}
		zeros = 0
		if trigMode && reloads < 3 {
			if v, rerr := rm.ReadFPGAReg(0x23); rerr == nil && v&0x04 != 0 {
				// The triggered frame is still in the FPGA buffer: reload and re-read it
				// instead of re-integrating (reg 0x23 bit2 path; does not consume a retry).
				reloads++
				if err := SetFPGABit(rm, 0x18, 0x01, true); err != nil {
					return 0, err
				}
				continue
			}
		}
		retries++
		if retries > 2 { // snap-mode give-up (SDK r14 > 2)
			return n, fmt.Errorf("imx290: short frame %d/%d after %d attempts", n, want, retries)
		}
		_ = ctl.ResetEndpoint()
	}
}

// imx290InitFPGA is the FPGA bringup that follows after the Sony init writes, using the
// FX3 register numbers:
//
//	FPGAReset                          reg0 bit0 -> 0
//	(20 ms delay; the SDK's SendCMD(0xAF) here is not replayed)
//	SetFPGAAsMaster(1)                 reg0 bit5 = 1
//	FPGAStop                           reg0 bit4 = 1
//	EnableFPGADDR(0)                   reg0xa bit6 = 1 (DDR disabled)
//	SetFPGAADCWidthOutputWidth(1, 0)   reg0xa bit0 = 1 (adc), bit4 = 0 (output), x2
//	SetFPGAGain(0x80,0x80,0x80,0x80)   FPGA 0x0c-0x0f, strobed by reg 1
//	WriteFPGAREG(0x1a, 0x04)           direct FPGA register write
//
// Camera bringup passes outputWidth = 0 (8-bit) and the SDK raises reg0xa bit4 later for RAW16;
// here bit4 comes from the live ReadoutMode. With bit4 = 0 the ASI290 streams a half-size RAW8
// frame (1936×1096×1 = 2121856 B) and the RAW16 read reports a short frame.
func imx290InitFPGA(rm Regmap, subtype int) error {
	_ = subtype                                           // no firmware-subtype split on the 290
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
	// SetFPGAADCWidthOutputWidth(adc=1, outputWidth): reg0xa bit0 = adc, bit4 = output width
	// (1 = RAW16, from the live ReadoutMode).
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
	return rm.WriteFPGAReg(0x1a, 0x04) // as camera bringup does
}

// imx290SetGain (SetGain) clamps to [0, 600], converts to the Sony analog-gain code (gain/3
// with the HCG rebase above 60) via SonyAnalogGain, sets/clears the 0x3009 HCG bit and writes
// the code byte to 0x3014, under the 0x3001 latch.
func imx290SetGain(rm Regmap, gain int) error {
	if gain > imx290GainMax {
		gain = imx290GainMax
	}
	if gain < 0 {
		gain = 0
	}
	code, hcg := SonyAnalogGain(gain, imx290GainHCGAt)
	return WithLatch(rm, imx290RegLatch, func() error {
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
		return rm.WriteReg(imx290RegGainCode, code&0xff)
	})
}

// imx290Shutter is the STARVIS ShutterModel (SetExp). ApplyExposure (shutter.go) derives the
// line time from the HMAX formula + the live ReadoutMode and applies VMAX = height + VBlankAdd,
// SHS = clamp(height + SHSOffset - lines, 1, height + SHSOffset - 1), stretching VMAX (SHS = 1)
// when the exposure exceeds one frame.
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

// imx290HighSpeed reports whether the 10-bit high-speed readout is in effect: the mode flag with
// RAW8 output (the SDK's ASI_HIGH_SPEED_MODE applies to 8-bit output only; RAW16 keeps the 12-bit
// format and clock). The format block, the clock select and the HMAX/line-time pair all key off
// this one predicate.
func imx290HighSpeed(rm Regmap) bool {
	m := ModeOf(rm)
	return m.HighSpeed && m.BytesPerPx < 2
}

// imx290ClockFloor returns the pixel clock + HMAX floor for the live readout mode: the 10-bit
// high-speed pair, else the normal 12-bit pair.
func imx290ClockFloor(rm Regmap) (clock, floor int) {
	if imx290HighSpeed(rm) {
		return imx290ClkKHzHS, imx290HMAXFloorHS
	}
	return imx290ClkKHz, imx290HMAXFloor
}

// imx290SetOutputFormat is the 16-bit output select: the sensor output bit format for the live mode.
// Normal or RAW16 uses the 12-bit ADBIT block (0x3005=1, OUTFMT 0x3046=0xf1, 0x3129=0,
// 0x317c=0, 0x31ec=0x0e) with FPGA ADC_BIT (reg 0x0a bit0) = 1. High-speed RAW8 uses the 10-bit
// reformat (0x3046=0xf0, 0x3005=0, 0x3129=0x1d, 0x317c=0x12, ADC_BIT=0): a shorter ADC ramp
// clocked 2× faster (imx290ClockFloor switches clock 18562→37124, floor 203→196).
func imx290SetOutputFormat(rm Regmap) error {
	out := []RegVal{{Reg: 0x3046, Val: 0xf1}, {Reg: 0x3005, Val: 0x01}, {Reg: 0x3129, Val: 0x00}, {Reg: 0x317c, Val: 0x00}, {Reg: 0x31ec, Val: 0x0e}}
	adcBit := uint16(0x01)
	if imx290HighSpeed(rm) {
		out = []RegVal{{Reg: 0x3046, Val: 0xf0}, {Reg: 0x3005, Val: 0x00}, {Reg: 0x3129, Val: 0x1d}, {Reg: 0x317c, Val: 0x12}}
		adcBit = 0x00
	}
	for _, rv := range out {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return FPGAWriteBits(rm, 0x0a, 0x01, adcBit)
}

// imx290SetExposure programs the STARVIS VMAX/SHS (ApplyExposure) and, at >= 1 s, engages FPGA
// trigger mode (reg0 bit7, EnableFPGATriggerMode) so the FPGA holds the frame until the worker's
// trigger-signal 1→0 edge and a long exposure reads out as one complete frame from row 0;
// without it the VMAX-extended sensor free-runs and the worker fires mid-frame (e.g. 1027 of
// 1096 rows at 2 s). Below 1 s the bit is cleared. The SDK's SelectExtTrigExp call is not
// reproduced.
func imx290SetExposure(rm Regmap, d time.Duration) error {
	trigger := d >= imx290LongExpUs*time.Microsecond            // >= 1 s
	if err := SetFPGABit(rm, 0x00, 0x80, trigger); err != nil { // bit7; ApplyExposure sets bit6
		return err
	}
	// Program the computed HMAX so the FPGA line rate agrees with the SHS math; a stale fast HMAX
	// would overflow one frame and collapse to the VMAX-stretch / SHS=1 path. Use the live ROI
	// dims when set, else full-frame.
	hw, hh := imx290FullWidth, imx290FullHeight
	if rd := ModeOf(rm); rd.Width > 0 && rd.Height > 0 { // zero Height collapses the candidate
		hw, hh = rd.Width, rd.Height
	}
	clk, floor := imx290ClockFloor(rm)
	if err := WriteFPGAHMAX(rm, HMAX(hw, hh, clk, floor, imx290VBlankAdd, ModeOf(rm))); err != nil {
		return err
	}
	if err := ApplyExposure(rm, imx290Shutter, imx290RegLatch, d); err != nil {
		return err
	}
	// the clock select at the tail, as SetExp does: the sensor clock select must track the live mode
	// or the HMAX/SHS math above runs at the wrong physical rate.
	return imx290WriteClockSel(rm)
}

// imx290WriteClockSel programs the sensor clock: program the
// 0x3009 clock/FRSEL select for the live readout mode, preserving the conversion-gain bit that
// SetGain maintains:
//
//	18562 kHz (12-bit normal)     -> FRSEL 0x01
//	37124 kHz (10-bit high-speed) -> FRSEL 0x00
//	 9281 kHz (hardware bin 2)    -> FRSEL 0x00   (gated on the SDK's hard-bin flag, which no
//	                                 control sets on this die; bin 2 is host-binned)
//	other                         -> FRSEL 0x02   (unused)
//
// The SDK calls it whenever the exposure, the window or the speed mode changes. The driver
// mirrors the first two (the init reglist leaves 0x3009 alone; a HighSpeed toggle takes effect
// at the next SetExposure/SetROI). Without this write the host math would switch clocks
// (imx290ClockFloor) while the sensor stayed on its init clock.
func imx290WriteClockSel(rm Regmap) error {
	cur, err := rm.ReadReg(imx290RegGainMode)
	if err != nil {
		return err
	}
	v := cur & 0x10 // preserve HCG; rewrite the FRSEL field below it
	if !imx290HighSpeed(rm) {
		v |= 0x01 // 12-bit normal clock (18562); high-speed (37124) runs FRSEL 0
	}
	return rm.WriteReg(imx290RegGainMode, v)
}

// imx290GetOffset reads the offset back from 0x300a/0x300b, the pair imx290SetOffset writes
// 16-bit little-endian with no scaling.
func imx290GetOffset(rm Regmap) (int, error) {
	v, err := ReadRegLE(rm, []uint16{0x300a, 0x300b})
	return int(v), err
}

func imx290SetOffset(rm Regmap, offset int) error {
	v := uint16(offset)
	if err := rm.WriteReg(0x300b, (v>>8)&0xff); err != nil {
		return err
	}
	return rm.WriteReg(0x300a, v&0xff)
}

// imx290SetROI programs the readout window: the window start (X aligned to 4, Y to 2) plus
// the window write (mode byte 0x3006, window, FPGA frame geometry). Both sensor groups are
// under the 0x3001 latch. FPGA geometry via the FX3 setters:
//
//	SetFPGAHBLK(0)   -> 0x02/0x03 (strobed)   horizontal blanking = 0
//	SetFPGAVBLK(9)   -> 0x06/0x07 (strobed)   vertical blanking = 9 (sensor-specific)
//	SetFPGAWidth(w)  -> 0x04/0x05 (strobed)
//	SetFPGAHeight(h) -> 0x08/0x09 (strobed)
//
// SetFPGAHMAX (FPGA 0x13/0x14, strobed) then throttles the readout to USB bandwidth; ProgramHMAX
// computes it from the window geometry + the live ReadoutMode.
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
	// (x,y,w,h) are binned output pixels. STARVIS binning: the sensor window
	// regs take the physical region (output·bin), the mode byte 0x3006 selects the 2× readout
	// (0x22, else 0x00); the FPGA frame geometry + HMAX take the output dims. Start is sensor
	// pixels = output·bin. SetExp is unchanged: VMAX uses the full sensor height (invariant under
	// binning) and the line time follows the throttle on the output frame.
	sx, sy := x*bin, y*bin
	sx &^= 3 // align X to 4 (#0x7ffffffc)
	sy &^= 1 // align Y to 2 (#0x7ffffffe)
	ux, uy := uint16(sx), uint16(sy)
	sw, sh := uint16(w*bin), uint16(h*bin) // sensor physical window = output·bin

	mode := uint16(0x00)
	if bin == 2 {
		mode = 0x22
	}

	// Window start: ROI offset (latched group).
	if err := WithLatch(rm, imx290RegLatch, func() error {
		for _, rv := range []RegVal{
			{Reg: imx290RegStartXL, Val: ux & 0xff}, {Reg: imx290RegStartXH, Val: (ux >> 8) & 0xff},
			{Reg: imx290RegStartYL, Val: uy & 0xff}, {Reg: imx290RegStartYH, Val: (uy >> 8) & 0xff},
		} {
			if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
				return err
			}
		}
		return nil
	}); err != nil {
		return err
	}

	// Window setup: mode byte + sensor window (physical region = output·bin → 0x3042/3,
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

	// the 16-bit output select: the output bit format for the live mode (12-bit, or 10-bit high-speed).
	if err := imx290SetOutputFormat(rm); err != nil {
		return err
	}

	// FPGA frame geometry + SetFPGAHMAX on the output dims; a binned frame is fewer output
	// pixels. HBLK/VBLK are sensor blanking.
	if err := ProgramFrameGeometry(rm, w, h, imx290HBLK, imx290VBLK); err != nil {
		return err
	}
	clk, floor := imx290ClockFloor(rm)
	if err := ProgramHMAX(rm, w, h, clk, floor, imx290VBlankAdd); err != nil {
		return err
	}
	// the clock select at the tail, as SetResolution does.
	return imx290WriteClockSel(rm)
}
