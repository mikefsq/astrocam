// Sony IMX462: STARVIS Type 1/2.8, 1936×1096, sibling of the IMX290 (ZWO ASI462, PlayerOne
// Mars/Ceres 462). The register map matches the IMX290MM Mini except the HCG threshold (gain 80
// vs 60), the init reglist values, init tail 0x305f (0x01 vs 0x00) and the HMAX floors. FX3 DDR
// frame markers bracket each frame (FX3DMAMarkers). Wire-verified against the SDK on an
// ASI462MC over USB3 and a real USB2 link (RAW16, RAW8, high-speed; single-shot, trigger band,
// host bin 2 at both depths per pixel; free-run streaming at the sensor rate on USB3 and at the
// link rate on USB2, 10.2/20.4 fps at FPS% 100). Profile entry points:
//
//	imx462Init, imx462InitFPGA  sensor and FPGA bringup
//	imx462WriteClockSel         pixel-clock select
//	imx462SetROI                window start, mode 0x3006, window, output width, geometry, HMAX
//	imx462SetGain               analog gain
//	imx462SetExposure           frame length and shutter position
//	imx462SetOffset             black level
//	imx462Worker                the single-shot capture worker

package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
	"time"
)

const (
	imx462RegLatch    = 0x3001 // write 1 before / 0 after a coupled register group
	imx462RegStandby  = 0x3000 // 1 = sensor standby, 0 = streaming
	imx462RegGainMode = 0x3009 // bit 0x10 = high conversion gain; low bits = FRSEL clock select
	imx462RegGainCode = 0x3014 // analog gain code (single byte)

	// Window start: X aligned to 4, Y to 2 before the write.
	imx462RegStartXL = 0x3040
	imx462RegStartXH = 0x3041
	imx462RegStartYL = 0x303c
	imx462RegStartYH = 0x303d

	// Window setup: output window size × binning.
	imx462RegWidthL  = 0x3042
	imx462RegWidthH  = 0x3043
	imx462RegHeightL = 0x303e
	imx462RegHeightH = 0x303f
	imx462RegMode    = 0x3006 // WINMODE/VMODE byte; 0x22 for the 2× readout, else 0x00

	// SetExp: shutter (SHS), 24-bit little-endian.
	imx462RegSHS0 = 0x3020
	imx462RegSHS1 = 0x3021
	imx462RegSHS2 = 0x3022

	imx462GainMax   = 600           // 60.0 dB, ASI 0.1 dB units (SetGain clamp)
	imx462GainHCGAt = 80            // above this, high conversion gain (0x3009 bit 0x10)
	imx462ExpMinUs  = 32            // µs floor (STARVIS SetExp clamp)
	imx462ExpMaxUs  = 2_000_000_000 // 2000 s ceiling
	imx462LongExpUs = 1_000_000     // >= 1 s enters FPGA trigger mode (reg0 bit7)

	// Readout constants for the shared engine (fps.go / shutter.go).
	imx462FullWidth  = 1936 // ASI462MC reports 1936×1096, the same array as the 290
	imx462FullHeight = 1096
	imx462ClkKHz     = 18562 // 12-bit normal pixel clock (INCK/2)
	// HMAX floor = REG_FRAME_LENGTH_PKG_MIN, written per clock by the clock select
	// (measured): 18562→261, 37124→245, 9281→145. HMAX = max(bandwidth candidate,
	// floor)·100/FPSPercent (the SDK's bandwidth formula; HMAX/HMAXBW in fps.go, bwUSB2/bwUSB3 = MAX_DATASIZE·
	// 10·100 with MAX_DATASIZE from the 16-bit output select: USB3 360715, USB2 43272). On USB2 the
	// candidate dominates (1634 full-frame; HMAX 4085 at pct 40, wire-confirmed); on USB3 the
	// floor pins HMAX (261 ≈ 64 fps full-frame RAW16, measured 63.4 fps on USB3).
	imx462HMAXFloor   = 261
	imx462ClkKHzHS    = 37124 // 10-bit high-speed pixel clock, 2× the 12-bit clock
	imx462HMAXFloorHS = 245
	imx462VBlankAdd   = 18 // VMAX = height + 0x12
	imx462SHSOffset   = 17 // SHS = (height + 0x11) - exposureLines
	imx462HBLK        = 0  // SetFPGAHBLK(0)
	imx462VBLK        = 9  // SetFPGAVBLK(9)
)

// imx462Init is the sensor-write half of camera bringup: the 73-entry reglist (reg 0xffff = delay
// ms) then the explicit sensor-reg tail. FPGA bringup is in imx462InitFPGA; the SDK's
// SendCMD(0xAF/0xAE) are not replayed.
var imx462Init = []RegVal{
	// --- reglist table, in programming order ---
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
	// --- explicit sensor-reg bringup tail, overrides the reglist ---
	{Reg: 0x305c, Val: 0x20}, // INCKSEL
	{Reg: 0x305d, Val: 0x00},
	{Reg: 0x305e, Val: 0x20},
	{Reg: 0x305f, Val: 0x01}, // (462: 0x01, vs 290's 0x00)
	{Reg: 0x3046, Val: 0xf1}, // output interface
	{Reg: 0x3005, Val: 0x01}, // ADBIT = 12-bit
	{Reg: 0x303a, Val: 0x08},
	{Reg: 0x3007, Val: 0x40}, // WINMODE
	// (FPGAReset here; see imx462InitFPGA)
	{Reg: 0x3002, Val: 0x01}, // XMSTA master stop during init
	{Reg: 0x304b, Val: 0x00},
}

// IMX462 is the Sony IMX462 STARVIS profile (ZWO ASI462, PlayerOne Mars/Ceres 462).
var IMX462 = Sensor{
	Name:          "IMX462", // mono die; MC adds a CFA
	TriggerBandUs: imx462LongExpUs,
	GainMax:       imx462GainMax,
	ExpMinUs:      imx462ExpMinUs,
	ExpMaxUs:      imx462ExpMaxUs,
	// ASI Brightness / black level: range from ASIGetControlCaps (SDK 1.41: min 0, max 500), and
	// the default the SDK's bringup programs rather than the 1 its caps declare — poisoning
	// regs 0x300a/0x300b with 200, running the SDK at its own default and reading them back gives
	// 30 (its log: "ASI462 SetBrightness 30-->30"). At 1 our frames sat 528 ADU below the SDK's.
	// Offset→DN is floor 41/71/141 in 12-bit at offsets 0/30/100.
	OffsetMax: 500, OffsetDef: 30,
	Info: CameraInfo{
		MaxWidth:  imx462FullWidth,
		MaxHeight: imx462FullHeight,
		PixelUm:   2.9,         // µm pitch
		BitDepth:  12,          // 12-bit ADC (ADBIT=12; tail 0x3005=0x01)
		Bayer:     "RGGB",      // CFA (color variant); surfaced when Model.Color
		Bins:      []int{1, 2}, // host-binned at every depth, as the SDK does (no HWBins)
	},
	Init:        imx462Init,
	InitFPGA:    imx462InitFPGA,
	SetGain:     imx462SetGain,
	SetExposure: imx462SetExposure,
	SetOffset:   imx462SetOffset,
	GetOffset:   imx462GetOffset,
	SetROI:      imx462SetROI,
	StreamStop:  func(rm Regmap) error { return rm.WriteReg(imx462RegStandby, 1) }, // standby on
	StreamStart: func(rm Regmap) error { return rm.WriteReg(imx462RegStandby, 0) }, // standby off
	Worker:      imx462Worker,

	FX3DMAMarkers: true, // FX3 brackets each frame with 0x5A7E/0x3CF0 marker words (HW-confirmed)
	// No HWBins: the die's 2× readout (mode 0x3006=0x22, which SetROI still programs for a
	// sensor bin 2) is one the SDK never selects (no ASI_HARDWARE_BIN control, and the window write
	// programs it only under the hard-bin flag), and its frames differ from the SDK's host bin,
	// so SetHardwareBin(true) host-bins here too, as the SDK does.
	ROIStartAlign: func(int) (int, int) { return 4, 2 }, // window-start masks
}

// imx462Worker is the host-timed single-shot capture worker (STARVIS standby-0x3000 gate; the
// >= 1 s band uses FPGA trigger mode and the trigger signal).
//
//	arm:    SendCMD(0xAA)·FPGAStop·0x3000=1·SendCMD(0xA9)·0x3000=0·50ms·FPGAStart
//	expose: >= 1 s only: EnableFPGATriggerSignal(1)·hold the full exposure·(0)
//	read:   StreamFramePrequeued of FrameBytes with the retry ladder below; no pre-read sleep
//	stop:   FPGAStop·0x3000=1·SendCMD(0xAA)·ResetEndPoint (the SDK's exit)
func imx462Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()
	longExp := exposure >= imx462LongExpUs*time.Microsecond // >= 1 s, SetExposure's trigger band
	arm := func(full bool) error {
		if full {
			if err := ctl.VendorCmd(FX3StreamStop); err != nil {
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
			if err := ctl.VendorCmd(FX3StreamStart); err != nil {
				return err
			}
		}
		if err := rm.WriteReg(imx462RegStandby, 0); err != nil { // stream (start)
			return err
		}
		time.Sleep(50 * time.Millisecond)
		// FPGAStart: clear bit4 (readout-stop). In free-run (< 1 s) also clear bit6 (WaitMode)
		// so the sensor streams frames (the SDK writes reg0 = 0x21). In trigger mode (>= 1 s)
		// keep bit6 set: the FPGA must stay in wait mode for the trigger signal (reg 0x0b bit0)
		// to gate the integration, otherwise the readout is un-gated and no frame arrives (the
		// SDK's trigger-mode FPGAStart is reg0 0xe1, wire-confirmed).
		mask := uint16(0x50) // bit4 + bit6 (free-run)
		if longExp {
			mask = 0x10 // bit4 only: keep WaitMode for the trigger
		}
		return SetFPGABit(rm, 0x00, mask, false)
	}
	// EnableFPGATriggerSignal (reg 0x0b bit0): only the >= 1 s trigger band uses it. In normal
	// mode the sensor free-runs (SHS-timed) and the FPGA starts and reads one frame.
	trigger := func(on bool) error { return SetFPGABit(rm, 0x0b, 0x01, on) }
	// integrate runs one full trigger cycle: the signal hold is the integration, so hold for
	// the full exposure (an exposure-200 ms hold under-integrates the 1 s boundary), polling
	// the abort flag. Used on the first arm and after a ResetDevice re-arm (the re-armed FPGA
	// is back in wait mode; without a fresh trigger edge no frame comes).
	integrate := func() error {
		if err := trigger(true); err != nil {
			return err
		}
		for start := time.Now(); time.Since(start) < exposure; {
			if ctl.Aborted() {
				// StopExposure ran: drop the trigger signal on the way out. Left asserted,
				// the next long exposure's trigger(true) is a no-edge write and the FPGA
				// never gates it.
				_ = trigger(false)
				return errExposureAborted
			}
			time.Sleep(100 * time.Millisecond)
		}
		return trigger(false)
	}

	if err := arm(true); err != nil {
		return 0, err
	}
	// Halt the readout on every return, as the SDK does on its way out: StopSensorStreaming
	// (FPGAStop + standby=1), SendCMD(0xAA), ResetEndPoint. A sensor left free-running with no
	// reader backs up the FX3 GPIF until the firmware crashes. Best-effort: a failed stop must
	// not fail a good frame.
	defer func() {
		_ = SetFPGABit(rm, 0x00, 0x10, true) // FPGAStop: reg0 bit4
		_ = rm.WriteReg(imx462RegStandby, 1) // standby (sensor stop)
		_ = ctl.VendorCmd(FX3StreamStop)
		_ = ctl.ResetEndpoint()
	}()
	_ = ctl.ResetEndpoint()
	if longExp {
		if err := integrate(); err != nil {
			return 0, err
		}
	}
	// Free-run (< 1 s): no pre-read sleep and no FPGABufReload. the SDK calls its windowed read
	// immediately after FPGAStart with timeout = exposure+1000 ms, so the URB batch waits on the
	// pipe through the integration and the GPIF never streams without a reader (a sleep before
	// the read backs up the FIFO and tears or stalls on USB2). The reg 0x18 control write is not
	// in the SDK's normal path; it appears only in the trigger retry below, gated on FPGA reg
	// 0x23 bit2.
	target := ctl.FrameBytes()
	if target > len(buf) {
		target = len(buf)
	}
	readTotal := exposure + time.Second // SDK free-run: read timeout = exp + 1000 ms
	if longExp {
		readTotal = time.Second // SDK trigger mode: frame already integrated; 1000 ms
	}
	// Retry ladder, mirroring the SDK's failure handling:
	//   short frame          -> ResetEndpoint, fresh full-frame read
	//   trigger-mode short   -> if FPGA reg 0x23 bit2 says the frame is still buffered, pulse
	//                           FPGABufReload (reg 0x18 bit0) and re-read without re-integrating
	//   4 consecutive zeros  -> ResetDevice + full re-arm
	// A frame short by 512 bytes counts as complete (the SDK treats completed+0x200 ==
	// frameBytes as success); the missing tail is zeroed.
	zeros, reloads := 0, 0
	for attempt := 0; ; attempt++ {
		n, err := ctl.StreamFramePrequeued(buf[:target], 500*time.Millisecond, readTotal)
		if err != nil {
			return n, err
		}
		// Complete = the exact byte count, or (the SDK's tolerance) exactly 512 short.
		// `n >= target-512` would let a small-ROI frame (target <= 512) accept n=0 and
		// fabricate an all-zero frame.
		if n >= target || (target > 512 && n == target-512) {
			for i := n; i < target; i++ {
				buf[i] = 0
			}
			return target, nil
		}
		if ctl.Aborted() {
			return n, errExposureAborted
		}
		if attempt >= 6 { // SDK snap gives up (ASI_EXP_FAILED) rather than retry forever
			return n, fmt.Errorf("imx462: short frame %d/%d after %d attempts", n, target, attempt+1)
		}
		ctl.NoteStall()
		if n < 4096 {
			// Empty or near-empty read. A parked GPIF shows up as a few header bytes then a
			// stalled pipe (64 bytes with a valid 0x5A7E marker, every attempt), so tiny reads
			// count as empty too; a plain ResetEndpoint does not unpark it.
			zeros++
			if zeros >= 4 { // SDK: 4 empty reads in a row -> ResetDevice + full re-arm
				_ = ctl.ResetDevice()
				time.Sleep(50 * time.Millisecond)
				if err := arm(true); err != nil {
					return 0, err
				}
				_ = ctl.ResetEndpoint()
				if longExp {
					// The re-arm put the FPGA back in wait mode (bit6 kept), so the recovery
					// re-runs the trigger cycle; without it the reads below wait ~10 s for a
					// frame that never arrives. Costs a full re-integration, as the SDK's
					// re-entered, as the SDK does. A fresh integration is a fresh buffer, so
					// the reg 0x23 reload budget resets too.
					if err := integrate(); err != nil {
						return 0, err
					}
					reloads = 0
				}
				zeros = 0
				continue
			}
			_ = ctl.ResetEndpoint()
			continue
		}
		zeros = 0
		if longExp && reloads < 3 {
			if v, rerr := rm.ReadFPGAReg(0x23); rerr == nil && v&0x04 != 0 {
				// The triggered frame is still in the FPGA buffer: reload and re-read it
				// instead of re-integrating (SDK reg 0x23 bit2 path, max 3 attempts).
				reloads++
				if err := SetFPGABit(rm, 0x18, 0x01, true); err != nil {
					return 0, err
				}
				continue
			}
		}
		_ = ctl.ResetEndpoint()
	}
}

// imx462InitFPGA is the FPGA bringup after the Sony init writes: FPGAReset, 20 ms,
// SetFPGAAsMaster(1), FPGAStop, EnableFPGADDR(0), SetFPGAADCWidthOutputWidth (bit4 = RAW16 from
// the live ReadoutMode), SetFPGAGain(0x80×4), WriteFPGAREG(0x1a, 0x04).
func imx462InitFPGA(rm Regmap, subtype int) error {
	if err := poaUnsupported(rm, "imx462", "FPGA bringup"); err != nil {
		return err
	}
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

// imx462SetGain (SetGain) clamps to [0, 600], converts to the Sony analog-gain code (gain/3
// with the HCG rebase above 80) via SonyAnalogGain, sets/clears the 0x3009 HCG bit and writes
// the code byte to 0x3014, under the 0x3001 latch.
func imx462SetGain(rm Regmap, gain int) error {
	if err := poaUnsupported(rm, "imx462", "SetGain"); err != nil {
		return err
	}
	if gain > imx462GainMax {
		gain = imx462GainMax
	}
	if gain < 0 {
		gain = 0
	}
	code, hcg := SonyAnalogGain(gain, imx462GainHCGAt)
	return WithLatch(rm, imx462RegLatch, func() error {
		mode, err := rm.ReadReg(imx462RegGainMode)
		if err != nil {
			return err
		}
		if hcg {
			mode |= 0x10
		} else {
			mode &= 0x0f
		}
		// The FRSEL low bits are left as-is: the clock select is owned by imx462WriteClockSel
		// (the clock select), rewritten at the end of every SetExp/SetResolution.
		if err := rm.WriteReg(imx462RegGainMode, mode); err != nil {
			return err
		}
		return rm.WriteReg(imx462RegGainCode, code&0xff)
	})
}

// imx462WriteClockSel programs the sensor clock: program the 0x3009
// clock/FRSEL select for the live readout mode, preserving the conversion-gain bit that SetGain
// maintains:
//
//	18562 kHz (12-bit normal)     -> FRSEL 0x01
//	37124 kHz (10-bit high-speed) -> FRSEL 0x00
//	 9281 kHz (hardware bin 2)    -> FRSEL 0x00   (gated on the SDK's hard-bin flag, which no
//	                                 control sets on this die; bin 2 is host-binned)
//	other                         -> FRSEL 0x02   (unused)
//
// The SDK calls it whenever the exposure, the window or the speed mode changes. The driver
// mirrors the first two (init writes 0x3009=0x01 in the reglist; a HighSpeed toggle takes effect
// at the next SetExposure/SetROI). Rewriting FRSEL at every tail gives normal captures after a
// high-speed session bit0 back.
func imx462WriteClockSel(rm Regmap) error {
	cur, err := rm.ReadReg(imx462RegGainMode)
	if err != nil {
		return err
	}
	v := cur & 0x10 // preserve HCG; rewrite the FRSEL field below it
	if !imx462HighSpeed(rm) {
		v |= 0x01 // 12-bit normal clock (18562); high-speed (37124) runs FRSEL 0
	}
	return rm.WriteReg(imx462RegGainMode, v)
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

// imx462HighSpeed reports whether the 10-bit high-speed readout is in effect: the mode flag with
// RAW8 output (the SDK's ASI_HIGH_SPEED_MODE applies to 8-bit output only; RAW16 keeps the 12-bit
// format and clock). The format block, the clock select and the HMAX/line-time pair all key off
// this one predicate.
func imx462HighSpeed(rm Regmap) bool {
	m := ModeOf(rm)
	return m.HighSpeed && m.BytesPerPx < 2
}

// imx462ClockFloor returns the pixel clock + HMAX floor for the live readout mode: the 10-bit
// high-speed pair, else the 12-bit normal pair.
func imx462ClockFloor(rm Regmap) (clock, floor int) {
	if imx462HighSpeed(rm) {
		return imx462ClkKHzHS, imx462HMAXFloorHS
	}
	return imx462ClkKHz, imx462HMAXFloor
}

// imx462SetExposure (SetExp): STARVIS VMAX/SHS (ApplyExposure) + FPGA trigger mode (reg0 bit7)
// at >= 1 s. The readout line clock is throttled (SetFPGAHMAX) so a long exposure fits inside
// one default-length frame and SHS carves the integration: VMAX stays at height+18, SHS =
// (height+17)-exposureLines. The computed HMAX is written to the FPGA first so the FPGA line
// rate and the SHS math agree. SDK reference at clock 18562, USB2 FPSPercent 40, 100 ms: HMAX
// 4085, VMAX 1114, SHS 659.
func imx462SetExposure(rm Regmap, d time.Duration) error {
	trigger := d >= imx462LongExpUs*time.Microsecond
	if err := SetFPGABit(rm, 0x00, 0x80, trigger); err != nil {
		return err
	}
	// HMAX from the live ROI dimensions when set (a sub-frame ROI sustains a shorter line
	// period; the bandwidth throttle scales with the frame size), else full-frame. HMAX =
	// max(link-bandwidth candidate, floor)·100/FPSPercent (the SDK's bandwidth formula; see the floor constants).
	hw, hh := imx462FullWidth, imx462FullHeight
	if rd := ModeOf(rm); rd.Width > 0 && rd.Height > 0 { // zero Height collapses the candidate
		hw, hh = rd.Width, rd.Height
	}
	clk, floor := imx462ClockFloor(rm)
	if err := WriteFPGAHMAX(rm, HMAX(hw, hh, clk, floor, imx462VBlankAdd, ModeOf(rm))); err != nil {
		return err
	}
	if err := ApplyExposure(rm, imx462Shutter, imx462RegLatch, d); err != nil {
		return err
	}
	// the clock select at the tail, as SetExp does: the sensor clock select must track the live mode
	// or the HMAX/SHS math above runs at the wrong physical rate.
	return imx462WriteClockSel(rm)
}

// imx462GetOffset reads the offset back from 0x300a/0x300b, the pair imx462SetOffset writes
// 16-bit little-endian with no scaling.
func imx462GetOffset(rm Regmap) (int, error) {
	v, err := ReadRegLE(rm, []uint16{0x300a, 0x300b})
	return int(v), err
}

func imx462SetOffset(rm Regmap, offset int) error {
	if err := poaUnsupported(rm, "imx462", "SetOffset"); err != nil {
		return err
	}
	v := uint16(offset)
	if err := rm.WriteReg(0x300b, (v>>8)&0xff); err != nil {
		return err
	}
	return rm.WriteReg(0x300a, v&0xff)
}

// imx462SetROI: the window start (X aligned to 4, Y to 2) + the window write (mode 0x3006, window,
// the 16-bit output select, FPGA geometry, HMAX), then the clock select at the tail.
func imx462SetROI(rm Regmap, x, y, w, h, bin int) error {
	if err := poaUnsupported(rm, "imx462", "SetROI"); err != nil {
		return err
	}
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

	if err := WithLatch(rm, imx462RegLatch, func() error {
		for _, rv := range []RegVal{
			{Reg: imx462RegStartXL, Val: ux & 0xff}, {Reg: imx462RegStartXH, Val: (ux >> 8) & 0xff},
			{Reg: imx462RegStartYL, Val: uy & 0xff}, {Reg: imx462RegStartYH, Val: (uy >> 8) & 0xff},
		} {
			if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
				return err
			}
		}
		return nil
	}); err != nil {
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

	// the 16-bit output select: the sensor output bit format, branching on the high-speed flag. Normal or
	// RAW16 uses the 12-bit ADBIT block (0x3005=1, OUTFMT 0x3046=0xf1, 0x3129=0, 0x317c=0,
	// 0x31ec=0x0e) with FPGA ADC_BIT=1. High-speed and RAW8 uses the 10-bit reformat
	// (0x3046=0xf0, 0x3005=0, 0x3129=0x1d, 0x317c=0x12, ADC_BIT=0): a shorter ADC ramp clocked
	// 2× faster (imx462ClockFloor switches clock 18562→37124, floor 261→245).
	out16 := []RegVal{{Reg: 0x3046, Val: 0xf1}, {Reg: 0x3005, Val: 0x01}, {Reg: 0x3129, Val: 0x00}, {Reg: 0x317c, Val: 0x00}, {Reg: 0x31ec, Val: 0x0e}}
	adcBit := uint16(0x01) // FPGA reg 0x0a bit0 (SetFPGAADCWidthOutputWidth's ADC_BIT): 1 = 12-bit
	if imx462HighSpeed(rm) {
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
	// Link-bandwidth HMAX for the new geometry (see imx462SetExposure).
	clk, floor := imx462ClockFloor(rm)
	if err := ProgramHMAX(rm, w, h, clk, floor, imx462VBlankAdd); err != nil {
		return err
	}
	// the clock select at the tail, as SetResolution does.
	return imx462WriteClockSel(rm)
}
