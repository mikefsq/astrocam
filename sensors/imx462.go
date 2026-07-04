// IMX462, STARVIS Type-1/2.8 sibling of the IMX290. Register map is byte-identical to the
// IMX290MM Mini except three deltas: the HCG threshold (gain 80 vs 60), the init reglist
// values, and init tail 0x305f (0x01 vs 0x00). Structure (gain via 0x3009 conv-gain + 0x3014
// code, SHS 0x3020-22, offset 0x300a/0b, STARVIS ROI, the host-timed trigger worker) mirrors
// the imx290.
//
// Op map:
//
//	InitCamera        (reglist 73 entries + explicit tail + FPGA bringup)
//	SetCMOSClk        (0x3009 FRSEL select + per-clock REG_FRAME_LENGTH_PKG_MIN; see imx462WriteClockSel)
//	Cam_SetResolution (mode 0x3006, window W 0x3042/0x3043 H 0x303e/0x303f, FPGA HBLK/VBLK 9/W/H)
//	SetStartPos       (ROI X 0x3040/0x3041, Y 0x303c/0x303d; X align 4, Y align 2)
//	SetGain           (clamp 0..600, HCG bit 0x10 in 0x3009 ABOVE gain 80, code to 0x3014)
//	SetExp            (SHS 24-bit 0x3020..0x3022, VMAX via SetFPGAVMAX; +18/+17 math)
//	SetBrightness     (offset -> 0x300a/0x300b, 16-bit LE, no scaling)
//	the capture worker (sensor stream gate: standby 0x3000 = 1 stop / 0 start)
package sensors

import . "github.com/mikefsq/astrocam"

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
	imx462FullWidth   = 1936 // MaxWidth (ASI462MC reports 1936×1096, same array as the 290)
	imx462FullHeight  = 1096
	imx462ClkKHz      = 18562 // RAW16 pixel clock (INCK/2); RAW8 would be 37124
	// HMAX floor = the ASI SDK's REG_FRAME_LENGTH_PKG_MIN (CCameraS462MC SetFPSPerc reads
	// .data+0x10). HMAX = max(bandwidth candidate, floor)·100/FPSPercent. The candidate is
	// link-switched — MAX_DATASIZE (.data+0) is written by SetOutput16Bits: USB3→360715,
	// USB2→43272 — so on USB3 the candidate (~196 full-frame) is under the floor and the
	// floor pins HMAX, while on USB2 the candidate (1634 full-frame) DOMINATES: the object
	// writes 1634·100/pct (wire-confirmed: HMAX 4085 at USB2 pct=40). astrocam's HMAX()/HMAXBW
	// is this exact formula (bwUSB2/bwUSB3 = MAX_DATASIZE·10·100).
	// REG_FRAME_LENGTH_PKG_MIN is DYNAMIC in the SDK: SetCMOSClk writes it per clock
	// (0071_CCameraS462MC.o .text+0x1d50 → .data+0x10; the 1100 static initializer is dead
	// after InitCamera's first SetCMOSClk call). Per-clock floors from the object:
	// 18562→261 (0x105), 37124→245 (0xf5), 9281→145 (0x91). The old in-tree 1100 (read
	// statically from .data) over-floored USB3 HMAX ~4× (261 ≈ 64 fps full-frame RAW16,
	// matching the published spec; 1100 ≈ 13.6 fps). USB2 is unaffected (the bandwidth
	// candidate 1634 dominates either floor — the wire-confirmed 4085 stands).
	// VERIFY-HW: USB3 readout at the 261 floor not yet exercised on a bench 462.
	imx462HMAXFloor   = 261   // SetCMOSClk(18562) → REG_FRAME_LENGTH_PKG_MIN
	imx462ClkKHzHS    = 37124 // 10-bit high-speed pixel clock: 2× the 12-bit clock
	imx462HMAXFloorHS = 245   // SetCMOSClk(37124) → REG_FRAME_LENGTH_PKG_MIN
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
	// ASI Brightness / black level. Default 100 matches the SDK (ASIInitCamera applies
	// Brightness=100 on the ASI462). The offset→DN mapping is floor 41/71/141 in 12-bit at
	// offsets 0/30/100.
	OffsetMax: 240, OffsetDef: 100,
	Info: CameraInfo{
		MaxWidth:  imx462FullWidth, // active pixels
		MaxHeight: imx462FullHeight,
		PixelUm:   2.9,         // 2.9 µm pixel pitch
		BitDepth:  12,          // 12-bit ADC (ADBIT=12; tail 0x3005=0x01)
		Bayer:     "RGGB",      // CFA (color/MC model only); surfaced when Model.Color
		Bins:      []int{1, 2}, // 1× and 2× (mode 0x3006=0x22, window·bin)
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

	FX3DMAMarkers: true, // FX3 brackets each frame with 0x5A7E/0x3CF0 marker words (HW-confirmed)
}

// imx462Worker — the capture worker, host-timed single-shot capture. Same skeleton
// as the imx290 (STARVIS standby-0x3000 gate; ≥1 s uses FPGA trigger MODE/SIGNAL).
func imx462Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()
	longExp := exposure >= imx462LongExpUs*time.Microsecond // ≥ 1 s — matches SetExposure's trigger band
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
		// FPGAStart. Always clear bit4 (readout-stop). In FREE-RUN (<1 s) also clear bit6 (WaitMode)
		// so the sensor streams frames (SDK writes reg0 = 0x21). In TRIGGER mode (≥1 s) KEEP bit6
		// SET — the FPGA must stay in wait mode for the trigger signal (reg 0x0b bit0) to gate the
		// integration; clearing it leaves the readout un-gated and the frame never arrives. Wire-
		// confirmed against the SDK: trigger-mode FPGAStart = reg0 0xe1 (bit6 kept), not 0xa1.
		mask := uint16(0x50) // bit4 + bit6 (free-run)
		if longExp {
			mask = 0x10 // bit4 only — keep WaitMode for the trigger
		}
		return SetFPGABit(rm, 0x00, mask, false)
	}
	// EnableFPGATriggerSignal (reg 0x0b bit0): only the ≥1 s trigger band uses it. In normal mode
	// the sensor free-runs (SHS-timed) and the FPGA just starts and reads one frame.
	trigger := func(on bool) error { return SetFPGABit(rm, 0x0b, 0x01, on) }
	// integrate runs one full trigger cycle: the signal hold IS the integration, so hold for the
	// FULL exposure (not exp−200 ms — that under-integrates the exactly-1 s boundary by 200 ms),
	// polling the abort flag. Used on the first arm and again after a ResetDevice re-arm (the
	// re-armed FPGA is back in wait mode; without a fresh trigger edge no frame ever comes).
	integrate := func() error {
		if err := trigger(true); err != nil {
			return err
		}
		for start := time.Now(); time.Since(start) < exposure; {
			if ctl.Aborted() {
				// StopExposure ran: bail instead of waiting out the integration. Drop the
				// trigger signal on the way out — left asserted, the next long exposure's
				// trigger(true) is a no-edge write and the FPGA never gates it.
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
	// Halt the readout on EVERY return — the SDK WorkingFunc exit: StopSensorStreaming
	// (FPGAStop + standby=1, per the 462 object) then SendCMD(0xAA) then ResetEndPoint. A
	// sensor left free-running with no reader backs up the FX3 GPIF until the firmware
	// crashes (the documented 174 mechanism). Best-effort: on the error paths the device
	// may already be gone, and a failed stop must not fail a good frame.
	defer func() {
		_ = SetFPGABit(rm, 0x00, 0x10, true) // FPGAStop: reg0 bit4
		_ = rm.WriteReg(imx462RegStandby, 1) // standby (sensor stop)
		_ = ctl.VendorCmd(0xAA)
		_ = ctl.ResetEndpoint()
	}()
	_ = ctl.ResetEndpoint()
	if longExp {
		if err := integrate(); err != nil {
			return 0, err
		}
	}
	// Free-run (<1 s): NO pre-read sleep and NO FPGABufReload. The SDK's WorkingFunc calls
	// startAsyncXfer immediately after FPGAStart with timeout = exposure+1000 ms — the URB batch
	// waits on the pipe through the integration, so the GPIF never streams without a reader.
	// (Sleeping first and reading afterwards backed up the FIFO and tore/stalled on USB2; the
	// pre-read reg 0x18 control write is also NOT in the SDK's normal path — it appears only in
	// the trigger-retry below, gated on FPGA reg 0x23 bit2.)
	target := ctl.FrameBytes()
	if target > len(buf) {
		target = len(buf)
	}
	readTotal := exposure + time.Second // SDK free-run: startAsyncXfer timeout = exp + 1000 ms
	if longExp {
		readTotal = time.Second // SDK trigger mode: frame already integrated; 1000 ms
	}
	// Retry ladder, mirroring the SDK WorkingFunc's failure handling:
	//   short frame          -> ResetEndpoint, fresh full-frame read
	//   trigger-mode short   -> if FPGA reg 0x23 bit2 says the frame is still buffered, pulse
	//                           FPGABufReload (reg 0x18 bit0) and re-read WITHOUT re-integrating
	//   4 consecutive zeros  -> ResetDevice + full re-arm
	// A frame short by ≤512 bytes counts as complete (the SDK's startAsyncXfer treats
	// completed+0x200 == frameBytes as success); the missing tail is zeroed.
	zeros, reloads := 0, 0
	for attempt := 0; ; attempt++ {
		n, err := ctl.StreamFramePrequeued(buf[:target], 500*time.Millisecond, readTotal)
		if err != nil {
			return n, err
		}
		// Complete = exact byte count, or (SDK startAsyncXfer tolerance) EXACTLY 512 short —
		// `n >= target-512` would let a small-ROI frame (target <= 512) accept n=0 and
		// fabricate an all-zero "frame".
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
			// stalled pipe (observed: 64 bytes with a valid 0x5A7E marker, every attempt), so
			// tiny reads count as empty too — a plain ResetEndpoint doesn't unpark it.
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
					// must re-run the trigger cycle — without it the reads below wait ~10 s
					// for a frame that can never arrive. Costs a full re-integration, as the
					// SDK's re-entered WorkingFunc does. Fresh integration ⇒ fresh buffer, so
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

// imx462InitFPGA — the FPGA-side bringup after the Sony init writes (InitCamera);
// same as the 290.
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
		// (the object's SetCMOSClk), rewritten at the end of every SetExp/SetResolution.
		if err := rm.WriteReg(imx462RegGainMode, mode); err != nil {
			return err
		}
		return rm.WriteReg(imx462RegGainCode, code&0xff)
	})
}

// imx462WriteClockSel is the object's SetCMOSClk (0071_CCameraS462MC.o, .text+0x1d50):
// program the 0x3009 clock / FRSEL select for the live readout mode, keeping the
// conversion-gain bit (the object computes gain>80 ? 0x10 : 0 and ORs the FRSEL field;
// the read-modify here preserves the same bit SetGain maintains). Decoded values:
//
//	18562 kHz (12-bit normal)     -> FRSEL 0x01
//	37124 kHz (10-bit high-speed) -> FRSEL 0x00
//	 9281 kHz (hardware bin 2)    -> FRSEL 0x00   (gated on the object's hard-bin flag;
//	                                 NOT wired — the bin-2 clock/floor swap is unverified
//	                                 on the wire, bin 2 runs the normal clock today)
//	other                         -> FRSEL 0x02   (unused)
//
// The object calls it from SetExp, SetResolution, InitCamera and SetHighSpeedMode; the
// driver mirrors the first two (init writes 0x3009=0x01 in the reglist, and a HighSpeed
// toggle takes effect at the next SetExposure/SetROI by design). This closes the one-way
// FRSEL clear (REVIEW 2.6): normal captures after a high-speed session get bit0 back.
func imx462WriteClockSel(rm Regmap) error {
	cur, err := rm.ReadReg(imx462RegGainMode)
	if err != nil {
		return err
	}
	v := cur & 0x10 // preserve HCG; rewrite the FRSEL field below it
	if !ModeOf(rm).HighSpeed {
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

// imx462ClockFloor returns the pixel clock + HMAX floor for the live readout mode: the
// 10-bit high-speed pair (37124/245) when mode.HighSpeed, else 12-bit normal (18562/261) —
// the per-clock REG_FRAME_LENGTH_PKG_MIN values SetCMOSClk installs (see the constants).
func imx462ClockFloor(rm Regmap) (clock, floor int) {
	if ModeOf(rm).HighSpeed {
		return imx462ClkKHzHS, imx462HMAXFloorHS
	}
	return imx462ClkKHz, imx462HMAXFloor
}

// imx462SetExposure — STARVIS VMAX/SHS (ApplyExposure) + FPGA trigger MODE (reg0 bit7) for ≥1 s,
// same as the 290 (SetExp).
//
// HMAX-stretch: the readout line clock is throttled (the FPS-percent throttle → SetFPGAHMAX) so a
// long exposure fits inside one default-length frame, then SHS carves the integration — VMAX stays
// at height+18, SHS = (height+17)−exposureLines. The computed HMAX must be written to the FPGA here
// first so the FPGA line rate and the SHS math agree. With clock 18562 + the USB2 FPSPercent=40 the
// SDK values are HMAX 4085, VMAX 1114, SHS 659 for a 100 ms exposure.
func imx462SetExposure(rm Regmap, d time.Duration) error {
	trigger := d >= imx462LongExpUs*time.Microsecond
	if err := SetFPGABit(rm, 0x00, 0x80, trigger); err != nil {
		return err
	}
	// HMAX (line time) from the live ROI dimensions when set, so a sub-frame ROI gets the
	// shorter line period it can sustain (the bandwidth throttle scales with the frame size);
	// falls back to full-frame. HMAX = max(link-bandwidth candidate, floor)·100/FPSPercent —
	// the SDK's SetFPSPerc with the link-switched MAX_DATASIZE (see the floor constants above).
	// On USB2 the candidate dominates (1634 full-frame → sensor rate pinned to the 43.272 MB/s
	// budget); on USB3 the per-clock floor (261 normal / 245 high-speed) does. Written before
	// the SHS math so the FPGA line rate and SHS agree (lineTimeNs derives the same).
	hw, hh := imx462FullWidth, imx462FullHeight
	if rd := ModeOf(rm); rd.Width > 0 && rd.Height > 0 { // both dims, or a zero Height collapses the candidate
		hw, hh = rd.Width, rd.Height
	}
	clk, floor := imx462ClockFloor(rm)
	if err := WriteFPGAHMAX(rm, HMAX(hw, hh, clk, floor, imx462VBlankAdd, ModeOf(rm))); err != nil {
		return err
	}
	if err := ApplyExposure(rm, imx462Shutter, imx462RegLatch, d); err != nil {
		return err
	}
	// SetCMOSClk at the tail, as the object's SetExp does: the sensor clock select must
	// track the live mode or the HMAX/SHS math above runs at the wrong physical rate.
	return imx462WriteClockSel(rm)
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
// (mode 0x3006, window→0x3042/3,0x303e/f, FPGA geometry). Same as the 290.
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

	// SetOutput16Bits (shared with the 290 Mini): set the sensor output bit-format, branching on
	// the high-speed mode flag. Normal OR RAW16 uses the 12-bit ADBIT block (0x3005=1, OUTFMT
	// 0x3046=0xf1, 0x3129=0, 0x317c=0, 0x31ec=0x0e) with FPGA ADC_BIT=1. High-speed AND RAW8 uses
	// the 10-bit reformat (0x3046=0xf0, 0x3005=0, 0x3129=0x1d, 0x317c=0x12, ADC_BIT=0) — a shorter
	// ADC ramp clocked 2× faster (imx462ClockFloor switches clock 18562→37124, floor 261→245).
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
	// Link-bandwidth HMAX for the new geometry (see imx462SetExposure).
	clk, floor := imx462ClockFloor(rm)
	if err := ProgramHMAX(rm, w, h, clk, floor, imx462VBlankAdd); err != nil {
		return err
	}
	// SetCMOSClk at the tail, as the object's SetResolution does.
	return imx462WriteClockSel(rm)
}
