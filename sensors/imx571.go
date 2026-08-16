// IMX571 (APS-C, 26 MP) in the ZWO ASI2600 family. Same Exmor/STARVIS-2 die family as the
// full-frame IMX455, so the two profiles share the same shape; the registers and constants here
// are the 571's own.
//
// The register model:
//   - Master/stream gate is WriteSONYREG 0x1ee: 1 = stream-start, 5 = stop.
//   - Exposure is TWO modes: < 1 s free-run (VMAX = vblank+effHeight, time encoded in SHS); >= 1 s
//     (> 0xf423f µs) FPGA wait+trigger mode — EnableFPGAWaitMode (reg0 bit6) +
//     EnableFPGATriggerMode (reg0 bit7) set, VMAX held near one frame, the integration HOST-timed
//     by the worker via EnableFPGATriggerSignal (FPGA reg 0x0b bit0).
//   - SHS ENCODING (CRITICAL): the normal free-run path writes (SHS>>1): low = (SHS>>1)&0xff to
//     0x18, high = (SHS>>9)&0xff to 0x19. (The high-speed/hardware-bin branch instead writes the
//     FULL SHS with no halve; we are always in the normal free-run path, so SHS>>1 is used.)
//   - Gain code = 4095·(1 − 10^(−gain/200)) (pow with K = 4095). Five effective ZWO segments split
//     at -25 / 0 / 100 / 460; the top band quantises to a 60-step coarse stage = ceil((gain−460)/60)
//     placed in reg 0x40 bits[4:7].
//   - Window regs: HEIGHT 0x0a/0x0b, WIDTH (×4-aligned +0x18) 0x1dd/0x1de, window-mode 0x1d8 =
//     4 full / 0 binned; ROI start X 0xa7/0xa8/0xa9, Y 0x08/0x09; 0x07 is the coupled-group apply.
//   - Optical-black crop SetFPGAVBLK/SetFPGAHBLK take the per-mode FPGA_SKIP_LINE / FPGA_SKIP_CLOUMN
//     values, rewritten per bin by the mode select.
//   - Data plane: windowed FX3 pump + FPGABufReload tail-flush + stall recovery; see imx571Worker.
//
// PlayerOne shares this die — SetGain/SetOffset/GainCaps/OffsetCaps dispatch on the regmap's VID
// (ZWO 0x03C3 vs PlayerOne 0xA0A0).

package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
	"math"
	"time"
)

// Sony IMX571 — APS-C BSI CMOS (ZWO ASI2600 family, 26 MP). The WriteSONYREG addresses are
// small FPGA-bank indices (0x18/0x19 shutter, 0x30..0x33 gain, 0x2f/0x40 conversion gain,
// 0xa7..0xa9 ROI-X, 0x08/0x09 ROI-Y, 0x0a/0x0b window-HEIGHT, 0x1dd/0x1de window-WIDTH).
// Register 0x07 is the coupled-group apply.
//
// Register map summary:
//
//	InitCamera          (reglist_init loop + explicit tail + FPGA bringup)
//	InitSensorMode      (per-mode table select by bin/depth + V/vblank/skip rewrite)
//	Cam_SetResolution   (window H 0x0a/0x0b, W 0x1dd/0x1de, mode 0x1d8)
//	SetStartPos         (ROI X 0xa7/0xa8/0xa9, Y 0x08/0x09)
//	SetGain             (setup 0x67f, code 0x30..0x33, conv 0x2f/0x40)
//	SetExp              (shutter SHS>>1 to 0x18/0x19, VMAX via FPGA)
//	StartSensorStreaming/StopSensorStreaming (master 0x1ee = 1 / 5)
const (
	// SetGain — gain code 4095*(1 - 10^(-gain/200)) split across two
	// 16-bit copies (0x30/0x31 and 0x32/0x33); 0x2f/0x40 pick conversion gain.
	imx571RegGainSetup = 0x67f // written at the start of a gain update (0 / 0x11)
	imx571RegGainAL    = 0x30  // analog gain code, low byte  (copy 1)
	imx571RegGainAH    = 0x31  // analog gain code, high byte (copy 1)
	imx571RegGainBL    = 0x32  // analog gain code, low byte  (copy 2)
	imx571RegGainBH    = 0x33  // analog gain code, high byte (copy 2)
	imx571RegConvLow   = 0x2f  // LCG/HCG conversion-gain select (0/1)
	imx571RegConvHi    = 0x40  // top-band coarse-stage nibble, bits[4:7] (>460 branch)

	// SetStartPos — X aligned to 16, written nibble-shifted; Y direct.
	imx571RegApply    = 0x07 // coupled-group apply (written 1 by ROI / res ops)
	imx571RegStartXEn = 0xa7 // ROI-X enable / mode flag (=1)
	imx571RegStartXL  = 0xa8 // X start bits [4:12]  (X>>4)
	imx571RegStartXH  = 0xa9 // X start bits [12:20] (X>>12)
	imx571RegStartYL  = 0x08 // (Y start + 0x19/0x1b) low byte
	imx571RegStartYH  = 0x09 // (Y start + 0x19/0x1b) high byte

	// Cam_SetResolution — output window in OUTPUT (binned) pixels.
	imx571RegWinMode = 0x1d8 // window mode: 4 full / 0 binned
	imx571RegHeightL = 0x0a  // window HEIGHT low  (effHeight, +2 when binned)
	imx571RegHeightH = 0x0b  // window HEIGHT high
	imx571RegWidthL  = 0x1dd // window WIDTH low  ((effWidth ×4-aligned)+0x18, &0xfc)
	imx571RegWidthH  = 0x1de // window WIDTH high

	// SetExp — shutter SHS, written as two bytes (FPGA holds VMAX). SHS is HALVED (>>1) on the
	// normal path before the split.
	imx571RegSHS0 = 0x18 // SHS low byte  = (SHS>>1)&0xff
	imx571RegSHS1 = 0x19 // SHS high byte = (SHS>>9)&0xff

	// Master streaming gate (WriteSONYREG 0x1ee): 1 = stream-start, 5 = stop. Init's reglist seeds
	// it to 1.
	imx571RegMaster = 0x1ee

	imx571GainMax    = 0x2bc // 700 (0.1 dB units); SetGain clamp hi
	imx571GainMin    = -0x19 // -25: clamp lo
	imx571GainHCGAt  = 0x64  // 100: at/above this the high-conv-gain path (0x2f=1)
	imx571GainHCGHi  = 0x1cc // 460: above this the top-band 60-step packing
	imx571GainCodeFS = 4095  // full-scale gain code = K = 4095.0
	imx571GainStage  = 0x3c  // 60: the top-band coarse-gain step

	imx571StartXAlign = 16   // X start aligned to 16 (mask 0xfffffff0)
	imx571StartYOff   = 0x19 // Y start += 0x19 before the 0x08/0x09 write (bin 1/2/4)
	imx571StartYOff3  = 0x1b // Y start += 0x1b for bin 3

	imx571ExpMinUs  = 0x20       // 32 µs: SetExp clamp lo
	imx571ExpMaxUs  = 0x77359400 // 2,000,000,000 µs: clamp hi
	imx571LongExpUs = 0xf4240    // 1,000,000 µs: > 0xf423f takes the wait+trigger path

	// Timing model. CalcFrameTime / SetExp compute (free-run band; the >= 1 s trigger band holds
	// VMAX at one frame with SHS = 20, see imx571SetExposure):
	//
	//	lineTime_ns = V * 1e6 / clock
	//	VMAX        = vblank + effHeight        (BLANK_LINE_OFFSET + effH)
	//	SHS         = VMAX - 1 - lines          (clamp >=1, 17-bit cap 0x1fffe)
	//	SHS         = SHS>>1                     (normal path)
	//
	// V (the HMAX line base) and the vblank (BLANK_LINE_OFFSET) are set PER BIN by the mode select
	// — see imx571SelectMode. clock = 20000. effHeight = height/bin, except bin4 = height/2 (the
	// bin-2-geometry reuse).
	imx571ClockHz   = 20000 // line-time clock divisor — no per-mode branch
	imx571SHSOffset = -1    // SHS = VMAX - 1 - lines

	// Per-mode V (line base) and vblank, rewritten by the mode select:
	//   bin1 V = 0x546 = 1350; bin2/4 V = 0x1ea = 490; bin3 V = 0x0fa = 250.
	//   BLANK_LINE_OFFSET: bin1=0x30(48), bin2/4=0x1c(28), bin3=0x18(24).
	imx571VBin1   = 1350 // bin1 line base
	imx571VBin2   = 490  // bin2 & bin4 line base
	imx571VBin3   = 250  // bin3 line base
	imx571VBlank1 = 48   // bin1 vblank (BLANK_LINE_OFFSET)
	imx571VBlank2 = 28   // bin2 & bin4 vblank
	imx571VBlank3 = 24   // bin3 vblank

	// FPGA optical-black crop — FPGA_SKIP_LINE / FPGA_SKIP_CLOUMN, rewritten per mode and read into
	// SetFPGAVBLK/SetFPGAHBLK by SetStartPos:
	//   FPGA_SKIP_LINE:   bin1=0x2d(45), bin2/4=0x19(25), bin3=0x17(23) → SetFPGAVBLK
	//   FPGA_SKIP_CLOUMN: bin1=0x18(24), bin2/4=0x12(18), bin3=0x0b(11) → SetFPGAHBLK
	imx571SkipLine1   = 45 // bin1 FPGA_SKIP_LINE → VBLK
	imx571SkipLine2   = 25 // bin2/4
	imx571SkipLine3   = 23 // bin3
	imx571SkipColumn1 = 24 // bin1 FPGA_SKIP_CLOUMN → HBLK
	imx571SkipColumn2 = 18 // bin2/4
	imx571SkipColumn3 = 11 // bin3

	imx571FullWidth  = 6224 // output width at full-frame bin 1 (= MaxWidth)
	imx571FullHeight = 4168 // output rows at full-frame bin 1 (= MaxHeight)
)

// imx571InitCommon is the common (mode-independent) first stage of InitCamera: the reglist_init
// table (54 entries; reg=0xffff is a delay of val ms) followed by the explicit WriteSONYREG tail.
var imx571InitCommon = []RegVal{
	// --- reglist_init: reg/val16 pairs ---
	{Reg: 0x01ee, Val: 0x01}, {Reg: 0x0000, Val: 0x04}, {Reg: 0xffff, Val: 0x0a}, // delay 10 ms
	{Reg: 0x0003, Val: 0x10}, {Reg: 0x0018, Val: 0x01}, {Reg: 0x0027, Val: 0x06}, {Reg: 0x0051, Val: 0x08},
	{Reg: 0x00d3, Val: 0x08}, {Reg: 0x0133, Val: 0x8c}, {Reg: 0x0324, Val: 0x01}, {Reg: 0x0325, Val: 0x0f},
	{Reg: 0x0368, Val: 0xe0}, {Reg: 0x0400, Val: 0x0e}, {Reg: 0x0454, Val: 0x22}, {Reg: 0x0456, Val: 0x22},
	{Reg: 0x0559, Val: 0x19}, {Reg: 0x055a, Val: 0x17}, {Reg: 0x055c, Val: 0x19}, {Reg: 0x055d, Val: 0x17},
	{Reg: 0x055f, Val: 0x20}, {Reg: 0x0560, Val: 0x1e}, {Reg: 0x0562, Val: 0x20}, {Reg: 0x0563, Val: 0x1e},
	{Reg: 0x056b, Val: 0x27}, {Reg: 0x056c, Val: 0x25}, {Reg: 0x056e, Val: 0x20}, {Reg: 0x056f, Val: 0x1e},
	{Reg: 0x0573, Val: 0x00}, {Reg: 0x0590, Val: 0x01}, {Reg: 0x0596, Val: 0x19}, {Reg: 0x0597, Val: 0x14},
	{Reg: 0x0598, Val: 0x20}, {Reg: 0x0599, Val: 0x1b}, {Reg: 0x0600, Val: 0x1c}, {Reg: 0x0635, Val: 0x19},
	{Reg: 0x0636, Val: 0x15}, {Reg: 0x0637, Val: 0x20}, {Reg: 0x0638, Val: 0x15}, {Reg: 0x063a, Val: 0x19},
	{Reg: 0x063b, Val: 0x15}, {Reg: 0x063c, Val: 0x20}, {Reg: 0x063d, Val: 0x15}, {Reg: 0x063f, Val: 0x19},
	{Reg: 0x0640, Val: 0x15}, {Reg: 0x0641, Val: 0x20}, {Reg: 0x0642, Val: 0x15}, {Reg: 0x066e, Val: 0x11},
	{Reg: 0x0671, Val: 0x11}, {Reg: 0x0674, Val: 0x11}, {Reg: 0x0677, Val: 0x11}, {Reg: 0x07cc, Val: 0x0a},
	{Reg: 0x0113, Val: 0x00}, {Reg: 0x0120, Val: 0xbc}, {Reg: 0x0121, Val: 0x01},
	// --- explicit WriteSONYREG tail ---
	{Reg: 0x03, Val: 0x10},
	{Reg: 0x07, Val: 0x01},
	{Reg: 0xa7, Val: 0x01},
	{Reg: 0x1d8, Val: 0x04},
	{Reg: 0x48, Val: 0x0f},
	{Reg: 0x51, Val: 0x08},
}

// imx571Init is the streaming default: the common init followed by the bin-1 16-bit per-mode
// table (the two-stage init — InitCamera then InitSensorMode). Only the bin-1 16-bit mode is
// wired as the default; the other tables are applied by SetROI per bin.
var imx571Init = append(append([]RegVal{}, imx571InitCommon...), imx571ModeFull16...)

// IMX571 is the Sony IMX571 APS-C profile (ZWO ASI2600 family, PlayerOne Poseidon). It tracks
// the hardware-validated IMX455 profile; the 571 itself is not yet hardware-validated.
var IMX571 = Sensor{
	Name:     "IMX571", // ASI2600MC Pro; Sony IMX571 APS-C BSI (color, RGGB)
	GainMax:  imx571GainMax,
	ExpMinUs: imx571ExpMinUs,
	ExpMaxUs: imx571ExpMaxUs,
	Info: CameraInfo{
		MaxWidth:  imx571FullWidth,
		MaxHeight: imx571FullHeight,
		PixelUm:   3.76,
		BitDepth:  16,
		Bayer:     "RGGB",            // MC = color
		Bins:      []int{1, 2, 3, 4}, // 1/2/3/4× decoded (per-mode tables + V/vblank/skip)
	},
	// ASI Brightness / black level. Caps: 0..240, def 1.
	OffsetMax:   240,
	OffsetDef:   1,
	Init:        imx571Init,
	InitFPGA:    imx571InitFPGA,
	SetGain:     imx571SetGain,
	GainCaps:    imx571GainCaps,
	SetExposure: imx571SetExposure,
	SetOffset:   imx571SetOffset,
	GetOffset:   imx571GetOffset,
	OffsetCaps:  imx571OffsetCaps,
	SetROI:      imx571SetROI,
	// Master/stream gate, tracking the hardware-verified IMX455 shape: StopSensorStreaming =
	// 0x1ee←5 + CamSetStandby(1) (reg0 bit0); StartSensorStreaming = 0x1ee←1 + CamSetWakeup(1)
	// (reg0 bit2) + 10 ms + CamSetStandby(0). The worker's inline stop/start encode the same
	// sequence; these hooks serve StopExposure and the StartVideo arm.
	StreamStop: func(rm Regmap) error {
		if err := rm.WriteReg(imx571RegMaster, 5); err != nil { // 0x1ee = 5 (stop)
			return err
		}
		return rm.WriteRegBits(0, 0, 0, 1) // CamSetStandby(1): sensor reg0 bit0 = 1
	},
	StreamStart: func(rm Regmap) error {
		if err := rm.WriteReg(imx571RegMaster, 1); err != nil { // 0x1ee = 1 (start)
			return err
		}
		if err := rm.WriteRegBits(0, 2, 2, 1); err != nil { // CamSetWakeup(1): reg0 bit2 = 1
			return err
		}
		time.Sleep(10 * time.Millisecond)  // usleep(0x2710)
		return rm.WriteRegBits(0, 0, 0, 0) // CamSetStandby(0): reg0 bit0 = 0
	},
	Arm:    imx571Arm,    // the object's arm (StartVideo / free-run streaming use it too)
	Worker: imx571Worker, // rich arm + windowed stream read

	FX3DMAMarkers: true, // FX3 bridge framing (0x5A7E/0x3CF0 marker words)
}

// imx571Worker is the host-timed single-shot capture. The sensor gate goes through the 0x1ee
// master register via the StartSensorStreaming/StopSensorStreaming helpers (which also toggle
// CamSetWakeup reg0 bit2 / CamSetStandby reg0 bit0). For the >= 1 s bands SetExp arms trigger
// MODE (reg0 bit6/bit7); this worker drives the trigger SIGNAL (EnableFPGATriggerSignal, FPGA
// reg 0x0b bit0) whose 1->0 edge releases the frame. The SetExp trigger MODE bits are only safe
// WITH this worker.
//
//	arm:    SendCMD(0xAA)·StopSensorStreaming·SendCMD(0xA9)·StartSensorStreaming·
//	        ResetEndPoint(0x81)
//	expose: EnableFPGATriggerSignal(1) · host-time (>= 1 s only) · EnableFPGATriggerSignal(0)
//	read:   continuous windowed pump (ctl.StreamFrame) + FPGABufReload tail-flush, FPGAStop/
//	        usleep/FPGAStart on stall
//
// StartSensorStreaming = FPGAStop · 0x1ee=1 · CamSetWakeup(1) · usleep(10ms) · CamSetStandby(0)
// · FPGAStart. StopSensorStreaming = FPGAStop · 0x1ee=5 · CamSetStandby(1). The arm tracks the
// hardware-verified IMX455 worker (no per-capture device reset, no extra settle); the 571 is
// unverified on hardware. The very-long-band multi-exposure accumulation cycle is NOT
// reproduced — single-shot only.
func imx571Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()

	// Sensor reg-0 read-modify-write (ReadSONYREG 0 -> mask -> WriteSONYREG 0) — the bits
	// CamSetWakeup/CamSetStandby toggle: standby = reg0 bit0, wakeup = reg0 bit2.
	regRMW := func(set, clr uint16) error {
		v, err := rm.ReadReg(0)
		if err != nil {
			return err
		}
		return rm.WriteReg(0, (v|set)&^clr)
	}
	fpgaStop := func() error { return SetFPGABit(rm, 0x00, 0x10, true) } // reg0 bit4 = 1 (FPGAStop)
	// FPGABufReload: FPGA reg 0x18 bit0 — commits the frame's final partial DMA buffer.
	bufReload := func() error { return SetFPGABit(rm, 0x18, 0x01, true) }
	// EnableFPGATriggerSignal: FPGA reg 0x0b bit0. In wait+trigger mode (>= 1 s) the host holds
	// this for the integration time.
	triggerSignal := func(on bool) error { return SetFPGABit(rm, 0x0b, 0x01, on) }

	// StopSensorStreaming: the 0x1ee master gate under FPGAStop with CamSetStandby(1); the deferred
	// halt below uses it (the arm is imx571Arm).
	stopStreaming := func() error {
		if err := fpgaStop(); err != nil {
			return err
		}
		if err := rm.WriteReg(imx571RegMaster, 5); err != nil { // 0x1ee = 5
			return err
		}
		return regRMW(0x01, 0) // CamSetStandby(1): reg0 |= bit0
	}

	// --- arm (imx571Arm, the IMX455 shape; shared with StartVideo) ---
	if err := imx571Arm(ctl); err != nil {
		return 0, err
	}
	// Halt the readout on EVERY return — the 2600 object's WorkingFunc exit:
	// StopSensorStreaming (FPGAStop -> 0x1ee<-5 -> CamSetStandby(1), per
	// 0080_CCameraS2600MC_Pro.o; stopStreaming() encodes this) then
	// SendCMD(0xAA) then ResetEndPoint. A sensor left free-running with no reader backs up
	// the FX3 GPIF. Best-effort: a failed stop must not fail a good frame. VERIFY-HW: ported
	// from the object (mirrors the hardware-verified 455/6200); no 571 on the bench at port time.
	defer func() {
		_ = stopStreaming()
		_ = ctl.VendorCmd(FX3StreamStop)
		_ = ctl.ResetEndpoint()
	}()
	_ = ctl.ResetEndpoint() // ResetEndPoint(0x81)

	// >= 1 s runs in FPGA wait+trigger mode (SetExp set reg0 bit6/bit7 and held VMAX near one
	// frame). The integration is HOST-timed: assert the trigger signal, hold for the exposure,
	// then release so the frame clocks out. Below 1 s is free-run (sensor self-times via SHS).
	if exposure >= imx571LongExpUs*time.Microsecond {
		if err := triggerSignal(true); err != nil {
			return 0, err
		}
		for start := time.Now(); time.Since(start) < exposure; {
			if ctl.Aborted() {
				// StopExposure ran: bail, dropping the trigger signal on the way out —
				// left asserted, the next triggerSignal(true) is a no-edge write and the
				// FPGA never gates the integration. (Host-side hygiene, not object-derived:
				// the SDK snap thread never host-aborts mid-integration.)
				_ = triggerSignal(false)
				return 0, errExposureAborted
			}
			time.Sleep(100 * time.Millisecond)
		}
		if err := triggerSignal(false); err != nil {
			return 0, err
		}
	}

	// --- read one whole frame (continuous windowed pump, target = FrameBytes) ---
	target := ctl.FrameBytes()
	if target > len(buf) {
		target = len(buf)
	}
	// In the >= 1 s trigger band the integration has ALREADY completed above (host-timed
	// trigger hold), so the read only spans the readout — exposure-scaled timeouts there
	// made a stalled readout after a long sub block StopExposure for up to 2·exp+5 s.
	idle := exposure + 2*time.Second
	total := 2*exposure + 5*time.Second
	if exposure >= imx571LongExpUs*time.Microsecond {
		idle = 2 * time.Second
		total = 15 * time.Second // full-frame readout ceiling incl. USB2 + retries
	}
	// Pulse FPGABufReload throughout so the frame's final partial DMA buffer (the bytes past the
	// last 1-MiB boundary) flushes into a posted transfer; the windowed reader treats the
	// frame-end ZLP as non-terminal and keeps cycling until the whole frame lands.
	stop := make(chan struct{})
	done := make(chan struct{})
	go func() {
		defer close(done)
		t := time.NewTicker(20 * time.Millisecond)
		defer t.Stop()
		for {
			select {
			case <-stop:
				return
			case <-t.C:
				_ = bufReload()
			}
		}
	}()
	n, err := ctl.StreamFrame(buf[:target], idle, total)
	close(stop)
	<-done // JOIN: wait for the ticker's last bufReload before returning, so no control transfer
	// outlives the worker to race the next operation / the TEC loop.
	if err == nil && n < target && ctl.Aborted() {
		return n, errExposureAborted // StopExposure broke the read (AbortRead): clean abort, not a stall
	}
	return n, err
}

// imx571Arm is the stream arm, tracking imx455Arm: SendCMD(0xAA) · StopSensorStreaming (FPGAStop ·
// 0x1ee=5 · CamSetStandby(1)) · SendCMD(0xA9) · StartSensorStreaming (FPGAStop · 0x1ee=1 ·
// CamSetWakeup(1) · 10 ms · CamSetStandby(0) · FPGAStart). The FPGAStop inside StartSensorStreaming
// is the second stop after 0xA9 the DDR readout needs. Shared by the worker and StartVideo.
func imx571Arm(ctl WorkerCtl) error {
	rm := ctl.Rm()
	regRMW := func(set, clr uint16) error {
		v, err := rm.ReadReg(0)
		if err != nil {
			return err
		}
		return rm.WriteReg(0, (v|set)&^clr)
	}
	fpgaStop := func() error { return SetFPGABit(rm, 0x00, 0x10, true) }
	fpgaStart := func() error { return SetFPGABit(rm, 0x00, 0x10, false) }
	if err := ctl.VendorCmd(FX3StreamStop); err != nil { // FX3 stream stop
		return err
	}
	if err := fpgaStop(); err != nil { // StopSensorStreaming
		return err
	}
	if err := rm.WriteReg(imx571RegMaster, 5); err != nil { // 0x1ee = 5
		return err
	}
	if err := regRMW(0x01, 0); err != nil { // CamSetStandby(1)
		return err
	}
	if err := ctl.VendorCmd(FX3StreamStart); err != nil { // FX3 stream start
		return err
	}
	if err := fpgaStop(); err != nil { // StartSensorStreaming
		return err
	}
	if err := rm.WriteReg(imx571RegMaster, 1); err != nil { // 0x1ee = 1
		return err
	}
	if err := regRMW(0x04, 0); err != nil { // CamSetWakeup(1)
		return err
	}
	time.Sleep(10 * time.Millisecond)       // usleep(0x2710)
	if err := regRMW(0, 0x01); err != nil { // CamSetStandby(0)
		return err
	}
	return fpgaStart()
}

// imx571InitFPGA is the FPGA-side bringup InitCamera performs after the Sony init tail, using the
// FX3 register numbers. Shared helpers do the read-modify-writes.
//
//	FPGAReset                          reg0 bit0 -> 0
//	(20 ms delay; SendCMD(0xAF) — Camera-level)
//	FPGADDRTest                        DDR self-test gate — NOT replicated
//	SetFPGAAsMaster(1)                 reg0 bit5 = 1
//	FPGAStop                           reg0 bit4 = 1
//	EnableFPGADDR(ddrFlag)             reg0xa bit6 = !ddr
//	SetFPGAADCWidthOutputWidth(1, 0)   reg0xa bit0 = 1, bit4 = 0
//	SetFPGABinMode(0)                  reg0x27 low 2 bits = 0
//	SetFPGAGain(0x80,0x80,0x80,0x80)   FPGA 0x0c-0x0f, strobed by reg 1
//
// EnableFPGADDR's argument is the runtime DDR flag set at OpenCamera. The 2600 uses DDR, so
// DDR-enabled (reg0xa bit6 = 0, the polarity hardware-verified on the IMX455/6200 and shared
// by every other profile) is assumed. The DDRTest self-test is not reproduced.
func imx571InitFPGA(rm Regmap, subtype int) error {
	_ = subtype
	if err := FPGAClearBits(rm, 0x00, 0x01); err != nil { // FPGAReset: reg0 bit0
		return err
	}
	time.Sleep(20 * time.Millisecond)                   // usleep(0x4e20)
	if err := FPGASetBits(rm, 0x00, 0x20); err != nil { // SetFPGAAsMaster(1): reg0 bit5
		return err
	}
	if err := FPGASetBits(rm, 0x00, 0x10); err != nil { // FPGAStop: reg0 bit4
		return err
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x40, 0x00); err != nil { // EnableFPGADDR(1): reg0xa bit6 = 0 (DDR enabled)
		return err
	}
	// SetFPGAADCWidthOutputWidth(adc=1, outputWidth): reg0xa bit0 = adc, bit4 = output width.
	// InitCamera passes outputWidth = 0; raise bit4 for RAW16 from the live ReadoutMode (without
	// it the FPGA streams a half-size RAW8 frame).
	adcOut := uint16(0x01) // bit0 = adc
	if ModeOf(rm).BytesPerPx >= 2 {
		adcOut |= 0x10 // bit4 = 1: 16-bit output (RAW16)
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x11, adcOut); err != nil {
		return err
	}
	if err := FPGAWriteBits(rm, 0x27, 0x03, 0x00); err != nil { // SetFPGABinMode(0): reg0x27 low 2 bits = 0
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
	return rm.WriteFPGAReg(0x01, 0)
}

// imx571GainCaps / imx571OffsetCaps return the advertised range per vendor — the dual of the
// dispatched SetGain/SetOffset. ZWO: gain -25..700 (0.1 dB), offset 0..240 def 1. PlayerOne: gain
// 0..550, offset 0..2000 def 20.
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
// value = offset·10 at bin 1, written 16-bit little-endian to sensor 0x42/0x43 and mirrored to
// 0x44/0x45. imx571SetOffset selects the encoding from the regmap's VID —
// PlayerOne offset·8, ZWO offset·10; an unrecognized vendor is an error.
func imx571SetOffset(rm Regmap, offset int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx571SetOffsetZWO(rm, offset)
	case POA.VID:
		return imx571SetOffsetPOA(rm, offset)
	default:
		return fmt.Errorf("astrocam: imx571 offset: unsupported vendor VID 0x%04x", rm.VID())
	}
}

// imx571GetOffset reads the black level back from 0x42/0x43 and undoes the vendor scale (ZWO
// offset·10, PlayerOne offset·8), the dual of SetOffset.
func imx571GetOffset(rm Regmap) (int, error) {
	v, err := ReadRegLE(rm, []uint16{0x42, 0x43})
	if err != nil {
		return 0, err
	}
	switch rm.VID() {
	case ZWO.VID:
		return int(v) / 10, nil
	case POA.VID:
		return int(v) / 8, nil
	default:
		return 0, fmt.Errorf("astrocam: imx571 offset: unsupported vendor VID 0x%04x", rm.VID())
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

// imx571Mode is one readout mode: its sensor register table plus the timing base V (line-time)
// and vblank that InitSensorMode writes to the timing-base/vblank fields.
type imx571Mode struct {
	table     []RegVal
	v, vblank int
	// skipLine/skipColumn feed SetFPGAVBLK / SetFPGAHBLK (the optical-black crop).
	skipLine, skipColumn int
}

// imx571SelectMode maps the binning factor to the readout mode (bin 4 reuses the bin-2 table+V).
// All V/vblank/skip values are the normal (non-strap) constants.
func imx571SelectMode(bin int) imx571Mode {
	switch bin {
	case 2, 4:
		return imx571Mode{imx571ModeBin2w12, imx571VBin2, imx571VBlank2, imx571SkipLine2, imx571SkipColumn2}
	case 3:
		return imx571Mode{imx571ModeBin3w12, imx571VBin3, imx571VBlank3, imx571SkipLine3, imx571SkipColumn3}
	default: // bin 1, 16-bit
		return imx571Mode{imx571ModeFull16, imx571VBin1, imx571VBlank1, imx571SkipLine1, imx571SkipColumn1}
	}
}

// imx571SetGain — SetGain. Clamps to [-25, 700], writes 0x67f (0, or 0x11 in the
// negative segment), then the analog code
//
//	code = 4095 * (1 - 10^(-gain/200))
//
// little-endian to BOTH 0x30/0x31 and 0x32/0x33. The HCG switch is reg 0x2f = 0 (LCG, gain < 100)
// / 1 (HCG, gain >= 100); the analog code RESETS at that boundary (gain re-based to gain-100).
// Above 460 the top band quantises to a 60-step coarse stage = ceil((gain-460)/60), placed in
// 0x40 bits[4:7], and re-bases gain -= 100 + 60·stage.
// imx571SetGain selects the vendor's gain encoding from the regmap's VID (same die, different
// per-vendor band structure); an unrecognized vendor is an error (no implicit default).
func imx571SetGain(rm Regmap, gain int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx571SetGainZWO(rm, gain)
	case POA.VID:
		return imx571SetGainPOA(rm, gain)
	default:
		return fmt.Errorf("astrocam: imx571 gain: unsupported vendor VID 0x%04x", rm.VID())
	}
}

// imx571SetGainPOA is PlayerOne's IMX571 gain encoding, gain-threshold M = 125. Four bands,
// conv-gain register 0x2f, code -> reg 0x30 block. 0x67f routes via CrypWrite in poaRegmap.
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

	var conv, setup, g uint16
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

// imx571SetGainZWO is ZWO's IMX571 gain encoding. Clamp [-25, 700]; the bands:
//
//	gain        seg            0x67f   0x2f   0x40            effGain (code input)
//	-25..-1     negative       0x11    0      0               gain+25
//	0..99       LCG            0       0      0               gain
//	100..460    HCG            0       1      0               gain-100
//	461..700    HCG top band   0       1      (stage<<4)&0xff gain-100-60·stage
//
// where stage = ceil((gain-460)/60). The code = trunc(4095·(1-10^(-effGain/200))) goes to
// 0x30/0x31 and the 0x32/0x33 copy.
func imx571SetGainZWO(rm Regmap, gain int) error {
	if gain > imx571GainMax {
		gain = imx571GainMax
	}
	if gain < imx571GainMin {
		gain = imx571GainMin
	}
	setup := uint16(0)  // 0x67f
	conv := uint16(0)   // 0x2f
	convHi := uint16(0) // 0x40
	effGain := gain
	switch {
	case gain < 0:
		// Negative segment: 0x67f = 0x11, gain shifted up by 25.
		setup = 0x11
		effGain = gain + (-imx571GainMin)
	case gain < imx571GainHCGAt: // 0..99: LCG, code on raw gain
		// conv = 0, convHi = 0, effGain = gain
	case gain <= imx571GainHCGHi: // 100..460: HCG on, code resets
		conv = 1
		effGain = gain - imx571GainHCGAt
	default: // 461..700: HCG top band — coarse 60-step stage in 0x40, re-based code.
		// stage = ceil((gain-460)/60), computed on the full (gain-460).
		stage := (gain - imx571GainHCGHi) / imx571GainStage
		if (gain-imx571GainHCGHi)%imx571GainStage != 0 {
			stage++
		}
		conv = 1
		convHi = (uint16(stage) << 4) & 0xff                     // 0x40 bits[4:7]
		effGain = gain - imx571GainHCGAt - imx571GainStage*stage // gain - 100 - 60·stage
	}

	if err := rm.WriteReg(imx571RegGainSetup, setup); err != nil {
		return err
	}
	// code = K·(1 - 10^(-effGain/200)) with K = 4095.0.
	lin := math.Pow(10, float64(effGain)/-200.0)
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

// imx571SetExposure — SetExp / CalcFrameTime. Indexed full-frame rolling-shutter model, the
// same two-mode structure as the hardware-verified IMX455:
//
//	lines       = exposure / lineTime,  lineTime_ns = V * 1e6 / clock
//	defaultVMAX = vblank + effHeight            (ONE frame; effHeight follows the live ROI)
//	>= 1 s : wait+trigger mode; VMAX = (frameµs+10ms)/line_time + 20 (≈ one frame); SHS = 20.
//	         The integration is HOST-timed by the worker (EnableFPGATriggerSignal), not VMAX.
//	<  1 s : free-run. exposure <= frame-time keeps defaultVMAX and encodes the time in SHS
//	         (VMAX-1-lines, clamped [1, VMAX-1]); a longer one extends VMAX = lines + 20.
//	SHS = SHS >> 1  (the normal-path halve), written little-endian to the indexed regs 0x18/0x19.
//
// VMAX is programmed via the camera FPGA (SetVMAX -> regs 0x10/0x11/0x12); the 16-bit halved SHS
// to the small indexed sensor regs 0x18/0x19. The SHS IS halved (>>1) on the normal path; omitting
// it 2x over-exposes. Holding VMAX at one frame in the trigger band matters: extending it to the
// exposure line count there stretched the frame period ~lines/height× (the bug the 455 profile
// fixed on the wire).
//
// bin is taken from the live ReadoutMode; the hardware-bin SHS branch is not modelled (always the
// free-run normal path).
func imx571SetExposure(rm Regmap, d time.Duration) error {
	bin := ModeOf(rm).Bin
	if bin < 1 {
		bin = 1
	}
	mode := imx571SelectMode(bin)
	// lineTime_ns = V * 1e6 / clock (V from the binned readout mode); bin 1 -> 1350·1e6/20000 = 67500.
	lineNs := uint64(mode.v) * 1_000_000 / imx571ClockHz
	// effHeight = output rows = MaxHeight/bin, except bin 4 = MaxHeight/2 (output rows<<1 reuse of
	// bin-2 geometry). Follow the live ROI height when set, else full/bin.
	effH := int64(imx571FullHeight) / int64(bin)
	if bin == 4 {
		effH = int64(imx571FullHeight) / 2
	}
	if h := ModeOf(rm).Height; h > 0 {
		effH = int64(h)
		if bin == 4 {
			effH *= 2
		}
	}
	defVMAX := int64(mode.vblank) + effH      // vblank + effHeight = one frame
	frameUs := defVMAX * int64(lineNs) / 1000 // frame-readout time in µs
	us := d.Microseconds()                    // exposure clamped to the SetExp range
	if us < imx571ExpMinUs {
		us = imx571ExpMinUs
	}
	if us > imx571ExpMaxUs {
		us = imx571ExpMaxUs
	}
	lines := us * 1000 / int64(lineNs)

	var vmax, shs int64
	if d >= imx571LongExpUs*time.Microsecond {
		// >= 1 s: wait+trigger mode. EnableFPGAWaitMode(1) = reg0 bit6, EnableFPGATriggerMode(1) =
		// bit7. VMAX is held at ONE frame; the worker host-times the integration via the trigger
		// signal (the IMX455 shape: (frame-readout time + 10 ms)/line_time + 20, SHS = 20).
		if err := SetFPGABit(rm, 0x00, 0x40, true); err != nil {
			return err
		}
		if err := SetFPGABit(rm, 0x00, 0x80, true); err != nil {
			return err
		}
		vmax = (frameUs+10000)*1000/int64(lineNs) + 20
		shs = 20
	} else {
		// < 1 s: free-run — clear trigger/wait mode; the sensor self-times via SHS.
		if err := SetFPGABit(rm, 0x00, 0x80, false); err != nil {
			return err
		}
		if err := SetFPGABit(rm, 0x00, 0x40, false); err != nil {
			return err
		}
		if us <= frameUs { // fits one frame: default VMAX, exposure encoded in SHS
			vmax = defVMAX
			shs = vmax + imx571SHSOffset - lines // SHS = VMAX - 1 - lines
			if shs > vmax+imx571SHSOffset {
				shs = vmax + imx571SHSOffset
			}
			if shs < 1 {
				shs = 1
			}
		} else { // extend VMAX to hold the exposure
			vmax = lines + 20
			shs = 20
		}
	}
	if vmax > 0xffffff {
		vmax = 0xffffff
	}
	if err := SetVMAX(rm, uint32(vmax)); err != nil {
		return err
	}
	if shs > 0x1fffe { // 17-bit cap
		shs = 0x1fffe
	}
	// Normal-path SHS halve: low = (SHS>>1)&0xff to 0x18, high = (SHS>>9)&0xff to 0x19.
	// Equivalent to writing (SHS>>1) little-endian since 0x18/0x19 hold the >>1 value's two bytes.
	return WriteRegLE(rm, imx571RegApply, []uint16{imx571RegSHS0, imx571RegSHS1}, uint32(shs>>1))
}

// imx571SetROI — InitSensorMode (per-mode table) + SetStartPos + Cam_SetResolution.
// Applies the bin's readout-mode register table (the second init stage), then the ROI
// start (X aligned to 16 -> 0xa7/0xa8/0xa9, Y+0x19/0x1b -> 0x08/0x09) and the output window
// (HEIGHT 0x0a/0x0b, WIDTH ×4-aligned+0x18 0x1dd/0x1de, window mode 0x1d8 = 4 full / 0 binned).
// Then the optical-black crop SetFPGAVBLK/SetFPGAHBLK (per-mode skip values) and the FPGA frame
// geometry + BinDataLen.
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
	// then the ROI. START is SENSOR pixels (= binned·bin); window mode 0x1d8 = 4 full / 0 binned;
	// HEIGHT(0x0a) takes the raw output height +2 when binned, WIDTH(0x1dd) the ×4-aligned width +0x18.
	mode := imx571SelectMode(bin)
	sx := x * bin
	sy := y * bin
	sx &^= imx571StartXAlign - 1 // align X to 16 (and 0xfffffff0)
	yBias := imx571StartYOff     // 0x19 for bin 1/2/4
	switch bin {
	case 2, 4:
		sy &^= 3 // align Y to 4 for binned (and 0xfffffffc)
	case 3:
		// bin 3: align Y down to a multiple of 6 and use Y bias 0x1b.
		sy = (sy / 6) * 6
		yBias = imx571StartYOff3
	default:
		sy &^= 1 // align Y to 2 (bin 1) (and 0xfffffffe)
	}
	ux := uint16(sx)
	uy := uint16(sy + yBias)

	winMode := uint16(4)
	heightAdj := h
	if bin > 1 {
		winMode = 0       // binned window mode
		heightAdj = h + 2 // +2 for binned
	}
	uh := uint16(heightAdj)       // HEIGHT -> 0x0a/0x0b, raw output height (+2 binned)
	wAligned := (w + 3) &^ 3      // WIDTH rounded up to ×4
	uw := uint16(wAligned + 0x18) // WIDTH -> 0x1dd/0x1de, +0x18

	// The whole mode table + ROI group under the reg 0x07 apply bracket (released even when a
	// write errors — a held apply freezes every later grouped update).
	if err := WithLatch(rm, imx571RegApply, func() error {
		for _, rv := range mode.table {
			if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
				return err
			}
		}
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
		return nil
	}); err != nil {
		return err
	}
	// Optical-black crop: SetFPGAVBLK = FPGA_SKIP_LINE, SetFPGAHBLK = FPGA_SKIP_CLOUMN — the
	// per-mode leading-blank counts the readout windows past. Without these the frame carries the
	// OB margin as a darker band on the left/top.
	if err := SetFPGAVBLK(rm, uint16(mode.skipLine)); err != nil {
		return err
	}
	if err := SetFPGAHBLK(rm, uint16(mode.skipColumn)); err != nil {
		return err
	}
	// FPGA frame geometry (SetFPGAHeight/SetFPGAWidth) = the OUTPUT dims — the FX3 transfer size.
	if err := FPGAWrite16(rm, 0x04, 0x05, uint16(w)); err != nil {
		return err
	}
	if err := FPGAWrite16(rm, 0x08, 0x09, uint16(h)); err != nil {
		return err
	}
	// SetFPGABinDataLen: per-frame DMA word count = output_area·bpp/4 (FPGA 0x40..0x43).
	bpp := ModeOf(rm).BytesPerPx
	return SetFPGABinDataLen(rm, uint32((w*h*bpp+3)/4))
}

// IMX571 per-mode register tables (InitSensorMode). Each entry {reg16,val16}, 53 entries.
//
// Mode -> table:
//
//	bin 1, 16-bit : reg_full_16bit  — the streaming default (imx571Init)
//	bin 2 / bin 4 : reg_bin2w_12bit
//	bin 3         : reg_bin3w_12bit
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
