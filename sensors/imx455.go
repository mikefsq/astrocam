// Sony IMX455: 35 mm full-frame 62 MP BSI CMOS, 9576×6388 (ZWO ASI6200 MM/MC Pro, PlayerOne
// Zeus 455). One die backs the whole family; the worker is variant-agnostic. Wire-verified
// against the SDK on an ASI6200MC over USB3 and USB2 (a real HighSpeed link: single shot,
// arm-per-frame soak, free-run at the link limit; the FPGABufReload ticker runs only on USB3,
// see the worker; the transports pace EP0 reads to 50/s during a USB2 readout, usb2InPace, a
// 500/s request stream then reads clean instead of taking the FX3 down): full-frame 16-bit,
// RAW8 (16-bit readout,
// high byte out, 2.0 fps free-run), high-speed RAW8 (12-bit table, 4.9 fps free-run,
// pixel-matched), the host bins 2/4 at both depths, and the hardware bins 2/3/4 at both depths
// (SetHardwareBin; per-mode V, vblank and OB crop, start mode 0 at bin 3; pixel-correlated
// ≥0.97 with the SDK); the config bytes 0x2d/0x46/sensor 0x40-43 (gain-range/mode table) are
// not. SDK reference values (SetExp): VMAX 0x1928 (6440 default) / 0x19c0 (6592 trigger), SHS1
// 0x14, line 1f:487830, 75.75 µs/line, long-exposure mode at 1 s.
//
// PlayerOne drives the same die: SetGain/SetOffset/GainCaps/OffsetCaps dispatch on the regmap's
// VID (ZWO 0x03C3 vs PlayerOne 0xA0A0). The gain/HCG registers (0x2d, 0x3a4/5/6), the offset
// regs + mirror and the code formula match across vendors; ASI never writes 0x67f on the 455,
// PlayerOne does.
//
// SDK op → profile function:
//
//	InitCamera         imx455InitCommon, imx455InitFPGA
//	InitSensorMode     imx455SelectMode + the mode tables at the end of this file
//	Cam_SetResolution  imx455SetROI (window, mode 0x187, OB crop, FPGA geometry, HMAX = V)
//	SetStartPos        imx455SetROI (ROI start)
//	SetGain            imx455SetGain (imx455SetGainZWO / imx455SetGainPOA)
//	SetExp             imx455SetExposure
//	SetBrightness      imx455SetOffset
//	WorkingFunc        imx455Worker (arm: imx455Arm)

package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
	"math"
	"time"
)

const (
	// SetGain: analog gain code (16-bit) low/high to 0x2e/0x2f, mirrored to 0x30/0x31; 0x3e
	// bits [4:7] carry the high-range coarse stage.
	imx455RegGainCodeL  = 0x2e
	imx455RegGainCodeH  = 0x2f
	imx455RegGainCodeL2 = 0x30
	imx455RegGainCodeH2 = 0x31
	imx455RegGainConv   = 0x3e // high-range coarse-gain stage, bits 4..7
	imx455RegGainMode   = 0x2d // conversion-gain mode byte; bit0 = HCG
	imx455RegGainAux    = 0x4d // auxiliary code byte (per band, 0x08/0x0a/0x0c)
	imx455RegGainCfg0   = 0x3a2
	imx455RegGainCfg1   = 0x3a3
	imx455RegGainCfg2   = 0x3a4
	imx455RegGainCfg3   = 0x3a5
	imx455RegGainCfg4   = 0x3a6

	// SetExp: shutter (SHS), the low two bytes of the 24-bit value (the top byte couples to the
	// FPGA-programmed VMAX).
	imx455RegSHS0 = 0x16
	imx455RegSHS1 = 0x17

	// Master streaming gate: 1 = start (StartSensorStreaming), 5 = stop (StopSensorStreaming);
	// the init reglist seeds it to 1.
	imx455RegMaster = 0x19e

	// SetStartPos: X (aligned to 16) >> 4 as two bytes to 0xa6/0xa7; Y (+0x19, or +0x1b at
	// bin 3) low/high to 0x06/0x07; 0xa5 is the start mode (1, or 0 at bin 3).
	imx455RegStartXL   = 0xa6 // X>>4, bits [0:7]
	imx455RegStartXH   = 0xa7 // X>>4, bits [8:15]
	imx455RegStartYL   = 0x06
	imx455RegStartYH   = 0x07
	imx455RegStartMode = 0xa5

	// Cam_SetResolution: sensor 0x08/0x09 = HEIGHT (e.g. 6388), 0x18c/0x18d = WIDTH+0x18
	// (9576+24 = 9600).
	imx455RegHeightL = 0x08  // window HEIGHT low
	imx455RegHeightH = 0x09  // window HEIGHT high
	imx455RegWidthL  = 0x18c // window WIDTH low  = width+0x18
	imx455RegWidthH  = 0x18d // window WIDTH high
	imx455RegWinMode = 0x187

	imx455GainMax    = 0x2bc      // 700 (0.1 dB units); SetGain clamp
	imx455GainHCGAt  = 0x64       // 100: HCG at/above; the ZWO bands split at 61/100/160/280/461
	imx455ExpMinUs   = 0x20       // 32 µs, SetExp clamp lo
	imx455ExpMaxUs   = 0x77359400 // 2 000 000 000 µs, SetExp clamp hi
	imx455YStartBias = 0x19       // Y row offset added before 0x06/0x07

	// FPGA optical-black crop per readout mode (InitSensorMode's FPGA_SKIP_LINE / the HBLK
	// select): leading blank rows/columns the readout windows past so the OB margin does not
	// reach the host frame. Bin 1 (16-bit and 12-bit): 49 / 24; the bin-2 table: 29 / 16; the
	// bin-3 table: 27 / 16 (24 FPGA units = 12 packed px).
	imx455VBLK1 = 49 // 0x31
	imx455VBLK2 = 29 // 0x1d
	imx455VBLK3 = 27 // 0x1b
	imx455HBLK1 = 24 // 0x18
	imx455HBLK2 = 16 // 0x10, the 12-bit bin tables

	// Timing model. InitSensorMode stores a per-mode base V (tables and mode map at the end of
	// this file):
	//
	//	VMAX (FPGA frame length) = vblank + effHeight   (SetExp; vblank per mode, not V)
	//	HMAX = V                                        (DDR path)
	//	lineTime = HMAX·1000/clock = V·1000/20000 ns
	//
	// V below is the normal value; the strap (FPGA reg 0x1c == 5) halves it and is not used.
	imx455ClkKHz  = 20000 // pixel clock in kHz (0x4e20)
	imx455VFull16 = 0x5eb // 1515: bin1 16-bit (strap 0x370=880)
	imx455VFull12 = 0x276 // 630:  bin1 12-bit = high-speed mode (strap 0x201=513)
	imx455VBin2   = 0x271 // 625:  bin2 (strap 0x16d=365)
	imx455VBin3   = 0x14a // 330:  bin3        (strap 0xe0=224)

	imx455ExpTrigUs = 1_000_000 // >= 1 s: FPGA wait+trigger mode, integration host-timed
	// vblank per mode (InitSensorMode): the default-VMAX base (defaultVMAX = vblank + height).
	// Bin 1: 52 (wire-verified); bin-2 table: 32; bin-3 table: 30.
	imx455VBlank1    = 52   // 0x34
	imx455VBlank2    = 32   // 0x20
	imx455VBlank3    = 30   // 0x1e
	imx455FullWidth  = 9576 // output width at full-frame bin 1
	imx455FullHeight = 6388 // output rows at full-frame bin 1
)

// imx455Mode is the readout-mode profile InitSensorMode applies: the per-mode register table,
// the timing base V (HMAX and the line time), the default-VMAX vblank, and the FPGA OB crop
// (SetFPGAVBLK / SetFPGAHBLK).
type imx455Mode struct {
	table      []RegVal
	v, vblank  int
	vblk, hblk int
}

// imx455SelectMode maps the sensor binning factor + the high-speed flag to the readout mode
// (InitSensorMode bin/depth branches). At bin 1 the 12-bit table is the
// SDK's ASI_HIGH_SPEED_MODE, not its RAW8: SDK RAW8 keeps the 16-bit readout (V=1515, 2.0 fps
// full frame) and lets the FPGA output the high byte, while high-speed switches to the 12-bit
// table (V=630, 4.9 fps); selecting the 12-bit table on output depth alone gives unreadable
// RAW8 frames on the ASI6200MC.
func imx455SelectMode(bin int, highSpeed bool) imx455Mode {
	switch bin {
	case 2:
		return imx455Mode{imx455ModeBin2w12, imx455VBin2, imx455VBlank2, imx455VBLK2, imx455HBLK2}
	case 3:
		return imx455Mode{imx455ModeBin3w12, imx455VBin3, imx455VBlank3, imx455VBLK3, imx455HBLK2}
	default: // bin 1
		if highSpeed {
			return imx455Mode{imx455ModeFull12, imx455VFull12, imx455VBlank1, imx455VBLK1, imx455HBLK1}
		}
		return imx455Mode{imx455ModeFull16, imx455VFull16, imx455VBlank1, imx455VBLK1, imx455HBLK1}
	}
}

// imx455InitCommon is the mode-independent first stage of InitCamera: the 36-entry reglist_init
// table (reg 0xffff = InitDelayReg, delay = val ms) then the explicit WriteSONYREG tail. The
// per-mode second stage is the mode tables at the end of this file; FPGA bringup is in
// imx455InitFPGA. The SDK's SendCMD(0xAF/0xAE) around this point and its cooling init are not
// replayed by Camera.Init (0xAF is sent once as the up-front quiesce).
var imx455InitCommon = []RegVal{
	// --- reglist_init: reg/val16 pairs ---
	{Reg: 0x019e, Val: 0x01}, {Reg: 0x0000, Val: 0x04}, {Reg: 0xffff, Val: 10}, // delay 10 ms
	{Reg: 0x0002, Val: 0x10}, {Reg: 0x0025, Val: 0x0a}, {Reg: 0x0046, Val: 0x03}, {Reg: 0x004f, Val: 0x08},
	{Reg: 0x00c6, Val: 0x08}, {Reg: 0x00da, Val: 0x31}, {Reg: 0x01a0, Val: 0x06}, {Reg: 0x031a, Val: 0x01},
	{Reg: 0x031b, Val: 0x0e}, {Reg: 0x03a0, Val: 0x0f}, {Reg: 0x03a2, Val: 0x07}, {Reg: 0x03a3, Val: 0x11},
	{Reg: 0x03a4, Val: 0x11}, {Reg: 0x03a5, Val: 0x11}, {Reg: 0x03a6, Val: 0x11}, {Reg: 0x04bf, Val: 0x01},
	{Reg: 0x04c3, Val: 0x01}, {Reg: 0x04cb, Val: 0x02}, {Reg: 0x0573, Val: 0x00}, {Reg: 0x0586, Val: 0x10},
	{Reg: 0x0587, Val: 0x10}, {Reg: 0x0588, Val: 0x10}, {Reg: 0x0589, Val: 0x10}, {Reg: 0x067e, Val: 0x06},
	{Reg: 0x06a2, Val: 0x03}, {Reg: 0x07d0, Val: 0x06}, {Reg: 0x07d1, Val: 0x0b}, {Reg: 0x07d3, Val: 0x06},
	{Reg: 0x07d4, Val: 0x0b}, {Reg: 0x07d6, Val: 0x06}, {Reg: 0x0113, Val: 0x00}, {Reg: 0x0120, Val: 0xbc},
	{Reg: 0x0121, Val: 0x01},
	// --- explicit WriteSONYREG tail ---
	{Reg: 0x0002, Val: 0x10},
	{Reg: 0x0005, Val: 0x01},
	{Reg: 0x00a5, Val: 0x01},
	{Reg: 0x0187, Val: 0x04},
	{Reg: 0x0046, Val: 0x0f},
	{Reg: 0x004f, Val: 0x08},
}

// imx455Init is the streaming default: the common init followed by the bin-1 16-bit per-mode
// table (InitCamera then InitSensorMode). The SDK applies the per-mode table after
// imx455InitFPGA; here it is in Init, which runs before InitFPGA. The mode tables are
// FPGA-independent Sony registers, so the order does not matter.
var imx455Init = append(append([]RegVal{}, imx455InitCommon...), imx455ModeFull16...)

// IMX455 is the Sony IMX455 full-frame profile (ZWO ASI6200 MM/MC Pro, PlayerOne Zeus 455).
var IMX455 = Sensor{
	Name:     "IMX455", // Sony IMX455 full-frame 62MP
	GainMax:  imx455GainMax,
	ExpMinUs: imx455ExpMinUs,
	ExpMaxUs: imx455ExpMaxUs,
	// ASI Brightness / black level: 0..200, default 50 (ASIInitCamera applies ~502 DN, offset 50;
	// offset→DN is ≈10·offset+2).
	OffsetMax: 200, OffsetDef: 50,
	Info: CameraInfo{
		MaxWidth:  9576,
		MaxHeight: 6388,
		PixelUm:   3.76,
		BitDepth:  16,
		Bayer:     "RGGB",            // MC = color
		Bins:      []int{1, 2, 3, 4}, // host-binned by default (SDK default); hardware 2/3 (4 = 2×2) via SetHardwareBin
	},
	Init:        imx455Init,
	InitFPGA:    imx455InitFPGA,
	SetGain:     imx455SetGain,
	GainCaps:    imx455GainCaps,
	SetExposure: imx455SetExposure,
	SetOffset:   imx455SetOffset,
	GetOffset:   imx455GetOffset,
	OffsetCaps:  imx455OffsetCaps,
	SetROI:      imx455SetROI,
	// Master/stream gate (0073 MM / 0074 MC objects agree): 0x19e = 1 start, 5 stop; the objects
	// never write 0. StopSensorStreaming = 0x19e←5 + CamSetStandby(1) (reg0 bit0);
	// StartSensorStreaming = 0x19e←1 + CamSetWakeup(1) (reg0 bit2) + 10 ms + CamSetStandby(0).
	// Used by StopExposure and the StartVideo arm; imx455Arm has the same sequence inline.
	StreamStop: func(rm Regmap) error {
		if err := rm.WriteReg(imx455RegMaster, 5); err != nil { // 0x19e = 5 (stop)
			return err
		}
		return rm.WriteRegBits(0, 0, 0, 1) // CamSetStandby(1): sensor reg0 bit0 = 1
	},
	StreamStart: func(rm Regmap) error {
		if err := rm.WriteReg(imx455RegMaster, 1); err != nil { // 0x19e = 1 (start)
			return err
		}
		if err := rm.WriteRegBits(0, 2, 2, 1); err != nil { // CamSetWakeup(1): reg0 bit2 = 1
			return err
		}
		time.Sleep(10 * time.Millisecond)  // usleep(0x2710)
		return rm.WriteRegBits(0, 0, 0, 0) // CamSetStandby(0): reg0 bit0 = 0
	},
	Arm:    imx455Arm,    // shared by the worker and StartVideo / free-run streaming
	Worker: imx455Worker, // arm + windowed stream read

	// FX3 DDR frame markers: the readout brackets every frame with fixed DMA words (first 32-bit
	// word 0x00005A7E, last 0x3CF00000), so RAW16 pixels 0,1 and N-2,N-1 are not sensor data;
	// repairFX3DMAMarkers (capture.go) replaces them from two rows away, as the SDK does. HW-confirmed.
	FX3DMAMarkers: true,
	// Hardware readout modes: the bin-2 and bin-3 12-bit tables (InitSensorMode); the SDK's
	// bin 4 is the bin-2 table over 2w×2h with the host binning 2× more, which the Camera
	// derives from this list.
	HWBins: []int{2, 3},
	// SetStartPos masks: X to 16; Y to 2 at bin 1, 4 at bin 2, 6 at bin 3.
	ROIStartAlign: func(bin int) (int, int) {
		switch bin {
		case 2:
			return 16, 4
		case 3:
			return 16, 6
		}
		return 16, 2
	},
}

// imx455Worker is the host-timed single-shot capture worker. The sensor gate is the 0x19e master
// register (1 = start, 5 = stop) with a 10 ms settle. At >= 1 s SetExp arms trigger mode
// (EnableFPGATriggerMode); the worker drives the trigger signal (EnableFPGATriggerSignal, FPGA
// reg 0x0b bit0), whose 1->0 edge releases the frame. The very-long-band multi-exposure
// accumulation cycle (FPGAStop/5 ms/FPGAStart/20 ms gated on ReadFPGAREG(0x23)) is not
// reproduced: single-shot only.
//
//	arm:    imx455Arm
//	expose: >= 1 s only: EnableFPGATriggerSignal(1)·hold the exposure·(0)
//	read:   continuous windowed pump (ctl.StreamFrame) of W×H×2 bytes with a 20 ms
//	        FPGABufReload ticker; one call, whatever it returns (no stall re-kick, and this
//	        worker never calls NoteStall, so StallCount stays 0 for the DDR profiles)
//	stop:   FPGAStop·0x19e=5·CamSetStandby(1)·SendCMD(0xAA)·ResetEndPoint (WorkingFunc exit)
func imx455Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()

	// Sensor reg-0 read-modify-write (ReadSONYREG 0 → mask → WriteSONYREG 0).
	regRMW := func(set, clr uint16) error {
		v, err := rm.ReadReg(0)
		if err != nil {
			return err
		}
		return rm.WriteReg(0, (v|set)&^clr)
	}
	fpgaStop := func() error { return SetFPGABit(rm, 0x00, 0x10, true) } // reg0 bit4 = 1
	// FPGABufReload: FPGA reg 0x18 |= 1. The FX3 commits whole 1-MiB DMA buffers and holds the
	// final partial buffer of a frame; this commits it so the frame's tail flushes to the host.
	bufReload := func() error { return SetFPGABit(rm, 0x18, 0x01, true) }
	// EnableFPGATriggerSignal: FPGA reg 0x0b bit0. In wait+trigger mode (>= 1 s, set by
	// SetExposure) the host holds this for the integration time.
	triggerSignal := func(on bool) error { return SetFPGABit(rm, 0x0b, 0x01, on) }

	// Halt the readout on every return, the arm's own failures included, as WorkingFunc's
	// exit does (0074_CCameraS6200MC_Pro.o):
	// StopSensorStreaming (FPGAStop, 0x19e←5, CamSetStandby(1)), SendCMD(0xAA), ResetEndPoint.
	// Best-effort: a failed stop must not fail a good frame.
	defer func() {
		_ = fpgaStop()
		_ = rm.WriteReg(imx455RegMaster, 5) // master stop
		_ = regRMW(0x01, 0)                 // CamSetStandby(1): sensor reg0 bit0 = 1
		_ = ctl.VendorCmd(FX3StreamStop)
		_ = ctl.ResetEndpoint()
	}()
	// --- arm (imx455Arm) ---
	// Empty the pipe before arming, not after. The previous capture's deferred halt races the
	// FX3, which keeps committing DMA buffers until it stops, and ResetEndpoint does not discard
	// them. Anything left heads this frame and shifts every pixel by that many bytes.
	if n := ctl.DrainPipe(250 * time.Millisecond); n > 0 {
		ctl.NoteStall()
	}
	if err := imx455Arm(ctl); err != nil {
		return 0, err
	}
	_ = ctl.ResetEndpoint() // ResetEndPoint(0x81)

	// >= 1 s runs in FPGA wait+trigger mode (SetExposure set reg0 bit6/bit7 and held VMAX at
	// one frame). The integration is host-timed: assert the trigger signal, hold for the
	// exposure, release so the frame clocks out. Below 1 s is free-run: the sensor self-times
	// via SHS and the read blocks on the frame.
	if exposure >= imx455ExpTrigUs*time.Microsecond {
		if err := triggerSignal(true); err != nil {
			return 0, err
		}
		for start := time.Now(); time.Since(start) < exposure; {
			if ctl.Aborted() {
				// StopExposure ran: drop the trigger signal on the way out. Left asserted,
				// the next triggerSignal(true) is a no-edge write and the FPGA never gates
				// the integration. (The SDK never host-aborts mid-integration.)
				_ = triggerSignal(false)
				return 0, errExposureAborted
			}
			time.Sleep(100 * time.Millisecond)
		}
		if err := triggerSignal(false); err != nil {
			return 0, err
		}
	}

	// --- read one whole frame (continuous windowed pump, FrameBytes) ---
	target := ctl.FrameBytes()
	if target > len(buf) {
		target = len(buf)
	}
	// idle must cover the wait for the first chunk (first data can take ~one frame period);
	// once flowing, chunks are ms apart. In the >= 1 s trigger band the integration completed
	// above, so the read spans only the readout and the timeouts are not exposure-scaled
	// (otherwise a stalled readout after a long sub blocks StopExposure for up to 2·exp+5 s).
	idle := exposure + 2*time.Second
	total := 2*exposure + 5*time.Second
	if exposure >= imx455ExpTrigUs*time.Microsecond {
		idle = 2 * time.Second
		total = 15 * time.Second // full-frame readout ceiling incl. USB2 + retries
	}
	// On USB3 the FX3 commits whole 1-MiB DMA buffers and holds the frame's final partial buffer
	// until it is filled or committed; pulse FPGABufReload throughout the read so partial commits
	// and the frame's tail flush into a posted transfer. The windowed reader treats the frame-end
	// ZLP as non-terminal and keeps cycling until the whole frame lands. On a USB2 link the tail
	// arrives on its own, and the pulses wedge the readout (ASI6200MC on USB2: a 20 ms ticker
	// stalls the read by the 7th back-to-back frame; without it 30 frames pass, even with 50 Hz
	// EP0 temperature reads interleaved), so the ticker runs only on USB3.
	stop := make(chan struct{})
	done := make(chan struct{})
	go func() {
		defer close(done)
		if !ModeOf(rm).USB3 {
			<-stop
			return
		}
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
	<-done // JOIN: wait for the ticker's last bufReload to finish before returning, so no
	// control transfer outlives the worker to race the next operation / the TEC loop.
	if err == nil && n < target && ctl.Aborted() {
		return n, errExposureAborted // AbortRead: clean abort, not a stall
	}
	return n, err
}

// imx455Arm is the WorkingFunc stream arm, shared by the snap worker and StartVideo (Sensor.Arm):
// SendCMD(0xAA) · FPGAStop · 0x19e=5 · reg0|=1 · SendCMD(0xA9) · FPGAStop · 0x19e=1 · reg0|=4 ·
// 10 ms · reg0&=~1 · FPGAStart. The second FPGAStop after SendCMD(0xA9) is required: without it
// the DDR readout delivers no free-run frame (with it, frames stream at the sensor period,
// 487.8 ms full-frame 16-bit).
func imx455Arm(ctl WorkerCtl) error {
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
	if err := fpgaStop(); err != nil {
		return err
	}
	if err := rm.WriteReg(imx455RegMaster, 5); err != nil { // master stop
		return err
	}
	if err := regRMW(0x01, 0); err != nil { // CamSetStandby(1): sensor reg0 |= 1
		return err
	}
	if err := ctl.VendorCmd(FX3StreamStart); err != nil { // FX3 stream start
		return err
	}
	if err := fpgaStop(); err != nil { // the second FPGAStop the DDR pipeline needs
		return err
	}
	if err := rm.WriteReg(imx455RegMaster, 1); err != nil { // master = 1
		return err
	}
	if err := regRMW(0x04, 0); err != nil { // CamSetWakeup(1): sensor reg0 |= 4
		return err
	}
	time.Sleep(10 * time.Millisecond)       // usleep(0x2710)
	if err := regRMW(0, 0x01); err != nil { // CamSetStandby(0): sensor reg0 &= ~1
		return err
	}
	return fpgaStart()
}

// imx455InitFPGA is the FPGA bringup InitCamera performs after the Sony init tail, using the
// FX3 register numbers:
//
//	FPGAReset                          reg0 bit0 -> 0
//	(20 ms delay; the SDK's SendCMD(0xAF) here is not replayed)
//	FPGADDRTest                        DDR self-test gate, not replicated
//	SetFPGAAsMaster(1)                 reg0 bit5 = 1
//	FPGAStop                           reg0 bit4 = 1
//	EnableFPGADDR(ddrFlag)             reg0xa bit6 = !ddr; the 6200 uses DDR, so bit6 = 0
//	SetFPGAADCWidthOutputWidth(1, 0)   reg0xa bit0 = 1, bit4 = 0
//	SetFPGABinMode(0)                  reg0x27 low 2 bits = 0
//	SetFPGAGain(0x80,0x80,0x80,0x80)   FPGA 0x0c-0x0f, strobed by reg 1
func imx455InitFPGA(rm Regmap, subtype int) error {
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
	if err := FPGAWriteBits(rm, 0x0a, 0x40, 0x00); err != nil { // EnableFPGADDR(1): bit6 = 0 (DDR)
		return err
	}
	// SetFPGAADCWidthOutputWidth(adc=1, outputWidth): reg0xa bit0 = adc, bit4 = output width.
	// InitCamera passes outputWidth = 0 (8-bit); bit4 = 1 for RAW16 from the live ReadoutMode,
	// otherwise the FPGA streams a half-size RAW8 frame while the host expects RAW16.
	adcOut := uint16(0x01) // bit0 = adc
	if ModeOf(rm).BytesPerPx >= 2 {
		adcOut |= 0x10 // bit4 = 1: 16-bit output (RAW16)
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x11, adcOut); err != nil {
		return err
	}
	if err := FPGAWriteBits(rm, 0x27, 0x03, 0x00); err != nil { // SetFPGABinMode(0)
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

// imx455GainCaps / imx455OffsetCaps return the advertised gain/offset range per vendor, the
// dual of the dispatched SetGain/SetOffset. ZWO: gain 0..700 (0.1 dB), offset 0..200 def 50.
// PlayerOne: gain 0..550, offset 0..2000 def 20. max 0 = undeclared.
func imx455GainCaps(vid uint16) (min, max int) {
	switch vid {
	case ZWO.VID:
		return 0, imx455GainMax // 0..700
	case POA.VID:
		return 0, 550
	default:
		return 0, 0
	}
}

func imx455OffsetCaps(vid uint16) (min, max, def int) {
	switch vid {
	case ZWO.VID:
		return 0, 200, 50
	case POA.VID:
		return 0, 2000, 20
	default:
		return 0, 0, 0
	}
}

// imx455SetGain selects the vendor's gain encoding from the regmap's VID (each vendor drives the
// shared die with its own band structure); an unrecognized vendor is an error.
func imx455SetGain(rm Regmap, gain int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx455SetGainZWO(rm, gain)
	case POA.VID:
		return imx455SetGainPOA(rm, gain)
	default:
		return fmt.Errorf("astrocam: imx455 gain: unsupported vendor VID 0x%04x", rm.VID())
	}
}

// imx455SetGainPOA is PlayerOne's IMX455 gain encoding, threshold M = 125. Each conversion-gain
// band rebases the gain into the shared analog-code formula; the gain-setup register 0x67f is
// written per band (poaRegmap routes 0x67f via CrypWrite, so this calls WriteReg).
//
//	gain      rebased g   0x2d  0x67f   0x3a4 / 0x3a5,0x3a6
//	0..4      g+30        0     0x22    0x11 / 0x11
//	5..29     g-5         0     0x11    0x11 / 0x11
//	30..89    g-30        0     0       0x11 / 0x11
//	90..124   g-30        4     0       0x11 / 0x11
//	125..184  g-125       1     0       0x11 / 0x11
//	185..304  g-125       5     0       0x11 / 0x11
//	305+      g-125       5     0       0x23 / 0x2d
//	code = trunc(4095·(1-10^(-g/200))) clamped to 4095 -> [lo,hi,lo,hi] block 0x2e/0x2f/0x30/0x31.
//
// The advertised gain max lives in the POAConfigAttributes builder (not decoded), so only the
// low end is clamped here; the analog code saturates at 4095.
func imx455SetGainPOA(rm Regmap, gain int) error {
	if gain < 0 {
		gain = 0
	}
	const m = 125 // gain threshold M

	var mode, setup, g uint16
	cfgA, cfgBC := uint16(0x11), uint16(0x11)
	switch {
	case gain <= 4:
		setup, g = 0x22, uint16(gain+30)
	case gain <= 29:
		setup, g = 0x11, uint16(gain-5)
	case gain < m: // bands 30..124: 0x67f=0, rebase -30, 0x2d switches at 90
		g = uint16(gain - 30)
		if gain >= 90 {
			mode = 4
		}
	default: // gain >= 125: 0x67f=0, rebase -M, 0x2d 1->5 at 185, high cfg at 305
		g = uint16(gain - m)
		if gain < 185 {
			mode = 1
		} else {
			mode = 5
		}
		if gain >= 305 {
			cfgA, cfgBC = 0x23, 0x2d
		}
	}

	code := int(4095.0 * (1.0 - math.Pow(10, float64(g)/-200.0)))
	if code > 4095 {
		code = 4095
	}
	codeU := uint16(code)

	writes := make([]RegVal, 0, 9)
	if gain <= 29 { // bands A/B write 0x2d before the 0x67f setup
		writes = append(writes, RegVal{Reg: 0x2d, Val: mode}, RegVal{Reg: 0x67f, Val: setup})
	} else { // bands C/D write the 0x67f setup first
		writes = append(writes, RegVal{Reg: 0x67f, Val: setup}, RegVal{Reg: 0x2d, Val: mode})
	}
	writes = append(writes,
		RegVal{Reg: 0x3a4, Val: cfgA},
		RegVal{Reg: 0x3a5, Val: cfgBC},
		RegVal{Reg: 0x3a6, Val: cfgBC},
		RegVal{Reg: 0x2e, Val: codeU & 0xff}, RegVal{Reg: 0x2f, Val: (codeU >> 8) & 0xff},
		RegVal{Reg: 0x30, Val: codeU & 0xff}, RegVal{Reg: 0x31, Val: (codeU >> 8) & 0xff},
	)
	for _, rv := range writes {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return nil
}

// imx455SetGainZWO is ZWO's IMX455 gain encoding (SetGain). The gain axis splits into five bands
// at 61/100/160/280/461:
//
//   - reg 0x2d is the conversion-gain mode byte; its bit0 is the HCG enable: 0 below gain 100
//     (LCG), 1 at/above 100 (HCG).
//   - The analog code resets at the HCG boundary: the exp10 ramp uses gain below 100 and
//     gain−100 at/above; the top band also subtracts a coarse 60-unit stage.
//   - reg 0x3e is not a binary HCG bit; it is the high-range coarse-gain stage index (0 for
//     gain <= 460; ceil(((gain+52)&0xff)/60) for 461..700), placed in bits[4:7].
func imx455SetGainZWO(rm Regmap, gain int) error {
	if gain > imx455GainMax {
		gain = imx455GainMax
	}
	if gain < 0 {
		gain = 0
	}

	var (
		mode    uint16 // 0x2d conversion-gain mode (bit0 = HCG)
		aux     uint16 // 0x4d
		cfgA    uint16 // 0x3a4
		cfgB    uint16 // 0x3a5 + 0x3a6
		stage   uint16 // 0x3e high-range coarse stage
		effGain int    // gain fed to the analog exp10 (resets across HCG)
	)
	switch {
	case gain < imx455GainHCGAt: // band A (0..99): LCG, code on the raw gain
		if gain > 60 {
			mode = 4
		}
		aux = 0xa
		if gain <= 60 {
			aux = 8
		}
		cfgA, cfgB = 0x11, 0x11
		effGain = gain
	case gain < 160: // band B (100..159): HCG on, code resets
		mode, aux, cfgA, cfgB = 1, 8, 0x11, 0x11
		effGain = gain - 100
	case gain < 280: // band C (160..279): HCG
		mode, aux, cfgA, cfgB = 5, 0xa, 0x11, 0x11
		effGain = gain - 100
	case gain < 461: // band D (280..460): HCG, high config
		mode, aux, cfgA, cfgB = 5, 0xc, 0x23, 0x2d
		effGain = gain - 100
	default: // band E (461..700): coarse 60-unit stage + residual code
		stage = imx455GainStage(gain)
		mode, aux, cfgA, cfgB = 5, 0xc, 0x23, 0x2d
		effGain = gain - 60*int(stage) - 100
	}

	// Analog code: code = trunc(4095·(1 − 10^(−effGain/200))) with K = 4095.0.
	g := math.Pow(10, float64(effGain)/-200.0)
	code := int32(4095.0 * (1.0 - g))
	codeU := uint16(code)

	writes := []RegVal{
		{Reg: imx455RegGainMode, Val: mode}, // 0x2d: HCG mode
		{Reg: imx455RegGainAux, Val: aux},   // 0x4d
		{Reg: imx455RegGainCfg0, Val: 7},    // 0x3a2 (const)
		{Reg: imx455RegGainCfg1, Val: 0x11}, // 0x3a3 (const)
		{Reg: imx455RegGainCfg2, Val: cfgA}, // 0x3a4
		{Reg: imx455RegGainCfg3, Val: cfgB}, // 0x3a5
		{Reg: imx455RegGainCfg4, Val: cfgB}, // 0x3a6
		{Reg: imx455RegGainCodeL, Val: codeU & 0xff},
		{Reg: imx455RegGainCodeH, Val: (codeU >> 8) & 0xff},
		{Reg: imx455RegGainCodeL2, Val: codeU & 0xff},
		{Reg: imx455RegGainCodeH2, Val: (codeU >> 8) & 0xff},
		{Reg: imx455RegGainConv, Val: (stage & 0xf) << 4}, // 0x3e: high-range stage
	}
	for _, rv := range writes {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return nil
}

// imx455GainStage is the coarse 60-unit gain-stage index for the top band (gain >= 461) =
// ceil(((gain+52)&0xff)/60), keeping the byte-wrapped quotient and full-width remainder check.
func imx455GainStage(gain int) uint16 {
	v := (gain + 52) & 0xff
	q := (v * 137) >> 13      // floor(v/60) via the 137/8192 reciprocal
	rem := (gain + 52) - q*60 // remainder on the FULL gain+52
	if rem&0xff != 0 {
		q++
	}
	return uint16(q)
}

// imx455SetOffset (SetBrightness) selects the vendor encoding from the regmap's VID: ZWO
// offset·10 at bin 1 (offset·100/16 binned), PlayerOne offset·8; 16-bit LE to sensor 0x40/0x41
// mirrored to 0x42/0x43. An unrecognized vendor is an error.
func imx455SetOffset(rm Regmap, offset int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx455SetOffsetZWO(rm, offset)
	case POA.VID:
		return imx455SetOffsetPOA(rm, offset)
	default:
		return fmt.Errorf("astrocam: imx455 offset: unsupported vendor VID 0x%04x", rm.VID())
	}
}

// imx455GetOffset reads the black level back from 0x40/0x41 and undoes the vendor scale
// (ZWO offset·10 at bin 1, offset·100/16 binned; PlayerOne offset·8), the dual of SetOffset.
// The binned scale floors on write (50 → 312), so the read-back rounds (312·16 = 4992 → 50);
// v·16 lies in (offset·100−16, offset·100], so +50 before the divide restores every offset.
func imx455GetOffset(rm Regmap) (int, error) {
	v, err := ReadRegLE(rm, []uint16{0x40, 0x41})
	if err != nil {
		return 0, err
	}
	switch rm.VID() {
	case ZWO.VID:
		if ModeOf(rm).Bin > 1 {
			return (int(v)*16 + 50) / 100, nil
		}
		return int(v) / 10, nil
	case POA.VID:
		return int(v) / 8, nil
	default:
		return 0, fmt.Errorf("astrocam: imx455 offset: unsupported vendor VID 0x%04x", rm.VID())
	}
}

// imx455SetOffsetPOA is PlayerOne's IMX455 black level: offset·8, same 0x40/0x41 mirror
// 0x42/0x43 block.
func imx455SetOffsetPOA(rm Regmap, offset int) error {
	v := uint16(offset * 8)
	for _, rv := range []RegVal{
		{Reg: 0x40, Val: v & 0xff}, {Reg: 0x41, Val: (v >> 8) & 0xff},
		{Reg: 0x42, Val: v & 0xff}, {Reg: 0x43, Val: (v >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return nil
}

func imx455SetOffsetZWO(rm Regmap, offset int) error {
	v := uint16(offset * 10)
	if ModeOf(rm).Bin > 1 {
		v = uint16(offset * 100 / 16) // binned; offset>=0 so floor
	}
	for _, rv := range []RegVal{
		{Reg: 0x40, Val: v & 0xff}, {Reg: 0x41, Val: (v >> 8) & 0xff},
		{Reg: 0x42, Val: v & 0xff}, {Reg: 0x43, Val: (v >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return nil
}

// imx455SetExposure (SetExp). Indexed full-frame rolling-shutter model:
//
//	lines       = exposure / lineTime,  lineTime_ns = V*1000/clock
//	defaultVMAX = vblank(52) + effHeight            (one frame; effHeight follows the live ROI)
//	>= 1 s : wait+trigger mode; VMAX = (frameµs+10ms)/line_time + 20 (≈ one frame); SHS = 20.
//	         The worker host-times the integration (EnableFPGATriggerSignal), not VMAX.
//	<  1 s : free-run. exposure <= frame-time keeps defaultVMAX and encodes the time in SHS
//	         ((VMAX-3)-lines, clamped [3, VMAX-3]); a longer one extends VMAX = lines + 20.
//	SHS is halved (>>1, floor 3) for the bin-1/bin-3 readout, skipped for bin 2/4.
//
// VMAX (FPGA frame length) goes via SetVMAX (regs 0x10/0x11/0x12); the SHS low two bytes to the
// indexed sensor regs 0x16/0x17 (no latch). Wire-checked: 2 s → VMAX 0x19c0=6592, SHS 10, reg0
// 0xf1; full-frame trigger VMAX 6592, 2252 for a 2048-row ROI. The SDK's long-exposure
// VMAX-stretch branch (VMAX = lines + 0x14, residual SHS 0x14) is not modelled.
func imx455SetExposure(rm Regmap, d time.Duration) error {
	bin := int64(ModeOf(rm).Bin)
	if bin < 1 {
		bin = 1
	}
	// Line-time base V per readout mode (InitSensorMode), so the binned line time follows the
	// binned V.
	mode := imx455SelectMode(int(bin), ModeOf(rm).HighSpeed)
	v := int64(mode.v)                            // HMAX line-time base
	lineNs := v * 1_000_000 / int64(imx455ClkKHz) // line time in ns (75750)

	us := d.Microseconds()
	if us < imx455ExpMinUs {
		us = imx455ExpMinUs
	}
	if us > imx455ExpMaxUs {
		us = imx455ExpMaxUs
	}

	// effHeight = the sensor-side readout rows (the live ROI height set by SetROI, else full/bin);
	// VMAX = the mode's vblank + effHeight.
	effH := int64(imx455FullHeight) / bin
	if h := ModeOf(rm).Height; h > 0 {
		effH = int64(h)
	}
	defVMAX := int64(mode.vblank) + effH // one frame (6440 at full bin 1)
	frameUs := defVMAX * lineNs / 1000   // frame-readout time (487830 µs)

	var vmax, shs int64
	if us >= imx455ExpTrigUs {
		// >= 1 s: hold VMAX at one frame; the worker host-times integration via the
		// trigger signal. EnableFPGAWaitMode(1)=reg0 bit6, EnableFPGATriggerMode(1)=bit7.
		if err := SetFPGABit(rm, 0x00, 0x40, true); err != nil {
			return err
		}
		if err := SetFPGABit(rm, 0x00, 0x80, true); err != nil {
			return err
		}
		vmax = (frameUs+10000)*1000/lineNs + 20 // (frame-readout time+10ms)/line_time + 20 ≈ 6592
		shs = 20
	} else {
		if err := SetFPGABit(rm, 0x00, 0x80, false); err != nil { // trigger mode off
			return err
		}
		if err := SetFPGABit(rm, 0x00, 0x40, false); err != nil { // wait mode off
			return err
		}
		lines := us * 1000 / lineNs
		if us <= frameUs { // fits one frame: default VMAX, exposure encoded in SHS
			vmax = defVMAX
			shs = (defVMAX - 3) - lines
			if shs > defVMAX-3 {
				shs = defVMAX - 3
			}
			if shs < 3 {
				shs = 3
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
	// SHS readout halving: halve for bin 1 and bin 3, skip for bin 2 (the bin-mode field==0
	// falls into the halve, bin 2 branches around it, bin 3 falls through). When halving:
	// shs = (shs < 6) ? 3 : shs>>1.
	if bin != 2 {
		if shs >= 6 {
			shs >>= 1
		} else {
			shs = 3
		}
	}
	return WriteRegLE(rm, 0, []uint16{imx455RegSHS0, imx455RegSHS1}, uint32(shs))
}

// imx455SetROI: InitSensorMode (per-mode table by binning + high-speed flag), SetStartPos,
// Cam_SetResolution (window + mode 0x187), the FPGA OB crop, the FPGA frame geometry,
// SetFPGABinDataLen and the mode's V as FPGA HMAX (DDR path, regs 0x13/0x14).
func imx455SetROI(rm Regmap, x, y, w, h, bin int) error {
	if bin < 1 {
		bin = 1
	}
	if bin > 3 {
		return fmt.Errorf("imx455: sensor bin %d has no readout mode (hardware modes are 2 and 3; the Camera host-bins the rest)", bin)
	}
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	// (x,y,w,h) is the sensor-side binned window (bin ∈ {1,2,3}; the SDK's bin 4 is bin 2 over
	// 2w×2h, host-binned 2× by the Camera): w,h go straight to the window/FPGA-geometry regs
	// (Cam_SetResolution: WIDTH = w+0x18, HEIGHT = h); the start is in sensor pixels (binned x,y
	// scaled by bin). Start X aligns to 16; start Y aligns to 4 for bin 2, to 2 for bin 1, to 6
	// for bin 3 (with the 0x1b bias).
	mode := imx455SelectMode(bin, ModeOf(rm).HighSpeed)
	for _, rv := range mode.table {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	// SetOutput16Bits / InitSensorMode: FPGA reg 0xa bit0 (ADC_BIT) follows the table, 1 for the
	// 16-bit readout and 0 for every 12-bit table (high-speed, hardware bin); bit4 is the output
	// width (1 = RAW16). ADC_BIT left at 1 on a 12-bit table delivers unreadable frames
	// (ASI6200MC, wire-checked against the SDK's high-speed RAW8).
	adcOut := uint16(0)
	if mode.v == imx455VFull16 {
		adcOut |= 0x01
	}
	if ModeOf(rm).BytesPerPx >= 2 {
		adcOut |= 0x10
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x11, adcOut); err != nil {
		return err
	}

	sx := x * bin // binned → sensor pixels (SetStartPos coordinate system)
	sy := y * bin
	sx &^= 0xf                // align X to 16
	yBias := imx455YStartBias // 0x19 for bin 1/2
	switch bin {
	case 2:
		sy &^= 3 // align Y to 4
	case 3:
		// bin 3: align Y down to a multiple of 6 and use Y bias 0x1b.
		sy = (sy / 6) * 6
		yBias = 0x1b
	default:
		sy &^= 1 // align Y to 2 (bin 1)
	}

	winMode := uint16(4)   // 0x187 = 4 normally
	startMode := uint16(1) // 0xa5 = 1 normally (SetStartPos)
	heightAdj := h
	if bin == 3 {
		winMode = 0       // bin-3: window mode 0
		startMode = 0     // bin-3: start mode 0
		heightAdj = h + 2 // bin-3: HEIGHT + 2
	}

	xShift := uint16(sx>>4) & 0xffff
	uy := uint16(sy+yBias) & 0xffff
	uw := uint16(w+0x18) & 0xffff    // output WIDTH reg = width + 0x18
	uh := uint16(heightAdj) & 0xffff // output HEIGHT reg

	writes := []RegVal{
		// Cam_SetResolution mode + window mode select.
		{Reg: imx455RegWinMode, Val: winMode},
		{Reg: imx455RegStartMode, Val: startMode}, // 0xa5: 1, or 0 at bin 3
		// ROI start X (sensorX>>4) and Y (sensorY+bias).
		{Reg: imx455RegStartXL, Val: xShift & 0xff},
		{Reg: imx455RegStartXH, Val: (xShift >> 8) & 0xff},
		{Reg: imx455RegStartYL, Val: uy & 0xff},
		{Reg: imx455RegStartYH, Val: (uy >> 8) & 0xff},
		// Output window: HEIGHT to 0x08/0x09, WIDTH(+0x18) to 0x18c/0x18d.
		{Reg: imx455RegHeightL, Val: uh & 0xff},
		{Reg: imx455RegHeightH, Val: (uh >> 8) & 0xff},
		{Reg: imx455RegWidthL, Val: uw & 0xff},
		{Reg: imx455RegWidthH, Val: (uw >> 8) & 0xff},
	}
	for _, rv := range writes {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	// FPGA optical-black crop: SetFPGAVBLK (0x06/0x07) and SetFPGAHBLK (0x02/0x03) give the FX3
	// the leading blank/OB rows and columns the sensor emits ahead of the active image, per mode.
	if err := SetFPGAVBLK(rm, uint16(mode.vblk)); err != nil {
		return err
	}
	if err := SetFPGAHBLK(rm, uint16(mode.hblk)); err != nil {
		return err
	}
	// FPGA frame geometry: SetFPGAWidth → 0x04/0x05, SetFPGAHeight → 0x08/0x09 (output dims,
	// the FX3 transfer size).
	if err := FPGAWrite16(rm, 0x04, 0x05, uint16(w)); err != nil {
		return err
	}
	if err := FPGAWrite16(rm, 0x08, 0x09, uint16(h)); err != nil {
		return err
	}
	// SetFPGABinDataLen: per-frame DMA word count = output_area · bytesPerPx / 4 (FPGA 0x40..0x43).
	bpp := ModeOf(rm).BytesPerPx
	dataWords := uint32((w*h*bpp + 3) / 4)
	if err := SetFPGABinDataLen(rm, dataWords); err != nil {
		return err
	}
	// DDR-branch HMAX: the per-mode V written to 0x13/0x14 (not the bandwidth-throttle formula).
	return FPGAWrite16(rm, 0x13, 0x14, uint16(mode.v))
}

// IMX455 per-mode register tables, the second stage of the two-stage init. InitSensorMode picks
// one by bin factor and output depth, applies it via WriteSONYREG and sets the timing base V:
//
//	bin 1, 16-bit    : reg_full_16bit  (76)   the streaming default (imx455Init), V=0x5eb
//	bin 1, high-speed: reg_full_12bit  (77)   the SDK's ASI_HIGH_SPEED_MODE (SDK RAW8 stays on
//	                                          reg_full_16bit)
//	bin 2            : reg_bin2w_12bit (77)
//	bin 3            : reg_bin3w_12bit (77)

// reg_full_16bit: bin 1, 16-bit (full-frame default).
var imx455ModeFull16 = []RegVal{
	{Reg: 0x0001, Val: 0x00}, {Reg: 0x0006, Val: 0x1d}, {Reg: 0x0016, Val: 0x02}, {Reg: 0x0028, Val: 0x0a},
	{Reg: 0x0067, Val: 0x30}, {Reg: 0x00a9, Val: 0x00}, {Reg: 0x00cc, Val: 0x8a}, {Reg: 0x00ce, Val: 0x8a},
	{Reg: 0x00d1, Val: 0x92}, {Reg: 0x00d3, Val: 0x92}, {Reg: 0x00d4, Val: 0x80}, {Reg: 0x00d5, Val: 0x01},
	{Reg: 0x00d7, Val: 0x22}, {Reg: 0x0112, Val: 0x04}, {Reg: 0x048f, Val: 0xcd}, {Reg: 0x0498, Val: 0xcd},
	{Reg: 0x0509, Val: 0xde}, {Reg: 0x050f, Val: 0x6e}, {Reg: 0x0510, Val: 0x00}, {Reg: 0x0512, Val: 0x70},
	{Reg: 0x0513, Val: 0x70}, {Reg: 0x0514, Val: 0x70}, {Reg: 0x0515, Val: 0x03}, {Reg: 0x0517, Val: 0x6e},
	{Reg: 0x0518, Val: 0x00}, {Reg: 0x051a, Val: 0x70}, {Reg: 0x051b, Val: 0x70}, {Reg: 0x051c, Val: 0x70},
	{Reg: 0x051f, Val: 0x7d}, {Reg: 0x0553, Val: 0xcc}, {Reg: 0x0574, Val: 0x02}, {Reg: 0x0575, Val: 0x02},
	{Reg: 0x0576, Val: 0x02}, {Reg: 0x0577, Val: 0x02}, {Reg: 0x0581, Val: 0x00}, {Reg: 0x0582, Val: 0x1c},
	{Reg: 0x0583, Val: 0x1c}, {Reg: 0x0584, Val: 0x1c}, {Reg: 0x0585, Val: 0x1c}, {Reg: 0x059a, Val: 0x00},
	{Reg: 0x05a1, Val: 0x6e}, {Reg: 0x05a2, Val: 0x00}, {Reg: 0x05a4, Val: 0x70}, {Reg: 0x05a5, Val: 0x70},
	{Reg: 0x05a6, Val: 0x70}, {Reg: 0x05a8, Val: 0x6e}, {Reg: 0x05a9, Val: 0x00}, {Reg: 0x05ab, Val: 0x70},
	{Reg: 0x05ac, Val: 0x70}, {Reg: 0x05ad, Val: 0x70}, {Reg: 0x05af, Val: 0x7d}, {Reg: 0x0603, Val: 0x6e},
	{Reg: 0x0605, Val: 0x6e}, {Reg: 0x062a, Val: 0xe0}, {Reg: 0x0630, Val: 0xde}, {Reg: 0x0646, Val: 0xd1},
	{Reg: 0x064a, Val: 0xd1}, {Reg: 0x066d, Val: 0x33}, {Reg: 0x066e, Val: 0x11}, {Reg: 0x0670, Val: 0x33},
	{Reg: 0x0671, Val: 0x11}, {Reg: 0x0673, Val: 0x33}, {Reg: 0x0674, Val: 0x11}, {Reg: 0x0676, Val: 0x33},
	{Reg: 0x0677, Val: 0x11}, {Reg: 0x0679, Val: 0x05}, {Reg: 0x068a, Val: 0x22}, {Reg: 0x0a80, Val: 0x81},
	{Reg: 0x0a81, Val: 0x02}, {Reg: 0x0a82, Val: 0x8d}, {Reg: 0x0a83, Val: 0x07}, {Reg: 0x0a84, Val: 0xd1},
	{Reg: 0x0a85, Val: 0x09}, {Reg: 0x0a86, Val: 0x5f}, {Reg: 0x0a87, Val: 0x11}, {Reg: 0x0a88, Val: 0x03},
}

// reg_full_12bit: bin 1, 12/8-bit (high-speed).
var imx455ModeFull12 = []RegVal{
	{Reg: 0x0001, Val: 0x80}, {Reg: 0x0006, Val: 0x1d}, {Reg: 0x0016, Val: 0x02}, {Reg: 0x0028, Val: 0x0a},
	{Reg: 0x0067, Val: 0x00}, {Reg: 0x00a9, Val: 0x02}, {Reg: 0x00cc, Val: 0x7e}, {Reg: 0x00ce, Val: 0x7e},
	{Reg: 0x00d1, Val: 0x86}, {Reg: 0x00d3, Val: 0x86}, {Reg: 0x00d4, Val: 0x00}, {Reg: 0x00d5, Val: 0x00},
	{Reg: 0x00d7, Val: 0x88}, {Reg: 0x0112, Val: 0x02}, {Reg: 0x048f, Val: 0xc1}, {Reg: 0x0498, Val: 0xc1},
	{Reg: 0x0509, Val: 0x9b}, {Reg: 0x050f, Val: 0x62}, {Reg: 0x0510, Val: 0x1b}, {Reg: 0x0512, Val: 0xd2},
	{Reg: 0x0513, Val: 0xff}, {Reg: 0x0514, Val: 0xff}, {Reg: 0x0515, Val: 0x00}, {Reg: 0x0517, Val: 0x62},
	{Reg: 0x0518, Val: 0x1b}, {Reg: 0x051a, Val: 0xd2}, {Reg: 0x051b, Val: 0xff}, {Reg: 0x051c, Val: 0xff},
	{Reg: 0x051f, Val: 0x71}, {Reg: 0x0553, Val: 0xc0}, {Reg: 0x0574, Val: 0x0d}, {Reg: 0x0575, Val: 0x0d},
	{Reg: 0x0576, Val: 0x0d}, {Reg: 0x0577, Val: 0x0d}, {Reg: 0x0581, Val: 0x04}, {Reg: 0x0582, Val: 0x1e},
	{Reg: 0x0583, Val: 0x1e}, {Reg: 0x0584, Val: 0x1e}, {Reg: 0x0585, Val: 0x1e}, {Reg: 0x059a, Val: 0x04},
	{Reg: 0x05a1, Val: 0x62}, {Reg: 0x05a2, Val: 0x1b}, {Reg: 0x05a4, Val: 0xd2}, {Reg: 0x05a5, Val: 0xff},
	{Reg: 0x05a6, Val: 0xff}, {Reg: 0x05a8, Val: 0x62}, {Reg: 0x05a9, Val: 0x1b}, {Reg: 0x05ab, Val: 0xd2},
	{Reg: 0x05ac, Val: 0xff}, {Reg: 0x05ad, Val: 0xff}, {Reg: 0x05af, Val: 0x71}, {Reg: 0x0603, Val: 0x62},
	{Reg: 0x0605, Val: 0x62}, {Reg: 0x062a, Val: 0xd4}, {Reg: 0x0630, Val: 0xd2}, {Reg: 0x0646, Val: 0xc5},
	{Reg: 0x064a, Val: 0xc5}, {Reg: 0x066d, Val: 0x00}, {Reg: 0x066e, Val: 0x00}, {Reg: 0x0670, Val: 0x00},
	{Reg: 0x0671, Val: 0x00}, {Reg: 0x0673, Val: 0x00}, {Reg: 0x0674, Val: 0x00}, {Reg: 0x0676, Val: 0x00},
	{Reg: 0x0677, Val: 0x00}, {Reg: 0x0679, Val: 0x07}, {Reg: 0x068a, Val: 0x88}, {Reg: 0x0a80, Val: 0x61},
	{Reg: 0x0a81, Val: 0x02}, {Reg: 0x0a82, Val: 0xe5}, {Reg: 0x0a83, Val: 0x02}, {Reg: 0x0a84, Val: 0x77},
	{Reg: 0x0a85, Val: 0x04}, {Reg: 0x0a86, Val: 0x39}, {Reg: 0x0a87, Val: 0x05}, {Reg: 0x0a88, Val: 0x03},
	{Reg: 0x0a96, Val: 0x01},
}

// reg_bin2w_12bit: bin 2.
var imx455ModeBin2w12 = []RegVal{
	{Reg: 0x0001, Val: 0x85}, {Reg: 0x0006, Val: 0x1d}, {Reg: 0x0016, Val: 0x02}, {Reg: 0x0028, Val: 0x04},
	{Reg: 0x0067, Val: 0x00}, {Reg: 0x00a9, Val: 0x02}, {Reg: 0x00cc, Val: 0x7e}, {Reg: 0x00ce, Val: 0x7e},
	{Reg: 0x00d1, Val: 0x86}, {Reg: 0x00d3, Val: 0x86}, {Reg: 0x00d4, Val: 0x00}, {Reg: 0x00d5, Val: 0x00},
	{Reg: 0x00d7, Val: 0x88}, {Reg: 0x0112, Val: 0x02}, {Reg: 0x048f, Val: 0xc1}, {Reg: 0x0498, Val: 0xc1},
	{Reg: 0x0509, Val: 0x9b}, {Reg: 0x050f, Val: 0x62}, {Reg: 0x0510, Val: 0x1b}, {Reg: 0x0512, Val: 0xd2},
	{Reg: 0x0513, Val: 0xff}, {Reg: 0x0514, Val: 0xff}, {Reg: 0x0515, Val: 0x00}, {Reg: 0x0517, Val: 0x62},
	{Reg: 0x0518, Val: 0x1b}, {Reg: 0x051a, Val: 0xd2}, {Reg: 0x051b, Val: 0xff}, {Reg: 0x051c, Val: 0xff},
	{Reg: 0x051f, Val: 0x71}, {Reg: 0x0553, Val: 0xc0}, {Reg: 0x0574, Val: 0x0d}, {Reg: 0x0575, Val: 0x0d},
	{Reg: 0x0576, Val: 0x0d}, {Reg: 0x0577, Val: 0x0d}, {Reg: 0x0581, Val: 0x04}, {Reg: 0x0582, Val: 0x1e},
	{Reg: 0x0583, Val: 0x1e}, {Reg: 0x0584, Val: 0x1e}, {Reg: 0x0585, Val: 0x1e}, {Reg: 0x059a, Val: 0x04},
	{Reg: 0x05a1, Val: 0x62}, {Reg: 0x05a2, Val: 0x1b}, {Reg: 0x05a4, Val: 0xd2}, {Reg: 0x05a5, Val: 0xff},
	{Reg: 0x05a6, Val: 0xff}, {Reg: 0x05a8, Val: 0x62}, {Reg: 0x05a9, Val: 0x1b}, {Reg: 0x05ab, Val: 0xd2},
	{Reg: 0x05ac, Val: 0xff}, {Reg: 0x05ad, Val: 0xff}, {Reg: 0x05af, Val: 0x71}, {Reg: 0x0603, Val: 0x62},
	{Reg: 0x0605, Val: 0x62}, {Reg: 0x062a, Val: 0xd4}, {Reg: 0x0630, Val: 0xd2}, {Reg: 0x0646, Val: 0xc5},
	{Reg: 0x064a, Val: 0xc5}, {Reg: 0x066d, Val: 0x00}, {Reg: 0x066e, Val: 0x00}, {Reg: 0x0670, Val: 0x00},
	{Reg: 0x0671, Val: 0x00}, {Reg: 0x0673, Val: 0x00}, {Reg: 0x0674, Val: 0x00}, {Reg: 0x0676, Val: 0x00},
	{Reg: 0x0677, Val: 0x00}, {Reg: 0x0679, Val: 0x07}, {Reg: 0x068a, Val: 0x88}, {Reg: 0x0a80, Val: 0x61},
	{Reg: 0x0a81, Val: 0x02}, {Reg: 0x0a82, Val: 0xe5}, {Reg: 0x0a83, Val: 0x02}, {Reg: 0x0a84, Val: 0x77},
	{Reg: 0x0a85, Val: 0x04}, {Reg: 0x0a86, Val: 0x39}, {Reg: 0x0a87, Val: 0x05}, {Reg: 0x0a88, Val: 0x03},
	{Reg: 0x0a96, Val: 0x01},
}

// reg_bin3w_12bit: bin 3 (differs from bin2w only at 0x0001/0x0006 and the 0x0a8x command
// block).
var imx455ModeBin3w12 = []RegVal{
	{Reg: 0x0001, Val: 0x89}, {Reg: 0x0006, Val: 0x27}, {Reg: 0x0016, Val: 0x03}, {Reg: 0x0028, Val: 0x04},
	{Reg: 0x0067, Val: 0x00}, {Reg: 0x00a9, Val: 0x02}, {Reg: 0x00cc, Val: 0x7e}, {Reg: 0x00ce, Val: 0x7e},
	{Reg: 0x00d1, Val: 0x86}, {Reg: 0x00d3, Val: 0x86}, {Reg: 0x00d4, Val: 0x00}, {Reg: 0x00d5, Val: 0x00},
	{Reg: 0x00d7, Val: 0x88}, {Reg: 0x0112, Val: 0x02}, {Reg: 0x048f, Val: 0xc1}, {Reg: 0x0498, Val: 0xc1},
	{Reg: 0x0509, Val: 0x9b}, {Reg: 0x050f, Val: 0x62}, {Reg: 0x0510, Val: 0x1b}, {Reg: 0x0512, Val: 0xd2},
	{Reg: 0x0513, Val: 0xff}, {Reg: 0x0514, Val: 0xff}, {Reg: 0x0515, Val: 0x00}, {Reg: 0x0517, Val: 0x62},
	{Reg: 0x0518, Val: 0x1b}, {Reg: 0x051a, Val: 0xd2}, {Reg: 0x051b, Val: 0xff}, {Reg: 0x051c, Val: 0xff},
	{Reg: 0x051f, Val: 0x71}, {Reg: 0x0553, Val: 0xc0}, {Reg: 0x0574, Val: 0x0d}, {Reg: 0x0575, Val: 0x0d},
	{Reg: 0x0576, Val: 0x0d}, {Reg: 0x0577, Val: 0x0d}, {Reg: 0x0581, Val: 0x04}, {Reg: 0x0582, Val: 0x1e},
	{Reg: 0x0583, Val: 0x1e}, {Reg: 0x0584, Val: 0x1e}, {Reg: 0x0585, Val: 0x1e}, {Reg: 0x059a, Val: 0x04},
	{Reg: 0x05a1, Val: 0x62}, {Reg: 0x05a2, Val: 0x1b}, {Reg: 0x05a4, Val: 0xd2}, {Reg: 0x05a5, Val: 0xff},
	{Reg: 0x05a6, Val: 0xff}, {Reg: 0x05a8, Val: 0x62}, {Reg: 0x05a9, Val: 0x1b}, {Reg: 0x05ab, Val: 0xd2},
	{Reg: 0x05ac, Val: 0xff}, {Reg: 0x05ad, Val: 0xff}, {Reg: 0x05af, Val: 0x71}, {Reg: 0x0603, Val: 0x62},
	{Reg: 0x0605, Val: 0x62}, {Reg: 0x062a, Val: 0xd4}, {Reg: 0x0630, Val: 0xd2}, {Reg: 0x0646, Val: 0xc5},
	{Reg: 0x064a, Val: 0xc5}, {Reg: 0x066d, Val: 0x00}, {Reg: 0x066e, Val: 0x00}, {Reg: 0x0670, Val: 0x00},
	{Reg: 0x0671, Val: 0x00}, {Reg: 0x0673, Val: 0x00}, {Reg: 0x0674, Val: 0x00}, {Reg: 0x0676, Val: 0x00},
	{Reg: 0x0677, Val: 0x00}, {Reg: 0x0679, Val: 0x07}, {Reg: 0x068a, Val: 0x88}, {Reg: 0x0a80, Val: 0x62},
	{Reg: 0x0a81, Val: 0x02}, {Reg: 0x0a82, Val: 0xe6}, {Reg: 0x0a83, Val: 0x02}, {Reg: 0x0a84, Val: 0x78},
	{Reg: 0x0a85, Val: 0x04}, {Reg: 0x0a86, Val: 0x3a}, {Reg: 0x0a87, Val: 0x05}, {Reg: 0x0a88, Val: 0x03},
	{Reg: 0x0a96, Val: 0x01},
}
