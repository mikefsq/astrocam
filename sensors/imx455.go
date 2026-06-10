// This profile captures full 9576×6388 16-bit frames through the pure-Go driver on
// hardware over BOTH USB3 and USB2, with correct exposure and gain. The register
// programming was cross-checked two ways: a USB2 Cynthion wire capture of the official
// SDK, AND the SDK's own debug log (asicamerasdk.log), which prints its internal SetExp
// values verbatim — VMAX:0x1928 (=6440 default) / 0x19c0 (=6592 trigger), SSH1:0x14,
// 1f:487830, 75.75us, "Enter long exp mode" at the 1 s threshold — all reproduced here.
// (So the "USB3 cameras can't be wire-sniffed" caveat in SENSORS.md no longer holds for
// the 6200: a USB2 host port + the interposer/Cynthion gives a full SDK reference.)
//
// The register model:
//   - Exposure is TWO modes: < 1 s free-run (VMAX = vblank+height, the time encoded in
//     SHS); >= 1 s FPGA wait+trigger mode (VMAX held at ONE frame ~6592, integration
//     HOST-timed via EnableFPGATriggerSignal, like the 174). The VMAX base is vblank = 52,
//     NOT V (=1515).
//   - Gain code = 4095·(1 − 10^(−gain/200)) (exp10 with K = 4095); gain 0 → 0.
//   - Window regs: sensor 0x08/0x09 = HEIGHT, 0x18c/0x18d = WIDTH+0x18; plus the FPGA
//     geometry (SetFPGAWidth 0x04/05, SetFPGAHeight 0x08/09).
//   - Data plane: the 6200 streams over the windowed FX3 pump with FPGABufReload tail-flush
//     and a time-based stall (transport_darwin.go); see imx455Worker.
//
// One IMX455 die backs the whole ASI6200 MM/MC family (the worker is variant-agnostic).
// Still UNVERIFIED: the binned/12-bit readout modes (only full-frame 16-bit is on the
// wire) and the small config bytes 0x2d / 0x46 / sensor 0x40-43 (gain-range/mode-table).
//
// SECOND-SOURCE CONFIRMED (PlayerOne, no extra hardware): the gain/HCG registers
// (0x2d, 0x3a4/5/6), the offset regs + mirror, and the code formula 4095·(1−10^(−g/200)) all
// match. PlayerOne also drives this die — SetGain/SetOffset/GainCaps/OffsetCaps dispatch on the
// regmap's VID (ZWO 0x03C3 vs PlayerOne 0xA0A0); see imx455SetGainPOA. (One real difference:
// ASI never writes 0x67f on the 455; PlayerOne does.)
package sensors

import . "asicam"

import (
	"fmt"
	"math"
	"time"
)

// Sony IMX455 — 35mm full-frame (62MP) back-illuminated CMOS, the imager in the
// ZWO ASI6200 family (ASI6200MM/MC Pro). Unlike the small "Type 1/2.x" Starvis
// parts (IMX290/462), this large sensor's register map lives in the low Sony
// address range: the SHS shutter is a 24-bit value packed into regs
// 0x16/0x17 (low two bytes; the top byte rides the FPGA VMAX path), the analog
// gain code is the 0x2e/0x2f (mirrored to 0x30/0x31) pair with a conversion-gain
// nibble in 0x3e, and the ROI start/window registers are the 0xa5..0xa7 / 0x06
// /0x07 / 0x08 / 0x09 / 0x18c / 0x18d command-byte registers written via
// WriteSONYREG. VMAX (frame length) is programmed through the FPGA (SetFPGAVMAX) and
// equals vblank + height (vblank = 52, not the line-time base V).
//
// Register map:
//
//	InitCamera         (reglist_init table loop + explicit Sony tail + FPGA bringup)
//	InitSensorMode     (per-mode table select by bin/depth + base V; see imx455_modes.go)
//	Cam_SetResolution  (window HEIGHT 0x08/0x09, WIDTH+0x18 0x18c/0x18d, mode 0x187;
//	                    FPGA geometry SetFPGAWidth 0x04/05 + SetFPGAHeight 0x08/09)
//	SetStartPos        (ROI X 0xa6/0xa7 (X>>4), Y 0x06/0x07; mode 0xa5=1; X aligned to 16)
//	SetGain            (code = 4095·(1−10^(−gain/200)) → 0x2e/0x2f mirrored 0x30/0x31)
//	SetExp             (>=1s wait+trigger mode, VMAX≈one frame, SHS=10; <1s free-run)
//	the FPS-percent throttle (HMAX + SetFPGAHMAX — line time is throttle-derived)
//	the capture worker (sensor stream gate: master 0x19e = 5 stream / 0 stop)
//	reglist_init       (36 reg/val16 entries)
const (
	// SetGain — analog gain code (16-bit) written low/high to 0x2e/0x2f
	// and mirrored to 0x30/0x31; the conversion-gain selection is a 4-bit field
	// placed at bits [4:7] of 0x3e.
	imx455RegGainCodeL  = 0x2e
	imx455RegGainCodeH  = 0x2f
	imx455RegGainCodeL2 = 0x30
	imx455RegGainCodeH2 = 0x31
	imx455RegGainConv   = 0x3e // conversion gain / HCG nibble (bits 4..7)
	imx455RegGainMode   = 0x2d // fixed mode byte written at start of the group (0x2d)
	imx455RegGainAux    = 0x4d // auxiliary code byte (per-range, 0x08/0x0a/0x0c)
	imx455RegGainCfg0   = 0x3a2
	imx455RegGainCfg1   = 0x3a3
	imx455RegGainCfg2   = 0x3a4
	imx455RegGainCfg3   = 0x3a5
	imx455RegGainCfg4   = 0x3a6

	// SetExp — shutter (SHS), the low two bytes of the 24-bit value
	// (top byte couples to the FPGA-programmed VMAX).
	imx455RegSHS0 = 0x16
	imx455RegSHS1 = 0x17

	// Master streaming gate: 5 = stream, 0 = stop. (1 is a
	// standby/ready value also written; init sets it 1 then 5.)
	imx455RegMaster = 0x19e

	// SetStartPos — ROI start. X (aligned to 16) is shifted right 4 and
	// written as two bytes to 0xa6/0xa7; Y is written low/next byte to 0x06/0x07
	// (with a +0x19 / +0x1b row offset folded in). 0xa5 is the X/Y mode select.
	imx455RegStartXL   = 0xa6 // X>>4, bits [0:7]
	imx455RegStartXH   = 0xa7 // X>>4, bits [8:15]
	imx455RegStartYL   = 0x06
	imx455RegStartYH   = 0x07
	imx455RegStartMode = 0xa5

	// Cam_SetResolution — output window W/H: sensor 0x08/0x09 carries HEIGHT
	// (e.g. 6388) and 0x18c/0x18d carries WIDTH+0x18 (e.g. 9576+24 = 9600).
	imx455RegHeightL = 0x08  // window HEIGHT low
	imx455RegHeightH = 0x09  // window HEIGHT high
	imx455RegWidthL  = 0x18c // window WIDTH low  = width+0x18
	imx455RegWidthH  = 0x18d // window WIDTH high
	imx455RegWinMode = 0x187

	imx455GainMax    = 0x2bc      // 700 (0.1 dB units); SetGain clamp
	imx455GainHCGAt  = 0x64       // 100: gain ranges split at 0x64 / 0x1cd / 0xa0
	imx455ExpMinUs   = 0x20       // 32 µs, SetExp clamp lo
	imx455ExpMaxUs   = 0x77359400 // 2 000 000 000 µs, SetExp clamp hi
	imx455SHSAdd     = 0x14       // long-exp SHS residual when VMAX = lines + 0x14
	imx455YStartBias = 0x19       // Y row offset added before 0x06/0x07

	// Timing model (the per-mode V values and mode→table map are in
	// imx455_modes.go). InitSensorMode stores a per-mode base constant V; V is used
	// TWO ways:
	//
	//	VMAX (FPGA frame length) = V + effHeight   (SetExp: V + effHeight)
	//	HMAX = V                    (the FPS-percent throttle DDR path)
	//	lineTime = HMAX*1000/clock = V*1000/20000 ns
	//
	// clock = 20000. VMAX is V + height (not a fixed 3912), and
	// vblank is V.
	//
	// Per-mode V below is the NORMAL value; the strap (FPGA reg 0x1c == 5) HALVES it.
	// Full-frame fps strongly favors the strap: V=880 + full height ≈ 3.1 fps (≈ the
	// ASI6200 ~3.5 spec) vs V=1515 ≈ 1.7 fps — so the real camera likely runs the strap
	// path, but reading reg 0x1c (and the 2-rows-per-line readout question) needs
	// hardware. The default (normal V) is used here.
	imx455ClkKHz    = 20000 // pixel clock in kHz (= 0x4e20)
	imx455VFull16   = 0x5eb // 1515: bin1 16-bit (strap 0x370=880)
	imx455VFull12   = 0x276 // 630:  bin1 12-bit (strap 0x201=513)
	imx455VBin2     = 0x271 // 625:  bin2 & bin4 (strap 0x16d=365)
	imx455VBin3     = 0x14a // 330:  bin3        (strap 0xe0=224)
	imx455SHSOffset = -3    // SHS = VMAX - 3 - lines

	// Exposures >= 1 s switch to FPGA wait+trigger mode — VMAX is held at ONE frame and
	// the integration is host-timed (EnableFPGATriggerSignal), like the 174's long path.
	// Below 1 s is free-run with the time encoded in VMAX/SHS.
	imx455ExpTrigUs = 1_000_000
	// vblank = 52: the default-VMAX VBLANK base for the full-frame 16-bit mode
	// (defaultVMAX = vblank + height). This is NOT V (= 1515). Only the bin1/16-bit value
	// is wire-verified; other readout modes carry their own vblank.
	imx455VBlankFull16 = 52
	imx455FullWidth    = 9576 // output width at full-frame bin 1 (= MaxWidth)
	imx455FullHeight   = 6388 // output rows at full-frame bin 1 (= MaxHeight); VMAX = V + effHeight
)

// imx455Mode is the readout-mode profile InitSensorMode applies: the per-mode
// register table plus the timing base V (drives VMAX = V + height and lineTime).
type imx455Mode struct {
	table []RegVal
	v     int
}

// imx455SelectMode maps binning + output bytes/pixel to the readout mode, mirroring the
// InitSensorMode bin/depth branches. bin 4 reuses the bin-2 table (and V). V is the
// normal (non-strap) value — see the timing-model note.
func imx455SelectMode(bin, bytesPerPx int) imx455Mode {
	switch bin {
	case 2:
		return imx455Mode{imx455ModeBin2w12, imx455VBin2}
	case 3:
		return imx455Mode{imx455ModeBin3w12, imx455VBin3}
	case 4:
		return imx455Mode{imx455ModeBin2w12, imx455VBin2}
	default: // bin 1
		if bytesPerPx >= 2 {
			return imx455Mode{imx455ModeFull16, imx455VFull16}
		}
		return imx455Mode{imx455ModeFull12, imx455VFull12}
	}
}

// imx455InitCommon is the common (mode-independent) first stage of InitCamera:
// the 36-entry reglist_init table (reg 0xffff = InitDelayReg, delay = val ms) followed
// by the explicit WriteSONYREG tail. The per-mode second stage is in imx455_modes.go.
//
// The FPGA-side bringup InitCamera performs after this tail (FPGAReset, the 20 ms delay,
// FPGADDRTest, SetFPGAAsMaster/FPGAStop/EnableFPGADDR/SetFPGAADCWidthOutputWidth/
// SetFPGABinMode/SetFPGAGain) is in imx455InitFPGA. SendCMD(0xAF/0xAE) and the cooling
// init are Camera-level (Camera.Init around InitFPGA).
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

// imx455Init is the streaming default: the common init followed by the bin-1 16-bit
// per-mode table. The 6200's init is TWO-STAGE — InitCamera applies imx455InitCommon,
// then InitSensorMode applies one per-mode table (see imx455_modes.go for all
// four and the mode→table mapping). Only the bin-1 16-bit mode is wired as the default;
// the asicam Sensor model has no mode-selection hook yet.
//
// ORDER CAVEAT: the SDK applies the per-mode table AFTER imx455InitFPGA; here it is in
// Init, which runs before InitFPGA. These are FPGA-independent Sony registers, so the
// order is not expected to matter — flag it if bringup misbehaves.
var imx455Init = append(append([]RegVal{}, imx455InitCommon...), imx455ModeFull16...)

var IMX455 = Sensor{
	Name:     "IMX455", // ASI6200MC Pro; Sony IMX455 full-frame 62MP
	GainMax:  imx455GainMax,
	ExpMinUs: imx455ExpMinUs,
	ExpMaxUs: imx455ExpMaxUs,
	// ASI Brightness / black level. Caps: 0..200, caps-default 1.
	// But ASIInitCamera APPLIES a higher black level on the 6200 — hardware-confirmed ~502 DN, i.e.
	// OFFSET 50 (gosnap's offset→DN is avg≈10·offset+2, matching the SDK 502 DN at offset 50). The
	// caps "default" (1) clips at the floor; default to 50 to match the SDK's actual init. (Same
	// caps-default-vs-init-applied split as the 462, which the SDK inits to Brightness 100.)
	OffsetMax: 200, OffsetDef: 50,
	Info: CameraInfo{
		MaxWidth:  9576,
		MaxHeight: 6388,
		PixelUm:   3.76,
		BitDepth:  16,
		Bayer:     "RGGB", // MC = color
		Bins:      []int{1, 2, 3, 4},
	},
	Init:        imx455Init,
	InitFPGA:    imx455InitFPGA,
	SetGain:     imx455SetGain,
	GainCaps:    imx455GainCaps,
	SetExposure: imx455SetExposure,
	SetOffset:   imx455SetOffset,
	OffsetCaps:  imx455OffsetCaps,
	SetROI:      imx455SetROI,
	StreamStop:  func(rm Regmap) error { return rm.WriteReg(imx455RegMaster, 0) }, // 0x19e = 0
	StreamStart: func(rm Regmap) error { return rm.WriteReg(imx455RegMaster, 5) }, // 0x19e = 5
	Worker:      imx455Worker,                                                     // rich arm + windowed stream read
}

// imx455Worker is the host-timed single-shot capture. Same skeleton as the IMX174/290,
// but the sensor gate is the 0x19e master register (5 = stream, 0 = stop) and the settle
// is 10 ms. For the long bands SetExp arms trigger MODE (EnableFPGATriggerMode); this
// worker drives the trigger SIGNAL (EnableFPGATriggerSignal, FPGA reg 0x0b bit0) whose
// 1->0 edge releases the frame. UNVERIFIED.
//
//	arm:    SendCMD(0xAA)·FPGAStop·0x19e=5·SendCMD(0xA9)·0x19e=0·usleep(10ms)·FPGAStart·ResetEndpoint
//	expose: EnableFPGATriggerSignal(1) · host-time (≤1 s: exposure−200 ms; >1 s: 100 ms poll)
//	fire:   EnableFPGATriggerSignal(0) · BulkRead
//
// SEAM: the 6200's very-long bands run a sub-integration cycle loop between FPGABufReload
// and the trigger — repeated FPGAStop/usleep(5 ms)/FPGAStart/usleep(20 ms) gated on
// ReadFPGAREG(0x23) status, the hardware multi-exposure accumulation. NOT reproduced
// here; this is the single-shot only. The exact arm gate values (write 0x19e=5 then a
// brief 1) are unconfirmed.
//
// The ASI6200/IMX455 is a free-run, SHS-timed USB3 sensor, so the capture is NOT a
// host-timed trigger like the 174 — it is "arm the sensor master gate, then pull one
// whole frame as a continuous stream."
//
// Two pieces matter:
//
//   - The arm: SendCMD(0xAA) → FPGAStop → master(0x19e)=5
//     → sensor reg0 |= 1 → SendCMD(0xA9) → FPGAStop → master=1 → reg0 |= 4 → usleep
//     10 ms → reg0 &= ~1 → FPGAStart → ResetEndPoint(0x81). This richer sequence (vs
//     the generic StreamStop/Start) is what actually gets the sensor streaming.
//   - The read: the SDK keeps a window of transfers cycling (startAsyncXfer) until the
//     full w23 = W×H×2 bytes are in, never stopping at a mid-frame short packet. That
//     is ctl.StreamFrame. On a stall the SDK kicks FPGAStop→usleep→FPGAStart; we mirror
//     that and continue reading the remainder.
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
	fpgaStop := func() error { return SetFPGABit(rm, 0x00, 0x10, true) }   // reg0 bit4 = 1
	fpgaStart := func() error { return SetFPGABit(rm, 0x00, 0x10, false) } // reg0 bit4 = 0
	// FPGABufReload: FPGA reg 0x18 |= 1. The FX3 commits whole 1-MiB DMA buffers and
	// HOLDS the final partial buffer of a frame; this commits it so the frame's tail
	// (the bytes past the last 1-MiB boundary) flushes to the host. Issued inside the
	// read loop.
	bufReload := func() error { return SetFPGABit(rm, 0x18, 0x01, true) }
	// EnableFPGATriggerSignal: FPGA reg 0x0b bit0. In wait+trigger mode
	// (>= 1 s, set by SetExposure) the host holds this for the integration time.
	triggerSignal := func(on bool) error { return SetFPGABit(rm, 0x0b, 0x01, on) }

	// --- arm ---
	if err := ctl.VendorCmd(0xAA); err != nil { // FX3 stream stop
		return 0, err
	}
	if err := fpgaStop(); err != nil {
		return 0, err
	}
	if err := rm.WriteReg(imx455RegMaster, 5); err != nil { // master stream on
		return 0, err
	}
	if err := regRMW(0x01, 0); err != nil { // sensor reg0 |= 1
		return 0, err
	}
	if err := ctl.VendorCmd(0xA9); err != nil { // FX3 stream start
		return 0, err
	}
	if err := fpgaStop(); err != nil {
		return 0, err
	}
	if err := rm.WriteReg(imx455RegMaster, 1); err != nil { // master = 1
		return 0, err
	}
	if err := regRMW(0x04, 0); err != nil { // sensor reg0 |= 4
		return 0, err
	}
	time.Sleep(10 * time.Millisecond)       // usleep(0x2710)
	if err := regRMW(0, 0x01); err != nil { // sensor reg0 &= ~1
		return 0, err
	}
	if err := fpgaStart(); err != nil {
		return 0, err
	}
	_ = ctl.ResetEndpoint() // ResetEndPoint(0x81)

	// >= 1 s runs in FPGA wait+trigger mode (SetExposure set reg0 bit6/bit7 and held VMAX
	// at one frame). The integration is HOST-timed: assert the trigger signal, hold for
	// the exposure, then release so the frame clocks out — mirroring the 174 long path.
	// Below 1 s is free-run: the sensor self-times via SHS and the read blocks on the frame.
	if exposure >= imx455ExpTrigUs*time.Microsecond {
		if err := triggerSignal(true); err != nil {
			return 0, err
		}
		time.Sleep(exposure)
		if err := triggerSignal(false); err != nil {
			return 0, err
		}
	}

	// --- read one whole frame (continuous windowed pump, w23 = FrameBytes) ---
	target := ctl.FrameBytes()
	if target > len(buf) {
		target = len(buf)
	}
	// idle must cover the wait for the first chunk (the sensor integrates then reads
	// out, so first data can take ~one frame period); once flowing, chunks are ms apart.
	idle := exposure + 2*time.Second
	total := 2*exposure + 5*time.Second
	// The FX3 commits whole 1-MiB DMA buffers and HOLDS the frame's final partial buffer
	// until it is filled or committed; pulse FPGABufReload throughout the read so that
	// partial commits and the frame's tail (the bytes past the last 1-MiB boundary)
	// flushes into a posted transfer. FPGABufReload is issued inside the read loop for
	// exactly this; the windowed reader treats the frame-end ZLP as non-terminal and
	// keeps cycling until the whole frame lands.
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
	<-done // JOIN: wait for the ticker's last bufReload to finish before returning, so no
	// control transfer outlives the worker to race the next operation / the TEC loop.
	return n, err
}

// imx455InitFPGA is the FPGA-side bringup InitCamera performs after
// the Sony init tail, using the FX3 register numbers. Shared helpers
// FPGASetBits/FPGAClearBits/FPGAWriteBits (imx174.go) do the read-modify-writes.
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
// SEAMS: EnableFPGADDR's argument is the runtime DDR flag (set at
// OpenCamera). The 6200 is a high-bandwidth USB3 part that uses DDR, so DDR-enabled
// (reg0xa bit6 = 0) is assumed; revisit if a non-DDR strap is found. The FPGADDRTest
// gate (a read-back self-test) is not reproduced — bringup proceeds unconditionally.
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
	if err := FPGAWriteBits(rm, 0x0a, 0x40, 0x00); err != nil { // EnableFPGADDR(1): reg0xa bit6 = 0 (DDR enabled)
		return err
	}
	// SetFPGAADCWidthOutputWidth(adc=1, outputWidth): reg0xa bit0 = adc, bit4 = output width.
	// InitCamera passes outputWidth = 0 (8-bit); raise bit4 for RAW16 from the live ReadoutMode.
	// Without this the FPGA streams a half-size RAW8 frame while the host expects RAW16 — the
	// same half-frame bug the ASI290 exhibited on hardware (imx290InitFPGA). UNVERIFIED on 455.
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

// imx455SetGain — SetGain. The gain axis splits into FIVE bands at 61/100/160/280/461.
// Two key details:
//
//   - HCG (high conversion gain): reg 0x2d is the conversion-gain MODE byte, and its bit0
//     is the HCG enable — 0 below gain 100 (LCG), 1 at/above 100 (HCG). The read-noise
//     break lives here.
//   - Analog code resets at the HCG boundary: the exp10 ramp uses `gain` below 100 and
//     `gain−100` at/above. For the top band it also subtracts a coarse 60-unit stage.
//
// reg 0x3e is NOT a binary HCG bit — it is the high-range coarse-gain STAGE index
// (0 for gain ≤ 460; ceil(((gain+52)&0xff)/60) for 461..700), placed in bits[4:7].
//
// VALIDATED: the LCG ramp (band A, gain 0..99) matches the SDK on the wire. The HCG bands
// and the top-stage logic are decoded but not yet wire-confirmed across the band edges —
// a gain-sweep capture (61/100/160/280/461) + a dark-frame read-noise check is the way to
// lock which threshold is the true read-noise break.
// imx455GainCaps / imx455OffsetCaps return the advertised gain/offset range per vendor — the
// dual of the dispatched SetGain/SetOffset. ZWO: gain 0..700 (0.1 dB), offset 0..200 def 1.
// PlayerOne: gain 0..550, offset 0..2000 def 20, confirmed by the PlayerOne SetGain/SetOffset clamp;
// identical across PlayerOne dies. max 0 = undeclared.
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
		return 0, 200, 1
	case POA.VID:
		return 0, 2000, 20
	default:
		return 0, 0, 0
	}
}

// imx455SetGain selects the vendor's gain encoding from the regmap's VID. The IMX455 die is
// shared across vendors, each driving it with a different gain-unit band structure; an
// unrecognized vendor is an error (no implicit default).
func imx455SetGain(rm Regmap, gain int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx455SetGainZWO(rm, gain)
	case POA.VID:
		return imx455SetGainPOA(rm, gain)
	default:
		return fmt.Errorf("asicam: imx455 gain: unsupported vendor VID 0x%04x", rm.VID())
	}
}

// imx455SetGainPOA is PlayerOne's IMX455 gain encoding, with the threshold
// M = 125. Same Sony die as ZWO but a different gain-unit band structure:
// each conversion-gain band rebases the gain into the shared analog-code formula, and the
// gain-setup register 0x67f is written per band through the obfuscated CrypWrite path
// (poaRegmap routes 0x67f automatically, so this just calls WriteReg). UNVERIFIED — no
// PlayerOne hardware.
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
// The advertised gain max lives in the POAConfigAttributes builder (not yet decoded), so only
// the low end is clamped here; the analog code saturates at 4095 as in the object code.
func imx455SetGainPOA(rm Regmap, gain int) error {
	if gain < 0 {
		gain = 0
	}
	const m = 125 // gain threshold M

	var mode, setup, cfgA, cfgBC, g uint16 = 0, 0, 0x11, 0x11, 0
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
	// effGain may exceed gain in the top band (stage offset).
	g := math.Pow(10, float64(effGain)/-200.0)
	code := int32(4095.0 * (1.0 - g))
	codeU := uint16(code)

	writes := []RegVal{
		{Reg: imx455RegGainMode, Val: mode}, // 0x2d — HCG mode
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
		{Reg: imx455RegGainConv, Val: (stage & 0xf) << 4}, // 0x3e — high-range stage
	}
	for _, rv := range writes {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}
	return nil
}

// imx455GainStage is the coarse 60-unit gain-stage index for the top
// band (gain ≥ 461) = ceil(((gain+52)&0xff)/60), keeping the exact byte-wrapped
// quotient and full-width remainder check.
func imx455GainStage(gain int) uint16 {
	v := (gain + 52) & 0xff
	q := (v * 137) >> 13      // floor(v/60) via the 137/8192 reciprocal
	rem := (gain + 52) - q*60 // remainder on the FULL gain+52
	if rem&0xff != 0 {
		q++
	}
	return uint16(q)
}

// imx455SetExposure — SetExp. Full-frame model:
//
//	lines = exposure / lineTime,  lineTime_ns = HMAX*1000/clock = V*1000/20000
//	VMAX  = V + effHeight          // extends for long exp
//	SHS   = (VMAX - 3) - lines      // clamped to [3, VMAX-3]
//	SHS   = SHS>=6 ? SHS>>1 : 3     // bin-1 readout halving
//	write SHS little-endian to the indexed regs 0x16/0x17 (no latch)
//
// VMAX (FPGA frame length) goes via SetVMAX (regs 0x10/0x11/0x12); the SHS low two bytes
// to the indexed sensor regs 0x16/0x17. effHeight is taken as the full-frame height (bin
// 1, no ROI argument here — same assumption as the VMAX default).
//
// SEAMS: ×2/×4 binning skips the SHS halve (height already folded into VMAX) — bin is not
// plumbed here (full-frame bin 1 assumed; depth comes from the live ReadoutMode). The
// long-exposure branch (VMAX = lines + 0x14, residual SHS 0x14) is not modelled.
// And the normal-vs-strap V choice plus the fps reconciliation need hardware (see the
// timing-model note).
//
// Cross-checked against a wire capture (2 s → VMAX 0x19c0=6592, SHS 10, reg0 0xf1).
// Full-frame assumption (no ROI arg): bin 1, output depth from the live ReadoutMode.
//
//	line_time   = V*1000/clock                     (HMAX*1000/clock)
//	defaultVMAX = vblank + height                  (vblank + effHeight, ONE frame)
//	>= 1 s : wait+trigger mode; VMAX = (frameµs+10ms)/line_time + 20 (≈ one frame); SHS = 20.
//	         The integration is HOST-timed by the worker (EnableFPGATriggerSignal), not VMAX.
//	<  1 s : free-run. exposure <= frame-time keeps defaultVMAX and encodes the time in SHS;
//	         a longer one extends VMAX = exposure/line_time + 20.
//	SHS is then halved for the bin-1 readout.
//
// imx455SetOffset — SetBrightness (ASI Brightness / black level).
// value = offset·10 at bin 1; binned scales to offset·100/16 (≈·6.25). Written
// 16-bit little-endian to sensor 0x40/0x41 and mirrored to 0x42/0x43.
// imx455SetOffset selects the black-level encoding from the regmap's VID — PlayerOne offset·8,
// ZWO offset·10; an unrecognized vendor is an error.
func imx455SetOffset(rm Regmap, offset int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx455SetOffsetZWO(rm, offset)
	case POA.VID:
		return imx455SetOffsetPOA(rm, offset)
	default:
		return fmt.Errorf("asicam: imx455 offset: unsupported vendor VID 0x%04x", rm.VID())
	}
}

// imx455SetOffsetPOA is PlayerOne's IMX455 black level: offset·8 (vs ZWO's ·10), same
// 0x40/0x41 mirror 0x42/0x43 block (offset << 3 → 4-byte block; the four singles here
// have the identical register effect).
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

func imx455SetExposure(rm Regmap, d time.Duration) error {
	bin := int64(ModeOf(rm).Bin)
	if bin < 1 {
		bin = 1
	}
	// Line-time base V, set per readout mode by InitSensorMode (bin 1:
	// VFull16/12; bin 2/4: VBin2; bin 3: VBin3) — so the binned line time follows the binned V.
	v := int64(imx455SelectMode(int(bin), ModeOf(rm).BytesPerPx).v) // HMAX line-time base
	lineNs := v * 1_000_000 / int64(imx455ClkKHz)                   // V*1000/clock(kHz) = line time in ns (75750)

	us := d.Microseconds()
	if us < imx455ExpMinUs {
		us = imx455ExpMinUs
	}
	if us > imx455ExpMaxUs {
		us = imx455ExpMaxUs
	}

	// effHeight = output rows = full/bin, EXCEPT bin 4,
	// which reuses bin-2 geometry and scans 2× the output rows (output rows<<1) → full/2.
	// VMAX base = 52, a constant vblank (never rewritten per mode —
	// every reference is a load), so VMAX = 52 + effHeight at every bin.
	// effHeight = output rows. It follows the live ROI (binned px, set by SetROI) so VMAX/frame
	// shrink for a sub-frame window — the SDK does this (full-frame trigger VMAX 6592, but 2252
	// for a 2048-row ROI; wire-confirmed). Falls back to full/bin when no ROI is set.
	effH := int64(imx455FullHeight) / bin
	if h := ModeOf(rm).Height; h > 0 {
		effH = int64(h)
	}
	if bin == 4 {
		effH *= 2 // output rows<<1 = bin 4 reuses bin-2 geometry, scans 2× the output rows
	}
	defVMAX := int64(imx455VBlankFull16) + effH // vblank(52) + effHeight = one frame (6440 at full bin 1)
	frameUs := defVMAX * lineNs / 1000          // frame-readout time (487830 µs)

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
	// SHS readout halving: halve for bin 1 and bin 3, SKIP for bin 2
	// and bin 4 — the bin-mode field==0 (bin 1) falls into the halve; bin==2/4 branch around it;
	// bin 3 falls through to halve. When halving: shs = (shs < 6) ? 3 : shs>>1.
	if bin != 2 && bin != 4 {
		if shs >= 6 {
			shs >>= 1
		} else {
			shs = 3
		}
	}
	return WriteRegLE(rm, 0, []uint16{imx455RegSHS0, imx455RegSHS1}, uint32(shs))
}

// imx455SetROI — InitSensorMode (per-mode table) + SetStartPos + Cam_SetResolution.
// First it selects the readout mode from
// the binning (width vs full) and output depth (live ReadoutMode) and applies that mode's
// register table — the second init stage. Then the ROI start: X aligned to 16 (and
// #0x7ffffff0), shifted right 4, written to 0xa6/0xa7; Y (+0x19 bias) to 0x06/0x07; 0xa5
// the X/Y readout mode. The output window W→0x08/0x09 and H→0x18c/0x18d (+0x18 bias), 0x187
// the window mode. Finally the mode's V is written to the FPGA HMAX (HMAX = V;
// the FPS-percent throttle DDR path), straight to regs 0x13/0x14 — not the bandwidth formula.
func imx455SetROI(rm Regmap, x, y, w, h, bin int) error {
	if bin < 1 {
		bin = 1
	}
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	// Mode selection: apply this binning + depth's readout-mode register table.
	// (x,y,w,h) is the BINNED OUTPUT window — w,h are output pixels and the
	// window/FPGA-geometry registers take them directly (Cam_SetResolution writes
	// the output dims: WIDTH = w+0x18, HEIGHT = h). The START, by contrast, is in SENSOR
	// pixels: the SetStartPos clamp is bin·outputDim + start ≤ fullDim, so the
	// binned (x,y) is scaled up by bin for the start registers. Start X aligns to 16 always;
	// start Y aligns to 4 for bin 2/4 and to 2 for bin 1. bin-3 takes a
	// separate start-pos path — its OFFSET is unverified,
	// but a full-frame bin-3 readout starts at (0,0) where alignment is moot.
	//
	// NOT wired: SetFPGABinDataLen (FX3 reg 0x40 = output_area·bpp/4). The FrameBytes-
	// driven windowed pump + FPGABufReload flush carry single-frame framing without it; it may
	// be needed for clean repeated binned/sub-frame readout — confirm on the 455 before adding
	// it to the validated transport.
	mode := imx455SelectMode(bin, ModeOf(rm).BytesPerPx)
	for _, rv := range mode.table {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}

	sx := x * bin // binned → sensor pixels (SetStartPos coordinate system)
	sy := y * bin
	sx &^= 0xf                // align X to 16
	yBias := imx455YStartBias // 0x19 for bin 1/2/4
	switch bin {
	case 2, 4:
		sy &^= 3 // align Y to 4
	case 3:
		// bin-3 takes a separate start path: align Y down to a multiple
		// of 6 and use Y bias 0x1b. Only affects a bin-3 SUB-frame;
		// full-frame starts at 0 where both this and align-2 give 0. The sensor/output
		// coordinate basis of the sub-frame offset is UNVERIFIED (no 6200 bin-3 crop on the wire).
		sy = (sy / 6) * 6
		yBias = 0x1b
	default:
		sy &^= 1 // align Y to 2 (bin 1)
	}

	winMode := uint16(4) // 0x187 = 4 normally
	heightAdj := h
	if bin == 3 {
		winMode = 0       // bin-3: window mode 0
		heightAdj = h + 2 // bin-3: HEIGHT + 2
	}

	xShift := uint16(sx>>4) & 0xffff
	uy := uint16(sy+yBias) & 0xffff
	uw := uint16(w+0x18) & 0xffff    // output WIDTH reg = width + 0x18
	uh := uint16(heightAdj) & 0xffff // output HEIGHT reg

	writes := []RegVal{
		// Cam_SetResolution mode + window mode select.
		{Reg: imx455RegWinMode, Val: winMode},
		{Reg: imx455RegStartMode, Val: 1}, // WriteSONYREG(5,1)/start-mode = 1
		// ROI start X (sensorX>>4) and Y (sensorY+bias).
		{Reg: imx455RegStartXL, Val: xShift & 0xff},
		{Reg: imx455RegStartXH, Val: (xShift >> 8) & 0xff},
		{Reg: imx455RegStartYL, Val: uy & 0xff},
		{Reg: imx455RegStartYH, Val: (uy >> 8) & 0xff},
		// Output window — HEIGHT to 0x08/0x09, WIDTH(+0x18) to 0x18c/0x18d (see consts).
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
	// FPGA frame geometry: SetFPGAWidth → 0x04/0x05 = output
	// width, SetFPGAHeight → 0x08/0x09 = output height. These size the FX3 transfer.
	if err := FPGAWrite16(rm, 0x04, 0x05, uint16(w)); err != nil {
		return err
	}
	if err := FPGAWrite16(rm, 0x08, 0x09, uint16(h)); err != nil {
		return err
	}
	// SetFPGABinDataLen: the per-frame DMA word count =
	// output_area · bytesPerPx / 4 (FPGA 0x40..0x43). For full-frame bin 1 this equals the
	// value already in place, so the validated full-frame path is unchanged; for binned /
	// sub-frame it sizes the FX3 transfer to the smaller frame.
	bpp := ModeOf(rm).BytesPerPx
	dataWords := uint32((w*h*bpp + 3) / 4)
	if err := SetFPGABinDataLen(rm, dataWords); err != nil {
		return err
	}
	// the FPS-percent throttle -> SetFPGAHMAX: the 6200 takes the DDR branch (the DDR flag!=0, since
	// InitCamera enables DDR), so HMAX = the per-mode V written straight to
	// 0x13/0x14 — NOT the bandwidth-throttle formula. The binned modes carry their own V
	// (VBin2/VBin3), so this also sets the binned line time.
	return FPGAWrite16(rm, 0x13, 0x14, uint16(mode.v))
}

// IMX455 (ASI6200) per-mode register tables — the second stage of the two-stage init.
// InitSensorMode picks ONE of these by binning (the bin factor) and output depth, applies
// it via WriteSONYREG, and sets the timing base V. All tables are byte-identical in the
// MM_Pro variant.
//
// Mode -> table (InitSensorMode bin branches):
//
//	bin 1, 16-bit : reg_full_16bit   (76 entries)   — the streaming default (imx455Init)
//	bin 1, 12/8-bit: reg_full_12bit  (77)
//	bin 2         : reg_bin2w_12bit  (77)
//	bin 4         : reg_bin2w_12bit  (reused)
//	bin 3         : reg_bin3w_12bit  (77)
//
// SEAM: the asicam Sensor model has no mode-selection hook (Init + SetROI only), so only
// the bin-1 16-bit default is currently applied (imx455Init = imx455InitCommon +
// imx455ModeFull16). The other three are captured here for when ROI/bin/depth selection
// is wired into SetROI. Each also implies its own V (InitSensorMode sets a different V per
// mode — full-16bit V=0x5eb, others differ).

// reg_full_16bit — bin 1, 16-bit (full-frame default).
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

// reg_full_12bit — bin 1, 12/8-bit.
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

// reg_bin2w_12bit — bin 2 (and reused for bin 4).
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

// reg_bin3w_12bit — bin 3. (Differs from bin2w only at 0x0001/0x0006 + the
// 0x0a8x command block — the rest is identical.)
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
