// Register/operation map:
//
//	InitCamera          (reglist loop, len 0x7c=31; explicit tail; FPGA bringup, firmware-subtype >=0x12 gated)
//	Cam_SetResolution   (VMAX 0x217/0x218 = h·bin+0x26; window W 0x305/6 H 0x307/8; FPGA HBLK0/VBLK0xb/W/H)
//	SetStartPos         (ROI X 0x301/0x302, Y 0x303/0x304; X align 4, Y align 2; latch 0x20c)
//	SetGain             (clamp 0..0x190; 16-bit code -> 0x404/0x405; latch 0x20c; no scaling/HCG)
//	SetExp              (SHS -> 0x29a/0x29b; VMAX via SetFPGAVMAX + sensor 0x217/0x218; long-exp 0x22a=1/0x25c=0xff)
//	SetCMOSClk()        (pixel clock = 0x4e20 = 20000; master 0x1220a = 74250)
//	capture worker      (stream gate: master 0x200 = 1 stop / 0 start)
//	reglist             (31 reg/val16 entries)
package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
	"time"
)

// Sony IMX174 — 1936×1216 global-shutter CMOS (ZWO ASI174 family). Registers in the
// 0x2xx–0x8xx space; coupled writes bracketed by a 1-then-0 latch on 0x20c; frame length
// VMAX = height·bin + 0x26 (sensor 0x217/0x218 + FPGA via SetFPGAVMAX). Firmware-subtype
// >= 0x12 (modern all-FPGA path).
const (
	imx174RegLatch = 0x20c // coupled write groups bracketed by 1 then 0

	// SetGain — 16-bit analog gain code, little-endian to 0x404/0x405. No
	// conversion-gain bit, no scaling.
	imx174RegGainL = 0x404
	imx174RegGainH = 0x405

	// Cam_SetResolution — VMAX (frame length) = height·bin + 0x26, 16-bit.
	imx174RegVMAXL = 0x217
	imx174RegVMAXH = 0x218

	// SetExp — shutter (SHS) line count, 16-bit little-endian to 0x29a/0x29b.
	imx174RegSHSL = 0x29a
	imx174RegSHSH = 0x29b

	// SetStartPos — ROI offset; X aligned to 4, Y aligned to 2 before write.
	imx174RegStartXL = 0x301
	imx174RegStartXH = 0x302
	imx174RegStartYL = 0x303
	imx174RegStartYH = 0x304

	// Cam_SetResolution — output window size × binning.
	imx174RegWidthL  = 0x305
	imx174RegWidthH  = 0x306
	imx174RegHeightL = 0x307
	imx174RegHeightH = 0x308

	imx174RegMaster = 0x200 // sensor stream gate: 1 stop / 0 start

	imx174GainMax    = 400        // 40.0 dB ceiling, ASI 0.1 dB units (SetGain clamp = 0x190)
	imx174ExpMinUs   = 32         // µs floor (SetExp: exp <= 31 -> 32)
	imx174ExpMaxUs   = 2000000000 // 2000 s ceiling (SetExp = 0x77359400)
	imx174VMAXOffset = 0x26       // VMAX = height·bin + 0x26 (Cam_SetResolution)
	imx174SHSFloor   = 0xa        // SHS clamped >= 0xa

	// Line time = HMAX*1000/clock (clock = 20000, SetCMOSClk). HMAX recomputed by
	// imx174HMAX via HMAXBW with: floor 780 (0x30c), H-term (height + 0x26), and the
	// USB2/USB3 bandwidth constants below. USB2 RAW16 full-frame at FPSPercent 40 = 4337.
	imx174ClkKHz    = 20000 // pixel clock in kHz (SetCMOSClk = 0x4e20)
	imx174HMAXFloor = 780   // 0x30c: HMAX floor
	imx174HMAXFull  = 4337  // 0x10f1: USB2 full-frame HMAX @ bandwidth 40 (reference)
	imx174FPGAHMAXL = 0x13  // SetFPGAHMAX -> FPGA 0x13/0x14
	imx174FPGAHMAXH = 0x14
	imx174BWUSB2    = 43272000.0  // HMAXBW USB2 bandwidth const 0x4c2511d0 (universal)
	imx174BWUSB3    = 385000000.0 // HMAXBW USB3 const 0x4db79512 (174-specific; != package bwUSB3)
)

// imx174HMAX computes the FPGA HMAX for a window from the live ReadoutMode (USB speed,
// output depth, FPSPercent) via HMAXBW with the 174's constants. USB2 RAW16 full-frame at
// FPSPercent=40 == imx174HMAXFull (4337).
func imx174HMAX(rm Regmap, w, h int) uint16 {
	return HMAXBW(w, h, imx174ClkKHz, imx174HMAXFloor, imx174VMAXOffset, imx174BWUSB2, imx174BWUSB3, ModeOf(rm))
}

// imx174LineTimeNs is the readout line time (ns) for the full-frame window: HMAX*1e6/clock.
func imx174LineTimeNs(rm Regmap) uint64 {
	return uint64(imx174HMAX(rm, imx174FullWidth, imx174FullHeight)) * 1_000_000 / imx174ClkKHz
}

// imx174Init is InitCamera: the 31-entry reglist (loop over 0x7c bytes; reg 0xffff =
// InitDelayReg, delay = val ms) then the explicit WriteSONYREG tail. FPGA bringup is in
// imx174InitFPGA; SendCMD(0xAE) is Camera-level.
var imx174Init = []RegVal{
	// --- reglist (31 entries) ---
	{Reg: 0x200, Val: 0x01}, {Reg: 0xffff, Val: 20}, // delay 20 ms
	{Reg: 0x228, Val: 0x30}, {Reg: 0x292, Val: 0x20}, {Reg: 0x293, Val: 0x00}, {Reg: 0x294, Val: 0x20},
	{Reg: 0x295, Val: 0x00}, {Reg: 0x2a0, Val: 0xa4}, {Reg: 0x2a5, Val: 0x08}, {Reg: 0x2bc, Val: 0x10},
	{Reg: 0x2be, Val: 0x45}, {Reg: 0x2bf, Val: 0x20}, {Reg: 0x2c0, Val: 0x02}, {Reg: 0x2d7, Val: 0x00},
	{Reg: 0x412, Val: 0x20}, {Reg: 0x413, Val: 0x20}, {Reg: 0x41a, Val: 0x08}, {Reg: 0x567, Val: 0x04},
	{Reg: 0x568, Val: 0x11}, {Reg: 0x56c, Val: 0x05}, {Reg: 0x573, Val: 0x0c}, {Reg: 0x575, Val: 0x0f},
	{Reg: 0x58f, Val: 0x7c}, {Reg: 0x7b7, Val: 0x04}, {Reg: 0x7c5, Val: 0x85}, {Reg: 0x7d5, Val: 0x5a},
	{Reg: 0x825, Val: 0x10}, {Reg: 0x82b, Val: 0xe0}, {Reg: 0x82c, Val: 0x0a}, {Reg: 0x830, Val: 0xaf},
	{Reg: 0x831, Val: 0x10},
	// --- explicit WriteSONYREG tail (InitCamera) ---
	{Reg: 0x2a9, Val: 0x30}, {Reg: 0x2c2, Val: 0xa0}, {Reg: 0x205, Val: 0x20}, {Reg: 0x21c, Val: 0x41},
	{Reg: 0x214, Val: 0x01}, {Reg: 0x300, Val: 0x03}, {Reg: 0x56a, Val: 0x21}, {Reg: 0x586, Val: 0x68},
	{Reg: 0x587, Val: 0x10}, {Reg: 0x5a8, Val: 0x31}, {Reg: 0x62a, Val: 0x90}, {Reg: 0x62b, Val: 0x51},
	{Reg: 0x62c, Val: 0xc9}, {Reg: 0x64c, Val: 0xa0}, {Reg: 0x652, Val: 0x90}, {Reg: 0x655, Val: 0xb0},
	{Reg: 0x7b1, Val: 0x26}, {Reg: 0x213, Val: 0x00},
}

var IMX174 = Sensor{
	Name:     "IMX174", // Sony IMX174 global-shutter CMOS (mono die; MC adds a CFA)
	GainMax:  imx174GainMax,
	ExpMinUs: imx174ExpMinUs,
	ExpMaxUs: imx174ExpMaxUs,
	// ASI Brightness / black level. Caps: 0..240, def 1.
	OffsetMax: 240, OffsetDef: 1,
	Info: CameraInfo{
		MaxWidth:  1936,     // datasheet / SDK MaxWidth
		MaxHeight: 1216,     // datasheet / SDK MaxHeight (= output rows)
		PixelUm:   5.86,     // 5.86 µm pitch (datasheet / SDK PixelSize)
		BitDepth:  12,       // 12-bit ADC
		Bayer:     "RGGB",   // CFA of the color (ASI174MC) variant; surfaced when Model.Color
		Bins:      []int{1}, // no hardware binning: every bin use is a geometry multiplier, no
		// binning-enable register (cf. the 290's 0x3006=0x22). See imx174SetROI.
	},
	Init:        imx174Init,
	InitFPGA:    imx174InitFPGA,
	SetGain:     imx174SetGain,
	SetExposure: imx174SetExposure,
	SetOffset:   imx174SetOffset,
	SetROI:      imx174SetROI,
	StreamStop:  func(rm Regmap) error { return rm.WriteReg(imx174RegMaster, 1) }, // master stop
	StreamStart: func(rm Regmap) error { return rm.WriteReg(imx174RegMaster, 0) }, // master start
	Worker:      imx174Worker,
}

// imx174Worker is the host-timed single-shot capture worker. It arms the sensor, integrates
// (host-timed only in the >4 s trigger band), reads the frame via the async windowed bulk pump,
// and halts the readout pipeline before returning. A short/failed read is returned to the caller
// (GetDataAfterExp decides dead vs transient via the firmware check); the worker neither retries
// nor resets the device.
//
// Post-read halt prevents the sustained-streaming FX3 firmware crash (leaving the sensor
// streaming backs up the FX3 until its firmware crashes at ~2-5k frames). The concurrency wedge
// (a control transfer landing mid-readout) is prevented upstream by the transport's
// control-vs-bulk gate.
//
//	arm:   SendCMD(0xAA)·FPGAStop·0x212=1·0x200=1·SendCMD(0xA9)·0x200=0·10ms·FPGAStart·0x212=0·50ms·0x22e=0x0a
//	integrate: trigger band only — EnableFPGATriggerSignal(1)·poll-sleep until elapsed·(0)·release
//	read:  ctl.BulkRead = async urb pump w/ exact-remainder tail (gets the FX3 held tail); no FPGABufReload
//	stop:  after a good read — FPGAStop·master-stop(0x200=1)·SendCMD(0xAA): halts free-run
//
// errExposureAborted is returned by a capture worker when StopExposure interrupts it mid-flight,
// so the driver can drop the discarded frame and the abort path returns promptly.
var errExposureAborted = fmt.Errorf("exposure aborted")

func imx174Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()
	// FPGA reg0 bit4 = readout stop (FPGAStop/Start); reg 0x0b bit0 = the trigger signal.
	fpgaBit := func(reg uint16, bit uint16, set bool) error {
		v, err := rm.ReadFPGAReg(reg)
		if err != nil {
			return err
		}
		nv := v &^ bit
		if set {
			nv |= bit
		}
		return rm.WriteFPGAReg(reg, nv&0xff)
	}
	fpgaStop := func() error { return fpgaBit(0x00, 0x10, true) }
	fpgaStart := func() error { return fpgaBit(0x00, 0x10, false) }
	trigger := func(on bool) error { return fpgaBit(0x0b, 0x01, on) } // EnableFPGATriggerSignal

	// armSensor: sensor-side toggle (0x212 + master 0x200) + FPGAStart + settle (0x212=0,
	// 50 ms, 0x22e=0x0a). full=true is arm-1 (issues the FX3 stop/start); full=false is the
	// re-arm (arm-2): sensor toggle + FPGAStart only.
	armSensor := func(full bool) error {
		if full {
			if err := ctl.VendorCmd(0xAA); err != nil {
				return err
			}
			if err := fpgaStop(); err != nil {
				return err
			}
		}
		if err := rm.WriteReg(0x212, 1); err != nil {
			return err
		}
		if err := rm.WriteReg(imx174RegMaster, 1); err != nil { // master stop
			return err
		}
		if full {
			if err := ctl.VendorCmd(0xA9); err != nil {
				return err
			}
		}
		if err := rm.WriteReg(imx174RegMaster, 0); err != nil { // master start
			return err
		}
		time.Sleep(10 * time.Millisecond) // usleep(0x2710)
		if err := fpgaStart(); err != nil {
			return err
		}
		if err := rm.WriteReg(0x212, 0); err != nil {
			return err
		}
		time.Sleep(50 * time.Millisecond) // usleep(0xc350)
		return rm.WriteReg(0x22e, 0x0a)
	}

	triggerBand := exposure.Microseconds() >= imx174TriggerUs

	// expose arms the sensor and, for the >4 s trigger band, opens/closes the host
	// integration window: EnableFPGATriggerSignal(1) opens integration, host waits it out,
	// EnableFPGATriggerSignal(0) releases the held frame. The ≤4 s bands are sensor-timed
	// (self-times on-chip or free-runs): no trigger signal, no host window — the read pulls
	// the delivered frame. full selects arm-1 vs the arm-2 re-arm.
	expose := func(full bool) error {
		if err := armSensor(full); err != nil {
			return err
		}
		_ = ctl.ResetEndpoint()
		if triggerBand {
			if err := trigger(true); err != nil { // EnableFPGATriggerSignal(1)
				return err
			}
			start := time.Now()
			for time.Since(start) < exposure {
				if ctl.Aborted() {
					// StopExposure ran: bail, dropping the trigger signal on the way out —
					// left asserted, the next trigger(true) is a no-edge write and the FPGA
					// never gates the integration. (Host-side hygiene, not object-derived:
					// the SDK snap thread never host-aborts mid-integration.)
					_ = trigger(false)
					return errExposureAborted
				}
				time.Sleep(100 * time.Millisecond)
			}
			_ = ctl.ResetEndpoint()
			if err := trigger(false); err != nil { // EnableFPGATriggerSignal(0) = release
				return err
			}
		}
		return nil
	}

	// readFrame pulls one whole frame via BulkRead, the async urb pump (readFrameURBs):
	// submits the whole frame's transfers up front (sustains the trigger band's fast
	// post-release burst) and sizes the last transfer to the exact remainder, which makes the
	// FX3 deliver its held final partial DMA buffer (a padded 1-MiB request returns ~4 MiB
	// short). No FPGABufReload: the SDK never touches reg 0x18 during readout in any band.
	target := ctl.FrameBytes()
	if target > len(buf) {
		target = len(buf)
	}
	// readTimeout: the trigger band already integrated in expose(), so the read is just the
	// readout; the ≤4 s bands are sensor-timed DURING the read, so it must cover the exposure.
	// In those sensor-timed bands the first ~exposure of the read is pure integration, so it
	// is declared as the BulkReadQuiet quiet window: the transfers stay armed (the GPIF must
	// never stream without a reader) but the control-transfer gate engages only near the end
	// of the exposure — TEC polls, telemetry and ST4 pulses flow instead of blocking for the
	// whole sub. 500 ms undershoot so the gate is in place before data can arrive.
	readTimeout := 3 * time.Second
	quiet := time.Duration(0)
	if !triggerBand {
		readTimeout = exposure + 3*time.Second
		if q := exposure - 500*time.Millisecond; q > 0 {
			quiet = q
		}
	}
	readFrame := func() (int, error) { return ctl.BulkReadQuiet(buf[:target], quiet, readTimeout) }

	// Halt the readout pipeline on EVERY return — the 174 object's WorkingFunc exit:
	// StopSensorStreaming (0x212=1 → master 0x200=1 → FPGAStop, per 0054_CCameraS174MM_Mini.o)
	// then SendCMD(0xAA) then ResetEndPoint. A free-running sensor piles undrained frames into
	// the FX3 GPIF/DMA until the firmware crashes (~2-5k frames). Replaces the earlier
	// success-path-only inline halt (fpgaStop → 0x200=1 → 0xAA, no 0x212), which predated the
	// object exit decode. Best-effort: a failed stop must not fail a good frame.
	defer func() {
		_ = rm.WriteReg(0x212, 1)
		_ = rm.WriteReg(imx174RegMaster, 1) // sensor master stop
		_ = fpgaStop()                      // FPGA reg0 bit4 = 1 (readout-stop)
		_ = ctl.VendorCmd(0xAA)             // SendCMD stream-stop
		_ = ctl.ResetEndpoint()
	}()

	// Integrate once, then read once. A short/failed read is returned to GetDataAfterExp, whose
	// firmware-version check decides dead (ErrDeviceWedged) vs a transient short — an in-worker
	// clear-pipe + re-arm retry never recovered anything in soak testing.
	if err := expose(true); err != nil {
		return 0, err
	}
	n, err := readFrame()
	if err == nil && n >= target {
		return n, nil
	}
	if ctl.Aborted() {
		// StopExposure broke the read (AbortRead): a clean abort, not a stall — surface it
		// as one so GetDataAfterExp doesn't mislabel the status or probe the firmware.
		return n, errExposureAborted
	}
	ctl.NoteStall() // short/failed read (not an abort) — for the soak StallCount diagnostic
	return n, err
}

// imx174InitFPGA is the modern (firmware-subtype >= 0x12) FPGA bringup InitCamera performs
// after the Sony tail, using the FX3 register numbers. FPGASetBits/FPGAClearBits/FPGAWriteBits
// do the reg0/reg0xa RMWs.
//
//	FPGAReset                          reg0 bit0 -> 0
//	usleep(20 ms)
//	firmware-subtype >= 0x12 ? modern : legacy (cmp #0x11, b.ls)
//	WriteSONYREG(0x212, 1)
//	WriteSONYREG(0x22e, 0)
//	SetFPGAAsMaster(1)                 reg0 bit5 = 1
//	FPGAStop                           reg0 bit4 = 1
//	EnableFPGADDR(0)                   reg0xa bit6 = 1
//	SetFPGAADCWidthOutputWidth(1, 0)   reg0xa bit0=1, bit4=0
//	SetFPGAGain(0x80,0x80,0x80,0x80)   FPGA 0x0c-0x0f, strobed
//
// Camera.Init then issues SendCMD(0xAE). A second SetFPGAADCWidthOutputWidth(1, output depth)
// follows the bank-select: outputWidth = bit-depth flag (1 for RAW16). Replicated here,
// parameterized for RAW8.
func imx174InitFPGA(rm Regmap, subtype int) error {
	if subtype < 0x12 {
		// Legacy <0x12 path not reproduced (this camera is >=0x12). It skips the writes
		// below and pokes FPGA 0x0c-0x0f raw + reg1/reg0xa.
		_ = subtype
	}
	if err := FPGAClearBits(rm, 0x00, 0x01); err != nil { // FPGAReset: reg0 bit0
		return err
	}
	time.Sleep(20 * time.Millisecond) // usleep(0x4e20)
	if err := rm.WriteReg(0x212, 0x01); err != nil {
		return err
	}
	if err := rm.WriteReg(0x22e, 0x00); err != nil {
		return err
	}
	if err := FPGASetBits(rm, 0x00, 0x20); err != nil { // SetFPGAAsMaster(1): reg0 bit5
		return err
	}
	if err := FPGASetBits(rm, 0x00, 0x10); err != nil { // FPGAStop: reg0 bit4
		return err
	}
	if err := FPGASetBits(rm, 0x0a, 0x40); err != nil { // EnableFPGADDR(0): reg0xa bit6
		return err
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x11, 0x01); err != nil { // SetFPGAADCWidthOutputWidth(1,0)
		return err
	}
	// SetFPGAGain(0x80×4): FPGA 0x0c-0x0f, committed by the reg-1 strobe.
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
	// Second SetFPGAADCWidthOutputWidth(1, output depth) after SendCMD(0xAE): output width =
	// bit depth. bit4 derived from the live ReadoutMode (1 for RAW16) so RAW8 isn't forced to
	// 16-bit (matches imx290InitFPGA).
	adcOut := uint16(0x01) // bit0 = adc
	if ModeOf(rm).BytesPerPx >= 2 {
		adcOut |= 0x10 // bit4 = 1: 16-bit output (RAW16)
	}
	return FPGAWriteBits(rm, 0x0a, 0x11, adcOut)
}

// imx174SetGain — SetGain. Clamps the ASI gain to [0, 400] and writes it to the 16-bit gain
// code 0x404/0x405 — no conversion-gain bit, no curve. Latched by 0x20c (1 before, 0 after).
// imx174SetOffset — SetBrightness (ASI Brightness / black level): offset written 16-bit
// little-endian to sensor 0x458 (low) / 0x459 (high), no scaling.
func imx174SetOffset(rm Regmap, offset int) error {
	v := uint16(offset)
	if err := rm.WriteReg(0x459, (v>>8)&0xff); err != nil {
		return err
	}
	return rm.WriteReg(0x458, v&0xff)
}

func imx174SetGain(rm Regmap, gain int) error {
	if gain > imx174GainMax {
		gain = imx174GainMax
	}
	if gain < 0 {
		gain = 0
	}
	g := uint16(gain)
	return WriteRegLE(rm, imx174RegLatch, []uint16{imx174RegGainL, imx174RegGainH}, uint32(g))
}

const (
	imx174FullWidth  = 1936    // output width at full-frame bin 1 (= MaxWidth)
	imx174FullHeight = 1216    // output rows at full-frame bin 1 (= MaxHeight)
	imx174TriggerUs  = 4000000 // >= 4 s -> FPGA trigger-mode band (SetExp = 0x3d0900)
	imx174TrigHMAX   = 0x1500  // SetFPGAHMAX value in the trigger band (SetExp)
)

// imx174SetExposure — SetExp (modern firmware-subtype >=0x12 path).
// Global-shutter, three exposure bands:
//
//	lineTime  = HMAX*1000/clock (≈ 216.85 µs USB2 full-res)
//	VMAX      = max(height+0x26, lines+0xa)   // FPGA frame length (SetFPGAVMAX; modern uses FPGA, not 0x217/0x218)
//	SHS       = VMAX - lines, floor 0xa        // -> 0x29a/0x29b
//	EnableFPGAWaitMode(1) every exposure (SetExp)
//
//	short  (exp <= frameTime + 0.1 s):  0x22a = 0
//	long   (exp >  frameTime + 0.1 s):  cycle-count mode — FRAME = height+0x26 and
//	                                    LINES = VMAX-0x12 as 24-bit LE to 0x244-0x249 (dup 0x24a-0x24f),
//	                                    0x25c = 0xff, 0x22a = 1
//	trigger(exp >= 4 s):                additionally EnableFPGATriggerMode(1) (reg0 bit7) +
//	                                    SetFPGAHMAX(0x1500)
//
// The long-frame threshold is SetExp's `exp > frameTime + 0x186a0` (0x186a0 = 100000 µs,
// frameTime = one default frame time). Line time follows the live HMAX (imx174HMAX): at the
// USB2 default it is 216.85 µs.
func imx174SetExposure(rm Regmap, d time.Duration) error {
	us := uint64(d.Microseconds())
	trigger := us >= imx174TriggerUs // >= 4 s: FPGA hardware-trigger band

	// Normal readout HMAX, recomputed for the live mode (USB speed / depth / bandwidth%).
	// In the trigger band HMAX flips to 0x1500, changing the line time (SetExp writes
	// HMAX=0x1500 before the line-time read), so the VMAX/SHS math below uses the trigger
	// line time, not the normal one.
	hmaxNormal := imx174HMAX(rm, imx174FullWidth, imx174FullHeight)
	normalLineTimeNs := uint64(hmaxNormal) * 1_000_000 / imx174ClkKHz
	lineTimeNs := normalLineTimeNs
	if trigger {
		lineTimeNs = uint64(imx174TrigHMAX) * 1_000_000 / imx174ClkKHz // 0x1500 -> 268800 ns/line
	}
	lines := ExposureLines(d, lineTimeNs, imx174ExpMinUs, imx174ExpMaxUs)

	defaultVMAX := uint64(imx174FullHeight) + imx174VMAXOffset // height + 0x26
	vmax := defaultVMAX
	if lines+imx174SHSFloor > vmax {
		vmax = lines + imx174SHSFloor // grow the frame to fit the exposure
	}
	shs := vmax - lines
	if shs < imx174SHSFloor {
		shs = imx174SHSFloor
	}

	// long-frame (cycle-count) band: exp exceeds one default (normal-readout) frame + 0.1 s
	// (SetExp: exp > frameTime + 0x186a0; frameTime = the normal frame time).
	frameTimeUs := defaultVMAX * normalLineTimeNs / 1000
	longFrame := us > frameTimeUs+100000

	// EnableFPGAWaitMode(1): reg0 bit6, set per exposure (InitCamera never calls WaitMode).
	if err := FPGASetBits(rm, 0x00, 0x40); err != nil {
		return err
	}
	// HMAX: 0x1500 only in the >4 s trigger band; normal readout HMAX otherwise.
	hmax := hmaxNormal
	if trigger {
		hmax = imx174TrigHMAX
	}
	if err := FPGAWrite16(rm, imx174FPGAHMAXL, imx174FPGAHMAXH, hmax); err != nil {
		return err
	}
	// EnableFPGATriggerMode (reg0 bit7) only in the >4 s trigger band (gated at 4 s for modern
	// firmware). The 0.4–4 s cycle-count band uses WaitMode only — the sensor self-times the
	// integration on-chip (LINES line-periods at the normal line rate), the FPGA holds the
	// frame. No trigger mode, no trigger signal, no host integration window (SDK 2 s wire has
	// zero reg 0x0b writes).
	if trigger {
		if err := FPGASetBits(rm, 0x00, 0x80); err != nil { // EnableFPGATriggerMode(1)
			return err
		}
	} else {
		if err := FPGAClearBits(rm, 0x00, 0x80); err != nil { // EnableFPGATriggerMode(0)
			return err
		}
	}

	if err := SetVMAX(rm, uint32(vmax)); err != nil { // modern: VMAX -> FPGA 0x10/0x11/0x12
		return err
	}
	return WithLatch(rm, imx174RegLatch, func() error {
		if longFrame {
			// Sensor long-exposure (cycle-count) mode (SetExp): FRAME = height+0x26,
			// LINES = VMAX-0x12 (24-bit LE), each written twice (0x244-0x249 then 0x24a-0x24f).
			frame := uint32(defaultVMAX)
			lns := uint32(vmax - 0x12)
			for _, regs := range [][]uint16{
				{0x244, 0x245, 0x246}, {0x247, 0x248, 0x249},
				{0x24a, 0x24b, 0x24c}, {0x24d, 0x24e, 0x24f},
			} {
				v := frame
				if regs[0] == 0x247 || regs[0] == 0x24d {
					v = lns
				}
				if err := WriteRegLE(rm, 0, regs, v); err != nil {
					return err
				}
			}
			if err := rm.WriteReg(0x25c, 0xff); err != nil {
				return err
			}
		}
		// 0x22a selects sensor timing: 1 = long-exposure (cycle-count) mode, 0 = normal.
		longBit := uint16(0)
		if longFrame {
			longBit = 1
		}
		if err := rm.WriteReg(0x22a, longBit); err != nil {
			return err
		}
		return WriteRegLE(rm, 0, []uint16{imx174RegSHSL, imx174RegSHSH}, uint32(shs))
	})
}

// imx174SetROI — SetStartPos (X->0x301/0x302, Y->0x303/0x304; X aligned to 4, Y to 2) plus
// Cam_SetResolution (window W->0x305/0x306, H->0x307/0x308, VMAX = h·bin+0x26 -> 0x217/0x218;
// FPGA HBLK=0, VBLK=0xb, Width, Height). Sensor group latched by 0x20c. FPGA geometry is the
// modern (>=0x12) path.
//
// Binning is decoded but not wired (errors below): this firmware has no pixel-binning step.
// Every bin use is a geometry multiplier, with no bin-gated mode register (no analog of the
// 290's 0x3006=0x22). Cam_SetResolution scales the geometry by bin: HEIGHT(0x307/8)=bin·height,
// WIDTH(0x305/6)=bin·width, VMAX(0x217/8)=bin·height+0x26, start·bin (SetStartPos), and the FX3
// output frame SetFPGAHeight/Width(0x498/0x4ac)=bin·height/width. So bin N reads a bin×-larger
// window at full resolution (region/FOV scale, not true binning); a binned capture would
// short-read under Camera.FrameBytes (output dims).
func imx174SetROI(rm Regmap, x, y, w, h, bin int) error {
	if bin > 1 {
		return fmt.Errorf("imx174: bin %d not wired — this firmware has no pixel-binning step (every "+
			"bin use is a geometry multiplier, no binning-enable register), so bin>1 just reads a "+
			"bin×-larger window at full res, not a true binned frame", bin)
	}
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	x &^= 3 // align to 4 (#0x7ffffffc)
	y &^= 1 // align to 2 (#0x7ffffffe)
	ux, uy := uint16(x), uint16(y)
	uw, uh := uint16(w), uint16(h)
	vmax := uint16(h + imx174VMAXOffset) // bin 1

	if err := WithLatch(rm, imx174RegLatch, func() error {
		for _, rv := range []RegVal{
			{Reg: imx174RegVMAXL, Val: vmax & 0xff}, {Reg: imx174RegVMAXH, Val: (vmax >> 8) & 0xff},
			{Reg: imx174RegStartXL, Val: ux & 0xff}, {Reg: imx174RegStartXH, Val: (ux >> 8) & 0xff},
			{Reg: imx174RegStartYL, Val: uy & 0xff}, {Reg: imx174RegStartYH, Val: (uy >> 8) & 0xff},
			{Reg: imx174RegWidthL, Val: uw & 0xff}, {Reg: imx174RegWidthH, Val: (uw >> 8) & 0xff},
			{Reg: imx174RegHeightL, Val: uh & 0xff}, {Reg: imx174RegHeightH, Val: (uh >> 8) & 0xff},
		} {
			if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
				return err
			}
		}
		return nil
	}); err != nil {
		return err
	}
	// FPGA frame geometry (modern path; Cam_SetResolution): HBLK=0, VBLK=0xb, Width, Height.
	if err := ProgramFrameGeometry(rm, w, h, 0x00, 0x0b); err != nil {
		return err
	}
	// SetFPGAHMAX: recompute HMAX for this window from the live ReadoutMode (USB speed /
	// output depth / FPSPercent).
	return FPGAWrite16(rm, imx174FPGAHMAXL, imx174FPGAHMAXH, imx174HMAX(rm, w, h))
}
