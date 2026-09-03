// Sony IMX174: 1936×1216 global-shutter CMOS (ZWO ASI174 family, PlayerOne Apollo). Registers
// sit in the 0x2xx–0x8xx space; coupled writes are bracketed by the 0x20c latch; the firmware
// subtype is >= 0x12 (all-FPGA path). Wire-verified against the SDK on an ASI174MM Mini (USB2):
// RAW16 full frame and offset levels, host bin 2 at both depths, a 640×480 ROI (level, 49.5 fps
// stream, 2 s and 5 s bands), full-frame 100 ms / 2 s / 5 s and the 9.2 fps stream; the FX3
// brackets each frame with the 0x5A7E/0x3CF0 marker words (FX3DMAMarkers). The black level holds
// for one capture cycle, so the worker rewrites it on every arm. Profile entry points:
//
//	imx174Init, imx174InitFPGA  sensor and FPGA bringup
//	imx174SetROI                window start, window size, VMAX, FPGA geometry, HMAX
//	imx174SetGain               analog gain
//	imx174SetExposure           frame length and shutter position
//	imx174SetOffset             black level
//	imx174Worker                the single-shot capture worker

package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
	"time"
)

const (
	imx174RegLatch = 0x20c // coupled write groups: 1 before, 0 after

	// SetGain: 16-bit analog gain code, little-endian; no conversion-gain bit, no curve.
	imx174RegGainL = 0x404
	imx174RegGainH = 0x405

	// Window setup: VMAX (frame length) = height·bin + 0x26, 16-bit.
	imx174RegVMAXL = 0x217
	imx174RegVMAXH = 0x218

	// SetExp: shutter (SHS) line count, 16-bit little-endian.
	imx174RegSHSL = 0x29a
	imx174RegSHSH = 0x29b

	// Window start: ROI offset; X aligned to 4, Y to 2 before the write.
	imx174RegStartXL = 0x301
	imx174RegStartXH = 0x302
	imx174RegStartYL = 0x303
	imx174RegStartYH = 0x304

	// Window setup: output window size × binning.
	imx174RegWidthL  = 0x305
	imx174RegWidthH  = 0x306
	imx174RegHeightL = 0x307
	imx174RegHeightH = 0x308

	imx174RegMaster = 0x200 // sensor stream gate: 1 stop / 0 start

	imx174GainMax    = 400        // 40.0 dB, ASI 0.1 dB units (SetGain clamp 0x190)
	imx174ExpMinUs   = 32         // SetExp floor (exp <= 31 -> 32)
	imx174ExpMaxUs   = 2000000000 // 2000 s ceiling (SetExp 0x77359400)
	imx174VMAXOffset = 0x26       // VMAX = height·bin + 0x26
	imx174SHSFloor   = 0xa        // SHS clamped >= 0xa

	// Line time = HMAX·1000/clock. HMAX comes from HMAXBW (imx174HMAX) with the floor, the
	// H-term (height + 0x26) and the bandwidth constants below; USB2 RAW16 full-frame at
	// FPSPercent 40 gives 4337 (0x10f1), the SDK value.
	imx174ClkKHz    = 20000 // pixel clock in kHz (0x4e20; master 0x1220a = 74250)
	imx174HMAXFloor = 780   // 0x30c
	imx174FPGAHMAXL = 0x13  // SetFPGAHMAX -> FPGA 0x13/0x14
	imx174FPGAHMAXH = 0x14
	imx174BWUSB2    = 43272000.0  // HMAXBW USB2 bandwidth const 0x4c2511d0 (universal)
	imx174BWUSB3    = 385000000.0 // HMAXBW USB3 const 0x4db79512 (174-specific; != package bwUSB3)
)

// imx174HMAX computes the FPGA HMAX for a window from the live ReadoutMode (USB speed, output
// depth, FPSPercent) via HMAXBW with the 174 constants.
func imx174HMAX(rm Regmap, w, h int) uint16 {
	return HMAXBW(w, h, imx174ClkKHz, imx174HMAXFloor, imx174VMAXOffset, imx174BWUSB2, imx174BWUSB3, ModeOf(rm))
}

// imx174Init is the sensor-write half of camera bringup: the 31-entry reglist (reg 0xffff =
// InitDelayReg, delay = val ms) then the explicit WriteSONYREG tail. FPGA bringup is in
// imx174InitFPGA; the SDK's trailing SendCMD(0xAE) is not replayed.
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
	// --- explicit WriteSONYREG bringup tail ---
	{Reg: 0x2a9, Val: 0x30}, {Reg: 0x2c2, Val: 0xa0}, {Reg: 0x205, Val: 0x20}, {Reg: 0x21c, Val: 0x41},
	{Reg: 0x214, Val: 0x01}, {Reg: 0x300, Val: 0x03}, {Reg: 0x56a, Val: 0x21}, {Reg: 0x586, Val: 0x68},
	{Reg: 0x587, Val: 0x10}, {Reg: 0x5a8, Val: 0x31}, {Reg: 0x62a, Val: 0x90}, {Reg: 0x62b, Val: 0x51},
	{Reg: 0x62c, Val: 0xc9}, {Reg: 0x64c, Val: 0xa0}, {Reg: 0x652, Val: 0x90}, {Reg: 0x655, Val: 0xb0},
	{Reg: 0x7b1, Val: 0x26}, {Reg: 0x213, Val: 0x00},
}

// IMX174 is the Sony IMX174 global-shutter profile (ZWO ASI174 family, PlayerOne Apollo).
var IMX174 = Sensor{
	Name:          "IMX174", // mono die; MC adds a CFA
	TriggerBandUs: imx174TriggerUs,
	GainMax:       imx174GainMax,
	ExpMinUs:      imx174ExpMinUs,
	ExpMaxUs:      imx174ExpMaxUs,
	// ASI Brightness / black level: 0..240, default 1.
	OffsetMax: 240, OffsetDef: 1,
	Info: CameraInfo{
		MaxWidth:  1936,        // SDK MaxWidth
		MaxHeight: 1216,        // SDK MaxHeight (= output rows)
		PixelUm:   5.86,        // µm pitch
		BitDepth:  12,          // 12-bit ADC
		Bayer:     "RGGB",      // CFA of the color variant; surfaced when Model.Color
		Bins:      []int{1, 2}, // host-binned (the SDK's bin 2 on this die is a host bin; no HWBins)
	},
	Init:        imx174Init,
	InitFPGA:    imx174InitFPGA,
	SetGain:     imx174SetGain,
	SetExposure: imx174SetExposure,
	SetOffset:   imx174SetOffset,
	GetOffset:   imx174GetOffset,
	SetROI:      imx174SetROI,
	StreamStop:  func(rm Regmap) error { return rm.WriteReg(imx174RegMaster, 1) }, // master stop
	StreamStart: func(rm Regmap) error { return rm.WriteReg(imx174RegMaster, 0) }, // master start
	Worker:      imx174Worker,
	// FX3 marker words 0x5A7E/0x3CF0 in the first/last DMA word (wire-confirmed against the SDK
	// on an ASI174MM Mini).
	FX3DMAMarkers: true,
	ROIStartAlign: func(int) (int, int) { return 4, 2 }, // window-start masks
}

// errExposureAborted is returned by a capture worker when StopExposure interrupts it, so the
// driver drops the frame and the abort path returns promptly.
var errExposureAborted = fmt.Errorf("exposure aborted")

// imx174Worker is the host-timed single-shot capture worker. It arms the sensor, integrates
// (host-timed only in the >= 4 s trigger band), reads the frame with the async windowed bulk
// pump and halts the readout pipeline before returning. A short or failed read is returned to
// GetDataAfterExp, whose firmware check decides dead vs transient; the worker neither retries
// nor resets the device. The post-read halt is required: a sensor left streaming backs up the
// FX3 until its firmware crashes (~2-5k frames).
//
//	arm:       SendCMD(0xAA)·FPGAStop·0x212=1·0x200=1·SendCMD(0xA9)·0x200=0·10ms·
//	           FPGAStart·0x212=0·50ms·0x22e=0x0a·ReapplyOffset (the black level holds for one
//	           capture cycle, so it is rewritten every arm)
//	integrate: trigger band only: EnableFPGATriggerSignal(1)·poll-sleep until elapsed·(0)
//	read:      BulkReadQuiet = async urb pump with exact-remainder tail; no FPGABufReload
//	stop:      0x212=1·0x200=1·FPGAStop·SendCMD(0xAA)·ResetEndPoint (the SDK's exit)
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

	// armSensor: sensor toggle (0x212 + master 0x200) + FPGAStart + settle (0x212=0, 50 ms,
	// 0x22e=0x0a). full=true also issues the FX3 stop/start; full=false is the re-arm.
	armSensor := func(full bool) error {
		if full {
			if err := ctl.VendorCmd(FX3StreamStop); err != nil {
				return err
			}
			if err := fpgaStop(); err != nil {
				return err
			}
		}
		// The black level (0x458/0x459) holds for one capture cycle on this die: a value set
		// before a capture is in the next frame, then the halt/re-arm leaves the register at 0
		// (wire-observed on an ASI174MM Mini; the SDK's frames keep the level). Rewrite it on
		// every arm.
		if err := ctl.ReapplyOffset(); err != nil {
			return err
		}
		if err := rm.WriteReg(0x212, 1); err != nil {
			return err
		}
		if err := rm.WriteReg(imx174RegMaster, 1); err != nil { // master stop
			return err
		}
		if full {
			if err := ctl.VendorCmd(FX3StreamStart); err != nil {
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

	// expose arms the sensor and, in the >= 4 s trigger band, opens and closes the host
	// integration window: EnableFPGATriggerSignal(1) opens integration, the host waits it out,
	// EnableFPGATriggerSignal(0) releases the held frame. The <= 4 s bands are sensor-timed
	// (on-chip self-timing or free-run): no trigger signal, no host window; the read pulls the
	// delivered frame.
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
					// StopExposure ran: drop the trigger signal on the way out. Left asserted,
					// the next trigger(true) is a no-edge write and the FPGA never gates the
					// integration. (The SDK never host-aborts mid-integration.)
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

	// readFrame pulls one whole frame via BulkReadQuiet, the async urb pump: it submits the
	// whole frame's transfers up front (sustains the trigger band's post-release burst) and
	// sizes the last transfer to the exact remainder, which makes the FX3 deliver its held final
	// partial DMA buffer (a padded 1-MiB request returns ~4 MiB short). No FPGABufReload: the SDK
	// never touches reg 0x18 during readout in any band.
	target := ctl.FrameBytes()
	if target > len(buf) {
		target = len(buf)
	}
	// readTimeout: the trigger band integrated in expose(), so the read is only the readout; the
	// <= 4 s bands integrate DURING the read, so it must cover the exposure. In those bands the
	// first ~exposure of the read is pure integration and is declared as the BulkReadQuiet quiet
	// window: the transfers stay armed (the GPIF must never stream without a reader) but the
	// control-transfer gate engages only near the end of the exposure, so TEC polls, telemetry
	// and ST4 pulses flow. The 500 ms undershoot puts the gate in place before data can arrive.
	readTimeout := 3 * time.Second
	quiet := time.Duration(0)
	if !triggerBand {
		readTimeout = exposure + 3*time.Second
		if q := exposure - 500*time.Millisecond; q > 0 {
			quiet = q
		}
	}
	readFrame := func() (int, error) { return ctl.BulkReadQuiet(buf[:target], quiet, readTimeout) }

	// Halt the readout on every return, as the SDK does on its way out:
	// StopSensorStreaming (0x212=1, master 0x200=1, FPGAStop), SendCMD(0xAA), ResetEndPoint.
	// Best-effort: a failed stop must not fail a good frame.
	defer func() {
		_ = rm.WriteReg(0x212, 1)
		_ = rm.WriteReg(imx174RegMaster, 1) // sensor master stop
		_ = fpgaStop()                      // FPGA reg0 bit4 = 1 (readout-stop)
		_ = ctl.VendorCmd(FX3StreamStop)    // SendCMD stream-stop
		_ = ctl.ResetEndpoint()
	}()

	// Integrate once, then read once. A short or failed read goes back to GetDataAfterExp, whose
	// firmware-version check decides dead (ErrDeviceWedged) vs a transient short; an in-worker
	// clear-pipe + re-arm retry does not recover it.
	if err := expose(true); err != nil {
		return 0, err
	}
	n, err := readFrame()
	if err == nil && n >= target {
		return n, nil
	}
	if ctl.Aborted() {
		// StopExposure broke the read (AbortRead): a clean abort, not a stall, so
		// GetDataAfterExp does not mislabel the status or probe the firmware.
		return n, errExposureAborted
	}
	ctl.NoteStall() // short/failed read (not an abort): StallCount diagnostic
	return n, err
}

// imx174InitFPGA is the FPGA bringup that follows after the Sony tail (firmware subtype
// >= 0x12), using the FX3 register numbers:
//
//	FPGAReset                          reg0 bit0 -> 0
//	usleep(20 ms)
//	WriteSONYREG(0x212, 1)
//	WriteSONYREG(0x22e, 0)
//	SetFPGAAsMaster(1)                 reg0 bit5 = 1
//	FPGAStop                           reg0 bit4 = 1
//	EnableFPGADDR(0)                   reg0xa bit6 = 1
//	SetFPGAADCWidthOutputWidth(1, 0)   reg0xa bit0=1, bit4=0
//	SetFPGAGain(0x80,0x80,0x80,0x80)   FPGA 0x0c-0x0f, strobed
//	SetFPGAADCWidthOutputWidth(1, d)   d = output depth (1 = RAW16), after SendCMD(0xAE)
//
// The SDK's SendCMD(0xAE) is not replayed; the legacy (< 0x12) path is not reproduced.
func imx174InitFPGA(rm Regmap, subtype int) error {
	if err := poaUnsupported(rm, "imx174", "FPGA bringup"); err != nil {
		return err
	}
	if subtype < 0x12 {
		// Legacy < 0x12 path (raw FPGA 0x0c-0x0f + reg1/reg0xa pokes) not reproduced.
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
	// Second SetFPGAADCWidthOutputWidth(1, output depth): bit4 from the live ReadoutMode (1 for
	// RAW16) so RAW8 is not forced to 16-bit.
	adcOut := uint16(0x01) // bit0 = adc
	if ModeOf(rm).BytesPerPx >= 2 {
		adcOut |= 0x10 // bit4 = 1: 16-bit output (RAW16)
	}
	return FPGAWriteBits(rm, 0x0a, 0x11, adcOut)
}

// imx174GetOffset reads the offset back from 0x458/0x459, the pair imx174SetOffset writes
// 16-bit little-endian with no scaling.
func imx174GetOffset(rm Regmap) (int, error) {
	v, err := ReadRegLE(rm, []uint16{0x458, 0x459})
	return int(v), err
}

func imx174SetOffset(rm Regmap, offset int) error {
	if err := poaUnsupported(rm, "imx174", "SetOffset"); err != nil {
		return err
	}
	v := uint16(offset)
	if err := rm.WriteReg(0x459, (v>>8)&0xff); err != nil {
		return err
	}
	return rm.WriteReg(0x458, v&0xff)
}

// imx174SetGain (SetGain) clamps the gain to [0, 400] and writes it as the 16-bit code under
// the 0x20c latch.
func imx174SetGain(rm Regmap, gain int) error {
	if err := poaUnsupported(rm, "imx174", "SetGain"); err != nil {
		return err
	}
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
	imx174FullWidth  = 1936    // output width at full-frame bin 1
	imx174FullHeight = 1216    // output rows at full-frame bin 1
	imx174TriggerUs  = 4000000 // >= 4 s: FPGA trigger-mode band (SetExp 0x3d0900)
	imx174TrigHMAX   = 0x1500  // SetFPGAHMAX value in the trigger band
)

// imx174SetExposure (SetExp, firmware subtype >= 0x12). Global shutter, three bands:
//
//	lineTime  = HMAX·1000/clock (HMAX from the live window; 0x1500 in the trigger band)
//	VMAX      = max(height+0x26, lines+0xa)   FPGA frame length via SetVMAX (not 0x217/0x218);
//	                                          height = the live window height
//	SHS       = VMAX - lines, floor 0xa        -> 0x29a/0x29b
//	EnableFPGAWaitMode(1) every exposure
//
//	short   (exp <= frameTime + 0.1 s): 0x22a = 0
//	long    (exp >  frameTime + 0.1 s): cycle-count mode; FRAME = height+0x26 and LINES =
//	                                    VMAX-0x12 as 24-bit LE to 0x244-0x249 (dup 0x24a-0x24f),
//	                                    0x25c = 0xff, 0x22a = 1
//	trigger (exp >= 4 s):               additionally EnableFPGATriggerMode(1) (reg0 bit7) and
//	                                    SetFPGAHMAX(0x1500)
//
// frameTime is one default frame at the normal line time (SetExp: exp > frameTime + 0x186a0 µs).
func imx174SetExposure(rm Regmap, d time.Duration) error {
	us := uint64(d.Microseconds())
	trigger := us >= imx174TriggerUs // >= 4 s: FPGA hardware-trigger band

	// Normal readout HMAX for the live window (the SDK's bandwidth formula on the ROI geometry: a 640×480 window
	// runs HMAX 780 against 1735 full-frame at USB2 FPS% 100), the same value SetROI programs, so
	// the line time here and the FPGA's agree. In the trigger band HMAX is 0x1500 (SetExp writes
	// it before the line-time read), so the VMAX/SHS math below uses the trigger line time.
	w, h := imx174FullWidth, imx174FullHeight
	if rd := ModeOf(rm); rd.Width > 0 && rd.Height > 0 {
		w, h = rd.Width, rd.Height
	}
	hmaxNormal := imx174HMAX(rm, w, h)
	normalLineTimeNs := uint64(hmaxNormal) * 1_000_000 / imx174ClkKHz
	lineTimeNs := normalLineTimeNs
	if trigger {
		lineTimeNs = uint64(imx174TrigHMAX) * 1_000_000 / imx174ClkKHz // 0x1500 -> 268800 ns/line
	}
	lines := ExposureLines(d, lineTimeNs, imx174ExpMinUs, imx174ExpMaxUs)

	// One default frame = the window height + 0x26 (SetExp: VMAX 0x206 for a 480-row ROI, 0x4e6
	// full frame), the value SetROI writes to the sensor VMAX pair.
	defaultVMAX := uint64(h) + imx174VMAXOffset
	vmax := defaultVMAX
	if lines+imx174SHSFloor > vmax {
		vmax = lines + imx174SHSFloor // grow the frame to fit the exposure
	}
	shs := vmax - lines
	if shs < imx174SHSFloor {
		shs = imx174SHSFloor
	}

	// Long-frame (cycle-count) band: exp exceeds one default (normal-readout) frame + 0.1 s.
	frameTimeUs := defaultVMAX * normalLineTimeNs / 1000
	longFrame := us > frameTimeUs+100000

	// EnableFPGAWaitMode(1): reg0 bit6, set per exposure (camera bringup never sets WaitMode).
	if err := FPGASetBits(rm, 0x00, 0x40); err != nil {
		return err
	}
	// HMAX: 0x1500 only in the >= 4 s trigger band; normal readout HMAX otherwise.
	hmax := hmaxNormal
	if trigger {
		hmax = imx174TrigHMAX
	}
	if err := FPGAWrite16(rm, imx174FPGAHMAXL, imx174FPGAHMAXH, hmax); err != nil {
		return err
	}
	// EnableFPGATriggerMode (reg0 bit7) only in the >= 4 s trigger band. The 0.4–4 s cycle-count
	// band uses WaitMode only: the sensor self-times the integration on-chip (LINES line periods
	// at the normal line rate) and the FPGA holds the frame; no trigger signal (the SDK's 2 s
	// capture never writes reg 0x0b), no host integration window.
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

// imx174SetROI: the window start (X aligned to 4, Y to 2) plus the window write (window, VMAX =
// h·bin+0x26, FPGA HBLK=0, VBLK=0xb, Width, Height, HMAX); the sensor group is under the 0x20c
// latch.
//
// A sensor bin is rejected: this firmware has no pixel-binning step and no bin-gated mode
// register (the window write only scales the geometry by bin, which is the SDK's host-bin path), so the
// profile lists no HWBins and the Camera always host-bins from the bin-1 window.
func imx174SetROI(rm Regmap, x, y, w, h, bin int) error {
	if err := poaUnsupported(rm, "imx174", "SetROI"); err != nil {
		return err
	}
	if bin > 1 {
		return fmt.Errorf("imx174: sensor bin %d not available (no pixel-binning step; the Camera host-bins)", bin)
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
	// FPGA frame geometry: HBLK=0, VBLK=0xb, Width, Height.
	if err := ProgramFrameGeometry(rm, w, h, 0x00, 0x0b); err != nil {
		return err
	}
	// SetFPGAHMAX for this window from the live ReadoutMode.
	return FPGAWrite16(rm, imx174FPGAHMAXL, imx174FPGAHMAXH, imx174HMAX(rm, w, h))
}
