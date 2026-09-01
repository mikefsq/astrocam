// Sony IMX178: Type 1/1.8, 6.4 MP (ZWO ASI178, PlayerOne Sedna). Own register map: the gain
// is a raw 16-bit code (the 0.1 dB value written directly, not the Sony /3 analog code), latch
// 0x3007, offset 0x3015/16, SHS 0x3034-36, ROI window/start in the 0x319x/0x31ax block.
// Exposure uses the shared STARVIS ShutterModel with a baked HMAX (420), not the 290/455
// bandwidth throttle. Not hardware-validated. Profile entry points:
//
//	imx178Init, imx178InitFPGA  sensor and FPGA bringup
//	imx178SetROI                window start, mode bytes, window, FPGA geometry, HMAX
//	imx178SetGain               analog gain
//	imx178SetExposure           frame length and shutter position
//	imx178SetOffset             black level
//	StreamStart / StreamStop    standby 0x3000: 6 on, 0 off
//	imx178Worker                the single-shot capture worker

package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
	"time"
)

const (
	imx178RegLatch   = 0x3007 // 1 before / 0 after a coupled register group
	imx178RegStandby = 0x3000 // sensor standby gate: 6 = standby, 0 = streaming

	imx178RegConvGain = 0x301b // conversion-gain select: 0 (LCG) / 0x1e (HCG, above gain 30)
	imx178RegGainL    = 0x301f // gain code, low byte (raw 0.1 dB value, 16-bit LE)
	imx178RegGainH    = 0x3020 // gain code, high byte

	imx178RegStartXL = 0x319c
	imx178RegStartXH = 0x319d
	imx178RegStartYL = 0x31a0
	imx178RegStartYH = 0x31a1
	imx178RegWidthL  = 0x31a2
	imx178RegWidthH  = 0x31a3
	imx178RegHeightL = 0x319e
	imx178RegHeightH = 0x319f
	imx178RegMode0   = 0x300e // window-mode byte: 0 at bin1, 0x23 at bin2/4
	imx178RegMode1   = 0x3010 // window-mode byte: 0 at bin1, 1 at bin2/4
	imx178RegClkByte = 0x3101 // clock byte: 0x30 (27 MHz, bin1), 0x32 (6750 kHz, bin2/4)

	imx178RegSHS0 = 0x3034
	imx178RegSHS1 = 0x3035
	imx178RegSHS2 = 0x3036

	imx178GainMax   = 510           // 51.0 dB, ASI 0.1 dB units (SetGain clamp)
	imx178GainHCGAt = 30            // above this, conv-gain 0x301b = 0x1e
	imx178ExpMinUs  = 32            // µs floor
	imx178ExpMaxUs  = 2_000_000_000 // 2000 s ceiling
	// imx178TrigReadTO bounds the frame read in the trigger band, where the worker has already
	// host-held the integration and the frame is buffered: only the wire transfer is left (a
	// 3096×2080 RAW16 frame is 13 MB). See imx290TrigReadTO.
	imx178TrigReadTO = 3 * time.Second

	imx178LongExpUs = 1_000_000 // >= 1 s enters FPGA trigger mode (inclusive bound)

	// Readout constants for the shared engine (fps.go / shutter.go). Geometry is image
	// orientation: 3072 wide × 2048 tall; the height drives VMAX/SHS.
	imx178FullWidth  = 3072
	imx178FullHeight = 2048
	imx178ClkKHz     = 27000 // bin1; bin2/4 = 6750
	imx178HMAX       = 420   // baked line-period HMAX; the SDK's FPS% default 80 is not applied
	imx178VBlankAdd  = 29    // VMAX = height + 0x1d (SetFPGAVMAX)
	imx178SHSOffset  = 29    // SHS = (height + 0x1d) - lines
	imx178HBLK       = 0     // SetFPGAHBLK(0)
	imx178VBLK       = 15    // SetFPGAVBLK(15) at bin1; bin2/4 = 11
)

// imx178Init is the sensor-write half of camera bringup: the 89-entry reglist (reg 0xffff = delay
// ms) then the explicit tail. FPGA bringup is in imx178InitFPGA.
var imx178Init = []RegVal{
	// --- reglist table (in order) ---
	{Reg: 0x3009, Val: 0x01}, {Reg: 0xffff, Val: 20}, // delay 20 ms
	{Reg: 0x310c, Val: 0x01}, {Reg: 0x33be, Val: 0x21}, {Reg: 0x33bf, Val: 0x21}, {Reg: 0x33c0, Val: 0x2c},
	{Reg: 0x33c1, Val: 0x2c}, {Reg: 0x33c2, Val: 0x21}, {Reg: 0x33c3, Val: 0x2c}, {Reg: 0x33c4, Val: 0x20},
	{Reg: 0x33c5, Val: 0x00}, {Reg: 0x311c, Val: 0x1e}, {Reg: 0x311d, Val: 0x15}, {Reg: 0x311e, Val: 0x72},
	{Reg: 0x311f, Val: 0x00}, {Reg: 0x3120, Val: 0x5c}, {Reg: 0x3121, Val: 0x00}, {Reg: 0x3122, Val: 0x72},
	{Reg: 0x3123, Val: 0x00}, {Reg: 0x3124, Val: 0xc7}, {Reg: 0x3125, Val: 0x01}, {Reg: 0x312d, Val: 0x00},
	{Reg: 0x312e, Val: 0x01}, {Reg: 0x312f, Val: 0x15}, {Reg: 0x3131, Val: 0x10}, {Reg: 0x3132, Val: 0x00},
	{Reg: 0x3133, Val: 0x72}, {Reg: 0x3134, Val: 0x00}, {Reg: 0x3137, Val: 0x38}, {Reg: 0x3138, Val: 0x00},
	{Reg: 0x3139, Val: 0x00}, {Reg: 0x313a, Val: 0x00}, {Reg: 0x313d, Val: 0x00}, {Reg: 0x3140, Val: 0x00},
	{Reg: 0x3220, Val: 0x89}, {Reg: 0x3221, Val: 0x00}, {Reg: 0x3222, Val: 0x54}, {Reg: 0x3223, Val: 0x00},
	{Reg: 0x3226, Val: 0x8d}, {Reg: 0x3227, Val: 0x00}, {Reg: 0x32a9, Val: 0x14}, {Reg: 0x32aa, Val: 0x00},
	{Reg: 0x32b3, Val: 0x0a}, {Reg: 0x32b4, Val: 0x00}, {Reg: 0x33d6, Val: 0x10}, {Reg: 0x33d7, Val: 0x0f},
	{Reg: 0x33d8, Val: 0x0e}, {Reg: 0x33d9, Val: 0x0c}, {Reg: 0x33da, Val: 0x06}, {Reg: 0x3011, Val: 0x00},
	{Reg: 0x301b, Val: 0x00}, {Reg: 0x3037, Val: 0x08}, {Reg: 0x3038, Val: 0x00}, {Reg: 0x3039, Val: 0x00},
	{Reg: 0x30ad, Val: 0x49}, {Reg: 0x30af, Val: 0x54}, {Reg: 0x30b0, Val: 0x33}, {Reg: 0x30b3, Val: 0x0a},
	{Reg: 0x30c4, Val: 0x30}, {Reg: 0x3103, Val: 0x03}, {Reg: 0x3104, Val: 0x08}, {Reg: 0x3107, Val: 0x10},
	{Reg: 0x310f, Val: 0x01}, {Reg: 0x32e5, Val: 0x06}, {Reg: 0x32e6, Val: 0x00}, {Reg: 0x32e7, Val: 0x1f},
	{Reg: 0x32e8, Val: 0x00}, {Reg: 0x32e9, Val: 0x00}, {Reg: 0x32ea, Val: 0x00}, {Reg: 0x32eb, Val: 0x00},
	{Reg: 0x32ec, Val: 0x00}, {Reg: 0x32ee, Val: 0x00}, {Reg: 0x32f2, Val: 0x02}, {Reg: 0x32f4, Val: 0x00},
	{Reg: 0x32f5, Val: 0x00}, {Reg: 0x32f6, Val: 0x00}, {Reg: 0x32f7, Val: 0x00}, {Reg: 0x32f8, Val: 0x00},
	{Reg: 0x32fc, Val: 0x02}, {Reg: 0x3310, Val: 0x11}, {Reg: 0x3338, Val: 0x81}, {Reg: 0x333d, Val: 0x00},
	{Reg: 0x3362, Val: 0x00}, {Reg: 0x336b, Val: 0x02}, {Reg: 0x336e, Val: 0x11}, {Reg: 0x33b4, Val: 0xfe},
	{Reg: 0x33b5, Val: 0x06}, {Reg: 0x33b9, Val: 0x00}, {Reg: 0x3018, Val: 0x00},
	// --- explicit bringup tail, in order ---
	{Reg: 0x3059, Val: 0x00}, {Reg: 0x300d, Val: 0x00}, {Reg: 0x3004, Val: 0x00},
	{Reg: 0x31a4, Val: 0x01}, {Reg: 0x31a5, Val: 0x01},
	// (FPGAReset + SendCMD(0xAF) here; see imx178InitFPGA / Camera.Init)
	{Reg: 0x3008, Val: 0x01}, {Reg: 0x305e, Val: 0x00},
}

// IMX178 is the Sony IMX178 profile (ZWO ASI178, PlayerOne Sedna). Not hardware-validated.
var IMX178 = Sensor{
	Name:      "IMX178", // mono die; MC adds a CFA
	GainMax:   imx178GainMax,
	ExpMinUs:  imx178ExpMinUs,
	ExpMaxUs:  imx178ExpMaxUs,
	OffsetMax: 240, OffsetDef: 1, // ASI Brightness range (family default)
	Info: CameraInfo{
		MaxWidth:  imx178FullWidth,
		MaxHeight: imx178FullHeight,
		PixelUm:   2.4,      // µm pitch
		BitDepth:  14,       // 14-bit ADC (RAW16 transport)
		Bayer:     "RGGB",   // CFA (color variant); surfaced when Model.Color
		Bins:      []int{1}, // bin2/4 mode bytes undecoded; SetROI rejects bin > 1
	},
	Init:          imx178Init,
	InitFPGA:      imx178InitFPGA,
	SetGain:       imx178SetGain,
	SetExposure:   imx178SetExposure,
	SetOffset:     imx178SetOffset,
	GetOffset:     imx178GetOffset,
	SetROI:        imx178SetROI,
	StreamStop:    func(rm Regmap) error { return rm.WriteReg(imx178RegStandby, 6) }, // standby on
	StreamStart:   func(rm Regmap) error { return rm.WriteReg(imx178RegStandby, 0) }, // standby off
	Worker:        imx178Worker,
	ROIStartAlign: func(int) (int, int) { return 4, 2 }, // window-start masks
}

// imx178Worker is the host-timed single-shot capture worker. XHSStop is FPGA reg 0x0b bit4
// (EnableFPGAXHSStop); TriggerSignal is reg 0x0b bit0.
//
//	arm:    SendCMD(0xAA)·FPGAStop·SendCMD(0xA9)·standby 0x3000=6·2ms·0x3000=0·10ms·FPGAStart·
//	        ResetEndPoint(0x81)
//	expose: EnableFPGATriggerSignal(1)+EnableFPGAXHSStop(1)·hold for the exposure·
//	        EnableFPGAXHSStop(0)+EnableFPGATriggerSignal(0)
//	read:   one BulkRead of FrameBytes
//	stop:   FPGAStop·SendCMD(0xAA)·ResetEndPoint (the SDK's exit)
func imx178Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()
	arm := func(full bool) error {
		if full {
			if err := ctl.VendorCmd(FX3StreamStop); err != nil {
				return err
			}
			if err := SetFPGABit(rm, 0x00, 0x10, true); err != nil { // FPGAStop
				return err
			}
			if err := ctl.VendorCmd(FX3StreamStart); err != nil {
				return err
			}
		}
		if err := rm.WriteReg(imx178RegStandby, 6); err != nil { // standby on
			return err
		}
		time.Sleep(2 * time.Millisecond)                         // usleep
		if err := rm.WriteReg(imx178RegStandby, 0); err != nil { // standby off → streaming
			return err
		}
		time.Sleep(10 * time.Millisecond)        // usleep
		return SetFPGABit(rm, 0x00, 0x10, false) // FPGAStart
	}
	// trigger arms the FPGA exposure window: TriggerSignal(1)+XHSStop(1) to start integrating,
	// (0)+(0) to end it.
	trigger := func(on bool) error {
		if on {
			if err := SetFPGABit(rm, 0x0b, 0x01, true); err != nil { // EnableFPGATriggerSignal(1)
				return err
			}
			return SetFPGABit(rm, 0x0b, 0x10, true) // EnableFPGAXHSStop(1)
		}
		if err := SetFPGABit(rm, 0x0b, 0x10, false); err != nil { // EnableFPGAXHSStop(0)
			return err
		}
		return SetFPGABit(rm, 0x0b, 0x01, false) // EnableFPGATriggerSignal(0)
	}

	// Halt the readout on every return, the arm's own failures included, as the SDK does on its
	// way out:
	// StopSensorStreaming (FPGAStop only, no sensor register), SendCMD(0xAA), ResetEndPoint.
	// Best-effort; a sensor left free-running with no reader backs up the FX3 GPIF.
	defer func() {
		_ = SetFPGABit(rm, 0x00, 0x10, true) // FPGAStop: reg0 bit4
		_ = ctl.VendorCmd(FX3StreamStop)
		_ = ctl.ResetEndpoint()
	}()
	if err := arm(true); err != nil {
		return 0, err
	}
	_ = ctl.ResetEndpoint()
	if err := trigger(true); err != nil {
		return 0, err
	}
	// The band split matches imx178SetExposure's inclusive >= 1 s trigger threshold: at 1 s the
	// FPGA is in trigger mode and the host hold is the integration, so it gets the full wait.
	if exposure < time.Second {
		if w := exposure - 200*time.Millisecond; w > 0 {
			time.Sleep(w)
		}
	} else {
		for start := time.Now(); time.Since(start) < exposure; {
			if ctl.Aborted() {
				// StopExposure ran: drop the trigger window on the way out. trigger(false)
				// clears both the trigger signal (reg 0x0b bit0) and XHSStop (reg 0x0b
				// bit4); left asserted, the next trigger(true) is a no-edge write.
				_ = trigger(false)
				return 0, errExposureAborted
			}
			time.Sleep(100 * time.Millisecond)
		}
	}
	if err := trigger(false); err != nil {
		return 0, err
	}
	_ = ctl.ResetEndpoint()
	// Read the frame byte count only, as the SDK does (its windowed read's size argument is
	// the image size): an oversized read runs into the next free-run frame or times out short.
	want := ctl.FrameBytes()
	if want > len(buf) {
		want = len(buf)
	}
	readTO := exposure + 3*time.Second // free-run: part of the integration is still to come
	if exposure >= time.Second {
		readTO = imx178TrigReadTO // trigger band: exposed and buffered, only the transfer remains
	}
	n, err := ctl.BulkRead(buf[:want], readTO)
	if err == nil && n < want && ctl.Aborted() {
		return n, errExposureAborted // AbortRead: clean abort, not a stall
	}
	return n, err
}

// imx178InitFPGA is the FPGA bringup after the Sony init: FPGAReset, 20 ms,
// SetFPGAAsMaster(1), FPGAStop, EnableFPGADDR(0), SetFPGAADCWidthOutputWidth (bit4 = RAW16
// from the live ReadoutMode), SetFPGAGain(0x80×4).
func imx178InitFPGA(rm Regmap, subtype int) error {
	if err := poaUnsupported(rm, "imx178", "FPGA bringup"); err != nil {
		return err
	}
	_ = subtype
	if err := FPGAClearBits(rm, 0x00, 0x01); err != nil { // FPGAReset
		return err
	}
	time.Sleep(20 * time.Millisecond)
	if err := FPGASetBits(rm, 0x00, 0x20); err != nil { // SetFPGAAsMaster(1)
		return err
	}
	if err := FPGASetBits(rm, 0x00, 0x10); err != nil { // FPGAStop
		return err
	}
	if err := FPGASetBits(rm, 0x0a, 0x40); err != nil { // EnableFPGADDR(0)
		return err
	}
	adcOut := uint16(0x01)
	if ModeOf(rm).BytesPerPx >= 2 {
		adcOut |= 0x10 // RAW16
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x11, adcOut); err != nil {
		return err
	}
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

// imx178SetGain (SetGain) clamps to [0, 510] and writes, under the 0x3007 latch, the conv-gain
// byte (0 LCG, 0x1e HCG above gain 30) and the raw 0.1 dB value as the 16-bit code.
func imx178SetGain(rm Regmap, gain int) error {
	if err := poaUnsupported(rm, "imx178", "SetGain"); err != nil {
		return err
	}
	if gain > imx178GainMax {
		gain = imx178GainMax
	}
	if gain < 0 {
		gain = 0
	}
	conv := uint16(0)
	if gain > imx178GainHCGAt {
		conv = 0x1e
	}
	g := uint16(gain)
	return WithLatch(rm, imx178RegLatch, func() error {
		for _, rv := range []RegVal{
			{Reg: imx178RegConvGain, Val: conv},
			{Reg: imx178RegGainL, Val: g & 0xff},
			{Reg: imx178RegGainH, Val: (g >> 8) & 0xff},
		} {
			if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
				return err
			}
		}
		return nil
	})
}

var imx178Shutter = ShutterModel{
	SHS0: imx178RegSHS0, SHS1: imx178RegSHS1, SHS2: imx178RegSHS2,
	SHSOffset:     imx178SHSOffset,
	MinExpUs:      imx178ExpMinUs,
	MaxExpUs:      imx178ExpMaxUs,
	Clock:         imx178ClkKHz,
	FixedHMAX:     imx178HMAX, // baked HMAX 420: line time = 420·1e6/27000 ≈ 15.56 µs
	VBlankAdd:     imx178VBlankAdd,
	DefaultWidth:  imx178FullWidth,
	DefaultHeight: imx178FullHeight,
}

// imx178SetExposure (SetExp): STARVIS SHS/VMAX via ApplyExposure (line time = HMAX·1000/clock,
// SHS = height+29-lines, VMAX via SetFPGAVMAX) under the 0x3007 latch, plus FPGA trigger mode
// (reg0 bit7) at >= 1 s.
func imx178SetExposure(rm Regmap, d time.Duration) error {
	trigger := d >= imx178LongExpUs*time.Microsecond
	if err := SetFPGABit(rm, 0x00, 0x80, trigger); err != nil {
		return err
	}
	return ApplyExposure(rm, imx178Shutter, imx178RegLatch, d)
}

// imx178GetOffset reads the offset back from 0x3015/0x3016, the pair imx178SetOffset writes
// 16-bit little-endian.
func imx178GetOffset(rm Regmap) (int, error) {
	v, err := ReadRegLE(rm, []uint16{0x3015, 0x3016})
	return int(v), err
}

func imx178SetOffset(rm Regmap, offset int) error {
	if err := poaUnsupported(rm, "imx178", "SetOffset"); err != nil {
		return err
	}
	v := uint16(offset)
	if err := rm.WriteReg(0x3016, (v>>8)&0xff); err != nil {
		return err
	}
	return rm.WriteReg(0x3015, v&0xff)
}

// imx178SetROI: the window start (X aligned to 4, Y to 2) + the window write (bin1 mode bytes,
// window, FPGA geometry, HMAX). Bin 1 only; bin2/4 (mode 0x23/1, VBLK 11, clock 6750) is the
// other window-setup branch, not decoded.
func imx178SetROI(rm Regmap, x, y, w, h, bin int) error {
	if err := poaUnsupported(rm, "imx178", "SetROI"); err != nil {
		return err
	}
	if bin != 1 {
		return fmt.Errorf("imx178: bin %d not supported (binning mode bytes not yet decoded)", bin)
	}
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	ux := uint16(x &^ 3) // align X to 4
	uy := uint16(y &^ 1) // align Y to 2
	uw, uh := uint16(w), uint16(h)

	if err := WithLatch(rm, imx178RegLatch, func() error {
		for _, rv := range []RegVal{
			{Reg: imx178RegStartXL, Val: ux & 0xff}, {Reg: imx178RegStartXH, Val: (ux >> 8) & 0xff},
			{Reg: imx178RegStartYL, Val: uy & 0xff}, {Reg: imx178RegStartYH, Val: (uy >> 8) & 0xff},
		} {
			if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
				return err
			}
		}
		return nil
	}); err != nil {
		return err
	}

	// Resolution-mode regs (SetResolution bin1 branch: 0x300d=2, 0x3059=2; bin2/4 writes
	// 0x300d=9), clock byte (27 MHz), bin1 mode bytes 0, window.
	for _, rv := range []RegVal{
		{Reg: 0x300d, Val: 0x02}, {Reg: 0x3059, Val: 0x02},
		{Reg: imx178RegClkByte, Val: 0x30},
		{Reg: imx178RegMode0, Val: 0x00}, {Reg: imx178RegMode1, Val: 0x00},
		{Reg: imx178RegWidthL, Val: uw & 0xff}, {Reg: imx178RegWidthH, Val: (uw >> 8) & 0xff},
		{Reg: imx178RegHeightL, Val: uh & 0xff}, {Reg: imx178RegHeightH, Val: (uh >> 8) & 0xff},
	} {
		if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
			return err
		}
	}

	if err := ProgramFrameGeometry(rm, w, h, imx178HBLK, imx178VBLK); err != nil {
		return err
	}
	// HMAX is the baked constant 420, written to the FPGA HMAX register directly.
	return WriteFPGAHMAX(rm, imx178HMAX)
}
