package astrocam

import "time"

// ShutterModel parameterizes a rolling-shutter (Sony STARVIS 0x30xx family) sensor's
// exposure programming.
//
//	lineTime_ns   = HMAX * 1000 / clock         (HMAX from the FPS/bandwidth engine, or FixedHMAX)
//	exposureLines = exposure / lineTime
//	VMAX          = height + VBlankAdd, stretched to exposureLines + 1 for long exposures
//	SHS           = height + SHSOffset - exposureLines, clamped to [1, height + SHSOffset - 1]
//
// VMAX (frame length) is programmed into the camera FPGA (SetVMAX); SHS into the sensor's
// 24-bit shutter registers. The line time is derived from the live ReadoutMode (USB2/3,
// binning, high-speed) at every call, never stored.
type ShutterModel struct {
	SHS0, SHS1, SHS2 uint16 // 24-bit SHS registers, little-endian
	SHSOffset        uint32 // SHS = (height + offset) - lines (e.g. 0x11 for IMX290/462)
	MinExpUs         uint64
	MaxExpUs         uint64

	Clock     uint32 // pixel clock for the readout mode (e.g. 0x2441); required
	FloorHMAX uint32 // REG_FRAME_LENGTH_PKG_MIN, the HMAX lower bound (e.g. 0xcb)
	VBlankAdd uint32 // VMAX = height + VBlankAdd (e.g. 0x12)

	// HighSpeedClock/HighSpeedFloorHMAX are the pixel clock + HMAX floor for the 10-bit
	// high-speed readout (mode.HighSpeed); used instead of Clock/FloorHMAX when the live mode is
	// high-speed. 0 = no high-speed mode.
	HighSpeedClock     uint32
	HighSpeedFloorHMAX uint32

	// FixedHMAX, when nonzero (with Clock != 0), pins the line time to FixedHMAX*1e6/Clock and
	// bypasses the bandwidth/FPS HMAX derivation, for sensors that bake a constant HMAX into the
	// object (e.g. IMX178's ctor HMAX=0x1a4).
	FixedHMAX     uint32
	DefaultWidth  uint32 // full-frame width  (HMAX geometry)
	DefaultHeight uint32 // full-frame height (VMAX/SHS reference + HMAX geometry)
}

// lineTimeNs returns the readout line time from the HMAX formula and the live ReadoutMode (or
// FixedHMAX), never below 1 ns.
func (m ShutterModel) lineTimeNs(rm Regmap) uint64 {
	if m.Clock == 0 {
		return 1
	}
	if m.FixedHMAX != 0 {
		lt := uint64(m.FixedHMAX) * 1_000_000 / uint64(m.Clock)
		if lt == 0 {
			return 1
		}
		return lt
	}
	// The live ROI dimensions keep the SHS line time equal to the ROI HMAX. Both dims must be
	// set: a Width with a zero Height would collapse the bandwidth candidate.
	w, h := int(m.DefaultWidth), int(m.DefaultHeight)
	clock, floor := m.Clock, m.FloorHMAX
	if rd := ModeOf(rm); rd.Width > 0 && rd.Height > 0 {
		w, h = rd.Width, rd.Height
	}
	// The high-speed readout is a 10-bit mode, so it applies to RAW8 output only: the profiles
	// gate it the same way (imx290HighSpeed / imx462HighSpeed) and leave the sensor on the normal
	// clock and the 12-bit format for RAW16 whatever the flag says. Reading the flag alone here
	// would convert the exposure at a line time the sensor is not running.
	if rd := ModeOf(rm); rd.HighSpeed && rd.BytesPerPx < 2 && m.HighSpeedClock != 0 {
		clock, floor = m.HighSpeedClock, m.HighSpeedFloorHMAX
	}
	lt := LineTimeNs(w, h, int(clock), int(floor), int(m.VBlankAdd), ModeOf(rm))
	if lt == 0 {
		return 1
	}
	return lt
}

// starvis is the STARVIS VMAX/SHS math: VMAX = height + VBlankAdd, SHS = clamp(height +
// SHSOffset - lines, 1, height + SHSOffset - 1), with VMAX stretched to lines+1 (SHS=1) when the
// exposure exceeds one frame, all clamped to 24 bits (0xffffff).
func (m ShutterModel) starvis(effH, lines uint64) (vmax, shs uint32) {
	v := effH + uint64(m.VBlankAdd)
	if lines+1 > v {
		vv := lines + 1
		if vv > 0xffffff {
			vv = 0xffffff
		}
		return uint32(vv), 1
	}
	s := effH + uint64(m.SHSOffset) - lines // lines <= effH+VBlankAdd-1 here
	if s == 0 {
		s = 1
	}
	if hi := effH + uint64(m.SHSOffset) - 1; s > hi {
		s = hi
	}
	return uint32(v), uint32(s)
}

// ExposureLines converts a requested duration to a sensor line count at the given line
// time, clamping to [minUs, maxUs]. Shared front half of every family's exposure math.
func ExposureLines(d time.Duration, lineTimeNs, minUs, maxUs uint64) uint64 {
	us := uint64(d.Microseconds())
	if us < minUs {
		us = minUs
	}
	if maxUs != 0 && us > maxUs {
		us = maxUs
	}
	if lineTimeNs == 0 {
		lineTimeNs = 1
	}
	return us * 1000 / lineTimeNs
}

// WithLatch runs fn with the sensor's coupled-group latch held (latchReg=1), releasing it
// (latchReg=0) even when fn errors: a held latch freezes every later grouped-register update.
// The first error wins. The shared bracket for every profile's latched register group (gain,
// ROI, exposure).
func WithLatch(rm Regmap, latchReg uint16, fn func() error) (err error) {
	if err = rm.WriteReg(latchReg, 1); err != nil {
		return err
	}
	defer func() {
		if rerr := rm.WriteReg(latchReg, 0); err == nil {
			err = rerr
		}
	}()
	return fn()
}

// WriteRegLE writes val little-endian across regs (regs[0]=low byte) through the sensor register
// space, optionally bracketed by latchReg (0 = no latch). The latch is released even when a data
// write errors (see WithLatch); the first error wins.
func WriteRegLE(rm Regmap, latchReg uint16, regs []uint16, val uint32) (err error) {
	if latchReg != 0 {
		if err = rm.WriteReg(latchReg, 1); err != nil {
			return err
		}
		defer func() {
			if rerr := rm.WriteReg(latchReg, 0); err == nil {
				err = rerr
			}
		}()
	}
	for i, r := range regs {
		if err = rm.WriteReg(r, uint16(val>>(8*uint(i)))&0xff); err != nil {
			return err
		}
	}
	return nil
}

// ReadRegLE reads regs (regs[0] = low byte) from the sensor register space and assembles the
// little-endian value, the read-back dual of WriteRegLE.
func ReadRegLE(rm Regmap, regs []uint16) (uint32, error) {
	var v uint32
	for i, r := range regs {
		b, err := rm.ReadReg(r)
		if err != nil {
			return 0, err
		}
		v |= uint32(b&0xff) << (8 * uint(i))
	}
	return v, nil
}

// ApplyExposure programs a requested exposure: EnableFPGAWaitMode (reg0 bit6), VMAX into the
// FPGA (SetVMAX), then the 24-bit SHS into the sensor bracketed by latchReg (0 = no latch). The
// shared SetExposure body for the STARVIS-family profiles; the readout height is the live ROI
// height, else DefaultHeight, so a sub-frame ROI free-runs at a higher fps.
func ApplyExposure(rm Regmap, m ShutterModel, latchReg uint16, d time.Duration) error {
	lines := ExposureLines(d, m.lineTimeNs(rm), m.MinExpUs, m.MaxExpUs)
	effH := uint64(m.DefaultHeight)
	if h := ModeOf(rm).Height; h > 0 {
		effH = uint64(h)
	}
	vmax, shs := m.starvis(effH, lines)
	if err := SetFPGABit(rm, 0x00, 0x40, true); err != nil {
		return err
	}
	if err := SetVMAX(rm, vmax); err != nil {
		return err
	}
	return WriteRegLE(rm, latchReg, []uint16{m.SHS0, m.SHS1, m.SHS2}, shs)
}
