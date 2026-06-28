package astrocam

import "time"

// ShutterModel parameterizes a rolling-shutter (Sony STARVIS 0x30xx family) sensor's
// exposure programming.
//
//	lineTime_ns   = HMAX * 1000 / clock
//	exposureLines = exposure / lineTime
//	VMAX          = max(DefaultVMAX, exposureLines + SHSOffset + 1)  // extend frame for long exp
//	SHS           = VMAX + SHSOffset - exposureLines                 // clamped to >= 1
//
// VMAX (frame length) is programmed into the camera FPGA (SetVMAX); SHS into the sensor's
// 24-bit shutter registers. LineTimeNs is the value at the sensor's default full-resolution
// mode; non-default modes (USB2/3, binning, high-speed) re-derive it.
type ShutterModel struct {
	SHS0, SHS1, SHS2 uint16 // 24-bit SHS registers, little-endian
	SHSOffset        uint32 // SHS = (height + offset) - lines (e.g. 0x11 for IMX290/462)
	MinExpUs         uint64
	MaxExpUs         uint64

	// --- computed form: when Clock != 0, ApplyExposure derives the line time from the
	// HMAX formula (fps.go) using the live ReadoutMode and the STARVIS VMAX/SHS math:
	// VMAX = height + VBlankAdd, SHS = clamp(height + SHSOffset - lines, 1, height +
	// SHSOffset - 1). DefaultWidth/Height are the full-frame geometry. ---
	Clock     uint32 // pixel clock for the readout mode (e.g. 0x2441)
	FloorHMAX uint32 // REG_FRAME_LENGTH_PKG_MIN, the HMAX lower bound (e.g. 0xcb)
	VBlankAdd uint32 // VMAX = height + VBlankAdd (e.g. 0x12)

	// HighSpeedClock/HighSpeedFloorHMAX are the pixel clock + HMAX floor for the 10-bit
	// high-speed readout (mode.HighSpeed). When set and the live mode is high-speed, the
	// exposure line time uses these instead of Clock/FloorHMAX. 0 = no high-speed mode.
	HighSpeedClock     uint32
	HighSpeedFloorHMAX uint32

	// FixedHMAX, when nonzero (with Clock != 0), pins the line time to FixedHMAX*1e6/Clock
	// and bypasses the bandwidth/FPS HMAX derivation. Use it for sensors that bake a constant
	// HMAX into the object (e.g. IMX178's ctor HMAX=0x1a4).
	FixedHMAX     uint32
	DefaultWidth  uint32 // full-frame width  (HMAX geometry)
	DefaultHeight uint32 // full-frame height (VMAX/SHS reference + HMAX geometry)

	// --- legacy form: baked line time + precomputed full-frame VMAX. Used only when
	// Clock == 0. Deprecated. ---
	LineTimeNs  uint64
	DefaultVMAX uint32
}

// lineTimeNs returns the readout line time: computed from the HMAX formula and the
// live ReadoutMode when the sensor supplies Clock, else the baked legacy value.
func (m ShutterModel) lineTimeNs(rm Regmap) uint64 {
	if m.Clock == 0 {
		if m.LineTimeNs == 0 {
			return 1
		}
		return m.LineTimeNs
	}
	if m.FixedHMAX != 0 { // baked-constant HMAX (e.g. IMX178): no bandwidth/FPS derivation
		lt := uint64(m.FixedHMAX) * 1_000_000 / uint64(m.Clock)
		if lt == 0 {
			return 1
		}
		return lt
	}
	// Use the live ROI dimensions (set by SetROI) so the SHS line time matches the ROI HMAX;
	// fall back to the full-frame defaults.
	w, h := int(m.DefaultWidth), int(m.DefaultHeight)
	clock, floor := m.Clock, m.FloorHMAX
	if rd := ModeOf(rm); rd.Width > 0 {
		w, h = rd.Width, rd.Height
	}
	if ModeOf(rm).HighSpeed && m.HighSpeedClock != 0 { // 10-bit high-speed: faster clock + lower floor
		clock, floor = m.HighSpeedClock, m.HighSpeedFloorHMAX
	}
	lt := LineTimeNs(w, h, int(clock), int(floor), int(m.VBlankAdd), ModeOf(rm))
	if lt == 0 {
		return 1
	}
	return lt
}

// starvis is the STARVIS VMAX/SHS math: VMAX = height + VBlankAdd, SHS = clamp(height +
// SHSOffset - lines, 1, height + SHSOffset - 1), with VMAX stretched to lines+1 (SHS=1)
// when the exposure exceeds one frame, all clamped to 24 bits (0xffffff).
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

// Shutter computes the (vmax, shs) pair for a requested exposure, clamping to the
// model's exposure range and extending VMAX when the exposure exceeds one frame.
func (m ShutterModel) Shutter(d time.Duration) (vmax, shs uint32) {
	us := uint64(d.Microseconds())
	if us < m.MinExpUs {
		us = m.MinExpUs
	}
	if m.MaxExpUs != 0 && us > m.MaxExpUs {
		us = m.MaxExpUs
	}
	lt := m.LineTimeNs
	if lt == 0 {
		lt = 1
	}
	lines := us * 1000 / lt // exposure(ns) / lineTime(ns)
	vmax = m.DefaultVMAX
	if uint64(vmax) < lines+uint64(m.SHSOffset)+1 {
		vmax = uint32(lines + uint64(m.SHSOffset) + 1)
	}
	if vmax > 0xffffff {
		vmax = 0xffffff
	}
	s := int64(vmax) + int64(m.SHSOffset) - int64(lines)
	if s < 1 {
		s = 1
	}
	return vmax, uint32(s)
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

// WriteRegLE writes val little-endian across regs (regs[0]=low byte) through the sensor
// register space, optionally bracketed by latchReg (0 = no latch).
func WriteRegLE(rm Regmap, latchReg uint16, regs []uint16, val uint32) error {
	if latchReg != 0 {
		if err := rm.WriteReg(latchReg, 1); err != nil {
			return err
		}
	}
	for i, r := range regs {
		if err := rm.WriteReg(r, uint16(val>>(8*uint(i)))&0xff); err != nil {
			return err
		}
	}
	if latchReg != 0 {
		return rm.WriteReg(latchReg, 0)
	}
	return nil
}

// RollingExposure is the shared body for rolling-shutter sensors whose shutter is
// SHS = VMAX + offset - exposureLines, with VMAX (frame length) programmed into the camera
// FPGA. shsRegs are written little-endian (low byte first), bracketed by latchReg (0 = none).
func RollingExposure(rm Regmap, d time.Duration, lineTimeNs, defaultVMAX uint64, offset int64, minUs, maxUs uint64, latchReg uint16, shsRegs ...uint16) error {
	lines := ExposureLines(d, lineTimeNs, minUs, maxUs)
	o := uint64(0)
	if offset > 0 {
		o = uint64(offset)
	}
	vmax := defaultVMAX
	if lines+o+1 > vmax {
		vmax = lines + o + 1
	}
	if err := SetVMAX(rm, uint32(vmax)); err != nil {
		return err
	}
	shs := int64(vmax) + offset - int64(lines)
	if shs < 1 {
		shs = 1
	}
	return WriteRegLE(rm, latchReg, shsRegs, uint32(shs))
}

// ApplyExposure programs a requested exposure: VMAX into the FPGA (SetVMAX) then the 24-bit
// SHS into the sensor, bracketed by latchReg (0 = no latch). Shared SetExposure body for the
// STARVIS-family profiles. When the model supplies Clock the line time is computed from the
// live ReadoutMode; otherwise the legacy baked line time / DefaultVMAX path runs.
func ApplyExposure(rm Regmap, m ShutterModel, latchReg uint16, d time.Duration) error {
	var vmax, shs uint32
	if m.Clock != 0 {
		lines := ExposureLines(d, m.lineTimeNs(rm), m.MinExpUs, m.MaxExpUs)
		// Effective readout height = the live ROI height (set by SetROI), else full-frame
		// height. VMAX follows it, so a sub-frame ROI free-runs at a higher fps.
		effH := uint64(m.DefaultHeight)
		if h := ModeOf(rm).Height; h > 0 {
			effH = uint64(h)
		}
		vmax, shs = m.starvis(effH, lines)
		// EnableFPGAWaitMode(1): reg0 bit6 — SetExp sets this on the standard path.
		if err := SetFPGABit(rm, 0x00, 0x40, true); err != nil {
			return err
		}
	} else {
		vmax, shs = m.Shutter(d)
	}
	if err := SetVMAX(rm, vmax); err != nil {
		return err
	}
	if latchReg != 0 {
		if err := rm.WriteReg(latchReg, 1); err != nil {
			return err
		}
	}
	for _, w := range []RegVal{
		{Reg: m.SHS0, Val: uint16(shs) & 0xff},
		{Reg: m.SHS1, Val: uint16(shs>>8) & 0xff},
		{Reg: m.SHS2, Val: uint16(shs>>16) & 0xff},
	} {
		if err := rm.WriteReg(w.Reg, w.Val); err != nil {
			return err
		}
	}
	if latchReg != 0 {
		return rm.WriteReg(latchReg, 0)
	}
	return nil
}
