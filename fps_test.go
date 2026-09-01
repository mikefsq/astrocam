package astrocam

import "testing"

// These tests pin the HMAX/line-time formula (fps.go) to wire-confirmed SDK values and its
// clamping edges.

// plainRegmap is a no-op Regmap without modeReader, so ModeOf returns normalized defaults.
type plainRegmap struct{}

func (plainRegmap) WriteReg(reg, val uint16) error                        { return nil }
func (plainRegmap) WriteRegBits(reg uint16, lo, hi uint8, v uint16) error { return nil }
func (plainRegmap) ReadReg(reg uint16) (uint16, error)                    { return 0, nil }
func (plainRegmap) WriteFPGAReg(reg, val uint16) error                    { return nil }
func (plainRegmap) ReadFPGAReg(reg uint16) (uint16, error)                { return 0, nil }
func (plainRegmap) VID() uint16                                           { return ZWO.VID }

// TestHMAX462USB2WireConfirmed: the IMX462 full frame (1936×1096 RAW16) at FPSPercent 40 on a
// USB2 link computes candidate 1634 (above the 261 floor) → HMAX 4085, what asicap programs.
func TestHMAX462USB2WireConfirmed(t *testing.T) {
	m := ReadoutMode{USB3: false, BytesPerPx: 2, FPSPercent: 40}
	if got := HMAX(1936, 1096, 18562, 261, 18, m); got != 4085 {
		t.Fatalf("HMAX = %d, want 4085 (wire-confirmed SDK USB2 value)", got)
	}
}

// TestHMAX462USB3FloorDominates: on USB3 the bandwidth candidate (~196) is below the sensor's
// per-clock floor, so HMAX pins to REG_FRAME_LENGTH_PKG_MIN, 261 for the 462's 18562 clock (the
// value its the clock select installs; the .data 1100 initializer is dead after init).
func TestHMAX462USB3FloorDominates(t *testing.T) {
	m := ReadoutMode{USB3: true, BytesPerPx: 2, FPSPercent: 100}
	if got := HMAX(1936, 1096, 18562, 261, 18, m); got != 261 {
		t.Fatalf("HMAX = %d, want 261 (the clock-select floor)", got)
	}
}

// TestHMAXFPSPercentClamps: the readout mode's own normalisation is a sanity clamp only — it
// bounds the percentage away from zero and above 100 so the line time cannot run away or divide
// by zero. The vendor's real floor (ZWO 40, PlayerOne 35) is applied by Camera.SetFPSPercent,
// which knows the vendor; normalising to 40 here would raise a percentage PlayerOne accepts.
func TestHMAXFPSPercentClamps(t *testing.T) {
	at := func(pct int) uint16 {
		return HMAX(1936, 1096, 18562, 1100, 18, ReadoutMode{BytesPerPx: 2, FPSPercent: pct})
	}
	if at(10) == at(40) {
		t.Errorf("pct 10 was normalised up to 40; the floor is the vendor's to apply, not norm()'s")
	}
	if at(1) == 0 {
		t.Error("pct 1 produced a zero line time")
	}
	if at(500) != at(100) {
		t.Errorf("pct 500 -> %d, want the pct-100 clamp value %d", at(500), at(100))
	}
	if at(0) != at(100) {
		t.Errorf("pct 0 -> %d, want the default-100 value %d", at(0), at(100))
	}
}

// TestHMAXCeiling: a value beyond the 16-bit register saturates at 0xffff instead of wrapping.
func TestHMAXCeiling(t *testing.T) {
	m := ReadoutMode{USB3: false, BytesPerPx: 2, FPSPercent: 40}
	if got := HMAX(9600, 6400, 200000, 1100, 18, m); got != 0xffff {
		t.Fatalf("HMAX = %d, want 0xffff saturation", got)
	}
}

// TestHMAXBWPerCameraBandwidth: HMAXBW uses the caller's own USB3 budget; the same geometry with
// a smaller bw3 yields a larger candidate.
func TestHMAXBWPerCameraBandwidth(t *testing.T) {
	m := ReadoutMode{USB3: true, BytesPerPx: 2, FPSPercent: 100}
	slow := HMAXBW(1936, 1096, 18562, 1, 18, bwUSB2, bwUSB3/8, m)
	fast := HMAXBW(1936, 1096, 18562, 1, 18, bwUSB2, bwUSB3, m)
	if slow <= fast {
		t.Fatalf("bw3/8 HMAX %d <= full-bw3 HMAX %d — per-camera bandwidth ignored", slow, fast)
	}
}

// TestLineTimeNs: line time = HMAX·1e6/clock, and a zero clock reports 0.
func TestLineTimeNs(t *testing.T) {
	m := ReadoutMode{USB3: true, BytesPerPx: 2, FPSPercent: 100}
	want := uint64(1100) * 1_000_000 / 18562 // floor-pinned USB3 case above
	if got := LineTimeNs(1936, 1096, 18562, 1100, 18, m); got != want {
		t.Fatalf("LineTimeNs = %d, want %d", got, want)
	}
	if got := LineTimeNs(1936, 1096, 0, 1100, 18, m); got != 0 {
		t.Fatalf("LineTimeNs with clock 0 = %d, want 0", got)
	}
}

// TestHMAXAgreesWithShutterLineTime: the SHS math (ShutterModel.lineTimeNs) and the FPGA HMAX
// write derive the same line time for the same mode, else the programmed exposure drifts from
// the sensor's real line rate.
func TestHMAXAgreesWithShutterLineTime(t *testing.T) {
	sm := ShutterModel{
		Clock: 18562, FloorHMAX: 1100, VBlankAdd: 18,
		DefaultWidth: 1936, DefaultHeight: 1096,
		SHS0: 1, SHS1: 2, SHS2: 3, SHSOffset: 17,
	}
	rm := plainRegmap{} // plain Regmap: ModeOf falls back to normalized defaults
	fromShutter := sm.lineTimeNs(rm)
	m := ModeOf(rm)
	fromHMAX := LineTimeNs(1936, 1096, 18562, 1100, 18, m)
	if fromShutter != fromHMAX {
		t.Fatalf("shutter line time %d ns != HMAX-derived %d ns (SHS math and FPGA HMAX disagree)", fromShutter, fromHMAX)
	}
}

// modeRM carries a live ReadoutMode into the shutter model, which a plainRegmap cannot.
type modeRM struct {
	plainRegmap
	mode ReadoutMode
}

func (m modeRM) ReadoutMode() ReadoutMode { return m.mode }

// TestShutterHighSpeedNeedsRAW8: the high-speed readout is a 10-bit mode, so the profiles engage
// it only for RAW8 output (imx290HighSpeed / imx462HighSpeed gate on HighSpeed && BytesPerPx < 2)
// and program the normal clock, its HMAX floor and the 12-bit format for RAW16 whatever the flag
// says. The shutter model has to read the flag the same way: switching to the high-speed clock on
// the flag alone converts the requested exposure at a line time the sensor is not running, so the
// frame integrates for the wrong length. Measured on an ASI462MC at 2 ms RAW16: mean 6391 with
// the flag clear against 7979 with it set, where the two must be identical.
func TestShutterHighSpeedNeedsRAW8(t *testing.T) {
	sm := ShutterModel{
		Clock: 18562, FloorHMAX: 261, VBlankAdd: 18,
		HighSpeedClock: 37124, HighSpeedFloorHMAX: 245,
		DefaultWidth: 1936, DefaultHeight: 1096,
		SHS0: 1, SHS1: 2, SHS2: 3, SHSOffset: 17,
	}
	base := ReadoutMode{USB3: true, FPSPercent: 100, Width: 1936, Height: 1096}
	line := func(hs bool, bpp int) uint64 {
		m := base
		m.HighSpeed, m.BytesPerPx = hs, bpp
		return sm.lineTimeNs(modeRM{mode: m})
	}
	if got, want := line(true, 2), line(false, 2); got != want {
		t.Errorf("RAW16 line time = %d ns with high-speed set and %d ns clear; the sensor runs the "+
			"normal clock either way, so they must agree", got, want)
	}
	if line(true, 1) == line(false, 1) {
		t.Error("RAW8 line time is unchanged by the high-speed flag; the fast clock is never used")
	}
	// The RAW8 high-speed line time is the one the HMAX engine programs for that mode.
	m := base
	m.HighSpeed, m.BytesPerPx = true, 1
	if got, want := line(true, 1), LineTimeNs(1936, 1096, 37124, 245, 18, m); got != want {
		t.Errorf("RAW8 high-speed line time %d ns != HMAX-derived %d ns", got, want)
	}
}
