package astrocam

import "testing"

// The HMAX/line-time engine (fps.go) drives every sensor's readout throttle; these tests
// pin the formula to wire-confirmed SDK values and its clamping edges.

// plainRegmap is a no-op Regmap with NO modeReader, so ModeOf returns normalized defaults.
type plainRegmap struct{}

func (plainRegmap) WriteReg(reg, val uint16) error                        { return nil }
func (plainRegmap) WriteRegBits(reg uint16, lo, hi uint8, v uint16) error { return nil }
func (plainRegmap) ReadReg(reg uint16) (uint16, error)                    { return 0, nil }
func (plainRegmap) WriteFPGAReg(reg, val uint16) error                    { return nil }
func (plainRegmap) ReadFPGAReg(reg uint16) (uint16, error)                { return 0, nil }
func (plainRegmap) VID() uint16                                           { return ZWO.VID }

// TestHMAX462USB2WireConfirmed pins the USB2 case to the wire-confirmed SDK value: the
// IMX462 full frame (1936×1096 RAW16) at FPSPercent 40 on a USB2 link computes candidate
// 1634 (which dominates the 1100 floor there) → HMAX 4085, exactly what asicap programs.
func TestHMAX462USB2WireConfirmed(t *testing.T) {
	m := ReadoutMode{USB3: false, BytesPerPx: 2, FPSPercent: 40}
	if got := HMAX(1936, 1096, 18562, 1100, 18, m); got != 4085 {
		t.Fatalf("HMAX = %d, want 4085 (wire-confirmed SDK USB2 value)", got)
	}
}

// TestHMAX462USB3FloorDominates: on USB3 the bandwidth candidate (~196) is far below the
// sensor floor, so HMAX pins to REG_FRAME_LENGTH_PKG_MIN (1100) at FPSPercent 100.
func TestHMAX462USB3FloorDominates(t *testing.T) {
	m := ReadoutMode{USB3: true, BytesPerPx: 2, FPSPercent: 100}
	if got := HMAX(1936, 1096, 18562, 1100, 18, m); got != 1100 {
		t.Fatalf("HMAX = %d, want 1100 (the floor)", got)
	}
}

// TestHMAXFPSPercentClamps: FPSPercent normalizes into [40, 100] — an out-of-range request
// must not divide by 0/produce a runaway line time.
func TestHMAXFPSPercentClamps(t *testing.T) {
	at := func(pct int) uint16 {
		return HMAX(1936, 1096, 18562, 1100, 18, ReadoutMode{BytesPerPx: 2, FPSPercent: pct})
	}
	if at(10) != at(40) {
		t.Errorf("pct 10 -> %d, want the pct-40 clamp value %d", at(10), at(40))
	}
	if at(500) != at(100) {
		t.Errorf("pct 500 -> %d, want the pct-100 clamp value %d", at(500), at(100))
	}
	if at(0) != at(100) {
		t.Errorf("pct 0 -> %d, want the default-100 value %d", at(0), at(100))
	}
}

// TestHMAXCeiling: a value beyond the 16-bit register saturates at 0xffff instead of
// wrapping.
func TestHMAXCeiling(t *testing.T) {
	m := ReadoutMode{USB3: false, BytesPerPx: 2, FPSPercent: 40}
	if got := HMAX(9600, 6400, 200000, 1100, 18, m); got != 0xffff {
		t.Fatalf("HMAX = %d, want 0xffff saturation", got)
	}
}

// TestHMAXBWPerCameraBandwidth: HMAXBW must use the caller's own USB3 budget — the same
// geometry with a smaller bw3 yields a proportionally larger candidate.
func TestHMAXBWPerCameraBandwidth(t *testing.T) {
	m := ReadoutMode{USB3: true, BytesPerPx: 2, FPSPercent: 100}
	slow := HMAXBW(1936, 1096, 18562, 1, 18, bwUSB2, bwUSB3/8, m)
	fast := HMAXBW(1936, 1096, 18562, 1, 18, bwUSB2, bwUSB3, m)
	if slow <= fast {
		t.Fatalf("bw3/8 HMAX %d <= full-bw3 HMAX %d — per-camera bandwidth ignored", slow, fast)
	}
}

// TestLineTimeNs: line time = HMAX·1e6/clock, and a zero clock reports 0 rather than
// dividing by zero.
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

// TestHMAXAgreesWithShutterLineTime: the SHS math (ShutterModel.lineTimeNs) and the FPGA
// HMAX write must derive the SAME line time for the same mode, or the programmed exposure
// drifts from the sensor's real line rate. Uses the computed-form ShutterModel.
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
