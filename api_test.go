package astrocam

import (
	"testing"
	"time"
)

// TestCameraCapabilityGetters checks the Alpaca-facing surface: the range methods report
// the sensor-declared bounds, and the getters reflect what SetGain/SetExposure/SetROI set —
// so goalpaca can answer ICameraV3 queries from public Camera methods, no Sensor poking.
func TestCameraCapabilityGetters(t *testing.T) {
	s := Sensor{
		Name:        "CAP",
		Info:        CameraInfo{MaxWidth: 100, MaxHeight: 80, BitDepth: 12, Bins: []int{1, 2, 4}},
		GainMax:     700,
		ExpMinUs:    32,
		ExpMaxUs:    2_000_000_000,
		SetGain:     func(Regmap, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
	}
	Register(ZWO.VID, 0x00CA, Model{Name: "Cap", Sensor: &s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00CA)
	if err != nil {
		t.Fatal(err)
	}

	// Ranges come from the profile.
	if lo, hi := c.GainRange(); lo != 0 || hi != 700 {
		t.Errorf("GainRange = %d..%d, want 0..700", lo, hi)
	}
	if lo, hi := c.ExposureRange(); lo != 32*time.Microsecond || hi != 2_000_000_000*time.Microsecond {
		t.Errorf("ExposureRange = %v..%v, want 32µs..2000s", lo, hi)
	}
	if c.ExposureStep() != time.Microsecond {
		t.Errorf("ExposureStep = %v, want 1µs", c.ExposureStep())
	}
	if got := c.Bins(); len(got) != 3 || got[2] != 4 {
		t.Errorf("Bins = %v, want [1 2 4]", got)
	}
	if c.Binning() != 1 {
		t.Errorf("Binning = %d, want 1", c.Binning())
	}

	// Getters reflect what was set.
	if err := c.SetGain(123); err != nil {
		t.Fatal(err)
	}
	if c.Gain() != 123 {
		t.Errorf("Gain = %d, want 123", c.Gain())
	}
	if err := c.SetExposure(250 * time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if c.Exposure() != 250*time.Millisecond {
		t.Errorf("Exposure = %v, want 250ms", c.Exposure())
	}
	if err := c.SetROI(4, 8, 40, 20); err != nil {
		t.Fatal(err)
	}
	if x, y, w, h := c.ROI(); x != 4 || y != 8 || w != 40 || h != 20 {
		t.Errorf("ROI = %d,%d,%d,%d, want 4,8,40,20", x, y, w, h)
	}
}

// TestCameraSetROIBounds locks the sub-frame ROI validation: a window must fit the sensor
// (offset ≥ 0, size ≥ 1, offset+size ≤ Max). Out-of-range requests are rejected rather than
// silently clamped, so FrameBytes can never disagree with the FX3 transfer size.
func TestCameraSetROIBounds(t *testing.T) {
	s := Sensor{
		Name:        "ROIB",
		Info:        CameraInfo{MaxWidth: 100, MaxHeight: 80, BitDepth: 12, Bins: []int{1}},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x00CB, Model{Name: "RoiB", Sensor: &s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00CB)
	if err != nil {
		t.Fatal(err)
	}

	ok := []struct{ x, y, w, h int }{
		{0, 0, 100, 80}, // full frame
		{0, 0, 1, 1},    // min window
		{60, 40, 40, 40},
		{50, 30, 50, 50}, // exactly to the edge
	}
	for _, r := range ok {
		if err := c.SetROI(r.x, r.y, r.w, r.h); err != nil {
			t.Errorf("SetROI(%d,%d,%d,%d) rejected, want accepted: %v", r.x, r.y, r.w, r.h, err)
		}
	}

	bad := []struct {
		x, y, w, h int
		why        string
	}{
		{-1, 0, 10, 10, "negative x"},
		{0, -1, 10, 10, "negative y"},
		{0, 0, 0, 10, "zero width"},
		{0, 0, 10, 0, "zero height"},
		{61, 0, 40, 10, "x+w > MaxWidth"},
		{0, 41, 10, 40, "y+h > MaxHeight"},
		{0, 0, 101, 80, "w > MaxWidth"},
	}
	for _, r := range bad {
		if err := c.SetROI(r.x, r.y, r.w, r.h); err == nil {
			t.Errorf("SetROI(%d,%d,%d,%d) accepted, want rejected (%s)", r.x, r.y, r.w, r.h, r.why)
		}
	}
	// A rejected SetROI must NOT mutate the recorded geometry (still the last good window).
	if x, y, w, h := c.ROI(); x != 50 || y != 30 || w != 50 || h != 50 {
		t.Errorf("ROI after rejected call = %d,%d,%d,%d, want the last good 50,30,50,50", x, y, w, h)
	}
}

// TestCameraOutputDepth checks the RAW16/RAW8 readout-mode seam: SetOutputDepth flips the live
// bytes-per-pixel so FrameBytes follows, and rejects anything but 1 or 2.
func TestCameraOutputDepth(t *testing.T) {
	s := Sensor{
		Name:        "DEPTH",
		Info:        CameraInfo{MaxWidth: 100, MaxHeight: 80, BitDepth: 16, Bins: []int{1}},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x00CD, Model{Name: "Depth", Sensor: &s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00CD)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetROI(0, 0, 100, 80); err != nil {
		t.Fatal(err)
	}
	if c.OutputDepth() != 2 || c.FrameBytes() != 100*80*2 {
		t.Errorf("default = depth %d / %d bytes, want 2 / %d (RAW16)", c.OutputDepth(), c.FrameBytes(), 100*80*2)
	}
	if err := c.SetOutputDepth(1); err != nil {
		t.Fatal(err)
	}
	if c.OutputDepth() != 1 || c.FrameBytes() != 100*80*1 {
		t.Errorf("RAW8 = depth %d / %d bytes, want 1 / %d", c.OutputDepth(), c.FrameBytes(), 100*80)
	}
	if err := c.SetOutputDepth(3); err == nil {
		t.Error("SetOutputDepth(3) accepted, want rejected")
	}
}

// TestCameraSetBinning checks the binning seam: SetBinning accepts only supported factors,
// resets the ROI to the full binned frame, and rescopes SetROI's bounds to binned pixels.
func TestCameraSetBinning(t *testing.T) {
	s := Sensor{
		Name:        "BINC",
		Info:        CameraInfo{MaxWidth: 100, MaxHeight: 80, BitDepth: 12, Bins: []int{1, 2}},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x00CC, Model{Name: "BinC", Sensor: &s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00CC)
	if err != nil {
		t.Fatal(err)
	}

	if c.Binning() != 1 {
		t.Errorf("default Binning = %d, want 1", c.Binning())
	}
	// An unsupported factor is rejected (4 not in Bins).
	if err := c.SetBinning(4); err == nil {
		t.Error("SetBinning(4) accepted, want rejected (not in Bins)")
	}
	// bin 2: accepted, ROI resets to the full binned frame (50×40).
	if err := c.SetBinning(2); err != nil {
		t.Fatalf("SetBinning(2): %v", err)
	}
	if c.Binning() != 2 {
		t.Errorf("Binning = %d, want 2", c.Binning())
	}
	if _, _, w, h := c.ROI(); w != 50 || h != 40 {
		t.Errorf("ROI after SetBinning(2) = %dx%d, want 50x40 (full binned frame)", w, h)
	}
	// At bin 2 a window up to 50×40 fits; 51 wide is past the binned edge.
	if err := c.SetROI(0, 0, 50, 40); err != nil {
		t.Errorf("SetROI(50x40) at bin 2 rejected, want accepted: %v", err)
	}
	if err := c.SetROI(0, 0, 51, 40); err == nil {
		t.Error("SetROI(51x40) at bin 2 accepted, want rejected (past binned MaxWidth/2)")
	}
}

// TestCameraVendorCapsDispatch checks GainRange/OffsetRange route through the live regmap's VID:
// the SAME shared sensor profile reports ZWO caps when opened on a zwoRegmap and PlayerOne caps on
// a poaRegmap (built by the POA vendor factory at Open). nil caps funcs fall back to static fields.
func TestCameraVendorCapsDispatch(t *testing.T) {
	s := Sensor{
		Name: "Caps",
		GainCaps: func(vid uint16) (int, int) {
			if vid == POA.VID {
				return 0, 550
			}
			return 0, 700
		},
		OffsetCaps: func(vid uint16) (int, int, int) {
			if vid == POA.VID {
				return 0, 2000, 20
			}
			return 0, 200, 1
		},
		SetOffset: func(Regmap, int) error { return nil },
	}
	Register(ZWO.VID, 0x0CE0, Model{Name: "z", Sensor: &s})
	Register(POA.VID, 0x0CE1, Model{Name: "p", Sensor: &s})

	cz, err := Open(NewStubTransport(), ZWO.VID, 0x0CE0)
	if err != nil {
		t.Fatal(err)
	}
	if _, mx := cz.GainRange(); mx != 700 {
		t.Errorf("ZWO GainRange max = %d, want 700", mx)
	}
	if _, mx, df, ok := cz.OffsetRange(); !ok || mx != 200 || df != 1 {
		t.Errorf("ZWO OffsetRange = max %d def %d ok %v, want 200/1", mx, df, ok)
	}

	cp, err := Open(NewStubTransport(), POA.VID, 0x0CE1)
	if err != nil {
		t.Fatal(err)
	}
	if _, mx := cp.GainRange(); mx != 550 {
		t.Errorf("PlayerOne GainRange max = %d, want 550", mx)
	}
	if _, mx, df, ok := cp.OffsetRange(); !ok || mx != 2000 || df != 20 {
		t.Errorf("PlayerOne OffsetRange = max %d def %d ok %v, want 2000/20", mx, df, ok)
	}
}

// TestCameraOffsetReadsBack: Camera.Offset reports the sensor's register value (Sensor.GetOffset),
// not the driver's cache — a change made behind the driver's back shows up, and a profile without
// read-back falls back to the last value set.
func TestCameraOffsetReadsBack(t *testing.T) {
	s := Sensor{
		Name:      "OFFRB",
		Info:      CameraInfo{MaxWidth: 4, MaxHeight: 2, BitDepth: 12},
		OffsetMax: 100, OffsetDef: 7,
		SetOffset: func(rm Regmap, o int) error { return WriteRegLE(rm, 0, []uint16{0x10, 0x11}, uint32(o)) },
		GetOffset: func(rm Regmap) (int, error) { v, err := ReadRegLE(rm, []uint16{0x10, 0x11}); return int(v), err },
	}
	Register(ZWO.VID, 0x0CF0, Model{Name: "OffRB", Sensor: &s})
	st := NewStubTransport() // echoes register writes back on reads
	c, err := Open(st, ZWO.VID, 0x0CF0)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetOffset(37); err != nil {
		t.Fatal(err)
	}
	if got := c.Offset(); got != 37 {
		t.Fatalf("Offset after SetOffset(37) = %d", got)
	}
	// Someone else reprograms the sensor: the read-back must reflect it, not the cache.
	if err := c.Rm().WriteReg(0x10, 0x2c); err != nil { // 300 = 0x012c
		t.Fatal(err)
	}
	if err := c.Rm().WriteReg(0x11, 0x01); err != nil {
		t.Fatal(err)
	}
	if got := c.Offset(); got != 300 {
		t.Fatalf("Offset after external write = %d, want 300 (register read-back)", got)
	}
	// No read-back on the profile: the cached last-set value is reported.
	noRB := s
	noRB.GetOffset = nil
	Register(ZWO.VID, 0x0CF1, Model{Name: "OffNoRB", Sensor: &noRB})
	c2, err := Open(NewStubTransport(), ZWO.VID, 0x0CF1)
	if err != nil {
		t.Fatal(err)
	}
	if err := c2.SetOffset(12); err != nil {
		t.Fatal(err)
	}
	if got := c2.Offset(); got != 12 {
		t.Fatalf("Offset without read-back = %d, want cached 12", got)
	}
}
