package astrocam

import (
	"testing"
	"time"
)

// TestCameraCapabilityGetters: the range methods report the sensor-declared bounds, and the
// getters reflect what SetGain/SetExposure/SetROI set.
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

// TestCameraSetROIBounds: a window must fit the sensor (offset ≥ 0, size ≥ 1, offset+size ≤
// Max); out-of-range requests are rejected rather than clamped, and a rejected call leaves the
// recorded ROI unchanged.
func TestCameraSetROIBounds(t *testing.T) {
	s := Sensor{
		Name:        "ROIB",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1}},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x00CB, Model{Name: "RoiB", Sensor: &s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00CB)
	if err != nil {
		t.Fatal(err)
	}

	ok := []struct{ x, y, w, h int }{
		{0, 0, 96, 80}, // full frame
		{0, 0, 8, 2},   // min window (width a multiple of 8, even height)
		{56, 40, 40, 40},
		{48, 30, 48, 50}, // reaches the edge
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
		{-1, 0, 8, 10, "negative x"},
		{0, -1, 8, 10, "negative y"},
		{0, 0, 0, 10, "zero width"},
		{0, 0, 8, 0, "zero height"},
		{57, 0, 40, 10, "x+w > MaxWidth"},
		{0, 41, 8, 40, "y+h > MaxHeight"},
		{0, 0, 104, 80, "w > MaxWidth"},
		{0, 0, 100, 80, "width not a multiple of 8 (SDK SetResolution rule)"},
		{0, 0, 96, 79, "odd height"},
	}
	for _, r := range bad {
		if err := c.SetROI(r.x, r.y, r.w, r.h); err == nil {
			t.Errorf("SetROI(%d,%d,%d,%d) accepted, want rejected (%s)", r.x, r.y, r.w, r.h, r.why)
		}
	}
	// A rejected SetROI leaves the last good window in place.
	if x, y, w, h := c.ROI(); x != 48 || y != 30 || w != 48 || h != 50 {
		t.Errorf("ROI after rejected call = %d,%d,%d,%d, want the last good 48,30,48,50", x, y, w, h)
	}
}

// TestCameraOutputDepth: SetOutputDepth flips the live bytes-per-pixel so FrameBytes follows,
// and rejects anything but 1 or 2.
func TestCameraOutputDepth(t *testing.T) {
	s := Sensor{
		Name:        "DEPTH",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 16, Bins: []int{1}},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x00CD, Model{Name: "Depth", Sensor: &s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00CD)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetROI(0, 0, 96, 80); err != nil {
		t.Fatal(err)
	}
	if c.OutputDepth() != 2 || c.FrameBytes() != 96*80*2 {
		t.Errorf("default = depth %d / %d bytes, want 2 / %d (RAW16)", c.OutputDepth(), c.FrameBytes(), 96*80*2)
	}
	if err := c.SetOutputDepth(1); err != nil {
		t.Fatal(err)
	}
	if c.OutputDepth() != 1 || c.FrameBytes() != 96*80*1 {
		t.Errorf("RAW8 = depth %d / %d bytes, want 1 / %d", c.OutputDepth(), c.FrameBytes(), 96*80)
	}
	if err := c.SetOutputDepth(3); err == nil {
		t.Error("SetOutputDepth(3) accepted, want rejected")
	}
}

// TestCameraSetBinning: SetBinning accepts only supported factors, resets the ROI to the full
// binned frame, and rescopes SetROI's bounds to binned pixels.
func TestCameraSetBinning(t *testing.T) {
	s := Sensor{
		Name:        "BINC",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1, 2}},
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
	// bin 2: accepted, ROI resets to the full binned frame (48×40).
	if err := c.SetBinning(2); err != nil {
		t.Fatalf("SetBinning(2): %v", err)
	}
	if c.Binning() != 2 {
		t.Errorf("Binning = %d, want 2", c.Binning())
	}
	if _, _, w, h := c.ROI(); w != 48 || h != 40 {
		t.Errorf("ROI after SetBinning(2) = %dx%d, want 48x40 (full binned frame)", w, h)
	}
	// At bin 2 a window up to 48×40 fits; 52 wide is past the binned edge.
	if err := c.SetROI(0, 0, 48, 40); err != nil {
		t.Errorf("SetROI(48x40) at bin 2 rejected, want accepted: %v", err)
	}
	if err := c.SetROI(0, 0, 52, 40); err == nil {
		t.Error("SetROI(52x40) at bin 2 accepted, want rejected (past binned MaxWidth/2)")
	}
}

// TestCameraVendorCapsDispatch: GainRange/OffsetRange route through the live regmap's VID, so
// one shared sensor profile reports ZWO caps on a zwoRegmap and PlayerOne caps on a poaRegmap.
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
	// The init-time offset follows the vendor default too (Open seeds it from OffsetCaps).
	if got := cz.Offset(); got != 1 {
		t.Errorf("ZWO initial offset = %d, want the OffsetCaps default 1", got)
	}
	if got := cp.Offset(); got != 20 {
		t.Errorf("PlayerOne initial offset = %d, want the OffsetCaps default 20", got)
	}
}

// TestCameraOffsetReadsBack: Camera.Offset reports the sensor's register value (Sensor.GetOffset)
// rather than the driver's cache, so an external register change shows up; a profile without
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
	// An external register write shows up in the read-back.
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

// TestCameraSetROIStartAlign: a profile with ROIStartAlign has its start aligned down on the
// sensor-pixel grid (on a step the bin also divides) and ROI reports the aligned start, so a
// caller sees the window it gets; the size rule (width % 8, height % 2 in sensor pixels) is
// enforced before.
func TestCameraSetROIStartAlign(t *testing.T) {
	var gotX, gotY int
	s := Sensor{
		Name:          "ALIGN",
		Info:          CameraInfo{MaxWidth: 256, MaxHeight: 128, BitDepth: 12, Bins: []int{1, 2}},
		ROIStartAlign: func(int) (int, int) { return 16, 2 },
		SetROI:        func(_ Regmap, x, y, w, h, bin int) error { gotX, gotY = x, y; return nil },
		SetExposure:   func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x00CE, Model{Name: "Align", Sensor: &s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00CE)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetROI(37, 5, 64, 32); err != nil {
		t.Fatal(err)
	}
	if x, y, _, _ := c.ROI(); x != 32 || y != 4 || gotX != 32 || gotY != 4 {
		t.Errorf("bin 1: ROI start = %d,%d (profile got %d,%d), want 32,4 (X to 16, Y to 2)", x, y, gotX, gotY)
	}
	// Host bin 2: the binned start 9 is sensor 18 → down to 16 (lcm of 16 and 2) → binned 8; the
	// profile sees the bin-1 sensor start.
	if err := c.SetBinning(2); err != nil {
		t.Fatal(err)
	}
	if err := c.SetROI(9, 3, 32, 16); err != nil {
		t.Fatal(err)
	}
	if x, y, _, _ := c.ROI(); x != 8 || y != 3 || gotX != 16 || gotY != 6 {
		t.Errorf("host bin 2: ROI start = %d,%d (profile got %d,%d), want 8,3 (sensor 16,6)", x, y, gotX, gotY)
	}
}

// TestModeChangeReappliesROI: once a window has been programmed, SetOutputDepth /
// SetHighSpeedMode / SetBinning / SetHardwareBin re-program it so the sensor format follows the
// mode without a caller-side SetROI; before the first SetROI they only record the mode. They do
// not touch the exposure: a duration is a per-capture parameter (ASCOM carries it on
// StartExposure, not as a property), so re-stating the previous one as a side effect of setting
// a property would program the sensor's shutter from stale data.
func TestModeChangeReappliesROI(t *testing.T) {
	var rois, exps int
	var lastW, lastH, lastBin int
	s := Sensor{
		Name:   "REAP",
		Info:   CameraInfo{MaxWidth: 96, MaxHeight: 48, BitDepth: 12, Bins: []int{1, 2}},
		HWBins: []int{2},
		SetROI: func(_ Regmap, x, y, w, h, bin int) error {
			rois++
			lastW, lastH, lastBin = w, h, bin
			return nil
		},
		SetExposure: func(Regmap, time.Duration) error { exps++; return nil },
	}
	Register(ZWO.VID, 0x0CF5, Model{Name: "Reap", Sensor: &s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x0CF5)
	if err != nil {
		t.Fatal(err)
	}
	// Before any SetROI: mode setters record only.
	if err := c.SetOutputDepth(1); err != nil {
		t.Fatal(err)
	}
	if err := c.SetBinning(2); err != nil {
		t.Fatal(err)
	}
	if rois != 0 {
		t.Fatalf("mode setters before the first SetROI programmed the sensor %d times, want 0", rois)
	}
	if err := c.SetROI(0, 0, 48, 24); err != nil {
		t.Fatal(err)
	}
	if err := c.SetExposure(10 * time.Millisecond); err != nil {
		t.Fatal(err)
	}
	rois, exps = 0, 0
	// A depth change re-programs the same window (host bin 2 → sensor sees 96×48 at bin 1) and
	// leaves the exposure alone.
	if err := c.SetOutputDepth(2); err != nil {
		t.Fatal(err)
	}
	if rois != 1 || exps != 0 || lastW != 96 || lastH != 48 || lastBin != 1 {
		t.Errorf("SetOutputDepth: rois %d exps %d last %dx%d bin %d, want 1/0 96x48 bin 1", rois, exps, lastW, lastH, lastBin)
	}
	// Hardware bin on: re-programmed as sensor bin 2 over the output window.
	if err := c.SetHardwareBin(true); err != nil {
		t.Fatal(err)
	}
	if rois != 2 || lastW != 48 || lastH != 24 || lastBin != 2 {
		t.Errorf("SetHardwareBin: rois %d last %dx%d bin %d, want 2, 48x24 bin 2", rois, lastW, lastH, lastBin)
	}
	// SetBinning(1) resets to the full frame and programs it.
	if err := c.SetBinning(1); err != nil {
		t.Fatal(err)
	}
	if rois != 3 || lastW != 96 || lastH != 48 || lastBin != 1 || c.FrameBytes() != 96*48*2 {
		t.Errorf("SetBinning(1): rois %d last %dx%d bin %d FrameBytes %d, want 3, 96x48 bin 1, %d", rois, lastW, lastH, lastBin, c.FrameBytes(), 96*48*2)
	}
	if err := c.SetHighSpeedMode(true); err != nil {
		t.Fatal(err)
	}
	if rois != 4 || exps != 0 {
		t.Errorf("SetHighSpeedMode: rois %d exps %d, want 4 windows and no exposure write", rois, exps)
	}
}

// TestSetROISizeRuleSplitsOnHardwareBin pins the SDK's two size rules, which differ by where the
// multiple-of-8 width and even height are counted. Host binning reads the sensor at bin 1 over a
// bin-scaled region, so the rule applies to the SENSOR extent (w*bin, h*bin); hardware binning
// reads a binned window, so it applies to the BINNED size. Getting this wrong is not theoretical:
// collapsing the two into one sensor-extent rule made the driver accept an ASI6200 window at
// hardware bin 2 that the SDK rejects with rc=8.
func TestSetROISizeRuleSplitsOnHardwareBin(t *testing.T) {
	s := Sensor{
		Name:        "SizeRule",
		Info:        CameraInfo{MaxWidth: 9576, MaxHeight: 6388, BitDepth: 16, Bins: []int{1, 2, 3}},
		HWBins:      []int{2, 3},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x00E1, Model{Name: "SizeRule", Sensor: &s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00E1)
	if err != nil {
		t.Fatal(err)
	}
	defer c.Close()

	for _, tc := range []struct {
		name    string
		hw      bool
		bin     int
		w, h    int
		wantErr bool
	}{
		// Host bin 2: the rule counts the sensor extent, so a binned width of 4788 is fine --
		// the sensor reads 9576, a multiple of 8. These are the real ASI6200 numbers.
		{"host bin 2, sensor extent legal", false, 2, 4788, 3194, false},
		// Hardware bin 2 at the same window: 4788 % 8 = 4, which the SDK refuses (rc=8).
		{"hardware bin 2, binned width not a multiple of 8", true, 2, 4788, 3194, true},
		// The window the driver picks instead at hardware bin 2.
		{"hardware bin 2, binned width legal", true, 2, 4784, 3194, false},
		// Bin 3 host: 3192*3 = 9576 and 2128*3 = 6384, both legal.
		{"host bin 3, sensor extent legal", false, 3, 3192, 2128, false},
		// An odd height at bin 3 makes the sensor extent odd: refused.
		{"host bin 3, odd sensor height", false, 3, 3192, 2129, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if err := c.SetHardwareBin(tc.hw); err != nil {
				t.Fatal(err)
			}
			if err := c.SetBinning(tc.bin); err != nil {
				t.Fatal(err)
			}
			err := c.SetROI(0, 0, tc.w, tc.h)
			if tc.wantErr && err == nil {
				t.Errorf("SetROI(%dx%d) at bin %d hw=%v was accepted, want rejected", tc.w, tc.h, tc.bin, tc.hw)
			}
			if !tc.wantErr && err != nil {
				t.Errorf("SetROI(%dx%d) at bin %d hw=%v: %v", tc.w, tc.h, tc.bin, tc.hw, err)
			}
		})
	}
}
