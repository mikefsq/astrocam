package astrocam

import (
	"fmt"
	"testing"
	"time"
)

// TestSetBinningDefaultWindowIsLegal: SetBinning resets the window to the full binned frame, so
// that default must satisfy the same size rule SetROI enforces (sensor extent a multiple of 8
// wide, even high). On the IMX455 geometry a plain MaxHeight/3 is 2129 rows, whose sensor extent
// 6387 is odd, so every SetBinning(3) would fail on a factor the profile advertises.
func TestSetBinningDefaultWindowIsLegal(t *testing.T) {
	var gotW, gotH int
	s := &Sensor{
		Name:        "DEFWIN",
		Info:        CameraInfo{MaxWidth: 9576, MaxHeight: 6388, BitDepth: 16, Bins: []int{1, 2, 3, 4}},
		HWBins:      []int{2, 3},
		SetROI:      func(_ Regmap, x, y, w, h, bin int) error { gotW, gotH = w, h; return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x0D20, Model{Name: "DefWin", Sensor: s, Color: true})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x0D20)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetROI(0, 0, 9576, 6388); err != nil {
		t.Fatal(err)
	}
	for _, tc := range []struct {
		bin, w, h int
		hw        bool
	}{
		// Host binning: the rule counts sensor pixels. Max/3 gives 2129 rows, whose 6387-row
		// sensor extent is odd, so the default drops a row.
		{1, 9576, 6388, false},
		{2, 4788, 3194, false},
		{3, 3192, 2128, false},
		{4, 2394, 1597, false},
		// Sensor binning: the rule counts output pixels, so the width rounds to a multiple of 8
		// and the height to even. ASISetROIFormat refuses 4788 and 2394 under ASI_HARDWARE_BIN
		// and accepts 4784 and 2392, measured on an ASI6200MC.
		{2, 4784, 3194, true},
		{3, 3192, 2128, true},
		{4, 2392, 1596, true},
	} {
		if err := c.SetHardwareBin(tc.hw); err != nil {
			t.Errorf("SetHardwareBin(%v): %v", tc.hw, err)
			continue
		}
		if err := c.SetBinning(tc.bin); err != nil {
			t.Errorf("SetBinning(%d) hw=%v: %v", tc.bin, tc.hw, err)
			continue
		}
		_, _, w, h := c.ROI()
		if w != tc.w || h != tc.h {
			t.Errorf("bin %d hw=%v: window %dx%d, want %dx%d", tc.bin, tc.hw, w, h, tc.w, tc.h)
		}
		if fb := c.FrameBytes(); fb != gotW*gotH*2 {
			t.Errorf("bin %d hw=%v: FrameBytes %d, sensor programmed %dx%d (%d bytes)", tc.bin, tc.hw, fb, gotW, gotH, gotW*gotH*2)
		}
	}
}

// TestModeSettersRollBackOnFailure: a mode setter that cannot program its window must leave the
// camera as it found it. Committing the scalars first leaves FrameBytes and ROI describing a
// window the sensor never received, so the next capture reads the wrong number of bytes off the
// wire.
func TestModeSettersRollBackOnFailure(t *testing.T) {
	reject := map[int]bool{}
	s := &Sensor{
		Name:   "ROLLBACK",
		Info:   CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1, 2}},
		HWBins: []int{2},
		SetROI: func(_ Regmap, x, y, w, h, bin int) error {
			if reject[bin] {
				return fmt.Errorf("profile rejects bin %d", bin)
			}
			return nil
		},
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x0D21, Model{Name: "Rollback", Sensor: s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x0D21)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetROI(0, 0, 96, 80); err != nil {
		t.Fatal(err)
	}
	want := func(what string) {
		t.Helper()
		x, y, w, h := c.ROI()
		if c.Binning() != 1 || x != 0 || y != 0 || w != 96 || h != 80 {
			t.Errorf("%s: bin %d ROI %d,%d %dx%d, want bin 1 / 0,0 96x80", what, c.Binning(), x, y, w, h)
		}
		if fb := c.FrameBytes(); fb != 96*80*2 {
			t.Errorf("%s: FrameBytes %d, want %d", what, fb, 96*80*2)
		}
		if c.HardwareBin() {
			t.Errorf("%s: hardware bin left on", what)
		}
	}

	reject[1] = true // the host-bin split programs bin 1 at the sensor
	if err := c.SetBinning(2); err == nil {
		t.Error("SetBinning(2) accepted though the profile rejected the window")
	}
	want("after a rejected SetBinning")

	reject[1] = false
	reject[2] = true // the hardware-bin split programs bin 2 at the sensor
	if err := c.SetBinning(2); err != nil {
		t.Fatal(err)
	}
	if err := c.SetHardwareBin(true); err == nil {
		t.Error("SetHardwareBin(true) accepted though the profile rejected the window")
	}
	if c.HardwareBin() {
		t.Error("hardware bin left on after a rejected SetHardwareBin")
	}
	if fb := c.FrameBytes(); fb != 96*80*2 {
		t.Errorf("after a rejected SetHardwareBin: FrameBytes %d, want the host-bin wire size %d", fb, 96*80*2)
	}
}

// TestInitProgramsFullFrame: a caller that never sets a window must still capture the whole
// frame. The readout geometry a frame depends on (FPGA width/height, per-frame DMA length, the
// optical-black crop, HMAX) is written only by the profile's SetROI, so Init programs the full
// frame itself; without it the ASI6200 delivers its optical-black rows as image data (measured:
// leading row means ~18900 against ~1600 for a correct frame). It also leaves a window
// programmed, so a later mode change has something to re-apply.
func TestInitProgramsFullFrame(t *testing.T) {
	var rois [][4]int
	var bins []int
	s := &Sensor{
		Name: "INITROI",
		Info: CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1, 2}},
		Init: []RegVal{},
		SetROI: func(_ Regmap, x, y, w, h, bin int) error {
			rois = append(rois, [4]int{x, y, w, h})
			bins = append(bins, bin)
			return nil
		},
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x0D22, Model{Name: "InitROI", Sensor: s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x0D22)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.Init(); err != nil {
		t.Fatal(err)
	}
	if len(rois) != 1 || rois[0] != [4]int{0, 0, 96, 80} {
		t.Fatalf("Init programmed %v, want one full-frame window 0,0 96x80", rois)
	}
	if fb := c.FrameBytes(); fb != 96*80*2 {
		t.Errorf("FrameBytes after Init = %d, want %d", fb, 96*80*2)
	}
	// A mode change with no explicit SetROI must now reach the sensor rather than only record
	// the mode: binning halves the window the readout delivers.
	rois = nil
	if err := c.SetBinning(2); err != nil {
		t.Fatal(err)
	}
	if len(rois) == 0 {
		t.Fatal("SetBinning(2) after Init programmed no window")
	}
	last := rois[len(rois)-1]
	if last != [4]int{0, 0, 96, 80} { // host bin: the sensor still reads the full region at bin 1
		t.Errorf("SetBinning(2) programmed %v, want the bin-scaled region 0,0 96x80", last)
	}
	if fb := c.FrameBytes(); fb != 96*80*2 {
		t.Errorf("FrameBytes at host bin 2 = %d, want the %d-byte wire frame", fb, 96*80*2)
	}
}

// TestMarkerRepairKeepsBayerPhase: the FX3 header/footer sync words sit in real pixel slots, so
// the repair has to invent two pixels at each end. It copies them from the same columns two rows
// away, which is the Bayer period vertically and keeps the column phase horizontally — what the
// SDK does, verified on an ASI6200MC where its row0[0:2] equals its row2[0:2] byte for byte and
// its lastrow[-2:] equals row[-3][-2:]. Replicating the nearest pixel along the row instead puts
// an even-phase value in the odd-phase slot.
func TestMarkerRepairKeepsBayerPhase(t *testing.T) {
	const w, h = 8, 6
	// RAW16 frame whose sample value encodes its mosaic phase and row, so a copy from the wrong
	// phase or the wrong row is visible.
	frame := func() []byte {
		b := make([]byte, w*h*2)
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				v := 1000 + 100*(y%2) + 10*(x%2) + y // phase in the tens, row in the units
				b[(y*w+x)*2] = byte(v)
				b[(y*w+x)*2+1] = byte(v >> 8)
			}
		}
		// The FX3 markers overwrite the first and last 32-bit word.
		b[0], b[1], b[2], b[3] = 0x7E, 0x5A, 0x00, 0x00
		n := len(b)
		b[n-4], b[n-3], b[n-2], b[n-1] = 0x00, 0x00, 0xF0, 0x3C
		return b
	}
	sample := func(b []byte, i int) int { return int(b[i*2]) | int(b[i*2+1])<<8 }

	b := frame()
	repairFX3DMAMarkers(b, 2, w, 2)
	for i := 0; i < 2; i++ {
		want := 1000 + 100*(2%2) + 10*(i%2) + 2 // row 2, same column
		if got := sample(b, i); got != want {
			t.Errorf("header sample %d = %d, want %d (same column, two rows down)", i, got, want)
		}
	}
	last := w*h - 1
	for i := 0; i < 2; i++ {
		idx := last - 1 + i
		x := idx % w
		want := 1000 + 100*((h-3)%2) + 10*(x%2) + (h - 3) // row h-3, same column
		if got := sample(b, idx); got != want {
			t.Errorf("footer sample %d = %d, want %d (same column, two rows up)", idx, got, want)
		}
	}
	// Whatever the geometry, the replacement must never cross the mosaic phase.
	for _, width := range []int{w, 0} { // 0 = geometry unknown, the in-row fallback
		b := frame()
		repairFX3DMAMarkers(b, 2, width, 2)
		for _, i := range []int{0, 1, w*h - 2, w*h - 1} {
			x := i % w
			if got := sample(b, i); (got/10)%2 != x%2 {
				t.Errorf("width %d: sample %d = %d has the wrong column phase (x=%d)", width, i, got, x)
			}
		}
	}
	// A frame without the signature is untouched.
	clean := frame()
	clean[0], clean[1] = 0x11, 0x22
	before := append([]byte(nil), clean...)
	repairFX3DMAMarkers(clean, 2, w, 2)
	if string(clean) != string(before) {
		t.Error("a frame without the marker signature was modified")
	}
}

// TestOffsetReadBackDoesNotReplaceRequest: Offset() answers with what the register says, but the
// value the driver programs is the one the caller asked for, and reading must not redefine it.
// The IMX174's black-level register does not survive a capture cycle (the halt/re-arm leaves it
// at 0, which is why WorkerCtl.ReapplyOffset rewrites it on every arm), so a read between frames
// would otherwise cache 0 and every later arm would program 0.
func TestOffsetReadBackDoesNotReplaceRequest(t *testing.T) {
	reg := 0 // the sensor's black-level register
	s := &Sensor{
		Name:        "OFFCACHE",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1}},
		Init:        []RegVal{},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
		SetOffset:   func(_ Regmap, v int) error { reg = v; return nil },
		GetOffset:   func(Regmap) (int, error) { return reg, nil },
	}
	Register(ZWO.VID, 0x0D23, Model{Name: "OffCache", Sensor: s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x0D23)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetOffset(30); err != nil {
		t.Fatal(err)
	}
	reg = 0 // the capture cycle cleared it
	if got := c.Offset(); got != 0 {
		t.Errorf("Offset() = %d, want the register's 0", got)
	}
	if err := c.ReapplyOffset(); err != nil {
		t.Fatal(err)
	}
	if reg != 30 {
		t.Errorf("the arm programmed offset %d, want the requested 30", reg)
	}
	if got := c.Offset(); got != 30 {
		t.Errorf("Offset() = %d once the register holds the requested value again, want 30", got)
	}
}

// TestModeChangeReappliesOffset: a profile may encode the offset differently per readout mode
// (the IMX455 scales it by the sensor bin), so a mode change has to re-program it. Otherwise the
// register keeps the old mode's encoding and reads back as a different number than was asked for.
func TestModeChangeReappliesOffset(t *testing.T) {
	reg, bin := 0, 1
	enc := func(v, b int) int { // a bin-dependent encoding, as imx455SetOffsetZWO has
		if b > 1 {
			return v * 100 / 16
		}
		return v * 10
	}
	s := &Sensor{
		Name:        "OFFMODE",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1, 2}},
		HWBins:      []int{2},
		Init:        []RegVal{},
		SetROI:      func(_ Regmap, x, y, w, h, b int) error { bin = b; return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
		SetOffset:   func(_ Regmap, v int) error { reg = enc(v, bin); return nil },
		GetOffset: func(Regmap) (int, error) {
			if bin > 1 {
				return (reg*16 + 50) / 100, nil
			}
			return reg / 10, nil
		},
	}
	Register(ZWO.VID, 0x0D24, Model{Name: "OffMode", Sensor: s})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x0D24)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.Init(); err != nil { // programs the full frame, so a mode change re-applies it
		t.Fatal(err)
	}
	if err := c.SetOffset(50); err != nil {
		t.Fatal(err)
	}
	if err := c.SetHardwareBin(true); err != nil {
		t.Fatal(err)
	}
	if err := c.SetBinning(2); err != nil {
		t.Fatal(err)
	}
	if got := c.Offset(); got != 50 {
		t.Errorf("Offset() = %d after a mode change, want the requested 50 (the register kept the old mode's encoding)", got)
	}
}

// TestMarkerRepairRowStepFollowsTheCFA: the marker pixels are replaced from the same columns a
// whole number of rows away, and how many rows depends on the colour filter array rather than the
// sensor. Two rows is the Bayer period, which a mosaic needs to keep its phase, and the SDK uses
// it on the ASI6200MC and ASI462MC. Without a CFA there is no phase to keep and the SDK takes the
// nearest row: measured on the ASI174MM and ASI290MM, where its row 0 equals its row 1.
func TestMarkerRepairRowStepFollowsTheCFA(t *testing.T) {
	const w, h = 8, 6
	frame := func() []byte {
		b := make([]byte, w*h*2)
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				v := 1000 + 100*(y%2) + 10*(x%2) + y
				b[(y*w+x)*2] = byte(v)
				b[(y*w+x)*2+1] = byte(v >> 8)
			}
		}
		b[0], b[1], b[2], b[3] = 0x7E, 0x5A, 0x00, 0x00
		n := len(b)
		b[n-4], b[n-3], b[n-2], b[n-1] = 0x00, 0x00, 0xF0, 0x3C
		return b
	}
	sample := func(b []byte, i int) int { return int(b[i*2]) | int(b[i*2+1])<<8 }
	for _, rows := range []int{1, 2} {
		b := frame()
		repairFX3DMAMarkers(b, 2, w, rows)
		for i := 0; i < 2; i++ {
			want := 1000 + 100*(rows%2) + 10*(i%2) + rows // row `rows`, same column
			if got := sample(b, i); got != want {
				t.Errorf("rows=%d: header sample %d = %d, want %d (same column, %d row(s) down)", rows, i, got, want, rows)
			}
		}
		last := w*h - 1
		for i := 0; i < 2; i++ {
			idx := last - 1 + i
			y, x := (h-1)-rows, idx%w
			want := 1000 + 100*(y%2) + 10*(x%2) + y
			if got := sample(b, idx); got != want {
				t.Errorf("rows=%d: footer sample %d = %d, want %d (same column, %d row(s) up)", rows, idx, got, want, rows)
			}
		}
	}
}
