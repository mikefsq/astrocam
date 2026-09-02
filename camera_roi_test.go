package astrocam

import (
	"testing"
	"time"
)

// roiClampSensor is a geometry-only profile: SetROI records what it was handed and nothing else,
// so a test can assert the window arithmetic without a wire.
func roiClampSensor(name string, w, h int, hwBins []int) *Sensor {
	return &Sensor{
		Name:        name,
		Info:        CameraInfo{MaxWidth: w, MaxHeight: h, BitDepth: 16, Bins: []int{1, 2, 3, 4}},
		HWBins:      hwBins,
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
}

// TestClampROIFullFrameIsLegalAtEveryBin: a client asking for a full frame divides the sensor
// extent by the bin factor and lands wherever the arithmetic falls. That size is illegal often
// enough to matter — a 3856x2180 IMX585 gives an odd 1285 columns at bin 3 and an odd 545 rows at
// bin 4 — and SetROI refuses it. ClampROI is what the front end calls instead of dividing, so its
// result must be the largest legal window at or below the request, and SetROI must then take it.
//
// The two vendors disagree on the rule, which is the whole reason this cannot live in a client:
// ZWO wants the SENSOR width a multiple of 8 (the host bins, so the sensor still reads the full
// region), PlayerOne the OUTPUT width a multiple of 4 (its camera bins, so the rule already counts
// output pixels).
func TestClampROIFullFrameIsLegalAtEveryBin(t *testing.T) {
	for _, cam := range []struct {
		name     string
		vid, pid uint16
		w, h     int
		color    bool
		hwBins   []int
		hwBinOn  bool
		wantW    [5]int // indexed by bin, [0] unused
		wantH    [5]int
	}{
		{
			// PlayerOne IMX585. Its camera bins at every factor, so the 4x2 step counts output
			// pixels directly. 3856/3 = 1285 (odd) and 2180/4 = 545 (odd) are the reported faults.
			name: "PlayerOne IMX585", vid: POA.VID, pid: 0x0D30,
			w: 3856, h: 2180, hwBins: []int{2},
			wantW: [5]int{0, 3856, 1928, 1284, 964},
			wantH: [5]int{0, 2180, 1090, 726, 544},
		},
		{
			// ZWO IMX455, host-binned: the 8x2 step counts sensor pixels, so it divides through by
			// the bin factor. 6388/3 = 2129 rows is a 6387-row sensor extent, which is odd.
			name: "ZWO IMX455 host bin", vid: ZWO.VID, pid: 0x0D31,
			w: 9576, h: 6388, color: true, hwBins: []int{2, 3},
			wantW: [5]int{0, 9576, 4788, 3192, 2394},
			wantH: [5]int{0, 6388, 3194, 2128, 1597},
		},
		{
			// The same die with the sensor doing the binning: now the rule counts output pixels,
			// so the width must itself be a multiple of 8 and 4788 at bin 2 has to give up 4.
			name: "ZWO IMX455 sensor bin", vid: ZWO.VID, pid: 0x0D32,
			w: 9576, h: 6388, color: true, hwBins: []int{2, 3}, hwBinOn: true,
			wantW: [5]int{0, 9576, 4784, 3192, 2392},
			wantH: [5]int{0, 6388, 3194, 2128, 1596},
		},
	} {
		t.Run(cam.name, func(t *testing.T) {
			Register(cam.vid, cam.pid, Model{
				Name:   cam.name,
				Sensor: roiClampSensor(cam.name, cam.w, cam.h, cam.hwBins),
				Color:  cam.color,
			})
			c, err := Open(NewStubTransport(), cam.vid, cam.pid)
			if err != nil {
				t.Fatal(err)
			}
			if cam.hwBinOn {
				if err := c.SetHardwareBin(true); err != nil {
					t.Fatal(err)
				}
			}
			for bin := 1; bin <= 4; bin++ {
				if err := c.SetBinning(bin); err != nil {
					t.Fatalf("bin %d: %v", bin, err)
				}
				// What a client computes: the sensor extent over the bin factor.
				reqW, reqH := cam.w/bin, cam.h/bin
				x, y, w, h := c.ClampROI(0, 0, reqW, reqH)
				if w != cam.wantW[bin] || h != cam.wantH[bin] {
					t.Errorf("bin %d: ClampROI(0,0,%d,%d) = %dx%d, want %dx%d",
						bin, reqW, reqH, w, h, cam.wantW[bin], cam.wantH[bin])
				}
				if w > reqW || h > reqH {
					t.Errorf("bin %d: clamp grew the window: %dx%d > %dx%d", bin, w, h, reqW, reqH)
				}
				// The point of the exercise: what comes back must be programmable.
				if err := c.SetROI(x, y, w, h); err != nil {
					t.Errorf("bin %d: SetROI rejected its own clamped window %dx%d: %v", bin, w, h, err)
				}
			}
		})
	}
}

// TestClampROISubframe checks the non-full-frame cases: an arbitrary drag is trimmed down, an
// already-legal window is untouched, an oversized one is cut to the frame, and a request below one
// step still yields something the camera can read out.
func TestClampROISubframe(t *testing.T) {
	Register(POA.VID, 0x0D33, Model{Name: "ClampSub", Sensor: roiClampSensor("CLAMPSUB", 3856, 2180, nil)})
	c, err := Open(NewStubTransport(), POA.VID, 0x0D33)
	if err != nil {
		t.Fatal(err)
	}
	for _, tc := range []struct {
		name           string
		x, y, w, h     int
		wx, wy, ww, wh int
	}{
		{"an arbitrary drag trims down to the 4x2 step", 100, 50, 641, 481, 100, 50, 640, 480},
		{"an already legal window is untouched", 0, 0, 640, 480, 0, 0, 640, 480},
		{"a window past the far edge is cut to the frame", 3800, 2100, 640, 480, 3800, 2100, 56, 80},
		{"a request below one step still gets one step", 0, 0, 1, 1, 0, 0, 4, 2},
		{"a negative origin is clamped", -8, -4, 640, 480, 0, 0, 640, 480},
	} {
		t.Run(tc.name, func(t *testing.T) {
			x, y, w, h := c.ClampROI(tc.x, tc.y, tc.w, tc.h)
			if x != tc.wx || y != tc.wy || w != tc.ww || h != tc.wh {
				t.Errorf("ClampROI(%d,%d,%d,%d) = %d,%d,%d,%d; want %d,%d,%d,%d",
					tc.x, tc.y, tc.w, tc.h, x, y, w, h, tc.wx, tc.wy, tc.ww, tc.wh)
			}
			if err := c.SetROI(x, y, w, h); err != nil {
				t.Errorf("SetROI rejected the clamped window (%d,%d %dx%d): %v", x, y, w, h, err)
			}
		})
	}
}
