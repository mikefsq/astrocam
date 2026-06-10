package asicam

import (
	"testing"
	"time"
)

// TestPulseGuideDirection checks the decoded wire format: bReq 0xB0 (on) / 0xB1
// (off) with the DIRECTION in wValue (the bug-trap — a naive sendCmd would send
// wValue=0 and always guide North).
func TestPulseGuideDirection(t *testing.T) {
	f := &fakeTransport{}
	Register(ZWO.VID, 0x00A0, Model{Name: "GuideTest", Sensor: &testSensor})
	c, err := Open(f, ZWO.VID, 0x00A0)
	if err != nil {
		t.Fatal(err)
	}

	for _, tc := range []struct {
		dir  GuideDir
		want uint16
	}{
		{GuideNorth, 0}, {GuideSouth, 1}, {GuideEast, 2}, {GuideWest, 3},
	} {
		f.out = nil
		if err := c.PulseGuide(tc.dir, time.Millisecond); err != nil {
			t.Fatalf("PulseGuide(%s): %v", tc.dir, err)
		}
		if len(f.out) != 2 {
			t.Fatalf("PulseGuide(%s) made %d transfers, want 2: %+v", tc.dir, len(f.out), f.out)
		}
		if f.out[0] != (ctrlCall{cmdST4N, tc.want, 0}) {
			t.Errorf("PulseGuide(%s) ON = %+v, want {0xB0, %d, 0}", tc.dir, f.out[0], tc.want)
		}
		if f.out[1] != (ctrlCall{cmdST4F, tc.want, 0}) {
			t.Errorf("PulseGuide(%s) OFF = %+v, want {0xB1, %d, 0}", tc.dir, f.out[1], tc.want)
		}
	}
}

// TestPulseGuideOutOfRange confirms dir>3 is a silent no-op, matching the SDK's
// `cmp #3 / b.hi` guard that returns success with no transfer.
func TestPulseGuideOutOfRange(t *testing.T) {
	f := &fakeTransport{}
	c, _ := Open(f, ZWO.VID, 0x00A0)
	if err := c.PulseGuideOn(GuideDir(7)); err != nil {
		t.Fatal(err)
	}
	if len(f.out) != 0 {
		t.Errorf("out-of-range dir produced transfers: %+v", f.out)
	}
}
