package astrocam

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

// pulseRecorder is a StubTransport variant that records whether the ST4 commands took the
// ungated path.
type pulseRecorder struct {
	StubTransport
	ungated []uint8
}

func (p *pulseRecorder) ControlOutUngated(bRequest uint8, wValue, wIndex uint16) error {
	p.ungated = append(p.ungated, bRequest)
	return p.ControlOut(bRequest, wValue, wIndex, nil)
}

// TestPulseGuideUngatedDispatch: the ST4 commands route through UngatedControlSender when
// the transport offers it (a pulse edge must not queue behind ioMu).
func TestPulseGuideUngatedDispatch(t *testing.T) {
	p := &pulseRecorder{}
	c := &Camera{t: p, vend: ZWO}
	if err := c.PulseGuide(GuideNorth, time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if len(p.ungated) != 2 || p.ungated[0] != cmdST4N || p.ungated[1] != cmdST4F {
		t.Fatalf("ungated ST4 sequence = %#v, want [0xB0 0xB1]", p.ungated)
	}
}

// TestIsPulseGuiding: line-state and in-flight-pulse tracking back ASCOM IsPulseGuiding.
func TestIsPulseGuiding(t *testing.T) {
	c := &Camera{t: NewStubTransport(), vend: ZWO}
	if c.IsPulseGuiding() {
		t.Fatal("idle camera reports pulse guiding")
	}
	if err := c.PulseGuideOn(GuideWest); err != nil {
		t.Fatal(err)
	}
	if !c.IsPulseGuiding() {
		t.Fatal("asserted line not reflected in IsPulseGuiding")
	}
	if err := c.PulseGuideOn(GuideNorth); err != nil {
		t.Fatal(err)
	}
	if err := c.PulseGuideOff(GuideWest); err != nil {
		t.Fatal(err)
	}
	if !c.IsPulseGuiding() {
		t.Fatal("North still asserted but IsPulseGuiding is false (overlapping pulses)")
	}
	if err := c.PulseGuideOff(GuideNorth); err != nil {
		t.Fatal(err)
	}
	if c.IsPulseGuiding() {
		t.Fatal("all lines released but IsPulseGuiding is true")
	}
	done := make(chan struct{})
	go func() { _ = c.PulseGuide(GuideEast, 100*time.Millisecond); close(done) }()
	time.Sleep(30 * time.Millisecond)
	if !c.IsPulseGuiding() {
		t.Fatal("host-timed pulse in flight but IsPulseGuiding is false")
	}
	<-done
	if c.IsPulseGuiding() {
		t.Fatal("pulse completed but IsPulseGuiding is true")
	}
}
