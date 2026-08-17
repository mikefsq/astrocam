package astrocam

import (
	"errors"
	"testing"
	"time"
)

// TestPulseGuideDirection: bReq 0xB0 (on) / 0xB1 (off) with the direction in wValue (a wValue
// of 0 would always guide North).
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

// TestPulseGuideOutOfRange: dir>3 is a silent no-op, matching the SDK's `cmp #3 / b.hi` guard
// that returns success with no transfer.
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

// TestPulseGuideUngatedDispatch: the ST4 commands route through UngatedControlSender when the
// transport offers it.
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

// failOutTransport fails every vendor OUT after the nth, so a test can let the assert through and
// break the release.
type failOutTransport struct {
	Transport
	outs, failAfter int
}

func (f *failOutTransport) ControlOut(b uint8, v, i uint16, d []byte) error {
	f.outs++
	if f.outs > f.failAfter {
		return errStubOutFailed
	}
	return f.Transport.ControlOut(b, v, i, d)
}

var errStubOutFailed = errors.New("stub: vendor OUT failed")

// TestPulseGuideOffKeepsLineOnFailure: the ST4 line state must follow the hardware, not the
// intent. PulseGuideOff clears its bit only when the release reached the camera, so a failed
// release leaves IsPulseGuiding true — a mount that is still slewing must not be reported as
// settled, which is what an optimistic clear would do.
func TestPulseGuideOffKeepsLineOnFailure(t *testing.T) {
	ft := &failOutTransport{Transport: NewStubTransport(), failAfter: 1}
	c := &Camera{t: ft, vend: ZWO}
	if err := c.PulseGuideOn(GuideEast); err != nil {
		t.Fatal(err)
	}
	if !c.IsPulseGuiding() {
		t.Fatal("line not asserted after PulseGuideOn")
	}
	if err := c.PulseGuideOff(GuideEast); err == nil {
		t.Fatal("PulseGuideOff returned nil with the transport failing")
	}
	if !c.IsPulseGuiding() {
		t.Error("IsPulseGuiding false after a FAILED release: the line is still asserted on the camera")
	}
	// A release that succeeds does clear it.
	ft.failAfter = 1 << 30
	if err := c.PulseGuideOff(GuideEast); err != nil {
		t.Fatal(err)
	}
	if c.IsPulseGuiding() {
		t.Error("IsPulseGuiding still true after a successful release")
	}
}
