package astrocam

import (
	"fmt"
	"time"
)

// ST4 autoguider pulse output. Each call is a single vendor OUT control transfer
// (bmRequestType 0x40, no data stage); the direction rides in wValue.

// GuideDir is an ST4 pulse direction, matching ASI_GUIDE_DIRECTION (NORTH=0, SOUTH=1,
// EAST=2, WEST=3).
type GuideDir uint8

const (
	GuideNorth GuideDir = iota // 0 — +Dec
	GuideSouth                 // 1 — -Dec
	GuideEast                  // 2 — +RA
	GuideWest                  // 3 — -RA
)

func (d GuideDir) String() string {
	switch d {
	case GuideNorth:
		return "N"
	case GuideSouth:
		return "S"
	case GuideEast:
		return "E"
	case GuideWest:
		return "W"
	}
	return "?"
}

// st4Out issues one ST4 pulse command (the vendor's ST4On/ST4Off request, 0xB0/0xB1 on ZWO),
// UNGATED when the transport offers it (see UngatedControlSender): a pulse edge must never queue
// behind a whole-frame read holding ioMu; the SDK's own pulses are issued concurrently with its
// capture thread's bulk stream (CCameraBase::pulseGuide, 0154_CCameraBase.o: SendCMD(0xB0) →
// usleep(ms) → SendCMD(0xB1), and the capture thread never holds the API mutex). Falls back to
// the gated ControlOut on backends without the capability. A vendor with the request not
// decoded errors instead of sending another vendor's opcode.
func (c *Camera) st4Out(on bool, dir GuideDir) error {
	code, what := c.vend.Cmds.ST4Off, "ST4Off"
	if on {
		code, what = c.vend.Cmds.ST4On, "ST4On"
	}
	if code == 0 {
		return fmt.Errorf("astrocam: FX3 %s not decoded for vendor %s", what, c.vend.Name)
	}
	if u, ok := c.t.(UngatedControlSender); ok {
		return u.ControlOutUngated(code, uint16(dir), 0)
	}
	return c.t.ControlOut(code, uint16(dir), 0, nil)
}

// PulseGuideOn asserts the ST4 line for dir (ZWO SendCMD 0xB0, wValue=dir). dir>3 is a no-op.
func (c *Camera) PulseGuideOn(dir GuideDir) error {
	if dir > GuideWest {
		return nil
	}
	if err := c.st4Out(true, dir); err != nil {
		return err
	}
	c.mu.Lock()
	c.st4Lines |= 1 << uint(dir)
	c.mu.Unlock()
	return nil
}

// PulseGuideOff releases the ST4 line for dir (ZWO SendCMD 0xB1, wValue=dir). The line state
// clears only when the release reached the camera; after an error the line may still be
// asserted, so IsPulseGuiding keeps reporting true.
func (c *Camera) PulseGuideOff(dir GuideDir) error {
	if dir > GuideWest {
		return nil
	}
	if err := c.st4Out(false, dir); err != nil {
		return err
	}
	c.mu.Lock()
	c.st4Lines &^= 1 << uint(dir)
	c.mu.Unlock()
	return nil
}

// PulseGuide asserts dir, waits d (host-timed), then releases, the object's
// CCameraBase::pulseGuide shape; the firmware does not gate the pulse width.
func (c *Camera) PulseGuide(dir GuideDir, d time.Duration) error {
	c.mu.Lock()
	c.st4Pulses++
	c.mu.Unlock()
	defer func() {
		c.mu.Lock()
		c.st4Pulses--
		c.mu.Unlock()
	}()
	if err := c.PulseGuideOn(dir); err != nil {
		return err
	}
	time.Sleep(d)
	return c.PulseGuideOff(dir)
}

// IsPulseGuiding reports whether any ST4 line is asserted or a host-timed PulseGuide is
// in flight (the ASCOM IsPulseGuiding backing state). Line state is tracked per direction, so
// overlapping On/On/Off/Off from concurrent callers resolves correctly.
func (c *Camera) IsPulseGuiding() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.st4Lines != 0 || c.st4Pulses > 0
}
