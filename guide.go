package astrocam

import "time"

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

// PulseGuideOn asserts the ST4 line for dir (SendCMD 0xB0, wValue=dir). dir>3 is a no-op.
func (c *Camera) PulseGuideOn(dir GuideDir) error {
	if dir > GuideWest {
		return nil
	}
	return c.t.ControlOut(cmdST4N, uint16(dir), 0, nil)
}

// PulseGuideOff releases the ST4 line for dir (SendCMD 0xB1, wValue=dir).
func (c *Camera) PulseGuideOff(dir GuideDir) error {
	if dir > GuideWest {
		return nil
	}
	return c.t.ControlOut(cmdST4F, uint16(dir), 0, nil)
}

// PulseGuide asserts dir, waits d (host-timed), then releases. The firmware does not gate
// the pulse width.
func (c *Camera) PulseGuide(dir GuideDir, d time.Duration) error {
	if err := c.PulseGuideOn(dir); err != nil {
		return err
	}
	time.Sleep(d)
	return c.PulseGuideOff(dir)
}
