package astrocam

import "time"

// ST4 autoguider pulse output: PulseGuideOn / PulseGuideOff / pulseGuide.
//
// Each is a single vendor OUT control transfer issued through the generic
// SendCMD(bReq, wValue, wIndex, isRead=0, data=NULL, wLen=0) — i.e. bmRequestType
// 0x40, no data stage. The DIRECTION rides in
// wValue (PulseGuideOn: `and w2, dir, #0xffff` → arg2 = wValue), NOT a
// fixed 0. The bare 1-arg sendCmd in protocol.go (wValue=0) would always guide
// North; ST4 must carry the direction here.

// GuideDir is an ST4 pulse direction, matching ASI_GUIDE_DIRECTION in
// ASICamera2.h (NORTH=0, SOUTH=1, EAST=2, WEST=3).
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

// PulseGuideOn asserts the ST4 line for dir (SendCMD 0xB0, wValue=dir). Mirrors
// PulseGuideOn: dir>3 is a no-op (the SDK returns success without a
// transfer), so an out-of-range direction silently does nothing.
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

// PulseGuide asserts dir, waits d, then releases — pulseGuide
// (on → usleep(ms*1000) → off). The wait is host-timed, exactly as the SDK does
// it; the firmware does not gate the pulse width.
func (c *Camera) PulseGuide(dir GuideDir, d time.Duration) error {
	if err := c.PulseGuideOn(dir); err != nil {
		return err
	}
	time.Sleep(d)
	return c.PulseGuideOff(dir)
}
