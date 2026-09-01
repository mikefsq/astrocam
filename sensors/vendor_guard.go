package sensors

import (
	"fmt"

	. "github.com/mikefsq/astrocam"
)

// The vendor is fixed at Open from the USB (VID,PID), so a camera drives exactly one vendor's
// dialect and the two never mix on the wire. What DOES mix is this package: the sensor profiles
// are keyed by Sony die and shared across vendors, because the die's own registers carry over.
// The FPGA's do not — it is vendor firmware with its own map, and the overlaps are live loads,
// not inert addresses:
//
//	reg 0x00 bit4  ZWO stop the readout    PlayerOne part of the START value
//	reg 0x04/0x08  ZWO width / height      PlayerOne readout mode byte / crop burst
//	reg 0x0c       ZWO analog gain block   PlayerOne image-size burst
//	reg 0x26       ZWO TEC cooling power   PlayerOne window heater
//
// Most profiles here were decoded on a ZWO camera, so their FPGA bringup and geometry are
// ZWO's. Running those against a PlayerOne body does not fail loudly — it silently
// misprograms the camera, which is worse. poaUnsupported turns that into an error at the entry
// point. The IMX585 is the exception: its PlayerOne path is decoded end to end and dispatches on
// the VID, so it never reaches this guard.
//
// It gates only the halves that are actually ZWO-specific. Where a profile has a decoded and
// dispatched PlayerOne encoding (the IMX455 and IMX571 gain and offset paths), that keeps working.
func poaUnsupported(rm Regmap, sensor, what string) error {
	if rm.VID() == POA.VID {
		return fmt.Errorf("%s: %s not implemented for PlayerOne (its FPGA register map differs)", sensor, what)
	}
	return nil
}
