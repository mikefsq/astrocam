package sensors

import . "github.com/mikefsq/astrocam"

// init wires each camera USB (VID,PID) to its sensor profile and per-variant Name/Color/
// USB3/Cooling/ST4 flags — for BOTH vendors that build on this platform: ZWO (VID 0x03C3)
// and PlayerOne (VID 0xA0A0). The sensor profiles are vendor-independent (keyed by Sony die),
// so the same *Sensor is shared across a ZWO and a PlayerOne camera that use the same silicon.
//
// Sensor bindings are by silicon (one *Sensor per die, shared across all variants).
// Sensors without a profile yet use a nil Sensor so Open() returns
// "no profile yet", not "unknown".
func init() {
	m := func(pid uint16, name string, s *Sensor, color, usb3, cooled, st4 bool) {
		Register(ZWO.VID, pid, Model{Name: name, Sensor: s, Color: color, USB3: usb3, Cooled: cooled, ST4: st4})
	}

	m(0x1749, "ASI174MM Mini", &IMX174, false, false, false, true)
	m(0x2601, "ASI2600MM", &IMX571, false, true, true, false)
	m(0x2602, "ASI2600MC", &IMX571, true, true, true, false)
	m(0x260A, "ASI2600MC Pro", &IMX571, true, true, true, false)
	m(0x260E, "ASI2600MM Pro", &IMX571, false, true, true, false)
	m(0x290F, "ASI290MM Mini", &IMX290, false, false, false, true)
	m(0x462b, "ASI462MC", &IMX462, true, true, false, true)
	m(0x620A, "ASI6200MC Pro", &IMX455, true, true, true, false)
	m(0x620B, "ASI6200MM Pro", &IMX455, false, true, true, false)

	// PlayerOne (USB VID 0xA0A0) builds the SAME Sony dies onto the same FX3 bridge, so the
	// vendor-independent sensor profiles are reused VERBATIM — only the Regmap dialect differs
	// (poaRegmap, the POA Vendor). The PID→die: the PID encodes the Sony sensor number in its top
	// three nibbles (0x462x→IMX462, 0x585x→IMX585, 0x455x→IMX455, …), low nibble = variant. The model NAMES and
	// the color/cooled flags come from PlayerOne's published lineup cross-referenced with the
	// decoded die + the low-nibble color parity (even = -C, odd = -M; matches "Ceres 462M" mono on
	// the odd 462 PID). The SDK has no static PID→name table — POACameraProperties is assembled at
	// runtime — so per-PID variant/cool exactness (e.g. a "-Pro" cooled Uranus) may need a unit.
	p := func(pid uint16, name string, s *Sensor, color, usb3, cooled, st4 bool) {
		Register(POA.VID, pid, Model{Name: name, Sensor: s, Color: color, USB3: usb3, Cooled: cooled, ST4: st4})
	}
	p(0x1740, "Apollo-C", &IMX174, true, true, false, true) // IMX174 global shutter
	p(0x1741, "Apollo-M", &IMX174, false, true, false, true)
	p(0x1780, "Sedna-C", &IMX178, true, true, false, true) // IMX178
	p(0x1781, "Sedna-M", &IMX178, false, true, false, true)
	p(0x1782, "Sedna-C (v2)", &IMX178, true, true, false, true)
	p(0x1783, "Sedna-M (v2)", &IMX178, false, true, false, true)
	p(0x178b, "Sedna-M (v3)", &IMX178, false, true, false, true)
	p(0x4554, "ZEUS 455C PRO", &IMX455, true, true, true, false) // IMX455 full-frame, cooled
	p(0x4555, "ZEUS 455M PRO", &IMX455, false, true, true, false)
	p(0x4620, "Mars-C", &IMX462, true, true, false, true) // IMX462
	p(0x4621, "Mars-M", &IMX462, false, true, false, true)
	p(0x4623, "Ceres 462M", &IMX462, false, true, false, true) // IMX462 guide cam
	p(0x462a, "Mars-C (v2)", &IMX462, true, true, false, true)
	p(0x5714, "Poseidon-C PRO", &IMX571, true, true, true, false) // IMX571 APS-C, cooled
	p(0x5715, "Poseidon-M PRO", &IMX571, false, true, true, false)
	p(0x5850, "Uranus-C", &IMX585, true, true, false, true) // IMX585 STARVIS 2
	p(0x5851, "Uranus-M", &IMX585, false, true, false, true)
	p(0x5853, "Uranus-M (v2)", &IMX585, false, true, false, true)
	p(0x5854, "Uranus-C (v2)", &IMX585, true, true, false, true)
	p(0x5855, "Uranus-M (v3)", &IMX585, false, true, false, true)
}
