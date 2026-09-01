package sensors

import . "github.com/mikefsq/astrocam"

// init wires each camera USB (VID,PID) to its sensor profile and per-variant Name/Color/
// USB3/Cooling/ST4 flags, for both ZWO (VID 0x03C3) and PlayerOne (VID 0xA0A0). Sensor
// profiles are keyed by Sony die and shared across all variants. A nil Sensor means no
// profile yet, so Open() returns "no profile yet", not "unknown".
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

	// PlayerOne (VID 0xA0A0) builds the same Sony dies onto the same FX3 bridge; the sensor
	// profiles are reused, only the Regmap dialect differs (poaRegmap). The PID encodes the
	// Sony sensor number in its top three nibbles (0x462x→IMX462, 0x585x→IMX585,
	// 0x455x→IMX455, …), low nibble = variant (even = -C, odd = -M).
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
	p(0x5853, "Xena 585M", &IMX585, false, true, false, true) // the body this die was brought up on
	p(0x5854, "Uranus-C (v2)", &IMX585, true, true, false, true)
	p(0x5855, "Uranus-M (v3)", &IMX585, false, true, false, true)
}
