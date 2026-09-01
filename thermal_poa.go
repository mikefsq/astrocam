package astrocam

import (
	"fmt"
	"math"
)

// PlayerOne cooling: the actuators live in the camera FPGA, at registers that overlap ZWO's with
// DIFFERENT meanings — ZWO's TEC power register 0x26 is PlayerOne's window heater, and ZWO's
// heater PWM 0x2a is PlayerOne's read-only status. Driving a PlayerOne body through camThermal
// would therefore energise the wrong load, so the Thermal backend travels with the vendor.
//
//	SetTECPower  : FPGA reg 0x25
//	SetHeater    : FPGA reg 0x26
//	SetFan       : FPGA reg 0x27
//	ReadTemp     : vendor IN 0xA8, signed 16-bit tenths (poaTempC)
//
// NOT HARDWARE-VALIDATED: the Xena 585M this vendor was brought up on is uncooled, so every
// write below is decoded but unexercised. The registers and the curves are exact;
// what is untested is the camera's response.
//
// PlayerOne also carries a firmware-side regulation loop, reached through vendor requests 0xA9
// and 0xAA, which this backend deliberately does not use: astrocam regulates in the
// host Cooler loop and drives the TEC level directly, as it does on ZWO. Whether the firmware
// loop must be disabled first, or whether the TEC responds at all with it off, is unverified.
const (
	poaFPGACool   = 0x25 // TEC drive level, 0..255
	poaFPGAWarm   = 0x26 // window heater level, 0..255
	poaFPGAFan    = 0x27 // fan level, 0..255
	poaFPGAStatus = 0x2a // status byte (read-only)
	poaFPGAVer    = 0x2b // FPGA firmware version (read-only)
)

// poaDriveLevel reproduces the SDK's TEC and heater curve: the caller's per-mille demand is clamped to
// 1000, then mapped onto the 8-bit drive by a SQUARE ROOT, not linearly —
// level = trunc(sqrt(v/1000) × 255) — with a non-zero demand never rounding down to a dead zero.
// The curve linearises the load's response, so a naive percentage would drive far too hard at the
// bottom of the range.
func poaDriveLevel(perMille int) uint16 {
	if perMille <= 0 {
		return 0
	}
	if perMille > 1000 {
		perMille = 1000
	}
	lvl := uint16(math.Sqrt(float64(perMille)/1000) * 255)
	if lvl == 0 {
		lvl = 1
	}
	return lvl
}

// poaFanLevel reproduces the SDK's fan curve: a plain 0..100 percentage onto 0..255.
func poaFanLevel(percent int) uint16 {
	if percent <= 0 {
		return 0
	}
	if percent > 100 {
		percent = 100
	}
	return uint16(percent * 255 / 100)
}

// poaThermal is the PlayerOne Thermal backend. writes is the Camera's shared write cache, so a
// steady regulation loop costs no control transfers between refreshes, exactly as on ZWO.
type poaThermal struct {
	t      Transport
	rm     Regmap
	cmds   FX3Cmds
	writes *coolWrites
}

// ReadTemp reads the sensor temperature through the vendor's request and decode (0xA8, signed
// 16-bit tenths of a degree).
func (h *poaThermal) ReadTemp() (float64, error) {
	if h.cmds.ReadTemp == 0 || h.cmds.TempC == nil {
		return 0, fmt.Errorf("astrocam: FX3 ReadTemp not decoded for vendor PlayerOne")
	}
	b := make([]byte, h.cmds.readTempBytes())
	if _, err := h.t.ControlIn(h.cmds.ReadTemp, 0, 0, b); err != nil {
		return 0, err
	}
	return h.cmds.TempC(b), nil
}

// ReadHumidity has no PlayerOne counterpart: the bridge answers no humidity request. The SDK's
// POAGetHumiAndTemp is undecoded, so this errors rather than reading ZWO's 0x85.
func (h *poaThermal) ReadHumidity() (int, error) {
	return 0, fmt.Errorf("astrocam: humidity read not decoded for vendor PlayerOne")
}

// SetTECPower sets the TEC drive 0..100 %, through the square-root curve onto FPGA reg 0x25. The
// fan runs whenever the TEC is driven, as on ZWO: the hot side must be fanned.
func (h *poaThermal) SetTECPower(percent float64) error {
	if percent < 0 {
		percent = 0
	}
	if percent > 100 {
		percent = 100
	}
	if err := h.SetFan(percent > 0); err != nil {
		return err
	}
	lvl := poaDriveLevel(int(percent * 10)) // percent -> per-mille
	if h.writes != nil && !h.writes.levelDue(lvl) {
		return nil
	}
	if err := h.rm.WriteFPGAReg(poaFPGACool, lvl); err != nil {
		return err
	}
	if h.writes != nil {
		h.writes.levelWritten(lvl)
	}
	return nil
}

// SetFan drives the fan at full or off (FPGA reg 0x27). PlayerOne's fan is a percentage; the
// Thermal seam is a switch, so "on" is 100 %.
func (h *poaThermal) SetFan(on bool) error {
	if h.writes != nil && !h.writes.fanDue(on) {
		return nil
	}
	lvl := uint16(0)
	if on {
		lvl = poaFanLevel(100)
	}
	if err := h.rm.WriteFPGAReg(poaFPGAFan, lvl); err != nil {
		return err
	}
	if h.writes != nil {
		h.writes.fanWritten(on)
	}
	return nil
}

// SetHeater sets the anti-dew window heater 0..100 %, through the same square-root curve onto
// FPGA reg 0x26.
func (h *poaThermal) SetHeater(percent int) error {
	return h.rm.WriteFPGAReg(poaFPGAWarm, poaDriveLevel(percent*10))
}

// POAFPGAVersion reads the camera-FPGA firmware version (reg 0x2b). This is the
// FPGA's own version, distinct from Camera.FirmwareVersion, which reads the FX3 bridge (0xA2).
func POAFPGAVersion(rm Regmap) (uint16, error) { return rm.ReadFPGAReg(poaFPGAVer) }

// POAFPGAStatus reads the FPGA status byte (reg 0x2a). The bit meanings are not
// decoded; the raw byte is returned.
func POAFPGAStatus(rm Regmap) (uint16, error) { return rm.ReadFPGAReg(poaFPGAStatus) }
