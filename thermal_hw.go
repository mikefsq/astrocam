package astrocam

// Hardware Thermal backend: the control-transfer implementation of the Thermal seam
// the Cooler drives (cooling.go), for FPGA-cooled models (ASI2600 / ASI6200):
//
//	ReadTemp     GetSensorTemp    — SendCMD 0xB3 IN 2B; signed 12-bit (hi<<4 | lo>>4) × 1/16 °C
//	ReadHumidity GetHumidity      — SendCMD(0x85, wValue 0xF5) IN 2B; Sensirion RH
//	SetTECPower  SetFPGACoolPower — WriteFPGAREG reg 0x26
//	SetFan       EnableCfan       — RMW FPGA reg 0x19 bit7 (0x80)
//	SetHeater    SetLensHeat      — WriteFPGAREG reg 0x2a = %, + EnableWarm
//	             EnableWarm       — RMW FPGA reg 0x19 bit6 (0x40)
//
// SendCMD/WriteFPGAREG/ReadFPGAREG map to Transport.ControlIn/ControlOut and the
// Regmap FPGA accessors. Non-FPGA cameras (ASI071/1600 class) instead drive the TEC
// through a calibrated DAC (SetDA cubic → SendCMD 0xB2) and read temperature via I2C
// ADC chips; that path is not implemented and would need its own Thermal.

import "fmt"

// FPGA register numbers for the cooling block (wValue to WriteFPGAREG 0xBD).
const (
	fpgaCoolPower = 0x26 // SetFPGACoolPower: TEC drive, 0..0xff
	fpgaCoolFlags = 0x19 // EnableCfan bit7 (0x80) = fan, EnableWarm bit6 (0x40) = anti-dew heater
	fpgaHeatPower = 0x2a // SetFPGAHeaterPowerPercent: anti-dew heater 8-bit PWM duty, 0..255 (percent mapped onto it)
)

const (
	reqReadTemp     = 0xB3 // GetSensorTemp: 2-byte IN
	reqReadHumidity = 0x85 // GetHumidity: 2-byte IN, wValue 0xF5
	humidityWValue  = 0xF5
)

// camThermal maps the Thermal interface to control transfers for one camera, holding
// the Transport (control transfers), the Regmap (FPGA register RMW) and the vendor's
// FX3 request codes for the temperature/humidity reads.
type camThermal struct {
	t    Transport
	rm   Regmap
	cmds FX3Cmds
	vend string
}

// HardwareThermal returns the control-transfer Thermal backend for this (FPGA-cooled)
// camera: cam.EnableCooling(cam.HardwareThermal(), …).
func (c *Camera) HardwareThermal() Thermal {
	return &camThermal{t: c.t, rm: c.rm, cmds: c.vend.Cmds, vend: c.vend.Name}
}

// ReadTemp reads the sensor temperature in °C (GetSensorTemp): a 2-byte IN packed as
// a signed 12-bit fixed-point value, temp = signed12 × 1/16, where the 12 bits are
// (hi << 4) | (lo >> 4). High byte is the integer °C, high nibble of the low byte is
// the sixteenths.
func (h *camThermal) ReadTemp() (float64, error) {
	if h.cmds.ReadTemp == 0 {
		return 0, fmt.Errorf("astrocam: FX3 ReadTemp not decoded for vendor %s", h.vend)
	}
	var b [2]byte
	if _, err := h.t.ControlIn(h.cmds.ReadTemp, 0, 0, b[:]); err != nil {
		return 0, err
	}
	raw := int(b[1])<<4 | int(b[0])>>4 // 12-bit fixed point
	if raw >= 0x800 {                  // sign-extend (temperatures go below 0 when cooling)
		raw -= 0x1000
	}
	return float64(raw) * 0.0625, nil
}

// ReadHumidity reads relative humidity % (GetHumidity): a 2-byte LE IN run through
// the Sensirion transfer function RH = -6 + 125·raw/2^16, clamped 0..100.
func (h *camThermal) ReadHumidity() (int, error) {
	if h.cmds.ReadHumidity == 0 {
		return 0, fmt.Errorf("astrocam: FX3 ReadHumidity not decoded for vendor %s", h.vend)
	}
	var b [2]byte
	if _, err := h.t.ControlIn(h.cmds.ReadHumidity, h.cmds.ReadHumidityWValue, 0, b[:]); err != nil {
		return 0, err
	}
	raw := int(b[0]) | int(b[1])<<8
	rh := 125*raw/65536 - 6
	if rh < 0 {
		rh = 0
	}
	if rh > 100 {
		rh = 100
	}
	return rh, nil
}

// SetTECPower sets the TEC drive 0..100 % (SetFPGACoolPower → WriteFPGAREG 0x26). The
// FPGA register is an 8-bit drive level, so the percentage maps linearly onto 0..255.
// Also gates the cooling fan on power (on whenever power > 0, off at 0): the TEC's
// hot side must be fanned while driven.
func (h *camThermal) SetTECPower(percent float64) error {
	if percent < 0 {
		percent = 0
	}
	if percent > 100 {
		percent = 100
	}
	if err := h.SetFan(percent > 0); err != nil {
		return err
	}
	return h.rm.WriteFPGAReg(fpgaCoolPower, uint16(percent/100*255+0.5))
}

// SetFan turns the cooling fan on/off (EnableCfan → FPGA reg 0x19 bit7). The bit is
// active-low: clear bit7 to run the fan, set it to stop (RMW preserving bits 0-6), so
// on => clear bit7 (!on).
func (h *camThermal) SetFan(on bool) error {
	return SetFPGABit(h.rm, fpgaCoolFlags, 0x80, !on)
}

// SetHeater sets the anti-dew lens heater 0..100 % (SetLensHeat → WriteFPGAREG 0x2a for the
// power, then EnableWarm → FPGA reg 0x19 bit6 to gate it on/off).
func (h *camThermal) SetHeater(percent int) error {
	if percent < 0 {
		percent = 0
	}
	if percent > 100 {
		percent = 100
	}
	// reg 0x2a is an 8-bit PWM duty (0..255), not a 0..100 percent. Map the percentage
	// onto the full duty (as with cool-power reg 0x26), then gate it via EnableWarm
	// (reg 0x19 bit6).
	if err := h.rm.WriteFPGAReg(fpgaHeatPower, uint16(float64(percent)/100*255+0.5)); err != nil {
		return err
	}
	return SetFPGABit(h.rm, fpgaCoolFlags, 0x40, percent > 0)
}
