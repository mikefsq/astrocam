package astrocam

// ZWO cooling: the control-transfer Thermal backend the Cooler loop drives (cooling.go), for
// FPGA-cooled models (ASI2600 / ASI6200):
//
//	ReadTemp     GetSensorTemp    : SendCMD 0xB3 IN 2B; signed 12-bit (hi<<4 | lo>>4) × 1/16 °C
//	                                (zwoTempC; PlayerOne packs the same reading differently)
//	ReadHumidity GetHumidity      : SendCMD(0x85, wValue 0xF5) IN 2B; Sensirion RH
//	SetTECPower  SetFPGACoolPower : WriteFPGAREG reg 0x26
//	SetFan       EnableCfan       : RMW FPGA reg 0x19 bit7 (0x80)
//	SetHeater    SetLensHeat      : WriteFPGAREG reg 0x2a = 8-bit PWM duty (0..255), + EnableWarm
//	             EnableWarm       : RMW FPGA reg 0x19 bit6 (0x40)
//
// The registers are ZWO's alone: PlayerOne puts a different actuator behind 0x26 and 0x2a, so
// its backend is a separate type (thermal_poa.go) rather than a parameterisation of this one.
//
// Non-FPGA cameras (ASI071/1600 class) drive the TEC through a calibrated DAC (SetDA cubic →
// SendCMD 0xB2) and read temperature via I2C ADC chips; that path is not implemented and would
// need its own Thermal.

import "fmt"

// FPGA register numbers for the cooling block (wValue to WriteFPGAREG 0xBD).
const (
	fpgaCoolPower = 0x26 // SetFPGACoolPower: TEC drive, 0..0xff
	fpgaCoolFlags = 0x19 // EnableCfan bit7 (0x80) = fan, EnableWarm bit6 (0x40) = anti-dew heater
	fpgaHeatPower = 0x2a // SetFPGAHeaterPowerPercent: anti-dew heater 8-bit PWM duty, 0..255
)

const (
	reqReadTemp     = 0xB3 // GetSensorTemp: 2-byte IN
	reqReadHumidity = 0x85 // GetHumidity: 2-byte IN, wValue 0xF5
	humidityWValue  = 0xF5
)

// camThermal maps the Thermal interface to control transfers for one camera: the Transport, the
// Regmap (FPGA register RMW) and the vendor's FX3 request codes for the temperature/humidity
// reads. writes is the Camera's shared write cache (nil = write every call).
type camThermal struct {
	t      Transport
	rm     Regmap
	cmds   FX3Cmds
	vend   string
	writes *coolWrites
}

// zwoThermal builds ZWO's cooling backend.
func zwoThermal(c *Camera) Thermal {
	return &camThermal{t: c.t, rm: c.rm, cmds: c.vend.Cmds, vend: c.vend.Name, writes: &c.coolW}
}

// ReadTemp reads the sensor temperature in °C (GetSensorTemp): a 2-byte IN packed as a signed
// 12-bit fixed-point value (hi << 4) | (lo >> 4), temp = signed12 × 1/16.
func (h *camThermal) ReadTemp() (float64, error) {
	if h.cmds.ReadTemp == 0 || h.cmds.TempC == nil {
		return 0, fmt.Errorf("astrocam: FX3 ReadTemp not decoded for vendor %s", h.vend)
	}
	b := make([]byte, h.cmds.readTempBytes())
	if _, err := h.t.ControlIn(h.cmds.ReadTemp, 0, 0, b); err != nil {
		return 0, err
	}
	return h.cmds.TempC(b), nil
}

// zwoTempC decodes ZWO's GetSensorTemp reply: a signed 12-bit value packed as (hi<<4 | lo>>4),
// in sixteenths of a degree.
func zwoTempC(b []byte) float64 {
	if len(b) < 2 {
		return 0
	}
	raw := int(b[1])<<4 | int(b[0])>>4
	if raw >= 0x800 { // sign-extend
		raw -= 0x1000
	}
	return float64(raw) * 0.0625
}

// ReadHumidity reads relative humidity % (GetHumidity): a 2-byte LE IN run through the
// Sensirion transfer function RH = -6 + 125·raw/2^16, clamped 0..100.
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

// SetTECPower sets the TEC drive 0..100 % (SetFPGACoolPower → WriteFPGAREG 0x26, an 8-bit level:
// the percentage maps linearly onto 0..255). It also runs the fan whenever power > 0: the TEC's
// hot side must be fanned while driven. A level or fan state the FPGA already holds is not
// rewritten (coolWrites), so a steady regulation loop costs no control transfers between
// refreshes.
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
	level := uint16(percent/100*255 + 0.5)
	if h.writes != nil && !h.writes.levelDue(level) {
		return nil
	}
	if err := h.rm.WriteFPGAReg(fpgaCoolPower, level); err != nil {
		if h.writes != nil {
			h.writes.invalidate() // the register state is unknown after a failed write
		}
		return err
	}
	if h.writes != nil {
		h.writes.levelWritten(level)
	}
	return nil
}

// SetFan turns the cooling fan on/off (EnableCfan → FPGA reg 0x19 bit7, RMW). The bit is
// active-low: clear bit7 to run the fan, set it to stop. A state the FPGA already holds is not
// rewritten (coolWrites).
func (h *camThermal) SetFan(on bool) error {
	if h.writes != nil && !h.writes.fanDue(on) {
		return nil
	}
	if err := SetFPGABit(h.rm, fpgaCoolFlags, 0x80, !on); err != nil {
		if h.writes != nil {
			h.writes.invalidate()
		}
		return err
	}
	if h.writes != nil {
		h.writes.fanWritten(on)
	}
	return nil
}

// SetHeater sets the anti-dew lens heater 0..100 % (SetLensHeat → WriteFPGAREG 0x2a, an 8-bit
// PWM duty the percentage maps onto, then EnableWarm → FPGA reg 0x19 bit6 to gate it on/off).
func (h *camThermal) SetHeater(percent int) error {
	if percent < 0 {
		percent = 0
	}
	if percent > 100 {
		percent = 100
	}
	if err := h.rm.WriteFPGAReg(fpgaHeatPower, uint16(float64(percent)/100*255+0.5)); err != nil {
		return err
	}
	return SetFPGABit(h.rm, fpgaCoolFlags, 0x40, percent > 0)
}
