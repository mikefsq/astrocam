package asicam

// Hardware Thermal backend: the control-transfer implementation of the Thermal seam the
// Cooler drives (cooling.go):
//
//	ReadTemp     GetSensorTemp — SendCMD 0xB3 IN 2B; °C = int8(hi) + lo/16
//	ReadHumidity GetHumidity   — SendCMD(0x85,wValue 0xF5) IN 2B; Sensirion RH
//	SetTECPower  SetFPGACoolPower — WriteFPGAREG reg 0x26 (FPGA-cooled models)
//	SetFan       EnableCfan      — RMW FPGA reg 0x19 bit7 (0x80)
//	SetHeater    SetLensHeat    — WriteFPGAREG reg 0x2a = %, + EnableWarm
//	             EnableWarm       — RMW FPGA reg 0x19 bit6 (0x40)
//
// In the SDK, SendCMD/WriteFPGAREG/ReadFPGAREG are just libusb_control_transfer wrappers,
// which are exactly Transport.ControlIn/ControlOut and the Regmap FPGA accessors here.
//
// VARIANTS NOT REPLICATED (no hardware to verify): older non-FPGA cameras (ASI071/1600
// class) drive the TEC through a calibrated DAC — SetDA → the power-to-DAC conversion (float) cubic →
// SendCMD 0xB2 with per-model max-power from InitCooling — and read temperature through
// I2C ADC chips (GetADC081Temp / GetAD7142Temp / GetTMP451Temp) selected by a member
// jump table. This backend implements the modern FPGA-cooled path (ASI2600 / ASI6200),
// which is hardware-validated below; a DAC-path model would need its own Thermal.

// FPGA register numbers for the cooling block (wValue to WriteFPGAREG 0xBD).
const (
	fpgaCoolPower = 0x26 // SetFPGACoolPower: TEC drive, 0..0xff
	fpgaCoolFlags = 0x19 // EnableCfan bit7 (0x80) = fan, EnableWarm bit6 (0x40) = anti-dew heater
	fpgaHeatPower = 0x2a // SetFPGAHeaterPowerPercent: anti-dew heater %, 0..100
)

const (
	reqReadTemp     = 0xB3 // GetSensorTemp: 2-byte IN
	reqReadHumidity = 0x85 // GetHumidity: 2-byte IN, wValue 0xF5
	humidityWValue  = 0xF5
)

// camThermal maps the Thermal interface to ZWO control transfers for one camera. It holds
// the Transport (SendCMD-style control transfers) and the Regmap (FPGA register RMW).
type camThermal struct {
	t  Transport
	rm Regmap
}

// HardwareThermal returns the control-transfer Thermal backend for this camera, for
// EnableCooling on a real (FPGA-cooled) camera: cam.EnableCooling(cam.HardwareThermal(), …).
func (c *Camera) HardwareThermal() Thermal { return &camThermal{t: c.t, rm: c.rm} }

// ReadTemp reads the sensor temperature in °C (GetSensorTemp): a
// 2-byte IN packed as a signed 12-bit fixed-point value — temp = signed12 × 1/16, where
// the 12 bits are (hi << 4) | (lo >> 4). So the high byte is the integer °C and the high
// nibble of the low byte is the sixteenths. HARDWARE-VALIDATED: raw a0/23 → 35.625 °C,
// matching the SDK's ASIGetControlValue(ASI_TEMPERATURE)=356 (35.6 °C) on the ASI6200MC Pro.
func (h *camThermal) ReadTemp() (float64, error) {
	var b [2]byte
	if _, err := h.t.ControlIn(reqReadTemp, 0, 0, b[:]); err != nil {
		return 0, err
	}
	raw := int(b[1])<<4 | int(b[0])>>4 // 12-bit fixed point
	if raw >= 0x800 {                  // sign-extend (temperatures go below 0 when cooling)
		raw -= 0x1000
	}
	return float64(raw) * 0.0625, nil
}

// ReadHumidity reads relative humidity % (GetHumidity): a 2-byte LE
// IN run through the Sensirion transfer function RH = -6 + 125·raw/2^16, clamped 0..100.
// UNVERIFIED: the ASI6200MC Pro reads 0 % (it likely has no RH sensor, or it
// reports only with the cooler powered) — the formula is a faithful transcription, not yet
// cross-checked against live data on a humidity-equipped body.
func (h *camThermal) ReadHumidity() (int, error) {
	var b [2]byte
	if _, err := h.t.ControlIn(reqReadHumidity, humidityWValue, 0, b[:]); err != nil {
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
// It also gates the cooling fan on power, exactly as the SDK's SetPowerPerc does
// (EnableCfan when power > 0): the TEC's hot side MUST be fanned while it is driven, so
// the fan follows the drive — on whenever there is power, off at 0.
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
// ACTIVE-LOW: EnableCfan clears bit7 to run the fan and sets it to stop it (a read-
// modify-write via bfxil that preserves bits 0-6). Getting this backwards runs the TEC
// with the fan off and overheats the hot side — so on => clear bit7 (!on).
// WIRE-VALIDATED (Packetry capture): cooler-on writes reg 0x19 = 0x00 (fan on) then reg
// 0x26 = power; cooler-off writes reg 0x19 = 0x80 (fan off) then reg 0x26 = 0.
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
	// reg 0x2a is an 8-bit PWM duty (0..255), not a 0..100 percent — the SDK's "on" writes
	// 197 (≈77%). Map the percentage onto the full duty (same as the cool-power reg 0x26),
	// then gate it via EnableWarm (reg 0x19 bit6).
	if err := h.rm.WriteFPGAReg(fpgaHeatPower, uint16(float64(percent)/100*255+0.5)); err != nil {
		return err
	}
	return SetFPGABit(h.rm, fpgaCoolFlags, 0x40, percent > 0)
}
