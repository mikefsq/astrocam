package astrocam

// SonyAnalogGain converts an ASI gain value (0.1 dB units) to a Sony STARVIS analog-gain
// register code plus the high-conversion-gain (HCG) select. The Sony analog gain step is
// 0.3 dB, so the code is gain/3. Above hcgThresholdTenthDb the sensor switches to high
// conversion gain: the scale is rebased ((gain-threshold)/3) and the HCG bit is set.
func SonyAnalogGain(asiGain, hcgThresholdTenthDb int) (code uint16, hcg bool) {
	if asiGain > hcgThresholdTenthDb {
		return uint16((asiGain - hcgThresholdTenthDb) / 3), true
	}
	return uint16(asiGain / 3), false
}
