package astrocam

// SonyAnalogGain converts an ASI gain value (0.1 dB units, the API scale) to a Sony
// STARVIS analog-gain register code plus the high-conversion-gain (HCG) select. The
// Sony analog gain step is 0.3 dB, so the code is simply gain/3. Above
// hcgThresholdTenthDb the sensor switches to high conversion gain: the scale is rebased
// at the threshold ((gain-threshold)/3) and the HCG bit is set.
//
// The SDK emits this /3 as compiler fixed-point — different SetGain bodies
// use gain*0xab>>9, (gain-thr)*0xaaab>>17, or gain*0x5556>>16 — all of which equal
// gain/3 over the reachable 0..maxGain domain (the >>9 form only diverges above 512,
// which the low branch, gated by gain<=threshold<=181, never reaches). Expressing it as
// /3 is the faithful intent; the hex multipliers are compiler noise, not sensor facts.
func SonyAnalogGain(asiGain, hcgThresholdTenthDb int) (code uint16, hcg bool) {
	if asiGain > hcgThresholdTenthDb {
		return uint16((asiGain - hcgThresholdTenthDb) / 3), true
	}
	return uint16(asiGain / 3), false
}
