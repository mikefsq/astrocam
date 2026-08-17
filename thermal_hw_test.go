package astrocam

import (
	"testing"
	"time"
)

// thermalFakeTransport serves crafted bytes for the temperature / humidity vendor-INs.
type thermalFakeTransport struct {
	temp [2]byte
	hum  [2]byte
}

func (f *thermalFakeTransport) ControlOut(uint8, uint16, uint16, []byte) error { return nil }
func (f *thermalFakeTransport) ControlIn(bRequest uint8, _, _ uint16, data []byte) (int, error) {
	switch bRequest {
	case reqReadTemp:
		return copy(data, f.temp[:]), nil
	case reqReadHumidity:
		return copy(data, f.hum[:]), nil
	}
	return len(data), nil
}
func (f *thermalFakeTransport) BulkRead(buf []byte, _ time.Duration) (int, error) {
	return 0, nil
}
func (f *thermalFakeTransport) Close() error { return nil }

// TestReadTempDecode: GetSensorTemp is signed 12-bit fixed point, raw = (hi<<4)|(lo>>4),
// temp = raw/16, including sign extension below 0 °C.
func TestReadTempDecode(t *testing.T) {
	cases := []struct {
		lo, hi byte
		want   float64
	}{
		{0x88, 0x19, 25.5},    // raw 0x198 = 408 → 25.5
		{0x00, 0x00, 0.0},     // zero
		{0xC0, 0xF3, -12.25},  // raw 0xF3C → sign-extended −196 → −12.25
		{0xF0, 0xFF, -0.0625}, // raw 0xFFF → −1 → −1/16 (sign extension of the smallest step)
	}
	for _, c := range cases {
		th := &camThermal{t: &thermalFakeTransport{temp: [2]byte{c.lo, c.hi}}, cmds: ZWO.Cmds}
		got, err := th.ReadTemp()
		if err != nil {
			t.Fatalf("ReadTemp(%02x %02x): %v", c.lo, c.hi, err)
		}
		if got != c.want {
			t.Errorf("ReadTemp(lo=%02x hi=%02x) = %v, want %v", c.lo, c.hi, got, c.want)
		}
	}
}

// TestReadHumidityDecode: the Sensirion transfer function RH = 125·raw/2^16 − 6, clamped to
// 0..100.
func TestReadHumidityDecode(t *testing.T) {
	cases := []struct {
		lo, hi byte
		want   int
	}{
		{0x00, 0x80, 56},  // raw 0x8000 → 125·32768/65536−6 = 56 (int math)
		{0x00, 0x00, 0},   // raw 0 → −6 → clamped to 0
		{0xFF, 0xFF, 100}, // raw 0xFFFF → ~119 → clamped to 100
	}
	for _, c := range cases {
		th := &camThermal{t: &thermalFakeTransport{hum: [2]byte{c.lo, c.hi}}, cmds: ZWO.Cmds}
		got, err := th.ReadHumidity()
		if err != nil {
			t.Fatalf("ReadHumidity(%02x %02x): %v", c.lo, c.hi, err)
		}
		if got != c.want {
			t.Errorf("ReadHumidity(lo=%02x hi=%02x) = %d, want %d", c.lo, c.hi, got, c.want)
		}
	}
}

// countRegmap counts FPGA register traffic and serves a settable reg 0x19.
type countRegmap struct {
	plainRegmap
	writes, reads int
	flags         uint16
}

func (r *countRegmap) WriteFPGAReg(reg, val uint16) error {
	r.writes++
	if reg == fpgaCoolFlags {
		r.flags = val
	}
	return nil
}
func (r *countRegmap) ReadFPGAReg(reg uint16) (uint16, error) { r.reads++; return r.flags, nil }

// TestCoolWritesSkipUnchanged: the hardware Thermal writes the TEC level and the fan RMW once
// per change, skips them while unchanged, rewrites them after coolRefresh, and after
// invalidate.
func TestCoolWritesSkipUnchanged(t *testing.T) {
	now := time.Unix(0, 0)
	w := &coolWrites{nowStamp: func() time.Time { return now }}
	rm := &countRegmap{}
	th := &camThermal{rm: rm, cmds: ZWO.Cmds, writes: w}

	// First call: fan RMW (1 read + 1 write) + level write.
	if err := th.SetTECPower(50); err != nil {
		t.Fatal(err)
	}
	if rm.reads != 1 || rm.writes != 2 {
		t.Fatalf("first SetTECPower: reads=%d writes=%d, want 1/2", rm.reads, rm.writes)
	}
	// Same level (50 % and 50.1 % quantize to the same 8-bit level), same fan: no traffic.
	for _, p := range []float64{50, 50.1, 50} {
		if err := th.SetTECPower(p); err != nil {
			t.Fatal(err)
		}
	}
	if rm.reads != 1 || rm.writes != 2 {
		t.Fatalf("unchanged SetTECPower issued traffic: reads=%d writes=%d", rm.reads, rm.writes)
	}
	// A new level writes reg 0x26 only (fan state unchanged).
	if err := th.SetTECPower(60); err != nil {
		t.Fatal(err)
	}
	if rm.reads != 1 || rm.writes != 3 {
		t.Fatalf("changed level: reads=%d writes=%d, want 1/3", rm.reads, rm.writes)
	}
	// Power 0 flips the fan: RMW again + level.
	if err := th.SetTECPower(0); err != nil {
		t.Fatal(err)
	}
	if rm.reads != 2 || rm.writes != 5 {
		t.Fatalf("fan off: reads=%d writes=%d, want 2/5", rm.reads, rm.writes)
	}
	// After the refresh interval both are rewritten though unchanged.
	now = now.Add(coolRefresh)
	if err := th.SetTECPower(0); err != nil {
		t.Fatal(err)
	}
	if rm.reads != 3 || rm.writes != 7 {
		t.Fatalf("refresh: reads=%d writes=%d, want 3/7", rm.reads, rm.writes)
	}
	// invalidate forces the next call onto the wire.
	w.invalidate()
	if err := th.SetTECPower(0); err != nil {
		t.Fatal(err)
	}
	if rm.reads != 4 || rm.writes != 9 {
		t.Fatalf("after invalidate: reads=%d writes=%d, want 4/9", rm.reads, rm.writes)
	}
	// Without a cache every call writes.
	bare := &camThermal{rm: rm, cmds: ZWO.Cmds}
	_ = bare.SetTECPower(0)
	if rm.reads != 5 || rm.writes != 11 {
		t.Fatalf("uncached: reads=%d writes=%d, want 5/11", rm.reads, rm.writes)
	}
}
