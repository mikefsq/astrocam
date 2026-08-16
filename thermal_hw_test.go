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

// TestReadTempDecode pins the GetSensorTemp decode: signed 12-bit fixed point,
// raw = (hi<<4)|(lo>>4), temp = raw/16 — including sign extension below 0 °C.
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

// TestReadHumidityDecode pins the Sensirion transfer function RH = 125·raw/2^16 − 6,
// clamped to 0..100.
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
