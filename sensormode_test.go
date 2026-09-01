package astrocam

import (
	"errors"
	"testing"
)

// modeSensor builds a profile with two readout programmes, recording what SetSensorMode was asked
// for. fail makes the profile refuse, which is how a profile reports an undecoded (mode, sample
// size) cell.
func modeSensor(name string, fail bool) (*Sensor, *[]int) {
	var got []int
	s := &Sensor{
		Name: name,
		Info: CameraInfo{MaxWidth: 64, MaxHeight: 32, BitDepth: 12},
		SensorModes: func(uint16) []SensorModeInfo {
			return []SensorModeInfo{{Name: "Normal"}, {Name: "HDR"}}
		},
		SetSensorMode: func(_ Regmap, mode int) error {
			got = append(got, mode)
			if fail {
				return errors.New("cell not decoded")
			}
			return nil
		},
	}
	return s, &got
}

// TestCameraSensorMode: the index round-trips onto the ReadoutMode, where both halves of a mode
// change read it — the profile's sensor block and SetROI's geometry.
func TestCameraSensorMode(t *testing.T) {
	s, got := modeSensor("SMODE", false)
	Register(POA.VID, 0x0D10, Model{Name: "p", Sensor: s})
	c, err := Open(NewStubTransport(), POA.VID, 0x0D10)
	if err != nil {
		t.Fatal(err)
	}
	if n := len(c.SensorModes()); n != 2 {
		t.Fatalf("SensorModes = %d entries, want 2", n)
	}
	if c.SensorMode() != 0 {
		t.Errorf("initial mode = %d, want 0 (normal)", c.SensorMode())
	}
	if err := c.SetSensorMode(1); err != nil {
		t.Fatalf("SetSensorMode(1): %v", err)
	}
	if c.SensorMode() != 1 {
		t.Errorf("mode after SetSensorMode(1) = %d, want 1", c.SensorMode())
	}
	if ModeOf(c.Rm()).SensorMode != 1 {
		t.Error("the mode did not reach the ReadoutMode, so SetROI would program the wrong geometry")
	}
	if len(*got) != 1 || (*got)[0] != 1 {
		t.Errorf("profile saw %v, want one call for mode 1", *got)
	}
	// Re-selecting the current mode is a no-op rather than a re-program.
	if err := c.SetSensorMode(1); err != nil {
		t.Fatal(err)
	}
	if len(*got) != 1 {
		t.Errorf("profile saw %v; re-selecting the current mode should not re-program", *got)
	}
	if err := c.SetSensorMode(2); err == nil {
		t.Error("SetSensorMode(2) succeeded with two modes declared")
	}
	if err := c.SetSensorMode(-1); err == nil {
		t.Error("SetSensorMode(-1) succeeded")
	}
	if c.SensorMode() != 1 {
		t.Errorf("a rejected index moved the mode to %d", c.SensorMode())
	}
}

// TestCameraSensorModeRefusalRestores: a profile that refuses a mode must leave the camera in the
// mode it was in. Otherwise the ReadoutMode would claim a mode whose registers were never
// written, and the next SetROI would program geometry the sensor is not configured for.
func TestCameraSensorModeRefusalRestores(t *testing.T) {
	s, _ := modeSensor("SMODEFAIL", true)
	Register(POA.VID, 0x0D11, Model{Name: "p", Sensor: s})
	c, err := Open(NewStubTransport(), POA.VID, 0x0D11)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetSensorMode(1); err == nil {
		t.Fatal("SetSensorMode(1) succeeded on a profile that refuses")
	}
	if c.SensorMode() != 0 {
		t.Errorf("mode = %d after a refusal, want 0", c.SensorMode())
	}
	if ModeOf(c.Rm()).SensorMode != 0 {
		t.Error("the ReadoutMode kept a mode the sensor was never programmed into")
	}
}

// TestCameraSensorModeAbsent: a profile with one programme reports no mode selection, which is
// what the SDK reports as a mode count of 0, and refuses rather than pretending.
func TestCameraSensorModeAbsent(t *testing.T) {
	s := &Sensor{Name: "NOMODE", Info: CameraInfo{MaxWidth: 64, MaxHeight: 32, BitDepth: 12}}
	Register(POA.VID, 0x0D12, Model{Name: "p", Sensor: s})
	c, err := Open(NewStubTransport(), POA.VID, 0x0D12)
	if err != nil {
		t.Fatal(err)
	}
	if got := c.SensorModes(); got != nil {
		t.Errorf("SensorModes = %v, want nil", got)
	}
	if err := c.SetSensorMode(1); err == nil {
		t.Error("SetSensorMode succeeded on a profile with no modes")
	}
}
