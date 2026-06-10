package astrocam

import "testing"

// TestColorMonoShareSensor verifies the core architecture: one sensor profile
// serves both a color (MC) and a mono (MM) model. The die is mono; the CFA
// pattern is surfaced by Info only when Model.Color is set. Color is a Model
// flag, not a different register profile.
func TestColorMonoShareSensor(t *testing.T) {
	die := Sensor{Name: "DIE", Info: CameraInfo{MaxWidth: 1936, MaxHeight: 1216, BitDepth: 12, Bayer: "RGGB"}}
	Register(ZWO.VID, 0x0C01, Model{Name: "Mono", Sensor: &die, Color: false})
	Register(ZWO.VID, 0x0C02, Model{Name: "Color", Sensor: &die, Color: true})
	f := &fakeTransport{}

	mono, err := Open(f, ZWO.VID, 0x0C01)
	if err != nil {
		t.Fatal(err)
	}
	if got := mono.Info().Bayer; got != "" {
		t.Errorf("mono model Bayer = %q, want \"\"", got)
	}
	if mono.Color() {
		t.Error("mono model should report Color() == false")
	}

	color, err := Open(f, ZWO.VID, 0x0C02)
	if err != nil {
		t.Fatal(err)
	}
	if got := color.Info().Bayer; got != "RGGB" {
		t.Errorf("color model Bayer = %q, want RGGB", got)
	}
	if !color.Color() {
		t.Error("color model should report Color() == true")
	}

	// Both bind the same sensor profile — the difference is purely the Model flag.
	if mono.Sensor() != color.Sensor() {
		t.Error("mono and color models should share one sensor profile")
	}
}
