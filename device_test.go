package asicam

import (
	"strings"
	"testing"
)

// TestFilterCameras checks the Enumerate filter: raw USB devices on the ZWO VID are
// kept only when their PID resolves to a registered camera Model (so non-camera ZWO
// devices — e.g. EFW filter wheels — are dropped), and a missing USB product name
// falls back to the registry name.
func TestFilterCameras(t *testing.T) {
	Register(ZWO.VID, 0xE001, Model{Name: "TestCamA", Sensor: &testSensor})
	Register(ZWO.VID, 0xE002, Model{Name: "TestCamB", Sensor: &testSensor})

	raw := []DeviceInfo{
		{VID: ZWO.VID, PID: 0xE001, Name: "ASI-A", Location: 0x100},
		{VID: ZWO.VID, PID: 0xEFEF, Name: "EFW Mini", Location: 0x200}, // not a registered camera → drop
		{VID: ZWO.VID, PID: 0xE002, Name: "", Location: 0x300},         // no USB name → fall back to registry
	}

	got := filterCameras(raw)
	if len(got) != 2 {
		t.Fatalf("got %d cameras, want 2 (EFW dropped): %+v", len(got), got)
	}
	if got[0].PID != 0xE001 || got[0].Name != "ASI-A" || got[0].Location != 0x100 {
		t.Errorf("first = %+v, want PID E001 / ASI-A / loc 0x100", got[0])
	}
	if got[1].PID != 0xE002 || got[1].Name != "TestCamB" { // name filled from registry
		t.Errorf("second = %+v, want PID E002 with registry name TestCamB", got[1])
	}
	for _, d := range got {
		if d.PID == 0xEFEF {
			t.Error("EFW (unregistered PID) leaked through the camera filter")
		}
	}
}

// TestDeviceInfoString sanity-checks that the human listing carries the identifying
// fields (VID:PID, name, location) without pinning the exact spacing.
func TestDeviceInfoString(t *testing.T) {
	d := DeviceInfo{VID: ZWO.VID, PID: 0x620A, Name: "ASI6200MC Pro", Location: 0x02200000}
	got := d.String()
	for _, want := range []string{"03c3:620a", `"ASI6200MC Pro"`, "loc=0x02200000"} {
		if !strings.Contains(got, want) {
			t.Errorf("String() = %q, missing %q", got, want)
		}
	}
}
