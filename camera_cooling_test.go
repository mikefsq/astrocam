package asicam

import (
	"testing"
	"time"
)

// TestCameraCoolingLifecycle exercises the per-camera TEC goroutine lifecycle: a
// non-cooled model refuses cooling; a cooled one starts the regulation goroutine, cools a
// simulated plant, retargets in place (no second goroutine), and is joined cleanly on
// Close (the coolWg.Wait in DisableCooling would hang the test on a leak). This is the
// cooling goroutine the Alpaca driver relies on so it never touches a thread.
func TestCameraCoolingLifecycle(t *testing.T) {
	// A model with no cooler rejects EnableCooling.
	Register(ZWO.VID, 0x00C1, Model{Name: "Warm", Sensor: &armSensor}) // Cooled:false
	warm, err := Open(NewStubTransport(), ZWO.VID, 0x00C1)
	if err != nil {
		t.Fatal(err)
	}
	if err := warm.EnableCooling(&rtPlant{}, -10, DefaultCoolerConfig()); err == nil {
		t.Error("EnableCooling should fail on a model with no cooler")
	}

	// A cooled model regulates a plant toward target and joins cleanly.
	Register(ZWO.VID, 0x00C2, Model{Name: "Cool", Sensor: &armSensor, Cooled: true})
	cool, err := Open(NewStubTransport(), ZWO.VID, 0x00C2)
	if err != nil {
		t.Fatal(err)
	}
	plant := &rtPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 0.1}}
	cfg := DefaultCoolerConfig()
	cfg.Tick = 5 * time.Millisecond
	cfg.SlewPerStep = 0 // let power move freely so cooling shows in a short test
	if err := cool.EnableCooling(plant, -10, cfg); err != nil {
		t.Fatal(err)
	}
	if !cool.CoolerOn() {
		t.Fatal("CoolerOn should be true after EnableCooling")
	}

	time.Sleep(300 * time.Millisecond) // ~60 ticks, several tau
	if pw := cool.CoolerPower(); pw <= 0 {
		t.Errorf("cooler applied no power (%.1f%%)", pw)
	}
	if temp, err := cool.Temperature(); err != nil || temp > 15 {
		t.Errorf("plant not cooling: temp %.1f, err %v (want < 15 from 20)", temp, err)
	}

	// Retarget the running loop in place.
	cool.SetTargetTemp(-20)
	if f, _, on := cool.TargetTemp(); !on || f != -20 {
		t.Errorf("retarget: final=%.1f on=%v, want -20 / true", f, on)
	}

	// Close joins the goroutine; cooling is then off, and a redundant DisableCooling is safe.
	if err := cool.Close(); err != nil {
		t.Fatal(err)
	}
	if cool.CoolerOn() {
		t.Error("CoolerOn should be false after Close")
	}
	cool.DisableCooling() // idempotent — no panic, no hang
}
