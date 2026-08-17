package astrocam

import (
	"testing"
	"time"
)

// TestCameraCoolingLifecycle: a non-cooled model refuses cooling; a cooled one starts the
// regulation goroutine, cools a simulated plant, retargets in place, and is joined on Close
// (a leaked goroutine would hang DisableCooling's coolWg.Wait).
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
	cool.DisableCooling() // idempotent: no panic, no hang
}

// TestCameraCoolerRetires: when the regulation loop gives up (thermal I/O failing for
// runMaxConsecFails ticks) the Camera drops it: CoolerOn turns false, CoolerFault reports the
// error, and a fresh EnableCooling starts a new loop instead of retargeting the dead one.
func TestCameraCoolerRetires(t *testing.T) {
	Register(ZWO.VID, 0x00C3, Model{Name: "CoolFlaky", Sensor: &armSensor, Cooled: true})
	cam, err := Open(NewStubTransport(), ZWO.VID, 0x00C3)
	if err != nil {
		t.Fatal(err)
	}
	dead := &flakyPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 5}, failN: -1}
	cfg := DefaultCoolerConfig()
	cfg.Tick = time.Millisecond
	if err := cam.EnableCooling(dead, -10, cfg); err != nil {
		t.Fatal(err)
	}
	deadline := time.Now().Add(2 * time.Second)
	for cam.CoolerOn() && time.Now().Before(deadline) {
		time.Sleep(5 * time.Millisecond)
	}
	if cam.CoolerOn() {
		t.Fatal("CoolerOn still true after the loop should have retired")
	}
	if cam.CoolerFault() == nil {
		t.Error("CoolerFault nil after the loop retired, want the give-up error")
	}
	// A working plant restarts a fresh loop.
	good := &rtPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 0.1}}
	if err := cam.EnableCooling(good, -10, cfg); err != nil {
		t.Fatal(err)
	}
	if !cam.CoolerOn() || cam.CoolerFault() != nil {
		t.Errorf("after re-enable: CoolerOn %v fault %v, want true / nil", cam.CoolerOn(), cam.CoolerFault())
	}
	time.Sleep(100 * time.Millisecond)
	if pw := cam.CoolerPower(); pw <= 0 {
		t.Errorf("re-enabled loop not driving: power %v", pw)
	}
	cam.DisableCooling()
}

// slowPlant blocks each ReadTemp for hold, standing in for a thermal read queued behind a gated
// USB2 readout.
type slowPlant struct {
	fakePlant
	hold time.Duration
}

func (p *slowPlant) ReadTemp() (float64, error) { time.Sleep(p.hold); return p.fakePlant.ReadTemp() }

// TestCoolerPollsDoNotWaitOnIO: Power/Target return promptly while a Step is blocked in the
// thermal read (the read runs outside Cooler.mu).
func TestCoolerPollsDoNotWaitOnIO(t *testing.T) {
	p := &slowPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 5}, hold: 300 * time.Millisecond}
	c := NewCooler(p, DefaultCoolerConfig())
	c.SetTarget(-10) // one slow read, seeds the ramp
	done := make(chan struct{})
	go func() { _, _ = c.Step(200 * time.Millisecond); close(done) }()
	time.Sleep(50 * time.Millisecond) // Step is inside ReadTemp now
	t0 := time.Now()
	_ = c.Power()
	_, _ = c.Target()
	if el := time.Since(t0); el > 50*time.Millisecond {
		t.Errorf("Power/Target waited %v behind a blocked thermal read, want no wait", el)
	}
	<-done
}

// TestDisableCoolingZeroesDrive: turning regulation off must leave the TEC off. The FPGA holds
// the drive level, so a loop that simply stops leaves the cooler pulling at its last power with
// nothing regulating it, while CoolerPower reports 0 — the driver and the hardware disagree, and
// the frost/condensation risk is real. Close must also leave a zeroed cooler behind whatever the
// caller did before it, which is the promise its own doc makes.
func TestDisableCoolingZeroesDrive(t *testing.T) {
	Register(ZWO.VID, 0x00C7, Model{Name: "CoolOff", Sensor: &armSensor, Cooled: true})
	newCam := func(t *testing.T) (*Camera, *rtPlant) {
		t.Helper()
		c, err := Open(NewStubTransport(), ZWO.VID, 0x00C7)
		if err != nil {
			t.Fatal(err)
		}
		p := &rtPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 0.1}}
		cfg := DefaultCoolerConfig()
		cfg.Tick = 5 * time.Millisecond
		if err := c.EnableCooling(p, -10, cfg); err != nil {
			t.Fatal(err)
		}
		time.Sleep(120 * time.Millisecond) // let the loop drive the TEC up
		if p.power() <= 0 {
			t.Fatal("test premise: the loop applied no power")
		}
		return c, p
	}

	c, p := newCam(t)
	c.DisableCooling()
	if pw := p.power(); pw != 0 {
		t.Errorf("TEC still at %.1f%% after DisableCooling, want 0", pw)
	}
	if p.fanOn() {
		t.Error("fan still running after DisableCooling")
	}
	if pw := c.CoolerPower(); pw != 0 {
		t.Errorf("CoolerPower reports %.1f%% with the drive off", pw)
	}
	_ = c.Close()

	// Close after a DisableCooling that left the drive up (a caller driving the TEC by hand, or
	// a loop whose own fail-safe zero failed) must still leave it off.
	c2, p2 := newCam(t)
	c2.DisableCooling()
	_ = c2.HardwareThermal() // the seam stays attached for temperature reads
	p2.SetTECPower(70)       // something drove the TEC after regulation stopped
	if err := c2.Close(); err != nil {
		t.Fatal(err)
	}
	if pw := p2.power(); pw != 0 {
		t.Errorf("TEC left at %.1f%% after Close, want 0", pw)
	}
}

// slowThermal is a plant whose temperature read takes a fixed time, so a test can hold a
// regulation loop inside one tick and interleave other cooling calls against it deterministically.
type slowThermal struct {
	rtPlant
	delay time.Duration
}

func (p *slowThermal) ReadTemp() (float64, error) {
	time.Sleep(p.delay)
	return p.rtPlant.ReadTemp()
}

// TestDisableCoolingWaitsOnlyForItsOwnLoop: DisableCooling cancels its loop and waits for it
// outside the lock, so an EnableCooling can slip into that window, see no loop and start a fresh
// one. The waiter must not end up waiting on that new loop, which exits only on its own cancel —
// with one shared counter it does, and the caller blocks indefinitely. Two Alpaca handlers
// (CoolerOn false and true) interleave exactly this way.
func TestDisableCoolingWaitsOnlyForItsOwnLoop(t *testing.T) {
	Register(ZWO.VID, 0x00C8, Model{Name: "CoolRace", Sensor: &armSensor, Cooled: true})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00C8)
	if err != nil {
		t.Fatal(err)
	}
	// Each tick blocks in ReadTemp, so a cancelled loop takes up to `delay` to notice and exit.
	plant := &slowThermal{rtPlant: rtPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 0.1}}, delay: 400 * time.Millisecond}
	cfg := DefaultCoolerConfig()
	cfg.Tick = time.Millisecond
	if err := c.EnableCooling(plant, -10, cfg); err != nil {
		t.Fatal(err)
	}

	// EnableCooling returned after its own temperature read, so the loop is part-way into the
	// next one. Cancel it half a read in, leaving it blocked for the rest of that read.
	time.Sleep(200 * time.Millisecond)
	stopped := make(chan struct{})
	go func() {
		defer close(stopped)
		c.DisableCooling() // cancels the first loop, then waits for it
	}()
	// Land inside the window between that cancel and the old loop noticing it.
	time.Sleep(100 * time.Millisecond)
	if err := c.EnableCooling(plant, -10, cfg); err != nil {
		t.Fatal(err)
	}

	select {
	case <-stopped:
	case <-time.After(5 * time.Second):
		t.Fatal("DisableCooling blocked: it is waiting on the loop EnableCooling started, not its own")
	}
	c.DisableCooling()
	if c.CoolerOn() {
		t.Error("cooling still on after the final DisableCooling")
	}
	if err := c.Close(); err != nil {
		t.Fatal(err)
	}
}

// TestEnableCoolingSwapsThermalBackend: a running loop drives the backend it was built with, so
// naming a different one stops that loop and starts again on the new backend. Storing it without
// restarting would leave Temperature reading one place while regulation drove another. Passing
// nil (the hardware seam, what every shipping caller does) or the same backend stays an ordinary
// retarget of the running loop.
func TestEnableCoolingSwapsThermalBackend(t *testing.T) {
	Register(ZWO.VID, 0x00C9, Model{Name: "CoolSeam", Sensor: &armSensor, Cooled: true})
	c, err := Open(NewStubTransport(), ZWO.VID, 0x00C9)
	if err != nil {
		t.Fatal(err)
	}
	a := &rtPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 0, tau: 1000}}
	b := &rtPlant{fakePlant: fakePlant{temp: -40, amb: -40, span: 0, tau: 1000}}
	cfg := DefaultCoolerConfig()
	cfg.Tick = 5 * time.Millisecond
	if err := c.EnableCooling(a, 10, cfg); err != nil {
		t.Fatal(err)
	}
	defer c.DisableCooling()
	time.Sleep(60 * time.Millisecond)
	if a.power() <= 0 {
		t.Fatal("test premise: the loop drove no power on the first backend")
	}

	// Move to the other backend: it is what Temperature reads and what the loop drives.
	if err := c.EnableCooling(b, -50, cfg); err != nil {
		t.Fatalf("EnableCooling on another backend: %v", err)
	}
	if !c.CoolerOn() {
		t.Error("cooling off after moving to another backend")
	}
	if temp, err := c.Temperature(); err != nil || temp > -30 {
		t.Errorf("Temperature = %.1f (err %v), want the new backend (~-40)", temp, err)
	}
	time.Sleep(60 * time.Millisecond)
	if b.power() <= 0 {
		t.Error("the loop is not driving the new backend")
	}
	if a.power() != 0 {
		t.Errorf("the old backend was left driving at %.1f%%", a.power())
	}

	// nil and the same backend are plain retargets, and keep the loop as it stands.
	if err := c.EnableCooling(nil, -45, cfg); err != nil {
		t.Errorf("retarget with nil: %v", err)
	}
	if err := c.EnableCooling(b, -44, cfg); err != nil {
		t.Errorf("retarget with the same backend: %v", err)
	}
	if f, _, on := c.TargetTemp(); !on || f != -44 {
		t.Errorf("target = %.1f on=%v, want -44 / true", f, on)
	}
}
