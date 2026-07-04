package astrocam

import (
	"context"
	"fmt"
	"math"
	"testing"
	"time"
)

// fakePlant is a first-order thermal model: TEC power pulls the equilibrium
// temperature below ambient (up to span °C at 100 %), and the plant relaxes toward
// that equilibrium with time constant tau. It implements Thermal so the cooling
// loop can be driven with no hardware.
type fakePlant struct {
	temp, amb, span, tau float64
	power                float64
	heater               int
	fan                  bool
}

func (p *fakePlant) ReadTemp() (float64, error)    { return p.temp, nil }
func (p *fakePlant) SetTECPower(pct float64) error { p.power = pct; return nil }
func (p *fakePlant) SetFan(on bool) error          { p.fan = on; return nil }
func (p *fakePlant) SetHeater(pct int) error       { p.heater = pct; return nil }
func (p *fakePlant) ReadHumidity() (int, error)    { return 50, nil }

func (p *fakePlant) advance(dt time.Duration) {
	eq := p.amb - p.span*(p.power/100) // equilibrium temp for the current drive
	a := 1 - math.Exp(-dt.Seconds()/p.tau)
	p.temp += (eq - p.temp) * a
}

// TestCoolerConverges drives the loop against the plant and checks it reaches and
// holds a sub-ambient target (the core of the PID + integral steady-state).
func TestCoolerConverges(t *testing.T) {
	p := &fakePlant{temp: 20, amb: 20, span: 50, tau: 5}
	c := NewCooler(p, DefaultCoolerConfig())
	c.SetTarget(-10)

	const dt = 200 * time.Millisecond
	var last []float64
	for i := 0; i < 4000; i++ { // 800 s simulated
		if _, err := c.Step(dt); err != nil {
			t.Fatal(err)
		}
		p.advance(dt)
		if i >= 3950 {
			last = append(last, p.temp)
		}
	}

	// Settled near the target.
	for _, v := range last {
		if math.Abs(v-(-10)) > 0.5 {
			t.Fatalf("temp %.2f did not settle within 0.5 °C of -10", v)
		}
	}
	// Stable (small ripple) over the tail.
	min, max := last[0], last[0]
	for _, v := range last {
		min, max = math.Min(min, v), math.Max(max, v)
	}
	if max-min > 0.2 {
		t.Errorf("steady-state ripple %.3f °C too large", max-min)
	}
	// Holding power is physical (≈60 % to hold -10 with span 50 below amb 20).
	if pw := c.Power(); pw < 50 || pw > 70 {
		t.Errorf("steady power %.1f%% outside expected ~60%%", pw)
	}
}

// TestCoolerNoHeating: a target above ambient needs no cooling — power must clamp
// to 0 (the loop never drives the TEC to warm).
func TestCoolerNoHeating(t *testing.T) {
	p := &fakePlant{temp: 20, amb: 20, span: 50, tau: 5}
	c := NewCooler(p, DefaultCoolerConfig())
	c.SetTarget(25) // warmer than ambient
	for i := 0; i < 200; i++ {
		c.Step(200 * time.Millisecond)
		p.advance(200 * time.Millisecond)
	}
	if c.Power() != 0 {
		t.Errorf("power = %.1f%%, want 0 when target is above ambient", c.Power())
	}
	if math.Abs(p.temp-20) > 0.5 {
		t.Errorf("temp drifted to %.2f; should stay near ambient", p.temp)
	}
}

// TestCoolerSlewLimit: TEC power may not jump more than SlewPerStep in one tick.
func TestCoolerSlewLimit(t *testing.T) {
	p := &fakePlant{temp: 30, amb: 20, span: 50, tau: 5}
	cfg := DefaultCoolerConfig()
	cfg.SlewPerStep = 5
	c := NewCooler(p, cfg)
	c.SetTarget(-20) // huge error → PID wants full power immediately
	if _, err := c.Step(200 * time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if c.Power() > cfg.SlewPerStep+1e-9 {
		t.Errorf("power jumped to %.1f%% in one step, slew cap is %.1f%%", c.Power(), cfg.SlewPerStep)
	}
}

// rtPlant is a fakePlant that self-advances on each ReadTemp by the wall-clock time
// since the previous read, so it evolves under a running Cooler.Run goroutine with no
// manual advance(). A small tau makes it converge in well under a second.
type rtPlant struct {
	fakePlant
	last time.Time
}

func (p *rtPlant) ReadTemp() (float64, error) {
	now := time.Now()
	if !p.last.IsZero() {
		p.advance(now.Sub(p.last))
	}
	p.last = now
	return p.fakePlant.temp, nil
}

// TestCoolerRun exercises the regulation GOROUTINE (the cooling-thread equivalent): Run
// must tick Step against the plant, drive the TEC, cool toward the target, and return
// promptly when its context is canceled (no leaked goroutine). Convergence accuracy is
// covered by the Step tests; this is about the thread doing its job and shutting down.
func TestCoolerRun(t *testing.T) {
	p := &rtPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 0.1}}
	cfg := DefaultCoolerConfig()
	cfg.Tick = 5 * time.Millisecond
	cfg.SlewPerStep = 0 // let power move freely so cooling is visible in a short test
	c := NewCooler(p, cfg)
	c.SetTarget(-10)

	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- c.Run(ctx) }()

	time.Sleep(400 * time.Millisecond) // ~80 ticks, ~4 tau — plenty to cool
	cancel()

	select {
	case <-done: // Run returned after cancel — clean shutdown, no leak
	case <-time.After(2 * time.Second):
		t.Fatal("Run did not return after ctx cancel")
	}

	if pw := c.Power(); pw <= 0 {
		t.Errorf("Run applied no TEC power (%.1f%%)", pw)
	}
	if temp, _ := p.ReadTemp(); temp > 15 { // from 20 °C, must have cooled toward -10
		t.Errorf("Run did not cool the plant: temp %.1f still near ambient", temp)
	}
}

// TestCoolerRampRate verifies the setpoint slews at RampRate °C/min independent of the
// tick — the Alpaca-facing cooldown-rate knob. Starting at 0 °C with a 6 °C/min ramp,
// after one simulated minute the effective target must have moved ~6 °C toward the goal.
func TestCoolerRampRate(t *testing.T) {
	p := &fakePlant{temp: 0, amb: 20, span: 50, tau: 5}
	cfg := DefaultCoolerConfig()
	cfg.RampRate = 6 // °C/min = 0.1 °C/s
	c := NewCooler(p, cfg)
	c.SetTarget(-30) // far below start; the ramp, not the target, paces the setpoint

	for i := 0; i < 60; i++ { // 60 × 1 s = 1 simulated minute
		if _, err := c.Step(time.Second); err != nil {
			t.Fatal(err)
		}
		p.advance(time.Second)
	}
	if _, eff := c.Target(); math.Abs(eff-(-6)) > 0.5 {
		t.Errorf("effective target = %.2f after 1 min at 6 °C/min, want ≈ -6", eff)
	}

	// Doubling the rate at runtime doubles the slew (another minute → another ~12 °C).
	c.SetRampRate(12)
	for i := 0; i < 60; i++ {
		c.Step(time.Second)
		p.advance(time.Second)
	}
	if _, eff := c.Target(); math.Abs(eff-(-18)) > 0.6 {
		t.Errorf("effective target = %.2f after a 2nd min at 12 °C/min, want ≈ -18", eff)
	}
}

// TestCoolerRateGuard: a fast transient (rate above RateGuard) is skipped — power
// is left unchanged that tick.
func TestCoolerRateGuard(t *testing.T) {
	p := &fakePlant{temp: 20, amb: 20, span: 50, tau: 5}
	cfg := DefaultCoolerConfig()
	cfg.RateGuard = 1.0 // °C/s
	cfg.SlewPerStep = 0
	c := NewCooler(p, cfg)
	c.SetTarget(-10)
	c.Step(time.Second) // prime (power moves)
	pw := c.Power()

	p.temp += 10 // 10 °C in 1 s = 10 °C/s, far above the 1 °C/s guard
	if _, err := c.Step(time.Second); err != nil {
		t.Fatal(err)
	}
	if c.Power() != pw {
		t.Errorf("rate guard should have skipped regulation: power %.1f -> %.1f", pw, c.Power())
	}
}

// flakyPlant wraps fakePlant, failing ReadTemp for the first failN calls (transient) or
// forever (failN < 0). It records the last TEC power applied so the fail-safe zero is
// observable.
type flakyPlant struct {
	fakePlant
	failN int // >0: fail this many ReadTemps then recover; <0: fail forever
	reads int
}

func (p *flakyPlant) ReadTemp() (float64, error) {
	p.reads++
	if p.failN < 0 || p.reads <= p.failN {
		return 0, errTransient
	}
	return p.fakePlant.ReadTemp()
}

var errTransient = fmt.Errorf("transient EP0 error")

// TestCoolerRunSurvivesTransientErrors: a few failed regulation ticks (an EP0 hiccup during a
// capture-recovery reset) must not kill the loop — the historical bug returned on the FIRST
// Step error, leaving the TEC energized with CoolerOn() still true.
func TestCoolerRunSurvivesTransientErrors(t *testing.T) {
	p := &flakyPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 5}, failN: 3}
	cfg := DefaultCoolerConfig()
	cfg.Tick = time.Millisecond
	c := NewCooler(p, cfg)
	c.SetTarget(-10)

	ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
	defer cancel()
	err := c.Run(ctx)
	if err != context.DeadlineExceeded {
		t.Fatalf("Run returned %v, want deadline (loop must survive %d transient errors)", err, p.failN)
	}
	if p.reads < 10 {
		t.Errorf("loop stopped ticking after the transient errors (only %d reads)", p.reads)
	}
}

// TestCoolerRunZeroesTECOnPersistentFailure: when the device is genuinely gone, Run must not
// leave the TEC driving at its last power — it zeroes the drive (best-effort) and returns.
func TestCoolerRunZeroesTECOnPersistentFailure(t *testing.T) {
	p := &flakyPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 5, power: 80}, failN: -1}
	cfg := DefaultCoolerConfig()
	cfg.Tick = time.Millisecond
	c := NewCooler(p, cfg)
	c.SetTarget(-10)
	c.SeedPower(80) // pretend we were mid-cooldown at 80%

	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	err := c.Run(ctx)
	if err == nil || err == context.DeadlineExceeded {
		t.Fatalf("Run returned %v, want persistent-failure error", err)
	}
	if p.power != 0 {
		t.Errorf("TEC left at %.0f%% after the loop died, want 0 (fail-safe zero)", p.power)
	}
}
