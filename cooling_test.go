package astrocam

import (
	"context"
	"fmt"
	"math"
	"sync"
	"testing"
	"time"
)

// fakePlant is a first-order thermal model implementing Thermal: TEC power pulls the equilibrium
// temperature below ambient (up to span °C at 100 %), and the plant relaxes toward that
// equilibrium with time constant tau.
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

// TestCoolerConverges: the loop reaches and holds a sub-ambient target at a physical holding
// power.
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

// TestCoolerNoHeating: a target above ambient clamps power to 0 (the loop never drives the TEC
// to warm).
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

// rtPlant is a fakePlant that self-advances on each ReadTemp by the wall-clock time since the
// previous read, so it evolves under a running Cooler.Run goroutine with no manual advance().
// Its own mutex stands in for the transport's ioMu: the Cooler reads temperature outside its
// state lock, so the loop's reads and a caller's Temperature() overlap.
type rtPlant struct {
	fakePlant
	mu   sync.Mutex
	last time.Time
}

func (p *rtPlant) ReadTemp() (float64, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	now := time.Now()
	if !p.last.IsZero() {
		p.advance(now.Sub(p.last))
	}
	p.last = now
	return p.fakePlant.temp, nil
}

func (p *rtPlant) SetTECPower(pct float64) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.fakePlant.SetTECPower(pct)
}

func (p *rtPlant) SetFan(on bool) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.fakePlant.SetFan(on)
}

// power and fanOn read the plant's actuator state under the lock, for assertions made while the
// regulation goroutine may still be running.
func (p *rtPlant) power() float64 {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.fakePlant.power
}

func (p *rtPlant) fanOn() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.fakePlant.fan
}

// TestCoolerRun: Run ticks Step against the plant, drives the TEC, cools toward the target, and
// returns promptly when its context is canceled.
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

	time.Sleep(400 * time.Millisecond) // ~80 ticks, ~4 tau
	cancel()

	select {
	case <-done: // clean shutdown, no leak
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

// TestCoolerRampRate: the setpoint slews at RampRate °C/min independent of the tick. Starting at
// 0 °C with a 6 °C/min ramp, after one simulated minute the effective target has moved ~6 °C.
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

// TestCoolerRateGuard: a fast transient (rate above RateGuard) is skipped; power is left
// unchanged that tick.
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

// flakyPlant wraps fakePlant, failing ReadTemp for the first failN calls (transient) or forever
// (failN < 0). It records the last TEC power applied so the fail-safe zero is observable.
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

// TestCoolerRunSurvivesTransientErrors: a few failed regulation ticks do not stop the loop; it
// keeps ticking until its context deadline.
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

// TestCoolerRunZeroesTECOnPersistentFailure: on persistent Step failure Run zeroes the TEC
// drive and returns an error.
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

// writeFailPlant fails SetTECPower on demand, leaving ReadTemp working: the failure mode where
// the loop learns the temperature but its correction never reaches the TEC.
type writeFailPlant struct {
	fakePlant
	failWrite bool
}

func (p *writeFailPlant) SetTECPower(pct float64) error {
	if p.failWrite {
		return errTransient
	}
	return p.fakePlant.SetTECPower(pct)
}

// TestCoolerFailedDriveDoesNotAdvanceHistory: a tick whose drive write failed applied no
// correction, so the velocity form's error history must not move on as though it had — the next
// tick's Kp and Kd terms would be differences against an increment the TEC never received. The
// temperature history is the other half: lastT belongs to the read, so it has to follow every
// successful read, or the rate guard measures two ticks of drift over one tick of dt and reads a
// steady ramp as a transient to skip.
func TestCoolerFailedDriveDoesNotAdvanceHistory(t *testing.T) {
	p := &writeFailPlant{fakePlant: fakePlant{temp: 20, amb: 20, span: 50, tau: 1000}}
	cfg := DefaultCoolerConfig()
	cfg.RateGuard = 1.0 // °C/s
	c := NewCooler(p, cfg)
	c.SetTarget(0)

	if _, err := c.Step(time.Second); err != nil {
		t.Fatalf("first step: %v", err)
	}
	c.mu.Lock()
	e1, prev1, power1 := c.prevErr, c.prevErr2, c.power
	c.mu.Unlock()

	// A tick that reads a new temperature but cannot drive the TEC.
	p.failWrite = true
	p.temp = 19.5
	if _, err := c.Step(time.Second); err == nil {
		t.Fatal("step with a failing drive write reported success")
	}
	c.mu.Lock()
	e2, prev2, power2, lastT := c.prevErr, c.prevErr2, c.power, c.lastT
	c.mu.Unlock()
	if e2 != e1 || prev2 != prev1 {
		t.Errorf("error history moved on a tick that applied nothing: (%.3f,%.3f) -> (%.3f,%.3f)", prev1, e1, prev2, e2)
	}
	if power2 != power1 {
		t.Errorf("power recorded as %.2f%% though the write failed (was %.2f%%)", power2, power1)
	}
	if lastT != 19.5 {
		t.Errorf("lastT = %.2f after reading 19.5: the temperature history skipped a reading", lastT)
	}

	// The next good tick regulates from that reading rather than skipping on a doubled rate.
	p.failWrite = false
	p.temp = 19.0
	if _, err := c.Step(time.Second); err != nil {
		t.Fatalf("recovered step: %v", err)
	}
	c.mu.Lock()
	applied := c.power
	c.mu.Unlock()
	if applied == power1 {
		t.Errorf("power unchanged at %.2f%% after recovery: the tick was skipped", applied)
	}
}
