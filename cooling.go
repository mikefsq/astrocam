package astrocam

// Host-side TEC cooling control: regulation step, PID init, setpoint ramp,
// power-to-DAC conversion, and the cooling worker goroutine. The temperature loop
// runs on the host over camera primitives: read sensor temperature, set TEC drive,
// fan and anti-dew heater on/off, read humidity.
//
// The controller drives a Thermal seam (decoupled from USB), so the loop is
// unit-testable against a simulated thermal plant (see cooling_test.go). A real
// camera supplies a Thermal backed by control transfers.
//
// The controller is the velocity (incremental) form of the PID: Δpower per tick,
// with the TEC drive itself as the accumulator (see Step). A deep setpoint arrives
// fast (the drive saturates during the descent) then settles at the correct holding
// power for any depth without integral windup. Per-model gains (Kp/Ki/Kd) are
// exposed as tunables with velocity-form defaults.

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"
)

// Thermal is the per-camera hardware seam the cooling loop drives. A real backend
// maps these to control transfers; tests map them to a simulated plant.
type Thermal interface {
	// ReadTemp returns the sensor temperature in °C (GetSensorTemp).
	ReadTemp() (float64, error)
	// SetTECPower sets the thermoelectric cooler drive, 0..100 % (SetPowerPerc →
	// the power-to-DAC conversion → SendCMD 0xB2, or SetFPGACoolPower on FPGA-cooled models).
	SetTECPower(percent float64) error
	// SetFan turns the cooling fan on/off (SetFanOn).
	SetFan(on bool) error
	// SetHeater sets the anti-dew lens heater, 0..100 % (SetLensHeat).
	SetHeater(percent int) error
	// ReadHumidity returns relative humidity % (GetHumidity → SendCMD 0x85/0xF5).
	ReadHumidity() (int, error)
}

// prevErrSentinel is the "no previous error yet" marker (−200 °C). While set, the
// velocity form's difference terms (Kp on e−e₁, Kd on e−2e₁+e₂) are skipped so the
// first ticks don't kick on a bogus error delta.
const prevErrSentinel = -200.0

// CoolerConfig holds the cooling-loop tunables. Gains are per-model; set them from
// the camera profile when known.
type CoolerConfig struct {
	// Kp, Ki, Kd are the velocity-form PID gains (Δpower = Kp·(e−e₁) + Ki·e +
	// Kd·(e−2e₁+e₂)). Ki sets how fast the drive ramps toward the setpoint (arrival
	// speed); Kp and Kd damp the approach. Ki is a per-tick ramp rate, not an integral
	// weight.
	Kp, Ki, Kd float64

	// Tick is the regulation period (default 200 ms).
	Tick time.Duration

	// MaxPower clamps TEC drive (≤100 %).
	MaxPower float64

	// SlewPerStep caps how far TEC power may move in one tick — a hard safety limit on
	// top of the velocity form's own Ki-paced ramp. 0 disables (the form self-limits).
	SlewPerStep float64

	// RampRate is the cooldown/warmup setpoint slew in °C per minute (tick-independent):
	// the effective target walks toward the final target at this rate, seeded at the
	// current temperature. The Alpaca-facing "cooldown rate" knob. 0 = jump straight to
	// the target.
	RampRate float64

	// RateGuard skips regulation when |dTemp/dt| exceeds this (°C/s), to avoid fighting a
	// fast thermal swing. 0 disables.
	RateGuard float64

	// MaxError clamps the error |temp − setpoint| the PID acts on, in °C (symmetric):
	// the drive only creeps (Ki·MaxError per tick) so the approach in either direction is
	// gentle and overshoot-free. Applies on cooldown and warmup. 0 disables (full error →
	// fast arrival).
	MaxError float64
}

// DefaultCoolerConfig returns conservative defaults. Gains are a starting point;
// tune per model.
func DefaultCoolerConfig() CoolerConfig {
	return CoolerConfig{
		// Velocity-form PD (no integral): Kp=0.6, Ki=0.2, Kd=0, with e = temp−setpoint and
		// Δ = Kp·(e−e₁) + Ki·e + Kd·(e−2e₁+e₂). Ki paces the drive ramp (arrival speed),
		// Kp damps the approach.
		Kp:          0.6,
		Ki:          0.20,
		Kd:          0.0,
		Tick:        200 * time.Millisecond,
		MaxPower:    100,
		SlewPerStep: 0, // velocity form self-limits via Ki; set >0 only as a hard safety cap
		RampRate:    0, // °C/min setpoint ramp; 0 = go straight to target (driver sets a rate)
		RateGuard:   0, // °C/s; 0 = always regulate
	}
}

// Cooler is the host-side PID cooling loop over a Thermal. Safe for the Run
// goroutine to regulate while others call SetTarget/Stop/Power/Target — all state
// access is guarded by mu.
type Cooler struct {
	io  Thermal
	cfg CoolerConfig

	mu       sync.Mutex
	target   float64 // final target temperature, °C
	effTgt   float64 // ramped ("effective") target the PID actually chases
	power    float64 // current TEC power % (the velocity-form accumulator), 0..MaxPower
	prevErr  float64 // error one tick ago (e₁); prevErrSentinel until first step
	prevErr2 float64 // error two ticks ago (e₂); prevErrSentinel until second step
	lastT    float64 // last temperature (for the rate guard)
	primed   bool    // have we seen one temperature yet
	on       bool
}

// normalizeCoolerConfig fills zero-value gains/Tick/MaxPower with the defaults.
func normalizeCoolerConfig(cfg CoolerConfig) CoolerConfig {
	d := DefaultCoolerConfig()
	if cfg.Kp == 0 && cfg.Ki == 0 && cfg.Kd == 0 {
		cfg.Kp, cfg.Ki, cfg.Kd = d.Kp, d.Ki, d.Kd
	}
	if cfg.Tick == 0 {
		cfg.Tick = d.Tick
	}
	if cfg.MaxPower == 0 {
		cfg.MaxPower = d.MaxPower
	}
	return cfg
}

// NewCooler builds a cooler. A zero-value config field falls back to the default.
func NewCooler(io Thermal, cfg CoolerConfig) *Cooler {
	return &Cooler{io: io, cfg: normalizeCoolerConfig(cfg), prevErr: prevErrSentinel, prevErr2: prevErrSentinel}
}

// SetConfig applies new tunables to a running cooler (gains, limits, ramp, guard), with
// the same zero-value normalization as NewCooler. Run reads Tick once at start, so a
// Tick change takes effect only on a future Run. Safe to call while Run is regulating.
func (c *Cooler) SetConfig(cfg CoolerConfig) {
	cfg = normalizeCoolerConfig(cfg)
	c.mu.Lock()
	c.cfg = cfg
	c.mu.Unlock()
}

// SetTarget arms cooling toward target °C: seeds the effective target at the current
// temperature so the ramp eases in from where we are, and clears the error history.
// Does not reset power — in the velocity form the current drive is the accumulator,
// so a retarget continues from the present power.
func (c *Cooler) SetTarget(target float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.target = target
	c.on = true
	c.prevErr = prevErrSentinel
	c.prevErr2 = prevErrSentinel
	if t, err := c.io.ReadTemp(); err == nil {
		c.effTgt = t // anti-shock: start the ramp from the current temperature
		c.lastT = t
		c.primed = true
	} else {
		c.effTgt = target
	}
}

// Stop disengages regulation (does not actively warm; call SetTarget to resume).
func (c *Cooler) Stop() { c.mu.Lock(); c.on = false; c.mu.Unlock() }

// Target returns the final target and the current ramped (effective) setpoint.
func (c *Cooler) Target() (final, effective float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.target, c.effTgt
}

// Power returns the last TEC power % applied.
func (c *Cooler) Power() float64 { c.mu.Lock(); defer c.mu.Unlock(); return c.power }

// rampTarget advances the effective target toward the final target by RampRate (°C/min)
// scaled to this tick's elapsed dt (a time-based anti-shock ramp). RampRate 0 = jump
// straight to the target.
func (c *Cooler) rampTarget(dt time.Duration) {
	if c.cfg.RampRate <= 0 {
		c.effTgt = c.target // no ramp configured: go straight to the target
		return
	}
	step := c.cfg.RampRate / 60 * dt.Seconds() // °C of setpoint movement this tick
	if step <= 0 {
		return // dt ≤ 0 (clock stall / degenerate tick): hold — never collapse the anti-shock ramp
	}
	if d := c.target - c.effTgt; math.Abs(d) <= step {
		c.effTgt = c.target
	} else if d > 0 {
		c.effTgt += step
	} else {
		c.effTgt -= step
	}
}

// SetRampRate sets the cooldown/warmup setpoint slew in °C per minute — the Alpaca
// "cooldown rate" setting. 0 disables the ramp. Safe to call while Run is regulating.
func (c *Cooler) SetRampRate(degPerMin float64) {
	c.mu.Lock()
	c.cfg.RampRate = degPerMin
	c.mu.Unlock()
}

// SeedPower sets the current TEC drive to pct % — in the velocity form the power is
// the accumulator, so this warm-starts (or restores on reconnect) the controller at
// a known drive instead of from 0. The loop then regulates from there. Safe to call
// while Run runs.
func (c *Cooler) SeedPower(pct float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.power = clampf(pct, 0, c.cfg.MaxPower) // power IS the accumulator
}

// Step runs one regulation cycle: read temperature, (optionally) guard against a
// fast transient, advance the setpoint ramp, run the PID, clamp + slew-limit the
// TEC power, and apply it. dt is the elapsed time since the previous Step, used
// only by the rate guard. It returns the temperature read this cycle.
func (c *Cooler) Step(dt time.Duration) (float64, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	temp, err := c.io.ReadTemp()
	if err != nil {
		return 0, err
	}
	if !c.on {
		c.lastT, c.primed = temp, true
		return temp, nil
	}

	// Rate-of-change guard: during a fast swing, skip this step.
	if c.cfg.RateGuard > 0 && c.primed && dt > 0 {
		if rate := math.Abs(temp-c.lastT) / dt.Seconds(); rate > c.cfg.RateGuard {
			c.lastT = temp
			return temp, nil
		}
	}

	c.rampTarget(dt)

	// Velocity (incremental) PID: the output power is the accumulator (clamped to
	// [0, MaxPower]), so it cannot wind up and holds its value at zero error, settling at
	// the steady-state holding power for any setpoint depth.
	//
	//	Δpower = Kp·(e − e₁) + Ki·e + Kd·(e − 2·e₁ + e₂)
	//
	// On the first tick only Ki·e applies (no e₁ yet); on the second, the Kp term joins;
	// from the third the Kd term joins.
	e := temp - c.effTgt // error: positive = too warm, cool harder
	if c.cfg.MaxError > 0 {
		// Clamp the error symmetrically: the loop never acts on more than MaxError, so the
		// drive only creeps and the approach in either direction is gentle. prevErr stores
		// the clamped error so the damping term stays consistent.
		e = clampf(e, -c.cfg.MaxError, c.cfg.MaxError)
	}
	du := c.cfg.Ki * e
	if c.prevErr != prevErrSentinel {
		du += c.cfg.Kp * (e - c.prevErr)
		if c.prevErr2 != prevErrSentinel {
			du += c.cfg.Kd * (e - 2*c.prevErr + c.prevErr2)
		}
	}
	c.prevErr2 = c.prevErr
	c.prevErr = e

	target := clampf(c.power+du, 0, c.cfg.MaxPower)
	if c.cfg.SlewPerStep > 0 {
		target = slew(c.power, target, c.cfg.SlewPerStep)
	}
	if err := c.io.SetTECPower(target); err != nil {
		return temp, err
	}
	c.power = target
	c.lastT, c.primed = temp, true
	return temp, nil
}

// runMaxConsecFails is how many consecutive Step failures Run tolerates before it gives up.
// One transient EP0 error (a timeout during a capture-recovery bus reset, say) must not kill
// regulation with the TEC still energized; ~30 s of solid failures means the device is gone.
const runMaxConsecFails = 15

// Run drives Step on the configured Tick until ctx is canceled. Call it in its own
// goroutine. Transient Step errors are tolerated (the loop keeps ticking and re-tries);
// after runMaxConsecFails consecutive failures it drives the TEC to zero (best-effort —
// never leave an unregulated TEC at its last power) and returns the last error.
func (c *Cooler) Run(ctx context.Context) error {
	t := time.NewTicker(c.cfg.Tick)
	defer t.Stop()
	last := time.Now()
	fails := 0
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-t.C:
			// dt from the wall clock, not the tick timestamp: a Step blocked behind a long
			// readout (ioMu) leaves a stale tick in the channel, and its old timestamp would
			// understate dt — the rate guard then reads a slow real drift as a fast swing
			// (Δtemp over 30 s divided by 200 ms) and skips regulation.
			now := time.Now()
			if _, err := c.Step(now.Sub(last)); err != nil {
				fails++
				if fails >= runMaxConsecFails {
					_ = c.io.SetTECPower(0)
					return fmt.Errorf("cooler: %d consecutive step failures, TEC zeroed: %w", fails, err)
				}
			} else {
				fails = 0
			}
			last = now
		}
	}
}

func clampf(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// slew moves cur toward want by at most step.
func slew(cur, want, step float64) float64 {
	if d := want - cur; d > step {
		return cur + step
	} else if d < -step {
		return cur - step
	}
	return want
}
