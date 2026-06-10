package asicam

// Host-side TEC cooling control: the regulation step, PID initialization, the
// setpoint ramp, SetPowerPerc, the power-to-DAC conversion, and the cooling worker
// thread.
//
// The temperature loop runs ON THE HOST, not in firmware. The camera exposes
// only primitives: read sensor temperature, set TEC drive (power %/DAC), fan and
// anti-dew heater on/off, read humidity. The vendor firmware path reimplements a PID loop with
// an anti-shock setpoint ramp over those primitives; this is the pure-Go analogue.
//
// Owning this loop in-process is also the fix for audit finding F1: the SDK's loop
// state (integral accumulator, setpoint ramp, target) lives in the SDK process and
// is lost on re-enumeration. Here the state is ours, so a reconnect can restore the
// setpoint cleanly instead of cold-starting the cooler.
//
// The controller is deliberately decoupled from USB: it drives a Thermal seam, so
// the whole loop is unit-testable against a simulated thermal plant with no
// hardware (see cooling_test.go) — the same fake-transport discipline the EFW
// driver uses. A real camera supplies a Thermal backed by control transfers
// (SetPowerPerc → the power-to-DAC conversion → SendCMD 0xB2 / SetFPGACoolPower; GetSensorTemp;
// GetHumidity → SendCMD 0x85/0xF5).
//
// The controller is the VELOCITY (incremental) form of the regulation step — Δpower per
// tick, with the TEC drive itself as the accumulator (see Step). That is what lets a deep
// setpoint arrive fast (the drive saturates during the descent) and then settle at the
// correct holding power for any depth without integral windup. HARDWARE-VALIDATED on the
// ASI6200 (gosnap -cool/-regulate). The per-model gains (Kp/Ki/Kd) ship in a calibration
// blob in the SDK and are exposed here as tunables with documented velocity-form defaults.

import (
	"context"
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

// prevErrSentinel is the "no previous error yet" marker (−200 °C, set by PID initialization);
// while set, the velocity form's difference terms (Kp on e−e₁, Kd
// on e−2e₁+e₂) are skipped so the first ticks don't kick on a bogus error delta.
const prevErrSentinel = -200.0

// CoolerConfig holds the tunables. Defaults mirror the SDK's behavior shape; the
// gains are per-model in the SDK (the ASI071 has its own tuning) and should be set
// from the camera profile when known.
type CoolerConfig struct {
	// Kp, Ki, Kd are the VELOCITY-form PID gains (per Δpower = Kp·(e−e₁) + Ki·e +
	// Kd·(e−2e₁+e₂), the PID state set by PID initialization). Ki sets how fast the drive
	// ramps toward the setpoint (arrival speed); Kp and Kd damp the approach. They are NOT
	// positional gains — Ki is a per-tick ramp rate, not an integral weight.
	Kp, Ki, Kd float64

	// Tick is the regulation period. The cooling worker thread steps the regulation step roughly every
	// 200 ms (after a 200 ms warm-up), polling a stop flag at 10 ms granularity.
	Tick time.Duration

	// MaxPower clamps TEC drive (SetPowerPerc clamps to ≤100 %).
	MaxPower float64

	// SlewPerStep caps how far TEC power may move in one tick — a hard safety limit on
	// top of the velocity form's own Ki-paced ramp. 0 disables (the form self-limits).
	SlewPerStep float64

	// RampRate is the cooldown/warmup setpoint slew in °C per MINUTE (tick-independent):
	// the *effective target* walks toward the final target at this rate, which is what
	// produces the SDK's gentle "power eases up from 0" curve (the setpoint ramp ramps the
	// setpoint, seeded at the current temperature — not a power-rate limit). It is the
	// Alpaca-facing "cooldown rate" knob. 0 = jump straight to the target.
	RampRate float64

	// RateGuard skips regulation when |dTemp/dt| exceeds this (°C/s) — the regulation step's
	// transient guard that avoids fighting a fast thermal swing. 0 disables.
	RateGuard float64

	// MaxError clamps the error |temp − setpoint| the PID acts on, in °C (symmetric, both
	// directions): the effective setpoint is held 2–3 °C from the measured temp, so the
	// controller never sees more than
	// MaxError of error, so the drive only ever creeps (Ki·MaxError per tick) and the
	// approach to the setpoint is gentle and overshoot-free — paced off the measured temp,
	// so it self-limits to whatever the TEC can deliver. Applies on cooldown AND warmup.
	// 0 disables (the controller sees the full error → fast/aggressive arrival).
	MaxError float64
}

// DefaultCoolerConfig returns conservative, documented defaults. Gains are a
// starting point, not the factory blob; tune per model.
func DefaultCoolerConfig() CoolerConfig {
	return CoolerConfig{
		// The SDK's own gains from the regulation step (PID initialization log "p%.2f d%.2f" = 0.2/0.6).
		// The SDK form is new_power = power − (0.2·e + 0.6·(e−e₁))
		// with e = setpoint−temp, i.e. a velocity-form PD (no integral). In our parameterization
		// (e = temp−setpoint, Δ = Kp·(e−e₁) + Ki·e + Kd·(e−2e₁+e₂)) that is Kp=0.6, Ki=0.2, Kd=0.
		// Ki here paces the drive ramp (arrival speed), Kp damps the approach.
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

// Cooler is the host-side PID cooling loop over a Thermal. It is safe for the Run
// goroutine to regulate while other goroutines (the Camera / Alpaca server) call
// SetTarget/Stop/Power/Target — all state access is guarded by mu.
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

// NewCooler builds a cooler. A zero-value config field falls back to the default.
func NewCooler(io Thermal, cfg CoolerConfig) *Cooler {
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
	return &Cooler{io: io, cfg: cfg, prevErr: prevErrSentinel, prevErr2: prevErrSentinel}
}

// SetTarget arms cooling toward target °C: it seeds the
// effective target at the current temperature so the ramp eases in from where we are, and
// clears the error history. It does NOT reset power: in the velocity form the current drive
// is the accumulator, so a retarget continues from the present power instead of dropping
// cooling and rebuilding.
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
	step := c.cfg.RampRate / 60 * dt.Seconds() // °C of setpoint movement this tick
	if step <= 0 {
		c.effTgt = c.target
		return
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

// SeedPower sets the current TEC drive to pct % — in the velocity form the power is the
// accumulator, so this directly warm-starts (or restores, audit F1) the controller at a
// known drive instead of from 0. The loop then regulates from there. With the velocity form
// arrival is already fast without a seed (the drive ramps to saturation on its own), so this
// is mainly for restoring a prior steady-state on reconnect. Safe to call while Run runs.
func (c *Cooler) SeedPower(pct float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.power = clampf(pct, 0, c.cfg.MaxPower) // in the velocity form the power IS the accumulator
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

	// Rate-of-change guard: during a fast swing, skip this step (the regulation step).
	if c.cfg.RateGuard > 0 && c.primed && dt > 0 {
		if rate := math.Abs(temp-c.lastT) / dt.Seconds(); rate > c.cfg.RateGuard {
			c.lastT = temp
			return temp, nil
		}
	}

	c.rampTarget(dt)

	// Velocity (incremental) PID: the OUTPUT power is the accumulator, not a separate
	// integral term, so it cannot wind up — it is clamped to [0, MaxPower] — and it HOLDS
	// its value at zero error, settling at exactly the steady-state holding power for any
	// setpoint depth (shallow or deep) with nothing to build up. This is the
	// regulation-step form and the reason a deep target arrives fast (the drive saturates during
	// the descent, then eases to the hold power) without the positional form's overshoot.
	//
	//	Δpower = Kp·(e − e₁) + Ki·e + Kd·(e − 2·e₁ + e₂)
	//
	// On the first tick only Ki·e applies (no e₁ yet); on the second, the Kp term joins;
	// from the third the Kd term joins.
	e := temp - c.effTgt // error: positive = too warm, cool harder
	if c.cfg.MaxError > 0 {
		// Clamp the error symmetrically (the SDK's setpoint-ramp chase): the loop
		// never acts on more than MaxError, so the drive only creeps and the approach — in
		// either direction — is gentle and overshoot-free. prevErr stores the CLAMPED error
		// so the damping term stays consistent (zero while saturated at the clamp).
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

// Run drives Step on the configured Tick until ctx is canceled. It is the
// cooling worker thread as a goroutine; call it in its own goroutine.
func (c *Cooler) Run(ctx context.Context) error {
	t := time.NewTicker(c.cfg.Tick)
	defer t.Stop()
	last := time.Now()
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case now := <-t.C:
			if _, err := c.Step(now.Sub(last)); err != nil {
				return err
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
