package astrocam

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"time"
)

// modeCarrier is implemented by a vendor Regmap that holds the live ReadoutMode (FPS%, output
// depth, bin), so the Camera can mutate it without asserting a concrete regmap type. Mutations
// run under the regmap's mode lock; capture goroutines read the mode (ModeOf) concurrently.
type modeCarrier interface{ updateMode(func(*ReadoutMode)) }

// superSpeedReporter is the optional Transport capability that reports the negotiated USB link
// speed (USB3 SuperSpeed vs USB2 HighSpeed). The readout mode follows the live link, not the
// model's static flag: a USB3 camera on a USB2 port must use the USB2 bandwidth budget, else the
// FX3 framing desyncs and garbles the frame.
type superSpeedReporter interface{ SuperSpeed() bool }

// Camera is an opened ASI camera: a Transport bound to a Model and its Sensor.
type Camera struct {
	t      Transport
	model  Model
	sensor *Sensor
	rm     Regmap
	vend   *Vendor // the wire dialect (Regmap factory + FX3 command table) for this VID

	// mu guards the capture-state scalars below. It is held only across the scalar transitions,
	// never across USB I/O, so a long exposure cannot block a status poll. USB serialization is
	// the transport's ioMu.
	mu sync.Mutex
	// Capture state (capture.go). roiW/roiH default to the full sensor and track SetROI; expDur
	// is the last SetExposure, used by the host-timed status poll.
	roiX, roiY int
	roiW, roiH int  // in binned output pixels (FrameBytes = roiW·roiH·bpp)
	bin        int  // symmetric binning factor (1 = full res); 0 normalized to 1
	hwBin      bool // SetHardwareBin: bin on the sensor where the profile can (default host bin)
	// roiProgrammed is set once SetROI has programmed a window. From then on a mode change (output
	// depth, high-speed, binning, hardware bin) re-programs that window, so the sensor format
	// never lags the mode.
	roiProgrammed bool
	gain          int // last SetGain (ASI 0.1 dB units), surfaced by Gain()
	offset        int // last SetOffset (ASI Brightness / black level), surfaced by Offset()
	status        ExposureStatus
	expDur        time.Duration
	expStart      time.Time
	// expGen is the exposure generation, bumped by every StartExposure and StartVideo. A capture in
	// flight snapshots it, and status writes and abort checks are generation-guarded, so a stale
	// worker can neither un-abort itself nor clobber the new exposure's state.
	//
	// The guard covers driver state, not the device: a Worker halts the readout from a deferred
	// call that runs whatever its generation, since an aborted capture must leave the sensor
	// stopped rather than streaming into a closed host. One capture is therefore in flight per
	// Camera, and both front ends hold to that.
	expGen uint64
	// expAborted is the abort signal for the current generation (StopExposure sets it,
	// StartExposure clears it). It is distinct from status: a status poll deriving Success must
	// not read as an abort to the worker still integrating.
	expAborted bool
	subtype    int // firmware subtype byte (GetFirmwareVer); gates init branches

	// ST4 pulse state (guide.go), under mu: st4Lines is the bitmask of asserted guide lines
	// (bit = GuideDir), st4Pulses counts host-timed PulseGuide calls in flight. Together they
	// back IsPulseGuiding.
	st4Lines  uint8
	st4Pulses int

	// stalls counts worker readout stalls (short/failed reads that triggered a re-arm) over the
	// camera's lifetime, surfaced by StallCount(). Atomic, not mu-guarded.
	stalls atomic.Int64

	// dead is the firmware-crash circuit breaker, set when a short or failed readout is followed by
	// a firmware-version read that differs from baseFW or fails: the FX3 dropped its firmware, and
	// only a USB reset, re-scan and re-Open recovers it. StartExposure and GetDataAfterExp then
	// return ErrDeviceWedged.
	dead     atomic.Bool
	baseFW   uint16 // firmware version read at Init (the known-good baseline)
	baseFWok bool

	// TEC cooling (cooling.go). cooler is non-nil while the regulation loop runs; coolCancel/
	// coolDone own that goroutine; thermal is the hardware seam it drives, kept so CCDTemperature
	// reads work with the cooler off. All four are guarded by mu (held only to swap them).
	cooler     *Cooler
	coolCancel context.CancelFunc
	// coolDone is closed by the running regulation goroutine when it exits. Each loop gets its own
	// channel, captured with its cancel under mu, so a caller stopping one loop waits for that
	// loop rather than whichever is running by the time it looks: DisableCooling releases mu
	// before waiting, and an EnableCooling can start a fresh loop in that window.
	coolDone chan struct{}
	thermal  Thermal
	// coolFault is the error the regulation goroutine gave up with (Run's consecutive-failure
	// exit); set when the loop retires itself, cleared by EnableCooling.
	coolFault error
	// coolW is the hardware Thermal's write cache (thermal_hw.go), invalidated by Init and a
	// device reset; it has its own lock.
	coolW coolWrites
}

// Open binds an already-open Transport to the camera identified by pid, selecting the Sensor
// profile and per-model flags.
func Open(t Transport, vid, pid uint16) (*Camera, error) {
	m, ok := Lookup(vid, pid)
	if !ok {
		return nil, fmt.Errorf("astrocam: unknown/unregistered camera %04x:%04x", vid, pid)
	}
	vend, ok := VendorOf(vid)
	if !ok {
		return nil, fmt.Errorf("astrocam: no vendor protocol registered for VID 0x%04x", vid)
	}
	if m.Sensor == nil {
		return nil, fmt.Errorf("astrocam: model %q has no sensor profile yet", m.Name)
	}
	// The readout mode the FPS/line-time engine reads: link speed from the negotiated USB link,
	// falling back to the model capability, output depth from the sensor ADC width, FPSPercent the
	// ASI bandwidth-overload.
	bpp := 1
	if m.Sensor.Info.BitDepth > 8 {
		bpp = 2
	}
	usb3 := m.USB3 // model capability; refined to the live link if the transport reports it
	if sr, ok := t.(superSpeedReporter); ok {
		usb3 = sr.SuperSpeed()
	}
	// FPSPercent defaults to 100 on USB3 and 40 on USB2, the SDK's USB2 BANDWIDTHOVERLOAD default.
	// A USB2 HighSpeed link cannot drain the readout at full rate without the FX3 FIFO
	// overrunning, which tears frames on the free-run STARVIS sensors.
	fpsPercent := 100
	if !usb3 {
		fpsPercent = 40
	}
	mode := ReadoutMode{USB3: usb3, BytesPerPx: bpp, FPSPercent: fpsPercent}
	// The init-time offset is the vendor's advertised default (OffsetCaps) when the profile has
	// one, else the profile's OffsetDef; the same value OffsetRange reports.
	offset := m.Sensor.OffsetDef
	if m.Sensor.OffsetCaps != nil {
		_, _, offset = m.Sensor.OffsetCaps(vid)
	}
	c := &Camera{
		t: t, model: m, sensor: m.Sensor, rm: vend.newRegmap(t, m.Sensor.Bus, mode), vend: vend,
		roiW: m.Sensor.Info.MaxWidth, roiH: m.Sensor.Info.MaxHeight, bin: 1,
		offset: offset,
	}
	// A cooled camera gets its hardware Thermal seam up front, so CCDTemperature is readable
	// (and EnableCooling defaults to it) without the caller wiring one.
	if m.Cooled {
		c.thermal = c.HardwareThermal()
	}
	return c, nil
}

// SetFPSPercent sets the frame-rate percentage (40..100) the readout throttle (HMAX / line time)
// uses; affects the next SetROI/SetExposure. 100 = fastest readout the bus allows.
func (c *Camera) SetFPSPercent(pct int) {
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.FPSPercent = pct })
	}
}

// FPSPercent reports the effective FPS-percent throttle: the user-set value, or the
// link-dependent default (100 on USB3, 40 on USB2 HighSpeed).
func (c *Camera) FPSPercent() int { return ModeOf(c.rm).FPSPercent }

// SetUSB3 forces the readout mode's link-speed assumption (the bandwidth budget the HMAX/
// line-time math uses): true = USB3 SuperSpeed (bwUSB3), false = USB2 HighSpeed (bwUSB2).
// Normally taken from the link/model. Does not change FPSPercent.
func (c *Camera) SetUSB3(usb3 bool) {
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.USB3 = usb3 })
	}
}

// Name returns the model name.
func (c *Camera) Name() string { return c.model.Name }

// Sensor returns the sensor profile.
func (c *Camera) Sensor() *Sensor { return c.sensor }

// Cooled reports whether the model has a TEC cooler.
func (c *Camera) Cooled() bool { return c.model.Cooled }

// Color reports whether the model is a color (MC) variant.
func (c *Camera) Color() bool { return c.model.Color }

// ST4 reports whether the model has an ST4 guide port (CanPulseGuide).
func (c *Camera) ST4() bool { return c.model.ST4 }

// Close stops any in-flight capture, stops the TEC regulation goroutine, zeroes the TEC if
// cooling was active, and closes the transport. A sensor left free-running into a closed host
// crashes the FX3, and the FPGA holds the TEC drive register, so an un-zeroed cooler keeps
// driving open-loop at its last power. A blocked GetDataAfterExp is not joined: its
// generation-guarded writes are no-ops and the transport's Close interlock covers the I/O. Call
// once.
func (c *Camera) Close() error {
	c.mu.Lock()
	busy := c.status == ExpWorking
	thermal := c.thermal
	c.mu.Unlock()
	if busy {
		_ = c.StopExposure() // best-effort; the transport is about to go away regardless
	}
	c.DisableCooling()
	// Zero the drive whatever the caller did before this. Regulation may have been stopped
	// earlier, a loop may have retired with its own fail-safe write failing, or the TEC may have
	// been driven by hand through the thermal seam, and each leaves it pulling open-loop.
	if c.model.Cooled && thermal != nil {
		c.coolW.invalidate()
		_ = thermal.SetTECPower(0)
	}
	return c.t.Close()
}

// EnableCooling starts host-side TEC regulation toward target °C, driving thermal (nil = the
// hardware seam attached at Open) on a goroutine the Camera owns and joins on Close. Errors on
// a model with no cooler. Calling it again retargets the running loop and applies cfg; naming a
// backend other than the one it is driving restarts the loop on the new one.
func (c *Camera) EnableCooling(thermal Thermal, target float64, cfg CoolerConfig) error {
	if !c.model.Cooled {
		return fmt.Errorf("astrocam: %s has no cooler", c.model.Name)
	}
	c.mu.Lock()
	if thermal == nil {
		thermal = c.thermal // default to the hardware seam attached at Open
	}
	if thermal == nil {
		c.mu.Unlock()
		return fmt.Errorf("astrocam: %s: no thermal backend", c.model.Name)
	}
	// A running loop drives the backend it was built with, so a call naming a different one stops
	// it and starts again. Leaving it has Temperature reading one place while regulation drives
	// another.
	swap := c.cooler != nil && thermal != c.cooler.io
	c.mu.Unlock()
	if swap {
		c.DisableCooling()
	}
	c.mu.Lock()
	c.thermal = thermal
	c.coolFault = nil
	cl := c.cooler
	if cl == nil {
		cl = NewCooler(thermal, cfg)
		c.cooler = cl
		ctx, cancel := context.WithCancel(context.Background())
		c.coolCancel = cancel
		done := make(chan struct{})
		c.coolDone = done
		go func() {
			defer close(done)
			err := cl.Run(ctx)
			if err == nil || ctx.Err() != nil {
				return // stopped by DisableCooling/Close
			}
			// Run retired itself (consecutive failures, TEC zeroed): drop the handle so
			// CoolerOn reports off and the next EnableCooling starts a fresh loop.
			c.mu.Lock()
			if c.cooler == cl {
				c.cooler, c.coolCancel = nil, nil
				c.coolFault = err
			}
			c.mu.Unlock()
			cancel()
		}()
	} else {
		cl.SetConfig(cfg) // a retarget also applies the new tunables
	}
	c.mu.Unlock()
	cl.SetTarget(target) // reads temp (I/O), outside c.mu; the Cooler is self-locked
	return nil
}

// DisableCooling stops the regulation goroutine, joins it, and drives the TEC to zero, which also
// stops the fan. The FPGA holds the drive level, so a loop that merely stopped would leave the
// cooler pulling at its last power with nothing regulating it. The thermal seam stays attached, so
// temperature is still readable. Idempotent.
func (c *Camera) DisableCooling() {
	c.mu.Lock()
	cancel := c.coolCancel
	done := c.coolDone
	running := c.cooler != nil
	thermal := c.thermal
	c.cooler = nil
	c.coolCancel = nil
	c.coolDone = nil
	c.mu.Unlock()
	if cancel != nil {
		cancel()
		if done != nil {
			<-done // this loop, not whichever one is running by now
		}
	}
	if !running || thermal == nil {
		return
	}
	// Zero the drive, unless another caller started a fresh loop while this one was waiting. That
	// loop owns the TEC now, and zeroing it here would fight it for a tick.
	c.mu.Lock()
	superseded := c.cooler != nil
	c.mu.Unlock()
	if !superseded {
		c.coolW.invalidate() // the shutdown zero must reach the wire, cached or not
		_ = thermal.SetTECPower(0)
	}
}

// SetTargetTemp retargets the running cooler (Alpaca SetCCDTemperature). No-op if cooling
// is not enabled.
func (c *Camera) SetTargetTemp(target float64) {
	if cl := c.activeCooler(); cl != nil {
		cl.SetTarget(target)
	}
}

// SetCoolerRampRate sets the cooldown/warmup setpoint slew in °C/min (the Alpaca-facing
// rate knob). No-op if cooling is not enabled.
func (c *Camera) SetCoolerRampRate(degPerMin float64) {
	if cl := c.activeCooler(); cl != nil {
		cl.SetRampRate(degPerMin)
	}
}

// SeedCoolerPower warm-starts the running cooler at pct % drive (Cooler.SeedPower). No-op if
// cooling is off.
func (c *Camera) SeedCoolerPower(pct float64) {
	if cl := c.activeCooler(); cl != nil {
		cl.SeedPower(pct)
	}
}

// CoolerPower returns the last TEC drive %, 0 if cooling is off (Alpaca CoolerPower).
func (c *Camera) CoolerPower() float64 {
	if cl := c.activeCooler(); cl != nil {
		return cl.Power()
	}
	return 0
}

// CoolerOn reports whether the regulation loop is running (Alpaca CoolerOn getter). It turns
// false by itself when the loop retires after repeated thermal I/O failures (CoolerFault).
func (c *Camera) CoolerOn() bool { return c.activeCooler() != nil }

// CoolerFault returns the error the regulation loop gave up with (nil while it runs or was
// stopped by DisableCooling); EnableCooling clears it.
func (c *Camera) CoolerFault() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.coolFault
}

// TargetTemp returns the final and current ramped setpoint, and whether cooling is on.
func (c *Camera) TargetTemp() (final, effective float64, on bool) {
	cl := c.activeCooler()
	if cl == nil {
		return 0, 0, false
	}
	f, e := cl.Target()
	return f, e, true
}

// Temperature reads the sensor temperature in °C from the thermal seam (Alpaca CCDTemperature),
// whether or not the cooler is running. Errors if no thermal backend is attached.
func (c *Camera) Temperature() (float64, error) {
	c.mu.Lock()
	th := c.thermal
	c.mu.Unlock()
	if th == nil {
		return 0, fmt.Errorf("astrocam: %s has no thermal sensor attached", c.model.Name)
	}
	return th.ReadTemp()
}

// activeCooler returns the running cooler under the lock (nil if cooling is off).
func (c *Camera) activeCooler() *Cooler {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.cooler
}

// Info returns the camera geometry/capabilities (sensor facts + model flags). A color (MC) model
// surfaces the die's CFA pattern (Sensor.Info.Bayer, e.g. "RGGB"); a mono (MM) model reports "".
func (c *Camera) Info() CameraInfo {
	info := c.sensor.Info
	if c.model.Color {
		if info.Bayer == "" {
			info.Bayer = "RGGB" // fallback for a color profile without a CFA
		}
	} else {
		info.Bayer = ""
	}
	return info
}

// InitDelayReg is the sentinel register in a sensor init table: reg==0xffff is a delay of Val
// milliseconds, not a register write.
const InitDelayReg uint16 = 0xffff

// Init brings the camera up: FX3 quiesce (stream stop, endpoint reset, flush), firmware
// version read, SPI-flash calibration read, the sensor init table (InitDelayReg entries are
// sleeps), InitFPGA, then the profile's default offset.
func (c *Camera) Init() error {
	if c.sensor.Init == nil {
		return fmt.Errorf("astrocam: %s init sequence not yet transcribed", c.sensor.Name)
	}
	c.coolW.invalidate() // the FPGA cooling registers are rewritten from scratch
	// FX3 streaming state persists across host close and open. A session that did not stop cleanly
	// leaves it streaming, and InitFPGA's writes then fail while the GPIF is busy, though the
	// vendor commands still work. Best-effort on the wire, since a fresh device may NAK.
	if c.vend.Cmds.StreamStop == 0 || c.vend.Cmds.StreamStart == 0 || c.vend.Cmds.Flush == 0 {
		return fmt.Errorf("astrocam: FX3 stream commands not decoded for vendor %s; cannot init %s", c.vend.Name, c.model.Name)
	}
	_ = c.vendorCmd(FX3StreamStop) // 0xAA on ZWO
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	_ = c.vendorCmd(FX3Flush) // 0xAF on ZWO

	// Firmware subtype (GetFirmwareVer's version byte) gates the init branches.
	if fw, err := c.FirmwareVersion(); err == nil {
		c.subtype = int((fw >> 8) & 0xff)
		c.baseFW, c.baseFWok = fw, true // baseline for the firmware-crash (dead) check
	}
	// Before any sensor/FPGA register write. The read brings the FPGA data path into
	// right-aligned RAW16 mode (without it pixels come out MSB-aligned, value<<4).
	c.readCalibrationBlob()
	for _, w := range c.sensor.Init {
		if w.Reg == InitDelayReg {
			time.Sleep(time.Duration(w.Val) * time.Millisecond)
			continue
		}
		if err := c.rm.WriteReg(w.Reg, w.Val); err != nil {
			return fmt.Errorf("init reg 0x%04x: %w", w.Reg, err)
		}
	}
	// FPGA-side init: the non-Sony-register part of InitCamera.
	if c.sensor.InitFPGA != nil {
		if err := c.sensor.InitFPGA(c.rm, c.subtype); err != nil {
			return fmt.Errorf("init fpga: %w", err)
		}
	}
	// Program the full frame, so a camera can capture without a window being set. The profile's
	// SetROI is the only thing that writes the readout geometry (FPGA width/height, per-frame DMA
	// length, optical-black crop, HMAX), and without it the ASI6200 delivers its optical-black
	// rows as image data.
	if c.sensor.SetROI != nil {
		c.mu.Lock()
		x, y, w, h := c.roiX, c.roiY, c.roiW, c.roiH
		c.mu.Unlock()
		if err := c.SetROI(x, y, w, h); err != nil {
			return fmt.Errorf("init roi: %w", err)
		}
	}
	// The profile's default offset / black level (OffsetDef); without it the sensor keeps its
	// power-on pedestal. A later SetOffset overrides it.
	if c.sensor.SetOffset != nil {
		if err := c.sensor.SetOffset(c.rm, c.offset); err != nil {
			return fmt.Errorf("init offset: %w", err)
		}
	}
	return nil
}

// readCalibrationBlob is the init-time SPI-flash calibration read: 0xBE=0 to take the GPIF data
// bus down, 6×0xC3 reading the ~10 KB per-unit config blob (wIndex in 256-byte pages from 0x400,
// 2 KB per chunk plus a 256 B tail), then 0xBE=1. The data is discarded, and the read itself is
// what puts the FPGA data path into right-aligned RAW16 mode. Best-effort.
func (c *Camera) readCalibrationBlob() {
	gpif, flash := c.vend.Cmds.EnableGPIF32DQ, c.vend.Cmds.ReadSPIFlash
	if gpif == 0 || flash == 0 {
		return
	}
	_ = c.t.ControlOut(gpif, 0, 0, nil) // 0xBE=0: quiesce the bus for the flash read
	buf := make([]byte, 2048)
	for _, idx := range []uint16{0x0400, 0x0408, 0x0410, 0x0418, 0x0420} {
		_, _ = c.t.ControlIn(flash, 0, idx, buf)
	}
	_, _ = c.t.ControlIn(flash, 0, 0x0428, buf[:256])
	_ = c.t.ControlOut(gpif, 1, 0, nil) // 0xBE=1: re-enable the data bus
}

// SetGain sets analog gain (ASI 0.1 dB units) via the sensor profile.
func (c *Camera) SetGain(gain int) error {
	if c.sensor.SetGain == nil {
		return fmt.Errorf("astrocam: %s SetGain not implemented", c.sensor.Name)
	}
	if err := c.sensor.SetGain(c.rm, gain); err != nil {
		return err
	}
	c.mu.Lock()
	c.gain = gain
	c.mu.Unlock()
	return nil
}

// SetOffset sets the sensor offset / black level (ASI Brightness) via the sensor profile.
func (c *Camera) SetOffset(offset int) error {
	if c.sensor.SetOffset == nil {
		return fmt.Errorf("astrocam: %s SetOffset not implemented", c.sensor.Name)
	}
	if err := c.sensor.SetOffset(c.rm, offset); err != nil {
		return err
	}
	c.mu.Lock()
	c.offset = offset
	c.mu.Unlock()
	return nil
}

// Offset returns the sensor offset / black level (ASI Brightness): the register read-back when the
// profile has Sensor.GetOffset, else the last value set.
//
// The read-back and the requested value can differ. The IMX174's black-level register does not
// survive a capture cycle, so between frames it reads 0 while the requested offset is whatever
// SetOffset last received, and the next arm rewrites it.
func (c *Camera) Offset() int {
	if c.sensor.GetOffset != nil {
		if v, err := c.sensor.GetOffset(c.rm); err == nil {
			return v
		}
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.offset
}

// SetOutputDepth selects the output bytes-per-pixel (1 = RAW8, 2 = RAW16), reprograms the FX3
// output width, and carries the depth on the readout mode. A programmed window is re-programmed
// for the new depth.
func (c *Camera) SetOutputDepth(bpp int) error {
	if bpp != 1 && bpp != 2 {
		return fmt.Errorf("astrocam: output depth %d invalid (1 = RAW8, 2 = RAW16)", bpp)
	}
	prevBpp := ModeOf(c.rm).BytesPerPx
	if err := SetFPGAOutputWidth(c.rm, bpp >= 2); err != nil {
		return err
	}
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.BytesPerPx = bpp })
	}
	c.mu.Lock()
	prev := c.snapshotWindow()
	c.mu.Unlock()
	if err := c.reprogramWindow(); err != nil {
		// Put the depth back so FrameBytes keeps matching the window the sensor holds.
		if r, ok := c.rm.(modeCarrier); ok {
			r.updateMode(func(m *ReadoutMode) { m.BytesPerPx = prevBpp })
		}
		_ = SetFPGAOutputWidth(c.rm, prevBpp >= 2)
		c.restoreWindow(prev)
		return err
	}
	return nil
}

// reprogramWindow re-programs the stored window after a mode change that alters how it is encoded
// (output depth, high-speed, binning). It is a no-op until SetROI has run once. The exposure is
// not re-applied: a duration is a per-capture parameter the caller programs after the mode.
func (c *Camera) reprogramWindow() error {
	c.mu.Lock()
	prog := c.roiProgrammed
	x, y, w, h := c.roiX, c.roiY, c.roiW, c.roiH
	c.mu.Unlock()
	if !prog {
		return nil
	}
	return c.SetROI(x, y, w, h)
}

// OutputDepth returns the current output bytes-per-pixel (1 = RAW8, 2 = RAW16).
func (c *Camera) OutputDepth() int { return ModeOf(c.rm).BytesPerPx }

// SetHighSpeedMode selects the sensor's 10-bit high-speed readout, about 2× the frame rate. It
// takes effect for RAW8 output only, so call SetOutputDepth(1) as well. A programmed window is
// re-programmed for the new mode.
func (c *Camera) SetHighSpeedMode(on bool) error {
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.HighSpeed = on })
	}
	return c.reprogramWindow()
}

// HighSpeedMode reports whether the 10-bit high-speed readout is selected.
func (c *Camera) HighSpeedMode() bool { return ModeOf(c.rm).HighSpeed }

// OffsetRange returns the supported offset bounds + default (Alpaca OffsetMin/OffsetMax). ok is
// false if the profile has no offset control.
func (c *Camera) OffsetRange() (min, max, def int, ok bool) {
	if c.sensor.SetOffset == nil {
		return 0, 0, 0, false
	}
	if c.sensor.OffsetCaps != nil { // vendor-specific range
		min, max, def = c.sensor.OffsetCaps(c.rm.VID())
		return min, max, def, true
	}
	return c.sensor.OffsetMin, c.sensor.OffsetMax, c.sensor.OffsetDef, true
}

// SetExposure sets the exposure time via the sensor profile.
func (c *Camera) SetExposure(d time.Duration) error {
	if c.sensor.SetExposure == nil {
		return fmt.Errorf("astrocam: %s SetExposure not implemented", c.sensor.Name)
	}
	// Clamp to the sensor's exposure range so the stored expDur (which the host-timed worker
	// integrates against) matches the programmed registers.
	if min := time.Duration(c.sensor.ExpMinUs) * time.Microsecond; c.sensor.ExpMinUs > 0 && d < min {
		d = min
	}
	if max := time.Duration(c.sensor.ExpMaxUs) * time.Microsecond; c.sensor.ExpMaxUs > 0 && d > max {
		d = max
	}
	if err := c.sensor.SetExposure(c.rm, d); err != nil {
		return err
	}
	c.mu.Lock()
	c.expDur = d
	c.mu.Unlock()
	return nil
}

// SetROI sets the readout window via the sensor profile and records the geometry for FrameBytes.
// (x, y, w, h) is in binned output pixels at the current Binning: 0 ≤ x, x+w ≤ MaxWidth/bin
// (likewise y/h), and the sensor-side window must have a width that is a multiple of 8 and an
// even height (the SDK's rule; the FX3 frames whole 32-bit words). An out-of-range or misaligned
// size is rejected, not clamped. The start is aligned down to the profile's SetStartPos grid
// (Sensor.ROIStartAlign) and ROI reports the aligned start.
func (c *Camera) SetROI(x, y, w, h int) error {
	if c.sensor.SetROI == nil {
		return fmt.Errorf("astrocam: %s SetROI not implemented", c.sensor.Name)
	}
	c.mu.Lock()
	bin, hwBin := c.bin, c.hwBin
	c.mu.Unlock()
	if bin < 1 {
		bin = 1
	}
	maxW, maxH := c.sensor.Info.MaxWidth/bin, c.sensor.Info.MaxHeight/bin
	if x < 0 || y < 0 || w < 1 || h < 1 {
		return fmt.Errorf("astrocam: ROI (%d,%d %dx%d) invalid: offset must be ≥0 and size ≥1", x, y, w, h)
	}
	if x+w > maxW || y+h > maxH {
		return fmt.Errorf("astrocam: ROI (%d,%d %dx%d) exceeds %dx%d at bin %d", x, y, w, h, maxW, maxH, bin)
	}
	hw, soft := c.binSplit(bin, hwBin)
	// The SDK's window rule, measured against ASISetROIFormat on an ASI6200MC: the width is a
	// multiple of 8 and the height even, counted in output pixels when the sensor bins and in
	// sensor pixels when the host does. The SDK enforces exactly that split: 4788×3194 at bin 2 is
	// accepted host-binned (sensor extent 9576) and rejected with ASI_ERROR_INVALID_SIZE under
	// ASI_HARDWARE_BIN (4788 % 8 = 4), while 3192×2128 at hardware bin 3 is accepted. The FX3
	// frames whole 32-bit words, so an odd size shifts the frames that follow.
	if hw > 1 {
		if w%8 != 0 || h%2 != 0 {
			return fmt.Errorf("astrocam: ROI %dx%d at sensor bin %d: width must be a multiple of 8 and height even", w, h, hw)
		}
	} else if (w*bin)%8 != 0 || (h*bin)%2 != 0 {
		return fmt.Errorf("astrocam: ROI %dx%d at bin %d: the sensor window %dx%d must have a width that is a multiple of 8 and an even height", w, h, bin, w*bin, h*bin)
	}
	// Align the sensor-pixel start down to the profile's grid, on a step the bin also divides, so
	// the reported binned start is exact.
	if c.sensor.ROIStartAlign != nil {
		ax, ay := c.sensor.ROIStartAlign(hw)
		x = alignStart(x, bin, ax)
		y = alignStart(y, bin, ay)
	}
	sx, sy, sw, sh, sbin := x*soft, y*soft, w*soft, h*soft, hw
	if soft > 1 && c.Color() && ((w*soft)%2 != 0 || (h*soft)%2 != 0) {
		// Color bins same-color samples to keep the Bayer mosaic, so the host-binned region must
		// cover an even extent on both axes: the 6200 accepts bin 2 (9576×6388) and bin 4 (→1597)
		// but not bin 3 (6388/3 is odd).
		return fmt.Errorf("astrocam: color software binning needs an even sensor extent for %s (bin %d → %dx%d px is odd); use a bin that evenly divides the sensor", c.sensor.Name, bin, w*soft, h*soft)
	}
	if err := c.sensor.SetROI(c.rm, sx, sy, sw, sh, sbin); err != nil {
		return err
	}
	// The live mode carries the sensor-side window and bin, so SetExposure's VMAX/HMAX follow
	// them, and SoftBin tells the read path to bin the rest.
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) {
			m.Width, m.Height = sw, sh
			m.Bin = sbin
			m.SoftBin = soft
		})
	}
	c.mu.Lock()
	c.roiX, c.roiY, c.roiW, c.roiH = x, y, w, h // client/output ROI (binned pixels)
	c.roiProgrammed = true
	c.mu.Unlock()
	// The offset encoding can depend on the sensor bin this call just programmed, as the IMX455
	// scales it, so the black level is rewritten in the new mode's terms. Left alone it keeps the
	// previous mode's encoding and means a different offset than the one that was set.
	return c.ReapplyOffset()
}

// alignStart aligns a binned start coordinate so that its sensor-pixel position (v·bin) is a
// multiple of align, rounding down on a step both align and bin divide (their lcm), so the
// returned binned coordinate is exact.
func alignStart(v, bin, align int) int {
	if align <= 1 || bin < 1 {
		return v
	}
	l := align * bin / gcd(align, bin)
	sensor := v * bin
	sensor -= sensor % l
	return sensor / bin
}

func gcd(a, b int) int {
	for b != 0 {
		a, b = b, a%b
	}
	return a
}

// SetBinning selects the symmetric binning factor for later SetROI calls and captures. bin must be
// one of the sensor's Bins. It resets the ROI to the full binned frame, so call it before a
// sub-frame SetROI.
func (c *Camera) SetBinning(bin int) error {
	if bin < 1 {
		return fmt.Errorf("astrocam: binning %d invalid (must be ≥1)", bin)
	}
	ok := false
	for _, b := range c.sensor.Info.Bins {
		if b == bin {
			ok = true
			break
		}
	}
	if !ok {
		return fmt.Errorf("astrocam: %s does not support bin %d (supported: %v)", c.sensor.Name, bin, c.sensor.Info.Bins)
	}
	c.mu.Lock()
	hwBin := c.hwBin
	c.mu.Unlock()
	w, h := c.fullWindow(bin, hwBin)
	c.mu.Lock()
	prev := c.snapshotWindow()
	c.bin = bin
	c.roiX, c.roiY = 0, 0
	c.roiW, c.roiH = w, h
	c.mu.Unlock()
	if prev.programmed {
		if err := c.SetROI(0, 0, w, h); err != nil {
			c.restoreWindow(prev)
			return err
		}
	}
	return nil
}

// binSplit divides a binning factor between the sensor and the host: the sensor takes the largest
// of the profile's HWBins that divides it (only when hardware binning is selected), the host takes
// the rest. bin 4 on a {2,3} profile is sensor 2 × host 2, the SDK's own split.
func (c *Camera) binSplit(bin int, hwBin bool) (hw, soft int) {
	if bin < 1 {
		bin = 1
	}
	hw = 1
	if hwBin {
		for _, b := range c.sensor.HWBins {
			if b > hw && b <= bin && bin%b == 0 {
				hw = b
			}
		}
	}
	return hw, bin / hw
}

// fullWindow is the whole frame at bin, rounded down to the size rule SetROI enforces for the
// current hardware/host split. A plain Max/bin is not always legal: on the IMX455 host-binned it
// gives 2129 rows at bin 3, a 6387-row sensor extent; sensor-binned it gives 4788 columns at bin
// 2, which the SDK refuses because the binned width is not a multiple of 8.
func (c *Camera) fullWindow(bin int, hwBin bool) (w, h int) {
	if bin < 1 {
		bin = 1
	}
	w, h = c.sensor.Info.MaxWidth/bin, c.sensor.Info.MaxHeight/bin
	stepW, stepH := 8, 2
	if hw, _ := c.binSplit(bin, hwBin); hw == 1 {
		// Host binning: the rule counts sensor pixels, so the binned step is smaller.
		stepW, stepH = 8/gcd(8, bin), 2/gcd(2, bin)
	}
	return w - w%stepW, h - h%stepH
}

// window is the geometry a mode setter must restore when it cannot program its change.
type window struct {
	bin        int
	hwBin      bool
	x, y, w, h int
	programmed bool
}

// snapshotWindow records the current geometry; the caller holds mu.
func (c *Camera) snapshotWindow() window {
	return window{bin: c.bin, hwBin: c.hwBin, x: c.roiX, y: c.roiY, w: c.roiW, h: c.roiH, programmed: c.roiProgrammed}
}

// restoreWindow puts the recorded geometry back after a failed mode change, so ROI and FrameBytes
// keep describing the window the sensor holds. A profile that failed part-way through its register
// writes leaves the device in its own hands, and this restores the driver's view of it.
func (c *Camera) restoreWindow(p window) {
	c.mu.Lock()
	c.bin, c.hwBin = p.bin, p.hwBin
	c.roiX, c.roiY, c.roiW, c.roiH = p.x, p.y, p.w, p.h
	c.roiProgrammed = p.programmed
	c.mu.Unlock()
	if p.programmed {
		_ = c.reprogramWindow() // best effort: re-state the window the caller still believes in
	}
}

// SetHardwareBin selects where binning happens (the SDK's ASI_HARDWARE_BIN). false, the default,
// bins on the host from a bin-1 readout at every depth. true bins on the sensor where the profile
// has a hardware mode, and host-bins any remainder. Hardware bin reads out faster, but the sensor
// sums the block and saturates 4× sooner at bin 2. The two splits disagree on which sizes are
// legal, so this resets the ROI to the full binned frame of the new split. Call it before a
// sub-frame SetROI.
func (c *Camera) SetHardwareBin(on bool) error {
	c.mu.Lock()
	prev := c.snapshotWindow()
	bin := c.bin
	c.mu.Unlock()
	// The split decides which windows are legal (output pixels when the sensor bins, sensor pixels
	// when the host does), so the frame resets to the full window of the new split.
	w, h := c.fullWindow(bin, on)
	c.mu.Lock()
	c.hwBin = on
	c.roiX, c.roiY = 0, 0
	c.roiW, c.roiH = w, h
	c.mu.Unlock()
	if prev.programmed {
		if err := c.SetROI(0, 0, w, h); err != nil {
			c.restoreWindow(prev)
			return err
		}
	}
	return nil
}

// HardwareBin reports whether sensor-side binning is selected (SetHardwareBin).
func (c *Camera) HardwareBin() bool { c.mu.Lock(); defer c.mu.Unlock(); return c.hwBin }

// --- Alpaca capability getters: current state + sensor-declared ranges. ---

// Gain returns the last gain set (ASI 0.1 dB units).
func (c *Camera) Gain() int { c.mu.Lock(); defer c.mu.Unlock(); return c.gain }

// GainRange returns the supported gain bounds (ASI 0.1 dB units; Alpaca GainMin/GainMax).
// max == 0 means the profile declares no range.
func (c *Camera) GainRange() (min, max int) {
	if c.sensor.GainCaps != nil { // vendor-specific range
		return c.sensor.GainCaps(c.rm.VID())
	}
	return c.sensor.GainMin, c.sensor.GainMax
}

// Exposure returns the last exposure set.
func (c *Camera) Exposure() time.Duration { return c.expDuration() }

// ExposureRange returns the supported exposure bounds (Alpaca ExposureMin/ExposureMax).
// max == 0 means the profile declares no range.
func (c *Camera) ExposureRange() (min, max time.Duration) {
	return time.Duration(c.sensor.ExpMinUs) * time.Microsecond,
		time.Duration(c.sensor.ExpMaxUs) * time.Microsecond
}

// ExposureStep is the exposure resolution (whole µs; Alpaca ExposureResolution).
func (c *Camera) ExposureStep() time.Duration { return time.Microsecond }

// ROI returns the current readout window (x, y, width, height): Alpaca StartX/StartY/NumX/NumY.
func (c *Camera) ROI() (x, y, w, h int) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.roiX, c.roiY, c.roiW, c.roiH
}

// Bins returns the symmetric binning factors the sensor supports (Alpaca SupportedBins /
// MaxBinX). SetROI errors on a factor whose readout mode the profile has not decoded.
func (c *Camera) Bins() []int { return c.sensor.Info.Bins }

// Binning returns the current symmetric binning factor (set by SetBinning, default 1).
func (c *Camera) Binning() int {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.bin < 1 {
		return 1
	}
	return c.bin
}

// StartExposure is implemented in capture.go (the snap data plane).
