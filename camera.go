package astrocam

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"time"
)

// modeCarrier is implemented by a vendor Regmap that holds the live ReadoutMode the Camera
// updates (FPS%, output depth, bin), letting the Camera mutate it without asserting a concrete
// regmap type. Mutations run under the regmap's mode lock — capture goroutines read the mode
// (ModeOf) concurrently with camera methods changing it.
type modeCarrier interface{ updateMode(func(*ReadoutMode)) }

// superSpeedReporter is the optional Transport capability that reports the negotiated USB link
// speed (USB3 SuperSpeed vs USB2 HighSpeed). The readout mode follows the live link, not the
// model's static flag: a USB3 camera on a USB2 port must use the USB2 bandwidth/FPS budget, else
// the FX3 framing desyncs and garbles the frame. Transports that can't report speed leave the
// model default in force.
type superSpeedReporter interface{ SuperSpeed() bool }

// Camera is an opened ASI camera: a Transport bound to a Model and its Sensor.
type Camera struct {
	t      Transport
	model  Model
	sensor *Sensor
	rm     Regmap
	pid    uint16

	// mu guards the mutable capture-state scalars below (the Camera is called from several
	// goroutines at once). It is held only across the scalar transitions, never across USB I/O
	// (arm/read/stop run unlocked) so a long exposure can't block a status poll. USB safety is
	// the transport's ioMu, a separate concern.
	mu sync.Mutex
	// Capture state (the snap data plane; see capture.go). roiW/roiH default to the full sensor
	// and track SetROI; expDur is the last SetExposure, used by the host-timed status poll.
	roiX, roiY int
	roiW, roiH int // in BINNED output pixels (FrameBytes = roiW·roiH·bpp)
	bin        int // symmetric binning factor (1 = full res); 0 normalized to 1
	gain       int // last SetGain (ASI 0.1 dB units), surfaced by Gain()
	offset     int // last SetOffset (ASI Brightness / black level), surfaced by Offset()
	status     ExposureStatus
	expDur     time.Duration
	expStart   time.Time
	// expGen is the exposure generation: bumped by every StartExposure/StartVideo. A capture
	// in flight snapshots it; status writes and abort checks are generation-guarded so a stale
	// worker (aborted then superseded by a new StartExposure) can neither un-abort itself nor
	// clobber the new exposure's state.
	expGen uint64
	// expAborted is the dedicated abort signal for the CURRENT generation (StopExposure sets
	// it; StartExposure clears it). Distinct from status: a status poll deriving SUCCESS must
	// not read as an abort to the worker still integrating.
	expAborted bool
	expLight   bool // light/dark flag of the current exposure (recovery re-arms with the same)
	subtype    int  // firmware subtype byte (GetFirmwareVer); gates init branches

	// stalls counts worker readout stalls (short/failed reads that triggered a re-arm) over the
	// camera's lifetime, surfaced by StallCount(). Atomic, not mu-guarded.
	stalls atomic.Int64

	// dead is the firmware-crash circuit breaker, set when a short/failed readout is followed by
	// a firmware-version read that changed from baseFW (or won't read) — the FX3 dropped its
	// firmware and only a USB reset + re-scan + re-Open recovers it. Once dead, the camera refuses
	// to arm new frames (StartExposure/GetDataAfterExp return ErrDeviceWedged).
	dead     atomic.Bool
	baseFW   uint16 // firmware version read at Init (the known-good baseline)
	baseFWok bool

	// TEC cooling (cooling.go). cooler is non-nil while the regulation loop runs; coolCancel/
	// coolWg own that goroutine's lifetime; thermal is the hardware seam it drives so
	// CCDTemperature reads work with the cooler off. All four are guarded by mu (held only to swap
	// them, never across the cooler's own I/O).
	cooler     *Cooler
	coolCancel context.CancelFunc
	coolWg     sync.WaitGroup
	thermal    Thermal
}

// Open binds an already-open Transport to the camera identified by pid, selecting the right
// Sensor profile and per-model flags.
func Open(t Transport, vid, pid uint16) (*Camera, error) {
	m, ok := Lookup(vid, pid)
	if !ok {
		return nil, fmt.Errorf("asicam: unknown/unregistered camera %04x:%04x", vid, pid)
	}
	vend, ok := VendorOf(vid)
	if !ok {
		return nil, fmt.Errorf("asicam: no vendor protocol registered for VID 0x%04x", vid)
	}
	if m.Sensor == nil {
		return nil, fmt.Errorf("asicam: model %q has no sensor profile yet", m.Name)
	}
	// The readout mode the shared FPS/line-time engine reads (fps.go): link speed from the
	// negotiated USB link (falls back to the model capability), output depth from the sensor ADC
	// width (RAW16 for >8-bit), FPSPercent the ASI bandwidth-overload. Override per session with
	// SetUSB3 / SetFPSPercent.
	bpp := 1
	if m.Sensor.Info.BitDepth > 8 {
		bpp = 2
	}
	usb3 := m.USB3 // model capability; refined to the live link if the transport reports it
	if sr, ok := t.(superSpeedReporter); ok {
		usb3 = sr.SuperSpeed()
	}
	// FPSPercent defaults to 100 (full speed) on a USB3 link, but a USB2 HighSpeed link can't drain
	// the readout at full rate without the FX3 FIFO overrunning (frame tearing on the free-run
	// STARVIS sensors), so default it to 50 there — a conservative throttle (larger HMAX). This is
	// a default only: the user overrides either with SetFPSPercent (Alpaca "fpspercent" action).
	fpsPercent := 100
	if !usb3 {
		fpsPercent = 40 // slow USB2 link: matches the SDK's USB2 default (its runtime BANDWIDTHOVERLOAD default)
	}
	mode := ReadoutMode{USB3: usb3, BytesPerPx: bpp, FPSPercent: fpsPercent}
	c := &Camera{
		t: t, model: m, sensor: m.Sensor, rm: vend.newRegmap(t, m.Sensor.Bus, mode), pid: pid,
		roiW: m.Sensor.Info.MaxWidth, roiH: m.Sensor.Info.MaxHeight, bin: 1,
		offset: m.Sensor.OffsetDef,
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

// SetUSB3 forces the readout mode's link-speed assumption (the bandwidth budget the HMAX/
// line-time math uses): true = USB3 SuperSpeed (bwUSB3), false = USB2 HighSpeed (bwUSB2).
// Normally taken from the model; forcing it exercises the USB2 vs USB3 path on a fixed link. Does
// not change FPSPercent (set that with SetFPSPercent).
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

// Close quiesces any in-flight capture (StopExposure: abort flag + sensor master-stop +
// FPGA stop + flush — a sensor left free-running into a closed host is the FX3-crash
// mechanism), stops the TEC regulation goroutine (joining it cleanly), zeroes the TEC if
// cooling was active (a disconnecting client must not leave the cooler driving open-loop at
// its last power — the FPGA holds the drive register), and closes the transport. The stop
// writes queue behind a bulk read still in flight (ioMu), which is bounded by the read's
// total gate; a blocked GetDataAfterExp is not joined — its generation-guarded writes are
// no-ops and the transport's own Close interlock covers the I/O. DisableCooling alone still
// leaves the TEC as-is for deliberate stop-regulation-keep-drive uses. Safe to call once.
func (c *Camera) Close() error {
	c.mu.Lock()
	busy := c.status == ExpWorking
	hadCooler := c.cooler != nil
	thermal := c.thermal
	c.mu.Unlock()
	if busy {
		_ = c.StopExposure() // best-effort; the transport is about to go away regardless
	}
	c.DisableCooling()
	if hadCooler && thermal != nil {
		_ = thermal.SetTECPower(0)
	}
	return c.t.Close()
}

// EnableCooling starts host-side TEC regulation toward target °C, driving the given Thermal seam
// on a background goroutine the Camera owns and joins on Close. Errors on a model with no cooler.
// The Thermal is injected and remembered so CCDTemperature can be read with the cooler off.
// Calling it again retargets the running loop instead of starting a second.
func (c *Camera) EnableCooling(thermal Thermal, target float64, cfg CoolerConfig) error {
	if !c.model.Cooled {
		return fmt.Errorf("asicam: %s has no cooler", c.model.Name)
	}
	c.mu.Lock()
	if thermal == nil {
		thermal = c.thermal // default to the hardware seam attached at Open
	}
	if thermal == nil {
		c.mu.Unlock()
		return fmt.Errorf("asicam: %s: no thermal backend", c.model.Name)
	}
	c.thermal = thermal
	cl := c.cooler
	if cl == nil {
		cl = NewCooler(thermal, cfg)
		c.cooler = cl
		ctx, cancel := context.WithCancel(context.Background())
		c.coolCancel = cancel
		c.coolWg.Add(1)
		go func() { defer c.coolWg.Done(); _ = cl.Run(ctx) }()
	} else {
		cl.SetConfig(cfg) // a retarget also applies the new tunables (previously ignored)
	}
	c.mu.Unlock()
	cl.SetTarget(target) // reads temp (I/O) — outside c.mu; the Cooler is self-locked
	return nil
}

// DisableCooling stops the regulation goroutine and joins it. The TEC is left as-is; the camera
// keeps its thermal seam so temperature is still readable. Idempotent.
func (c *Camera) DisableCooling() {
	c.mu.Lock()
	cancel := c.coolCancel
	c.cooler = nil
	c.coolCancel = nil
	c.mu.Unlock()
	if cancel != nil {
		cancel()
		c.coolWg.Wait()
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

// SeedCoolerPower warm-starts the running cooler at pct % drive instead of ramping from 0 (see
// Cooler.SeedPower). No-op if cooling is off.
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

// CoolerOn reports whether the regulation loop is running (Alpaca CoolerOn getter).
func (c *Camera) CoolerOn() bool { return c.activeCooler() != nil }

// TargetTemp returns the final and current ramped setpoint, and whether cooling is on.
func (c *Camera) TargetTemp() (final, effective float64, on bool) {
	cl := c.activeCooler()
	if cl == nil {
		return 0, 0, false
	}
	f, e := cl.Target()
	return f, e, true
}

// Temperature reads the current sensor temperature in °C from the thermal seam (Alpaca
// CCDTemperature), independent of whether the cooler is running. Errors if no thermal backend
// is attached.
func (c *Camera) Temperature() (float64, error) {
	c.mu.Lock()
	th := c.thermal
	c.mu.Unlock()
	if th == nil {
		return 0, fmt.Errorf("asicam: %s has no thermal sensor attached", c.model.Name)
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
// surfaces the die's CFA pattern (Sensor.Info.Bayer, e.g. "RGGB"); a mono (MM) model reports no
// Bayer.
func (c *Camera) Info() CameraInfo {
	info := c.sensor.Info
	if c.model.Color {
		if info.Bayer == "" {
			info.Bayer = "RGGB" // fallback if a color profile didn't specify its CFA
		}
	} else {
		info.Bayer = ""
	}
	return info
}

// InitDelayReg is the sentinel register in a sensor init table: reg==0xffff is a delay of Val
// milliseconds, not a register write.
const InitDelayReg uint16 = 0xffff

// Init replays the sensor's init register sequence, honoring InitDelayReg entries as sleeps.
func (c *Camera) Init() error {
	if c.sensor.Init == nil {
		return fmt.Errorf("asicam: %s init sequence not yet transcribed", c.sensor.Name)
	}
	// FX3 streaming state persists across host close/open. A prior session that didn't stop
	// cleanly can leave it streaming, and FPGA register writes in InitFPGA then fail while the
	// GPIF is busy. Quiesce first with FX3 vendor commands (stop, clear pipe, flush), which work
	// even when the FPGA path is blocked. Best-effort; errors are expected on a fresh device.
	_ = c.t.ControlOut(cmdStreamStop, 0, 0, nil) // 0xAA: stop any leftover async stream
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	_ = c.t.ControlOut(cmdFlush, 0, 0, nil) // 0xAF: flush the pipeline

	// Firmware subtype (GetFirmwareVer's version byte) gates the init branches.
	if fw, err := c.FirmwareVersion(); err == nil {
		c.subtype = int((fw >> 8) & 0xff)
		c.baseFW, c.baseFWok = fw, true // baseline for the firmware-crash (dead) check
	}
	// Init-time SPI-flash calibration read, before any sensor/FPGA register write: disable the
	// GPIF data bus, read the ~10 KB per-unit config blob from flash, re-enable the bus. The blob
	// is discarded; the read brings the FPGA data path into right-aligned RAW16 mode (without it
	// pixels come out MSB-aligned, value<<4).
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
	// FPGA-side init (brings up the readout pipeline) — the non-Sony-register part of InitCamera.
	// The GPIF data bus was already enabled once by readCalibrationBlob above.
	if c.sensor.InitFPGA != nil {
		if err := c.sensor.InitFPGA(c.rm, c.subtype); err != nil {
			return fmt.Errorf("init fpga: %w", err)
		}
	}
	// Apply the profile's default offset / black level (e.g. 30 for the 462); without it the
	// sensor keeps its power-on pedestal. An explicit SetOffset later overrides this.
	if c.sensor.SetOffset != nil {
		if err := c.sensor.SetOffset(c.rm, c.offset); err != nil {
			return fmt.Errorf("init offset: %w", err)
		}
	}
	return nil
}

// readCalibrationBlob runs the init-time SPI-flash calibration read (opcode 0xC3). With the GPIF
// data bus disabled (0xBE=0) it reads the ~10 KB per-unit config blob — wIndex in 256-byte pages
// from 0x400, 2 KB (8 pages) per chunk plus a 256 B tail — then re-enables the bus (0xBE=1).
// Shape: 0xBE=0 → 6×0xC3 → 0xBE=1. The data is discarded; the read brings the FPGA data path into
// right-aligned RAW16 mode. Best-effort: a fresh/odd device may NAK.
func (c *Camera) readCalibrationBlob() {
	_ = c.t.ControlOut(cmdEnableGPIF32DQ, 0, 0, nil) // 0xBE=0: quiesce the bus for the flash read
	buf := make([]byte, 2048)
	for _, idx := range []uint16{0x0400, 0x0408, 0x0410, 0x0418, 0x0420} {
		_, _ = c.t.ControlIn(reqReadSPIFlash, 0, idx, buf)
	}
	_, _ = c.t.ControlIn(reqReadSPIFlash, 0, 0x0428, buf[:256])
	_ = c.t.ControlOut(cmdEnableGPIF32DQ, 1, 0, nil) // 0xBE=1: re-enable the data bus
}

// SetGain sets analog gain (ASI 0.1 dB units) via the sensor profile.
func (c *Camera) SetGain(gain int) error {
	if c.sensor.SetGain == nil {
		return fmt.Errorf("asicam: %s SetGain not implemented", c.sensor.Name)
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
		return fmt.Errorf("asicam: %s SetOffset not implemented", c.sensor.Name)
	}
	if err := c.sensor.SetOffset(c.rm, offset); err != nil {
		return err
	}
	c.mu.Lock()
	c.offset = offset
	c.mu.Unlock()
	return nil
}

// Offset returns the last offset set (ASI Brightness / black level).
func (c *Camera) Offset() int { c.mu.Lock(); defer c.mu.Unlock(); return c.offset }

// SetOutputDepth selects the output bytes-per-pixel (1 = RAW8, 2 = RAW16) and reprograms the FX3
// output width, carrying the depth on the readout mode so the next SetROI/SetExposure/FrameBytes
// follow it. Call before arming a capture, not mid-stream.
func (c *Camera) SetOutputDepth(bpp int) error {
	if bpp != 1 && bpp != 2 {
		return fmt.Errorf("asicam: output depth %d invalid (1 = RAW8, 2 = RAW16)", bpp)
	}
	if err := SetFPGAOutputWidth(c.rm, bpp >= 2); err != nil {
		return err
	}
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.BytesPerPx = bpp })
	}
	return nil
}

// OutputDepth returns the current output bytes-per-pixel (1 = RAW8, 2 = RAW16).
func (c *Camera) OutputDepth() int { return ModeOf(c.rm).BytesPerPx }

// SetHighSpeedMode selects the sensor's 10-bit high-speed readout — ~2× the frame rate by trading
// 2 bits of depth. Only effective for RAW8 output, so also call SetOutputDepth(1). Carried on the
// live mode; the next SetROI/SetExposure reprogram the sensor format, pixel clock, and HMAX floor.
// Call before arming.
func (c *Camera) SetHighSpeedMode(on bool) {
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.HighSpeed = on })
	}
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
		return fmt.Errorf("asicam: %s SetExposure not implemented", c.sensor.Name)
	}
	// Clamp to the sensor's exposure range (e.g. IMX174 ≤31µs→32µs, >2000s→2000s). Clamping here
	// keeps the stored expDur — which the host-timed worker integrates against — in range, so an
	// out-of-range request can't desync the worker's sleep from the programmed registers.
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

// SetROI sets the readout window via the sensor profile and records the geometry so the snap data
// plane knows the frame size. (x, y, w, h) is in binned output pixels at the current Binning:
// 0 ≤ x, x+w ≤ MaxWidth/bin (likewise y/h). An out-of-range request is rejected rather than
// clamped. The sensor profile applies its own per-axis pixel alignment and bin→register
// translation.
func (c *Camera) SetROI(x, y, w, h int) error {
	if c.sensor.SetROI == nil {
		return fmt.Errorf("asicam: %s SetROI not implemented", c.sensor.Name)
	}
	c.mu.Lock()
	bin := c.bin
	c.mu.Unlock()
	if bin < 1 {
		bin = 1
	}
	maxW, maxH := c.sensor.Info.MaxWidth/bin, c.sensor.Info.MaxHeight/bin
	if x < 0 || y < 0 || w < 1 || h < 1 {
		return fmt.Errorf("asicam: ROI (%d,%d %dx%d) invalid: offset must be ≥0 and size ≥1", x, y, w, h)
	}
	if x+w > maxW || y+h > maxH {
		return fmt.Errorf("asicam: ROI (%d,%d %dx%d) exceeds %dx%d at bin %d", x, y, w, h, maxW, maxH, bin)
	}
	// RAW16 binning is done in software: these sensors have no hardware 16-bit binned mode (the
	// bin register tables are 12-bit, for the RAW8/hardware-bin path). Drive a bin-1
	// full-resolution readout of the bin-scaled region, then downsample after the read
	// (GetDataAfterExp → binFrame). RAW8 (bpp 1) keeps the sensor's hardware-bin path; bin 1 is
	// unchanged.
	soft := 1
	sx, sy, sw, sh, sbin := x, y, w, h, bin
	if ModeOf(c.rm).BytesPerPx >= 2 && bin > 1 {
		// Mono averages a full 16-bit frame; color averages same-color samples to keep the Bayer
		// mosaic (binFrame picks the routine). Bayer needs the binned region to cover an even
		// sensor extent on both axes (whole 2×2 mosaic units), i.e. bin must divide the sensor dims
		// to an even product: the 6200 accepts bin 2 (9576×6388) and bin 4 (→1597) but rejects
		// bin 3 (6388/3 is odd). bin·output = the sensor-pixel extent.
		if c.Color() && ((w*bin)%2 != 0 || (h*bin)%2 != 0) {
			return fmt.Errorf("asicam: RAW16 color binning needs an even sensor extent for %s (bin %d → %dx%d sensor px is odd); use a bin that evenly divides the sensor, or RAW8", c.sensor.Name, bin, w*bin, h*bin)
		}
		soft = bin
		sx, sy, sw, sh, sbin = x*bin, y*bin, w*bin, h*bin, 1
	}
	if err := c.sensor.SetROI(c.rm, sx, sy, sw, sh, sbin); err != nil {
		return err
	}
	// Carry the actual readout dims on the live mode so SetExposure's VMAX/HMAX follow the real
	// frame (for software bin, the full bin-scaled size at bin 1); SoftBin tells the read path to
	// downsample.
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) {
			m.Width, m.Height = sw, sh
			m.Bin = sbin
			m.SoftBin = soft
		})
	}
	c.mu.Lock()
	c.roiX, c.roiY, c.roiW, c.roiH = x, y, w, h // client/output ROI (binned pixels)
	c.mu.Unlock()
	return nil
}

// SetBinning selects the symmetric binning factor for subsequent SetROI/captures; bin must be one
// of the sensor's supported Bins. Changing the factor resets the ROI to the full binned frame, so
// call SetBinning before SetROI. The readout-mode switch happens in the next SetROI.
func (c *Camera) SetBinning(bin int) error {
	if bin < 1 {
		return fmt.Errorf("asicam: binning %d invalid (must be ≥1)", bin)
	}
	ok := false
	for _, b := range c.sensor.Info.Bins {
		if b == bin {
			ok = true
			break
		}
	}
	if !ok {
		return fmt.Errorf("asicam: %s does not support bin %d (supported: %v)", c.sensor.Name, bin, c.sensor.Info.Bins)
	}
	c.mu.Lock()
	c.bin = bin
	c.roiX, c.roiY = 0, 0
	c.roiW, c.roiH = c.sensor.Info.MaxWidth/bin, c.sensor.Info.MaxHeight/bin
	c.mu.Unlock()
	// Carry the factor on the readout mode so SetExposure (no bin arg) picks the binned mode's
	// timing base.
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.Bin = bin })
	}
	return nil
}

// --- Alpaca capability getters: current state + sensor-declared ranges. ---

// Gain returns the last gain set (ASI 0.1 dB units).
func (c *Camera) Gain() int { c.mu.Lock(); defer c.mu.Unlock(); return c.gain }

// GainRange returns the supported gain bounds (ASI 0.1 dB units) from the sensor's SetGain clamp
// (Alpaca GainMin/GainMax). max == 0 means the profile declares no range.
func (c *Camera) GainRange() (min, max int) {
	if c.sensor.GainCaps != nil { // vendor-specific range
		return c.sensor.GainCaps(c.rm.VID())
	}
	return c.sensor.GainMin, c.sensor.GainMax
}

// Exposure returns the last exposure set.
func (c *Camera) Exposure() time.Duration { return c.expDuration() }

// ExposureRange returns the supported exposure bounds from the sensor's SetExp
// clamp (Alpaca ExposureMin/ExposureMax). max == 0 means the profile declares no range.
func (c *Camera) ExposureRange() (min, max time.Duration) {
	return time.Duration(c.sensor.ExpMinUs) * time.Microsecond,
		time.Duration(c.sensor.ExpMaxUs) * time.Microsecond
}

// ExposureStep is the exposure resolution (whole µs; Alpaca ExposureResolution).
func (c *Camera) ExposureStep() time.Duration { return time.Microsecond }

// ROI returns the current readout window (x, y, width, height) — Alpaca StartX/StartY/NumX/NumY.
func (c *Camera) ROI() (x, y, w, h int) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.roiX, c.roiY, c.roiW, c.roiH
}

// Bins returns the symmetric binning factors the sensor supports (Alpaca SupportedBins /
// MaxBinX). A factor is usable only if the profile has decoded that readout mode; SetROI errors
// on an undecoded factor.
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
