package asicam

import (
	"context"
	"fmt"
	"sync"
	"time"
)

// modeCarrier is implemented by a vendor Regmap that holds the live ReadoutMode the
// Camera updates (FPS%, output depth, bin). The Camera mutates it through this interface
// rather than asserting a concrete regmap type, so every vendor dialect carries the
// updates (both zwoRegmap and poaRegmap implement liveMode).
type modeCarrier interface{ liveMode() *ReadoutMode }

// superSpeedReporter is the optional Transport capability that reports the NEGOTIATED USB
// link speed (USB3 SuperSpeed vs USB2 HighSpeed). The readout mode follows the live link,
// not the model's static capability flag: a USB3-capable camera plugged into a USB2-only
// port reads at USB2 and must use the USB2 bandwidth/FPS budget — applying the model's
// USB3 budget there desyncs the FX3 framing and garbles the frame. Transports that can't
// report speed are not asserted, leaving the model default in force.
type superSpeedReporter interface{ SuperSpeed() bool }

// Camera is an opened ASI camera: a Transport bound to a Model and its Sensor.
type Camera struct {
	t      Transport
	model  Model
	sensor *Sensor
	rm     Regmap
	pid    uint16

	// mu guards the mutable capture-state scalars below. The Alpaca server calls into
	// one Camera from several goroutines at once (an ImageReady poll while a frame is
	// in flight, an AbortExposure arriving mid-capture), so every read/write of these
	// fields is locked. The lock is held only across the scalar transitions, NEVER
	// across USB I/O (arm/read/stop run unlocked) so a long exposure can't block a
	// status poll. USB safety itself is the transport's ctrlMu, a separate concern.
	mu sync.Mutex
	// Capture state (the snap data plane; see capture.go). roiW/roiH default to
	// the full sensor and track SetROI so the frame size is known; expDur is the
	// last SetExposure, used by the host-timed status poll.
	roiX, roiY int
	roiW, roiH int // in BINNED output pixels (FrameBytes = roiW·roiH·bpp)
	bin        int // symmetric binning factor (1 = full res); 0 normalized to 1
	gain       int // last SetGain (ASI 0.1 dB units), surfaced by Gain()
	offset     int // last SetOffset (ASI Brightness / black level), surfaced by Offset()
	status     ExposureStatus
	expDur     time.Duration
	expStart   time.Time
	subtype    int // firmware subtype byte (GetFirmwareVer); gates init branches

	// TEC cooling (cooling.go). cooler is non-nil while the regulation loop runs;
	// coolCancel/coolWg own that goroutine's lifetime, and
	// thermal is the hardware seam it drives so CCDTemperature reads work with the
	// cooler off too. The cooler pointer/cancel/thermal are guarded by mu (held only to
	// swap them, never across the cooler's own I/O — the Cooler is internally locked).
	cooler     *Cooler
	coolCancel context.CancelFunc
	coolWg     sync.WaitGroup
	thermal    Thermal
}

// Open binds an already-open Transport to the camera identified by pid (so the
// right Sensor profile and per-model flags are selected). The Transport is the
// vendor/libusb part; everything above it is pure Go.
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
	// ACTUAL negotiated USB link (the transport reports it; falls back to the model's
	// capability if it can't), output depth from the sensor's ADC width (RAW16 for >8-bit),
	// and the ASI bandwidth-overload as FPSPercent. Override per session with SetUSB3 /
	// SetFPSPercent.
	bpp := 1
	if m.Sensor.Info.BitDepth > 8 {
		bpp = 2
	}
	usb3 := m.USB3 // model capability; refined to the live link if the transport reports it
	if sr, ok := t.(superSpeedReporter); ok {
		usb3 = sr.SuperSpeed()
	}
	fps := 40 // USB2 HighSpeed default (bandwidth-overload)
	if usb3 {
		fps = 100
	}
	mode := ReadoutMode{USB3: usb3, BytesPerPx: bpp, FPSPercent: fps}
	c := &Camera{
		t: t, model: m, sensor: m.Sensor, rm: vend.newRegmap(t, m.Sensor.Bus, mode), pid: pid,
		roiW: m.Sensor.Info.MaxWidth, roiH: m.Sensor.Info.MaxHeight, bin: 1,
		offset: m.Sensor.OffsetDef,
	}
	// A cooled camera gets its hardware Thermal seam up front, so CCDTemperature is
	// readable (and EnableCooling defaults to it) without the caller wiring one.
	if m.Cooled {
		c.thermal = c.HardwareThermal()
	}
	return c, nil
}

// SetFPSPercent sets the requested frame-rate percentage (40..100) that the readout
// throttle (HMAX / line time) uses, mirroring the SDK's FPS-percent input. It affects
// the next SetROI/SetExposure. 100 = fastest readout the bus allows (least throttling).
func (c *Camera) SetFPSPercent(pct int) {
	if r, ok := c.rm.(modeCarrier); ok {
		r.liveMode().FPSPercent = pct
	}
}

// SetUSB3 forces the readout mode's link-speed assumption (the bandwidth budget and the
// default FPS-overload the HMAX/line-time math uses): true = USB3 SuperSpeed (bwUSB3,
// 100%), false = USB2 HighSpeed (bwUSB2, 40%). The driver normally takes this from the
// model, but forcing it lets you exercise the USB2 vs USB3 readout path on a fixed
// physical link (e.g. while the camera is bridged through an analyzer) without replugging.
// It resets FPSPercent to the forced speed's default; call SetFPSPercent after to override.
func (c *Camera) SetUSB3(usb3 bool) {
	if r, ok := c.rm.(modeCarrier); ok {
		m := r.liveMode()
		m.USB3 = usb3
		if usb3 {
			m.FPSPercent = 100
		} else {
			m.FPSPercent = 40
		}
	}
}

func (c *Camera) Name() string    { return c.model.Name }
func (c *Camera) Sensor() *Sensor { return c.sensor }
func (c *Camera) Cooled() bool    { return c.model.Cooled }
func (c *Camera) Color() bool     { return c.model.Color }
func (c *Camera) ST4() bool       { return c.model.ST4 } // has an ST4 guide port (CanPulseGuide)

// Close stops the TEC regulation goroutine (joining it cleanly so no control transfer
// outlives the camera) and then closes the transport. Safe to call once.
func (c *Camera) Close() error {
	c.DisableCooling()
	return c.t.Close()
}

// EnableCooling starts host-side TEC regulation toward target °C, driving the given
// Thermal seam on a background goroutine the Camera owns and joins on Close — the
// in-process equivalent of the SDK's cooling thread. It errors on a model with no cooler.
// The Thermal is injected (tests pass a simulated plant; a real camera passes its
// control-transfer Thermal), and is remembered so CCDTemperature can be read with the
// cooler off. Calling it again retargets the running loop instead of starting a second.
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
	}
	c.mu.Unlock()
	cl.SetTarget(target) // reads temp (I/O) — done outside c.mu; the Cooler is self-locked
	return nil
}

// DisableCooling stops the regulation goroutine and joins it. The TEC is left as-is (it
// does not force a warm-up); the camera keeps its thermal seam so temperature is still
// readable. Idempotent.
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

// SeedCoolerPower warm-starts the running cooler at pct % drive instead of ramping from 0
// (see Cooler.SeedPower) — used to jump-start a deep cooldown or restore drive on reconnect.
// No-op if cooling is off.
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
// CCDTemperature), independent of whether the cooler is running. Errors if no thermal
// backend has been attached (EnableCooling attaches one).
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

// Info returns the camera geometry/capabilities (sensor facts + model flags).
// The sensor die is mono; a color (MC) model surfaces the die's CFA pattern
// (Sensor.Info.Bayer, e.g. "RGGB"), while a mono (MM) model reports no Bayer —
// the one register profile serves both, color being a Model flag, not silicon.
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

// InitDelayReg is the sentinel register in a sensor init table: the SDK's table
// walker treats reg==0xffff not as a register write but as a delay, of Val
// milliseconds (the SDK scales the table value by 1000 into microseconds).
const InitDelayReg uint16 = 0xffff

// Init replays the sensor's init register sequence (the InitCamera table + the
// explicit tail), honoring InitDelayReg entries as sleeps.
func (c *Camera) Init() error {
	if c.sensor.Init == nil {
		return fmt.Errorf("asicam: %s init sequence not yet transcribed", c.sensor.Name)
	}
	// A camera persists its FX3 streaming state across host close/open. A prior session
	// that didn't stop cleanly (a timed-out read, a killed process) can leave it
	// streaming, and FPGA register writes in InitFPGA then fail while the GPIF is busy —
	// the observed intermittent total-failure mode. Quiesce it first with FX3 vendor
	// commands (stop, clear pipe, flush): these work even when the FPGA path is blocked,
	// unlike a register write. Best-effort; errors here are expected on a fresh device.
	_ = c.t.ControlOut(cmdStreamStop, 0, 0, nil) // 0xAA: stop any leftover async stream
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	_ = c.t.ControlOut(cmdFlush, 0, 0, nil) // 0xAF: flush the pipeline

	// Firmware subtype (GetFirmwareVer's version byte) gates the init branches.
	if fw, err := c.FirmwareVersion(); err == nil {
		c.subtype = int((fw >> 8) & 0xff)
	}
	// Init-time SPI-flash calibration read, replicating the SDK's first substantive
	// init step (pcap-confirmed order): disable the GPIF data bus, read the ~10 KB
	// per-unit config blob from flash, then re-enable the bus — all BEFORE any
	// sensor/FPGA register write. The blob bytes are discarded; the read is what the
	// firmware needs to bring the FPGA data path into the SDK's RIGHT-ALIGNED output
	// mode (without it gosnap's RAW16 pixels come out MSB-aligned, i.e. value<<4).
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
	// FPGA-side init (brings up the readout pipeline) — the part of
	// InitCamera that isn't Sony registers. The GPIF data bus was already
	// enabled by readCalibrationBlob above (the SDK enables it once, early, and never
	// toggles it again — no 0xAE bank-select, no late re-enable).
	if c.sensor.InitFPGA != nil {
		if err := c.sensor.InitFPGA(c.rm, c.subtype); err != nil {
			return fmt.Errorf("init fpga: %w", err)
		}
	}
	// Apply the profile's default offset / black level, mirroring the SDK's ASIInitCamera
	// (which programs its default Brightness at init — 30 for the 462). Without this the
	// sensor keeps its power-on pedestal and gosnap's default black level sits below the
	// SDK's. An explicit SetOffset later (gosnap -offset N) overrides this.
	if c.sensor.SetOffset != nil {
		if err := c.sensor.SetOffset(c.rm, c.offset); err != nil {
			return fmt.Errorf("init offset: %w", err)
		}
	}
	return nil
}

// readCalibrationBlob replays the SDK's init-time SPI-flash calibration read
// (opcode 0xC3). With the GPIF data bus disabled (0xBE=0) it reads the
// per-unit config blob — ~10 KB from flash, wIndex in 256-byte pages starting at
// 0x400, 2 KB (8 pages) per chunk plus a 256 B tail — then re-enables the bus
// (0xBE=1). Pcap-confirmed shape: 0xBE=0 → 6×0xC3 → 0xBE=1. The data is discarded;
// the read is the step that brings the FPGA data path into the SDK's right-aligned
// RAW16 mode. Best-effort: the bytes don't matter and a fresh/odd device may NAK.
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
// output width (SetFPGAADCWidthOutputWidth). It carries the depth on the readout mode, so the
// next SetROI/SetExposure/FrameBytes follow it. Call before arming a capture (not mid-stream).
func (c *Camera) SetOutputDepth(bpp int) error {
	if bpp != 1 && bpp != 2 {
		return fmt.Errorf("asicam: output depth %d invalid (1 = RAW8, 2 = RAW16)", bpp)
	}
	if err := SetFPGAOutputWidth(c.rm, bpp >= 2); err != nil {
		return err
	}
	if r, ok := c.rm.(modeCarrier); ok {
		r.liveMode().BytesPerPx = bpp
	}
	return nil
}

// OutputDepth returns the current output bytes-per-pixel (1 = RAW8, 2 = RAW16).
func (c *Camera) OutputDepth() int { return ModeOf(c.rm).BytesPerPx }

// SetHighSpeedMode selects the sensor's 10-bit high-speed readout (the SDK's
// ASI_HIGH_SPEED_MODE / the high-speed flag) — ~2× the frame rate by trading 2 bits of depth.
// It only takes effect for RAW8 output (the 462 keeps 12-bit for RAW16), so the caller
// should also SetOutputDepth(1). Carried on the live mode; the next SetROI/SetExposure
// reprogram the sensor format, pixel clock, and HMAX floor accordingly. Call before arming.
func (c *Camera) SetHighSpeedMode(on bool) {
	if r, ok := c.rm.(modeCarrier); ok {
		r.liveMode().HighSpeed = on
	}
}

// HighSpeedMode reports whether the 10-bit high-speed readout is selected.
func (c *Camera) HighSpeedMode() bool { return ModeOf(c.rm).HighSpeed }

// OffsetRange returns the supported offset bounds + default (Alpaca OffsetMin/OffsetMax). The
// third return is the sensor's default. ok is false if the profile has no offset control.
func (c *Camera) OffsetRange() (min, max, def int, ok bool) {
	if c.sensor.SetOffset == nil {
		return 0, 0, 0, false
	}
	if c.sensor.OffsetCaps != nil { // vendor-specific range (the dual of the dispatched SetOffset)
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
	if err := c.sensor.SetExposure(c.rm, d); err != nil {
		return err
	}
	c.mu.Lock()
	c.expDur = d
	c.mu.Unlock()
	return nil
}

// SetROI sets the readout window via the sensor profile and records the geometry so the snap
// data plane knows the frame size. (x, y, w, h) is in BINNED output pixels at the current
// Binning: 0 ≤ x, x+w ≤ MaxWidth/bin (and likewise y/h). The window must fit the binned
// sensor — an out-of-range request is rejected rather than silently clamped, so a frame size
// mismatch (FrameBytes vs the FX3 transfer) can't slip through. The sensor profile applies
// its own per-axis pixel alignment and bin→register translation.
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
	// RAW16 binning is done in SOFTWARE: these sensors have no hardware 16-bit binned mode
	// (the bin register tables are 12-bit, for the RAW8/hardware-bin path). The SDK reads the
	// FULL 16-bit frame and averages bin×bin on the host (GetImage →
	// MonoBin). Mirror that exactly: drive a bin-1 full-resolution readout of the
	// bin·-scaled region, then downsample after the read (GetDataAfterExp → binFrame). RAW8
	// (bpp 1) keeps the sensor's hardware-bin path. bin 1 is unchanged.
	soft := 1
	sx, sy, sw, sh, sbin := x, y, w, h, bin
	if ModeOf(c.rm).BytesPerPx >= 2 && bin > 1 {
		// Mono averages a full 16-bit frame (GetImage → MonoBin); color averages SAME-COLOR
		// samples to keep the Bayer mosaic (GetImage → ColorRAWBin). Both read the full frame and
		// bin on the host (binFrame picks the routine). Bayer needs the binned region to cover an
		// EVEN sensor extent on both axes (whole 2×2 mosaic units) — i.e. bin must divide the
		// sensor dims to an even product. This mirrors the SDK exactly: on the 6200 it accepts
		// bin 2 (9576×6388) and bin 4 (4 divides 6388 → 1597) but REJECTS bin 3 (6388/3 leaves an
		// odd extent → ASISetROIFormat rc=8). bin·output = the sensor-pixel extent.
		if c.Color() && ((w*bin)%2 != 0 || (h*bin)%2 != 0) {
			return fmt.Errorf("asicam: RAW16 color binning needs an even sensor extent for %s (bin %d → %dx%d sensor px is odd); use a bin that evenly divides the sensor, or RAW8", c.sensor.Name, bin, w*bin, h*bin)
		}
		soft = bin
		sx, sy, sw, sh, sbin = x*bin, y*bin, w*bin, h*bin, 1
	}
	if err := c.sensor.SetROI(c.rm, sx, sy, sw, sh, sbin); err != nil {
		return err
	}
	// Carry the ACTUAL readout dims on the live mode so SetExposure's VMAX/HMAX follow the
	// real frame (for software bin that's the full bin·-scaled size at bin 1); SoftBin tells
	// the read path to downsample. A sub-frame streams at a higher free-run fps (the SDK's
	// per-ROI frame-rate scaling).
	if r, ok := c.rm.(modeCarrier); ok {
		m := r.liveMode()
		m.Width, m.Height = sw, sh
		m.Bin = sbin
		m.SoftBin = soft
	}
	c.mu.Lock()
	c.roiX, c.roiY, c.roiW, c.roiH = x, y, w, h // client/output ROI (binned pixels)
	c.mu.Unlock()
	return nil
}

// SetBinning selects the symmetric binning factor for subsequent SetROI/captures. bin must
// be one of the sensor's supported Bins. Changing the factor resets the ROI to the full
// binned frame (the previous window was in the old factor's pixel units), so a caller should
// SetBinning before SetROI. The actual readout-mode switch happens in the next SetROI.
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
	// Carry the factor on the readout mode so SetExposure (which has no bin arg) picks the
	// binned mode's timing base V — same seam as SetFPSPercent/BytesPerPx.
	if r, ok := c.rm.(modeCarrier); ok {
		r.liveMode().Bin = bin
	}
	return nil
}

// --- Alpaca capability getters: current state + the sensor-declared ranges, so the
// goalpaca driver can answer ICameraV3 Gain/Exposure/ROI/Bin queries from public Camera
// methods without reaching into the Sensor profile. ---

// Gain returns the last gain set (ASI 0.1 dB units).
func (c *Camera) Gain() int { c.mu.Lock(); defer c.mu.Unlock(); return c.gain }

// GainRange returns the supported gain bounds (ASI 0.1 dB units) from the sensor's
// SetGain clamp (Alpaca GainMin/GainMax). max == 0 means the profile declares no range.
func (c *Camera) GainRange() (min, max int) {
	if c.sensor.GainCaps != nil { // vendor-specific range (the dual of the dispatched SetGain)
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

// ExposureStep is the exposure resolution: the SDK programs exposure in whole µs
// (Alpaca ExposureResolution).
func (c *Camera) ExposureStep() time.Duration { return time.Microsecond }

// ROI returns the current readout window (x, y, width, height) — Alpaca StartX/StartY/
// NumX/NumY.
func (c *Camera) ROI() (x, y, w, h int) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.roiX, c.roiY, c.roiW, c.roiH
}

// Bins returns the symmetric binning factors the sensor supports (Alpaca SupportedBins /
// MaxBinX). A factor here is selectable via SetBinning only if the profile has decoded that
// readout mode; profiles return an error from SetROI for an undecoded factor.
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
