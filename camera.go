package astrocam

import (
	"context"
	"fmt"
	"math"
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
	// vidStream is the resident stream session a free-run capture reads through (StartVideo then
	// ReadFrame). It exists because a free-run frame cannot be read with one exact-size transfer:
	// the FX3 commits its DMA in whole 1024-byte units, so a frame whose length is not a multiple
	// of 1024 ends mid-unit, and the device fills the rest of that unit from the NEXT frame. A
	// transfer sized to the frame overruns by the difference (kIOReturnOverrun on macOS), and the
	// bytes past the end are real data that the following frame needs. A session reads fixed
	// segments and carries the remainder, so the boundary stays where it belongs. nil when the
	// backend has no session, which leaves ReadFrame on the whole-frame BulkRead.
	vidStream FrameStream
	subtype   int // firmware subtype byte (GetFirmwareVer); gates init branches

	// ST4 pulse state (guide.go), under mu: st4Lines is the bitmask of asserted guide lines
	// (bit = GuideDir), st4Pulses counts host-timed PulseGuide calls in flight. Together they
	// back IsPulseGuiding.
	st4Lines  uint8
	st4Pulses int

	// stalls counts worker readout stalls (short/failed reads that triggered a re-arm) over the
	// camera's lifetime, surfaced by StallCount(). Atomic, not mu-guarded.
	stalls atomic.Int64
	// dropped counts frames lost within the CURRENT capture (POAGetDroppedImagesCount), reset by
	// StartExposure and StartVideo. Distinct from stalls, which is a lifetime total.
	dropped atomic.Int64
	// wbR/wbG/wbB are the last white-balance gains set, under mu.
	wbR, wbG, wbB int
	// defects is the factory hot-pixel map, loaded once at Init and applied to every frame by
	// RepairFrame. nil when the camera has no map or the vendor's flash layout is not decoded,
	// which is not an error: the frame is simply returned uncorrected.
	defects *DefectMap

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
	_, defUSB2, defUSB3 := vend.fpsBounds()
	fpsPercent := defUSB3
	if !usb3 {
		fpsPercent = defUSB2
	}
	mode := ReadoutMode{USB3: usb3, BytesPerPx: bpp, FPSPercent: fpsPercent}
	// The init-time offset is the vendor's advertised default (OffsetCaps) when the profile has
	// one, else the profile's OffsetDef; the same value OffsetRange reports.
	offset := m.Sensor.OffsetDef
	if m.Sensor.OffsetCaps != nil {
		_, _, offset = m.Sensor.OffsetCaps(vid)
	}
	// The full-frame default follows the vendor's geometry: the two read out different areas of
	// the same die, and SizeByVID carries the override Info() applies.
	w, h := m.Sensor.Info.MaxWidth, m.Sensor.Info.MaxHeight
	if wh, ok := m.Sensor.SizeByVID[vid]; ok {
		w, h = wh[0], wh[1]
	}
	c := &Camera{
		t: t, model: m, sensor: m.Sensor, rm: vend.newRegmap(t, m.Sensor.Bus, mode), vend: vend,
		roiW: w, roiH: h, bin: 1,
		offset: offset,
	}
	// A cooled camera gets its hardware Thermal seam up front, so CCDTemperature is readable
	// (and EnableCooling defaults to it) without the caller wiring one.
	if m.Cooled {
		c.thermal = c.HardwareThermal()
	}
	return c, nil
}

// SetFPSPercent sets the frame-rate percentage the readout throttle (HMAX / line time) uses,
// clamped to the vendor's range (FPSPercentRange). It affects the next SetROI/SetExposure;
// 100 = fastest readout the bus allows.
func (c *Camera) SetFPSPercent(pct int) {
	min, _, _ := c.vend.fpsBounds()
	if pct < min {
		pct = min
	}
	if pct > 100 {
		pct = 100
	}
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.FPSPercent = pct })
	}
}

// FPSPercentRange returns the bandwidth-percentage bounds this vendor accepts (the SDK's
// USBBandWidthLimit / BANDWIDTHOVERLOAD range).
func (c *Camera) FPSPercentRange() (min, max int) {
	min, _, _ = c.vend.fpsBounds()
	return min, 100
}

// FPSPercent reports the effective FPS-percent throttle: the user-set value, or the vendor's
// per-link default (ZWO 100 on USB3 and 40 on USB2; PlayerOne 90 on both).
func (c *Camera) FPSPercent() int { return ModeOf(c.rm).FPSPercent }

// SetUSB3 forces the readout mode's link-speed assumption (the bandwidth budget the HMAX/
// line-time math uses): true = USB3 SuperSpeed (bwUSB3), false = USB2 HighSpeed (bwUSB2).
// Normally taken from the link/model. Does not change FPSPercent.
func (c *Camera) SetUSB3(usb3 bool) {
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.USB3 = usb3 })
	}
}

// EGain returns the conversion gain in electrons per ADU at the current gain setting, the value
// the vendor SDK reports as e/ADU. It falls off as base / 10^(gain/200) — 200 gain units per
// decade of voltage gain, which is what makes the unit 0.1 dB. ok is false when the profile
// carries no measured base for the die.
func (c *Camera) EGain() (float64, bool) {
	if c.sensor.EGainBase == 0 {
		return 0, false
	}
	return c.sensor.EGainBase / math.Pow(10, float64(c.Gain())/200), true
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

// Attachment is the identity of the plugging-in this camera's handle was opened on
// (DeviceInfo.Attachment), or 0 when the transport offers none. Enumerate reports the current
// attachment at the same location; a difference means the device was replugged and this
// handle is dead.
func (c *Camera) Attachment() uint64 {
	if a, ok := c.t.(Attached); ok {
		return a.Attachment()
	}
	return 0
}

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
	c.closeVideoStream() // its pump thread references the interface the transport is about to release
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
	if wh, ok := c.sensor.SizeByVID[c.vend.VID]; ok {
		info.MaxWidth, info.MaxHeight = wh[0], wh[1]
	}
	if b, ok := c.sensor.BinsByVID[c.vend.VID]; ok {
		info.Bins = b
	}
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

// Init brings the camera up: FX3 quiesce (stream stop, endpoint reset, and the flush for vendors
// that have one), firmware version read, SPI-flash calibration read, the sensor init table
// (InitDelayReg entries are sleeps), InitFPGA, then the profile's default offset.
func (c *Camera) Init() error {
	if c.sensor.Init == nil {
		return fmt.Errorf("astrocam: %s init sequence not yet decoded", c.sensor.Name)
	}
	c.coolW.invalidate() // the FPGA cooling registers are rewritten from scratch
	// FX3 streaming state persists across host close and open. A session that did not stop cleanly
	// leaves it streaming, and InitFPGA's writes then fail while the GPIF is busy, though the
	// vendor commands still work. Best-effort on the wire, since a fresh device may NAK.
	if !c.vend.Cmds.StreamStop.decoded() || !c.vend.Cmds.StreamStart.decoded() {
		return fmt.Errorf("astrocam: FX3 stream commands not decoded for vendor %s; cannot init %s", c.vend.Name, c.model.Name)
	}
	_ = c.vendorCmd(FX3StreamStop) // 0xAA on ZWO; 0xA0 wValue 0 on PlayerOne
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	// The flush is ZWO's (0xAF). PlayerOne's FX3 has no equivalent command — its SDK recovers the
	// pipeline host-side with a libusb bulk clear/reset — so the endpoint reset above is all the
	// quiesce it gets.
	if c.vend.Cmds.Flush.decoded() {
		_ = c.vendorCmd(FX3Flush)
	}

	// Firmware subtype (GetFirmwareVer's version byte) gates the init branches.
	if fw, err := c.FirmwareVersion(); err == nil {
		c.subtype = int((fw >> 8) & 0xff)
		c.baseFW, c.baseFWok = fw, true // baseline for the firmware-crash (dead) check
	}
	// Before any sensor/FPGA register write. The read brings the FPGA data path into
	// right-aligned RAW16 mode (without it pixels come out MSB-aligned, value<<4).
	c.readCalibrationBlob()
	// The profile's pre-table hook, for a vendor that must reset the sensor before it is
	// programmed rather than after.
	if c.sensor.PreInit != nil {
		if err := c.sensor.PreInit(c.rm); err != nil {
			return fmt.Errorf("init pre: %w", err)
		}
	}
	for _, w := range c.initTable() {
		if w.Reg == InitDelayReg {
			time.Sleep(time.Duration(w.Val) * time.Millisecond)
			continue
		}
		if err := c.rm.WriteReg(w.Reg, w.Val); err != nil {
			return fmt.Errorf("init reg 0x%04x: %w", w.Reg, err)
		}
	}
	// FPGA-side init: the part of camera bringup that is not a Sony sensor register.
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
	// The factory hot-pixel map, read once. Both vendor SDKs correct every frame from it, so the
	// driver does too (RepairDefects). Loaded only for a vendor whose map has been shown to name
	// real defects (Vendor.defectMapTrusted); a camera with no map, an undecoded flash layout or
	// an unreadable blob simply goes uncorrected, which is not an init failure. The map stays
	// readable either way through Camera.LoadDefectMap.
	if info := c.Info(); c.vend.defectMapTrusted && info.MaxWidth > 0 && info.MaxHeight > 0 {
		if dm, derr := c.LoadDefectMap(info.MaxWidth, info.MaxHeight); derr == nil {
			c.defects = dm
		}
	}
	return nil
}

// initTable is the sensor init reglist for this camera's vendor: the profile's per-vendor table
// when it has one, else the default. Each vendor's table is the one its own firmware programs, so
// a body is never brought up with the other's framing.
func (c *Camera) initTable() []RegVal {
	if t, ok := c.sensor.InitByVID[c.vend.VID]; ok {
		return t
	}
	return c.sensor.Init
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
	// The sensor's mode block is indexed by sensor mode AND sample size together, so a depth
	// change has to rewrite it for the same reason a mode change does. Without this the block
	// keeps whatever depth Init programmed and the sensor emits samples of one width into a
	// frame laid out for the other.
	//
	// Gated on the vendor advertising modes, not merely on the profile having the hook: a shared
	// die declares one SetSensorMode that refuses for the vendor it has not decoded, and calling
	// it unconditionally would make SetOutputDepth fail on that vendor's body for no reason.
	if c.sensor.SetSensorMode != nil && len(c.SensorModes()) > 0 {
		if err := c.sensor.SetSensorMode(c.rm, ModeOf(c.rm).SensorMode); err != nil {
			if r, ok := c.rm.(modeCarrier); ok {
				r.updateMode(func(m *ReadoutMode) { m.BytesPerPx = prevBpp })
			}
			_ = SetFPGAOutputWidth(c.rm, prevBpp >= 2)
			return err
		}
	}
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

// SensorModes lists the readout programmes this body offers, index 0 being the normal mode
// (POAGetSensorModeCount / POAGetSensorModeInfo). A nil or single-entry result means the camera
// has no mode selection, which is what the SDK reports as a mode count of 0.
func (c *Camera) SensorModes() []SensorModeInfo {
	if c.sensor == nil || c.sensor.SensorModes == nil {
		return nil
	}
	if m := c.sensor.SensorModes(c.rm.VID()); len(m) > 1 {
		return m
	}
	return nil
}

// SensorMode returns the current readout programme's index (POAGetSensorMode).
func (c *Camera) SensorMode() int { return ModeOf(c.rm).SensorMode }

// SetSensorMode selects a readout programme by index (POASetSensorMode). A mode is a re-tuning
// of the same register block, so it takes two halves: the sensor block the profile writes here,
// and the geometry, which is re-programmed through the stored window because the frame length,
// the crop origin and the frame period all move with the mode. Stop the exposure first, as the
// SDK requires.
//
// A mode the profile has not decoded at the current output depth is refused rather than
// approximated: the mode block is indexed by mode and sample size jointly, and reusing the wrong
// cell leaves the sensor emitting data the frame layout does not describe.
func (c *Camera) SetSensorMode(mode int) error {
	modes := c.SensorModes()
	if len(modes) == 0 {
		return fmt.Errorf("astrocam: %s has no sensor-mode selection", c.model.Name)
	}
	if mode < 0 || mode >= len(modes) {
		return fmt.Errorf("astrocam: sensor mode %d out of range (0..%d)", mode, len(modes)-1)
	}
	if c.sensor.SetSensorMode == nil {
		return fmt.Errorf("astrocam: %s declares sensor modes but cannot program them", c.sensor.Name)
	}
	prevMode := ModeOf(c.rm).SensorMode
	if mode == prevMode {
		return nil
	}
	// The mode has to be visible on the ReadoutMode before the profile runs: both halves read it
	// from there, so that the sensor block and the geometry cannot disagree about which mode is
	// being programmed.
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.SensorMode = mode })
	}
	c.mu.Lock()
	prev := c.snapshotWindow()
	c.mu.Unlock()
	restore := func(err error) error {
		if r, ok := c.rm.(modeCarrier); ok {
			r.updateMode(func(m *ReadoutMode) { m.SensorMode = prevMode })
		}
		c.restoreWindow(prev)
		return err
	}
	if err := c.sensor.SetSensorMode(c.rm, mode); err != nil {
		return restore(err)
	}
	if err := c.reprogramWindow(); err != nil {
		return restore(err)
	}
	// Gain and offset are mode-dependent on at least one profile — the IMX585's fine-gain curve
	// and its HDR offset register both read the mode — so a mode change has to re-apply them, or
	// the camera keeps values computed for the mode it just left.
	c.mu.Lock()
	gain, offset := c.gain, c.offset
	c.mu.Unlock()
	if c.sensor.SetGain != nil {
		if err := c.SetGain(gain); err != nil {
			return restore(err)
		}
	}
	if c.sensor.SetOffset != nil {
		if err := c.SetOffset(offset); err != nil {
			return restore(err)
		}
	}
	return nil
}

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
	lo, hi := c.ExposureRange()
	if lo > 0 && d < lo {
		d = lo
	}
	if hi > 0 && d > hi {
		d = hi
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
// (likewise y/h), and the sensor-side window must meet the vendor's granularity (Vendor.roiStep:
// width a multiple of 8 and an even height on ZWO, width a multiple of 4 on PlayerOne), because
// the FX3 frames whole 32-bit words. An out-of-range or misaligned size is rejected, not clamped.
// The start is aligned down to the profile's window-start grid (Sensor.ROIStartAlign) and ROI
// reports the aligned start.
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
	// Bound against the geometry THIS vendor reads out, not the profile's default: the two expose
	// different areas of the same die, so a shared bound rejects one vendor's own full frame.
	info := c.Info()
	maxW, maxH := info.MaxWidth/bin, info.MaxHeight/bin
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
	stepW, stepH := c.vend.roiStep()
	if hw > 1 {
		if w%stepW != 0 || h%stepH != 0 {
			return fmt.Errorf("astrocam: ROI %dx%d at sensor bin %d: width must be a multiple of %d and height a multiple of %d", w, h, hw, stepW, stepH)
		}
	} else if (w*bin)%stepW != 0 || (h*bin)%stepH != 0 {
		return fmt.Errorf("astrocam: ROI %dx%d at bin %d: the sensor window %dx%d must have a width that is a multiple of %d and a height that is a multiple of %d", w, h, bin, w*bin, h*bin, stepW, stepH)
	}
	// Align the sensor-pixel start down to the profile's grid, on a step the bin also divides, so
	// the reported binned start is exact.
	if c.sensor.ROIStartAlign != nil {
		ax, ay := c.sensor.ROIStartAlign(hw)
		x = alignStart(x, bin, ax)
		y = alignStart(y, bin, ay)
	}
	sx, sy, sw, sh, sbin := x*soft, y*soft, w*soft, h*soft, hw
	senBin := c.sensorSplit(bin)
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
			m.SensorBin = senBin
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

// ClampROI returns the largest window at or below the requested one that SetROI accepts at the
// current binning and hardware-bin split, with the start already aligned to the profile's grid.
// It never grows a window past what was asked for, and never returns one the frame cannot hold.
//
// A caller cannot compute this itself. The granularity is vendor policy — ZWO wants the width a
// multiple of 8, PlayerOne a multiple of 4 — and whether the rule counts output or sensor pixels
// depends on where the binning happens, which is a property of the camera rather than the
// request. Dividing the sensor extent by the bin factor lands wherever the arithmetic falls: on a
// 3856x2180 IMX585 that is an odd 1285 columns at bin 3 and an odd 545 rows at bin 4, both of
// which SetROI refuses.
//
// SetROI still refuses a misaligned window rather than quietly capturing a different one, so a
// caller that means to be exact keeps that guarantee. This is the seam for a front end whose
// protocol expects the driver to adapt instead: clamp, program, then report what was programmed.
func (c *Camera) ClampROI(x, y, w, h int) (int, int, int, int) {
	c.mu.Lock()
	bin, hwBin := c.bin, c.hwBin
	c.mu.Unlock()
	if bin < 1 {
		bin = 1
	}
	info := c.Info()
	maxW, maxH := info.MaxWidth/bin, info.MaxHeight/bin
	stepW, stepH := c.roiStep(bin, hwBin)
	// The origin comes first, since it decides how much room is left for the window. Aligning it
	// only ever moves it down, so it cannot push the far edge past the frame.
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	if c.sensor.ROIStartAlign != nil {
		hw, _ := c.binSplit(bin, hwBin)
		ax, ay := c.sensor.ROIStartAlign(hw)
		x, y = alignStart(x, bin, ax), alignStart(y, bin, ay)
	}
	if x >= maxW {
		x = 0
	}
	if y >= maxH {
		y = 0
	}
	// Fit the size inside the frame, then floor it to the granularity.
	if w > maxW-x {
		w = maxW - x
	}
	if h > maxH-y {
		h = maxH - y
	}
	w, h = w-w%stepW, h-h%stepH
	// A request below one step still gets one step, where the frame has room: a zero-sized window
	// is not a readout the camera can perform.
	if w < stepW && stepW <= maxW-x {
		w = stepW
	}
	if h < stepH && stepH <= maxH-y {
		h = stepH
	}
	return x, y, w, h
}

// roiStep is the window granularity SetROI enforces, expressed in the OUTPUT pixels a caller
// passes. The vendor's own rule counts SENSOR pixels, so it divides through by the bin factor
// wherever the host or the FPGA does the reduction and the sensor still reads the full region;
// where the sensor itself bins, the rule already counts output pixels and carries over as it is.
// A colour body that host-bins by an odd factor needs an even extent on top of that, or the
// same-colour block the bin averages straddles the Bayer phase.
func (c *Camera) roiStep(bin int, hwBin bool) (w, h int) {
	w, h = c.vend.roiStep()
	hw, soft := c.binSplit(bin, hwBin)
	if hw <= 1 {
		w, h = w/gcd(w, bin), h/gcd(h, bin)
	}
	if soft > 1 && c.Color() && soft%2 != 0 {
		w, h = evenMultiple(w), evenMultiple(h)
	}
	if w < 1 {
		w = 1
	}
	if h < 1 {
		h = 1
	}
	return w, h
}

// evenMultiple is the smallest multiple of v that is itself even.
func evenMultiple(v int) int {
	if v%2 == 0 {
		return v
	}
	return v * 2
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
	// Through Info(), so a vendor override (BinsByVID) is honoured: the two makers do not offer
	// the same binning on the same die.
	bins := c.Info().Bins
	ok := false
	for _, b := range bins {
		if b == bin {
			ok = true
			break
		}
	}
	if !ok {
		return fmt.Errorf("astrocam: %s does not support bin %d (supported: %v)", c.sensor.Name, bin, bins)
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
	// A vendor whose device bins at every factor leaves nothing for the host: the whole factor is
	// the device's, and the frame arrives already reduced. Which part of the device does it —
	// the sensor die or the FPGA — is sensorSplit's business, not this one's.
	if c.vend.deviceBins {
		return bin, 1
	}
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
	// The vendor's geometry, not the profile's default: SizeByVID means the two read out
	// different areas, and a full frame at bin N must cover the area this camera actually has.
	info := c.Info()
	w, h = info.MaxWidth/bin, info.MaxHeight/bin
	stepW, stepH := c.vend.roiStep()
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
	lo, hi := c.sensor.ExpMinUs, c.sensor.ExpMaxUs
	if c.sensor.ExpCaps != nil { // vendor-specific range over a shared die
		lo, hi = c.sensor.ExpCaps(c.rm.VID())
	}
	return time.Duration(lo) * time.Microsecond, time.Duration(hi) * time.Microsecond
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
//
// It goes through Info so the VENDOR's factors are reported (BinsByVID), not the profile's
// static default: the two makers of one die need not offer the same binning.
func (c *Camera) Bins() []int { return c.Info().Bins }

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

// SetBinSum selects whether binned pixels are summed or averaged (POA_PIXEL_BIN_SUM). false, the
// default and the SDK's, averages. Summing preserves the electron count and uses the full sample
// container — bin 2 of 12-bit data occupies 14 bits, bin 4 occupies 16 — which keeps faint signal
// clear of the quantisation floor, at the cost of saturating N² sooner. It takes effect on the
// next SetROI; a programmed window is re-programmed here.
func (c *Camera) SetBinSum(sum bool) error {
	prev := ModeOf(c.rm).BinSum
	if prev == sum {
		return nil
	}
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.BinSum = sum })
	}
	if err := c.reprogramWindow(); err != nil {
		if r, ok := c.rm.(modeCarrier); ok {
			r.updateMode(func(m *ReadoutMode) { m.BinSum = prev })
		}
		return err
	}
	return nil
}

// BinSum reports whether binned pixels are summed rather than averaged.
func (c *Camera) BinSum() bool { return ModeOf(c.rm).BinSum }

// SetFrameRateLimit caps the frame rate in frames per second (POA_FRAME_LIMIT); 0 removes the cap.
// It is independent of SetFPSPercent: the percentage scales the link budget, this bounds the frame
// period outright, and the sensor takes whichever of the two and the exposure is longest. It takes
// effect on the next SetExposure, which is where the frame period is programmed.
func (c *Camera) SetFrameRateLimit(fps int) {
	if fps < 0 {
		fps = 0
	}
	if fps > frameLimitMax {
		fps = frameLimitMax
	}
	if r, ok := c.rm.(modeCarrier); ok {
		r.updateMode(func(m *ReadoutMode) { m.FrameLimit = fps })
	}
}

// FrameRateLimit returns the frame-rate cap in fps (0 = none).
func (c *Camera) FrameRateLimit() int { return ModeOf(c.rm).FrameLimit }

// frameLimitMax is the ceiling POA_FRAME_LIMIT advertises.
const frameLimitMax = 2000

// sensorSplit reports how much of a device-side binning factor the SENSOR DIE takes, for a vendor
// that bins in the camera (Vendor.deviceBins). It is the largest factor in the profile's HWBins
// that divides bin, or 1 when hardware binning is off or the profile has no die-side mode for it.
// The rest of the factor stays with the FPGA. On the IMX585 the die bins by 2 only, so bin 2 goes
// wholly to the die, bin 4 splits 2 and 2, and bin 3 has no die mode and falls back entirely —
// which is what the vendor SDK does too.
func (c *Camera) sensorSplit(bin int) int {
	c.mu.Lock()
	on := c.hwBin
	c.mu.Unlock()
	if !on || !c.vend.deviceBins || bin < 2 {
		return 1
	}
	sen := 1
	for _, b := range c.sensor.HWBins {
		if b > sen && b <= bin && bin%b == 0 {
			sen = b
		}
	}
	return sen
}

// GainOffsetPresets returns the vendor's preset gain/offset operating points for this camera
// (POAGetGainsAndOffsets): the gain for the most dynamic range, the one where high conversion
// gain engages, unity gain, and the maximum analog gain, each with the offset that belongs with
// it. ok is false when the profile has not decoded them for this vendor.
func (c *Camera) GainOffsetPresets() (GainOffsetPresets, bool) {
	if c.sensor.Presets == nil {
		return GainOffsetPresets{}, false
	}
	return c.sensor.Presets(c.rm.VID())
}

// ImageFormats lists the pixel layouts this camera can deliver (POACameraProperties.imgFormats).
// It is RAW8 and RAW16 for every supported body: the vendor SDKs also offer RGB24 and MONO8, but
// those are host-side debayer conversions of a raw frame rather than readout modes, and this
// driver returns the raw frame for the caller to convert.
func (c *Camera) ImageFormats() []ImageFormat {
	return []ImageFormat{FormatRAW8, FormatRAW16}
}

// SetWhiteBalance sets the per-channel white-balance gains (POA_WB_R/G/B) on a colour camera.
// Each channel is bounded by the vendor's limit and 0 is unity. A mono body has no CFA to balance
// and is refused, as is a vendor whose registers are not decoded.
func (c *Camera) SetWhiteBalance(r, g, b int) error {
	if c.vend.setWhiteBalance == nil || c.vend.wbLimit == 0 {
		return fmt.Errorf("astrocam: white balance not decoded for vendor %s", c.vend.Name)
	}
	if !c.Color() {
		return fmt.Errorf("astrocam: %s is monochrome; there is no white balance to set", c.model.Name)
	}
	lim := c.vend.wbLimit
	clamp := func(v int) int {
		if v < -lim {
			return -lim
		}
		if v > lim {
			return lim
		}
		return v
	}
	r, g, b = clamp(r), clamp(g), clamp(b)
	if err := c.vend.setWhiteBalance(c.rm, r, g, b); err != nil {
		return err
	}
	c.mu.Lock()
	c.wbR, c.wbG, c.wbB = r, g, b
	c.mu.Unlock()
	return nil
}

// WhiteBalance returns the last per-channel gains set (0, 0, 0 = unity).
func (c *Camera) WhiteBalance() (r, g, b int) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.wbR, c.wbG, c.wbB
}

// WhiteBalanceRange returns the per-channel bound this vendor accepts; ok is false when white
// balance is unsupported on this camera.
func (c *Camera) WhiteBalanceRange() (limit int, ok bool) {
	if c.vend.setWhiteBalance == nil || c.vend.wbLimit == 0 || !c.Color() {
		return 0, false
	}
	return c.vend.wbLimit, true
}

// DroppedFrames returns the number of frames lost since the current capture started: reads that
// came back short or failed while the sensor was free-running, which is a frame the camera
// produced and the host did not collect (POAGetDroppedImagesCount). StartExposure and StartVideo
// reset it.
//
// This is a different quantity from StallCount, which counts readout stalls over the camera's
// whole lifetime and never resets.
func (c *Camera) DroppedFrames() int64 { return c.dropped.Load() }

// noteDropped records one lost frame in the current capture.
func (c *Camera) noteDropped() { c.dropped.Add(1) }
