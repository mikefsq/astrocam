package astrocam

import "time"

// RegVal is one entry of an init register table (cf. the Linux kernel's
// struct regval). Sensor profiles are mostly these.
type RegVal struct{ Reg, Val uint16 }

// CameraInfo is the sensor-intrinsic geometry/capability. Per-model facts
// (color, cooler) are layered on by the Model, not the sensor.
type CameraInfo struct {
	MaxWidth  int
	MaxHeight int
	PixelUm   float64
	BitDepth  int // ADC depth (e.g. 12)
	// Bayer is the CFA pattern of the die's COLOR variant (e.g. "RGGB"). The bare
	// die is mono and the registers are color-agnostic, so one profile serves both
	// the MM and MC models; Camera.Info surfaces this pattern only when Model.Color
	// is set, and reports "" for a mono model. Leave "" for dies with no color variant.
	Bayer string
	Bins  []int // supported symmetric binning factors
}

// Sensor is a per-chip profile, structured like a Linux V4L2 sensor driver: an
// init register table plus a few ops over a Regmap. The ops carry the sensor's
// register knowledge (sourceable from the ZWO SDK, a kernel imx*.c, a datasheet,
// or a USB capture — the Camera lib never knows which).
//
// Gain is in ASI units (0.1 dB steps). Exposure ops receive a duration.
type Sensor struct {
	Name string
	Info CameraInfo

	// GainMin/GainMax bound the ASI gain (0.1 dB units) that SetGain accepts — the
	// decoded SetGain clamp, surfaced by Camera.GainRange for the Alpaca Gain caps.
	// GainMax 0 means the profile has not declared a range yet.
	GainMin, GainMax int
	// OffsetMin/OffsetMax/OffsetDef bound the sensor offset / black level (ASI "Brightness",
	// SetBrightness), surfaced by Camera.OffsetRange for the Alpaca Offset caps.
	// OffsetMax 0 means undeclared. (The exact ASI control caps come from GetControlCaps; until
	// that's known these are the register-sane bounds — see the per-sensor SetOffset.)
	OffsetMin, OffsetMax, OffsetDef int
	// GainCaps/OffsetCaps, when set, return the VENDOR-specific advertised range — the dual of
	// the vendor-dispatched SetGain/SetOffset for a die shared across vendors (same silicon,
	// different gain/offset unit scale). Camera.GainRange/OffsetRange call them with the
	// regmap's VID; nil falls back to the static GainMin/Max / OffsetMin/Max/Def above
	// (single-vendor sensors, e.g. imx174/imx290). A max of 0 means undeclared.
	GainCaps   func(vid uint16) (min, max int)
	OffsetCaps func(vid uint16) (min, max, def int)
	// ExpMinUs/ExpMaxUs bound SetExposure in microseconds — the SetExp clamp,
	// surfaced by Camera.ExposureRange. ExpMaxUs 0 means undeclared.
	ExpMinUs, ExpMaxUs int64

	// Bus selects which vendor request WriteReg/ReadReg use. The zero value is
	// BusSony (WriteSONYREG 0xB6 / ReadSONYREG 0xB7), correct for the Sony IMX
	// majority; non-Sony dies (Aptina/ON, Panasonic, SmartSens) drive the generic
	// camera-register bus (BusCamera, 0xA6). FPGA-side timing always goes through
	// WriteFPGAReg regardless of Bus.
	Bus RegBus

	// Init is the sensor-side init sequence (ZWO/FPGA-facing — NOT a kernel
	// driver's CSI-2 init, which configures a different output interface).
	Init []RegVal

	// InitFPGA, if set, runs the FPGA-side init after the Init table: the FX3
	// FPGA setup (FPGAReset / SetFPGAAsMaster / FPGAStop / EnableFPGADDR /
	// SetFPGAADCWidthOutputWidth) as FPGA-register RMWs. subtype is the
	// firmware subtype byte (OpenCamera's GetFirmwareVer) — the
	// InitCamera branches key off it (e.g. < 0x12 writes the FPGA black level, >=
	// 0x12 programs the gain channels instead). Camera.Init then issues the FPGA
	// bank-select command (SendCMD 0xAE). nil = no FPGA-side init.
	InitFPGA func(rm Regmap, subtype int) error

	SetGain     func(rm Regmap, gain int) error
	SetExposure func(rm Regmap, d time.Duration) error
	// SetOffset programs the sensor offset / black level (ASI Brightness). nil = unsupported.
	SetOffset func(rm Regmap, offset int) error
	// SetROI programs a readout window. (x, y) is the top-left in BINNED output pixels and
	// (w, h) is the output size in binned pixels; bin is the symmetric binning factor (1 =
	// full resolution). The profile owns the bin→register translation: it applies the bin's
	// readout-mode table and converts the binned window to whatever coordinate system its
	// start/size registers use (e.g. the IMX455 start is in sensor pixels = binned·bin). A
	// profile that has only decoded bin 1 must return an error for bin > 1 rather than
	// silently reading full resolution.
	SetROI func(rm Regmap, x, y, w, h, bin int) error

	// StreamStop / StreamStart are the sensor-side master stop/start that bracket a
	// capture (the capture worker): the generic FX3 SendCMD 0xAA/0xA9 and
	// FPGA start/stop are issued by the Camera, but the sensor's own streaming gate
	// (e.g. IMX174 WriteSONYREG 0x200 = 1 then 0) lives here. nil = no sensor gate.
	StreamStop  func(rm Regmap) error
	StreamStart func(rm Regmap) error

	// Worker, if set, runs the full per-sensor capture worker
	// and supersedes the generic arm()/readFrame() snap path. It arms, HOST-TIMES the
	// exposure, re-arms, fires, and reads one frame into buf; exposure is the integration
	// time; it returns the bytes read. For IMX174: a double-arm bracketing
	// EnableFPGATriggerSignal(1→0) around usleep(exposure−200ms) — the deterministic
	// single shot that fixes the free-run 2×. nil = use the generic snap path.
	Worker func(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error)
}

// WorkerCtl gives a per-sensor Worker the generic FX3/FPGA/transport primitives it needs
// around its own sensor-register choreography. Implemented by *Camera. (Sensor + FPGA
// register R/W go through Rm(); FX3 vendor commands, the bulk pipe, and the bulk read
// are not Regmap ops, so they are exposed here.)
type WorkerCtl interface {
	Rm() Regmap                                              // sensor + FPGA register R/W
	VendorCmd(cmd uint8) error                               // FX3 vendor cmd (0xAA/0xA9/0xAF)
	ResetEndpoint() error                                    // clear the bulk-IN pipe (EP 0x81)
	BulkRead(buf []byte, timeout time.Duration) (int, error) // whole-frame bulk read
	FrameBytes() int                                         // bytes to read off the wire (W*H*bpp; ×SoftBin² for RAW16 software bin)
	// StreamFrame reads one frame with the continuous windowed pump (the USB3
	// startAsyncXfer window): transfers kept cycling on EP 0x81 until len(buf) bytes
	// are in, gap-free across short packets — what a large IMX455/IMX571 frame needs
	// and a one-shot BulkRead can't do. idle bounds a per-completion stall (returns
	// short so the worker can re-kick the FPGA and continue into buf[n:]); total bounds
	// the whole read. Falls back to BulkStreamer / BulkRead on backends without it.
	StreamFrame(buf []byte, idle, total time.Duration) (int, error)
}

// ExposureStatus mirrors ASI_EXPOSURE_STATUS (the int at the camera's +0x254).
type ExposureStatus int

const (
	ExpIdle    ExposureStatus = 0
	ExpWorking ExposureStatus = 1
	ExpSuccess ExposureStatus = 2
	ExpFailed  ExposureStatus = 3
)

func (s ExposureStatus) String() string {
	switch s {
	case ExpIdle:
		return "idle"
	case ExpWorking:
		return "working"
	case ExpSuccess:
		return "success"
	case ExpFailed:
		return "failed"
	}
	return "unknown"
}
