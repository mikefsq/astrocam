package astrocam

import "time"

// RegVal is one entry of an init register table.
type RegVal struct{ Reg, Val uint16 }

// CameraInfo is the sensor-intrinsic geometry/capability; per-model facts (color, cooler) are
// layered on by the Model.
type CameraInfo struct {
	MaxWidth  int
	MaxHeight int
	PixelUm   float64
	BitDepth  int // ADC depth (e.g. 12)
	// Bayer is the CFA pattern of the die's color variant (e.g. "RGGB"). One profile serves both
	// the MM and MC models; Camera.Info surfaces this only when Model.Color is set, else "".
	// Leave "" for dies with no color variant.
	Bayer string
	Bins  []int // supported symmetric binning factors
}

// Sensor is a per-chip profile: an init register table plus a few ops over a Regmap. Gain is in
// ASI units (0.1 dB steps); Exposure ops receive a duration.
type Sensor struct {
	Name string
	Info CameraInfo

	// GainMin/GainMax bound the ASI gain (0.1 dB units) that SetGain accepts, surfaced by
	// Camera.GainRange. GainMax 0 means no declared range.
	GainMin, GainMax int
	// OffsetMin/OffsetMax/OffsetDef bound the sensor offset / black level (ASI Brightness),
	// surfaced by Camera.OffsetRange. OffsetMax 0 means undeclared.
	OffsetMin, OffsetMax, OffsetDef int
	// GainCaps/OffsetCaps, when set, return the vendor-specific advertised range for a die
	// shared across vendors. Camera.GainRange and OffsetRange call them with the regmap's VID;
	// nil falls back to the static fields above. A max of 0 means undeclared.
	GainCaps   func(vid uint16) (min, max int)
	OffsetCaps func(vid uint16) (min, max, def int)
	// ExpMinUs/ExpMaxUs bound SetExposure in microseconds, surfaced by Camera.ExposureRange.
	// ExpMaxUs 0 means undeclared.
	ExpMinUs, ExpMaxUs int64

	// Bus selects which vendor request WriteReg/ReadReg use. Zero value is BusSony (WriteSONYREG
	// 0xB6 / ReadSONYREG 0xB7); non-Sony dies (Aptina/ON, Panasonic, SmartSens) use the generic
	// camera-register bus (BusCamera, 0xA6). FPGA timing always goes through WriteFPGAReg.
	Bus RegBus

	// ROIStartAlign, when set, reports the sensor-pixel alignment of the readout window start
	// for a sensor bin factor (the SDK's SetStartPos masks: X to 16 on the IMX455/571, to 4 on
	// the IMX290/462 and the 174, to 2 on the IMX585; Y to 2, to 4 on the 585, or 4/6 for the
	// 455/571 bin-2/3 tables). Camera.SetROI
	// aligns the requested start down by it and reports the aligned start from ROI, so a caller
	// sees the window it gets. nil = the profile aligns silently.
	ROIStartAlign func(bin int) (x, y int)

	// HWBins lists the binning factors the profile has a hardware (on-sensor) readout mode for;
	// SetROI receives one of them, or 1. The Camera uses them only when SetHardwareBin(true) is
	// set (the SDK's ASI_HARDWARE_BIN); by default every factor is binned on the host from a
	// bin-1 readout, as the SDK does. A requested factor with no hardware mode of its own is
	// split into the largest listed divisor on the sensor and the rest on the host (the SDK's
	// bin 4 on the IMX455/571 = the bin-2 table over 2w×2h, host-binned 2×). nil = no hardware
	// binning.
	HWBins []int

	// FX3DMAMarkers marks a sensor whose FX3 readout brackets every frame with fixed DDR
	// header/footer marker words (see repairFX3DMAMarkers). Hardware-confirmed on the IMX455,
	// IMX462, IMX290 and IMX174; the IMX571 sets it from the 455's behaviour without a camera to
	// check it on. Leave
	// false on unverified sensors.
	FX3DMAMarkers bool

	// Init is the sensor-side init sequence.
	Init []RegVal

	// InitFPGA, if set, runs the FPGA-side init after the Init table (FPGAReset /
	// SetFPGAAsMaster / FPGAStop / EnableFPGADDR / SetFPGAADCWidthOutputWidth as FPGA-register
	// RMWs). subtype is the firmware subtype byte; branches key off it (e.g. <0x12 writes the FPGA
	// black level, >=0x12 programs the gain channels). nil = no FPGA-side init.
	InitFPGA func(rm Regmap, subtype int) error

	SetGain     func(rm Regmap, gain int) error
	SetExposure func(rm Regmap, d time.Duration) error
	// SetOffset programs the sensor offset / black level (ASI Brightness). nil = unsupported.
	SetOffset func(rm Regmap, offset int) error
	// GetOffset reads the offset back from the sensor registers, in SetOffset's units. nil = not
	// readable (Camera.Offset falls back to the last value set).
	GetOffset func(rm Regmap) (int, error)
	// SetROI programs a readout window. (x, y) is the top-left and (w, h) the size, in binned
	// output pixels; bin is the symmetric binning factor (1 = full resolution). The profile owns
	// the bin→register translation (e.g. IMX455 start is in sensor pixels = binned·bin). A profile
	// that has only decoded bin 1 must return an error for bin > 1.
	SetROI func(rm Regmap, x, y, w, h, bin int) error

	// StreamStop / StreamStart are the sensor-side master stop/start that bracket a capture. The
	// FX3 SendCMD 0xAA/0xA9 and FPGA start/stop are issued by the Camera; the sensor's own
	// streaming gate (e.g. IMX174 WriteSONYREG 0x200 = 1 then 0) lives here. nil = no sensor gate.
	StreamStop  func(rm Regmap) error
	StreamStart func(rm Regmap) error

	// Arm, if set, replaces Camera.arm's generic sequence (SendCMD stop, FPGAStop, StreamStop,
	// SendCMD start, StreamStart, settle, FPGAStart) for StartVideo, the generic snap path and
	// the sensor's own Worker. The DDR cameras need it: the IMX455 object issues a second
	// FPGAStop after SendCMD(0xA9) before the master start, without which the readout never
	// delivers a free-run frame.
	Arm func(ctl WorkerCtl) error

	// Worker, if set, runs the full per-sensor capture, superseding the generic arm()/readFrame()
	// snap path: it arms the sensor, host-times the integration where needed, reads one frame into
	// buf, and halts the readout before returning so the sensor does not free-run between captures
	// (a 174 left streaming backs up the FX3 and crashes its firmware). exposure is the integration
	// time; it returns the bytes read.
	//
	// The halt runs on every return, including an aborted one, and is not generation-guarded, so
	// a Camera runs one capture at a time: the previous Worker must have returned before the next
	// is armed, or its halt would stop the new capture's readout.
	Worker func(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error)
}

// WorkerCtl gives a per-sensor Worker the generic FX3/FPGA/transport primitives it needs around
// its own sensor-register choreography. Implemented by *Camera; see the Camera methods of the
// same names for the fallbacks.
type WorkerCtl interface {
	Rm() Regmap               // sensor + FPGA register R/W
	VendorCmd(op FX3Op) error // FX3StreamStop/Start/Flush
	ResetEndpoint() error     // clear the bulk-IN pipe (EP 0x81)
	ResetDevice() error       // USB device reset (last resort)
	NoteStall()               // record one readout stall
	// ReapplyOffset rewrites the last offset set (Camera.SetOffset / the init default) through
	// the profile's SetOffset, for a die whose black-level register does not survive the
	// capture cycle (IMX174).
	ReapplyOffset() error
	Aborted() bool                                           // StopExposure ran: bail out
	BulkRead(buf []byte, timeout time.Duration) (int, error) // whole-frame bulk read
	// BulkReadQuiet is BulkRead whose first `quiet` is a sensor-timed integration the read spans
	// (QuietBulkReader). Pass quiet undershooting the real integration (leave ≥500 ms margin);
	// quiet 0 = BulkRead.
	BulkReadQuiet(buf []byte, quiet, timeout time.Duration) (int, error)
	FrameBytes() int // bytes to read off the wire (W*H*bpp; ×SoftBin² for RAW16 software bin)
	// StreamFrame reads one frame with the windowed pump (FrameStreamer). idle bounds a
	// per-completion stall (returns short; no worker re-kicks and resumes into
	// buf[n:]); total bounds the whole read.
	StreamFrame(buf []byte, idle, total time.Duration) (int, error)
	// StreamFramePrequeued reads one frame with a pre-queued transfer batch
	// (PrequeuedFrameStreamer). Only the 462 uses it: the other free-run STARVIS profile, the
	// 290, reads with BulkRead, whose backends are whole-frame batch engines anyway.
	StreamFramePrequeued(buf []byte, idle, total time.Duration) (int, error)
}

// ExposureStatus mirrors ASI_EXPOSURE_STATUS (the int at the SDK camera object's +0x254).
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
