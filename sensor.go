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
	// ExpCaps, when set, returns the vendor-specific exposure bounds for a die the vendors
	// advertise differently, the same seam as GainCaps/OffsetCaps. nil falls back to the static
	// fields above; a max of 0 means undeclared.
	ExpCaps func(vid uint16) (minUs, maxUs int64)

	// TriggerBandUs is the exposure at or above which this die stops holding its own shutter and
	// the FPGA times the integration instead, parking the sensor for the duration. Free-run arms
	// once and expects the sensor to keep producing frames, so it yields NOTHING in that band —
	// a continuous capture has to loop single shots there instead. Surfaced by
	// Camera.TriggerBand.
	//
	// Per die, not per vendor, and the values genuinely differ: 1 s on the IMX178/290/455/462/
	// 571/585, but 4 s on the IMX174. 0 means the die free-runs at any exposure this profile
	// declares.
	TriggerBandUs int64

	// Bus selects which vendor request WriteReg/ReadReg use. Zero value is BusSony (WriteSONYREG
	// 0xB6 / ReadSONYREG 0xB7); non-Sony dies (Aptina/ON, Panasonic, SmartSens) use the generic
	// camera-register bus (BusCamera, 0xA6). FPGA timing always goes through WriteFPGAReg.
	Bus RegBus

	// ROIStartAlign, when set, reports the sensor-pixel alignment of the readout window start
	// for a sensor bin factor (the SDK's window-start masks: X to 16 on the IMX455/571, to 4 on
	// the IMX290/462 and the 174, to 2 on the IMX585; Y to 2, to 4 on the 585, or 4/6 for the
	// 455/571 bin-2/3 tables). Camera.SetROI aligns the requested start down by it and reports
	// the aligned start from ROI, so a caller sees the window it gets. nil = the profile aligns
	// silently.
	ROIStartAlign func(bin int) (x, y int)

	// HWBins lists the binning factors the profile has a hardware (on-sensor) readout mode for;
	// SetROI receives one of them, or 1. The Camera uses them only when SetHardwareBin(true) is
	// set (the SDK's ASI_HARDWARE_BIN); by default every factor is binned on the host from a
	// bin-1 readout, as the SDK does. A requested factor with no hardware mode of its own is
	// split into the largest listed divisor on the sensor and the rest on the host (the SDK's
	// bin 4 on the IMX455/571 = the bin-2 table over 2w×2h, host-binned 2×). nil = no hardware
	// binning.
	//
	// On a vendor whose camera bins itself at every factor (Vendor.deviceBins, PlayerOne) nothing
	// is left for the host; there this list says only how much of the factor the DIE takes and
	// how much the FPGA does (Camera.sensorSplit).
	HWBins []int

	// FX3DMAMarkers marks a sensor whose FX3 readout brackets every frame with fixed DDR
	// header/footer marker words (see repairFX3DMAMarkers). Hardware-confirmed on the IMX455,
	// IMX462, IMX290 and IMX174; the IMX571 sets it from the 455's behaviour without a camera to
	// check it on. Leave false on unverified sensors.
	FX3DMAMarkers bool

	// Init is the sensor-side init sequence.
	Init []RegVal
	// InitByVID overrides Init for a vendor whose firmware frames the same Sony tuning
	// differently. The two decoded vendors agree on the die's analog table almost register for
	// register — where they overlap on the IMX455 and IMX585, not one value differs — but they
	// disagree on which registers belong in the table versus the init sequence around it, and on
	// the IMX571 two values differ outright, so the table has to be vendor-selected. A VID with
	// no entry uses Init.
	InitByVID map[uint16][]RegVal

	// SizeByVID overrides Info's MaxWidth/MaxHeight for a vendor that reads out a different area
	// of the same die. The IMX585's effective array is 3856x2180 and PlayerOne exposes all of it;
	// the ZWO transcription this profile came from programs the 3840x2160 UHD crop. Only the
	// geometry varies, so the rest of Info stays shared.
	SizeByVID map[uint16][2]int

	// BinsByVID overrides Info.Bins for a vendor whose camera offers binning the other's does not.
	BinsByVID map[uint16][]int

	// EGainBase is the sensor's electrons-per-ADU at gain 0. The conversion falls off with gain
	// as base / 10^(gain/200), which is where the SDK's 0.1 dB gain unit shows itself: 200 units
	// per decade of voltage gain. Zero means the die's value has not been read out.
	EGainBase float64

	// SensorModes, when set, returns the alternative readout programmes the die offers on this
	// vendor's body, index 0 being the normal mode (POAGetSensorModeCount / POAGetSensorModeInfo).
	// It is vendor-keyed for the same reason the caps are: the mode list is what a vendor's
	// firmware exposes over the die, not a property of the silicon. nil, or a result of fewer
	// than two entries, means the profile offers no mode selection.
	SensorModes func(vid uint16) []SensorModeInfo
	// Presets, when set, returns the vendor's preset gain/offset operating points for this die.
	// They are vendor policy over one part, like the caps, so they travel by VID. ok false means
	// the profile has not decoded them for that vendor.
	Presets func(vid uint16) (GainOffsetPresets, bool)
	// SetSensorMode programs the sensor-side block for a mode index (POASetSensorMode). The
	// geometry half of a mode change belongs to SetROI, which reads the mode off the ReadoutMode;
	// this writes only what the sensor itself holds. It must return an error for a mode the
	// profile has not decoded at the current sample size rather than reuse a neighbouring
	// combination. nil = no mode selection.
	SetSensorMode func(rm Regmap, mode int) error

	// PreInit, if set, runs BEFORE the Init table. It exists because PlayerOne resets the FPGA
	// and pulses the sensor reset line as the first thing it does, and a sensor reset after the
	// register table has been applied would discard it. ZWO resets in InitFPGA instead, which is
	// after the table, so it leaves this nil.
	PreInit func(rm Regmap) error

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

// SensorModeInfo names one readout programme the die offers (POASensorModeInfo). Name is short
// enough for a control, Desc is the tooltip-length explanation.
type SensorModeInfo struct {
	Name string
	Desc string
}

// WorkerCtl gives a per-sensor Worker the generic FX3/FPGA/transport primitives it needs around
// its own sensor-register choreography. Implemented by *Camera; see the Camera methods of the
// same names for the fallbacks.
type WorkerCtl interface {
	Rm() Regmap               // sensor + FPGA register R/W
	VendorCmd(op FX3Op) error // FX3StreamStop/Start/Flush
	// FPGARun starts and stops the readout pipeline through the vendor's encoding. A profile
	// must not open-code it: ZWO clears bit 4 of FPGA register 0 to run, PlayerOne writes 0x10
	// to the same register to run, so the literal that starts one camera stops the other.
	FPGARun(start bool) error
	ResetEndpoint() error // clear the bulk-IN pipe (EP 0x81)
	// DrainPipe discards data the device already queued on EP 0x81 and returns the byte
	// count, so a frame cannot start behind an earlier frame's tail. Call it before arming.
	DrainPipe(budget time.Duration) int
	ResetDevice() error // USB device reset (last resort)
	NoteStall()         // record one readout stall
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

// ExposureStatus mirrors ASI_EXPOSURE_STATUS.
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

// GainOffsetPresets are the operating points a client uses to choose a gain without knowing the
// sensor (POAGetGainsAndOffsets). Each gain has the offset that belongs with it.
type GainOffsetPresets struct {
	GainHighestDR int // usually 0: the most dynamic range
	GainHCG       int // where high conversion gain engages
	GainUnity     int // e/ADU == 1
	GainLowestRN  int // maximum analog gain, the least read noise
	OffsetHighestDR,
	OffsetHCG,
	OffsetUnity,
	OffsetLowestRN int
}

// ImageFormat is a pixel layout the driver can deliver off the wire. The vendor SDKs also define
// debayered outputs (RGB24, MONO8) but those are host-side conversions of a raw frame, not
// readout modes, and this driver returns the raw frame.
type ImageFormat int

const (
	FormatRAW8  ImageFormat = 1 // 1 byte per pixel
	FormatRAW16 ImageFormat = 2 // 2 bytes per pixel
)

func (f ImageFormat) String() string {
	switch f {
	case FormatRAW8:
		return "RAW8"
	case FormatRAW16:
		return "RAW16"
	}
	return "?"
}
