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
	// GainCaps/OffsetCaps, when set, return the vendor-specific advertised range (the dual of the
	// vendor-dispatched SetGain/SetOffset for a die shared across vendors). Camera.GainRange/
	// OffsetRange call them with the regmap's VID; nil falls back to the static fields above. A
	// max of 0 means undeclared.
	GainCaps   func(vid uint16) (min, max int)
	OffsetCaps func(vid uint16) (min, max, def int)
	// ExpMinUs/ExpMaxUs bound SetExposure in microseconds, surfaced by Camera.ExposureRange.
	// ExpMaxUs 0 means undeclared.
	ExpMinUs, ExpMaxUs int64

	// Bus selects which vendor request WriteReg/ReadReg use. Zero value is BusSony (WriteSONYREG
	// 0xB6 / ReadSONYREG 0xB7) for the Sony IMX majority; non-Sony dies (Aptina/ON, Panasonic,
	// SmartSens) drive the generic camera-register bus (BusCamera, 0xA6). FPGA timing always goes
	// through WriteFPGAReg regardless.
	Bus RegBus

	// FX3DMAMarkers marks a sensor whose FX3 readout brackets every frame with fixed DDR
	// header/footer marker words — the first and last 32-bit DMA word (0x5A7E header / 0x3CF0
	// footer) are not pixel data. When set, GetDataAfterExp runs the frame through
	// repairFX3DMAMarkers (which signature-checks, so it can't corrupt a frame that lacks the
	// markers). Confirmed: IMX455, IMX462. Leave false on unverified sensors.
	FX3DMAMarkers bool

	// Init is the sensor-side init sequence (ZWO/FPGA-facing).
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
	// SetROI programs a readout window. (x, y) is the top-left and (w, h) the size, in binned
	// output pixels; bin is the symmetric binning factor (1 = full resolution). The profile owns
	// the bin→register translation, converting the binned window to its registers' coordinate
	// system (e.g. IMX455 start is in sensor pixels = binned·bin). A profile that has only decoded
	// bin 1 must return an error for bin > 1.
	SetROI func(rm Regmap, x, y, w, h, bin int) error

	// StreamStop / StreamStart are the sensor-side master stop/start that bracket a capture. The
	// generic FX3 SendCMD 0xAA/0xA9 and FPGA start/stop are issued by the Camera; the sensor's own
	// streaming gate (e.g. IMX174 WriteSONYREG 0x200 = 1 then 0) lives here. nil = no sensor gate.
	StreamStop  func(rm Regmap) error
	StreamStart func(rm Regmap) error

	// Worker, if set, runs the full per-sensor capture worker, superseding the generic arm()/
	// readFrame() snap path. It arms the sensor, host-times the integration where needed, reads
	// one frame into buf, and halts the readout before returning so the sensor does not free-run
	// between captures (leaving the 174 streaming backs up the FX3 and crashes its firmware).
	// exposure is the integration time; it returns the bytes read. nil = use the generic snap path.
	Worker func(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error)
}

// WorkerCtl gives a per-sensor Worker the generic FX3/FPGA/transport primitives it needs around
// its own sensor-register choreography. Implemented by *Camera.
type WorkerCtl interface {
	Rm() Regmap                                              // sensor + FPGA register R/W
	VendorCmd(cmd uint8) error                               // FX3 vendor cmd (0xAA/0xA9/0xAF)
	ResetEndpoint() error                                    // clear the bulk-IN pipe (EP 0x81)
	ResetDevice() error                                      // USB device reset (last-resort wedge recovery; errors on backends without it)
	NoteStall()                                              // record one readout stall for the soak diagnostic (Camera.StallCount)
	Aborted() bool                                           // StopExposure was called: a host-timed integration loop should bail out
	BulkRead(buf []byte, timeout time.Duration) (int, error) // whole-frame bulk read
	// BulkReadQuiet is BulkRead whose first `quiet` is a SENSOR-TIMED integration the read
	// spans (cycle-count / free-run bands): transfers armed up front, but the control-transfer
	// gate engages only at quiet-elapsed or first data — so the exposure doesn't blind EP0.
	// Pass quiet undershooting the real integration (leave ≥500 ms margin); quiet 0 = BulkRead.
	// Falls back to a fully-gated BulkRead on backends without it.
	BulkReadQuiet(buf []byte, quiet, timeout time.Duration) (int, error)
	FrameBytes() int                                         // bytes to read off the wire (W*H*bpp; ×SoftBin² for RAW16 software bin)
	// StreamFrame reads one frame with the continuous windowed pump: transfers cycle on EP 0x81
	// until len(buf) bytes are in, gap-free across short packets (what a large IMX455/IMX571 frame
	// needs and a one-shot BulkRead can't do). idle bounds a per-completion stall (returns short so
	// the worker can re-kick the FPGA and continue into buf[n:]); total bounds the whole read.
	// Falls back to BulkRead on backends without it.
	StreamFrame(buf []byte, idle, total time.Duration) (int, error)
	// StreamFramePrequeued reads one frame with a pre-queued URB batch covering it exactly (the
	// SDK's async-transfer model): the transfers wait on the pipe before the frame arrives, so
	// the read overlaps the sensor readout — the free-run STARVIS sensors tear on a USB2 link
	// with the one-at-a-time StreamFrame. Falls back to StreamFrame where unavailable.
	StreamFramePrequeued(buf []byte, idle, total time.Duration) (int, error)
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
