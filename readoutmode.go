package astrocam

// ReadoutMode is the live readout context: what the Camera has negotiated or been asked for,
// as opposed to what the sensor profile bakes in. Sensors and the timing math read it through
// ModeOf; the Camera mutates it through the modeCarrier capability its Regmap implements.

// ReadoutMode is the runtime readout context the FPS/line-time math needs (USB link speed,
// output depth, frame-rate setting). Supplied by the Camera, not the sensor profile.
type ReadoutMode struct {
	USB3       bool // negotiated link speed (Model.USB3); selects the bandwidth budget
	BytesPerPx int  // output bytes/pixel: 2 = RAW16, 1 = RAW8
	FPSPercent int  // requested frame-rate percentage; the vendor's floor is applied by the Camera
	Bin        int  // the sensor-side (hardware) binning factor; 0 normalized to 1
	// SoftBin is the host-side bin factor applied after readout (1 = none). By default the whole
	// factor is host-side (the sensor runs bin 1 over the bin-scaled region, Width/Height = its
	// dims); with hardware bin on, Bin is the sensor's factor and SoftBin the remainder. The
	// read bins SoftBin×SoftBin blocks (RAW16 mean, RAW8 clipped sum); the output is SoftBin²
	// smaller.
	SoftBin int
	// Width/Height are the live readout dimensions (the ROI after binning). VMAX = Height +
	// VBlankAdd sets the free-run frame period, and the HMAX (line-time) throttle scales with
	// Width, so a sub-frame ROI streams faster. 0 means the full-frame dimension.
	Width  int
	Height int
	// HighSpeed selects the sensor's 10-bit high-speed readout (ASI_HIGH_SPEED_MODE): a shorter
	// ADC ramp at a doubled pixel clock (~2× fps) for RAW8. The profile reformats to 10-bit and
	// switches clock/HMAX-floor when set.
	HighSpeed bool
	// SensorMode is the index into the profile's Sensor.SensorModes (POASetSensorMode), 0 being
	// the die's normal readout. A mode is a re-tuning of the same register block rather than a
	// different programme, so it travels with the readout mode: the geometry, the frame period
	// and the sensor mode block all key off it. A profile with one mode ignores this.
	SensorMode int
	// BinSum selects summed rather than averaged binned pixels (POA_PIXEL_BIN_SUM). Averaging
	// keeps the result inside the original sample's range, so binning buys noise reduction but
	// discards the dynamic range it earns; summing puts the value in the full container and
	// preserves the electron count, at the cost of saturating N^2 sooner. The SDK defaults to
	// averaging and so does this.
	BinSum bool
	// FrameLimit caps the frame rate in frames per second (POA_FRAME_LIMIT, 0 = no limit). It is
	// a THIRD term in the frame period beside the link budget and the exposure, not a rescaling
	// of either: period = max(bandwidthTime, exposureTime, 1/FrameLimit).
	FrameLimit int
	// SensorBin is the part of Bin the SENSOR DIE performs, for a vendor whose camera bins in the
	// FPGA by default (PlayerOne). 0 or 1 means the die is not binning and the whole factor is
	// the FPGA's. It is not SoftBin: nothing here is host-side. The split matters on the wire —
	// with the die binning, the FPGA is handed the already-binned geometry and a smaller factor.
	SensorBin int
}

func (m ReadoutMode) norm() ReadoutMode {
	if m.BytesPerPx == 0 {
		m.BytesPerPx = 2 // RAW16 default
	}
	if m.Bin < 1 {
		m.Bin = 1
	}
	if m.FPSPercent == 0 {
		m.FPSPercent = 100
	}
	// Only a sanity clamp here: the real floor is vendor policy (ZWO 40, PlayerOne 35) and is
	// applied by Camera.SetFPSPercent and Open, which know the vendor. norm() runs on every
	// ModeOf and must not raise a percentage the vendor legitimately accepts.
	if m.FPSPercent < 1 {
		m.FPSPercent = 1
	}
	if m.FPSPercent > 100 {
		m.FPSPercent = 100
	}
	return m
}

// modeReader is the optional Regmap capability that carries the live ReadoutMode into the
// shared exposure/HMAX bodies. A plain Regmap (e.g. a test fake) falls back to defaults.
type modeReader interface{ ReadoutMode() ReadoutMode }

// ModeOf returns the live ReadoutMode carried by rm, or normalized defaults.
func ModeOf(rm Regmap) ReadoutMode {
	if p, ok := rm.(modeReader); ok {
		return p.ReadoutMode().norm()
	}
	return ReadoutMode{}.norm()
}
