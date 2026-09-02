package astrocam

import "sort"

// DeviceID is a USB (VID,PID) identifier, the key the camera-model registry uses. The VID is
// part of the key because two vendors can reuse the same PID.
type DeviceID struct{ VID, PID uint16 }

// Vendor describes a camera maker on the shared FX3 + Sony-sensor platform: its USB vendor id
// and the wire-protocol dialect its cameras speak. Sensor profiles are vendor-independent; a
// Vendor supplies the Regmap dialect and the FX3 bridge command table. One VID per Vendor.
type Vendor struct {
	VID  uint16
	Name string
	// Cmds are the vendor requests the FX3 bridge firmware answers outside the register dialect
	// (stream control, GPIF bus, flash, identity). A zero entry means "not decoded for this
	// vendor" and the operation errors instead of sending another vendor's bytes.
	Cmds FX3Cmds
	// newRegmap builds this vendor's Regmap dialect over an open Transport. bus selects the
	// sensor-register path (Sony I2C vs generic camera reg); mode carries the live readout
	// context.
	newRegmap func(t Transport, bus RegBus, mode ReadoutMode) Regmap
	// fpgaRun starts and stops the readout pipeline. The camera FPGA is vendor firmware and the
	// two decoded vendors disagree on the encoding: ZWO read-modify-writes bit 4 of register 0
	// as a STOP flag, PlayerOne writes whole-byte values to the same register with 0x10 meaning
	// START. Open fixes the vendor from the USB (VID,PID), so the capture path calls this rather
	// than open-coding either encoding.
	fpgaRun func(rm Regmap, start bool) error
	// newThermal builds this vendor's cooling backend. The actuators are FPGA registers and the
	// two vendors' maps collide with different loads behind them — ZWO's TEC power register 0x26
	// is PlayerOne's window heater, ZWO's heater PWM 0x2a is PlayerOne's read-only status — so a
	// shared implementation would energise the wrong hardware.
	newThermal func(c *Camera) Thermal
	// deviceBins reports that this vendor's camera reduces the frame itself at every binning
	// factor, so the host never bins after readout. ZWO ships the full frame and the host bins the
	// remainder (ReadoutMode.SoftBin); PlayerOne's FPGA bins whenever the sensor does not, and the
	// wire carries the already-reduced frame. That changes how many bytes a frame is, so it has to
	// be known before the read.
	deviceBins bool
	// roiStepW / roiStepH are the window granularity the vendor's firmware enforces. ZWO wants the
	// width a multiple of 8 and the height even; PlayerOne wants the width a multiple of 4. Zero
	// means ZWO's rule, so a vendor that has not been measured keeps the stricter one.
	roiStepW, roiStepH int
	// fpsMin is the lowest bandwidth percentage this vendor's firmware accepts, and fpsDefUSB2 /
	// fpsDefUSB3 the defaults its SDK applies per link. ZWO clamps at 40 and runs the link flat
	// out on USB3; PlayerOne accepts 35 and defaults to 90 on both, which poasnap -caps reports
	// as USBBandWidthLimit min=35 max=100 def=90. Zero values fall back to ZWO's.
	fpsMin, fpsDefUSB2, fpsDefUSB3 int
	// frameTrailer is how many bytes this vendor's FX3 firmware sends after a frame's pixels, in
	// free-run. A reader that counts only the pixels leaves them in the pipe and every later frame
	// starts that far in, which reads as a torn image and never as an error.
	//
	// It belongs to the vendor because the two decoded ones disagree, and measurement is the only
	// way to know: an ASI6200MC sends exactly width×height×bpp and nothing else, across three
	// window sizes, while a PlayerOne body appends sixteen bytes to every frame. It cannot be
	// inferred from the stream — a short USB transfer marks a DMA commit as well as a frame end,
	// so "whatever follows the pixels" is sometimes the next frame's pixels.
	frameTrailer int
	// frameMarker repairs the fixed header this vendor's firmware writes over the first pixels of
	// every frame. It belongs to the vendor rather than the die: the marker is the FX3/FPGA
	// firmware's, and the same sensor under the other vendor carries a different one, or none.
	// nil = this vendor's frames start with pixel data. width is in output pixels and rows is how
	// far down to reach for replacements.
	frameMarker func(buf []byte, bpp, width, rows int)
	// frameStart locates a frame boundary inside a buffer taken from the free-run byte stream and
	// returns its byte offset, or -1 when the buffer already begins on one (or the vendor has no
	// marker to find it by).
	//
	// A resident stream session hands out frameBytes at a time from a continuous stream, and
	// nothing makes the first of those bytes the first byte of a frame. Every frame then comes out
	// rotated by the same offset, which reads as a vertical seam rather than as an error: the byte
	// count is right and only the content is wrong. Locating the boundary once, right after the
	// session opens, lets the reader swallow the offset so every later frame lands square.
	frameStart func(buf []byte) int
	// loadDefectMap reads this vendor's factory hot-pixel map from flash. The two layouts share
	// nothing: ZWO stores a sparse-RLE bitmap behind an "ASID" header, PlayerOne a per-row run
	// list behind an "HPC:" one. nil = not decoded for this vendor.
	loadDefectMap func(c *Camera, fullW, fullH int) (*DefectMap, error)
	// defectMapTrusted gates AUTOMATIC application of that map. The map stays readable either way
	// (Camera.LoadDefectMap, gosnap -defects); this decides whether Init loads it for RepairFrame
	// to apply to every frame. False for a vendor whose map has not been shown to select real
	// defects on hardware — correcting pixels that are not defective replaces good data with a
	// neighbour average, which is worse than doing nothing.
	defectMapTrusted bool
	// setWhiteBalance programs the per-channel white-balance gains on a colour body. nil = the
	// vendor's registers are not decoded, and Camera.SetWhiteBalance refuses.
	setWhiteBalance func(rm Regmap, r, g, b int) error
	// wbLimit bounds each channel; 0 means setWhiteBalance is unsupported.
	wbLimit int
}

// fpsBounds returns this vendor's bandwidth-percentage floor and its per-link defaults.
func (v *Vendor) fpsBounds() (min, defUSB2, defUSB3 int) {
	min, defUSB2, defUSB3 = v.fpsMin, v.fpsDefUSB2, v.fpsDefUSB3
	if min == 0 {
		min = 40 // ZWO's BANDWIDTHOVERLOAD floor
	}
	if defUSB2 == 0 {
		defUSB2 = 40
	}
	if defUSB3 == 0 {
		defUSB3 = 100
	}
	return min, defUSB2, defUSB3
}

// roiStep resolves the window granularity, defaulting to ZWO's.
func (v *Vendor) roiStep() (w, h int) {
	w, h = v.roiStepW, v.roiStepH
	if w == 0 {
		w = 8
	}
	if h == 0 {
		h = 2
	}
	return w, h
}

// FX3Cmd is one SendCMD-style vendor OUT: a request code plus the wValue that selects the
// operation. ZWO gives every operation its own request code and leaves wValue 0; PlayerOne
// reuses one code for a pair and selects between them with wValue (0xA0 wValue 1/0 = stream
// start/stop). A zero Req means the operation is not decoded for the vendor.
type FX3Cmd struct {
	Req    uint8
	WValue uint16
}

// decoded reports whether the vendor has this command.
func (c FX3Cmd) decoded() bool { return c.Req != 0 }

// FX3ST4 describes how a vendor encodes an ST4 guide pulse. ZWO gives assert and release their
// own request codes and carries the direction in wValue. PlayerOne uses one code (0xA6) with the
// line state in wValue and the direction in wIndex, so DirInWIndex selects that layout.
type FX3ST4 struct {
	On, Off     uint8 // request codes; equal when one code carries both states
	DirInWIndex bool  // true: wValue = 1/0 line state, wIndex = direction
}

// FX3Cmds holds a vendor's FX3 bridge request codes (bRequest values). A zero entry means "not
// decoded for this vendor", and the operation errors instead of sending another vendor's bytes.
// Fields that vary in more than their opcode carry the rest of their shape alongside it, because
// the two decoded vendors do not agree on reply lengths or argument placement.
type FX3Cmds struct {
	StreamStop  FX3Cmd // stop/prepare before (re)arming
	StreamStart FX3Cmd // begin streaming
	// Flush is the pipeline flush / drop-recovery command. Optional: PlayerOne has no such
	// command and recovers host-side (libusb bulk clear/reset) instead, so it leaves this zero.
	Flush FX3Cmd

	// EnableGPIF32DQ gates the FPGA->FX3 32-bit data bus (vendor OUT, wValue 0/1). Optional:
	// only vendors whose SPI flash shares the FX3 pins with the data bus need it (ZWO does,
	// PlayerOne does not).
	EnableGPIF32DQ uint8
	// ReadSPIFlash is a vendor IN reading up to 2 KiB per transfer with wIndex = address >> 8.
	// Both vendors fit: a PlayerOne page number IS the address >> 8 (0xD1). They differ only in
	// needing the bus gate above, which ZWO does and PlayerOne does not.
	ReadSPIFlash uint8

	FirmwareVersion uint8 // vendor IN, little-endian; FirmwareBytes long
	FirmwareBytes   uint8 // reply length; 0 means 2 (ZWO). PlayerOne answers 1 byte
	SerialNumber    uint8 // vendor IN, SerialBytes of factory serial
	SerialBytes     uint8 // reply length; 0 means 8 (ZWO). PlayerOne burns 20
	// SerialASCII reports that the serial bytes are printable text (PlayerOne, e.g.
	// "CAMGF252416072209000") rather than the raw id ZWO renders as hex.
	SerialASCII bool

	ST4 FX3ST4 // guide-pulse assert/release

	ReadTemp      uint8 // vendor IN: sensor temperature, ReadTempBytes long, decoded by TempC
	ReadTempBytes uint8 // request length; 0 means 2
	// TempC converts a ReadTemp reply to degrees Celsius. The two decoded vendors disagree on
	// the packing — ZWO sends a signed 12-bit value as (hi<<4 | lo>>4) in sixteenths of a
	// degree, PlayerOne a signed 16-bit little-endian value in tenths — so the conversion
	// travels with the request code. Required whenever ReadTemp is set.
	TempC func([]byte) float64

	ReadHumidity uint8 // vendor IN, 2 bytes: Sensirion RH raw; wValue = ReadHumidityWValue
	// ReadHumidityWValue is the wValue the ReadHumidity request carries (0xF5 on ZWO).
	ReadHumidityWValue uint16
}

// firmwareBytes / serialBytes resolve the reply lengths, defaulting to ZWO's.
func (c FX3Cmds) firmwareBytes() int {
	if c.FirmwareBytes == 0 {
		return 2
	}
	return int(c.FirmwareBytes)
}

func (c FX3Cmds) serialBytes() int {
	if c.SerialBytes == 0 {
		return 8
	}
	return int(c.SerialBytes)
}

func (c FX3Cmds) readTempBytes() int {
	if c.ReadTempBytes == 0 {
		return 2
	}
	return int(c.ReadTempBytes)
}

// FX3Op names a SendCMD-style FX3 vendor command a sensor Worker may issue through
// WorkerCtl.VendorCmd; the Camera resolves it to the vendor's request code.
type FX3Op int

const (
	FX3StreamStop  FX3Op = iota + 1 // Vendor.Cmds.StreamStop
	FX3StreamStart                  // Vendor.Cmds.StreamStart
	FX3Flush                        // Vendor.Cmds.Flush
)

func (op FX3Op) String() string {
	switch op {
	case FX3StreamStop:
		return "StreamStop"
	case FX3StreamStart:
		return "StreamStart"
	case FX3Flush:
		return "Flush"
	}
	return "FX3Op(?)"
}

// cmd resolves op against the table; a zero Req means not decoded.
func (c FX3Cmds) cmd(op FX3Op) FX3Cmd {
	switch op {
	case FX3StreamStop:
		return c.StreamStop
	case FX3StreamStart:
		return c.StreamStart
	case FX3Flush:
		return c.Flush
	}
	return FX3Cmd{}
}

// vendors maps a USB VID to the Vendor that owns it. Protocol layers register themselves from
// init().
var vendors = map[uint16]*Vendor{}

// RegisterVendor records a vendor descriptor under its VID.
func RegisterVendor(v *Vendor) { vendors[v.VID] = v }

// VendorOf returns the vendor that owns a USB VID, if one is registered.
func VendorOf(vid uint16) (*Vendor, bool) { v, ok := vendors[vid]; return v, ok }

// KnownVIDs returns, sorted, the USB vendor ids the driver knows (the set Enumerate scans).
func KnownVIDs() []uint16 {
	out := make([]uint16, 0, len(vendors))
	for vid := range vendors {
		out = append(out, vid)
	}
	sort.Slice(out, func(i, j int) bool { return out[i] < out[j] })
	return out
}
