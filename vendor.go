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
}

// FX3Cmds holds a vendor's FX3 bridge request codes (bRequest values). SendCMD-style entries
// take no payload (vendor OUT, wValue 0). Zero = not decoded for the vendor.
type FX3Cmds struct {
	StreamStop      uint8 // SendCMD: stop/prepare before (re)arming
	StreamStart     uint8 // SendCMD: begin streaming
	Flush           uint8 // SendCMD: pipeline flush / drop recovery
	EnableGPIF32DQ  uint8 // vendor OUT, wValue 0/1 disables/enables the FPGA->FX3 32-bit data bus
	ReadSPIFlash    uint8 // vendor IN, wIndex = flash address >> 8, up to 2 KiB per transfer
	FirmwareVersion uint8 // vendor IN, 2 bytes little-endian
	SerialNumber    uint8 // vendor IN, 8 raw factory-serial bytes
	ST4On           uint8 // vendor OUT, wValue = guide direction (GuideDir), asserts the ST4 line
	ST4Off          uint8 // vendor OUT, wValue = guide direction, releases the ST4 line
	ReadTemp        uint8 // vendor IN, 2 bytes: sensor temperature (12-bit signed sixteenths)
	ReadHumidity    uint8 // vendor IN, 2 bytes: Sensirion RH raw; wValue = ReadHumidityWValue
	// ReadHumidityWValue is the wValue the ReadHumidity request carries (0xF5 on ZWO).
	ReadHumidityWValue uint16
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

// cmd resolves op against the table; 0 = not decoded.
func (c FX3Cmds) cmd(op FX3Op) uint8 {
	switch op {
	case FX3StreamStop:
		return c.StreamStop
	case FX3StreamStart:
		return c.StreamStart
	case FX3Flush:
		return c.Flush
	}
	return 0
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
