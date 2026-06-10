package asicam

import "sort"

// DeviceID is a USB vendor:product identifier — the vendor-independent key the
// camera-model registry is keyed on. Two vendors can reuse the same PID number, so
// the VID must be part of the key; (VID,PID) together name exactly one product.
type DeviceID struct{ VID, PID uint16 }

// Vendor describes a camera maker built on the shared FX3 + Sony-sensor platform:
// its USB vendor id and the wire-protocol dialect its cameras speak. Sensor profiles
// are vendor-independent (keyed by silicon); a Vendor supplies only the Regmap dialect
// (the control-transfer opcodes) and — later — the per-vendor gain/offset tuning. Each
// USB-IF VID maps to exactly one Vendor.
type Vendor struct {
	VID  uint16
	Name string
	// newRegmap builds this vendor's Regmap dialect over an open Transport. bus selects
	// the sensor-register path (Sony I2C vs generic camera reg); mode carries the live
	// readout context (USB speed, output depth, FPS%).
	newRegmap func(t Transport, bus RegBus, mode ReadoutMode) Regmap
}

// vendors maps a USB VID to the Vendor that owns it. Protocol layers register
// themselves from init() (ZWO in protocol.go), so the core never hardcodes a vendor.
var vendors = map[uint16]*Vendor{}

// RegisterVendor records a vendor descriptor under its VID.
func RegisterVendor(v *Vendor) { vendors[v.VID] = v }

// VendorOf returns the vendor that owns a USB VID, if one is registered.
func VendorOf(vid uint16) (*Vendor, bool) { v, ok := vendors[vid]; return v, ok }

// KnownVIDs returns, sorted, the USB vendor ids the driver knows how to enumerate and
// talk to — the set Enumerate scans across.
func KnownVIDs() []uint16 {
	out := make([]uint16, 0, len(vendors))
	for vid := range vendors {
		out = append(out, vid)
	}
	sort.Slice(out, func(i, j int) bool { return out[i] < out[j] })
	return out
}
