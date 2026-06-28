package astrocam

import "sort"

// DeviceID is a USB (VID,PID) identifier, the key the camera-model registry uses. The
// VID is part of the key because two vendors can reuse the same PID.
type DeviceID struct{ VID, PID uint16 }

// Vendor describes a camera maker on the shared FX3 + Sony-sensor platform: its USB
// vendor id and the wire-protocol dialect its cameras speak. Sensor profiles are
// vendor-independent; a Vendor supplies only the Regmap dialect. One VID per Vendor.
type Vendor struct {
	VID  uint16
	Name string
	// newRegmap builds this vendor's Regmap dialect over an open Transport. bus selects
	// the sensor-register path (Sony I2C vs generic camera reg); mode carries the live
	// readout context (USB speed, output depth, FPS%).
	newRegmap func(t Transport, bus RegBus, mode ReadoutMode) Regmap
}

// vendors maps a USB VID to the Vendor that owns it. Protocol layers register themselves
// from init(), so the core never hardcodes a vendor.
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
