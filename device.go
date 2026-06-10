package asicam

import (
	"errors"
	"fmt"
)

// DeviceInfo describes one attached ZWO camera discovered on the USB bus WITHOUT
// opening it — the cheap identity the host can read straight from the OS USB registry:
// VID/PID, the USB product-name string, and a platform location id. The factory serial
// is deliberately absent: these cameras carry no USB serial-number descriptor (it lives
// in SPI flash, behind the 0xC8 read decoded in GetSerialNumber), so reading it means
// opening the device — see OpenSerial / Camera.SerialNumber.
type DeviceInfo struct {
	VID, PID uint16
	Name     string // USB product-name string, e.g. "ASI6200MC Pro"
	Location uint32 // platform USB location id — stable per physical port; pass to OpenLocation
}

func (d DeviceInfo) String() string {
	return fmt.Sprintf("%04x:%04x %-16q loc=0x%08x", d.VID, d.PID, d.Name, d.Location)
}

// errEnumUnsupported is returned by the not-yet-implemented platform backends.
var errEnumUnsupported = errors.New("asicam: USB enumeration not implemented on this platform yet")

// Enumerate lists attached cameras (across every known vendor VID) that map to a
// registered camera model, without opening any of them. For each VID the driver knows
// (KnownVIDs), the platform backend (enumerateRaw) supplies every VID-matched USB
// device; this filters to (VID,PID) pairs that resolve to a registered camera Model,
// dropping other USB devices that share a vendor id — notably EFW filter wheels.
// Location is what binds a result to a physical port for OpenLocation.
func Enumerate() ([]DeviceInfo, error) {
	var raw []DeviceInfo
	for _, vid := range KnownVIDs() {
		devs, err := enumerateRaw(vid)
		if err != nil {
			return nil, err
		}
		raw = append(raw, devs...)
	}
	return filterCameras(raw), nil
}

// filterCameras keeps only the raw USB devices whose PID resolves to a registered camera
// Model (dropping non-camera ZWO devices on the same VID, e.g. EFW wheels) and fills a
// missing Name from the registry. Split out from Enumerate so it is testable without a
// platform backend.
func filterCameras(raw []DeviceInfo) []DeviceInfo {
	out := make([]DeviceInfo, 0, len(raw))
	for _, d := range raw {
		m, ok := Lookup(d.VID, d.PID)
		if !ok {
			continue // not a camera (e.g. an EFW wheel on the same VID)
		}
		if d.Name == "" {
			d.Name = m.Name // fall back to the registry name if the USB string was unreadable
		}
		out = append(out, d)
	}
	return out
}

// readSerial reads the 8-byte factory serial straight off a Transport — the 0xC8
// vendor-IN decoded in GetSerialNumber — without binding a full Camera (no model needed).
func readSerial(t Transport) (Serial, error) {
	var s Serial
	if _, err := t.ControlIn(reqSerialNumber, 0, 0, s[:]); err != nil {
		return Serial{}, err
	}
	return s, nil
}

// OpenSerial finds and opens the attached camera whose factory serial matches (hex, as
// Serial.String renders it). This is the stable bind an Alpaca device should persist:
// USB exposes no serial string and bus locations move when the camera is replugged or
// the hub topology changes, so the serial is the only durable per-unit key. It
// enumerates, opens each candidate by location, reads its serial, and returns the match
// (closing the others). Errors if no attached camera has that serial.
func OpenSerial(serial string) (Transport, DeviceInfo, error) {
	devs, err := Enumerate()
	if err != nil {
		return nil, DeviceInfo{}, err
	}
	for _, d := range devs {
		t, err := OpenLocation(d.VID, d.Location)
		if err != nil {
			continue // busy or vanished — try the next candidate
		}
		if sn, err := readSerial(t); err == nil && sn.String() == serial {
			return t, d, nil
		}
		t.Close()
	}
	return nil, DeviceInfo{}, fmt.Errorf("asicam: no attached camera with serial %s", serial)
}
