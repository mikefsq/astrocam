package astrocam

import (
	"errors"
	"fmt"
)

// DeviceInfo describes one attached camera discovered on the USB bus without opening it:
// VID/PID, the USB product-name string, and a platform location id. The factory serial is
// absent (no USB serial-number descriptor; reading it means opening the device — see
// OpenSerial / Camera.SerialNumber).
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

// Enumerate lists attached cameras (across every known vendor VID) that map to a registered
// camera model, without opening any of them. Devices that share a vendor id but aren't
// cameras (e.g. EFW filter wheels) are filtered out. Location binds a result to a physical
// port for OpenLocation. One vendor's scan failing does not hide the healthy vendors: the
// error is returned only when NO vendor could be scanned at all.
func Enumerate() ([]DeviceInfo, error) {
	var raw []DeviceInfo
	var errs []error
	for _, vid := range KnownVIDs() {
		devs, err := enumerateRaw(vid)
		if err != nil {
			errs = append(errs, fmt.Errorf("vid %04x: %w", vid, err))
			continue
		}
		raw = append(raw, devs...)
	}
	if len(raw) == 0 && len(errs) > 0 {
		return nil, errors.Join(errs...)
	}
	return filterCameras(raw), nil
}

// filterCameras keeps only the raw USB devices whose PID resolves to a registered camera
// Model (dropping non-camera devices on the same VID, e.g. EFW wheels) and fills a missing
// Name from the registry.
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

// readSerial reads the 8-byte factory serial off a Transport (the 0xC8 vendor-IN) without
// binding a full Camera.
func readSerial(t Transport) (Serial, error) {
	var s Serial
	if _, err := t.ControlIn(reqSerialNumber, 0, 0, s[:]); err != nil {
		return Serial{}, err
	}
	return s, nil
}

// OpenSerial finds and opens the attached camera whose factory serial matches (hex, as
// Serial.String renders it) — the stable per-unit key, since bus locations move on replug.
// It enumerates, opens each candidate by location, reads its serial, and returns the match
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
