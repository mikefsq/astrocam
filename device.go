package astrocam

import (
	"errors"
	"fmt"
	"strings"
)

// DeviceInfo describes one attached camera discovered on the USB bus without opening it. The
// factory serial is absent: reading it means opening the device (OpenSerial /
// Camera.SerialNumber).
type DeviceInfo struct {
	VID, PID uint16
	Name     string // USB product-name string, e.g. "ASI6200MC Pro"
	Location uint32 // platform USB location id, stable per physical port; pass to OpenLocation
	// Attachment identifies this plugging-in of the device: the OS assigns a new value every
	// time the device enumerates, so a camera unplugged and replugged at the same port keeps
	// its Location and gets a new Attachment. Compare it to the open handle's (Camera.Attachment)
	// to tell continued presence from a replug that left the handle dead. macOS: the IORegistry
	// entry id; Linux: busnum/devnum, the same value as Location; 0 where the platform offers
	// no such identity (Windows), in which case only Location can be compared.
	Attachment uint64
}

func (d DeviceInfo) String() string {
	return fmt.Sprintf("%04x:%04x %-16q loc=0x%08x", d.VID, d.PID, d.Name, d.Location)
}

// Enumerate lists attached cameras (across every known vendor VID) that map to a registered
// camera model, without opening any of them. Devices that share a vendor id but are not cameras
// (e.g. EFW filter wheels) are filtered out. An error is returned only when no vendor could be
// scanned at all.
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

// filterCameras keeps only the raw USB devices whose PID resolves to a registered camera Model
// and fills a missing Name from the registry.
func filterCameras(raw []DeviceInfo) []DeviceInfo {
	out := make([]DeviceInfo, 0, len(raw))
	for _, d := range raw {
		m, ok := Lookup(d.VID, d.PID)
		if !ok {
			continue // not a camera
		}
		if d.Name == "" {
			d.Name = m.Name // the USB string was unreadable
		}
		out = append(out, d)
	}
	return out
}

// readSerial reads the factory serial off a Transport (the vendor's SerialNumber request, 0xC8
// on ZWO and 0xA3 on PlayerOne) without binding a full Camera.
func readSerial(t Transport, vid uint16) (Serial, error) {
	v, ok := VendorOf(vid)
	if !ok || v.Cmds.SerialNumber == 0 {
		return "", fmt.Errorf("astrocam: serial-number request not decoded for VID 0x%04x", vid)
	}
	raw := make([]byte, v.Cmds.serialBytes())
	if _, err := t.ControlIn(v.Cmds.SerialNumber, 0, 0, raw); err != nil {
		return "", err
	}
	return decodeSerial(raw, v.Cmds.SerialASCII), nil
}

// OpenSerial finds and opens the attached camera whose factory serial matches (hex, as
// Serial.String renders it): it enumerates, opens each candidate by location, reads its serial,
// and returns the match, closing the others. Errors if no attached camera has that serial.
func OpenSerial(serial string) (Transport, DeviceInfo, error) {
	devs, err := Enumerate()
	if err != nil {
		return nil, DeviceInfo{}, err
	}
	for _, d := range devs {
		t, err := OpenLocation(d.VID, d.Location)
		if err != nil {
			continue // busy or vanished: try the next candidate
		}
		// EqualFold: PlayerOne burns the serial as uppercase ASCII while ZWO
		// renders raw bytes as lowercase hex, so an exact compare made a
		// PlayerOne camera unmatchable from a config value in any other case
		// (measured: CAMGF... bound, camgf... did not).
		if sn, err := readSerial(t, d.VID); err == nil && strings.EqualFold(sn.String(), serial) {
			return t, d, nil
		}
		t.Close()
	}
	return nil, DeviceInfo{}, fmt.Errorf("astrocam: no attached camera with serial %s", serial)
}
