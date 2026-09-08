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

// FoundCamera is one attached camera plus the identity that survives a replug.
type FoundCamera struct {
	DeviceInfo
	// Serial is the factory serial, or "" when it could not be read — the camera is open in
	// another process, or the vendor's serial request is not decoded. Absent rather than fatal:
	// a camera that cannot be identified is still attached, and a caller listing hardware needs
	// to show it.
	Serial string
}

// EnumerateWithSerials lists attached cameras and reads each one's factory serial, opening every
// camera briefly and closing it again.
//
// It exists because Enumerate alone cannot identify a camera across replugs: it reports VID, PID
// and Location, and Location is the physical PORT, so moving a camera to another socket changes
// it. The serial is the only stable identity, and reading it costs a transport open — which is
// why this is a separate call rather than something Enumerate does for everyone.
//
// A camera that is already open elsewhere is REPORTED WITHOUT ITS SERIAL, not skipped: the whole
// point of a listing is to show what is attached, and "in use" is not "absent".
func EnumerateWithSerials() ([]FoundCamera, error) {
	devs, err := Enumerate()
	if err != nil {
		return nil, err
	}
	out := make([]FoundCamera, 0, len(devs))
	for _, d := range devs {
		f := FoundCamera{DeviceInfo: d}
		if t, err := OpenLocation(d.VID, d.Location); err == nil {
			if sn, err := readSerial(t, d.VID); err == nil {
				f.Serial = sn.String()
			}
			t.Close()
		}
		out = append(out, f)
	}
	return out, nil
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
