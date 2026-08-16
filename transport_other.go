//go:build !darwin && !linux && !windows

// Fallback for platforms without a USB backend: the package compiles (StubTransport and the
// hardware-free tests work), and every hardware entry point reports errNoBackend.

package astrocam

import "errors"

// errNoBackend is returned by OpenHost/OpenLocation/Enumerate on platforms without a USB backend
// (anything but darwin, linux, windows); the package still compiles there for the hardware-free
// tests and StubTransport.
var errNoBackend = errors.New("astrocam: no USB backend on this platform (darwin, linux, windows only)")

// OpenHost reports that this platform has no USB backend.
func OpenHost(vid, pid uint16) (Transport, error) { return nil, errNoBackend }

// OpenLocation reports that this platform has no USB backend.
func OpenLocation(vid uint16, loc uint32) (Transport, error) { return nil, errNoBackend }

// enumerateRaw reports that this platform has no USB backend.
func enumerateRaw(vid uint16) ([]DeviceInfo, error) { return nil, errNoBackend }
