//go:build linux

// Pure-Go Linux USB transport over usbfs (/dev/bus/usb), no cgo: control transfers
// and bulk-IN frame reads driven directly through usbfs ioctls (the same kernel path
// libusb uses).
//
// HARDWARE-VALIDATED against real cameras — ASI174MM Mini (USB2) and ASI6200MM Pro
// (USB3, full 122 MB frames): serial-bind enumeration plus capture. Frame reads go
// through fixed 1 MiB usbfs bulk transfers assembled to a contiguous watermark
// (BulkRead / ReadFrameStream), for usbfs-specific reasons a single whole-frame ioctl
// gets wrong: usbfs kmallocs a contiguous kernel buffer per transfer (so a multi-MB
// read fails with ENOMEM), and the FX3's mid-frame short packets / ZLPs must be read
// THROUGH rather than taken as the frame end. Needs a udev rule granting access to the
// ZWO VID 0x03C3 (see deploy/).

package astrocam

import (
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"sync"
	"syscall"
	"time"
	"unsafe"
)

// usbfs ioctl encoding (asm-generic _IOC, 64-bit). dir: 0 none, 1 write, 2 read,
// 3 read+write. layout: (dir<<30)|(size<<16)|(type<<8)|nr, type='U' (0x55).
func ioc(dir, nr, size uintptr) uintptr { return dir<<30 | size<<16 | 0x55<<8 | nr }

var (
	usbdevfsControl        = ioc(3, 0, 24) // _IOWR('U',0,usbdevfs_ctrltransfer)
	usbdevfsBulk           = ioc(3, 2, 24) // _IOWR('U',2,usbdevfs_bulktransfer)
	usbdevfsClaimInterface = ioc(2, 15, 4) // _IOR('U',15,uint)
	usbdevfsClearHalt      = ioc(2, 21, 4) // _IOR('U',21,uint)
)

// Kernel structs (64-bit layout from <linux/usbdevice_fs.h>).
type usbCtrlTransfer struct {
	bRequestType uint8
	bRequest     uint8
	wValue       uint16
	wIndex       uint16
	wLength      uint16
	timeout      uint32
	_            uint32 // pad to 8-align the pointer
	data         uintptr
}

type usbBulkTransfer struct {
	ep      uint32
	len     uint32
	timeout uint32
	_       uint32 // pad
	data    uintptr
}

// usbfsDevice is a usbfs-backed Transport for one open camera.
type usbfsDevice struct {
	f *os.File
	// ctrlMu serializes control transfers — the capture path and the TEC loop drive
	// one open camera concurrently. Bulk reads (EP 0x81) are a separate pipe and stay
	// unlocked so a long readout can't stall a TEC tick. See transport_darwin.go.
	ctrlMu sync.Mutex
}

// usbNode is one /dev/bus/usb device node and the VID/PID read from its descriptor
// head — the cheap identity a scan reads without claiming the device.
type usbNode struct {
	path     string
	bus, dev int
	vid, pid uint16
}

// location packs busnum/devnum into the platform location id used by DeviceInfo and
// OpenLocation. On Linux this is stable only while the device stays attached: a replug
// reassigns devnum, after which the acquire loop re-binds by serial (OpenSerial) and
// picks up the new id. (macOS uses a topology-derived locationID that survives replug;
// matching that on Linux would mean parsing the sysfs port path — not needed here, since
// the serial is the durable key and the location is just the per-session open handle.)
func (n usbNode) location() uint32 { return uint32(n.bus)<<16 | uint32(n.dev&0xffff) }

// busDevFromPath parses the bus and device numbers out of a /dev/bus/usb/BBB/DDD path.
func busDevFromPath(p string) (bus, dev int, err error) {
	if dev, err = strconv.Atoi(filepath.Base(p)); err != nil {
		return 0, 0, err
	}
	bus, err = strconv.Atoi(filepath.Base(filepath.Dir(p)))
	return bus, dev, err
}

// scanUSB lists every /dev/bus/usb node whose device-descriptor VID matches vid (and
// PID, when pid != 0), reading the 18-byte descriptor head read-only — no claim, so it
// is safe to run against a camera another process has open. Unreadable nodes (permission
// / vanished) are skipped.
func scanUSB(vid, pid uint16) []usbNode {
	nodes, _ := filepath.Glob("/dev/bus/usb/*/*")
	var out []usbNode
	for _, p := range nodes {
		f, err := os.OpenFile(p, os.O_RDONLY, 0)
		if err != nil {
			continue // permission / busy — skip
		}
		var desc [18]byte // device descriptor is the head of the node
		_, err = f.ReadAt(desc[:], 0)
		f.Close()
		if err != nil {
			continue
		}
		gotVID := uint16(desc[8]) | uint16(desc[9])<<8
		gotPID := uint16(desc[10]) | uint16(desc[11])<<8
		if gotVID != vid || (pid != 0 && gotPID != pid) {
			continue
		}
		bus, dev, err := busDevFromPath(p)
		if err != nil {
			continue
		}
		out = append(out, usbNode{path: p, bus: bus, dev: dev, vid: gotVID, pid: gotPID})
	}
	return out
}

// openNodeAndClaim opens one /dev/bus/usb node read-write and claims interface 0,
// returning the Transport plus the descriptor's VID/PID for verification.
func openNodeAndClaim(path string) (*usbfsDevice, uint16, uint16, error) {
	f, err := os.OpenFile(path, os.O_RDWR, 0)
	if err != nil {
		return nil, 0, 0, fmt.Errorf("asicam: open %s: %w (check udev permissions for the ZWO VID)", path, err)
	}
	var desc [18]byte // device descriptor is the head of the node
	if _, err := f.ReadAt(desc[:], 0); err != nil {
		f.Close()
		return nil, 0, 0, err
	}
	vid := uint16(desc[8]) | uint16(desc[9])<<8
	pid := uint16(desc[10]) | uint16(desc[11])<<8
	d := &usbfsDevice{f: f}
	if err := d.claim(0); err != nil {
		f.Close()
		return nil, 0, 0, fmt.Errorf("asicam: claim interface 0 on %s: %w", path, err)
	}
	return d, vid, pid, nil
}

// OpenUSBFS finds the first device matching vid/pid under /dev/bus/usb, claims
// interface 0, and returns a Transport. The ZWO VID is astrocam.ZWO.VID (0x03C3).
func OpenUSBFS(vid, pid uint16) (*usbfsDevice, error) {
	var lastErr error
	for _, n := range scanUSB(vid, pid) {
		d, _, _, err := openNodeAndClaim(n.path)
		if err != nil {
			lastErr = err // device is there but couldn't be opened/claimed — surface why
			continue
		}
		return d, nil
	}
	if lastErr != nil {
		return nil, lastErr
	}
	return nil, fmt.Errorf("asicam: no usbfs device for %04x:%04x (check udev permissions)", vid, pid)
}

func (d *usbfsDevice) ioctl(req uintptr, arg unsafe.Pointer) error {
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.f.Fd(), req, uintptr(arg))
	if errno != 0 {
		return errno
	}
	return nil
}

func (d *usbfsDevice) claim(iface uint32) error {
	return d.ioctl(usbdevfsClaimInterface, unsafe.Pointer(&iface))
}

func (d *usbfsDevice) control(reqType, bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	d.ctrlMu.Lock()
	defer d.ctrlMu.Unlock()
	ct := usbCtrlTransfer{
		bRequestType: reqType, bRequest: bRequest,
		wValue: wValue, wIndex: wIndex, wLength: uint16(len(data)),
		timeout: 500,
	}
	if len(data) > 0 {
		ct.data = uintptr(unsafe.Pointer(&data[0]))
	}
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.f.Fd(), usbdevfsControl, uintptr(unsafe.Pointer(&ct)))
	if errno != 0 {
		return 0, errno
	}
	return len(data), nil
}

func (d *usbfsDevice) ControlOut(bRequest uint8, wValue, wIndex uint16, data []byte) error {
	_, err := d.control(0x40, bRequest, wValue, wIndex, data)
	return err
}

func (d *usbfsDevice) ControlIn(bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	return d.control(0xC0, bRequest, wValue, wIndex, data)
}

// maxBulkChunk caps a single USBDEVFS_BULK transfer. usbfs kmallocs a physically
// contiguous kernel buffer per ioctl, so a whole-frame read (several MB) fails with
// ENOMEM above the kernel's per-allocation limit (~4 MB, lower under fragmentation).
// 1 MiB matches the SDK/darwin xferLen and stays well clear of it.
const maxBulkChunk = 1 << 20

// bulkOne issues one USBDEVFS_BULK read of up to len(buf) bytes and returns the
// count actually transferred (which is < len(buf) when the device ends with a short
// packet).
func (d *usbfsDevice) bulkOne(buf []byte, timeoutMs uint32) (int, error) {
	bt := usbBulkTransfer{ep: bulkEndpoint, len: uint32(len(buf)), timeout: timeoutMs}
	if len(buf) > 0 {
		bt.data = uintptr(unsafe.Pointer(&buf[0]))
	}
	r, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.f.Fd(), usbdevfsBulk, uintptr(unsafe.Pointer(&bt)))
	if errno != 0 {
		return 0, errno
	}
	return int(r), nil
}

// BulkRead reads one frame from the bulk-IN endpoint into buf. It issues fixed,
// maxBulkChunk-sized reads into a scratch buffer and copies each to a running
// watermark in buf, for three reasons the obvious single whole-buffer ioctl gets
// wrong on usbfs:
//
//   - Allocation: usbfs kmallocs a contiguous kernel buffer per ioctl, so a multi-MB
//     whole-frame request fails with ENOMEM. 1 MiB stays under the kernel limit.
//   - Mid-frame short packets: the FX3 ends each DDR buffer with a short packet that
//     is NOT the frame end. Breaking on it (or reading to a fixed per-chunk offset)
//     truncates the frame; the watermark copy keeps it contiguous and we read on.
//   - Overflow: a request whose length is not a multiple of the endpoint max-packet
//     size overflows (EOVERFLOW) when the device delivers a full packet into the
//     short remainder. A full maxBulkChunk request is always packet-aligned, so the
//     incoming packet always fits.
//
// This mirrors the darwin async pump's windowed read; the difference from the prior
// single-ioctl version is why a whole-frame read worked on macOS but not here.
func (d *usbfsDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	deadline := time.Now().Add(timeout)
	scratch := make([]byte, maxBulkChunk)
	total := 0
	for total < len(buf) {
		ms := time.Until(deadline).Milliseconds()
		if ms <= 0 {
			break // out of time; return what arrived (caller validates by frame size)
		}
		n, err := d.bulkOne(scratch, uint32(ms))
		if n > 0 {
			total += copy(buf[total:], scratch[:n])
		}
		if err != nil {
			return total, err
		}
		if n == 0 {
			break // no more data = frame end
		}
	}
	return total, nil
}

// ReadFrameStream reads one whole frame, treating the FX3's mid-frame short packets
// and zero-length packets as NON-terminal (the DDR cameras — 6200/585 — hold the
// frame's final partial buffer until FPGABufReload flushes it, which the worker
// pulses on a ticker for the duration of this call). It keeps issuing full
// maxBulkChunk reads into scratch and copying to a watermark until the frame is in,
// `idle` passes with no data at all (a genuine stall), or the `total` deadline hits.
// usbfsDevice satisfying FrameStreamer is what makes StreamFrame use this instead of
// the generic BulkStreamer loop, which (correctly for darwin) stops on the first ZLP.
func (d *usbfsDevice) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	deadline := time.Now().Add(total)
	scratch := make([]byte, maxBulkChunk)
	got := 0
	lastData := time.Now()
	for got < len(buf) {
		rem := time.Until(deadline)
		if rem <= 0 {
			break // overall deadline
		}
		ms := rem
		if ms > idle {
			ms = idle
		}
		n, err := d.bulkOne(scratch, uint32(ms.Milliseconds()))
		if n > 0 {
			got += copy(buf[got:], scratch[:n])
			lastData = time.Now()
			continue
		}
		// No data this read (per-read timeout or a ZLP). Not necessarily the end —
		// keep cycling until `idle` elapses with no data at all.
		if errno, ok := err.(syscall.Errno); err != nil && (!ok || (errno != syscall.ETIMEDOUT && errno != syscall.ETIME)) {
			return got, err // a real pipe/device error, not a timeout
		}
		if time.Since(lastData) >= idle {
			break // genuine stall: no data for the whole idle window
		}
	}
	return got, nil
}

// ResetEndpoint clears a halt/stall on the bulk-IN endpoint (USBDEVFS_CLEAR_HALT).
// usbfsDevice satisfying EndpointResetter is what makes the capture worker's
// ResetEndpoint() (issued before each frame) actually clear the pipe — it was a
// silent no-op on Linux (the method was missing), so a stale pipe state from a prior
// aborted/partial read carried into the next transfer as the intermittent EPROTO.
func (d *usbfsDevice) ResetEndpoint(ep uint8) error {
	e := uint32(ep)
	return d.ioctl(usbdevfsClearHalt, unsafe.Pointer(&e))
}

func (d *usbfsDevice) Close() error { return d.f.Close() }

// OpenHost opens the default USB backend for this platform (Linux: usbfs).
func OpenHost(vid, pid uint16) (Transport, error) {
	d, err := OpenUSBFS(vid, pid)
	if err != nil {
		return nil, err
	}
	return d, nil
}

// enumerateRaw lists every VID-matched USB device under /dev/bus/usb WITHOUT opening
// it (the shared Enumerate filters these to known camera PIDs). Name is left empty —
// the kernel's product string isn't read here, and filterCameras fills it from the
// model registry. Location is busnum/devnum, the key OpenLocation reopens by.
func enumerateRaw(vid uint16) ([]DeviceInfo, error) {
	var out []DeviceInfo
	for _, n := range scanUSB(vid, 0) {
		out = append(out, DeviceInfo{VID: n.vid, PID: n.pid, Location: n.location()})
	}
	return out, nil
}

// OpenLocation opens the device at a specific busnum/devnum location (from a DeviceInfo)
// and claims its bulk interface — the way to bind the exact unit chosen from Enumerate.
// The descriptor VID is re-checked because devnum can be reused for a different device
// after a replug between enumerate and open.
func OpenLocation(vid uint16, loc uint32) (Transport, error) {
	bus, dev := int(loc>>16), int(loc&0xffff)
	path := fmt.Sprintf("/dev/bus/usb/%03d/%03d", bus, dev)
	d, gotVID, _, err := openNodeAndClaim(path)
	if err != nil {
		return nil, err
	}
	if gotVID != vid {
		d.Close()
		return nil, fmt.Errorf("asicam: device at %s is now %04x, not %04x (replugged?)", path, gotVID, vid)
	}
	return d, nil
}
