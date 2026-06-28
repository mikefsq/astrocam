//go:build linux

// Pure-Go Linux USB transport over usbfs (/dev/bus/usb), no cgo: control transfers
// and bulk-IN frame reads driven directly through usbfs ioctls.
//
// Frame reads use fixed 1 MiB usbfs bulk transfers: usbfs kmallocs a contiguous kernel
// buffer per transfer (so a multi-MB read fails with ENOMEM), and the FX3's mid-frame
// short packets / ZLPs must be read through rather than taken as the frame end. Two read
// paths: BulkRead uses the async-URB API (SUBMITURB/REAPURB), posting all of a frame's
// transfers up front with the last sized to the exact remainder — the only way the FX3
// releases its held final partial DMA buffer (the >4 s single-shot TRIGGER tail);
// ReadFrameStream uses the synchronous windowed-read loop with a FPGABufReload tail-flush
// for the USB3 DDR parts. Needs a udev rule granting access to the ZWO VID 0x03C3 (see deploy/).

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
	usbdevfsControl        = ioc(3, 0, 24)  // _IOWR('U',0,usbdevfs_ctrltransfer)
	usbdevfsBulk           = ioc(3, 2, 24)  // _IOWR('U',2,usbdevfs_bulktransfer)
	usbdevfsSubmitURB      = ioc(2, 10, 56) // _IOR('U',10,usbdevfs_urb)  = 0x8038550a (64-bit)
	usbdevfsDiscardURB     = ioc(0, 11, 0)  // _IO('U',11)                = 0x0000550b
	usbdevfsReapURB        = ioc(1, 12, 8)  // _IOW('U',12,void*)         = 0x4008550c (blocking)
	usbdevfsReapURBNDelay  = ioc(1, 13, 8)  // _IOW('U',13,void*)         = 0x4008550d (non-blocking)
	usbdevfsClaimInterface = ioc(2, 15, 4)  // _IOR('U',15,uint)
	usbdevfsClearHalt      = ioc(2, 21, 4)  // _IOR('U',21,uint)
	usbdevfsReset          = ioc(0, 20, 0)  // _IO('U',20)                = 0x00005514
)

// usbURB is the 64-bit <linux/usbdevice_fs.h> struct usbdevfs_urb (56 bytes, no trailing
// iso packet descriptors), posted and reaped by the async SUBMITURB/REAPURB path.
type usbURB struct {
	typ          uint8
	endpoint     uint8
	_            [2]byte
	status       int32
	flags        uint32
	buffer       uintptr
	bufferLength int32
	actualLength int32
	startFrame   int32
	numberOfPkts int32 // union with stream_id
	errorCount   int32
	signr        uint32
	usercontext  uintptr
}

const urbTypeBulk = 3 // USBDEVFS_URB_TYPE_BULK

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
	// ioMu serializes every USB I/O the FX3 bridge must not see interleaved: all control
	// transfers AND a whole-frame BulkRead hold it. A control transfer (a TEC poll, or a
	// client's CCDTemperature → 0xB3) landing on the bridge mid-readout hard-wedges the
	// un-buffered USB2 path (ASI174) — it parks the GPIF and takes the control plane down
	// with it (EP0 then times out, -110), recoverable only by a device reset + re-Init.
	// bulkOne (ReadFrameStream — the USB3/6200 DDR path) deliberately does NOT take ioMu:
	// that path requires its worker's concurrent FPGABufReload, and its DDR frame buffer
	// makes the interleave harmless.
	ioMu sync.Mutex
}

// usbNode is one /dev/bus/usb device node and the VID/PID read from its descriptor
// head — the identity a scan reads without claiming the device.
type usbNode struct {
	path     string
	bus, dev int
	vid, pid uint16
}

// location packs busnum/devnum into the platform location id used by DeviceInfo and
// OpenLocation. Stable only while the device stays attached: a replug reassigns devnum,
// after which the acquire loop re-binds by serial (OpenSerial) and picks up the new id.
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
// is safe against a camera another process has open. Unreadable nodes are skipped.
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
	d.ioMu.Lock()
	defer d.ioMu.Unlock()
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
// 1 MiB matches the SDK/darwin xferLen.
const maxBulkChunk = 1 << 20

// bulkOne issues one USBDEVFS_BULK read of up to len(buf) bytes and returns the
// count actually transferred (which is < len(buf) when the device ends with a short
// packet). Used by ReadFrameStream (the synchronous windowed pump); BulkRead uses the
// async-URB path instead — see readFrameURBs.
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

// bulkTrace, when ASTROCAM_BULK_TRACE is set, logs one line per submitted/reaped urb to
// stderr (held-tail / short-packet timing). Diagnostic only — off by default.
var bulkTrace = os.Getenv("ASTROCAM_BULK_TRACE") != ""

// BulkRead reads one whole frame from the bulk-IN endpoint into buf via the async-URB pump
// (readFrameURBs), which gets the FX3's held >4 s TRIGGER tail. Holds ioMu for the whole
// frame so no control transfer can interleave with the readout (see usbfsDevice.ioMu). This
// is the un-buffered USB2 path; the USB3 DDR path reads through bulkOne/ReadFrameStream.
func (d *usbfsDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	d.ioMu.Lock()
	defer d.ioMu.Unlock()
	return d.readFrameURBs(buf, timeout)
}

// readFrameURBs reads one whole frame through the usbfs async urb API. It submits
// ceil(len/chunk) bulk-IN transfers up front (USBDEVFS_SUBMITURB) — each into its slice of
// buf at a fixed offset, the last sized to the exact remainder, not a padded full chunk —
// then reaps them in completion order (USBDEVFS_REAPURBNDELAY) and counts contiguous bytes
// up to the first short or failed transfer (the FX3 ends a frame with a short packet).
//
// The exact-remainder final transfer is required for the >4 s single-shot TRIGGER tail: the
// FX3 commits whole 1-MiB DMA buffers and holds the frame's final partial buffer; asked for
// a full 1 MiB it cannot fill, it keeps holding. Requesting exactly the pending tail bytes
// makes it deliver. usbfs reports a short bulk-IN as status 0 with actual_length <
// buffer_length (no error), so a short transfer marks the frame end.
func (d *usbfsDevice) readFrameURBs(buf []byte, timeout time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	start := time.Now()
	deadline := start.Add(timeout)
	n := (len(buf) + maxBulkChunk - 1) / maxBulkChunk
	urbs := make([]usbURB, n)
	submitted := make([]bool, n)
	done := make([]bool, n)
	slotOf := func(p uintptr) int {
		for i := range urbs {
			if p == uintptr(unsafe.Pointer(&urbs[i])) {
				return i
			}
		}
		return -1
	}
	// On any exit, discard + blocking-drain every submitted-but-unreaped urb so no
	// completion lands against buf after we return.
	defer func() {
		for i := range submitted {
			if submitted[i] && !done[i] {
				_ = d.ioctl(usbdevfsDiscardURB, unsafe.Pointer(&urbs[i]))
			}
		}
		for {
			pending := false
			for i := range submitted {
				if submitted[i] && !done[i] {
					pending = true
				}
			}
			if !pending {
				break
			}
			var p uintptr
			if err := d.ioctl(usbdevfsReapURB, unsafe.Pointer(&p)); err != nil {
				break
			}
			if i := slotOf(p); i >= 0 {
				done[i] = true
			}
		}
	}()

	for i := 0; i < n; i++ {
		off := i * maxBulkChunk
		l := maxBulkChunk
		if off+l > len(buf) {
			l = len(buf) - off // last transfer: the exact remainder (see doc)
		}
		urbs[i] = usbURB{typ: urbTypeBulk, endpoint: bulkEndpoint,
			buffer: uintptr(unsafe.Pointer(&buf[off])), bufferLength: int32(l)}
		if err := d.ioctl(usbdevfsSubmitURB, unsafe.Pointer(&urbs[i])); err != nil {
			if i == 0 {
				return 0, err // couldn't post a single transfer — fatal
			}
			break // submit what we can; the reaped prefix is still a valid frame
		}
		submitted[i] = true
	}
	if bulkTrace {
		fmt.Fprintf(os.Stderr, "[urb] start want=%d n=%d timeout=%s\n", len(buf), n, timeout)
	}

	// Reap in completion order (== submission order on a single pipe) until every submitted
	// transfer is in or the deadline hits.
	for {
		pending := false
		for i := range submitted {
			if submitted[i] && !done[i] {
				pending = true
			}
		}
		if !pending || time.Now().After(deadline) {
			break
		}
		var p uintptr
		err := d.ioctl(usbdevfsReapURBNDelay, unsafe.Pointer(&p))
		if err == syscall.EAGAIN {
			time.Sleep(200 * time.Microsecond)
			continue
		}
		if err != nil {
			return 0, err
		}
		if i := slotOf(p); i >= 0 {
			done[i] = true
			if bulkTrace {
				fmt.Fprintf(os.Stderr, "[urb] t=%.3fs slot=%d n=%d status=%d\n",
					time.Since(start).Seconds(), i, urbs[i].actualLength, urbs[i].status)
			}
		}
	}

	// Assemble: contiguous bytes in order, up to and including the frame-terminating short
	// transfer. A slot that never completed or failed (status != 0) truncates the frame.
	total := 0
	for i := 0; i < n; i++ {
		if !done[i] || urbs[i].status != 0 {
			break
		}
		req := maxBulkChunk
		if off := i * maxBulkChunk; off+req > len(buf) {
			req = len(buf) - off
		}
		total += int(urbs[i].actualLength)
		if int(urbs[i].actualLength) < req {
			break // short packet = end of frame
		}
	}
	if bulkTrace {
		fmt.Fprintf(os.Stderr, "[urb] done total=%d/%d elapsed=%.3fs\n", total, len(buf), time.Since(start).Seconds())
	}
	return total, nil
}

// ReadFrameStream reads one whole frame, treating the FX3's mid-frame short packets and
// zero-length packets as non-terminal (the DDR cameras — 6200/585 — hold the frame's final
// partial buffer until FPGABufReload flushes it, which the worker pulses on a ticker for the
// duration of this call). It keeps issuing full maxBulkChunk reads into scratch and copying
// to a watermark until the frame is in, `idle` passes with no data at all (a genuine stall),
// or the `total` deadline hits. usbfsDevice satisfies FrameStreamer.
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
func (d *usbfsDevice) ResetEndpoint(ep uint8) error {
	e := uint32(ep)
	return d.ioctl(usbdevfsClearHalt, unsafe.Pointer(&e))
}

// ResetDevice performs a USB port reset of the whole device (USBDEVFS_RESET) — last-resort
// recovery when the readout wedges. The reset drops the kernel's interface claim, so we
// re-claim interface 0 afterwards; the device keeps its address (no node change).
func (d *usbfsDevice) ResetDevice() error {
	if err := d.ioctl(usbdevfsReset, nil); err != nil {
		return err
	}
	return d.claim(0) // the reset releases the interface claim — re-take it
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

// enumerateRaw lists every VID-matched USB device under /dev/bus/usb without opening it
// (the shared Enumerate filters these to known camera PIDs). Name is left empty
// (filterCameras fills it from the model registry); Location is busnum/devnum, the key
// OpenLocation reopens by.
func enumerateRaw(vid uint16) ([]DeviceInfo, error) {
	var out []DeviceInfo
	for _, n := range scanUSB(vid, 0) {
		out = append(out, DeviceInfo{VID: n.vid, PID: n.pid, Location: n.location()})
	}
	return out, nil
}

// OpenLocation opens the device at a specific busnum/devnum location (from a DeviceInfo)
// and claims its bulk interface — binds the exact unit chosen from Enumerate. The
// descriptor VID is re-checked because devnum can be reused for a different device after a
// replug between enumerate and open.
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
