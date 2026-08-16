//go:build linux

// Pure-Go Linux USB transport over usbfs (/dev/bus/usb), no cgo: control transfers
// and bulk-IN frame reads driven directly through usbfs ioctls.
//
// Frame reads use fixed 1 MiB usbfs bulk transfers: usbfs kmallocs a contiguous kernel
// buffer per transfer (so a multi-MB read fails with ENOMEM), and the FX3's mid-frame
// short packets / ZLPs must be read through rather than taken as the frame end. Two read
// paths: BulkRead uses the async-URB API (SUBMITURB/REAPURB), posting all of a frame's
// transfers up front with the last sized to the exact remainder, the only way the FX3
// releases its held final partial DMA buffer (the >4 s single-shot TRIGGER tail);
// ReadFrameStream uses the synchronous windowed-read loop with a FPGABufReload tail-flush
// for the USB3 DDR parts. Needs a udev rule granting access to the camera's USB VID (ZWO 0x03C3).

package astrocam

import (
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"strconv"
	"sync"
	"sync/atomic"
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
	usbdevfsReapURBNDelay  = ioc(1, 13, 8)  // _IOW('U',13,void*)         = 0x4008550d (non-blocking)
	usbdevfsClaimInterface = ioc(2, 15, 4)  // _IOR('U',15,uint)
	usbdevfsClearHalt      = ioc(2, 21, 4)  // _IOR('U',21,uint)
	usbdevfsReset          = ioc(0, 20, 0)  // _IO('U',20)                = 0x00005514
)

// debugPreq dumps per-URB completion timing/lengths from ReadFrameStreamPrequeued to stderr
// (bring-up diagnostic; set ASTROCAM_DEBUG_PREQ=1).
var debugPreq = os.Getenv("ASTROCAM_DEBUG_PREQ") != ""

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
	// closeMu is the Close interlock: every public I/O method holds it shared for the
	// duration of its I/O (including the URB drain), and Close takes it exclusively, so
	// Close waits for in-flight I/O instead of yanking the fd out from under a drain
	// (an EBADF mid-drain would abandon kernel-held pointers into the caller's buffer).
	// Lock order: closeMu (shared) BEFORE ioMu. All I/O is deadline-bounded, so Close is too.
	closeMu sync.RWMutex
	closed  bool // under closeMu
	// broken poisons the device after an undrainable readout (see drainURBs): the kernel may
	// still hold references into parked buffers, so no further I/O is allowed on this fd:
	// fail fast until the camera is re-opened.
	broken atomic.Bool
	// ioMu serializes every USB I/O the FX3 bridge must not see interleaved: all control
	// transfers AND a whole-frame BulkRead hold it. A control transfer (a TEC poll, or a
	// client's CCDTemperature → 0xB3) landing on the bridge mid-readout hard-wedges the
	// un-buffered USB2 path (ASI174): it parks the GPIF and takes the control plane down
	// with it (EP0 then times out, -110), recoverable only by a device reset + re-Init.
	// bulkOne (ReadFrameStream, the USB3/6200 DDR path) does NOT take ioMu:
	// that path requires its worker's concurrent FPGABufReload, and its DDR frame buffer
	// makes the interleave harmless.
	ioMu sync.Mutex
	// readAborted is the ReadAborter state (level-triggered): while set, in-flight frame
	// reads drain and return their short prefix within ~one poll interval, and new frame
	// reads fail fast, so StopExposure's master-stop writes are never stuck behind a
	// blocked read holding ioMu. Set by AbortRead (StopExposure), cleared by ArmRead
	// (StartExposure/StartVideo). Control transfers and ResetEndpoint are unaffected.
	readAborted atomic.Bool
	// superSpeed is the negotiated link speed (USB3 SuperSpeed bulk maxpacket ≥1024), read from the
	// endpoint descriptor at open. The readout's bandwidth budget follows it (see camera.go).
	superSpeed bool
}

// AbortRead / ArmRead implement ReadAborter (see transport.go). Level-triggered by design:
// a read issued by a stale worker just AFTER the abort must also fail fast, or it re-takes
// ioMu for a full read window against a sensor the abort already master-stopped.
func (d *usbfsDevice) AbortRead() { d.readAborted.Store(true) }
func (d *usbfsDevice) ArmRead()   { d.readAborted.Store(false) }

// usbNode is one /dev/bus/usb device node and the VID/PID read from its descriptor
// head, the identity a scan reads without claiming the device.
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
// PID, when pid != 0), reading the 18-byte descriptor head read-only (no claim), so it
// is safe against a camera another process has open. Unreadable nodes are skipped.
func scanUSB(vid, pid uint16) []usbNode {
	nodes, _ := filepath.Glob("/dev/bus/usb/*/*")
	var out []usbNode
	for _, p := range nodes {
		f, err := os.OpenFile(p, os.O_RDONLY, 0)
		if err != nil {
			continue // permission / busy: skip
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
		return nil, 0, 0, fmt.Errorf("astrocam: open %s: %w (check udev permissions for the ZWO VID)", path, err)
	}
	var desc [18]byte // device descriptor is the head of the node
	if _, err := f.ReadAt(desc[:], 0); err != nil {
		f.Close()
		return nil, 0, 0, err
	}
	vid := uint16(desc[8]) | uint16(desc[9])<<8
	pid := uint16(desc[10]) | uint16(desc[11])<<8
	d := &usbfsDevice{f: f}
	// Negotiated link speed from the bulk-IN endpoint's max packet size: USB3 SuperSpeed bulk =
	// 1024, USB2 HighSpeed = 512. The node read returns the device descriptor followed by the
	// active config/interface/endpoint descriptors, which reflect the speed actually negotiated,
	// so a USB3 camera on a HighSpeed link (e.g. through a Cynthion) correctly reads as USB2 and the
	// readout uses the USB2 bandwidth budget. Without this the readout runs at the USB3 line rate and
	// overruns the bus, shearing the frame.
	var raw [1024]byte
	if n, _ := f.ReadAt(raw[:], 0); n > 0 {
		d.superSpeed = bulkInMaxPacket(raw[:n], bulkEndpoint) >= 1024
	}
	if err := d.claim(0); err != nil {
		f.Close()
		return nil, 0, 0, fmt.Errorf("astrocam: claim interface 0 on %s: %w", path, err)
	}
	return d, vid, pid, nil
}

// bulkInMaxPacket walks the concatenated USB descriptors and returns the wMaxPacketSize of the
// endpoint with address ep (0 if not found). Endpoint descriptor: bLength, bDescriptorType=0x05,
// bEndpointAddress, bmAttributes, wMaxPacketSize(LE)...
func bulkInMaxPacket(b []byte, ep uint8) int {
	for i := 0; i+1 < len(b); {
		l := int(b[i])
		if l < 2 || i+l > len(b) {
			break
		}
		if b[i+1] == 0x05 && l >= 7 && b[i+2] == ep {
			return int(b[i+4]) | int(b[i+5])<<8
		}
		i += l
	}
	return 0
}

// SuperSpeed reports whether the bulk-IN endpoint negotiated USB3 SuperSpeed (≥1024-byte max
// packet); false = USB2 HighSpeed. The Camera readout follows this, not the model capability.
func (d *usbfsDevice) SuperSpeed() bool { return d.superSpeed }

// openUSBFS finds the first device matching vid/pid under /dev/bus/usb, claims
// interface 0, and returns the transport (OpenHost is the public entry).
func openUSBFS(vid, pid uint16) (*usbfsDevice, error) {
	var lastErr error
	for _, n := range scanUSB(vid, pid) {
		d, _, _, err := openNodeAndClaim(n.path)
		if err != nil {
			lastErr = err // device is there but couldn't be opened/claimed: surface why
			continue
		}
		return d, nil
	}
	if lastErr != nil {
		return nil, lastErr
	}
	return nil, fmt.Errorf("astrocam: no usbfs device for %04x:%04x (check udev permissions)", vid, pid)
}

func (d *usbfsDevice) ioctl(req uintptr, arg unsafe.Pointer) error {
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.f.Fd(), req, uintptr(arg))
	if errno != 0 {
		return errno
	}
	return nil
}

var (
	errTransportClosed = fmt.Errorf("astrocam: transport closed")
	errTransportBroken = fmt.Errorf("astrocam: transport poisoned by an undrainable readout (re-open to recover)")
)

// enter gates every public I/O method: fail fast on a closed or poisoned device, and hold
// the Close interlock (shared) for the duration of the I/O so Close cannot invalidate the fd
// under an in-flight transfer or drain. The returned release runs when the I/O is done.
func (d *usbfsDevice) enter() (release func(), err error) {
	if d.broken.Load() {
		return nil, errTransportBroken
	}
	d.closeMu.RLock()
	if d.closed {
		d.closeMu.RUnlock()
		return nil, errTransportClosed
	}
	return d.closeMu.RUnlock, nil
}

// drainTimeout bounds drainURBs' wait for discarded urbs to come back. A discard completes
// near-instantly on a live HCD; one that can't deliver within this window forfeits the
// DEVICE (park + poison) rather than the process (an unbounded wait would hold ioMu forever).
const drainTimeout = 5 * time.Second

// leakedIO pins memory the kernel may still write to (urbs that could not be reaped): a
// deliberate, bounded leak instead of a use-after-free when usbfs performs a late copyout.
var (
	leakedIOMu sync.Mutex
	leakedIO   []interface{}
)

// drainURBs discards and reaps every pending urb so no kernel copyout can land against the
// caller's memory after return: usbfs writes into buf and the urb structs at REAP time, so
// returning with an unreaped urb is a future memory corruption. EINTR and EAGAIN are retried
// (the Go runtime's SIGURG preemption interrupts ioctls routinely). If the urbs cannot all be reaped within drainTimeout, or the
// reap fails outright (e.g. ENODEV after a disconnect), the buffers are parked for the
// process lifetime and the device is poisoned: every later I/O fails fast until re-open,
// and the eventual fd close releases the kernel's urbs without copyout.
func (d *usbfsDevice) drainURBs(urbs []usbURB, pending []bool, buf []byte) {
	npend := 0
	for i := range pending {
		if pending[i] {
			_ = d.ioctl(usbdevfsDiscardURB, unsafe.Pointer(&urbs[i]))
			npend++
		}
	}
	if npend == 0 {
		return
	}
	forfeit := func() {
		leakedIOMu.Lock()
		leakedIO = append(leakedIO, urbs, buf)
		leakedIOMu.Unlock()
		d.broken.Store(true)
	}
	deadline := time.Now().Add(drainTimeout)
	for npend > 0 {
		var p uintptr
		err := d.ioctl(usbdevfsReapURBNDelay, unsafe.Pointer(&p))
		switch {
		case err == nil:
			for i := range urbs {
				if p == uintptr(unsafe.Pointer(&urbs[i])) {
					if pending[i] {
						pending[i] = false
						npend--
					}
					break
				}
			}
		case err == syscall.EAGAIN || err == syscall.EINTR:
			if time.Now().After(deadline) {
				forfeit()
				return
			}
			time.Sleep(time.Millisecond)
		default:
			forfeit()
			return
		}
	}
}

func (d *usbfsDevice) claim(iface uint32) error {
	return d.ioctl(usbdevfsClaimInterface, unsafe.Pointer(&iface))
}

func (d *usbfsDevice) control(reqType, bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
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
	// USBDEVFS_CONTROL's return value is the transferred byte count; report it, don't
	// fabricate len(data): a short control-IN (e.g. the SPI flash read hitting the blob
	// end) must be visible to callers that terminate on `got < want`.
	r1, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.f.Fd(), usbdevfsControl, uintptr(unsafe.Pointer(&ct)))
	runtime.KeepAlive(data) // ct.data holds a uintptr into data across the syscall
	if errno != 0 {
		return 0, errno
	}
	return int(r1), nil
}

func (d *usbfsDevice) ControlOut(bRequest uint8, wValue, wIndex uint16, data []byte) error {
	_, err := d.control(0x40, bRequest, wValue, wIndex, data)
	return err
}

// ControlOutUngated implements UngatedControlSender (ST4 guide pulses ONLY, see
// transport.go): the vendor OUT is issued WITHOUT taking ioMu, so a pulse edge cannot
// queue behind a whole-frame read. The SDK does the same (its capture thread never
// holds the API mutex); usbfs control ioctls are independently safe per call. The Close
// interlock and broken poison still apply.
func (d *usbfsDevice) ControlOutUngated(bRequest uint8, wValue, wIndex uint16) error {
	release, err := d.enter()
	if err != nil {
		return err
	}
	defer release()
	ct := usbCtrlTransfer{
		bRequestType: 0x40, bRequest: bRequest,
		wValue: wValue, wIndex: wIndex,
		timeout: 500,
	}
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.f.Fd(), usbdevfsControl, uintptr(unsafe.Pointer(&ct)))
	if errno != 0 {
		return errno
	}
	return nil
}

func (d *usbfsDevice) ControlIn(bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	return d.control(0xC0, bRequest, wValue, wIndex, data)
}

// maxBulkChunk caps a single USBDEVFS_BULK transfer. usbfs kmallocs a physically
// contiguous kernel buffer per ioctl, so a whole-frame read (several MB) fails with
// ENOMEM above the kernel's per-allocation limit (~4 MB, lower under fragmentation).
// 1 MiB matches the SDK/darwin xferLen.
const maxBulkChunk = 1 << 20

// urbWindow caps the async-URB paths' in-flight transfers: usbfs bounds the outstanding
// URB buffer memory it will pin per fd (16 MiB default), so 12×1 MiB leaves headroom.
const urbWindow = 12

// bulkOne issues one USBDEVFS_BULK read of up to len(buf) bytes and returns the
// count actually transferred (which is < len(buf) when the device ends with a short
// packet). Used by ReadFrameStream (the synchronous windowed pump); BulkRead uses the
// async-URB path instead; see readFrameURBs.
func (d *usbfsDevice) bulkOne(buf []byte, timeoutMs uint32) (int, error) {
	bt := usbBulkTransfer{ep: bulkEndpoint, len: uint32(len(buf)), timeout: timeoutMs}
	if len(buf) > 0 {
		bt.data = uintptr(unsafe.Pointer(&buf[0]))
	}
	r, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.f.Fd(), usbdevfsBulk, uintptr(unsafe.Pointer(&bt)))
	runtime.KeepAlive(buf) // bt.data holds a uintptr into buf across the syscall
	if errno != 0 {
		return 0, errno
	}
	return int(r), nil
}

// bulkTrace, when ASTROCAM_BULK_TRACE is set, logs one line per submitted/reaped urb to
// stderr (held-tail / short-packet timing). Diagnostic only; off by default.
var bulkTrace = os.Getenv("ASTROCAM_BULK_TRACE") != ""

// BulkRead reads one whole frame from the bulk-IN endpoint into buf via the async-URB pump
// (readFrameURBs), which gets the FX3's held >4 s TRIGGER tail. Holds ioMu for the whole
// frame so no control transfer can interleave with the readout (see usbfsDevice.ioMu). This
// is the un-buffered USB2 path; the USB3 DDR path reads through bulkOne/ReadFrameStream.
func (d *usbfsDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
	return d.readFrameURBs(buf, timeout, 0)
}

// BulkReadQuiet implements QuietBulkReader: like BulkRead, but the first `quiet` of the
// read is a host-timed integration window during which ioMu stays RELEASED (the sensor is
// exposing, so no data can arrive, but the transfers are already armed so the GPIF never
// streams without a reader). The gate engages at quiet-elapsed or first data, whichever
// comes first. timeout still bounds the WHOLE call, quiet included.
func (d *usbfsDevice) BulkReadQuiet(buf []byte, quiet, timeout time.Duration) (int, error) {
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
	return d.readFrameURBs(buf, timeout, quiet)
}

// readFrameURBs reads one whole frame through the usbfs async urb API. It submits
// ceil(len/chunk) bulk-IN transfers up front (USBDEVFS_SUBMITURB), each into its slice of
// buf at a fixed offset, the last sized to the exact remainder, not a padded full chunk,
// then reaps them in completion order (USBDEVFS_REAPURBNDELAY) and counts contiguous bytes
// up to the first short or failed transfer (the FX3 ends a frame with a short packet).
//
// The exact-remainder final transfer is required for the >4 s single-shot TRIGGER tail: the
// FX3 commits whole 1-MiB DMA buffers and holds the frame's final partial buffer; asked for
// a full 1 MiB it cannot fill, it keeps holding. Requesting exactly the pending tail bytes
// makes it deliver. usbfs reports a short bulk-IN as status 0 with actual_length <
// buffer_length (no error), so a short transfer marks the frame end.
//
// It owns the ioMu gate: with quiet 0 the gate is held from before the first submit to
// after the drain (the classic whole-read wedge gate). With quiet > 0 the first `quiet`
// is a host-declared integration window (transfers armed, gate RELEASED so EP0 traffic
// (TEC, telemetry, ST4) flows), and the gate engages at quiet-elapsed or first completion,
// whichever comes first, then stays held through drain. An AbortRead (StopExposure) breaks
// the reap loop within ~one poll interval; the drained short prefix is returned with a nil
// error and the worker's Aborted() check maps it to the abort outcome.
func (d *usbfsDevice) readFrameURBs(buf []byte, timeout, quiet time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	if d.readAborted.Load() {
		return 0, nil // aborted before we armed anything: don't take ioMu at all
	}
	start := time.Now()
	deadline := start.Add(timeout)
	quietEnd := start.Add(quiet)
	gated := false
	gate := func() {
		if !gated {
			d.ioMu.Lock()
			gated = true
		}
	}
	if quiet <= 0 {
		gate()
	}
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
	// On any exit, discard + reap every submitted-but-unreaped urb so no completion lands
	// against buf after we return (see drainURBs: EINTR-safe, bounded, parks + poisons on
	// failure). The drain runs under the gate: an abort/deadline can strike mid-stream, and
	// the discard/reap window is readout as far as the FX3 is concerned. Assembly reads
	// `done` before this defer runs, so the derived mask is fine.
	defer func() {
		gate()
		pending := make([]bool, n)
		for i := range submitted {
			pending[i] = submitted[i] && !done[i]
		}
		d.drainURBs(urbs, pending, buf)
		d.ioMu.Unlock()
	}()

	// Submit with at most urbWindow transfers in flight; completions slide the window forward.
	// usbfs pins every in-flight URB's buffer in kernel memory against a per-fd cap (16 MiB
	// default), so posting a large frame all up front fails at ~slot 16, and the reaped
	// prefix would come back as a silently short frame with a nil error. Frames within the
	// window (all current BulkRead sensors) still get every transfer armed before data flows.
	next := 0
	noMore := false
	submitNext := func() error {
		off := next * maxBulkChunk
		l := maxBulkChunk
		if off+l > len(buf) {
			l = len(buf) - off // last transfer: the exact remainder (see doc)
		}
		urbs[next] = usbURB{typ: urbTypeBulk, endpoint: bulkEndpoint,
			buffer: uintptr(unsafe.Pointer(&buf[off])), bufferLength: int32(l)}
		if err := d.ioctl(usbdevfsSubmitURB, unsafe.Pointer(&urbs[next])); err != nil {
			noMore = true // submit what we could; the reaped prefix is still a valid frame
			return err
		}
		submitted[next] = true
		next++
		return nil
	}
	for next < n && next < urbWindow {
		if err := submitNext(); err != nil {
			if next == 0 {
				return 0, err // couldn't post a single transfer: fatal
			}
			break
		}
	}
	if bulkTrace {
		fmt.Fprintf(os.Stderr, "[urb] start want=%d n=%d timeout=%s\n", len(buf), n, timeout)
	}

	// Reap in completion order (== submission order on a single pipe) until every submitted
	// transfer is in, the deadline hits, or an AbortRead breaks the wait.
	gotAny := false
	for {
		pending := false
		for i := range submitted {
			if submitted[i] && !done[i] {
				pending = true
			}
		}
		if !pending || time.Now().After(deadline) || d.readAborted.Load() {
			break
		}
		if !gated && (gotAny || !time.Now().Before(quietEnd)) {
			gate() // quiet window over (or data arrived early): wedge gate for the readout
		}
		var p uintptr
		err := d.ioctl(usbdevfsReapURBNDelay, unsafe.Pointer(&p))
		if err == syscall.EAGAIN || err == syscall.EINTR { // EINTR: Go's SIGURG preemption
			// Adaptive wait: tight (200 µs) around an active readout, coarse (2 ms) once the
			// pre-data wait has clearly become an integration: 500 wakeups/s instead of 5000
			// for a multi-second exposure, without adding reap latency while data streams.
			s := 200 * time.Microsecond
			if !gotAny && time.Since(start) > 100*time.Millisecond {
				s = 2 * time.Millisecond
			}
			time.Sleep(s)
			continue
		}
		if err != nil {
			return 0, err
		}
		if i := slotOf(p); i >= 0 {
			done[i] = true
			gotAny = true
			if bulkTrace {
				fmt.Fprintf(os.Stderr, "[urb] t=%.3fs slot=%d n=%d status=%d\n",
					time.Since(start).Seconds(), i, urbs[i].actualLength, urbs[i].status)
			}
			if !noMore && next < n {
				_ = submitNext() // keep the window full; a failure just truncates as before
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
// zero-length packets as non-terminal (the DDR cameras, 6200/585, hold the frame's final
// partial buffer until FPGABufReload flushes it, which the worker pulses on a ticker for the
// duration of this call). It keeps issuing full maxBulkChunk reads into scratch and copying
// to a watermark until the frame is in, `idle` passes with no data at all (a genuine stall),
// or the `total` deadline hits. usbfsDevice satisfies FrameStreamer.
func (d *usbfsDevice) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
	if len(buf) == 0 {
		return 0, nil
	}
	if d.readAborted.Load() {
		return 0, nil // aborted before the first transfer
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
		if d.readAborted.Load() {
			break // StopExposure broke the wait (AbortRead); return the short prefix
		}
		// Do NOT slice this transfer's timeout below the idle window: a USBDEVFS_BULK
		// timeout CANCELS the in-flight URB, and a cancel racing an arriving DDR burst
		// (the worker's 20 ms FPGABufReload cadence keeps them coming) drops the bytes in
		// flight; every later byte then lands earlier in the frame, a horizontal shear.
		// Observed on the 6200/IMX455 when this was briefly sliced to 500 ms for abort
		// promptness: every few frames displaced sideways. Cancels must stay confined to
		// the idle boundary, where the stream has been silent for the whole window. The
		// AbortRead check above still bounds an abort to ≤ one idle window on this path.
		ms := rem
		if ms > idle {
			ms = idle
		}
		msec := uint32(ms.Milliseconds())
		if msec == 0 {
			msec = 1 // <1 ms remaining truncates to 0 = INFINITE for usbfs bulk; never block forever
		}
		n, err := d.bulkOne(scratch, msec)
		if n > 0 {
			got += copy(buf[got:], scratch[:n])
			lastData = time.Now()
			continue
		}
		// No data this read (per-read timeout or a ZLP). Not necessarily the end:
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

// ReadFrameStreamPrequeued reads one frame with the SDK's async-transfer model (initAsyncXfer/
// startAsyncXfer): bulk-IN URBs covering buf EXACTLY (slot i targets buf[i·1MiB:] and the last
// slot is sized to the frame remainder), so the batch completes at frame end without relying on
// a short packet (a full-1MiB tail URB never completes once the FPGA stops after the snap
// frame). The slots are pre-queued before the frame arrives,
// so the pipe is armed while the sensor integrates and the GPIF never streams without a reader.
// No slot is resubmitted: one frame, one batch; a retry is a fresh call. Data lands directly
// at each slot's fixed offset, so completion order doesn't matter. Returns the contiguous byte
// count from the start of buf. idle bounds a no-completion stall; total bounds the whole read.
// Satisfies PrequeuedFrameStreamer.
func (d *usbfsDevice) ReadFrameStreamPrequeued(buf []byte, idle, total time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
	if d.readAborted.Load() {
		return 0, nil // aborted before arming: don't take ioMu at all
	}
	d.ioMu.Lock()
	defer d.ioMu.Unlock()
	nslots := (len(buf) + maxBulkChunk - 1) / maxBulkChunk
	const window = urbWindow // in-flight cap: usbfs bounds outstanding URB memory (16 MiB default)
	urbs := make([]usbURB, nslots)
	want := make([]int, nslots)
	inflight := make([]bool, nslots)
	for i := range urbs {
		l := len(buf) - i*maxBulkChunk
		if l > maxBulkChunk {
			l = maxBulkChunk
		}
		want[i] = l
	}
	slotOf := func(p uintptr) int {
		for i := range urbs {
			if p == uintptr(unsafe.Pointer(&urbs[i])) {
				return i
			}
		}
		return -1
	}
	submit := func(i int) error {
		urbs[i] = usbURB{typ: urbTypeBulk, endpoint: bulkEndpoint,
			buffer: uintptr(unsafe.Pointer(&buf[i*maxBulkChunk])), bufferLength: int32(want[i])}
		if err := d.ioctl(usbdevfsSubmitURB, unsafe.Pointer(&urbs[i])); err != nil {
			return err
		}
		inflight[i] = true
		return nil
	}
	// drain discards + reaps every in-flight urb so no completion lands against buf after we
	// return (see drainURBs: EINTR-safe, bounded, parks + poisons on failure). The kernel
	// reports a discarded urb's partial length on the reap, so a stuck slot's bytes still
	// count toward the contiguous prefix. Idempotent; also deferred as a safety net.
	drain := func() { d.drainURBs(urbs, inflight, buf) }
	defer drain()
	// prefix is the contiguous byte count from the start of buf: full slots plus the partial
	// tail of the first incomplete OR errored one. A slot's actualLength counts (the kernel
	// delivered those bytes; a discarded urb reaps -ENOENT with its partial data in place,
	// a babbled one -EOVERFLOW with everything up to the oversized packet), but counting
	// STOPS there: a full-length slot with a nonzero status must not let the count run past
	// a gap. Only meaningful after drain().
	prefix := func() int {
		got := 0
		for i := range urbs {
			got += int(urbs[i].actualLength)
			if int(urbs[i].actualLength) < want[i] || urbs[i].status != 0 {
				break
			}
		}
		return got
	}

	next := 0 // next slot to submit; completions slide the window forward
	for ; next < nslots && next < window; next++ {
		if err := submit(next); err != nil {
			if next == 0 {
				return 0, fmt.Errorf("astrocam: submit urb: %w", err)
			}
			break // window truncated (e.g. usbfs memory cap); completions extend it below
		}
	}

	deadline := time.Now().Add(total)
	lastData := time.Now()
	gotFirst := false // idle gates only AFTER the first completion: the pre-frame quiet is the
	// integration (URBs are armed before the exposure and the first slot completes only once
	// ~1 MiB has streamed), so only `total` bounds it, matching startAsyncXfer, which has no
	// idle cutoff at all, just the overall timeout.
	for done := 0; done < nslots; {
		if time.Now().After(deadline) || d.readAborted.Load() {
			break // deadline, or StopExposure broke the wait (AbortRead)
		}
		var p uintptr
		err := d.ioctl(usbdevfsReapURBNDelay, unsafe.Pointer(&p))
		if err == syscall.EAGAIN || err == syscall.EINTR { // EINTR: Go's SIGURG preemption
			if gotFirst && time.Since(lastData) >= idle {
				break // genuine stall: no completion for the whole idle window
			}
			// Adaptive wait: tight around an active readout, coarse (2 ms) once the pre-data
			// wait is clearly the integration (see readFrameURBs).
			s := 200 * time.Microsecond
			if !gotFirst && time.Since(deadline.Add(-total)) > 100*time.Millisecond {
				s = 2 * time.Millisecond
			}
			time.Sleep(s)
			continue
		}
		if err != nil {
			drain()
			return prefix(), err
		}
		i := slotOf(p)
		if i < 0 {
			continue
		}
		inflight[i] = false
		done++
		lastData = time.Now()
		gotFirst = true
		if debugPreq {
			fmt.Fprintf(os.Stderr, "[preq] +%6.1fms slot %d done: %d/%d status=%d\n",
				time.Since(deadline.Add(-total)).Seconds()*1e3, i, urbs[i].actualLength, want[i], urbs[i].status)
		}
		// A nonzero status (halt, babble) surfaces as a short prefix; the worker's retry
		// ladder resets the endpoint, matching the SDK, which only counts bytes.
		if next < nslots {
			if submit(next) == nil {
				next++
			}
		}
	}
	drain()
	return prefix(), nil
}

// ResetEndpoint clears a halt/stall on the bulk-IN endpoint (USBDEVFS_CLEAR_HALT). CLEAR_HALT
// is EP0 control traffic, so it takes ioMu like every other control transfer: landing it
// mid-readout is the GPIF-wedge interleave the mutex exists to prevent. All callers
// are capture-sequential (they reset BETWEEN reads); the lock enforces it. NOT a recovery
// hatch for a stuck read: that is ResetDevice, which bypasses ioMu.
func (d *usbfsDevice) ResetEndpoint(ep uint8) error {
	release, err := d.enter()
	if err != nil {
		return err
	}
	defer release()
	d.ioMu.Lock()
	defer d.ioMu.Unlock()
	e := uint32(ep)
	return d.ioctl(usbdevfsClearHalt, unsafe.Pointer(&e))
}

// ResetDevice performs a USB port reset of the whole device (USBDEVFS_RESET), the last-resort
// recovery when the readout wedges. It does NOT take ioMu: its job includes
// recovering while a stuck read still holds the lock, and serializing it would deadlock that
// recovery. The reset drops the kernel's interface claim, so we re-claim interface 0
// afterwards; the device keeps its address (no node change).
func (d *usbfsDevice) ResetDevice() error {
	release, err := d.enter()
	if err != nil {
		return err
	}
	defer release()
	if err := d.ioctl(usbdevfsReset, nil); err != nil {
		return err
	}
	return d.claim(0) // the reset releases the interface claim; re-take it
}

// Close waits for in-flight I/O (the closeMu interlock; every I/O is deadline-bounded, so
// this is too), then closes the fd. The fd release makes the kernel free any still-queued
// urbs WITHOUT copyout, so nothing can land against user memory afterwards. Idempotent.
func (d *usbfsDevice) Close() error {
	d.closeMu.Lock()
	defer d.closeMu.Unlock()
	if d.closed {
		return nil
	}
	d.closed = true
	return d.f.Close()
}

// OpenHost opens the default USB backend for this platform (Linux: usbfs).
func OpenHost(vid, pid uint16) (Transport, error) {
	d, err := openUSBFS(vid, pid)
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
// and claims its bulk interface: it binds the exact unit chosen from Enumerate. The
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
		return nil, fmt.Errorf("astrocam: device at %s is now %04x, not %04x (replugged?)", path, gotVID, vid)
	}
	return d, nil
}
