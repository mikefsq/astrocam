//go:build linux

// Pure-Go Linux USB transport over usbfs (/dev/bus/usb), no cgo. Control transfers and bulk-IN
// frame reads go through usbfs ioctls in 1 MiB transfers (maxBulkChunk). BulkRead and
// ReadFrameStreamPrequeued post a frame's URBs up front for USB2; ReadFrameStream cycles a URB
// window while the worker pulses FPGABufReload, the USB3 DDR path. Access needs a udev rule for
// the camera's USB VID (ZWO 0x03C3).

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

// usbfs ioctl encoding (asm-generic _IOC). dir: 0 none, 1 write, 2 read, 3 read+write.
// layout: (dir<<30)|(size<<16)|(type<<8)|nr, type='U' (0x55). The sizes come from the Go
// structs below, whose natural alignment reproduces the kernel layout on both 64-bit
// (ctrltransfer 24, bulktransfer 24, urb 56, pointer 8) and 32-bit ARM/x86 (16, 16, 44, 4)
// hosts.
func ioc(dir, nr, size uintptr) uintptr { return dir<<30 | size<<16 | 0x55<<8 | nr }

var (
	usbdevfsControl        = ioc(3, 0, unsafe.Sizeof(usbCtrlTransfer{})) // _IOWR('U',0,usbdevfs_ctrltransfer)
	usbdevfsBulk           = ioc(3, 2, unsafe.Sizeof(usbBulkTransfer{})) // _IOWR('U',2,usbdevfs_bulktransfer)
	usbdevfsSubmitURB      = ioc(2, 10, unsafe.Sizeof(usbURB{}))         // _IOR('U',10,usbdevfs_urb) = 0x8038550a on 64-bit
	usbdevfsDiscardURB     = ioc(0, 11, 0)                               // _IO('U',11)               = 0x0000550b
	usbdevfsReapURBNDelay  = ioc(1, 13, unsafe.Sizeof(uintptr(0)))       // _IOW('U',13,void*)        = 0x4008550d on 64-bit (non-blocking)
	usbdevfsClaimInterface = ioc(2, 15, 4)                               // _IOR('U',15,uint)
	usbdevfsClearHalt      = ioc(2, 21, 4)                               // _IOR('U',21,uint)
	usbdevfsReset          = ioc(0, 20, 0)                               // _IO('U',20)               = 0x00005514
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

// Kernel structs (<linux/usbdevice_fs.h>); the pointer fields' natural alignment gives the
// kernel's layout at either pointer size.
type usbCtrlTransfer struct {
	bRequestType uint8
	bRequest     uint8
	wValue       uint16
	wIndex       uint16
	wLength      uint16
	timeout      uint32
	data         uintptr
}

type usbBulkTransfer struct {
	ep      uint32
	len     uint32
	timeout uint32
	data    uintptr
}

// Compile-time layout check: the struct sizes equal the kernel's at this pointer size
// (ctrl/bulk 16 + 2·(ps−4), urb 44 + 3·(ps−4): 24/24/56 on 64-bit, 16/16/44 on 32-bit). A
// mismatch underflows the uintptr constant and the package fails to build for that target.
const (
	ptrSize = unsafe.Sizeof(uintptr(0))
	_       = unsafe.Sizeof(usbCtrlTransfer{}) - (16 + 2*(ptrSize-4))
	_       = (16 + 2*(ptrSize-4)) - unsafe.Sizeof(usbCtrlTransfer{})
	_       = unsafe.Sizeof(usbBulkTransfer{}) - (16 + 2*(ptrSize-4))
	_       = (16 + 2*(ptrSize-4)) - unsafe.Sizeof(usbBulkTransfer{})
	_       = unsafe.Sizeof(usbURB{}) - (44 + 3*(ptrSize-4))
	_       = (44 + 3*(ptrSize-4)) - unsafe.Sizeof(usbURB{})
	_       = unsafe.Offsetof(usbURB{}.usercontext) - (40 + 2*(ptrSize-4))
	_       = (40 + 2*(ptrSize-4)) - unsafe.Offsetof(usbURB{}.usercontext)
)

// usbfsDevice is a usbfs-backed Transport for one open camera.
type usbfsDevice struct {
	f *os.File
	// closeMu is the Close interlock: every public I/O method holds it shared for the whole
	// I/O (URB drain included) and Close takes it exclusively, so Close cannot close the fd
	// under a drain and leave kernel-held pointers into the caller's buffer. Lock order:
	// closeMu (shared) before ioMu. All I/O is deadline-bounded, so Close is too.
	closeMu sync.RWMutex
	closed  bool // under closeMu
	// broken poisons the device after an undrainable readout (see drainURBs): the kernel may
	// still write into parked buffers, so every later I/O on this fd fails fast until the
	// camera is re-opened.
	broken atomic.Bool
	// ioMu serializes the USB I/O the FX3 bridge must not see interleaved: every control transfer
	// and every whole-frame BulkRead and ReadFrameStreamPrequeued hold it. A control transfer (a
	// TEC poll, a CCDTemperature read → 0xB3) landing mid-readout wedges the un-buffered USB2 path
	// on an ASI174: the GPIF parks and EP0 times out (-110) until a device reset and re-Init.
	// ReadFrameStream skips it, since the DDR path needs the worker's concurrent FPGABufReload and
	// its frame buffer makes the interleave harmless. ResetDevice and ControlOutUngated bypass it.
	ioMu sync.Mutex
	// readAborted is the level-triggered ReadAborter latch: while set, in-flight frame reads
	// return their short prefix within about one poll interval and new frame reads return (0, nil)
	// without taking ioMu, so StopExposure's writes never queue behind a blocked read. Control
	// transfers and ResetEndpoint ignore it.
	readAborted atomic.Bool
	// readActive counts frame reads in flight. On a USB2 link the IN control path paces itself
	// while it is non-zero.
	readActive atomic.Int32
	inPace     inPacer
	// streams are the open resident sessions (StartStream); Close stops them first so no URB is
	// left with the kernel against a closed fd. Guarded by streamMu.
	streamMu sync.Mutex
	streams  map[*usbfsStream]struct{}
	// superSpeed is the negotiated link speed (bulk-IN max packet ≥1024 = USB3 SuperSpeed),
	// read from the endpoint descriptor at open. The readout's bandwidth budget follows it.
	superSpeed bool
}

// AbortRead / ArmRead implement ReadAborter (see readAborted).
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
	d := &usbfsDevice{f: f, inPace: inPacer{min: usb2InPace}}
	// Negotiated link speed from the bulk-IN endpoint's max packet size (SuperSpeed 1024,
	// HighSpeed 512). The node read returns the active config/interface/endpoint descriptors,
	// which reflect the negotiated speed, so a USB3 camera on a HighSpeed link reads as USB2 and
	// takes the USB2 bandwidth budget instead of overrunning the bus and shearing the frame.
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

// enter gates every public I/O method: it fails fast on a closed or poisoned device and
// holds closeMu shared until the returned release runs.
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

// drainTimeout bounds drainURBs' wait for discarded URBs to come back. A discard completes
// almost immediately on a live HCD; one that does not deliver within this window forfeits
// the device (park + poison) rather than holding ioMu forever.
const drainTimeout = 5 * time.Second

// leakedIO pins memory the kernel may still write to (URBs that could not be reaped): usbfs
// copies out at reap time, so freeing it would be a use-after-free. Never cleared.
var (
	leakedIOMu sync.Mutex
	leakedIO   []interface{}
)

// drainURBs discards and reaps every pending URB, so no kernel copyout can land against the
// caller's memory after return: usbfs writes buf and the URB structs at reap time. EINTR and
// EAGAIN are retried, since the Go runtime's SIGURG preemption interrupts ioctls routinely. URBs
// still unreaped after drainTimeout, or a reap that fails outright (ENODEV after a disconnect),
// park their buffers in leakedIO and poison the device.
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
	if reqType&0x80 != 0 && d.readActive.Load() > 0 && !d.superSpeed {
		d.inPace.wait() // a USB2 readout in flight: pace EP0 reads (usb2InPace)
	}
	ct := usbCtrlTransfer{
		bRequestType: reqType, bRequest: bRequest,
		wValue: wValue, wIndex: wIndex, wLength: uint16(len(data)),
		timeout: 500,
	}
	if len(data) > 0 {
		ct.data = uintptr(unsafe.Pointer(&data[0]))
	}
	// USBDEVFS_CONTROL returns the transferred byte count; report it rather than len(data): a
	// short control-IN (the SPI flash read hitting the blob end) must be visible to callers
	// that stop on got < want.
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

// ControlOutUngated issues a vendor OUT without ioMu, for ST4 guide pulses only, so a pulse edge
// cannot queue behind a whole-frame read. usbfs control ioctls are safe per call, and the SDK's
// capture thread holds no API mutex either. Gates: closeMu shared and the broken poison.
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

// maxBulkChunk caps a single USBDEVFS_BULK transfer: usbfs kmallocs a physically contiguous
// kernel buffer per ioctl, so a whole-frame read of several MB fails with ENOMEM above the
// kernel's per-allocation limit (~4 MB, lower under fragmentation). 1 MiB matches the SDK
// xferLen.
const maxBulkChunk = 1 << 20

// urbWindow caps the async-URB paths' in-flight transfers: usbfs bounds the outstanding
// URB buffer memory it will pin per fd (16 MiB default), so 12×1 MiB leaves headroom.
const urbWindow = 12

// bulkTrace, when ASTROCAM_BULK_TRACE is set, logs one line per submitted/reaped urb to
// stderr (held-tail / short-packet timing). Diagnostic only; off by default.
var bulkTrace = os.Getenv("ASTROCAM_BULK_TRACE") != ""

// BulkRead reads one whole frame through readFrameURBs (the async-URB pump). Gates: closeMu
// shared, ioMu for the whole frame. Returns the contiguous prefix with a nil error on stall
// or abort.
func (d *usbfsDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
	d.readActive.Add(1)
	defer d.readActive.Add(-1)
	return d.readFrameURBs(buf, timeout, 0)
}

// BulkReadQuiet implements QuietBulkReader: like BulkRead, but ioMu stays released for the
// first `quiet` of the read, a host-timed integration window in which the transfers are
// armed (so the GPIF never streams without a reader) but no data can arrive. ioMu engages at
// quiet-elapsed or first data, whichever comes first. timeout bounds the whole call, quiet
// included.
func (d *usbfsDevice) BulkReadQuiet(buf []byte, quiet, timeout time.Duration) (int, error) {
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
	d.readActive.Add(1)
	defer d.readActive.Add(-1)
	return d.readFrameURBs(buf, timeout, quiet)
}

// urbReqLen is slot i's requested length in a batch covering bufLen bytes in chunk-sized
// transfers: a whole chunk, except the last, which asks for the exact remainder.
func urbReqLen(i, chunk, bufLen int) int {
	if off := i * chunk; off+chunk > bufLen {
		return bufLen - off
	}
	return chunk
}

// urbFrameEnded reports whether the in-order prefix of completed slots has reached the transfer
// that ends the frame: one that failed, or came back shorter than it asked for. Every slot behind
// it was submitted before the frame ended and will never complete, since the FX3 has stopped
// sending, so a reap loop that waits for them burns its whole timeout on a frame that is already
// in. It stays false while the prefix is unbroken, including when every slot is in, which the
// caller's own no-pending check ends.
func urbFrameEnded(urbs []usbURB, done []bool, chunk, bufLen int) bool {
	for i := range urbs {
		if !done[i] {
			return false
		}
		if urbs[i].status != 0 || int(urbs[i].actualLength) < urbReqLen(i, chunk, bufLen) {
			return true
		}
	}
	return false
}

// readFrameURBs reads one whole frame through the usbfs async URB API. It submits bulk-IN
// transfers of maxBulkChunk each into buf at fixed offsets, the last sized to the exact
// remainder, reaps them in completion order, and counts contiguous bytes up to the first short or
// failed transfer. usbfs reports a short bulk-IN as status 0 with actual_length <
// buffer_length, so a short transfer marks the frame end.
//
// The >4 s single-shot TRIGGER tail needs that exact-remainder final transfer. The FX3 commits
// whole 1-MiB DMA buffers and holds the frame's final partial buffer: asked for a full 1 MiB it
// cannot fill, it keeps holding, and asked for the pending tail bytes it delivers.
//
// Gates: the caller holds closeMu shared, and this function owns ioMu. quiet 0 holds ioMu from
// before the first submit to after the drain. quiet > 0 arms the transfers with ioMu released, so
// EP0 traffic flows during the integration, and takes ioMu at quiet-elapsed or first completion.
func (d *usbfsDevice) readFrameURBs(buf []byte, timeout, quiet time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	if d.readAborted.Load() {
		return 0, nil // abort latched: fail fast without taking ioMu
	}
	if timeout <= 0 {
		timeout = defaultTimeoutBound // see the backend contract in transport.go
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
	// Discard and reap every submitted-but-unreaped URB on any exit, so no completion lands
	// against buf after return. The drain runs under ioMu: an abort or deadline can strike
	// mid-stream, and the discard/reap window is readout as far as the FX3 is concerned.
	defer func() {
		gate()
		pending := make([]bool, n)
		for i := range submitted {
			pending[i] = submitted[i] && !done[i]
		}
		d.drainURBs(urbs, pending, buf)
		d.ioMu.Unlock()
	}()

	// Keep at most urbWindow transfers in flight; each completion slides the window forward.
	// usbfs pins every in-flight URB's buffer against a per-fd cap (16 MiB default), so posting a
	// large frame all up front fails at about slot 16 and returns the reaped prefix as a silently
	// short frame. Frames within the window still have every transfer armed before data flows.
	next := 0
	submitNext := func() error {
		off := next * maxBulkChunk
		l := maxBulkChunk
		if off+l > len(buf) {
			l = len(buf) - off // last transfer: the exact remainder (see doc)
		}
		urbs[next] = usbURB{typ: urbTypeBulk, endpoint: bulkEndpoint,
			buffer: uintptr(unsafe.Pointer(&buf[off])), bufferLength: int32(l)}
		if err := d.ioctl(usbdevfsSubmitURB, unsafe.Pointer(&urbs[next])); err != nil {
			// Leave `next` where it is so the next completion retries this slot. Latching the
			// failure truncates the frame on a transient usbfs memory-cap ENOMEM and returns it
			// with a nil error.
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

	// Reap in completion order (submission order on a single pipe) until every submitted
	// transfer is in, the deadline hits, or an AbortRead breaks the wait.
	gotAny := false
	for {
		pending := false
		for i := range submitted {
			if submitted[i] && !done[i] {
				pending = true
			}
		}
		if !pending || urbFrameEnded(urbs, done, maxBulkChunk, len(buf)) ||
			time.Now().After(deadline) || d.readAborted.Load() {
			break
		}
		if !gated && (gotAny || !time.Now().Before(quietEnd)) {
			gate() // quiet window over, or data arrived early
		}
		var p uintptr
		err := d.ioctl(usbdevfsReapURBNDelay, unsafe.Pointer(&p))
		if err == syscall.EAGAIN || err == syscall.EINTR { // EINTR: Go's SIGURG preemption
			// Adaptive wait: 200 µs around an active readout, 2 ms once the pre-data wait has
			// become an integration (500 wakeups/s instead of 5000 for a multi-second
			// exposure), without adding reap latency while data streams.
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
			if next < n {
				_ = submitNext() // keep the window full; a failed submit is retried next completion
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
		req := urbReqLen(i, maxBulkChunk, len(buf))
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

// ddrWindow is how many bulk URBs the DDR reader keeps in flight. The frame arrives as a run of
// 1 MiB DMA buffers interleaved with ZLPs, so the transfer count is not known up front and slots
// are resubmitted as they drain; a window this deep keeps the pipe armed across the gaps the
// FPGABufReload ticker leaves.
const ddrWindow = 4

// ReadFrameStream reads one whole frame for the DDR cameras (6200/585), which hold the frame's
// final partial buffer until FPGABufReload flushes it. The worker pulses that on a ticker for the
// duration of this call, and mid-frame short packets and ZLPs are not terminal. It keeps
// ddrWindow async URBs cycling into per-slot scratch and copies each completion to a watermark
// until the frame is in, idle passes with no data, or the total deadline hits. Gates: closeMu
// shared only.
//
// The path uses async URBs rather than blocking USBDEVFS_BULK calls, which cannot honour
// AbortRead: each ioctl blocks in the kernel for up to idle, which here is the worker's exposure
// plus 2 s, so a StopExposure during a 60 s sub's readout waits up to 62 s. Slicing the timeout
// does not work either, since a USBDEVFS_BULK timeout cancels the in-flight URB and a cancel
// racing an arriving DDR burst drops the bytes in flight, shearing the frame. A non-blocking reap
// tests the latch every pass, and the only cancel is the drain on the way out. A timed-out
// USBDEVFS_BULK also reports -ETIMEDOUT with no count and loses its partial payload, where a
// discarded URB reaps with its bytes in place.
func (d *usbfsDevice) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
	d.readActive.Add(1)
	defer d.readActive.Add(-1)
	if len(buf) == 0 {
		return 0, nil
	}
	if d.readAborted.Load() {
		return 0, nil // abort latched: fail fast
	}
	if idle <= 0 {
		idle = defaultIdleBound // see the backend contract in transport.go
	}
	if total <= 0 {
		total = defaultTotalBound
	}
	start := time.Now()
	deadline := start.Add(total)

	n := ddrWindow
	urbs := make([]usbURB, n)
	armed := make([]bool, n)
	// One contiguous pool sliced per slot rather than n separate buffers: drainURBs pins a single
	// []byte when it forfeits, and the kernel writes into this scratch, not into buf.
	pool := make([]byte, n*maxBulkChunk)
	scratch := make([][]byte, n)
	for i := range scratch {
		scratch[i] = pool[i*maxBulkChunk : (i+1)*maxBulkChunk]
	}
	// Discard and reap anything still with the kernel on the way out, so no completion lands in
	// the scratch after return. This runs on every exit, including a clean one: the window is
	// still armed when the frame's last byte arrives, so up to ddrWindow-1 transfers are discarded
	// with whatever they hold. The path is single-shot and the worker halts the readout on return,
	// so no following frame can lose its head.
	defer func() { d.drainURBs(urbs, armed, pool) }()

	submit := func(i int) bool {
		urbs[i] = usbURB{typ: urbTypeBulk, endpoint: bulkEndpoint,
			buffer: uintptr(unsafe.Pointer(&scratch[i][0])), bufferLength: int32(maxBulkChunk)}
		if err := d.ioctl(usbdevfsSubmitURB, unsafe.Pointer(&urbs[i])); err != nil {
			armed[i] = false
			return false
		}
		armed[i] = true
		return true
	}
	slotOf := func(p uintptr) int {
		for i := range urbs {
			if p == uintptr(unsafe.Pointer(&urbs[i])) {
				return i
			}
		}
		return -1
	}
	inflight := 0
	for i := 0; i < n; i++ {
		if !submit(i) {
			break // window truncated (usbfs memory cap); the armed slots carry the stream
		}
		inflight++
	}
	if inflight == 0 {
		return 0, fmt.Errorf("astrocam: stream read: no URB could be submitted")
	}

	got := 0
	lastData := time.Now()
	for got < len(buf) {
		if d.readAborted.Load() {
			break // AbortRead: return the short prefix, within one reap pass
		}
		if time.Now().After(deadline) {
			break // overall deadline
		}
		var p uintptr
		rerr := d.ioctl(usbdevfsReapURBNDelay, unsafe.Pointer(&p))
		if rerr == syscall.EAGAIN || rerr == syscall.EINTR { // EINTR: Go's SIGURG preemption
			if time.Since(lastData) >= idle {
				break // stall: no data for the whole idle window
			}
			// Adaptive wait, as the other reap loops: tight around a live readout, slacker
			// while the pre-data wait is really the integration.
			w := 200 * time.Microsecond
			if got == 0 && time.Since(start) > 100*time.Millisecond {
				w = 2 * time.Millisecond
			}
			time.Sleep(w)
			continue
		}
		if rerr != nil {
			return got, rerr
		}
		i := slotOf(p)
		if i < 0 {
			continue
		}
		armed[i] = false
		inflight--
		// URB_SHORT_NOT_OK is not set, so a short transfer completes with status 0 and only a
		// real pipe error is nonzero here. Discards happen in the drain, after this loop.
		if st := urbs[i].status; st != 0 {
			return got, fmt.Errorf("astrocam: stream read: urb status %d", st)
		}
		if ln := int(urbs[i].actualLength); ln > 0 {
			got += copy(buf[got:], scratch[i][:ln])
			lastData = time.Now()
		}
		// A ZLP is the FX3's inter-buffer marker, not end of frame: recycle and keep cycling.
		if got < len(buf) && submit(i) {
			inflight++
		}
		if inflight == 0 {
			break // every slot failed to resubmit; nothing can complete
		}
	}
	return got, nil
}

// ReadFrameStreamPrequeued reads one frame with the SDK's async-transfer model
// (initAsyncXfer/startAsyncXfer): bulk-IN URBs cover buf exactly, slot i targeting buf[i·1MiB:]
// and the last sized to the remainder, so the pipe is armed while the sensor integrates and the
// GPIF never streams without a reader. usbfs pins every in-flight URB's buffer against a per-fd
// cap, so only the first urbWindow slots (12 MiB) are queued before the frame arrives and
// completions slide the window over the rest. The batch completes at frame end without relying on
// a short packet: a full-1MiB tail URB never completes once the FPGA stops after the snap frame.
// No slot is resubmitted, so a retry is a fresh call. Data lands at each slot's fixed offset, so
// completion order does not matter. idle bounds a no-completion stall after the first completion;
// total bounds the whole read. Gates: closeMu shared, ioMu for the whole frame.
func (d *usbfsDevice) ReadFrameStreamPrequeued(buf []byte, idle, total time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
	d.readActive.Add(1)
	defer d.readActive.Add(-1)
	if d.readAborted.Load() {
		return 0, nil // abort latched: fail fast without taking ioMu
	}
	if idle <= 0 {
		idle = defaultIdleBound // see the backend contract in transport.go
	}
	if total <= 0 {
		total = defaultTotalBound
	}
	d.ioMu.Lock()
	defer d.ioMu.Unlock()
	nslots := (len(buf) + maxBulkChunk - 1) / maxBulkChunk
	const window = urbWindow // in-flight cap (see urbWindow)
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
	// drain discards and reaps every in-flight URB, so no completion lands against buf after
	// return. The kernel reports a discarded URB's partial length on the reap, so a stuck slot's
	// bytes still count toward the contiguous prefix. Idempotent.
	drain := func() { d.drainURBs(urbs, inflight, buf) }
	defer drain()
	// prefix is the contiguous byte count from the start of buf: full slots plus the partial tail
	// of the first incomplete or errored one. A discarded URB reaps -ENOENT with its partial data
	// in place and a babbled one -EOVERFLOW with everything up to the oversized packet, so both
	// count, but counting stops there and cannot run past a gap. Only meaningful after drain().
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
	cursor, frameEnd := 0, false // in-order scan for the frame-ending short/failed slot
	// idle gates only after the first completion. The pre-frame quiet is the integration: URBs are
	// armed before the exposure and the first slot completes only once ~1 MiB has streamed, so
	// only total bounds it, as in startAsyncXfer.
	gotFirst := false
	for done := 0; done < nslots; {
		if time.Now().After(deadline) || d.readAborted.Load() {
			break // deadline, or AbortRead
		}
		var p uintptr
		err := d.ioctl(usbdevfsReapURBNDelay, unsafe.Pointer(&p))
		if err == syscall.EAGAIN || err == syscall.EINTR { // EINTR: Go's SIGURG preemption
			if gotFirst && time.Since(lastData) >= idle {
				break // stall: no completion for the whole idle window
			}
			// Adaptive wait: 200 µs around an active readout, 2 ms once the pre-data wait
			// is the integration (see readFrameURBs).
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
		// ladder resets the endpoint. The SDK likewise only counts bytes.
		if next < nslots {
			if submit(next) == nil {
				next++
			}
		}
		// Stop once the in-order prefix reaches the frame's terminating short transfer or a
		// failed slot. The slots behind it were queued before the frame arrived and never
		// complete, so waiting them out costs the whole idle window on a finished frame, and
		// leaving them armed lets them absorb the head of the next free-run frame.
		for cursor < next && !inflight[cursor] {
			if urbs[cursor].status != 0 || int(urbs[cursor].actualLength) < want[cursor] {
				frameEnd = true
				break
			}
			cursor++
		}
		if frameEnd {
			break
		}
	}
	drain()
	return prefix(), nil
}

// ResetEndpoint clears a halt on the bulk-IN endpoint (USBDEVFS_CLEAR_HALT). CLEAR_HALT is
// EP0 control traffic, so it takes closeMu shared and ioMu like every control transfer;
// callers reset between reads. It is not a recovery hatch for a stuck read: that is
// ResetDevice.
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
// recovery for a wedged readout. Gates: closeMu shared only; it must run while a stuck read
// still holds ioMu. The reset drops the kernel's interface claim, so interface 0 is
// re-claimed afterwards; the device keeps its address (no node change).
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

// Close waits for in-flight I/O (closeMu exclusive), then closes the fd, which makes the
// kernel free any still-queued URBs without copyout. Idempotent.
func (d *usbfsDevice) Close() error {
	d.closeMu.Lock()
	defer d.closeMu.Unlock()
	if d.closed {
		return nil
	}
	// Stop open stream sessions under the exclusive lock, as darwin does. A snapshot taken before
	// that lock races StartStream, which registers its session only after arming the window under
	// the shared lock: a session registered while Close waits for the exclusive lock misses the
	// snapshot, keeps its transfers with the kernel across the handle close, and poisons the
	// device on its later Next or Close. The session Close takes streamMu and its own mu but never
	// closeMu, so calling it from here cannot deadlock.
	d.streamMu.Lock()
	open := make([]*usbfsStream, 0, len(d.streams))
	for st := range d.streams {
		open = append(open, st)
	}
	d.streamMu.Unlock()
	for _, st := range open {
		_ = st.Close()
	}
	d.closed = true
	// Closing the fd kills any URB still with the kernel after a forfeit; the parked buffers stay
	// pinned in leakedIO regardless, so a poisoned device needs no separate path here.
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
