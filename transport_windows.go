//go:build windows

// Pure-Go Windows USB transport over WinUSB, no cgo (stdlib syscall + lazy DLLs): control via
// WinUsb_ControlTransfer and bulk-IN frame reads via overlapped WinUsb_ReadPipe, bounded by the
// pipe's transfer-timeout policy.
//
// The camera must be bound to the generic WinUSB (or libusbK) kernel driver; rebind with Zadig.
// ZWO's own installer binds their proprietary ASICAMUSB3.sys, which the WinUSB API cannot open,
// and the bind is exclusive, so ZWO's native software will not see the camera until it is
// reverted in Device Manager. Devices are matched by VID/PID in the device-interface path under
// the generic USB device interface GUID.
//
// WinUSB exposes no device-level USB reset, so this backend has no DeviceResetter.
//
// Compile-checked for windows/amd64 but not run on hardware.

package astrocam

import (
	"errors"
	"fmt"
	"runtime"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"
)

var (
	modSetupAPI = syscall.NewLazyDLL("setupapi.dll")
	modWinUSB   = syscall.NewLazyDLL("winusb.dll")
	modKernel32 = syscall.NewLazyDLL("kernel32.dll")

	procGetClassDevs     = modSetupAPI.NewProc("SetupDiGetClassDevsW")
	procEnumInterfaces   = modSetupAPI.NewProc("SetupDiEnumDeviceInterfaces")
	procGetInterfaceDtl  = modSetupAPI.NewProc("SetupDiGetDeviceInterfaceDetailW")
	procDestroyDevInfo   = modSetupAPI.NewProc("SetupDiDestroyDeviceInfoList")
	procWinUsbInit       = modWinUSB.NewProc("WinUsb_Initialize")
	procWinUsbFree       = modWinUSB.NewProc("WinUsb_Free")
	procWinUsbControl    = modWinUSB.NewProc("WinUsb_ControlTransfer")
	procWinUsbReadPipe   = modWinUSB.NewProc("WinUsb_ReadPipe")
	procWinUsbSetPipePol = modWinUSB.NewProc("WinUsb_SetPipePolicy")
	procWinUsbResetPipe  = modWinUSB.NewProc("WinUsb_ResetPipe")
	procWinUsbAbortPipe  = modWinUSB.NewProc("WinUsb_AbortPipe")
	procWinUsbQueryPipe  = modWinUSB.NewProc("WinUsb_QueryPipe")
	procWinUsbGetOvlRes  = modWinUSB.NewProc("WinUsb_GetOverlappedResult")
	procCreateFile       = modKernel32.NewProc("CreateFileW")
	procCloseHandle      = modKernel32.NewProc("CloseHandle")
	procCreateEvent      = modKernel32.NewProc("CreateEventW")
	procWaitForSingle    = modKernel32.NewProc("WaitForSingleObject")
)

// GUID_DEVINTERFACE_USB_DEVICE {A5DCBF10-6530-11D2-901F-00C04FB951ED}.
var usbDeviceGUID = guid{0xA5DCBF10, 0x6530, 0x11D2, [8]byte{0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED}}

type guid struct {
	Data1 uint32
	Data2 uint16
	Data3 uint16
	Data4 [8]byte
}

type spDeviceInterfaceData struct {
	cbSize             uint32
	InterfaceClassGUID guid
	Flags              uint32
	Reserved           uintptr
}

const (
	digcfPresent         = 0x02
	digcfDeviceInterface = 0x10
	genericReadWrite     = 0xC0000000 // GENERIC_READ|GENERIC_WRITE
	fileShareRW          = 0x03
	openExisting         = 3
	fileFlagOverlapped   = 0x40000000
	pipeTransferTimeout  = 0x03 // WINUSB_PIPE_POLICY PIPE_TRANSFER_TIMEOUT (ULONG ms)
	pipeIn               = bulkEndpoint

	// maxBulkChunk caps a single ReadPipe, so a whole-frame read time-bounds (WinUSB blocks
	// forever by default) and reads through the FX3's mid-frame short packets and ZLPs at a
	// contiguous watermark. 1 MiB matches the SDK xferLen.
	maxBulkChunk = 1 << 20

	// Windows error codes that mean "the bounded read elapsed", not a pipe failure.
	errSemTimeout = 121  // ERROR_SEMAPHORE_TIMEOUT
	errOpAborted  = 995  // ERROR_OPERATION_ABORTED
	errTimeout    = 1460 // ERROR_TIMEOUT

	errIOPending = 997 // ERROR_IO_PENDING: the overlapped read was queued

	waitObject0 = 0 // WaitForSingleObject: the event signaled

	// winPrequeuedWindow is how many overlapped reads ReadFrameStreamPrequeued keeps armed
	// at once (12 MiB in flight, matching the usbfs batch window); winDrainTimeout bounds
	// the post-abort wait for the kernel to release the frame buffer.
	winPrequeuedWindow = 12
	winDrainTimeout    = 5 * time.Second
)

// errWinTransportClosed is returned by a transfer attempted after Close released the handle.
var errWinTransportClosed = errors.New("astrocam: transport closed")

// errWinTransportBroken marks a device whose aborted overlapped reads never completed (see
// winusbDevice.broken).
var errWinTransportBroken = errors.New("astrocam: transport broken (undrainable overlapped I/O)")

// errWinStreamDead marks a stream session whose window has emptied: every slot failed to
// resubmit, so no completion can arrive and no further frame will ever be produced. The session
// is finished; Close it and StartStream again.
var errWinStreamDead = errors.New("astrocam: stream session has no armed transfers left; close and restart it")

// winLeakedIO pins buffers (and their OVERLAPPEDs/events) that could not be drained after an
// abort: the kernel can still write into them, so they must never be reused or collected.
// Never cleared.
var (
	winLeakedIOMu sync.Mutex
	winLeakedIO   []any
)

// winusbDevice is a WinUSB-backed Transport for one open camera.
type winusbDevice struct {
	handle syscall.Handle // file handle
	winusb uintptr        // WINUSB_INTERFACE_HANDLE
	// closeMu is the Close interlock: every public I/O holds it shared for the whole transfer
	// and Close takes it exclusively, so Close waits for in-flight I/O and a transfer after
	// Close returns errWinTransportClosed instead of using the freed handle. Lock order:
	// closeMu (shared) before ioMu.
	closeMu sync.RWMutex
	closed  bool // under closeMu
	// ioMu serializes the USB I/O the FX3 must not see interleaved: every control transfer and
	// every whole-frame BulkRead and ReadFrameStreamPrequeued hold it, so a control transfer
	// cannot land mid-readout and park the GPIF on the un-buffered USB2 path. ReadFrameStream
	// skips it, since the DDR path needs the worker's concurrent FPGABufReload.
	ioMu sync.Mutex
	// inMaxPacket is the bulk-IN pipe's max packet size (SuperSpeed ≥1024, HighSpeed 512), read
	// at open. The readout's bandwidth budget follows it through SuperSpeed().
	inMaxPacket uint16
	// broken latches when an aborted overlapped read could not be drained. The kernel still owns
	// a caller buffer, pinned in winLeakedIO, so every later transfer fails fast and Close leaks
	// the handles. Only a process restart recovers.
	broken atomic.Bool
	// readAborted is the level-triggered ReadAborter latch: while set, in-flight frame reads abort
	// their pipe I/O and return the short prefix, and new frame reads return (0, nil) without
	// taking ioMu, so StopExposure's writes never queue behind a blocked read.
	readAborted atomic.Bool
	// readActive counts frame reads in flight. On a USB2 link the IN control path paces itself
	// while it is non-zero.
	readActive atomic.Int32
	inPace     inPacer
	// streams are the open resident sessions (StartStream); Close stops them first so no
	// overlapped read is left with the kernel against a freed handle. Guarded by streamMu.
	streamMu sync.Mutex
	streams  map[*winusbStream]struct{}
}

// AbortRead / ArmRead implement ReadAborter (see readAborted).
func (d *winusbDevice) AbortRead() { d.readAborted.Store(true) }
func (d *winusbDevice) ArmRead()   { d.readAborted.Store(false) }

// enter gates every public I/O method: it fails fast on a closed or poisoned device and
// holds closeMu shared until the returned release runs.
func (d *winusbDevice) enter() (release func(), err error) {
	if d.broken.Load() {
		return nil, errWinTransportBroken
	}
	d.closeMu.RLock()
	if d.closed {
		d.closeMu.RUnlock()
		return nil, errWinTransportClosed
	}
	return d.closeMu.RUnlock, nil
}

// winNode is one WinUSB device-interface path with its parsed VID/PID.
type winNode struct {
	path     string
	vid, pid uint16
}

// scanWinUSB lists every WinUSB device-interface path whose path carries the given vid
// (and pid, when pid != 0), with the PID parsed from the path.
func scanWinUSB(vid, pid uint16) []winNode {
	h, _, _ := procGetClassDevs.Call(uintptr(unsafe.Pointer(&usbDeviceGUID)), 0, 0, digcfPresent|digcfDeviceInterface)
	if h == 0 || h == ^uintptr(0) {
		return nil
	}
	defer procDestroyDevInfo.Call(h)

	var out []winNode
	for idx := uint32(0); ; idx++ {
		var ifData spDeviceInterfaceData
		ifData.cbSize = uint32(unsafe.Sizeof(ifData))
		r, _, _ := procEnumInterfaces.Call(h, 0, uintptr(unsafe.Pointer(&usbDeviceGUID)), uintptr(idx), uintptr(unsafe.Pointer(&ifData)))
		if r == 0 {
			break // ERROR_NO_MORE_ITEMS
		}
		path := interfacePath(h, &ifData)
		if path == "" {
			continue
		}
		gotVID, gotPID, ok := parseVIDPID(path)
		if !ok || gotVID != vid || (pid != 0 && gotPID != pid) {
			continue
		}
		out = append(out, winNode{path: path, vid: gotVID, pid: gotPID})
	}
	return out
}

// parseVIDPID pulls vid_XXXX / pid_XXXX (hex) out of a WinUSB device-interface path.
func parseVIDPID(path string) (vid, pid uint16, ok bool) {
	lp := toLowerASCII(path)
	v := hexAfter(lp, "vid_")
	p := hexAfter(lp, "pid_")
	if v < 0 || p < 0 {
		return 0, 0, false
	}
	return uint16(v), uint16(p), true
}

// hexAfter parses the 4 hex digits following marker in s, or -1.
func hexAfter(s, marker string) int {
	i := strings.Index(s, marker)
	if i < 0 || i+len(marker)+4 > len(s) {
		return -1
	}
	val := 0
	for _, c := range s[i+len(marker) : i+len(marker)+4] {
		var d int
		switch {
		case c >= '0' && c <= '9':
			d = int(c - '0')
		case c >= 'a' && c <= 'f':
			d = int(c-'a') + 10
		default:
			return -1
		}
		val = val*16 + d
	}
	return val
}

// pathLocation hashes a device-interface path to a stable per-port location id (FNV-1a).
// The path encodes the hub/port topology, so it survives a replug.
func pathLocation(path string) uint32 {
	const offset, prime = 2166136261, 16777619
	h := uint32(offset)
	for i := 0; i < len(path); i++ {
		h ^= uint32(path[i])
		h *= prime
	}
	return h
}

// openWinUSB opens the first WinUSB-bound device matching vid/pid that can be opened: a node
// already held by another process (or vanished) is skipped and the next tried, so two identical
// cameras with the first busy still open (as the Linux backend does). The last error is
// reported when none opens.
func openWinUSB(vid, pid uint16) (*winusbDevice, error) {
	nodes := scanWinUSB(vid, pid)
	if len(nodes) == 0 {
		return nil, fmt.Errorf("astrocam: no WinUSB device for %04x:%04x (bind it with WinUSB / libusbK via Zadig)", vid, pid)
	}
	var lastErr error
	for _, n := range nodes {
		d, err := openPath(n.path)
		if err == nil {
			return d, nil
		}
		lastErr = err
	}
	return nil, fmt.Errorf("astrocam: %d WinUSB device(s) for %04x:%04x, none could be opened: %w", len(nodes), vid, pid, lastErr)
}

// interfacePath fetches the device-interface path (SetupDiGetDeviceInterfaceDetailW
// with the two-call size dance).
func interfacePath(h uintptr, ifData *spDeviceInterfaceData) string {
	var needed uint32
	procGetInterfaceDtl.Call(h, uintptr(unsafe.Pointer(ifData)), 0, 0, uintptr(unsafe.Pointer(&needed)), 0)
	if needed == 0 {
		return ""
	}
	buf := make([]byte, needed)
	// SP_DEVICE_INTERFACE_DETAIL_DATA_W: cbSize (uint32) then WCHAR DevicePath[].
	*(*uint32)(unsafe.Pointer(&buf[0])) = 8 // fixed header size on 64-bit (4 + 2 padded)
	r, _, _ := procGetInterfaceDtl.Call(h, uintptr(unsafe.Pointer(ifData)), uintptr(unsafe.Pointer(&buf[0])), uintptr(needed), 0, 0)
	if r == 0 {
		return ""
	}
	return utf16BytesToString(buf[4:])
}

func openPath(path string) (*winusbDevice, error) {
	p, err := syscall.UTF16PtrFromString(path)
	if err != nil {
		return nil, err
	}
	h, _, callErr := procCreateFile.Call(uintptr(unsafe.Pointer(p)), genericReadWrite, fileShareRW, 0, openExisting, fileFlagOverlapped, 0)
	if h == ^uintptr(0) {
		return nil, fmt.Errorf("astrocam: CreateFile %q failed: %v", path, callErr)
	}
	d := &winusbDevice{handle: syscall.Handle(h), inPace: inPacer{min: usb2InPace}}
	if r, _, callErr := procWinUsbInit.Call(h, uintptr(unsafe.Pointer(&d.winusb))); r == 0 {
		procCloseHandle.Call(h)
		return nil, fmt.Errorf("astrocam: WinUsb_Initialize failed: %v", callErr)
	}
	// EP0 timeout on the default control pipe: 500 ms as on Linux, so a wedged control plane
	// errors out under ioMu instead of blocking the caller for WinUSB's infinite default.
	ep0Timeout := uint32(500)
	procWinUsbSetPipePol.Call(d.winusb, 0, pipeTransferTimeout, 4, uintptr(unsafe.Pointer(&ep0Timeout)))
	// Negotiated link speed from the bulk-IN pipe's max packet size, so a USB3 camera on a
	// HighSpeed link takes the USB2 bandwidth budget instead of overrunning the bus and shearing
	// the frame.
	for idx := 0; idx < 16; idx++ {
		var pi winusbPipeInfo
		r, _, _ := procWinUsbQueryPipe.Call(d.winusb, 0, uintptr(idx), uintptr(unsafe.Pointer(&pi)))
		if r == 0 {
			break // no more pipes
		}
		if pi.PipeId == bulkEndpoint {
			d.inMaxPacket = pi.MaximumPacketSize
			break
		}
	}
	return d, nil
}

// winusbPipeInfo is WINUSB_PIPE_INFORMATION (12 bytes incl. padding): PipeType(4), PipeId(1),
// pad(1), MaximumPacketSize(2), Interval(1), pad(3).
type winusbPipeInfo struct {
	PipeType          uint32
	PipeId            uint8
	_                 uint8
	MaximumPacketSize uint16
	Interval          uint8
	_                 [3]uint8
}

// SuperSpeed reports whether the bulk-IN pipe negotiated USB3 SuperSpeed (≥1024-byte max
// packet); false = USB2 HighSpeed. The Camera readout's bandwidth budget follows this, not
// the model capability.
func (d *winusbDevice) SuperSpeed() bool { return d.inMaxPacket >= 1024 }

// winusbSetupPacket is WINUSB_SETUP_PACKET (8 bytes, matches a USB setup packet).
type winusbSetupPacket struct {
	RequestType uint8
	Request     uint8
	Value       uint16
	Index       uint16
	Length      uint16
}

func (d *winusbDevice) control(reqType, bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	release, err := d.enter()
	if err != nil {
		return 0, err
	}
	defer release()
	d.ioMu.Lock()
	defer d.ioMu.Unlock()
	if reqType&0x80 != 0 && d.readActive.Load() > 0 && !d.SuperSpeed() {
		d.inPace.wait() // a USB2 readout in flight: pace EP0 reads (usb2InPace)
	}
	return d.controlRaw(reqType, bRequest, wValue, wIndex, data)
}

// ControlOutUngated implements UngatedControlSender: an ST4 pulse edge issued without ioMu, so
// it is not queued behind a frame read holding the gate. ST4 only (see UngatedControlSender).
func (d *winusbDevice) ControlOutUngated(bRequest uint8, wValue, wIndex uint16) error {
	release, err := d.enter()
	if err != nil {
		return err
	}
	defer release()
	_, err = d.controlRaw(0x40, bRequest, wValue, wIndex, nil)
	return err
}

// controlRaw issues the control transfer with no gate; callers hold closeMu (enter) and, except
// the ungated ST4 sender, ioMu.
func (d *winusbDevice) controlRaw(reqType, bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	pkt := winusbSetupPacket{RequestType: reqType, Request: bRequest, Value: wValue, Index: wIndex, Length: uint16(len(data))}
	var transferred uint32
	var bufPtr uintptr
	if len(data) > 0 {
		bufPtr = uintptr(unsafe.Pointer(&data[0]))
	}
	r, _, callErr := procWinUsbControl.Call(d.winusb, *(*uintptr)(unsafe.Pointer(&pkt)), bufPtr, uintptr(len(data)), uintptr(unsafe.Pointer(&transferred)), 0)
	runtime.KeepAlive(data) // bufPtr is a bare uintptr into data across the blocking call
	if r == 0 {
		return 0, fmt.Errorf("astrocam: WinUsb_ControlTransfer (req 0x%02x) failed: %v", bRequest, callErr)
	}
	return int(transferred), nil
}

func (d *winusbDevice) ControlOut(bRequest uint8, wValue, wIndex uint16, data []byte) error {
	_, err := d.control(0x40, bRequest, wValue, wIndex, data)
	return err
}

func (d *winusbDevice) ControlIn(bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	return d.control(0xC0, bRequest, wValue, wIndex, data)
}

// setPipeTimeout bounds a bulk-IN read: WinUSB cancels a ReadPipe that hasn't completed
// within timeoutMs (0 = block forever, the default).
func (d *winusbDevice) setPipeTimeout(timeoutMs uint32) {
	procWinUsbSetPipePol.Call(d.winusb, pipeIn, pipeTransferTimeout, 4, uintptr(unsafe.Pointer(&timeoutMs)))
}

// readChunk issues one overlapped ReadPipe of up to len(buf) bytes, bounded by timeoutMs, and
// returns the count transferred. A timeout or cancel returns (n, nil), and only a pipe error
// returns non-nil. Default WinUSB (no RAW_IO) returns on a short packet, so n may be < len(buf).
//
// Two rules of the overlapped sequence. The ReadPipe return must be checked: a synchronous
// failure never arms the OVERLAPPED, and GetOverlappedResult on a zero OVERLAPPED reports
// STATUS_SUCCESS with 0 bytes, presenting a dead device as a clean empty frame. The wait must be
// on the transfer's own event: without an hEvent it lands on the file handle, which any other
// completing I/O on the device (a concurrent TEC poll) signals while this read is still in flight
// against the caller's buffer.
func (d *winusbDevice) readChunk(buf []byte, timeoutMs uint32) (int, error) {
	if d.broken.Load() {
		return 0, errWinTransportBroken
	}
	if d.readAborted.Load() {
		return 0, nil // abort latched: don't arm a new transfer
	}
	d.setPipeTimeout(timeoutMs)
	ev, _, _ := procCreateEvent.Call(0, 1, 0, 0) // manual-reset, unsignaled, unnamed
	if ev == 0 {
		return 0, fmt.Errorf("astrocam: CreateEvent failed")
	}
	// Heap-allocate the OVERLAPPED: the kernel writes it asynchronously after ReadPipe
	// returns, and a stack copy can move under a growing goroutine stack.
	ov := &overlapped{HEvent: syscall.Handle(ev)}
	r, _, callErr := procWinUsbReadPipe.Call(d.winusb, pipeIn, uintptr(unsafe.Pointer(&buf[0])), uintptr(len(buf)), 0, uintptr(unsafe.Pointer(ov)))
	if r == 0 {
		if errno, ok := callErr.(syscall.Errno); !ok || errno != errIOPending {
			procCloseHandle.Call(ev)
			return 0, fmt.Errorf("astrocam: WinUsb_ReadPipe failed: %v", callErr)
		}
	}
	// Wait on this transfer's event in ≤500 ms slices so an AbortRead can break the wait. The
	// pipe's transfer-timeout policy bounds the transfer itself, completing it with
	// ERROR_SEM_TIMEOUT. An abort cancels through WinUsb_AbortPipe and still waits for the
	// completion, since the kernel owns buf and ov until then.
	for {
		if w, _, _ := procWaitForSingle.Call(ev, 500); w == waitObject0 {
			break
		}
		if d.readAborted.Load() {
			procWinUsbAbortPipe.Call(d.winusb, pipeIn)
			if w, _, _ := procWaitForSingle.Call(ev, uintptr(winDrainTimeout.Milliseconds())); w != waitObject0 {
				d.broken.Store(true)
				winLeakedIOMu.Lock()
				winLeakedIO = append(winLeakedIO, buf, ov)
				winLeakedIOMu.Unlock()
				return 0, errWinTransportBroken // event leaked too; signaled at (eventual) completion
			}
			break
		}
	}
	var transferred uint32
	r, _, callErr = procWinUsbGetOvlRes.Call(d.winusb, uintptr(unsafe.Pointer(ov)), uintptr(unsafe.Pointer(&transferred)), 0)
	runtime.KeepAlive(buf)
	runtime.KeepAlive(ov)
	procCloseHandle.Call(ev)
	if r != 0 {
		return int(transferred), nil
	}
	if errno, ok := callErr.(syscall.Errno); ok {
		switch errno {
		case errSemTimeout, errOpAborted, errTimeout:
			return int(transferred), nil // bounded read elapsed: no (more) data this round
		}
	}
	return int(transferred), fmt.Errorf("astrocam: ReadPipe failed: %v", callErr)
}

// winOvSlot is one armed overlapped read of the prequeued batch: its OVERLAPPED + event and
// the caller-frame slice the kernel DMAs into.
type winOvSlot struct {
	ov    *overlapped
	ev    uintptr
	buf   []byte
	armed bool
}

// ReadFrameStreamPrequeued reads one frame with the SDK's async-transfer model: the batchRead
// engine with no quiet window. Gates: closeMu shared, ioMu for the whole frame.
func (d *winusbDevice) ReadFrameStreamPrequeued(buf []byte, idle, total time.Duration) (int, error) {
	return d.batchRead(buf, idle, total, 0)
}

// BulkRead reads one frame as a prequeued batch covering the frame exactly, so the pipe never
// idles between transfers and the last slice is the exact remainder. A timeout, frame end or
// abort returns the prefix with a nil error. Gates: closeMu shared, ioMu for the whole frame.
func (d *winusbDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	return d.batchRead(buf, timeout, timeout, 0)
}

// BulkReadQuiet arms the batch at once, since the GPIF never streams without a reader, but takes
// ioMu only when quiet elapses or the first slice completes, so TEC polls, telemetry and ST4
// pulses flow during a host-timed integration. quiet 0 is BulkRead.
func (d *winusbDevice) BulkReadQuiet(buf []byte, quiet, timeout time.Duration) (int, error) {
	return d.batchRead(buf, timeout, timeout, quiet)
}

// batchRead is the prequeued batch engine behind BulkRead, BulkReadQuiet and
// ReadFrameStreamPrequeued: 1 MiB overlapped slices covering buf exactly, winPrequeuedWindow of
// them armed at once, each filled slice arming the next. total bounds the whole read, idle gates
// after the first byte, and quiet is the ungated window before ioMu. A short slot (the FX3
// frame-end short packet), a gate expiry or an AbortRead ends the read: everything in flight is
// aborted and drained, and the contiguous prefix comes back with a nil error. A drain timeout
// pins the frame buffer and poisons the transport. Gates: closeMu shared, ioMu from the end of
// the quiet window.
func (d *winusbDevice) batchRead(buf []byte, idle, total, quiet time.Duration) (int, error) {
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
	// Non-positive bounds take the shared defaults rather than reading as already expired.
	if idle <= 0 {
		idle = defaultIdleBound
	}
	if total <= 0 {
		total = defaultTotalBound
	}
	gated := false
	gate := func() {
		if !gated {
			d.ioMu.Lock()
			gated = true
		}
	}
	defer func() {
		if gated {
			d.ioMu.Unlock()
		}
	}()
	if quiet <= 0 {
		gate()
	}

	// Exact cover: 1 MiB slices, last = remainder.
	nslots := (len(buf) + maxBulkChunk - 1) / maxBulkChunk
	slots := make([]winOvSlot, nslots)
	for i := range slots {
		lo := i * maxBulkChunk
		hi := lo + maxBulkChunk
		if hi > len(buf) {
			hi = len(buf)
		}
		slots[i].buf = buf[lo:hi]
	}

	// The driver-side transfer timeout outlasts the host gates below; the host reaps with its
	// own total/idle windows and aborts explicitly.
	totalMs := total.Milliseconds()
	if totalMs <= 0 {
		totalMs = 1
	}
	d.setPipeTimeout(uint32(totalMs) + 1000)

	arm := func(i int) error {
		ev, _, _ := procCreateEvent.Call(0, 1, 0, 0)
		if ev == 0 {
			return fmt.Errorf("astrocam: CreateEvent failed")
		}
		slots[i].ev = ev
		slots[i].ov = &overlapped{HEvent: syscall.Handle(ev)}
		r, _, callErr := procWinUsbReadPipe.Call(d.winusb, pipeIn,
			uintptr(unsafe.Pointer(&slots[i].buf[0])), uintptr(len(slots[i].buf)),
			0, uintptr(unsafe.Pointer(slots[i].ov)))
		if r == 0 {
			if errno, ok := callErr.(syscall.Errno); !ok || errno != errIOPending {
				procCloseHandle.Call(ev)
				slots[i].ev = 0
				return fmt.Errorf("astrocam: WinUsb_ReadPipe failed: %v", callErr)
			}
		}
		slots[i].armed = true
		return nil
	}

	// abortAndDrain cancels everything still in flight and waits for every armed slot to complete,
	// since the kernel writes the buffer and OVERLAPPED at completion time. A drain timeout pins
	// the whole batch and poisons the transport instead of returning that memory to the caller.
	drained := true
	abortAndDrain := func() {
		procWinUsbAbortPipe.Call(d.winusb, pipeIn)
		dl := time.Now().Add(winDrainTimeout)
		for i := range slots {
			if !slots[i].armed {
				continue
			}
			ms := time.Until(dl).Milliseconds()
			if ms < 0 {
				ms = 0
			}
			if w, _, _ := procWaitForSingle.Call(slots[i].ev, uintptr(ms)); w != waitObject0 {
				drained = false
				break
			}
		}
		if !drained {
			d.broken.Store(true)
			winLeakedIOMu.Lock()
			winLeakedIO = append(winLeakedIO, buf, slots)
			winLeakedIOMu.Unlock()
			return // leak the events too; the kernel signals them at (eventual) completion
		}
		for i := range slots {
			if slots[i].ev != 0 {
				procCloseHandle.Call(slots[i].ev)
				slots[i].ev = 0
			}
		}
	}

	window := winPrequeuedWindow
	if window > nslots {
		window = nslots
	}
	next := 0 // next slot to arm
	for ; next < window; next++ {
		if err := arm(next); err != nil {
			if next == 0 {
				return 0, err
			}
			break // keep what's armed; the reap below drains it
		}
	}

	deadline := time.Now().Add(total)
	if !gated {
		// Quiet window: the slices are armed; wait for it to elapse, or for the first slice to
		// complete (data arriving early), or an abort, then take the gate for the readout.
		qEnd := time.Now().Add(quiet)
		for {
			if d.readAborted.Load() {
				break
			}
			wait := time.Until(qEnd)
			if wait <= 0 {
				break
			}
			if wait > 50*time.Millisecond {
				wait = 50 * time.Millisecond
			}
			if wait < time.Millisecond {
				wait = time.Millisecond
			}
			if w, _, _ := procWaitForSingle.Call(slots[0].ev, uintptr(wait.Milliseconds())); w == waitObject0 {
				break // first slice done: data is flowing
			}
		}
		gate()
	}
	got := 0
	gotFirst := false
	for i := 0; i < next; i++ {
		// Host-side gate: total before the first byte, idle after. The wait runs in ≤500 ms
		// slices so an AbortRead breaks it promptly.
		gateEnd := deadline
		if gotFirst {
			if ie := time.Now().Add(idle); ie.Before(gateEnd) {
				gateEnd = ie
			}
		}
		signaled := false
		for !signaled {
			if d.readAborted.Load() {
				break // AbortRead: abort + drain below, short prefix out
			}
			wait := time.Until(gateEnd)
			if wait <= 0 {
				break
			}
			if wait > 500*time.Millisecond {
				wait = 500 * time.Millisecond
			}
			if wait < time.Millisecond {
				wait = time.Millisecond
			}
			if w, _, _ := procWaitForSingle.Call(slots[i].ev, uintptr(wait.Milliseconds())); w == waitObject0 {
				signaled = true
			}
		}
		if !signaled {
			abortAndDrain()
			if !drained {
				return got, errWinTransportBroken
			}
			runtime.KeepAlive(slots)
			return got, nil // gate expired or aborted: short prefix, the worker recovers
		}
		var transferred uint32
		r, _, callErr := procWinUsbGetOvlRes.Call(d.winusb, uintptr(unsafe.Pointer(slots[i].ov)), uintptr(unsafe.Pointer(&transferred)), 0)
		got += int(transferred)
		if transferred > 0 {
			gotFirst = true
		}
		if r == 0 {
			// Completed with an error: driver timeout/abort ends the contiguous prefix
			// cleanly; anything else is a hard pipe error.
			abortAndDrain()
			if !drained {
				return got, errWinTransportBroken
			}
			runtime.KeepAlive(slots)
			if errno, ok := callErr.(syscall.Errno); ok {
				switch errno {
				case errSemTimeout, errOpAborted, errTimeout:
					return got, nil
				}
			}
			return got, fmt.Errorf("astrocam: ReadPipe failed: %v", callErr)
		}
		if int(transferred) < len(slots[i].buf) {
			// Short slot = the FX3 frame-end short packet: the prefix is the frame.
			abortAndDrain()
			if !drained {
				return got, errWinTransportBroken
			}
			runtime.KeepAlive(slots)
			return got, nil
		}
		// Slot fully filled: keep the window covered.
		if next < nslots {
			if err := arm(next); err == nil {
				next++
			}
		}
	}
	// Every armed slot completed full: nothing in flight, just release the events.
	for i := range slots {
		if slots[i].ev != 0 {
			procCloseHandle.Call(slots[i].ev)
			slots[i].ev = 0
		}
	}
	runtime.KeepAlive(buf)
	runtime.KeepAlive(slots)
	return got, nil
}

// ReadFrameStream reads one whole frame for the DDR cameras, which hold the frame's final partial
// buffer until FPGABufReload. The worker pulses that for the duration of this call, and mid-frame
// short packets and ZLPs are not terminal. It cycles maxBulkChunk reads into scratch to a
// watermark until the frame is in, idle passes with no data, or the total deadline hits. Gates:
// closeMu shared only.
func (d *winusbDevice) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
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
		return 0, nil // abort latched: fail fast
	}
	deadline := time.Now().Add(total)
	scratch := make([]byte, maxBulkChunk)
	got := 0
	lastData := time.Now()
	for got < len(buf) {
		rem := time.Until(deadline)
		if rem <= 0 {
			break
		}
		if d.readAborted.Load() {
			break // AbortRead: return the short prefix
		}
		ms := rem
		if ms > idle {
			ms = idle
		}
		msec := uint32(ms.Milliseconds())
		if msec == 0 {
			msec = 1 // <1 ms remaining truncates to 0 = INFINITE for WinUSB PIPE_TRANSFER_TIMEOUT
		}
		n, err := d.readChunk(scratch, msec)
		if n > 0 {
			got += copy(buf[got:], scratch[:n])
			lastData = time.Now()
			continue
		}
		if err != nil {
			return got, err
		}
		if time.Since(lastData) >= idle {
			break // stall: no data for the whole idle window
		}
	}
	return got, nil
}

// ResetEndpoint resets the bulk-IN pipe, so stale pipe state from a prior aborted read cannot
// fail the next transfer. WinUsb_ResetPipe is a CLEAR_FEATURE(ENDPOINT_HALT) on EP0, so it takes
// ioMu like every other control transfer. Gates: closeMu shared, ioMu.
func (d *winusbDevice) ResetEndpoint(ep uint8) error {
	release, err := d.enter()
	if err != nil {
		return err
	}
	defer release()
	d.ioMu.Lock()
	defer d.ioMu.Unlock()
	if r, _, callErr := procWinUsbResetPipe.Call(d.winusb, uintptr(ep)); r == 0 {
		return fmt.Errorf("astrocam: WinUsb_ResetPipe(0x%02x) failed: %v", ep, callErr)
	}
	return nil
}

// Close waits for in-flight I/O (closeMu exclusive), then releases the WinUSB interface and
// the file handle. On a broken transport the handles are leaked (see broken). Idempotent.
func (d *winusbDevice) Close() error {
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
	open := make([]*winusbStream, 0, len(d.streams))
	for st := range d.streams {
		open = append(open, st)
	}
	d.streamMu.Unlock()
	for _, st := range open {
		_ = st.Close()
	}
	d.closed = true
	if d.broken.Load() {
		return errWinTransportBroken
	}
	procWinUsbFree.Call(d.winusb)
	procCloseHandle.Call(uintptr(d.handle))
	return nil
}

// OpenHost opens the default USB backend for this platform (Windows: WinUSB).
func OpenHost(vid, pid uint16) (Transport, error) {
	d, err := openWinUSB(vid, pid)
	if err != nil {
		return nil, err
	}
	return d, nil
}

// enumerateRaw lists every VID-matched WinUSB device without opening it (the shared
// Enumerate filters these to known camera PIDs). Name is left empty (filterCameras fills it
// from the model registry); Location is the path hash, the key OpenLocation reopens by.
func enumerateRaw(vid uint16) ([]DeviceInfo, error) {
	var out []DeviceInfo
	for _, n := range scanWinUSB(vid, 0) {
		out = append(out, DeviceInfo{VID: n.vid, PID: n.pid, Location: pathLocation(n.path)})
	}
	return out, nil
}

// OpenLocation opens the WinUSB device whose path hashes to loc (from a DeviceInfo): it binds
// the exact unit chosen from Enumerate when several identical cameras are attached.
func OpenLocation(vid uint16, loc uint32) (Transport, error) {
	for _, n := range scanWinUSB(vid, 0) {
		if pathLocation(n.path) == loc {
			return openPath(n.path)
		}
	}
	return nil, fmt.Errorf("astrocam: no WinUSB device at location 0x%08x for vid %04x (unplugged or moved)", loc, vid)
}

type overlapped struct {
	Internal     uintptr
	InternalHigh uintptr
	Offset       uint32
	OffsetHigh   uint32
	HEvent       syscall.Handle
}

// --- small helpers (avoid pulling extra deps) ---

func utf16BytesToString(b []byte) string {
	u := make([]uint16, 0, len(b)/2)
	for i := 0; i+1 < len(b); i += 2 {
		c := uint16(b[i]) | uint16(b[i+1])<<8
		if c == 0 {
			break
		}
		u = append(u, c)
	}
	return syscall.UTF16ToString(u)
}

func toLowerASCII(s string) string {
	b := []byte(s)
	for i, c := range b {
		if c >= 'A' && c <= 'Z' {
			b[i] = c + 32
		}
	}
	return string(b)
}
