//go:build windows

// Pure-Go Windows USB transport over WinUSB, no cgo (stdlib syscall + lazy DLLs): control
// via WinUsb_ControlTransfer and bulk-IN frame reads via overlapped WinUsb_ReadPipe, bounded
// by the pipe's transfer-timeout policy.
//
// The camera must be bound to WinUSB/libusbK (the ZWO installer or Zadig). The device is
// matched by VID/PID in its device-interface path under the generic USB device interface GUID.
//
// Frame reads use fixed 1 MiB chunked reads assembled to a contiguous watermark (BulkRead /
// ReadFrameStream): a whole-frame read must time-bound (default WinUSB blocks forever) and
// read through the FX3's mid-frame short packets / ZLPs rather than stop on the first.
//
// Compile-checked for windows/amd64 but not run on hardware. Mirrors the Linux usbfs backend.

package astrocam

import (
	"fmt"
	"strings"
	"sync"
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
	procCreateFile       = modKernel32.NewProc("CreateFileW")
	procCloseHandle      = modKernel32.NewProc("CloseHandle")
	procGetOverlappedRes = modKernel32.NewProc("GetOverlappedResult")
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

	// maxBulkChunk caps a single ReadPipe so a whole-frame read time-bounds and reads
	// through mid-frame short packets at a contiguous watermark; 1 MiB matches the
	// SDK/darwin/usbfs xferLen.
	maxBulkChunk = 1 << 20

	// Windows error codes that mean "the bounded read elapsed", not a pipe failure.
	errSemTimeout = 121  // ERROR_SEMAPHORE_TIMEOUT
	errOpAborted  = 995  // ERROR_OPERATION_ABORTED
	errTimeout    = 1460 // ERROR_TIMEOUT
)

// winusbDevice is a WinUSB-backed Transport for one open camera.
type winusbDevice struct {
	handle syscall.Handle // file handle
	winusb uintptr        // WINUSB_INTERFACE_HANDLE
	// ioMu serializes every USB I/O the FX3 must not see interleaved: all control transfers
	// and a whole-frame BulkRead hold it, so a control transfer can't land on the bridge
	// mid-readout and wedge the un-buffered USB2 path. ReadFrameStream (the USB3/DDR path)
	// stays unlocked — it needs the worker's concurrent FPGABufReload. See transport_linux.go
	// (usbfsDevice.ioMu) for the full rationale.
	ioMu sync.Mutex
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

// OpenWinUSB finds the first WinUSB-bound device matching vid/pid and opens it.
func OpenWinUSB(vid, pid uint16) (*winusbDevice, error) {
	for _, n := range scanWinUSB(vid, pid) {
		return openPath(n.path)
	}
	return nil, fmt.Errorf("asicam: no WinUSB device for %04x:%04x (bind it with WinUSB / libusbK via Zadig)", vid, pid)
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
	h, _, _ := procCreateFile.Call(uintptr(unsafe.Pointer(p)), genericReadWrite, fileShareRW, 0, openExisting, fileFlagOverlapped, 0)
	if h == ^uintptr(0) {
		return nil, fmt.Errorf("asicam: CreateFile %q failed", path)
	}
	d := &winusbDevice{handle: syscall.Handle(h)}
	if r, _, _ := procWinUsbInit.Call(h, uintptr(unsafe.Pointer(&d.winusb))); r == 0 {
		procCloseHandle.Call(h)
		return nil, fmt.Errorf("asicam: WinUsb_Initialize failed")
	}
	return d, nil
}

// winusbSetupPacket is WINUSB_SETUP_PACKET (8 bytes, matches a USB setup packet).
type winusbSetupPacket struct {
	RequestType uint8
	Request     uint8
	Value       uint16
	Index       uint16
	Length      uint16
}

func (d *winusbDevice) control(reqType, bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	d.ioMu.Lock()
	defer d.ioMu.Unlock()
	pkt := winusbSetupPacket{RequestType: reqType, Request: bRequest, Value: wValue, Index: wIndex, Length: uint16(len(data))}
	var transferred uint32
	var bufPtr uintptr
	if len(data) > 0 {
		bufPtr = uintptr(unsafe.Pointer(&data[0]))
	}
	r, _, _ := procWinUsbControl.Call(d.winusb, *(*uintptr)(unsafe.Pointer(&pkt)), bufPtr, uintptr(len(data)), uintptr(unsafe.Pointer(&transferred)), 0)
	if r == 0 {
		return 0, fmt.Errorf("asicam: WinUsb_ControlTransfer failed")
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

// readChunk issues one overlapped ReadPipe of up to len(buf) bytes, bounded by timeoutMs,
// and returns the count transferred. A timeout/cancel returns (n, nil); only a genuine pipe
// error returns non-nil. Default WinUSB (no RAW_IO) returns on a short packet, so n may be
// < len(buf).
func (d *winusbDevice) readChunk(buf []byte, timeoutMs uint32) (int, error) {
	d.setPipeTimeout(timeoutMs)
	var ov overlapped
	procWinUsbReadPipe.Call(d.winusb, pipeIn, uintptr(unsafe.Pointer(&buf[0])), uintptr(len(buf)), 0, uintptr(unsafe.Pointer(&ov)))
	var transferred uint32
	r, _, callErr := procGetOverlappedRes.Call(uintptr(d.handle), uintptr(unsafe.Pointer(&ov)), uintptr(unsafe.Pointer(&transferred)), 1)
	if r != 0 {
		return int(transferred), nil
	}
	if errno, ok := callErr.(syscall.Errno); ok {
		switch errno {
		case 0, errSemTimeout, errOpAborted, errTimeout:
			return int(transferred), nil // bounded read elapsed: no (more) data this round
		}
	}
	return int(transferred), fmt.Errorf("asicam: ReadPipe failed: %v", callErr)
}

// BulkRead reads one frame from the bulk-IN endpoint into buf, in fixed maxBulkChunk reads
// into a scratch buffer copied to a running watermark — so the read time-bounds (default
// WinUSB blocks forever) and assembles the frame contiguously across the FX3's mid-frame
// short packets, which a single whole-frame ReadPipe would stop on.
func (d *winusbDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	d.ioMu.Lock() // hold for the whole frame so no control transfer interleaves (USB2 wedge gate)
	defer d.ioMu.Unlock()
	deadline := time.Now().Add(timeout)
	scratch := make([]byte, maxBulkChunk)
	total := 0
	for total < len(buf) {
		ms := time.Until(deadline).Milliseconds()
		if ms <= 0 {
			break
		}
		n, err := d.readChunk(scratch, uint32(ms))
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

// ReadFrameStream reads one whole frame, treating the FX3's mid-frame short packets and ZLPs
// as non-terminal (DDR cameras hold the frame's final partial buffer until FPGABufReload,
// which the worker pulses for the duration of this call). It cycles maxBulkChunk reads into
// scratch to a watermark until the frame is in, `idle` passes with no data (a genuine
// stall), or the `total` deadline hits. winusbDevice satisfies FrameStreamer.
func (d *winusbDevice) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
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
			break
		}
		ms := rem
		if ms > idle {
			ms = idle
		}
		n, err := d.readChunk(scratch, uint32(ms.Milliseconds()))
		if n > 0 {
			got += copy(buf[got:], scratch[:n])
			lastData = time.Now()
			continue
		}
		if err != nil {
			return got, err
		}
		if time.Since(lastData) >= idle {
			break // genuine stall: no data for the whole idle window
		}
	}
	return got, nil
}

// ResetEndpoint clears/aborts the bulk-IN pipe (WinUsb_ResetPipe) so a stale pipe state
// from a prior aborted read can't fail the next transfer.
func (d *winusbDevice) ResetEndpoint(ep uint8) error {
	if r, _, _ := procWinUsbResetPipe.Call(d.winusb, uintptr(ep)); r == 0 {
		return fmt.Errorf("asicam: WinUsb_ResetPipe(0x%02x) failed", ep)
	}
	return nil
}

func (d *winusbDevice) Close() error {
	procWinUsbFree.Call(d.winusb)
	procCloseHandle.Call(uintptr(d.handle))
	return nil
}

// OpenHost opens the default USB backend for this platform (Windows: WinUSB).
func OpenHost(vid, pid uint16) (Transport, error) {
	d, err := OpenWinUSB(vid, pid)
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

// OpenLocation opens the WinUSB device whose path hashes to loc (from a DeviceInfo) — binds
// the exact unit chosen from Enumerate when several identical cameras are attached.
func OpenLocation(vid uint16, loc uint32) (Transport, error) {
	for _, n := range scanWinUSB(vid, 0) {
		if pathLocation(n.path) == loc {
			return openPath(n.path)
		}
	}
	return nil, fmt.Errorf("asicam: no WinUSB device at location 0x%08x for vid %04x (unplugged or moved)", loc, vid)
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
