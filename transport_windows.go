//go:build windows

// Pure-Go Windows USB transport over WinUSB, no cgo (stdlib syscall + lazy DLLs).
// WinUSB is the fastest cgo-free path on Windows: control via WinUsb_ControlTransfer
// and a bulk stream of overlapped WinUsb_ReadPipe reads reaped through an I/O
// completion port — the same submit-window/reap model as startAsyncXfer.
//
// The camera must be bound to WinUSB/libusbK (the ZWO installer or Zadig). We match
// the device by VID/PID in its device-interface path under the generic USB device
// interface GUID.
//
// UNVERIFIED: compile-checked for windows/amd64 but not run on hardware. Validate
// on a Windows box with a WinUSB-bound camera before trusting it.

package astrocam

import (
	"fmt"
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
	procCreateFile       = modKernel32.NewProc("CreateFileW")
	procCloseHandle      = modKernel32.NewProc("CloseHandle")
	procCreateIoCP       = modKernel32.NewProc("CreateIoCompletionPort")
	procGetQueuedCS      = modKernel32.NewProc("GetQueuedCompletionStatus")
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
	rawIOPolicy          = 0x07 // WINUSB pipe policy RAW_IO
	pipeIn               = bulkEndpoint
)

// winusbDevice is a WinUSB-backed Transport + BulkStreamer for one open camera.
type winusbDevice struct {
	handle syscall.Handle // file handle
	winusb uintptr        // WINUSB_INTERFACE_HANDLE
	// ctrlMu serializes control transfers — the capture path and the TEC loop drive
	// one open camera concurrently. Bulk reads (EP 0x81) are a separate pipe and stay
	// unlocked so a long readout can't stall a TEC tick. See transport_darwin.go.
	ctrlMu sync.Mutex
}

// OpenWinUSB finds the first WinUSB-bound device matching vid/pid and opens it.
func OpenWinUSB(vid, pid uint16) (*winusbDevice, error) {
	want := fmt.Sprintf("vid_%04x&pid_%04x", vid, pid)
	h, _, _ := procGetClassDevs.Call(uintptr(unsafe.Pointer(&usbDeviceGUID)), 0, 0, digcfPresent|digcfDeviceInterface)
	if h == 0 || h == ^uintptr(0) {
		return nil, fmt.Errorf("asicam: SetupDiGetClassDevs failed")
	}
	defer procDestroyDevInfo.Call(h)

	var idx uint32
	for ; ; idx++ {
		var ifData spDeviceInterfaceData
		ifData.cbSize = uint32(unsafe.Sizeof(ifData))
		r, _, _ := procEnumInterfaces.Call(h, 0, uintptr(unsafe.Pointer(&usbDeviceGUID)), uintptr(idx), uintptr(unsafe.Pointer(&ifData)))
		if r == 0 {
			break // ERROR_NO_MORE_ITEMS
		}
		path := interfacePath(h, &ifData)
		if path == "" || !containsFold(path, want) {
			continue
		}
		dev, err := openPath(path)
		if err != nil {
			return nil, err
		}
		return dev, nil
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
	d.ctrlMu.Lock()
	defer d.ctrlMu.Unlock()
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

func (d *winusbDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	var ov overlapped
	var bufPtr uintptr
	if len(buf) > 0 {
		bufPtr = uintptr(unsafe.Pointer(&buf[0]))
	}
	procWinUsbReadPipe.Call(d.winusb, pipeIn, bufPtr, uintptr(len(buf)), 0, uintptr(unsafe.Pointer(&ov)))
	n, err := d.waitOverlapped(&ov, timeout)
	return n, err
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

// enumerateRaw / OpenLocation: not implemented on Windows yet. The WinUSB backend can
// list ZWO devices via SetupDiGetClassDevs on the device interface GUID (parsing the
// instance id for VID/PID and using it as the location) — a follow-up to match darwin.
func enumerateRaw(uint16) ([]DeviceInfo, error)      { return nil, errEnumUnsupported }
func OpenLocation(uint16, uint32) (Transport, error) { return nil, errEnumUnsupported }

type overlapped struct {
	Internal     uintptr
	InternalHigh uintptr
	Offset       uint32
	OffsetHigh   uint32
	HEvent       syscall.Handle
}

// waitOverlapped blocks for a single overlapped read to complete and returns the
// transferred length. timeout is not honored here (the high-rate path is
// BulkStream, which reaps via the IOCP with a real timeout); BulkRead is the
// simple one-shot fallback.
func (d *winusbDevice) waitOverlapped(ov *overlapped, timeout time.Duration) (int, error) {
	_ = timeout
	var transferred uint32
	r, _, _ := procGetOverlappedRes.Call(uintptr(d.handle), uintptr(unsafe.Pointer(ov)), uintptr(unsafe.Pointer(&transferred)), 1)
	if r == 0 {
		return 0, fmt.Errorf("asicam: GetOverlappedResult failed")
	}
	return int(transferred), nil
}

// BulkStream submits nBuffers overlapped reads and reaps them through an IOCP,
// resubmitting on completion (the startAsyncXfer window model).
func (d *winusbDevice) BulkStream(bufSize, nBuffers int) (Stream, error) {
	iocp, _, _ := procCreateIoCP.Call(uintptr(d.handle), 0, 0, 0)
	if iocp == 0 {
		return nil, fmt.Errorf("asicam: CreateIoCompletionPort failed")
	}
	// RAW_IO pipe policy for max throughput (no buffering/short-read coalescing).
	var on uint8 = 1
	procWinUsbSetPipePol.Call(d.winusb, pipeIn, rawIOPolicy, 1, uintptr(unsafe.Pointer(&on)))

	s := &winusbStream{
		d:    d,
		iocp: syscall.Handle(iocp),
		bufs: make([][]byte, nBuffers),
		ovs:  make([]overlapped, nBuffers),
	}
	for i := range s.bufs {
		s.bufs[i] = make([]byte, bufSize)
		s.submit(i)
	}
	return s, nil
}

type winusbStream struct {
	d    *winusbDevice
	iocp syscall.Handle
	bufs [][]byte
	ovs  []overlapped
}

func (s *winusbStream) submit(i int) {
	s.ovs[i] = overlapped{}
	procWinUsbReadPipe.Call(s.d.winusb, pipeIn, uintptr(unsafe.Pointer(&s.bufs[i][0])), uintptr(len(s.bufs[i])), 0, uintptr(unsafe.Pointer(&s.ovs[i])))
}

func (s *winusbStream) Next(timeout time.Duration) ([]byte, error) {
	var nbytes uint32
	var key uintptr
	var povl uintptr
	r, _, _ := procGetQueuedCS.Call(uintptr(s.iocp), uintptr(unsafe.Pointer(&nbytes)), uintptr(unsafe.Pointer(&key)), uintptr(unsafe.Pointer(&povl)), uintptr(timeout.Milliseconds()))
	if r == 0 {
		return nil, fmt.Errorf("asicam: bulk stream wait failed/timeout")
	}
	// Match the completed OVERLAPPED by address, copy out, resubmit.
	for i := range s.ovs {
		if uintptr(unsafe.Pointer(&s.ovs[i])) != povl {
			continue
		}
		out := make([]byte, nbytes)
		copy(out, s.bufs[i][:nbytes])
		s.submit(i)
		return out, nil
	}
	return nil, fmt.Errorf("asicam: reaped unknown overlapped")
}

func (s *winusbStream) Close() error {
	procCloseHandle.Call(uintptr(s.iocp))
	return nil
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

func containsFold(s, sub string) bool {
	ls, lsub := toLowerASCII(s), toLowerASCII(sub)
	for i := 0; i+len(lsub) <= len(ls); i++ {
		if ls[i:i+len(lsub)] == lsub {
			return true
		}
	}
	return false
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
