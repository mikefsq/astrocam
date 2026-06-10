//go:build linux

// Pure-Go Linux USB transport over usbfs (/dev/bus/usb), no cgo. This is the
// fastest cgo-free backend: it drives the same kernel path libusb uses —
// USBDEVFS async URBs for the bulk stream — directly.
//
// UNVERIFIED: compile-checked for linux but not yet run on hardware. The usbfs
// ioctl numbers and struct layouts are transcribed from <linux/usbdevice_fs.h>
// for the 64-bit ABI; validate on a Pi (drop a udev rule granting access to the
// ZWO VID 0x03C3) before trusting it. Mirrors the async pump
// initAsyncXfer/startAsyncXfer (window of buffers in flight, reap by
// completion, resubmit).

package asicam

import (
	"fmt"
	"os"
	"path/filepath"
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
	usbdevfsSetConfig      = ioc(2, 5, 4)   // _IOR('U',5,uint)
	usbdevfsSubmitURB      = ioc(2, 10, 56) // _IOR('U',10,usbdevfs_urb)
	usbdevfsDiscardURB     = ioc(0, 11, 0)  // _IO('U',11)
	usbdevfsReapURBNDelay  = ioc(1, 13, 8)  // _IOW('U',13,void*)
	usbdevfsClaimInterface = ioc(2, 15, 4)  // _IOR('U',15,uint)
	usbdevfsClearHalt      = ioc(2, 21, 4)  // _IOR('U',21,uint)
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

type usbURB struct {
	typ           uint8
	endpoint      uint8
	_             [2]uint8 // pad to 4-align status
	status        int32
	flags         uint32
	_             uint32 // pad to 8-align buffer
	buffer        uintptr
	bufferLength  int32
	actualLength  int32
	startFrame    int32
	numberPackets int32
	errorCount    int32
	signr         uint32
	usercontext   uintptr
}

const urbTypeBulk = 3 // USBDEVFS_URB_TYPE_BULK

// usbfsDevice is a usbfs-backed Transport + BulkStreamer for one open camera.
type usbfsDevice struct {
	f *os.File
	// ctrlMu serializes control transfers — the capture path and the TEC loop drive
	// one open camera concurrently. Bulk URBs (EP 0x81) are a separate pipe and stay
	// unlocked so a long readout can't stall a TEC tick. See transport_darwin.go.
	ctrlMu sync.Mutex
}

// OpenUSBFS finds the first device matching vid/pid under /dev/bus/usb, claims
// interface 0, and returns a Transport. The ZWO VID is asicam.ZWO.VID (0x03C3).
func OpenUSBFS(vid, pid uint16) (*usbfsDevice, error) {
	nodes, _ := filepath.Glob("/dev/bus/usb/*/*")
	for _, n := range nodes {
		f, err := os.OpenFile(n, os.O_RDWR, 0)
		if err != nil {
			continue // permission / busy — skip
		}
		var desc [18]byte // device descriptor is the head of the node
		if _, err := f.ReadAt(desc[:], 0); err != nil {
			f.Close()
			continue
		}
		gotVID := uint16(desc[8]) | uint16(desc[9])<<8
		gotPID := uint16(desc[10]) | uint16(desc[11])<<8
		if gotVID != vid || gotPID != pid {
			f.Close()
			continue
		}
		d := &usbfsDevice{f: f}
		if err := d.claim(0); err != nil {
			f.Close()
			return nil, err
		}
		return d, nil
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

func (d *usbfsDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	bt := usbBulkTransfer{ep: bulkEndpoint, len: uint32(len(buf)), timeout: uint32(timeout.Milliseconds())}
	if len(buf) > 0 {
		bt.data = uintptr(unsafe.Pointer(&buf[0]))
	}
	r, _, errno := syscall.Syscall(syscall.SYS_IOCTL, d.f.Fd(), usbdevfsBulk, uintptr(unsafe.Pointer(&bt)))
	if errno != 0 {
		return 0, errno
	}
	return int(r), nil
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

// enumerateRaw / OpenLocation: not implemented on Linux yet. The usbfs backend can
// list ZWO devices by scanning /sys/bus/usb/devices (idVendor==03c3, reading busnum/
// devnum for the location) — a follow-up to match the darwin IORegistry enumeration.
func enumerateRaw(uint16) ([]DeviceInfo, error)      { return nil, errEnumUnsupported }
func OpenLocation(uint16, uint32) (Transport, error) { return nil, errEnumUnsupported }

// BulkStream implements the async pump: nBuffers URBs of bufSize on EP 0x81, all
// submitted up front and resubmitted as they complete (startAsyncXfer).
func (d *usbfsDevice) BulkStream(bufSize, nBuffers int) (Stream, error) {
	s := &usbfsStream{
		d:    d,
		urbs: make([]usbURB, nBuffers),
		bufs: make([][]byte, nBuffers),
	}
	for i := range s.urbs {
		s.bufs[i] = make([]byte, bufSize)
		s.urbs[i] = usbURB{
			typ:          urbTypeBulk,
			endpoint:     bulkEndpoint,
			buffer:       uintptr(unsafe.Pointer(&s.bufs[i][0])),
			bufferLength: int32(bufSize),
		}
		if err := d.ioctl(usbdevfsSubmitURB, unsafe.Pointer(&s.urbs[i])); err != nil {
			s.Close()
			return nil, fmt.Errorf("asicam: submit URB %d: %w", i, err)
		}
	}
	return s, nil
}

type usbfsStream struct {
	d    *usbfsDevice
	urbs []usbURB
	bufs [][]byte
}

// Next reaps one completed URB (non-blocking ioctl polled to the deadline),
// returns its data, and resubmits it to keep the window full.
func (s *usbfsStream) Next(timeout time.Duration) ([]byte, error) {
	deadline := time.Now().Add(timeout)
	var p uintptr // kernel writes the completed urb's address here
	for {
		_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, s.d.f.Fd(), usbdevfsReapURBNDelay, uintptr(unsafe.Pointer(&p)))
		if errno == 0 {
			break
		}
		if errno != syscall.EAGAIN {
			return nil, errno
		}
		if time.Now().After(deadline) {
			return nil, fmt.Errorf("asicam: bulk stream timeout")
		}
		time.Sleep(50 * time.Microsecond)
	}
	// Match the reaped URB by address (compare uintptrs — never convert the
	// kernel-returned uintptr back to a pointer).
	for i := range s.urbs {
		if uintptr(unsafe.Pointer(&s.urbs[i])) != p {
			continue
		}
		u := &s.urbs[i]
		if u.status != 0 {
			return nil, fmt.Errorf("asicam: URB status %d", u.status)
		}
		out := make([]byte, u.actualLength)
		copy(out, s.bufs[i][:u.actualLength])
		u.actualLength = 0
		_ = s.d.ioctl(usbdevfsSubmitURB, unsafe.Pointer(u)) // resubmit to refill the window
		return out, nil
	}
	return nil, fmt.Errorf("asicam: reaped unknown URB")
}

func (s *usbfsStream) Close() error {
	for i := range s.urbs {
		_ = s.d.ioctl(usbdevfsDiscardURB, unsafe.Pointer(&s.urbs[i]))
	}
	return nil
}
