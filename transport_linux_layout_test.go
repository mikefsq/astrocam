//go:build linux

package astrocam

import (
	"testing"
	"unsafe"
)

// TestUsbfsLayout: the Go structs match <linux/usbdevice_fs.h> at this host's pointer size, so
// the ioctl numbers derived from them are the kernel's (64-bit: 24/24/56, 32-bit: 16/16/44).
func TestUsbfsLayout(t *testing.T) {
	ps := unsafe.Sizeof(uintptr(0))
	want := map[string][2]uintptr{ // {64-bit, 32-bit}
		"ctrl": {24, 16}, "bulk": {24, 16}, "urb": {56, 44},
	}
	idx := 0
	if ps == 4 {
		idx = 1
	}
	if got := unsafe.Sizeof(usbCtrlTransfer{}); got != want["ctrl"][idx] {
		t.Errorf("usbCtrlTransfer size %d, want %d", got, want["ctrl"][idx])
	}
	if got := unsafe.Sizeof(usbBulkTransfer{}); got != want["bulk"][idx] {
		t.Errorf("usbBulkTransfer size %d, want %d", got, want["bulk"][idx])
	}
	if got := unsafe.Sizeof(usbURB{}); got != want["urb"][idx] {
		t.Errorf("usbURB size %d, want %d", got, want["urb"][idx])
	}
	if ps == 8 && (usbdevfsSubmitURB != 0x8038550a || usbdevfsReapURBNDelay != 0x4008550d) {
		t.Errorf("64-bit ioctl numbers %#x / %#x, want 0x8038550a / 0x4008550d", usbdevfsSubmitURB, usbdevfsReapURBNDelay)
	}
	if ps == 4 && (usbdevfsSubmitURB != 0x802c550a || usbdevfsReapURBNDelay != 0x4004550d) {
		t.Errorf("32-bit ioctl numbers %#x / %#x, want 0x802c550a / 0x4004550d", usbdevfsSubmitURB, usbdevfsReapURBNDelay)
	}
	if o := unsafe.Offsetof(usbURB{}.usercontext); (ps == 8 && o != 48) || (ps == 4 && o != 40) {
		t.Errorf("usbURB.usercontext offset %d, want 48 (64-bit) / 40 (32-bit)", o)
	}
}
