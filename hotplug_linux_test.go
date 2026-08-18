package astrocam

import "testing"

// TestParseUevent: a usb_device add on a known vendor becomes an attach event
// with an exact location; a remove a detach; other subsystems, device types,
// and vendors are dropped.
func TestParseUevent(t *testing.T) {
	vid := KnownVIDs()[0]
	add := []byte("add@/devices/pci0000:00/0000:00:14.0/usb1/1-3\x00ACTION=add\x00SUBSYSTEM=usb\x00DEVTYPE=usb_device\x00PRODUCT=" +
		hex4(vid) + "/120e/100\x00BUSNUM=001\x00DEVNUM=007\x00")
	ev, ok := parseUevent(add)
	if !ok || !ev.Attached || ev.VID != vid || ev.PID != 0x120e || ev.Location != 1<<16|7 || ev.Attachment != uint64(1<<16|7) {
		t.Fatalf("add: %+v %v", ev, ok)
	}
	rm := []byte("remove@/devices/x\x00ACTION=remove\x00SUBSYSTEM=usb\x00DEVTYPE=usb_device\x00PRODUCT=" + hex4(vid) + "/120e/100\x00BUSNUM=1\x00DEVNUM=7\x00")
	if ev, ok := parseUevent(rm); !ok || ev.Attached {
		t.Fatalf("remove: %+v %v", ev, ok)
	}
	for _, bad := range []string{
		"add@/x\x00SUBSYSTEM=usb\x00DEVTYPE=usb_interface\x00PRODUCT=" + hex4(vid) + "/1/1\x00",
		"add@/x\x00SUBSYSTEM=tty\x00DEVTYPE=usb_device\x00PRODUCT=" + hex4(vid) + "/1/1\x00",
		"add@/x\x00SUBSYSTEM=usb\x00DEVTYPE=usb_device\x00PRODUCT=dead/1/1\x00",
		"change@/x\x00SUBSYSTEM=usb\x00DEVTYPE=usb_device\x00PRODUCT=" + hex4(vid) + "/1/1\x00",
	} {
		if _, ok := parseUevent([]byte(bad)); ok {
			t.Errorf("accepted %q", bad)
		}
	}
}

func hex4(v uint16) string {
	const digits = "0123456789abcdef"
	s := ""
	for v > 0 {
		s = string(digits[v&0xf]) + s
		v >>= 4
	}
	return s
}
