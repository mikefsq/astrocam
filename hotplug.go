package astrocam

import "context"

// HotplugEvent reports a USB device on a known camera vendor id appearing on
// or leaving the bus. Attachment is the plugging-in's identity where the
// platform gives one (DeviceInfo.Attachment); on a detach it names the
// attachment that ended, so a holder of an open handle can match it. Location
// and PID are filled where the platform reports them in the notification.
type HotplugEvent struct {
	Attached   bool
	VID, PID   uint16
	Location   uint32
	Attachment uint64
}

// Hotplug subscribes to attach and detach notifications for devices on the
// known vendor ids and returns the channel they arrive on. The subscription
// ends when ctx does, and the channel is closed then. It is the interrupt an
// acquire-monitor loop selects on beside its slow poll: the OS reports the
// change within milliseconds, where a poll sees it on its next pass.
//
// Sources: IOKit matching notifications on macOS (first-match and terminated
// on the USB device class), the kernel uevent netlink socket on Linux
// (SUBSYSTEM=usb, DEVTYPE=usb_device), and CM_Register_Notification on
// Windows. A platform without one returns ErrNoHotplug and the caller keeps
// polling. On macOS the first-match notification also reports every device
// already attached when the subscription starts; a caller that only re-checks
// presence on an event is unaffected.
func Hotplug(ctx context.Context) (<-chan HotplugEvent, error) {
	return hotplug(ctx)
}

// knownVID reports whether vid is a registered camera vendor.
func knownVID(vid uint16) bool {
	for _, v := range KnownVIDs() {
		if v == vid {
			return true
		}
	}
	return false
}
