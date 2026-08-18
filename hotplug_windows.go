//go:build windows

package astrocam

// Hotplug on Windows through cfgmgr32's CM_Register_Notification (Windows 8
// and later): a callback on a system thread for every device-interface arrival
// and removal on GUID_DEVINTERFACE_USB_DEVICE, no window or message loop
// needed. The symbolic link names the vendor and product (VID_xxxx&PID_xxxx);
// the notification carries no location or per-attachment identity, so those
// stay zero and the event is a trigger for the caller's presence check.
// Compile-checked for windows/amd64, not run on hardware.

import (
	"context"
	"errors"
	"strconv"
	"sync"
	"syscall"
	"unsafe"
)

// ErrNoHotplug is returned by Hotplug on a platform with no notification
// source; the caller keeps polling.
var ErrNoHotplug = errors.New("astrocam: no hotplug notification source on this platform")

var (
	modCfgMgr32                = syscall.NewLazyDLL("cfgmgr32.dll")
	procCMRegisterNotification = modCfgMgr32.NewProc("CM_Register_Notification")
	procCMUnregister           = modCfgMgr32.NewProc("CM_Unregister_Notification")
)

const (
	cmNotifyFilterTypeDeviceInterface = 0
	cmNotifyActionArrival             = 0
	cmNotifyActionRemoval             = 1
	maxDeviceIDLen                    = 200
)

// guidDevInterfaceUSBDevice is GUID_DEVINTERFACE_USB_DEVICE
// {A5DCBF10-6530-11D2-901F-00C04FB951ED}.
var guidDevInterfaceUSBDevice = [16]byte{0x10, 0xBF, 0xDC, 0xA5, 0x30, 0x65, 0xD2, 0x11, 0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED}

// cmNotifyFilter mirrors CM_NOTIFY_FILTER: header, then a union whose largest
// member is DeviceInstanceId[MAX_DEVICE_ID_LEN] WCHARs; the class GUID sits at
// its start.
type cmNotifyFilter struct {
	cbSize     uint32
	flags      uint32
	filterType uint32
	reserved   uint32
	union      [maxDeviceIDLen * 2]byte
}

var (
	hotplugWinMu   sync.Mutex
	hotplugWinSubs = map[uintptr]chan HotplugEvent{}
	hotplugWinNext uintptr
	hotplugWinCB   = syscall.NewCallback(hotplugWinCallback)
)

// cmNotifyEventData mirrors CM_NOTIFY_EVENT_DATA for a device-interface
// event: FilterType, Reserved, ClassGuid, then the NUL-terminated WCHAR
// symbolic link.
type cmNotifyEventData struct {
	filterType   uint32
	reserved     uint32
	classGuid    [16]byte
	symbolicLink [1]uint16
}

// hotplugWinCallback is the CM_NOTIFY_CALLBACK: it reads the symbolic link
// out of the event data, parses VID and PID, and hands the event on.
func hotplugWinCallback(hNotify, context uintptr, action uint32, ev *cmNotifyEventData, eventDataSize uint32) uintptr {
	if action != cmNotifyActionArrival && action != cmNotifyActionRemoval {
		return 0
	}
	if ev == nil || eventDataSize < 24 {
		return 0
	}
	link := utf16z(unsafe.Slice(&ev.symbolicLink[0], (int(eventDataSize)-24)/2))
	vid, pid, ok := parseVIDPID(link)
	if !ok || !knownVID(vid) {
		return 0
	}
	hotplugWinMu.Lock()
	ch := hotplugWinSubs[context]
	hotplugWinMu.Unlock()
	if ch == nil {
		return 0
	}
	select {
	case ch <- HotplugEvent{Attached: action == cmNotifyActionArrival, VID: vid, PID: pid}:
	default:
	}
	return 0
}

// utf16z returns the string up to the first NUL in u.
func utf16z(u []uint16) string {
	for i, c := range u {
		if c == 0 {
			return syscall.UTF16ToString(u[:i])
		}
	}
	return syscall.UTF16ToString(u)
}

func hotplug(ctx context.Context) (<-chan HotplugEvent, error) {
	if err := procCMRegisterNotification.Find(); err != nil {
		return nil, ErrNoHotplug
	}
	ch := make(chan HotplugEvent, 32)
	hotplugWinMu.Lock()
	hotplugWinNext++
	handle := hotplugWinNext
	hotplugWinSubs[handle] = ch
	hotplugWinMu.Unlock()

	var filter cmNotifyFilter
	filter.cbSize = uint32(unsafe.Sizeof(filter))
	filter.filterType = cmNotifyFilterTypeDeviceInterface
	copy(filter.union[:16], guidDevInterfaceUSBDevice[:])
	var hNotify uintptr
	cr, _, _ := procCMRegisterNotification.Call(uintptr(unsafe.Pointer(&filter)), handle, hotplugWinCB, uintptr(unsafe.Pointer(&hNotify)))
	if cr != 0 {
		hotplugWinMu.Lock()
		delete(hotplugWinSubs, handle)
		hotplugWinMu.Unlock()
		return nil, errors.New("astrocam: CM_Register_Notification failed (CONFIGRET " + strconv.FormatUint(uint64(cr), 10) + ")")
	}
	go func() {
		<-ctx.Done()
		procCMUnregister.Call(hNotify) // waits for in-flight callbacks
		hotplugWinMu.Lock()
		delete(hotplugWinSubs, handle)
		hotplugWinMu.Unlock()
		close(ch)
	}()
	return ch, nil
}
