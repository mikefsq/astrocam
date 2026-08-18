package astrocam

/*
#cgo LDFLAGS: -framework IOKit -framework CoreFoundation
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdlib.h>
#include <stdint.h>

// asicamHotplugCB is the Go side (hotplug_darwin_export.go); one call per
// device that appeared or went away.
extern void asicamHotplugCB(int attached, uint32_t vid, uint32_t pid, uint32_t loc, uint64_t entry, uintptr_t handle);

typedef struct {
    IONotificationPortRef port;
    io_iterator_t added, removed;
    CFRunLoopRef rl;
    uintptr_t handle;
} asicam_hotplug;

static int asicam_hp_u32(io_service_t svc, CFStringRef key, uint32_t* out) {
    CFTypeRef v = IORegistryEntryCreateCFProperty(svc, key, kCFAllocatorDefault, 0);
    if (!v) return -1;
    int ok = 0;
    if (CFGetTypeID(v) == CFNumberGetTypeID())
        ok = CFNumberGetValue((CFNumberRef)v, kCFNumberSInt32Type, out);
    CFRelease(v);
    return ok ? 0 : -1;
}

// asicam_hp_drain reports every service on it and releases each. Draining is
// also what arms a matching notification: IOKit delivers the next change only
// once the iterator has been walked to its end.
static void asicam_hp_drain(io_iterator_t it, int attached, uintptr_t handle) {
    io_service_t svc;
    while ((svc = IOIteratorNext(it))) {
        uint32_t vid = 0, pid = 0, loc = 0; uint64_t entry = 0;
        asicam_hp_u32(svc, CFSTR("idVendor"), &vid);
        asicam_hp_u32(svc, CFSTR("idProduct"), &pid);
        asicam_hp_u32(svc, CFSTR("locationID"), &loc);
        IORegistryEntryGetRegistryEntryID(svc, &entry);
        asicamHotplugCB(attached, vid, pid, loc, entry, handle);
        IOObjectRelease(svc);
    }
}
static void asicam_hp_added(void* refcon, io_iterator_t it)   { asicam_hp_drain(it, 1, ((asicam_hotplug*)refcon)->handle); }
static void asicam_hp_removed(void* refcon, io_iterator_t it) { asicam_hp_drain(it, 0, ((asicam_hotplug*)refcon)->handle); }

// asicam_hp_start registers first-match and terminated notifications on the
// USB device class and arms them. The initial drain of the first-match
// iterator reports the devices already attached.
static asicam_hotplug* asicam_hp_start(uintptr_t handle) {
    asicam_hotplug* h = (asicam_hotplug*)calloc(1, sizeof(asicam_hotplug));
    h->handle = handle;
    h->port = IONotificationPortCreate(kIOMainPortDefault);
    if (!h->port) { free(h); return NULL; }
    CFMutableDictionaryRef m1 = IOServiceMatching(kIOUSBDeviceClassName);
    CFMutableDictionaryRef m2 = IOServiceMatching(kIOUSBDeviceClassName);
    if (!m1 || !m2 ||
        IOServiceAddMatchingNotification(h->port, kIOFirstMatchNotification, m1, asicam_hp_added, h, &h->added) != KERN_SUCCESS ||
        IOServiceAddMatchingNotification(h->port, kIOTerminatedNotification, m2, asicam_hp_removed, h, &h->removed) != KERN_SUCCESS) {
        if (h->added) IOObjectRelease(h->added);
        IONotificationPortDestroy(h->port);
        free(h);
        return NULL;
    }
    asicam_hp_drain(h->added, 1, handle);
    asicam_hp_drain(h->removed, 0, handle);
    return h;
}

// asicam_hp_run serves the notification port on the calling thread's run loop
// until asicam_hp_stop. rl is published before the loop starts so stop can
// find it; stop spins on it in case it lands first.
static void asicam_hp_run(asicam_hotplug* h) {
    CFRunLoopRef rl = CFRunLoopGetCurrent();
    CFRunLoopAddSource(rl, IONotificationPortGetRunLoopSource(h->port), kCFRunLoopDefaultMode);
    __atomic_store_n(&h->rl, rl, __ATOMIC_RELEASE);
    CFRunLoopRun();
}
static void asicam_hp_stop(asicam_hotplug* h) {
    CFRunLoopRef rl;
    while ((rl = __atomic_load_n(&h->rl, __ATOMIC_ACQUIRE)) == NULL) usleep(1000);
    CFRunLoopStop(rl);
}
static void asicam_hp_free(asicam_hotplug* h) {
    IOObjectRelease(h->added);
    IOObjectRelease(h->removed);
    IONotificationPortDestroy(h->port);
    free(h);
}
*/
import "C"

import (
	"context"
	"errors"
	"runtime"
	"sync"
)

// ErrNoHotplug is returned by Hotplug on a platform with no notification
// source; the caller keeps polling.
var ErrNoHotplug = errors.New("astrocam: no hotplug notification source on this platform")

// hotplugSubs maps a subscription handle to its channel, for the C callback.
var (
	hotplugMu   sync.Mutex
	hotplugSubs = map[uintptr]chan HotplugEvent{}
	hotplugNext uintptr
)

// hotplug registers IOKit matching notifications and serves them on a run
// loop of their own thread until ctx ends.
func hotplug(ctx context.Context) (<-chan HotplugEvent, error) {
	ch := make(chan HotplugEvent, 32)
	hotplugMu.Lock()
	hotplugNext++
	handle := hotplugNext
	hotplugSubs[handle] = ch
	hotplugMu.Unlock()

	h := C.asicam_hp_start(C.uintptr_t(handle))
	if h == nil {
		hotplugMu.Lock()
		delete(hotplugSubs, handle)
		hotplugMu.Unlock()
		return nil, errors.New("astrocam: IOKit hotplug notification could not be registered")
	}
	done := make(chan struct{})
	go func() {
		// The run loop belongs to one OS thread for its whole life.
		runtime.LockOSThread()
		defer runtime.UnlockOSThread()
		C.asicam_hp_run(h)
		close(done)
	}()
	go func() {
		<-ctx.Done()
		C.asicam_hp_stop(h)
		<-done
		C.asicam_hp_free(h)
		hotplugMu.Lock()
		delete(hotplugSubs, handle)
		hotplugMu.Unlock()
		close(ch)
	}()
	return ch, nil
}

// deliverHotplug is the callback's Go side: it filters to known vendors (a
// terminated service whose idVendor is no longer readable passes, since a
// spurious re-check costs less than a missed detach) and hands the event to
// the subscription, dropping it when the channel is full, where the caller's
// poll covers it.
func deliverHotplug(attached bool, vid, pid uint16, loc uint32, entry uint64, handle uintptr) {
	if vid != 0 && !knownVID(vid) {
		return
	}
	hotplugMu.Lock()
	ch := hotplugSubs[handle]
	hotplugMu.Unlock()
	if ch == nil {
		return
	}
	select {
	case ch <- HotplugEvent{Attached: attached, VID: vid, PID: pid, Location: loc, Attachment: entry}:
	default:
	}
}
