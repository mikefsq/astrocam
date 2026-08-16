//go:build darwin

// macOS USB transport over IOKit's IOUSBLib (the IOUSBHost stack). The one backend that
// needs cgo — there is no pure-Go path to IOKit on macOS.
//
// Scope: control transfers + three bulk read paths — BulkRead (asi_read_frame_async,
// post-N-and-wait, for small USB2 frames), the FrameStreamer windowed pump
// (asi_read_frame_stream, ReadPipeAsync on a dedicated CFRunLoop) that streams large USB3
// frames, keeping a window of transfers cycling and copying contiguously so a short packet
// at a burst boundary can't leave a gap, and the PrequeuedFrameStreamer batch
// (asi_read_frame_prequeued) that arms the whole frame before it arrives — the SDK's
// async-transfer model, what a USB2 link needs to not shear free-run frames. Stall detection
// is time-based on real data (not ZLP count) so the FX3's held final partial-buffer tail
// still completes.
//
// Teardown safety: every batch/session is heap state; if an aborted transfer cannot be
// drained the state (and any caller buffer the kernel may still DMA into) is leaked, the
// transport is poisoned (errTransportBroken), and Close leaks the handle rather than
// releasing an interface with I/O still in flight.

package astrocam

/*
#cgo LDFLAGS: -framework IOKit -framework CoreFoundation
#include <IOKit/IOKitLib.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/usb/IOUSBLib.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <dispatch/dispatch.h>

// Forward-declared: asi_now_ms is defined further down but called by the read pumps above it.
// Modern clang treats a call to an undeclared function as an error (C99+), so declare it up front.
static int64_t asi_now_ms(void);

// asicam_diag carries where-did-it-fail info out of asi_open so the Go side can
// turn the otherwise-opaque failure into a precise message.
typedef struct {
    int matched;       // # of VID/PID matches the iterator returned
    int openKR;        // IOReturn of USBDeviceOpen on a matched device (0 if never failed)
    int ifaceKR;       // open_interface result on the opened device (0 = ok)
    int numEndpoints;  // endpoints on the claimed interface
    int inPipe;        // pipe ref of the bulk-IN endpoint (0 = not found)
    int inMaxPacket;   // its max packet size (512 ~ USB2 HS, 1024 ~ USB3 SS)
} asicam_diag;

typedef struct {
    IOUSBDeviceInterface**    dev;
    IOUSBInterfaceInterface** intf;
    UInt8 inPipe;
    // read_abort is the ReadAborter latch (level-triggered, set/cleared from Go): the frame-
    // read reap loops poll it each slice and abort the pipe, so a StopExposure breaks a
    // blocked read within ~100 ms instead of waiting out its gates.
    volatile int read_abort;
} asicam_dev;

static void asi_set_read_abort(asicam_dev* d, int v) { __sync_lock_test_and_set(&d->read_abort, v); }
#define ASI_READ_ABORT(d) __sync_fetch_and_add(&(d)->read_abort, 0)

static IOUSBDeviceInterface** device_interface(io_service_t svc) {
    IOCFPlugInInterface** plugin = NULL; SInt32 score;
    if (IOCreatePlugInInterfaceForService(svc, kIOUSBDeviceUserClientTypeID,
            kIOCFPlugInInterfaceID, &plugin, &score) != KERN_SUCCESS || !plugin)
        return NULL;
    IOUSBDeviceInterface** dev = NULL;
    (*plugin)->QueryInterface(plugin, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID), (LPVOID*)&dev);
    (*plugin)->Release(plugin);
    return dev;
}

static int open_interface(IOUSBDeviceInterface** dev, IOUSBInterfaceInterface*** outIntf,
                          UInt8* outPipe, int* outMax, int* outNumEp) {
    IOUSBFindInterfaceRequest req;
    req.bInterfaceClass    = kIOUSBFindInterfaceDontCare;
    req.bInterfaceSubClass = kIOUSBFindInterfaceDontCare;
    req.bInterfaceProtocol = kIOUSBFindInterfaceDontCare;
    req.bAlternateSetting  = kIOUSBFindInterfaceDontCare;
    io_iterator_t it;
    if ((*dev)->CreateInterfaceIterator(dev, &req, &it) != kIOReturnSuccess) return -1;
    io_service_t usbIf = IOIteratorNext(it);
    IOObjectRelease(it);
    if (!usbIf) return -2;

    int rc = -3;
    IOCFPlugInInterface** pl = NULL; SInt32 score;
    if (IOCreatePlugInInterfaceForService(usbIf, kIOUSBInterfaceUserClientTypeID,
            kIOCFPlugInInterfaceID, &pl, &score) == KERN_SUCCESS && pl) {
        IOUSBInterfaceInterface** intf = NULL;
        (*pl)->QueryInterface(pl, CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID), (LPVOID*)&intf);
        (*pl)->Release(pl);
        if (intf) {
            IOReturn kr = (*intf)->USBInterfaceOpen(intf);
            if (kr == kIOReturnSuccess) {
                UInt8 n = 0; (*intf)->GetNumEndpoints(intf, &n);
                if (outNumEp) *outNumEp = n;
                for (UInt8 i = 1; i <= n; i++) {
                    UInt8 dir, num, tt, interval; UInt16 maxp;
                    if ((*intf)->GetPipeProperties(intf, i, &dir, &num, &tt, &maxp, &interval) == kIOReturnSuccess
                            && dir == kUSBIn && tt == kUSBBulk) {
                        *outPipe = i;
                        if (outMax) *outMax = maxp;
                        break;
                    }
                }
                *outIntf = intf;
                rc = 0;
            } else {
                (*intf)->Release(intf);
                rc = -4;
            }
        }
    }
    IOObjectRelease(usbIf);
    return rc;
}

// open_svc opens one already-matched USB device service and claims its bulk
// interface, filling out + diag. Returns 0 on success (keeps dev + intf), else <0.
// Factored out of asi_open so both PID-matched and location-matched opens share it.
static int open_svc(io_service_t svc, asicam_dev* out, asicam_diag* diag) {
    IOUSBDeviceInterface** dev = device_interface(svc);
    if (!dev) return -3;
    IOReturn kr = (*dev)->USBDeviceOpen(dev);
    diag->openKR = kr;
    if (kr != kIOReturnSuccess) {
        (*dev)->Release(dev);
        return -3;
    }
    (*dev)->SetConfiguration(dev, 1);
    IOUSBInterfaceInterface** intf = NULL;
    UInt8 inPipe = 0;
    int ifrc = open_interface(dev, &intf, &inPipe, &diag->inMaxPacket, &diag->numEndpoints);
    diag->ifaceKR = ifrc;
    if (ifrc != 0) {
        (*dev)->USBDeviceClose(dev);
        (*dev)->Release(dev);
        return -3;
    }
    out->dev = dev; out->intf = intf; out->inPipe = inPipe;
    diag->inPipe = inPipe;
    return 0; // success — keep dev + intf
}

// reg_u32 / reg_str read a property off a USB device's IORegistry entry without opening the
// device — the OS cached idProduct, locationID and the USB product-name string at
// enumeration, so listing costs no bus traffic.
static int reg_u32(io_service_t svc, CFStringRef key, uint32_t* out) {
    CFTypeRef v = IORegistryEntryCreateCFProperty(svc, key, kCFAllocatorDefault, 0);
    if (!v) return -1;
    int ok = 0;
    if (CFGetTypeID(v) == CFNumberGetTypeID())
        ok = CFNumberGetValue((CFNumberRef)v, kCFNumberSInt32Type, out);
    CFRelease(v);
    return ok ? 0 : -1;
}
static int reg_str(io_service_t svc, CFStringRef key, char* buf, int n) {
    CFTypeRef v = IORegistryEntryCreateCFProperty(svc, key, kCFAllocatorDefault, 0);
    if (!v) return -1;
    int ok = 0;
    if (CFGetTypeID(v) == CFStringGetTypeID())
        ok = CFStringGetCString((CFStringRef)v, buf, n, kCFStringEncodingUTF8);
    CFRelease(v);
    return ok ? 0 : -1;
}

typedef struct {
    uint16_t pid;
    uint32_t location;
    char name[64];
} asicam_devinfo;

// asi_enumerate lists every VID-matched USB device (no open, no PID filter — the Go side
// filters to known camera PIDs). Fills up to max entries; returns the total found (may
// exceed max, so the caller can detect truncation), or <0 on failure.
static int asi_enumerate(uint16_t vid, asicam_devinfo* out, int max) {
    // Match every USB device and filter by reading idVendor, rather than putting idVendor in
    // the match dictionary: on the IOUSBHost stack a class + idVendor-only dictionary matches
    // nothing (idVendor+idProduct does, the open path), so we iterate all and compare here.
    CFMutableDictionaryRef match = IOServiceMatching(kIOUSBDeviceClassName);
    if (!match) return -1;

    io_iterator_t iter;
    if (IOServiceGetMatchingServices(kIOMainPortDefault, match, &iter) != KERN_SUCCESS) return -2;

    int count = 0;
    io_service_t svc;
    while ((svc = IOIteratorNext(iter))) {
        uint32_t gotVID = 0;
        if (reg_u32(svc, CFSTR("idVendor"), &gotVID) == 0 && gotVID == vid) {
            if (count < max) {
                uint32_t pid = 0, loc = 0;
                reg_u32(svc, CFSTR("idProduct"), &pid);
                reg_u32(svc, CFSTR("locationID"), &loc);
                out[count].pid = (uint16_t)pid;
                out[count].location = loc;
                out[count].name[0] = 0;
                reg_str(svc, CFSTR("USB Product Name"), out[count].name, (int)sizeof(out[count].name));
            }
            count++;
        }
        IOObjectRelease(svc);
    }
    IOObjectRelease(iter);
    return count;
}

static int asi_open(uint16_t vid, uint16_t pid, asicam_dev* out, asicam_diag* diag) {
    CFMutableDictionaryRef match = IOServiceMatching(kIOUSBDeviceClassName);
    if (!match) return -1;
    SInt32 v = vid, p = pid;
    CFNumberRef vn = CFNumberCreate(NULL, kCFNumberSInt32Type, &v);
    CFNumberRef pn = CFNumberCreate(NULL, kCFNumberSInt32Type, &p);
    CFDictionarySetValue(match, CFSTR("idVendor"), vn);
    CFDictionarySetValue(match, CFSTR("idProduct"), pn);
    CFRelease(vn); CFRelease(pn);

    io_iterator_t iter;
    if (IOServiceGetMatchingServices(kIOMainPortDefault, match, &iter) != KERN_SUCCESS) return -2;

    int rc = -3;
    io_service_t svc;
    while (rc != 0 && (svc = IOIteratorNext(iter))) {
        diag->matched++;
        rc = open_svc(svc, out, diag);
        IOObjectRelease(svc);
    }
    IOObjectRelease(iter);
    return rc;
}

// asi_open_location opens the VID-matched device at a specific USB location id (the stable
// per-port key from asi_enumerate) — opens the exact one of several identical cameras.
static int asi_open_location(uint16_t vid, uint32_t loc, asicam_dev* out, asicam_diag* diag) {
    // Match all USB devices and filter by idVendor + locationID in code (see
    // asi_enumerate for why idVendor-only matching is avoided).
    CFMutableDictionaryRef match = IOServiceMatching(kIOUSBDeviceClassName);
    if (!match) return -1;

    io_iterator_t iter;
    if (IOServiceGetMatchingServices(kIOMainPortDefault, match, &iter) != KERN_SUCCESS) return -2;

    int rc = -3;
    io_service_t svc;
    while (rc != 0 && (svc = IOIteratorNext(iter))) {
        uint32_t gotVID = 0, l = 0;
        if (reg_u32(svc, CFSTR("idVendor"), &gotVID) == 0 && gotVID == vid &&
            reg_u32(svc, CFSTR("locationID"), &l) == 0 && l == loc) {
            diag->matched++;
            rc = open_svc(svc, out, diag);
        }
        IOObjectRelease(svc);
    }
    IOObjectRelease(iter);
    return rc;
}

static int asi_control(asicam_dev* d, uint8_t reqType, uint8_t req, uint16_t val,
        uint16_t idx, void* data, uint16_t len, uint32_t* done) {
    // DeviceRequestTO (not DeviceRequest): a bounded timeout so a wedged control transfer
    // can't hang the whole capture, plus a few retries so one transient USB glitch on an
    // init/arm register write doesn't kill the capture.
    IOUSBDevRequestTO r;
    r.bmRequestType = reqType;
    r.bRequest = req;
    r.wValue = val;
    r.wIndex = idx;
    r.wLength = len;
    r.pData = data;
    r.noDataTimeout = 1000;     // ms
    r.completionTimeout = 1000; // ms
    IOReturn kr = kIOReturnSuccess;
    for (int attempt = 0; attempt < 4; attempt++) {
        r.wLenDone = 0;
        kr = (*d->dev)->DeviceRequestTO(d->dev, &r);
        if (kr == kIOReturnSuccess) { if (done) *done = r.wLenDone; return 0; }
        usleep(2000); // 2 ms before retrying a transient failure
    }
    if (done) *done = r.wLenDone;
    return -1;
}

// Async multi-transfer bulk read, driven from a dedicated, always-spinning CFRunLoop
// thread. A pthread runs CFRunLoopRun() continuously so completions are serviced the
// instant they fire (a wait-mode FPGA paces its readout on the host servicing the pipe).
//
// Lifecycle: the thread creates the interface async event source on its runloop and
// signals `ready`; the caller then submits N transfers (completions fire on the thread);
// the completion callback counts down and, when all N are in, signals `finished` and
// stops the runloop. On timeout we AbortPipe (which completes the rest), then reset the
// endpoint so the next capture starts from a quiescent pipe.
typedef struct asi_rd asi_rd;
typedef struct { volatile int done; uint32_t len; IOReturn kr; asi_rd* rd; } aslot;
struct asi_rd {
    aslot* slots;
    int n;
    CFRunLoopRef rl;
    dispatch_semaphore_t comp; // V'd once per completion (success, error, or abort)
};

static void asi_async_cb(void* refcon, IOReturn result, void* arg0) {
    aslot* s = (aslot*)refcon;
    s->len = (uint32_t)(uintptr_t)arg0;
    s->kr = result;
    s->done = 1;
    // One signal per completion; the reader counts them down and only then stops the runloop,
    // so every callback runs before the runloop thread exits and rd is freed.
    dispatch_semaphore_signal(s->rd->comp);
}

typedef struct {
    asicam_dev* d; asi_rd* rd; dispatch_semaphore_t ready;
    CFRunLoopSourceRef src; IOReturn srcErr;
} asi_rd_thr;

static void* asi_rd_run(void* arg) {
    asi_rd_thr* t = (asi_rd_thr*)arg;
    t->rd->rl = CFRunLoopGetCurrent();
    t->srcErr = (*t->d->intf)->CreateInterfaceAsyncEventSource(t->d->intf, &t->src);
    if (t->srcErr == kIOReturnSuccess)
        CFRunLoopAddSource(t->rd->rl, t->src, kCFRunLoopDefaultMode);
    dispatch_semaphore_signal(t->ready); // runloop + source ready; caller may submit now
    if (t->srcErr == kIOReturnSuccess) {
        CFRunLoopRun(); // spins continuously until CFRunLoopStop (all transfers done)
        CFRunLoopRemoveSource(t->rd->rl, t->src, kCFRunLoopDefaultMode);
        CFRelease(t->src);
    }
    return NULL;
}

// rc 0 = ok; -2..-5 = setup failure (nothing in flight); -6 = transfers ABANDONED after an
// abort: the kernel may still DMA into buf and a late completion can still be delivered
// through a future event source on this interface (IOKit queues it on the interface's async
// port), so the batch state is leaked on purpose and the CALLER must pin buf and poison
// the transport.
static int asi_read_frame_async(asicam_dev* d, void* buf, uint32_t bufSize,
                                uint32_t chunk, uint32_t timeoutMs, uint32_t* outLen) {
    *outLen = 0;
    if (!d->inPipe) return -2;
    if (chunk == 0) chunk = 1 << 20;
    int n = (int)((bufSize + chunk - 1) / chunk);

    // Heap-allocate the batch state: a late completion dereferences it via its refcon, so it
    // must outlive this call whenever a transfer could not be drained.
    asi_rd* rd = (asi_rd*)calloc(1, sizeof(asi_rd));
    if (!rd) return -5;
    rd->slots = (aslot*)calloc(n, sizeof(aslot));
    if (!rd->slots) { free(rd); return -5; }
    rd->n = n;
    rd->comp = dispatch_semaphore_create(0);
    for (int i = 0; i < n; i++) rd->slots[i].rd = rd;

    dispatch_semaphore_t ready = dispatch_semaphore_create(0);
    asi_rd_thr ta = { d, rd, ready, NULL, kIOReturnSuccess };
    pthread_t th;
    if (pthread_create(&th, NULL, asi_rd_run, &ta) != 0) {
        free(rd->slots); dispatch_release(rd->comp); free(rd); dispatch_release(ready);
        return -4;
    }
    dispatch_semaphore_wait(ready, DISPATCH_TIME_FOREVER); // runloop + source ready
    dispatch_release(ready);
    if (ta.srcErr != kIOReturnSuccess) {
        pthread_join(th, NULL);
        free(rd->slots); dispatch_release(rd->comp); free(rd);
        return -3;
    }

    // Submit all transfers; their completions fire on the dedicated runloop thread. Track how
    // many are actually in flight: a synchronous submit failure does not schedule a callback,
    // so it must not be counted (else the drain below waits forever for a completion that
    // never comes).
    int inflight = 0;
    for (int i = 0; i < n; i++) {
        uint32_t off = (uint32_t)i * chunk;
        uint32_t len = (off + chunk <= bufSize) ? chunk : (bufSize - off);
        IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, (char*)buf + off, len,
                          timeoutMs, timeoutMs, (IOAsyncCallback1)asi_async_cb, &rd->slots[i]);
        if (kr == kIOReturnSuccess) {
            inflight++;
        } else {
            rd->slots[i].done = 1; rd->slots[i].kr = kr;
        }
    }

    // Drain every outstanding completion before tearing down: count completions down one by
    // one in 100 ms slices so the read_abort latch (StopExposure) is seen promptly; on the
    // overall deadline or an abort request, AbortPipe (which completes the rest as aborted)
    // and keep draining with a 2 s per-completion grace. Only once inflight hits 0 do we stop
    // the runloop and join, so no callback can outlive rd. (Mirrors asi_read_frame_stream.)
    int64_t deadline = asi_now_ms() + (int64_t)timeoutMs + 500;
    int aborted = 0;
    while (inflight > 0) {
        if (dispatch_semaphore_wait(rd->comp, dispatch_time(DISPATCH_TIME_NOW, 100000000LL)) == 0) {
            inflight--;
            if (aborted) deadline = asi_now_ms() + 2000; // fresh grace per drained completion
            continue;
        }
        if (!aborted && (ASI_READ_ABORT(d) || asi_now_ms() > deadline)) {
            (*d->intf)->AbortPipe(d->intf, d->inPipe);
            aborted = 1;
            deadline = asi_now_ms() + 2000;
            continue;
        }
        if (aborted && asi_now_ms() > deadline) break; // genuinely wedged even after the abort
    }
    CFRunLoopStop(rd->rl);
    pthread_join(th, NULL);

    // Bytes transferred, in order, up to and including the frame-terminating short transfer.
    // A bulk-IN returning fewer bytes than requested completes with kIOReturnUnderrun, not
    // kIOReturnSuccess — the FX3 ends a frame with a short packet, so the final sub-chunk
    // legitimately underruns; its bytes are real pixel data and must be counted, then stop.
    // Only a slot that never completed, or failed with a non-underrun error, truncates.
    uint32_t total = 0;
    for (int i = 0; i < n; i++) {
        if (!rd->slots[i].done) break;
        if (rd->slots[i].kr != kIOReturnSuccess && rd->slots[i].kr != kIOReturnUnderrun) break;
        total += rd->slots[i].len;
        uint32_t off = (uint32_t)i * chunk;
        uint32_t req = (off + chunk <= bufSize) ? chunk : (bufSize - off);
        if (rd->slots[i].len < req) break; // short packet = end of frame
    }
    *outLen = total;

    if (inflight > 0) return -6; // abandoned: leak rd/slots/comp, skip the pipe clear

    (*d->intf)->ClearPipeStallBothEnds(d->intf, d->inPipe); // quiescent pipe for next capture
    free(rd->slots);
    dispatch_release(rd->comp);
    free(rd);
    return 0;
}

// asi_read_frame_prequeued reads one frame the way the ASI SDK capture thread does: the whole
// frame's transfers (chunk-sized, last = the remainder) are queued on the pipe BEFORE the
// frame arrives, so the transfer overlaps the sensor readout and the pipe never idles — the
// one-at-a-time windowed read leaves gaps that shear a USB2 HighSpeed frame. Gates: totalMs
// bounds the whole read; idleMs gates only AFTER the first byte (a quiet integration ahead of
// the first completion must not trip it). Completions land in submission order on a single
// bulk pipe, so *outLen is the in-order contiguous prefix. rc as asi_read_frame_async
// (-6 = abandoned; caller pins buf + poisons transport).
static int asi_read_frame_prequeued(asicam_dev* d, void* buf, uint32_t bufSize, uint32_t chunk,
                                    uint32_t idleMs, uint32_t totalMs, uint32_t* outLen) {
    *outLen = 0;
    if (!d->inPipe) return -2;
    if (chunk == 0) chunk = 1 << 20;
    int n = (int)((bufSize + chunk - 1) / chunk);

    asi_rd* rd = (asi_rd*)calloc(1, sizeof(asi_rd));
    if (!rd) return -5;
    rd->slots = (aslot*)calloc(n, sizeof(aslot));
    if (!rd->slots) { free(rd); return -5; }
    rd->n = n;
    rd->comp = dispatch_semaphore_create(0);
    for (int i = 0; i < n; i++) rd->slots[i].rd = rd;

    dispatch_semaphore_t ready = dispatch_semaphore_create(0);
    asi_rd_thr ta = { d, rd, ready, NULL, kIOReturnSuccess };
    pthread_t th;
    if (pthread_create(&th, NULL, asi_rd_run, &ta) != 0) {
        free(rd->slots); dispatch_release(rd->comp); free(rd); dispatch_release(ready);
        return -4;
    }
    dispatch_semaphore_wait(ready, DISPATCH_TIME_FOREVER);
    dispatch_release(ready);
    if (ta.srcErr != kIOReturnSuccess) {
        pthread_join(th, NULL);
        free(rd->slots); dispatch_release(rd->comp); free(rd);
        return -3;
    }

    // Arm the whole frame up front. The driver-side per-transfer timeout is total plus a
    // margin: the reap loop below owns the idle/total gating and aborts explicitly.
    uint32_t xferTo = totalMs + 1000;
    int inflight = 0;
    for (int i = 0; i < n; i++) {
        uint32_t off = (uint32_t)i * chunk;
        uint32_t len = (off + chunk <= bufSize) ? chunk : (bufSize - off);
        IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, (char*)buf + off, len,
                          xferTo, xferTo, (IOAsyncCallback1)asi_async_cb, &rd->slots[i]);
        if (kr == kIOReturnSuccess) {
            inflight++;
        } else {
            rd->slots[i].done = 1; rd->slots[i].kr = kr;
        }
    }

    int64_t start = asi_now_ms();
    int64_t lastData = start;   // last time a completion carried bytes
    int64_t abortAt = 0;
    int gotFirst = 0;
    int aborted = 0;
    int completed = 0;          // completion signals consumed
    int cursor = 0;             // in-order scan over completed slots (data + frame-end)
    while (completed < inflight) {
        if (dispatch_semaphore_wait(rd->comp, dispatch_time(DISPATCH_TIME_NOW, 100000000LL)) == 0) {
            completed++;
        } else if (aborted) {
            if (asi_now_ms() - abortAt > 5000) break; // wedged even after the abort
            continue;
        }
        // Advance over freshly-completed slots: note data arrivals for the idle gate; a
        // short, underrun or errored slot is the end of the contiguous frame — abort the
        // rest (they'd otherwise sit armed until their driver timeout).
        while (cursor < n && rd->slots[cursor].done) {
            if (rd->slots[cursor].len > 0) { gotFirst = 1; lastData = asi_now_ms(); }
            uint32_t off = (uint32_t)cursor * chunk;
            uint32_t req = (off + chunk <= bufSize) ? chunk : (bufSize - off);
            if (!aborted && (rd->slots[cursor].kr != kIOReturnSuccess || rd->slots[cursor].len < req)) {
                (*d->intf)->AbortPipe(d->intf, d->inPipe);
                aborted = 1; abortAt = asi_now_ms();
            }
            cursor++;
        }
        if (!aborted) {
            int64_t nowv = asi_now_ms();
            int stall = gotFirst ? (nowv - lastData > (int64_t)idleMs)
                                 : (nowv - start   > (int64_t)totalMs);
            if (ASI_READ_ABORT(d) || stall) { (*d->intf)->AbortPipe(d->intf, d->inPipe); aborted = 1; abortAt = nowv; }
        }
    }
    CFRunLoopStop(rd->rl);
    pthread_join(th, NULL);

    uint32_t total = 0;
    for (int i = 0; i < n; i++) {
        if (!rd->slots[i].done) break;
        if (rd->slots[i].kr != kIOReturnSuccess && rd->slots[i].kr != kIOReturnUnderrun) break;
        total += rd->slots[i].len;
        uint32_t off = (uint32_t)i * chunk;
        uint32_t req = (off + chunk <= bufSize) ? chunk : (bufSize - off);
        if (rd->slots[i].len < req) break;
    }
    *outLen = total;

    if (completed < inflight) return -6; // abandoned: leak rd/slots/comp, skip the pipe clear

    (*d->intf)->ClearPipeStallBothEnds(d->intf, d->inPipe);
    free(rd->slots);
    dispatch_release(rd->comp);
    free(rd);
    return 0;
}

// ---- Continuous windowed stream read (USB3 large frames) -------------------
//
// asi_read_frame_async posts ceil(size/chunk) transfers once and waits for all to settle;
// on a bursty USB3 stream the trailing transfers complete empty when an intermediate short
// packet lands, returning a truncated frame. This keeps a small window of transfers cycling
// on the pipe, resubmitting each as it completes, until the whole frame is in.
//
// A single bulk-IN pipe completes transfers in submission order, so completions are reaped
// FIFO; each completed chunk is copied to a contiguous watermark in the frame buffer (not a
// fixed chunk*offset), keeping the frame gap-free even when a chunk returns short at a USB
// burst boundary. It stops only when the whole frame is in, on an idle stall (no completion
// within idleMs — the caller then kicks the FPGA and calls again for the remainder), or
// after totalMs. Returns contiguous bytes in *outLen.
#define ASI_SWIN 8
typedef struct asi_sd asi_sd;
typedef struct { volatile int done; uint32_t len; IOReturn kr; asi_sd* sd; int idx; char* scratch; uint64_t seq; } sslot;
struct asi_sd {
    sslot slots[ASI_SWIN];
    int window;
    dispatch_semaphore_t comp;            // V'd once per completion
    int q[ASI_SWIN * 4]; volatile int qh, qt; pthread_mutex_t qlk; // completed-slot FIFO
    CFRunLoopRef rl; dispatch_semaphore_t ready; CFRunLoopSourceRef src; IOReturn srcErr;
    asicam_dev* d;
};
static void asi_stream_cb(void* refcon, IOReturn result, void* arg0) {
    sslot* s = (sslot*)refcon; asi_sd* sd = s->sd;
    s->len = (uint32_t)(uintptr_t)arg0; s->kr = result; s->done = 1;
    pthread_mutex_lock(&sd->qlk);
    sd->q[sd->qt % (ASI_SWIN * 4)] = s->idx; sd->qt++;
    pthread_mutex_unlock(&sd->qlk);
    dispatch_semaphore_signal(sd->comp);
}
static void* asi_sd_run(void* arg) {
    asi_sd* sd = (asi_sd*)arg;
    sd->rl = CFRunLoopGetCurrent();
    sd->srcErr = (*sd->d->intf)->CreateInterfaceAsyncEventSource(sd->d->intf, &sd->src);
    if (sd->srcErr == kIOReturnSuccess)
        CFRunLoopAddSource(sd->rl, sd->src, kCFRunLoopDefaultMode);
    dispatch_semaphore_signal(sd->ready);
    if (sd->srcErr == kIOReturnSuccess) {
        CFRunLoopRun();
        CFRunLoopRemoveSource(sd->rl, sd->src, kCFRunLoopDefaultMode);
        CFRelease(sd->src);
    }
    return NULL;
}
static int64_t asi_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}
// rc 0 = ok; -2..-5 = setup failure; -6 = transfers ABANDONED after the abort: the kernel may
// still DMA into the (leaked) scratch buffers and a late completion can still dereference the
// (leaked) session state, so nothing is freed and the caller must poison the transport.
static int asi_read_frame_stream(asicam_dev* d, void* buf, uint32_t bufSize, uint32_t chunk,
                                 uint32_t idleMs, uint32_t totalMs, uint32_t* outLen) {
    *outLen = 0;
    if (!d->inPipe) return -2;
    if (chunk == 0) chunk = 1 << 20;
    uint32_t nchunks = (bufSize + chunk - 1) / chunk;
    int window = ASI_SWIN;
    if ((uint32_t)window > nchunks) window = (int)nchunks;

    // Heap-allocate the session state (see asi_read_frame_async): a late completion
    // dereferences it via its refcon, so it must outlive this call when a transfer is
    // abandoned.
    asi_sd* sd = (asi_sd*)calloc(1, sizeof(asi_sd));
    if (!sd) return -5;
    sd->window = window; sd->d = d;
    sd->comp = dispatch_semaphore_create(0);
    sd->ready = dispatch_semaphore_create(0);
    pthread_mutex_init(&sd->qlk, NULL);
    for (int i = 0; i < window; i++) {
        sd->slots[i].sd = sd; sd->slots[i].idx = i;
        sd->slots[i].scratch = (char*)malloc(chunk);
        if (!sd->slots[i].scratch) {
            for (int j = 0; j < i; j++) free(sd->slots[j].scratch);
            dispatch_release(sd->comp); dispatch_release(sd->ready);
            pthread_mutex_destroy(&sd->qlk); free(sd); return -5;
        }
    }

    pthread_t th;
    if (pthread_create(&th, NULL, asi_sd_run, sd) != 0) {
        for (int i = 0; i < window; i++) free(sd->slots[i].scratch);
        dispatch_release(sd->comp); dispatch_release(sd->ready);
        pthread_mutex_destroy(&sd->qlk); free(sd); return -4;
    }
    dispatch_semaphore_wait(sd->ready, DISPATCH_TIME_FOREVER);
    if (sd->srcErr != kIOReturnSuccess) {
        pthread_join(th, NULL);
        for (int i = 0; i < window; i++) free(sd->slots[i].scratch);
        dispatch_release(sd->comp); dispatch_release(sd->ready);
        pthread_mutex_destroy(&sd->qlk); free(sd); return -3;
    }

    uint32_t received = 0; // contiguous bytes confirmed into buf
    int inflight = 0;
    // Prime the window: each slot reads a full chunk into its own scratch buffer.
    for (int i = 0; i < window; i++) {
        sd->slots[i].done = 0;
        IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, sd->slots[i].scratch, chunk,
                          totalMs, totalMs, (IOAsyncCallback1)asi_stream_cb, &sd->slots[i]);
        if (kr != kIOReturnSuccess) break;
        inflight++;
    }

    // Stall detection is time-based on real data, not zero-length-packet count: when the FX3
    // holds the frame's final partial DMA buffer it emits a burst of zero-length packets (fast
    // on USB2), so a count guard trips long before the tail commits (via FPGABufReload or the
    // next free-run frame). Give up only after idleMs with no actual bytes; a ZLP blocks ~one
    // packet time on the semaphore, so this doesn't busy-spin.
    int64_t sliceNs = (int64_t)idleMs * 1000000LL;
    if (sliceNs > 100000000LL) sliceNs = 100000000LL; // wake at least every 100 ms to re-check
    int64_t lastReal = asi_now_ms();
    while (received < bufSize && inflight > 0) {
        if (ASI_READ_ABORT(d)) break; // StopExposure broke the read; the teardown below drains
        if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, sliceNs)) != 0) {
            if (asi_now_ms() - lastReal > (int64_t)idleMs) break; // no real data for the idle window
            continue;
        }
        pthread_mutex_lock(&sd->qlk);
        int slot = sd->q[sd->qh % (ASI_SWIN * 4)]; sd->qh++;
        pthread_mutex_unlock(&sd->qlk);
        inflight--;
        sslot* s = &sd->slots[slot];
        if (s->kr != kIOReturnSuccess && s->kr != kIOReturnUnderrun) break; // hard error
        uint32_t take = s->len;
        if (take > bufSize - received) take = bufSize - received;
        if (take) {
            memcpy((char*)buf + received, s->scratch, take); received += take;
            lastReal = asi_now_ms();
        } else if (asi_now_ms() - lastReal > (int64_t)idleMs) {
            break; // only zero-length packets for the whole idle window = a real stall
        }
        if (received < bufSize) {
            // A zero-length completion is the FX3's inter-buffer / frame-boundary marker, not
            // end-of-frame: the tail of the last partial 1-MiB DMA buffer is still to come
            // (FPGABufReload, or the next free-run frame, commits it). Keep cycling.
            s->done = 0;
            IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, s->scratch, chunk,
                              totalMs, totalMs, (IOAsyncCallback1)asi_stream_cb, s);
            if (kr == kIOReturnSuccess) inflight++;
        }
    }

    // Abort outstanding transfers and drain their callbacks before the session state can be
    // freed, so no completion fires against freed memory.
    (*d->intf)->AbortPipe(d->intf, d->inPipe);
    while (inflight > 0) {
        if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, 2000000000LL)) != 0) break;
        pthread_mutex_lock(&sd->qlk); sd->qh++; pthread_mutex_unlock(&sd->qlk);
        inflight--;
    }
    CFRunLoopStop(sd->rl);
    pthread_join(th, NULL);
    *outLen = received;

    if (inflight > 0) return -6; // abandoned: leak sd + scratches, skip the pipe clear

    (*d->intf)->ClearPipeStallBothEnds(d->intf, d->inPipe);
    for (int i = 0; i < window; i++) free(sd->slots[i].scratch);
    dispatch_release(sd->comp); dispatch_release(sd->ready);
    pthread_mutex_destroy(&sd->qlk);
    free(sd);
    return 0;
}

// ---- Persistent stream session (video / planetary burst) -------------------------
// asi_read_frame_stream above sets up the window, spawns the CFRunLoop thread, reads one
// frame, and tears it all down — a fixed per-frame cost (~tens of ms) that dominates when
// frames are small. The session below keeps that machinery resident across a whole burst:
// asi_stream_start primes it once, asi_stream_next pulls exactly one frame (bufSize bytes)
// out of the continuously-cycling window — copying across chunk boundaries and re-arming
// each slot as it drains — and asi_stream_stop aborts and frees. The window never stops, so
// the FX3 stream stays saturated and small frames read at the sensor's true rate.
typedef struct {
    asi_sd   sd;
    pthread_t th;
    uint32_t chunk;
    uint32_t totalMs;
    uint64_t next_seq;   // segment to consume next (in-order watermark)
    uint64_t submit_seq; // segment number for the next submission
    uint32_t seg_off;    // bytes already consumed from the current next_seq segment
    int      started;
    int      held;       // slot handed out by next_zc, awaiting release (-1 = none)
} asicam_stream;

static void asi_stream_free(asicam_stream* s) {
    asi_sd* sd = &s->sd;
    for (int i = 0; i < sd->window; i++) if (sd->slots[i].scratch) free(sd->slots[i].scratch);
    if (sd->comp) dispatch_release(sd->comp);
    if (sd->ready) dispatch_release(sd->ready);
    pthread_mutex_destroy(&sd->qlk);
    free(s);
}

static asicam_stream* asi_stream_start(asicam_dev* d, uint32_t chunk, uint32_t totalMs) {
    if (!d->inPipe) return NULL;
    if (chunk == 0) chunk = 1 << 20;
    asicam_stream* s = (asicam_stream*)calloc(1, sizeof(asicam_stream));
    if (!s) return NULL;
    asi_sd* sd = &s->sd;
    sd->window = ASI_SWIN; sd->d = d;
    sd->comp = dispatch_semaphore_create(0);
    sd->ready = dispatch_semaphore_create(0);
    pthread_mutex_init(&sd->qlk, NULL);
    for (int i = 0; i < sd->window; i++) {
        sd->slots[i].sd = sd; sd->slots[i].idx = i;
        sd->slots[i].scratch = (char*)malloc(chunk);
        if (!sd->slots[i].scratch) { asi_stream_free(s); return NULL; }
    }
    s->chunk = chunk; s->totalMs = totalMs;
    if (pthread_create(&s->th, NULL, asi_sd_run, sd) != 0) { asi_stream_free(s); return NULL; }
    dispatch_semaphore_wait(sd->ready, DISPATCH_TIME_FOREVER);
    if (sd->srcErr != kIOReturnSuccess) { pthread_join(s->th, NULL); asi_stream_free(s); return NULL; }
    // Prime the window: every slot reads a full chunk, tagged with its stream-segment number.
    for (int i = 0; i < sd->window; i++) {
        sd->slots[i].done = 0; sd->slots[i].seq = s->submit_seq;
        IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, sd->slots[i].scratch, chunk,
                          totalMs, totalMs, (IOAsyncCallback1)asi_stream_cb, &sd->slots[i]);
        if (kr != kIOReturnSuccess) break;
        s->submit_seq++;
    }
    s->held = -1;
    s->started = 1;
    return s;
}

// asi_stream_next_zc is the zero-copy variant: instead of memcpy'ing a frame into a caller
// buffer, it returns a pointer to the window scratch buffer that already holds the next
// whole frame, and holds that slot (does not recycle it) until asi_stream_release. Valid
// only when chunk == one frame (sub-MiB ROI), so the frame is contiguous in one scratch.
// *outBuf = scratch; returns the frame length, 0 on idle stall, -1 on a hard error. The
// caller must consume *outBuf before calling release (which re-arms the slot and overwrites it).
static int asi_stream_next_zc(asicam_stream* s, char** outBuf, uint32_t idleMs) {
    asi_sd* sd = &s->sd;
    int64_t lastReal = asi_now_ms();
    const int64_t sliceNs = 50 * 1000000LL;
    for (;;) {
        pthread_mutex_lock(&sd->qlk);
        sslot* cur = NULL;
        for (int i = 0; i < sd->window; i++)
            if (sd->slots[i].done && sd->slots[i].seq == s->next_seq) { cur = &sd->slots[i]; break; }
        pthread_mutex_unlock(&sd->qlk);
        if (cur) {
            if (cur->kr != kIOReturnSuccess && cur->kr != kIOReturnUnderrun) return -1;
            if (cur->len == 0) { // ZLP frame-boundary marker: skip it, recycle, keep going
                s->next_seq++;
                cur->done = 0; cur->seq = s->submit_seq;
                if ((*sd->d->intf)->ReadPipeAsyncTO(sd->d->intf, sd->d->inPipe, cur->scratch, s->chunk,
                        s->totalMs, s->totalMs, (IOAsyncCallback1)asi_stream_cb, cur) == kIOReturnSuccess)
                    s->submit_seq++;
                continue;
            }
            *outBuf = cur->scratch;
            s->held = cur->idx;
            return (int)cur->len;
        }
        while (dispatch_semaphore_wait(sd->comp, DISPATCH_TIME_NOW) == 0) {}
        if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, sliceNs)) != 0) {
            if (asi_now_ms() - lastReal > (int64_t)idleMs) return 0;
        }
    }
}

// asi_stream_release recycles the slot handed out by the last asi_stream_next_zc — re-arms
// it for the next segment and advances the in-order watermark. No-op if nothing is held.
static void asi_stream_release(asicam_stream* s) {
    asi_sd* sd = &s->sd;
    if (s->held < 0) return;
    sslot* cur = &sd->slots[s->held];
    s->held = -1;
    s->next_seq++;
    cur->done = 0; cur->seq = s->submit_seq;
    if ((*sd->d->intf)->ReadPipeAsyncTO(sd->d->intf, sd->d->inPipe, cur->scratch, s->chunk,
            s->totalMs, s->totalMs, (IOAsyncCallback1)asi_stream_cb, cur) == kIOReturnSuccess)
        s->submit_seq++;
}

// asi_stream_next copies exactly one frame (bufSize bytes) out of the resident stream,
// strictly in segment order, re-submitting each slot the moment it is fully drained. A
// chunk may straddle frame boundaries (seg_off remembers the partial remainder for the
// next call), so small frames pack several to a chunk with no extra reads. Returns bytes
// copied (== bufSize on success), 0/short on an idle stall, -1 on a hard pipe error.
static int asi_stream_next(asicam_stream* s, void* buf, uint32_t bufSize, uint32_t idleMs) {
    asi_sd* sd = &s->sd;
    uint32_t copied = 0;
    int64_t lastReal = asi_now_ms();
    const int64_t sliceNs = 50 * 1000000LL; // 50 ms re-check cadence
    while (copied < bufSize) {
        int progressed = 0;
        for (;;) {
            pthread_mutex_lock(&sd->qlk);
            sslot* cur = NULL;
            for (int i = 0; i < sd->window; i++)
                if (sd->slots[i].done && sd->slots[i].seq == s->next_seq) { cur = &sd->slots[i]; break; }
            pthread_mutex_unlock(&sd->qlk);
            if (!cur) break;
            if (cur->kr != kIOReturnSuccess && cur->kr != kIOReturnUnderrun) return -1;
            uint32_t avail = cur->len - s->seg_off;
            uint32_t take = avail; if (take > bufSize - copied) take = bufSize - copied;
            if (take) {
                memcpy((char*)buf + copied, cur->scratch + s->seg_off, take);
                copied += take; s->seg_off += take; progressed = 1; lastReal = asi_now_ms();
            }
            if (s->seg_off >= cur->len) { // segment fully consumed — recycle the slot
                s->seg_off = 0; s->next_seq++;
                cur->done = 0; cur->seq = s->submit_seq;
                if ((*sd->d->intf)->ReadPipeAsyncTO(sd->d->intf, sd->d->inPipe, cur->scratch, s->chunk,
                        s->totalMs, s->totalMs, (IOAsyncCallback1)asi_stream_cb, cur) == kIOReturnSuccess)
                    s->submit_seq++;
            }
            if (copied >= bufSize) break;
        }
        if (copied >= bufSize) break;
        // Need more bytes: drain any stale signals, then block for a fresh completion so the
        // semaphore count can't run away over a long burst (which would busy-spin and defeat
        // the stall guard). The 50 ms slice + outer re-check covers a drain/block race.
        while (dispatch_semaphore_wait(sd->comp, DISPATCH_TIME_NOW) == 0) {}
        if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, sliceNs)) != 0) {
            if (!progressed && asi_now_ms() - lastReal > (int64_t)idleMs) break; // genuine stall
        }
    }
    return (int)copied;
}

// rc 0 = ok; -6 = a transfer could not be drained after the abort: the kernel may still DMA
// into a scratch buffer and a late completion can still dereference the session, so the whole
// session is leaked (not freed) and the caller must poison the transport.
static int asi_stream_stop(asicam_stream* s) {
    if (!s) return 0;
    asi_sd* sd = &s->sd;
    if (s->started) {
        (*sd->d->intf)->AbortPipe(sd->d->intf, sd->d->inPipe);
        for (int g = 0; g < sd->window + 4; g++)
            if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, 500000000LL)) != 0) break;
        // Freeing is safe only once every slot's completion has actually run (the semaphore
        // drain above can exit early on residual counts): an undone slot means the kernel
        // still owns its scratch buffer.
        int undone = 0;
        for (int i = 0; i < sd->window; i++)
            if (!sd->slots[i].done) { undone = 1; break; }
        if (sd->rl) CFRunLoopStop(sd->rl);
        pthread_join(s->th, NULL);
        if (undone) return -6; // abandoned: leak the session, skip the pipe clear
        (*sd->d->intf)->ClearPipeStallBothEnds(sd->d->intf, sd->d->inPipe);
    }
    asi_stream_free(s);
    return 0;
}

static int asi_reset_pipe(asicam_dev* d) {
    if (!d->inPipe) return -2;
    return (int)(*d->intf)->ClearPipeStallBothEnds(d->intf, d->inPipe);
}

// asi_reset_device issues a USB bus reset (last-resort recovery). It re-runs the FX3
// firmware to a known state; the device keeps its address (no re-enumerate), but all camera
// state is lost, so a re-Init is required afterwards.
static int asi_reset_device(asicam_dev* d) {
    if (!d->dev) return -2;
    return (int)(*d->dev)->ResetDevice(d->dev);
}

static void asi_close(asicam_dev* d) {
    if (d->intf) { (*d->intf)->USBInterfaceClose(d->intf); (*d->intf)->Release(d->intf); }
    if (d->dev)  { (*d->dev)->USBDeviceClose(d->dev);     (*d->dev)->Release(d->dev); }
}
*/
import "C"

import (
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
	"time"
	"unsafe"
)

// errClosed is returned by a transfer attempted after Close has freed the handle,
// instead of dereferencing the dangling t.d (a SIGSEGV inside the C library).
var errClosed = errors.New("astrocam: transport closed")

// errTransportBroken marks a device whose aborted transfers never completed (C rc -6): the
// kernel may still DMA into outstanding buffers, so all further I/O (including the final
// interface release) is refused. Only unplugging the device / process exit recovers.
var errTransportBroken = errors.New("astrocam: transport broken (abandoned in-flight transfers)")

// darwinLeakedIO pins Go buffers the kernel may still DMA into (a read returned rc -6):
// they must never be reused or collected. Package-level and never cleared, on purpose.
var (
	darwinLeakedIOMu sync.Mutex
	darwinLeakedIO   [][]byte
)

// IOReturn codes we special-case for diagnostics.
const (
	kIOReturnExclusiveAccess = 0xe00002c5
	kIOReturnNoDevice        = 0xe00002c0
)

// darwinDevice is an IOUSBLib-backed Transport for one open camera.
type darwinDevice struct {
	d    *C.asicam_dev
	diag C.asicam_diag

	// ioMu serializes EP0 control transfers (the IOUSBLib handle isn't safe for concurrent
	// DeviceRequestTO) and a whole-frame BulkRead, so a control transfer can't interleave with
	// the readout and wedge the un-buffered USB2 path. ReadFrameStream (the USB3/DDR path)
	// stays unlocked; it needs the worker's concurrent FPGABufReload. See transport_linux.go
	// (usbfsDevice.ioMu) for the full rationale.
	ioMu sync.Mutex

	// closeMu guards the handle's lifetime against the transfers that use it. Every operation
	// that touches t.d takes it as a reader (so a readout plus a TEC tick run concurrently);
	// Close takes it as the writer, so it cannot free t.d while a transfer is mid-flight, and a
	// transfer started after Close sees closed and returns errClosed instead of dereferencing
	// freed memory.
	closeMu sync.RWMutex
	closed  bool

	// broken latches when an aborted read could not be drained (C rc -6): the kernel still
	// owns outstanding buffers, so every subsequent transfer fails fast and Close leaks the
	// handle instead of releasing an interface with I/O in flight.
	broken atomic.Bool

	// readAborted mirrors the C-side read_abort latch (ReadAborter) for the Go-side entry
	// fail-fast: while set, new frame reads return (0, nil) without arming anything; the
	// C reap loops poll the C flag to break a read already blocked in cgo.
	readAborted atomic.Bool

	// streamMu guards the open resident-session registry (locked after closeMu). Close stops
	// leftover sessions before releasing the interface their pump threads reference.
	streamMu sync.Mutex
	streams  map[*darwinStream]struct{}
}

// forfeit pins buf forever and poisons the transport: an abandoned transfer means the kernel
// can still DMA into it at any time.
func (t *darwinDevice) forfeit(buf []byte) {
	t.broken.Store(true)
	darwinLeakedIOMu.Lock()
	darwinLeakedIO = append(darwinLeakedIO, buf)
	darwinLeakedIOMu.Unlock()
}

// openIOUSBHost finds the first device matching vid/pid via IOKit, opens it, and
// claims its bulk interface (OpenHost is the public entry). On failure it reports where it
// stopped.
func openIOUSBHost(vid, pid uint16) (*darwinDevice, error) {
	dev := (*C.asicam_dev)(C.calloc(1, C.size_t(unsafe.Sizeof(C.asicam_dev{}))))
	t := &darwinDevice{d: dev}
	if rc := C.asi_open(C.uint16_t(vid), C.uint16_t(pid), dev, &t.diag); rc != 0 {
		C.free(unsafe.Pointer(dev))
		return nil, openError(vid, pid, &t.diag)
	}
	return t, nil
}

// enumerateRaw lists every USB device on vid the OS sees, without opening any (the shared
// Enumerate filters these to known camera PIDs). It reads idProduct, locationID and the
// cached USB product-name string straight from the IORegistry.
func enumerateRaw(vid uint16) ([]DeviceInfo, error) {
	const max = 32
	arr := make([]C.asicam_devinfo, max)
	n := int(C.asi_enumerate(C.uint16_t(vid), &arr[0], C.int(max)))
	if n < 0 {
		return nil, fmt.Errorf("astrocam: USB enumerate failed (rc %d)", n)
	}
	if n > max {
		n = max // C reported more than fit; we only have the first max
	}
	out := make([]DeviceInfo, 0, n)
	for i := 0; i < n; i++ {
		out = append(out, DeviceInfo{
			VID:      vid,
			PID:      uint16(arr[i].pid),
			Location: uint32(arr[i].location),
			Name:     C.GoString(&arr[i].name[0]),
		})
	}
	return out, nil
}

// OpenLocation opens the camera at a specific USB location id (from a DeviceInfo) and
// claims its bulk interface: it binds the exact unit chosen from Enumerate when several
// identical cameras are attached.
func OpenLocation(vid uint16, loc uint32) (Transport, error) {
	dev := (*C.asicam_dev)(C.calloc(1, C.size_t(unsafe.Sizeof(C.asicam_dev{}))))
	t := &darwinDevice{d: dev}
	if rc := C.asi_open_location(C.uint16_t(vid), C.uint32_t(loc), dev, &t.diag); rc != 0 {
		C.free(unsafe.Pointer(dev))
		if t.diag.matched == 0 {
			return nil, fmt.Errorf("astrocam: no device at USB location 0x%08x (unplugged or moved)", loc)
		}
		if kr := uint32(t.diag.openKR); kr != 0 {
			return nil, fmt.Errorf("astrocam: device at location 0x%08x found but USBDeviceOpen failed (IOReturn 0x%08x; busy/exclusively open?)", loc, kr)
		}
		return nil, fmt.Errorf("astrocam: device at location 0x%08x found but interface/bulk setup failed (rc %d, %d endpoints, inPipe %d)",
			loc, int(t.diag.ifaceKR), int(t.diag.numEndpoints), int(t.diag.inPipe))
	}
	return t, nil
}

func openError(vid, pid uint16, d *C.asicam_diag) error {
	switch {
	case d.matched == 0:
		return fmt.Errorf("astrocam: no device %04x:%04x found: not plugged in, wrong PID, or claimed by another driver before it enumerated", vid, pid)
	case d.openKR != 0:
		kr := uint32(d.openKR)
		hint := ""
		switch kr {
		case kIOReturnExclusiveAccess:
			hint = ": busy/exclusively open (another app, or a prior run that didn't Close cleanly); unplug and replug to clear"
		case kIOReturnNoDevice:
			hint = ": device went away mid-open"
		}
		return fmt.Errorf("astrocam: %04x:%04x found but USBDeviceOpen failed (IOReturn 0x%08x)%s", vid, pid, kr, hint)
	default:
		return fmt.Errorf("astrocam: %04x:%04x opened but interface/bulk setup failed (rc %d, %d endpoints, inPipe %d)",
			vid, pid, int(d.ifaceKR), int(d.numEndpoints), int(d.inPipe))
	}
}

// Describe returns the negotiated transport facts (for bring-up diagnostics).
func (t *darwinDevice) Describe() string {
	link := "?"
	switch mp := int(t.diag.inMaxPacket); {
	case mp == 512:
		link = "USB2 HighSpeed"
	case mp >= 1024: // 1024, or the burst-multiplied value IOUSBHost reports on SuperSpeed
		link = "USB3 SuperSpeed"
	}
	return fmt.Sprintf("IOUSBHost: %d endpoints, bulk-IN pipe %d, maxPacket %d (%s)",
		int(t.diag.numEndpoints), int(t.diag.inPipe), int(t.diag.inMaxPacket), link)
}

// SuperSpeed reports whether the bulk-IN endpoint negotiated USB3 SuperSpeed (1024-byte
// max packet) rather than USB2 HighSpeed (512).
func (t *darwinDevice) SuperSpeed() bool { return int(t.diag.inMaxPacket) >= 1024 }

func (t *darwinDevice) control(reqType, bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if t.closed {
		return 0, errClosed
	}
	if t.broken.Load() {
		return 0, errTransportBroken
	}
	t.ioMu.Lock()
	defer t.ioMu.Unlock()
	var done C.uint32_t
	var ptr unsafe.Pointer
	if len(data) > 0 {
		ptr = unsafe.Pointer(&data[0])
	}
	rc := C.asi_control(t.d, C.uint8_t(reqType), C.uint8_t(bRequest), C.uint16_t(wValue), C.uint16_t(wIndex),
		ptr, C.uint16_t(len(data)), &done)
	if rc != 0 {
		return 0, fmt.Errorf("astrocam: control transfer (req 0x%02x) failed", bRequest)
	}
	return int(done), nil
}

func (t *darwinDevice) ControlOut(bRequest uint8, wValue, wIndex uint16, data []byte) error {
	_, err := t.control(0x40, bRequest, wValue, wIndex, data)
	return err
}

func (t *darwinDevice) ControlIn(bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	return t.control(0xC0, bRequest, wValue, wIndex, data)
}

// AbortRead / ArmRead implement ReadAborter (see transport.go): the Go latch handles the
// entry fail-fast, and the C-side read_abort flag breaks a read already blocked inside a
// cgo call; its reap loops poll the flag every ≤100 ms and abort the pipe.
func (t *darwinDevice) AbortRead() {
	t.readAborted.Store(true)
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if !t.closed {
		C.asi_set_read_abort(t.d, 1)
	}
}

func (t *darwinDevice) ArmRead() {
	t.readAborted.Store(false)
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if !t.closed {
		C.asi_set_read_abort(t.d, 0)
	}
}

func (t *darwinDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if t.closed {
		return 0, errClosed
	}
	if t.broken.Load() {
		return 0, errTransportBroken
	}
	if t.readAborted.Load() {
		return 0, nil // read-abort latched (StopExposure): fail fast, don't take ioMu
	}
	t.ioMu.Lock() // hold for the whole frame so no control transfer interleaves (USB2 wedge gate)
	defer t.ioMu.Unlock()
	ms := timeout.Milliseconds()
	if ms <= 0 {
		ms = 2000
	}
	// asi_read_frame_async submits ceil(len(buf)/chunk) transfers across the buffer, drains
	// them in order, and tears the pipe down cleanly. 1 MiB matches the SDK's xferLen.
	const chunk = 1 << 20 // 1 MiB per transfer
	var n C.uint32_t
	rc := C.asi_read_frame_async(t.d, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)),
		C.uint32_t(chunk), C.uint32_t(ms), &n)
	if rc == -6 {
		t.forfeit(buf) // the kernel still owns buf; pin it and poison the transport
		return int(n), errTransportBroken
	}
	if rc != 0 {
		return 0, fmt.Errorf("astrocam: async bulk read setup failed (rc %d)", int(rc))
	}
	if n == 0 {
		if t.readAborted.Load() {
			return 0, nil // aborted mid-read: a clean short prefix, not a timeout failure
		}
		return 0, fmt.Errorf("astrocam: async bulk read got no data (timeout)")
	}
	return int(n), nil
}

// ReadFrameStream reads one whole frame with the continuous windowed pump
// (asi_read_frame_stream): a small window of transfers kept cycling on EP 0x81 until
// len(buf) bytes are in, copied contiguously so a short packet at a USB burst boundary
// can't leave a gap. idle bounds a per-completion stall (the caller recovers and reads the
// remainder); total bounds the whole read. Returns the contiguous bytes received, which
// may be < len(buf) on a stall, leaving the caller to continue into buf[n:]. Satisfies
// FrameStreamer.
func (t *darwinDevice) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if t.closed {
		return 0, errClosed
	}
	if t.broken.Load() {
		return 0, errTransportBroken
	}
	if t.readAborted.Load() {
		return 0, nil // read-abort latched (StopExposure): fail fast
	}
	idleMs := idle.Milliseconds()
	if idleMs <= 0 {
		idleMs = 800
	}
	totalMs := total.Milliseconds()
	if totalMs <= 0 {
		totalMs = 5000
	}
	const chunk = 1 << 20 // 1 MiB per transfer (SDK xferLen)
	var n C.uint32_t
	rc := C.asi_read_frame_stream(t.d, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)),
		C.uint32_t(chunk), C.uint32_t(idleMs), C.uint32_t(totalMs), &n)
	if rc == -6 {
		// The abandoned transfers land in C-side scratch (leaked there), not buf: no pin
		// needed, but the pipe can no longer be trusted.
		t.broken.Store(true)
		return int(n), errTransportBroken
	}
	if rc != 0 {
		return 0, fmt.Errorf("astrocam: windowed stream read setup failed (rc %d)", int(rc))
	}
	return int(n), nil
}

// ReadFrameStreamPrequeued reads one frame the way the ASI SDK's capture thread does
// (PrequeuedFrameStreamer): the whole frame's transfers are queued on the pipe before the
// frame arrives so the read overlaps the sensor readout, which a USB2 HighSpeed link needs
// to not shear free-run frames. `total` bounds the whole read; `idle` gates only after the
// first byte. Holds ioMu for the whole frame (the USB2 control-interleave wedge gate), like
// BulkRead and unlike ReadFrameStream (the USB3/DDR path that needs concurrent FPGABufReload).
func (t *darwinDevice) ReadFrameStreamPrequeued(buf []byte, idle, total time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if t.closed {
		return 0, errClosed
	}
	if t.broken.Load() {
		return 0, errTransportBroken
	}
	if t.readAborted.Load() {
		return 0, nil // read-abort latched (StopExposure): fail fast, don't take ioMu
	}
	t.ioMu.Lock()
	defer t.ioMu.Unlock()
	idleMs := idle.Milliseconds()
	if idleMs <= 0 {
		idleMs = 800
	}
	totalMs := total.Milliseconds()
	if totalMs <= 0 {
		totalMs = 5000
	}
	const chunk = 1 << 20 // 1 MiB per transfer (SDK xferLen)
	var n C.uint32_t
	rc := C.asi_read_frame_prequeued(t.d, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)),
		C.uint32_t(chunk), C.uint32_t(idleMs), C.uint32_t(totalMs), &n)
	if rc == -6 {
		t.forfeit(buf) // the kernel still owns buf; pin it and poison the transport
		return int(n), errTransportBroken
	}
	if rc != 0 {
		return 0, fmt.Errorf("astrocam: prequeued frame read setup failed (rc %d)", int(rc))
	}
	return int(n), nil
}

// darwinStream is a resident windowed-stream session (the video/burst path): the C
// pump stays primed across frames so the per-frame setup cost is paid once. Its methods
// guard against the transport closing under them (closeMu readers), but the session itself
// is single-consumer: Next/NextZC/Release/Close must not race each other.
type darwinStream struct {
	t *darwinDevice
	s *C.asicam_stream
}

// StartStream opens a persistent windowed stream and primes it. frameBytes is informational
// (each Next call passes the actual buffer); total is the per-transfer timeout. The session
// is registered on the device so a transport Close stops it before releasing the interface
// its pump thread and in-flight transfers reference.
func (t *darwinDevice) StartStream(frameBytes int, total time.Duration) (FrameStream, error) {
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if t.closed {
		return nil, errClosed
	}
	if t.broken.Load() {
		return nil, errTransportBroken
	}
	totalMs := total.Milliseconds()
	if totalMs <= 0 {
		totalMs = 5000
	}
	// Chunk = one frame for sub-MiB frames (each transfer lands a whole frame, no cross-chunk
	// straddle, lowest latency); 1 MiB for large frames the window pipelines.
	chunk := 1 << 20
	if frameBytes > 0 && frameBytes < chunk {
		chunk = frameBytes
	}
	s := C.asi_stream_start(t.d, C.uint32_t(chunk), C.uint32_t(totalMs))
	if s == nil {
		return nil, fmt.Errorf("astrocam: stream session start failed")
	}
	st := &darwinStream{t: t, s: s}
	t.streamMu.Lock()
	if t.streams == nil {
		t.streams = map[*darwinStream]struct{}{}
	}
	t.streams[st] = struct{}{}
	t.streamMu.Unlock()
	return st, nil
}

// Next pulls exactly one frame (len(buf) bytes) from the resident stream.
func (st *darwinStream) Next(buf []byte, idle time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	st.t.closeMu.RLock()
	defer st.t.closeMu.RUnlock()
	if st.t.closed || st.s == nil {
		return 0, errClosed
	}
	if st.t.broken.Load() {
		return 0, errTransportBroken
	}
	idleMs := idle.Milliseconds()
	if idleMs <= 0 {
		idleMs = 800
	}
	n := C.asi_stream_next(st.s, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)), C.uint32_t(idleMs))
	if n < 0 {
		return 0, fmt.Errorf("astrocam: stream read hard error")
	}
	return int(n), nil
}

// NextZC returns the next frame as a slice that aliases the session's scratch buffer (no
// copy). Valid only until the next Release call (which re-arms the slot and overwrites the
// memory), so the caller must consume it before releasing. Returns nil on an idle stall.
// Use when the stream's chunk == one frame (sub-MiB ROI).
func (st *darwinStream) NextZC(idle time.Duration) ([]byte, error) {
	st.t.closeMu.RLock()
	defer st.t.closeMu.RUnlock()
	if st.t.closed || st.s == nil {
		return nil, errClosed
	}
	if st.t.broken.Load() {
		return nil, errTransportBroken
	}
	idleMs := idle.Milliseconds()
	if idleMs <= 0 {
		idleMs = 800
	}
	var p *C.char
	n := C.asi_stream_next_zc(st.s, &p, C.uint32_t(idleMs))
	if n < 0 {
		return nil, fmt.Errorf("astrocam: stream read hard error")
	}
	if n == 0 || p == nil {
		return nil, nil // idle stall
	}
	return unsafe.Slice((*byte)(unsafe.Pointer(p)), int(n)), nil
}

// Release recycles the slot handed out by the last NextZC (re-arms it for the next frame).
func (st *darwinStream) Release() {
	st.t.closeMu.RLock()
	defer st.t.closeMu.RUnlock()
	if st.t.closed || st.s == nil || st.t.broken.Load() {
		return
	}
	C.asi_stream_release(st.s)
}

// Close aborts and frees the stream session (idempotent; a transport Close that already
// stopped the session makes this a no-op). If the session's transfers could not be drained
// the whole session is leaked C-side and the transport is poisoned.
func (st *darwinStream) Close() error {
	st.t.closeMu.RLock()
	defer st.t.closeMu.RUnlock()
	st.t.streamMu.Lock()
	defer st.t.streamMu.Unlock()
	if st.s == nil {
		return nil
	}
	rc := C.asi_stream_stop(st.s)
	st.s = nil
	delete(st.t.streams, st)
	if rc != 0 {
		st.t.broken.Store(true)
		return errTransportBroken
	}
	return nil
}

// ResetEndpoint clears a stall/flushes the bulk pipe (ClearPipeStallBothEnds). It holds the
// Close interlock (closeMu reader) like every other transfer, so a worker's teardown reset
// that lands after Close returns errClosed instead of touching the freed handle. It does not
// take ioMu.
func (t *darwinDevice) ResetEndpoint(ep uint8) error {
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if t.closed {
		return errClosed
	}
	if t.broken.Load() {
		return errTransportBroken
	}
	if kr := C.asi_reset_pipe(t.d); kr != 0 {
		return fmt.Errorf("astrocam: clear pipe stall IOReturn 0x%08x", uint32(kr))
	}
	return nil
}

// ResetDevice issues a USB bus reset (DeviceResetter), the last-resort recovery. The
// device loses all state, so a re-Init is required after. It holds the Close interlock
// (closeMu reader) but NOT ioMu: its job includes recovering a read that still holds ioMu.
func (t *darwinDevice) ResetDevice() error {
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if t.closed {
		return errClosed
	}
	if t.broken.Load() {
		return errTransportBroken
	}
	if kr := C.asi_reset_device(t.d); kr != 0 {
		return fmt.Errorf("astrocam: reset device IOReturn 0x%08x", uint32(kr))
	}
	return nil
}

// Close releases the IOKit interfaces and frees the handle. Takes closeMu as the writer, so
// it blocks until every in-flight transfer has returned and no new one can start (they
// observe closed and return errClosed). Any resident stream session still open is stopped
// first; its pump thread and in-flight transfers reference the interface being released.
// On a broken transport (abandoned transfers) the handle is leaked: releasing
// the interface would let IOKit complete into freed state. Idempotent.
func (t *darwinDevice) Close() error {
	t.closeMu.Lock()
	defer t.closeMu.Unlock()
	if t.closed {
		return nil
	}
	t.closed = true
	t.streamMu.Lock()
	for st := range t.streams {
		if st.s != nil {
			if C.asi_stream_stop(st.s) != 0 {
				t.broken.Store(true)
			}
			st.s = nil
		}
		delete(t.streams, st)
	}
	t.streamMu.Unlock()
	if t.broken.Load() {
		return errTransportBroken // leak t.d: the kernel still owns transfers against it
	}
	C.asi_close(t.d)
	C.free(unsafe.Pointer(t.d))
	return nil
}

// OpenHost opens the default USB backend for this platform (macOS: IOUSBHost).
func OpenHost(vid, pid uint16) (Transport, error) {
	d, err := openIOUSBHost(vid, pid)
	if err != nil {
		return nil, err
	}
	return d, nil
}
