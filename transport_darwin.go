//go:build darwin

// macOS USB transport over IOKit's IOUSBLib. This backend needs cgo: macOS has no pure-Go path
// to IOKit.
//
// Four bulk-read paths: BulkRead (asi_read_frame_async) for USB2, ReadFrameStreamPrequeued
// (asi_read_frame_prequeued), ReadFrameStream (asi_read_frame_stream) for USB3, and StartStream
// (asicam_stream), the same window kept resident across a video burst.
//
// An aborted transfer that cannot be drained leaks its batch state and any caller buffer the
// kernel may still DMA into, poisons the transport (errTransportBroken), and makes Close leak
// the handle.

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

// asi_now_ms is defined further down and called by the read pumps above it; clang rejects a
// call to an undeclared function (C99+).
static int64_t asi_now_ms(void);

// asicam_diag carries where asi_open failed so the Go side can report a specific message.
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
    // entry is the IORegistry entry id of the opened device service: a fresh id on every
    // enumeration, so a replug at the same locationID is told from continued presence.
    uint64_t entry;
    // abort_gen is the ReadAborter latch: a generation AbortRead bumps, not a level it sets.
    // Each frame read snapshots it at entry and the reap loops abort the pipe once it no longer
    // matches. A clearable flag loses the abort when ArmRead lands inside the loops' 100 ms poll
    // slice: on an ASI6200MC a 5 ms gap between AbortRead and ArmRead left the read running its
    // full 10 s timeout. A bumped generation cannot be un-seen.
    volatile int32_t abort_gen;
    // rd_first is set by the one-shot read's callback on the first completion carrying data, so
    // BulkReadQuiet can end the quiet window when the frame arrives early.
    volatile int rd_first;
} asicam_dev;

// asi_abort_reads bumps the generation (ReadAborter AbortRead). There is no clear: ArmRead only
// drops the Go-side entry latch, which is what fails a later read fast.
static void asi_abort_reads(asicam_dev* d) { __sync_add_and_fetch(&d->abort_gen, 1); }
static int32_t asi_abort_gen(asicam_dev* d) { return __sync_fetch_and_add(&d->abort_gen, 0); }
// ASI_ABORTED is true once an abort has been requested since this read snapshotted gen0.
#define ASI_ABORTED(d, gen0) (asi_abort_gen(d) != (gen0))
static int asi_rd_first(asicam_dev* d) { return __sync_fetch_and_add(&d->rd_first, 0); }
static void asi_rd_first_clear(asicam_dev* d) { __sync_lock_test_and_set(&d->rd_first, 0); }

// The completion callback publishes a slot's done flag after its len/kr, and the reap loops read
// it on the caller's thread with no lock in between. Apple silicon may make plain stores to
// different addresses visible out of order, so done is a release store and every reader an
// acquire load, which carries len/kr with it.
#define ASI_DONE_SET(s)   __atomic_store_n(&(s)->done, 1, __ATOMIC_RELEASE)
#define ASI_DONE_CLEAR(s) __atomic_store_n(&(s)->done, 0, __ATOMIC_RELAXED)
#define ASI_DONE(s)       __atomic_load_n(&(s)->done, __ATOMIC_ACQUIRE)

// Runloop-thread bring-up shared by the read pumps and the resident session. A kCFRunLoopEntry
// observer signals `ready` from inside the running loop, never before CFRunLoopRun: CFRunLoopStop
// against a loop that is not running is a no-op, so a caller that stopped the loop between the
// signal and the run would join a thread that never exits. A source that cannot be created
// signals ready with srcErr set.
static void asi_rl_entry_cb(CFRunLoopObserverRef obs, CFRunLoopActivity act, void* info) {
    dispatch_semaphore_signal((dispatch_semaphore_t)info);
}
static void asi_rl_serve(asicam_dev* d, CFRunLoopRef* rlOut, dispatch_semaphore_t ready, IOReturn* srcErr) {
    CFRunLoopRef rl = CFRunLoopGetCurrent();
    *rlOut = rl;
    CFRunLoopSourceRef src = NULL;
    *srcErr = (*d->intf)->CreateInterfaceAsyncEventSource(d->intf, &src);
    if (*srcErr != kIOReturnSuccess) { dispatch_semaphore_signal(ready); return; }
    CFRunLoopAddSource(rl, src, kCFRunLoopDefaultMode);
    CFRunLoopObserverContext ctx = { 0, (void*)ready, NULL, NULL, NULL };
    CFRunLoopObserverRef obs = CFRunLoopObserverCreate(kCFAllocatorDefault, kCFRunLoopEntry,
                                                       false, 0, asi_rl_entry_cb, &ctx);
    if (obs) CFRunLoopAddObserver(rl, obs, kCFRunLoopDefaultMode);
    else dispatch_semaphore_signal(ready); // no observer: the pre-run signal, the old behavior
    CFRunLoopRun(); // spins until CFRunLoopStop (all transfers reaped)
    if (obs) { CFRunLoopRemoveObserver(rl, obs, kCFRunLoopDefaultMode); CFRelease(obs); }
    CFRunLoopRemoveSource(rl, src, kCFRunLoopDefaultMode);
    CFRelease(src);
}

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

// open_svc opens one already-matched USB device service and claims its bulk interface,
// filling out + diag. Returns 0 on success (keeps dev + intf), else <0. Shared by asi_open
// and asi_open_location.
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
    IORegistryEntryGetRegistryEntryID(svc, &out->entry);
    diag->inPipe = inPipe;
    return 0; // success: keep dev + intf
}

// reg_u32 / reg_str read a property off a USB device's IORegistry entry without opening the
// device: the OS cached idProduct, locationID and the USB product-name string at
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
    uint64_t entry; // IORegistry entry id: new on every plugging-in
    char name[64];
} asicam_devinfo;

// asi_enumerate lists every VID-matched USB device (no open, no PID filter; the Go side
// filters to known camera PIDs). Fills up to max entries; returns the total found (may
// exceed max, so the caller can detect truncation), or <0 on failure.
static int asi_enumerate(uint16_t vid, asicam_devinfo* out, int max) {
    // Match every USB device and filter by reading idVendor: on the IOUSBHost stack a class +
    // idVendor-only match dictionary matches nothing (idVendor+idProduct does, the open path).
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
                out[count].entry = 0;
                IORegistryEntryGetRegistryEntryID(svc, &out[count].entry);
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
// per-port key from asi_enumerate): one chosen unit of several identical cameras.
static int asi_open_location(uint16_t vid, uint32_t loc, asicam_dev* out, asicam_diag* diag) {
    // Match all USB devices and filter by idVendor + locationID in code (see asi_enumerate).
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
        uint16_t idx, void* data, uint16_t len, uint32_t* done, IOReturn* outKr) {
    // DeviceRequestTO (not DeviceRequest): a bounded timeout so a wedged control transfer
    // cannot hang the capture, plus a few retries so one transient USB glitch on an init/arm
    // register write does not kill it.
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
        if (kr == kIOReturnSuccess) { if (done) *done = r.wLenDone; if (outKr) *outKr = kr; return 0; }
        // A device that is gone or not answering will not come back inside three more tries; the
        // retries only buy a wedged control plane four 1 s timeouts before anything is reported,
        // and every attempt re-issues a request the device may already have acted on.
        if (kr == kIOReturnNoDevice || kr == kIOReturnNotResponding) break;
        usleep(2000); // 2 ms before retrying a transient failure
    }
    if (done) *done = r.wLenDone;
    if (outKr) *outKr = kr; // the caller reports which failure this was
    return -1;
}

// Async multi-transfer bulk read, driven from a dedicated CFRunLoop thread. A wait-mode FPGA
// paces its readout on the host servicing the pipe, so the thread runs CFRunLoopRun()
// continuously and completions are serviced as they fire.
typedef struct asi_rd asi_rd;
typedef struct { int done; uint32_t len; IOReturn kr; asi_rd* rd; } aslot; // done: ASI_DONE* only
struct asi_rd {
    aslot* slots;
    int n;
    CFRunLoopRef rl;
    dispatch_semaphore_t comp; // V'd once per completion (success, error, or abort)
    asicam_dev* d;
};

static void asi_async_cb(void* refcon, IOReturn result, void* arg0) {
    aslot* s = (aslot*)refcon;
    s->len = (uint32_t)(uintptr_t)arg0;
    s->kr = result;
    if (s->len > 0 && s->rd->d) __sync_lock_test_and_set(&s->rd->d->rd_first, 1);
    ASI_DONE_SET(s);
    // One signal per completion; the reader counts them down and only then stops the runloop,
    // so every callback runs before the runloop thread exits and rd is freed.
    dispatch_semaphore_signal(s->rd->comp);
}

typedef struct {
    asicam_dev* d; asi_rd* rd; dispatch_semaphore_t ready; IOReturn srcErr;
} asi_rd_thr;

static void* asi_rd_run(void* arg) {
    asi_rd_thr* t = (asi_rd_thr*)arg;
    asi_rl_serve(t->d, &t->rd->rl, t->ready, &t->srcErr);
    return NULL;
}

// A hard pipe error is any completion status other than success, underrun, our own abort, or the
// driver's per-transfer timeout. The reap loops read those four as a short read and report a
// stalled pipe or a vanished device to Go.
#define ASI_HARD_KR(kr) ((kr) != kIOReturnSuccess && (kr) != kIOReturnUnderrun && \
                         (kr) != kIOReturnAborted && (kr) != kIOUSBTransactionTimeout)

// rc 0 = ok; -2..-5 = setup failure (nothing in flight); -6 = transfers abandoned after an
// abort, so the batch state is leaked and the caller must pin buf and poison the transport (the
// kernel may still DMA into buf, and IOKit can still deliver a late completion through a future
// event source on this interface); -7 = a hard pipe error, its IOReturn in *outKr and the good
// prefix in *outLen. A stalled pipe reported as rc 0 with a short count is indistinguishable
// from an idle stall, and the retry ladders would re-arm against a dead device.
static int asi_read_frame_async(asicam_dev* d, void* buf, uint32_t bufSize,
                                uint32_t chunk, uint32_t timeoutMs, uint32_t* outLen, IOReturn* outKr) {
    *outLen = 0; *outKr = kIOReturnSuccess;
    if (!d->inPipe) return -2;
    int32_t gen0 = asi_abort_gen(d); // any abort after this point is ours to honor
    if (chunk == 0) chunk = 1 << 20;
    int n = (int)((bufSize + chunk - 1) / chunk);

    // Heap-allocate the batch state: a late completion dereferences it via its refcon, so it
    // must outlive this call whenever a transfer could not be drained.
    asi_rd* rd = (asi_rd*)calloc(1, sizeof(asi_rd));
    if (!rd) return -5;
    rd->slots = (aslot*)calloc(n, sizeof(aslot));
    if (!rd->slots) { free(rd); return -5; }
    rd->n = n;
    rd->d = d;
    rd->comp = dispatch_semaphore_create(0);
    for (int i = 0; i < n; i++) rd->slots[i].rd = rd;
    __sync_lock_test_and_set(&d->rd_first, 0);

    dispatch_semaphore_t ready = dispatch_semaphore_create(0);
    asi_rd_thr ta = { d, rd, ready, kIOReturnSuccess };
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

    // Submit all transfers; completions fire on the runloop thread. Count only the ones in
    // flight: a synchronous submit failure schedules no callback, and counting it would make
    // the drain below wait forever.
    int inflight = 0;
    for (int i = 0; i < n; i++) {
        uint32_t off = (uint32_t)i * chunk;
        uint32_t len = (off + chunk <= bufSize) ? chunk : (bufSize - off);
        IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, (char*)buf + off, len,
                          timeoutMs, timeoutMs, (IOAsyncCallback1)asi_async_cb, &rd->slots[i]);
        if (kr == kIOReturnSuccess) {
            inflight++;
        } else {
            rd->slots[i].kr = kr; ASI_DONE_SET(&rd->slots[i]);
        }
    }

    // Drain every outstanding completion before tearing down, testing the abort latch and the
    // deadline after every wake, a completion as much as a timeout. A flowing readout completes a
    // 1 MiB transfer every few tens of ms, so a loop that checks the latch only on the timeout
    // slice never acts on it: on an ASI6200MC an AbortRead 500 ms into a 2.5 s read was ignored
    // and all 64 MiB arrived. The runloop stops and joins only once inflight hits 0, so no
    // callback can outlive rd.
    int64_t deadline = asi_now_ms() + (int64_t)timeoutMs + 500;
    int aborted = 0;
    while (inflight > 0) {
        if (dispatch_semaphore_wait(rd->comp, dispatch_time(DISPATCH_TIME_NOW, 100000000LL)) == 0) {
            inflight--;
            if (aborted) { deadline = asi_now_ms() + 2000; continue; } // fresh grace per drained completion
        }
        if (!aborted && (ASI_ABORTED(d, gen0) || asi_now_ms() > deadline)) {
            (*d->intf)->AbortPipe(d->intf, d->inPipe);
            aborted = 1;
            deadline = asi_now_ms() + 2000;
            continue;
        }
        if (aborted && asi_now_ms() > deadline) break; // wedged even after the abort
    }
    CFRunLoopStop(rd->rl);
    pthread_join(th, NULL);

    // Bytes transferred, in order, up to and including the frame-terminating short transfer. A
    // bulk-IN returning fewer bytes than requested completes with kIOReturnUnderrun, and the FX3
    // ends a frame with a short packet, so the final sub-chunk underruns and its bytes are real
    // pixel data.
    uint32_t total = 0;
    IOReturn hardKr = kIOReturnSuccess;
    for (int i = 0; i < n; i++) {
        if (!ASI_DONE(&rd->slots[i])) break;
        if (rd->slots[i].kr != kIOReturnSuccess && rd->slots[i].kr != kIOReturnUnderrun) {
            if (ASI_HARD_KR(rd->slots[i].kr)) hardKr = rd->slots[i].kr;
            break;
        }
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
    if (hardKr != kIOReturnSuccess) { *outKr = hardKr; return -7; } // pipe error, prefix in *outLen
    return 0;
}

// asi_read_frame_prequeued queues the whole frame's transfers (chunk-sized, last = the
// remainder) before the frame arrives, so the transfer overlaps the sensor readout and the pipe
// never idles. A one-at-a-time windowed read leaves gaps that shear a USB2 HighSpeed frame.
// totalMs bounds the whole read; idleMs gates only after the first byte, so a quiet integration
// cannot trip it. Completions land in submission order, so *outLen is the contiguous prefix. rc
// as asi_read_frame_async.
static int asi_read_frame_prequeued(asicam_dev* d, void* buf, uint32_t bufSize, uint32_t chunk,
                                    uint32_t idleMs, uint32_t totalMs, uint32_t* outLen, IOReturn* outKr) {
    *outLen = 0; *outKr = kIOReturnSuccess;
    if (!d->inPipe) return -2;
    int32_t gen0 = asi_abort_gen(d); // any abort after this point is ours to honor
    if (chunk == 0) chunk = 1 << 20;
    int n = (int)((bufSize + chunk - 1) / chunk);

    asi_rd* rd = (asi_rd*)calloc(1, sizeof(asi_rd));
    if (!rd) return -5;
    rd->slots = (aslot*)calloc(n, sizeof(aslot));
    if (!rd->slots) { free(rd); return -5; }
    rd->n = n;
    rd->d = d;
    rd->comp = dispatch_semaphore_create(0);
    for (int i = 0; i < n; i++) rd->slots[i].rd = rd;
    __sync_lock_test_and_set(&d->rd_first, 0);

    dispatch_semaphore_t ready = dispatch_semaphore_create(0);
    asi_rd_thr ta = { d, rd, ready, kIOReturnSuccess };
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
            rd->slots[i].kr = kr; ASI_DONE_SET(&rd->slots[i]);
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
        // Advance over freshly completed slots: note data arrivals for the idle gate; a
        // short, underrun or errored slot ends the contiguous frame, so abort the rest (they
        // would otherwise sit armed until their driver timeout).
        while (cursor < n && ASI_DONE(&rd->slots[cursor])) {
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
            int stall = gotFirst ? (nowv - lastData > (int64_t)idleMs || nowv - start > (int64_t)totalMs)
                                 : (nowv - start   > (int64_t)totalMs);
            if (ASI_ABORTED(d, gen0) || stall) { (*d->intf)->AbortPipe(d->intf, d->inPipe); aborted = 1; abortAt = nowv; }
        }
    }
    CFRunLoopStop(rd->rl);
    pthread_join(th, NULL);

    uint32_t total = 0;
    IOReturn hardKr = kIOReturnSuccess;
    for (int i = 0; i < n; i++) {
        if (!ASI_DONE(&rd->slots[i])) break;
        if (rd->slots[i].kr != kIOReturnSuccess && rd->slots[i].kr != kIOReturnUnderrun) {
            if (ASI_HARD_KR(rd->slots[i].kr)) hardKr = rd->slots[i].kr;
            break;
        }
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
    if (hardKr != kIOReturnSuccess) { *outKr = hardKr; return -7; } // pipe error, prefix in *outLen
    return 0;
}

// ---- Continuous windowed stream read (USB3 large frames) -------------------
//
// A small window of transfers cycles on the pipe, each resubmitted as it completes. Completions
// are reaped FIFO and copied to a contiguous watermark rather than a fixed chunk*offset, which
// keeps the frame gap-free when a chunk returns short at a USB burst boundary. Posting all
// transfers once, as asi_read_frame_async does, truncates a bursty USB3 frame: the trailing
// transfers complete empty when an intermediate short packet lands. The read stops with the whole
// frame in, on an idle stall (the caller kicks the FPGA and calls again for the remainder), or
// after totalMs.
#define ASI_SWIN 8
typedef struct asi_sd asi_sd;
typedef struct { int done; uint32_t len; IOReturn kr; asi_sd* sd; int idx; char* scratch; uint64_t seq; int armed; uint32_t req; } sslot; // done: ASI_DONE* only; req = bytes this submission asked for
struct asi_sd {
    sslot slots[ASI_SWIN];
    int window;
    dispatch_semaphore_t comp;            // V'd once per completion
    // Completed-slot FIFO, read only by asi_read_frame_stream (the resident session scans its
    // slots by sequence number instead and never pops). The indices are unsigned so the
    // session's write-only qt wraps defined rather than overflowing a signed int, which it
    // would reach after 2^31 completions of an uninterrupted stream.
    int q[ASI_SWIN * 4]; volatile uint32_t qh, qt; pthread_mutex_t qlk;
    CFRunLoopRef rl; dispatch_semaphore_t ready; IOReturn srcErr;
    asicam_dev* d;
};
static void asi_stream_cb(void* refcon, IOReturn result, void* arg0) {
    sslot* s = (sslot*)refcon; asi_sd* sd = s->sd;
    s->len = (uint32_t)(uintptr_t)arg0; s->kr = result; ASI_DONE_SET(s);
    pthread_mutex_lock(&sd->qlk);
    sd->q[sd->qt % (ASI_SWIN * 4)] = s->idx; sd->qt++;
    pthread_mutex_unlock(&sd->qlk);
    dispatch_semaphore_signal(sd->comp);
}
static void* asi_sd_run(void* arg) {
    asi_sd* sd = (asi_sd*)arg;
    asi_rl_serve(sd->d, &sd->rl, sd->ready, &sd->srcErr);
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
                                 uint32_t idleMs, uint32_t totalMs, uint32_t* outLen, IOReturn* outKr) {
    *outLen = 0; *outKr = kIOReturnSuccess;
    if (!d->inPipe) return -2;
    int32_t gen0 = asi_abort_gen(d); // any abort after this point is ours to honor
    if (chunk == 0) chunk = 1 << 20;
    uint32_t nchunks = (bufSize + chunk - 1) / chunk;
    int window = ASI_SWIN;
    if ((uint32_t)window > nchunks) window = (int)nchunks;

    // Heap-allocate the session state (see asi_read_frame_async): a late completion
    // dereferences it via its refcon.
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
    IOReturn primeKr = kIOReturnSuccess;
    // Prime the window. Each slot is sized to what the frame still needs at its offset. A
    // full-chunk request for the frame's tail pulls in the head of the next free-run frame, and
    // the copy below discards anything past bufSize, so every later frame comes up short by that
    // much. outstanding tracks the bytes in-flight transfers have spoken for, so a re-arm cannot
    // over-request either.
    uint32_t outstanding = 0;
    for (int i = 0; i < window; i++) {
        uint32_t off = (uint32_t)i * chunk;
        uint32_t want = (off < bufSize && bufSize - off < chunk) ? bufSize - off : chunk;
        ASI_DONE_CLEAR(&sd->slots[i]);
        sd->slots[i].req = want;
        IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, sd->slots[i].scratch, want,
                          totalMs, totalMs, (IOAsyncCallback1)asi_stream_cb, &sd->slots[i]);
        if (kr != kIOReturnSuccess) { primeKr = kr; break; }
        inflight++;
        outstanding += want;
    }
    // An empty window is fatal: the reap loop runs no pass and the read returns rc 0 with 0
    // bytes, which the caller reads as an idle stall and answers by re-arming into the same
    // failure. A partially primed window still works.
    if (inflight == 0) {
        if (sd->rl) CFRunLoopStop(sd->rl);
        pthread_join(th, NULL);
        for (int i = 0; i < window; i++) free(sd->slots[i].scratch);
        dispatch_release(sd->comp); dispatch_release(sd->ready);
        pthread_mutex_destroy(&sd->qlk); free(sd);
        *outKr = primeKr;
        return -7;
    }

    // Stall detection is time-based on real data, not zero-length-packet count. The FX3 emits a
    // burst of ZLPs while it holds the frame's final partial DMA buffer, so a count guard trips
    // long before the tail commits through FPGABufReload or the next free-run frame. A ZLP blocks
    // about one packet time on the semaphore, so waiting out idleMs does not busy-spin.
    int64_t sliceNs = (int64_t)idleMs * 1000000LL;
    if (sliceNs > 100000000LL) sliceNs = 100000000LL; // wake at least every 100 ms to re-check
    int64_t lastReal = asi_now_ms();
    int64_t start = lastReal;
    IOReturn hardKr = kIOReturnSuccess;
    while (received < bufSize && inflight > 0) {
        if (ASI_ABORTED(d, gen0)) break; // abort latched; the teardown below drains
        if (asi_now_ms() - start > (int64_t)totalMs) break; // whole-read bound
        if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, sliceNs)) != 0) {
            if (asi_now_ms() - lastReal > (int64_t)idleMs) break; // no data for the idle window
            continue;
        }
        pthread_mutex_lock(&sd->qlk);
        int slot = sd->q[sd->qh % (ASI_SWIN * 4)]; sd->qh++;
        pthread_mutex_unlock(&sd->qlk);
        inflight--;
        sslot* s = &sd->slots[slot];
        if (outstanding >= s->req) outstanding -= s->req; else outstanding = 0;
        if (s->kr != kIOReturnSuccess && s->kr != kIOReturnUnderrun) { // stall or pipe error
            if (ASI_HARD_KR(s->kr)) hardKr = s->kr;
            break;
        }
        uint32_t take = s->len;
        if (take > bufSize - received) take = bufSize - received;
        if (take) {
            memcpy((char*)buf + received, s->scratch, take); received += take;
            lastReal = asi_now_ms();
        } else if (asi_now_ms() - lastReal > (int64_t)idleMs) {
            break; // only zero-length packets for the whole idle window = a stall
        }
        if (received < bufSize) {
            // A zero-length completion is the FX3's inter-buffer marker, not end-of-frame: the
            // tail of the last partial 1-MiB DMA buffer still has to come, committed by
            // FPGABufReload or the next free-run frame. A ZLP does not advance received, so its
            // slot re-arms at once and the window never runs dry while bytes are owed.
            uint32_t left = bufSize - received;
            uint32_t want = (left > outstanding) ? left - outstanding : 0;
            if (want > chunk) want = chunk;
            if (want > 0) {
                ASI_DONE_CLEAR(s);
                s->req = want;
                IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, s->scratch, want,
                                  totalMs, totalMs, (IOAsyncCallback1)asi_stream_cb, s);
                if (kr == kIOReturnSuccess) { inflight++; outstanding += want; }
            }
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
    if (hardKr != kIOReturnSuccess) { *outKr = hardKr; }

    (*d->intf)->ClearPipeStallBothEnds(d->intf, d->inPipe);
    for (int i = 0; i < window; i++) free(sd->slots[i].scratch);
    dispatch_release(sd->comp); dispatch_release(sd->ready);
    pthread_mutex_destroy(&sd->qlk);
    free(sd);
    if (hardKr != kIOReturnSuccess) return -7; // pipe error, prefix in *outLen
    return 0;
}

// ---- Persistent stream session (video / planetary burst) -------------------------
//
// Building the window is cheap: 0.14 ms per read on an ASI6200MC, scratch allocation included,
// the same at a 512x512 ROI as at the full 122 MB frame. Stopping is what costs. A read that
// tears the window down leaves the pipe idle until the next one primes it, and the FX3 stream
// drains in that gap. The session keeps the window cycling across frames instead, which holds
// the stream saturated and reads small frames at the sensor's rate: 23.4 fps against 0.9 fps for
// arm-per-frame on that camera.
typedef struct {
    asi_sd   sd;
    pthread_t th;
    uint32_t chunk;
    uint32_t totalMs;
    uint32_t xferMs;     // driver-side per-transfer timeout (totalMs + margin)
    uint64_t next_seq;   // segment to consume next (in-order watermark)
    uint64_t submit_seq; // segment number for the next submission
    uint32_t seg_off;    // bytes already consumed from the current next_seq segment
    int      started;
    int      held;       // slot handed out by next_zc, awaiting release (-1 = none)
    int      desynced;   // a Next returned mid-frame; the segment stream no longer aligns
    int      unclaimed;  // completion signals consumed while waiting, not yet matched to a recycled slot
} asicam_stream;

// Semaphore accounting for a resident session. A caller waiting for a completion consumes one
// signal (unclaimed++); recycling a done slot claims its completion (unclaimed--, or one signal
// if none was consumed for it yet). Signals never accumulate over a burst, a fresh completion is
// never discarded, and stop polls the armed slots rather than counting signals.
static void asi_stream_claim(asicam_stream* s) {
    if (s->unclaimed > 0) { s->unclaimed--; return; }
    dispatch_semaphore_wait(s->sd.comp, dispatch_time(DISPATCH_TIME_NOW, 100 * 1000000LL));
}
// asi_stream_rearm resubmits a recycled slot for the next segment; armed marks it as owned by
// the kernel until its callback runs. The driver-side timeout is xferMs, not totalMs: it bounds
// one transfer's own wait for data, and the reap in asi_stream_next owns the caller's idle gate.
static void asi_stream_rearm(asicam_stream* s, sslot* cur) {
    asi_sd* sd = &s->sd;
    ASI_DONE_CLEAR(cur); cur->seq = s->submit_seq; cur->armed = 1;
    IOReturn kr = (*sd->d->intf)->ReadPipeAsyncTO(sd->d->intf, sd->d->inPipe, cur->scratch, s->chunk,
            s->xferMs, s->xferMs, (IOAsyncCallback1)asi_stream_cb, cur);
    if (kr == kIOReturnSuccess)
        s->submit_seq++;
    else
        cur->armed = 0; // dead slot: no completion will come; stop must not wait for it
}

// asi_stream_live reports whether the session can still produce anything. armed stays set from
// submission through completion and clears only when a re-arm fails, so a slot counts until it
// is retired. An empty window can never deliver a completion, so Next would return an idle stall
// on every call against a finished session. Checked only when Next is about to wait.
static int asi_stream_live(asicam_stream* s) {
    asi_sd* sd = &s->sd;
    for (int i = 0; i < sd->window; i++)
        if (sd->slots[i].armed || ASI_DONE(&sd->slots[i])) return 1;
    return 0;
}

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
    // The driver bounds each transfer's own wait for data from when that transfer reaches the
    // head of the pipe, not from when it was queued: on an ASI6200MC with 8 slots armed and a
    // 101 ms frame period, transfers survived a 160 ms timeout and failed at 150 ms. One frame
    // period is what has to fit, and the margin keeps a frame period that nearly fills totalMs
    // off the knife edge.
    s->xferMs = totalMs + 1000;
    if (pthread_create(&s->th, NULL, asi_sd_run, sd) != 0) { asi_stream_free(s); return NULL; }
    dispatch_semaphore_wait(sd->ready, DISPATCH_TIME_FOREVER);
    if (sd->srcErr != kIOReturnSuccess) { pthread_join(s->th, NULL); asi_stream_free(s); return NULL; }
    // Prime the window: every slot reads a full chunk, tagged with its stream-segment number.
    int armed = 0;
    for (int i = 0; i < sd->window; i++) {
        ASI_DONE_CLEAR(&sd->slots[i]); sd->slots[i].seq = s->submit_seq; sd->slots[i].armed = 1;
        IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, sd->slots[i].scratch, chunk,
                          s->xferMs, s->xferMs, (IOAsyncCallback1)asi_stream_cb, &sd->slots[i]);
        if (kr != kIOReturnSuccess) { sd->slots[i].armed = 0; break; }
        s->submit_seq++;
        armed++;
    }
    s->held = -1;
    s->started = 1;
    // A partially primed window still streams, with less pipelining. An empty one never will, so
    // every Next would idle out against a live-but-silent session. Fail the start instead.
    if (armed == 0) {
        s->started = 0; // nothing to abort or wait for
        if (sd->rl) CFRunLoopStop(sd->rl);
        pthread_join(s->th, NULL);
        asi_stream_free(s);
        return NULL;
    }
    return s;
}

// asi_stream_release recycles the slot handed out by the last asi_stream_next_zc: it re-arms the
// slot for the next segment and advances the in-order watermark. No-op if nothing is held.
//
// Both next entry points call it first. A held slot sits at the watermark, so asi_stream_next
// would otherwise drain and re-arm it while `held` still named it, and the caller's later release
// would re-arm an already armed slot (two transfers into one scratch) and step the watermark past
// a sequence number no slot carries. The session then stalls one window later, not at once: on an
// ASI6200MC 6 more frames read fine and every Next after that idled out.
static void asi_stream_release(asicam_stream* s) {
    asi_sd* sd = &s->sd;
    if (s->held < 0) return;
    sslot* cur = &sd->slots[s->held];
    s->held = -1;
    s->next_seq++;
    asi_stream_claim(s);
    asi_stream_rearm(s, cur);
}

// asi_stream_next_zc returns a pointer to the window scratch holding the next whole frame and
// holds that slot until asi_stream_release. It applies only when chunk == one frame (sub-MiB
// ROI), so the frame is contiguous in one scratch. Returns the frame length, 0 on an idle stall,
// -1 on a hard error with its IOReturn in *outKr. The caller must consume *outBuf before the next
// call, which recycles the slot and overwrites it.
static int asi_stream_next_zc(asicam_stream* s, char** outBuf, uint32_t idleMs, IOReturn* outKr) {
    asi_sd* sd = &s->sd;
    *outKr = kIOReturnSuccess;
    asi_stream_release(s); // a slot still held from the last call (see asi_stream_release)
    int64_t lastReal = asi_now_ms();
    const int64_t sliceNs = 50 * 1000000LL;
    for (;;) {
        pthread_mutex_lock(&sd->qlk);
        sslot* cur = NULL;
        for (int i = 0; i < sd->window; i++)
            if (ASI_DONE(&sd->slots[i]) && sd->slots[i].seq == s->next_seq) { cur = &sd->slots[i]; break; }
        pthread_mutex_unlock(&sd->qlk);
        if (cur) {
            if (ASI_HARD_KR(cur->kr)) { *outKr = cur->kr; return -1; }
            if (cur->len == 0) { // ZLP frame-boundary marker: skip it, recycle, keep going
                s->next_seq++;
                asi_stream_claim(s);
                asi_stream_rearm(s, cur);
                continue;
            }
            *outBuf = cur->scratch;
            s->held = cur->idx;
            return (int)cur->len;
        }
        if (!asi_stream_live(s)) return -1; // window died: no completion can ever arrive
        // Wait for one completion (its signal is claimed when that slot is recycled).
        if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, sliceNs)) == 0) {
            s->unclaimed++;
        } else if (asi_now_ms() - lastReal > (int64_t)idleMs) {
            return 0;
        }
    }
}

// asi_stream_next copies one frame (bufSize bytes) out of the resident stream in segment order,
// re-submitting each slot once it is fully drained. A chunk may straddle frame boundaries, and
// seg_off remembers the partial remainder for the next call, so small frames pack several to a
// chunk. Returns bytes copied, 0 or short on an idle stall, -1 on a hard pipe error, -2 once
// desynced.
//
// A short return means the idle window elapsed part way through a frame. Nothing can realign the
// segment watermark afterwards: the FX3 emits ZLPs at inter-buffer boundaries as well as frame
// boundaries, and the session aligns in the first place only by starting just after the sensor
// arms. The session latches `desynced`, and the caller must close it and start a new one.
//
// Only ASI_HARD_KR ends the session. A driver per-transfer timeout or our own abort carries real
// pixel data, since the rest of that frame has already landed in the following slot, so its slot
// drains and recycles like any other segment. Dropping those completions misaligns the segment
// stream for good.
static int asi_stream_next(asicam_stream* s, void* buf, uint32_t bufSize, uint32_t idleMs, IOReturn* outKr) {
    asi_sd* sd = &s->sd;
    *outKr = kIOReturnSuccess;
    if (s->desynced) return -2; // a previous call ended mid-frame; Close and StartStream to realign
    asi_stream_release(s); // a slot still held from a next_zc (see asi_stream_release)
    uint32_t copied = 0;
    int64_t lastReal = asi_now_ms();
    const int64_t sliceNs = 50 * 1000000LL; // idle re-check cadence
    while (copied < bufSize) {
        int progressed = 0;
        for (;;) {
            pthread_mutex_lock(&sd->qlk);
            sslot* cur = NULL;
            for (int i = 0; i < sd->window; i++)
                if (ASI_DONE(&sd->slots[i]) && sd->slots[i].seq == s->next_seq) { cur = &sd->slots[i]; break; }
            pthread_mutex_unlock(&sd->qlk);
            if (!cur) break;
            if (ASI_HARD_KR(cur->kr)) { *outKr = cur->kr; return -1; }
            uint32_t avail = cur->len - s->seg_off;
            uint32_t take = avail; if (take > bufSize - copied) take = bufSize - copied;
            if (take) {
                memcpy((char*)buf + copied, cur->scratch + s->seg_off, take);
                copied += take; s->seg_off += take; progressed = 1; lastReal = asi_now_ms();
            }
            if (s->seg_off >= cur->len) { // segment fully consumed: recycle the slot
                s->seg_off = 0; s->next_seq++;
                asi_stream_claim(s);
                asi_stream_rearm(s, cur);
            }
            if (copied >= bufSize) break;
        }
        if (copied >= bufSize) break;
        if (!asi_stream_live(s)) return -1; // window died: no completion can ever arrive
        // Need more bytes: wait for one completion, then re-scan at once (no drain, so a
        // completion landing between the scan and the wait is never discarded).
        if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, sliceNs)) == 0) {
            s->unclaimed++;
        } else if (!progressed && asi_now_ms() - lastReal > (int64_t)idleMs) {
            break; // stall
        }
    }
    // Short and off a frame boundary: the watermark has passed bytes the caller never got a whole
    // frame from, so latch rather than hand back offset frames.
    if (copied < bufSize && (copied > 0 || s->seg_off > 0)) s->desynced = 1;
    return (int)copied;
}

// rc 0 = ok; -6 = the kernel still owns a transfer after the abort, so the whole session leaks
// and the caller must poison the transport. Completion is judged per armed slot against a
// deadline, not by counting signals.
static int asi_stream_stop(asicam_stream* s) {
    if (!s) return 0;
    asi_sd* sd = &s->sd;
    if (s->started) {
        (*sd->d->intf)->AbortPipe(sd->d->intf, sd->d->inPipe);
        int undone = 1;
        for (int64_t t0 = asi_now_ms(); asi_now_ms() - t0 < 3000; ) {
            undone = 0;
            for (int i = 0; i < sd->window; i++)
                if (sd->slots[i].armed && !ASI_DONE(&sd->slots[i])) { undone = 1; break; }
            if (!undone) break;
            dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, 5 * 1000000LL)); // 5 ms
        }
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

// errClosed is returned by a transfer attempted after Close has freed the handle.
var errClosed = errors.New("astrocam: transport closed")

// errTransportBroken marks a device whose aborted transfers never completed (C rc -6; see
// darwinDevice.broken).
var errTransportBroken = errors.New("astrocam: transport broken (abandoned in-flight transfers)")

// errStreamBusy is returned by StartStream while another session holds the bulk pipe: one byte
// stream cannot feed two windows (see StartStream).
var errStreamBusy = errors.New("astrocam: a stream session is already open on this device")

// ErrStreamDesynced is returned by a session Next after an earlier Next ended part way through a
// frame. The segment stream cannot be realigned in place, so the caller must Close the session
// and StartStream again rather than abandon the capture.
var ErrStreamDesynced = errors.New("astrocam: stream session desynced by a short read; close and restart it")

// darwinLeakedIO pins Go buffers the kernel may still DMA into (a read returned rc -6); they
// must never be reused or collected. Never cleared.
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
	// entry is the IORegistry entry id the handle was opened on, copied out at open so
	// Attachment needs no access to d after Close.
	entry uint64

	// ioMu serializes gated EP0 control transfers against the whole-frame reads BulkRead,
	// BulkReadQuiet (after its quiet window) and ReadFrameStreamPrequeued, so a control transfer
	// cannot interleave with a readout and wedge the un-buffered USB2 path. ReadFrameStream (the
	// USB3 DDR path) skips it, since that path needs the worker's concurrent FPGABufReload, and
	// ControlOutUngated bypasses it. Lock order: closeMu (shared) before ioMu.
	ioMu sync.Mutex

	// closeMu is the Close interlock: every operation touching t.d holds it shared, and Close
	// takes it exclusively, so Close cannot free t.d under a transfer.
	closeMu sync.RWMutex
	closed  bool

	// dMu guards t.d against Close's free for the ReadAborter calls alone. Those cannot use
	// closeMu: a frame read holds it shared for seconds, a concurrent Close parks on the
	// exclusive lock, and Go's RWMutex then queues every new reader behind the waiting writer.
	// AbortRead would block on the read it exists to break (measured: a 2.5 s read, AbortRead
	// blocked 2.1 s, all 64 MiB delivered). A plain mutex has no writer preference, and Close
	// holds it only across the free. Lock order: dMu is a leaf, taken alone.
	dMu    sync.Mutex
	dFreed bool // set under dMu immediately before asi_close/free

	// broken latches when an aborted read could not be drained (C rc -6). The kernel still owns
	// outstanding buffers, so every later transfer fails fast and Close leaks the handle. Only a
	// replug or process exit recovers.
	broken atomic.Bool

	// readAborted is the Go-side entry latch of ReadAborter: while set, new frame reads return
	// (0, nil) without taking ioMu. Breaking a read already blocked in cgo is abort_gen's job.
	readAborted atomic.Bool
	// readActive counts frame reads in flight, including open stream sessions. On a USB2 link the
	// IN control path paces itself while it is non-zero.
	readActive atomic.Int32
	inPace     inPacer

	// streamMu guards the open resident-session registry (locked after closeMu). Close stops
	// leftover sessions before releasing the interface their pump threads reference.
	streamMu sync.Mutex
	streams  map[*darwinStream]struct{}
}

// forfeit pins buf in darwinLeakedIO and poisons the transport: an abandoned transfer means
// the kernel can still DMA into buf.
func (t *darwinDevice) forfeit(buf []byte) {
	t.broken.Store(true)
	darwinLeakedIOMu.Lock()
	darwinLeakedIO = append(darwinLeakedIO, buf)
	darwinLeakedIOMu.Unlock()
}

// openIOUSBHost finds the first device matching vid/pid via IOKit, opens it, and claims its
// bulk interface (OpenHost is the public entry). On failure it reports where it stopped.
func openIOUSBHost(vid, pid uint16) (*darwinDevice, error) {
	dev := (*C.asicam_dev)(C.calloc(1, C.size_t(unsafe.Sizeof(C.asicam_dev{}))))
	t := &darwinDevice{d: dev, inPace: inPacer{min: usb2InPace}}
	if rc := C.asi_open(C.uint16_t(vid), C.uint16_t(pid), dev, &t.diag); rc != 0 {
		C.free(unsafe.Pointer(dev))
		return nil, openError(vid, pid, &t.diag)
	}
	t.entry = uint64(dev.entry)
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
			VID:        vid,
			PID:        uint16(arr[i].pid),
			Location:   uint32(arr[i].location),
			Attachment: uint64(arr[i].entry),
			Name:       C.GoString(&arr[i].name[0]),
		})
	}
	return out, nil
}

// OpenLocation opens the camera at a specific USB location id (from a DeviceInfo) and
// claims its bulk interface: it binds the exact unit chosen from Enumerate when several
// identical cameras are attached.
func OpenLocation(vid uint16, loc uint32) (Transport, error) {
	dev := (*C.asicam_dev)(C.calloc(1, C.size_t(unsafe.Sizeof(C.asicam_dev{}))))
	t := &darwinDevice{d: dev, inPace: inPacer{min: usb2InPace}}
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
	t.entry = uint64(dev.entry)
	return t, nil
}

// Attachment is the IORegistry entry id of the device this handle was opened on.
func (t *darwinDevice) Attachment() uint64 { return t.entry }

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
	if reqType&0x80 != 0 && t.readActive.Load() > 0 && !t.SuperSpeed() {
		t.inPace.wait() // a USB2 readout in flight: pace EP0 reads (usb2InPace)
	}
	var done C.uint32_t
	var ptr unsafe.Pointer
	if len(data) > 0 {
		ptr = unsafe.Pointer(&data[0])
	}
	var kr C.IOReturn
	rc := C.asi_control(t.d, C.uint8_t(reqType), C.uint8_t(bRequest), C.uint16_t(wValue), C.uint16_t(wIndex),
		ptr, C.uint16_t(len(data)), &done, &kr)
	if rc != 0 {
		return 0, fmt.Errorf("astrocam: control transfer (req 0x%02x) failed: IOReturn 0x%08x", bRequest, uint32(kr))
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

// AbortRead breaks a read in flight. The Go latch fails later reads fast, and the C-side
// generation breaks a read already blocked inside a cgo call. It gates on dMu, not closeMu, so an
// abort is never parked behind a Close waiting for the read it has to break.
func (t *darwinDevice) AbortRead() {
	t.readAborted.Store(true)
	t.dMu.Lock()
	defer t.dMu.Unlock()
	if !t.dFreed {
		C.asi_abort_reads(t.d)
	}
}

// ArmRead drops the entry latch and leaves the C generation alone. A read in flight has
// snapshotted its own generation, so a re-arm cannot cancel that read's abort.
func (t *darwinDevice) ArmRead() {
	t.readAborted.Store(false)
}

// BulkRead reads one whole frame with asi_read_frame_async (all transfers posted up front,
// 1 MiB each). Gates: closeMu shared, ioMu for the whole frame. Returns (0, nil) on abort,
// an error on a timeout with no data, and the contiguous prefix otherwise.
func (t *darwinDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	return t.bulkRead(buf, 0, timeout)
}

// BulkReadQuiet implements QuietBulkReader: the transfers are armed at once (the GPIF never
// streams without a reader), but ioMu is taken only when `quiet` elapses or the first data
// completion lands, whichever comes first, so TEC polls, telemetry and ST4 pulses flow during a
// host-timed integration. quiet 0 is BulkRead.
func (t *darwinDevice) BulkReadQuiet(buf []byte, quiet, timeout time.Duration) (int, error) {
	return t.bulkRead(buf, quiet, timeout)
}

// bulkRead is the one-shot batch read (asi_read_frame_async, all transfers armed up front) with
// an optional quiet window before the ioMu gate. Gates: closeMu shared, ioMu for the frame
// (after quiet). Returns (0, nil) on abort, (n, nil) on a short prefix.
func (t *darwinDevice) bulkRead(buf []byte, quiet, timeout time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	if len(buf) > maxFrameBytes {
		return 0, fmt.Errorf("astrocam: frame read of %d bytes exceeds the %d-byte limit", len(buf), maxFrameBytes)
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
		return 0, nil // abort latched: fail fast without taking ioMu
	}
	t.readActive.Add(1)
	defer t.readActive.Add(-1)
	ms := timeout.Milliseconds()
	if ms <= 0 {
		ms = defaultTimeoutBound.Milliseconds()
	}
	const chunk = 1 << 20 // 1 MiB per transfer (SDK xferLen)
	type readRes struct {
		n  C.uint32_t
		rc C.int
		kr C.IOReturn
	}
	finish := func(r readRes) (int, error) {
		if r.rc == -6 {
			t.forfeit(buf) // the kernel still owns buf
			return int(r.n), errTransportBroken
		}
		if r.rc == -7 { // stalled pipe or vanished device, with whatever arrived first
			return int(r.n), fmt.Errorf("astrocam: bulk pipe error IOReturn 0x%08x during a frame read", uint32(r.kr))
		}
		if r.rc != 0 {
			return 0, fmt.Errorf("astrocam: async bulk read setup failed (rc %d)", int(r.rc))
		}
		// A read that produced nothing is a short prefix, not a failure. All three backends
		// report (0, nil), and the sensor workers count zero reads and escalate to a pipe reset
		// and a re-arm. An error here would take a worker out of its ladder on this backend
		// alone.
		return int(r.n), nil
	}
	if quiet <= 0 {
		t.ioMu.Lock() // whole-frame gate (see ioMu)
		defer t.ioMu.Unlock()
		var n C.uint32_t
		var kr C.IOReturn
		rc := C.asi_read_frame_async(t.d, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)),
			C.uint32_t(chunk), C.uint32_t(ms), &n, &kr)
		return finish(readRes{n, rc, kr})
	}
	// Quiet window: the C read runs ungated on its own thread, and the gate closes at quiet
	// elapsed or first data. The first-data flag is cleared before the read starts, so the
	// previous read's flag cannot close the window at once.
	C.asi_rd_first_clear(t.d)
	done := make(chan readRes, 1)
	go func() {
		var n C.uint32_t
		var kr C.IOReturn
		rc := C.asi_read_frame_async(t.d, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)),
			C.uint32_t(chunk), C.uint32_t(ms), &n, &kr)
		done <- readRes{n, rc, kr}
	}()
	deadline := time.Now().Add(quiet)
	for {
		select {
		case r := <-done:
			return finish(r) // finished inside the quiet window (abort, or a fast frame)
		default:
		}
		if time.Now().After(deadline) || C.asi_rd_first(t.d) != 0 {
			break
		}
		time.Sleep(2 * time.Millisecond)
	}
	t.ioMu.Lock() // gate for the readout
	r := <-done
	t.ioMu.Unlock()
	return finish(r)
}

// ControlOutUngated issues an ST4 pulse edge without ioMu, so it does not queue behind a frame
// read holding the gate. ST4 only: ioMu exists to keep EP0 reads out of a readout. A write on the
// same interface handle alongside an in-flight bulk read is what the DDR path already does with
// its FPGABufReload pulses.
func (t *darwinDevice) ControlOutUngated(bRequest uint8, wValue, wIndex uint16) error {
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if t.closed {
		return errClosed
	}
	if t.broken.Load() {
		return errTransportBroken
	}
	var done C.uint32_t
	var kr C.IOReturn
	rc := C.asi_control(t.d, 0x40, C.uint8_t(bRequest), C.uint16_t(wValue), C.uint16_t(wIndex), nil, 0, &done, &kr)
	if rc != 0 {
		return fmt.Errorf("astrocam: control transfer (req 0x%02x) failed: IOReturn 0x%08x", bRequest, uint32(kr))
	}
	return nil
}

// ReadFrameStream reads one whole frame with the windowed pump. idle bounds a per-completion
// stall, after which the caller reads the remainder into buf[n:]; total bounds the whole read. A
// stall or abort returns the contiguous prefix with a nil error, a hard pipe error the prefix
// with an error. Gates: closeMu shared only.
func (t *darwinDevice) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	if len(buf) > maxFrameBytes {
		return 0, fmt.Errorf("astrocam: frame read of %d bytes exceeds the %d-byte limit", len(buf), maxFrameBytes)
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
		return 0, nil // abort latched: fail fast
	}
	t.readActive.Add(1)
	defer t.readActive.Add(-1)
	idleMs := idle.Milliseconds()
	if idleMs <= 0 {
		idleMs = defaultIdleBound.Milliseconds()
	}
	totalMs := total.Milliseconds()
	if totalMs <= 0 {
		totalMs = defaultTotalBound.Milliseconds()
	}
	const chunk = 1 << 20 // 1 MiB per transfer (SDK xferLen)
	var n C.uint32_t
	var kr C.IOReturn
	rc := C.asi_read_frame_stream(t.d, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)),
		C.uint32_t(chunk), C.uint32_t(idleMs), C.uint32_t(totalMs), &n, &kr)
	if rc == -6 {
		// The abandoned transfers target C-side scratch (leaked there), not buf, so no pin;
		// the pipe is poisoned all the same.
		t.broken.Store(true)
		return int(n), errTransportBroken
	}
	if rc == -7 {
		return int(n), fmt.Errorf("astrocam: bulk pipe error IOReturn 0x%08x during a frame read", uint32(kr))
	}
	if rc != 0 {
		return 0, fmt.Errorf("astrocam: windowed stream read setup failed (rc %d)", int(rc))
	}
	return int(n), nil
}

// ReadFrameStreamPrequeued queues the whole frame's transfers before the frame arrives, so the
// read overlaps the sensor readout. This is the USB2 path. total bounds the whole read; idle
// gates only after the first byte. A stall or abort returns the contiguous prefix with a nil
// error. Gates: closeMu shared, ioMu for the whole frame.
func (t *darwinDevice) ReadFrameStreamPrequeued(buf []byte, idle, total time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	if len(buf) > maxFrameBytes {
		return 0, fmt.Errorf("astrocam: frame read of %d bytes exceeds the %d-byte limit", len(buf), maxFrameBytes)
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
		return 0, nil // abort latched: fail fast without taking ioMu
	}
	t.readActive.Add(1)
	defer t.readActive.Add(-1)
	t.ioMu.Lock()
	defer t.ioMu.Unlock()
	idleMs := idle.Milliseconds()
	if idleMs <= 0 {
		idleMs = defaultIdleBound.Milliseconds()
	}
	totalMs := total.Milliseconds()
	if totalMs <= 0 {
		totalMs = defaultTotalBound.Milliseconds()
	}
	const chunk = 1 << 20 // 1 MiB per transfer (SDK xferLen)
	var n C.uint32_t
	var kr C.IOReturn
	rc := C.asi_read_frame_prequeued(t.d, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)),
		C.uint32_t(chunk), C.uint32_t(idleMs), C.uint32_t(totalMs), &n, &kr)
	if rc == -6 {
		t.forfeit(buf) // the kernel still owns buf
		return int(n), errTransportBroken
	}
	if rc == -7 {
		return int(n), fmt.Errorf("astrocam: bulk pipe error IOReturn 0x%08x during a frame read", uint32(kr))
	}
	if rc != 0 {
		return 0, fmt.Errorf("astrocam: prequeued frame read setup failed (rc %d)", int(rc))
	}
	return int(n), nil
}

// darwinStream is a resident windowed-stream session for the video and burst path. The session
// is single-consumer: Next, NextZC, Release and Close must not race each other.
type darwinStream struct {
	t *darwinDevice
	s *C.asicam_stream
}

// StartStream opens a persistent windowed stream and primes it. total sets the driver-side
// per-transfer timeout and so has to cover one frame period, not the whole burst. The session is
// registered on the device, so a transport Close stops it before releasing the interface its pump
// thread references. Gates: closeMu shared.
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
		totalMs = defaultTotalBound.Milliseconds()
	}
	// Chunk = one frame for sub-MiB frames (each transfer lands a whole frame, no cross-chunk
	// straddle, lowest latency); 1 MiB for large frames the window pipelines.
	chunk := 1 << 20
	if frameBytes > 0 && frameBytes < chunk {
		chunk = frameBytes
	}
	// One session per device. Two windows on the same bulk pipe compete for one byte stream:
	// each session's transfers take whatever segments arrive next, so both read torn frames and
	// neither can tell. The slot is reserved before arming, so two concurrent StartStream calls
	// cannot both pass the check.
	t.streamMu.Lock()
	if len(t.streams) > 0 {
		t.streamMu.Unlock()
		return nil, errStreamBusy
	}
	if t.streams == nil {
		t.streams = map[*darwinStream]struct{}{}
	}
	st := &darwinStream{t: t}
	t.streams[st] = struct{}{}
	t.streamMu.Unlock()

	s := C.asi_stream_start(t.d, C.uint32_t(chunk), C.uint32_t(totalMs))
	if s == nil {
		t.streamMu.Lock()
		delete(t.streams, st)
		t.streamMu.Unlock()
		return nil, fmt.Errorf("astrocam: stream session start failed")
	}
	st.s = s
	t.readActive.Add(1) // a session is a read in flight until it closes
	return st, nil
}

// maxFrameBytes bounds a single read. The C layer takes lengths as uint32_t and reports counts as
// int, so anything past 2 GiB truncates on the way in or comes back negative. Every sensor here
// tops out around 122 MB.
const maxFrameBytes = 1<<31 - 1

// Next pulls one frame (len(buf) bytes) from the resident stream. Gates: closeMu shared.
//
// A short count with a nil error means the idle window elapsed part way through a frame. Nothing
// can resync the session: the watermark has moved past the dropped bytes, and the FX3's ZLPs mark
// inter-buffer boundaries as well as frame ones. Every later Next returns ErrStreamDesynced until
// the session is closed and a new one started.
func (st *darwinStream) Next(buf []byte, idle time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	if len(buf) > maxFrameBytes {
		return 0, fmt.Errorf("astrocam: stream read of %d bytes exceeds the %d-byte limit", len(buf), maxFrameBytes)
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
		idleMs = defaultIdleBound.Milliseconds()
	}
	var kr C.IOReturn
	n := C.asi_stream_next(st.s, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)), C.uint32_t(idleMs), &kr)
	if n == -2 {
		return 0, ErrStreamDesynced
	}
	if n < 0 {
		return 0, fmt.Errorf("astrocam: stream read hard error IOReturn 0x%08x", uint32(kr))
	}
	return int(n), nil
}

// NextZC returns the next frame as a slice aliasing the session's scratch buffer. Use it when the
// stream's chunk == one frame (sub-MiB ROI). An idle stall returns nil, with the same no-resync
// consequence as Next. The slice is valid until the next call on this session, since Release,
// Next and NextZC all recycle a held slot. Gates: closeMu shared.
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
		idleMs = defaultIdleBound.Milliseconds()
	}
	var p *C.char
	var kr C.IOReturn
	n := C.asi_stream_next_zc(st.s, &p, C.uint32_t(idleMs), &kr)
	if n < 0 {
		return nil, fmt.Errorf("astrocam: stream read hard error IOReturn 0x%08x", uint32(kr))
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
	st.t.readActive.Add(-1)
	if rc != 0 {
		st.t.broken.Store(true)
		return errTransportBroken
	}
	return nil
}

// ResetEndpoint clears a stall on the bulk pipe (ClearPipeStallBothEnds). Gates: closeMu
// shared only; a worker teardown reset that lands after Close returns errClosed.
func (t *darwinDevice) ResetEndpoint(ep uint8) error {
	t.closeMu.RLock()
	defer t.closeMu.RUnlock()
	if t.closed {
		return errClosed
	}
	if t.broken.Load() {
		return errTransportBroken
	}
	// ClearPipeStallBothEnds is a CLEAR_FEATURE on EP0, and EP0 traffic inside a gated USB2
	// readout parks the FX3 GPIF, so it runs under ioMu as it does on Linux and Windows. Every
	// caller invokes this between reads, never while holding the gate, so it cannot deadlock.
	t.ioMu.Lock()
	defer t.ioMu.Unlock()
	if kr := C.asi_reset_pipe(t.d); kr != 0 {
		return fmt.Errorf("astrocam: clear pipe stall IOReturn 0x%08x", uint32(kr))
	}
	return nil
}

// ResetDevice issues a USB bus reset (DeviceResetter), the last-resort recovery; the device
// loses all state, so a re-Init is required after. Gates: closeMu shared only; it must run
// while a stuck read still holds ioMu.
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

// Close releases the IOKit interfaces and frees the handle. It takes closeMu exclusively, so it
// blocks until every in-flight transfer has returned. Any open stream session is stopped first,
// since its pump thread references the interface. A broken transport leaks the handle.
// Idempotent.
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
			t.readActive.Add(-1)
		}
		delete(t.streams, st)
	}
	t.streamMu.Unlock()
	if t.broken.Load() {
		return errTransportBroken // leak t.d: the kernel still owns transfers against it
	}
	// dFreed under dMu, so a concurrent ReadAborter call either sets the latch on a live t.d or
	// sees it gone.
	t.dMu.Lock()
	t.dFreed = true
	C.asi_close(t.d)
	C.free(unsafe.Pointer(t.d))
	t.dMu.Unlock()
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
