//go:build darwin

// macOS USB transport over IOKit's IOUSBLib (the C-callable USB host interface;
// the IOUSBHost stack). This is the one backend that needs cgo — there is no
// pure-Go path to IOKit on macOS.
//
// Scope: control transfers + two bulk read paths — BulkRead (asi_read_frame_async,
// post-N-and-wait, for small USB2 frames) and the FrameStreamer windowed pump
// (asi_read_frame_stream, ReadPipeAsync on a dedicated CFRunLoop) that streams large
// USB3 frames, keeping a window of transfers cycling and copying contiguously so a short
// packet at a burst boundary can't leave a gap. Stall detection is time-based on real
// data (not ZLP count) so the FX3's held final partial-buffer tail still completes.
//
// HARDWARE-VALIDATED: control plane + both bulk paths against real cameras — ASI174MM Mini
// (USB2) and ASI6200 MM/MC (full 122 MB frames over USB3 and USB2).

package asicam

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
} asicam_dev;

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

// reg_u32 / reg_str read a property off a USB device's IORegistry entry WITHOUT
// opening the device — the OS already cached idProduct, locationID and the USB
// product-name string at enumeration, so listing costs no bus traffic.
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

// asi_enumerate lists every VID-matched USB device (no open, no PID filter — the Go
// side filters to known camera PIDs). Fills up to max entries; returns the total found
// (which may exceed max, so the caller can detect truncation), or <0 on failure.
static int asi_enumerate(uint16_t vid, asicam_devinfo* out, int max) {
    // Match every USB device and filter by reading idVendor, rather than putting
    // idVendor in the match dictionary: on the modern IOUSBHost stack a class +
    // idVendor-only dictionary matches nothing (idVendor+idProduct does, which is the
    // open path), so we iterate all and compare the property ourselves.
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

// asi_open_location opens the VID-matched device sitting at a specific USB location id
// (the stable per-port key from asi_enumerate). Two identical cameras differ only here,
// so this is what lets the caller open the exact one it picked from the list.
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
    // DeviceRequestTO (not DeviceRequest): a bounded timeout so a wedged control
    // transfer can't hang the whole capture, plus a few RETRIES so one transient USB
    // glitch on an init/arm register write doesn't kill the capture (the observed
    // intermittent-failure mode: a single failed reg0 write in InitFPGA aborted Init).
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

// Async multi-transfer bulk read, driven from a DEDICATED, always-spinning CFRunLoop
// thread (mirroring how libusb's mac backend runs IOKit). The previous version pumped
// the runloop inline in 0.05 s slices, exiting it after every completion to run our
// collect logic; on a wait-mode FPGA (which paces its readout on the host servicing the
// pipe) that left the async event source unserviced between completions. Here a pthread
// runs CFRunLoopRun() continuously so completions are serviced the instant they fire.
//
// Lifecycle: the thread creates the interface async event source on ITS runloop and
// signals `ready`; the caller then submits N transfers (completions fire on the thread);
// the completion callback counts down and, when all N are in, signals `finished` and
// stops the runloop. On timeout we AbortPipe (which completes the rest), then reset the
// endpoint so the next capture starts from a quiescent pipe.
typedef struct asi_rd asi_rd;
typedef struct { volatile int done; uint32_t len; IOReturn kr; asi_rd* rd; } aslot;
struct asi_rd {
    aslot* slots;
    int n;
    volatile int completed;
    CFRunLoopRef rl;
    dispatch_semaphore_t finished;
};

static void asi_async_cb(void* refcon, IOReturn result, void* arg0) {
    aslot* s = (aslot*)refcon;
    s->len = (uint32_t)(uintptr_t)arg0;
    s->kr = result;
    s->done = 1;
    asi_rd* rd = s->rd;
    if (__sync_add_and_fetch(&rd->completed, 1) == rd->n) {
        dispatch_semaphore_signal(rd->finished);
        CFRunLoopStop(rd->rl);
    }
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

static int asi_read_frame_async(asicam_dev* d, void* buf, uint32_t bufSize,
                                uint32_t chunk, uint32_t timeoutMs, uint32_t* outLen) {
    *outLen = 0;
    if (!d->inPipe) return -2;
    if (chunk == 0) chunk = 1 << 20;
    int n = (int)((bufSize + chunk - 1) / chunk);

    asi_rd rd;
    rd.slots = (aslot*)calloc(n, sizeof(aslot));
    rd.n = n;
    rd.completed = 0;
    rd.rl = NULL;
    rd.finished = dispatch_semaphore_create(0);
    for (int i = 0; i < n; i++) rd.slots[i].rd = &rd;

    dispatch_semaphore_t ready = dispatch_semaphore_create(0);
    asi_rd_thr ta = { d, &rd, ready, NULL, kIOReturnSuccess };
    pthread_t th;
    if (pthread_create(&th, NULL, asi_rd_run, &ta) != 0) {
        free(rd.slots); dispatch_release(rd.finished); dispatch_release(ready);
        return -4;
    }
    dispatch_semaphore_wait(ready, DISPATCH_TIME_FOREVER); // runloop + source ready
    if (ta.srcErr != kIOReturnSuccess) {
        pthread_join(th, NULL);
        free(rd.slots); dispatch_release(rd.finished); dispatch_release(ready);
        return -3;
    }

    // Submit all transfers; completions fire on the dedicated runloop thread.
    for (int i = 0; i < n; i++) {
        uint32_t off = (uint32_t)i * chunk;
        uint32_t len = (off + chunk <= bufSize) ? chunk : (bufSize - off);
        IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, (char*)buf + off, len,
                          timeoutMs, timeoutMs, (IOAsyncCallback1)asi_async_cb, &rd.slots[i]);
        if (kr != kIOReturnSuccess) {
            rd.slots[i].done = 1; rd.slots[i].kr = kr;
            if (__sync_add_and_fetch(&rd.completed, 1) == n) {
                dispatch_semaphore_signal(rd.finished); CFRunLoopStop(rd.rl);
            }
        }
    }

    // Wait for all transfers to complete, or time out.
    dispatch_time_t dl = dispatch_time(DISPATCH_TIME_NOW, (int64_t)timeoutMs * 1000000LL + 500000000LL);
    if (dispatch_semaphore_wait(rd.finished, dl) != 0) {
        (*d->intf)->AbortPipe(d->intf, d->inPipe); // forces remaining callbacks -> finished
        dispatch_semaphore_wait(rd.finished, dispatch_time(DISPATCH_TIME_NOW, 2000000000LL));
    }
    CFRunLoopStop(rd.rl); // ensure the thread's runloop exits even on a race
    pthread_join(th, NULL);

    // Bytes transferred, in order, up to and INCLUDING the frame-terminating short
    // transfer. A bulk-IN that returns fewer bytes than requested completes with
    // kIOReturnUnderrun, not kIOReturnSuccess — the FX3 ends a frame with a short
    // packet, so the final sub-chunk transfer legitimately underruns. Its bytes are
    // REAL pixel data and must be counted; the short read just marks the frame
    // boundary, so stop after it. Only a slot that never completed, or that failed
    // with a non-underrun error, truncates the frame (a dropped/stalled transfer).
    uint32_t total = 0;
    for (int i = 0; i < n; i++) {
        if (!rd.slots[i].done) break;
        if (rd.slots[i].kr != kIOReturnSuccess && rd.slots[i].kr != kIOReturnUnderrun) break;
        total += rd.slots[i].len;
        uint32_t off = (uint32_t)i * chunk;
        uint32_t req = (off + chunk <= bufSize) ? chunk : (bufSize - off);
        if (rd.slots[i].len < req) break; // short packet = end of frame
    }
    *outLen = total;

    (*d->intf)->ClearPipeStallBothEnds(d->intf, d->inPipe); // quiescent pipe for next capture
    free(rd.slots);
    dispatch_release(rd.finished);
    dispatch_release(ready);
    return 0;
}

// ---- Continuous windowed stream read (USB3 large frames) -------------------
//
// asi_read_frame_async (above) posts ceil(size/chunk) transfers ONCE and waits for
// all to settle: on a bursty USB3 stream the trailing transfers complete empty when
// an intermediate short packet lands, so it returns a truncated frame. The ASI6200
// (IMX455) SDK reads differently — startAsyncXfer keeps a small WINDOW of
// transfers cycling on the pipe, resubmitting each as it completes, until the whole
// frame is in. That is what this does.
//
// A single bulk-IN pipe completes transfers in submission order, so completions are
// reaped FIFO; each completed chunk is copied to a contiguous watermark in the frame
// buffer (NOT a fixed chunk*offset), which keeps the frame gap-free even when a chunk
// returns short at a USB burst boundary. It stops only when the whole frame is in, on
// an idle stall (no completion within idleMs — the caller then kicks the FPGA and
// calls again for the remainder), or after totalMs. Returns contiguous bytes in *outLen.
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
static int asi_read_frame_stream(asicam_dev* d, void* buf, uint32_t bufSize, uint32_t chunk,
                                 uint32_t idleMs, uint32_t totalMs, uint32_t* outLen) {
    *outLen = 0;
    if (!d->inPipe) return -2;
    if (chunk == 0) chunk = 1 << 20;
    uint32_t nchunks = (bufSize + chunk - 1) / chunk;
    int window = ASI_SWIN;
    if ((uint32_t)window > nchunks) window = (int)nchunks;

    asi_sd sd; memset(&sd, 0, sizeof(sd));
    sd.window = window; sd.d = d;
    sd.comp = dispatch_semaphore_create(0);
    sd.ready = dispatch_semaphore_create(0);
    pthread_mutex_init(&sd.qlk, NULL);
    for (int i = 0; i < window; i++) {
        sd.slots[i].sd = &sd; sd.slots[i].idx = i;
        sd.slots[i].scratch = (char*)malloc(chunk);
        if (!sd.slots[i].scratch) {
            for (int j = 0; j < i; j++) free(sd.slots[j].scratch);
            dispatch_release(sd.comp); dispatch_release(sd.ready);
            pthread_mutex_destroy(&sd.qlk); return -5;
        }
    }

    pthread_t th;
    if (pthread_create(&th, NULL, asi_sd_run, &sd) != 0) {
        for (int i = 0; i < window; i++) free(sd.slots[i].scratch);
        dispatch_release(sd.comp); dispatch_release(sd.ready);
        pthread_mutex_destroy(&sd.qlk); return -4;
    }
    dispatch_semaphore_wait(sd.ready, DISPATCH_TIME_FOREVER);
    if (sd.srcErr != kIOReturnSuccess) {
        pthread_join(th, NULL);
        for (int i = 0; i < window; i++) free(sd.slots[i].scratch);
        dispatch_release(sd.comp); dispatch_release(sd.ready);
        pthread_mutex_destroy(&sd.qlk); return -3;
    }

    uint32_t received = 0; // contiguous bytes confirmed into buf
    int inflight = 0;
    // Prime the window: each slot reads a full chunk into its own scratch buffer.
    for (int i = 0; i < window; i++) {
        sd.slots[i].done = 0;
        IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, sd.slots[i].scratch, chunk,
                          totalMs, totalMs, (IOAsyncCallback1)asi_stream_cb, &sd.slots[i]);
        if (kr != kIOReturnSuccess) break;
        inflight++;
    }

    // Stall detection is TIME-based on REAL data, not zero-length-packet COUNT: when the
    // FX3 holds the frame's final partial DMA buffer it emits a BURST of zero-length
    // packets (fast on USB2), so a count guard trips long before the tail commits (via
    // FPGABufReload or the next free-run frame). Give up only after idleMs with no actual
    // bytes; a ZLP blocks ~one packet time on the semaphore, so this doesn't busy-spin.
    int64_t sliceNs = (int64_t)idleMs * 1000000LL;
    if (sliceNs > 100000000LL) sliceNs = 100000000LL; // wake at least every 100 ms to re-check
    int64_t lastReal = asi_now_ms();
    while (received < bufSize && inflight > 0) {
        if (dispatch_semaphore_wait(sd.comp, dispatch_time(DISPATCH_TIME_NOW, sliceNs)) != 0) {
            if (asi_now_ms() - lastReal > (int64_t)idleMs) break; // no real data for the idle window
            continue;
        }
        pthread_mutex_lock(&sd.qlk);
        int slot = sd.q[sd.qh % (ASI_SWIN * 4)]; sd.qh++;
        pthread_mutex_unlock(&sd.qlk);
        inflight--;
        sslot* s = &sd.slots[slot];
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
            // A zero-length completion is the FX3's inter-buffer / frame-boundary marker,
            // NOT end-of-frame: the tail of the last partial 1-MiB DMA buffer is still to
            // come (FPGABufReload, or the next free-run frame, commits it). Keep cycling.
            s->done = 0;
            IOReturn kr = (*d->intf)->ReadPipeAsyncTO(d->intf, d->inPipe, s->scratch, chunk,
                              totalMs, totalMs, (IOAsyncCallback1)asi_stream_cb, s);
            if (kr == kIOReturnSuccess) inflight++;
        }
    }

    // Abort outstanding transfers and drain their callbacks before sd leaves scope,
    // so no completion fires against freed state.
    (*d->intf)->AbortPipe(d->intf, d->inPipe);
    while (inflight > 0) {
        if (dispatch_semaphore_wait(sd.comp, dispatch_time(DISPATCH_TIME_NOW, 2000000000LL)) != 0) break;
        pthread_mutex_lock(&sd.qlk); sd.qh++; pthread_mutex_unlock(&sd.qlk);
        inflight--;
    }
    CFRunLoopStop(sd.rl);
    pthread_join(th, NULL);
    (*d->intf)->ClearPipeStallBothEnds(d->intf, d->inPipe);
    for (int i = 0; i < window; i++) free(sd.slots[i].scratch);
    dispatch_release(sd.comp); dispatch_release(sd.ready);
    pthread_mutex_destroy(&sd.qlk);
    *outLen = received;
    return 0;
}

// ---- Persistent stream session (video / planetary burst) -------------------------
// asi_read_frame_stream above sets up the window, spawns the CFRunLoop thread, reads ONE
// frame, and tears it all down — a fixed per-frame cost (~tens of ms) that dominates when
// frames are small. The session below keeps that machinery RESIDENT across a whole burst:
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

// asi_stream_next_zc is the ZERO-COPY variant: instead of memcpy'ing a frame into a caller
// buffer, it returns a pointer to the window scratch buffer that already holds the next
// whole frame, and HOLDS that slot (does not recycle it) until asi_stream_release. Valid
// only when chunk == one frame (sub-MiB ROI), so the frame is contiguous in one scratch —
// the common planetary case. *outBuf = scratch; returns the frame length, 0 on idle stall,
// -1 on a hard error. The caller MUST consume *outBuf before calling release (which re-arms
// the slot and overwrites it).
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
        // semaphore count can't run away over a long burst (which would busy-spin + defeat the
        // stall guard). The 50 ms slice + outer re-check covers a drain/block race.
        while (dispatch_semaphore_wait(sd->comp, DISPATCH_TIME_NOW) == 0) {}
        if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, sliceNs)) != 0) {
            if (!progressed && asi_now_ms() - lastReal > (int64_t)idleMs) break; // genuine stall
        }
    }
    return (int)copied;
}

static void asi_stream_stop(asicam_stream* s) {
    if (!s) return;
    asi_sd* sd = &s->sd;
    if (s->started) {
        (*sd->d->intf)->AbortPipe(sd->d->intf, sd->d->inPipe);
        for (int g = 0; g < sd->window + 4; g++)
            if (dispatch_semaphore_wait(sd->comp, dispatch_time(DISPATCH_TIME_NOW, 500000000LL)) != 0) break;
        if (sd->rl) CFRunLoopStop(sd->rl);
        pthread_join(s->th, NULL);
        (*sd->d->intf)->ClearPipeStallBothEnds(sd->d->intf, sd->d->inPipe);
    }
    asi_stream_free(s);
}

static int asi_reset_pipe(asicam_dev* d) {
    if (!d->inPipe) return -2;
    return (int)(*d->intf)->ClearPipeStallBothEnds(d->intf, d->inPipe);
}

// asi_reset_device issues a USB bus reset (the last-resort recovery). It re-runs the
// FX3 firmware to a known state; the device keeps its address (no re-enumerate), but
// all camera state is lost, so a re-Init is required afterwards.
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
	"fmt"
	"sync"
	"time"
	"unsafe"
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

	// ctrlMu serializes control transfers (EP0). One open camera is driven by
	// several goroutines at once — the capture path's register writes and the TEC
	// loop's SendCMD(0xB2) most of all — and the IOUSBLib device handle is not safe
	// for concurrent DeviceRequestTO (the SDK wraps SendCMD in a pthread_mutex for
	// the same reason). Bulk reads (EP 0x81) are a separate pipe and stay UNLOCKED,
	// so a multi-second readout can't stall a TEC tick.
	ctrlMu sync.Mutex
}

// OpenIOUSBHost finds the first device matching vid/pid via IOKit, opens it, and
// claims its bulk interface. On failure it reports exactly where it stopped.
func OpenIOUSBHost(vid, pid uint16) (*darwinDevice, error) {
	dev := (*C.asicam_dev)(C.calloc(1, C.size_t(unsafe.Sizeof(C.asicam_dev{}))))
	t := &darwinDevice{d: dev}
	if rc := C.asi_open(C.uint16_t(vid), C.uint16_t(pid), dev, &t.diag); rc != 0 {
		C.free(unsafe.Pointer(dev))
		return nil, openError(vid, pid, &t.diag)
	}
	return t, nil
}

// enumerateRaw lists every ZWO-VID USB device the OS sees, without opening any (the
// shared Enumerate filters these to known camera PIDs). It reads idProduct, locationID
// and the cached USB product-name string straight from the IORegistry.
func enumerateRaw(vid uint16) ([]DeviceInfo, error) {
	const max = 32
	arr := make([]C.asicam_devinfo, max)
	n := int(C.asi_enumerate(C.uint16_t(vid), &arr[0], C.int(max)))
	if n < 0 {
		return nil, fmt.Errorf("asicam: USB enumerate failed (rc %d)", n)
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
// claims its bulk interface — the way to bind the exact unit chosen from Enumerate when
// several identical cameras are attached.
func OpenLocation(vid uint16, loc uint32) (Transport, error) {
	dev := (*C.asicam_dev)(C.calloc(1, C.size_t(unsafe.Sizeof(C.asicam_dev{}))))
	t := &darwinDevice{d: dev}
	if rc := C.asi_open_location(C.uint16_t(vid), C.uint32_t(loc), dev, &t.diag); rc != 0 {
		C.free(unsafe.Pointer(dev))
		if t.diag.matched == 0 {
			return nil, fmt.Errorf("asicam: no device at USB location 0x%08x (unplugged or moved)", loc)
		}
		if kr := uint32(t.diag.openKR); kr != 0 {
			return nil, fmt.Errorf("asicam: device at location 0x%08x found but USBDeviceOpen failed (IOReturn 0x%08x — busy/exclusively open?)", loc, kr)
		}
		return nil, fmt.Errorf("asicam: device at location 0x%08x found but interface/bulk setup failed (rc %d, %d endpoints, inPipe %d)",
			loc, int(t.diag.ifaceKR), int(t.diag.numEndpoints), int(t.diag.inPipe))
	}
	return t, nil
}

func openError(vid, pid uint16, d *C.asicam_diag) error {
	switch {
	case d.matched == 0:
		return fmt.Errorf("asicam: no device %04x:%04x found — not plugged in, wrong PID, or claimed by another driver before it enumerated", vid, pid)
	case d.openKR != 0:
		kr := uint32(d.openKR)
		hint := ""
		switch kr {
		case kIOReturnExclusiveAccess:
			hint = " — busy/exclusively open (another app, or a prior run that didn't Close cleanly); unplug and replug to clear"
		case kIOReturnNoDevice:
			hint = " — device went away mid-open"
		}
		return fmt.Errorf("asicam: %04x:%04x found but USBDeviceOpen failed (IOReturn 0x%08x)%s", vid, pid, kr, hint)
	default:
		return fmt.Errorf("asicam: %04x:%04x opened but interface/bulk setup failed (rc %d, %d endpoints, inPipe %d)",
			vid, pid, int(d.ifaceKR), int(d.numEndpoints), int(d.inPipe))
	}
}

// Describe returns the negotiated transport facts (for bring-up diagnostics).
func (t *darwinDevice) Describe() string {
	link := "?"
	switch int(t.diag.inMaxPacket) {
	case 512:
		link = "USB2 HighSpeed"
	case 1024:
		link = "USB3 SuperSpeed"
	}
	return fmt.Sprintf("IOUSBHost: %d endpoints, bulk-IN pipe %d, maxPacket %d (%s)",
		int(t.diag.numEndpoints), int(t.diag.inPipe), int(t.diag.inMaxPacket), link)
}

// SuperSpeed reports whether the bulk-IN endpoint negotiated USB3 SuperSpeed (1024-byte
// max packet) rather than USB2 HighSpeed (512) — the live link speed the readout mode
// follows (see superSpeedReporter / Camera.Open).
func (t *darwinDevice) SuperSpeed() bool { return int(t.diag.inMaxPacket) >= 1024 }

func (t *darwinDevice) control(reqType, bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	t.ctrlMu.Lock()
	defer t.ctrlMu.Unlock()
	var done C.uint32_t
	var ptr unsafe.Pointer
	if len(data) > 0 {
		ptr = unsafe.Pointer(&data[0])
	}
	rc := C.asi_control(t.d, C.uint8_t(reqType), C.uint8_t(bRequest), C.uint16_t(wValue), C.uint16_t(wIndex),
		ptr, C.uint16_t(len(data)), &done)
	if rc != 0 {
		return 0, fmt.Errorf("asicam: control transfer (req 0x%02x) failed", bRequest)
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

func (t *darwinDevice) BulkRead(buf []byte, timeout time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	ms := timeout.Milliseconds()
	if ms <= 0 {
		ms = 2000
	}
	// asi_read_frame_async submits ceil(len(buf)/chunk) transfers across the buffer,
	// drains them in order, and tears the pipe down cleanly (see its comment). 1 MiB
	// matches the SDK's xferLen and keeps the transfer count (and thus the teardown
	// cancellations) low.
	const chunk = 1 << 20 // 1 MiB per transfer
	var n C.uint32_t
	rc := C.asi_read_frame_async(t.d, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)),
		C.uint32_t(chunk), C.uint32_t(ms), &n)
	if rc != 0 {
		return 0, fmt.Errorf("asicam: async bulk read setup failed (rc %d)", int(rc))
	}
	if n == 0 {
		return 0, fmt.Errorf("asicam: async bulk read got no data (timeout)")
	}
	return int(n), nil
}

// readFrameStream reads one whole frame with the continuous windowed pump
// (asi_read_frame_stream): a small window of transfers kept cycling on EP 0x81 until
// len(buf) bytes are in, copied contiguously so a short packet at a USB burst boundary
// can't leave a gap. idle bounds a per-completion stall (the caller recovers and reads
// the remainder); total bounds the whole read. Returns the contiguous bytes received —
// which may be < len(buf) on a stall, leaving the caller to continue into buf[n:].
// This is the data plane the ASI6200 (IMX455) worker uses; the 174 path is unaffected.
// (Exported to satisfy FrameStreamer so the -v logging wrapper can forward it.)
func (t *darwinDevice) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
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
	if rc != 0 {
		return 0, fmt.Errorf("asicam: windowed stream read setup failed (rc %d)", int(rc))
	}
	return int(n), nil
}

// darwinStream is a resident windowed-stream session (the video/burst path) — the C
// pump stays primed across frames so the per-frame setup cost is paid once.
type darwinStream struct {
	s *C.asicam_stream
}

// StartStream opens a persistent windowed stream and primes it. frameBytes is informational
// (each Next call passes the actual buffer); total is the per-transfer timeout.
func (t *darwinDevice) StartStream(frameBytes int, total time.Duration) (FrameStream, error) {
	totalMs := total.Milliseconds()
	if totalMs <= 0 {
		totalMs = 5000
	}
	// Chunk = one frame for sub-MiB frames (each transfer lands a whole frame — no
	// cross-chunk straddle, lowest latency); 1 MiB for large frames the window pipelines.
	chunk := 1 << 20
	if frameBytes > 0 && frameBytes < chunk {
		chunk = frameBytes
	}
	s := C.asi_stream_start(t.d, C.uint32_t(chunk), C.uint32_t(totalMs))
	if s == nil {
		return nil, fmt.Errorf("asicam: stream session start failed")
	}
	return &darwinStream{s: s}, nil
}

// Next pulls exactly one frame (len(buf) bytes) from the resident stream.
func (st *darwinStream) Next(buf []byte, idle time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	idleMs := idle.Milliseconds()
	if idleMs <= 0 {
		idleMs = 800
	}
	n := C.asi_stream_next(st.s, unsafe.Pointer(&buf[0]), C.uint32_t(len(buf)), C.uint32_t(idleMs))
	if n < 0 {
		return 0, fmt.Errorf("asicam: stream read hard error")
	}
	return int(n), nil
}

// NextZC returns the next frame as a slice that ALIASES the session's scratch buffer (no
// copy). The slice is valid only until the next Release call (which re-arms the slot and
// overwrites the memory), so the caller must consume it before releasing. Returns nil on an
// idle stall. Use when the stream's chunk == one frame (sub-MiB ROI).
func (st *darwinStream) NextZC(idle time.Duration) ([]byte, error) {
	idleMs := idle.Milliseconds()
	if idleMs <= 0 {
		idleMs = 800
	}
	var p *C.char
	n := C.asi_stream_next_zc(st.s, &p, C.uint32_t(idleMs))
	if n < 0 {
		return nil, fmt.Errorf("asicam: stream read hard error")
	}
	if n == 0 || p == nil {
		return nil, nil // idle stall
	}
	return unsafe.Slice((*byte)(unsafe.Pointer(p)), int(n)), nil
}

// Release recycles the slot handed out by the last NextZC (re-arms it for the next frame).
func (st *darwinStream) Release() { C.asi_stream_release(st.s) }

// Close aborts and frees the stream session.
func (st *darwinStream) Close() error {
	if st.s != nil {
		C.asi_stream_stop(st.s)
		st.s = nil
	}
	return nil
}

// ResetEndpoint clears a stall/flushes the bulk pipe (ClearPipeStallBothEnds).
func (t *darwinDevice) ResetEndpoint(ep uint8) error {
	if kr := C.asi_reset_pipe(t.d); kr != 0 {
		return fmt.Errorf("asicam: clear pipe stall IOReturn 0x%08x", uint32(kr))
	}
	return nil
}

// ResetDevice issues a USB bus reset (DeviceResetter) — the last-resort recovery. The
// device loses all state, so a re-Init is required after.
func (t *darwinDevice) ResetDevice() error {
	if kr := C.asi_reset_device(t.d); kr != 0 {
		return fmt.Errorf("asicam: reset device IOReturn 0x%08x", uint32(kr))
	}
	return nil
}

func (t *darwinDevice) Close() error {
	C.asi_close(t.d)
	C.free(unsafe.Pointer(t.d))
	return nil
}

// OpenHost opens the default USB backend for this platform (macOS: IOUSBHost).
func OpenHost(vid, pid uint16) (Transport, error) {
	d, err := OpenIOUSBHost(vid, pid)
	if err != nil {
		return nil, err
	}
	return d, nil
}
