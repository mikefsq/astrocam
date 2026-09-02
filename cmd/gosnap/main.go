// Command gosnap is the bring-up and capture tool for the astrocam driver.
//
// The default mode connects, prints the camera identity, and disconnects. -capture runs
// full init and one exposure and reads a frame; -n with a .ser output records a burst.
// -v logs every USB transfer. The camera is selected by -vid/-pid, or by -serial (factory
// serial, as -list prints it) for every mode.
//
// Usage:
//
//	gosnap [-pid 0x1749 | -serial 06118f061f090900] [-capture] [-v] [-exposure 100ms] [-gain 200] [-out frame.fits]
//
// macOS uses IOUSBHost; Linux usbfs (needs udev access to the camera VID); Windows WinUSB.
//
// Files: main.go (flags, camera selection, dispatch), capture.go (-capture single frame and
// burst), tools.go (the one-shot tools and benches: -list, -thermal, -defects, -dumpflash,
// -flashat, -dumpregs, -guide, -tecoff, -heater, -cool, -regulate, -replay, -soak, -wedge),
// logt.go (the -v transport logger), fits.go, ser.go.
package main

import (
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/mikefsq/astrocam"
	_ "github.com/mikefsq/astrocam/sensors" // registers the PID -> sensor profile table
)

func main() {
	log.SetFlags(0)
	vidFlag := flag.Uint("vid", uint(astrocam.ZWO.VID), "USB vendor id (0x03c3 = ZWO, 0xa0a0 = PlayerOne)")
	pid := flag.Uint("pid", 0x1749, "USB product id")
	serial := flag.String("serial", "", "select the camera by factory serial (hex, as -list prints it) instead of -vid/-pid")
	capture := flag.Bool("capture", false, "run full init and one exposure, then read a frame")
	verbose := flag.Bool("v", false, "log every USB transfer")
	exposure := flag.Duration("exposure", 50*time.Millisecond, "exposure time (with -capture)")
	gain := flag.Int("gain", 0, "gain in 0.1 dB units (with -capture)")
	offset := flag.Int("offset", -1, "offset / black level (with -capture); -1 = sensor default")
	bin := flag.Int("bin", 1, "binning factor 1..4 (with -capture)")
	hwbin := flag.Bool("hwbin", false, "bin on the sensor where the profile can (SDK ASI_HARDWARE_BIN); default host bin")
	sensorMode := flag.Int("sensormode", 0, "sensor readout mode index (0 = normal; IMX585 PlayerOne: 1 = HDR, RAW16 only)")
	binsum := flag.Bool("binsum", false, "sum binned pixels instead of averaging (SDK POA_PIXEL_BIN_SUM)")
	frameLimit := flag.Int("framelimit", 0, "cap the frame rate in fps (SDK POA_FRAME_LIMIT); 0 = no cap")
	roi := flag.String("roi", "", "sub-frame ROI x,y,w,h in binned pixels (with -capture); empty = full frame")
	raw8 := flag.Bool("raw8", false, "capture RAW8 instead of RAW16 (with -capture)")
	highspeed := flag.Bool("highspeed", false, "10-bit high-speed readout, implies -raw8 (with -capture)")
	usb2 := flag.Bool("usb2", false, "drive the camera as if on USB2 (bandwidth budget, GPIF divider, EP0 pacing) while staying on the SuperSpeed wire")
	fpsPerc := flag.Int("fps", 0, "FPS / bandwidth percent, clamped to the vendor's range (0 = the vendor's per-link default)")
	out := flag.String("out", "frame.fits", "output file (with -capture): .fits/.fit = FITS, .ser = SER video, else raw")
	timeout := flag.Duration("timeout", 0, "max wait for one frame (single-frame -capture only; a burst uses its own per-frame bounds); 0 = exposure + wire time + 3 s")
	replay := flag.String("replay", "", "replay an SDK 'req reg val' write sequence from this file, then read a frame")
	nframes := flag.Int("n", 1, "with -capture: frames to read after one arm; >1 records only with a .ser -out, else the frames are discarded (throughput benchmark)")
	discard := flag.Bool("discard", false, "with -capture -n: capture frames without writing them (throughput benchmark)")
	fixDefects := flag.Bool("fixdefects", true, "apply the factory hot-pixel map to every frame (as the vendor SDKs do); -fixdefects=false for uncorrected sensor output")
	list := flag.Bool("list", false, "enumerate attached cameras with serials and exit")
	thermal := flag.Bool("thermal", false, "read sensor temperature and humidity and exit (read-only)")
	dumpflash := flag.String("dumpflash", "", "write the factory defect-map blob from SPI flash to this path and exit")
	guide := flag.String("guide", "", "issue one ST4 pulse and exit: dir,duration (e.g. N,250ms)")
	defects := flag.Bool("defects", false, "read the factory defect map and report it, then exit")
	flashat := flag.String("flashat", "", "dump a raw SPI flash range \"addr,len\" in hex and exit (e.g. 42000,40)")
	dumpregs := flag.String("dumpregs", "", "read these registers back and exit, writing nothing: comma list of hex sensor regs, \"f:\" prefix for FPGA, lo-hi for a range (e.g. 0x301a,f:0x14-0x18)")
	keepMarkers := flag.Bool("keepmarkers", false, "keep the FX3 DDR frame-marker corner pixels unrepaired")
	tecoff := flag.Bool("tecoff", false, "force the TEC and fan off and exit")
	heater := flag.Int("heater", -1, "set the anti-dew heater to this %% (0..100) and exit; -1 = unchanged")
	cool := flag.Bool("cool", false, "actuate the TEC: open-loop power steps, then closed-loop regulation")
	regulate := flag.Bool("regulate", false, "actuate the TEC: closed-loop regulate to -regtarget")
	regtarget := flag.Float64("regtarget", 10, "target temperature in °C for -regulate")
	dc := astrocam.DefaultCoolerConfig() // flag defaults follow the library gains
	kp := flag.Float64("kp", dc.Kp, "-regulate velocity-form Kp")
	ki := flag.Float64("ki", dc.Ki, "-regulate velocity-form Ki")
	kd := flag.Float64("kd", dc.Kd, "-regulate velocity-form Kd")
	slew := flag.Float64("slew", 0, "-regulate max TEC power change per tick in %% (0 = off)")
	seed := flag.Float64("seed", 0, "-regulate warm-start TEC power in %% (0 = none)")
	maxerr := flag.Float64("maxerr", 0, "-regulate clamp on |temp-setpoint| in °C (0 = off)")
	regtarget2 := flag.Float64("regtarget2", math.NaN(), "-regulate second target in °C after -regtarget settles; unset = single phase")
	wedge := flag.Bool("wedge", false, "reproduce the USB2 readout wedge with a concurrent ControlIn(0xB3) antagonist, then recover")
	wedgeIters := flag.Int("wedgeiters", 5000, "with -wedge: max frames before giving up")
	wedgeAntagonist := flag.Bool("wedgeantagonist", true, "with -wedge: run the ControlIn(0xB3) antagonist (false = A/B control)")
	wedgeInterval := flag.Int("wedgeinterval", 2, "with -wedge: antagonist interval in ms")
	wedgeReq := flag.Int("wedgereq", 0xB3, "with -wedge: the vendor IN request the antagonist issues (0xB3 temperature, 0xBC FPGA reg read, 0xAD firmware)")
	wedgeMaxSec := flag.Int("wedgemaxsec", 90, "with -wedge: wall-clock cap in seconds")
	soak := flag.Bool("soak", false, "single-threaded continuous capture of -soakframes frames, reporting the stall count")
	soakFrames := flag.Int("soakframes", 5000, "with -soak: number of frames")
	soakVideo := flag.Bool("soakvideo", false, "with -soak: use the free-run path (arm once) instead of arm-per-frame")
	soakPoll := flag.Bool("soakpoll", false, "with -soak: poll EP0 (temperature) at 200/s during the run, as an Alpaca client does")
	flag.Parse()
	// The driver corrects every frame from the factory map by default, as the vendor SDKs do;
	// this flag turns that off rather than turning a separate pass on.
	astrocam.RepairDefects = *fixDefects
	toolVerbose = *verbose
	if *keepMarkers {
		astrocam.RepairDMAMarkers = false
	}
	tg := target{vid: uint16(*vidFlag), pid: uint16(*pid), serial: *serial}
	o := captureOpts{
		exposure: *exposure, gain: *gain, offset: *offset, bin: *bin, hwbin: *hwbin, roi: *roi,
		sensorMode: *sensorMode,
		binsum:     *binsum, frameLimit: *frameLimit,
		raw8: *raw8 || *highspeed, out: *out, timeout: *timeout, nframes: *nframes, usb2: *usb2,
		discard: *discard, highspeed: *highspeed, fpsPerc: *fpsPerc, fixDefects: *fixDefects,
		antagonist: *soakPoll,
	}

	// One mode per run, first match wins; every mode selects the camera through tg.
	fail := func(what string, err error) {
		if err != nil {
			log.Fatalf("%s: %v", what, err)
		}
	}
	switch {
	case *list:
		fail("list", doList())
	case *thermal:
		fail("thermal", doThermal(tg))
	case *dumpflash != "":
		fail("dumpflash", doDumpFlash(tg, *dumpflash))
	case *dumpregs != "":
		fail("dumpregs", doDumpRegs(tg, *dumpregs))
	case *flashat != "":
		fail("flashat", doFlashAt(tg, *flashat))
	case *defects:
		fail("defects", doDefects(tg))
	case *guide != "":
		fail("guide", doGuide(tg, *guide))
	case *tecoff:
		fail("tecoff", doTECOff(tg))
	case *heater >= 0:
		fail("heater", doHeater(tg, *heater))
	case *cool:
		fail("cool", doCool(tg))
	case *regulate:
		fail("regulate", doRegulate(tg, *regtarget, *kp, *ki, *kd, *slew, *seed, *maxerr, *regtarget2))
	case *replay != "":
		fail("replay", doReplay(tg, *replay))
	case *wedge:
		fail("wedge", doWedge(tg, o, *wedgeIters, *wedgeAntagonist, *wedgeInterval, *wedgeMaxSec, uint8(*wedgeReq)))
	case *soak:
		fail("soak", doSoak(tg, o, *soakFrames, *soakVideo))
	default:
		fail("error", run(tg, *capture, *verbose, o))
	}
}

// target selects the camera: by factory serial when -serial is set, else by -vid/-pid.
type target struct {
	vid, pid uint16
	serial   string
}

// open opens the selected camera's transport. The returned target carries the VID/PID the
// selection resolved to (the serial lookup fills them in), which is what Camera binding needs.
func (tg target) open() (astrocam.Transport, target, error) {
	if tg.serial != "" {
		t, d, err := astrocam.OpenSerial(tg.serial)
		if err != nil {
			return nil, tg, err
		}
		tg.vid, tg.pid = d.VID, d.PID
		return t, tg, nil
	}
	t, err := astrocam.OpenHost(tg.vid, tg.pid)
	if err != nil {
		return nil, tg, fmt.Errorf("connect %04x:%04x: %w\n(camera plugged in, not claimed by another driver, accessible?)", tg.vid, tg.pid, err)
	}
	return t, tg, nil
}

// toolVerbose is set from -v so the tool subcommands log transfers too, not just -capture. They
// share openCamera, and a tool that cannot show its own bytes is hard to trust.
var toolVerbose bool

// openCamera opens the transport and binds the Camera on it. The caller closes the transport.
func openCamera(tg target) (astrocam.Transport, *astrocam.Camera, target, error) {
	raw, tg, err := tg.open()
	if err != nil {
		return nil, nil, tg, err
	}
	var t astrocam.Transport = raw
	if toolVerbose {
		t = &logT{t: raw, w: os.Stderr, start: time.Now()}
	}
	cam, err := astrocam.Open(t, tg.vid, tg.pid)
	if err != nil {
		raw.Close()
		return nil, nil, tg, fmt.Errorf("bind PID 0x%04x: %w", tg.pid, err)
	}
	return raw, cam, tg, nil
}

// exitHooks collects cleanups an interrupt must run before the process exits (a SER writer to
// close so its frame count is written, a stream session to stop); run executes them in reverse
// order of registration.
type exitHooks struct {
	mu  sync.Mutex
	fns []func()
}

func (h *exitHooks) add(f func()) { h.mu.Lock(); h.fns = append(h.fns, f); h.mu.Unlock() }
func (h *exitHooks) run() {
	h.mu.Lock()
	fns := h.fns
	h.fns = nil
	h.mu.Unlock()
	for i := len(fns) - 1; i >= 0; i-- {
		fns[i]()
	}
}

// installInterrupt makes SIGINT/SIGTERM leave the camera clean: it runs the registered hooks,
// then cam.Close (StopExposure, cooler off with the TEC zeroed, transport close), and exits 1. A
// sensor left free-running into a closed host backs up the FX3 GPIF until the firmware crashes,
// and an un-zeroed TEC keeps driving open-loop. The returned func removes the handler for a
// normal return.
func installInterrupt(cam *astrocam.Camera, hooks *exitHooks) func() {
	sigc := make(chan os.Signal, 1)
	signal.Notify(sigc, os.Interrupt, syscall.SIGTERM)
	done := make(chan struct{})
	go func() {
		select {
		case <-sigc:
			fmt.Println("\ninterrupted: stopping the camera")
			if hooks != nil {
				hooks.run()
			}
			_ = cam.Close()
			os.Exit(1)
		case <-done:
		}
	}()
	return func() { signal.Stop(sigc); close(done) }
}

// captureOpts bundles the -capture controls.
type captureOpts struct {
	exposure   time.Duration
	gain       int
	offset     int // -1 = sensor default
	bin        int
	hwbin      bool   // sensor-side binning (SetHardwareBin); default host bin
	sensorMode int    // readout programme index (SetSensorMode); 0 = normal
	binsum     bool   // sum rather than average binned pixels (SetBinSum)
	frameLimit int    // frame-rate cap in fps (SetFrameRateLimit); 0 = none
	roi        string // "x,y,w,h" in binned pixels, or "" = full frame
	raw8       bool
	out        string
	timeout    time.Duration
	nframes    int
	usb2       bool // force the USB2 readout path
	discard    bool // capture without writing (throughput benchmark)
	highspeed  bool // 10-bit high-speed readout (implies raw8)
	fpsPerc    int  // FPS / bandwidth percent; 0 = the vendor's per-link default
	fixDefects bool // apply the factory defect map to the frame
	antagonist bool // poll EP0 during the soak, as an Alpaca client's property reads do
}

func (o captureOpts) binOr1() int {
	if o.bin < 1 {
		return 1
	}
	return o.bin
}
