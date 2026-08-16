// Command gosnap is the bring-up tool for the pure-Go ZWO camera driver.
//
// Default mode connects, prints the camera identity, and disconnects. With -capture
// it runs full init + one exposure and reads a frame. With -v it logs every USB
// transfer on the wire.
//
// Usage:
//
//	gosnap [-pid 0x1749] [-capture] [-v] [-exposure 100ms] [-gain 200] [-out frame.raw]
//
// macOS uses IOUSBHost; Linux usbfs (needs udev access to VID 0x03C3); Windows WinUSB.
package main

import (
	"encoding/binary"
	"flag"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"os/signal"
	"sort"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"

	"github.com/mikefsq/astrocam"
	_ "github.com/mikefsq/astrocam/sensors" // registers the PID -> sensor profile table
)

func main() {
	log.SetFlags(0)
	vidFlag := flag.Uint("vid", uint(astrocam.ZWO.VID), "USB vendor id (default 0x03c3 = ZWO; 0xa0a0 = PlayerOne)")
	pid := flag.Uint("pid", 0x1749, "USB product id (default 0x1749 = ASI174MM Mini)")
	capture := flag.Bool("capture", false, "run full init + one exposure and read a frame (first-light test)")
	verbose := flag.Bool("v", false, "log every USB transfer on the wire")
	exposure := flag.Duration("exposure", 50*time.Millisecond, "exposure time (with -capture). Per-sensor long-exposure handling is implemented: the 174 (>4s) and 6200 (>=1s) switch to FPGA wait+trigger mode and host-time the integration, so wall-clock ≈ exposure + readout")
	gain := flag.Int("gain", 0, "gain in 0.1 dB units (with -capture)")
	offset := flag.Int("offset", -1, "offset / black level (with -capture); -1 = leave default")
	bin := flag.Int("bin", 1, "binning factor 1..4 (with -capture)")
	roi := flag.String("roi", "", "sub-frame ROI as x,y,w,h in BINNED pixels (with -capture); empty = full binned frame")
	raw8 := flag.Bool("raw8", false, "capture RAW8 (1 byte/pixel) instead of RAW16 (with -capture)")
	highspeed := flag.Bool("highspeed", false, "10-bit HIGH-SPEED readout (~2× fps; implies RAW8): the sensor's ASI_HIGH_SPEED_MODE, a shorter ADC ramp at a doubled pixel clock (with -capture)")
	usb2 := flag.Bool("usb2", false, "force the USB2 HighSpeed readout path (bwUSB2 bandwidth budget) regardless of the model/link, to toggle the USB2 vs USB3 behavior on a fixed physical link without replugging")
	fpsPerc := flag.Int("fps", 0, "bandwidth-overload / FPS percent 40..100 (0 = default 100 = max throughput, matching the SDK). Lower throttles the readout (larger HMAX)")
	out := flag.String("out", "frame.fits", "frame output file (with -capture); .fits/.fit writes FITS, any other extension writes raw RAW16")
	timeout := flag.Duration("timeout", 5*time.Second, "max wait for the frame (with -capture); on USB2 the abort queues behind the in-flight read (ioMu), so expiry mostly bounds the wait rather than cutting the read short")
	replay := flag.String("replay", "", "replay an SDK 'req reg val' write sequence (file) then read a frame")
	nframes := flag.Int("n", 1, "with -capture: after one arm, read N frames back-to-back (no re-arm) and time each: the cold-start/throwaway test")
	discard := flag.Bool("discard", false, "with -capture -n N: capture frames via the resident stream but DON'T write them: the pure-capture throughput benchmark, matching the SDK video bench (no disk I/O)")
	fixDefects := flag.Bool("fixdefects", false, "with -capture: apply the factory defect map (read once from flash) to the RAW16 frame (neighbour-average each hot/dead pixel). OFF by default: raw frames are better fixed by dithering + integration")
	list := flag.Bool("list", false, "enumerate attached cameras (with serials) and exit")
	serial := flag.String("serial", "", "open the camera with this factory serial (hex) instead of -pid")
	thermal := flag.Bool("thermal", false, "read sensor temperature + humidity via the hardware Thermal backend and exit (read-only; no TEC actuation)")
	dumpflash := flag.String("dumpflash", "", "read the factory defect map from SPI flash (0x40000) and write the raw blob to this path, then exit (read-only)")
	keepMarkers := flag.Bool("keepmarkers", false, "do NOT repair the FX3 DDR frame header/footer marker pixels: deliver the genuine raw frame with the corner marker pixels intact (default off: markers are repaired on sensors that have them, e.g. IMX455/IMX462)")
	tecoff := flag.Bool("tecoff", false, "force the TEC drive and fan OFF and exit (safety/recovery)")
	heater := flag.Int("heater", -1, "set the anti-dew lens heater to this %% (0..100) and exit; reads back reg 0x2a/0x19. -1 = leave unchanged")
	cool := flag.Bool("cool", false, "ACTUATE the TEC: open-loop power steps then a closed-loop regulation run (returns to 0 on exit/Ctrl-C)")
	regulate := flag.Bool("regulate", false, "ACTUATE the TEC: closed-loop regulate to -regtarget (returns to 0 on exit/Ctrl-C). Tune with -kp/-ki/-kd/-slew/-seed.")
	regtarget := flag.Float64("regtarget", 10, "target temperature °C for -regulate")
	dc := astrocam.DefaultCoolerConfig() // flag defaults track the library's velocity-form gains
	kp := flag.Float64("kp", dc.Kp, "-regulate velocity-form Kp (damps the approach)")
	ki := flag.Float64("ki", dc.Ki, "-regulate velocity-form Ki (drive ramp rate; raise to arrive faster)")
	kd := flag.Float64("kd", dc.Kd, "-regulate velocity-form Kd (fine damping)")
	slew := flag.Float64("slew", 0, "-regulate max TEC power change per tick %% (0 = disabled). A small value damps overshoot.")
	seed := flag.Float64("seed", 0, "-regulate warm-start TEC power %% (0 = none; high values overshoot)")
	maxerr := flag.Float64("maxerr", 0, "-regulate clamp on |temp-setpoint| °C the PID acts on (0 = off; ~3 = SDK-like gentle glide, both directions)")
	regtarget2 := flag.Float64("regtarget2", math.NaN(), "-regulate second target °C: after reaching -regtarget, ramp to this (tests warmup); unset = single phase")
	wedge := flag.Bool("wedge", false, "REPRODUCE the USB2 readout wedge: run continuous USB2 captures while an antagonist goroutine fires ControlIn(0xB3) into the bulk-read window (the goastro telemetry-poll interleave), loop until a frame fails, then confirm the wedge is sticky and recover via ResetDevice+Init")
	wedgeIters := flag.Int("wedgeiters", 5000, "with -wedge: max frames to capture before giving up")
	wedgeAntagonist := flag.Bool("wedgeantagonist", true, "with -wedge: run the concurrent ControlIn(0xB3) antagonist. false = A/B control (single-threaded stress should NOT wedge)")
	wedgeInterval := flag.Int("wedgeinterval", 2, "with -wedge: antagonist ControlIn(0xB3) interval in ms (smaller = more overlap with the readout)")
	wedgeMaxSec := flag.Int("wedgemaxsec", 90, "with -wedge: hard wall-clock cap in seconds (never runs longer)")
	soak := flag.Bool("soak", false, "single-threaded continuous-capture SOAK: run -soakframes frames (no antagonist) and report the readout StallCount, to find whether soft stalls still happen absent concurrency. Honors -exposure/-gain/-usb2/-fps")
	soakFrames := flag.Int("soakframes", 5000, "with -soak: number of frames to capture")
	soakVideo := flag.Bool("soakvideo", false, "with -soak: use the FREE-RUN path (StartVideo arm-once + ReadFrame loop, the SDK ASIStartVideoCapture shape) instead of single-shot arm-per-frame: the discriminator for whether the per-frame arm sequence is what wedges the FX3")
	flag.Parse()
	vid := uint16(*vidFlag)
	if *keepMarkers {
		astrocam.RepairDMAMarkers = false
	}

	if *list {
		if err := doList(); err != nil {
			log.Fatalf("list error: %v", err)
		}
		return
	}
	if *thermal {
		if err := doThermal(vid, uint16(*pid)); err != nil {
			log.Fatalf("thermal error: %v", err)
		}
		return
	}
	if *dumpflash != "" {
		if err := doDumpFlash(vid, uint16(*pid), *dumpflash); err != nil {
			log.Fatalf("dumpflash: %v", err)
		}
		return
	}
	if *tecoff {
		raw, err := astrocam.OpenHost(vid, uint16(*pid))
		if err != nil {
			log.Fatalf("tecoff: %v", err)
		}
		defer raw.Close()
		cam, err := astrocam.Open(raw, vid, uint16(*pid))
		if err != nil {
			log.Fatalf("tecoff: %v", err)
		}
		th := cam.HardwareThermal()
		_ = th.SetTECPower(0)
		_ = th.SetFan(false)
		t, _ := th.ReadTemp()
		fmt.Printf("TEC + fan OFF; temp %.2f °C\n", t)
		return
	}
	if *heater >= 0 {
		raw, err := astrocam.OpenHost(vid, uint16(*pid))
		if err != nil {
			log.Fatalf("heater: %v", err)
		}
		defer raw.Close()
		cam, err := astrocam.Open(raw, vid, uint16(*pid))
		if err != nil {
			log.Fatalf("heater: %v", err)
		}
		th := cam.HardwareThermal()
		rm := cam.Rm()
		if err := th.SetHeater(*heater); err != nil {
			log.Fatalf("heater: %v", err)
		}
		rb2a, _ := rm.ReadFPGAReg(0x2a)
		rb19, _ := rm.ReadFPGAReg(0x19)
		fmt.Printf("heater %d%% -> reg0x2a=0x%02x (%d/255), warm-enable (reg0x19 bit6)=%d\n",
			*heater, rb2a, rb2a, (rb19>>6)&1)
		return
	}
	if *cool {
		if err := doCool(vid, uint16(*pid)); err != nil {
			log.Fatalf("cool error: %v", err)
		}
		return
	}
	if *regulate {
		if err := doRegulate(vid, uint16(*pid), *regtarget, *kp, *ki, *kd, *slew, *seed, *maxerr, *regtarget2); err != nil {
			log.Fatalf("regulate error: %v", err)
		}
		return
	}
	if *serial != "" {
		if err := doSerial(*serial); err != nil {
			log.Fatalf("serial error: %v", err)
		}
		return
	}
	if *replay != "" {
		if err := doReplay(vid, uint16(*pid), *replay); err != nil {
			log.Fatalf("replay error: %v", err)
		}
		return
	}
	o := captureOpts{
		exposure: *exposure, gain: *gain, offset: *offset, bin: *bin, roi: *roi,
		raw8: *raw8 || *highspeed, out: *out, timeout: *timeout, nframes: *nframes, usb2: *usb2,
		discard: *discard, highspeed: *highspeed, fpsPerc: *fpsPerc, fixDefects: *fixDefects,
	}
	if *wedge {
		if err := doWedge(vid, uint16(*pid), o, *wedgeIters, *wedgeAntagonist, *wedgeInterval, *wedgeMaxSec); err != nil {
			log.Fatalf("wedge error: %v", err)
		}
		return
	}
	if *soak {
		if err := doSoak(vid, uint16(*pid), o, *soakFrames, *soakVideo); err != nil {
			log.Fatalf("soak error: %v", err)
		}
		return
	}
	if err := run(vid, uint16(*pid), *capture, *verbose, o); err != nil {
		log.Fatalf("error: %v", err)
	}
}

// captureOpts bundles the -capture controls (exposure/gain/offset/bin/roi/depth + output).
type captureOpts struct {
	exposure   time.Duration
	gain       int
	offset     int // -1 = leave the sensor default
	bin        int
	roi        string // "x,y,w,h" in binned pixels, or "" = full binned frame
	raw8       bool
	out        string
	timeout    time.Duration
	nframes    int
	usb2       bool // force the USB2 readout path regardless of model/link
	discard    bool // capture frames via the stream but don't write (pure-capture benchmark)
	highspeed  bool // 10-bit high-speed readout (implies raw8)
	fpsPerc    int  // bandwidth-overload / FPS percent (40..100); 0 = bus default
	fixDefects bool // apply the factory defect map to the frame; opt-in
}

func (o captureOpts) binOr1() int {
	if o.bin < 1 {
		return 1
	}
	return o.bin
}

func run(vid, pid uint16, capture, verbose bool, o captureOpts) error {
	raw, err := astrocam.OpenHost(vid, pid)
	if err != nil {
		return fmt.Errorf("connect %04x:%04x: %w\n(camera plugged in, not claimed by another driver, accessible?)", vid, pid, err)
	}
	defer raw.Close() // always disconnect cleanly

	t0 := time.Now()
	el := func() string { return fmt.Sprintf("[%8.3fs]", time.Since(t0).Seconds()) }

	var t astrocam.Transport = raw
	if verbose {
		t = &logT{t: raw, w: os.Stderr, start: t0}
	}
	cam, err := astrocam.Open(t, vid, pid)
	if err != nil {
		return fmt.Errorf("bind PID 0x%04x: %w", pid, err)
	}
	if o.usb2 {
		cam.SetUSB3(false) // force the USB2 readout path (bwUSB2 bandwidth budget)
		fmt.Println("  link mode: FORCED USB2 (bwUSB2) via -usb2")
	}
	if o.fpsPerc != 0 {
		cam.SetFPSPercent(o.fpsPerc) // bandwidth-overload override (40..100); raises USB2 throughput
		fmt.Printf("  fps percent: %d (bandwidth-overload override)\n", o.fpsPerc)
	}

	fmt.Printf("connected %04x:%04x\n", vid, pid)
	if d, ok := interface{}(raw).(interface{ Describe() string }); ok {
		fmt.Printf("  link     : %s\n", d.Describe())
	}
	info := cam.Info()
	fmt.Printf("  model    : %s\n", cam.Name())
	fmt.Printf("  sensor   : %s\n", cam.Sensor().Name)
	fmt.Printf("  geometry : %d x %d px, %.2f µm pixel, %d-bit\n", info.MaxWidth, info.MaxHeight, info.PixelUm, info.BitDepth)
	fmt.Printf("  color    : %v (bayer %q)\n", cam.Color(), info.Bayer)
	fmt.Printf("  cooled   : %v\n", cam.Cooled())
	if v, err := cam.FirmwareVersion(); err != nil {
		fmt.Printf("  firmware : read failed: %v\n", err)
	} else {
		fmt.Printf("  firmware : 0x%04x\n", v)
	}
	if sn, err := cam.SerialNumber(); err != nil {
		fmt.Printf("  serial   : read failed: %v\n", err)
	} else {
		fmt.Printf("  serial   : %s\n", sn)
	}

	if !capture {
		fmt.Println("disconnecting.")
		return nil
	}

	// first-light test: full init + one exposure + read one frame
	step := func(name string, err error) error {
		if err != nil {
			return fmt.Errorf("%s: %w", name, err)
		}
		fmt.Printf("  %-14s ok\n", name)
		return nil
	}
	fmt.Println("capture:")
	defer cam.StopExposure() // leave the device stopped/clean on exit
	if err := step("Init", cam.Init()); err != nil {
		return err
	}
	// Output depth, then binning, then the ROI window (each affects the next).
	if o.raw8 {
		if err := step("SetOutputDepth(RAW8)", cam.SetOutputDepth(1)); err != nil {
			return err
		}
	}
	if o.highspeed {
		cam.SetHighSpeedMode(true) // 10-bit high-speed; must precede SetROI (sensor format) + SetExposure (HMAX)
		fmt.Println("  high-speed   : 10-bit readout (2× clock)")
	}
	if o.bin > 1 {
		if err := step("SetBinning", cam.SetBinning(o.bin)); err != nil {
			return err
		}
	}
	x, y, w, h := 0, 0, info.MaxWidth/o.binOr1(), info.MaxHeight/o.binOr1()
	if o.roi != "" {
		if _, e := fmt.Sscanf(o.roi, "%d,%d,%d,%d", &x, &y, &w, &h); e != nil {
			return fmt.Errorf("bad -roi %q (want x,y,w,h): %v", o.roi, e)
		}
	}
	if err := step("SetROI", cam.SetROI(x, y, w, h)); err != nil {
		return err
	}
	if err := step("SetGain", cam.SetGain(o.gain)); err != nil {
		return err
	}
	if o.offset >= 0 {
		if err := step("SetOffset", cam.SetOffset(o.offset)); err != nil {
			return err
		}
	}
	fmt.Printf("  offset       : %d (read back from the sensor)\n", cam.Offset())
	if err := step("SetExposure", cam.SetExposure(o.exposure)); err != nil {
		return err
	}
	fmt.Printf("  exposing %s (gain %d, offset %d, bin %d, %dx%d, %d-bit out)...\n",
		o.exposure, o.gain, o.offset, o.binOr1(), w, h, cam.OutputDepth()*8)
	fmt.Printf("%s acquisition START (StartExposure)\n", el())
	if err := step("StartExposure", cam.StartExposure(true)); err != nil {
		return err
	}
	// Burst path: .ser writes a SER video container; -discard captures with no write
	// (pure-capture throughput benchmark). Both arm once and pull from a resident stream.
	isSer := strings.HasSuffix(strings.ToLower(o.out), ".ser")
	if isSer || o.discard {
		bpp := cam.OutputDepth()
		var sw *serWriter
		if !o.discard {
			bayer := ""
			if cam.Color() {
				bayer = info.Bayer
			}
			var serr error
			if sw, serr = newSER(o.out, w, h, bpp, serColorID(cam.Color(), bayer), cam.Name()); serr != nil {
				return serr
			}
		}
		buf := make([]byte, cam.FrameBytes())
		fbytes := cam.FrameBytes()
		nf := o.nframes
		if nf < 1 {
			nf = 1
		}
		count := 0
		t0 := time.Now()
		idle, total := time.Second, o.exposure+2*time.Second
		streamed := false
		var capEnd time.Time // capture-loop end stamp (before teardown), for a fair rate
		// Preferred path: arm once for free-run, then a resident stream session (each frame
		// pulled with Next). Falls back to the single-shot worker if unsupported.
		if verr := cam.StartVideo(true); verr == nil {
			if sess, serr := cam.StartStream(total); serr == nil {
				// Warm-up frame doubles as a free-run probe: DDR cameras (6200/585) can't free-run
				// a resident stream, so a short/empty warm-up means fall back to the single-shot
				// worker loop below. Free-run cameras (290/462/174) return a full frame and stream.
				wn, werr := sess.Next(buf, idle)
				streamed = wn == fbytes
				if !streamed {
					fmt.Printf("%s resident stream probe: %d of %d bytes (err %v); falling back to the single-shot worker\n", el(), wn, fbytes, werr)
				}
				if streamed && o.discard {
					// Pure-capture benchmark: read a frame, drop it. Use the zero-copy path when
					// the backend offers it (no per-frame memcpy), only when a whole frame fits in
					// one transfer (sub-MiB ROI); larger frames span chunks, so fall back to Next.
					zc, hasZC := sess.(astrocam.FrameStreamZC)
					hasZC = hasZC && fbytes <= (1<<20)
					ivs := make([]float64, 0, nf) // per-frame intervals (ms)
					t0 = time.Now()               // time only the steady-state loop
					last := t0
					for f := 0; f < nf; f++ {
						if hasZC {
							fr, cerr := zc.NextZC(idle)
							if cerr != nil || len(fr) != fbytes {
								fmt.Printf("%s frame %d short/err: %d/%d (%v)\n", el(), f, len(fr), fbytes, cerr)
								break
							}
							zc.Release()
						} else {
							cn, cerr := sess.Next(buf, idle)
							if cerr != nil || cn != fbytes {
								fmt.Printf("%s frame %d short/err: %d/%d (%v)\n", el(), f, cn, fbytes, cerr)
								break
							}
						}
						now := time.Now()
						ivs = append(ivs, float64(now.Sub(last).Microseconds())/1000.0)
						last = now
						count++
					}
					// Interval distribution: tight unimodal spread = bandwidth-limited (no drops);
					// a cluster of ~2× intervals = dropped frames at the higher true rate.
					if len(ivs) > 2 {
						s := append([]float64(nil), ivs...)
						sort.Float64s(s)
						med := s[len(s)/2]
						var gaps int
						for _, v := range ivs {
							if v > 1.5*med {
								gaps++
							}
						}
						fmt.Printf("  intervals(ms): med=%.2f p1=%.2f p99=%.2f max=%.2f | >1.5×med (likely drops)=%d/%d\n",
							med, s[len(s)/100], s[len(s)*99/100], s[len(s)-1], gaps, len(ivs))
						// Dump the per-frame interval (ms) of every frame to -out for plotting —
						// only when -out names a plausible text target. The dump is plain text, so
						// never let a benchmark run silently clobber an image path (the default
						// -out frame.fits, or a .ser the user meant for a real capture).
						if o.out != "" && !strings.HasSuffix(strings.ToLower(o.out), ".fits") &&
							!strings.HasSuffix(strings.ToLower(o.out), ".ser") {
							var b strings.Builder
							for _, v := range ivs {
								fmt.Fprintf(&b, "%.3f\n", v)
							}
							if werr := os.WriteFile(o.out, []byte(b.String()), 0o644); werr == nil {
								fmt.Printf("  wrote %d per-frame intervals -> %s\n", len(ivs), o.out)
							}
						} else if o.out != "" {
							fmt.Printf("  (interval dump skipped: -out %q looks like an image path; use a .txt/.csv)\n", o.out)
						}
					}
				} else if streamed {
					// Async double-buffered writer: SER disk I/O runs on its own goroutine,
					// overlapping the next frame's capture so the read loop runs at full rate.
					// Each queued frame carries its capture stamp — the writer drains up to
					// pool frames behind, so stamping at write time would skew the trailer.
					const pool = 4
					type serFrame struct {
						data []byte
						at   time.Time
					}
					free := make(chan []byte, pool)
					queue := make(chan serFrame, pool)
					for i := 0; i < pool; i++ {
						free <- make([]byte, fbytes)
					}
					var writeErr error
					done := make(chan struct{})
					go func() {
						defer close(done)
						for fr := range queue {
							if writeErr == nil {
								writeErr = sw.writeFrame(fr.data, fr.at)
							}
							free <- fr.data[:cap(fr.data)]
						}
					}()
					t0 = time.Now() // time ONLY the steady-state loop, matching the SDK bench
					for f := 0; f < nf; f++ {
						fb := <-free
						cn, cerr := sess.Next(fb, idle)
						if cerr != nil || cn != fbytes {
							fmt.Printf("%s frame %d short/err: %d/%d (%v)\n", el(), f, cn, fbytes, cerr)
							free <- fb
							break
						}
						cam.RepairFrame(fb[:cn]) // FX3 marker corners, as GetDataAfterExp does
						queue <- serFrame{data: fb[:cn], at: time.Now()}
					}
					close(queue)
					<-done
					count = sw.count
					if writeErr != nil {
						sess.Close()
						sw.close()
						return writeErr
					}
				}
				if streamed {
					capEnd = time.Now() // stamp before teardown: sess.Close (abort+join) must not count toward fps
				}
				sess.Close()
			}
		}
		if !streamed {
			// Fallback: single-shot worker, one full arm/expose/read per frame. The path for
			// DDR cameras (6200/585) that can't free-run a resident stream: each frame is a
			// complete arm + windowed read + FPGABufReload.
			t0 = time.Now() // time only the worker loop
			ivs := make([]float64, 0, nf)
			last := t0
			for f := 0; f < nf; f++ {
				if e := cam.StartExposure(true); e != nil {
					fmt.Printf("%s frame %d arm error: %v\n", el(), f, e)
					break
				}
				cn, cerr := cam.GetDataAfterExp(buf)
				if cerr != nil || cn != fbytes {
					fmt.Printf("%s frame %d short/err: %d/%d (%v)\n", el(), f, cn, fbytes, cerr)
					break
				}
				if sw != nil {
					if werr := sw.writeFrame(buf[:cn], time.Now()); werr != nil {
						sw.close()
						return werr
					}
				}
				now := time.Now()
				ivs = append(ivs, float64(now.Sub(last).Microseconds())/1000.0)
				last = now
				count++
			}
			if o.discard && len(ivs) > 2 {
				s := append([]float64(nil), ivs...)
				sort.Float64s(s)
				fmt.Printf("  intervals(ms): med=%.1f min=%.1f max=%.1f (per-frame arm+read)\n",
					s[len(s)/2], s[0], s[len(s)-1])
			}
		}
		dt := time.Since(t0).Seconds()
		if !capEnd.IsZero() {
			dt = capEnd.Sub(t0).Seconds() // exclude stream teardown (sess.Close) from the rate
		}
		if sw != nil {
			if cerr := sw.close(); cerr != nil {
				return cerr
			}
			fmt.Printf("\n*** SER *** %d frames -> %s  %dx%d %d-bit  %.1f fps (%.3fs)\n",
				count, o.out, w, h, bpp*8, float64(count)/dt, dt)
		} else {
			fmt.Printf("\n*** CAPTURE (discard) *** %d frames  %dx%d %d-bit  %.1f fps (%.3fs)\n",
				count, w, h, bpp*8, float64(count)/dt, dt)
		}
		return nil
	}
	// Continuous test (-n N): one arm, then N back-to-back reads (no re-arm). Frame 0
	// at ~2× the exposure with frames 1..N-1 at ~1× means a one-time cold-start throwaway.
	if o.nframes > 1 {
		cbuf := make([]byte, cam.FrameBytes())
		for f := 0; f < o.nframes; f++ {
			t1 := time.Now()
			cn, cerr := cam.ReadFrame(cbuf, false)
			fmt.Printf("%s frame %d: %d bytes in %.3fs (err: %v)\n", el(), f, cn, time.Since(t1).Seconds(), cerr)
		}
		return nil
	}
	fmt.Printf("%s begin readout (read blocks until the frame arrives)\n", el())
	// Read exactly one frame: an over-sized buffer makes the read wait for the next frame
	// to top off the extra bytes, and in long-exposure mode frames aren't back-to-back, so
	// a margin costs a whole extra exposure. FrameBytes ends on frame 1.
	buf := make([]byte, cam.FrameBytes())
	// Bound the read by -timeout. On expiry, abort the exposure and bail — buf is still owned
	// by the read goroutine, so don't touch it after a timeout.
	type readRes struct {
		n   int
		err error
	}
	resCh := make(chan readRes, 1)
	go func() { rn, rerr := cam.GetDataAfterExp(buf); resCh <- readRes{rn, rerr} }()
	var n int
	select {
	case r := <-resCh:
		n, err = r.n, r.err
	case <-time.After(o.timeout):
		_ = cam.StopExposure() // best-effort abort of the in-flight read
		fmt.Printf("%s readout TIMEOUT after %s\n", el(), o.timeout)
		return fmt.Errorf("readout timed out after %s", o.timeout)
	}
	fmt.Printf("%s readout returned %d bytes (err: %v)\n", el(), n, err)
	if err == nil {
		x, y, w, h := cam.ROI()
		step := cam.OutputDepth()
		bayer := ""
		if cam.Color() {
			bayer = info.Bayer
		}
		// Opt-in factory defect correction (off by default). Full-frame RAW16 only.
		if o.fixDefects {
			if step == 2 && x == 0 && y == 0 && w == info.MaxWidth && h == info.MaxHeight {
				if dm, derr := cam.LoadDefectMap(info.MaxWidth, info.MaxHeight); derr != nil {
					fmt.Printf("  -fixdefects: %v (frame left raw)\n", derr)
				} else {
					dm.ApplyRAW16(buf[:n])
					fmt.Printf("  -fixdefects: corrected %d factory defect pixels\n", dm.Count())
				}
			} else {
				fmt.Printf("  -fixdefects: only full-frame RAW16 is supported; frame left raw\n")
			}
		}
		if werr := writeFrameFile(o.out, buf[:n], w, h, step, bayer, info.PixelUm, o.exposure, o.gain, cam.Name()); werr != nil {
			fmt.Printf("  warning: writing %s: %v\n", o.out, werr)
		}
		// Raw pixel stats (min/max/avg/stdev). Pixels are RAW16 LE or RAW8 per OutputDepth.
		mn, mx, cnt, sum, sumsq := 1<<16, 0, 0, 0.0, 0.0
		for i := 0; i+step <= n; i += step {
			v := int(buf[i])
			if step == 2 {
				v |= int(buf[i+1]) << 8
			}
			if v < mn {
				mn = v
			}
			if v > mx {
				mx = v
			}
			sum += float64(v)
			sumsq += float64(v) * float64(v)
			cnt++
		}
		avg, sd := 0.0, 0.0
		if cnt > 0 {
			avg = sum / float64(cnt)
			if va := sumsq/float64(cnt) - avg*avg; va > 0 {
				sd = math.Sqrt(va)
			}
		}
		fmt.Printf("\n*** FRAME *** %d bytes -> %s  ROI %d,%d %dx%d, %d-bit out  pixels: min=%d max=%d avg=%.1f stdev=%.2f\n",
			n, o.out, x, y, w, h, step*8, mn, mx, avg, sd)
		return nil
	}
	// On a timeout/over-read, data may still have flowed: dump what arrived for inspection.
	if n <= 0 {
		return fmt.Errorf("no data on EP 0x81 (sensor not streaming): %w", err)
	}
	fmt.Printf("  got %d bytes (err: %v)\n", n, err)
	fmt.Printf("  first 16 bytes: % x\n", buf[:16])
	rawOut := rawPath(o.out)                 // a partial frame is raw: no valid FITS dims
	_ = os.WriteFile(rawOut, buf[:n], 0o644) // dump the raw stream for inspection
	fmt.Printf("%s wrote %s (%d bytes; frame size %d)\n", el(), rawOut, n, cam.FrameBytes())
	return nil
}

// writeFrameFile saves a completed capture at its actual ROI dims (w×h): FITS if the path ends
// in .fits/.fit (viewable directly in any astronomy tool), otherwise the raw little-endian bytes.
// FITS here is 16-bit (BITPIX 16); a RAW8 capture is written raw with a note.
func writeFrameFile(path string, data []byte, w, h, bpp int, bayer string, pixelUm float64, exposure time.Duration, gain int, model string) error {
	l := strings.ToLower(path)
	if strings.HasSuffix(l, ".fits") || strings.HasSuffix(l, ".fit") {
		return writeFITS(path, data, w, h, bpp, bayer, exposure.Seconds(), pixelUm, gain, model)
	}
	return os.WriteFile(path, data, 0o644)
}

// rawPath swaps a .fits/.fit extension for .raw (partial/debug dumps are always raw).
func rawPath(p string) string {
	l := strings.ToLower(p)
	switch {
	case strings.HasSuffix(l, ".fits"):
		return p[:len(p)-5] + ".raw"
	case strings.HasSuffix(l, ".fit"):
		return p[:len(p)-4] + ".raw"
	}
	return p
}

// doList enumerates attached ZWO cameras and reads each one's factory serial.
func doList() error {
	devs, err := astrocam.Enumerate()
	if err != nil {
		return err
	}
	if len(devs) == 0 {
		fmt.Println("no cameras found")
		return nil
	}
	for _, d := range devs {
		line := d.String()
		t, err := astrocam.OpenLocation(d.VID, d.Location)
		if err != nil {
			fmt.Printf("%s  serial=?(open failed: %v)\n", line, err)
			continue
		}
		if cam, err := astrocam.Open(t, d.VID, d.PID); err == nil {
			if sn, err := cam.SerialNumber(); err == nil {
				line += "  serial=" + sn.String()
			} else {
				line += fmt.Sprintf("  serial=?(%v)", err)
			}
		}
		t.Close()
		fmt.Println(line)
	}
	return nil
}

// doSerial opens the camera by factory serial and prints its identity.
func doSerial(serial string) error {
	t, d, err := astrocam.OpenSerial(serial)
	if err != nil {
		return err
	}
	defer t.Close()
	fmt.Printf("opened %s\n", d)
	cam, err := astrocam.Open(t, d.VID, d.PID)
	if err != nil {
		return err
	}
	fmt.Printf("  model  : %s\n", cam.Name())
	if sn, err := cam.SerialNumber(); err == nil {
		fmt.Printf("  serial : %s\n", sn)
	}
	return nil
}

// doDumpFlash reads the factory defect-map blob from SPI flash (FlashHPCMapAddr) to a file.
// Layout: 2 KiB header with magic "ASID" (defect) / "ASIG" (gain) + big-endian uint32 payload
// length at offset 4, then the compressed 1-bit-per-pixel defect bitmap.
func doDumpFlash(vid, pid uint16, path string) error {
	raw, err := astrocam.OpenHost(vid, pid)
	if err != nil {
		return err
	}
	defer raw.Close()
	cam, err := astrocam.Open(raw, vid, pid)
	if err != nil {
		return err
	}
	fmt.Printf("connected %04x:%04x  %s\n", vid, pid, cam.Name())

	// The flash read needs the firmware + GPIF up, so Init first.
	if err := cam.Init(); err != nil {
		return fmt.Errorf("init: %w", err)
	}
	head, err := cam.ReadSPIFlash(astrocam.FlashHPCMapAddr, 2048)
	if err != nil {
		return fmt.Errorf("read header: %w", err)
	}
	if len(head) < 16 {
		return fmt.Errorf("short header read (%d B)", len(head))
	}
	magic := string(head[:4])
	length := binary.BigEndian.Uint32(head[4:8])
	fmt.Printf("flash @0x%05x: magic=%q  payload_len=%d (0x%x)  next8=% x\n",
		astrocam.FlashHPCMapAddr, magic, length, length, head[8:16])
	if magic != "ASID" && magic != "ASIG" {
		fmt.Println("  NOTE: no ASID/ASIG header — flash region may be empty/erased or a different layout")
	}
	if length == 0 || length > 0x30000 {
		return fmt.Errorf("implausible payload length %d", length)
	}
	total := int((length + 2047) &^ 2047)
	blob, err := cam.ReadSPIFlash(astrocam.FlashHPCMapAddr, total)
	if err != nil {
		return fmt.Errorf("read blob (%d B): %w", total, err)
	}
	if err := os.WriteFile(path, blob, 0o644); err != nil {
		return err
	}
	fmt.Printf("wrote %d bytes -> %s\n", len(blob), path)
	return nil
}

// doThermal reads temperature + humidity via the hardware Thermal backend and dumps the
// raw 0xB3 temp bytes. Read-only: it does not drive the TEC, fan, or heater.
func doThermal(vid, pid uint16) error {
	raw, err := astrocam.OpenHost(vid, pid)
	if err != nil {
		return err
	}
	defer raw.Close()
	cam, err := astrocam.Open(raw, vid, pid)
	if err != nil {
		return err
	}
	fmt.Printf("connected %04x:%04x  %s  cooled=%v\n", vid, pid, cam.Name(), cam.Cooled())

	// Raw 0xB3 bytes, with both candidate decodings.
	var b [2]byte
	if _, err := raw.ControlIn(0xB3, 0, 0, b[:]); err != nil {
		return fmt.Errorf("read temp (0xB3): %w", err)
	}
	fmt.Printf("  raw 0xB3 : % x   (lo=%d hi=%d)\n", b, b[0], b[1])
	fmt.Printf("    int8(hi)+lo/16 : %.3f °C\n", float64(int8(b[1]))+float64(b[0])/16.0)
	s12 := int(b[1])<<4 | int(b[0])>>4
	if s12 >= 0x800 {
		s12 -= 0x1000
	}
	fmt.Printf("    signed12×0.0625: %.3f °C\n", float64(s12)*0.0625)

	th := cam.HardwareThermal()
	if t, err := th.ReadTemp(); err != nil {
		fmt.Printf("  ReadTemp : error: %v\n", err)
	} else {
		fmt.Printf("  ReadTemp : %.2f °C\n", t)
	}
	if rh, err := th.ReadHumidity(); err != nil {
		fmt.Printf("  Humidity : error: %v\n", err)
	} else {
		fmt.Printf("  Humidity : %d %%\n", rh)
	}
	return nil
}

// doCool actuates the TEC on a cooled camera: an open-loop power-step sweep then a
// closed-loop regulation run via the Cooler goroutine. Always returns the TEC to 0 and
// the fan off on exit (Ctrl-C included).
func doCool(vid, pid uint16) error {
	raw, err := astrocam.OpenHost(vid, pid)
	if err != nil {
		return err
	}
	defer raw.Close()
	cam, err := astrocam.Open(raw, vid, pid)
	if err != nil {
		return err
	}
	if !cam.Cooled() {
		return fmt.Errorf("%s has no cooler", cam.Name())
	}
	// Init first: cooling control on an uninitialized FX3/FPGA wedges control transfers.
	if err := cam.Init(); err != nil {
		return fmt.Errorf("init: %w", err)
	}
	defer cam.StopExposure() // leave the pipeline quiescent on exit
	th := cam.HardwareThermal()
	rm := cam.Rm()
	t0 := time.Now()
	el := func() string { return fmt.Sprintf("[%5.1fs]", time.Since(t0).Seconds()) }
	temp := func() float64 {
		v, err := th.ReadTemp()
		if err != nil {
			fmt.Printf("%s    (ReadTemp error: %v)\n", el(), err)
		}
		return v
	}

	// Safety: always drive the TEC to 0 / fan off on the way out, Ctrl-C included.
	stop := func() { _ = th.SetTECPower(0); _ = th.SetFan(false) }
	defer stop()
	sigc := make(chan os.Signal, 1)
	signal.Notify(sigc, os.Interrupt, syscall.SIGTERM)
	go func() { <-sigc; fmt.Println("\ninterrupted — TEC -> 0"); stop(); os.Exit(1) }()

	base := temp()
	fmt.Printf("%s %s  baseline %.2f °C (cooler off)\n", el(), cam.Name(), base)

	fmt.Println("Phase 1 — open-loop power steps (fan auto-on with power):")
	for _, p := range []float64{20, 40, 60} {
		if err := th.SetTECPower(p); err != nil {
			return err
		}
		rb, _ := rm.ReadFPGAReg(0x26)
		fmt.Printf("%s  set %.0f%%  (reg0x26 readback = 0x%02x)\n", el(), p, rb)
		for i := 0; i < 4; i++ {
			time.Sleep(4 * time.Second)
			tc := temp()
			fmt.Printf("%s    %.2f °C  (Δ%+.2f)\n", el(), tc, tc-base)
		}
	}
	stop()
	fmt.Printf("%s  power 0, fan off — recovering 8s\n", el())
	time.Sleep(8 * time.Second)
	fmt.Printf("%s  recovered to %.2f °C\n", el(), temp())

	target := base - 10
	fmt.Printf("Phase 2 — closed-loop regulation to %.1f °C (Cooler goroutine):\n", target)
	cfg := astrocam.DefaultCoolerConfig()
	cfg.RampRate = 30 // °C/min setpoint ramp
	if err := cam.EnableCooling(nil, target, cfg); err != nil {
		return err
	}
	for i := 0; i < 30; i++ { // ~60 s
		time.Sleep(2 * time.Second)
		final, eff, _ := cam.TargetTemp()
		tc, _ := cam.Temperature()
		fmt.Printf("%s  temp %.2f °C  setpt %.2f→%.1f  power %.0f%%\n", el(), tc, eff, final, cam.CoolerPower())
	}
	cam.DisableCooling()
	fmt.Printf("%s  cooling disabled; returning TEC to 0\n", el())
	return nil
}

// doRegulate actuates the TEC in closed loop to target °C and watches it converge and
// hold. Returns the TEC to 0 / fan off on exit (Ctrl-C included).
func doRegulate(vid, pid uint16, target, kp, ki, kd, slew, seed, maxerr, target2 float64) error {
	raw, err := astrocam.OpenHost(vid, pid)
	if err != nil {
		return err
	}
	defer raw.Close()
	cam, err := astrocam.Open(raw, vid, pid)
	if err != nil {
		return err
	}
	if !cam.Cooled() {
		return fmt.Errorf("%s has no cooler", cam.Name())
	}
	if err := cam.Init(); err != nil {
		return fmt.Errorf("init: %w", err)
	}
	defer cam.StopExposure()
	th := cam.HardwareThermal()
	t0 := time.Now()
	el := func() string { return fmt.Sprintf("[%5.1fs]", time.Since(t0).Seconds()) }

	stop := func() { _ = th.SetTECPower(0); _ = th.SetFan(false) }
	defer stop()
	sigc := make(chan os.Signal, 1)
	signal.Notify(sigc, os.Interrupt, syscall.SIGTERM)
	go func() { <-sigc; fmt.Println("\ninterrupted — TEC -> 0"); stop(); os.Exit(1) }()

	base, _ := cam.Temperature()
	fmt.Printf("%s %s  baseline %.2f °C  (Kp=%.3g Ki=%.3g Kd=%.3g slew=%.3g seed=%.3g maxerr=%.3g)\n",
		el(), cam.Name(), base, kp, ki, kd, slew, seed, maxerr)

	cfg := astrocam.DefaultCoolerConfig()
	cfg.Kp, cfg.Ki, cfg.Kd = kp, ki, kd
	cfg.SlewPerStep = slew // 0 = per-tick rate limit disabled
	cfg.RampRate = 0       // no setpoint ramp; MaxError paces the approach
	cfg.MaxError = maxerr  // symmetric error clamp (°C)
	if err := cam.EnableCooling(nil, target, cfg); err != nil {
		return err
	}
	if seed > 0 {
		cam.SeedCoolerPower(seed)
	}

	// runPhase drives the loop to tgt and reports until it holds ±0.3 °C or hits the limit.
	runPhase := func(tgt float64, maxIter int) {
		cam.SetTargetTemp(tgt)
		fmt.Printf("%s ===> target %.1f °C\n", el(), tgt)
		held := 0
		for i := 0; i < maxIter; i++ {
			time.Sleep(2 * time.Second)
			temp, _ := cam.Temperature()
			e := temp - tgt
			eff := e // the error the loop actually acts on, after the symmetric clamp
			if maxerr > 0 {
				if eff > maxerr {
					eff = maxerr
				} else if eff < -maxerr {
					eff = -maxerr
				}
			}
			mark := ""
			if e < 0.3 && e > -0.3 {
				held++
				mark = "  <= at setpoint"
			} else {
				held = 0
			}
			fmt.Printf("%s temp %.2f °C  power %.0f%%  (err %+.2f, loop acts on %+.2f)%s\n",
				el(), temp, cam.CoolerPower(), e, eff, mark)
			if held >= 8 { // ~16 s within ±0.3 °C
				fmt.Printf("%s settled at %.1f °C.\n", el(), tgt)
				return
			}
		}
		fmt.Printf("%s phase time limit reached.\n", el())
	}

	runPhase(target, 150)
	if !math.IsNaN(target2) {
		runPhase(target2, 150) // second leg, e.g. ramp back up (tests the clamp on warmup)
	}

	cam.DisableCooling()
	fmt.Printf("%s cooling disabled; returning TEC to 0\n", el())
	return nil
}

// doReplay sends a captured SDK 'req reg val' (hex) write sequence verbatim, then reads
// one frame: a ground-truth test that the transport can drive the camera.
func doReplay(vid, pid uint16, file string) error {
	data, err := os.ReadFile(file)
	if err != nil {
		return err
	}
	t, err := astrocam.OpenHost(vid, pid)
	if err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer t.Close()

	n := 0
	for _, ln := range strings.Split(string(data), "\n") {
		f := strings.Fields(ln)
		if len(f) < 3 {
			continue
		}
		req, _ := strconv.ParseUint(f[0], 16, 8)
		reg, _ := strconv.ParseUint(f[1], 16, 16)
		val, _ := strconv.ParseUint(f[2], 16, 16)
		if err := t.ControlOut(uint8(req), uint16(reg), uint16(val), nil); err != nil {
			return fmt.Errorf("replay write #%d (req=%s reg=%s val=%s): %w", n, f[0], f[1], f[2], err)
		}
		n++
	}
	fmt.Printf("replayed %d SDK writes; reading frame...\n", n)
	if r, ok := interface{}(t).(astrocam.EndpointResetter); ok {
		r.ResetEndpoint(0x81)
	}
	buf := make([]byte, 4708352) // exactly one frame (no over-read)
	t0 := time.Now()
	nb, rerr := t.BulkRead(buf, 12*time.Second)
	fmt.Printf("read took %.3fs\n", time.Since(t0).Seconds())
	nz := 0
	for _, b := range buf[:nb] {
		if b != 0 {
			nz++
		}
	}
	fmt.Printf("bulk read: %d bytes (err=%v), nonzero=%d\n", nb, rerr, nz)
	if nb >= 16 {
		fmt.Printf("first 16 bytes: % x\n", buf[:16])
	}
	if nb > 0 {
		os.WriteFile("frame_replay.raw", buf[:nb], 0o644)
		fmt.Println("wrote frame_replay.raw")
	}
	return nil
}

// doSoak runs a long single-threaded continuous capture (no antagonist) and reports the
// worker's cumulative StallCount, testing whether soft stalls recur absent concurrency.
// Honors -exposure/-gain/-usb2/-fps.
func doSoak(vid, pid uint16, o captureOpts, frames int, video bool) error {
	raw, err := astrocam.OpenHost(vid, pid)
	if err != nil {
		return fmt.Errorf("connect %04x:%04x: %w", vid, pid, err)
	}
	defer raw.Close()
	cam, err := astrocam.Open(raw, vid, pid)
	if err != nil {
		return err
	}
	if o.usb2 {
		cam.SetUSB3(false)
	}
	if o.fpsPerc != 0 {
		cam.SetFPSPercent(o.fpsPerc)
	}
	if err := cam.Init(); err != nil {
		return fmt.Errorf("init: %w", err)
	}
	defer cam.StopExposure()
	info := cam.Info()
	if err := cam.SetROI(0, 0, info.MaxWidth, info.MaxHeight); err != nil {
		return err
	}
	_ = cam.SetGain(o.gain)
	if err := cam.SetExposure(o.exposure); err != nil {
		return err
	}
	fb := cam.FrameBytes()
	buf := make([]byte, fb)
	t0 := time.Now()
	el := func() string { return fmt.Sprintf("[%6.1fs]", time.Since(t0).Seconds()) }
	// Two paths: single-shot arms every frame (StartExposure + GetDataAfterExp); video/free-run
	// arms once (StartVideo) then reads back-to-back (ReadFrame, no re-arm).
	mode := "single-shot (arm every frame)"
	arm := func() error { return cam.StartExposure(true) }
	read := func() (int, error) { return cam.GetDataAfterExp(buf) }
	if video {
		if err := cam.StartVideo(true); err != nil {
			return fmt.Errorf("start video: %w", err)
		}
		mode = "free-run (arm once, StartVideo + ReadFrame)"
		arm = func() error { return nil } // armed once, up front
		read = func() (int, error) { return cam.ReadFrame(buf, false) }
	}
	fmt.Printf("soak: exp %s, %d B/frame, %d frames (USB2=%v, fps=%d) — %s, counting stalls\n",
		o.exposure, fb, frames, o.usb2, o.fpsPerc, mode)

	fails := 0
	var prevStalls int64
	for i := 0; i < frames; i++ {
		if err := arm(); err != nil {
			fails++
			fmt.Printf("%s frame %d arm error: %v [stalls=%d]\n", el(), i, err, cam.StallCount())
			continue
		}
		n, err := read()
		st := cam.StallCount()
		switch {
		case err != nil || n < fb:
			fails++
			fmt.Printf("%s frame %d FAILED: %d/%d B (err %v) [stalls=%d]\n", el(), i, n, fb, err, st)
		case st != prevStalls:
			fmt.Printf("%s frame %d ok but STALLED+recovered (stalls now %d)\n", el(), i, st)
		case i%200 == 0:
			fmt.Printf("%s frame %d ok [stalls=%d]\n", el(), i, st)
		}
		prevStalls = st
	}
	dt := time.Since(t0).Seconds()
	fmt.Printf("\n*** SOAK *** %d frames, %d failed, %d stalls, %.1f fps (%.0fs)\n",
		frames, fails, cam.StallCount(), float64(frames)/dt, dt)
	if cam.StallCount() == 0 && fails == 0 {
		fmt.Println("  clean: no stalls, no failures — soft stalls did not recur single-threaded.")
	}
	return nil
}

// doWedge reproduces the USB2 readout wedge on demand to prove its cause (a control transfer
// interleaving with the in-flight bulk frame read) and verify a transport fix.
//
// The wedge: a concurrent ControlIn(0xB3) (the goastro telemetry poll's transfer) landing
// mid-readout parks the GPIF; every capture then fails until a hard reset. doWedge runs
// continuous full-frame USB2 captures while an antagonist goroutine fires ControlIn(0xB3) on
// a tight interval until a frame fails, confirms the wedge is sticky, then recovers via
// ResetDevice + full re-Init. -wedgeantagonist=false is the A/B control (no antagonist).
func doWedge(vid, pid uint16, o captureOpts, iters int, antagonist bool, intervalMs, maxSec int) error {
	raw, err := astrocam.OpenHost(vid, pid)
	if err != nil {
		return fmt.Errorf("connect %04x:%04x: %w", vid, pid, err)
	}
	defer raw.Close()
	cam, err := astrocam.Open(raw, vid, pid)
	if err != nil {
		return err
	}
	cam.SetUSB3(false)     // force the USB2 readout path (the un-buffered, wedge-prone one)
	cam.SetFPSPercent(100) // max throughput = peak bus pressure during the read
	if err := cam.Init(); err != nil {
		return fmt.Errorf("init: %w", err)
	}
	defer cam.StopExposure()
	info := cam.Info()
	if err := cam.SetROI(0, 0, info.MaxWidth, info.MaxHeight); err != nil {
		return err
	}
	_ = cam.SetGain(o.gain)
	exp := o.exposure
	if err := cam.SetExposure(exp); err != nil {
		return err
	}
	fb := cam.FrameBytes()
	buf := make([]byte, fb)
	if intervalMs < 1 {
		intervalMs = 1
	}
	if maxSec < 1 {
		maxSec = 90
	}
	t0 := time.Now()
	el := func() string { return fmt.Sprintf("[%6.1fs]", time.Since(t0).Seconds()) }
	fmt.Printf("wedge: USB2, fps100, exp %s, %d B/frame, antagonist=%v (0xB3 every %dms), cap %ds / %d frames\n",
		exp, fb, antagonist, intervalMs, maxSec, iters)

	var curFrame, fired int64
	var wedgeMu sync.Mutex
	var wedgeReason string // first detector (antagonist or read) wins
	markWedge := func(reason string) {
		wedgeMu.Lock()
		if wedgeReason == "" {
			wedgeReason = reason
			fmt.Printf("%s *** WEDGE DETECTED: %s\n", el(), reason)
		}
		wedgeMu.Unlock()
	}
	isWedged := func() bool {
		wedgeMu.Lock()
		defer wedgeMu.Unlock()
		return wedgeReason != ""
	}

	stop := make(chan struct{})
	var wg sync.WaitGroup

	// Heartbeat: 2 s tick reporting current frame + antagonist transfer count, so the run is
	// never silent even while blocked inside a wedged frame.
	wg.Add(1)
	go func() {
		defer wg.Done()
		tk := time.NewTicker(2 * time.Second)
		defer tk.Stop()
		for {
			select {
			case <-stop:
				return
			case <-tk.C:
				fmt.Printf("%s …working: frame=%d antagonist_xfers=%d wedged=%v\n",
					el(), atomic.LoadInt64(&curFrame), atomic.LoadInt64(&fired), isWedged())
			}
		}
	}()

	// Antagonist: fire ControlIn(0xB3) into the readout window. On its first failure it has
	// found the wedge (EP0 stopped ACKing) so it records that and stops.
	if antagonist {
		wg.Add(1)
		go func() {
			defer wg.Done()
			var b [2]byte
			tk := time.NewTicker(time.Duration(intervalMs) * time.Millisecond)
			defer tk.Stop()
			for {
				select {
				case <-stop:
					return
				case <-tk.C:
					if _, err := raw.ControlIn(0xB3, 0, 0, b[:]); err != nil {
						markWedge(fmt.Sprintf("antagonist 0xB3 control transfer failed: %v", err))
						return
					}
					atomic.AddInt64(&fired, 1)
				}
			}
		}()
	}

	// captureBounded runs one frame under a watchdog: on timeout it asks the worker to abort
	// (StopExposure), then force-returns the device with a USB reset (ResetDevice, an ioctl, works
	// even when EP0 is timing out). The frame goroutine sends to a buffered channel so a late
	// completion after we've given up never blocks or leaks.
	type res struct {
		n   int
		err error
	}
	captureBounded := func(timeout time.Duration) (int, error) {
		if err := cam.StartExposure(true); err != nil {
			return 0, err
		}
		done := make(chan res, 1)
		go func() { n, err := cam.GetDataAfterExp(buf); done <- res{n, err} }()
		select {
		case r := <-done:
			return r.n, r.err
		case <-time.After(timeout):
		}
		go cam.StopExposure() // gentle unblock (may itself be slow on a dead EP0)
		select {
		case r := <-done:
			return r.n, r.err
		case <-time.After(3 * time.Second):
		}
		_ = cam.ResetDevice() // forcible: makes the stuck read/control error out
		select {
		case r := <-done:
			return r.n, r.err
		case <-time.After(3 * time.Second):
			return 0, fmt.Errorf("frame stuck > %s even after StopExposure+ResetDevice", timeout)
		}
	}

	frameTimeout := 3 * time.Second // generous vs a healthy ~234 ms USB2 frame
	deadline := time.Now().Add(time.Duration(maxSec) * time.Second)

	failedAt, fN := -1, 0
	var fErr error
	for i := 0; i < iters; i++ {
		atomic.StoreInt64(&curFrame, int64(i))
		if isWedged() { // antagonist found it between frames
			failedAt, fErr = i, fmt.Errorf("%s", wedgeReason)
			break
		}
		if time.Now().After(deadline) {
			fmt.Printf("%s reached %ds cap with no wedge.\n", el(), maxSec)
			break
		}
		n, err := captureBounded(frameTimeout)
		if err != nil || n < fb {
			markWedge(fmt.Sprintf("frame %d read failed: %d/%d B (%v)", i, n, fb, err))
			failedAt, fN, fErr = i, n, err
			break
		}
		if i%100 == 0 {
			fmt.Printf("%s frame %d ok (%d B, %d antagonist xfers)\n", el(), i, n, atomic.LoadInt64(&fired))
		}
	}

	// Stop the helpers (bounded: an in-flight 0xB3 returns within the driver's 500 ms ctrl timeout).
	close(stop)
	wg.Wait()

	if failedAt < 0 {
		fmt.Printf("%s NO wedge (%d antagonist xfers).\n", el(), atomic.LoadInt64(&fired))
		if antagonist {
			fmt.Println("  Try -wedgeinterval 1 -exposure 2ms for more readout overlap.")
		} else {
			fmt.Println("  (antagonist off — single-threaded stress did not wedge, as expected.)")
		}
		return nil
	}
	fmt.Printf("%s WEDGED at frame %d: %d/%d B (err %v) after %d antagonist xfers\n",
		el(), failedAt, fN, fb, fErr, atomic.LoadInt64(&fired))

	// Sticky or transient? Probe once more with the antagonist stopped and NO reset: a parked
	// GPIF fails again (sticky wedge); a one-frame stall reads a full frame.
	if fw, ferr := cam.FirmwareVersion(); ferr != nil {
		fmt.Printf("  EP0 after failure: firmware read failed (%v) -> control plane down\n", ferr)
	} else {
		fmt.Printf("  EP0 after failure: firmware 0x%04x readable -> control plane alive\n", fw)
	}
	if pn, perr := captureBounded(5 * time.Second); perr == nil && pn >= fb {
		fmt.Printf("  next frame WITHOUT reset: %d/%d B -> TRANSIENT stall (readout recovered by itself)\n", pn, fb)
	} else {
		fmt.Printf("  next frame WITHOUT reset: %d/%d B (err %v) -> STICKY wedge\n", pn, fb, perr)
	}

	// Recover via whole-device reset + full re-Init. ResetDevice may already have fired inside
	// captureBounded; this is idempotent.
	fmt.Println("  recovering via ResetDevice + Init...")
	_ = cam.ResetDevice()
	time.Sleep(300 * time.Millisecond)
	if err := cam.Init(); err != nil {
		return fmt.Errorf("re-init after reset: %w", err)
	}
	_ = cam.SetROI(0, 0, info.MaxWidth, info.MaxHeight)
	_ = cam.SetExposure(exp)
	n, err := captureBounded(5 * time.Second)
	fmt.Printf("  post-recovery probe: %d/%d B (err %v) -> recovered=%v\n", n, fb, err, err == nil && n >= fb)
	return nil
}

// logT wraps a Transport and logs every transfer (the -v debug path). It forwards EVERY optional
// capability the driver dispatches on (FrameStreamer, PrequeuedFrameStreamer, QuietBulkReader,
// ReadAborter, StreamStarter, EndpointResetter, DeviceResetter, UngatedControlSender, SuperSpeed,
// Describe), so a -v run takes the same code paths as a plain run; where the wrapped transport
// lacks one, logT applies the same fallback the Camera would.
type logT struct {
	t     astrocam.Transport
	w     io.Writer
	start time.Time
}

func (l *logT) ts() string { return fmt.Sprintf("[%8.3fs]", time.Since(l.start).Seconds()) }

func (l *logT) ControlOut(b uint8, wv, wi uint16, d []byte) error {
	err := l.t.ControlOut(b, wv, wi, d)
	fmt.Fprintf(l.w, "%s OUT  req=0x%02x val=0x%04x idx=0x%04x len=%d%s\n", l.ts(), b, wv, wi, len(d), res(err))
	return err
}

func (l *logT) ControlIn(b uint8, wv, wi uint16, d []byte) (int, error) {
	n, err := l.t.ControlIn(b, wv, wi, d)
	dump := ""
	if n > 0 {
		m := n
		if m > 8 {
			m = 8
		}
		dump = fmt.Sprintf(" data=% x", d[:m])
	}
	fmt.Fprintf(l.w, "%s IN   req=0x%02x val=0x%04x idx=0x%04x len=%d -> %d%s%s\n", l.ts(), b, wv, wi, len(d), n, dump, res(err))
	return n, err
}

// ControlOutUngated forwards the ST4 pulse path (UngatedControlSender); falls back to the gated
// ControlOut like the Camera does.
func (l *logT) ControlOutUngated(b uint8, wv, wi uint16) error {
	var err error
	if u, ok := l.t.(astrocam.UngatedControlSender); ok {
		err = u.ControlOutUngated(b, wv, wi)
	} else {
		err = l.t.ControlOut(b, wv, wi, nil)
	}
	fmt.Fprintf(l.w, "%s OUT* req=0x%02x val=0x%04x idx=0x%04x (ungated)%s\n", l.ts(), b, wv, wi, res(err))
	return err
}

func (l *logT) BulkRead(buf []byte, to time.Duration) (int, error) {
	fmt.Fprintf(l.w, "%s BULK read<=%d timeout=%s start...\n", l.ts(), len(buf), to)
	n, err := l.t.BulkRead(buf, to)
	fmt.Fprintf(l.w, "%s BULK <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

// BulkReadQuiet forwards QuietBulkReader; falls back to BulkRead like the Camera.
func (l *logT) BulkReadQuiet(buf []byte, quiet, to time.Duration) (int, error) {
	q, ok := l.t.(astrocam.QuietBulkReader)
	if !ok {
		return l.BulkRead(buf, to)
	}
	fmt.Fprintf(l.w, "%s BULKQ read<=%d quiet=%s timeout=%s start...\n", l.ts(), len(buf), quiet, to)
	n, err := q.BulkReadQuiet(buf, quiet, to)
	fmt.Fprintf(l.w, "%s BULKQ <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

// ReadFrameStream forwards the windowed-stream read (FrameStreamer); falls back to BulkRead.
func (l *logT) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	fs, ok := l.t.(astrocam.FrameStreamer)
	if !ok {
		return l.BulkRead(buf, total)
	}
	fmt.Fprintf(l.w, "%s STREAM read<=%d idle=%s total=%s start...\n", l.ts(), len(buf), idle, total)
	n, err := fs.ReadFrameStream(buf, idle, total)
	fmt.Fprintf(l.w, "%s STREAM <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

// ReadFrameStreamPrequeued forwards the pre-queued batch read (PrequeuedFrameStreamer); falls back
// to ReadFrameStream like the Camera.
func (l *logT) ReadFrameStreamPrequeued(buf []byte, idle, total time.Duration) (int, error) {
	p, ok := l.t.(astrocam.PrequeuedFrameStreamer)
	if !ok {
		return l.ReadFrameStream(buf, idle, total)
	}
	fmt.Fprintf(l.w, "%s PREQ read<=%d idle=%s total=%s start...\n", l.ts(), len(buf), idle, total)
	n, err := p.ReadFrameStreamPrequeued(buf, idle, total)
	fmt.Fprintf(l.w, "%s PREQ <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

// AbortRead / ArmRead forward ReadAborter (no-ops when the wrapped transport lacks it).
func (l *logT) AbortRead() {
	if ra, ok := l.t.(astrocam.ReadAborter); ok {
		ra.AbortRead()
	}
	fmt.Fprintf(l.w, "%s ABORT-READ\n", l.ts())
}

func (l *logT) ArmRead() {
	if ra, ok := l.t.(astrocam.ReadAborter); ok {
		ra.ArmRead()
	}
	fmt.Fprintf(l.w, "%s ARM-READ\n", l.ts())
}

// StartStream forwards the resident stream session (StreamStarter); the session's per-frame
// reads are not logged (a burst would drown the log), only its open and close.
func (l *logT) StartStream(frameBytes int, total time.Duration) (astrocam.FrameStream, error) {
	ss, ok := l.t.(astrocam.StreamStarter)
	if !ok {
		return nil, fmt.Errorf("transport has no resident stream session")
	}
	sess, err := ss.StartStream(frameBytes, total)
	fmt.Fprintf(l.w, "%s STREAM-SESSION open frame=%d total=%s%s\n", l.ts(), frameBytes, total, res(err))
	if err != nil {
		return nil, err
	}
	return &logStream{FrameStream: sess, l: l}, nil
}

// logStream logs a resident session's close; frames pass straight through (including the
// zero-copy path when the underlying session offers it).
type logStream struct {
	astrocam.FrameStream
	l *logT
}

func (s *logStream) Close() error {
	err := s.FrameStream.Close()
	fmt.Fprintf(s.l.w, "%s STREAM-SESSION close%s\n", s.l.ts(), res(err))
	return err
}

func (s *logStream) NextZC(idle time.Duration) ([]byte, error) {
	if zc, ok := s.FrameStream.(astrocam.FrameStreamZC); ok {
		return zc.NextZC(idle)
	}
	return nil, fmt.Errorf("stream session has no zero-copy path")
}

func (s *logStream) Release() {
	if zc, ok := s.FrameStream.(astrocam.FrameStreamZC); ok {
		zc.Release()
	}
}

func (l *logT) Close() error { return l.t.Close() }

// SuperSpeed forwards the optional link-speed report under -v.
func (l *logT) SuperSpeed() bool {
	if sr, ok := l.t.(interface{ SuperSpeed() bool }); ok {
		return sr.SuperSpeed()
	}
	return false
}

// Describe forwards the transport's bring-up description.
func (l *logT) Describe() string {
	if d, ok := l.t.(interface{ Describe() string }); ok {
		return d.Describe()
	}
	return "(no description)"
}

// ResetEndpoint forwards the optional pipe-flush so it still runs under -v.
func (l *logT) ResetEndpoint(ep uint8) error {
	r, ok := l.t.(astrocam.EndpointResetter)
	if !ok {
		return nil
	}
	err := r.ResetEndpoint(ep)
	fmt.Fprintf(l.w, "%s RESET ep=0x%02x%s\n", l.ts(), ep, res(err))
	return err
}

// ResetDevice forwards the whole-device reset (DeviceResetter); errors like the Camera does when
// the wrapped transport has none.
func (l *logT) ResetDevice() error {
	r, ok := l.t.(astrocam.DeviceResetter)
	if !ok {
		return fmt.Errorf("transport has no device reset")
	}
	err := r.ResetDevice()
	fmt.Fprintf(l.w, "%s RESET-DEVICE%s\n", l.ts(), res(err))
	return err
}

func res(err error) string {
	if err != nil {
		return "  ERR: " + err.Error()
	}
	return ""
}
