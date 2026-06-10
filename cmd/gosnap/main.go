// Command gosnap is the bring-up tool for the pure-Go ZWO camera driver.
//
// Default mode detects the camera, connects, prints its identity, and disconnects
// (control transfers only). With -capture it additionally runs the full init +
// single exposure and tries to read one frame — the first-light test. With -v it
// logs every USB transfer on the wire, so an init/arm sequence can be debugged
// against the real device.
//
// Usage:
//
//	gosnap [-pid 0x1749] [-capture] [-v] [-exposure 100ms] [-gain 200] [-out frame.raw]
//
// macOS uses IOUSBHost; Linux usbfs (needs udev access to VID 0x03C3); Windows WinUSB.
package main

import (
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
	"syscall"
	"time"

	"github.com/mikefsq/astrocam"
	_ "github.com/mikefsq/astrocam/sensors" // registers the PID -> sensor profile table
)

func main() {
	log.SetFlags(0)
	pid := flag.Uint("pid", 0x1749, "USB product id (default 0x1749 = ASI174MM Mini)")
	capture := flag.Bool("capture", false, "run full init + one exposure and read a frame (first-light test)")
	verbose := flag.Bool("v", false, "log every USB transfer on the wire")
	exposure := flag.Duration("exposure", 50*time.Millisecond, "exposure time (with -capture). Per-sensor long-exposure handling is implemented: the 174 (>4s) and 6200 (>=1s) switch to FPGA wait+trigger mode and host-time the integration, so wall-clock ≈ exposure + readout")
	gain := flag.Int("gain", 0, "gain in 0.1 dB units (with -capture)")
	offset := flag.Int("offset", -1, "offset / black level (with -capture); -1 = leave default")
	bin := flag.Int("bin", 1, "binning factor 1..4 (with -capture)")
	roi := flag.String("roi", "", "sub-frame ROI as x,y,w,h in BINNED pixels (with -capture); empty = full binned frame")
	raw8 := flag.Bool("raw8", false, "capture RAW8 (1 byte/pixel) instead of RAW16 (with -capture)")
	highspeed := flag.Bool("highspeed", false, "10-bit HIGH-SPEED readout (~2× fps; implies RAW8): the sensor's ASI_HIGH_SPEED_MODE — shorter ADC ramp at a doubled pixel clock (with -capture)")
	usb2 := flag.Bool("usb2", false, "force the USB2 HighSpeed readout path (bwUSB2 + 40% FPS) regardless of the model/link — toggle the USB2 vs USB3 behavior on a fixed physical link without replugging")
	fpsPerc := flag.Int("fps", 0, "bandwidth-overload / FPS percent 40..100 (0 = bus default: USB2→40, USB3→100). 100 = max throughput")
	out := flag.String("out", "frame.fits", "frame output file (with -capture); .fits/.fit writes FITS, any other extension writes raw RAW16")
	timeout := flag.Duration("timeout", 5*time.Second, "max wait for the frame (with -capture)")
	replay := flag.String("replay", "", "replay an SDK 'req reg val' write sequence (file) then read a frame")
	nframes := flag.Int("n", 1, "with -capture: after one arm, read N frames back-to-back (no re-arm) and time each — the cold-start/throwaway test")
	discard := flag.Bool("discard", false, "with -capture -n N: capture frames via the resident stream but DON'T write them — the pure-capture throughput benchmark, matching the SDK video bench (no disk I/O)")
	list := flag.Bool("list", false, "enumerate attached cameras (with serials) and exit")
	serial := flag.String("serial", "", "open the camera with this factory serial (hex) instead of -pid")
	thermal := flag.Bool("thermal", false, "read sensor temperature + humidity via the hardware Thermal backend and exit (read-only; no TEC actuation)")
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
	flag.Parse()

	if *list {
		if err := doList(); err != nil {
			log.Fatalf("list error: %v", err)
		}
		return
	}
	if *thermal {
		if err := doThermal(uint16(*pid)); err != nil {
			log.Fatalf("thermal error: %v", err)
		}
		return
	}
	if *tecoff {
		raw, err := astrocam.OpenHost(astrocam.ZWO.VID, uint16(*pid))
		if err != nil {
			log.Fatalf("tecoff: %v", err)
		}
		defer raw.Close()
		cam, _ := astrocam.Open(raw, astrocam.ZWO.VID, uint16(*pid))
		th := cam.HardwareThermal()
		_ = th.SetTECPower(0)
		_ = th.SetFan(false)
		t, _ := th.ReadTemp()
		fmt.Printf("TEC + fan OFF; temp %.2f °C\n", t)
		return
	}
	if *heater >= 0 {
		raw, err := astrocam.OpenHost(astrocam.ZWO.VID, uint16(*pid))
		if err != nil {
			log.Fatalf("heater: %v", err)
		}
		defer raw.Close()
		cam, _ := astrocam.Open(raw, astrocam.ZWO.VID, uint16(*pid))
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
		if err := doCool(uint16(*pid)); err != nil {
			log.Fatalf("cool error: %v", err)
		}
		return
	}
	if *regulate {
		if err := doRegulate(uint16(*pid), *regtarget, *kp, *ki, *kd, *slew, *seed, *maxerr, *regtarget2); err != nil {
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
		if err := doReplay(uint16(*pid), *replay); err != nil {
			log.Fatalf("replay error: %v", err)
		}
		return
	}
	o := captureOpts{
		exposure: *exposure, gain: *gain, offset: *offset, bin: *bin, roi: *roi,
		raw8: *raw8 || *highspeed, out: *out, timeout: *timeout, nframes: *nframes, usb2: *usb2,
		discard: *discard, highspeed: *highspeed, fpsPerc: *fpsPerc,
	}
	if err := run(uint16(*pid), *capture, *verbose, o); err != nil {
		log.Fatalf("error: %v", err)
	}
}

// captureOpts bundles the -capture controls (exposure/gain/offset/bin/roi/depth + output).
type captureOpts struct {
	exposure  time.Duration
	gain      int
	offset    int // -1 = leave the sensor default
	bin       int
	roi       string // "x,y,w,h" in binned pixels, or "" = full binned frame
	raw8      bool
	out       string
	timeout   time.Duration
	nframes   int
	usb2      bool // force the USB2 readout path regardless of model/link
	discard   bool // capture frames via the stream but don't write (pure-capture benchmark)
	highspeed bool // 10-bit high-speed readout (implies raw8)
	fpsPerc   int  // bandwidth-overload / FPS percent (40..100); 0 = bus default
}

func (o captureOpts) binOr1() int {
	if o.bin < 1 {
		return 1
	}
	return o.bin
}

func run(pid uint16, capture, verbose bool, o captureOpts) error {
	raw, err := astrocam.OpenHost(astrocam.ZWO.VID, pid)
	if err != nil {
		return fmt.Errorf("connect %04x:%04x: %w\n(camera plugged in, not claimed by another driver, accessible?)", astrocam.ZWO.VID, pid, err)
	}
	defer raw.Close() // always disconnect cleanly

	t0 := time.Now()
	el := func() string { return fmt.Sprintf("[%8.3fs]", time.Since(t0).Seconds()) }

	var t astrocam.Transport = raw
	if verbose {
		t = &logT{t: raw, w: os.Stderr, start: t0}
	}
	cam, err := astrocam.Open(t, astrocam.ZWO.VID, pid)
	if err != nil {
		return fmt.Errorf("bind PID 0x%04x: %w", pid, err)
	}
	if o.usb2 {
		cam.SetUSB3(false) // force the USB2 readout path (bwUSB2 + 40% FPS)
		fmt.Println("  link mode: FORCED USB2 (bwUSB2 + 40% FPS) via -usb2")
	}
	if o.fpsPerc != 0 {
		cam.SetFPSPercent(o.fpsPerc) // bandwidth-overload override (40..100); raises USB2 throughput
		fmt.Printf("  fps percent: %d (bandwidth-overload override)\n", o.fpsPerc)
	}

	fmt.Printf("connected %04x:%04x\n", astrocam.ZWO.VID, pid)
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

	// --- first-light test: full init + one exposure + read one frame ---
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
	if err := step("SetExposure", cam.SetExposure(o.exposure)); err != nil {
		return err
	}
	fmt.Printf("  exposing %s (gain %d, offset %d, bin %d, %dx%d, %d-bit out)...\n",
		o.exposure, o.gain, o.offset, o.binOr1(), w, h, cam.OutputDepth()*8)
	fmt.Printf("%s acquisition START (StartExposure)\n", el())
	if err := step("StartExposure", cam.StartExposure(true)); err != nil {
		return err
	}
	// Burst path: .ser writes a SER video container; -discard captures with NO write (the
	// pure-capture throughput benchmark, apples-to-apples with the SDK video bench — both just
	// read frames into a buffer and drop them). Both arm once and pull from a resident stream.
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
		var capEnd time.Time // capture-loop end stamp (before teardown), for an SDK-fair rate
		// Preferred path: arm once for free-run, then a RESIDENT stream session — the windowed
		// pump is primed once and each frame is pulled with Next, so the per-frame setup cost is
		// gone (the planetary hot path). Falls back to the single-shot worker if unsupported.
		if verr := cam.StartVideo(true); verr == nil {
			if sess, serr := cam.StartStream(total); serr == nil {
				// Warm-up frame doubles as a free-run PROBE: DDR cameras (6200/585) can't free-run a
				// resident stream (the FX3 holds each frame's final partial buffer for the worker's
				// FPGABufReload), so a short/empty warm-up means fall back to the single-shot worker
				// loop below. Free-run cameras (290/462/174) return a full frame and stream as before.
				wn, _ := sess.Next(buf, idle)
				streamed = wn == fbytes
				if streamed && o.discard {
					// Pure-capture benchmark: read a frame, drop it — exactly the SDK video bench's
					// loop, so the comparison is driver-read vs driver-read. Use the ZERO-COPY path
					// when the backend offers it (no per-frame memcpy), to isolate read overhead.
					// Zero-copy only when a whole frame fits in one transfer (sub-MiB ROI); larger
					// frames span chunks and aren't contiguous in one scratch, so fall back to Next.
					zc, hasZC := sess.(astrocam.FrameStreamZC)
					hasZC = hasZC && fbytes <= (1<<20)
					ivs := make([]float64, 0, nf) // per-frame intervals (ms) — drop vs bandwidth probe
					t0 = time.Now()               // time ONLY the steady-state loop (exclude arm/prime/warm-up)
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
					// Interval distribution: a tight unimodal spread = bandwidth-limited (no drops);
					// a cluster of ~2× intervals = dropped frames at the sensor's higher true rate.
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
						// Dump the raw per-frame interval (ms) of every frame to -out, so the
						// full run can be plotted (frame-time vs frame number — jitter/drift).
						if o.out != "" {
							var b strings.Builder
							for _, v := range ivs {
								fmt.Fprintf(&b, "%.3f\n", v)
							}
							if werr := os.WriteFile(o.out, []byte(b.String()), 0o644); werr == nil {
								fmt.Printf("  wrote %d per-frame intervals -> %s\n", len(ivs), o.out)
							}
						}
					}
				} else if streamed {
					// Async double-buffered writer: SER disk I/O runs on its own goroutine so it
					// overlaps the NEXT frame's capture — the read loop then runs at the sensor's
					// full rate instead of stalling on each write (the full-res ~2 ms/frame tax).
					const pool = 4
					free := make(chan []byte, pool)
					queue := make(chan []byte, pool)
					for i := 0; i < pool; i++ {
						free <- make([]byte, fbytes)
					}
					var writeErr error
					done := make(chan struct{})
					go func() {
						defer close(done)
						for fr := range queue {
							if writeErr == nil {
								writeErr = sw.writeFrame(fr)
							}
							free <- fr[:cap(fr)]
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
						queue <- fb[:cn]
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
					capEnd = time.Now() // stamp BEFORE teardown — sess.Close (abort+join) must not count toward fps, matching the SDK bench which stamps before ASIStopVideoCapture
				}
				sess.Close()
			}
		}
		if !streamed {
			// Fallback: single-shot worker, one full arm→expose→read per frame. This is the path
			// for DDR cameras (6200/585) that can't free-run a resident stream — each frame is a
			// complete arm + windowed read + FPGABufReload, like the SDK's per-frame capture worker.
			t0 = time.Now() // time ONLY the worker loop (exclude the failed free-run probe + arming)
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
					if werr := sw.writeFrame(buf[:cn]); werr != nil {
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
	// Continuous test (-n N): one arm, then N back-to-back reads (no re-arm, no pipe
	// reset). If frame 0 is ~2× the exposure and frames 1..N-1 are ~1×, the 2× is a
	// one-time cold-start throwaway and a warm/streaming sensor reads at 1×.
	if o.nframes > 1 {
		cbuf := make([]byte, cam.FrameBytes())
		for f := 0; f < o.nframes; f++ {
			t1 := time.Now()
			cn, cerr := cam.ReadFrame(cbuf, false)
			fmt.Printf("%s frame %d: %d bytes in %.3fs (err: %v)\n", el(), f, cn, time.Since(t1).Seconds(), cerr)
		}
		return nil
	}
	_ = o.timeout
	fmt.Printf("%s begin readout (read blocks until the frame arrives)\n", el())
	// Read EXACTLY one frame. An over-sized buffer would make the read wait for the
	// NEXT frame to top off the extra bytes — and in long-exposure mode frames aren't
	// back-to-back (a full integration period sits between them), so a 64 KiB margin
	// cost a whole extra exposure (the "2× latency"). FrameBytes ends on frame 1.
	buf := make([]byte, cam.FrameBytes())
	n, err := cam.GetDataAfterExp(buf)
	fmt.Printf("%s readout returned %d bytes (err: %v)\n", el(), n, err)
	if err == nil {
		x, y, w, h := cam.ROI()
		step := cam.OutputDepth()
		bayer := ""
		if cam.Color() {
			bayer = info.Bayer
		}
		if werr := writeFrameFile(o.out, buf[:n], w, h, step, bayer, info.PixelUm, o.exposure, o.gain, cam.Name()); werr != nil {
			fmt.Printf("  warning: writing %s: %v\n", o.out, werr)
		}
		// Raw pixel stats — proof of real signal, and STDEV (the read-noise metric a dark-frame
		// gain sweep needs to find the HCG break). Pixels are RAW16 LE or RAW8 per OutputDepth.
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
	// Even on a timeout/over-read, data may have flowed — inspect it. n is the
	// bytes that arrived; scan for the frame header to confirm we have real frames.
	if n <= 0 {
		return fmt.Errorf("no data on EP 0x81 (sensor not streaming): %w", err)
	}
	fmt.Printf("  got %d bytes (err: %v)\n", n, err)
	fmt.Printf("  first 16 bytes: % x\n", buf[:16])
	off := findMagic(buf[:n])
	rawOut := rawPath(o.out)                 // a partial frame is raw — no valid FITS dims
	_ = os.WriteFile(rawOut, buf[:n], 0o644) // dump the raw stream for inspection
	fmt.Printf("%s wrote %s (%d bytes)\n", el(), rawOut, n)
	if off >= 0 {
		fmt.Printf("\n*** SENSOR IS STREAMING *** frame magic 0xBB00AA11 found at offset %d of %d bytes\n", off, n)
		fmt.Printf("    raw stream dumped to %s (frame size %d) — alignment/framing is the only gap left\n", rawOut, cam.FrameBytes())
		return nil
	}
	fmt.Printf("\n  %d bytes flowed but no 0xBB00AA11 header found; raw dumped to %s for inspection\n", n, rawOut)
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

// rawPath swaps a .fits/.fit extension for .raw (partial/debug dumps are always raw,
// since a truncated frame has no valid FITS dimensions).
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

// doReplay sends a captured SDK 'req reg val' (hex) write sequence verbatim, then
// reads one frame — a ground-truth test that our transport can drive the camera,
// independent of our decoded init.
// doList enumerates attached ZWO cameras and reads each one's factory serial (opening
// then closing it) — the Alpaca "what's plugged in, by stable id" listing.
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

// doSerial opens the camera with the given factory serial (the stable Alpaca bind) and
// prints its identity, proving the enumerate -> open-by-location -> match-serial search.
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

// doThermal reads temperature + humidity off a (cooled) camera via the decoded hardware
// Thermal backend, and dumps the raw 0xB3 temp bytes so the decode can be checked against
// the SDK. Read-only — it does NOT drive the TEC, fan, or heater.
func doThermal(pid uint16) error {
	raw, err := astrocam.OpenHost(astrocam.ZWO.VID, pid)
	if err != nil {
		return err
	}
	defer raw.Close()
	cam, err := astrocam.Open(raw, astrocam.ZWO.VID, pid)
	if err != nil {
		return err
	}
	fmt.Printf("connected %04x:%04x  %s  cooled=%v\n", astrocam.ZWO.VID, pid, cam.Name(), cam.Cooled())

	// Raw 0xB3 bytes, so we can see both candidate decodings vs the SDK ground truth.
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

// doCool ACTUATES the TEC on a cooled camera: first an open-loop step sweep (proving the
// FPGA cool-power register physically cools the sensor, with the fan auto-driven), then a
// closed-loop regulation run through the Cooler goroutine. It always returns the TEC to 0
// and the fan off on exit — including Ctrl-C — so the camera is never left driven.
func doCool(pid uint16) error {
	raw, err := astrocam.OpenHost(astrocam.ZWO.VID, pid)
	if err != nil {
		return err
	}
	defer raw.Close()
	cam, err := astrocam.Open(raw, astrocam.ZWO.VID, pid)
	if err != nil {
		return err
	}
	if !cam.Cooled() {
		return fmt.Errorf("%s has no cooler", cam.Name())
	}
	// The SDK runs InitCamera (which sets up the FPGA/cooling block) before driving the
	// TEC; doing cooling control on an uninitialized FX3/FPGA wedges control transfers.
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

// doRegulate ACTUATES the TEC in closed loop to target °C with the per-tick power slew
// disabled and a 50% warm-start, then watches it converge and hold — the temperature-
// regulation test. Returns the TEC to 0 / fan off on exit (Ctrl-C included).
func doRegulate(pid uint16, target, kp, ki, kd, slew, seed, maxerr, target2 float64) error {
	raw, err := astrocam.OpenHost(astrocam.ZWO.VID, pid)
	if err != nil {
		return err
	}
	defer raw.Close()
	cam, err := astrocam.Open(raw, astrocam.ZWO.VID, pid)
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
	cfg.RampRate = 0       // no setpoint ramp — MaxError is what paces the approach here
	cfg.MaxError = maxerr  // symmetric error clamp (°C); the SDK-style gentle glide
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
		runPhase(target2, 150) // second leg — e.g. ramp back up (tests the clamp on warmup)
	}

	cam.DisableCooling()
	fmt.Printf("%s cooling disabled; returning TEC to 0\n", el())
	return nil
}

func doReplay(pid uint16, file string) error {
	data, err := os.ReadFile(file)
	if err != nil {
		return err
	}
	t, err := astrocam.OpenHost(astrocam.ZWO.VID, pid)
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
	buf := make([]byte, 4708352) // EXACTLY one frame (no over-read)
	t0 := time.Now()
	nb, rerr := t.BulkRead(buf, 12*time.Second)
	fmt.Printf("read took %.3fs\n", time.Since(t0).Seconds())
	nz := 0
	for _, b := range buf[:nb] {
		if b != 0 {
			nz++
		}
	}
	fmt.Printf("bulk read: %d bytes (err=%v), nonzero=%d, magic@%d\n", nb, rerr, nz, findMagic(buf[:nb]))
	if nb >= 16 {
		fmt.Printf("first 16 bytes: % x\n", buf[:16])
	}
	if nb > 0 {
		os.WriteFile("frame_replay.raw", buf[:nb], 0o644)
		fmt.Println("wrote frame_replay.raw")
	}
	return nil
}

// findMagic scans for the FrameMagic 0xBB00AA11 (little-endian: bytes 11 AA 00 BB).
func findMagic(b []byte) int {
	for i := 0; i+4 <= len(b); i++ {
		if b[i] == 0x11 && b[i+1] == 0xAA && b[i+2] == 0x00 && b[i+3] == 0xBB {
			return i
		}
	}
	return -1
}

// logT wraps a Transport and logs every transfer (the -v debug path). It forwards
// only the Transport methods, so a streaming backend falls back to logged BulkRead.
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

func (l *logT) BulkRead(buf []byte, to time.Duration) (int, error) {
	fmt.Fprintf(l.w, "%s BULK read<=%d timeout=%s start...\n", l.ts(), len(buf), to)
	n, err := l.t.BulkRead(buf, to)
	fmt.Fprintf(l.w, "%s BULK <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

// ReadFrameStream forwards the optional windowed-stream read (FrameStreamer) so the
// IMX455 worker's real data plane runs under -v instead of falling back to BulkRead.
func (l *logT) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	fs, ok := l.t.(astrocam.FrameStreamer)
	if !ok {
		return 0, fmt.Errorf("transport has no FrameStreamer")
	}
	fmt.Fprintf(l.w, "%s STREAM read<=%d idle=%s total=%s start...\n", l.ts(), len(buf), idle, total)
	n, err := fs.ReadFrameStream(buf, idle, total)
	fmt.Fprintf(l.w, "%s STREAM <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

func (l *logT) Close() error { return l.t.Close() }

// SuperSpeed forwards the optional link-speed report so the readout mode tracks the live
// USB link even under -v (the wrapper would otherwise hide it, defaulting to the model).
func (l *logT) SuperSpeed() bool {
	if sr, ok := l.t.(interface{ SuperSpeed() bool }); ok {
		return sr.SuperSpeed()
	}
	return false
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

func res(err error) string {
	if err != nil {
		return "  ERR: " + err.Error()
	}
	return ""
}
