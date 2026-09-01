package main

import (
	"fmt"
	"math"
	"os"
	"sort"
	"strings"
	"time"

	"github.com/mikefsq/astrocam"
)

// run is the default mode: connect and print the identity; with -capture, configure the
// sensor from o and read one frame (captureSingle) or a burst (captureBurst).
func run(tg target, capture, verbose bool, o captureOpts) error {
	raw, tg, err := tg.open()
	if err != nil {
		return err
	}
	defer raw.Close()

	t0 := time.Now()
	el := func() string { return fmt.Sprintf("[%8.3fs]", time.Since(t0).Seconds()) }

	var t astrocam.Transport = raw
	if verbose {
		t = &logT{t: raw, w: os.Stderr, start: t0}
	}
	// Forcing the link has to happen BEFORE Open, which reads the reported speed to pick the
	// bandwidth percentage default (90 on PlayerOne either way, 40/100 on ZWO) and to decide
	// whether EP0 needs pacing during a readout.
	if o.usb2 {
		if lf, ok := raw.(astrocam.LinkForcer); ok {
			lf.ForceUSB2(true)
		} else {
			fmt.Println("  warning: -usb2 not supported by this transport; only the budget is forced")
		}
	}
	cam, err := astrocam.Open(t, tg.vid, tg.pid)
	if err != nil {
		return fmt.Errorf("bind PID 0x%04x: %w", tg.pid, err)
	}
	if o.usb2 {
		cam.SetUSB3(false) // belt and braces: the readout mode too, whatever the transport said
		fmt.Println("  link mode: FORCED USB2 (budget, GPIF divider and EP0 pacing) via -usb2")
	}
	if o.fpsPerc != 0 {
		cam.SetFPSPercent(o.fpsPerc) // clamped to the vendor's range
		fmt.Printf("  fps percent: %d (bandwidth-overload override)\n", o.fpsPerc)
	}
	printIdentity(cam, raw, tg)
	if !capture {
		fmt.Println("disconnecting.")
		return nil
	}

	fmt.Println("capture:")
	defer cam.StopExposure() // leave the device stopped on exit
	if err := step("Init", cam.Init()); err != nil {
		return err
	}
	hooks := &exitHooks{}
	defer installInterrupt(cam, hooks)()
	w, h, err := configureCapture(cam, o)
	if err != nil {
		return err
	}
	fmt.Printf("  exposing %s (gain %d, offset %d, bin %d, %dx%d, %d-bit out)...\n",
		cam.Exposure(), cam.Gain(), cam.Offset(), o.binOr1(), w, h, cam.OutputDepth()*8)
	// Burst path: .ser writes a SER video container; -n N without one discards the frames and
	// reports per-frame intervals. Both arm once and pull frames from a resident stream. Each
	// path arms where it takes its frames, so a burst is not armed for a single shot first.
	isSer := strings.HasSuffix(strings.ToLower(o.out), ".ser")
	if o.nframes > 1 && !isSer {
		// A burst records to a SER container only. Say so rather than write nothing: -out names
		// one image, and the interval dump below refuses to overwrite an image path, so a run
		// asking for frames would otherwise finish having produced no file at all.
		if !o.discard {
			fmt.Printf("  -n %d with -out %q: frames are read and discarded (throughput benchmark).\n", o.nframes, o.out)
			fmt.Printf("  To record the burst use a .ser output (-out run.ser); for the interval dump use a .txt path.\n")
		}
		o.discard = true
	}
	if isSer || o.discard {
		return captureBurst(cam, o, w, h, hooks, el)
	}
	return captureSingle(cam, o, el)
}

// step reports one configuration call by name and wraps its error.
func step(name string, err error) error {
	if err != nil {
		return fmt.Errorf("%s: %w", name, err)
	}
	fmt.Printf("  %-14s ok\n", name)
	return nil
}

// printIdentity prints the connected camera's link, model, sensor and identity lines.
func printIdentity(cam *astrocam.Camera, raw astrocam.Transport, tg target) {
	fmt.Printf("connected %04x:%04x\n", tg.vid, tg.pid)
	if d, ok := raw.(interface{ Describe() string }); ok {
		fmt.Printf("  link     : %s\n", d.Describe())
	}
	info := cam.Info()
	fmt.Printf("  model    : %s\n", cam.Name())
	fmt.Printf("  sensor   : %s\n", cam.Sensor().Name)
	fmt.Printf("  geometry : %d x %d px, %.2f µm pixel, %d-bit\n", info.MaxWidth, info.MaxHeight, info.PixelUm, info.BitDepth)
	fmt.Printf("  color    : %v (bayer %q)\n", cam.Color(), info.Bayer)
	fmt.Printf("  cooled   : %v\n", cam.Cooled())
	// Advertised control ranges. These are vendor policy over a shared die, so they are the
	// cheapest thing to diff against the vendor SDK's own caps dump.
	gmin, gmax := cam.GainRange()
	fmt.Printf("  gain     : %d..%d\n", gmin, gmax)
	if omin, omax, odef, ok := cam.OffsetRange(); ok {
		fmt.Printf("  offset   : %d..%d default %d\n", omin, omax, odef)
	}
	// PlayerOne carries a second version behind the FX3 bridge — the camera FPGA's own — plus a
	// status byte, both read-only. ZWO's equivalents are not decoded, so this is vendor-gated.
	if tg.vid == astrocam.POA.VID {
		if v, err := astrocam.POAFPGAVersion(cam.Rm()); err == nil {
			fmt.Printf("  fpga     : version 0x%02x\n", v)
		}
		if v, err := astrocam.POAFPGAStatus(cam.Rm()); err == nil {
			fmt.Printf("  fpga stat: 0x%02x\n", v)
		}
	}
	fmt.Printf("  formats  : %v\n", cam.ImageFormats())
	if p, ok := cam.GainOffsetPresets(); ok {
		fmt.Printf("  presets  : gain highestDR=%d HCG=%d unity=%d lowestRN=%d\n",
			p.GainHighestDR, p.GainHCG, p.GainUnity, p.GainLowestRN)
		fmt.Printf("             offset highestDR=%d HCG=%d unity=%d lowestRN=%d\n",
			p.OffsetHighestDR, p.OffsetHCG, p.OffsetUnity, p.OffsetLowestRN)
	}
	if lim, ok := cam.WhiteBalanceRange(); ok {
		fmt.Printf("  wb range : +-%d per channel\n", lim)
	}
	if m := cam.SensorModes(); len(m) > 0 {
		names := make([]string, len(m))
		for i, mi := range m {
			names[i] = fmt.Sprintf("%d=%s", i, mi.Name)
		}
		fmt.Printf("  modes    : %s\n", strings.Join(names, " "))
	}
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
}

// configureCapture applies the -capture controls in the order the driver needs (output depth,
// high-speed, binning, ROI, gain, offset, exposure) and returns the binned window size.
func configureCapture(cam *astrocam.Camera, o captureOpts) (w, h int, err error) {
	if o.raw8 {
		if err := step("SetOutputDepth(RAW8)", cam.SetOutputDepth(1)); err != nil {
			return 0, 0, err
		}
	}
	// The sensor mode goes after the sample size and before the window: the mode block is
	// indexed by both, and the geometry a mode programs depends on which mode is selected.
	if o.sensorMode != 0 {
		if err := step("SetSensorMode", cam.SetSensorMode(o.sensorMode)); err != nil {
			return 0, 0, err
		}
		if m := cam.SensorModes(); o.sensorMode < len(m) {
			fmt.Printf("  sensor mode  : %d (%s)\n", o.sensorMode, m[o.sensorMode].Name)
		}
	}
	if o.highspeed {
		if err := step("SetHighSpeedMode", cam.SetHighSpeedMode(true)); err != nil {
			return 0, 0, err
		}
		fmt.Println("  high-speed   : 10-bit readout (2× clock)")
	}
	// The frame-rate cap is programmed with the frame period, so it only has to be set before
	// SetExposure; bin-sum is part of the window, so it goes before SetROI.
	if o.frameLimit > 0 {
		cam.SetFrameRateLimit(o.frameLimit)
		fmt.Printf("  frame limit  : %d fps\n", o.frameLimit)
	}
	if o.binsum {
		if err := step("SetBinSum", cam.SetBinSum(true)); err != nil {
			return 0, 0, err
		}
		fmt.Println("  binned pixels: summed (not averaged)")
	}
	if o.bin > 1 {
		if err := step("SetHardwareBin", cam.SetHardwareBin(o.hwbin)); err != nil {
			return 0, 0, err
		}
		if err := step("SetBinning", cam.SetBinning(o.bin)); err != nil {
			return 0, 0, err
		}
		if o.hwbin {
			fmt.Println("  binning      : sensor-side (hardware) where the profile allows")
		}
	}
	x, y := 0, 0
	if o.roi != "" {
		if _, e := fmt.Sscanf(o.roi, "%d,%d,%d,%d", &x, &y, &w, &h); e != nil {
			return 0, 0, fmt.Errorf("bad -roi %q (want x,y,w,h): %v", o.roi, e)
		}
	} else {
		// The whole frame at this binning, as the driver sizes it: Max/bin is not always a legal
		// window (the IMX455 at bin 3 needs one row less), and SetBinning has already stored the
		// one it will accept.
		_, _, w, h = cam.ROI()
	}
	if err := step("SetROI", cam.SetROI(x, y, w, h)); err != nil {
		return 0, 0, err
	}
	if err := step("SetGain", cam.SetGain(o.gain)); err != nil {
		return 0, 0, err
	}
	if o.offset >= 0 {
		if err := step("SetOffset", cam.SetOffset(o.offset)); err != nil {
			return 0, 0, err
		}
	}
	fmt.Printf("  offset       : %d (read back from the sensor)\n", cam.Offset())
	if err := step("SetExposure", cam.SetExposure(o.exposure)); err != nil {
		return 0, 0, err
	}
	return w, h, nil
}

// ---- burst -----------------------------------------------------------------------------

// burst is one -n capture: a SER file (sw set) or a discard run with per-frame interval
// statistics (sw nil). It prefers the resident stream session and falls back to the
// single-shot worker when the camera cannot free-run one (DDR readout: 6200/585).
type burst struct {
	cam       *astrocam.Camera
	o         captureOpts
	w, h, bpp int
	fbytes    int // wire frame size (× SoftBin² when host-binning)
	delivered int // the frame after the host bin, what SER stores
	nf        int
	idle      time.Duration // per-completion stall bound
	total     time.Duration // whole-read bound
	buf       []byte
	sw        *serWriter
	el        func() string
	// process turns one wire frame into what the SER file stores (the FX3 marker repair and the
	// host bin) and returns its length. It is a field so the burst plumbing can be exercised
	// without a camera.
	process func(frame []byte, n int) int
}

func captureBurst(cam *astrocam.Camera, o captureOpts, w, h int, hooks *exitHooks, el func() string) error {
	b := &burst{cam: cam, o: o, w: w, h: h, bpp: cam.OutputDepth(), el: el}
	b.fbytes = cam.FrameBytes()
	b.delivered = w * h * b.bpp
	b.nf = o.nframes
	if b.nf < 1 {
		b.nf = 1
	}
	b.idle, b.total = o.exposure+time.Second, 2*o.exposure+3*time.Second
	b.buf = make([]byte, b.fbytes)
	b.process = func(frame []byte, n int) int {
		if !cam.RepairFrame(frame[:n]) { // FX3 marker corners
			fmt.Printf("  warning  : frame desynced (FX3 markers not at the boundaries)\n")
		}
		return cam.BinFrame(frame, n) // host bin (no-op at bin 1)
	}
	if !o.discard {
		bayer := ""
		if cam.Color() {
			bayer = cam.Info().Bayer
		}
		sw, err := newSER(o.out, w, h, b.bpp, serColorID(cam.Color(), bayer), cam.Name())
		if err != nil {
			return err
		}
		b.sw = sw
		hooks.add(func() { _ = sw.close() }) // an interrupt still writes the frame count
	}

	fmt.Printf("%s acquisition START (%d frames)\n", el(), b.nf)
	count, dt, streamed, err := b.stream(hooks)
	if err != nil {
		return err
	}
	if !streamed {
		count, dt, err = b.fallback()
		if err != nil {
			return err
		}
	}
	if b.sw != nil {
		if cerr := b.sw.close(); cerr != nil {
			return cerr
		}
		fmt.Printf("\n*** SER *** %d frames -> %s  %dx%d %d-bit  %.1f fps (%.3fs)\n",
			count, o.out, w, h, b.bpp*8, float64(count)/dt, dt)
	} else {
		fmt.Printf("\n*** CAPTURE (discard) *** %d frames  %dx%d %d-bit  %.1f fps (%.3fs)\n",
			count, w, h, b.bpp*8, float64(count)/dt, dt)
	}
	return nil
}

// stream arms free-run once and pulls the burst from a resident stream session. streamed is
// false when the camera cannot free-run a session (the warm-up frame comes back short), in
// which case the caller falls back; dt excludes the session teardown.
func (b *burst) stream(hooks *exitHooks) (count int, dt float64, streamed bool, err error) {
	if verr := b.cam.StartVideo(true); verr != nil {
		return 0, 0, false, nil
	}
	sess, serr := b.cam.StartStream(b.total)
	if serr != nil {
		return 0, 0, false, nil
	}
	hooks.add(func() { _ = sess.Close() })
	// The warm-up frame doubles as a free-run probe: DDR cameras (6200/585) cannot free-run a
	// resident stream and return a short/empty frame here.
	wn, werr := sess.Next(b.buf, b.idle)
	if wn != b.fbytes {
		fmt.Printf("%s resident stream probe: %d of %d bytes (err %v); falling back to the single-shot worker\n", b.el(), wn, b.fbytes, werr)
		b.closeSession(sess)
		return 0, 0, false, nil
	}
	t0 := time.Now()
	if b.sw == nil {
		count = b.discardLoop(sess)
	} else {
		count, err = b.serLoop(sess)
		if err != nil {
			sess.Close()
			b.sw.close()
			return 0, 0, true, err
		}
	}
	dt = time.Since(t0).Seconds() // sess.Close must not count toward the rate
	b.closeSession(sess)
	return count, dt, true, nil
}

func (b *burst) closeSession(sess astrocam.FrameStream) {
	if cerr := sess.Close(); cerr != nil {
		// A −6 here means the transport is poisoned (transfers left with the kernel); the
		// camera needs a replug after this process exits.
		fmt.Printf("%s stream session close: %v\n", b.el(), cerr)
	}
}

// discardLoop reads nf frames from the session and drops them, timing the intervals. The
// zero-copy path applies only when a whole frame fits in one transfer (sub-MiB); larger frames
// span chunks and use Next.
func (b *burst) discardLoop(sess astrocam.FrameStream) (count int) {
	zc, hasZC := sess.(astrocam.FrameStreamZC)
	hasZC = hasZC && b.fbytes <= (1<<20)
	ivs := make([]float64, 0, b.nf) // per-frame intervals (ms)
	last := time.Now()
	for f := 0; f < b.nf; f++ {
		if hasZC {
			fr, cerr := zc.NextZC(b.idle)
			if cerr != nil || len(fr) != b.fbytes {
				fmt.Printf("%s frame %d short/err: %d/%d (%v)\n", b.el(), f, len(fr), b.fbytes, cerr)
				break
			}
			zc.Release()
		} else {
			cn, cerr := sess.Next(b.buf, b.idle)
			if cerr != nil || cn != b.fbytes {
				fmt.Printf("%s frame %d short/err: %d/%d (%v)\n", b.el(), f, cn, b.fbytes, cerr)
				break
			}
		}
		now := time.Now()
		ivs = append(ivs, float64(now.Sub(last).Microseconds())/1000.0)
		last = now
		count++
	}
	b.printIntervals(ivs, true)
	return count
}

// serPool is how many frame buffers the SER burst cycles between the read loop and the writer.
const serPool = 4

// serLoop reads nf frames from the session into the SER file. The pixel work and the disk I/O
// run on their own goroutine so the read loop does nothing but pull frames: a sess.Next issued
// late drops a frame outright, while the writer can fall up to pool frames behind and catch up.
// Each queued frame therefore carries its own capture stamp, or the trailer would record when
// the writer got to it.
func (b *burst) serLoop(sess astrocam.FrameStream) (count int, err error) {
	const pool = serPool
	type serFrame struct {
		data []byte
		n    int
		at   time.Time
	}
	free := make(chan []byte, pool)
	queue := make(chan serFrame, pool)
	for i := 0; i < pool; i++ {
		free <- make([]byte, b.fbytes)
	}
	var writeErr error
	done := make(chan struct{})
	go func() {
		defer close(done)
		for fr := range queue {
			if writeErr == nil {
				n := b.process(fr.data, fr.n)
				writeErr = b.sw.writeFrame(fr.data[:n], fr.at)
			}
			free <- fr.data[:cap(fr.data)]
		}
	}()
	for f := 0; f < b.nf; f++ {
		fb := <-free
		cn, cerr := sess.Next(fb, b.idle)
		if cerr != nil || cn != b.fbytes {
			fmt.Printf("%s frame %d short/err: %d/%d (%v)\n", b.el(), f, cn, b.fbytes, cerr)
			free <- fb
			break
		}
		queue <- serFrame{data: fb, n: cn, at: time.Now()}
	}
	close(queue)
	<-done
	return b.sw.frameCount(), writeErr
}

// fallback is the single-shot worker path for cameras that cannot free-run a resident stream:
// one full arm and read per frame, written to the SER file when there is one. A probe that got
// far enough to arm free-run leaves the sensor streaming, so the stream is halted first and each
// frame starts from a stopped sensor.
func (b *burst) fallback() (count int, dt float64, err error) {
	_ = b.cam.StopExposure()
	t0 := time.Now()
	ivs := make([]float64, 0, b.nf)
	last := t0
	for f := 0; f < b.nf; f++ {
		if e := b.cam.StartExposure(true); e != nil {
			fmt.Printf("%s frame %d arm error: %v\n", b.el(), f, e)
			break
		}
		cn, cerr := b.cam.GetDataAfterExp(b.buf) // delivered (host-binned) bytes
		if cerr != nil || cn != b.delivered {
			fmt.Printf("%s frame %d short/err: %d/%d (%v)\n", b.el(), f, cn, b.delivered, cerr)
			break
		}
		if b.sw != nil {
			if werr := b.sw.writeFrame(b.buf[:cn], time.Now()); werr != nil {
				b.sw.close()
				return 0, 0, werr
			}
		}
		now := time.Now()
		ivs = append(ivs, float64(now.Sub(last).Microseconds())/1000.0)
		last = now
		count++
	}
	if b.sw == nil {
		b.printIntervals(ivs, false)
	}
	return count, time.Since(t0).Seconds(), nil
}

// printIntervals reports the per-frame interval spread. On the streamed path a cluster of ~2×
// intervals means dropped frames and the intervals are dumped to -out when it is not an image
// path; the fallback path reports the arm+read spread only.
func (b *burst) printIntervals(ivs []float64, streamed bool) {
	if len(ivs) <= 2 {
		return
	}
	s := append([]float64(nil), ivs...)
	sort.Float64s(s)
	med := s[len(s)/2]
	if !streamed {
		fmt.Printf("  intervals(ms): med=%.1f min=%.1f max=%.1f (per-frame arm+read)\n", med, s[0], s[len(s)-1])
		return
	}
	var gaps int
	for _, v := range ivs {
		if v > 1.5*med {
			gaps++
		}
	}
	fmt.Printf("  intervals(ms): med=%.2f p1=%.2f p99=%.2f max=%.2f | >1.5×med (likely drops)=%d/%d\n",
		med, s[len(s)/100], s[len(s)*99/100], s[len(s)-1], gaps, len(ivs))
	out := strings.ToLower(b.o.out)
	switch {
	case out == "":
	case strings.HasSuffix(out, ".fits"), strings.HasSuffix(out, ".fit"), strings.HasSuffix(out, ".ser"):
		fmt.Printf("  (interval dump skipped: -out %q looks like an image path; use a .txt/.csv)\n", b.o.out)
	default:
		var sb strings.Builder
		for _, v := range ivs {
			fmt.Fprintf(&sb, "%.3f\n", v)
		}
		if werr := os.WriteFile(b.o.out, []byte(sb.String()), 0o644); werr == nil {
			fmt.Printf("  wrote %d per-frame intervals -> %s\n", len(ivs), b.o.out)
		}
	}
}

// ---- single frame ------------------------------------------------------------------------

// captureSingle reads the one armed frame under a watchdog, writes it to -out and prints its
// pixel statistics; a partial frame after a read error is dumped raw for inspection.
func captureSingle(cam *astrocam.Camera, o captureOpts, el func() string) error {
	fmt.Printf("%s acquisition START (StartExposure)\n", el())
	if err := step("StartExposure", cam.StartExposure(true)); err != nil {
		return err
	}
	fmt.Printf("%s begin readout (read blocks until the frame arrives)\n", el())
	// Read one frame with no margin: an over-sized buffer makes the read wait for the next
	// frame to fill the extra bytes, which in long-exposure mode costs a whole exposure.
	buf := make([]byte, cam.FrameBytes())
	watchdog := readBudget(cam.Exposure(), cam.FrameBytes())
	if o.timeout > 0 {
		if o.timeout < watchdog {
			fmt.Printf("  -timeout %s is below the exposure + readout budget; using %s\n", o.timeout, watchdog)
		} else {
			watchdog = o.timeout
		}
	}
	n, err, ok := readBounded(cam, buf, watchdog)
	if !ok {
		fmt.Printf("%s readout TIMEOUT after %s\n", el(), watchdog)
		return fmt.Errorf("readout timed out after %s", watchdog)
	}
	fmt.Printf("%s readout returned %d bytes (err: %v)\n", el(), n, err)
	if err != nil {
		// On a read error, dump whatever arrived for inspection.
		if n <= 0 {
			return fmt.Errorf("no data on EP 0x81 (sensor not streaming): %w", err)
		}
		fmt.Printf("  got %d bytes (err: %v)\n", n, err)
		fmt.Printf("  first 16 bytes: % x\n", buf[:16])
		rawOut := rawPath(o.out) // a partial frame has no valid FITS dims
		_ = os.WriteFile(rawOut, buf[:n], 0o644)
		fmt.Printf("%s wrote %s (%d bytes; frame size %d)\n", el(), rawOut, n, cam.FrameBytes())
		return nil
	}
	info := cam.Info()
	x, y, w, h := cam.ROI()
	depth := cam.OutputDepth()
	bayer := ""
	if cam.Color() {
		bayer = info.Bayer
	}
	// FITS records the effective exposure and gain (what the sensor was programmed with), not
	// the requested values.
	if werr := writeFrameFile(o.out, buf[:n], w, h, depth, bayer, info.PixelUm, cam.Exposure(), cam.Gain(), cam.Name()); werr != nil {
		fmt.Printf("  warning: writing %s: %v\n", o.out, werr)
	}
	mn, mx, avg, sd := frameStats(buf[:n], depth)
	fmt.Printf("\n*** FRAME *** %d bytes -> %s  ROI %d,%d %dx%d, %d-bit out  pixels: min=%d max=%d avg=%.1f stdev=%.2f\n",
		n, o.out, x, y, w, h, depth*8, mn, mx, avg, sd)
	return nil
}

// readBudget is the time a single frame is allowed: the exposure (a worker integrates inside
// GetDataAfterExp) plus the frame at a conservative USB2 rate (20 MiB/s; a 122 MB IMX455 frame
// takes ~3.4 s on the wire) plus a 3 s margin. The wire term is carried at sub-second precision:
// rounded down to whole seconds every frame below 20 MiB would be allowed none at all.
func readBudget(exposure time.Duration, frameBytes int) time.Duration {
	return exposure + time.Duration(frameBytes)*time.Second/(20<<20) + 3*time.Second
}

// readBounded runs GetDataAfterExp on its own goroutine under a watchdog. ok is false on
// timeout, after a best-effort StopExposure abort; buf then still belongs to the read
// goroutine and must not be reused.
func readBounded(cam *astrocam.Camera, buf []byte, watchdog time.Duration) (n int, err error, ok bool) {
	type readRes struct {
		n   int
		err error
	}
	resCh := make(chan readRes, 1)
	go func() { rn, rerr := cam.GetDataAfterExp(buf); resCh <- readRes{rn, rerr} }()
	select {
	case r := <-resCh:
		return r.n, r.err, true
	case <-time.After(watchdog):
		_ = cam.StopExposure()
		return 0, nil, false
	}
}

// frameStats returns min, max, mean and standard deviation of the samples in frame (RAW16
// little-endian or RAW8 per depth).
//
// The sums are exact: a 62 MP frame near saturation puts the sum of squares at 2e20, far past
// where a float64 running total keeps whole units, and the low bits it drops are the whole of
// the variance the last line then subtracts out. Measured on a 16.7 MP synthetic flat, float64
// accumulation reported the standard deviation 1.3 % low. uint64 holds the sum of squares to
// 4.3e9 samples at 16 bits, well past any sensor here.
func frameStats(frame []byte, depth int) (mn, mx int, avg, sd float64) {
	mn, mx = 1<<16, 0
	var cnt, sum, sumsq uint64
	for i := 0; i+depth <= len(frame); i += depth {
		v := int(frame[i])
		if depth == 2 {
			v |= int(frame[i+1]) << 8
		}
		if v < mn {
			mn = v
		}
		if v > mx {
			mx = v
		}
		sum += uint64(v)
		sumsq += uint64(v) * uint64(v)
		cnt++
	}
	if cnt > 0 {
		n := float64(cnt)
		avg = float64(sum) / n
		if va := float64(sumsq)/n - avg*avg; va > 0 {
			sd = math.Sqrt(va)
		}
	}
	return mn, mx, avg, sd
}

// writeFrameFile saves a capture at its ROI dims: FITS if the path ends in .fits/.fit,
// otherwise the raw bytes.
func writeFrameFile(path string, data []byte, w, h, bpp int, bayer string, pixelUm float64, exposure time.Duration, gain int, model string) error {
	l := strings.ToLower(path)
	if strings.HasSuffix(l, ".fits") || strings.HasSuffix(l, ".fit") {
		return writeFITS(path, data, w, h, bpp, bayer, exposure.Seconds(), pixelUm, gain, model)
	}
	return os.WriteFile(path, data, 0o644)
}

// rawPath swaps a .fits/.fit extension for .raw.
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
