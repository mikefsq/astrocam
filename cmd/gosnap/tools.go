package main

import (
	"encoding/binary"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/mikefsq/astrocam"
)

// doList enumerates attached cameras and reads each one's factory serial.
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

// doDumpFlash writes the factory defect-map blob from SPI flash (FlashHPCMapAddr) to a file.
// Layout: 2 KiB header with magic "ASID" (defect) or "ASIG" (gain) and a big-endian uint32
// payload length at offset 4, then the compressed 1-bit-per-pixel defect bitmap.
func doDumpFlash(tg target, path string) error {
	raw, cam, tg, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	fmt.Printf("connected %04x:%04x  %s\n", tg.vid, tg.pid, cam.Name())

	// The flash read needs the firmware and GPIF up, so Init first.
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

// doTECOff forces the TEC and fan off. Cooling registers on an uninitialized FPGA wedge EP0,
// so it runs Init first.
func doTECOff(tg target) error {
	raw, cam, _, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	if err := cam.Init(); err != nil {
		return fmt.Errorf("init: %w", err)
	}
	th := cam.HardwareThermal()
	_ = th.SetTECPower(0)
	_ = th.SetFan(false)
	t, _ := th.ReadTemp()
	fmt.Printf("TEC + fan OFF; temp %.2f °C\n", t)
	return nil
}

// doHeater sets the anti-dew heater to pct % and reads the register back. Init first, as
// doTECOff.
func doHeater(tg target, pct int) error {
	raw, cam, _, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	if err := cam.Init(); err != nil {
		return fmt.Errorf("init: %w", err)
	}
	th := cam.HardwareThermal()
	rm := cam.Rm()
	if err := th.SetHeater(pct); err != nil {
		return err
	}
	rb2a, _ := rm.ReadFPGAReg(0x2a)
	rb19, _ := rm.ReadFPGAReg(0x19)
	fmt.Printf("heater %d%% -> reg0x2a=0x%02x (%d/255), warm-enable (reg0x19 bit6)=%d\n",
		pct, rb2a, rb2a, (rb19>>6)&1)
	return nil
}

// doThermal reads temperature and humidity through the vendor's Thermal backend, and on ZWO also
// dumps the raw 0xB3 bytes with both candidate decodings. Read-only: it does not drive the TEC,
// fan, or heater.
func doThermal(tg target) error {
	raw, cam, tg, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	fmt.Printf("connected %04x:%04x  %s  cooled=%v\n", tg.vid, tg.pid, cam.Name(), cam.Cooled())

	// The raw dump is ZWO's temperature request and ZWO's packing. It must not be sent to another
	// vendor: 0xB3 is PlayerOne's protected sensor-register write, so issuing it here as a
	// control-IN is exactly the cross-vendor mistake the driver refuses to make elsewhere.
	if tg.vid == astrocam.ZWO.VID {
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
	}

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

// doCool actuates the TEC: an open-loop power-step sweep, then a closed-loop regulation
// run. It returns the TEC to 0 and the fan off on exit (Ctrl-C included).
func doCool(tg target) error {
	raw, cam, _, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	if !cam.Cooled() {
		return fmt.Errorf("%s has no cooler", cam.Name())
	}
	// Cooling control on an uninitialized FX3/FPGA wedges control transfers, so Init first.
	if err := cam.Init(); err != nil {
		return fmt.Errorf("init: %w", err)
	}
	defer cam.StopExposure()
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

	// Drive the TEC to 0 and the fan off on the way out, Ctrl-C included (the open-loop steps
	// below drive the TEC directly, so cam.Close alone would not zero it).
	stop := func() { _ = th.SetTECPower(0); _ = th.SetFan(false) }
	defer stop()
	hooks := &exitHooks{}
	hooks.add(stop)
	defer installInterrupt(cam, hooks)()

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

	setpt := base - 10
	fmt.Printf("Phase 2 — closed-loop regulation to %.1f °C (Cooler goroutine):\n", setpt)
	cfg := astrocam.DefaultCoolerConfig()
	cfg.RampRate = 30 // °C/min setpoint ramp
	if err := cam.EnableCooling(nil, setpt, cfg); err != nil {
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

// doRegulate regulates the TEC in closed loop to setpt °C and reports until it holds.
// It returns the TEC to 0 and the fan off on exit (Ctrl-C included).
func doRegulate(tg target, setpt, kp, ki, kd, slew, seed, maxerr, target2 float64) error {
	raw, cam, _, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
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

	// cam.Close (via the interrupt handler) disables the regulation loop and zeroes the TEC;
	// stop covers the normal return.
	stop := func() { cam.DisableCooling(); _ = th.SetTECPower(0); _ = th.SetFan(false) }
	defer stop()
	defer installInterrupt(cam, nil)()

	base, _ := cam.Temperature()
	fmt.Printf("%s %s  baseline %.2f °C  (Kp=%.3g Ki=%.3g Kd=%.3g slew=%.3g seed=%.3g maxerr=%.3g)\n",
		el(), cam.Name(), base, kp, ki, kd, slew, seed, maxerr)

	cfg := astrocam.DefaultCoolerConfig()
	cfg.Kp, cfg.Ki, cfg.Kd = kp, ki, kd
	cfg.SlewPerStep = slew // 0 = no per-tick rate limit
	cfg.RampRate = 0       // no setpoint ramp; MaxError paces the approach
	cfg.MaxError = maxerr  // symmetric error clamp (°C)
	if err := cam.EnableCooling(nil, setpt, cfg); err != nil {
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
			eff := e // the error the loop acts on, after the clamp
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

	runPhase(setpt, 150)
	if !math.IsNaN(target2) {
		runPhase(target2, 150) // second leg, e.g. ramp back up
	}

	cam.DisableCooling()
	fmt.Printf("%s cooling disabled; returning TEC to 0\n", el())
	return nil
}

// doReplay sends a captured SDK 'req reg val' (hex) write sequence verbatim, then reads
// one frame. It tests that the transport alone can drive the camera.
func doReplay(tg target, file string) error {
	data, err := os.ReadFile(file)
	if err != nil {
		return err
	}
	t, _, err := tg.open()
	if err != nil {
		return err
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
	buf := make([]byte, 4708352) // one frame, no over-read
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

// doSoak runs a long single-threaded continuous capture and reports the cumulative
// StallCount. It honors -exposure/-gain/-usb2/-fps.
func doSoak(tg target, o captureOpts, frames int, video bool) error {
	raw, cam, _, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
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
	defer installInterrupt(cam, nil)()
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
	// Single-shot arms every frame; free-run arms once and reads back-to-back.
	mode := "single-shot (arm every frame)"
	arm := func() error { return cam.StartExposure(true) }
	read := func() (int, error) { return cam.GetDataAfterExp(buf) }
	if video {
		if err := cam.StartVideo(true); err != nil {
			return fmt.Errorf("start video: %w", err)
		}
		mode = "free-run (arm once, StartVideo + ReadFrame)"
		arm = func() error { return nil } // armed once above
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

// doWedge reproduces the USB2 readout wedge: a control transfer landing mid-readout parks
// the GPIF, and every capture then fails until a hard reset. It runs continuous full-frame
// USB2 captures while an antagonist goroutine fires ControlIn(0xB3) on a tight interval
// until a frame fails, checks whether the wedge is sticky, then recovers via ResetDevice
// and a full re-Init. -wedgeantagonist=false is the A/B control.
func doWedge(tg target, o captureOpts, iters int, antagonist bool, intervalMs, maxSec int, req uint8) error {
	raw, cam, _, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	cam.SetUSB3(false)     // the USB2 readout path is the wedge-prone one
	cam.SetFPSPercent(100) // peak bus pressure during the read
	if err := cam.Init(); err != nil {
		return fmt.Errorf("init: %w", err)
	}
	defer cam.StopExposure()
	defer installInterrupt(cam, nil)()
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
	var buf []byte // the frame buffer; nil after a read was abandoned to a stuck goroutine
	if intervalMs < 1 {
		intervalMs = 1
	}
	if maxSec < 1 {
		maxSec = 90
	}
	t0 := time.Now()
	el := func() string { return fmt.Sprintf("[%6.1fs]", time.Since(t0).Seconds()) }
	fmt.Printf("wedge: USB2, fps100, exp %s, %d B/frame, antagonist=%v (0x%02X every %dms), cap %ds / %d frames\n",
		exp, fb, antagonist, req, intervalMs, maxSec, iters)

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

	// Heartbeat every 2 s, so the run is never silent while blocked inside a wedged frame.
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

	// Antagonist: fire ControlIn(0xB3) into the readout window; its first failure (EP0 stopped
	// ACKing) marks the wedge and stops it.
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
					if _, err := raw.ControlIn(req, 0, 0, b[:]); err != nil {
						markWedge(fmt.Sprintf("antagonist 0x%02X control transfer failed: %v", req, err))
						return
					}
					atomic.AddInt64(&fired, 1)
				}
			}
		}()
	}

	// captureBounded runs one frame under a watchdog: on timeout it aborts via StopExposure,
	// then via ResetDevice (an ioctl, which works even when EP0 is timing out). The frame
	// goroutine sends to a buffered channel so a late completion never blocks or leaks. A read
	// that never returns keeps its buffer (the kernel may still DMA into it); the next attempt
	// allocates a fresh one.
	type res struct {
		n   int
		err error
	}
	captureBounded := func(timeout time.Duration) (int, error) {
		if err := cam.StartExposure(true); err != nil {
			return 0, err
		}
		if buf == nil {
			buf = make([]byte, fb)
		}
		b := buf
		done := make(chan res, 1)
		go func() { n, err := cam.GetDataAfterExp(b); done <- res{n, err} }()
		select {
		case r := <-done:
			return r.n, r.err
		case <-time.After(timeout):
		}
		go cam.StopExposure() // may itself be slow on a dead EP0
		select {
		case r := <-done:
			return r.n, r.err
		case <-time.After(3 * time.Second):
		}
		_ = cam.ResetDevice() // makes the stuck read/control error out
		select {
		case r := <-done:
			return r.n, r.err
		case <-time.After(3 * time.Second):
			buf = nil // abandoned: b stays with the stuck goroutine
			return 0, fmt.Errorf("frame stuck > %s even after StopExposure+ResetDevice", timeout)
		}
	}

	// Watchdog per frame: the exposure plus the frame at a conservative USB2 rate (20 MB/s; a
	// 122 MB IMX455 frame takes ~3.4 s on the wire) plus a 3 s margin.
	frameTimeout := readBudget(exp, fb)
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

	// Stop the helpers; an in-flight 0xB3 returns within the driver's 500 ms control timeout.
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

	// Probe once more with the antagonist stopped and no reset: a parked GPIF fails again
	// (sticky wedge); a one-frame stall reads a full frame.
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

	// Recover via whole-device reset and full re-Init (idempotent after captureBounded).
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

// doDumpRegs reads sensor and FPGA registers back and prints them, writing nothing. It exists to
// read a mode block off a camera the VENDOR SDK has just programmed: run the SDK tool, leave the
// camera powered, then dump. The register file survives the SDK closing the device, so this
// recovers what the vendor wrote without a USB analyzer in the path.
//
// spec is a comma-separated list; a bare number is a sensor register (vendor read 0xB2) and an
// "f:" prefix an FPGA register (0xC2). Ranges are written lo-hi. Everything is hex, 0x optional:
//
//	-dumpregs 0x301a,0x3069,f:0x14-0x18
func doDumpRegs(tg target, spec string) error {
	raw, cam, tg, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	fmt.Printf("connected %04x:%04x  %s\n", tg.vid, tg.pid, cam.Name())

	parse := func(s string) (uint16, error) {
		v, err := strconv.ParseUint(strings.TrimPrefix(strings.TrimSpace(s), "0x"), 16, 16)
		return uint16(v), err
	}
	rm := cam.Rm()
	for _, item := range strings.Split(spec, ",") {
		item = strings.TrimSpace(item)
		if item == "" {
			continue
		}
		fpga := strings.HasPrefix(item, "f:")
		item = strings.TrimPrefix(item, "f:")
		lo, hi := item, item
		if a, b, ok := strings.Cut(item, "-"); ok {
			lo, hi = a, b
		}
		first, err := parse(lo)
		if err != nil {
			return fmt.Errorf("bad register %q: %w", lo, err)
		}
		last, err := parse(hi)
		if err != nil {
			return fmt.Errorf("bad register %q: %w", hi, err)
		}
		for r := first; r <= last; r++ {
			var v uint16
			var err error
			if fpga {
				v, err = rm.ReadFPGAReg(r)
			} else {
				v, err = rm.ReadReg(r)
			}
			bus := "sensor"
			if fpga {
				bus = "fpga  "
			}
			if err != nil {
				fmt.Printf("  %s 0x%04x = error: %v\n", bus, r, err)
				continue
			}
			fmt.Printf("  %s 0x%04x = 0x%02x  (%d)\n", bus, r, v, v)
			if r == last {
				break // guard the uint16 wrap at 0xffff
			}
		}
	}
	return nil
}

// doFlashAt dumps a raw SPI flash range, "addr,len" in hex. The layout differs per vendor — ZWO
// keeps its defect map behind an ASID header at 0x40000, PlayerOne an "HPC:" header at 0x42000 —
// so this reads whatever address is asked for and leaves interpretation to the caller.
func doFlashAt(tg target, spec string) error {
	raw, cam, tg, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	a, l, ok := strings.Cut(spec, ",")
	if !ok {
		return fmt.Errorf("want addr,len (hex), got %q", spec)
	}
	addr, err := strconv.ParseUint(strings.TrimPrefix(strings.TrimSpace(a), "0x"), 16, 32)
	if err != nil {
		return fmt.Errorf("bad address %q: %w", a, err)
	}
	n, err := strconv.ParseUint(strings.TrimPrefix(strings.TrimSpace(l), "0x"), 16, 32)
	if err != nil {
		return fmt.Errorf("bad length %q: %w", l, err)
	}
	fmt.Printf("connected %04x:%04x  %s\n", tg.vid, tg.pid, cam.Name())
	b, err := cam.ReadSPIFlash(uint32(addr), int(n))
	if err != nil {
		return err
	}
	fmt.Printf("flash @0x%x  %d bytes\n", addr, len(b))
	for i := 0; i < len(b); i += 16 {
		e := i + 16
		if e > len(b) {
			e = len(b)
		}
		fmt.Printf("  %06x  % x  |%s|\n", int(addr)+i, b[i:e], printable(b[i:e]))
	}
	return nil
}

func printable(b []byte) string {
	out := make([]byte, len(b))
	for i, c := range b {
		if c >= 0x20 && c < 0x7f {
			out[i] = c
		} else {
			out[i] = '.'
		}
	}
	return string(out)
}

// doDefects reads the factory defect map and reports it, writing nothing to the camera.
func doDefects(tg target) error {
	raw, cam, tg, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	info := cam.Info()
	fmt.Printf("connected %04x:%04x  %s  array %dx%d\n", tg.vid, tg.pid, cam.Name(), info.MaxWidth, info.MaxHeight)
	m, err := cam.LoadDefectMap(info.MaxWidth, info.MaxHeight)
	if err != nil {
		return err
	}
	px := info.MaxWidth * info.MaxHeight
	fmt.Printf("defect map: %d pixels (%.4f%% of %d)\n", m.Count(), 100*float64(m.Count())/float64(px), px)
	for _, p := range m.Defects {
		fmt.Printf("%d %d\n", p%info.MaxWidth, p/info.MaxWidth)
	}
	return nil
}

// doGuide issues one ST4 pulse and reports the transfers it took. The guide lines are outputs to
// a mount's ST4 port; with nothing plugged in the pulse is electrically a no-op, so this is safe
// to run as a wire check.
func doGuide(tg target, spec string) error {
	raw, cam, tg, err := openCamera(tg)
	if err != nil {
		return err
	}
	defer raw.Close()
	dirName, durStr, ok := strings.Cut(spec, ",")
	if !ok {
		return fmt.Errorf("want dir,duration (e.g. N,250ms), got %q", spec)
	}
	dirs := map[string]astrocam.GuideDir{
		"N": astrocam.GuideNorth, "S": astrocam.GuideSouth,
		"E": astrocam.GuideEast, "W": astrocam.GuideWest,
	}
	dir, okDir := dirs[strings.ToUpper(strings.TrimSpace(dirName))]
	if !okDir {
		return fmt.Errorf("direction %q: want N, S, E or W", dirName)
	}
	d, err := time.ParseDuration(strings.TrimSpace(durStr))
	if err != nil {
		return fmt.Errorf("bad duration %q: %w", durStr, err)
	}
	if !cam.ST4() {
		return fmt.Errorf("%s has no ST4 port", cam.Name())
	}
	fmt.Printf("connected %04x:%04x  %s  ST4=%v\n", tg.vid, tg.pid, cam.Name(), cam.ST4())
	t0 := time.Now()
	if err := cam.PulseGuide(dir, d); err != nil {
		return fmt.Errorf("pulse %s for %v: %w", dirName, d, err)
	}
	fmt.Printf("  pulsed %s for %v (took %v); guiding now: %v\n",
		strings.ToUpper(dirName), d, time.Since(t0).Round(time.Millisecond), cam.IsPulseGuiding())
	return nil
}
