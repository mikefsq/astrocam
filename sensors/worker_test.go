package sensors

import (
	"errors"
	"testing"
	"time"

	. "github.com/mikefsq/astrocam"
)

// fakeCtl is a minimal WorkerCtl for band/​timing tests: every read succeeds and returns a
// full buffer, no aborts, register writes land in the embedded fakeRegmap.
type fakeCtl struct {
	rm      *fakeRegmap
	fb      int
	lastTO  time.Duration // timeout of the most recent frame read
	drained int           // DrainPipe calls, to assert the pipe is emptied before the arm
}

func (f *fakeCtl) Rm() Regmap            { return f.rm }
func (f *fakeCtl) VendorCmd(FX3Op) error { return nil }
func (f *fakeCtl) ResetEndpoint() error  { return nil }
func (f *fakeCtl) ResetDevice() error    { return nil }
func (f *fakeCtl) NoteStall()            {}
func (f *fakeCtl) DrainPipe(time.Duration) int {
	f.drained++
	return 0
}
func (f *fakeCtl) ReapplyOffset() error  { return nil }
func (f *fakeCtl) Aborted() bool         { return false }
func (f *fakeCtl) FrameBytes() int       { return f.fb }
func (f *fakeCtl) BulkRead(buf []byte, to time.Duration) (int, error) {
	f.lastTO = to
	return len(buf), nil
}
func (f *fakeCtl) BulkReadQuiet(buf []byte, _, to time.Duration) (int, error) {
	f.lastTO = to
	return len(buf), nil
}
func (f *fakeCtl) StreamFrame(buf []byte, _, to time.Duration) (int, error) {
	f.lastTO = to
	return len(buf), nil
}
func (f *fakeCtl) StreamFramePrequeued(buf []byte, _, to time.Duration) (int, error) {
	f.lastTO = to
	return len(buf), nil
}

// exactly1sBoundary runs a worker at the 1 s trigger threshold and asserts the host hold spans
// the full exposure: SetExposure engages FPGA trigger mode at >= 1 s, where the host trigger
// window is the integration, so a worker band predicate of `<= 1s` (instead of `< 1s`) would put
// the 1 s case in the free-run band's exposure-200 ms sleep and under-integrate it by 20%.
func exactly1sBoundary(t *testing.T, worker func(WorkerCtl, []byte, time.Duration) (int, error)) {
	t.Helper()
	ctl := &fakeCtl{rm: &fakeRegmap{}, fb: 64}
	start := time.Now()
	n, err := worker(ctl, make([]byte, 64), time.Second)
	elapsed := time.Since(start)
	if err != nil {
		t.Fatalf("worker: %v", err)
	}
	if n != 64 {
		t.Fatalf("worker read %d bytes, want 64", n)
	}
	if elapsed < time.Second {
		t.Errorf("exactly-1s exposure held the trigger window only %v — the worker took the "+
			"free-run band (exposure−200ms) while SetExposure armed trigger mode", elapsed)
	}
}

func TestIMX178WorkerTriggerBandBoundary(t *testing.T) { exactly1sBoundary(t, imx178Worker) }
func TestIMX585WorkerTriggerBandBoundary(t *testing.T) { exactly1sBoundary(t, imx585Worker) }

// scriptCtl is a WorkerCtl whose successive frame-read results are scripted, recording the
// recovery calls a worker makes (ResetEndpoint / ResetDevice / stall notes) so the retry
// ladders are assertable without hardware. All three read paths consume the same script;
// an exhausted script serves full frames.
type scriptCtl struct {
	rm    *fakeRegmap
	rmO   Regmap // optional Rm() override (defaults to rm)
	fb    int
	reads []int // successive read byte counts; past the end every read is full

	idx         int // reads consumed so far
	lastReadLen int // len(buf) of the most recent read
	resetEP     int
	resetDev    int
	stalls      int
	lastQuiet   time.Duration // quiet passed to the most recent BulkReadQuiet
	abortAfter  int           // Aborted() reports true once idx >= abortAfter (0 = never abort)
	readDelay   time.Duration // StreamFrame blocks this long before serving (a slow readout)
	reapplied   int           // ReapplyOffset calls
}

func (s *scriptCtl) Rm() Regmap {
	if s.rmO != nil {
		return s.rmO
	}
	return s.rm
}
func (s *scriptCtl) VendorCmd(FX3Op) error { return nil }
func (s *scriptCtl) ResetEndpoint() error  { s.resetEP++; return nil }
func (s *scriptCtl) ResetDevice() error    { s.resetDev++; return nil }
func (s *scriptCtl) DrainPipe(time.Duration) int { return 0 }
func (s *scriptCtl) NoteStall()            { s.stalls++ }
func (s *scriptCtl) ReapplyOffset() error  { s.reapplied++; return nil }
func (s *scriptCtl) Aborted() bool         { return s.abortAfter > 0 && s.idx >= s.abortAfter }
func (s *scriptCtl) FrameBytes() int       { return s.fb }
func (s *scriptCtl) read(buf []byte) (int, error) {
	s.lastReadLen = len(buf)
	if s.idx >= len(s.reads) {
		s.idx++
		return len(buf), nil
	}
	n := s.reads[s.idx]
	s.idx++
	if n > len(buf) {
		n = len(buf)
	}
	return n, nil
}
func (s *scriptCtl) BulkRead(buf []byte, _ time.Duration) (int, error) { return s.read(buf) }
func (s *scriptCtl) BulkReadQuiet(buf []byte, quiet, _ time.Duration) (int, error) {
	s.lastQuiet = quiet
	return s.read(buf)
}
func (s *scriptCtl) StreamFrame(buf []byte, _, _ time.Duration) (int, error) {
	if s.readDelay > 0 {
		time.Sleep(s.readDelay)
	}
	return s.read(buf)
}
func (s *scriptCtl) StreamFramePrequeued(buf []byte, _, _ time.Duration) (int, error) {
	return s.read(buf)
}

// TestIMX462WorkerRetryShortFrame: a transient short read (>= 4 KiB, so not the "empty"
// rung) is retried after a ResetEndpoint and the next full frame succeeds.
func TestIMX462WorkerRetryShortFrame(t *testing.T) {
	const fb = 64 * 1024
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: []int{8192, fb}}
	n, err := imx462Worker(ctl, make([]byte, fb), 10*time.Millisecond)
	if err != nil || n != fb {
		t.Fatalf("worker: n=%d err=%v, want %d nil", n, err, fb)
	}
	if ctl.stalls != 1 {
		t.Errorf("stalls=%d, want 1 (one short read)", ctl.stalls)
	}
	if ctl.resetDev != 0 {
		t.Errorf("ResetDevice called %d times on a plain short read (only 4 empties escalate)", ctl.resetDev)
	}
}

// TestIMX462WorkerZerosResetDevice: four consecutive (near-)empty reads escalate to
// ResetDevice + full re-arm, then the read succeeds.
func TestIMX462WorkerZerosResetDevice(t *testing.T) {
	const fb = 64 * 1024
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: []int{0, 0, 0, 0, fb}}
	n, err := imx462Worker(ctl, make([]byte, fb), 10*time.Millisecond)
	if err != nil || n != fb {
		t.Fatalf("worker: n=%d err=%v, want %d nil", n, err, fb)
	}
	if ctl.resetDev != 1 {
		t.Errorf("ResetDevice called %d times, want exactly 1 (SDK: 4 empty reads)", ctl.resetDev)
	}
}

// TestIMX462WorkerShortBy512Tolerated: the SDK startAsyncXfer treats a frame short by
// exactly 512 bytes as complete; the worker zero-fills the missing tail.
func TestIMX462WorkerShortBy512Tolerated(t *testing.T) {
	const fb = 8192
	buf := make([]byte, fb)
	for i := range buf {
		buf[i] = 0xFF
	}
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: []int{fb - 512}}
	n, err := imx462Worker(ctl, buf, 10*time.Millisecond)
	if err != nil || n != fb {
		t.Fatalf("worker: n=%d err=%v, want %d nil", n, err, fb)
	}
	for i := fb - 512; i < fb; i++ {
		if buf[i] != 0 {
			t.Fatalf("buf[%d]=0x%x, want 0 (missing tail must be zeroed, not stale)", i, buf[i])
		}
	}
}

// TestIMX462WorkerAbortDuringRetry: an abort observed after a failed read bails out with
// the abort error instead of grinding through the remaining retry budget.
func TestIMX462WorkerAbortDuringRetry(t *testing.T) {
	const fb = 64 * 1024
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: []int{0, 0, 0, 0, 0, 0, 0, 0}, abortAfter: 1}
	_, err := imx462Worker(ctl, make([]byte, fb), 10*time.Millisecond)
	if err != errExposureAborted {
		t.Fatalf("worker err=%v, want errExposureAborted", err)
	}
	if ctl.idx > 2 {
		t.Errorf("worker consumed %d reads after the abort, want <= 2", ctl.idx)
	}
}

// TestIMX462WorkerGivesUp: persistent short frames exhaust the attempt cap and error out
// (the SDK snap reports ASI_EXP_FAILED rather than retrying forever).
func TestIMX462WorkerGivesUp(t *testing.T) {
	const fb = 64 * 1024
	reads := make([]int, 10)
	for i := range reads {
		reads[i] = 8192 // short but >= 4 KiB: the retry rung, never the ResetDevice rung
	}
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: reads}
	_, err := imx462Worker(ctl, make([]byte, fb), 10*time.Millisecond)
	if err == nil {
		t.Fatal("worker succeeded on persistent short frames, want an error after the attempt cap")
	}
	if ctl.resetDev != 0 {
		t.Errorf("ResetDevice called %d times for non-empty shorts, want 0", ctl.resetDev)
	}
}

// TestIMX462WorkerTriggerBandReintegrates: in the >= 1 s trigger band, a ResetDevice recovery
// re-runs the trigger cycle: the re-armed FPGA is back in wait mode, and without a fresh trigger
// edge (reg 0x0b bit0 on->off) the frame never arrives.
func TestIMX462WorkerTriggerBandReintegrates(t *testing.T) {
	if testing.Short() {
		t.Skip("two full 1 s integrations")
	}
	const fb = 8192
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: []int{0, 0, 0, 0, fb}}
	n, err := imx462Worker(ctl, make([]byte, fb), time.Second)
	if err != nil || n != fb {
		t.Fatalf("worker: n=%d err=%v, want %d nil", n, err, fb)
	}
	if ctl.resetDev != 1 {
		t.Fatalf("ResetDevice called %d times, want 1", ctl.resetDev)
	}
	trig := 0
	for _, w := range ctl.rm.fpgaWrites {
		if w.Reg == 0x0b {
			trig++
		}
	}
	if trig < 4 {
		t.Errorf("trigger reg 0x0b written %d times, want >= 4 (on+off for the initial "+
			"integration AND the post-ResetDevice re-integration)", trig)
	}
}

// TestIMX174WorkerAbortMapsCleanly: a read broken by StopExposure (the transport AbortRead
// returns a short prefix with a nil error) surfaces as errExposureAborted, not as a stall;
// GetDataAfterExp would otherwise mark the status FAILED and probe the firmware.
func TestIMX174WorkerAbortMapsCleanly(t *testing.T) {
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: 4096, reads: []int{0}, abortAfter: 1}
	_, err := imx174Worker(ctl, make([]byte, 4096), 50*time.Millisecond)
	if err != errExposureAborted {
		t.Fatalf("worker err = %v, want errExposureAborted", err)
	}
	if ctl.stalls != 0 {
		t.Errorf("stalls = %d, want 0 (an abort is not a stall)", ctl.stalls)
	}
}

// TestIMX290WorkerAbortMapsCleanly: same contract for the 290's post-integration read.
func TestIMX290WorkerAbortMapsCleanly(t *testing.T) {
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: 4096, reads: []int{0}, abortAfter: 1}
	_, err := imx290Worker(ctl, make([]byte, 4096), 50*time.Millisecond)
	if err != errExposureAborted {
		t.Fatalf("worker err = %v, want errExposureAborted", err)
	}
}

// TestIMX174WorkerQuietWindow asserts the quiet-window wiring: in the sensor-timed (<= 4 s)
// bands the read spans the integration, so the worker declares quiet = exposure-500 ms
// (clamped at 0) to BulkReadQuiet; the > 4 s trigger band integrates before the read, so its
// quiet is 0 (the read is pure readout and keeps the full wedge gate).
func TestIMX174WorkerQuietWindow(t *testing.T) {
	// 2 s cycle-count band: quiet = 1.5 s.
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: 4096}
	if _, err := imx174Worker(ctl, make([]byte, 4096), 2*time.Second); err != nil {
		t.Fatal(err)
	}
	if want := 1500 * time.Millisecond; ctl.lastQuiet != want {
		t.Errorf("2s exposure: quiet = %v, want %v", ctl.lastQuiet, want)
	}
	// 300 ms short band: the 500 ms undershoot clamps quiet to 0.
	ctl = &scriptCtl{rm: &fakeRegmap{}, fb: 4096}
	if _, err := imx174Worker(ctl, make([]byte, 4096), 300*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if ctl.lastQuiet != 0 {
		t.Errorf("300ms exposure: quiet = %v, want 0", ctl.lastQuiet)
	}
}

// reg23Regmap is a fakeRegmap whose FPGA reg 0x23 reads back bit2 set ("triggered frame
// still buffered"), steering a worker into its FPGABufReload rung.
type reg23Regmap struct{ fakeRegmap }

func (r *reg23Regmap) ReadFPGAReg(reg uint16) (uint16, error) {
	if reg == 0x23 {
		return 0x04, nil
	}
	return 0, nil
}

// TestIMX290WorkerRetryShortFrame: a transient short read (the observed
// tiny-ROI 960/38400 shape) is retried after a ResetEndpoint and the next full read succeeds.
func TestIMX290WorkerRetryShortFrame(t *testing.T) {
	const fb = 38400
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: []int{960, fb}}
	n, err := imx290Worker(ctl, make([]byte, fb), 10*time.Millisecond)
	if err != nil || n != fb {
		t.Fatalf("worker: n=%d err=%v, want %d nil", n, err, fb)
	}
	if ctl.stalls != 1 {
		t.Errorf("stalls=%d, want 1", ctl.stalls)
	}
	if ctl.resetDev != 0 {
		t.Errorf("ResetDevice called %d times on a plain short read, want 0", ctl.resetDev)
	}
}

// TestIMX290WorkerZerosResetDevice: the 4th consecutive zero read escalates to ResetDevice + the
// full re-arm, then the read succeeds.
func TestIMX290WorkerZerosResetDevice(t *testing.T) {
	const fb = 38400
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: []int{0, 0, 0, 0, fb}}
	n, err := imx290Worker(ctl, make([]byte, fb), 10*time.Millisecond)
	if err != nil || n != fb {
		t.Fatalf("worker: n=%d err=%v, want %d nil", n, err, fb)
	}
	if ctl.resetDev != 1 {
		t.Errorf("ResetDevice called %d times, want exactly 1 (object: 4 zero reads)", ctl.resetDev)
	}
}

// TestIMX290WorkerGivesUp: persistent short frames exhaust the snap-mode retry cap (SDK r14 > 2)
// and error out.
func TestIMX290WorkerGivesUp(t *testing.T) {
	const fb = 38400
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: []int{960, 960, 960, 960, 960}}
	_, err := imx290Worker(ctl, make([]byte, fb), 10*time.Millisecond)
	if err == nil {
		t.Fatal("worker succeeded on persistent short frames, want an error after the retry cap")
	}
	if ctl.idx != 3 {
		t.Errorf("worker consumed %d reads, want 3 (retries capped at >2)", ctl.idx)
	}
}

// TestIMX290WorkerZerosBounded: a pipe that never delivers (every read returns 0, as usbfs does
// on a deadline) fails the capture after imx290ReadAttempts reads instead of cycling ResetDevice
// forever.
func TestIMX290WorkerZerosBounded(t *testing.T) {
	const fb = 38400
	reads := make([]int, 40)
	ctl := &scriptCtl{rm: &fakeRegmap{}, fb: fb, reads: reads}
	_, err := imx290Worker(ctl, make([]byte, fb), 10*time.Millisecond)
	if err == nil {
		t.Fatal("worker succeeded on a dead pipe, want an error")
	}
	if ctl.idx != imx290ReadAttempts {
		t.Errorf("worker consumed %d reads, want %d", ctl.idx, imx290ReadAttempts)
	}
	if ctl.resetDev < 1 || ctl.resetDev > 3 {
		t.Errorf("ResetDevice called %d times, want 1..3", ctl.resetDev)
	}
}

// TestIMX290WorkerTriggerReload: a trigger-band short with FPGA reg 0x23 bit2 set reloads
// the buffered frame (FPGABufReload, reg 0x18 bit0) and re-reads WITHOUT re-integrating or
// consuming a retry.
func TestIMX290WorkerTriggerReload(t *testing.T) {
	if testing.Short() {
		t.Skip("one full 1 s integration")
	}
	const fb = 38400
	rm := &reg23Regmap{}
	ctl := &scriptCtl{rm: &rm.fakeRegmap, rmO: rm, fb: fb, reads: []int{960, fb}}
	n, err := imx290Worker(ctl, make([]byte, fb), time.Second)
	if err != nil || n != fb {
		t.Fatalf("worker: n=%d err=%v, want %d nil", n, err, fb)
	}
	reload := false
	for _, w := range rm.fpgaWrites {
		if w.Reg == 0x18 && w.Val&0x01 != 0 {
			reload = true
		}
	}
	if !reload {
		t.Error("no FPGABufReload (reg 0x18 bit0) write — the reload rung was not taken")
	}
	if ctl.resetDev != 0 {
		t.Errorf("ResetDevice called %d times, want 0", ctl.resetDev)
	}
}

// TestWorkerReadClamp: a caller buffer larger than the frame does not widen the wire read; the
// WorkingFuncs read only the frame byte count, and an oversized read runs into the next free-run
// frame or times out short.
func TestWorkerReadClamp(t *testing.T) {
	workers := []struct {
		name string
		w    func(WorkerCtl, []byte, time.Duration) (int, error)
	}{
		{"imx290", imx290Worker},
		{"imx178", imx178Worker},
	}
	for _, tc := range workers {
		ctl := &scriptCtl{rm: &fakeRegmap{}, fb: 1000}
		n, err := tc.w(ctl, make([]byte, 2000), 10*time.Millisecond)
		if err != nil {
			t.Fatalf("%s: %v", tc.name, err)
		}
		if n != 1000 {
			t.Errorf("%s: returned %d bytes, want 1000 (FrameBytes)", tc.name, n)
		}
		if ctl.lastReadLen != 1000 {
			t.Errorf("%s: read %d bytes off the wire, want exactly FrameBytes=1000", tc.name, ctl.lastReadLen)
		}
	}
}

// TestXHSStopRegister: the 178 and 585 workers gate the exposure window with EnableFPGAXHSStop
// (FPGA reg 0x0b bit4) alongside the trigger signal (reg 0x0b bit0); they never touch reg 0x0a,
// whose bit4 is the FX3 output-width select (SetFPGAADCWidthOutputWidth).
func TestXHSStopRegister(t *testing.T) {
	workers := []struct {
		name string
		w    func(WorkerCtl, []byte, time.Duration) (int, error)
	}{
		{"imx178", imx178Worker},
		{"imx585", imx585Worker},
	}
	for _, tc := range workers {
		ctl := &scriptCtl{rm: &fakeRegmap{}, fb: 4096}
		if _, err := tc.w(ctl, make([]byte, 4096), 10*time.Millisecond); err != nil {
			t.Fatalf("%s: %v", tc.name, err)
		}
		on, off := false, false
		for _, w := range ctl.rm.fpgaWrites {
			switch w.Reg {
			case 0x0a:
				t.Errorf("%s: worker wrote FPGA reg 0x0a (output width) = 0x%02x; XHSStop is reg 0x0b bit4", tc.name, w.Val)
			case 0x0b:
				if w.Val&0x10 != 0 {
					on = true
				} else if on {
					off = true
				}
			}
		}
		if !on || !off {
			t.Errorf("%s: XHSStop (reg 0x0b bit4) on=%v off=%v, want both", tc.name, on, off)
		}
	}
}

// TestDDRWorkersPulseBufReload: on a USB3 link the DDR workers (455, 571, 585) pulse
// FPGABufReload (reg 0x18 bit0) while the frame read is in flight, so the FX3 commits the
// frame's final partial 1-MiB buffer, and the ticker is joined before the worker returns; on a
// USB2 link they do not (the pulses wedge a USB2 readout, ASI6200MC wire finding).
func TestDDRWorkersPulseBufReload(t *testing.T) {
	workers := []struct {
		name string
		w    func(WorkerCtl, []byte, time.Duration) (int, error)
	}{
		{"imx455", imx455Worker},
		{"imx571", imx571Worker},
		{"imx585", imx585Worker},
	}
	for _, tc := range workers {
		for _, usb3 := range []bool{true, false} {
			rm := &modeRegmap{mode: ReadoutMode{USB3: usb3, BytesPerPx: 2}, regVals: map[uint16]uint16{}}
			ctl := &scriptCtl{rm: &rm.fakeRegmap, rmO: rm, fb: 4096, readDelay: 120 * time.Millisecond}
			if _, err := tc.w(ctl, make([]byte, 4096), 10*time.Millisecond); err != nil {
				t.Fatalf("%s usb3=%v: %v", tc.name, usb3, err)
			}
			pulses := 0
			for _, w := range rm.fpgaWrites {
				if w.Reg == 0x18 && w.Val&0x01 != 0 {
					pulses++
				}
			}
			if usb3 && pulses < 3 {
				t.Errorf("%s USB3: %d FPGABufReload pulses during a 120 ms read, want >= 3 (20 ms ticker)", tc.name, pulses)
			}
			if !usb3 && pulses != 0 {
				t.Errorf("%s USB2: %d FPGABufReload pulses, want 0 (ticker off on a USB2 link)", tc.name, pulses)
			}
		}
	}
}

// TestTriggerBandReadTimeoutIsBounded: in the trigger band the worker host-holds the whole
// integration itself, so by the time it reads, the frame is already exposed and sitting in the
// camera's buffer and only the wire transfer remains. A timeout of exposure+margin there scales
// a failure with the exposure instead of the transfer: on a dead pipe the IMX290 retries up to
// imx290ReadAttempts times, which at a 600 s sub is hours before the capture fails. The bound is
// a fixed constant, as it already is on the IMX462 (the SDK's own trigger-mode value) and the
// IMX585.
func TestTriggerBandReadTimeoutIsBounded(t *testing.T) {
	for _, tc := range []struct {
		name   string
		worker func(WorkerCtl, []byte, time.Duration) (int, error)
		want   time.Duration
	}{
		{"IMX290", imx290Worker, imx290TrigReadTO},
		{"IMX178", imx178Worker, imx178TrigReadTO},
	} {
		ctl := &fakeCtl{rm: &fakeRegmap{}, fb: 64}
		if _, err := tc.worker(ctl, make([]byte, 64), time.Second); err != nil {
			t.Fatalf("%s: %v", tc.name, err)
		}
		if ctl.lastTO != tc.want {
			t.Errorf("%s trigger-band read timeout = %v, want the fixed %v (not exposure-scaled)",
				tc.name, ctl.lastTO, tc.want)
		}
	}
}

// failWriteRegmap fails every register write past the first failAfter, so a test can break a
// worker's arm at any step of the sequence.
type failWriteRegmap struct {
	fakeRegmap
	failAfter int
	n         int
}

var errArmWrite = errors.New("injected register write failure")

func (r *failWriteRegmap) WriteReg(reg, val uint16) error {
	r.n++
	if r.n > r.failAfter {
		return errArmWrite
	}
	return r.fakeRegmap.WriteReg(reg, val)
}

func (r *failWriteRegmap) WriteRegBits(reg uint16, lo, hi uint8, val uint16) error {
	r.n++
	if r.n > r.failAfter {
		return errArmWrite
	}
	return r.fakeRegmap.WriteRegBits(reg, lo, hi, val)
}

func (r *failWriteRegmap) WriteFPGAReg(reg, val uint16) error {
	r.n++
	if r.n > r.failAfter {
		return errArmWrite
	}
	return r.fakeRegmap.WriteFPGAReg(reg, val)
}

// armFailCtl is a fakeCtl over a failWriteRegmap that counts endpoint resets. No worker's arm
// sequence resets the endpoint, so a reset can only have come from the halt.
type armFailCtl struct {
	fakeCtl
	rmap   *failWriteRegmap
	resets int
}

func (c *armFailCtl) Rm() Regmap           { return c.rmap }
func (c *armFailCtl) ResetEndpoint() error { c.resets++; return nil }

// TestWorkerHaltsAfterAFailedArm: a Worker that returns an error must leave the readout halted.
// The halt is a deferred call, so registering it only after the arm succeeds means a control
// transfer failing part-way through the arm returns with the sensor out of standby and the master
// gate open — free-running into a host that has stopped reading, which is the state the halt
// exists to prevent (a 174 left streaming backs up the FX3 until its firmware crashes).
func TestWorkerHaltsAfterAFailedArm(t *testing.T) {
	for _, tc := range []struct {
		name   string
		worker func(WorkerCtl, []byte, time.Duration) (int, error)
	}{
		{"IMX455", imx455Worker},
		{"IMX571", imx571Worker},
		{"IMX585", imx585Worker},
		{"IMX178", imx178Worker},
	} {
		failed := 0
		for cut := 0; cut < 24; cut++ {
			ctl := &armFailCtl{
				fakeCtl: fakeCtl{fb: 64},
				rmap:    &failWriteRegmap{failAfter: cut},
			}
			ctl.fakeCtl.rm = &ctl.rmap.fakeRegmap
			_, err := tc.worker(ctl, make([]byte, 64), 10*time.Millisecond)
			if err == nil {
				continue // the injected failure landed somewhere harmless
			}
			failed++
			if ctl.resets == 0 {
				t.Errorf("%s: a write failure at step %d returned %v with the readout never halted",
					tc.name, cut+1, err)
			}
		}
		if failed == 0 {
			t.Errorf("%s: no injected failure reached the worker; the test proves nothing", tc.name)
		}
	}
}

// TestIMX455WorkerDrainsBeforeArm asserts the worker empties the bulk pipe before it arms. The
// FX3 keeps committing DMA buffers until the previous capture's deferred halt lands, and
// ResetEndpoint discards none of them, so anything left heads this frame and shifts every pixel.
func TestIMX455WorkerDrainsBeforeArm(t *testing.T) {
	ctl := &fakeCtl{rm: &fakeRegmap{}, fb: 64}
	_, _ = imx455Worker(ctl, make([]byte, ctl.fb), time.Millisecond)
	if ctl.drained == 0 {
		t.Error("imx455Worker armed without draining the bulk pipe first")
	}
}
