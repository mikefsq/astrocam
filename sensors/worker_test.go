package sensors

import (
	"testing"
	"time"

	. "github.com/mikefsq/astrocam"
)

// fakeCtl is a minimal WorkerCtl for band/​timing tests: every read succeeds and returns a
// full buffer, no aborts, register writes land in the embedded fakeRegmap.
type fakeCtl struct {
	rm *fakeRegmap
	fb int
}

func (f *fakeCtl) Rm() Regmap                                        { return f.rm }
func (f *fakeCtl) VendorCmd(uint8) error                             { return nil }
func (f *fakeCtl) ResetEndpoint() error                              { return nil }
func (f *fakeCtl) ResetDevice() error                                { return nil }
func (f *fakeCtl) NoteStall()                                        {}
func (f *fakeCtl) Aborted() bool                                     { return false }
func (f *fakeCtl) FrameBytes() int                                   { return f.fb }
func (f *fakeCtl) BulkRead(buf []byte, _ time.Duration) (int, error) { return len(buf), nil }
func (f *fakeCtl) BulkReadQuiet(buf []byte, _, _ time.Duration) (int, error) {
	return len(buf), nil
}
func (f *fakeCtl) StreamFrame(buf []byte, _, _ time.Duration) (int, error) {
	return len(buf), nil
}
func (f *fakeCtl) StreamFramePrequeued(buf []byte, _, _ time.Duration) (int, error) {
	return len(buf), nil
}

// exactly1sBoundary runs a worker at EXACTLY the 1 s trigger threshold and asserts the host
// hold spans the full exposure. SetExposure engages FPGA trigger mode at >= 1 s, where the
// host trigger window IS the integration — a worker band predicate of `<= 1s` (instead of
// `< 1s`) put the exactly-1 s case in the free-run band's exposure−200 ms sleep, silently
// under-integrating it by 20%.
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
}

func (s *scriptCtl) Rm() Regmap {
	if s.rmO != nil {
		return s.rmO
	}
	return s.rm
}
func (s *scriptCtl) VendorCmd(uint8) error { return nil }
func (s *scriptCtl) ResetEndpoint() error  { s.resetEP++; return nil }
func (s *scriptCtl) ResetDevice() error    { s.resetDev++; return nil }
func (s *scriptCtl) NoteStall()            { s.stalls++ }
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
// EXACTLY 512 bytes as complete; the worker zero-fills the missing tail.
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

// TestIMX462WorkerTriggerBandReintegrates locks finding 2.8: in the >= 1 s trigger band, a
// ResetDevice recovery must re-run the trigger cycle — the re-armed FPGA is back in wait
// mode, and without a fresh trigger edge (reg 0x0b bit0 on->off) the frame can never arrive.
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
// returns a short prefix with a nil error) must surface as errExposureAborted, not as a
// stall — GetDataAfterExp would otherwise mark the status FAILED and probe the firmware.
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

// TestIMX174WorkerQuietWindow pins the finding-3.5 wiring: in the sensor-timed (≤4 s)
// bands the read spans the integration, so the worker must declare quiet = exposure−500 ms
// (clamped at 0) to BulkReadQuiet; the >4 s trigger band integrates BEFORE the read, so its
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

// TestIMX290WorkerRetryShortFrame locks the 2.13 fix: a transient short read (the observed
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

// TestIMX290WorkerZerosResetDevice: the object's 4th consecutive zero read escalates to
// ResetDevice + the full re-arm, then the read succeeds.
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

// TestIMX290WorkerGivesUp: persistent short frames exhaust the object's snap-mode retry
// cap (r14 > 2) and error out.
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

// TestWorkerReadClamp locks finding 2.11: a caller buffer larger than the frame must not
// widen the wire read — the objects' WorkingFuncs read EXACTLY the frame byte count, and an
// oversized read runs into the next free-run frame or times out short.
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
