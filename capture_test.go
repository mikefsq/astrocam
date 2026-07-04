package astrocam

import (
	"encoding/binary"
	"errors"
	"sync"
	"testing"
	"time"
)

// frameTransport records control writes (the arm sequence) and serves one
// magic-prefixed frame from its bulk endpoint.
type frameTransport struct {
	out   []ctrlCall
	frame []byte
}

func (f *frameTransport) ControlOut(b uint8, wv, wi uint16, _ []byte) error {
	f.out = append(f.out, ctrlCall{b, wv, wi})
	return nil
}
func (f *frameTransport) ControlIn(_ uint8, _, _ uint16, data []byte) (int, error) {
	return len(data), nil // real transports fill the request; a 0-count read is now an error
}
func (f *frameTransport) BulkRead(buf []byte, _ time.Duration) (int, error) {
	return copy(buf, f.frame), nil
}
func (f *frameTransport) Close() error { return nil }

// armSensor is a tiny in-package profile exercising the snap arm + readout.
var armSensor = Sensor{
	Name:        "ARM",
	Info:        CameraInfo{MaxWidth: 4, MaxHeight: 2, BitDepth: 12}, // 4*2*2 = 16-byte frame
	SetExposure: func(Regmap, time.Duration) error { return nil },
	SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
	StreamStop:  func(rm Regmap) error { return rm.WriteReg(0x200, 1) },
	StreamStart: func(rm Regmap) error { return rm.WriteReg(0x200, 0) },
}

func TestSnapDataPlane(t *testing.T) {
	// A 16-byte frame prefixed with the header magic.
	frame := make([]byte, 4+16)
	binary.LittleEndian.PutUint32(frame[:4], FrameMagic)
	for i := 4; i < len(frame); i++ {
		frame[i] = byte(i)
	}
	f := &frameTransport{frame: frame}
	Register(ZWO.VID, 0x00D0, Model{Name: "Snap", Sensor: &armSensor})
	c, err := Open(f, ZWO.VID, 0x00D0)
	if err != nil {
		t.Fatal(err)
	}
	c.SetExposure(10 * time.Millisecond)

	// Drive a fake clock so the host-timed status poll is deterministic.
	base := time.Unix(1000, 0)
	now := base
	nowFunc = func() time.Time { return now }
	defer func() { nowFunc = time.Now }()

	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	// Arm sequence: 0xAA, FPGAStop, WriteReg(0x200,1), 0xA9, WriteReg(0x200,0), FPGAStart.
	want := []ctrlCall{
		{cmdStreamStop, 0, 0},
		{reqWriteFPGAReg, fpgaModeReg0, fpgaStopBit}, // FPGAStop (reg0 bit4 set)
		{reqWriteSonyReg, 0x200, 1},
		{cmdStreamStart, 0, 0},
		{reqWriteSonyReg, 0x200, 0},
		{reqWriteFPGAReg, fpgaModeReg0, 0}, // FPGAStart (reg0 bit4 clear)
	}
	if len(f.out) != len(want) {
		t.Fatalf("arm produced %d transfers, want %d: %+v", len(f.out), len(want), f.out)
	}
	for i := range want {
		if f.out[i] != want[i] {
			t.Errorf("arm[%d] = %+v, want %+v", i, f.out[i], want[i])
		}
	}

	// After the arm the sensor is WORKING; the read happens immediately (the bulk
	// read blocks for the frame) rather than host-waiting the exposure first.
	if s := c.GetExpStatus(); s != ExpWorking {
		t.Errorf("status after arm = %s, want working", s)
	}
	buf := make([]byte, 64)
	n, err := c.GetDataAfterExp(buf)
	if err != nil {
		t.Fatal(err)
	}
	if n != len(frame) {
		t.Errorf("read %d bytes, want %d", n, len(frame))
	}
	if c.GetExpStatus() != ExpIdle {
		t.Error("status should return to idle after consuming the frame")
	}
}

// dispatchTransport is a bare Transport (BulkRead only) that records which read path the
// Camera dispatched to; the wrapper types below add FrameStreamer / PrequeuedFrameStreamer
// so a test can lock the StreamFramePrequeued capability-dispatch order.
type dispatchTransport struct {
	path string
}

func (d *dispatchTransport) ControlOut(uint8, uint16, uint16, []byte) error       { return nil }
func (d *dispatchTransport) ControlIn(_ uint8, _, _ uint16, data []byte) (int, error) {
	return len(data), nil
}
func (d *dispatchTransport) Close() error                                         { return nil }
func (d *dispatchTransport) BulkRead(buf []byte, _ time.Duration) (int, error) {
	d.path = "bulk"
	return len(buf), nil
}

type fsDispatchTransport struct{ dispatchTransport }

func (d *fsDispatchTransport) ReadFrameStream(buf []byte, _, _ time.Duration) (int, error) {
	d.path = "stream"
	return len(buf), nil
}

type pqDispatchTransport struct{ fsDispatchTransport }

func (d *pqDispatchTransport) ReadFrameStreamPrequeued(buf []byte, _, _ time.Duration) (int, error) {
	d.path = "prequeued"
	return len(buf), nil
}

// TestStreamFramePrequeuedDispatch locks the capability-dispatch order of the worker read
// primitives: StreamFramePrequeued uses the pre-queued batch when the backend has it, falls
// back to the windowed pump, then to a plain BulkRead (finding 2.4: the free-run 462 worker
// depends on the pre-queued path whenever the backend offers it; all in-tree backends now do).
func TestStreamFramePrequeuedDispatch(t *testing.T) {
	buf := make([]byte, 32)
	cases := []struct {
		name      string
		t         Transport
		path      func(Transport) string
		prequeued string // path StreamFramePrequeued must take
		stream    string // path StreamFrame must take
	}{
		{"prequeued backend", &pqDispatchTransport{},
			func(tr Transport) string { return tr.(*pqDispatchTransport).path }, "prequeued", "stream"},
		{"windowed-pump backend", &fsDispatchTransport{},
			func(tr Transport) string { return tr.(*fsDispatchTransport).path }, "stream", "stream"},
		{"bulk-only backend", &dispatchTransport{},
			func(tr Transport) string { return tr.(*dispatchTransport).path }, "bulk", "bulk"},
	}
	for _, tc := range cases {
		c := &Camera{t: tc.t}
		if _, err := c.StreamFramePrequeued(buf, time.Second, time.Second); err != nil {
			t.Fatalf("%s: StreamFramePrequeued: %v", tc.name, err)
		}
		if got := tc.path(tc.t); got != tc.prequeued {
			t.Errorf("%s: StreamFramePrequeued took %q, want %q", tc.name, got, tc.prequeued)
		}
		if _, err := c.StreamFrame(buf, time.Second, time.Second); err != nil {
			t.Fatalf("%s: StreamFrame: %v", tc.name, err)
		}
		if got := tc.path(tc.t); got != tc.stream {
			t.Errorf("%s: StreamFrame took %q, want %q", tc.name, got, tc.stream)
		}
	}
}

// TestStubImplementsPrequeued pins finding 2.4's fix: every in-tree backend the free-run
// worker can land on must offer the pre-queued read (the StreamFrame fallback idles out
// from t0 and breaks free-run exposures ≳ the idle window). Linux/darwin/windows are
// build-tagged; the stub is assertable everywhere.
func TestStubImplementsPrequeued(t *testing.T) {
	var tr Transport = NewStubTransport()
	if _, ok := tr.(PrequeuedFrameStreamer); !ok {
		t.Error("StubTransport must implement PrequeuedFrameStreamer")
	}
}

// TestCameraConcurrentAccess drives one Camera the way the Alpaca server does: several
// goroutines at once — a capture loop (StartExposure → the post-exposure data read), an independent
// ImageReady poller (GetExpStatus), an aborter (StopExposure), and config writers
// (SetExposure / SetROI / FrameBytes) all hitting the same camera. Run under -race this is
// the regression guard for the capture-state mutex and the transport's control serialization
// (think PHD2 polling the guide cam while a long exposure runs). Errors are ignored — only
// data races fail the test.
func TestCameraConcurrentAccess(t *testing.T) {
	st := NewStubTransport()
	Register(ZWO.VID, 0x00C0, Model{Name: "Conc", Sensor: &armSensor})
	c, err := Open(st, ZWO.VID, 0x00C0)
	if err != nil {
		t.Fatal(err)
	}
	c.SetROI(0, 0, 4, 2)
	c.SetExposure(0)

	stop := make(chan struct{})
	var wg sync.WaitGroup
	launch := func(fn func()) {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for {
				select {
				case <-stop:
					return
				default:
					fn()
				}
			}
		}()
	}

	launch(func() {
		c.StartExposure(true)
		c.GetDataAfterExp(make([]byte, 64))
	})
	launch(func() { _ = c.GetExpStatus() })
	launch(func() { _ = c.StopExposure() })
	launch(func() { _ = c.SetExposure(time.Millisecond) })
	launch(func() { _ = c.SetROI(0, 0, 4, 2) })
	launch(func() { _ = c.FrameBytes() })

	time.Sleep(100 * time.Millisecond)
	close(stop)
	wg.Wait()
}

// TestSnapShortFrame: a short frame (fewer bytes than FrameBytes — validation is by SIZE,
// there is no magic check on the pixel data) fails readout and marks the exposure FAILED
// (the dropped-frame path).
func TestSnapShortFrame(t *testing.T) {
	f := &frameTransport{frame: []byte{0xde, 0xad, 0xbe, 0xef, 1, 2, 3, 4}} // 8 < 16-byte frame
	c, _ := Open(f, ZWO.VID, 0x00D0)
	c.SetExposure(0)
	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	if _, err := c.GetDataAfterExp(make([]byte, 64)); err == nil {
		t.Error("expected short-frame error")
	}
	if c.GetExpStatus() != ExpFailed {
		t.Errorf("status = %s, want failed", c.GetExpStatus())
	}
}

// TestAbortSupersede locks the exposure-generation guard: a capture aborted mid-flight and
// superseded by a NEW StartExposure must (a) observe the abort through its WorkerCtl, and
// (b) leave the new exposure's status untouched when it unwinds — the historical bug was the
// stale worker seeing status==Working again ("un-aborting" itself) and then consuming or
// failing the new exposure's state.
func TestAbortSupersede(t *testing.T) {
	started := make(chan struct{})
	release := make(chan struct{})
	var sawAbort bool
	var mu sync.Mutex
	wrk := Sensor{
		Name:        "WRK",
		Info:        CameraInfo{MaxWidth: 4, MaxHeight: 2, BitDepth: 12},
		SetExposure: func(Regmap, time.Duration) error { return nil },
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		Worker: func(ctl WorkerCtl, buf []byte, _ time.Duration) (int, error) {
			close(started)
			<-release
			if ctl.Aborted() {
				mu.Lock()
				sawAbort = true
				mu.Unlock()
				return 0, errors.New("exposure aborted")
			}
			return ctl.FrameBytes(), nil
		},
	}
	f := &frameTransport{}
	Register(ZWO.VID, 0x00D1, Model{Name: "Wrk", Sensor: &wrk})
	c, err := Open(f, ZWO.VID, 0x00D1)
	if err != nil {
		t.Fatal(err)
	}
	c.SetExposure(10 * time.Second) // long window: the derived-Success poll stays Working

	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	done := make(chan error, 1)
	go func() {
		_, err := c.GetDataAfterExp(make([]byte, 64))
		done <- err
	}()
	<-started // stale capture is inside its worker

	if err := c.StopExposure(); err != nil {
		t.Fatal(err)
	}
	if err := c.StartExposure(true); err != nil { // supersede with a fresh exposure
		t.Fatal(err)
	}
	close(release) // let the stale worker unwind

	if err := <-done; err == nil {
		t.Error("stale aborted capture returned success, want abort error")
	}
	mu.Lock()
	if !sawAbort {
		t.Error("stale worker did not observe the abort (Aborted() lost across the new StartExposure)")
	}
	mu.Unlock()
	if s := c.GetExpStatus(); s != ExpWorking {
		t.Errorf("new exposure status = %s, want working — the stale capture clobbered it", s)
	}
}

// TestStatusPollDoesNotAbortWorker locks GetExpStatus purity: polling the status past the
// host-timed window derives SUCCESS without mutating the stored status, so the worker still
// integrating must NOT observe an abort. (The historical bug: the getter stored ExpSuccess and
// Aborted() keyed off status != Working, so any poll in the arm-lag tail killed the exposure
// at ~99% integrated.)
func TestStatusPollDoesNotAbortWorker(t *testing.T) {
	workerDone := make(chan struct{})
	var abortedDuringRun bool
	var mu sync.Mutex
	wrk := Sensor{
		Name:        "WRKPOLL",
		Info:        CameraInfo{MaxWidth: 4, MaxHeight: 2, BitDepth: 12},
		SetExposure: func(Regmap, time.Duration) error { return nil },
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		Worker: func(ctl WorkerCtl, buf []byte, _ time.Duration) (int, error) {
			defer close(workerDone)
			// Integrate PAST the host-timed window (the arm-lag tail), polling the abort
			// signal the way real workers do.
			for end := time.Now().Add(120 * time.Millisecond); time.Now().Before(end); {
				if ctl.Aborted() {
					mu.Lock()
					abortedDuringRun = true
					mu.Unlock()
					return 0, errors.New("exposure aborted")
				}
				time.Sleep(5 * time.Millisecond)
			}
			return ctl.FrameBytes(), nil
		},
	}
	f := &frameTransport{}
	Register(ZWO.VID, 0x00D2, Model{Name: "WrkPoll", Sensor: &wrk})
	c, err := Open(f, ZWO.VID, 0x00D2)
	if err != nil {
		t.Fatal(err)
	}
	c.SetExposure(30 * time.Millisecond) // window elapses well before the worker finishes

	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	done := make(chan error, 1)
	go func() {
		_, err := c.GetDataAfterExp(make([]byte, 64))
		done <- err
	}()
	// Poll like an Alpaca ImageReady loop: after the window the DERIVED status is Success.
	sawSuccess := false
	for i := 0; i < 20; i++ {
		if c.GetExpStatus() == ExpSuccess {
			sawSuccess = true
		}
		time.Sleep(10 * time.Millisecond)
	}
	<-workerDone
	if err := <-done; err != nil {
		t.Errorf("worker exposure failed under status polling: %v", err)
	}
	mu.Lock()
	if abortedDuringRun {
		t.Error("status poll aborted the in-flight worker (GetExpStatus mutated status)")
	}
	mu.Unlock()
	if !sawSuccess {
		t.Error("GetExpStatus never derived SUCCESS after the host-timed window")
	}
}
