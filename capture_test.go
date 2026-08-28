package astrocam

import (
	"errors"
	"strings"
	"sync"
	"testing"
	"time"
)

// frameTransport records control writes (the arm sequence) and serves one frame from its
// bulk endpoint.
type frameTransport struct {
	out   []ctrlCall
	frame []byte
}

func (f *frameTransport) ControlOut(b uint8, wv, wi uint16, _ []byte) error {
	f.out = append(f.out, ctrlCall{b, wv, wi})
	return nil
}
func (f *frameTransport) ControlIn(_ uint8, _, _ uint16, data []byte) (int, error) {
	return len(data), nil // real transports fill the request; a 0-count read is an error
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
	// One 16-byte frame (4×2 RAW16), validated by size.
	frame := make([]byte, 16)
	for i := range frame {
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

	// After the arm the sensor is Working; the read happens immediately (the bulk read blocks
	// for the frame) rather than host-waiting the exposure first.
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

// dispatchTransport is a bare Transport (BulkRead only) that records which read path the Camera
// dispatched to; the wrapper types below add FrameStreamer / PrequeuedFrameStreamer.
type dispatchTransport struct {
	path string
}

func (d *dispatchTransport) ControlOut(uint8, uint16, uint16, []byte) error { return nil }
func (d *dispatchTransport) ControlIn(_ uint8, _, _ uint16, data []byte) (int, error) {
	return len(data), nil
}
func (d *dispatchTransport) Close() error { return nil }
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

// TestStreamFramePrequeuedDispatch: StreamFramePrequeued uses the pre-queued batch when the
// backend has it, falls back to the windowed pump, then to a plain BulkRead.
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

// TestStubImplementsPrequeued: StubTransport offers the pre-queued read (the StreamFrame
// fallback idles out from t0 and breaks free-run exposures longer than the idle window). The
// hardware backends are build-tagged; the stub is assertable everywhere.
func TestStubImplementsPrequeued(t *testing.T) {
	var tr Transport = NewStubTransport()
	if _, ok := tr.(PrequeuedFrameStreamer); !ok {
		t.Error("StubTransport must implement PrequeuedFrameStreamer")
	}
}

// TestCameraConcurrentAccess drives one Camera from several goroutines at once: a capture loop
// (StartExposure → GetDataAfterExp), a status poller (GetExpStatus), an aborter (StopExposure),
// and config writers (SetExposure / SetROI / FrameBytes). Under -race it guards the
// capture-state mutex. Errors are ignored; only data races fail the test.
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

// TestSnapShortFrame: a frame shorter than FrameBytes (validation is by size, not a header)
// fails readout and marks the exposure Failed.
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

// TestAbortSupersede: a capture aborted mid-flight and superseded by a new StartExposure
// observes the abort through its WorkerCtl and leaves the new exposure's status untouched when
// it unwinds.
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

// TestStatusPollDoesNotAbortWorker: polling GetExpStatus past the host-timed window derives
// Success without mutating the stored status, so a worker still integrating does not observe an
// abort.
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
			// Integrate past the host-timed window, polling the abort signal as real workers do.
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
	// After the window the derived status is Success.
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

// quietDispatchTransport adds QuietBulkReader on top of the bare dispatcher.
type quietDispatchTransport struct{ dispatchTransport }

func (d *quietDispatchTransport) BulkReadQuiet(buf []byte, _, _ time.Duration) (int, error) {
	d.path = "quiet"
	return len(buf), nil
}

// TestBulkReadQuietDispatch: BulkReadQuiet routes to the transport's QuietBulkReader when
// offered and falls back to BulkRead otherwise.
func TestBulkReadQuietDispatch(t *testing.T) {
	buf := make([]byte, 32)
	q := &quietDispatchTransport{}
	c := &Camera{t: q}
	if _, err := c.BulkReadQuiet(buf, time.Second, 2*time.Second); err != nil {
		t.Fatal(err)
	}
	if q.path != "quiet" {
		t.Errorf("quiet backend took %q, want \"quiet\"", q.path)
	}
	b := &dispatchTransport{}
	c = &Camera{t: b}
	if _, err := c.BulkReadQuiet(buf, time.Second, 2*time.Second); err != nil {
		t.Fatal(err)
	}
	if b.path != "bulk" {
		t.Errorf("bulk-only backend took %q, want \"bulk\" (fallback)", b.path)
	}
}

// abortableTransport blocks BulkRead until AbortRead (or the read timeout) and counts the
// ReadAborter calls.
type abortableTransport struct {
	mu         sync.Mutex
	aborted    chan struct{}
	abortCalls int
	armCalls   int
}

func newAbortableTransport() *abortableTransport {
	return &abortableTransport{aborted: make(chan struct{})}
}
func (a *abortableTransport) ControlOut(uint8, uint16, uint16, []byte) error { return nil }
func (a *abortableTransport) ControlIn(_ uint8, _, _ uint16, data []byte) (int, error) {
	return len(data), nil
}
func (a *abortableTransport) Close() error { return nil }
func (a *abortableTransport) BulkRead(buf []byte, to time.Duration) (int, error) {
	a.mu.Lock()
	ch := a.aborted
	a.mu.Unlock()
	select {
	case <-ch:
		return 0, nil // aborted: short prefix, like the real backends
	case <-time.After(to):
		return len(buf), nil
	}
}
func (a *abortableTransport) AbortRead() {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.abortCalls++
	select {
	case <-a.aborted:
	default:
		close(a.aborted)
	}
}
func (a *abortableTransport) ArmRead() {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.armCalls++
	select {
	case <-a.aborted:
		a.aborted = make(chan struct{})
	default:
	}
}

// TestStopExposureBreaksBlockedRead: a GetDataAfterExp blocked in a multi-second transport read
// returns promptly after StopExposure (via ReadAborter) with an abort error, and StartExposure /
// StopExposure make one ArmRead / AbortRead call each.
func TestStopExposureBreaksBlockedRead(t *testing.T) {
	at := newAbortableTransport()
	Register(ZWO.VID, 0x00D3, Model{Name: "Abort", Sensor: &armSensor})
	c, err := Open(at, ZWO.VID, 0x00D3)
	if err != nil {
		t.Fatal(err)
	}
	c.SetExposure(10 * time.Millisecond)
	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	if at.armCalls != 1 {
		t.Errorf("StartExposure made %d ArmRead calls, want 1", at.armCalls)
	}
	done := make(chan error, 1)
	go func() {
		// The generic read path's timeout is 2·exp+3 s ≈ 3 s per attempt.
		_, err := c.GetDataAfterExp(make([]byte, c.FrameBytes()))
		done <- err
	}()
	time.Sleep(50 * time.Millisecond) // let the read block in the transport
	t0 := time.Now()
	if err := c.StopExposure(); err != nil {
		t.Fatal(err)
	}
	select {
	case err := <-done:
		if err == nil || !strings.Contains(err.Error(), "abort") {
			t.Errorf("GetDataAfterExp = %v, want an abort error", err)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("GetDataAfterExp still blocked 2 s after StopExposure — abort was not prompt")
	}
	if el := time.Since(t0); el > time.Second {
		t.Errorf("read released %v after StopExposure, want ~immediate", el)
	}
	if at.abortCalls != 1 {
		t.Errorf("StopExposure made %d AbortRead calls, want 1", at.abortCalls)
	}
}

// TestStubImplementsAbortAndQuiet: StubTransport implements ReadAborter and QuietBulkReader,
// and its abort latch works.
func TestStubImplementsAbortAndQuiet(t *testing.T) {
	var tr Transport = NewStubTransport()
	if _, ok := tr.(ReadAborter); !ok {
		t.Error("StubTransport does not implement ReadAborter")
	}
	if _, ok := tr.(QuietBulkReader); !ok {
		t.Error("StubTransport does not implement QuietBulkReader")
	}
	st := tr.(*StubTransport)
	st.AbortRead()
	if n, err := st.BulkRead(make([]byte, 8), time.Second); n != 0 || err != nil {
		t.Errorf("aborted stub BulkRead = (%d, %v), want (0, nil)", n, err)
	}
	st.ArmRead()
	if n, _ := st.BulkRead(make([]byte, 8), time.Second); n != 8 {
		t.Errorf("re-armed stub BulkRead = %d bytes, want 8", n)
	}
}

// TestStartVideoUsesSensorArm: StartVideo runs a profile's own Arm hook instead of the generic
// SendCMD/FPGA/master sequence.
func TestStartVideoUsesSensorArm(t *testing.T) {
	calls := 0
	s := Sensor{
		Name:        "ARMHOOK",
		Info:        CameraInfo{MaxWidth: 4, MaxHeight: 2, BitDepth: 12},
		SetExposure: func(Regmap, time.Duration) error { return nil },
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		Arm:         func(ctl WorkerCtl) error { calls++; return ctl.Rm().WriteReg(0x19e, 1) },
	}
	f := &frameTransport{}
	Register(ZWO.VID, 0x00D4, Model{Name: "ArmHook", Sensor: &s})
	c, err := Open(f, ZWO.VID, 0x00D4)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.StartVideo(true); err != nil {
		t.Fatal(err)
	}
	if calls != 1 {
		t.Fatalf("Sensor.Arm called %d times by StartVideo, want 1", calls)
	}
	// The generic arm's SendCMD stop/start must not have been issued alongside the hook.
	for _, x := range f.out {
		if x.bRequest == cmdStreamStop || x.bRequest == cmdStreamStart {
			t.Fatalf("generic arm command 0x%02x issued despite Sensor.Arm: %+v", x.bRequest, f.out)
		}
	}
}

// TestSumBinRAW8 checks the RAW8 software bin: block sum clipped at 255.
func TestSumBinRAW8(t *testing.T) {
	// 4×2 frame, bin 2 → 2×1: blocks {1,2,3,4}=10 and {100,100,100,100}=400 → 255.
	buf := []byte{1, 2, 100, 100, 3, 4, 100, 100}
	if n := sumBinRAW8(buf, 4, 2, 2); n != 2 || buf[0] != 10 || buf[1] != 255 {
		t.Errorf("sumBinRAW8 = n %d, out %v, want 2, [10 255]", n, buf[:2])
	}
}

// TestBinSplit: binning is host-side by default (the sensor SetROI sees the bin-scaled region at
// bin 1, FrameBytes is the wire size, the delivered RAW8 frame is the clipped block sum); with
// SetHardwareBin(true) the sensor takes the largest HWBins divisor and the host bins the rest
// (bin 4 on a {2,3} profile = sensor 2 × host 2; bin 3 = sensor 3 alone).
func TestBinSplit(t *testing.T) {
	var gotBin, gotW, gotH int
	s := &Sensor{
		Name:        "BSPL",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 48, BitDepth: 12, Bins: []int{1, 2, 3, 4}},
		HWBins:      []int{2, 3},
		SetROI:      func(_ Regmap, x, y, w, h, bin int) error { gotW, gotH, gotBin = w, h, bin; return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
	}
	Register(ZWO.VID, 0x0CE8, Model{Name: "BinSplit", Sensor: s})
	st := NewStubTransport()
	st.Frame = func(buf []byte) {
		for i := range buf {
			buf[i] = 100 // every RAW8 pixel 100 → 2×2 block sum 400 → 255
		}
	}
	c, err := Open(st, ZWO.VID, 0x0CE8)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetOutputDepth(1); err != nil {
		t.Fatal(err)
	}
	set := func(bin, w, h int) {
		t.Helper()
		if err := c.SetBinning(bin); err != nil {
			t.Fatal(err)
		}
		if err := c.SetROI(0, 0, w, h); err != nil {
			t.Fatal(err)
		}
	}
	// Default: host bin. bin 2 over 48×24 → sensor 96×48 at bin 1, wire 4608 B, delivered 1152 B
	// of 255.
	set(2, 48, 24)
	if gotBin != 1 || gotW != 96 || gotH != 48 {
		t.Errorf("host bin 2: sensor SetROI got %dx%d bin %d, want 96x48 bin 1", gotW, gotH, gotBin)
	}
	if fb := c.FrameBytes(); fb != 4608 {
		t.Errorf("host bin 2: FrameBytes = %d, want 4608 (96×48×1 wire)", fb)
	}
	buf := make([]byte, c.FrameBytes())
	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	n, err := c.GetDataAfterExp(buf)
	if err != nil || n != 1152 {
		t.Fatalf("host bin 2: GetDataAfterExp = %d, %v, want 1152 bytes (48×24 RAW8)", n, err)
	}
	for i := 0; i < n; i++ {
		if buf[i] != 255 {
			t.Fatalf("host bin 2: pixel %d = %d, want 255 (clipped sum of four 100s)", i, buf[i])
		}
	}
	// Hardware bin on: bin 2 goes to the sensor whole; bin 4 = sensor 2 over 2w×2h + host 2;
	// bin 3 = sensor 3 alone.
	c.SetHardwareBin(true)
	set(2, 48, 24)
	if gotBin != 2 || gotW != 48 || gotH != 24 || c.FrameBytes() != 1152 {
		t.Errorf("hw bin 2: sensor got %dx%d bin %d, FrameBytes %d; want 48x24 bin 2, 1152", gotW, gotH, gotBin, c.FrameBytes())
	}
	set(4, 24, 12)
	if gotBin != 2 || gotW != 48 || gotH != 24 {
		t.Errorf("hw bin 4: sensor got %dx%d bin %d, want 48x24 bin 2 (bin-2 table over 2w×2h)", gotW, gotH, gotBin)
	}
	if fb := c.FrameBytes(); fb != 1152 {
		t.Errorf("hw bin 4: FrameBytes = %d, want 1152 (48×24 wire, host-binned 2× to 24×12)", fb)
	}
	if m := ModeOf(c.rm); m.Bin != 2 || m.SoftBin != 2 || m.Width != 48 || m.Height != 24 {
		t.Errorf("hw bin 4: mode Bin %d SoftBin %d %dx%d, want 2/2 48x24", m.Bin, m.SoftBin, m.Width, m.Height)
	}
	set(3, 32, 16)
	if gotBin != 3 || gotW != 32 || gotH != 16 || ModeOf(c.rm).SoftBin != 1 {
		t.Errorf("hw bin 3: sensor got %dx%d bin %d SoftBin %d, want 32x16 bin 3 / 1", gotW, gotH, gotBin, ModeOf(c.rm).SoftBin)
	}
	// When the sensor bins, the rule counts output pixels: a 20-wide window is refused though its
	// 40-wide sensor extent would satisfy the host-bin form. ASISetROIFormat behaves the same way
	// (4788×3194 at bin 2 is accepted host-binned and rejected under ASI_HARDWARE_BIN).
	if err := c.SetROI(0, 0, 20, 6); err == nil {
		t.Error("hw bin 2 20x6 accepted; want rejected (20 is not a multiple of 8)")
	}
	// Back to host bin: bin 3 → sensor 96×48 at bin 1.
	c.SetHardwareBin(false)
	set(3, 32, 16)
	if gotBin != 1 || gotW != 96 || gotH != 48 {
		t.Errorf("host bin 3: sensor got %dx%d bin %d, want 96x48 bin 1", gotW, gotH, gotBin)
	}
}

// TestColorBinRule pins the color host bin to the SDK's mapping: each 2·bin × 2·bin block of the
// mosaic gives one 2×2 output cell whose pixels combine the bin² same-color samples (RAW16 mean,
// RAW8 clipped sum), so the output keeps the RGGB phase and its level per phase.
func TestColorBinRule(t *testing.T) {
	// 8×8 RAW8 mosaic with per-phase constants R=1 G=2 G=3 B=4 (rows even/odd × cols even/odd).
	const w, h = 8, 8
	mk := func() []byte {
		b := make([]byte, w*h)
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				b[y*w+x] = byte(1 + (y&1)*2 + (x & 1))
			}
		}
		return b
	}
	buf := mk()
	if n := colorSumBinRAW8(buf, w, h, 2); n != 16 {
		t.Fatalf("colorSumBinRAW8 bin2 n = %d, want 16", n)
	}
	// bin 2: 4×4 block → 2×2 cell, each = 4 same-color samples → 4×phase.
	for oy := 0; oy < 4; oy++ {
		for ox := 0; ox < 4; ox++ {
			want := byte(4 * (1 + (oy&1)*2 + (ox & 1)))
			if got := buf[oy*4+ox]; got != want {
				t.Errorf("RAW8 bin2 out(%d,%d) = %d, want %d", oy, ox, got, want)
			}
		}
	}
	buf = mk()
	if n := colorSumBinRAW8(buf, w, h, 4); n != 4 {
		t.Fatalf("colorSumBinRAW8 bin4 n = %d, want 4", n)
	}
	// bin 4: 8×8 block → one 2×2 cell of 16-sample sums: 16, 32, 48, 64.
	if buf[0] != 16 || buf[1] != 32 || buf[2] != 48 || buf[3] != 64 {
		t.Errorf("RAW8 bin4 cell = %v, want [16 32 48 64]", buf[:4])
	}
	// RAW16 bin 2 keeps the phase level (mean of 4 equal samples).
	b16 := make([]byte, w*h*2)
	for i, v := range mk() {
		u := int(v) * 100
		b16[2*i], b16[2*i+1] = byte(u), byte(u>>8)
	}
	if n := colorBinRAW16(b16, w, h, 2); n != 32 {
		t.Fatalf("colorBinRAW16 bin2 n = %d, want 32", n)
	}
	for oy := 0; oy < 4; oy++ {
		for ox := 0; ox < 4; ox++ {
			want := 100 * (1 + (oy&1)*2 + (ox & 1))
			o := (oy*4 + ox) * 2
			if got := int(b16[o]) | int(b16[o+1])<<8; got != want {
				t.Errorf("RAW16 bin2 out(%d,%d) = %d, want %d", oy, ox, got, want)
			}
		}
	}
}

// colorBinRAW16Ref / colorSumBinRAW8Ref are colorBinRAW16 / colorSumBinRAW8 computed into a
// separate output buffer: the reference the in-place versions must match.
func colorBinRAW16Ref(src []byte, fullW, fullH, bin int) []byte {
	outW, outH := fullW/bin, fullH/bin
	out := make([]byte, outW*outH*2)
	for oy := 0; oy < outH; oy++ {
		row0 := 2*bin*(oy/2) + oy&1
		for ox := 0; ox < outW; ox++ {
			col0 := 2*bin*(ox/2) + ox&1
			sum, cnt := 0, 0
			for i := 0; i < bin; i++ {
				r := row0 + 2*i
				if r >= fullH {
					break
				}
				for j := 0; j < bin; j++ {
					cc := col0 + 2*j
					if cc >= fullW {
						break
					}
					p := (r*fullW + cc) * 2
					sum += int(src[p]) | int(src[p+1])<<8
					cnt++
				}
			}
			v := 0
			if cnt > 0 {
				v = (sum + cnt/2) / cnt
			}
			o := (oy*outW + ox) * 2
			out[o], out[o+1] = byte(v), byte(v>>8)
		}
	}
	return out
}

func colorSumBinRAW8Ref(src []byte, fullW, fullH, bin int) []byte {
	outW, outH := fullW/bin, fullH/bin
	out := make([]byte, outW*outH)
	for oy := 0; oy < outH; oy++ {
		row0 := 2*bin*(oy/2) + oy&1
		for ox := 0; ox < outW; ox++ {
			col0 := 2*bin*(ox/2) + ox&1
			sum := 0
			for i := 0; i < bin; i++ {
				r := row0 + 2*i
				if r >= fullH {
					break
				}
				for j := 0; j < bin; j++ {
					cc := col0 + 2*j
					if cc >= fullW {
						break
					}
					sum += int(src[r*fullW+cc])
				}
			}
			if sum > 0xff {
				sum = 0xff
			}
			out[oy*outW+ox] = byte(sum)
		}
	}
	return out
}

// TestColorBinInPlace: the in-place color bins (no scratch allocation, E3) equal the
// scratch-buffer reference for every geometry the driver produces, including edge widths that
// are not a multiple of 2·bin (partial blocks) and bin 3/4. The frame content varies per pixel
// and per Bayer phase, so any read of an already-overwritten sample changes the result.
func TestColorBinInPlace(t *testing.T) {
	geoms := []struct{ w, h, bin int }{
		{16, 8, 2}, {24, 12, 2}, {26, 10, 2}, {24, 12, 3}, {30, 14, 3}, {32, 16, 4}, {36, 18, 4},
		{640, 480, 2}, {642, 482, 2}, {96, 60, 4},
	}
	for _, g := range geoms {
		src16 := make([]byte, g.w*g.h*2)
		src8 := make([]byte, g.w*g.h)
		for y := 0; y < g.h; y++ {
			for x := 0; x < g.w; x++ {
				v := (x*7 + y*131 + (x&1)*3000 + (y&1)*12000 + (x*y)%97) & 0xffff
				src16[(y*g.w+x)*2] = byte(v)
				src16[(y*g.w+x)*2+1] = byte(v >> 8)
				src8[y*g.w+x] = byte((x*3 + y*5 + (x&1)*40 + (y&1)*80) & 0x3f)
			}
		}
		want16 := colorBinRAW16Ref(src16, g.w, g.h, g.bin)
		buf16 := append([]byte(nil), src16...)
		n := colorBinRAW16(buf16, g.w, g.h, g.bin)
		if n != len(want16) || string(buf16[:n]) != string(want16) {
			t.Errorf("colorBinRAW16 %dx%d bin %d: in-place result differs from the reference", g.w, g.h, g.bin)
		}
		want8 := colorSumBinRAW8Ref(src8, g.w, g.h, g.bin)
		buf8 := append([]byte(nil), src8...)
		n = colorSumBinRAW8(buf8, g.w, g.h, g.bin)
		if n != len(want8) || string(buf8[:n]) != string(want8) {
			t.Errorf("colorSumBinRAW8 %dx%d bin %d: in-place result differs from the reference", g.w, g.h, g.bin)
		}
	}
}

// TestFX3DesyncDetection covers the frame-boundary detector against the signature measured on
// two desynced ASI6200MC frames: the previous frame's footer word immediately ahead of this
// frame's header word, 32-bit aligned, at a whole multiple of 16 KiB.
func TestFX3DesyncDetection(t *testing.T) {
	const n = 1 << 20
	synced := func() []byte {
		b := make([]byte, n)
		for i := range b {
			b[i] = byte(i * 7)
		}
		b[0], b[1], b[2], b[3] = 0x7E, 0x5A, 0x01, 0x00
		b[n-4], b[n-3], b[n-2], b[n-1] = 0x00, 0x00, 0xF0, 0x3C
		return b
	}

	if b := synced(); !repairFX3DMAMarkers(b, 2, 256, 2) {
		t.Error("a frame with markers at both boundaries must repair")
	}
	if got := fx3MarkerOffset(synced()); got != -1 {
		t.Errorf("a synced frame has no interior boundary, got offset %d", got)
	}

	// A desync: K bytes of the previous frame's tail ahead of this frame's start.
	for _, K := range []int{16384, 13 * 16384, 14 * 16384} {
		b := make([]byte, n)
		for i := range b {
			b[i] = byte(i * 11)
		}
		b[K-4], b[K-3], b[K-2], b[K-1] = 0x00, 0x00, 0xF0, 0x3C
		b[K], b[K+1], b[K+2], b[K+3] = 0x7E, 0x5A, 0x01, 0x00
		if repairFX3DMAMarkers(b, 2, 256, 2) {
			t.Errorf("K=%d: a desynced frame must not pass the boundary check", K)
		}
		if got := fx3MarkerOffset(b); got != K {
			t.Errorf("K=%d: fx3MarkerOffset = %d, want %d", K, got, K)
		}
	}

	// A lone 0x7E 0x5A pair in sensor noise, with no footer ahead of it, is not a boundary.
	b := make([]byte, n)
	for i := 0; i < n; i += 4 {
		b[i], b[i+1] = 0x7E, 0x5A
	}
	if got := fx3MarkerOffset(b); got != -1 {
		t.Errorf("header words without a preceding footer are noise, got offset %d", got)
	}
}
