package astrocam

import (
	"errors"
	"strings"
	"sync"
	"testing"
	"time"
)

// orderTransport records the abort latch alongside control transfers, so a test can assert that
// a read is broken before the arm writes go out.
type orderTransport struct {
	*StubTransport
	mu     sync.Mutex
	events []string
}

func (o *orderTransport) rec(e string) { o.mu.Lock(); o.events = append(o.events, e); o.mu.Unlock() }
func (o *orderTransport) log() []string {
	o.mu.Lock()
	defer o.mu.Unlock()
	return append([]string(nil), o.events...)
}
func (o *orderTransport) AbortRead() { o.rec("abort"); o.StubTransport.AbortRead() }
func (o *orderTransport) ArmRead()   { o.rec("armread"); o.StubTransport.ArmRead() }
func (o *orderTransport) ControlOut(b uint8, wv, wi uint16, d []byte) error {
	o.rec("ctrl")
	return o.StubTransport.ControlOut(b, wv, wi, d)
}

// TestStartVideoBreaksInFlightRead: a single-shot capture in flight holds the transport's I/O
// gate for the whole frame read, so StartVideo must break that read before issuing its arm
// writes; otherwise the arm queues behind the readout for its full timeout. StopExposure takes
// the same step for the same reason.
func TestStartVideoBreaksInFlightRead(t *testing.T) {
	s := &Sensor{
		Name:        "VID",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1}},
		Init:        []RegVal{},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
		Worker: func(ctl WorkerCtl, buf []byte, _ time.Duration) (int, error) {
			return ctl.BulkRead(buf, time.Second)
		},
	}
	Register(ZWO.VID, 0x0D11, Model{Name: "Vid", Sensor: s})
	tr := &orderTransport{StubTransport: NewStubTransport()}
	c, err := Open(tr, ZWO.VID, 0x0D11)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetROI(0, 0, 96, 80); err != nil {
		t.Fatal(err)
	}
	// Claim a capture: a Worker profile arms inside GetDataAfterExp, so the frame read is what
	// would be in flight here.
	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	tr.mu.Lock()
	tr.events = nil
	tr.mu.Unlock()
	if err := c.StartVideo(true); err != nil {
		t.Fatal(err)
	}
	ev := tr.log()
	if len(ev) == 0 {
		t.Fatal("StartVideo issued nothing")
	}
	if ev[0] != "abort" {
		t.Errorf("StartVideo event order = %v, want the read abort before the arm writes", ev)
	}
	_ = c.StopExposure()
}

// TestStartVideoRefusesWedgedDevice: once the firmware-crash latch is set, StartVideo must
// refuse like StartExposure does, rather than push an arm at a device that has dropped its
// firmware.
func TestStartVideoRefusesWedgedDevice(t *testing.T) {
	s := &Sensor{
		Name:        "VIDW",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1}},
		Init:        []RegVal{},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
		Worker:      func(WorkerCtl, []byte, time.Duration) (int, error) { return 0, nil }, // short frame
	}
	Register(ZWO.VID, 0x0D12, Model{Name: "VidW", Sensor: s})
	st := NewStubTransport()
	c, err := Open(st, ZWO.VID, 0x0D12)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.Init(); err != nil { // records the firmware baseline
		t.Fatal(err)
	}
	if err := c.SetROI(0, 0, 96, 80); err != nil {
		t.Fatal(err)
	}
	st.Firmware = 0x1234 // the FX3 dropped its firmware
	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	if _, err := c.GetDataAfterExp(make([]byte, c.FrameBytes())); !errors.Is(err, ErrDeviceWedged) {
		t.Fatalf("short frame with a changed firmware: err = %v, want ErrDeviceWedged", err)
	}
	if !c.Wedged() {
		t.Fatal("camera not latched dead")
	}
	if err := c.StartVideo(true); !errors.Is(err, ErrDeviceWedged) {
		t.Errorf("StartVideo on a wedged camera = %v, want ErrDeviceWedged", err)
	}
}

// TestGetDataAfterExpRejectsShortBuffer: a buffer smaller than FrameBytes is a caller error, not
// a device fault. Workers clamp their read to len(buf) and return that count, which the frame
// check would otherwise read as a short frame: that marks the exposure failed and fires the
// firmware-crash probe (an EP0 read at a camera that may still be streaming), and a probe that
// cannot answer latches the camera dead. The call must be refused before any of that.
func TestGetDataAfterExpRejectsShortBuffer(t *testing.T) {
	s := &Sensor{
		Name:        "SHORTBUF",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1}},
		Init:        []RegVal{},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
		Worker: func(ctl WorkerCtl, buf []byte, _ time.Duration) (int, error) {
			want := ctl.FrameBytes()
			if want > len(buf) {
				want = len(buf) // the clamp every shipping worker applies
			}
			return ctl.BulkRead(buf[:want], time.Second)
		},
	}
	Register(ZWO.VID, 0x0D13, Model{Name: "ShortBuf", Sensor: s})
	st := NewStubTransport()
	c, err := Open(st, ZWO.VID, 0x0D13)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.Init(); err != nil {
		t.Fatal(err)
	}
	if err := c.SetROI(0, 0, 96, 80); err != nil {
		t.Fatal(err)
	}
	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	st.Log, st.Reads = nil, 0
	n, err := c.GetDataAfterExp(make([]byte, 100)) // FrameBytes is 96*80*2
	if err == nil {
		t.Fatalf("undersized buffer accepted (%d bytes returned)", n)
	}
	if errors.Is(err, ErrDeviceWedged) || c.Wedged() {
		t.Errorf("undersized buffer diagnosed as a device fault: err = %v, wedged = %v", err, c.Wedged())
	}
	if !strings.Contains(err.Error(), "buffer") {
		t.Errorf("err = %v, want it to name the buffer as the problem", err)
	}
	for _, x := range st.Log {
		if x.In && x.BRequest == 0xAD {
			t.Errorf("firmware-crash probe issued for a caller buffer error: %+v", st.Log)
			break
		}
	}
	if st.Reads != 0 {
		t.Errorf("served %d frame reads, want none (the call is refused up front)", st.Reads)
	}
	if got := c.GetExpStatus(); got == ExpFailed {
		t.Errorf("exposure marked %s by a caller error; the claim should survive a retry with a correct buffer", got)
	}
}

// TestStartExposureRejectsBusy: arming while a capture is already claimed is a caller error, not
// a silent no-op. The old behavior returned nil having ignored the request, so a caller believed
// it had armed the exposure it just asked for while the device kept integrating the previous
// one. ASCOM requires the busy start to fail; the error is a distinct sentinel so a caller can
// tell it from a device fault, and the claimed exposure is left untouched.
func TestStartExposureRejectsBusy(t *testing.T) {
	s := &Sensor{
		Name:        "BUSY",
		Info:        CameraInfo{MaxWidth: 96, MaxHeight: 80, BitDepth: 12, Bins: []int{1}},
		Init:        []RegVal{},
		SetROI:      func(Regmap, int, int, int, int, int) error { return nil },
		SetExposure: func(Regmap, time.Duration) error { return nil },
		Worker: func(ctl WorkerCtl, buf []byte, _ time.Duration) (int, error) {
			return ctl.BulkRead(buf, time.Second)
		},
	}
	Register(ZWO.VID, 0x0D14, Model{Name: "Busy", Sensor: s})
	st := NewStubTransport()
	c, err := Open(st, ZWO.VID, 0x0D14)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetROI(0, 0, 96, 80); err != nil {
		t.Fatal(err)
	}
	// A window still running, so the derived status stays Working rather than reporting the
	// host-timed exposure as already complete.
	if err := c.SetExposure(5 * time.Second); err != nil {
		t.Fatal(err)
	}
	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	st.Log = nil
	if err := c.StartExposure(true); !errors.Is(err, ErrCaptureBusy) {
		t.Errorf("second StartExposure = %v, want ErrCaptureBusy", err)
	}
	if len(st.Log) != 0 {
		t.Errorf("busy StartExposure touched the device: %+v", st.Log)
	}
	if got := c.GetExpStatus(); got != ExpWorking {
		t.Errorf("status after a rejected arm = %s, want working (the claim survives)", got)
	}
	// The frame the first arm claimed still reads, and the next arm is accepted once it is
	// consumed.
	if _, err := c.GetDataAfterExp(make([]byte, c.FrameBytes())); err != nil {
		t.Fatal(err)
	}
	if err := c.StartExposure(true); err != nil {
		t.Errorf("arm after the frame was consumed = %v, want accepted", err)
	}
	_ = c.StopExposure()
	if err := c.StartExposure(true); err != nil {
		t.Errorf("arm after StopExposure = %v, want accepted", err)
	}
}
