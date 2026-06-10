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
func (f *frameTransport) ControlIn(uint8, uint16, uint16, []byte) (int, error) { return 0, nil }
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

// streamTransport implements BulkStreamer, serving a frame in bufSize chunks (the
// last one short) — the async-pump fast path.
type streamTransport struct {
	out   []ctrlCall
	frame []byte
}

func (s *streamTransport) ControlOut(b uint8, wv, wi uint16, _ []byte) error {
	s.out = append(s.out, ctrlCall{b, wv, wi})
	return nil
}
func (s *streamTransport) ControlIn(uint8, uint16, uint16, []byte) (int, error) { return 0, nil }
func (s *streamTransport) BulkRead([]byte, time.Duration) (int, error) {
	return 0, errors.New("use BulkStream")
}
func (s *streamTransport) Close() error { return nil }
func (s *streamTransport) BulkStream(bufSize, _ int) (Stream, error) {
	return &sliceStream{data: s.frame, chunk: bufSize}, nil
}

type sliceStream struct {
	data  []byte
	chunk int
	off   int
}

func (s *sliceStream) Next(time.Duration) ([]byte, error) {
	if s.off >= len(s.data) {
		return nil, errors.New("stream drained")
	}
	n := s.chunk
	if rem := len(s.data) - s.off; n > rem {
		n = rem
	}
	b := s.data[s.off : s.off+n]
	s.off += n
	return b, nil
}
func (s *sliceStream) Close() error { return nil }

// TestSnapStreamingReadout exercises the async-pump path: a frame reassembled from
// multiple chunks via BulkStream, terminated by the final short packet.
func TestSnapStreamingReadout(t *testing.T) {
	old := bulkChunkBytes
	bulkChunkBytes = 8 // shrink so a small frame spans several chunks
	defer func() { bulkChunkBytes = old }()

	// armSensor: 4*2*2 = 16 pixel bytes; + 4-byte magic header = 20 → 3 chunks (8,8,4).
	frame := make([]byte, 4+16)
	binary.LittleEndian.PutUint32(frame[:4], FrameMagic)
	for i := 4; i < len(frame); i++ {
		frame[i] = byte(i)
	}
	st := &streamTransport{frame: frame}
	Register(ZWO.VID, 0x00D1, Model{Name: "Stream", Sensor: &armSensor})
	c, err := Open(st, ZWO.VID, 0x00D1)
	if err != nil {
		t.Fatal(err)
	}
	c.SetExposure(0)
	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	buf := make([]byte, 64)
	n, err := c.GetDataAfterExp(buf)
	if err != nil {
		t.Fatal(err)
	}
	if n != len(frame) {
		t.Errorf("streamed %d bytes, want %d", n, len(frame))
	}
	for i := 4; i < n; i++ {
		if buf[i] != byte(i) {
			t.Errorf("reassembled byte %d = %d, want %d", i, buf[i], byte(i))
			break
		}
	}
	if c.GetExpStatus() != ExpIdle {
		t.Error("status should be idle after a successful streamed readout")
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

// TestSnapBadMagic: a frame without the header magic fails readout and marks the
// exposure FAILED (the dropped-frame path).
func TestSnapBadMagic(t *testing.T) {
	f := &frameTransport{frame: []byte{0xde, 0xad, 0xbe, 0xef, 1, 2, 3, 4}}
	c, _ := Open(f, ZWO.VID, 0x00D0)
	c.SetExposure(0)
	if err := c.StartExposure(true); err != nil {
		t.Fatal(err)
	}
	if _, err := c.GetDataAfterExp(make([]byte, 64)); err == nil {
		t.Error("expected bad-magic error")
	}
	if c.GetExpStatus() != ExpFailed {
		t.Errorf("status = %s, want failed", c.GetExpStatus())
	}
}
