package main

import (
	"encoding/binary"
	"os"
	"path/filepath"
	"sync"
	"testing"
	"time"
)

// TestSERWriter round-trips a two-frame SER file: LittleEndian=0 with LE data, DateTime
// and DateTimeUTC differing by the zone offset, FrameCount patched on close, and trailer
// stamps equal to the capture times passed to writeFrame.
func TestSERWriter(t *testing.T) {
	path := filepath.Join(t.TempDir(), "test.ser")
	const w, h, bpp = 4, 2, 2
	sw, err := newSER(path, w, h, bpp, serBayerRGGB, "unit test")
	if err != nil {
		t.Fatal(err)
	}
	frame := make([]byte, w*h*bpp)
	for i := range frame {
		frame[i] = byte(i)
	}
	at1 := time.Date(2026, 7, 3, 12, 0, 0, 0, time.UTC)
	at2 := at1.Add(40 * time.Millisecond)
	if err := sw.writeFrame(frame, at1); err != nil {
		t.Fatal(err)
	}
	if err := sw.writeFrame(frame, at2); err != nil {
		t.Fatal(err)
	}
	if err := sw.writeFrame(frame[:4], time.Time{}); err == nil {
		t.Error("writeFrame accepted a wrong-size frame")
	}
	if err := sw.close(); err != nil {
		t.Fatal(err)
	}

	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	wantLen := serHeaderSize + 2*len(frame) + 2*8
	if len(b) != wantLen {
		t.Fatalf("file is %d bytes, want %d (header + 2 frames + 2 trailer stamps)", len(b), wantLen)
	}
	le := binary.LittleEndian
	if string(b[0:14]) != "LUCAM-RECORDER" {
		t.Errorf("FileID = %q", b[0:14])
	}
	if got := le.Uint32(b[18:]); got != serBayerRGGB {
		t.Errorf("ColorID = %d, want %d", got, serBayerRGGB)
	}
	if got := le.Uint32(b[22:]); got != serLittleEndian {
		t.Errorf("LittleEndian field = %d, want %d (de-facto FireCapture/SharpCap convention)", got, serLittleEndian)
	}
	if got := le.Uint32(b[26:]); got != w {
		t.Errorf("width = %d, want %d", got, w)
	}
	if got := le.Uint32(b[30:]); got != h {
		t.Errorf("height = %d, want %d", got, h)
	}
	if got := le.Uint32(b[34:]); got != bpp*8 {
		t.Errorf("depth = %d, want %d", got, bpp*8)
	}
	if got := le.Uint32(b[38:]); got != 2 {
		t.Errorf("FrameCount = %d, want 2 (patched on close)", got)
	}
	// DateTime (local) minus DateTimeUTC equals the live zone offset in ticks (0 under UTC).
	dtLocal := int64(le.Uint64(b[162:]))
	dtUTC := int64(le.Uint64(b[170:]))
	_, off := time.Now().Zone()
	if diff := dtLocal - dtUTC; diff != int64(off)*10_000_000 {
		t.Errorf("DateTime-DateTimeUTC = %d ticks, want zone offset %d", diff, int64(off)*10_000_000)
	}
	// Trailer: the two capture stamps.
	tr := b[serHeaderSize+2*len(frame):]
	if got := int64(le.Uint64(tr[0:])); got != netTicks(at1) {
		t.Errorf("trailer[0] = %d, want capture stamp %d", got, netTicks(at1))
	}
	if got := int64(le.Uint64(tr[8:])); got != netTicks(at2) {
		t.Errorf("trailer[1] = %d, want capture stamp %d", got, netTicks(at2))
	}
	// Frame payloads verbatim at their offsets.
	for i := 0; i < len(frame); i++ {
		if b[serHeaderSize+i] != frame[i] {
			t.Fatalf("frame 0 byte %d = %d, want %d", i, b[serHeaderSize+i], frame[i])
		}
	}
}

// TestSERWriterCloseRacesWriter: on SIGINT the interrupt hook closes the SER file while the
// burst's writer goroutine may be mid-writeFrame, so the two must not touch count, stamps and
// the file concurrently. Without serialization the trailer can be written from a half-updated
// stamp slice and FrameCount can disagree with the frames on disk.
func TestSERWriterCloseRacesWriter(t *testing.T) {
	path := filepath.Join(t.TempDir(), "race.ser")
	sw, err := newSER(path, 4, 4, 1, serMono, "T")
	if err != nil {
		t.Fatal(err)
	}
	frame := make([]byte, 16)
	var wg sync.WaitGroup
	wg.Add(1)
	go func() { // the burst's async writer
		defer wg.Done()
		for i := 0; i < 200; i++ {
			if err := sw.writeFrame(frame, time.Now()); err != nil {
				return // the file is closed under us: expected, must not corrupt
			}
		}
	}()
	time.Sleep(time.Millisecond)
	if err := sw.close(); err != nil { // the interrupt hook
		t.Logf("close: %v", err)
	}
	wg.Wait()
}
