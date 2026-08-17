package main

import (
	"math"
	"os"
	"path/filepath"
	"testing"
	"time"
)

// rawFrame builds a RAW16 frame of n samples: a 60000 ADU pedestal plus a deterministic 0..15
// ripple, the shape of a near-saturated flat. It also returns the exact mean and standard
// deviation, computed from the ripple histogram alone (adding a constant pedestal does not
// change the spread), so the reference needs no wide-integer accumulation of its own.
func rawFrame(n int) (frame []byte, mean, sd float64) {
	const pedestal = 60000
	frame = make([]byte, n*2)
	var hist [16]int64
	seed := uint32(12345)
	for i := 0; i < n; i++ {
		seed = seed*1664525 + 1013904223
		d := seed >> 28
		v := pedestal + int(d)
		frame[2*i] = byte(v)
		frame[2*i+1] = byte(v >> 8)
		hist[d]++
	}
	var s, sq int64
	for d, c := range hist {
		s += int64(d) * c
		sq += int64(d) * int64(d) * c
	}
	fn := float64(n)
	md := float64(s) / fn
	return frame, pedestal + md, math.Sqrt(float64(sq)/fn - md*md)
}

// TestFrameStatsPrecision pins the accumulator's accuracy on a frame big enough for the sums to
// matter: 16.7 M samples near saturation put sumsq at 6e19, where a float64 running total loses
// the low bits of every add and the sd comes out over 1 % wrong.
func TestFrameStatsPrecision(t *testing.T) {
	const n = 1 << 24 // 16.7 Mpx, a third of an IMX455 frame
	frame, wantAvg, wantSD := rawFrame(n)

	mn, mx, avg, sd := frameStats(frame, 2)
	if mn != 60000 || mx != 60015 {
		t.Errorf("min/max = %d/%d, want 60000/60015", mn, mx)
	}
	if rel := math.Abs(avg-wantAvg) / wantAvg; rel > 1e-12 {
		t.Errorf("avg = %.9f, want %.9f (rel %.2e)", avg, wantAvg, rel)
	}
	if rel := math.Abs(sd-wantSD) / wantSD; rel > 1e-6 {
		t.Errorf("sd = %.9f, want %.9f (rel error %.4f%%)", sd, wantSD, 100*rel)
	}
}

// TestReadBudgetWireTerm checks that the wire allowance tracks the frame size instead of
// collapsing to whole seconds: every frame under 20 MiB otherwise gets no wire time at all.
func TestReadBudgetWireTerm(t *testing.T) {
	const base = 3 * time.Second
	for _, tc := range []struct {
		name  string
		bytes int
		want  time.Duration // wire term only, exposure 0
	}{
		{"6 MB ASI290 RAW16", 6_000_000, 286 * time.Millisecond},
		{"just under 20 MiB", 20<<20 - 1, time.Second},
		{"122 MB IMX455 frame", 122_000_000, 5817 * time.Millisecond},
	} {
		got := readBudget(0, tc.bytes) - base
		if d := got - tc.want; d < -5*time.Millisecond || d > 5*time.Millisecond {
			t.Errorf("%s: wire term %s, want ~%s", tc.name, got, tc.want)
		}
	}
	if got, want := readBudget(2*time.Second, 0), 5*time.Second; got != want {
		t.Errorf("readBudget(2s, 0) = %s, want %s", got, want)
	}
}

// fakeStream is a FrameStream that hands out prefilled frames and records when each Next
// returned, so a test can see whether the read loop stalled between frames.
type fakeStream struct {
	size int
	at   []time.Duration
	t0   time.Time
}

func (f *fakeStream) Next(buf []byte, idle time.Duration) (int, error) {
	f.at = append(f.at, time.Since(f.t0))
	return f.size, nil
}

func (f *fakeStream) Close() error { return nil }

// TestSERLoopKeepsProcessingOffTheReadThread checks that per-frame pixel work (the FX3 marker
// repair and the host bin, which on a color frame is the heaviest step in the burst) does not
// sit between the stream reads. With a frame pool at least as deep as the burst, the read loop
// must issue every Next back to back and let the writer goroutine absorb the work.
func TestSERLoopKeepsProcessingOffTheReadThread(t *testing.T) {
	const (
		nf   = serPool // fits the pool, so the reader never waits for a buffer
		work = 20 * time.Millisecond
		w, h = 4, 4
	)
	sw, err := newSER(filepath.Join(t.TempDir(), "burst.ser"), w, h, 2, serMono, "test")
	if err != nil {
		t.Fatal(err)
	}
	b := &burst{
		fbytes: w * h * 2,
		nf:     nf,
		sw:     sw,
		el:     func() string { return "" },
		process: func(fb []byte, n int) int {
			time.Sleep(work)
			return n
		},
	}
	fs := &fakeStream{size: b.fbytes, t0: time.Now()}

	start := time.Now()
	count, err := b.serLoop(fs)
	readSpan := fs.at[len(fs.at)-1] // when the last frame was read, not when the loop returned
	if err != nil {
		t.Fatal(err)
	}
	if cerr := sw.close(); cerr != nil {
		t.Fatal(cerr)
	}
	if count != nf {
		t.Fatalf("wrote %d frames, want %d", count, nf)
	}
	// Inline processing would space the reads by work each; off the read thread they are
	// immediate. Half a work unit separates the two by a wide margin.
	if readSpan > work/2 {
		t.Errorf("read loop took %s to pull %d frames (limit %s): the per-frame work is on the read thread",
			readSpan, nf, work/2)
	}
	// The work still has to happen before serLoop returns, or the frames are unprocessed.
	if total := time.Since(start); total < work {
		t.Errorf("serLoop returned in %s, less than one %s work unit: the processing was skipped", total, work)
	}
}

// TestSERLoopProcessesEveryFrame checks the moved work still runs once per frame and that what
// it returns sizes the frame written to the file.
func TestSERLoopProcessesEveryFrame(t *testing.T) {
	const (
		nf   = 6
		w, h = 4, 4
		full = w * h * 2
		half = full / 2 // as a 2x2 host bin would leave it
	)
	path := filepath.Join(t.TempDir(), "burst.ser")
	sw, err := newSER(path, w, h/2, 2, serMono, "test")
	if err != nil {
		t.Fatal(err)
	}
	var calls int
	b := &burst{
		fbytes: full,
		nf:     nf,
		sw:     sw,
		el:     func() string { return "" },
		process: func(fb []byte, n int) int {
			if n != full {
				t.Errorf("process got %d bytes, want the whole wire frame (%d)", n, full)
			}
			calls++
			return half
		},
	}
	count, err := b.serLoop(&fakeStream{size: full, t0: time.Now()})
	if err != nil {
		t.Fatal(err)
	}
	if cerr := sw.close(); cerr != nil {
		t.Fatal(cerr)
	}
	if calls != nf {
		t.Errorf("process ran %d times, want %d", calls, nf)
	}
	if count != nf {
		t.Errorf("wrote %d frames, want %d", count, nf)
	}
	st, serr := os.Stat(path)
	if serr != nil {
		t.Fatal(serr)
	}
	if want := int64(serHeaderSize + nf*half + nf*8); st.Size() != want {
		t.Errorf("SER file is %d bytes, want %d (header + %d processed frames + trailer)", st.Size(), want, nf)
	}
}
