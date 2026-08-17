package astrocam

import (
	"sync"
	"testing"
	"time"
)

// TestInPacerSpacing: concurrent callers of inPacer.wait complete no faster than one per min,
// so N calls take at least (N-1)·min; the USB2 EP0-read pace the backends apply while a frame
// read is in flight (usb2InPace = 20 ms, 50/s, under the 200/s wire-clean and 500/s FX3-fatal
// points on an ASI6200MC).
func TestInPacerSpacing(t *testing.T) {
	const n = 40
	const min = 5 * time.Millisecond
	p := &inPacer{min: min}
	var wg sync.WaitGroup
	start := time.Now()
	for i := 0; i < n; i++ {
		wg.Add(1)
		go func() { defer wg.Done(); p.wait() }()
	}
	wg.Wait()
	if el := time.Since(start); el < (n-1)*min {
		t.Errorf("%d paced calls took %v, want >= %v", n, el, (n-1)*min)
	}
	// An idle pacer does not delay a lone caller.
	time.Sleep(2 * min)
	t0 := time.Now()
	p.wait()
	if el := time.Since(t0); el > min {
		t.Errorf("idle pacer delayed a lone call by %v, want < %v", el, min)
	}
	if usb2InPace < 20*time.Millisecond {
		t.Errorf("usb2InPace = %v, want >= 20 ms (50/s ceiling)", usb2InPace)
	}
}
