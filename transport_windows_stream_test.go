//go:build windows

package astrocam

import (
	"errors"
	"testing"
	"time"
)

// TestWinStreamDeadWindowDoesNotSpin: when every slot has failed to resubmit, no armed slot can
// carry the in-order segment and no completion will ever arrive. The loop used to answer that by
// incrementing the segment number and continuing, with no sleep, idle bound or exit — a tight
// spin at 100 % CPU for the life of the process. It must report the dead session instead.
//
// The dead-window path returns before any WinUSB call, so this needs no device: the slot scans
// find nothing done and nothing armed, and live() decides it from the slot flags alone.
func TestWinStreamDeadWindowDoesNotSpin(t *testing.T) {
	st := &winusbStream{
		d:     &winusbDevice{},
		chunk: 1 << 20,
		// Three slots as rearm leaves them after a failed WinUsb_ReadPipe: not armed (no
		// completion coming) and not done (nothing to consume).
		slots: []winStreamSlot{{armed: false, done: false}, {armed: false, done: false}, {armed: false, done: false}},
		next:  7,
	}
	if st.live() {
		t.Fatal("live() true with every slot dead")
	}

	done := make(chan struct{})
	var n int
	var err error
	go func() {
		n, err = st.Next(make([]byte, 4096), 50*time.Millisecond)
		close(done)
	}()
	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("Next did not return on a dead window: it is spinning")
	}
	if !errors.Is(err, errWinStreamDead) {
		t.Errorf("Next err = %v, want errWinStreamDead", err)
	}
	if n != 0 {
		t.Errorf("Next copied %d bytes from a dead window, want 0", n)
	}
	// The segment number must not have run away while the loop span.
	if st.next != 7 {
		t.Errorf("next advanced to %d on a dead window, want 7", st.next)
	}
}

// TestWinStreamLiveWindowSkipsDeadSegment: with some slots still armed, a dead slot at the
// in-order segment is skipped so the stream continues from the next armed one. That path must
// stay — only the all-dead case is fatal.
func TestWinStreamLiveWindowSkipsDeadSegment(t *testing.T) {
	st := &winusbStream{
		slots: []winStreamSlot{{armed: false, done: false}, {armed: true, done: false, seq: 8}},
	}
	if !st.live() {
		t.Error("live() false with an armed slot present")
	}
	st.slots = []winStreamSlot{{armed: false, done: true, seq: 8}}
	if !st.live() {
		t.Error("live() false with a completed slot waiting to be consumed")
	}
}
