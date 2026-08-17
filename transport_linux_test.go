package astrocam

import (
	"os"
	"testing"
	"time"
)

// newFakeUsbfs returns a usbfsDevice over /dev/null: every ioctl fails instantly (ENOTTY),
// so the only observable behavior is the lock discipline these tests assert. ioMu
// serialization is the transport's central safety property (a control transfer
// interleaving a USB2 bulk readout parks the FX3 GPIF).
func newFakeUsbfs(t *testing.T) *usbfsDevice {
	t.Helper()
	f, err := os.OpenFile("/dev/null", os.O_RDWR, 0)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = f.Close() })
	return &usbfsDevice{f: f}
}

// runsWhileLocked reports whether op completes while d.ioMu is held by the test. wait is how
// long to give the op: guarded ops use a short window (they are blocked on the held lock);
// exempt ops need a generous one so scheduler jitter under -race cannot make a lock-free op
// look blocked.
func runsWhileLocked(t *testing.T, d *usbfsDevice, wait time.Duration, op func()) bool {
	t.Helper()
	d.ioMu.Lock()
	done := make(chan struct{})
	go func() { op(); close(done) }()
	var completed bool
	select {
	case <-done:
		completed = true
	case <-time.After(wait):
		completed = false
	}
	d.ioMu.Unlock()
	if !completed {
		select {
		case <-done: // released by the unlock: the op honored ioMu
		case <-time.After(2 * time.Second):
			t.Fatal("op never completed even after ioMu was released")
		}
	}
	return completed
}

// TestIoMuDiscipline asserts which transport operations serialize on ioMu: guarded ops block
// while another I/O holds the lock; the exempt ops do not.
func TestIoMuDiscipline(t *testing.T) {
	buf := make([]byte, 4096)
	guarded := []struct {
		name string
		op   func(d *usbfsDevice)
	}{
		{"ControlIn", func(d *usbfsDevice) { _, _ = d.ControlIn(0xB3, 0, 0, buf[:2]) }},
		{"ControlOut", func(d *usbfsDevice) { _ = d.ControlOut(0xB6, 0, 0, nil) }},
		{"BulkRead", func(d *usbfsDevice) { _, _ = d.BulkRead(buf, time.Second) }},
		{"ReadFrameStreamPrequeued", func(d *usbfsDevice) {
			_, _ = d.ReadFrameStreamPrequeued(buf, 100*time.Millisecond, time.Second)
		}},
		// CLEAR_HALT is EP0 traffic: it must not land mid-readout any more than a
		// register read may.
		{"ResetEndpoint", func(d *usbfsDevice) { _ = d.ResetEndpoint(bulkEndpoint) }},
	}
	for _, g := range guarded {
		t.Run(g.name, func(t *testing.T) {
			d := newFakeUsbfs(t)
			if runsWhileLocked(t, d, 100*time.Millisecond, func() { g.op(d) }) {
				t.Errorf("%s ran while ioMu was held — EP0/bulk interleave is the GPIF-wedge mechanism", g.name)
			}
		})
	}

	exempt := []struct {
		name string
		why  string
		op   func(d *usbfsDevice)
	}{
		// ReadFrameStream is the USB3 DDR path, which needs a concurrent
		// FPGABufReload; its DDR buffering makes the interleave harmless (see ioMu).
		{"ReadFrameStream", "DDR path needs concurrent FPGABufReload", func(d *usbfsDevice) {
			_, _ = d.ReadFrameStream(buf, 50*time.Millisecond, 200*time.Millisecond)
		}},
		// ResetDevice is the last-resort recovery: it must work while a stuck read holds
		// ioMu; serializing it would deadlock the recovery.
		{"ResetDevice", "must recover a wedged read that holds ioMu", func(d *usbfsDevice) {
			_ = d.ResetDevice()
		}},
		// ST4 pulse edges must land mid-readout (the SDK issues them concurrently with its
		// capture thread); a gated pulse-off stretches a guide correction.
		{"ControlOutUngated", "ST4 pulse edges must not queue behind a frame read", func(d *usbfsDevice) {
			_ = d.ControlOutUngated(0xB0, 0, 0)
		}},
	}
	for _, e := range exempt {
		t.Run(e.name, func(t *testing.T) {
			d := newFakeUsbfs(t)
			if !runsWhileLocked(t, d, time.Second, func() { e.op(d) }) {
				t.Errorf("%s blocked on ioMu — it is a deliberate exception (%s)", e.name, e.why)
			}
		})
	}
}

// TestClosedTransportFailsFast: post-Close I/O returns errTransportClosed instead of EBADF
// (or worse, racing the fd); Close is idempotent; Close waits for in-flight I/O.
func TestClosedTransportFailsFast(t *testing.T) {
	d := newFakeUsbfs(t)
	if err := d.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := d.ControlIn(0xB3, 0, 0, make([]byte, 2)); err != errTransportClosed {
		t.Errorf("ControlIn after Close = %v, want errTransportClosed", err)
	}
	if _, err := d.BulkRead(make([]byte, 16), time.Second); err != errTransportClosed {
		t.Errorf("BulkRead after Close = %v, want errTransportClosed", err)
	}
	if err := d.Close(); err != nil {
		t.Errorf("second Close = %v, want nil (idempotent)", err)
	}
}

// TestCloseWaitsForInflightIO: Close must block until in-flight I/O releases the interlock;
// closing the fd under a live URB drain abandons kernel-held pointers into the caller's buf.
func TestCloseWaitsForInflightIO(t *testing.T) {
	d := newFakeUsbfs(t)
	release, err := d.enter() // simulate an in-flight read holding the interlock
	if err != nil {
		t.Fatal(err)
	}
	done := make(chan struct{})
	go func() { _ = d.Close(); close(done) }()
	select {
	case <-done:
		t.Fatal("Close completed while I/O was in flight")
	case <-time.After(100 * time.Millisecond):
	}
	release()
	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("Close never completed after the I/O released")
	}
}

// TestBrokenTransportFailsFast: a poisoned device (undrainable readout) refuses further I/O.
func TestBrokenTransportFailsFast(t *testing.T) {
	d := newFakeUsbfs(t)
	d.broken.Store(true)
	if _, err := d.BulkRead(make([]byte, 16), time.Second); err != errTransportBroken {
		t.Errorf("BulkRead on poisoned device = %v, want errTransportBroken", err)
	}
	if err := d.ResetEndpoint(bulkEndpoint); err != errTransportBroken {
		t.Errorf("ResetEndpoint on poisoned device = %v, want errTransportBroken", err)
	}
}

// Compile-time capability assertions: the linux transport implements the abort/quiet seams
// the Camera wires StopExposure and the sensor workers to.
var (
	_ ReadAborter     = (*usbfsDevice)(nil)
	_ QuietBulkReader = (*usbfsDevice)(nil)
)

// TestReadAbortFailsFast: with the read-abort latched, every frame read returns (0, nil)
// immediately, even while ioMu is held (queueing behind the lock is what the latch
// prevents), and ArmRead restores the guarded behavior.
func TestReadAbortFailsFast(t *testing.T) {
	buf := make([]byte, 4096)
	reads := []struct {
		name string
		op   func(d *usbfsDevice) (int, error)
	}{
		{"BulkRead", func(d *usbfsDevice) (int, error) { return d.BulkRead(buf, time.Second) }},
		{"BulkReadQuiet", func(d *usbfsDevice) (int, error) {
			return d.BulkReadQuiet(buf, 200*time.Millisecond, time.Second)
		}},
		{"ReadFrameStreamPrequeued", func(d *usbfsDevice) (int, error) {
			return d.ReadFrameStreamPrequeued(buf, 100*time.Millisecond, time.Second)
		}},
		{"ReadFrameStream", func(d *usbfsDevice) (int, error) {
			return d.ReadFrameStream(buf, 100*time.Millisecond, time.Second)
		}},
	}
	for _, r := range reads {
		t.Run(r.name, func(t *testing.T) {
			d := newFakeUsbfs(t)
			d.AbortRead()
			var n int
			var err error
			if !runsWhileLocked(t, d, time.Second, func() { n, err = r.op(d) }) {
				t.Fatalf("%s queued behind ioMu despite the read-abort latch", r.name)
			}
			if n != 0 || err != nil {
				t.Errorf("%s aborted = (%d, %v), want (0, nil)", r.name, n, err)
			}
		})
	}
	// ArmRead restores the wedge gate: BulkRead must block on ioMu again.
	d := newFakeUsbfs(t)
	d.AbortRead()
	d.ArmRead()
	if runsWhileLocked(t, d, 100*time.Millisecond, func() { _, _ = d.BulkRead(buf, time.Second) }) {
		t.Error("BulkRead ran while ioMu was held after ArmRead — the latch failed to clear")
	}
}
