//go:build windows

package astrocam

import (
	"fmt"
	"runtime"
	"sync"
	"syscall"
	"time"
	"unsafe"
)

// winusbStream is the Windows resident windowed-stream session (StreamStarter / FrameStream):
// a window of overlapped reads, each with its own scratch buffer and event, kept cycling on
// EP 0x81 across frames; Next copies the next whole frame out in segment order and re-arms each
// slot as it is drained. Single-consumer. It takes no ioMu (a free-run stream would hold it
// forever) and counts as a read in flight for the USB2 EP0 pacing. Compile-checked, not run on a
// Windows host.
type winusbStream struct {
	d      *winusbDevice
	chunk  int
	slots  []winStreamSlot
	next   uint64 // segment to consume next
	submit uint64 // segment number for the next submission
	segOff int
	closed bool
	mu     sync.Mutex
}

type winStreamSlot struct {
	ov    *overlapped
	ev    uintptr
	buf   []byte
	seq   uint64
	armed bool // read in flight: the kernel owns buf and ov
	done  bool
	n     int
	err   error // completion error (nil, or the ReadPipe failure)
}

// StartStream implements StreamStarter: it primes the window and returns the session. total is
// applied as the driver-side per-transfer timeout (plus a margin); Next's idle bounds a stall.
func (d *winusbDevice) StartStream(frameBytes, trailer int, total time.Duration) (FrameStream, error) {
	release, err := d.enter()
	if err != nil {
		return nil, err
	}
	defer release()
	chunk := maxBulkChunk
	if frameBytes > 0 && frameBytes < chunk {
		chunk = frameBytes // one transfer per frame for sub-MiB frames
	}
	totalMs := total.Milliseconds()
	if totalMs <= 0 {
		totalMs = defaultTotalBound.Milliseconds() // see the backend contract in transport.go
	}
	d.setPipeTimeout(uint32(totalMs) + 1000)
	st := &winusbStream{d: d, chunk: chunk, slots: make([]winStreamSlot, winPrequeuedWindow)}
	for i := range st.slots {
		st.slots[i].buf = make([]byte, chunk)
		ev, _, _ := procCreateEvent.Call(0, 1, 0, 0)
		if ev == 0 {
			st.freeEvents()
			return nil, fmt.Errorf("astrocam: CreateEvent failed")
		}
		st.slots[i].ev = ev
		st.slots[i].ov = &overlapped{HEvent: syscall.Handle(ev)}
	}
	armed := 0
	for i := range st.slots {
		if err := st.rearm(i); err != nil {
			if armed == 0 {
				st.freeEvents()
				return nil, err
			}
			break // window truncated; the armed slots carry the stream
		}
		armed++
	}
	d.streamMu.Lock()
	if d.streams == nil {
		d.streams = map[*winusbStream]struct{}{}
	}
	d.streams[st] = struct{}{}
	d.streamMu.Unlock()
	d.readActive.Add(1)
	return st, nil
}

func (st *winusbStream) freeEvents() {
	for i := range st.slots {
		if st.slots[i].ev != 0 {
			procCloseHandle.Call(st.slots[i].ev)
			st.slots[i].ev = 0
		}
	}
}

// rearm submits slot i for the next segment (an overlapped ReadPipe on its scratch).
func (st *winusbStream) rearm(i int) error {
	s := &st.slots[i]
	*s.ov = overlapped{HEvent: syscall.Handle(s.ev)}
	s.done, s.n, s.err = false, 0, nil
	s.seq = st.submit
	r, _, callErr := procWinUsbReadPipe.Call(st.d.winusb, pipeIn,
		uintptr(unsafe.Pointer(&s.buf[0])), uintptr(len(s.buf)), 0, uintptr(unsafe.Pointer(s.ov)))
	if r == 0 {
		if errno, ok := callErr.(syscall.Errno); !ok || errno != errIOPending {
			s.armed = false // dead slot: no completion will come
			return fmt.Errorf("astrocam: WinUsb_ReadPipe failed: %v", callErr)
		}
	}
	s.armed = true
	st.submit++
	return nil
}

// complete records slot i's result once its event has signaled.
func (st *winusbStream) complete(i int) {
	s := &st.slots[i]
	var transferred uint32
	r, _, callErr := procWinUsbGetOvlRes.Call(st.d.winusb, uintptr(unsafe.Pointer(s.ov)), uintptr(unsafe.Pointer(&transferred)), 0)
	s.armed = false
	s.done = true
	s.n = int(transferred)
	if r == 0 {
		if errno, ok := callErr.(syscall.Errno); ok {
			switch errno {
			case errSemTimeout, errOpAborted, errTimeout:
				return // a stall/abort: a short segment, not a pipe error
			}
		}
		s.err = fmt.Errorf("astrocam: stream ReadPipe failed: %v", callErr)
	}
}

// live reports whether the session can still produce anything: a slot armed with the driver, or
// one already completed and waiting to be consumed. rearm clears armed on a failed resubmit and
// leaves done false, so a slot that failed to resubmit is neither, and a window whose slots have
// all failed can never recover on its own.
func (st *winusbStream) live() bool {
	for i := range st.slots {
		if st.slots[i].armed || st.slots[i].done {
			return true
		}
	}
	return false
}

// Next pulls one frame (len(buf) bytes) from the session, in segment order; a chunk may straddle
// frame boundaries (segOff carries the remainder to the next call). Returns a short count with a
// nil error on an idle stall, an error on a hard pipe error or a closed session.
func (st *winusbStream) Next(buf []byte, idle time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	st.mu.Lock()
	defer st.mu.Unlock()
	if st.closed {
		return 0, errWinTransportClosed
	}
	if st.d.broken.Load() {
		return 0, errWinTransportBroken
	}
	if idle <= 0 {
		idle = defaultIdleBound // see the backend contract in transport.go
	}
	copied := 0
	lastReal := time.Now()
	for copied < len(buf) {
		progressed := false
		for {
			cur := -1
			for i := range st.slots {
				if st.slots[i].done && st.slots[i].seq == st.next {
					cur = i
					break
				}
			}
			if cur < 0 {
				break
			}
			s := &st.slots[cur]
			if s.err != nil {
				return copied, s.err
			}
			avail := s.n - st.segOff
			take := avail
			if take > len(buf)-copied {
				take = len(buf) - copied
			}
			if take > 0 {
				copy(buf[copied:], s.buf[st.segOff:st.segOff+take])
				copied += take
				st.segOff += take
				progressed = true
				lastReal = time.Now()
			}
			if st.segOff >= s.n { // segment fully consumed (a ZLP at once): recycle
				st.segOff = 0
				st.next++
				_ = st.rearm(cur) // a failed resubmit leaves a dead slot; the window shrinks
			}
			if copied >= len(buf) {
				break
			}
		}
		if copied >= len(buf) {
			break
		}
		if st.d.readAborted.Load() {
			return copied, nil
		}
		// Wait for the in-order slot's completion in short slices (its event signals in
		// submission order on a single pipe); on a slice timeout apply the idle bound.
		wi := -1
		for i := range st.slots {
			if st.slots[i].armed && st.slots[i].seq == st.next {
				wi = i
				break
			}
		}
		if wi < 0 {
			// The in-order segment has no armed slot (a dead slot after a failed resubmit).
			// If nothing at all is armed or waiting to be consumed, every slot died that way
			// and no completion can ever arrive: skipping the number then never finds an armed
			// segment and this loop spins at 100 % CPU with no sleep, idle bound or exit. Fail
			// the session instead; only Close can recover it.
			if !st.live() {
				return copied, errWinStreamDead
			}
			// Some slots are still armed, so advancing past the dead segment reaches one and the
			// loop terminates. The segment's bytes are gone, which leaves the stream offset by
			// that much -- the same desync darwin latches (see darwinStream.Next); the caller
			// sees it as a short frame.
			st.next++
			continue
		}
		if w, _, _ := procWaitForSingle.Call(st.slots[wi].ev, uintptr(50)); w == waitObject0 {
			st.complete(wi)
			continue
		}
		if !progressed && time.Since(lastReal) > idle {
			break // stall
		}
	}
	runtime.KeepAlive(st.slots)
	return copied, nil
}

// Close aborts and drains the armed reads, then frees the session. If an armed read does not
// complete within winDrainTimeout the scratch buffers and OVERLAPPEDs are pinned in
// winLeakedIO and the device is poisoned. Idempotent.
func (st *winusbStream) Close() error {
	st.mu.Lock()
	defer st.mu.Unlock()
	if st.closed {
		return nil
	}
	st.closed = true
	d := st.d
	d.streamMu.Lock()
	delete(d.streams, st)
	d.streamMu.Unlock()
	d.readActive.Add(-1)
	procWinUsbAbortPipe.Call(d.winusb, pipeIn)
	dl := time.Now().Add(winDrainTimeout)
	for i := range st.slots {
		if !st.slots[i].armed {
			continue
		}
		ms := time.Until(dl).Milliseconds()
		if ms < 0 {
			ms = 0
		}
		if w, _, _ := procWaitForSingle.Call(st.slots[i].ev, uintptr(ms)); w != waitObject0 {
			d.broken.Store(true)
			winLeakedIOMu.Lock()
			winLeakedIO = append(winLeakedIO, st.slots)
			winLeakedIOMu.Unlock()
			return errWinTransportBroken // leak the events too
		}
		st.slots[i].armed = false
	}
	st.freeEvents()
	return nil
}
