//go:build linux

package astrocam

import (
	"fmt"
	"sync"
	"syscall"
	"time"
	"unsafe"
)

// usbfsStream is the Linux resident windowed-stream session (StreamStarter / FrameStream): a
// window of bulk-IN URBs, each with its own scratch buffer, kept cycling on EP 0x81 across
// frames; Next copies the next whole frame out in segment order and resubmits each slot as it
// is drained, so the per-frame setup cost is paid once per burst and no frame boundary is a
// resync. Single-consumer: Next/Close are called from one goroutine at a time. It takes no ioMu
// (a free-run stream would hold it forever); it counts as a read in flight for the USB2 EP0
// pacing. Another frame reader on the same fd would steal its completions, so a session must
// be the only reader while open (the Camera's contract).
type usbfsStream struct {
	d      *usbfsDevice
	chunk  int
	slots  []streamSlot
	next   uint64 // segment to consume next
	submit uint64 // segment number for the next submission
	segOff int    // bytes already consumed from the current `next` segment
	desync bool   // a Next ended mid-frame: the segment stream no longer aligns
	closed bool
	mu     sync.Mutex // Next vs Close from different goroutines
}

type streamSlot struct {
	urb   usbURB
	buf   []byte
	seq   uint64
	done  bool
	armed bool // submitted, not yet reaped: the kernel owns buf
	n     int  // actualLength at completion
	st    int32
}

// StartStream implements StreamStarter: it primes the window and returns the session. total is
// unused on usbfs (URBs carry no timeout; Next's idle bounds a stall and Close discards).
func (d *usbfsDevice) StartStream(frameBytes, trailer int, total time.Duration) (FrameStream, error) {
	release, err := d.enter()
	if err != nil {
		return nil, err
	}
	defer release()
	chunk := maxBulkChunk
	if frameBytes > 0 && frameBytes < chunk {
		chunk = frameBytes // one transfer per frame for sub-MiB frames
	}
	st := &usbfsStream{d: d, chunk: chunk, slots: make([]streamSlot, urbWindow)}
	for i := range st.slots {
		st.slots[i].buf = make([]byte, chunk)
	}
	armed := 0
	for i := range st.slots {
		if err := st.rearm(i); err != nil {
			if armed == 0 {
				return nil, fmt.Errorf("astrocam: stream session: submit urb: %w", err)
			}
			break // window truncated (usbfs memory cap); the armed slots carry the stream
		}
		armed++
	}
	d.streamMu.Lock()
	if d.streams == nil {
		d.streams = map[*usbfsStream]struct{}{}
	}
	d.streams[st] = struct{}{}
	d.streamMu.Unlock()
	d.readActive.Add(1)
	return st, nil
}

// rearm submits slot i for the next segment.
func (st *usbfsStream) rearm(i int) error {
	s := &st.slots[i]
	s.urb = usbURB{typ: urbTypeBulk, endpoint: bulkEndpoint,
		buffer: uintptr(unsafe.Pointer(&s.buf[0])), bufferLength: int32(len(s.buf))}
	s.done, s.n, s.st = false, 0, 0
	s.seq = st.submit
	if err := st.d.ioctl(usbdevfsSubmitURB, unsafe.Pointer(&s.urb)); err != nil {
		s.armed = false // dead slot: no completion will come
		return err
	}
	s.armed = true
	st.submit++
	return nil
}

// reapOne reaps one completed URB without blocking (usbdevfsReapURBNDelay) and marks its slot
// done. Returns false on EAGAIN (nothing completed) and any other ioctl error.
func (st *usbfsStream) reapOne() (bool, error) {
	var p uintptr
	err := st.d.ioctl(usbdevfsReapURBNDelay, unsafe.Pointer(&p))
	if err == syscall.EAGAIN || err == syscall.EINTR {
		return false, nil
	}
	if err != nil {
		return false, err
	}
	for i := range st.slots {
		s := &st.slots[i]
		if p == uintptr(unsafe.Pointer(&s.urb)) {
			s.armed = false
			s.done = true
			s.n = int(s.urb.actualLength)
			s.st = s.urb.status
			return true, nil
		}
	}
	return true, nil // not ours (another reader's URB): ignore
}

// Next pulls one frame (len(buf) bytes) from the session, in segment order; a chunk may straddle
// frame boundaries (segOff carries the remainder to the next call). Returns a short count with a
// nil error on an idle stall (no completion for idle), an error on a hard URB status or a closed
// session.
// markDesync latches the session when a Next is about to return short. Matching darwinStream:
// a partial frame poisons the stream only if it actually consumed something, or left a segment
// part drained -- a Next that returned nothing and touched no segment leaves the stream aligned,
// so an idle poll on a quiet camera is not a desync.
func (st *usbfsStream) markDesync(copied, want int) {
	if copied < want && (copied > 0 || st.segOff > 0) {
		st.desync = true
	}
}

func (st *usbfsStream) Next(buf []byte, idle time.Duration) (int, error) {
	if len(buf) == 0 {
		return 0, nil
	}
	st.mu.Lock()
	defer st.mu.Unlock()
	if st.closed {
		return 0, errTransportClosed
	}
	if st.d.broken.Load() {
		return 0, errTransportBroken
	}
	// A previous Next stopped part way through a frame, so `next`/`segOff` point into the middle
	// of the stream and every frame from here would start at the wrong offset. In-place realign
	// is not possible: the caller closes the session and starts a new one.
	if st.desync {
		return 0, ErrStreamDesynced
	}
	if idle <= 0 { // see the backend contract in transport.go
		idle = defaultIdleBound
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
			if s.st != 0 && s.st != -int32(syscall.ENOENT) {
				// A halted or babbled URB: surface it; the caller resets the endpoint.
				st.markDesync(copied, len(buf))
				return copied, fmt.Errorf("astrocam: stream urb status %d", s.st)
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
			if st.segOff >= s.n { // segment fully consumed (a ZLP consumes at once): recycle
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
			st.markDesync(copied, len(buf))
			return copied, nil
		}
		got, err := st.reapOne()
		if err != nil {
			st.markDesync(copied, len(buf))
			return copied, err
		}
		if !got {
			if !progressed && time.Since(lastReal) > idle {
				st.markDesync(copied, len(buf))
				break // stall
			}
			time.Sleep(200 * time.Microsecond)
		}
	}
	return copied, nil
}

// Close discards and reaps the armed URBs, then frees the session. If an armed URB does not
// come back within drainTimeout the scratch buffers are parked in leakedIO and the device is
// poisoned (the kernel may still write into them). Idempotent.
func (st *usbfsStream) Close() error {
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
	err := st.drain()
	return err
}

// drain discards every armed URB and reaps until none is armed (or drainTimeout passes).
func (st *usbfsStream) drain() error {
	npend := 0
	for i := range st.slots {
		if st.slots[i].armed {
			_ = st.d.ioctl(usbdevfsDiscardURB, unsafe.Pointer(&st.slots[i].urb))
			npend++
		}
	}
	deadline := time.Now().Add(drainTimeout)
	for npend > 0 {
		var p uintptr
		err := st.d.ioctl(usbdevfsReapURBNDelay, unsafe.Pointer(&p))
		switch {
		case err == nil:
			for i := range st.slots {
				s := &st.slots[i]
				if p == uintptr(unsafe.Pointer(&s.urb)) && s.armed {
					s.armed = false
					npend--
					break
				}
			}
		case err == syscall.EAGAIN || err == syscall.EINTR:
			if time.Now().After(deadline) {
				return st.forfeit()
			}
			time.Sleep(time.Millisecond)
		default:
			return st.forfeit()
		}
	}
	return nil
}

// forfeit parks the session's URBs and buffers (the kernel may still DMA into them) and
// poisons the device.
func (st *usbfsStream) forfeit() error {
	leakedIOMu.Lock()
	for i := range st.slots {
		leakedIO = append(leakedIO, &st.slots[i].urb, st.slots[i].buf)
	}
	leakedIOMu.Unlock()
	st.d.broken.Store(true)
	return errTransportBroken
}
