package main

import (
	"fmt"
	"io"
	"time"

	"github.com/mikefsq/astrocam"
)

// logT wraps a Transport and logs every transfer (-v). It forwards every optional capability
// the driver dispatches on, so a -v run takes the same code paths as a plain run. Where the
// wrapped transport lacks one, logT falls back the way the driver's own call site does, with one
// exception: SuperSpeed returns false, while Open keeps the model's USB3 capability. Every
// hardware backend implements SuperSpeed, so this only shows on the stub.
type logT struct {
	t     astrocam.Transport
	w     io.Writer
	start time.Time
}

func (l *logT) ts() string { return fmt.Sprintf("[%8.3fs]", time.Since(l.start).Seconds()) }

func (l *logT) ControlOut(b uint8, wv, wi uint16, d []byte) error {
	err := l.t.ControlOut(b, wv, wi, d)
	fmt.Fprintf(l.w, "%s OUT  req=0x%02x val=0x%04x idx=0x%04x len=%d%s\n", l.ts(), b, wv, wi, len(d), res(err))
	return err
}

func (l *logT) ControlIn(b uint8, wv, wi uint16, d []byte) (int, error) {
	n, err := l.t.ControlIn(b, wv, wi, d)
	dump := ""
	if n > 0 {
		m := n
		if m > 8 {
			m = 8
		}
		dump = fmt.Sprintf(" data=% x", d[:m])
	}
	fmt.Fprintf(l.w, "%s IN   req=0x%02x val=0x%04x idx=0x%04x len=%d -> %d%s%s\n", l.ts(), b, wv, wi, len(d), n, dump, res(err))
	return n, err
}

// ControlOutUngated forwards UngatedControlSender (the ST4 pulse path); falls back to the
// gated ControlOut.
func (l *logT) ControlOutUngated(b uint8, wv, wi uint16) error {
	var err error
	if u, ok := l.t.(astrocam.UngatedControlSender); ok {
		err = u.ControlOutUngated(b, wv, wi)
	} else {
		err = l.t.ControlOut(b, wv, wi, nil)
	}
	fmt.Fprintf(l.w, "%s OUT* req=0x%02x val=0x%04x idx=0x%04x (ungated)%s\n", l.ts(), b, wv, wi, res(err))
	return err
}

func (l *logT) BulkRead(buf []byte, to time.Duration) (int, error) {
	fmt.Fprintf(l.w, "%s BULK read<=%d timeout=%s start...\n", l.ts(), len(buf), to)
	n, err := l.t.BulkRead(buf, to)
	fmt.Fprintf(l.w, "%s BULK <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

// BulkReadQuiet forwards QuietBulkReader; falls back to BulkRead.
func (l *logT) BulkReadQuiet(buf []byte, quiet, to time.Duration) (int, error) {
	q, ok := l.t.(astrocam.QuietBulkReader)
	if !ok {
		return l.BulkRead(buf, to)
	}
	fmt.Fprintf(l.w, "%s BULKQ read<=%d quiet=%s timeout=%s start...\n", l.ts(), len(buf), quiet, to)
	n, err := q.BulkReadQuiet(buf, quiet, to)
	fmt.Fprintf(l.w, "%s BULKQ <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

// ReadFrameStream forwards the windowed-stream read (FrameStreamer); falls back to BulkRead.
func (l *logT) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	fs, ok := l.t.(astrocam.FrameStreamer)
	if !ok {
		return l.BulkRead(buf, total)
	}
	fmt.Fprintf(l.w, "%s STREAM read<=%d idle=%s total=%s start...\n", l.ts(), len(buf), idle, total)
	n, err := fs.ReadFrameStream(buf, idle, total)
	fmt.Fprintf(l.w, "%s STREAM <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

// ReadFrameStreamPrequeued forwards PrequeuedFrameStreamer; falls back to ReadFrameStream.
func (l *logT) ReadFrameStreamPrequeued(buf []byte, idle, total time.Duration) (int, error) {
	p, ok := l.t.(astrocam.PrequeuedFrameStreamer)
	if !ok {
		return l.ReadFrameStream(buf, idle, total)
	}
	fmt.Fprintf(l.w, "%s PREQ read<=%d idle=%s total=%s start...\n", l.ts(), len(buf), idle, total)
	n, err := p.ReadFrameStreamPrequeued(buf, idle, total)
	fmt.Fprintf(l.w, "%s PREQ <- %d bytes%s\n", l.ts(), n, res(err))
	return n, err
}

// AbortRead / ArmRead forward ReadAborter (no-ops when the wrapped transport lacks it).
func (l *logT) AbortRead() {
	if ra, ok := l.t.(astrocam.ReadAborter); ok {
		ra.AbortRead()
	}
	fmt.Fprintf(l.w, "%s ABORT-READ\n", l.ts())
}

func (l *logT) ArmRead() {
	if ra, ok := l.t.(astrocam.ReadAborter); ok {
		ra.ArmRead()
	}
	fmt.Fprintf(l.w, "%s ARM-READ\n", l.ts())
}

// StartStream forwards StreamStarter; only the session's open and close are logged, not its
// per-frame reads.
func (l *logT) StartStream(frameBytes int, total time.Duration) (astrocam.FrameStream, error) {
	ss, ok := l.t.(astrocam.StreamStarter)
	if !ok {
		return nil, fmt.Errorf("transport has no resident stream session")
	}
	sess, err := ss.StartStream(frameBytes, total)
	fmt.Fprintf(l.w, "%s STREAM-SESSION open frame=%d total=%s%s\n", l.ts(), frameBytes, total, res(err))
	if err != nil {
		return nil, err
	}
	return &logStream{FrameStream: sess, l: l}, nil
}

// logStream logs a resident session's close; frames pass straight through, zero-copy included.
type logStream struct {
	astrocam.FrameStream
	l *logT
}

func (s *logStream) Close() error {
	err := s.FrameStream.Close()
	fmt.Fprintf(s.l.w, "%s STREAM-SESSION close%s\n", s.l.ts(), res(err))
	return err
}

func (s *logStream) NextZC(idle time.Duration) ([]byte, error) {
	if zc, ok := s.FrameStream.(astrocam.FrameStreamZC); ok {
		return zc.NextZC(idle)
	}
	return nil, fmt.Errorf("stream session has no zero-copy path")
}

func (s *logStream) Release() {
	if zc, ok := s.FrameStream.(astrocam.FrameStreamZC); ok {
		zc.Release()
	}
}

func (l *logT) Close() error { return l.t.Close() }

// SuperSpeed forwards the optional link-speed report.
func (l *logT) SuperSpeed() bool {
	if sr, ok := l.t.(interface{ SuperSpeed() bool }); ok {
		return sr.SuperSpeed()
	}
	return false
}

// Describe forwards the transport's bring-up description.
func (l *logT) Describe() string {
	if d, ok := l.t.(interface{ Describe() string }); ok {
		return d.Describe()
	}
	return "(no description)"
}

// ResetEndpoint forwards EndpointResetter (no-op when the wrapped transport lacks it).
func (l *logT) ResetEndpoint(ep uint8) error {
	r, ok := l.t.(astrocam.EndpointResetter)
	if !ok {
		return nil
	}
	err := r.ResetEndpoint(ep)
	fmt.Fprintf(l.w, "%s RESET ep=0x%02x%s\n", l.ts(), ep, res(err))
	return err
}

// ResetDevice forwards DeviceResetter; errors when the wrapped transport has none.
func (l *logT) ResetDevice() error {
	r, ok := l.t.(astrocam.DeviceResetter)
	if !ok {
		return fmt.Errorf("transport has no device reset")
	}
	err := r.ResetDevice()
	fmt.Fprintf(l.w, "%s RESET-DEVICE%s\n", l.ts(), res(err))
	return err
}

func res(err error) string {
	if err != nil {
		return "  ERR: " + err.Error()
	}
	return ""
}
