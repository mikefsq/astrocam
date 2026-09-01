// Package astrocam is a pure-Go driver for FX3-bridged Sony-sensor astronomy cameras: a
// vendor transport, per-sensor profiles, and a model table.
//
// Layering:
//
//	Transport     the USB physical layer
//	protocol.go   ZWO's control-transfer register dialect (zwoRegmap) over a Transport
//	protocol_poa  PlayerOne's dialect (poaRegmap): different opcodes, same Regmap
//	Vendor        VID -> { Name, newRegmap } (vendor.go); each die maps to one Vendor
//	Regmap        the sensor-register interface the profiles write to (incl. VID())
//	Sensor        a per-chip profile: init table + gain/exposure/ROI ops (vendor-dispatched)
//	Models        (VID,PID) -> { sensor, mono|color, cooled, usb3 }
//	Camera        binds a Transport + Model + Sensor into the control flow
package astrocam

import (
	"errors"
	"sync"
	"time"
)

// ErrStreamDesynced is returned by a session Next after an earlier Next ended part way through a
// frame. The segment stream cannot be realigned in place, so the caller must Close the session
// and StartStream again rather than abandon the capture. Every backend latches it: a short read
// that consumed anything, or that left a segment part drained, makes every later frame in the
// session start at the wrong offset.
var ErrStreamDesynced = errors.New("astrocam: stream session desynced by a short read; close and restart it")

// Default bounds, substituted when a caller passes a non-positive value.
const (
	defaultIdleBound    = 800 * time.Millisecond // no-completion stall bound, after first data
	defaultTotalBound   = 5 * time.Second        // whole-read bound
	defaultTimeoutBound = 2 * time.Second        // single-shot BulkRead bound
)

// Transport is the USB seam each backend implements: vendor control transfers (bmRequestType
// 0x40 OUT, 0xC0 IN) and bulk-IN frame reads.
type Transport interface {
	// ControlOut issues a vendor OUT control transfer.
	ControlOut(bRequest uint8, wValue, wIndex uint16, data []byte) error
	// ControlIn issues a vendor IN control transfer, fills data, and returns the byte count.
	ControlIn(bRequest uint8, wValue, wIndex uint16, data []byte) (int, error)
	// BulkRead reads one whole frame from the bulk-IN endpoint. A short or empty read returns
	// (n, nil). An error means a failure the caller cannot retry past.
	BulkRead(buf []byte, timeout time.Duration) (int, error)
	// Close releases the device handle.
	Close() error
}

// Attached is implemented by a Transport that knows which attachment of the device it holds
// (DeviceInfo.Attachment). 0 means the platform offers no such identity.
type Attached interface {
	Attachment() uint64
}

// FrameStreamer reads one whole frame with a window of transfers cycling on EP 0x81. Large USB3
// frames (IMX455/571) need it: a one-shot BulkRead truncates them.
type FrameStreamer interface {
	// ReadFrameStream fills buf and returns short on an idle stall, so the caller can continue
	// into buf[n:].
	ReadFrameStream(buf []byte, idle, total time.Duration) (int, error)
}

// PrequeuedFrameStreamer reads one frame with its whole batch of transfers queued before the
// frame arrives. ReadFrameStream leaves a gap between transfers and shears a free-run frame on a
// USB2 HighSpeed link.
type PrequeuedFrameStreamer interface {
	// ReadFrameStreamPrequeued reads one frame per call; a retry is a fresh call.
	ReadFrameStreamPrequeued(buf []byte, idle, total time.Duration) (int, error)
}

// FrameStream is a resident streaming session for the video and planetary-burst path.
type FrameStream interface {
	// Next pulls one frame into buf.
	Next(buf []byte, idle time.Duration) (int, error)
	// Close aborts the session and frees it.
	Close() error
}

// LinkForcer is the optional Transport capability that pins the reported link speed. The
// negotiated speed is a hardware matter and cannot be changed from the host: this makes the
// transport REPORT High Speed on a SuperSpeed link, so everything downstream behaves as it would
// on USB2 — the bandwidth budget and GPIF divider the profile picks, Open's default bandwidth
// percentage, and the EP0 pacing that only runs on USB2 readouts. The wire stays SuperSpeed, so
// it exercises the USB2 CONFIGURATION at SuperSpeed throughput, which is what makes it useful for
// separating a timing-model bug from a link bug.
type LinkForcer interface {
	ForceUSB2(on bool)
}

// StreamStarter opens a resident FrameStream session.
type StreamStarter interface {
	StartStream(frameBytes int, total time.Duration) (FrameStream, error)
}

// FrameStreamZC is a zero-copy extension a FrameStream may implement, for frames small enough to
// arrive in one transfer (sub-MiB ROI).
type FrameStreamZC interface {
	// NextZC returns a slice aliasing the session's buffer, valid until Release.
	NextZC(idle time.Duration) ([]byte, error)
	// Release re-arms the slot the last NextZC slice aliases.
	Release()
}

// ReadAborter breaks a frame read in flight, so a stop does not wait out a gated whole-frame
// read.
type ReadAborter interface {
	// AbortRead breaks the read in flight and fails later reads until ArmRead.
	AbortRead()
	// ArmRead clears the abort state. The camera calls it from StartExposure and StartVideo.
	ArmRead()
}

// QuietBulkReader reads one whole frame whose first quiet duration is a host-timed integration.
type QuietBulkReader interface {
	// BulkReadQuiet arms the transfers, then releases ioMu until the quiet window elapses or
	// the first transfer completes. Callers must undershoot the real integration. quiet 0 is
	// BulkRead.
	BulkReadQuiet(buf []byte, quiet, timeout time.Duration) (int, error)
}

// UngatedControlSender issues a vendor OUT without ioMu, so a pulse edge does not queue behind a
// frame read.
type UngatedControlSender interface {
	// ControlOutUngated serves the ST4 guide pulses (0xB0 on, 0xB1 off) only. An EP0 read
	// mid-readout wedges the GPIF.
	ControlOutUngated(bRequest uint8, wValue, wIndex uint16) error
}

// EndpointResetter clears a stalled bulk endpoint.
type EndpointResetter interface {
	ResetEndpoint(ep uint8) error
}

// DeviceResetter resets the whole device on the bus. It wipes device state, so the caller must
// re-Init.
type DeviceResetter interface {
	ResetDevice() error
}

// Regmap is the sensor-register interface a Sensor profile writes to. WriteReg and ReadReg reach
// the sensor's own registers; WriteFPGAReg and ReadFPGAReg reach the camera FPGA, which holds the
// exposure timing (VMAX) and HBLANK.
type Regmap interface {
	WriteReg(reg, val uint16) error
	// WriteRegBits sets bits [lo:hi] of reg to val (read-modify-write).
	WriteRegBits(reg uint16, lo, hi uint8, val uint16) error
	ReadReg(reg uint16) (uint16, error)
	WriteFPGAReg(reg, val uint16) error
	ReadFPGAReg(reg uint16) (uint16, error)
	// VID reports the USB vendor id of the dialect this regmap speaks, so a shared profile can
	// select the vendor's gain/offset encoding at call time.
	VID() uint16
}

// usb2InPace is the minimum spacing between vendor IN control transfers while a frame read is in
// flight on a USB2 HighSpeed link. 500/s took the FX3 control plane down on an ASI6200MC.
const usb2InPace = 20 * time.Millisecond

// inPacer spaces calls at least min apart; concurrent callers queue in order.
type inPacer struct {
	mu   sync.Mutex
	last time.Time
	min  time.Duration
}

// wait blocks until min has elapsed since the previous wait returned, then claims the slot.
func (p *inPacer) wait() {
	p.mu.Lock()
	defer p.mu.Unlock()
	if d := p.min - time.Since(p.last); d > 0 {
		time.Sleep(d)
	}
	p.last = time.Now()
}
