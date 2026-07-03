// Package asicam is a pure-Go driver for ASI-class cameras: a vendor transport +
// per-sensor profiles + a model table. Vendor-independent: ZWO (VID 0x03C3) and
// PlayerOne (VID 0xA0A0) share one profile per Sony die; the per-vendor difference is
// the Regmap opcode dialect and the gain/offset unit scale, selected from the regmap's VID.
//
// Layering:
//
//	Transport     the libusb seam (control transfers + bulk) — vendor-neutral
//	protocol.go   ZWO's control-transfer register dialect (zwoRegmap) over a Transport
//	protocol_poa  PlayerOne's dialect (poaRegmap) — different opcodes, same Regmap
//	Vendor        VID -> { Name, newRegmap } (vendor.go); each die maps to one Vendor
//	Regmap        the sensor-register interface the profiles write to (incl. VID())
//	Sensor        a per-chip profile: init table + gain/exposure/ROI ops (vendor-dispatched)
//	Models        (VID,PID) -> { sensor, mono|color, cooled, usb3 }
//	Camera        binds a Transport + Model + Sensor into the control flow
//
// Backends: macOS IOUSBHost, Linux usbfs, Windows WinUSB. Sensor profiles validated on
// hardware: imx174 (ASI174 Mini, USB2), imx290 (ASI290MM), imx455 (ASI6200 MM/MC, USB3 +
// USB2, 122 MB frames). PlayerOne IMX455/IMX571 are wired via poaRegmap (protocol_poa.go)
// over the same shared profiles, unit-tested but not yet validated on a live PlayerOne camera.
package astrocam

import "time"

// Transport is the per-vendor USB seam. Control transfers are vendor-typed:
// ControlOut uses bmRequestType 0x40, ControlIn 0xC0.
type Transport interface {
	// ControlOut issues a vendor OUT control transfer (bmRequestType 0x40).
	ControlOut(bRequest uint8, wValue, wIndex uint16, data []byte) error
	// ControlIn issues a vendor IN control transfer (bmRequestType 0xC0) and
	// fills data; returns the byte count read.
	ControlIn(bRequest uint8, wValue, wIndex uint16, data []byte) (int, error)
	// BulkRead reads from the bulk-IN endpoint (small one-shots / low-rate frames).
	// High-throughput frame data should use BulkStreamer instead.
	BulkRead(buf []byte, timeout time.Duration) (int, error)
	Close() error
}

// Stream is a high-throughput bulk-IN read: the backend keeps a pool of buffers
// submitted and resubmitted on EP 0x81, delivering completed chunks in order. The
// async pump reaches USB3 line rate, which a one-shot BulkRead per frame cannot.
type Stream interface {
	// Next returns the next completed chunk; its length may be < bufSize on the
	// final short packet (the FX3 frame-end delimiter). The slice is valid until
	// the following Next call.
	Next(timeout time.Duration) ([]byte, error)
	Close() error
}

// BulkStreamer is an optional Transport capability. A backend that implements the
// async streaming pump (usbfs URBs / WinUSB overlapped / IOUSBHost async pipe)
// satisfies it; the data plane uses it when present and falls back to BulkRead.
type BulkStreamer interface {
	// BulkStream opens a streaming read with nBuffers buffers of bufSize bytes
	// kept cycling on the bulk-IN endpoint.
	BulkStream(bufSize, nBuffers int) (Stream, error)
}

// FrameStreamer is an optional Transport capability: read one whole frame with a
// window of transfers kept cycling on EP 0x81, copied contiguously so a short packet
// at a burst boundary can't leave a gap. Needed for large USB3 frames (IMX455/571)
// that a one-shot BulkRead truncates; returns short on an idle stall so the caller can
// flush/recover and continue into buf[n:].
type FrameStreamer interface {
	ReadFrameStream(buf []byte, idle, total time.Duration) (int, error)
}

// PrequeuedFrameStreamer reads one frame the way the ASI SDK's capture thread does: a batch of
// async bulk-IN transfers covering the frame exactly (initAsyncXfer/startAsyncXfer, 1 MiB slices
// on EP 0x81, last slice = the remainder), all queued BEFORE the frame arrives so the transfer
// overlaps the sensor read and the pipe never idles. The per-frame one-at-a-time ReadFrameStream
// leaves a gap between transfers and doesn't overlap; on a USB2 HighSpeed link that shears the
// frame. One frame per batch (a retry is a fresh call); the DDR mid-frame-hold path keeps using
// ReadFrameStream.
type PrequeuedFrameStreamer interface {
	ReadFrameStreamPrequeued(buf []byte, idle, total time.Duration) (int, error)
}

// FrameStream is a resident streaming session (the video / planetary-burst path): the
// windowed pump is primed once and Next pulls one frame per call, so the per-frame setup
// cost (thread spawn, window prime, teardown) is paid once for the whole burst. Close
// aborts and frees the session.
type FrameStream interface {
	Next(buf []byte, idle time.Duration) (int, error)
	Close() error
}

// StreamStarter is an optional Transport capability: open a resident FrameStream session.
// Backends without it (linux/windows/stub) fall back to per-frame reads.
type StreamStarter interface {
	StartStream(frameBytes int, total time.Duration) (FrameStream, error)
}

// FrameStreamZC is an optional zero-copy extension a FrameStream may implement: NextZC
// returns a slice aliasing the session's internal buffer (no per-frame memcpy), valid until
// Release re-arms the slot. Only meaningful when each frame is a single transfer (sub-MiB
// ROI). The caller must consume the slice before Release.
type FrameStreamZC interface {
	NextZC(idle time.Duration) ([]byte, error)
	Release()
}

// EndpointResetter is an optional Transport capability: clear a stalled bulk endpoint
// (per-platform clear-halt) to drop stale data before a capture — ResetEndpoint(0x81),
// issued before each session.
type EndpointResetter interface {
	ResetEndpoint(ep uint8) error
}

// DeviceResetter is an optional Transport capability: a USB bus reset of the whole
// device — last-resort recovery when a pipe-level reset can't unwedge the camera. It
// wipes the device's state, so the caller must re-Init afterwards.
type DeviceResetter interface {
	ResetDevice() error
}

// Regmap is the sensor-register interface a Sensor profile writes to. The ZWO
// implementation (zwoRegmap) carries each access as a control transfer. Two register
// spaces, routed through two distinct vendor requests in the FX3 bridge:
//
//   - WriteReg/ReadReg   → the sensor's own registers (Sony WriteSONYREG 0xB6 /
//     ReadSONYREG 0xB7 by default; a non-Sony profile can select the generic
//     camera-register bus 0xA6 via Sensor.Bus).
//   - WriteFPGAReg/ReadFPGAReg → the camera FPGA's registers (0xBD / 0xBC).
//     Exposure timing (VMAX) and HBLANK live here, not on the sensor — that is
//     why every profile's SetExposure needs this path. See SetVMAX.
type Regmap interface {
	WriteReg(reg, val uint16) error
	// WriteRegBits sets bits [lo:hi] of reg to val (read-modify-write).
	WriteRegBits(reg uint16, lo, hi uint8, val uint16) error
	ReadReg(reg uint16) (uint16, error)
	// WriteFPGAReg writes a camera-FPGA register (WriteFPGAREG 0xBD).
	WriteFPGAReg(reg, val uint16) error
	// ReadFPGAReg reads a camera-FPGA register (ReadFPGAREG 0xBC).
	ReadFPGAReg(reg uint16) (uint16, error)
	// VID reports the USB vendor id of the dialect this regmap speaks, so a sensor
	// profile shared across vendors (same Sony die) can select the vendor-specific
	// gain/offset encoding at call time. zwoRegmap -> 0x03C3, poaRegmap -> 0xA0A0.
	VID() uint16
}
