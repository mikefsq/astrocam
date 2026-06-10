// Package asicam is a pure-Go driver for ASI-class cameras, structured as a
// vendor transport + per-sensor profiles (Linux-V4L2-driver style) + a model
// table — so "support all cameras on a given Sony die" is a data problem, not a
// code rewrite. It is VENDOR-INDEPENDENT: ZWO (VID 0x03C3) and PlayerOne (VID
// 0xA0A0) share one profile per die; the per-vendor difference is the Regmap
// opcode dialect and the gain/offset unit scale, selected from the regmap's VID.
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
// STATUS (2026-06-04): the architecture, control plane, real backends (macOS IOUSBHost,
// Linux usbfs, Windows WinUSB), and the bulk data plane are all implemented and
// HARDWARE-VALIDATED. Three sensor profiles capture real frames end-to-end — imx174
// (ASI174 Mini, USB2), imx290 (ASI290MM), and imx455 (ASI6200 MM/MC, USB3 + USB2,
// full 122 MB frames). The remaining work is breadth: further sensor profiles are decoded
// from the static `.a` and added as needed (an open-ended set, one per Sony die), each
// carrying an UNVERIFIED header / TODOs until hardware confirms its register table + gain curve.
//
// MULTI-VENDOR (2026-06): PlayerOne IMX455/IMX571 are wired and unit-tested — the poaRegmap
// opcode dialect (protocol_poa.go) plus gain/offset/caps that dispatch on the regmap's VID
// over the SAME shared profiles. PlayerOne being an independent implementation of the same
// registers also second-source confirms the ASI 455/571 decode. What's left for a live
// PlayerOne camera is hardware-gated: PID->sensor rows (PlayerOne ships no static PID
// table) and capture-path validation.
package asicam

import "time"

// Transport is the per-vendor USB seam. ZWO's protocol (protocol.go) is built on
// top of it; a real backend wraps libusb (gousb / usbfs). Control transfers are
// always vendor-typed: ControlOut uses bmRequestType 0x40, ControlIn 0xC0.
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
// submitted and resubmitted on EP 0x81, delivering completed chunks in order.
// This is the async pump (initAsyncXfer/startAsyncXfer) —
// a window of ~200 MB / chunk in flight is what reaches USB3 line rate, which a
// one-shot BulkRead per frame cannot. Frame data flows through here.
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

// FrameStreamer is an optional Transport capability: read one whole frame with the
// continuous windowed pump (the USB3 startAsyncXfer window) — a window of transfers
// kept cycling on EP 0x81, copied contiguously so a short packet at a burst boundary
// can't leave a gap, until the frame is in. Needed for large USB3 frames (IMX455/571)
// that a one-shot BulkRead truncates; returns short on an idle stall so the caller can
// flush/recover and continue into buf[n:].
type FrameStreamer interface {
	ReadFrameStream(buf []byte, idle, total time.Duration) (int, error)
}

// FrameStream is a resident streaming session (the video / planetary-burst path): the
// windowed pump is primed ONCE and Next pulls one frame per call, so the per-frame setup
// cost (thread spawn, window prime, teardown) that ReadFrameStream pays every call is paid
// just once for the whole burst. Close aborts and frees the session.
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
// returns a slice ALIASING the session's internal buffer (no per-frame memcpy), valid until
// Release re-arms the slot. Only meaningful when each frame is a single transfer (sub-MiB
// ROI). The caller must consume the slice before Release.
type FrameStreamZC interface {
	NextZC(idle time.Duration) ([]byte, error)
	Release()
}

// EndpointResetter is an optional Transport capability: clear a stalled/streaming
// bulk endpoint (libusb_clear_halt / IOKit ClearPipeStall) to drop stale data
// before a capture — ResetEndPoint(0x81), issued before each session.
type EndpointResetter interface {
	ResetEndpoint(ep uint8) error
}

// DeviceResetter is an optional Transport capability: a USB bus reset of the whole
// device (libusb_reset_device / IOKit ResetDevice) — the last-resort recovery when a
// pipe-level reset can't unwedge the camera. It wipes the device's state, so the caller
// must re-Init afterwards; the snap path uses it only as a final give-up to leave the
// device clean for the next session.
type DeviceResetter interface {
	ResetDevice() error
}

// Regmap is the sensor-register interface a Sensor profile writes to — the
// pure-Go analogue of the Linux kernel's regmap. The ZWO implementation
// (zwoRegmap) carries each access as a control transfer.
//
// Two register spaces are exposed because the SDK uses two distinct
// vendor requests with different routing inside the FX3 bridge:
//
//   - WriteReg/ReadReg   → the sensor's own registers (Sony WriteSONYREG 0xB6 /
//     ReadSONYREG 0xB7 by default; a non-Sony profile can select the generic
//     camera-register bus 0xA6 via Sensor.Bus).
//   - WriteFPGAReg/ReadFPGAReg → the camera FPGA's registers (0xBD / 0xBC).
//     Exposure timing (VMAX) and HBLANK live here, NOT on the sensor — that is
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
