package astrocam

// StubTransport is an in-memory ZWO-camera Transport for hardware-free, end-to-end tests of
// the capture pipeline (Init → SetROI → SetGain → SetExposure → StartExposure → data read),
// exercising the per-sensor Workers and both read paths. No cgo, no hardware, available on
// every platform.
//
// It speaks the real control-transfer convention (protocol.go): register writes are stored
// and echoed back on the matching reads, the FX3 vendor commands (stop/start/flush/bank/GPIF)
// are accepted as no-ops, ReadFirmwareVer returns a configurable version, and every bulk /
// windowed-stream read is filled with a synthetic frame. It satisfies Transport plus
// EndpointResetter and FrameStreamer.

import (
	"sync"
	"time"
)

// StubXfer is one recorded control transfer, exposed via StubTransport.Log.
type StubXfer struct {
	In             bool // true = ControlIn, false = ControlOut
	BRequest       uint8
	WValue, WIndex uint16
	Len            int
}

// StubTransport is an in-memory stub ZWO camera. Use NewStubTransport; the zero value
// is not usable. Fields are safe to set before a capture and to read after it.
type StubTransport struct {
	mu   sync.Mutex
	sony map[uint16]uint16 // WriteSONYREG/ReadSONYREG     (0xB6/0xB7)
	cam  map[uint16]uint16 // WriteCameraReg/ReadCameraReg (0xA6/0xA7)
	fpga map[uint16]uint16 // WriteFPGAREG/ReadFPGAREG     (0xBD/0xBC)

	// Firmware is returned for ReadFirmwareVer (0xAD). Its HIGH byte is the init
	// subtype gate; the default 0x5056 is the real ASI6200 value (subtype 0x50 ≥ 0x12,
	// the all-FPGA init path). Set before Init to exercise a different branch.
	Firmware uint16
	// Serial is returned for the GetSerialNumber read (0xC8) — the 8-byte ASI_ID.
	Serial Serial
	// Frame fills a caller buffer with synthetic pixel data; default is a 16-bit ramp.
	Frame func(buf []byte)
	// Log records every control transfer, in order.
	Log []StubXfer
	// Reads counts the bulk/stream frame reads served.
	Reads int
}

// NewStubTransport returns a ready stub camera transport.
func NewStubTransport() *StubTransport {
	return &StubTransport{
		sony:     map[uint16]uint16{},
		cam:      map[uint16]uint16{},
		fpga:     map[uint16]uint16{},
		Firmware: 0x5056,
		Frame:    RampFrame,
	}
}

// Reg returns the last value written to a sensor (Sony-bus) register.
func (t *StubTransport) Reg(reg uint16) uint16 {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.sony[reg]
}

// FPGAReg returns the last value written to an FPGA register.
func (t *StubTransport) FPGAReg(reg uint16) uint16 {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.fpga[reg]
}

func (t *StubTransport) ControlOut(bRequest uint8, wValue, wIndex uint16, data []byte) error {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.Log = append(t.Log, StubXfer{In: false, BRequest: bRequest, WValue: wValue, WIndex: wIndex, Len: len(data)})
	switch bRequest {
	case reqWriteSonyReg:
		t.sony[wValue] = wIndex & 0xff
	case reqWriteCamReg:
		t.cam[wValue] = wIndex
	case reqWriteFPGAReg:
		t.fpga[wValue] = wIndex
	}
	// Any other bRequest is an FX3 vendor command (0xAA/0xA9/0xAF/0xAE/0xBE) — no state.
	return nil
}

func (t *StubTransport) ControlIn(bRequest uint8, wValue, wIndex uint16, data []byte) (int, error) {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.Log = append(t.Log, StubXfer{In: true, BRequest: bRequest, WValue: wValue, WIndex: wIndex, Len: len(data)})
	if bRequest == reqSerialNumber { // 8 raw bytes, not a little-endian register
		n := copy(data, t.Serial[:])
		return n, nil
	}
	var v uint16
	switch bRequest {
	case reqReadSonyReg:
		v = t.sony[wValue]
	case reqReadFPGAReg:
		v = t.fpga[wValue]
	case reqReadCamReg:
		v = t.cam[wValue]
	case reqFirmwareVer:
		v = t.Firmware
	}
	for i := range data { // little-endian into the requested length
		switch i {
		case 0:
			data[i] = byte(v)
		case 1:
			data[i] = byte(v >> 8)
		default:
			data[i] = 0
		}
	}
	return len(data), nil
}

// BulkRead serves one synthetic frame (the small-frame / USB2 path).
func (t *StubTransport) BulkRead(buf []byte, timeout time.Duration) (int, error) { return t.serve(buf) }

// ReadFrameStream satisfies FrameStreamer (the large-frame USB3 windowed pump).
func (t *StubTransport) ReadFrameStream(buf []byte, idle, total time.Duration) (int, error) {
	return t.serve(buf)
}

func (t *StubTransport) serve(buf []byte) (int, error) {
	t.mu.Lock()
	t.Reads++
	frame := t.Frame
	t.mu.Unlock()
	if frame != nil {
		frame(buf)
	}
	return len(buf), nil
}

// ResetEndpoint satisfies EndpointResetter (the stub pipe never stalls).
func (t *StubTransport) ResetEndpoint(ep uint8) error { return nil }

func (t *StubTransport) Close() error { return nil }

// RampFrame fills buf with a deterministic little-endian 16-bit ramp (pixel i = i),
// so a test can assert the frame round-tripped intact.
func RampFrame(buf []byte) {
	for i := 0; i+1 < len(buf); i += 2 {
		v := uint16((i / 2) & 0xffff)
		buf[i] = byte(v)
		buf[i+1] = byte(v >> 8)
	}
}
