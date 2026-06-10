package astrocam

import (
	"testing"
	"time"
)

// fakeTransport records control transfers and serves canned IN replies, so the
// protocol is testable with no hardware and no libusb.
type fakeTransport struct {
	out     []ctrlCall
	inReply map[uint8][]byte
}

type ctrlCall struct {
	bRequest       uint8
	wValue, wIndex uint16
}

func (f *fakeTransport) ControlOut(b uint8, wv, wi uint16, _ []byte) error {
	f.out = append(f.out, ctrlCall{b, wv, wi})
	return nil
}
func (f *fakeTransport) ControlIn(b uint8, _, _ uint16, d []byte) (int, error) {
	if r, ok := f.inReply[b]; ok {
		return copy(d, r), nil
	}
	return 0, nil
}
func (f *fakeTransport) BulkRead(_ []byte, _ time.Duration) (int, error) { return 0, nil }
func (f *fakeTransport) Close() error                                    { return nil }

// testSensor is a minimal in-package profile for exercising Open + the control
// flow without importing the sensors package.
var testSensor = Sensor{
	Name:    "TEST",
	Info:    CameraInfo{MaxWidth: 100, MaxHeight: 100, BitDepth: 12},
	SetGain: func(rm Regmap, g int) error { return rm.WriteReg(0x10, uint16(g)) },
}

func TestOpenAndControl(t *testing.T) {
	Register(ZWO.VID, 0x0001, Model{Name: "Test", Sensor: &testSensor})
	f := &fakeTransport{}
	c, err := Open(f, ZWO.VID, 0x0001)
	if err != nil {
		t.Fatal(err)
	}
	if c.Name() != "Test" || c.Sensor().Name != "TEST" {
		t.Errorf("got %q / %q", c.Name(), c.Sensor().Name)
	}
	if err := c.SetGain(50); err != nil {
		t.Fatal(err)
	}
	if len(f.out) != 1 || f.out[0] != (ctrlCall{reqWriteSonyReg, 0x10, 50}) {
		t.Errorf("SetGain produced %+v", f.out)
	}
	if _, err := Open(f, ZWO.VID, 0xFFFF); err == nil {
		t.Error("unregistered PID should error")
	}
}

// TestWriteRegBits checks the read-modify-write bitfield path of the protocol.
func TestWriteRegBits(t *testing.T) {
	f := &fakeTransport{inReply: map[uint8][]byte{reqReadSonyReg: {0xf0}}}
	rm := &zwoRegmap{t: f}
	if err := rm.WriteRegBits(0x10, 0, 3, 0x5); err != nil {
		t.Fatal(err)
	}
	last := f.out[len(f.out)-1]
	if last != (ctrlCall{reqWriteSonyReg, 0x10, 0xf5}) {
		t.Errorf("RMW wrote %+v, want reg 0x10 = 0xf5", last)
	}
}

// TestFPGAandVMAX checks the FPGA register space and the SetVMAX helper match
// WriteFPGAREG (0xBD) / SetFPGAVMAX (strobe + 0x10/11/12).
func TestFPGAandVMAX(t *testing.T) {
	f := &fakeTransport{}
	rm := &zwoRegmap{t: f}
	if err := SetVMAX(rm, 0x123456); err != nil {
		t.Fatal(err)
	}
	want := []ctrlCall{
		{reqWriteFPGAReg, 0x01, 1},    // strobe open
		{reqWriteFPGAReg, 0x10, 0x56}, // VMAX byte 0
		{reqWriteFPGAReg, 0x11, 0x34}, // VMAX byte 1
		{reqWriteFPGAReg, 0x12, 0x12}, // VMAX byte 2
		{reqWriteFPGAReg, 0x01, 0},    // strobe close (commit; pcap-confirmed)
	}
	if len(f.out) != len(want) {
		t.Fatalf("SetVMAX produced %d calls, want %d: %+v", len(f.out), len(want), f.out)
	}
	for i := range want {
		if f.out[i] != want[i] {
			t.Errorf("call %d = %+v, want %+v", i, f.out[i], want[i])
		}
	}
}

// TestCameraBus confirms a BusCamera sensor routes WriteReg to 0xA6, not 0xB6.
func TestCameraBus(t *testing.T) {
	f := &fakeTransport{}
	rm := &zwoRegmap{t: f, bus: BusCamera}
	if err := rm.WriteReg(0x05, 0x99); err != nil {
		t.Fatal(err)
	}
	if f.out[0] != (ctrlCall{reqWriteCamReg, 0x05, 0x99}) {
		t.Errorf("BusCamera write = %+v, want req 0xA6", f.out[0])
	}
}

func TestFirmwareVersion(t *testing.T) {
	f := &fakeTransport{inReply: map[uint8][]byte{reqFirmwareVer: {0x09, 0x03}}}
	Register(ZWO.VID, 0x0002, Model{Name: "Test2", Sensor: &testSensor})
	c, _ := Open(f, ZWO.VID, 0x0002)
	if v, err := c.FirmwareVersion(); err != nil || v != 0x0309 {
		t.Fatalf("FirmwareVersion() = 0x%x, %v; want 0x0309", v, err)
	}
}

// TestSerialNumber locks GetSerialNumber: a single vendor
// control-IN, bRequest 0xC8 / wValue 0 / wIndex 0, returning the 8 raw ASI_ID
// bytes verbatim (no little-endian register fold), formatted as lowercase hex.
func TestSerialNumber(t *testing.T) {
	st := NewStubTransport()
	st.Serial = Serial{0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef}
	Register(ZWO.VID, 0x0003, Model{Name: "Test3", Sensor: &testSensor})
	c, _ := Open(st, ZWO.VID, 0x0003)

	s, err := c.SerialNumber()
	if err != nil {
		t.Fatal(err)
	}
	if s != st.Serial {
		t.Errorf("SerialNumber() = % x, want % x", s, st.Serial)
	}
	if s.String() != "0123456789abcdef" {
		t.Errorf("String() = %q, want 0123456789abcdef", s.String())
	}

	last := st.Log[len(st.Log)-1]
	if !last.In || last.BRequest != 0xC8 || last.WValue != 0 || last.WIndex != 0 || last.Len != 8 {
		t.Errorf("control transfer = %+v, want IN bReq 0xC8 wVal 0 wIdx 0 len 8", last)
	}
}
