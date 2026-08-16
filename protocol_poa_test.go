package astrocam

import (
	"testing"
	"time"
)

// poaFake records every control transfer with full args. (The shared fakeTransport
// discards read wValue/wIndex, but the PlayerOne swapped-order assertions need them.)
type poaCtl struct {
	dir            byte // 'O' = ControlOut, 'I' = ControlIn
	bRequest       uint8
	wValue, wIndex uint16
}

type poaFake struct {
	calls  []poaCtl
	inByte byte
}

func (f *poaFake) ControlOut(b uint8, wv, wi uint16, _ []byte) error {
	f.calls = append(f.calls, poaCtl{'O', b, wv, wi})
	return nil
}
func (f *poaFake) ControlIn(b uint8, wv, wi uint16, d []byte) (int, error) {
	f.calls = append(f.calls, poaCtl{'I', b, wv, wi})
	if len(d) > 0 {
		d[0] = f.inByte
	}
	return len(d), nil
}
func (f *poaFake) BulkRead(_ []byte, _ time.Duration) (int, error) { return 0, nil }
func (f *poaFake) Close() error                                    { return nil }

func (f *poaFake) last() poaCtl { return f.calls[len(f.calls)-1] }

// TestPOARegmapWire locks PlayerOne's decoded control-transfer dialect (POAFx3.o): the
// opcodes, and crucially the reg/val order — OPPOSITE ZWO's (value in wValue, register in
// wIndex) — plus the CrypWrite obfuscation on the protected gain-setup register 0x67f.
func TestPOARegmapWire(t *testing.T) {
	f := &poaFake{}
	rm := POA.newRegmap(f, BusSony, ReadoutMode{})

	// Plain sensor write: bReq 0xB0, value in wValue, register in wIndex (swapped vs ZWO).
	if err := rm.WriteReg(0x10, 50); err != nil {
		t.Fatal(err)
	}
	if got, want := f.last(), (poaCtl{'O', poaReqImgSenWrite, 50, 0x10}); got != want {
		t.Errorf("WriteReg wire = %+v, want %+v (val in wValue, reg in wIndex)", got, want)
	}

	// Protected register 0x67f → CrypWrite: distinct opcode 0xB3 and address offset +0xABCD
	// (0x67f + 0xABCD = 0xB24C), value unchanged.
	if err := rm.WriteReg(0x67f, 0x22); err != nil {
		t.Fatal(err)
	}
	if got, want := f.last(), (poaCtl{'O', poaReqImgSenCryp, 0x22, 0xB24C}); got != want {
		t.Errorf("CrypWrite wire = %+v, want %+v (opcode 0xB3, reg+0xABCD, val unchanged)", got, want)
	}

	// Sensor read: bReq 0xB2, wValue 0, register in wIndex; returns the byte.
	f.inByte = 0xF0
	v, err := rm.ReadReg(0x300)
	if err != nil || v != 0xF0 {
		t.Fatalf("ReadReg = 0x%x, %v; want 0xF0", v, err)
	}
	if got, want := f.last(), (poaCtl{'I', poaReqImgSenRead, 0, 0x300}); got != want {
		t.Errorf("ReadReg wire = %+v, want %+v", got, want)
	}

	// FPGA write/read: bReq 0xC0 / 0xC2, same swapped order.
	if err := rm.WriteFPGAReg(0x40, 0x12); err != nil {
		t.Fatal(err)
	}
	if got, want := f.last(), (poaCtl{'O', poaReqFpgaWrite, 0x12, 0x40}); got != want {
		t.Errorf("WriteFPGAReg wire = %+v, want %+v", got, want)
	}
	if _, err := rm.ReadFPGAReg(0x05); err != nil {
		t.Fatal(err)
	}
	if got, want := f.last(), (poaCtl{'I', poaReqFpgaRead, 0, 0x05}); got != want {
		t.Errorf("ReadFPGAReg wire = %+v, want %+v", got, want)
	}
}

// TestPOAVendorRegistered checks the PlayerOne vendor is discoverable by the
// vendor-independent core: VID 0xA0A0 resolves to POA, and KnownVIDs carries both vendors.
func TestPOAVendorRegistered(t *testing.T) {
	v, ok := VendorOf(0xA0A0)
	if !ok || v != POA || v.Name != "PlayerOne" {
		t.Fatalf("VendorOf(0xA0A0) = %v, %v; want the PlayerOne descriptor", v, ok)
	}
	var sawZWO, sawPOA bool
	for _, vid := range KnownVIDs() {
		switch vid {
		case ZWO.VID:
			sawZWO = true
		case POA.VID:
			sawPOA = true
		}
	}
	if !sawZWO || !sawPOA {
		t.Errorf("KnownVIDs missing a vendor: ZWO=%v POA=%v", sawZWO, sawPOA)
	}

	// modeCarrier works through the poaRegmap too (the de-coupling that replaced the
	// *zwoRegmap assertions): mutating the live mode must stick.
	rm := POA.newRegmap(&poaFake{}, BusSony, ReadoutMode{})
	mc, ok := rm.(modeCarrier)
	if !ok {
		t.Fatal("poaRegmap does not implement modeCarrier")
	}
	mc.updateMode(func(m *ReadoutMode) { m.Bin = 2 })
	if got := ModeOf(rm).Bin; got != 2 {
		t.Errorf("poaRegmap live mode Bin = %d, want 2", got)
	}
}

// TestPOACmdsNotDecoded: with an empty FX3 command table the Camera must refuse to drive a
// PlayerOne body with ZWO's opcodes. Init errors before any vendor command, FirmwareVersion /
// SerialNumber / ReadSPIFlash error, and the transport log carries none of ZWO's 0xAA/0xA9/
// 0xAF/0xBE/0xC3/0xAD/0xC8 requests. The ZWO table itself carries the known opcodes.
func TestPOACmdsNotDecoded(t *testing.T) {
	if z := ZWO.Cmds; z.StreamStop != 0xAA || z.StreamStart != 0xA9 || z.Flush != 0xAF ||
		z.EnableGPIF32DQ != 0xBE || z.ReadSPIFlash != 0xC3 || z.FirmwareVersion != 0xAD || z.SerialNumber != 0xC8 ||
		z.ST4On != 0xB0 || z.ST4Off != 0xB1 || z.ReadTemp != 0xB3 || z.ReadHumidity != 0x85 || z.ReadHumidityWValue != 0xF5 {
		t.Fatalf("ZWO.Cmds = %+v, want the decoded ZWO opcodes", z)
	}
	if POA.Cmds != (FX3Cmds{}) {
		t.Fatalf("POA.Cmds = %+v, want zero (not decoded)", POA.Cmds)
	}
	s := Sensor{
		Name: "POAINIT",
		Info: CameraInfo{MaxWidth: 4, MaxHeight: 2, BitDepth: 12},
		Init: []RegVal{{Reg: 0x3000, Val: 1}},
	}
	Register(POA.VID, 0x0CE2, Model{Name: "poa-init", Sensor: &s})
	st := NewStubTransport()
	c, err := Open(st, POA.VID, 0x0CE2)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.Init(); err == nil {
		t.Error("Init on a vendor with no FX3 commands decoded succeeded, want an error")
	}
	if _, err := c.FirmwareVersion(); err == nil {
		t.Error("FirmwareVersion succeeded, want a not-decoded error")
	}
	if _, err := c.SerialNumber(); err == nil {
		t.Error("SerialNumber succeeded, want a not-decoded error")
	}
	if _, err := c.ReadSPIFlash(FlashHPCMapAddr, 16); err == nil {
		t.Error("ReadSPIFlash succeeded, want a not-decoded error")
	}
	if err := c.VendorCmd(FX3StreamStop); err == nil {
		t.Error("VendorCmd(FX3StreamStop) succeeded, want a not-decoded error")
	}
	// ST4 and thermal: 0xB0/0xB3 are PlayerOne's sensor-register write/CrypWrite opcodes, so a
	// ZWO-literal pulse or temperature read would land as a sensor register write on a POA body.
	if err := c.PulseGuideOn(GuideNorth); err == nil {
		t.Error("PulseGuideOn succeeded, want a not-decoded error")
	}
	th := c.HardwareThermal()
	if _, err := th.ReadTemp(); err == nil {
		t.Error("ReadTemp succeeded, want a not-decoded error")
	}
	if _, err := th.ReadHumidity(); err == nil {
		t.Error("ReadHumidity succeeded, want a not-decoded error")
	}
	for _, x := range st.Log {
		switch x.BRequest {
		case 0xAA, 0xA9, 0xAF, 0xBE, 0xC3, 0xAD, 0xC8, 0xB0, 0xB1, 0xB3, 0x85:
			t.Errorf("ZWO opcode 0x%02x sent to a PlayerOne camera: %+v", x.BRequest, x)
		}
	}
}
