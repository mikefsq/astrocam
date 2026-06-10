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
	mc.liveMode().Bin = 2
	if got := ModeOf(rm).Bin; got != 2 {
		t.Errorf("poaRegmap live mode Bin = %d, want 2", got)
	}
}
