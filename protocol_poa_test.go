package astrocam

import (
	"bytes"
	"testing"
	"time"
)

// poaFake records every control transfer with full args (the shared fakeTransport discards read
// wValue/wIndex, which the PlayerOne swapped-order assertions need).
type poaCtl struct {
	dir            byte // 'O' = ControlOut, 'I' = ControlIn
	bRequest       uint8
	wValue, wIndex uint16
}

type poaFake struct {
	calls  []poaCtl
	inByte byte
	// outData holds each ControlOut's data stage, indexed alongside calls, so the burst-write
	// assertions can check a payload. poaCtl itself stays comparable for the == checks below.
	outData [][]byte
}

func (f *poaFake) ControlOut(b uint8, wv, wi uint16, d []byte) error {
	f.calls = append(f.calls, poaCtl{'O', b, wv, wi})
	f.outData = append(f.outData, append([]byte(nil), d...))
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

// TestPOARegmapWire: PlayerOne's control-transfer dialect: the opcodes, the reg/val
// order (value in wValue, register in wIndex, the reverse of ZWO's), and the CrypWrite
// obfuscation on the protected gain-setup register 0x67f.
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

// TestPOAVendorRegistered: VID 0xA0A0 resolves to POA, KnownVIDs carries both vendors, and
// poaRegmap implements modeCarrier.
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

	// Mutating the live mode through modeCarrier sticks on a poaRegmap.
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

// TestPOAFX3Commands: the PlayerOne FX3 command table drives a PlayerOne body with PlayerOne's
// opcodes and argument placement, never with ZWO's. Streaming goes out on 0xA0 with wValue
// selecting start from stop (ZWO uses two codes, 0xA9/0xAA), an ST4 pulse puts the line state in
// wValue and the direction in wIndex on 0xA6 (ZWO puts the direction in wValue on 0xB0/0xB1),
// the serial is 20 ASCII bytes off 0xA3 and the firmware a single byte off 0xA2. The operations
// PlayerOne has no counterpart for still error rather than borrowing ZWO's bytes.
func TestPOAFX3Commands(t *testing.T) {
	if z := ZWO.Cmds; z.StreamStop.Req != 0xAA || z.StreamStart.Req != 0xA9 || z.Flush.Req != 0xAF ||
		z.EnableGPIF32DQ != 0xBE || z.ReadSPIFlash != 0xC3 || z.FirmwareVersion != 0xAD || z.SerialNumber != 0xC8 ||
		z.ST4 != (FX3ST4{On: 0xB0, Off: 0xB1}) ||
		z.ReadTemp != 0xB3 || z.ReadHumidity != 0x85 || z.ReadHumidityWValue != 0xF5 {
		t.Fatalf("ZWO.Cmds = %+v, want the decoded ZWO opcodes", z)
	}
	p := POA.Cmds
	if p.StreamStart != (FX3Cmd{Req: 0xA0, WValue: 1}) || p.StreamStop != (FX3Cmd{Req: 0xA0, WValue: 0}) ||
		p.FirmwareVersion != 0xA2 || p.FirmwareBytes != 1 ||
		p.SerialNumber != 0xA3 || p.SerialBytes != 20 || !p.SerialASCII ||
		p.ST4 != (FX3ST4{On: 0xA6, Off: 0xA6, DirInWIndex: true}) ||
		p.ReadTemp != 0xA8 || p.ReadTempBytes != 8 || p.ReadSPIFlash != 0xD1 {
		t.Fatalf("POA.Cmds = %+v, want the decoded PlayerOne opcodes", p)
	}
	// These stay zero because PlayerOne HAS no counterpart, not because one is undecoded: no
	// pipeline flush, no data-bus gate, no humidity sensor. EnableGPIF32DQ in particular is
	// absent BECAUSE the flash read needs no gate — PlayerOne's flash does not share the FX3
	// pins with the data bus, where ZWO's does.
	if p.Flush.decoded() || p.EnableGPIF32DQ != 0 || p.ReadHumidity != 0 {
		t.Fatalf("POA.Cmds carries an entry PlayerOne does not have: %+v", p)
	}

	s := Sensor{
		Name: "POAINIT",
		Info: CameraInfo{MaxWidth: 4, MaxHeight: 2, BitDepth: 12},
		Init: []RegVal{{Reg: 0x3000, Val: 1}},
	}
	Register(POA.VID, 0x0CE2, Model{Name: "poa-init", Sensor: &s})
	st := NewStubTransport()
	st.Serial = []byte("CAMGF252416072209000") // as poasnap reads it off a Xena 585M
	c, err := Open(st, POA.VID, 0x0CE2)
	if err != nil {
		t.Fatal(err)
	}
	// Init must now succeed: the missing flush is skipped, not treated as a fault.
	if err := c.Init(); err != nil {
		t.Fatalf("Init on the decoded PlayerOne table: %v", err)
	}
	if sn, err := c.SerialNumber(); err != nil || sn.String() != "CAMGF252416072209000" {
		t.Errorf("SerialNumber() = %q, %v; want the 20 ASCII characters verbatim", sn, err)
	}
	if err := c.PulseGuideOn(GuideSouth); err != nil {
		t.Errorf("PulseGuideOn: %v", err)
	}
	if err := c.PulseGuideOff(GuideSouth); err != nil {
		t.Errorf("PulseGuideOff: %v", err)
	}
	th := c.HardwareThermal()
	if _, err := th.ReadTemp(); err != nil {
		t.Errorf("ReadTemp: %v", err)
	}

	// The flash read IS available on PlayerOne, and must go out WITHOUT the GPIF toggle that
	// brackets ZWO's: sending that opcode to a PlayerOne body would be a cross-vendor write.
	mark := len(st.Log) // later assertions read the whole log, so scope this to what follows
	if _, err := c.ReadSPIFlash(poaFlashHPCAddr, 16); err != nil {
		t.Errorf("ReadSPIFlash: %v", err)
	}
	sawRead := false
	for _, x := range st.Log[mark:] {
		if !x.In && x.BRequest == cmdEnableGPIF32DQ {
			t.Error("ReadSPIFlash emitted ZWO's EnableGPIF32DQ on a PlayerOne body")
		}
		if x.In && x.BRequest == poaReqFlashRead {
			sawRead = true
			if x.WIndex != poaFlashHPCAddr>>8 {
				t.Errorf("flash read wIndex = %#x, want the page %#x", x.WIndex, poaFlashHPCAddr>>8)
			}
		}
	}
	if !sawRead {
		t.Error("no 0xD1 flash page read went out")
	}
	if err := c.VendorCmd(FX3Flush); err == nil {
		t.Error("VendorCmd(FX3Flush) succeeded, want a not-decoded error")
	}
	if _, err := th.ReadHumidity(); err == nil {
		t.Error("ReadHumidity succeeded, want a not-decoded error")
	}

	var start, stop, st4On, st4Off, fwLen1, snLen20, temp bool
	for _, x := range st.Log {
		switch x.BRequest {
		// ZWO-only codes. 0xB0/0xB3 are absent from this list on purpose: they are PlayerOne's
		// own sensor write and CrypWrite, so the reglist legitimately uses them. 0xB1 is
		// PlayerOne's burst sensor write, which this driver never issues, so seeing it would
		// mean ZWO's ST4-off leaked through.
		case 0xAA, 0xA9, 0xAF, 0xBE, 0xC3, 0xAD, 0xC8, 0xB1, 0x85:
			t.Errorf("ZWO opcode 0x%02x sent to a PlayerOne camera: %+v", x.BRequest, x)
		case 0xA0:
			start, stop = start || x.WValue == 1, stop || x.WValue == 0
		case 0xA2:
			fwLen1 = fwLen1 || (x.In && x.Len == 1)
		case 0xA3:
			snLen20 = snLen20 || (x.In && x.Len == 20)
		case 0xA6:
			// wValue = line state, wIndex = direction — the reverse of ZWO's placement.
			if x.WIndex == uint16(GuideSouth) {
				st4On, st4Off = st4On || x.WValue == 1, st4Off || x.WValue == 0
			}
		case 0xA8:
			temp = temp || (x.In && x.Len == 8)
		}
	}
	if !stop {
		t.Error("no 0xA0 wValue 0 (stream stop) issued during Init")
	}
	if !fwLen1 {
		t.Error("no 1-byte 0xA2 firmware read issued during Init")
	}
	if !snLen20 {
		t.Error("no 20-byte 0xA3 serial read issued")
	}
	if !st4On || !st4Off {
		t.Errorf("ST4 pulse did not go out as 0xA6 wValue 1/0 with wIndex = direction (on %v, off %v)", st4On, st4Off)
	}
	if !temp {
		t.Error("no 8-byte 0xA8 temperature read issued")
	}
	_ = start // Init only stops the stream; a start goes out when a capture arms.
}

// TestPOAFrameMarkerRepair: PlayerOne writes a fixed twelve-byte header over the start of every
// frame — the magic 77 ee 00 0c then eight zeros. It is a BYTE count, not a pixel count: the same
// twelve bytes appear at RAW8 and RAW16, so they eat twelve pixels of an 8-bit frame and six of a
// 16-bit one. Repairing it took the RAW8 comparison against the SDK to zero differing pixels.
func TestPOAFrameMarkerRepair(t *testing.T) {
	const w, rows = 32, 1
	mk := func(bpp int) []byte {
		buf := make([]byte, w*8*bpp)
		for i := range buf {
			buf[i] = 0x40 // plausible pixel data everywhere
		}
		copy(buf, []byte{0x77, 0xee, 0x00, 0x0c, 0, 0, 0, 0, 0, 0, 0, 0})
		return buf
	}
	for _, bpp := range []int{1, 2} {
		buf := mk(bpp)
		repairPOAFrameMarker(buf, bpp, w, rows)
		for i := 0; i < poaFrameMarkerLen; i++ {
			if buf[i] != 0x40 {
				t.Errorf("bpp %d: byte %d = 0x%02x after repair, want the copied pixel 0x40", bpp, i, buf[i])
			}
		}
	}
	// A die-binned frame carries TWO markers — sequence 0 at the start and sequence 1 partway in,
	// which fits the readout making two passes over the window — so the repair scans rather than
	// only fixing offset zero.
	two := mk(2)
	second := 40 * 2
	copy(two[second:], []byte{0x77, 0xee, 0x01, 0x0c, 0, 0, 0, 0, 0, 0, 0, 0})
	repairPOAFrameMarker(two, 2, w, rows)
	for _, at := range []int{0, second} {
		for i := at; i < at+poaFrameMarkerLen; i++ {
			if two[i] != 0x40 {
				t.Errorf("two-marker frame: byte %d = 0x%02x after repair, want 0x40", i, two[i])
			}
		}
	}

	// A frame without the magic is left alone: the repair must not corrupt real pixels that
	// happen to sit at the start.
	buf := make([]byte, w*8)
	for i := range buf {
		buf[i] = byte(i)
	}
	orig := append([]byte(nil), buf...)
	repairPOAFrameMarker(buf, 1, w, rows)
	if !bytes.Equal(buf, orig) {
		t.Error("repair modified a frame that carries no marker")
	}
	// The magic alone is not enough: the eight trailing zeros are what make scanning safe, so a
	// run of pixels that merely starts 77 ee .. 0c must be left alone.
	near := mk(1)
	copy(near[24:], []byte{0x77, 0xee, 0x02, 0x0c, 1, 2, 3, 4, 5, 6, 7, 8})
	before := append([]byte(nil), near[24:36]...)
	repairPOAFrameMarker(near, 1, w, rows)
	if !bytes.Equal(near[24:36], before) {
		t.Error("repair fired on the magic alone, without the eight zero bytes")
	}

	// So is a frame too small to reach a clean row.
	small := []byte{0x77, 0xee, 0x00, 0x0c, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2}
	origSmall := append([]byte(nil), small...)
	repairPOAFrameMarker(small, 1, w, rows)
	if !bytes.Equal(small, origSmall) {
		t.Error("repair reached past the end of a short frame")
	}
}
