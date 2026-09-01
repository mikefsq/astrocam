package astrocam

import (
	"bytes"
	"testing"
)

// poaRM builds a PlayerOne regmap over a recording fake, the dialect the FPGA helpers assume.
func poaRM(f *poaFake) Regmap { return POA.newRegmap(f, BusSony, ReadoutMode{}) }

// TestFPGARunPolarityDiffersByVendor: the same FPGA register 0 value means opposite things on the
// two vendors — 0x10 is ZWO's STOP bit and PlayerOne's START value — so the run control has to
// come from the vendor descriptor. This is what left a Xena 585M armed but silent: the shared
// capture path was clearing bit 4 to "start", which on PlayerOne is a stop.
func TestFPGARunPolarityDiffersByVendor(t *testing.T) {
	f := &poaFake{}
	rm := poaRM(f)
	if err := POA.fpgaRun(rm, true); err != nil {
		t.Fatal(err)
	}
	if got, want := f.last(), (poaCtl{'O', poaReqFpgaWrite, poaRunStart, poaFPGARun}); got != want {
		t.Errorf("PlayerOne start = %+v, want a write of 0x10 to FPGA reg 0", got)
	}
	if err := POA.fpgaRun(rm, false); err != nil {
		t.Fatal(err)
	}
	if got, want := f.last(), (poaCtl{'O', poaReqFpgaWrite, poaRunStop, poaFPGARun}); got != want {
		t.Errorf("PlayerOne stop = %+v, want a write of 0x00 to FPGA reg 0", got)
	}

	// ZWO, same register, inverted meaning: bit 4 SET is the stop. Its dialect is also the
	// mirror of PlayerOne's on the wire — register in wValue, value in wIndex.
	z := &poaFake{}
	zrm := ZWO.newRegmap(z, BusSony, ReadoutMode{})
	if err := ZWO.fpgaRun(zrm, false); err != nil {
		t.Fatal(err)
	}
	if got := z.last(); got.wValue != fpgaModeReg0 || got.wIndex != fpgaStopBit {
		t.Errorf("ZWO stop = %+v, want reg 0 in wValue and the 0x%02x stop bit in wIndex", got, fpgaStopBit)
	}
	if err := ZWO.fpgaRun(zrm, true); err != nil {
		t.Fatal(err)
	}
	if got := z.last(); got.wValue != fpgaModeReg0 || got.wIndex != 0 {
		t.Errorf("ZWO start = %+v, want the stop bit cleared", got)
	}
}

// TestPOAFPGABurstLatched: a burst is bracketed by the group latch on register 1 and carries its
// payload in the data stage of one 0xC1 transfer addressed by wIndex.
func TestPOAFPGABurstLatched(t *testing.T) {
	f := &poaFake{}
	if err := POAFPGABurst(poaRM(f), 0x0c, []byte{1, 2, 3}); err != nil {
		t.Fatal(err)
	}
	if len(f.calls) != 3 {
		t.Fatalf("got %d transfers, want latch-on, burst, latch-off: %+v", len(f.calls), f.calls)
	}
	if got, want := f.calls[0], (poaCtl{'O', poaReqFpgaWrite, 1, poaFPGALatch}); got != want {
		t.Errorf("latch on = %+v, want %+v", got, want)
	}
	if got, want := f.calls[1], (poaCtl{'O', poaReqFpgaBurst, 0, 0x0c}); got != want {
		t.Errorf("burst = %+v, want 0xC1 with wIndex = the first register", got)
	}
	if !bytes.Equal(f.outData[1], []byte{1, 2, 3}) {
		t.Errorf("burst payload = % x, want 01 02 03", f.outData[1])
	}
	if got, want := f.calls[2], (poaCtl{'O', poaReqFpgaWrite, 0, poaFPGALatch}); got != want {
		t.Errorf("latch off = %+v, want %+v", got, want)
	}

	// A dialect with no burst request must refuse rather than silently drop the payload.
	if err := POAFPGABurst(ZWO.newRegmap(&poaFake{}, BusSony, ReadoutMode{}), 0x0c, []byte{1}); err == nil {
		t.Error("burst on a non-burst regmap succeeded, want an error")
	}
}

// TestPOAFPGAImageSize: the format, mode and 0x38 bytes go out as singles, then the geometry as
// one 8-byte burst of [w u16][h u16][dmaWords u32].
//
// Every literal here is what the SDK put on the wire driving a real Xena 585M, read off a
// Captured off a Xena 585M at 640x480 RAW16:
// format 0x81, mode 0x00, reg 0x38 0x00, payload 100f 8408 20224000.
func TestPOAFPGAImageSize(t *testing.T) {
	f := &poaFake{}
	// A Xena 585M full frame: 3856x2180 RAW16, hardware bin factor 1, no software bin.
	if err := POAFPGAImageSize(poaRM(f), 3856, 2180, true, 1, false, 0, false, false); err != nil {
		t.Fatal(err)
	}
	if got, want := f.calls[0], (poaCtl{'O', poaReqFpgaWrite, 0x81, poaFPGAFormat}); got != want {
		t.Errorf("format byte = %+v, want 0x81 (16-bit samples | hardware bin 1)", got)
	}
	if got, want := f.calls[1], (poaCtl{'O', poaReqFpgaWrite, 0x00, poaFPGAMode}); got != want {
		t.Errorf("mode byte = %+v, want 0", got)
	}
	if got, want := f.calls[2], (poaCtl{'O', poaReqFpgaWrite, 0x00, poaFPGAFlag38}); got != want {
		t.Errorf("reg 0x38 = %+v, want 0", got)
	}
	// calls[3] is the latch; calls[4] the burst.
	if got, want := f.calls[4], (poaCtl{'O', poaReqFpgaBurst, 0, poaFPGASize}); got != want {
		t.Errorf("size burst = %+v, want 0xC1 to reg 0x0c", got)
	}
	want := []byte{0x10, 0x0f, 0x84, 0x08, 0x20, 0x22, 0x40, 0x00} // 3856, 2180, 4203040
	if !bytes.Equal(f.outData[4], want) {
		t.Errorf("size payload = % x, want % x", f.outData[4], want)
	}
}

// TestPOAFPGAImageSizeMatchesCapture replays every image-size write the SDK issued across a capture
// matrix over ROI, sample size and binning, all captured on the same Xena 585M. The register 0x02 and 0x04 bytes and the whole
// 8-byte payload are the recorded wire bytes.
//
// The two bin 2 rows are the interesting pair: software binning programs the FULL sensor geometry
// with bin 1, hardware binning programs the BINNED geometry with bin 0, and both land on the same
// frame length. A sub-frame is programmed as the image size itself — register 0x08 carries a fixed
// origin offset, not the ROI position.
func TestPOAFPGAImageSizeMatchesCapture(t *testing.T) {
	for _, c := range []struct {
		name        string
		w, h        int
		bpp16       bool
		format      uint8
		bin         uint8
		wide        bool
		wantFormatB uint16
		wantModeB   uint16
		wantBurst   []byte
	}{
		{"RAW16 3856x2180 full", 3856, 2180, true, 1, 0, false, 0x81, 0x00,
			[]byte{0x10, 0x0f, 0x84, 0x08, 0x20, 0x22, 0x40, 0x00}}, // dma 4203040
		{"RAW8 3856x2180 full", 3856, 2180, false, 0, 0, false, 0x00, 0x00,
			[]byte{0x10, 0x0f, 0x84, 0x08, 0x10, 0x11, 0x20, 0x00}}, // dma 2101520
		{"RAW16 640x480", 640, 480, true, 1, 0, false, 0x81, 0x00,
			[]byte{0x80, 0x02, 0xe0, 0x01, 0x00, 0x58, 0x02, 0x00}}, // dma 153600
		{"RAW8 640x480", 640, 480, false, 0, 0, false, 0x00, 0x00,
			[]byte{0x80, 0x02, 0xe0, 0x01, 0x00, 0x2c, 0x01, 0x00}}, // dma 76800
		{"RAW8 320x240", 320, 240, false, 0, 0, false, 0x00, 0x00,
			[]byte{0x40, 0x01, 0xf0, 0x00, 0x00, 0x4b, 0x00, 0x00}}, // dma 19200
		{"RAW16 1920x1080", 1920, 1080, true, 1, 0, false, 0x81, 0x00,
			[]byte{0x80, 0x07, 0x38, 0x04, 0x00, 0xd2, 0x0f, 0x00}}, // dma 1036800
		{"RAW16 bin2 software", 3856, 2180, true, 1, 1, false, 0x81, 0x01,
			[]byte{0x10, 0x0f, 0x84, 0x08, 0x88, 0x08, 0x10, 0x00}}, // dma 1050760
		{"RAW16 bin2 hardware", 1928, 1090, true, 1, 0, false, 0x81, 0x00,
			[]byte{0x88, 0x07, 0x42, 0x04, 0x88, 0x08, 0x10, 0x00}}, // dma 1050760
		// HDR mode reads two exposures per frame, so it programs DOUBLE the height and sets the
		// wide flag at register 0x38. The extra shift the flag selects cancels the doubling, and
		// the frame length comes out identical to the same window in Normal mode.
		{"RAW16 HDR 640x480", 640, 960, true, 1, 0, true, 0x81, 0x00,
			[]byte{0x80, 0x02, 0xc0, 0x03, 0x00, 0x58, 0x02, 0x00}}, // dma 153600, as Normal
		{"RAW16 HDR full frame", 3856, 4360, true, 1, 0, true, 0x81, 0x00,
			[]byte{0x10, 0x0f, 0x08, 0x11, 0x20, 0x22, 0x40, 0x00}}, // dma 4203040, as Normal
	} {
		f := &poaFake{}
		if err := POAFPGAImageSize(poaRM(f), c.w, c.h, c.bpp16, c.format, false, c.bin, c.wide, false); err != nil {
			t.Fatalf("%s: %v", c.name, err)
		}
		if got := f.calls[0].wValue; got != c.wantFormatB {
			t.Errorf("%s: reg 0x02 = 0x%02x, want 0x%02x", c.name, got, c.wantFormatB)
		}
		if got := f.calls[1].wValue; got != c.wantModeB {
			t.Errorf("%s: reg 0x04 = 0x%02x, want 0x%02x", c.name, got, c.wantModeB)
		}
		wantWide := uint16(0)
		if c.wide {
			wantWide = 1
		}
		if got := f.calls[2].wValue; got != wantWide {
			t.Errorf("%s: reg 0x38 = 0x%02x, want 0x%02x", c.name, got, wantWide)
		}
		if !bytes.Equal(f.outData[4], c.wantBurst) {
			t.Errorf("%s: size payload = % x, want % x", c.name, f.outData[4], c.wantBurst)
		}
	}
}

// TestPOAFPGADMAWords pins the image-size frame-length arithmetic.
func TestPOAFPGADMAWords(t *testing.T) {
	for _, c := range []struct {
		name  string
		w, h  int
		bpp16 bool
		bin   uint8
		wide  bool
		want  uint32
	}{
		// 3856·2180·2 bytes = 16812160, in 32-bit words.
		{"585 full frame RAW16", 3856, 2180, true, 0, false, 4203040},
		// Same frame in 64-bit words.
		{"wide transfer halves it", 3856, 2180, true, 0, true, 2101520},
		// RAW8 drops the sample scale.
		{"RAW8", 3856, 2180, false, 0, false, 2101520},
		// bin encoding is factor-1, and the divide is applied once per axis.
		{"bin 2", 3856, 2180, true, 1, false, 1050760},
		{"bin 4", 3856, 2180, true, 3, false, 262690},
	} {
		if got := POAFPGADMAWords(c.w, c.h, c.bpp16, c.bin, c.wide); got != c.want {
			t.Errorf("%s: POAFPGADMAWords = %d, want %d", c.name, got, c.want)
		}
	}
}

// TestPOAFPGADriveAndCrop: the drive pair is a 16-bit then a 24-bit field, little-endian, and an
// over-wide frame period is rejected rather than truncated onto the wire.
func TestPOAFPGADriveAndCrop(t *testing.T) {
	f := &poaFake{}
	if err := POAFPGADrive(poaRM(f), 0x00c0, 0x0008c4); err != nil {
		t.Fatal(err)
	}
	want := []byte{0xc0, 0x00, 0xc4, 0x08, 0x00}
	if !bytes.Equal(f.outData[1], want) {
		t.Errorf("drive payload = % x, want % x", f.outData[1], want)
	}
	if err := POAFPGADrive(poaRM(&poaFake{}), 1, 0x1000000); err == nil {
		t.Error("a frame period wider than 24 bits was accepted, want an error")
	}

	c := &poaFake{}
	if err := POAFPGACropOrigin(poaRM(c), 0, 42); err != nil {
		t.Fatal(err)
	}
	if want := []byte{0, 0, 42, 0}; !bytes.Equal(c.outData[1], want) {
		t.Errorf("crop payload = % x, want % x", c.outData[1], want)
	}
}

// TestPOAFPGAExposure: the register holds the exposure divided by 6.4, truncated. Both cases are
// off the wire — the SDK sent 350c0000 for the 20 ms this test asked for, and 1a060000 for its
// own 10 ms default.
func TestPOAFPGAExposure(t *testing.T) {
	for _, c := range []struct {
		us   uint64
		want []byte
	}{
		{1000, []byte{0x9c, 0x00, 0x00, 0x00}},    // 156 ticks, 1000/6.4 = 156.25 truncated
		{5000, []byte{0x0d, 0x03, 0x00, 0x00}},    // 781 ticks, 5000/6.4 = 781.25 truncated
		{10000, []byte{0x1a, 0x06, 0x00, 0x00}},   // 1562 ticks, the SDK default
		{20000, []byte{0x35, 0x0c, 0x00, 0x00}},   // 3125 ticks
		{100000, []byte{0x09, 0x3d, 0x00, 0x00}},  // 15625 ticks, exact
		{1000000, []byte{0x5a, 0x62, 0x02, 0x00}}, // 156250 ticks, exact
	} {
		f := &poaFake{}
		if err := POAFPGAExposure(poaRM(f), c.us); err != nil {
			t.Fatal(err)
		}
		if !bytes.Equal(f.outData[1], c.want) {
			t.Errorf("%d us: exposure payload = % x, want % x", c.us, f.outData[1], c.want)
		}
	}
}

// TestPOAFPGADriveMatchesCapture: the drive pair as the SDK sent it for the full frame in RAW16.
// The first field doubles between RAW8 (145) and RAW16 (290), so it is a per-mode line period,
// not the fixed HMAX the ZWO-derived IMX585 profile assumes.
func TestPOAFPGADriveMatchesCapture(t *testing.T) {
	f := &poaFake{}
	if err := POAFPGADrive(poaRM(f), 290, 29996); err != nil {
		t.Fatal(err)
	}
	if want := []byte{0x22, 0x01, 0x2c, 0x75, 0x00}; !bytes.Equal(f.outData[1], want) {
		t.Errorf("drive payload = % x, want % x", f.outData[1], want)
	}
}

// TestPOAFPGAWhiteBalance: unity is 0x4000 per channel, which is what the SDK left on the wire
// after init (register 0x1a burst `0040 0040 0040`), and the curve is 10^(v/2000) in Q14.
func TestPOAFPGAWhiteBalance(t *testing.T) {
	if got := POAFPGAWhiteBalanceGain(0); got != 0x4000 {
		t.Errorf("unity gain = 0x%04x, want 0x4000", got)
	}
	// The config range end points, checked against the curve rather than a captured value:
	// 10^(1200/2000) = 3.98107 -> 65225.47, 10^(-1200/2000) = 0.251189 -> 4115.47, both truncated.
	if got := POAFPGAWhiteBalanceGain(1200); got != 65225 {
		t.Errorf("gain(+1200) = %d, want 65225", got)
	}
	if got := POAFPGAWhiteBalanceGain(-1200); got != 4115 {
		t.Errorf("gain(-1200) = %d, want 4115", got)
	}

	f := &poaFake{}
	if err := POAFPGAWhiteBalance(poaRM(f), 0, 0, 0); err != nil {
		t.Fatal(err)
	}
	if got, want := f.calls[1], (poaCtl{'O', poaReqFpgaBurst, 0, poaFPGAWB}); got != want {
		t.Errorf("WB burst = %+v, want 0xC1 to reg 0x1a", got)
	}
	want := []byte{0x00, 0x40, 0x00, 0x40, 0x00, 0x40}
	if !bytes.Equal(f.outData[1], want) {
		t.Errorf("WB payload = % x, want % x", f.outData[1], want)
	}

	m := &poaFake{}
	if err := POAFPGAWhiteBalanceMode(poaRM(m), true, false, true); err != nil {
		t.Fatal(err)
	}
	if got, want := m.last(), (poaCtl{'O', poaReqFpgaWrite, 0x11, poaFPGAWBMode}); got != want {
		t.Errorf("WB mode = %+v, want bits 0 and 4 set on reg 0x19", got)
	}
}

// TestPOACoolingCurves: the TEC and heater levels follow a SQUARE ROOT of the per-mille demand
// not a linear percentage, and a non-zero demand never rounds down
// to a dead zero. The fan curve is linear.
func TestPOACoolingCurves(t *testing.T) {
	for _, c := range []struct {
		perMille int
		want     uint16
	}{
		{0, 0},
		{1, 8},     // sqrt(0.001)*255 = 8.06
		{10, 25},   // sqrt(0.01)*255  = 25.5
		{250, 127}, // sqrt(0.25)*255  = 127.5
		{1000, 255},
		{2000, 255}, // clamped
	} {
		if got := poaDriveLevel(c.perMille); got != c.want {
			t.Errorf("poaDriveLevel(%d) = %d, want %d", c.perMille, got, c.want)
		}
	}
	for _, c := range []struct {
		percent int
		want    uint16
	}{{0, 0}, {50, 127}, {100, 255}, {150, 255}} {
		if got := poaFanLevel(c.percent); got != c.want {
			t.Errorf("poaFanLevel(%d) = %d, want %d", c.percent, got, c.want)
		}
	}
}

// TestPOAThermalUsesPlayerOneRegisters: the cooling backend must never touch ZWO's registers,
// because 0x26 is PlayerOne's window heater and 0x2a its read-only status. This is the check that
// a shared backend would fail.
func TestPOAThermalUsesPlayerOneRegisters(t *testing.T) {
	f := &poaFake{}
	th := &poaThermal{t: f, rm: poaRM(f), cmds: POA.Cmds}

	if err := th.SetTECPower(100); err != nil {
		t.Fatal(err)
	}
	var sawCool, sawFan bool
	for _, x := range f.calls {
		if x.bRequest != poaReqFpgaWrite {
			continue
		}
		switch x.wIndex {
		case poaFPGACool:
			sawCool = x.wValue == 255
		case poaFPGAFan:
			sawFan = x.wValue == 255
		case 0x2a:
			t.Errorf("cooling wrote 0x2a, PlayerOne's read-only status register: %+v", x)
		}
	}
	if !sawCool || !sawFan {
		t.Errorf("SetTECPower(100) did not drive reg 0x25 and the fan at full: %+v", f.calls)
	}

	h := &poaFake{}
	hth := &poaThermal{t: h, rm: poaRM(h), cmds: POA.Cmds}
	if err := hth.SetHeater(100); err != nil {
		t.Fatal(err)
	}
	if got, want := h.last(), (poaCtl{'O', poaReqFpgaWrite, 255, poaFPGAWarm}); got != want {
		t.Errorf("SetHeater = %+v, want reg 0x26 (PlayerOne's heater)", got)
	}

	// Humidity is not decoded for PlayerOne and must refuse rather than send ZWO's 0x85.
	if _, err := hth.ReadHumidity(); err == nil {
		t.Error("ReadHumidity succeeded, want a not-decoded error")
	}
}
