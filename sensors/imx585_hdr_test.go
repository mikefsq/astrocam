package sensors

import (
	"bytes"
	"testing"
	"time"

	. "github.com/mikefsq/astrocam"
)

// The HDR programming, register for register, as the SDK put it on the wire driving a Xena 585M
// off the wire. That session programmed Normal
// RAW8 twice and then HDR RAW16 twice, which is why the Normal values below are the RAW8 ones and
// the HDR values the RAW16 ones — the capture is the only place three of the four (mode, sample
// size) cells appear together, and it is what makes the block demonstrably two-dimensional.

// hdrPOA builds a PlayerOne regmap carrying a sensor mode and sample size.
func hdrPOA(mode, bpp int) *modeRegmap {
	rm := &modeRegmap{mode: ReadoutMode{BytesPerPx: bpp, SensorMode: mode}, regVals: map[uint16]uint16{}}
	rm.vid = POA.VID
	return rm
}

// TestIMX585HDRSetROIMatchesCapture: the FPGA side of HDR, against both windows in the capture.
// The doubled height with the wide flag is the whole trick — it leaves the DMA word count, and so
// the frame the host receives, identical to Normal's.
func TestIMX585HDRSetROIMatchesCapture(t *testing.T) {
	for _, c := range []struct {
		name     string
		w, h     int
		wantSize []byte // FPGA 0x0c: [w u16][h u16][dmaWords u32]
	}{
		// h on the wire is 4360 = 2180x2, and 0x00402220 = 4203040 words, the same count Normal
		// programs for this window in RAW16.
		{"full frame", 3856, 2180, []byte{0x10, 0x0f, 0x08, 0x11, 0x20, 0x22, 0x40, 0x00}},
		// 960 = 480x2, and 0x00025800 = 153600, again Normal's count.
		{"640x480", 640, 480, []byte{0x80, 0x02, 0xc0, 0x03, 0x00, 0x58, 0x02, 0x00}},
	} {
		rm := hdrPOA(imx585ModeHDRPOA, 2)
		if err := IMX585.SetROI(rm, 0, 0, c.w, c.h, 1); err != nil {
			t.Fatalf("%s: %v", c.name, err)
		}
		if got := rm.fpgaBursts[0x0c]; !bytes.Equal(got, c.wantSize) {
			t.Errorf("%s: FPGA size burst = % x, want % x", c.name, got, c.wantSize)
		}
		// Crop origin y = 42 in HDR where Normal programs 21.
		if got, want := rm.fpgaBursts[0x08], []byte{0, 0, imx585POACropYHDR, 0}; !bytes.Equal(got, want) {
			t.Errorf("%s: FPGA crop burst = % x, want % x", c.name, got, want)
		}
		// The HDR offset register is offset-derived, so SetROI must NOT write it. Reading the
		// captured 0x0102 as a mode constant is exactly the mistake this guards.
		if got, ok := rm.fpgaBursts[imx585FPGAHDROffset]; ok {
			t.Errorf("%s: SetROI wrote the HDR offset 0x36 = % x; it belongs to SetOffset", c.name, got)
		}
		fpga := lastVals(rm.fpgaWrites)
		if fpga[0x38] != 1 {
			t.Errorf("%s: wide flag (reg 0x38) = %d, want 1", c.name, fpga[0x38])
		}
		if fpga[0x02] != 0x81 {
			t.Errorf("%s: format byte = 0x%02x, want 0x81 (RAW16)", c.name, fpga[0x02])
		}
		if fpga[0x04] != 0x00 {
			t.Errorf("%s: mode byte = 0x%02x, want 0x00 (bin 1, average)", c.name, fpga[0x04])
		}
		// The SENSOR window is not doubled: 2x2180 is past the end of the array. The doubling is
		// the DOL readout making two passes over the same window, which only the FPGA sees.
		sen := lastVals(rm.writes)
		if got := int(sen[imx585RegHeightL]) | int(sen[imx585RegHeightH])<<8; got != c.h {
			t.Errorf("%s: sensor window height = %d, want %d undoubled", c.name, got, c.h)
		}
	}
}

// TestIMX585NormalSetROIUnaffectedByHDR guards the working path: adding HDR must not move
// anything in Normal mode, which is the mode every validated capture so far was taken in.
func TestIMX585NormalSetROIUnaffectedByHDR(t *testing.T) {
	rm := hdrPOA(imx585ModeNormalPOA, 2)
	if err := IMX585.SetROI(rm, 0, 0, 640, 480, 1); err != nil {
		t.Fatal(err)
	}
	wantSize := []byte{0x80, 0x02, 0xe0, 0x01, 0x00, 0x58, 0x02, 0x00}
	if got := rm.fpgaBursts[0x0c]; !bytes.Equal(got, wantSize) {
		t.Errorf("size burst = % x, want % x (height NOT doubled in Normal)", got, wantSize)
	}
	if got, want := rm.fpgaBursts[0x08], []byte{0, 0, imx585POACropY, 0}; !bytes.Equal(got, want) {
		t.Errorf("crop burst = % x, want % x", got, want)
	}
	if got, ok := rm.fpgaBursts[imx585FPGAHDROffset]; ok {
		t.Errorf("Normal wrote the HDR offset 0x36 = % x; the SDK never writes it outside HDR", got)
	}
	if got := lastVals(rm.fpgaWrites)[0x38]; got != 0 {
		t.Errorf("wide flag = %d, want 0 in Normal", got)
	}
}

// TestIMX585HDRSensorBlockMatchesCapture: the thirteen registers HDR re-tunes, and the fact that
// it re-tunes rather than extends — no register is added or removed, so a mode change cannot be
// implemented by appending writes.
func TestIMX585HDRSensorBlockMatchesCapture(t *testing.T) {
	normal, err := imx585ModePOA(imx585ModeNormalPOA, 2)
	if err != nil {
		t.Fatal(err)
	}
	hdr, err := imx585ModePOA(imx585ModeHDRPOA, 2)
	if err != nil {
		t.Fatal(err)
	}
	if len(normal) != len(hdr) {
		t.Fatalf("block sizes differ: Normal %d, HDR %d — HDR re-tunes the same block", len(normal), len(hdr))
	}
	nm, hm := lastVals(normal), lastVals(hdr)
	// Every value read off the wire in the HDR RAW16 blocks of the capture.
	// Twelve, not the thirteen a capture suggests: 0x423d looked like a mode register because
	// every capture of a mode block was taken at some gain, and it is gain-derived.
	want := map[uint16]uint16{
		0x301a: 0x10, 0x3069: 0x02, 0x3074: 0x63, 0x3930: 0xe6, 0x3931: 0x00,
		0x3a4c: 0x61, 0x3a4d: 0x02, 0x3a50: 0x70, 0x3a51: 0x02, 0x3e10: 0x17,
		0x493c: 0x41, 0x4940: 0x41,
	}
	if _, ok := hm[imx585RegFineGain]; ok {
		t.Error("the mode block still carries 0x423d; it is gain-derived, not a mode register")
	}
	for reg, val := range want {
		if got := hm[reg]; got != val {
			t.Errorf("HDR 0x%04x = 0x%02x, want 0x%02x", reg, got, val)
		}
	}
	// Exactly those thirteen differ from Normal RAW16 and nothing else does.
	var diff []uint16
	for reg, val := range nm {
		if hm[reg] != val {
			diff = append(diff, reg)
		}
	}
	if len(diff) != len(want) {
		t.Errorf("%d registers differ from Normal RAW16 (%v), want exactly %d", len(diff), diff, len(want))
	}
	for _, reg := range diff {
		if _, ok := want[reg]; !ok {
			t.Errorf("unexpected register 0x%04x differs between Normal and HDR", reg)
		}
	}
	// Three of the thirteen sit in the sample-size half. That overlap is what makes the table
	// two-dimensional: HDR is not expressible as a patch over a sample size.
	for _, reg := range []uint16{0x3930, 0x3931} {
		n8, err := imx585ModePOA(imx585ModeNormalPOA, 1)
		if err != nil {
			t.Fatal(err)
		}
		if lastVals(n8)[reg] == nm[reg] {
			t.Errorf("0x%04x does not move with sample size; the joint indexing claim is wrong", reg)
		}
	}
}

// TestIMX585HDRRefusesRAW8: the one uncaptured cell. Both halves of a mode change must refuse it
// — reusing either neighbour is what leaves the sensor emitting samples the frame layout does not
// describe, and that failure reads as a plausible image rather than an error.
func TestIMX585HDRRefusesRAW8(t *testing.T) {
	if _, err := imx585ModePOA(imx585ModeHDRPOA, 1); err == nil {
		t.Error("imx585ModePOA(HDR, RAW8) returned a block; the cell is not captured")
	}
	if err := IMX585.SetSensorMode(hdrPOA(imx585ModeHDRPOA, 1), imx585ModeHDRPOA); err == nil {
		t.Error("SetSensorMode(HDR) at RAW8 succeeded; want a refusal")
	}
	if err := IMX585.SetROI(hdrPOA(imx585ModeHDRPOA, 1), 0, 0, 640, 480, 1); err == nil {
		t.Error("SetROI in HDR at RAW8 succeeded; want a refusal")
	}
	// An undecoded mode index is refused too, rather than silently reading as Normal.
	if _, err := imx585ModePOA(7, 2); err == nil {
		t.Error("imx585ModePOA(7, RAW16) returned a block for an undecoded mode")
	}
}

// TestIMX585HDRDriveMatchesCapture: HDR moves the whole timing budget into VMAX. HMAX sits at the
// RAW16 floor even at full width, where Normal scales it with width to 290, and the frame PERIOD
// is unchanged — which is the invariant worth asserting, because it is what makes the two modes
// take the same time per frame off the same link.
func TestIMX585HDRDriveMatchesCapture(t *testing.T) {
	// The capture ran on USB2 at the SDK's default 90% bandwidth, 10 ms exposure, bin 1.
	const (
		usb3 = false
		pct  = 90
		exp  = 10 * time.Millisecond
	)
	normH, normV := imx585DrivePOA(3856, 2180, 2, 1, exp, usb3, pct, 0, false)
	hdrH, hdrV := imx585DrivePOA(3856, 2180, 2, 1, exp, usb3, pct, 0, true)

	if normH != 290 {
		t.Errorf("Normal full-frame HMAX = %d, want 290 (the width term)", normH)
	}
	if hdrH != imx585POAHMAXFloor16 {
		t.Errorf("HDR full-frame HMAX = %d, want the RAW16 floor %d", hdrH, imx585POAHMAXFloor16)
	}
	// The periods agree: on the wire 290x29996 against 154x56488, 0.004% apart.
	nP, hP := float64(normH)*float64(normV), float64(hdrH)*float64(hdrV)
	if d := (nP - hP) / nP; d > 0.001 || d < -0.001 {
		t.Errorf("frame period differs between modes by %.3f%% (Normal %.0f, HDR %.0f); HDR only re-splits it", d*100, nP, hP)
	}
	// VMAX against the bytes the SDK sent. The tolerance is the throughput constant's own error,
	// which shows up identically in the Normal path (30001 computed against 29996 captured).
	for _, c := range []struct {
		name     string
		w, h     int
		wantVMAX uint32
	}{
		{"full frame", 3856, 2180, 56488},
		{"640x480", 640, 480, 2064},
	} {
		hm, vm := imx585DrivePOA(c.w, c.h, 2, 1, exp, usb3, pct, 0, true)
		if hm != imx585POAHMAXFloor16 {
			t.Errorf("%s: HDR HMAX = %d, want %d", c.name, hm, imx585POAHMAXFloor16)
		}
		if d := (float64(vm) - float64(c.wantVMAX)) / float64(c.wantVMAX); d > 0.001 || d < -0.001 {
			t.Errorf("%s: HDR VMAX = %d, want %d within 0.1%% (off by %.3f%%)", c.name, vm, c.wantVMAX, d*100)
		}
	}
}

// TestIMX585SensorModesAreVendorGated: the mode list is what a vendor's firmware exposes over the
// die, not a property of the silicon. The ZWO transcription of this die has one programme, and
// its SetSensorMode must refuse rather than write PlayerOne's register values to a ZWO body.
func TestIMX585SensorModesAreVendorGated(t *testing.T) {
	if got := IMX585.SensorModes(POA.VID); len(got) != 2 {
		t.Fatalf("PlayerOne sensor modes = %v, want Normal and HDR", got)
	}
	if got := IMX585.SensorModes(POA.VID); got[imx585ModeHDRPOA].Name != "HDR" {
		t.Errorf("mode %d is %q, want HDR", imx585ModeHDRPOA, got[imx585ModeHDRPOA].Name)
	}
	if got := IMX585.SensorModes(ZWO.VID); got != nil {
		t.Errorf("ZWO sensor modes = %v, want none", got)
	}
	zwo := &modeRegmap{mode: ReadoutMode{BytesPerPx: 2}, regVals: map[uint16]uint16{}}
	if err := IMX585.SetSensorMode(zwo, imx585ModeHDRPOA); err == nil {
		t.Error("SetSensorMode on a ZWO body succeeded; want a refusal")
	}
	if len(zwo.writes) != 0 {
		t.Errorf("SetSensorMode wrote %d registers on the way to refusing on ZWO", len(zwo.writes))
	}
}

// TestIMX585FineGainPOA: the fine-gain register 0x423d, against values read off the camera after
// the SDK programmed it. One cubic serves both modes; Normal enters it 270 units in and stops at
// gain 45, HDR enters at 0 and stops at 72, and above that it is a flat 0x80 in both.
func TestIMX585FineGainPOA(t *testing.T) {
	for _, c := range []struct {
		gain int
		hdr  bool
		want uint16
	}{
		// Read back off the camera after a poasnap capture at that gain.
		{0, false, 0xa6},   // and the value the Normal RAW16 mode block used to carry
		{0, true, 0xb6},    // and the value the HDR RAW16 mode block used to carry
		{100, false, 0x80}, // above Normal's limit
		{100, true, 0x80},  // above HDR's limit
		{300, false, 0x80},
		{600, false, 0x80},
		// Computed from the decoded cubic; the curve falls monotonically to the flat value.
		{15, false, 0x9c}, // the value the RAW8 mode block used to carry: a gain, not a depth
		{45, false, 0x80}, // Normal's limit, where the curve meets the flat value
		{46, false, 0x80},
		{72, true, 0x80}, // HDR's limit, likewise
		{73, true, 0x80},
	} {
		mode := "normal"
		if c.hdr {
			mode = "HDR"
		}
		if got := imx585FineGainPOA(c.gain, c.hdr); got != c.want {
			t.Errorf("fine gain %s gain %d = 0x%02x, want 0x%02x", mode, c.gain, got, c.want)
		}
	}
	// The register is always even and never below the flat value: it is 0x80 plus twice a
	// non-negative polynomial.
	for _, hdr := range []bool{false, true} {
		for g := 0; g <= 750; g++ {
			v := imx585FineGainPOA(g, hdr)
			if v&1 != 0 || v < imx585FineGainFlatPOA {
				t.Fatalf("fine gain hdr=%v gain %d = 0x%02x, want even and >= 0x80", hdr, g, v)
			}
		}
	}
}

// TestIMX585HDROffsetPOA: the HDR offset register against the SDK sweep read off the camera. The
// capture's 0x0102 was one offset's worth, not a constant — 3 lands on it.
func TestIMX585HDROffsetPOA(t *testing.T) {
	for offset, want := range map[int]uint16{
		0: 0, 10: 860, 30: 2581, 60: 5163, 100: 8606, 150: 12909, 200: 17213, 250: 21516,
	} {
		if got := imx585HDROffsetPOA(offset); got != want {
			t.Errorf("HDR offset for %d = %d, want %d", offset, got, want)
		}
	}
	if got := imx585HDROffsetPOA(3); got != 0x0102 {
		t.Errorf("HDR offset for 3 = 0x%04x, want 0x0102 (the value the capture showed)", got)
	}
}

// TestIMX585SetOffsetHDRWritesFPGA: Normal writes only the sensor register, HDR writes both.
func TestIMX585SetOffsetHDRWritesFPGA(t *testing.T) {
	norm := hdrPOA(imx585ModeNormalPOA, 2)
	if err := IMX585.SetOffset(norm, 30); err != nil {
		t.Fatal(err)
	}
	if got, ok := norm.fpgaBursts[imx585FPGAHDROffset]; ok {
		t.Errorf("Normal wrote the HDR offset = % x; the SDK never writes it outside HDR", got)
	}
	hdr := hdrPOA(imx585ModeHDRPOA, 2)
	if err := IMX585.SetOffset(hdr, 30); err != nil {
		t.Fatal(err)
	}
	if got, want := hdr.fpgaBursts[imx585FPGAHDROffset], []byte{0x15, 0x0a}; !bytes.Equal(got, want) {
		t.Errorf("HDR offset burst = % x, want % x (2581 little-endian)", got, want)
	}
	// A ZWO body must never see the PlayerOne FPGA register.
	zwo := &modeRegmap{mode: ReadoutMode{BytesPerPx: 2, SensorMode: imx585ModeHDRPOA}, regVals: map[uint16]uint16{}}
	if err := IMX585.SetOffset(zwo, 30); err != nil {
		t.Fatal(err)
	}
	if _, ok := zwo.fpgaBursts[imx585FPGAHDROffset]; ok {
		t.Error("SetOffset wrote PlayerOne's HDR offset register on a ZWO body")
	}
}

// TestIMX585GainHDREncoding: HDR drops the high-conversion-gain band entirely — 0x3030 stays 0 at
// every gain — rebases on 72 instead of 45, and clamps at 500 rather than the advertised 750.
func TestIMX585GainHDREncoding(t *testing.T) {
	code := func(rm *modeRegmap) (conv, code uint16) {
		v := lastVals(rm.writes)
		return v[imx585RegConvGain], v[imx585RegGainL] | v[imx585RegGainH]<<8
	}
	for _, c := range []struct {
		gain             int
		wantConv, wantCd uint16
	}{
		{0, 0, 0},     // below the rebase the code is 0 and fine gain carries it
		{72, 0, 0},    // at the rebase
		{75, 0, 1},    // (75-72)/3
		{300, 0, 76},  // (300-72)/3
		{500, 0, 142}, // the HDR clamp
		{750, 0, 142}, // clamped: 750 is the Normal maximum, not HDR's
	} {
		rm := hdrPOA(imx585ModeHDRPOA, 2)
		if err := IMX585.SetGain(rm, c.gain); err != nil {
			t.Fatalf("gain %d: %v", c.gain, err)
		}
		conv, cd := code(rm)
		if conv != c.wantConv || cd != c.wantCd {
			t.Errorf("HDR gain %d: conv=%d code=%d, want conv=%d code=%d", c.gain, conv, cd, c.wantConv, c.wantCd)
		}
	}
	// Normal still reaches HCG at 210, which HDR never does.
	rm := hdrPOA(imx585ModeNormalPOA, 2)
	if err := IMX585.SetGain(rm, 300); err != nil {
		t.Fatal(err)
	}
	if conv, cd := code(rm); conv != 1 || cd != (300-imx585HCGSubPOA)/3 {
		t.Errorf("Normal gain 300: conv=%d code=%d, want conv=1 code=%d", conv, cd, (300-imx585HCGSubPOA)/3)
	}
}

// TestIMX585ExposureCapsAreVendorSpecific: PlayerOne advertises 10 us to 7200 s on this die where
// the ZWO transcription clamps to 32 us and 2000 s. The 7200 s ceiling only shows up in the
// float config: POA_EXPOSURE counts microseconds in an int32 and so stops at 2,000,000,000, while
// POA_EXP reports 7200 seconds, and the header calls POA_EXP the one to use.
func TestIMX585ExposureCapsAreVendorSpecific(t *testing.T) {
	if IMX585.ExpCaps == nil {
		t.Fatal("IMX585 declares no ExpCaps, so both vendors would share one range")
	}
	if lo, hi := IMX585.ExpCaps(POA.VID); lo != 10 || hi != 7_200_000_000 {
		t.Errorf("PlayerOne exposure caps = %d..%d us, want 10..7200000000", lo, hi)
	}
	if lo, hi := IMX585.ExpCaps(ZWO.VID); lo != imx585ExpMinUs || hi != imx585ExpMaxUs {
		t.Errorf("ZWO exposure caps = %d..%d us, want %d..%d", lo, hi, imx585ExpMinUs, imx585ExpMaxUs)
	}
}

// TestIMX585ShortExposureSHS: the shutter comes off VMAX directly, not off a VMAX-minus-guard
// window. Subtracting the guard twice put an eight-line floor — 62 us at this HMAX — under every
// short exposure, so a requested 10 us integrated for 62. Measured against the SDK, which
// programs SHS = VMAX-2 at 10 us and VMAX-4 at 32 us.
func TestIMX585ShortExposureSHS(t *testing.T) {
	const hmax = 154 // 640x480 RAW16 on this link
	for _, c := range []struct {
		exp       time.Duration
		wantLines uint32
	}{
		{10 * time.Microsecond, imx585MinExpLinesPOA}, // under one line: the floor applies
		{32 * time.Microsecond, 4},                    // 32 / 7.7
		{100 * time.Microsecond, 12},
	} {
		rm := hdrPOA(imx585ModeNormalPOA, 2)
		rm.mode.Width, rm.mode.Height, rm.mode.USB3, rm.mode.FPSPercent = 640, 480, false, 90
		if err := IMX585.SetExposure(rm, c.exp); err != nil {
			t.Fatalf("%v: %v", c.exp, err)
		}
		v := lastVals(rm.writes)
		shs := uint32(v[imx585RegSHS0]) | uint32(v[imx585RegSHS1])<<8 | uint32(v[imx585RegSHS2])<<16
		drive := rm.fpgaBursts[0x14]
		vmax := uint32(drive[2]) | uint32(drive[3])<<8 | uint32(drive[4])<<16
		if got := vmax - shs; got != c.wantLines {
			t.Errorf("%v: integration = %d lines (VMAX %d - SHS %d), want %d", c.exp, got, vmax, shs, c.wantLines)
		}
		if uint16(drive[0])|uint16(drive[1])<<8 != hmax {
			t.Errorf("%v: HMAX = %d, want %d", c.exp, uint16(drive[0])|uint16(drive[1])<<8, hmax)
		}
	}
}

// TestIMX585BinSumAndFrameLimit: the two controls that were decoded but unwired. Bin-sum is bit 4
// of the FPGA mode byte, confirmed on the wire as 0x01 averaged against 0x11 summed at bin 2, and
// the frame-rate cap is a third term in the period beside the link budget and the exposure.
func TestIMX585BinSumAndFrameLimit(t *testing.T) {
	mode := func(sum bool) uint16 {
		rm := hdrPOA(imx585ModeNormalPOA, 2)
		rm.mode.BinSum = sum
		if err := IMX585.SetROI(rm, 0, 0, 320, 240, 2); err != nil {
			t.Fatal(err)
		}
		return lastVals(rm.fpgaWrites)[0x04]
	}
	if got := mode(false); got != 0x01 {
		t.Errorf("bin 2 averaged: mode byte = 0x%02x, want 0x01", got)
	}
	if got := mode(true); got != 0x11 {
		t.Errorf("bin 2 summed: mode byte = 0x%02x, want 0x11 (bit 4)", got)
	}

	// A cap slower than the link budget lengthens the period; a faster one changes nothing,
	// because the period is the longest of the three constraints, not the last one set.
	const exp = time.Millisecond // short, so the link budget rules when uncapped
	base := func(limit int) uint32 {
		_, v := imx585DrivePOA(640, 480, 2, 1, exp, false, 90, limit, false)
		return v
	}
	uncapped := base(0)
	if got := base(100); got != uncapped {
		t.Errorf("a 100 fps cap (10 ms) moved VMAX to %d from %d; it is shorter than the link budget", got, uncapped)
	}
	for _, c := range []struct {
		limit  int
		wantMs float64
	}{{30, 33.33}, {10, 100.0}} {
		v := base(c.limit)
		ms := float64(154) * float64(v) / 2e7 * 1000
		if ms < c.wantMs*0.99 || ms > c.wantMs*1.01 {
			t.Errorf("%d fps cap: period = %.2f ms, want %.2f", c.limit, ms, c.wantMs)
		}
	}
}

// TestIMX585SensorBinSplit: die-side binning against the SDK. The die bins by 2 and nothing else,
// so bin 2 goes wholly to it, bin 4 splits 2 and 2 with the FPGA, and bin 3 has no die mode and
// falls back entirely — the same fallback the SDK makes. The sensor WINDOW never changes; only
// what the FPGA is handed does.
func TestIMX585SensorBinSplit(t *testing.T) {
	for _, c := range []struct {
		bin, senBin        int
		wantMode, wantFact uint16 // sensor 0x301b / 0x30d5
		wantFPGABin        uint16 // FPGA 0x04
		wantW, wantH       int    // FPGA image size
	}{
		{2, 1, 0, 4, 0x01, 3856, 2180}, // FPGA does it all
		{2, 2, 1, 2, 0x00, 1928, 1090}, // the die does it all
		{3, 1, 0, 4, 0x02, 3852, 2178}, // no die mode at 3
		{4, 1, 0, 4, 0x03, 3856, 2176},
		{4, 2, 1, 2, 0x01, 1928, 1088}, // 2 on the die, 2 in the FPGA
	} {
		rm := hdrPOA(imx585ModeNormalPOA, 2)
		rm.mode.SensorBin = c.senBin
		w, h := 3856/c.bin, 2180/c.bin
		if c.bin == 3 {
			w, h = 1284, 726 // the SDK's bin-3 geometry, which is not a plain divide
		}
		if c.bin == 4 {
			h = 544
		}
		if err := IMX585.SetROI(rm, 0, 0, w, h, c.bin); err != nil {
			t.Fatalf("bin %d senBin %d: %v", c.bin, c.senBin, err)
		}
		sen := lastVals(rm.writes)
		if sen[imx585RegSenBinMode] != c.wantMode || sen[imx585RegSenBinFactor] != c.wantFact {
			t.Errorf("bin %d senBin %d: 0x301b=%d 0x30d5=%d, want %d/%d",
				c.bin, c.senBin, sen[imx585RegSenBinMode], sen[imx585RegSenBinFactor], c.wantMode, c.wantFact)
		}
		if got := lastVals(rm.fpgaWrites)[0x04]; got != c.wantFPGABin {
			t.Errorf("bin %d senBin %d: FPGA bin field = 0x%02x, want 0x%02x", c.bin, c.senBin, got, c.wantFPGABin)
		}
		sz := rm.fpgaBursts[0x0c]
		if gw, gh := int(sz[0])|int(sz[1])<<8, int(sz[2])|int(sz[3])<<8; gw != c.wantW || gh != c.wantH {
			t.Errorf("bin %d senBin %d: FPGA size = %dx%d, want %dx%d", c.bin, c.senBin, gw, gh, c.wantW, c.wantH)
		}
		// The sensor window is the full unbinned extent, rounded UP to 16 — the FPGA crops the
		// margin. At bin 3 that is 3852 rounded to 3856.
		wantSenW := (w*c.bin + 15) &^ 15
		if got := int(sen[imx585RegWidthL]) | int(sen[imx585RegWidthH])<<8; got != wantSenW {
			t.Errorf("bin %d senBin %d: sensor window width = %d, want %d (%d rounded up to 16)",
				c.bin, c.senBin, got, wantSenW, w*c.bin)
		}
	}
}

// TestIMX585GpifBwIsLinkDependent: the GPIF bandwidth divider's numerator is link-dependent,
// because the divisor it comes from is the link rate scaled by the percentage. Both constants are
// measured against the SDK on the camera — eight percentages on each link, every one exact.
// Programming USB2's value on a SuperSpeed link sets the GPIF an order of magnitude wrong; that
// took the device off the bus under die-side binning, the highest sustained rate there is.
func TestIMX585GpifBwIsLinkDependent(t *testing.T) {
	for _, c := range []struct {
		pct  int
		usb3 bool
		want uint16
	}{
		{35, false, 6914}, {50, false, 4763}, {90, false, 2532}, {100, false, 2253},
		{35, true, 545}, {50, true, 305}, {90, true, 55}, {100, true, 24},
	} {
		link := "USB2"
		if c.usb3 {
			link = "USB3"
		}
		if got := imx585GpifBwPOA(c.pct, c.usb3); got != c.want {
			t.Errorf("%s at %d%%: GPIF bw = %d, want %d", link, c.pct, got, c.want)
		}
	}
	// The two links must never agree at the same percentage, which is the bug this guards.
	if imx585GpifBwPOA(90, false) == imx585GpifBwPOA(90, true) {
		t.Error("the two links produced the same divider; the constant is not link-dependent")
	}
}

// TestIMX585SensorWidthRounding: the SENSOR window width is rounded up to a multiple of 16 while
// the FPGA gets the requested width and crops the margin. Programming it verbatim leaves the
// sensor emitting short lines into a frame laid out for the full width, and the readout shears.
// Values are what the SDK programs, read back off the camera.
func TestIMX585SensorWidthRounding(t *testing.T) {
	for _, c := range []struct{ req, wantSensor int }{
		{320, 320}, // already aligned
		{324, 336}, {328, 336}, {332, 336},
		{340, 352}, {344, 352}, {348, 352},
		{352, 352},
		{640, 640}, {3856, 3856}, // the common windows, all already aligned
	} {
		rm := hdrPOA(imx585ModeNormalPOA, 1)
		if err := IMX585.SetROI(rm, 0, 0, c.req, 240, 1); err != nil {
			t.Fatalf("w %d: %v", c.req, err)
		}
		sen := lastVals(rm.writes)
		if got := int(sen[imx585RegWidthL]) | int(sen[imx585RegWidthH])<<8; got != c.wantSensor {
			t.Errorf("requested %d: sensor width = %d, want %d", c.req, got, c.wantSensor)
		}
		// The FPGA still gets the exact request — that is what makes the delivered frame the
		// size the caller asked for.
		sz := rm.fpgaBursts[0x0c]
		if got := int(sz[0]) | int(sz[1])<<8; got != c.req {
			t.Errorf("requested %d: FPGA width = %d, want the request unrounded", c.req, got)
		}
	}
}
