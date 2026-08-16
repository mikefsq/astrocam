package sensors

import (
	"testing"
	"time"

	. "github.com/mikefsq/astrocam"
)

// fakeRegmap records register writes so a sensor profile's encoding can be asserted without any
// transport. Sensor-bus writes (WriteReg/WriteRegBits) and FPGA-bus writes (WriteFPGAReg) go to
// SEPARATE logs: they are distinct register spaces on hardware, and merging them lets the FX3
// reg-1 commit strobe (FPGAWrite16) collide with a sensor reg 0x0001. Use lastVals(f.writes)
// for sensor regs and lastVals(f.fpgaWrites) for FPGA regs.
type fakeRegmap struct {
	writes     []RegVal // sensor bus (WriteSONYREG / camera-reg)
	fpgaWrites []RegVal // FPGA bus (WriteFPGAREG)
	vid        uint16   // vendor the profile should dispatch on (0 → ZWO path)
}

// VID defaults to ZWO so the single-vendor sensor tests dispatch to the ZWO encoding without
// setting it; PlayerOne (or unknown-vendor) tests set vid explicitly.
func (f *fakeRegmap) VID() uint16 {
	if f.vid == 0 {
		return ZWO.VID
	}
	return f.vid
}

func (f *fakeRegmap) WriteReg(reg, val uint16) error {
	f.writes = append(f.writes, RegVal{Reg: reg, Val: val})
	return nil
}
func (f *fakeRegmap) WriteRegBits(reg uint16, _, _ uint8, val uint16) error {
	f.writes = append(f.writes, RegVal{Reg: reg, Val: val})
	return nil
}
func (f *fakeRegmap) ReadReg(uint16) (uint16, error) { return 0, nil }
func (f *fakeRegmap) WriteFPGAReg(reg, val uint16) error {
	f.fpgaWrites = append(f.fpgaWrites, RegVal{Reg: reg, Val: val})
	return nil
}
func (f *fakeRegmap) ReadFPGAReg(uint16) (uint16, error) { return 0, nil }

// lastVals folds a write log into reg→last-value-written.
func lastVals(ws []RegVal) map[uint16]uint16 {
	m := map[uint16]uint16{}
	for _, w := range ws {
		m[w.Reg] = w.Val
	}
	return m
}

// TestIMX174Gain asserts the IMX174 gain register sequence (SetGain): gain 100
// (within the 0..400 clamp) is written straight to the 16-bit code 0x404/0x405
// (low=0x64, high=0), latched by 0x20c.
func TestIMX174Gain(t *testing.T) {
	f := &fakeRegmap{}
	if err := IMX174.SetGain(f, 100); err != nil { // 100 = 0x64
		t.Fatal(err)
	}
	want := []RegVal{{Reg: 0x20c, Val: 1}, {Reg: 0x404, Val: 0x64}, {Reg: 0x405, Val: 0x00}, {Reg: 0x20c, Val: 0}}
	if len(f.writes) != len(want) {
		t.Fatalf("got %d writes, want %d: %+v", len(f.writes), len(want), f.writes)
	}
	for i := range want {
		if f.writes[i] != want[i] {
			t.Errorf("write %d = %+v, want %+v", i, f.writes[i], want[i])
		}
	}
}

// TestIMX178Exposure locks the IMX178 exposure path: a BAKED HMAX of 420
// (line_time = 420·1e6/27000 = 15555 ns), VMAX = height+29, SHS = height+29−lines.
// A 10 ms exposure stays within one frame; a 2 s exposure trips FPGA trigger mode.
func TestIMX178Exposure(t *testing.T) {
	f := &fakeRegmap{}
	if err := IMX178.SetExposure(f, 10*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	sens, fpga := lastVals(f.writes), lastVals(f.fpgaWrites)
	// line_time = 15555 ns; lines = 10_000_000/15555 = 642; VMAX = 2048+29 = 2077 = 0x00081d;
	// SHS = 2048+29-642 = 1435 = 0x00059b.
	if fpga[0x10] != 0x1d || fpga[0x11] != 0x08 || fpga[0x12] != 0x00 {
		t.Errorf("VMAX = %02x/%02x/%02x, want 1d/08/00 (2077)", fpga[0x10], fpga[0x11], fpga[0x12])
	}
	if sens[0x3034] != 0x9b || sens[0x3035] != 0x05 || sens[0x3036] != 0x00 {
		t.Errorf("SHS = %02x/%02x/%02x, want 9b/05/00 (1435)", sens[0x3034], sens[0x3035], sens[0x3036])
	}
	// ≥ 1 s engages FPGA trigger mode (reg0 bit7).
	g := &fakeRegmap{}
	if err := IMX178.SetExposure(g, 2*time.Second); err != nil {
		t.Fatal(err)
	}
	var sawTrig bool
	for _, w := range g.fpgaWrites {
		if w.Reg == 0x00 && w.Val&0x80 != 0 {
			sawTrig = true
		}
	}
	if !sawTrig {
		t.Errorf("2 s exposure: FPGA trigger mode (reg0 bit7) not set")
	}
}

// TestIMX462Exposure locks the 462 STARVIS exposure path:
// VMAX = height+18 = 1098 for a sub-frame exposure (SetExp), and ≥ 1 s trips
// FPGA trigger mode (reg0 bit7).
func TestIMX462Exposure(t *testing.T) {
	f := &fakeRegmap{}
	if err := IMX462.SetExposure(f, 10*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	fpga := lastVals(f.fpgaWrites)
	// VMAX = 1096+18 = 1114 = 0x00045a → FPGA 0x10/0x11/0x12 = 5a/04/00 (height is the
	// hardware-confirmed 1096, not the datasheet 1080; mode-independent for sub-frame).
	if fpga[0x10] != 0x5a || fpga[0x11] != 0x04 || fpga[0x12] != 0x00 {
		t.Errorf("VMAX = %02x/%02x/%02x, want 5a/04/00 (1114 = height+18)", fpga[0x10], fpga[0x11], fpga[0x12])
	}
	g := &fakeRegmap{}
	if err := IMX462.SetExposure(g, 2*time.Second); err != nil {
		t.Fatal(err)
	}
	var sawTrig bool
	for _, w := range g.fpgaWrites {
		if w.Reg == 0x00 && w.Val&0x80 != 0 {
			sawTrig = true
		}
	}
	if !sawTrig {
		t.Errorf("2 s exposure: FPGA trigger mode (reg0 bit7) not set")
	}
}

// TestIMX585Gain locks the STARVIS-2 dual-gain (SetGain): below the threshold gain/3
// to 0x306c at LCG (0x3030=0); above gain 199 the conversion gain engages (0x3030=1) and the code is
// (gain-150)/3 — a reset, not a continuation.
func TestIMX585Gain(t *testing.T) {
	// gain 199: LCG, code = 199/3 = 66 = 0x42.
	f := &fakeRegmap{}
	if err := IMX585.SetGain(f, 199); err != nil {
		t.Fatal(err)
	}
	m := map[uint16]uint16{}
	for _, w := range f.writes {
		m[w.Reg] = w.Val
	}
	if m[0x3030] != 0 || m[0x306c] != 0x42 || m[0x306d] != 0 {
		t.Errorf("gain 199 (LCG): conv=%d code=%02x/%02x, want 0 / 42/00", m[0x3030], m[0x306c], m[0x306d])
	}
	// gain 200: HCG, code = (200-150)/3 = 16 = 0x10, conv = 1.
	g := &fakeRegmap{}
	if err := IMX585.SetGain(g, 200); err != nil {
		t.Fatal(err)
	}
	m = map[uint16]uint16{}
	for _, w := range g.writes {
		m[w.Reg] = w.Val
	}
	if m[0x3030] != 1 || m[0x306c] != 0x10 || m[0x306d] != 0 {
		t.Errorf("gain 200 (HCG): conv=%d code=%02x/%02x, want 1 / 10/00", m[0x3030], m[0x306c], m[0x306d])
	}
}

// TestIMX585Exposure locks the STARVIS-2 exposure (baked HMAX 192 → line 9600 ns, VMAX=height+2,
// SHS=clamp((VMAX-8)-lines,8,VMAX-8) to 0x3050-52) and the ≥1 s trigger mode.
func TestIMX585Exposure(t *testing.T) {
	f := &fakeRegmap{}
	if err := IMX585.SetExposure(f, 10*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	sens, fpga := lastVals(f.writes), lastVals(f.fpgaWrites)
	// lines = 10_000_000/9600 = 1041; VMAX = 2160+2 = 2162 = 0x000872; SHS = (2162-8)-1041 = 1113 = 0x000459.
	if fpga[0x10] != 0x72 || fpga[0x11] != 0x08 || fpga[0x12] != 0x00 {
		t.Errorf("VMAX = %02x/%02x/%02x, want 72/08/00 (2162)", fpga[0x10], fpga[0x11], fpga[0x12])
	}
	if sens[0x3050] != 0x59 || sens[0x3051] != 0x04 || sens[0x3052] != 0x00 {
		t.Errorf("SHS = %02x/%02x/%02x, want 59/04/00 (1113)", sens[0x3050], sens[0x3051], sens[0x3052])
	}
	g := &fakeRegmap{}
	if err := IMX585.SetExposure(g, 2*time.Second); err != nil {
		t.Fatal(err)
	}
	var sawTrig bool
	for _, w := range g.fpgaWrites {
		if w.Reg == 0x00 && w.Val&0x80 != 0 {
			sawTrig = true
		}
	}
	if !sawTrig {
		t.Errorf("2 s exposure: FPGA trigger mode (reg0 bit7) not set")
	}
}

// TestIMX455Exposure2s locks the trigger-mode exposure (SetExp) to the SDK USB capture: a 2 s
// exposure (>= 1 s) must enter wait+trigger mode (FPGA reg0 bit6 0x40 + bit7 0x80), hold VMAX
// at one frame = 0x0019c0 = 6592 (not exposure/line_time = 26400), and write SHS = 10. The
// integration itself is host-timed.
func TestIMX455Exposure2s(t *testing.T) {
	f := &fakeRegmap{}
	if err := IMX455.SetExposure(f, 2*time.Second); err != nil {
		t.Fatal(err)
	}
	sens, fpga := lastVals(f.writes), lastVals(f.fpgaWrites)
	var reg0 []uint16
	for _, w := range f.fpgaWrites {
		if w.Reg == 0x00 {
			reg0 = append(reg0, w.Val)
		}
	}
	// VMAX = 6592 = 0x0019c0 → FPGA 0x10/0x11/0x12 = c0/19/00
	if fpga[0x10] != 0xc0 || fpga[0x11] != 0x19 || fpga[0x12] != 0x00 {
		t.Errorf("VMAX = %02x/%02x/%02x, want c0/19/00 (6592)", fpga[0x10], fpga[0x11], fpga[0x12])
	}
	// SHS = 10 → sensor 0x16/0x17 = 0a/00
	if sens[0x16] != 0x0a || sens[0x17] != 0x00 {
		t.Errorf("SHS = %02x/%02x, want 0a/00 (10)", sens[0x16], sens[0x17])
	}
	var sawWait, sawTrig bool
	for _, v := range reg0 {
		sawWait = sawWait || v&0x40 != 0
		sawTrig = sawTrig || v&0x80 != 0
	}
	if !sawWait || !sawTrig {
		t.Errorf("mode bits: wait=%v trigger=%v (want both); reg0 writes=%v", sawWait, sawTrig, reg0)
	}
}

// TestIMX571Exposure2s locks the 571's trigger-mode exposure to the IMX455 shape it tracks: a
// 2 s exposure enters wait+trigger mode (reg0 bit6+bit7), holds VMAX at ONE frame — bin 1:
// (frameUs+10ms)/line+20 with frameUs = (48+4168)·67.5 µs → 4384 = 0x001120, NOT the exposure
// line count (~29630) — and writes SHS = 20>>1 = 10 to 0x18/0x19. A 10 ms exposure stays in
// free-run: default VMAX 4216 = 0x001078, SHS = (4216−1−148)>>1 = 2033 = 0x07f1.
func TestIMX571Exposure2s(t *testing.T) {
	f := &fakeRegmap{}
	if err := IMX571.SetExposure(f, 2*time.Second); err != nil {
		t.Fatal(err)
	}
	sens, fpga := lastVals(f.writes), lastVals(f.fpgaWrites)
	if fpga[0x10] != 0x20 || fpga[0x11] != 0x11 || fpga[0x12] != 0x00 {
		t.Errorf("VMAX = %02x/%02x/%02x, want 20/11/00 (4384, one frame)", fpga[0x10], fpga[0x11], fpga[0x12])
	}
	if sens[0x18] != 0x0a || sens[0x19] != 0x00 {
		t.Errorf("SHS = %02x/%02x, want 0a/00 (20>>1)", sens[0x18], sens[0x19])
	}
	var sawWait, sawTrig bool
	for _, w := range f.fpgaWrites {
		if w.Reg == 0x00 {
			sawWait = sawWait || w.Val&0x40 != 0
			sawTrig = sawTrig || w.Val&0x80 != 0
		}
	}
	if !sawWait || !sawTrig {
		t.Errorf("mode bits: wait=%v trigger=%v (want both)", sawWait, sawTrig)
	}

	g := &fakeRegmap{}
	if err := IMX571.SetExposure(g, 10*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	sens, fpga = lastVals(g.writes), lastVals(g.fpgaWrites)
	if fpga[0x10] != 0x78 || fpga[0x11] != 0x10 || fpga[0x12] != 0x00 {
		t.Errorf("free-run VMAX = %02x/%02x/%02x, want 78/10/00 (4216)", fpga[0x10], fpga[0x11], fpga[0x12])
	}
	if sens[0x18] != 0xf1 || sens[0x19] != 0x07 {
		t.Errorf("free-run SHS = %02x/%02x, want f1/07 (2033)", sens[0x18], sens[0x19])
	}
}

// regsOf runs IMX455.SetGain at one gain and returns the last value written per register.
func regsOf(t *testing.T, gain int) map[uint16]uint16 {
	t.Helper()
	f := &fakeRegmap{}
	if err := IMX455.SetGain(f, gain); err != nil {
		t.Fatal(err)
	}
	m := map[uint16]uint16{}
	for _, w := range f.writes {
		m[w.Reg] = w.Val
	}
	return m
}

// TestIMX455GainHCG locks the re-traced IMX455 gain map (SetGain).
// The decisive facts: reg 0x2d is the conversion-gain MODE byte whose bit0 is HCG enable
// (0 below gain 100, 1 at/above), and the analog code (0x2e/0x2f, mirror 0x30/0x31) RESETS at
// that boundary — exp10(gain) below 100, exp10(gain−100) at/above. The old decode wrote the
// constant 0x2d to reg 0x2d and never reset the code (gain 100 → 0x0af0); both are corrected.
func TestIMX455GainHCG(t *testing.T) {
	// gain 0: LCG. code = 0, mode bit0 clear, aux 0x4d = 8.
	g0 := regsOf(t, 0)
	for _, reg := range []uint16{0x2e, 0x2f, 0x30, 0x31} {
		if g0[reg] != 0 {
			t.Errorf("gain 0: reg 0x%02x = 0x%02x, want 0", reg, g0[reg])
		}
	}
	if g0[0x2d]&1 != 0 {
		t.Errorf("gain 0: 0x2d = 0x%02x, want HCG bit0 clear (LCG)", g0[0x2d])
	}
	if g0[0x4d] != 8 {
		t.Errorf("gain 0: aux 0x4d = 0x%02x, want 0x08", g0[0x4d])
	}

	// gain 61: still LCG, but mode→4 (bit2) and aux→0x0a at the 60/61 step.
	g61 := regsOf(t, 61)
	if g61[0x2d] != 4 {
		t.Errorf("gain 61: 0x2d = 0x%02x, want 0x04", g61[0x2d])
	}
	if g61[0x4d] != 0xa {
		t.Errorf("gain 61: aux 0x4d = 0x%02x, want 0x0a", g61[0x4d])
	}
	if g61[0x2d]&1 != 0 {
		t.Errorf("gain 61: still LCG, want 0x2d bit0 clear, got 0x%02x", g61[0x2d])
	}

	// gain 100: HCG engages — mode bit0 set AND the analog code resets to 0 (the whole point).
	g100 := regsOf(t, 100)
	if g100[0x2d]&1 != 1 {
		t.Errorf("gain 100: 0x2d = 0x%02x, want HCG bit0 set", g100[0x2d])
	}
	for _, reg := range []uint16{0x2e, 0x2f, 0x30, 0x31} {
		if g100[reg] != 0 {
			t.Errorf("gain 100: reg 0x%02x = 0x%02x, want 0 (code resets at HCG)", reg, g100[reg])
		}
	}
	if g100[0x4d] != 8 {
		t.Errorf("gain 100: aux 0x4d = 0x%02x, want 0x08", g100[0x4d])
	}

	// gain 160 (band C): code restarts the ramp from the reset → exp10(60) = 0x07fa.
	g160 := regsOf(t, 160)
	if g160[0x2e] != 0xfa || g160[0x2f] != 0x07 {
		t.Errorf("gain 160: code = 0x%02x%02x, want 0x07fa", g160[0x2f], g160[0x2e])
	}

	// gain 280 (band D): the high-config switch — 0x3a4→0x23, 0x3a5/0x3a6→0x2d.
	g280 := regsOf(t, 280)
	if g280[0x3a4] != 0x23 || g280[0x3a5] != 0x2d || g280[0x3a6] != 0x2d {
		t.Errorf("gain 280: cfg = 0x3a4=0x%02x 0x3a5=0x%02x 0x3a6=0x%02x, want 0x23/0x2d/0x2d",
			g280[0x3a4], g280[0x3a5], g280[0x3a6])
	}

	// gain 461 (band E): the top-range coarse stage appears in 0x3e bits[4:7] (stage 1 → 0x10);
	// below 461 the conv nibble is 0.
	if g280[0x3e] != 0 {
		t.Errorf("gain 280: 0x3e = 0x%02x, want 0 (no high-range stage below 461)", g280[0x3e])
	}
	g461 := regsOf(t, 461)
	if g461[0x3e] != 0x10 {
		t.Errorf("gain 461: 0x3e = 0x%02x, want 0x10 (stage 1)", g461[0x3e])
	}
}

// TestIMX455ROISubFrame locks the ROI/binning disentanglement: a sub-frame window — even a
// narrow one that the old width-inference (imx455Bin) would have misread as 2× binning —
// must select the BIN-1 readout table and program the window at full resolution, not a
// binned mode. Reg 0x0001 discriminates the table (0x00 = bin-1 full16; 0x85 = bin-2).
func TestIMX455ROISubFrame(t *testing.T) {
	f := &fakeRegmap{}
	// Half-width crop (4788 of 9576) at bin 1: the old width inference would have picked the
	// bin-2 table; an explicit bin 1 must stay the full-res table. x=64 (16-aligned), y=128.
	if err := IMX455.SetROI(f, 64, 128, 4788, 3194, 1); err != nil {
		t.Fatal(err)
	}
	got := map[uint16]uint16{}
	for _, w := range f.writes {
		got[w.Reg] = w.Val
	}
	if got[0x0001] != 0x00 {
		t.Errorf("sub-frame ROI: reg 0x0001 = 0x%02x, want 0x00 (bin-1 full16 table, NOT 0x85 bin-2)", got[0x0001])
	}
	// ROI start X is x>>4 (64 → 4) in 0xa6; window WIDTH reg is width+0x18 in 0x18c.
	if got[0xa6] != 4 {
		t.Errorf("sub-frame ROI: 0xa6 (StartX>>4) = 0x%02x, want 0x04", got[0xa6])
	}
	if want := uint16(4788+0x18) & 0xff; got[0x18c] != want {
		t.Errorf("sub-frame ROI: 0x18c (WidthL) = 0x%02x, want 0x%02x", got[0x18c], want)
	}
}

// TestIMX455Bin2 locks the bin-2 readout geometry: SetROI at bin 2 applies the bin-2
// mode table (reg 0x0001 = 0x85, vs 0x00 full-res), writes the binned output dims to the FPGA
// width/height, and programs HMAX = the bin-2 timing base VBin2 (625). (InitSensorMode /
// Cam_SetResolution.)
func TestIMX455Bin2(t *testing.T) {
	f := &fakeRegmap{}
	// Full-frame bin 2: output = 9576/2 × 6388/2 = 4788 × 3194, start (0,0).
	if err := IMX455.SetROI(f, 0, 0, 4788, 3194, 2); err != nil {
		t.Fatal(err)
	}
	sens, fpga := lastVals(f.writes), lastVals(f.fpgaWrites)
	if sens[0x0001] != 0x85 {
		t.Errorf("bin 2: sensor reg 0x0001 = 0x%02x, want 0x85 (bin-2 table, not 0x00 full-res)", sens[0x0001])
	}
	// FPGA width 0x04/0x05 = output width 4788 = 0x12b4.
	if fpga[0x04] != 0xb4 || fpga[0x05] != 0x12 {
		t.Errorf("bin 2: FPGA width = 0x%02x%02x, want 0x12b4 (4788)", fpga[0x05], fpga[0x04])
	}
	// HMAX 0x13/0x14 = VBin2 = 625 = 0x0271.
	if fpga[0x13] != 0x71 || fpga[0x14] != 0x02 {
		t.Errorf("bin 2: HMAX = 0x%02x%02x, want 0x0271 (VBin2=625)", fpga[0x14], fpga[0x13])
	}
}

// TestSensorOffset locks the offset / black-level (ASI Brightness, SetBrightness): the
// 455 writes offset·10 16-bit LE to sensor 0x40/0x41 mirrored 0x42/0x43; the 290 writes the raw
// value to 0x300a/0x300b.
func TestSensorOffset(t *testing.T) {
	f := &fakeRegmap{}
	if err := IMX455.SetOffset(f, 50); err != nil { // 50·10 = 500 = 0x01f4
		t.Fatal(err)
	}
	s := lastVals(f.writes)
	if s[0x40] != 0xf4 || s[0x41] != 0x01 || s[0x42] != 0xf4 || s[0x43] != 0x01 {
		t.Errorf("455 offset 50: 0x40..0x43 = %02x/%02x/%02x/%02x, want f4/01/f4/01 (500)",
			s[0x40], s[0x41], s[0x42], s[0x43])
	}
	f2 := &fakeRegmap{}
	if err := IMX290.SetOffset(f2, 300); err != nil { // raw 300 = 0x012c
		t.Fatal(err)
	}
	s2 := lastVals(f2.writes)
	if s2[0x300a] != 0x2c || s2[0x300b] != 0x01 {
		t.Errorf("290 offset 300: 0x300a/b = %02x/%02x, want 2c/01", s2[0x300a], s2[0x300b])
	}
}

// TestIMX290Binning locks the IMX290 2× binning (Cam_SetResolution):
// mode byte 0x3006 = 0x22; the sensor window regs (0x3042/3 width, 0x303e/f height)
// take the PHYSICAL region = output·bin; the FPGA geometry takes the OUTPUT dims. bin 3 rejected.
func TestIMX290Binning(t *testing.T) {
	if err := IMX290.SetROI(&fakeRegmap{}, 0, 0, 100, 100, 3); err == nil {
		t.Error("bin 3 accepted, want rejected (290 does 1×/2× only)")
	}
	f := &fakeRegmap{}
	// output 640×480 at bin 2 → sensor window 1280×960, mode 0x22.
	if err := IMX290.SetROI(f, 0, 0, 640, 480, 2); err != nil {
		t.Fatal(err)
	}
	s, fp := lastVals(f.writes), lastVals(f.fpgaWrites)
	if s[0x3006] != 0x22 {
		t.Errorf("bin2: mode 0x3006 = 0x%02x, want 0x22", s[0x3006])
	}
	// sensor WIDTH 0x3042/3 = 640·2 = 1280 = 0x0500.
	if s[0x3042] != 0x00 || s[0x3043] != 0x05 {
		t.Errorf("bin2: sensor width 0x3042/3 = 0x%02x%02x, want 0x0500 (1280 = 640·2)", s[0x3043], s[0x3042])
	}
	// sensor HEIGHT 0x303e/f = 480·2 = 960 = 0x03c0.
	if s[0x303e] != 0xc0 || s[0x303f] != 0x03 {
		t.Errorf("bin2: sensor height 0x303e/f = 0x%02x%02x, want 0x03c0 (960 = 480·2)", s[0x303f], s[0x303e])
	}
	// FPGA width (0x04/0x05) = OUTPUT width 640 = 0x0280 (not ·bin).
	if fp[0x04] != 0x80 || fp[0x05] != 0x02 {
		t.Errorf("bin2: FPGA width = 0x%02x%02x, want 0x0280 (640, output dims)", fp[0x05], fp[0x04])
	}
}

// TestIMX571Binning locks the IMX571 binning + the W/H-swap fix (Cam_SetResolution /
// InitSensorMode). bin 1 writes HEIGHT to 0x0a and the
// ×4-aligned WIDTH+0x18 to 0x1dd (these were swapped before); bin 2 applies the bin-2 mode
// table (reg 0x0001=0x05), sets window-mode 0x1d8=0, and programs the binned FPGA geometry.
func TestIMX571Binning(t *testing.T) {
	// bin 1 full frame: HEIGHT(0x0a)=4168=0x1048; WIDTH(0x1dd)=(6224+0x18)=6248=0x1868 (&0xfc).
	f1 := &fakeRegmap{}
	if err := IMX571.SetROI(f1, 0, 0, 6224, 4168, 1); err != nil {
		t.Fatal(err)
	}
	s1 := lastVals(f1.writes)
	if s1[0x0a] != 0x48 || s1[0x0b] != 0x10 {
		t.Errorf("bin1: HEIGHT reg 0x0a/0x0b = 0x%02x%02x, want 0x1048 (4168)", s1[0x0b], s1[0x0a])
	}
	if s1[0x1dd] != (0x68&0xfc) || s1[0x1de] != 0x18 {
		t.Errorf("bin1: WIDTH reg 0x1dd/0x1de = 0x%02x%02x, want 0x1868&0xfc (6248)", s1[0x1de], s1[0x1dd])
	}
	if s1[0x0001] != 0x00 || s1[0x1d8] != 0x04 {
		t.Errorf("bin1: table 0x0001=0x%02x (want 0x00) winmode 0x1d8=0x%02x (want 0x04)", s1[0x0001], s1[0x1d8])
	}

	// bin 2: output 3112×2084 → bin-2 table (0x0001=0x05), window mode 0, FPGA width=3112.
	f2 := &fakeRegmap{}
	if err := IMX571.SetROI(f2, 0, 0, 3112, 2084, 2); err != nil {
		t.Fatal(err)
	}
	s2, fp2 := lastVals(f2.writes), lastVals(f2.fpgaWrites)
	if s2[0x0001] != 0x05 {
		t.Errorf("bin2: reg 0x0001 = 0x%02x, want 0x05 (bin-2 table)", s2[0x0001])
	}
	if s2[0x1d8] != 0x00 {
		t.Errorf("bin2: window mode 0x1d8 = 0x%02x, want 0x00 (binned)", s2[0x1d8])
	}
	// HEIGHT gets +2 when binned: 2084+2 = 2086 = 0x0826.
	if s2[0x0a] != 0x26 || s2[0x0b] != 0x08 {
		t.Errorf("bin2: HEIGHT 0x0a/0x0b = 0x%02x%02x, want 0x0826 (2086 = output+2)", s2[0x0b], s2[0x0a])
	}
	// FPGA width 0x04/0x05 = output width 3112 = 0x0c28.
	if fp2[0x04] != 0x28 || fp2[0x05] != 0x0c {
		t.Errorf("bin2: FPGA width = 0x%02x%02x, want 0x0c28 (3112)", fp2[0x05], fp2[0x04])
	}
}

// regsOf571 runs IMX571.SetGain and returns the last value written per register.
func regsOf571(t *testing.T, gain int) map[uint16]uint16 {
	t.Helper()
	f := &fakeRegmap{}
	if err := IMX571.SetGain(f, gain); err != nil {
		t.Fatal(err)
	}
	m := map[uint16]uint16{}
	for _, w := range f.writes {
		m[w.Reg] = w.Val
	}
	return m
}

// TestIMX571GainHCG locks the re-traced IMX571 gain map (SetGain).
// The IMX571 HCG switch is the clean reg 0x2f = 0 (LCG, gain < 100) / 1 (HCG, gain ≥ 100),
// the code resets at that boundary, gain clamps DOWN to −25 (0x67f = 0x11 in that segment),
// and the top band (gain > 460) carries a byte-wrapped coarse stage in reg 0x40 bits[4:7].
func TestIMX571GainHCG(t *testing.T) {
	// gain 0: LCG, conv 0x2f = 0, code 0, setup 0x67f = 0, no high stage.
	g0 := regsOf571(t, 0)
	if g0[0x2f] != 0 {
		t.Errorf("gain 0: 0x2f = 0x%02x, want 0 (LCG)", g0[0x2f])
	}
	for _, reg := range []uint16{0x30, 0x31, 0x32, 0x33, 0x40, 0x67f} {
		if g0[reg] != 0 {
			t.Errorf("gain 0: reg 0x%02x = 0x%02x, want 0", reg, g0[reg])
		}
	}

	// gain -25 (the low clamp): setup 0x67f = 0x11, conv 0x2f = 0, code rebased to gain+25 = 0.
	gNeg := regsOf571(t, -25)
	if gNeg[0x67f] != 0x11 {
		t.Errorf("gain -25: 0x67f = 0x%02x, want 0x11 (negative segment)", gNeg[0x67f])
	}
	if gNeg[0x2f] != 0 || gNeg[0x30] != 0 || gNeg[0x31] != 0 {
		t.Errorf("gain -25: 0x2f=0x%02x code=0x%02x%02x, want conv 0 / code 0", gNeg[0x2f], gNeg[0x31], gNeg[0x30])
	}

	// gain 100: HCG engages — 0x2f = 1 and the code resets to 0.
	g100 := regsOf571(t, 100)
	if g100[0x2f] != 1 {
		t.Errorf("gain 100: 0x2f = 0x%02x, want 1 (HCG)", g100[0x2f])
	}
	for _, reg := range []uint16{0x30, 0x31, 0x32, 0x33} {
		if g100[reg] != 0 {
			t.Errorf("gain 100: reg 0x%02x = 0x%02x, want 0 (code resets at HCG)", reg, g100[reg])
		}
	}

	// gain 160: HCG, code restarts the ramp → exp10(60) = 4095·(1−10^-0.3) = 2042 = 0x07fa.
	g160 := regsOf571(t, 160)
	if g160[0x30] != 0xfa || g160[0x31] != 0x07 {
		t.Errorf("gain 160: code = 0x%02x%02x, want 0x07fa", g160[0x31], g160[0x30])
	}

	// gain 461 (top band): the byte-wrapped stage is 1 → 0x40 = 0x10, NOT the over-counted
	// stage-9/0x90 the missing &0xff produced. conv 0x2f stays 1.
	g461 := regsOf571(t, 461)
	if g461[0x40] != 0x10 {
		t.Errorf("gain 461: 0x40 = 0x%02x, want 0x10 (stage 1, byte-wrapped)", g461[0x40])
	}
	if g461[0x2f] != 1 {
		t.Errorf("gain 461: 0x2f = 0x%02x, want 1", g461[0x2f])
	}
	// And the re-based code must be a sane in-range value (the bug drove it negative/garbage).
	code461 := g461[0x30] | g461[0x31]<<8
	if code461 == 0 || code461 > 4095 {
		t.Errorf("gain 461: code = 0x%04x, want a sane 1..4095 (bug drove it out of range)", code461)
	}
}

// TestProfileRanges confirms each validated profile carries its gain/exposure
// bounds, which Camera.GainRange/ExposureRange surface for the Alpaca caps. The values are
// the SetGain/SetExp clamps (imx455GainMax etc.), not invented.
func TestProfileRanges(t *testing.T) {
	for _, tc := range []struct {
		s       *Sensor
		gainMax int
	}{
		{&IMX455, 700}, {&IMX174, 400}, {&IMX290, 600},
	} {
		if tc.s.GainMax != tc.gainMax {
			t.Errorf("%s GainMax = %d, want %d", tc.s.Name, tc.s.GainMax, tc.gainMax)
		}
		if tc.s.ExpMinUs != 32 {
			t.Errorf("%s ExpMinUs = %d, want 32", tc.s.Name, tc.s.ExpMinUs)
		}
		if tc.s.ExpMaxUs != 2_000_000_000 {
			t.Errorf("%s ExpMaxUs = %d, want 2e9", tc.s.Name, tc.s.ExpMaxUs)
		}
	}
}

// TestRegistered confirms init() wired representative PIDs to the right sensor.
// 0x260A is the ASI2600 block (IMX571); 0x620A is ASI6200 (IMX455), which must
// NOT resolve to IMX571 — that pairing was a prior bug.
func TestRegistered(t *testing.T) {
	for pid, sensor := range map[uint16]string{
		0x1749: "IMX174", // ASI174
		0x260A: "IMX571", // ASI2600
		0x290F: "IMX290", // ASI290MM Mini
		0x620A: "IMX455", // ASI6200
		0x462b: "IMX462", // ASI462MC (PID hardware-confirmed: ioreg idProduct 17963)
	} {
		m, ok := Lookup(ZWO.VID, pid)
		if !ok || m.Sensor == nil || m.Sensor.Name != sensor {
			t.Errorf("PID 0x%04x: got %v / %v, want sensor %s", pid, ok, m.Sensor, sensor)
		}
	}
}

// TestVendorOffsetDispatch verifies the single shared IMX455/IMX571 profile selects the
// vendor offset encoding from the regmap's VID: PlayerOne (0xA0A0) → offset·8, ZWO (0x03C3)
// → offset·10, same [lo,hi,lo,hi] mirror block. An unrecognized vendor is an error (the
// dispatch is vendor-agnostic — no implicit default).
func TestVendorOffsetDispatch(t *testing.T) {
	eq := func(t *testing.T, tag string, got, want []RegVal) {
		t.Helper()
		if len(got) != len(want) {
			t.Fatalf("%s: got %d writes, want %d: %+v", tag, len(got), len(want), got)
		}
		for i := range want {
			if got[i] != want[i] {
				t.Errorf("%s write %d = %+v, want %+v", tag, i, got[i], want[i])
			}
		}
	}
	for _, tc := range []struct {
		name string
		s    Sensor
		base uint16
	}{
		{"imx455", IMX455, 0x40},
		{"imx571", IMX571, 0x42},
	} {
		fp := &fakeRegmap{vid: POA.VID}
		if err := tc.s.SetOffset(fp, 100); err != nil { // 100·8 = 800 = 0x0320
			t.Fatalf("%s POA: %v", tc.name, err)
		}
		eq(t, tc.name+" POA", fp.writes, []RegVal{
			{Reg: tc.base, Val: 0x20}, {Reg: tc.base + 1, Val: 0x03},
			{Reg: tc.base + 2, Val: 0x20}, {Reg: tc.base + 3, Val: 0x03},
		})

		fz := &fakeRegmap{vid: ZWO.VID}
		if err := tc.s.SetOffset(fz, 100); err != nil { // 100·10 = 1000 = 0x03E8
			t.Fatalf("%s ZWO: %v", tc.name, err)
		}
		eq(t, tc.name+" ZWO", fz.writes, []RegVal{
			{Reg: tc.base, Val: 0xE8}, {Reg: tc.base + 1, Val: 0x03},
			{Reg: tc.base + 2, Val: 0xE8}, {Reg: tc.base + 3, Val: 0x03},
		})

		if err := tc.s.SetOffset(&fakeRegmap{vid: 0x9999}, 100); err == nil {
			t.Errorf("%s: unknown VID should error, not default to a vendor", tc.name)
		}
	}
}

// TestVendorGainDispatch confirms gain dispatch is wired and vendor-agnostic: ZWO drives its
// encoding, an unknown vendor errors, and the PlayerOne arm works for both dies.
func TestVendorGainDispatch(t *testing.T) {
	for _, s := range []Sensor{IMX455, IMX571} {
		if err := s.SetGain(&fakeRegmap{vid: ZWO.VID}, 100); err != nil {
			t.Errorf("%s: ZWO gain should work: %v", s.Name, err)
		}
		if err := s.SetGain(&fakeRegmap{vid: 0x9999}, 100); err == nil {
			t.Errorf("%s: unknown VID gain should error", s.Name)
		}
	}
	for _, s := range []Sensor{IMX455, IMX571} {
		if err := s.SetGain(&fakeRegmap{vid: POA.VID}, 100); err != nil {
			t.Errorf("%s: PlayerOne gain is decoded and should work: %v", s.Name, err)
		}
	}
}

// TestVendorCaps locks the advertised gain/offset ranges per vendor (the dual of the
// dispatched SetGain/SetOffset). PlayerOne uses a unified 0..550 gain / 0..2000 offset scale
// across both dies (CamAttributesInit + the SetGain/SetOffset clamp).
func TestVendorCaps(t *testing.T) {
	for _, tc := range []struct {
		s          Sensor
		vid        uint16
		gmin, gmax int
	}{
		{IMX455, ZWO.VID, 0, 700},
		{IMX455, POA.VID, 0, 550},
		{IMX571, ZWO.VID, -25, 700},
		{IMX571, POA.VID, 0, 550},
	} {
		if mn, mx := tc.s.GainCaps(tc.vid); mn != tc.gmin || mx != tc.gmax {
			t.Errorf("%s GainCaps(%#x) = %d..%d, want %d..%d", tc.s.Name, tc.vid, mn, mx, tc.gmin, tc.gmax)
		}
	}
	for _, tc := range []struct {
		s                Sensor
		vid              uint16
		omin, omax, odef int
	}{
		{IMX455, ZWO.VID, 0, 200, 50},
		{IMX455, POA.VID, 0, 2000, 20},
		{IMX571, ZWO.VID, 0, 240, 1},
		{IMX571, POA.VID, 0, 2000, 20},
	} {
		if mn, mx, df := tc.s.OffsetCaps(tc.vid); mn != tc.omin || mx != tc.omax || df != tc.odef {
			t.Errorf("%s OffsetCaps(%#x) = %d..%d def %d, want %d..%d def %d", tc.s.Name, tc.vid, mn, mx, df, tc.omin, tc.omax, tc.odef)
		}
	}
}

// TestIMX571GainPOA locks PlayerOne's IMX571 gain bands (CamGainSet, M=125):
// the per-band 0x2f conv-gain mode and 0x67f setup, the analog-code mirror (0x30/0x31 →
// 0x32/0x33), and the code reset at each conversion-gain boundary. The 571 has no 0x3a4/5/6.
func TestIMX571GainPOA(t *testing.T) {
	for _, c := range []struct {
		gain        int
		conv, setup uint16
	}{
		{0, 0, 0x22}, // band A
		{4, 0, 0x22},
		{5, 0, 0x11}, // band B
		{29, 0, 0x11},
		{30, 0, 0x00}, // band C
		{124, 0, 0x00},
		{125, 1, 0x00},    // band D, conv=1
		{229, 1, 0x00},    // gain-M = 104, not > 104
		{230, 0x11, 0x00}, // gain-M = 105 > 104
	} {
		f := &fakeRegmap{vid: POA.VID}
		if err := IMX571.SetGain(f, c.gain); err != nil {
			t.Fatalf("gain %d: %v", c.gain, err)
		}
		v := lastVals(f.writes)
		if v[0x2f] != c.conv || v[0x67f] != c.setup {
			t.Errorf("gain %d: 0x2f=%#x 0x67f=%#x; want %#x/%#x", c.gain, v[0x2f], v[0x67f], c.conv, c.setup)
		}
		if v[0x30] != v[0x32] || v[0x31] != v[0x33] { // analog code mirrored
			t.Errorf("gain %d: code not mirrored: 0x30=%#x 0x32=%#x 0x31=%#x 0x33=%#x", c.gain, v[0x30], v[0x32], v[0x31], v[0x33])
		}
		if _, wrote3a4 := v[0x3a4]; wrote3a4 {
			t.Errorf("gain %d: IMX571 must not write the 455-only 0x3a4 config", c.gain)
		}
	}
	for _, gain := range []int{5, 30, 125} { // boundaries rebase to g==0
		f := &fakeRegmap{vid: POA.VID}
		if err := IMX571.SetGain(f, gain); err != nil {
			t.Fatal(err)
		}
		if v := lastVals(f.writes); v[0x30] != 0 || v[0x31] != 0 {
			t.Errorf("gain %d: code should reset to 0, got 0x30=%#x 0x31=%#x", gain, v[0x30], v[0x31])
		}
	}
}

// TestIMX455GainPOA locks PlayerOne's IMX455 gain bands (CamGainSet, M=125):
// the per-band 0x2d conv-gain mode, 0x67f setup, and 0x3a4/5/6 config, plus the analog-code
// mirror (0x2e/0x2f → 0x30/0x31) and its reset to 0 at each conversion-gain boundary.
func TestIMX455GainPOA(t *testing.T) {
	for _, c := range []struct {
		gain                    int
		mode, setup, cfgA, cfgB uint16
	}{
		{0, 0, 0x22, 0x11, 0x11}, // band A
		{4, 0, 0x22, 0x11, 0x11},
		{5, 0, 0x11, 0x11, 0x11}, // band B
		{29, 0, 0x11, 0x11, 0x11},
		{30, 0, 0x00, 0x11, 0x11}, // band C, 0x2d=0
		{89, 0, 0x00, 0x11, 0x11},
		{90, 4, 0x00, 0x11, 0x11}, // band C, 0x2d=4
		{124, 4, 0x00, 0x11, 0x11},
		{125, 1, 0x00, 0x11, 0x11}, // band D, 0x2d=1
		{184, 1, 0x00, 0x11, 0x11},
		{185, 5, 0x00, 0x11, 0x11}, // band D, 0x2d=5
		{304, 5, 0x00, 0x11, 0x11},
		{305, 5, 0x00, 0x23, 0x2d}, // band D, high config
	} {
		f := &fakeRegmap{vid: POA.VID}
		if err := IMX455.SetGain(f, c.gain); err != nil {
			t.Fatalf("gain %d: %v", c.gain, err)
		}
		v := lastVals(f.writes)
		if v[0x2d] != c.mode || v[0x67f] != c.setup || v[0x3a4] != c.cfgA || v[0x3a5] != c.cfgB || v[0x3a6] != c.cfgB {
			t.Errorf("gain %d: 0x2d=%#x 0x67f=%#x 0x3a4=%#x 0x3a5=%#x 0x3a6=%#x; want %#x/%#x/%#x/%#x/%#x",
				c.gain, v[0x2d], v[0x67f], v[0x3a4], v[0x3a5], v[0x3a6], c.mode, c.setup, c.cfgA, c.cfgB, c.cfgB)
		}
		if v[0x2e] != v[0x30] || v[0x2f] != v[0x31] { // analog code mirrored
			t.Errorf("gain %d: code not mirrored: 0x2e=%#x 0x30=%#x 0x2f=%#x 0x31=%#x", c.gain, v[0x2e], v[0x30], v[0x2f], v[0x31])
		}
	}
	// conversion-gain boundaries rebase to g==0, resetting the analog code to 0.
	for _, gain := range []int{5, 30, 125} {
		f := &fakeRegmap{vid: POA.VID}
		if err := IMX455.SetGain(f, gain); err != nil {
			t.Fatal(err)
		}
		v := lastVals(f.writes)
		if v[0x2e] != 0 || v[0x2f] != 0 {
			t.Errorf("gain %d: code should reset to 0, got 0x2e=%#x 0x2f=%#x", gain, v[0x2e], v[0x2f])
		}
	}
}

// TestIMX462Gain locks the 462's STARVIS gain map (SetGain): the conv-gain
// HCG bit (0x3009 bit 0x10) engages ABOVE gain 80 — the 462's threshold, vs the 290's 60 — and the
// analog code lands in 0x3014, the group latched by 0x3001.
func TestIMX462Gain(t *testing.T) {
	f := &fakeRegmap{} // gain 80 → LCG (HCG bit clear)
	if err := IMX462.SetGain(f, 80); err != nil {
		t.Fatal(err)
	}
	if v := lastVals(f.writes); v[0x3009]&0x10 != 0 {
		t.Errorf("gain 80: 0x3009=%#x, want HCG bit 0x10 clear (LCG)", v[0x3009])
	}
	f = &fakeRegmap{} // gain 81 → HCG (bit set)
	if err := IMX462.SetGain(f, 81); err != nil {
		t.Fatal(err)
	}
	v := lastVals(f.writes)
	if v[0x3009]&0x10 == 0 {
		t.Errorf("gain 81: 0x3009=%#x, want HCG bit 0x10 set", v[0x3009])
	}
	if _, ok := v[0x3014]; !ok {
		t.Error("gain: analog code 0x3014 not written")
	}
}

// TestIMX178Gain locks the 178's gain map (SetGain): conv-gain 0x301b
// (0 LCG / 0x1e above gain 30) and the RAW 0.1 dB value written 16-bit LE to 0x301f/0x3020.
func TestIMX178Gain(t *testing.T) {
	f := &fakeRegmap{} // gain 30 → LCG, code 30
	if err := IMX178.SetGain(f, 30); err != nil {
		t.Fatal(err)
	}
	v := lastVals(f.writes)
	if v[0x301b] != 0 || v[0x301f] != 30 || v[0x3020] != 0 {
		t.Errorf("gain 30: 0x301b=%#x code=%#x/%#x, want 0 / 30 / 0", v[0x301b], v[0x301f], v[0x3020])
	}
	f = &fakeRegmap{} // gain 300=0x12c → HCG, code lo 0x2c hi 0x01
	if err := IMX178.SetGain(f, 300); err != nil {
		t.Fatal(err)
	}
	v = lastVals(f.writes)
	if v[0x301b] != 0x1e || v[0x301f] != 0x2c || v[0x3020] != 0x01 {
		t.Errorf("gain 300: 0x301b=%#x code=%#x/%#x, want 0x1e / 0x2c / 0x01", v[0x301b], v[0x301f], v[0x3020])
	}
}

// modeRegmap is a fakeRegmap that carries a live ReadoutMode (so profiles see HighSpeed /
// geometry) and serves canned read-back values (so read-modify-write paths are steerable).
type modeRegmap struct {
	fakeRegmap
	mode    ReadoutMode
	regVals map[uint16]uint16
}

func (m *modeRegmap) ReadoutMode() ReadoutMode { return m.mode }
func (m *modeRegmap) ReadReg(reg uint16) (uint16, error) {
	return m.regVals[reg], nil
}

// clockSelCase runs one profile op and asserts the FINAL 0x3009 value — the clock/FRSEL
// select the tail SetCMOSClk write must leave: FRSEL 0x01 for the 12-bit
// normal clock, 0x00 for 10-bit high-speed, preserving the conversion-gain bit 0x10.
func clockSelCase(t *testing.T, name string, highSpeed bool, hcgIn uint16, want uint16, op func(rm Regmap) error) {
	t.Helper()
	rm := &modeRegmap{mode: ReadoutMode{HighSpeed: highSpeed, BytesPerPx: 2}, regVals: map[uint16]uint16{0x3009: hcgIn}}
	if err := op(rm); err != nil {
		t.Fatalf("%s: %v", name, err)
	}
	if got := lastVals(rm.writes)[0x3009]; got != want {
		t.Errorf("%s: final 0x3009 = 0x%02x, want 0x%02x", name, got, want)
	}
}

// TestIMX462ClockSelect: SetExposure and SetROI end with the object's SetCMOSClk write —
// normal restores FRSEL 1 (closing the one-way high-speed clear), high-speed runs FRSEL 0,
// and the HCG bit rides along untouched.
func TestIMX462ClockSelect(t *testing.T) {
	exp := func(rm Regmap) error { return imx462SetExposure(rm, 10*time.Millisecond) }
	roi := func(rm Regmap) error { return imx462SetROI(rm, 0, 0, 1936, 1096, 1) }
	clockSelCase(t, "SetExposure normal", false, 0x00, 0x01, exp)
	clockSelCase(t, "SetExposure normal+HCG", false, 0x10, 0x11, exp)
	clockSelCase(t, "SetExposure highspeed+HCG", true, 0x10, 0x10, exp)
	clockSelCase(t, "SetROI normal", false, 0x00, 0x01, roi)
	clockSelCase(t, "SetROI highspeed", true, 0x00, 0x00, roi)
}

// TestIMX290ClockSelect: same contract from the 290's own object (0003_CCameraS290MM_Mini.o).
func TestIMX290ClockSelect(t *testing.T) {
	exp := func(rm Regmap) error { return imx290SetExposure(rm, 10*time.Millisecond) }
	roi := func(rm Regmap) error { return imx290SetROI(rm, 0, 0, 1936, 1096, 1) }
	clockSelCase(t, "SetExposure normal", false, 0x00, 0x01, exp)
	clockSelCase(t, "SetExposure normal+HCG", false, 0x10, 0x11, exp)
	clockSelCase(t, "SetExposure highspeed+HCG", true, 0x10, 0x10, exp)
	clockSelCase(t, "SetROI normal", false, 0x00, 0x01, roi)
	clockSelCase(t, "SetROI highspeed", true, 0x00, 0x00, roi)
}

// TestIMX455ADCBitFollowsTable locks SetOutput16Bits: FPGA reg 0xa bit0 (ADC_BIT) is 1 for the
// 16-bit readout table and 0 for the 12-bit tables (high-speed at bin 1, hardware bin 2/4), with
// bit4 = RAW16. Wire-checked on the ASI6200MC: ADC_BIT left at 1 on a 12-bit table delivers
// unreadable frames; the SDK's high-speed RAW8 clears it.
func TestIMX455ADCBitFollowsTable(t *testing.T) {
	cases := []struct {
		name  string
		hs    bool
		bpp   int
		bin   int
		w, h  int
		want  uint16 // reg 0xa bits {4,0}
		table uint16 // reg 0x0001 of the selected table
	}{
		{"bin1 RAW16", false, 2, 1, 9576, 6388, 0x11, 0x00},
		{"bin1 RAW8", false, 1, 1, 9576, 6388, 0x01, 0x00},
		{"bin1 RAW8 high-speed", true, 1, 1, 9576, 6388, 0x00, 0x80},
		{"bin2 RAW8", false, 1, 2, 4788, 3194, 0x00, 0x85},
	}
	for _, c := range cases {
		rm := &modeRegmap{mode: ReadoutMode{HighSpeed: c.hs, BytesPerPx: c.bpp, Bin: c.bin}, regVals: map[uint16]uint16{}}
		if err := IMX455.SetROI(rm, 0, 0, c.w, c.h, c.bin); err != nil {
			t.Fatalf("%s: %v", c.name, err)
		}
		if got := lastVals(rm.fpgaWrites)[0x0a] & 0x11; got != c.want {
			t.Errorf("%s: FPGA reg 0xa bits{4,0} = 0x%02x, want 0x%02x", c.name, got, c.want)
		}
		if got := lastVals(rm.writes)[0x0001]; got != c.table {
			t.Errorf("%s: mode table reg 0x0001 = 0x%02x, want 0x%02x", c.name, got, c.table)
		}
	}
}

// TestGetOffsetRoundTrip: every profile's GetOffset reads back what SetOffset programmed, in the
// same units, for the ZWO scale (and PlayerOne on the shared dies).
func TestGetOffsetRoundTrip(t *testing.T) {
	for _, tc := range []struct {
		s   Sensor
		vid uint16
	}{
		{IMX174, ZWO.VID}, {IMX178, ZWO.VID}, {IMX290, ZWO.VID}, {IMX462, ZWO.VID}, {IMX585, ZWO.VID},
		{IMX455, ZWO.VID}, {IMX455, POA.VID}, {IMX571, ZWO.VID}, {IMX571, POA.VID},
	} {
		for _, off := range []int{0, 1, 30, 50, 100} {
			rm := &modeRegmap{fakeRegmap: fakeRegmap{vid: tc.vid}, mode: ReadoutMode{BytesPerPx: 2, Bin: 1}, regVals: map[uint16]uint16{}}
			if err := tc.s.SetOffset(rm, off); err != nil {
				t.Fatalf("%s/%#x SetOffset(%d): %v", tc.s.Name, tc.vid, off, err)
			}
			for r, v := range lastVals(rm.writes) { // the sensor now holds what was written
				rm.regVals[r] = v
			}
			got, err := tc.s.GetOffset(rm)
			if err != nil {
				t.Fatalf("%s/%#x GetOffset: %v", tc.s.Name, tc.vid, err)
			}
			if got != off {
				t.Errorf("%s/%#x: SetOffset(%d) reads back %d", tc.s.Name, tc.vid, off, got)
			}
		}
	}
}
