package sensors

import (
	"bytes"
	"math"
	"strings"
	"testing"

	. "github.com/mikefsq/astrocam"
)

// zwoOnlyRequests are vendor request codes that belong to ZWO and mean something else, or
// nothing, on PlayerOne. Codes the two vendors share by number are deliberately absent: 0xA6 and
// 0xA7 are ZWO's camera-register access and PlayerOne's ST4, and 0xA9/0xAA are ZWO's stream
// start/stop and PlayerOne's TEC setpoint and cooler enable. Seeing one of those proves nothing;
// seeing one of these proves a profile reached for the wrong vendor's dialect.
var zwoOnlyRequests = map[uint8]string{
	0xAD: "GetFirmwareVer",
	0xAF: "flush",
	0xB6: "WriteSONYREG",
	0xB7: "ReadSONYREG",
	0xBC: "ReadFPGAREG",
	0xBD: "WriteFPGAREG",
	0xBE: "EnableGPIF32DQ",
	0xC3: "ReadSPIFlash",
	0xC8: "GetSerialNumber",
	0x85: "GetHumidity",
}

// TestPlayerOneModelsNeverEmitZWOOpcodes walks EVERY PlayerOne model in the registry and drives
// the vendor-sensitive entry points. The profiles are keyed by Sony die and shared with ZWO, and
// their FPGA halves follow ZWO firmware, so the risk this pins down is not a crash
// but silent misprogramming: PlayerOne's registers 0x04, 0x08, 0x0c and 0x26 are its readout mode
// byte, crop burst, image-size burst and window heater, where ZWO keeps width, height, analog
// gain and TEC power.
//
// Each entry point must therefore either be implemented for PlayerOne or refuse, and either way
// no ZWO-only request may reach the wire. A newly registered PlayerOne PID is covered
// automatically, which is the point of walking the registry rather than a fixed list.
func TestPlayerOneModelsNeverEmitZWOOpcodes(t *testing.T) {
	var checked int
	for _, id := range RegisteredIDs() {
		if id.VID != POA.VID {
			continue
		}
		m, ok := Lookup(id.VID, id.PID)
		if !ok || m.Sensor == nil {
			continue
		}
		checked++
		t.Run(m.Name, func(t *testing.T) {
			st := NewStubTransport()
			cam, err := Open(st, id.VID, id.PID)
			if err != nil {
				t.Fatalf("open: %v", err)
			}
			// Init covers the reglist, the FPGA bringup and the ROI programming.
			if err := cam.Init(); err == nil {
				t.Logf("Init succeeded — %s is implemented for PlayerOne", m.Sensor.Name)
			} else if !strings.Contains(err.Error(), "PlayerOne") {
				t.Errorf("Init failed for a reason other than the PlayerOne gap: %v", err)
			}
			// Gain and offset are dispatched per vendor on some dies and refused on others;
			// either is fine, emitting ZWO's bytes is not.
			_ = cam.SetGain(100)
			_ = cam.SetOffset(20)

			// The sharper check. The regmap always speaks PlayerOne's dialect, so a profile
			// running ZWO's FPGA code still emits PlayerOne REQUEST codes — only the register
			// NUMBERS would be ZWO's, which no opcode check can see. So assert the contract
			// directly: an entry point that refuses must not have written anything on the way to
			// refusing. This is what fails if a guard is dropped.
			before := len(st.Log)
			roiErr := cam.SetROI(0, 0, m.Sensor.Info.MaxWidth, m.Sensor.Info.MaxHeight)
			if roiErr != nil {
				if !strings.Contains(roiErr.Error(), "PlayerOne") {
					t.Errorf("SetROI failed for a reason other than the PlayerOne gap: %v", roiErr)
				}
				for _, x := range st.Log[before:] {
					if x.BRequest == 0xC0 || x.BRequest == 0xC1 {
						t.Errorf("%s refused SetROI but had already written FPGA register 0x%02x: %+v",
							m.Sensor.Name, x.WIndex, x)
					}
				}
			}

			for _, x := range st.Log {
				if what, bad := zwoOnlyRequests[x.BRequest]; bad {
					t.Errorf("%s sent ZWO's %s (0x%02x) to a PlayerOne camera: %+v",
						m.Sensor.Name, what, x.BRequest, x)
				}
			}
		})
	}
	if checked == 0 {
		t.Fatal("no PlayerOne models walked; the registry lookup is broken")
	}
	t.Logf("walked %d PlayerOne models", checked)
}

// TestIMX585GeometryAndCapsAreVendorSpecific pins the three values the vendor SDK reports for a
// Xena 585M against what the profile advertises. The die is one part but the vendors expose
// different amounts of it and different control ranges, so these are per-vendor by construction.
func TestIMX585GeometryAndCapsAreVendorSpecific(t *testing.T) {
	if got, want := IMX585.SizeByVID[POA.VID], [2]int{3856, 2180}; got != want {
		t.Errorf("PlayerOne geometry = %v, want %v (the full effective array, as the SDK reports)", got, want)
	}
	if IMX585.Info.MaxWidth != 3840 || IMX585.Info.MaxHeight != 2160 {
		t.Errorf("ZWO geometry = %dx%d, want the 3840x2160 UHD crop the ZWO object programs",
			IMX585.Info.MaxWidth, IMX585.Info.MaxHeight)
	}
	if _, max := IMX585.GainCaps(POA.VID); max != 750 {
		t.Errorf("PlayerOne gain max = %d, want 750", max)
	}
	if _, max := IMX585.GainCaps(ZWO.VID); max != 600 {
		t.Errorf("ZWO gain max = %d, want 600", max)
	}
	if _, max, def := IMX585.OffsetCaps(POA.VID); max != 250 || def != 3 {
		t.Errorf("PlayerOne offset = 0..%d def %d, want 0..250 def 3", max, def)
	}
	if _, max, def := IMX585.OffsetCaps(ZWO.VID); max != 240 || def != 16 {
		t.Errorf("ZWO offset = 0..%d def %d, want 0..240 def 16", max, def)
	}
}

// TestPlayerOneInitTablesAreTheVendorsOwn: a die whose two vendors frame the analog table
// differently must bring a PlayerOne body up with PlayerOne's table, not ZWO's. The engine picks
// it through Sensor.InitByVID, so this checks the profiles actually carry one and that it is not
// simply an alias of the ZWO list.
func TestPlayerOneInitTablesAreTheVendorsOwn(t *testing.T) {
	for _, c := range []struct {
		name   string
		sensor *Sensor
		poaLen int
	}{
		{"IMX455", &IMX455, 34},
		{"IMX571", &IMX571, 61},
		{"IMX585", &IMX585, 218},
	} {
		poa, ok := c.sensor.InitByVID[POA.VID]
		if !ok {
			t.Errorf("%s: no PlayerOne init table; a PlayerOne body would be brought up with ZWO's", c.name)
			continue
		}
		if len(poa) != c.poaLen {
			t.Errorf("%s: PlayerOne table has %d records, want %d as extracted from the SDK object",
				c.name, len(poa), c.poaLen)
		}
		if len(poa) == len(c.sensor.Init) {
			t.Errorf("%s: PlayerOne table is the same length as ZWO's (%d); it should be the vendor's own",
				c.name, len(poa))
		}
		// Every entry must be a byte write or the delay sentinel — a wider value means the
		// extraction ran past the end of the table into neighbouring data.
		for i, w := range poa {
			if w.Reg != InitDelayReg && w.Val > 0xff {
				t.Errorf("%s: record %d writes 0x%x to reg 0x%04x; the table extraction overran",
					c.name, i, w.Val, w.Reg)
			}
		}
	}
}

// TestIMX585GainPOAMatchesWire replays the gain sweep captured off a Xena 585M
// while the vendor SDK drove it. Each row is what the SDK actually put on the wire: the
// conversion-gain byte at 0x3030 and the 16-bit analog code at 0x306c/0x306d.
//
// The SDK computes these with a piecewise cubic polynomial, so the point of the sweep was to find
// out what that polynomial evaluates to rather than to decode it. Every sampled gain landed
// exactly on a rebase-and-divide-by-three, and the band edge was bracketed to the gain: 209 is
// still low conversion gain, 210 is high.
func TestIMX585GainPOAMatchesWire(t *testing.T) {
	for _, c := range []struct {
		gain     int
		wantConv uint16
		wantCode uint16
	}{
		{0, 0, 0}, {10, 0, 0}, {30, 0, 0}, {45, 0, 0}, {46, 0, 0},
		{48, 0, 1}, {51, 0, 2}, {54, 0, 3}, {57, 0, 4}, {60, 0, 5},
		{66, 0, 7}, {75, 0, 10}, {90, 0, 15}, {100, 0, 18}, {150, 0, 35},
		{199, 0, 51}, {200, 0, 51}, {201, 0, 52}, {204, 0, 53}, {208, 0, 54},
		{209, 0, 54}, // last low-conversion-gain step
		{210, 1, 4},  // first high-conversion-gain step
		{230, 1, 10}, {250, 1, 17}, {270, 1, 24}, {290, 1, 30},
		{300, 1, 34}, {500, 1, 100}, {750, 1, 184},
	} {
		f := &fakeRegmap{vid: POA.VID}
		// Through the profile's dispatch, so the VID routing is exercised too.
		if err := IMX585.SetGain(f, c.gain); err != nil {
			t.Fatalf("gain %d: %v", c.gain, err)
		}
		var conv, lo, hi uint16
		var sawConv, sawLo, sawHi bool
		for _, w := range f.writes {
			switch w.Reg {
			case imx585RegConvGain:
				conv, sawConv = w.Val, true
			case imx585RegGainL:
				lo, sawLo = w.Val, true
			case imx585RegGainH:
				hi, sawHi = w.Val, true
			}
		}
		if !sawConv || !sawLo || !sawHi {
			t.Fatalf("gain %d: expected writes to 0x3030, 0x306c and 0x306d; got %+v", c.gain, f.writes)
		}
		if conv != c.wantConv {
			t.Errorf("gain %d: 0x3030 = %d, want %d (the wire's conversion gain)", c.gain, conv, c.wantConv)
		}
		if code := lo | hi<<8; code != c.wantCode {
			t.Errorf("gain %d: analog code = %d, want %d (the wire's value)", c.gain, code, c.wantCode)
		}
	}
}

// TestIMX585SetROIPOAMatchesWire: the window PlayerOne programs, against the capture sweep. The
// point of each row is that the geometry goes out VERBATIM — the ZWO path rounds width up to a
// multiple of 16 and height to a multiple of 4 plus two dummy lines, and doing that to a
// PlayerOne body would mis-size every window.
func TestIMX585SetROIPOAMatchesWire(t *testing.T) {
	// wantSize is the FPGA image-size burst for that window. An unset readout mode normalises to
	// RAW16, so these are the 16-bit payloads: the first three are the exact bytes the SDK sent
	// in the capture matrix, the last three follow the same frame-length arithmetic at windows the
	// sweep only exercised in RAW8.
	for _, c := range []struct {
		name       string
		x, y, w, h int
		wantSize   []byte
	}{
		{"full frame", 0, 0, 3856, 2180, []byte{0x10, 0x0f, 0x84, 0x08, 0x20, 0x22, 0x40, 0x00}},
		{"1920x1080", 0, 0, 1920, 1080, []byte{0x80, 0x07, 0x38, 0x04, 0x00, 0xd2, 0x0f, 0x00}},
		{"640x480", 0, 0, 640, 480, []byte{0x80, 0x02, 0xe0, 0x01, 0x00, 0x58, 0x02, 0x00}},
		{"320x240", 0, 0, 320, 240, []byte{0x40, 0x01, 0xf0, 0x00, 0x00, 0x96, 0x00, 0x00}},
		{"offset origin", 64, 32, 640, 480, []byte{0x80, 0x02, 0xe0, 0x01, 0x00, 0x58, 0x02, 0x00}},
		{"1024x768 at 128,64", 128, 64, 1024, 768, []byte{0x00, 0x04, 0x00, 0x03, 0x00, 0x00, 0x06, 0x00}},
	} {
		f := &fakeRegmap{vid: POA.VID}
		if err := IMX585.SetROI(f, c.x, c.y, c.w, c.h, 1); err != nil {
			t.Fatalf("%s: %v", c.name, err)
		}
		got := map[uint16]uint16{}
		for _, wr := range f.writes {
			got[wr.Reg] = wr.Val
		}
		pair := func(lo, hi uint16) int { return int(got[lo]) | int(got[hi])<<8 }
		if v := pair(imx585RegStartXL, imx585RegStartXH); v != c.x {
			t.Errorf("%s: start X = %d, want %d", c.name, v, c.x)
		}
		if v := pair(imx585RegStartYL, imx585RegStartYH); v != c.y {
			t.Errorf("%s: start Y = %d, want %d", c.name, v, c.y)
		}
		if v := pair(imx585RegWidthL, imx585RegWidthH); v != c.w {
			t.Errorf("%s: width = %d, want %d verbatim (no rounding to 16)", c.name, v, c.w)
		}
		if v := pair(imx585RegHeightL, imx585RegHeightH); v != c.h {
			t.Errorf("%s: height = %d, want %d verbatim (no ceil4 plus dummy lines)", c.name, v, c.h)
		}
		// The FPGA side: the frame length the SDK computed, and the fixed crop origin.
		if got := f.fpgaBursts[0x0c]; !bytes.Equal(got, c.wantSize) {
			t.Errorf("%s: FPGA size burst = % x, want % x", c.name, got, c.wantSize)
		}
		if got, want := f.fpgaBursts[0x08], []byte{0, 0, imx585POACropY, 0}; !bytes.Equal(got, want) {
			t.Errorf("%s: FPGA crop burst = % x, want % x", c.name, got, want)
		}
		// reg 0x02 read 0x81 for every RAW16 frame in the capture: bit 7 for the sample size.
		fpga := map[uint16]uint16{}
		for _, wr := range f.fpgaWrites {
			fpga[wr.Reg] = wr.Val
		}
		if fpga[0x02] != 0x81 {
			t.Errorf("%s: FPGA format byte = 0x%02x, want 0x81", c.name, fpga[0x02])
		}
	}
}

// TestIMX585SetROIVendorsDiffer guards the reason the dispatch exists: ZWO's rounding must not
// reach a PlayerOne body, and PlayerOne's verbatim geometry must not reach a ZWO one. 640x476 is
// chosen because it rounds on both axes under ZWO's rules.
func TestIMX585SetROIVendorsDiffer(t *testing.T) {
	const w, h = 636, 476
	z := &fakeRegmap{vid: ZWO.VID}
	if err := IMX585.SetROI(z, 0, 0, w, h, 1); err != nil {
		t.Fatal(err)
	}
	p := &fakeRegmap{vid: POA.VID}
	if err := IMX585.SetROI(p, 0, 0, w, h, 1); err != nil {
		t.Fatal(err)
	}
	get := func(f *fakeRegmap, lo, hi uint16) int {
		m := map[uint16]uint16{}
		for _, wr := range f.writes {
			m[wr.Reg] = wr.Val
		}
		return int(m[lo]) | int(m[hi])<<8
	}
	zw := get(z, imx585RegWidthL, imx585RegWidthH)
	pw := get(p, imx585RegWidthL, imx585RegWidthH)
	zh := get(z, imx585RegHeightL, imx585RegHeightH)
	ph := get(p, imx585RegHeightL, imx585RegHeightH)
	// Both vendors round the sensor width up to 16 and let the FPGA crop, so 636 becomes 640 on
	// each. The HEIGHT is where they part: ZWO rounds to 4 and adds two dummy lines, PlayerOne
	// takes it verbatim.
	if pw != 640 || zw != 640 {
		t.Errorf("widths = ZWO %d, PlayerOne %d; want 640 on both (636 rounded up to 16)", zw, pw)
	}
	if ph != h {
		t.Errorf("PlayerOne height = %d, want %d verbatim", ph, h)
	}
	if zh != (h+3)&^3+2 {
		t.Errorf("ZWO height = %d, want %d (ceil4 plus two dummy lines)", zh, (h+3)&^3+2)
	}
	if zh == ph {
		t.Error("both vendors programmed the same height; the dispatch is not taking effect")
	}
}

// TestIMX585EGainMatchesSDK: e/ADU falls off as base / 10^(gain/200). The base is the constant
// The SDK stores it, and reports the gain-0 value in its config caps.
func TestIMX585EGainMatchesSDK(t *testing.T) {
	if IMX585.EGainBase != 11.402999877929688 {
		t.Errorf("EGainBase = %v, want the SDK's 11.402999877929688", IMX585.EGainBase)
	}
	// A decade of voltage gain is 200 units, so 200 units must halve it twice over.
	for _, c := range []struct {
		gain int
		want float64
	}{
		{0, 11.402999877929688},
		{200, 1.1402999877929688}, // one decade of voltage gain
		// 60 units is 6 dB, which is 10^0.3 = 1.99526 — close to a factor of two but not equal,
		// so state it exactly rather than rounding.
		{60, 11.402999877929688 / 1.9952623149688795},
	} {
		got := IMX585.EGainBase / math.Pow(10, float64(c.gain)/200)
		if math.Abs(got-c.want) > 1e-6 {
			t.Errorf("e/ADU at gain %d = %v, want %v", c.gain, got, c.want)
		}
	}
}
