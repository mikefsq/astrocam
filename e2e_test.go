package astrocam_test

import (
	"testing"
	"time"

	"github.com/mikefsq/astrocam"
	_ "github.com/mikefsq/astrocam/sensors" // registers the PID → sensor table
)

// TestE2ECapture runs the full hardware-free capture pipeline against StubTransport
// for each validated camera, asserting the frame round-trips and the control plane
// actually drove the device.
func TestE2ECapture(t *testing.T) {
	for _, tc := range []struct {
		pid  uint16
		name string
	}{
		{0x1749, "ASI174MM Mini"}, // IMX174, USB2, BulkRead path
		{0x620A, "ASI6200MC Pro"}, // IMX455, USB3 windowed-stream path
	} {
		t.Run(tc.name, func(t *testing.T) {
			ft := astrocam.NewStubTransport()
			cam, err := astrocam.Open(ft, astrocam.ZWO.VID, tc.pid)
			if err != nil {
				t.Fatalf("Open: %v", err)
			}
			defer ft.Close()
			defer cam.StopExposure()

			info := cam.Info()
			step := func(name string, err error) {
				t.Helper()
				if err != nil {
					t.Fatalf("%s: %v", name, err)
				}
			}
			step("Init", cam.Init())
			step("SetROI", cam.SetROI(0, 0, info.MaxWidth, info.MaxHeight))
			step("SetGain", cam.SetGain(0))
			step("SetExposure", cam.SetExposure(10*time.Millisecond)) // < 1 s → free-run, no host wait
			step("StartExposure", cam.StartExposure(true))

			buf := make([]byte, cam.FrameBytes())
			n, err := cam.GetDataAfterExp(buf)
			step("GetDataAfterExp", err)

			if n != cam.FrameBytes() {
				t.Fatalf("read %d bytes, want a full frame of %d", n, cam.FrameBytes())
			}
			if ft.Reads == 0 {
				t.Errorf("no frame read was served")
			}
			// The stub served a 16-bit ramp (pixel 0=0, pixel 1=1); confirm it round-tripped.
			if buf[0] != 0 || buf[1] != 0 || buf[2] != 1 || buf[3] != 0 {
				t.Errorf("frame head = % x, want ramp 00 00 01 00", buf[:4])
			}
			// The control plane must have actually programmed the camera (init + ops).
			if len(ft.Log) < 20 {
				t.Errorf("only %d control transfers logged — pipeline did not run", len(ft.Log))
			}
		})
	}
}

// TestE2ETriggerMode exercises the IMX455 ≥1 s trigger path end-to-end: the worker
// must host-time the integration AND the device must end up programmed for wait+trigger
// mode (FPGA reg0 bit6+bit7) with VMAX held at one frame (0x02cc for this 512 ROI, ROI-following)
// and SHS=10 — the values cross-checked against the SDK wire. The 1 s host wait keeps it off -short.
func TestE2ETriggerMode(t *testing.T) {
	if testing.Short() {
		t.Skip("trigger mode host-times a 1 s integration; skipped in -short")
	}
	ft := astrocam.NewStubTransport()
	cam, err := astrocam.Open(ft, astrocam.ZWO.VID, 0x620A)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	defer ft.Close()
	defer cam.StopExposure()

	for _, s := range []struct {
		name string
		err  error
	}{
		{"Init", cam.Init()},
		{"SetROI", cam.SetROI(0, 0, 512, 512)}, // small ROI → light frame buffer
		{"SetGain", cam.SetGain(0)},
		{"SetExposure", cam.SetExposure(1 * time.Second)}, // == threshold → trigger mode
	} {
		if s.err != nil {
			t.Fatalf("%s: %v", s.name, s.err)
		}
	}
	// SetExposure must have programmed wait+trigger mode and the one-frame VMAX.
	if reg0 := ft.FPGAReg(0x00); reg0&0x40 == 0 || reg0&0x80 == 0 {
		t.Errorf("reg0 = 0x%02x, want wait(0x40)+trigger(0x80) bits set", reg0)
	}
	// One-frame VMAX follows the ROI: 512-row window → vblank(52)+512=564 → (frameUs+10ms)/line+20
	// = 0x02cc (716). (Full-frame would be 0x19c0; the SDK shrinks it for a sub-frame ROI.)
	if lo, mid := ft.FPGAReg(0x10), ft.FPGAReg(0x11); lo != 0xcc || mid != 0x02 {
		t.Errorf("VMAX = 0x%02x%02x.., want 0x02cc (one frame for the 512 ROI)", mid, lo)
	}
	if shs := ft.Reg(0x16); shs != 0x0a {
		t.Errorf("SHS = 0x%02x, want 0x0a (10)", shs)
	}

	if err := cam.StartExposure(true); err != nil {
		t.Fatalf("StartExposure: %v", err)
	}
	buf := make([]byte, cam.FrameBytes())
	start := time.Now()
	n, err := cam.GetDataAfterExp(buf)
	if err != nil {
		t.Fatalf("GetDataAfterExp: %v", err)
	}
	if n != cam.FrameBytes() {
		t.Fatalf("read %d bytes, want %d", n, cam.FrameBytes())
	}
	// The worker host-times the integration, so the read takes ~the exposure.
	if elapsed := time.Since(start); elapsed < 900*time.Millisecond {
		t.Errorf("trigger capture returned in %v — integration was not host-timed", elapsed)
	}
}
