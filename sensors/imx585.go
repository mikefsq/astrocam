// Sony IMX585: Type 1/1.2, 8.3 MP, STARVIS 2, 3840×2160 (ZWO ASI585, PlayerOne Uranus/Xena).
// Own register map with a STARVIS-2 dual gain (0x3030 conv-gain select + 0x306c/0x306d analog
// code), offset 0x30dc/0x30dd, SHS 0x3050-52, ROI in the 0x303c-47 block, latch 0x3001.
// DDR-buffered USB3 camera (DDR enabled at bringup, per-frame DMA length plus a windowed read
// for capture), so the readout path follows the IMX455/6200.
//
// The PlayerOne half is hardware-validated on a Xena 585M. The ZWO half is unexercised: no ZWO
// PID is registered against this die (sensors/models.go). Profile entry points:
//
//	imx585Init, imx585InitFPGA   sensor and FPGA bringup
//	imx585SetROI                 window, per-frame DMA length, FPGA geometry, HMAX
//	imx585SetGain                analog gain and conversion gain
//	imx585SetExposure            frame period and shutter position
//	imx585SetOffset              black level
//	imx585StreamStart / StreamStop
//	imx585Worker                 the single-shot capture worker

package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
	"math"
	"time"
)

const (
	imx585RegLatch   = 0x3001 // 1 before / 0 after a coupled register group
	imx585RegStandby = 0x3000 // standby gate: 1 = standby (stop), 0 = streaming
	imx585RegXmsta   = 0x3004 // master-start gate, cleared to 0 alongside standby on stream start

	imx585RegConvGain = 0x3030 // conversion-gain select: 0 = LCG, 1 = HCG (gain > 199)
	imx585RegGainL    = 0x306c // analog gain code, low byte  (eff/3, 16-bit LE)
	imx585RegGainH    = 0x306d // analog gain code, high byte

	imx585RegOffsetL = 0x30dc // ASI Brightness / black level, low byte (16-bit LE)
	imx585RegOffsetH = 0x30dd // high byte

	imx585RegSHS0 = 0x3050 // 24-bit shutter SHS, little-endian
	imx585RegSHS1 = 0x3051
	imx585RegSHS2 = 0x3052

	imx585RegStartXL = 0x303c // ROI start X (align 2)
	imx585RegStartXH = 0x303d
	imx585RegStartYL = 0x3044 // ROI start Y (align 4)
	imx585RegStartYH = 0x3045
	imx585RegWidthL  = 0x303e // output window width
	imx585RegWidthH  = 0x303f
	imx585RegHeightL = 0x3046 // output window height
	// Die-side binning (POA_HARDWARE_BIN): mode 1 with factor 2 enables it, mode 0 with factor 4
	// is the default. Only a factor of 2 exists on this die.
	imx585RegSenBinMode   = 0x301b
	imx585RegSenBinFactor = 0x30d5
	imx585RegHeightH      = 0x3047
	imx585RegWinMode      = 0x3018 // window/mode byte, full-res value 0x14

	imx585GainMax   = 600 // 60.0 dB, ASI 0.1 dB units (SetGain clamp)
	imx585GainHCGAt = 199 // above this (gain > 199), HCG: conv 0x3030=1
	imx585HCGSub    = 150 // the HCG analog code uses (gain - 150)

	imx585ExpMinUs = 32            // µs floor
	imx585ExpMaxUs = 2_000_000_000 // 2000 s ceiling
	// PlayerOne advertises a wider range on this die. The floor is 10 µs. The ceiling is 7200 s,
	// which takes reading BOTH exposure configs to see: POA_EXPOSURE is an integer count of
	// microseconds and stops at 2,000,000,000 because that is where int32 runs out, while
	// POA_EXP is a double in seconds and reports 7200. The larger one is the real limit and the
	// header calls POA_EXP the recommended control.
	imx585ExpMinUsPOA = 10
	imx585ExpMaxUsPOA = 7_200_000_000
	// imx585MinExpLinesPOA is the shortest integration the SDK programs: at 10 us, well under
	// one line at this HMAX, it still leaves two lines between SHS and VMAX.
	imx585MinExpLinesPOA = 2
	imx585LongExpUs      = 1_000_000 // >= 1 s enters FPGA trigger mode (inclusive bound)

	// Readout constants for the shared engine (fps.go / shutter.go). Geometry is image
	// orientation, effective 4K. HMAX is baked: the SDK sets clock 20000 with no static HMAX
	// floor, and its FPS% default of 80 is not applied.
	imx585FullWidth  = 3840  // horizontal (lit16 0x3440)
	imx585FullHeight = 2160  // vertical, the line count VMAX = height + 2 uses
	imx585ClkKHz     = 20000 // 20 MHz
	imx585HMAX       = 192   // baked line-period HMAX; line time = 192·1e6/20000 = 9600 ns
	imx585VBlankAdd  = 2     // VMAX = height + 2 (the 60 in the Sony output-height regs 0x3046/47
	//                     is not the VMAX addend); affects the frame period, not the integration
	// imx585ArmSettle is the wait between releasing standby and starting the readout. Start the
	// readout before the sensor has settled and the FIRST frame carries a pedestal, about 70 DN
	// at gain 100 and growing with gain; only the single-frame path returns that frame, since the
	// free-run path spends one on its probe. Measured against the SDK at 640x480/100 ms,
	// 640x480/1 ms and the full frame: the vendor's own usleep(0x2710) of 10 ms fails, 20 ms is
	// exact, and 30 to 250 ms make no further difference.
	imx585ArmSettle = 30 * time.Millisecond
	// imx585AbortPoll bounds how long a host-timed integration goes without checking for an
	// abort. It is a CEILING on each sleep, not a fixed step: the last sleep is trimmed to what
	// is actually left.
	imx585AbortPoll = 100 * time.Millisecond

	imx585SHSGuard = 8 // SHS = clamp((VMAX-8) - lines, 8, VMAX-8)
	// HDR doubles the shutter guard: the SDK parks SHS at 16 there against 8 in Normal, measured
	// across every exposure-bound window on the camera. It is the floor SHS falls to once the
	// exposure sets the frame period, and VMAX is laid out to leave exactly that much room.
	imx585SHSGuardHDR = 16
)

// imx585Init is the ZWO bringup reglist (234 records, reg 0xffff = delay ms, the STARVIS-2
// analog tuning table) plus its tail (0x3015/0x3002/0x3018/0x301b/0x3022/0x3023), all inside
// one 0x3001 latch group. FPGA bringup is in imx585InitFPGA.
var imx585Init = []RegVal{
	{Reg: 0x3001, Val: 0x01}, // latch on; the latch brackets the whole reglist and tail
	{Reg: 0x3018, Val: 0x14}, {Reg: 0x3014, Val: 0x01}, {Reg: 0x3015, Val: 0x06}, {Reg: 0x3460, Val: 0x21}, {Reg: 0x3478, Val: 0xa1}, {Reg: 0x347c, Val: 0x01},
	{Reg: 0x3480, Val: 0x01}, {Reg: 0x3a4e, Val: 0x14}, {Reg: 0x3409, Val: 0x00}, {Reg: 0x340b, Val: 0x00}, {Reg: 0x3458, Val: 0x00}, {Reg: 0x3a4d, Val: 0x01},
	{Reg: 0x3a50, Val: 0x48}, {Reg: 0x3a51, Val: 0x01}, {Reg: 0x3a52, Val: 0x14}, {Reg: 0x3a56, Val: 0x00}, {Reg: 0x3a5a, Val: 0x00}, {Reg: 0x3a5e, Val: 0x00},
	{Reg: 0x3a62, Val: 0x00}, {Reg: 0x3a6a, Val: 0x20}, {Reg: 0x3a6c, Val: 0x42}, {Reg: 0x3a6e, Val: 0xa0}, {Reg: 0x3b2c, Val: 0x0c}, {Reg: 0x3b30, Val: 0x1c},
	{Reg: 0x3b34, Val: 0x0c}, {Reg: 0x3b38, Val: 0x1c}, {Reg: 0x3ba0, Val: 0x0c}, {Reg: 0x3ba4, Val: 0x1c}, {Reg: 0x3ba8, Val: 0x0c}, {Reg: 0x3bac, Val: 0x1c},
	{Reg: 0x3d3c, Val: 0x11}, {Reg: 0x3d46, Val: 0x0b}, {Reg: 0x3de0, Val: 0x3f}, {Reg: 0x3de1, Val: 0x08}, {Reg: 0x3e14, Val: 0x87}, {Reg: 0x3e16, Val: 0x91},
	{Reg: 0x3e18, Val: 0x91}, {Reg: 0x3e1a, Val: 0x87}, {Reg: 0x3e1c, Val: 0x78}, {Reg: 0x3e1e, Val: 0x50}, {Reg: 0x3e20, Val: 0x50}, {Reg: 0x3e22, Val: 0x50},
	{Reg: 0x3e24, Val: 0x87}, {Reg: 0x3e26, Val: 0x91}, {Reg: 0x3e28, Val: 0x91}, {Reg: 0x3e2a, Val: 0x87}, {Reg: 0x3e2c, Val: 0x78}, {Reg: 0x3e2e, Val: 0x50},
	{Reg: 0x3e30, Val: 0x50}, {Reg: 0x3e32, Val: 0x50}, {Reg: 0x3e34, Val: 0x87}, {Reg: 0x3e36, Val: 0x91}, {Reg: 0x3e38, Val: 0x91}, {Reg: 0x3e3a, Val: 0x87},
	{Reg: 0x3e3c, Val: 0x78}, {Reg: 0x3e3e, Val: 0x50}, {Reg: 0x3e40, Val: 0x50}, {Reg: 0x3e42, Val: 0x50}, {Reg: 0x4054, Val: 0x64}, {Reg: 0x4148, Val: 0xfe},
	{Reg: 0x4149, Val: 0x05}, {Reg: 0x414a, Val: 0xff}, {Reg: 0x414b, Val: 0x05}, {Reg: 0x420a, Val: 0x03}, {Reg: 0x423d, Val: 0x9c}, {Reg: 0x4242, Val: 0xb4},
	{Reg: 0x4246, Val: 0xb4}, {Reg: 0x424e, Val: 0xb4}, {Reg: 0x425c, Val: 0xb4}, {Reg: 0x425e, Val: 0xb6}, {Reg: 0x426c, Val: 0xb4}, {Reg: 0x426e, Val: 0xb6},
	{Reg: 0x428c, Val: 0xb4}, {Reg: 0x428e, Val: 0xb6}, {Reg: 0x4708, Val: 0x00}, {Reg: 0x4709, Val: 0x00}, {Reg: 0x470a, Val: 0xff}, {Reg: 0x470b, Val: 0x03},
	{Reg: 0x470c, Val: 0x00}, {Reg: 0x470d, Val: 0x00}, {Reg: 0x470e, Val: 0xff}, {Reg: 0x470f, Val: 0x03}, {Reg: 0x47eb, Val: 0x1c}, {Reg: 0x47f0, Val: 0xa6},
	{Reg: 0x47f2, Val: 0xa6}, {Reg: 0x47f4, Val: 0xa0}, {Reg: 0x47f6, Val: 0x96}, {Reg: 0x4808, Val: 0xa6}, {Reg: 0x480a, Val: 0xa6}, {Reg: 0x480c, Val: 0xa0},
	{Reg: 0x480e, Val: 0x96}, {Reg: 0x492c, Val: 0xb2}, {Reg: 0x4930, Val: 0x03}, {Reg: 0x4932, Val: 0x03}, {Reg: 0x4936, Val: 0x5b}, {Reg: 0x4938, Val: 0x82},
	{Reg: 0x493e, Val: 0x23}, {Reg: 0x4ba8, Val: 0x1c}, {Reg: 0x4ba9, Val: 0x03}, {Reg: 0x4bac, Val: 0x1c}, {Reg: 0x4bad, Val: 0x1c}, {Reg: 0x4bae, Val: 0x1c},
	{Reg: 0x4baf, Val: 0x1c}, {Reg: 0x4bb0, Val: 0x1c}, {Reg: 0x4bb1, Val: 0x1c}, {Reg: 0x4bb2, Val: 0x1c}, {Reg: 0x4bb3, Val: 0x1c}, {Reg: 0x4bb4, Val: 0x1c},
	{Reg: 0x4bb8, Val: 0x03}, {Reg: 0x4bb9, Val: 0x03}, {Reg: 0x4bba, Val: 0x03}, {Reg: 0x4bbb, Val: 0x03}, {Reg: 0x4bbc, Val: 0x03}, {Reg: 0x4bbd, Val: 0x03},
	{Reg: 0x4bbe, Val: 0x03}, {Reg: 0x4bbf, Val: 0x03}, {Reg: 0x4bc0, Val: 0x03}, {Reg: 0x4c14, Val: 0x87}, {Reg: 0x4c16, Val: 0x91}, {Reg: 0x4c18, Val: 0x91},
	{Reg: 0x4c1a, Val: 0x87}, {Reg: 0x4c1c, Val: 0x78}, {Reg: 0x4c1e, Val: 0x50}, {Reg: 0x4c20, Val: 0x50}, {Reg: 0x4c22, Val: 0x50}, {Reg: 0x4c24, Val: 0x87},
	{Reg: 0x4c26, Val: 0x91}, {Reg: 0x4c28, Val: 0x91}, {Reg: 0x4c2a, Val: 0x87}, {Reg: 0x4c2c, Val: 0x78}, {Reg: 0x4c2e, Val: 0x50}, {Reg: 0x4c30, Val: 0x50},
	{Reg: 0x4c32, Val: 0x50}, {Reg: 0x4c34, Val: 0x87}, {Reg: 0x4c36, Val: 0x91}, {Reg: 0x4c38, Val: 0x91}, {Reg: 0x4c3a, Val: 0x87}, {Reg: 0x4c3c, Val: 0x78},
	{Reg: 0x4c3e, Val: 0x50}, {Reg: 0x4c40, Val: 0x50}, {Reg: 0x4c42, Val: 0x50}, {Reg: 0x4d12, Val: 0x1f}, {Reg: 0x4d13, Val: 0x1e}, {Reg: 0x4d26, Val: 0x33},
	{Reg: 0x4e0e, Val: 0x59}, {Reg: 0x4e14, Val: 0x55}, {Reg: 0x4e16, Val: 0x59}, {Reg: 0x4e1e, Val: 0x3b}, {Reg: 0x4e20, Val: 0x47}, {Reg: 0x4e22, Val: 0x54},
	{Reg: 0x4e26, Val: 0x81}, {Reg: 0x4e2c, Val: 0x7d}, {Reg: 0x4e2e, Val: 0x81}, {Reg: 0x4e36, Val: 0x63}, {Reg: 0x4e38, Val: 0x6f}, {Reg: 0x4e3a, Val: 0x7c},
	{Reg: 0x4f3a, Val: 0x3c}, {Reg: 0x4f3c, Val: 0x46}, {Reg: 0x4f3e, Val: 0x59}, {Reg: 0x4f42, Val: 0x64}, {Reg: 0x4f44, Val: 0x6e}, {Reg: 0x4f46, Val: 0x81},
	{Reg: 0x4f4a, Val: 0x82}, {Reg: 0x4f5a, Val: 0x81}, {Reg: 0x4f62, Val: 0xaa}, {Reg: 0x4f72, Val: 0xa9}, {Reg: 0x4f78, Val: 0x36}, {Reg: 0x4f7a, Val: 0x41},
	{Reg: 0x4f7c, Val: 0x61}, {Reg: 0x4f7d, Val: 0x01}, {Reg: 0x4f7e, Val: 0x7c}, {Reg: 0x4f7f, Val: 0x01}, {Reg: 0x4f80, Val: 0x77}, {Reg: 0x4f82, Val: 0x7b},
	{Reg: 0x4f88, Val: 0x37}, {Reg: 0x4f8a, Val: 0x40}, {Reg: 0x4f8c, Val: 0x62}, {Reg: 0x4f8d, Val: 0x01}, {Reg: 0x4f8e, Val: 0x76}, {Reg: 0x4f8f, Val: 0x01},
	{Reg: 0x4f90, Val: 0x5e}, {Reg: 0x4f91, Val: 0x02}, {Reg: 0x4f92, Val: 0x69}, {Reg: 0x4f93, Val: 0x02}, {Reg: 0x4f94, Val: 0x89}, {Reg: 0x4f95, Val: 0x02},
	{Reg: 0x4f96, Val: 0xa4}, {Reg: 0x4f97, Val: 0x02}, {Reg: 0x4f98, Val: 0x9f}, {Reg: 0x4f99, Val: 0x02}, {Reg: 0x4f9a, Val: 0xa3}, {Reg: 0x4f9b, Val: 0x02},
	{Reg: 0x4fa0, Val: 0x5f}, {Reg: 0x4fa1, Val: 0x02}, {Reg: 0x4fa2, Val: 0x68}, {Reg: 0x4fa3, Val: 0x02}, {Reg: 0x4fa4, Val: 0x8a}, {Reg: 0x4fa5, Val: 0x02},
	{Reg: 0x4fa6, Val: 0x9e}, {Reg: 0x4fa7, Val: 0x02}, {Reg: 0x519e, Val: 0x79}, {Reg: 0x51a6, Val: 0xa1}, {Reg: 0x51f0, Val: 0xac}, {Reg: 0x51f2, Val: 0xaa},
	{Reg: 0x51f4, Val: 0xa5}, {Reg: 0x51f6, Val: 0xa0}, {Reg: 0x5200, Val: 0x9b}, {Reg: 0x5202, Val: 0x91}, {Reg: 0x5204, Val: 0x87}, {Reg: 0x5206, Val: 0x82},
	{Reg: 0x5208, Val: 0xac}, {Reg: 0x520a, Val: 0xaa}, {Reg: 0x520c, Val: 0xa5}, {Reg: 0x520e, Val: 0xa0}, {Reg: 0x5210, Val: 0x9b}, {Reg: 0x5212, Val: 0x91},
	{Reg: 0x5214, Val: 0x87}, {Reg: 0x5216, Val: 0x82}, {Reg: 0x5218, Val: 0xac}, {Reg: 0x521a, Val: 0xaa}, {Reg: 0x521c, Val: 0xa5}, {Reg: 0x521e, Val: 0xa0},
	{Reg: 0x5220, Val: 0x9b}, {Reg: 0x5222, Val: 0x91}, {Reg: 0x5224, Val: 0x87}, {Reg: 0x5226, Val: 0x82},
	// --- explicit bringup tail, still inside the latch group ---
	{Reg: 0x3015, Val: 0x07}, {Reg: 0x3002, Val: 0x01}, {Reg: 0x3018, Val: 0x14},
	{Reg: 0x301b, Val: 0x00}, {Reg: 0x3022, Val: 0x02}, {Reg: 0x3023, Val: 0x01},
	{Reg: 0x3001, Val: 0x00}, // latch off; FPGAReset + SendCMD(0xAF) follow in imx585InitFPGA / Camera.Init
}

// imx585GainCaps / imx585OffsetCaps are the advertised ranges per vendor. ZWO's are the decoded
// clamps; PlayerOne's are what the SDK reports on a Xena 585M (poasnap -caps): gain 0..750 where
// ZWO stops at 600, and offset 0..250 default 3 where ZWO uses 0..240 default 16. The ranges are
// vendor policy over one die, so they travel separately from the encoding.
func imx585GainCaps(vid uint16) (min, max int) {
	switch vid {
	case ZWO.VID:
		return 0, imx585GainMax
	case POA.VID:
		return 0, 750
	}
	return 0, 0
}

func imx585OffsetCaps(vid uint16) (min, max, def int) {
	switch vid {
	case ZWO.VID:
		return 0, 240, 16
	case POA.VID:
		return 0, 250, 3
	}
	return 0, 0, 0
}

// imx585PreInit runs the pre-table reset for vendors that need one. Only PlayerOne does; ZWO
// resets inside its FPGA bringup, after the table.
func imx585PreInit(rm Regmap) error {
	if rm.VID() == POA.VID {
		return imx585PreInitPOA(rm)
	}
	return nil
}

// imx585InitPOA is PlayerOne's own init reglist for this die (218 register/value records)
// rather than a reuse of ZWO's. Against the ZWO table's 226, the 218 are a strict subset and
// not one value differs.
// The vendors agree on Sony's analog tuning; what they disagree on is which registers sit in the
// table and which move into the init sequence around it.
var imx585InitPOA = []RegVal{
	{Reg: 0x3458, Val: 0x00}, {Reg: 0x3460, Val: 0x21}, {Reg: 0x3478, Val: 0xa1}, {Reg: 0x347c, Val: 0x01},
	{Reg: 0x3480, Val: 0x01}, {Reg: 0x3a4e, Val: 0x14}, {Reg: 0x3a52, Val: 0x14}, {Reg: 0x3a56, Val: 0x00},
	{Reg: 0x3a5a, Val: 0x00}, {Reg: 0x3a5e, Val: 0x00}, {Reg: 0x3a62, Val: 0x00}, {Reg: 0x3a6a, Val: 0x20},
	{Reg: 0x3a6c, Val: 0x42}, {Reg: 0x3a6e, Val: 0xa0}, {Reg: 0x3b2c, Val: 0x0c}, {Reg: 0x3b30, Val: 0x1c},
	{Reg: 0x3b34, Val: 0x0c}, {Reg: 0x3b38, Val: 0x1c}, {Reg: 0x3ba0, Val: 0x0c}, {Reg: 0x3ba4, Val: 0x1c},
	{Reg: 0x3ba8, Val: 0x0c}, {Reg: 0x3bac, Val: 0x1c}, {Reg: 0x3d3c, Val: 0x11}, {Reg: 0x3d46, Val: 0x0b},
	{Reg: 0x3de0, Val: 0x3f}, {Reg: 0x3de1, Val: 0x08}, {Reg: 0x3e14, Val: 0x87}, {Reg: 0x3e16, Val: 0x91},
	{Reg: 0x3e18, Val: 0x91}, {Reg: 0x3e1a, Val: 0x87}, {Reg: 0x3e1c, Val: 0x78}, {Reg: 0x3e1e, Val: 0x50},
	{Reg: 0x3e20, Val: 0x50}, {Reg: 0x3e22, Val: 0x50}, {Reg: 0x3e24, Val: 0x87}, {Reg: 0x3e26, Val: 0x91},
	{Reg: 0x3e28, Val: 0x91}, {Reg: 0x3e2a, Val: 0x87}, {Reg: 0x3e2c, Val: 0x78}, {Reg: 0x3e2e, Val: 0x50},
	{Reg: 0x3e30, Val: 0x50}, {Reg: 0x3e32, Val: 0x50}, {Reg: 0x3e34, Val: 0x87}, {Reg: 0x3e36, Val: 0x91},
	{Reg: 0x3e38, Val: 0x91}, {Reg: 0x3e3a, Val: 0x87}, {Reg: 0x3e3c, Val: 0x78}, {Reg: 0x3e3e, Val: 0x50},
	{Reg: 0x3e40, Val: 0x50}, {Reg: 0x3e42, Val: 0x50}, {Reg: 0x4054, Val: 0x64}, {Reg: 0x4148, Val: 0xfe},
	{Reg: 0x4149, Val: 0x05}, {Reg: 0x414a, Val: 0xff}, {Reg: 0x414b, Val: 0x05}, {Reg: 0x420a, Val: 0x03},
	{Reg: 0x423d, Val: 0x9c}, {Reg: 0x4242, Val: 0xb4}, {Reg: 0x4246, Val: 0xb4}, {Reg: 0x424e, Val: 0xb4},
	{Reg: 0x425c, Val: 0xb4}, {Reg: 0x425e, Val: 0xb6}, {Reg: 0x426c, Val: 0xb4}, {Reg: 0x426e, Val: 0xb6},
	{Reg: 0x428c, Val: 0xb4}, {Reg: 0x428e, Val: 0xb6}, {Reg: 0x4708, Val: 0x00}, {Reg: 0x4709, Val: 0x00},
	{Reg: 0x470a, Val: 0xff}, {Reg: 0x470b, Val: 0x03}, {Reg: 0x470c, Val: 0x00}, {Reg: 0x470d, Val: 0x00},
	{Reg: 0x470e, Val: 0xff}, {Reg: 0x470f, Val: 0x03}, {Reg: 0x47eb, Val: 0x1c}, {Reg: 0x47f0, Val: 0xa6},
	{Reg: 0x47f2, Val: 0xa6}, {Reg: 0x47f4, Val: 0xa0}, {Reg: 0x47f6, Val: 0x96}, {Reg: 0x4808, Val: 0xa6},
	{Reg: 0x480a, Val: 0xa6}, {Reg: 0x480c, Val: 0xa0}, {Reg: 0x480e, Val: 0x96}, {Reg: 0x492c, Val: 0xb2},
	{Reg: 0x4930, Val: 0x03}, {Reg: 0x4932, Val: 0x03}, {Reg: 0x4936, Val: 0x5b}, {Reg: 0x4938, Val: 0x82},
	{Reg: 0x493e, Val: 0x23}, {Reg: 0x4ba8, Val: 0x1c}, {Reg: 0x4ba9, Val: 0x03}, {Reg: 0x4bac, Val: 0x1c},
	{Reg: 0x4bad, Val: 0x1c}, {Reg: 0x4bae, Val: 0x1c}, {Reg: 0x4baf, Val: 0x1c}, {Reg: 0x4bb0, Val: 0x1c},
	{Reg: 0x4bb1, Val: 0x1c}, {Reg: 0x4bb2, Val: 0x1c}, {Reg: 0x4bb3, Val: 0x1c}, {Reg: 0x4bb4, Val: 0x1c},
	{Reg: 0x4bb8, Val: 0x03}, {Reg: 0x4bb9, Val: 0x03}, {Reg: 0x4bba, Val: 0x03}, {Reg: 0x4bbb, Val: 0x03},
	{Reg: 0x4bbc, Val: 0x03}, {Reg: 0x4bbd, Val: 0x03}, {Reg: 0x4bbe, Val: 0x03}, {Reg: 0x4bbf, Val: 0x03},
	{Reg: 0x4bc0, Val: 0x03}, {Reg: 0x4c14, Val: 0x87}, {Reg: 0x4c16, Val: 0x91}, {Reg: 0x4c18, Val: 0x91},
	{Reg: 0x4c1a, Val: 0x87}, {Reg: 0x4c1c, Val: 0x78}, {Reg: 0x4c1e, Val: 0x50}, {Reg: 0x4c20, Val: 0x50},
	{Reg: 0x4c22, Val: 0x50}, {Reg: 0x4c24, Val: 0x87}, {Reg: 0x4c26, Val: 0x91}, {Reg: 0x4c28, Val: 0x91},
	{Reg: 0x4c2a, Val: 0x87}, {Reg: 0x4c2c, Val: 0x78}, {Reg: 0x4c2e, Val: 0x50}, {Reg: 0x4c30, Val: 0x50},
	{Reg: 0x4c32, Val: 0x50}, {Reg: 0x4c34, Val: 0x87}, {Reg: 0x4c36, Val: 0x91}, {Reg: 0x4c38, Val: 0x91},
	{Reg: 0x4c3a, Val: 0x87}, {Reg: 0x4c3c, Val: 0x78}, {Reg: 0x4c3e, Val: 0x50}, {Reg: 0x4c40, Val: 0x50},
	{Reg: 0x4c42, Val: 0x50}, {Reg: 0x4d12, Val: 0x1f}, {Reg: 0x4d13, Val: 0x1e}, {Reg: 0x4d26, Val: 0x33},
	{Reg: 0x4e0e, Val: 0x59}, {Reg: 0x4e14, Val: 0x55}, {Reg: 0x4e16, Val: 0x59}, {Reg: 0x4e1e, Val: 0x3b},
	{Reg: 0x4e20, Val: 0x47}, {Reg: 0x4e22, Val: 0x54}, {Reg: 0x4e26, Val: 0x81}, {Reg: 0x4e2c, Val: 0x7d},
	{Reg: 0x4e2e, Val: 0x81}, {Reg: 0x4e36, Val: 0x63}, {Reg: 0x4e38, Val: 0x6f}, {Reg: 0x4e3a, Val: 0x7c},
	{Reg: 0x4f3a, Val: 0x3c}, {Reg: 0x4f3c, Val: 0x46}, {Reg: 0x4f3e, Val: 0x59}, {Reg: 0x4f42, Val: 0x64},
	{Reg: 0x4f44, Val: 0x6e}, {Reg: 0x4f46, Val: 0x81}, {Reg: 0x4f4a, Val: 0x82}, {Reg: 0x4f5a, Val: 0x81},
	{Reg: 0x4f62, Val: 0xaa}, {Reg: 0x4f72, Val: 0xa9}, {Reg: 0x4f78, Val: 0x36}, {Reg: 0x4f7a, Val: 0x41},
	{Reg: 0x4f7c, Val: 0x61}, {Reg: 0x4f7d, Val: 0x01}, {Reg: 0x4f7e, Val: 0x7c}, {Reg: 0x4f7f, Val: 0x01},
	{Reg: 0x4f80, Val: 0x77}, {Reg: 0x4f82, Val: 0x7b}, {Reg: 0x4f88, Val: 0x37}, {Reg: 0x4f8a, Val: 0x40},
	{Reg: 0x4f8c, Val: 0x62}, {Reg: 0x4f8d, Val: 0x01}, {Reg: 0x4f8e, Val: 0x76}, {Reg: 0x4f8f, Val: 0x01},
	{Reg: 0x4f90, Val: 0x5e}, {Reg: 0x4f91, Val: 0x02}, {Reg: 0x4f92, Val: 0x69}, {Reg: 0x4f93, Val: 0x02},
	{Reg: 0x4f94, Val: 0x89}, {Reg: 0x4f95, Val: 0x02}, {Reg: 0x4f96, Val: 0xa4}, {Reg: 0x4f97, Val: 0x02},
	{Reg: 0x4f98, Val: 0x9f}, {Reg: 0x4f99, Val: 0x02}, {Reg: 0x4f9a, Val: 0xa3}, {Reg: 0x4f9b, Val: 0x02},
	{Reg: 0x4fa0, Val: 0x5f}, {Reg: 0x4fa1, Val: 0x02}, {Reg: 0x4fa2, Val: 0x68}, {Reg: 0x4fa3, Val: 0x02},
	{Reg: 0x4fa4, Val: 0x8a}, {Reg: 0x4fa5, Val: 0x02}, {Reg: 0x4fa6, Val: 0x9e}, {Reg: 0x4fa7, Val: 0x02},
	{Reg: 0x519e, Val: 0x79}, {Reg: 0x51a6, Val: 0xa1}, {Reg: 0x51f0, Val: 0xac}, {Reg: 0x51f2, Val: 0xaa},
	{Reg: 0x51f4, Val: 0xa5}, {Reg: 0x51f6, Val: 0xa0}, {Reg: 0x5200, Val: 0x9b}, {Reg: 0x5202, Val: 0x91},
	{Reg: 0x5204, Val: 0x87}, {Reg: 0x5206, Val: 0x82}, {Reg: 0x5208, Val: 0xac}, {Reg: 0x520a, Val: 0xaa},
	{Reg: 0x520c, Val: 0xa5}, {Reg: 0x520e, Val: 0xa0}, {Reg: 0x5210, Val: 0x9b}, {Reg: 0x5212, Val: 0x91},
	{Reg: 0x5214, Val: 0x87}, {Reg: 0x5216, Val: 0x82}, {Reg: 0x5218, Val: 0xac}, {Reg: 0x521a, Val: 0xaa},
	{Reg: 0x521c, Val: 0xa5}, {Reg: 0x521e, Val: 0xa0}, {Reg: 0x5220, Val: 0x9b}, {Reg: 0x5222, Val: 0x91},
	{Reg: 0x5224, Val: 0x87}, {Reg: 0x5226, Val: 0x82},
}

// IMX585 is the Sony IMX585 STARVIS 2 profile, shared by ZWO's ASI585 and PlayerOne's
// Uranus/Xena bodies.
var IMX585 = Sensor{
	Name:      "IMX585", // mono die; MC adds a CFA
	GainMax:   imx585GainMax,
	ExpMinUs:  imx585ExpMinUs,
	ExpMaxUs:  imx585ExpMaxUs,
	OffsetMax: 240, OffsetDef: 16, // ASI Brightness range (family default)
	Info: CameraInfo{
		MaxWidth:  imx585FullWidth,
		MaxHeight: imx585FullHeight,
		PixelUm:   2.9,      // µm pitch
		BitDepth:  12,       // 12-bit ADC (RAW16 transport)
		Bayer:     "RGGB",   // CFA (color variant); surfaced when Model.Color
		Bins:      []int{1}, // ZWO's binned readout tables are not decoded; see BinsByVID
	},
	GainCaps:   imx585GainCaps,
	OffsetCaps: imx585OffsetCaps,
	ExpCaps:    imx585ExpCaps,
	Presets:    imx585Presets,
	Init:       imx585Init,
	InitByVID:  map[uint16][]RegVal{POA.VID: imx585InitPOA},
	PreInit:    imx585PreInit,
	// PlayerOne reads out the full effective array; the ZWO transcription uses the UHD crop.
	// Confirmed against the SDK on a Xena 585M (poasnap reports 3856x2180).
	SizeByVID: map[uint16][2]int{POA.VID: {3856, 2180}},
	// PlayerOne bins 1..4 in the FPGA; the ZWO path here has no binned mode tables.
	BinsByVID: map[uint16][]int{POA.VID: {1, 2, 3, 4}},
	// The die itself bins by 2 and nothing else. Bin 4 therefore splits 2 on the die and 2 in the
	// FPGA, and bin 3 has no die mode at all and falls back to the FPGA entirely — measured
	// against the SDK, which does exactly the same.
	HWBins: []int{2},
	// The SDK reports 11.402999877929688 as the die's gain-0 conversion.
	EGainBase: 11.402999877929688,
	// Normal and HDR, PlayerOne only. HDR is decoded and wired at RAW16; RAW8 is refused.
	SensorModes:   imx585SensorModes,
	SetSensorMode: imx585SetSensorMode,
	InitFPGA:      imx585InitFPGA,
	SetGain:       imx585SetGain,
	SetExposure:   imx585SetExposure,
	SetOffset:     imx585SetOffset,
	GetOffset:     imx585GetOffset,
	SetROI:        imx585SetROI,
	StreamStop:    func(rm Regmap) error { return rm.WriteReg(imx585RegStandby, 1) }, // standby on
	StreamStart:   imx585StreamStart,
	Worker:        imx585Worker,
	ROIStartAlign: func(int) (int, int) { return 2, 4 }, // window-start masks (X to 2, Y to 4)
}

// imx585StreamStart releases the sensor for readout. PlayerOne clears standby alone, with the
// FX3 stream start and the FPGA run bracketing it from the capture worker, while ZWO also clears
// the master-start gate first.
func imx585StreamStart(rm Regmap) error {
	if rm.VID() == POA.VID {
		return rm.WriteReg(imx585RegStandby, 0)
	}
	if err := rm.WriteReg(imx585RegXmsta, 0); err != nil {
		return err
	}
	return rm.WriteReg(imx585RegStandby, 0)
}

// imx585SetGain selects the vendor's gain encoding from the regmap's VID. Both vendors rebase the
// gain and divide by three onto the same registers, but with different constants, so neither
// band structure can stand in for the other.
func imx585SetGain(rm Regmap, gain int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx585SetGainZWO(rm, gain)
	case POA.VID:
		return imx585SetGainPOA(rm, gain)
	}
	return fmt.Errorf("imx585 gain: unsupported vendor VID 0x%04x", rm.VID())
}

// PlayerOne's IMX585 gain constants, measured on the wire rather than taken from the SDK's
// arithmetic: the SDK computes the code with a piecewise cubic polynomial, but a wire sweep of
// 33 gains from 0 to 750 lands every one exactly on a rebase-and-divide-by-three. The bands were bracketed to the gain: 209 is still LCG, 210 is HCG.
const (
	imx585GainMaxPOA = 750 // the range the SDK advertises
	imx585HCGAtPOA   = 210 // gain >= this selects high conversion gain
	imx585LCGSubPOA  = 45  // LCG rebase
	imx585HCGSubPOA  = 198 // HCG rebase

	// HDR drops the second conversion-gain band entirely and rebases on 72, and the SDK clamps
	// the gain to 500 there rather than the 750 the config advertises.
	imx585GainMaxHDRPOA = 500
	imx585HDRSubPOA     = 72
)

// The fine-gain register and the cubic behind it. The coefficients are round numbers, which is a
// good sign the reading is right: f(x) = 26.989 - 0.0252x - 1e-05x^2 - 1e-08x^3.
const (
	imx585RegFineGain = 0x423d
	// imx585FineGainFlatPOA is the value above the cubic's range, and also the additive term
	// inside it — the register is 0x80 plus twice the polynomial.
	imx585FineGainFlatPOA = 0x80
	imx585FineGainLimPOA  = 45 // Normal: the cubic applies at or below this gain
	// Normal enters the cubic 270 units in, so its input spans 270..720 over gain 0..45. HDR
	// enters at 0 and spans 0..720 over gain 0..72 — the same curve over a longer stretch.
	imx585FineGainOffPOA    = 270
	imx585FineGainLimHDRPOA = 72

	imx585FineGainC3 = -1e-08
	imx585FineGainC2 = -1e-05
	imx585FineGainC1 = -0.0252
	imx585FineGainC0 = 26.989
)

// imx585SetGainPOA is PlayerOne's encoding:
//
//	gain <  210: 0x3030 = 0, code = max(0, (gain - 45) / 3)
//	gain >= 210: 0x3030 = 1, code = (gain - 198) / 3
//
// The conversion gain goes out first and unlatched, then the 16-bit code to 0x306c/0x306d inside
// a 0x3001 latch group. The SDK writes that pair as one burst; two register writes cover the same
// two registers with the same bytes.
func imx585SetGainPOA(rm Regmap, gain int) error {
	if gain < 0 {
		gain = 0
	}
	hdr := ModeOf(rm).SensorMode == imx585ModeHDRPOA
	max := imx585GainMaxPOA
	if hdr {
		max = imx585GainMaxHDRPOA
	}
	if gain > max {
		gain = max
	}
	// The fine-gain register comes first and is written on every path, in both modes.
	if err := rm.WriteReg(imx585RegFineGain, imx585FineGainPOA(gain, hdr)); err != nil {
		return err
	}
	// HDR has no high conversion gain: 0x3030 stays 0 at every gain, and the analog code is
	// rebased on 72 rather than 45 with no second band. Below the rebase the code is 0 and the
	// fine-gain register carries the whole adjustment.
	conv, eff := uint16(0), 0
	switch {
	case hdr:
		if eff = gain - imx585HDRSubPOA; eff < 0 {
			eff = 0
		}
	case gain >= imx585HCGAtPOA:
		conv, eff = 1, gain-imx585HCGSubPOA
	default:
		if eff = gain - imx585LCGSubPOA; eff < 0 {
			eff = 0
		}
	}
	if err := rm.WriteReg(imx585RegConvGain, conv); err != nil {
		return err
	}
	return WriteRegLE(rm, imx585RegLatch, []uint16{imx585RegGainL, imx585RegGainH}, uint32(eff/3))
}

// imx585FineGainPOA is the value the SDK puts in the fine-gain register 0x423d.
// It is gain-derived, not a mode or sample-size setting. Above a mode-dependent limit it is a
// flat 0x80; below, it follows one cubic that both modes share, differing only in where the gain
// enters it — Normal starts 270 units in, HDR at 0, and each runs until the input reaches 720.
//
// The float widths matter and are the SDK's: the powers are computed in float32, accumulated in
// float64, narrowed back to float32, then rounded half away from zero (fcvtas) and clamped at 0.
func imx585FineGainPOA(gain int, hdr bool) uint16 {
	limit, base := imx585FineGainLimPOA, imx585FineGainOffPOA
	if hdr {
		limit, base = imx585FineGainLimHDRPOA, 0
	}
	if gain > limit {
		return imx585FineGainFlatPOA
	}
	x := float32(gain*10 + base)
	x2, x3 := float32(x*x), float32(float32(x*x)*x)
	v := float32(float64(x3)*imx585FineGainC3 + float64(x2)*imx585FineGainC2 +
		float64(x)*imx585FineGainC1 + imx585FineGainC0)
	n := int(math.Round(float64(v)))
	if n < 0 {
		n = 0
	}
	return uint16(n*2+imx585FineGainFlatPOA) & 0xfffe
}

func imx585SetGainZWO(rm Regmap, gain int) error {
	if gain > imx585GainMax {
		gain = imx585GainMax
	}
	if gain < 0 {
		gain = 0
	}
	conv := uint16(0)
	eff := gain
	if gain > imx585GainHCGAt {
		conv = 1
		eff = gain - imx585HCGSub
	}
	code := uint16(eff / 3)
	return WithLatch(rm, imx585RegLatch, func() error {
		for _, rv := range []RegVal{
			{Reg: imx585RegConvGain, Val: conv},
			{Reg: imx585RegGainL, Val: code & 0xff},
			{Reg: imx585RegGainH, Val: (code >> 8) & 0xff},
		} {
			if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
				return err
			}
		}
		return nil
	})
}

// PlayerOne frame-timing constants, derived from the capture sweeps rather than the SDK's float
// pipeline. The frame period is whichever is longer, the time to ship the frame over the link or
// the exposure, and HMAX x VMAX counts that period in 20 MHz ticks:
//
//	period    = max(frameBytes / linkThroughput, exposure)
//	HMAX      = max(floor, width x bytesPerPixel / 26.593)
//	VMAX      = period x 20 MHz / HMAX
//
// The clock fell out of an exposure-limited capture (92 x 0x446 over 5 ms = 20.13 MHz, the same
// 20 MHz the profile already uses) and the throughput out of five bandwidth-limited windows that
// agreed to three digits. Reproduces every measured drive value: HMAX exactly in all ten cases,
// VMAX inside 0.05% except the exposure-limited ones, which run ~0.5% low — the SDK evidently adds
// a little readout overhead to the exposure term that is not modelled here.
//
// The throughputs are per link at 100% bandwidth. USB2 is measured (38.65 MB/s at the SDK's 90%
// default); USB3 comes from video benchmarks that plateaued at ~300 MB/s, also at 90%.
const (
	imx585POAHMAXDiv     = 26.593
	imx585POAHMAXFloor8  = 92  // RAW8
	imx585POAHMAXFloor16 = 154 // RAW16
	// imx585GpifBwCPOA is the numerator of the GPIF bandwidth divider. The SDK computes
	// trunc((C/rate - 1) x 256) where rate is the link constant scaled by the bandwidth
	// percentage, so the whole thing collapses to trunc((C/pct - 1) x 256). C is measured on the
	// wire rather than read out, because neither term is written anywhere observable. Validated
	// against 19 percentages from 35 to 100, every one exact, which bounds C to
	// [980.390625, 980.410156).
	imx585GpifBwCPOA     = 980.4
	imx585GpifBwCPOAUSB3 = 109.6

	imx585POAThroughputUSB2 = 42.94e6 // bytes/s at 100%
	// Measured on the camera from the SDK's own drive register, at geometries large enough for
	// the link to be the binding constraint: a full frame and 1920x1080 both give 384.1 MB/s at
	// 100%, agreeing to 0.003%. Smaller windows cannot be used to fit it — they hit the line-count
	// floor instead and would read low.
	imx585POAThroughputUSB3 = 384.1e6
	imx585POAVBlank         = 40 // rows added to the readout height for the line count
)

// imx585DrivePOA computes the [HMAX][VMAX] pair for a window, sample size, exposure and link.
func imx585DrivePOA(w, h, bpp, bin int, d time.Duration, usb3 bool, fpsPercent, frameLimit int, hdr bool) (hmax uint16, vmax uint32) {
	floor := imx585POAHMAXFloor8
	if bpp >= 2 {
		floor = imx585POAHMAXFloor16
	}
	hm := float64(w*bpp) / imx585POAHMAXDiv
	if hm < float64(floor) {
		hm = float64(floor)
	}
	// HDR does not let the line period grow with width: both HDR windows in the capture program
	// HMAX 154, the RAW16 floor, including the full frame where Normal programs 290. The frame
	// PERIOD is the same in both modes — 154 x 56488 and 290 x 29996 agree to 0.004% — so HDR
	// moves the whole timing budget into VMAX, exactly as the bandwidth percentage does.
	if hdr {
		hm = float64(floor)
	}
	throughput := imx585POAThroughputUSB2
	if usb3 {
		throughput = imx585POAThroughputUSB3
	}
	if fpsPercent > 0 && fpsPercent < 100 {
		throughput *= float64(fpsPercent) / 100
	}
	period := float64(w*h*bpp) / throughput
	if e := d.Seconds(); e > period {
		period = e
	}
	// The frame-rate cap is a third competing constraint, not a rescaling of the other two: the
	// SDK takes max(bandwidthTime, exposureTime, 1/limit).
	if frameLimit > 0 {
		if l := 1 / float64(frameLimit); l > period {
			period = l
		}
	}
	ticks := period * float64(imx585ClkKHz) * 1000
	hmax = uint16(hm + 0.5)
	v := uint32(ticks/float64(hmax) + 0.5)
	expLines := uint32(d.Seconds()*float64(imx585ClkKHz)*1000/float64(hmax) + 0.5)
	// VMAX also has to cover the frame itself. The SDK computes a line count — the readout rows
	// plus 40 — and the period can never be shorter than that. On USB2 the bandwidth term
	// always dominated so this never bound; on SuperSpeed it is the binding constraint for
	// anything but the largest windows, and a VMAX below it truncates the readout.
	// HDR reads the window twice, so the floor counts twice the lines, as the SDK's own line
	// count does. Neither captured HDR window is anywhere near this bound — both are
	// bandwidth-bound — so the doubling is UNVERIFIED on the wire. It binds only on SuperSpeed
	// at large windows.
	lines := uint32(h*bin) + imx585POAVBlank
	if hdr {
		lines = uint32(h*bin)*2 + imx585POAVBlank
	}
	if v < lines {
		v = lines
	}
	// The frame period must clear the exposure by the shutter guard, not merely equal it. Set
	// VMAX to the exposure exactly and SHS has nowhere to sit: it clamps to the guard, the
	// integration comes out short, and the readout has no lines left. Normal tolerates that;
	// HDR delivers four rows and stops. Measured across the SDK's exposure-bound range, VMAX
	// lands on the exposure plus the guard — 8 lines in Normal, 16 in HDR — every time.
	if guard := expLines + uint32(imx585SHSGuardFor(hdr)); v < guard {
		v = guard
	}
	if v > 0xffffff {
		v = 0xffffff
	}
	return hmax, v
}

// imx585SetExposurePOA programs PlayerOne's exposure: the frame period into the FPGA drive
// register, the exposure itself into the FPGA timer, and the shutter position into the sensor.
func imx585SetExposurePOA(rm Regmap, d time.Duration) error {
	m := ModeOf(rm)
	w, h := m.Width, m.Height
	if w == 0 || h == 0 {
		w, h = imx585POAFullWidth, imx585POAFullHeight
	}
	bin := m.Bin
	if bin < 1 {
		bin = 1
	}
	// At or above one second the FPGA times the exposure itself: the mode byte goes to 1 and the
	// frame period collapses to the sensor's own minimum, because the sensor is no longer what
	// holds the shutter open. Measured — a 1 s frame programs VMAX 520 for a 480-row window,
	// exactly the line count, where 100 ms programs 12994. The threshold is POACamera's
	// `exposure >= 1,000,000 us`, the same inclusive bound the ZWO path uses.
	long := d >= imx585LongExpUs*time.Microsecond
	hmax, vmax := imx585DrivePOA(w, h, m.BytesPerPx, bin, d, m.USB3, m.FPSPercent, m.FrameLimit, m.SensorMode == imx585ModeHDRPOA)
	// The GPIF bandwidth goes out with the drive pair, as the SDK sends it. This is not
	// optional: an FPGA reset leaves the register at 8, and the readout then delivers a few rows
	// and stops. Normal mode survives it; HDR, which puts twice the lines through the same
	// window, does not.
	if err := POAFPGABandwidth(rm, imx585GpifBwPOA(m.FPSPercent, m.USB3)); err != nil {
		return err
	}
	if long {
		vmax = uint32(h*bin) + imx585POAVBlank
	}
	if err := POAFPGADrive(rm, hmax, vmax); err != nil {
		return err
	}
	mode := uint16(0)
	if long {
		mode = 1
	}
	if err := POAFPGAExposureMode(rm, mode); err != nil {
		return err
	}
	if err := POAFPGAExposure(rm, uint64(d.Microseconds())); err != nil {
		return err
	}
	// Shutter position. The SDK subtracts the integration straight from VMAX — SHS = VMAX - lines
	// — rather than from a VMAX-guard window: measured against it, a 10 us exposure programs SHS
	// VMAX-2 and 32 us programs VMAX-4. Subtracting the guard as well put a floor of eight lines,
	// 62 us at this HMAX, under every short exposure. The guard is the LOWER bound on SHS, and
	// VMAX is laid out to leave room for it (imx585DrivePOA), so it should not bind here.
	lineNs := uint64(hmax) * 1_000_000 / imx585ClkKHz
	lines := ExposureLines(d, lineNs, imx585ExpMinUsPOA, imx585ExpMaxUsPOA)
	if lines < imx585MinExpLinesPOA {
		lines = imx585MinExpLinesPOA // the shortest integration the SDK ever programs
	}
	guard := uint64(imx585SHSGuardFor(m.SensorMode == imx585ModeHDRPOA))
	shs := int64(vmax) - int64(lines)
	if shs < int64(guard) {
		shs = int64(guard)
	}
	return WriteRegLE(rm, imx585RegLatch, []uint16{imx585RegSHS0, imx585RegSHS1, imx585RegSHS2}, uint32(shs))
}

// imx585SetExposure dispatches per vendor: the two compute the frame period from different models
// and write it to different registers.
func imx585SetExposure(rm Regmap, d time.Duration) error {
	if rm.VID() == POA.VID {
		return imx585SetExposurePOA(rm, d)
	}
	return imx585SetExposureZWO(rm, d)
}

func imx585SetExposureZWO(rm Regmap, d time.Duration) error {
	trigger := d >= imx585LongExpUs*time.Microsecond
	// Set both FPGA flags for the >= 1 s band and clear both below: reg0 bit6 (EnableFPGAWaitMode)
	// + bit7 (EnableFPGATriggerMode), WaitMode then TriggerMode to set, the reverse to clear.
	if trigger {
		if err := SetFPGABit(rm, 0x00, 0x40, true); err != nil { // EnableFPGAWaitMode
			return err
		}
		if err := SetFPGABit(rm, 0x00, 0x80, true); err != nil { // EnableFPGATriggerMode
			return err
		}
	} else {
		if err := SetFPGABit(rm, 0x00, 0x80, false); err != nil { // EnableFPGATriggerMode off
			return err
		}
		if err := SetFPGABit(rm, 0x00, 0x40, false); err != nil { // EnableFPGAWaitMode off
			return err
		}
	}
	lineNs := uint64(imx585HMAX) * 1_000_000 / imx585ClkKHz // 9600 ns
	lines := ExposureLines(d, lineNs, imx585ExpMinUs, imx585ExpMaxUs)

	// effHeight = the sensor-side readout rows (the live ROI height set by SetROI, else full), so
	// a sub-frame free-runs at its own frame period.
	effH := uint64(imx585FullHeight)
	if h := ModeOf(rm).Height; h > 0 {
		effH = uint64(h)
	}
	vmax := effH + imx585VBlankAdd // 2162 at full height
	hi := vmax - imx585SHSGuard    // SHS base / ceiling = VMAX-8
	if lines+1 > hi {              // past the in-frame window: stretch VMAX
		vmax = lines + 1 + imx585SHSGuard
		hi = vmax - imx585SHSGuard
	}
	if vmax > 0xffffff {
		vmax = 0xffffff
	}
	shs := int64(hi) - int64(lines)
	if shs < int64(imx585SHSGuard) { // floor 8
		shs = int64(imx585SHSGuard)
	}
	if shs > int64(hi) { // ceiling VMAX-8
		shs = int64(hi)
	}
	if err := SetVMAX(rm, uint32(vmax)); err != nil {
		return err
	}
	return WriteRegLE(rm, imx585RegLatch, []uint16{imx585RegSHS0, imx585RegSHS1, imx585RegSHS2}, uint32(shs))
}

// imx585GetOffset reads the offset back from 0x30dc/0x30dd, the pair imx585SetOffset writes.
func imx585GetOffset(rm Regmap) (int, error) {
	v, err := ReadRegLE(rm, []uint16{imx585RegOffsetL, imx585RegOffsetH})
	return int(v), err
}

func imx585SetOffset(rm Regmap, offset int) error {
	if err := WriteRegLE(rm, imx585RegLatch, []uint16{imx585RegOffsetL, imx585RegOffsetH}, uint32(uint16(offset))); err != nil {
		return err
	}
	// HDR carries a second copy of the offset, in the FPGA's own units, written right after the
	// sensor register. Normal never writes it, and without it the HDR pedestal is wrong.
	if rm.VID() == POA.VID && ModeOf(rm).SensorMode == imx585ModeHDRPOA {
		return POAFPGABurst(rm, imx585FPGAHDROffset, imx585LE16(imx585HDROffsetPOA(offset)))
	}
	return nil
}

// imx585HDROffsetPOA converts a POA offset to the FPGA's HDR offset register. The SDK scales the
// offset by four, multiplies by a fixed constant, then truncates toward zero.
func imx585HDROffsetPOA(offset int) uint16 {
	if offset < 0 {
		offset = 0
	}
	v := int(float64(offset<<2) * imx585HDROffsetScalePOA)
	if v > 0xffff {
		v = 0xffff
	}
	return uint16(v)
}

// imx585POACropY is the FPGA crop origin PlayerOne programs in Normal sensor mode; HDR uses 42.
// It is a fixed offset into the array, not the ROI position, and is the same in every window of
// the capture sweep.
const (
	imx585POACropY = 21
	// HDR reads two exposures per frame, and the crop origin moves with it. The pair 21/42 is
	// what the SDK's mode branches hold, and both were seen on the wire.
	imx585POACropYHDR = 42
	// imx585POAWidthAlign is the multiple the SENSOR window width is rounded up to; the FPGA
	// still receives the requested width and crops.
	imx585POAWidthAlign = 16
	imx585POAFullWidth  = 3856
	imx585POAFullHeight = 2180
	// imx585HDROffsetScalePOA scales a POA offset onto FPGA register 0x36 in HDR (0x36 is not
	// written in Normal). The offset is shifted left two before the multiply, so a unit of offset
	// is 86.0658 here.
	imx585HDROffsetScalePOA = 21.51645416744618
)

// imx585SetROI dispatches the window programming per vendor. The two do not share it: PlayerOne
// takes the requested geometry verbatim while ZWO rounds it, and the FPGA halves are different
// register maps entirely.
func imx585SetROI(rm Regmap, x, y, w, h, bin int) error {
	switch rm.VID() {
	case ZWO.VID:
		return imx585SetROIZWO(rm, x, y, w, h, bin)
	case POA.VID:
		return imx585SetROIPOA(rm, x, y, w, h, bin)
	}
	return fmt.Errorf("imx585 SetROI: unsupported vendor VID 0x%04x", rm.VID())
}

// imx585SetROIPOA programs a window on a PlayerOne body, measured on the wire from a six-window
// sweep.
//
// The FPGA gets the requested geometry exactly. The SENSOR window width is rounded UP to a
// multiple of 16 and the FPGA crops the margin — the same split the ZWO path uses, though with
// different rounding and no dummy lines. Measured against the SDK, widths 324, 328 and 332 all
// program a sensor width of 336, and 340/344/348 program 352.
//
// Programming the width verbatim leaves the sensor emitting short lines into a frame laid out for
// the full width, which shears the readout. The damage is easy to miss: of sixteen widths tested,
// only 328 and 340 produced outliers large enough to see by eye.
//
// Two registers the SDK also touches around a window change are deliberately absent. The drive
// register (FPGA 0x14, the HMAX/VMAX frame period) is computed from the exposure, so it belongs
// to the exposure path rather than here. The GPIF bandwidth (FPGA 0x28) did not vary across the
// sweep — it tracks the bandwidth setting, not the window.
func imx585SetROIPOA(rm Regmap, x, y, w, h, bin int) error {
	if bin < 1 || bin > 4 {
		return fmt.Errorf("imx585: PlayerOne bin %d not supported (1..4)", bin)
	}
	// PlayerOne bins in the FPGA, so the sensor still reads the full region: the window is the
	// OUTPUT size scaled back up by the factor, and the FPGA is handed that same region with the
	// factor in its mode byte. With die-side binning on, the sensor window is UNCHANGED — it is
	// still the full unbinned extent — and only what the FPGA is told changes: it receives the
	// already-halved geometry and a correspondingly smaller factor. Measured across bins 2, 3 and
	// 4: a bin-3 full frame programs 3852x2178 for a 1284x726 output, which is the output rounded
	// to the SDK's width%4 and height%2 rule and then scaled.
	//
	// Die-side binning is selected by 0x301b = 1 with 0x30d5 = 2 (off: 0 and 4). The die only
	// bins by 2, so bin 4 is 2 on the die and 2 in the FPGA and bin 3 cannot use it at all.
	senBin := ModeOf(rm).SensorBin
	if senBin < 1 || bin%senBin != 0 {
		senBin = 1
	}
	fpgaBin := bin / senBin
	if err := rm.WriteReg(imx585RegSenBinMode, map[bool]uint16{true: 1, false: 0}[senBin > 1]); err != nil {
		return err
	}
	if err := rm.WriteReg(imx585RegSenBinFactor, map[bool]uint16{true: 2, false: 4}[senBin > 1]); err != nil {
		return err
	}
	w, h = w*bin, h*bin
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	ux := uint16(x &^ 1) // the window start aligns X to 2 and Y to 4, as on ZWO
	uy := uint16(y &^ 3)
	// The sensor window is wider than the frame: the FPGA is handed the exact width below and
	// crops the margin.
	senW := (w + imx585POAWidthAlign - 1) &^ (imx585POAWidthAlign - 1)

	// Sensor window: start and size, four 16-bit pairs in one latch group. The SDK writes each
	// pair as a burst; two register writes cover the same registers with the same bytes.
	if err := WithLatch(rm, imx585RegLatch, func() error {
		for _, rv := range []RegVal{
			{Reg: imx585RegStartXL, Val: ux & 0xff}, {Reg: imx585RegStartXH, Val: (ux >> 8) & 0xff},
			{Reg: imx585RegStartYL, Val: uy & 0xff}, {Reg: imx585RegStartYH, Val: (uy >> 8) & 0xff},
			{Reg: imx585RegWidthL, Val: uint16(senW) & 0xff}, {Reg: imx585RegWidthH, Val: uint16(senW>>8) & 0xff},
			{Reg: imx585RegHeightL, Val: uint16(h) & 0xff}, {Reg: imx585RegHeightH, Val: uint16(h>>8) & 0xff},
		} {
			if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
				return err
			}
		}
		return nil
	}); err != nil {
		return err
	}

	// FPGA: the crop origin, then the format/mode bytes and the frame length.
	hdr := ModeOf(rm).SensorMode == imx585ModeHDRPOA
	cropY := imx585POACropY
	if hdr {
		cropY = imx585POACropYHDR
	}
	// The crop origin is in POST-DIE-BIN rows, so die-side binning scales it down: 21 becomes 11
	// at a die factor of 2, rounded up rather than truncated. Measured at both bin 2 and bin 4
	// with -hwbin, which share the same die factor.
	cropY = (cropY + senBin - 1) / senBin
	if err := POAFPGACropOrigin(rm, 0, uint16(cropY)); err != nil {
		return err
	}
	bpp16 := ModeOf(rm).BytesPerPx >= 2
	format := uint8(0)
	if bpp16 {
		format = 1 // reg 0x02 reads 0x81 for RAW16 and 0x00 for RAW8 on the wire
	}
	if hdr && !bpp16 {
		return fmt.Errorf("imx585: HDR is a RAW16 mode; the vendor SDK accepts the combination but programs Normal registers and returns a dead frame")
	}
	// HDR puts two exposures on the bus per frame, so the FPGA is given twice the height. The
	// sensor's own window is NOT doubled: 2×2180 is past the end of the array, and the doubling
	// is the DOL readout emitting two passes over the same window. The wide flag pays for it —
	// it switches the frame-length shift from >>2 to >>3, which exactly cancels the doubling, so
	// the DMA word count and the frame the host receives are the same size in both modes
	// (153600 at 640x480 and 4203040 at full frame, measured in both).
	fpgaW, fpgaH := w/senBin, h/senBin
	if hdr {
		fpgaH *= 2
	}
	// The colour flag in reg 0x04 bit 7 is the SDK's isColour AND NOT monoBin; a mono body clears
	// it, and the profile has no colour signal here, so it stays false until a colour Uranus is
	// available to measure.
	// The HDR offset register is NOT written here: it is offset-derived, so it belongs to
	// SetOffset, which the Camera re-applies after a mode change for exactly this reason.
	return POAFPGAImageSize(rm, fpgaW, fpgaH, bpp16, format, false, uint8(fpgaBin-1), hdr, ModeOf(rm).BinSum)
}

func imx585SetROIZWO(rm Regmap, x, y, w, h, bin int) error {
	if bin != 1 {
		return fmt.Errorf("imx585: bin %d not supported (binned mode regs not yet decoded)", bin)
	}
	if x < 0 {
		x = 0
	}
	if y < 0 {
		y = 0
	}
	ux := uint16(x &^ 1) // align X to 2 (&~1)
	uy := uint16(y &^ 3) // align Y to 4 (&~3)
	// Window sizes carry margins: width rounded up to 16, height rounded up
	// to 4 then +2 dummy lines. The sensor window is deliberately wider than the frame the host
	// receives: the FPGA below is given the exact requested w×h and crops the margin, the same
	// split the IMX455 uses (its window register is width+0x18 against an exact SetFPGAWidth),
	// there confirmed pixel-for-pixel against the SDK. The margin falls in the die's non-active
	// columns/rows, so a window reaching the array edge does not over-read.
	uw := uint16((w + 15) &^ 15)
	uh := uint16(((h + 3) &^ 3) + 2)

	if err := rm.WriteReg(imx585RegWinMode, 0x14); err != nil {
		return err
	}
	if err := WithLatch(rm, imx585RegLatch, func() error {
		for _, rv := range []RegVal{
			{Reg: imx585RegStartXL, Val: ux & 0xff}, {Reg: imx585RegStartXH, Val: (ux >> 8) & 0xff},
			{Reg: imx585RegStartYL, Val: uy & 0xff}, {Reg: imx585RegStartYH, Val: (uy >> 8) & 0xff},
			{Reg: imx585RegWidthL, Val: uw & 0xff}, {Reg: imx585RegWidthH, Val: (uw >> 8) & 0xff},
			{Reg: imx585RegHeightL, Val: uh & 0xff}, {Reg: imx585RegHeightH, Val: (uh >> 8) & 0xff},
		} {
			if err := rm.WriteReg(rv.Reg, rv.Val); err != nil {
				return err
			}
		}
		return nil
	}); err != nil {
		return err
	}

	// FPGA frame geometry for the FX3 DDR transfer: the per-frame DMA word
	// count, then output width/height. HMAX is the baked 192, written to the FPGA HMAX register
	// directly (DDR branch, as on the IMX455; not the bandwidth throttle).
	bpp := ModeOf(rm).BytesPerPx
	if err := SetFPGABinDataLen(rm, uint32((w*h*bpp+3)/4)); err != nil {
		return err
	}
	if err := FPGAWrite16(rm, 0x04, 0x05, uint16(w)); err != nil { // SetFPGAWidth
		return err
	}
	if err := FPGAWrite16(rm, 0x08, 0x09, uint16(h)); err != nil { // SetFPGAHeight
		return err
	}
	return WriteFPGAHMAX(rm, imx585HMAX)
}

// imx585PreInitPOA is PlayerOne's reset sequence, which runs BEFORE the register table: FPGA
// reset, then the sensor reset line asserted and released. Doing this after the table would throw
// the table away, which is why the profile needs a pre-table hook at all.
//
// Read off the wire: FPGA reg 0 takes 1, then 4, then 0, and the SDK
// sleeps between them. The sleeps are not measured — 20 ms follows the ZWO path's reset delay.
func imx585PreInitPOA(rm Regmap) error {
	if err := POAFPGAReset(rm); err != nil {
		return err
	}
	time.Sleep(20 * time.Millisecond)
	if err := POAFPGASensorReset(rm, true); err != nil {
		return err
	}
	time.Sleep(20 * time.Millisecond)
	return POAFPGASensorReset(rm, false)
}

// imx585TailPOA and the mode blocks below are what the SDK runs after the register table, read
// off the wire in order. Three parts: the five sensor registers PlayerOne keeps out
// of its table, the FPGA's white-balance and HDR defaults, and the Normal-mode sensor block — the
// last being the part of the window setup that a capture makes plain.
var (
	imx585TailPOA = []RegVal{
		{Reg: 0x3014, Val: 0x01}, {Reg: 0x3040, Val: 0x07}, {Reg: 0x3015, Val: 0x05},
		{Reg: 0x3018, Val: 0x14}, {Reg: 0x3081, Val: 0x02},
	}
	// imx585ModeCommonPOA is the part of the block that does not move with the sample size.
	imx585ModeCommonPOA = []RegVal{
		{Reg: 0x301a, Val: 0x00}, {Reg: 0x3069, Val: 0x00}, {Reg: 0x3074, Val: 0x64},
		{Reg: 0x3a4c, Val: 0x39}, {Reg: 0x3a4d, Val: 0x01}, {Reg: 0x3a50, Val: 0x48},
		{Reg: 0x3a51, Val: 0x01}, {Reg: 0x3e10, Val: 0x10}, {Reg: 0x493c, Val: 0x23},
		{Reg: 0x4940, Val: 0x23}, {Reg: 0x301b, Val: 0x00}, {Reg: 0x30d5, Val: 0x04},
	}
	// These six DO move with it — they are the sample-size switch. Both sets are the final values
	// the SDK left on the wire, RAW8 from a 320x240 -raw8 capture and RAW16 from a 640x480 one.
	// Applying the RAW8 set to a 16-bit readout leaves the sensor emitting 8-bit data into a
	// 16-bit frame, which reads back as alternating 0x0000/0x0100 samples.
	imx585Mode8POA = []RegVal{
		{Reg: 0x3022, Val: 0x00}, {Reg: 0x3023, Val: 0x00}, {Reg: 0x4231, Val: 0x18},
		{Reg: 0x3930, Val: 0x66}, {Reg: 0x3931, Val: 0x00},
	}
	imx585Mode16POA = []RegVal{
		{Reg: 0x3022, Val: 0x02}, {Reg: 0x3023, Val: 0x01}, {Reg: 0x4231, Val: 0x08},
		{Reg: 0x3930, Val: 0x0c}, {Reg: 0x3931, Val: 0x01},
	}
	// HDR re-tunes the same block: exactly thirteen registers change and none is added or
	// removed. Ten of them sit in the size-independent half, read off the wire from
	// a capture where the SDK programmed Normal RAW8 and HDR RAW16 in one session. 0x301b and 0x30d5 hold their Normal values, so they stay in the common set.
	imx585ModeCommonHDRPOA = []RegVal{
		{Reg: 0x301a, Val: 0x10}, {Reg: 0x3069, Val: 0x02}, {Reg: 0x3074, Val: 0x63},
		{Reg: 0x3a4c, Val: 0x61}, {Reg: 0x3a4d, Val: 0x02}, {Reg: 0x3a50, Val: 0x70},
		{Reg: 0x3a51, Val: 0x02}, {Reg: 0x3e10, Val: 0x17}, {Reg: 0x493c, Val: 0x41},
		{Reg: 0x4940, Val: 0x41}, {Reg: 0x301b, Val: 0x00}, {Reg: 0x30d5, Val: 0x04},
	}
	// The other three of the thirteen — 0x3930, 0x3931 and 0x423d — fall in the sample-size half,
	// which is what makes the block two-dimensional rather than an overlay: HDR cannot be
	// expressed as a patch over a sample size, nor a sample size as a patch over a mode. The
	// remaining three here carry their Normal RAW16 values unchanged.
	imx585Mode16HDRPOA = []RegVal{
		{Reg: 0x3022, Val: 0x02}, {Reg: 0x3023, Val: 0x01}, {Reg: 0x4231, Val: 0x08},
		{Reg: 0x3930, Val: 0xe6}, {Reg: 0x3931, Val: 0x00},
	}
)

// imx585ModePOA is the sensor mode block for a sensor mode and a sample size. Three of the four
// (mode, size) cells are captured; HDR at RAW8 is not, and it is refused rather than filled in
// from a neighbour — the two neighbours disagree on 0x3930, 0x3931 and 0x423d, and picking the
// wrong one leaves the sensor emitting samples the frame layout does not describe.
func imx585ModePOA(mode, bpp int) ([]RegVal, error) {
	switch mode {
	case imx585ModeNormalPOA:
		out := append([]RegVal{}, imx585ModeCommonPOA...)
		if bpp >= 2 {
			return append(out, imx585Mode16POA...), nil
		}
		return append(out, imx585Mode8POA...), nil
	case imx585ModeHDRPOA:
		if bpp < 2 {
			return nil, fmt.Errorf("imx585: HDR is a RAW16 mode; the vendor SDK accepts the combination but programs Normal registers and returns a dead frame")
		}
		return append(append([]RegVal{}, imx585ModeCommonHDRPOA...), imx585Mode16HDRPOA...), nil
	}
	return nil, fmt.Errorf("imx585: sensor mode %d not decoded", mode)
}

// The sensor modes PlayerOne exposes on this die, as POAGetSensorModeInfo reports them.
const (
	imx585ModeNormalPOA = 0
	imx585ModeHDRPOA    = 1
)

// imx585SensorModes reports the mode list per vendor. Only PlayerOne offers the choice: the ZWO
// transcription of this die has one readout programme, which the SDK reports as a mode count of 0.
func imx585SensorModes(vid uint16) []SensorModeInfo {
	if vid != POA.VID {
		return nil
	}
	return []SensorModeInfo{
		{Name: "Normal", Desc: "The default mode, has relatively high frame rate."},
		{Name: "HDR", Desc: "High Dynamic Range mode, Frame rate is lower than Normal mode."},
	}
}

// imx585SetSensorMode writes the sensor half of a mode change. The FPGA half — the crop origin,
// the doubled frame height, the wide flag and the HDR offset — belongs to the window, so SetROI
// programs it from the mode on the ReadoutMode; Camera.SetSensorMode runs both in that order.
func imx585SetSensorMode(rm Regmap, mode int) error {
	if rm.VID() != POA.VID {
		return fmt.Errorf("imx585 sensor mode: unsupported vendor VID 0x%04x", rm.VID())
	}
	block, err := imx585ModePOA(mode, ModeOf(rm).BytesPerPx)
	if err != nil {
		return err
	}
	for _, w := range block {
		if err := rm.WriteReg(w.Reg, w.Val); err != nil {
			return err
		}
	}
	return nil
}

// imx585HDR* are the FPGA HDR defaults the SDK writes at init in BOTH sensor modes; only the
// offset at reg 0x36 is HDR-only, written by SetROI when the mode selects it.
const (
	imx585HDRGainPOA  uint16 = 0xb421
	imx585HDRLimitPOA uint16 = 0x0fe0

	imx585FPGAHDRGain   = 0x32
	imx585FPGAHDRLimit  = 0x34
	imx585FPGAHDROffset = 0x36
)

// imx585LE16 renders a 16-bit value as the little-endian pair an FPGA burst carries.
func imx585LE16(v uint16) []byte { return []byte{byte(v), byte(v >> 8)} }

// imx585InitFPGAPOA is the FPGA-side bringup for PlayerOne, in the SDK's order.
func imx585InitFPGAPOA(rm Regmap) error {
	for _, w := range imx585TailPOA {
		if err := rm.WriteReg(w.Reg, w.Val); err != nil {
			return err
		}
	}
	// White balance off, and the channel gains explicitly at unity. The SDK writes both rather
	// than leaving the gain registers at whatever the FPGA powered up with.
	if err := POAFPGAWhiteBalanceMode(rm, false, false, false); err != nil {
		return err
	}
	if err := POAFPGAWhiteBalance(rm, 0, 0, 0); err != nil {
		return err
	}
	if err := POAFPGABurst(rm, imx585FPGAHDRGain, imx585LE16(imx585HDRGainPOA)); err != nil {
		return err
	}
	if err := POAFPGABurst(rm, imx585FPGAHDRLimit, imx585LE16(imx585HDRLimitPOA)); err != nil {
		return err
	}
	block, err := imx585ModePOA(ModeOf(rm).SensorMode, ModeOf(rm).BytesPerPx)
	if err != nil {
		return err
	}
	for _, w := range block {
		if err := rm.WriteReg(w.Reg, w.Val); err != nil {
			return err
		}
	}
	return nil
}

// imx585InitFPGA dispatches the FPGA bringup per vendor; the two share no registers here.
func imx585InitFPGA(rm Regmap, subtype int) error {
	switch rm.VID() {
	case POA.VID:
		return imx585InitFPGAPOA(rm)
	case ZWO.VID:
		return imx585InitFPGAZWO(rm, subtype)
	}
	return fmt.Errorf("imx585 FPGA bringup: unsupported vendor VID 0x%04x", rm.VID())
}

// imx585InitFPGAZWO is ZWO's bringup after the Sony reglist: FPGA reset, a sleep, master mode,
// stop, DDR enabled (the 585 is a DDR camera), the ADC and output width with bit4 = RAW16 from
// the live ReadoutMode, then the four gain channels at 0x80 (regs 0x0c-0x0f). The SDK's DDR
// self-test has no host-visible state and is omitted. There is no bin-mode write, which the 455
// and 571 do and this die does not.
func imx585InitFPGAZWO(rm Regmap, subtype int) error {
	_ = subtype
	if err := FPGAClearBits(rm, 0x00, 0x01); err != nil { // FPGAReset
		return err
	}
	time.Sleep(20 * time.Millisecond)                   // usleep(0x4e20)
	if err := FPGASetBits(rm, 0x00, 0x20); err != nil { // SetFPGAAsMaster(1)
		return err
	}
	if err := FPGASetBits(rm, 0x00, 0x10); err != nil { // FPGAStop
		return err
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x40, 0x00); err != nil { // EnableFPGADDR(1): bit6 = 0 (DDR)
		return err
	}
	adcOut := uint16(0x01) // bit0 = ADC
	if ModeOf(rm).BytesPerPx >= 2 {
		adcOut |= 0x10 // bit4 = 1 → RAW16
	}
	if err := FPGAWriteBits(rm, 0x0a, 0x11, adcOut); err != nil {
		return err
	}
	if err := rm.WriteFPGAReg(0x01, 1); err != nil {
		return err
	}
	for _, r := range []uint16{0x0c, 0x0d, 0x0e, 0x0f} {
		if err := rm.WriteFPGAReg(r, 0x80); err != nil {
			return err
		}
	}
	return rm.WriteFPGAReg(0x01, 0)
}

// imx585Worker is the host-timed single-shot DDR capture worker (the IMX455/6200 shape). XHSStop
// is FPGA reg 0x0b bit4 (EnableFPGAXHSStop); TriggerSignal is reg 0x0b bit0.
//
//	arm:    SendCMD(0xAA)·FPGAStop·SendCMD(0xA9)·0x3004=0·0x3000=0·10ms·FPGAStart·ResetEndPoint
//	expose: EnableFPGATriggerSignal(1)+EnableFPGAXHSStop(1)·hold for the exposure·
//	        EnableFPGAXHSStop(0)+EnableFPGATriggerSignal(0)
//	read:   one frame with the continuous windowed pump (ctl.StreamFrame), FPGABufReload pulsed
//	        every 20 ms so the FX3 commits the frame's final partial DDR buffer
//	stop:   FPGAStop·0x3000=1·SendCMD(0xAA)·ResetEndPoint
func imx585Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()
	// Halt the readout on every return, the arm's own failures included, as the SDK does on its
	// way out: stop the readout (FPGAStop + standby 0x3000=1), SendCMD(0xAA), ResetEndPoint. A sensor left free-running with no reader backs up the FX3
	// GPIF. Best-effort: a failure here cannot be reported past the frame's own result.
	defer func() {
		_ = ctl.FPGARun(false)               // halt the readout in the vendor's encoding
		_ = rm.WriteReg(imx585RegStandby, 1) // standby (sensor stop)
		_ = ctl.VendorCmd(FX3StreamStop)
		_ = ctl.ResetEndpoint()
	}()
	if err := ctl.VendorCmd(FX3StreamStop); err != nil {
		return 0, err
	}
	if err := ctl.FPGARun(false); err != nil { // halt the readout
		return 0, err
	}
	if err := ctl.VendorCmd(FX3StreamStart); err != nil {
		return 0, err
	}
	if err := imx585StreamStart(rm); err != nil { // 0x3004=0, 0x3000=0
		return 0, err
	}
	time.Sleep(imx585ArmSettle)
	if err := ctl.FPGARun(true); err != nil { // run the readout
		return 0, err
	}
	_ = ctl.ResetEndpoint()

	open := func(on bool) error {
		if rm.VID() == POA.VID {
			// Below a second the sensor holds its own shutter and nothing is gated — the SDK
			// writes no control register at all, and reg 0x0b is not even in its FPGA map.
			if exposure < imx585LongExpUs*time.Microsecond {
				return nil
			}
			// At or above a second the FPGA times the exposure, bracketed by this nested gesture
			// on register 0x06. Read off a 1 s capture: 0x01, 0x03, 0x07 to open and 0x03, 0x01,
			// 0x00 to close. The sensor is parked in low power for the integration.
			seq := []uint16{POACtlExpStart, POACtlExpStart | POACtlDrvStop, POACtlExpStart | POACtlDrvStop | POACtlLowPower}
			if !on {
				seq = []uint16{POACtlExpStart | POACtlDrvStop, POACtlExpStart, 0}
			}
			for _, v := range seq {
				if err := POAFPGAExpControl(rm, v); err != nil {
					return err
				}
			}
			return nil
		}
		if on {
			if err := SetFPGABit(rm, 0x0b, 0x01, true); err != nil { // EnableFPGATriggerSignal(1)
				return err
			}
			return SetFPGABit(rm, 0x0b, 0x10, true) // EnableFPGAXHSStop(1)
		}
		if err := SetFPGABit(rm, 0x0b, 0x10, false); err != nil { // EnableFPGAXHSStop(0)
			return err
		}
		return SetFPGABit(rm, 0x0b, 0x01, false) // EnableFPGATriggerSignal(0)
	}
	if err := open(true); err != nil {
		return 0, err
	}
	// The band split matches imx585SetExposure's inclusive >= 1 s trigger threshold: at 1 s the
	// FPGA is in trigger mode and the host hold is the integration, so it gets the full wait.
	if exposure < time.Second {
		if w := exposure - 200*time.Millisecond; w > 0 {
			time.Sleep(w)
		}
	} else {
		// Poll for the abort at imx585AbortPoll, but never sleep PAST the end of the integration:
		// a flat sleep overshoots by up to one poll interval, which measured as +34 ms on a 5 s
		// exposure and +81 ms on a 10 s one against a vendor overhead that is flat at ~64 ms.
		for start := time.Now(); ; {
			left := exposure - time.Since(start)
			if left <= 0 {
				break
			}
			if ctl.Aborted() {
				// StopExposure ran: drop the trigger window on the way out. open(false)
				// clears both XHSStop (reg 0x0b bit4) and the trigger signal (reg 0x0b
				// bit0); left asserted, the next open(true) is a no-edge write.
				_ = open(false)
				return 0, errExposureAborted
			}
			if left > imx585AbortPoll {
				left = imx585AbortPoll
			}
			time.Sleep(left)
		}
	}
	if err := open(false); err != nil {
		return 0, err
	}

	// Read one whole frame with the continuous windowed pump (DDR / USB3), re-kicking on a stall.
	target := ctl.FrameBytes()
	if target > len(buf) {
		target = len(buf)
	}
	// idle covers the wait for the first chunk (about one frame period); once flowing, chunks
	// are ms apart. In the >= 1 s trigger band the integration completed above, so the read spans
	// only the readout and the timeouts are not exposure-scaled.
	idle := exposure + 2*time.Second
	total := 2*exposure + 5*time.Second
	if exposure >= time.Second {
		idle = 2 * time.Second
		total = 15 * time.Second
	}
	// On USB3 the FX3 commits whole 1-MiB DMA buffers and holds the frame's final partial buffer
	// until it is filled or committed (a 3840×2160×2 frame ends in a partial buffer); pulse
	// FPGABufReload (reg 0x18 bit0) throughout the read so the tail flushes into a posted
	// transfer, as the IMX455 worker does; on a USB2 link the pulses wedge the readout (the 455
	// finding) and the ticker stays off.
	stop := make(chan struct{})
	done := make(chan struct{})
	go func() {
		defer close(done)
		// ZWO only. Register 0x18 is not in PlayerOne's FPGA map — its SDK never writes it — and
		// pulsing an undecoded register 50 times a second through the read terminates the
		// transfer early on SuperSpeed, where the pulses are enabled. PlayerOne's FX3 delivers
		// the exact frame length without a flush.
		if !ModeOf(rm).USB3 || rm.VID() != ZWO.VID {
			<-stop
			return
		}
		t := time.NewTicker(20 * time.Millisecond)
		defer t.Stop()
		for {
			select {
			case <-stop:
				return
			case <-t.C:
				_ = SetFPGABit(rm, 0x18, 0x01, true)
			}
		}
	}()
	n, err := ctl.StreamFrame(buf[:target], idle, total)
	close(stop)
	<-done // no control transfer outlives the worker
	if err == nil && n < target && ctl.Aborted() {
		return n, errExposureAborted // AbortRead: clean abort, not a stall
	}
	return n, err
}

// imx585GpifBwPOA is the GPIF bandwidth divider for a bandwidth percentage (FPGA register 0x28).
// The constant is LINK-DEPENDENT, which is exactly what the structure predicts: the divisor is the
// link rate scaled by the percentage, so a different link means a different numerator. Programming
// USB2's value on a SuperSpeed link sets the GPIF an order of magnitude wrong, and the readout
// fails on the highest sustained rate — die-side binning, where it took the device off the bus.
func imx585GpifBwPOA(fpsPercent int, usb3 bool) uint16 {
	if fpsPercent <= 0 {
		fpsPercent = 100
	}
	c := imx585GpifBwCPOA
	if usb3 {
		c = imx585GpifBwCPOAUSB3
	}
	v := int((c/float64(fpsPercent) - 1) * 256)
	if v < 0 {
		v = 0
	}
	if v > 0xffff {
		v = 0xffff
	}
	return uint16(v)
}

// imx585SHSGuardFor is the shutter guard for a sensor mode: the floor SHS falls to when the
// exposure, rather than the link, sets the frame period.
func imx585SHSGuardFor(hdr bool) int {
	if hdr {
		return imx585SHSGuardHDR
	}
	return imx585SHSGuard
}

// imx585ExpCaps is the advertised exposure range per vendor, as poasnap -caps reports it against
// the decoded ZWO clamps.
func imx585ExpCaps(vid uint16) (minUs, maxUs int64) {
	switch vid {
	case ZWO.VID:
		return imx585ExpMinUs, imx585ExpMaxUs
	case POA.VID:
		return imx585ExpMinUsPOA, imx585ExpMaxUsPOA
	}
	return 0, 0
}

// imx585Presets are the preset operating points, read off a Xena 585M with
// POAGetGainsAndOffsets. Two of them corroborate decodes made elsewhere: the HCG gain is 210,
// the threshold bracketed on the wire, and the lowest-read-noise gain is 500, which is also
// where the SDK clamps the HDR gain. The ZWO path for this die has no preset block decoded.
func imx585Presets(vid uint16) (GainOffsetPresets, bool) {
	if vid != POA.VID {
		return GainOffsetPresets{}, false
	}
	return GainOffsetPresets{
		GainHighestDR: 0, GainHCG: 210, GainUnity: 210, GainLowestRN: 500,
		OffsetHighestDR: 3, OffsetHCG: 6, OffsetUnity: 6, OffsetLowestRN: 120,
	}, true
}
