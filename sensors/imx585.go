// IMX585 (Type 1/1.2, 8.3 MP, STARVIS 2). Own register map with a STARVIS-2 dual-gain: gain
// registers 0x3030 (conv-gain select) + 0x306c/0x306d (analog code), offset 0x30dc/0x30dd, SHS
// 0x3050-52, ROI in the 0x303c-47 block, latch 0x3001. DDR-buffered USB3 camera (FPGADDRTest +
// EnableFPGADDR at bringup, SetFPGABinDataLen + windowed startAsyncXfer for capture), so the
// readout path mirrors the IMX455/6200 rather than the 290.
//
// Register/behavior summary:
//
//	ctor              (geometry 3840x2160 (lit16 0x3440); clock 20000; HMAX 192; the SDK's FPS%-default 80 is not applied here)
//	InitCamera        (reglist = 226 records [reg:u16le][val:u16le], size 0x388;
//	                   tail 0x3015/0x3002/0x3018/0x301b/0x3022/0x3023; FPGAReset + DDR bringup)
//	SetCMOSClk        (clock 20000 kHz; no static HMAX floor → HMAX is baked 192)
//	Cam_SetResolution (window X 0x303e/3f, Y 0x3046/47; SetFPGABinDataLen + SetFPGAWidth/Height)
//	SetStartPos       (0x3018=0x14; ROI X 0x303c/3d align 2, Y 0x3044/45 align 4)
//	SetGain           (clamp 0..600; HCG when gain>199 → conv 0x3030=1 and eff=gain-150; LCG eff=gain;
//	                   code = eff/3 (eff*0xaaab>>17) → 0x306c/0x306d 16-bit LE; latch 0x3001)
//	SetExp            (SHS 24-bit 0x3050-52; line_time=HMAX*1000/clock; VMAX=height+2;
//	                   SHS=clamp((VMAX-8)-lines, 8, VMAX-8); ≥1 s → FPGA trigger mode)
//	SetBrightness     (offset -> 0x30dc (low) / 0x30dd (high), 16-bit LE; latch 0x3001)
//	Start/StopSensorStreaming (start: 0x3004=0, 0x3000=0, FPGAStart; stop: FPGAStop, 0x3000=1)
//	the capture worker (DDR capture: arm, TriggerSignal+XHSStop window, windowed xfer)

package sensors

import . "github.com/mikefsq/astrocam"

import (
	"fmt"
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
	imx585RegHeightH = 0x3047
	imx585RegWinMode = 0x3018 // window/mode byte, full-res value 0x14

	imx585GainMax   = 600 // 60.0 dB ceiling, ASI 0.1 dB units (SetGain clamp)
	imx585GainHCGAt = 199 // ABOVE this (gain > 199) → HCG: conv 0x3030=1
	imx585HCGSub    = 150 // HCG analog code uses (gain - 150)

	imx585ExpMinUs  = 32            // µs floor (clamp 32)
	imx585ExpMaxUs  = 2_000_000_000 // 2000 s ceiling
	imx585LongExpUs = 1_000_000     // ≥ 1 s enters FPGA trigger mode — EXACT

	// Die/mode readout facts (shared engine: fps.go / shutter.go). Geometry is image-orientation;
	// effective 4K (3840×2160).
	imx585FullWidth  = 3840  // horizontal (lit16 0x3440)
	imx585FullHeight = 2160  // vertical — the line count VMAX = height + 2 uses
	imx585ClkKHz     = 20000 // 20 MHz
	imx585HMAX       = 192   // baked line-period HMAX, fixed post-init; line_time = 192·1e6/20000 = 9600 ns
	imx585VBlankAdd  = 2     // VMAX = height + 2 (the 60 used for the Sony output-height registers
	//                     0x3046/0x3047 in Cam_SetResolution is NOT the VMAX addend). Affects the
	//                     frame period, not the integration time.
	imx585SHSGuard = 8 // SHS = clamp((VMAX-8) − lines, 8, VMAX-8) (the -8 / floor 8)
)

// imx585Init — the InitCamera reglist: 226 records, [reg:u16le][val:u16le], reg 0xffff = delay ms.
// The big STARVIS-2 analog tuning table. (The InitCamera tail 0x3015/0x3002/0x3018/0x301b/0x3022/0x3023
// + the FPGA bringup are in imx585InitFPGA / Camera.Init.)
var imx585Init = []RegVal{
	{Reg: 0x3001, Val: 0x01}, // latch on — InitCamera brackets the whole reglist+tail
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
	// --- explicit tail (InitCamera), still inside the latch group ---
	{Reg: 0x3015, Val: 0x07}, {Reg: 0x3002, Val: 0x01}, {Reg: 0x3018, Val: 0x14},
	{Reg: 0x301b, Val: 0x00}, {Reg: 0x3022, Val: 0x02}, {Reg: 0x3023, Val: 0x01},
	{Reg: 0x3001, Val: 0x00}, // latch off; FPGAReset + SendCMD(0xAF) follow in imx585InitFPGA / Camera.Init
}

// IMX585 is the Sony IMX585 STARVIS 2 profile (ZWO ASI585, PlayerOne Uranus). Not yet hardware-validated.
var IMX585 = Sensor{
	Name:      "IMX585", // ASI585 / Xena 585M; Sony IMX585 STARVIS 2 (mono die; MC adds a CFA)
	GainMax:   imx585GainMax,
	ExpMinUs:  imx585ExpMinUs,
	ExpMaxUs:  imx585ExpMaxUs,
	OffsetMax: 240, OffsetDef: 16, // ASI Brightness range (family default)
	Info: CameraInfo{
		MaxWidth:  imx585FullWidth,
		MaxHeight: imx585FullHeight,
		PixelUm:   2.9,      // 2.9 µm pixel pitch
		BitDepth:  12,       // 12-bit ADC (RAW16 transport)
		Bayer:     "RGGB",   // CFA (color/MC only); surfaced when Model.Color
		Bins:      []int{1}, // bin modes (InitSensorMode 0x30d5 branches) not yet decoded → SetROI errors
	},
	Init:        imx585Init,
	InitFPGA:    imx585InitFPGA,
	SetGain:     imx585SetGain,
	SetExposure: imx585SetExposure,
	SetOffset:   imx585SetOffset,
	GetOffset:   imx585GetOffset,
	SetROI:      imx585SetROI,
	StreamStop:  func(rm Regmap) error { return rm.WriteReg(imx585RegStandby, 1) }, // standby on (StopSensorStreaming)
	StreamStart: imx585StreamStart,
	Worker:      imx585Worker,
}

// imx585StreamStart — StartSensorStreaming: clear the master-start gate (0x3004=0) then release
// standby (0x3000=0). The capture framework brackets this with FPGAStop/Start and the 0xAA/0xA9 vendor cmds.
func imx585StreamStart(rm Regmap) error {
	if err := rm.WriteReg(imx585RegXmsta, 0); err != nil {
		return err
	}
	return rm.WriteReg(imx585RegStandby, 0)
}

// imx585SetGain — STARVIS-2 dual gain: clamp [0,600]; above gain 199 the high conversion gain engages
// (0x3030 = 1) and the analog code is computed on (gain-150); at/below 199 it is LCG (0x3030 = 0) on the
// raw gain. The analog code = eff/3 (eff*0xaaab>>17), written 16-bit LE to 0x306c/0x306d. Bracketed by
// the 0x3001 latch.
func imx585SetGain(rm Regmap, gain int) error {
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

// imx585SetExposure — STARVIS-2 rolling shutter: line_time = HMAX·1000/clock (HMAX baked at 192),
// VMAX = height + 2, SHS = clamp((VMAX-8) − lines, 8, VMAX-8) to the 24-bit 0x3050-52 (bracketed by
// 0x3001), VMAX to the FPGA (SetFPGAVMAX). Exposures ≥ 1 s engage FPGA trigger mode (reg0 bit7) and the
// worker host-times them.
func imx585SetExposure(rm Regmap, d time.Duration) error {
	trigger := d >= imx585LongExpUs*time.Microsecond
	// Engage both FPGA flags for the ≥1 s band and clear both below: reg0 bit6 (EnableFPGAWaitMode)
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

	vmax := uint64(imx585FullHeight) + imx585VBlankAdd // 2220
	hi := vmax - imx585SHSGuard                        // SHS base / ceiling = VMAX-8 = 2212
	if lines+1 > hi {                                  // exposure exceeds the in-frame window → stretch VMAX
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

// imx585SetOffset — SetBrightness: offset 16-bit LE to 0x30dc (low) / 0x30dd (high), bracketed by
// the 0x3001 latch.
// imx585GetOffset reads the offset back from 0x30dc (low) / 0x30dd (high).
func imx585GetOffset(rm Regmap) (int, error) {
	v, err := ReadRegLE(rm, []uint16{imx585RegOffsetL, imx585RegOffsetH})
	return int(v), err
}

func imx585SetOffset(rm Regmap, offset int) error {
	return WriteRegLE(rm, imx585RegLatch, []uint16{imx585RegOffsetL, imx585RegOffsetH}, uint32(uint16(offset)))
}

// imx585SetROI — SetStartPos (0x3018=0x14; ROI X→0x303c/3d align 2, Y→0x3044/45 align 4) +
// Cam_SetResolution (window W→0x303e/3f, H→0x3046/47; SetFPGABinDataLen + SetFPGAWidth/Height).
// Only bin 1 — the binned modes (InitSensorMode 0x30d5 branches) are not yet decoded.
func imx585SetROI(rm Regmap, x, y, w, h, bin int) error {
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
	// Window sizes carry margins (Cam_SetResolution): width rounded up to 16,
	// height rounded up to 4 then +2 dummy lines.
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

	// FPGA frame geometry for the FX3 DDR transfer (Cam_SetResolution): the per-frame DMA word count,
	// then output width/height. HMAX is the baked 192 (DDR branch writes it straight to the FPGA HMAX
	// register, like the IMX455/6200 — not the bandwidth throttle).
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

// imx585InitFPGA — the FPGA bringup after the Sony reglist (InitCamera).
// Sequence: FPGAReset, usleep, SendCMD(0xAF) [Camera.Init], FPGADDRTest, SetFPGAAsMaster(1),
// FPGAStop, EnableFPGADDR(1) (585 is DDR), SetFPGAADCWidthOutputWidth(adc, outputWidth),
// SetFPGAGain(0x80×4 → 0x0c-0x0f). No SetFPGABinMode here (that is IMX455-only). FPGADDRTest is a
// DDR self-test with no host-visible state — omitted. outputWidth = 0 (8-bit) at init; bit4 is
// raised for RAW16 from the live ReadoutMode.
func imx585InitFPGA(rm Regmap, subtype int) error {
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
	if err := FPGAWriteBits(rm, 0x0a, 0x40, 0x00); err != nil { // EnableFPGADDR(1): bit6 = 0 (DDR on)
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

// imx585Worker — the capture worker, host-timed single-shot DDR capture. Same skeleton as
// the IMX455/6200 (DDR + windowed startAsyncXfer): arm (SendCMD 0xAA + FPGAStop, SendCMD 0xA9, stream the
// sensor, FPGAStart, ResetEndPoint), open the FPGA exposure window with EnableFPGATriggerSignal(1) +
// EnableFPGAXHSStop(1), hold for the exposure, close it, then read one frame with the continuous windowed
// pump. XHSStop is FPGA reg0a bit4; TriggerSignal is reg0b bit0.
func imx585Worker(ctl WorkerCtl, buf []byte, exposure time.Duration) (int, error) {
	rm := ctl.Rm()
	if err := ctl.VendorCmd(FX3StreamStop); err != nil {
		return 0, err
	}
	if err := SetFPGABit(rm, 0x00, 0x10, true); err != nil { // FPGAStop
		return 0, err
	}
	if err := ctl.VendorCmd(FX3StreamStart); err != nil {
		return 0, err
	}
	if err := imx585StreamStart(rm); err != nil { // 0x3004=0, 0x3000=0
		return 0, err
	}
	time.Sleep(10 * time.Millisecond)
	if err := SetFPGABit(rm, 0x00, 0x10, false); err != nil { // FPGAStart
		return 0, err
	}
	// Halt the readout on EVERY return — the 585 object's WorkingFunc exit:
	// StopSensorStreaming (FPGAStop + standby 0x3000=1, per 0104_CCameraS585MC.o) then
	// SendCMD(0xAA) then ResetEndPoint. A sensor left free-running with no reader backs up
	// the FX3 GPIF. Best-effort. VERIFY-HW: ported from the object; no 585 on the bench at
	// port time.
	defer func() {
		_ = SetFPGABit(rm, 0x00, 0x10, true) // FPGAStop: reg0 bit4
		_ = rm.WriteReg(imx585RegStandby, 1) // standby (sensor stop)
		_ = ctl.VendorCmd(FX3StreamStop)
		_ = ctl.ResetEndpoint()
	}()
	_ = ctl.ResetEndpoint()

	open := func(on bool) error {
		if on {
			if err := SetFPGABit(rm, 0x0b, 0x01, true); err != nil { // EnableFPGATriggerSignal(1)
				return err
			}
			return SetFPGABit(rm, 0x0a, 0x10, true) // EnableFPGAXHSStop(1)
		}
		if err := SetFPGABit(rm, 0x0a, 0x10, false); err != nil { // EnableFPGAXHSStop(0)
			return err
		}
		return SetFPGABit(rm, 0x0b, 0x01, false) // EnableFPGATriggerSignal(0)
	}
	if err := open(true); err != nil {
		return 0, err
	}
	// Band split must match imx585SetExposure's trigger threshold (>= 1 s, EXACT): at exactly
	// 1 s the FPGA is in trigger mode and the host hold IS the integration, so it gets the
	// full-exposure wait — `<=` here under-integrated a 1.000 s exposure by 200 ms.
	if exposure < time.Second {
		if w := exposure - 200*time.Millisecond; w > 0 {
			time.Sleep(w)
		}
	} else {
		for start := time.Now(); time.Since(start) < exposure; {
			if ctl.Aborted() {
				// StopExposure ran: bail, dropping the trigger window on the way out —
				// open(false) clears BOTH XHSStop (reg 0x0a bit4) and the trigger signal
				// (reg 0x0b bit0); left asserted, the next open(true) is a no-edge write.
				// (Host-side hygiene, not object-derived: the SDK snap thread never
				// host-aborts mid-integration.)
				_ = open(false)
				return 0, errExposureAborted
			}
			time.Sleep(100 * time.Millisecond)
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
	n, err := ctl.StreamFrame(buf[:target], 500*time.Millisecond, exposure+5*time.Second)
	if err == nil && n < target && ctl.Aborted() {
		return n, errExposureAborted // StopExposure broke the read (AbortRead): clean abort, not a stall
	}
	return n, err
}
