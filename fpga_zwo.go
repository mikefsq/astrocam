package astrocam

// ZWO camera-FPGA registers. The Sony dies are shared with PlayerOne and their register
// semantics carry over; the FPGA is vendor firmware and none of it does. PlayerOne's mirror of
// this file is fpga_poa.go, which lists the three registers whose numbers collide with a
// different actuator behind them.

// FPGA mode register 0: bit4 stops the readout pipeline (FPGAStop sets it, FPGAStart clears it).
const (
	fpgaModeReg0 = 0x00
	fpgaStopBit  = 0x10
)

// zwoFPGARun is ZWO's readout run control: bit 4 of FPGA register 0 is the STOP flag, set to
// halt and cleared to run, read-modify-written so the rest of the mode byte survives.
func zwoFPGARun(rm Regmap, start bool) error {
	return SetFPGABit(rm, fpgaModeReg0, fpgaStopBit, !start)
}

// FPGA register numbers (the wValue passed to WriteFPGAREG 0xBD). Shared by every ZWO camera,
// not per-sensor: PlayerOne packs the same quantities into different registers and burst writes
// (poaFPGASize, poaFPGACrop, poaFPGADrive), so nothing below carries over.
const (
	fpgaStrobe  = 0x01 // reg-1 commit strobe (1 before a group, 0 after); all setters
	fpgaHBLK0   = 0x02 // SetFPGAHBLK  lo
	fpgaHBLK1   = 0x03 // SetFPGAHBLK  hi
	fpgaWidth0  = 0x04 // SetFPGAWidth lo
	fpgaWidth1  = 0x05 // SetFPGAWidth hi
	fpgaVBLK0   = 0x06 // SetFPGAVBLK  lo
	fpgaVBLK1   = 0x07 // SetFPGAVBLK  hi
	fpgaHeight0 = 0x08 // SetFPGAHeight lo
	fpgaHeight1 = 0x09 // SetFPGAHeight hi
	fpgaHMAX0   = 0x13 // SetFPGAHMAX  lo
	fpgaHMAX1   = 0x14 // SetFPGAHMAX  hi
)

// FPGAWrite16 writes a 16-bit value little-endian to an FPGA register pair, bracketed by the
// reg-1 commit strobe (1 then 0), the form every SetFPGA{HBLK,VBLK,Width,Height,HMAX} setter
// takes. The strobe is released even when a data write errors (a held strobe gates every later
// FPGA group commit); the first error wins.
func FPGAWrite16(rm Regmap, loReg, hiReg, val uint16) (err error) {
	if err = rm.WriteFPGAReg(fpgaStrobe, 1); err != nil {
		return err
	}
	defer func() {
		if rerr := rm.WriteFPGAReg(fpgaStrobe, 0); err == nil {
			err = rerr
		}
	}()
	if err = rm.WriteFPGAReg(loReg, val&0xff); err != nil {
		return err
	}
	return rm.WriteFPGAReg(hiReg, (val>>8)&0xff)
}

// SetFPGAHBLK / SetFPGAVBLK program the FX3 optical-black crop: the count of leading blank /
// optical-black columns (HBLK → FPGA 0x02/0x03) and rows (VBLK → 0x06/0x07) windowed out
// before the active image. Values are sensor-specific (from the profile).
func SetFPGAHBLK(rm Regmap, hblk uint16) error { return FPGAWrite16(rm, fpgaHBLK0, fpgaHBLK1, hblk) }
func SetFPGAVBLK(rm Regmap, vblk uint16) error { return FPGAWrite16(rm, fpgaVBLK0, fpgaVBLK1, vblk) }

// SetFPGAOutputWidth programs the FX3 output bit width: FPGA reg 0xa bit4 = output width
// (1 = 16-bit RAW16, 0 = 8-bit RAW8). Bit0 (ADC_BIT, the readout ADC width) belongs to the
// sensor profile (its InitFPGA/SetROI choose it per readout mode) and is left untouched.
func SetFPGAOutputWidth(rm Regmap, raw16 bool) error {
	v := uint16(0)
	if raw16 {
		v = 0x10 // bit4 = 1 → 16-bit output
	}
	return FPGAWriteBits(rm, 0x0a, 0x10, v)
}

// SetFPGABinDataLen programs the per-frame DMA word count: a 32-bit little-endian value to FPGA
// regs 0x40..0x43, bracketed by the reg-1 commit strobe (1 then 0). dataWords = frameBytes/4.
// IMX455/IMX571 program it so the FX3 frames the exact (binned / sub-frame) transfer; the STARVIS
// 290 does not use it.
func SetFPGABinDataLen(rm Regmap, dataWords uint32) (err error) {
	if err = rm.WriteFPGAReg(fpgaStrobe, 1); err != nil {
		return err
	}
	defer func() { // release the strobe on the error paths too; the first error wins
		if rerr := rm.WriteFPGAReg(fpgaStrobe, 0); err == nil {
			err = rerr
		}
	}()
	for i, reg := range []uint16{0x40, 0x41, 0x42, 0x43} {
		if err = rm.WriteFPGAReg(reg, uint16(dataWords>>(8*uint(i)))&0xff); err != nil {
			return err
		}
	}
	return nil
}

// ProgramFrameGeometry writes the FPGA frame dimensions the FX3 needs to package the bulk
// stream: HBLK, width, VBLK, height. hblk/vblk are sensor-specific blanking values.
func ProgramFrameGeometry(rm Regmap, w, h, hblk, vblk int) error {
	for _, g := range []struct {
		lo, hi uint16
		val    int
	}{
		{fpgaHBLK0, fpgaHBLK1, hblk},
		{fpgaWidth0, fpgaWidth1, w},
		{fpgaVBLK0, fpgaVBLK1, vblk},
		{fpgaHeight0, fpgaHeight1, h},
	} {
		if err := FPGAWrite16(rm, g.lo, g.hi, uint16(g.val)); err != nil {
			return err
		}
	}
	return nil
}

// ProgramHMAX computes the readout line period for the window from the live ReadoutMode and
// writes it to the FPGA (SetFPGAHMAX 0x13/0x14, strobed). clock/floor/vblankAdd are the sensor's.
func ProgramHMAX(rm Regmap, w, h, clock, floor, vblankAdd int) error {
	hm := HMAX(w, h, clock, floor, vblankAdd, ModeOf(rm))
	return FPGAWrite16(rm, fpgaHMAX0, fpgaHMAX1, hm)
}

// WriteFPGAHMAX writes a constant HMAX line period to the FPGA (0x13/0x14, strobed), for sensors
// whose HMAX is a baked constant (e.g. IMX178's ctor HMAX=0x1a4).
func WriteFPGAHMAX(rm Regmap, hmax uint16) error {
	return FPGAWrite16(rm, fpgaHMAX0, fpgaHMAX1, hmax)
}
