package astrocam

import (
	"fmt"
	"math"
)

// PlayerOne camera-FPGA register map. The Sony dies are shared with ZWO and their register
// semantics carry over; the FPGA is vendor firmware and NONE of it does. Three registers overlap ZWO's with a different actuator behind
// them, so a ZWO write reaching a PlayerOne body is not merely inert:
//
//	reg 0x00 bit4  ZWO: stop the readout      PlayerOne: part of the START value
//	reg 0x26       ZWO: TEC cooling power     PlayerOne: window heater (TEC power is 0x25)
//	reg 0x2a       ZWO: anti-dew heater PWM   PlayerOne: status, read-only
//
// The vendor is fixed at Open from the USB (VID,PID), so the two paths never meet on one camera;
// these entry points exist so PlayerOne code never has to reach for ZWO's numbers.
const (
	poaFPGARun    = 0x00 // run control (whole-byte values, not a bit RMW)
	poaFPGALatch  = 0x01 // group latch: 1 before a burst group, 0 after
	poaFPGAFormat = 0x02 // (bpp16 ? 0x80 : 0) | format
	poaFPGAExpMod = 0x03 // exposure mode
	poaFPGAMode   = 0x04 // (flag ? 0x80 : 0) | bin; bit4 = binned-output mode, see poaFPGABinSum
	poaFPGACtl    = 0x06 // b0 exposure-control start, b1 drive stop, b2 low power, b3 reconnect
	poaFPGALoad   = 0x07 // write 1 to start an FPGA load; read bit4 for done
	poaFPGACrop   = 0x08 // burst 4 B: [x u16][y u16]
	poaFPGASize   = 0x0c // burst 8 B: [w u16][h u16][dmaWords u32]
	poaFPGADrive  = 0x14 // burst 5 B: [u16][u24] sensor line/frame period pair
	poaFPGAExp    = 0x20 // burst 4 B: u32 = exposure / 6.4
	poaFPGAPause  = 0x24 // GPIF pause
	poaFPGAWBMode = 0x19 // white-balance mode byte
	poaFPGAWB     = 0x1a // burst 6 B: three Q14 channel gains
	poaFPGABw     = 0x28 // burst 2 B: u16 GPIF bandwidth
	poaFPGAFlag38 = 0x38 // u8 flag written alongside the image size
)

// Run-control values for poaFPGARun. These are whole-byte writes: PlayerOne does not
// read-modify-write this register the way ZWO's bit4 stop does.
const (
	poaRunStop     = 0x00 // halt the readout
	poaRunReset    = 0x01 // reset the FPGA
	poaRunSenReset = 0x04 // assert the sensor reset line; releasing it writes poaRunStop
	poaRunStart    = 0x10 // run the readout
)

// poaExpTick is the divisor applied before writing poaFPGAExp: the register holds exposure/6.4
// in the SDK's units.
const poaExpTick = 6.4

// FPGABurstWriter is a Regmap that can load several consecutive FPGA registers from one transfer.
// PlayerOne needs it — its image-size, drive, crop, exposure and bandwidth registers are
// burst-only (the 0xC1 request) — while ZWO's dialect has no such request, so the capability is
// an optional interface asserted at the call site rather than a Regmap method every dialect must
// answer.
type FPGABurstWriter interface {
	WriteFPGABurst(reg uint16, data []byte) error
}

// POAFPGAStart / POAFPGAStop drive the readout pipeline. Note the polarity against ZWO, where
// the same 0x10 is the STOP bit.
func POAFPGAStart(rm Regmap) error { return rm.WriteFPGAReg(poaFPGARun, poaRunStart) }
func POAFPGAStop(rm Regmap) error  { return rm.WriteFPGAReg(poaFPGARun, poaRunStop) }

// POAFPGAReset resets the FPGA. The SDK follows it with a sleep and a sensor-reset pulse; see
// POAFPGASensorReset.
func POAFPGAReset(rm Regmap) error { return rm.WriteFPGAReg(poaFPGARun, poaRunReset) }

// POAFPGASensorReset asserts or releases the sensor reset line. Releasing writes the same value
// as a stop, which is what the SDK does.
func POAFPGASensorReset(rm Regmap, on bool) error {
	v := uint16(poaRunStop)
	if on {
		v = poaRunSenReset
	}
	return rm.WriteFPGAReg(poaFPGARun, v)
}

// POAFPGABurst writes data to consecutive FPGA registers starting at reg, bracketed by the
// group latch on poaFPGALatch, the way the SDK brackets every burst. The latch is released even
// when the payload write fails, so a failed group cannot leave the FPGA latched.
func POAFPGABurst(rm Regmap, reg uint16, data []byte) (err error) {
	bw, ok := rm.(FPGABurstWriter)
	if !ok {
		return fmt.Errorf("astrocam: FPGA burst write to reg 0x%02x needs a burst-capable regmap, got %T", reg, rm)
	}
	if err := rm.WriteFPGAReg(poaFPGALatch, 1); err != nil {
		return err
	}
	defer func() {
		if rerr := rm.WriteFPGAReg(poaFPGALatch, 0); err == nil {
			err = rerr
		}
	}()
	return bw.WriteFPGABurst(reg, data)
}

// POAFPGAImageSize programs the readout geometry: the format byte, the mode
// byte, the flag at 0x38, then an 8-byte burst of [w u16][h u16][dmaWords u32].
//
// dmaWords is the per-frame count the FX3 expects, and the SDK derives it rather than taking it
// from the caller; see POAFPGADMAWords. bin is the SDK's encoding, one less than the SOFTWARE
// binning factor, so bin 0 means no division.
//
// format lands in the low bits of register 0x02 beside the 16-bit flag. The SDK derives it from
// a value that tracks the HARDWARE bin factor, but a capture matrix over eight configurations
// disagrees: register 0x02 was 0x81 for every RAW16 frame and 0x00 for every RAW8
// one, including a hardware bin 2 frame that would have had to read 0x82. Whatever else that
// member feeds, on the wire this byte tracks the sample size alone — so callers pass 1 with
// bpp16 and 0 without.
//
// w and h are the geometry the FPGA reads out, which is not the host frame when software binning
// is in play: the capture shows a software bin 2 programming the full 3856x2180 with bin 1, and a
// hardware bin 2 programming the binned 1928x1090 with bin 0. Both produce the same dmaWords.
func POAFPGAImageSize(rm Regmap, w, h int, bpp16 bool, format uint8, flag bool, bin uint8, wide, sum bool) error {
	fb := uint16(format)
	if bpp16 {
		fb |= 0x80
	}
	if err := rm.WriteFPGAReg(poaFPGAFormat, fb); err != nil {
		return err
	}
	mb := uint16(bin)
	if flag {
		mb |= 0x80
	}
	if sum {
		mb |= poaFPGABinSum
	}
	if err := rm.WriteFPGAReg(poaFPGAMode, mb); err != nil {
		return err
	}
	f38 := uint16(0)
	if wide {
		f38 = 1
	}
	if err := rm.WriteFPGAReg(poaFPGAFlag38, f38); err != nil {
		return err
	}
	n := POAFPGADMAWords(w, h, bpp16, bin, wide)
	buf := []byte{
		byte(w), byte(w >> 8),
		byte(h), byte(h >> 8),
		byte(n), byte(n >> 8), byte(n >> 16), byte(n >> 24),
	}
	return POAFPGABurst(rm, poaFPGASize, buf)
}

// POAFPGADMAWords reproduces the SDK's frame-length arithmetic: pixels, scaled by the sample
// size, shifted down by the transfer word width, then divided by (bin+1) once per axis. The two
// divisions stay separate to mirror the SDK, not because the result differs from dividing by the
// square.
func POAFPGADMAWords(w, h int, bpp16 bool, bin uint8, wide bool) uint32 {
	n := uint32(w) * uint32(h)
	if bpp16 {
		n *= 2
	}
	if wide {
		n >>= 3
	} else {
		n >>= 2
	}
	if bin != 0 {
		d := uint32(bin) + 1
		n /= d
		n /= d
	}
	return n
}

// POAFPGADrive writes the sensor drive pair: a 16-bit value and a 24-bit value,
// little-endian, as one 5-byte burst. Values wider than their field are rejected, as the SDK
// rejects them.
func POAFPGADrive(rm Regmap, a uint16, b uint32) error {
	if b>>24 != 0 {
		return fmt.Errorf("astrocam: POA FPGA drive: 0x%x exceeds the 24-bit field", b)
	}
	return POAFPGABurst(rm, poaFPGADrive, []byte{
		byte(a), byte(a >> 8),
		byte(b), byte(b >> 8), byte(b >> 16),
	})
}

// POAFPGACropOrigin sets the readout crop origin, the rows and columns the FPGA
// drops before the frame the host receives.
func POAFPGACropOrigin(rm Regmap, x, y uint16) error {
	return POAFPGABurst(rm, poaFPGACrop, []byte{
		byte(x), byte(x >> 8),
		byte(y), byte(y >> 8),
	})
}

// POAFPGAExposure writes the FPGA exposure timer. us is the exposure in the
// SDK's argument units; the register holds us/6.4, truncated.
func POAFPGAExposure(rm Regmap, us uint64) error {
	ticks := uint32(float64(us) / poaExpTick)
	return POAFPGABurst(rm, poaFPGAExp, []byte{
		byte(ticks), byte(ticks >> 8), byte(ticks >> 16), byte(ticks >> 24),
	})
}

// poaWBUnit is the divisor in the white-balance gain curve: the SDK maps a channel's WB value v
// (the POA_WB_R/G/B config range is -1200..1200) to 10^(v/2000), then stores it in Q14.
const poaWBUnit = 2000

// POAFPGAWhiteBalanceGain converts one channel's WB value to the register's Q14 form. v = 0 is
// unity and yields 0x4000, which is what the camera holds after init.
func POAFPGAWhiteBalanceGain(v int) uint16 {
	return uint16(math.Pow(10, float64(v)/poaWBUnit) * (1 << 14))
}

// POAFPGAWhiteBalance writes the three channel gains as one 6-byte burst of
// little-endian Q14 values to register 0x1a.
func POAFPGAWhiteBalance(rm Regmap, r, g, b int) error {
	buf := make([]byte, 0, 6)
	for _, v := range []int{r, g, b} {
		q := POAFPGAWhiteBalanceGain(v)
		buf = append(buf, byte(q), byte(q>>8))
	}
	return POAFPGABurst(rm, poaFPGAWB, buf)
}

// POAFPGAWhiteBalanceMode writes the white-balance mode byte (register 0x19). The SDK folds
// three arguments into bit 0, bit 1 and bit 4; their meanings are not decoded, so they pass
// through as raw flags. Camera init sends all three clear on a mono body.
func POAFPGAWhiteBalanceMode(rm Regmap, bit0, bit1, bit4 bool) error {
	v := uint16(0)
	if bit0 {
		v |= 0x01
	}
	if bit1 {
		v |= 0x02
	}
	if bit4 {
		v |= 0x10
	}
	return rm.WriteFPGAReg(poaFPGAWBMode, v)
}

// poaFPGABinSum is bit 4 of the readout mode byte: set, binned pixels are SUMMED; clear, they are
// AVERAGED. It is what the SDK exposes as POA_PIXEL_BIN_SUM, and it matters
// for photometry — summing preserves total signal, averaging preserves the level. The SDK
// defaults it clear, which is what POAFPGAImageSize writes.
const poaFPGABinSum = 0x10

// PlayerOne's control-register bits (FPGA 0x06). The SDK sets each one through its own call
// over a host-side shadow of the byte.
const (
	POACtlExpStart = 0x01
	POACtlDrvStop  = 0x02
	POACtlLowPower = 0x04
	POACtlReConn   = 0x08
)

// POAFPGAExpControl writes the control byte at register 0x06. A long exposure is bracketed by a
// nested gesture on these bits — start the exposure controller, stop the sensor drive, drop the
// sensor to low power for the integration, then unwind in reverse — which is how the FPGA, rather
// than the sensor, holds a multi-second shutter open.
func POAFPGAExpControl(rm Regmap, bits uint16) error {
	return rm.WriteFPGAReg(poaFPGACtl, bits)
}

// POAFPGAExposureMode writes the exposure-mode byte (register 0x03). Every
// capture of a normal single-frame or video exposure wrote 0; the flag bits are undecoded.
func POAFPGAExposureMode(rm Regmap, mode uint16) error {
	return rm.WriteFPGAReg(poaFPGAExpMod, mode)
}

// POAFPGABandwidth writes the GPIF bandwidth limit.
func POAFPGABandwidth(rm Regmap, bw uint16) error {
	return POAFPGABurst(rm, poaFPGABw, []byte{byte(bw), byte(bw >> 8)})
}
