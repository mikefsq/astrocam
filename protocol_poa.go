package astrocam

import "fmt"

// PlayerOne control-transfer opcodes. Each is a
// vendor control transfer via UsbCmd(bRequest, wValue, wIndex, isRead, buf, len)
// — the same FX3 bridge as ZWO, a different host dialect.
//
// CRITICAL: PlayerOne's argument order is the OPPOSITE of ZWO's. ZWO's WriteSONYREG puts
// the register in wValue and the value in wIndex; PlayerOne's Fx3ImgSenWrite puts the
// VALUE in wValue and the REGISTER in wIndex. Transport.ControlOut/In is generic
// (bRequest, wValue, wIndex, data), so this file just supplies PlayerOne's order.
//
//	Fx3ImgSenWrite(reg,val)      OUT bReq 0xB0  wValue=val wIndex=reg
//	Fx3ImgSenRead(reg)           IN  bReq 0xB2  wValue=0   wIndex=reg
//	Fx3ImgSenCrypWrite(reg,val)  OUT bReq 0xB3  wValue=val wIndex=(reg+0xABCD)&0xffff
//	Fx3FpgaWrite(reg,val)        OUT bReq 0xC0  wValue=val wIndex=reg
//	Fx3FpgaRead(reg)             IN  bReq 0xC2  wValue=0   wIndex=reg
const (
	poaReqImgSenWrite = 0xB0 // sensor register write (8-bit value)
	poaReqImgSenRead  = 0xB2 // sensor register read (IN)
	poaReqImgSenCryp  = 0xB3 // obfuscated write for protected registers (see poaCrypRegBias)
	poaReqFpgaWrite   = 0xC0 // camera-FPGA register write (8-bit value)
	poaReqFpgaRead    = 0xC2 // camera-FPGA register read (IN)

	// poaCrypRegBias is the additive obfuscation PlayerOne applies to a protected
	// register address before CrypWrite: wire_reg = (reg + 0xABCD) & 0xffff. Equivalently
	// it adds -0x5433 (= +0xABCD mod 2^16); it is NOT encryption, just a fixed offset.
	poaCrypRegBias uint16 = 0xABCD
)

// poaProtectedRegs are the registers PlayerOne writes via the obfuscated CrypWrite path
// instead of a plain sensor write, per CamGainSet — the
// only protected register on either Sony die is the gain-setup 0x67f (four CrypWrite
// call sites each, all reg 0x67f). Shared across PlayerOne's Sony dies; extend per-sensor
// if new CrypWrite sites are found. Read-only after init.
var poaProtectedRegs = map[uint16]bool{0x67f: true}

// poaRegmap implements Regmap over a Transport using PlayerOne's control-transfer
// protocol. The Sony dies (IMX455/571/…) and their register semantics are identical to
// ZWO's — so the sensor profiles are shared unchanged; only this wire dialect differs.
// protected routes the gain-setup register through CrypWrite automatically, so a profile
// can call WriteReg(0x67f, …) without knowing the vendor obfuscation.
type poaRegmap struct {
	t         Transport
	bus       RegBus // PlayerOne's Sony dies all use the ImgSen path; kept for interface parity
	mode      ReadoutMode
	protected map[uint16]bool
}

// ReadoutMode implements modeReader so the shared exposure/HMAX bodies (fps.go) read the
// live runtime context the Camera set.
func (r *poaRegmap) ReadoutMode() ReadoutMode { return r.mode }

// liveMode implements modeCarrier so the Camera can mutate the live ReadoutMode (FPS%,
// output depth, bin) without a concrete-type assertion. (Named liveMode, not mode, to
// avoid colliding with the mode field.)
func (r *poaRegmap) liveMode() *ReadoutMode { return &r.mode }

// VID reports the PlayerOne vendor id so shared sensor profiles select the PlayerOne
// gain/offset encoding at call time.
func (r *poaRegmap) VID() uint16 { return POA.VID }

// WriteReg writes a sensor register. A protected register (the gain-setup 0x67f) goes via
// CrypWrite — distinct opcode 0xB3 and an address offset by +0xABCD — exactly as the
// PlayerOne SDK does; the value is unchanged. All others use the plain sensor write 0xB0.
func (r *poaRegmap) WriteReg(reg, val uint16) error {
	if r.protected[reg] {
		return r.t.ControlOut(poaReqImgSenCryp, val, reg+poaCrypRegBias, nil)
	}
	return r.t.ControlOut(poaReqImgSenWrite, val, reg, nil)
}

// ReadReg reads a sensor register (8-bit; reg in wIndex, wValue 0).
func (r *poaRegmap) ReadReg(reg uint16) (uint16, error) {
	buf := make([]byte, 1)
	if _, err := r.t.ControlIn(poaReqImgSenRead, 0, reg, buf); err != nil {
		return 0, fmt.Errorf("read reg 0x%x: %w", reg, err)
	}
	return uint16(buf[0]), nil
}

// WriteRegBits is a read-modify-write of bits [lo:hi] (inclusive) of reg.
func (r *poaRegmap) WriteRegBits(reg uint16, lo, hi uint8, val uint16) error {
	cur, err := r.ReadReg(reg)
	if err != nil {
		return err
	}
	mask := uint16(((1 << (hi - lo + 1)) - 1) << lo)
	cur = (cur &^ mask) | ((val << lo) & mask)
	return r.WriteReg(reg, cur)
}

// WriteFPGAReg writes a camera-FPGA register (Fx3FpgaWrite 0xC0; reg in wIndex, value in
// wValue). PlayerOne's single-register form carries an 8-bit value (the low byte) — which
// matches how the FPGA register block is used (byte-wide regs, written one byte at a time).
func (r *poaRegmap) WriteFPGAReg(reg, val uint16) error {
	return r.t.ControlOut(poaReqFpgaWrite, val, reg, nil)
}

// ReadFPGAReg reads a camera-FPGA register (Fx3FpgaRead 0xC2).
func (r *poaRegmap) ReadFPGAReg(reg uint16) (uint16, error) {
	buf := make([]byte, 1)
	if _, err := r.t.ControlIn(poaReqFpgaRead, 0, reg, buf); err != nil {
		return 0, fmt.Errorf("read FPGA reg 0x%x: %w", reg, err)
	}
	return uint16(buf[0]), nil
}

// POA is the vendor descriptor for PlayerOne Astronomy cameras (USB VID 0xA0A0).
// Its Regmap dialect is defined above. Registered at init so the
// enumeration/Open paths discover it by VID — exactly like [[ZWO]], no core changes.
//
// NOTE: PlayerOne product rows (PID -> sensor) are not yet registered. Unlike ZWO, the
// PlayerOne SDK carries no static PID->sensor table — it filters on VID 0xA0A0 and reads
// the sensor identity from the opened device at runtime — so the per-product PIDs must be
// captured from real hardware (or a runtime sensor probe added) before IMX455/571
// PlayerOne bodies enumerate. The transport dialect, however, is complete.
var POA = &Vendor{
	VID:  0xA0A0,
	Name: "PlayerOne",
	newRegmap: func(t Transport, bus RegBus, mode ReadoutMode) Regmap {
		return &poaRegmap{t: t, bus: bus, mode: mode, protected: poaProtectedRegs}
	},
}

func init() { RegisterVendor(POA) }
