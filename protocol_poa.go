package astrocam

import (
	"fmt"
	"sync"
)

// PlayerOne control-transfer opcodes. Same FX3 bridge as ZWO, different host dialect. The
// argument order is the reverse of ZWO's: PlayerOne puts the value in wValue and the register in
// wIndex.
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

	// poaCrypRegBias is the additive address obfuscation applied before CrypWrite:
	// wire_reg = (reg + 0xABCD) & 0xffff. A fixed offset, not encryption.
	poaCrypRegBias uint16 = 0xABCD
)

// poaProtectedRegs are the registers written via CrypWrite instead of a plain sensor write. The
// only protected register on either Sony die is the gain-setup 0x67f. Read-only after init.
var poaProtectedRegs = map[uint16]bool{0x67f: true}

// poaRegmap implements Regmap over a Transport using PlayerOne's control-transfer protocol. The
// Sony dies and register semantics are identical to ZWO's; only the wire dialect differs.
// protected routes the gain-setup register through CrypWrite.
type poaRegmap struct {
	t         Transport
	bus       RegBus // PlayerOne's Sony dies all use the ImgSen path; kept for interface parity
	modeMu    sync.RWMutex
	mode      ReadoutMode
	protected map[uint16]bool
}

// ReadoutMode implements modeReader (read under the mode lock).
func (r *poaRegmap) ReadoutMode() ReadoutMode {
	r.modeMu.RLock()
	defer r.modeMu.RUnlock()
	return r.mode
}

// updateMode implements modeCarrier (mutate under the mode lock).
func (r *poaRegmap) updateMode(f func(*ReadoutMode)) {
	r.modeMu.Lock()
	defer r.modeMu.Unlock()
	f(&r.mode)
}

// VID reports the PlayerOne vendor id (selects the PlayerOne gain/offset encoding).
func (r *poaRegmap) VID() uint16 { return POA.VID }

// WriteReg writes a sensor register: plain write 0xB0, or CrypWrite (0xB3, address +0xABCD,
// value unchanged) for a protected register.
func (r *poaRegmap) WriteReg(reg, val uint16) error {
	if r.protected[reg] {
		return r.t.ControlOut(poaReqImgSenCryp, val, reg+poaCrypRegBias, nil)
	}
	return r.t.ControlOut(poaReqImgSenWrite, val, reg, nil)
}

// ReadReg reads a sensor register (8-bit; reg in wIndex, wValue 0).
func (r *poaRegmap) ReadReg(reg uint16) (uint16, error) {
	buf := make([]byte, 1)
	got, err := r.t.ControlIn(poaReqImgSenRead, 0, reg, buf)
	if err != nil {
		return 0, fmt.Errorf("read reg 0x%x: %w", reg, err)
	}
	if got < 1 {
		return 0, fmt.Errorf("read reg 0x%x: empty control-IN", reg)
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

// WriteFPGAReg writes a camera-FPGA register (Fx3FpgaWrite 0xC0). val is transmitted as given:
// FPGA regs are byte-wide and every in-tree caller pre-masks to the low byte; what an over-byte
// wValue does on the POA wire is unverified.
func (r *poaRegmap) WriteFPGAReg(reg, val uint16) error {
	return r.t.ControlOut(poaReqFpgaWrite, val, reg, nil)
}

// ReadFPGAReg reads a camera-FPGA register (Fx3FpgaRead 0xC2).
func (r *poaRegmap) ReadFPGAReg(reg uint16) (uint16, error) {
	buf := make([]byte, 1)
	got, err := r.t.ControlIn(poaReqFpgaRead, 0, reg, buf)
	if err != nil {
		return 0, fmt.Errorf("read FPGA reg 0x%x: %w", reg, err)
	}
	if got < 1 {
		return 0, fmt.Errorf("read FPGA reg 0x%x: empty control-IN", reg)
	}
	return uint16(buf[0]), nil
}

// POA is the vendor descriptor for PlayerOne Astronomy cameras (USB VID 0xA0A0). Registered at
// init; the product rows (PID -> sensor) are registered by the sensors package. Not driven on
// hardware; its FX3 vendor commands are not decoded, so Cmds is zero and Camera.Init,
// ReadSPIFlash, FirmwareVersion and SerialNumber return "not decoded" errors instead of sending
// ZWO's opcodes.
var POA = &Vendor{
	VID:  0xA0A0,
	Name: "PlayerOne",
	newRegmap: func(t Transport, bus RegBus, mode ReadoutMode) Regmap {
		return &poaRegmap{t: t, bus: bus, mode: mode, protected: poaProtectedRegs}
	},
}

func init() { RegisterVendor(POA) }
