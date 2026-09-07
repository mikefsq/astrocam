package astrocam

// Vendor-neutral camera-FPGA primitives: read-modify-write helpers where the caller supplies the
// register number, so either vendor's code can use them. The register numbers themselves are
// vendor firmware and live in fpga_zwo.go / fpga_poa.go.

// SetFPGABit read-modify-writes a mask of an FPGA mode register: on sets the bits, !on clears
// them.
func SetFPGABit(rm Regmap, reg, bits uint16, on bool) error {
	v, err := rm.ReadFPGAReg(reg)
	if err != nil {
		return err
	}
	if on {
		v |= bits
	} else {
		v &^= bits
	}
	return rm.WriteFPGAReg(reg, v&0xff)
}

// FPGASetBits / FPGAClearBits / FPGAWriteBits are FPGA mode-register RMW helpers: set a mask,
// clear a mask, or write a masked field.
func FPGASetBits(rm Regmap, reg, bits uint16) error {
	v, err := rm.ReadFPGAReg(reg)
	if err != nil {
		return err
	}
	return rm.WriteFPGAReg(reg, (v|bits)&0xff)
}

func FPGAClearBits(rm Regmap, reg, bits uint16) error {
	v, err := rm.ReadFPGAReg(reg)
	if err != nil {
		return err
	}
	return rm.WriteFPGAReg(reg, (v&^bits)&0xff)
}

func FPGAWriteBits(rm Regmap, reg, mask, val uint16) error {
	v, err := rm.ReadFPGAReg(reg)
	if err != nil {
		return err
	}
	return rm.WriteFPGAReg(reg, ((v&^mask)|val)&0xff)
}
