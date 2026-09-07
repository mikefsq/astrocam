package astrocam

// ZWO's factory defect-map layout: an "ASID" header at FlashHPCMapAddr followed by a sparse-RLE
// 1-bit-per-pixel bitmap. PlayerOne's "HPC:" per-row run list is defectmap_poa.go; the DefectMap
// type and the repair passes both feed are vendor-neutral (defectmap.go).

import (
	"encoding/binary"
	"fmt"
)

// loadDefectMapZWO reads and decompresses the factory defect map from SPI flash
// (FlashHPCMapAddr). fullW×fullH are the full-sensor dimensions the map is indexed in. The blob
// is an "ASID" header (magic + a big-endian length that counts the header too, not just the
// payload: the decoder rejects < 9 and rounds it up to a 2048 boundary) followed by a sparse-RLE
// 1-bit-per-pixel bitmap. Returns an error if no valid "ASID" map is present.
func loadDefectMapZWO(c *Camera, fullW, fullH int) (*DefectMap, error) {
	head, err := c.ReadSPIFlash(FlashHPCMapAddr, 2048)
	if err != nil {
		return nil, err
	}
	if len(head) < 8 || string(head[:4]) != "ASID" {
		return nil, fmt.Errorf("astrocam: no ASID defect map at flash 0x%x", FlashHPCMapAddr)
	}
	length := int(binary.BigEndian.Uint32(head[4:8]))
	if length < 9 || length > 0x30000 {
		return nil, fmt.Errorf("astrocam: implausible defect-map length %d", length)
	}
	total := (length + 2047) &^ 2047
	blob, err := c.ReadSPIFlash(FlashHPCMapAddr, total)
	if err != nil {
		return nil, err
	}
	return parseDefectMap(blob, length, fullW, fullH, c.Color()), nil
}

// parseDefectMap decompresses an "ASID" blob (header included) into a DefectMap: the parsing
// half of LoadDefectMap, testable without flash I/O.
func parseDefectMap(blob []byte, length, fullW, fullH int, color bool) *DefectMap {
	m := &DefectMap{W: fullW, H: fullH, Color: color}
	m.bitmap = decompressASID(blob, length, fullW*fullH)
	npix := fullW * fullH
	for k, b := range m.bitmap {
		if b == 0 {
			continue
		}
		for bit := 0; bit < 8; bit++ {
			if b&(1<<uint(bit)) != 0 {
				// Tail bits of the last bitmap byte are padding, not pixels (0xFF flash fill
				// would flag p ≥ W·H and ApplyRAW16 would write past the frame end).
				if p := k*8 + bit; p < npix {
					m.Defects = append(m.Defects, p)
				}
			}
		}
	}
	return m
}

// decompressASID expands the "ASID" sparse-RLE payload into the packed 1-bit-per-pixel defect
// bitmap. The payload is a stream of 2-byte entries starting at offset 8 (past the "ASID"+length
// header): a 00 00 entry advances the 256-byte block base, any other entry sets
// bitmap[base + (b0>>4) + (b0&0xf)<<4] = b1 (b0 is the nibble-swapped offset within the current
// block, b1 the bitmap byte). Bit b of byte k is the defect flag for pixel 8k+b (LSB-first).
func decompressASID(blob []byte, length, npix int) []byte {
	bitmap := make([]byte, (npix+7)/8)
	base := 0
	// Both bytes of an entry must sit inside the declared payload (x9+9 < length), so a
	// 2-byte entry can never straddle an odd payload end.
	for x9 := 0; x9+9 < length && x9+9 < len(blob); x9 += 2 {
		b0, b1 := blob[x9+8], blob[x9+9]
		if b0 == 0 && b1 == 0 {
			base += 256
			continue
		}
		if idx := base + int(b0>>4) + int(b0&0x0f)<<4; idx < len(bitmap) {
			bitmap[idx] = b1
		}
	}
	return bitmap
}
