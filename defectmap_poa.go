package astrocam

import (
	"encoding/binary"
	"fmt"
)

// PlayerOne's factory hot-pixel map. The layout is unrelated to ZWO's ASID blob and is read
// through the same vendor request as any other flash page (0xD1, wIndex = address >> 8), with no
// GPIF toggle around it.
//
// A 64-byte header sits at a fixed address and points at the payload:
//
//	0   "HPC:"   magic
//	4   u8       must be 0
//	8   u32      payload byte address (the SDK converts it to a page with >> 8)
//	12  u32      payload length, rejected above 0xA0000
//	20  u16      checksum: the sum of all 64 header bytes, minus its own two bytes
//
// The payload is a per-row run list rather than a bitmap, which is why it is so much smaller than
// ZWO's: a u32 defect count, then one block per sensor row — the sentinel 0xFFFF, the row index,
// then that row's defect X coordinates ascending — and a closing sentinel. Every row gets a block
// whether or not it has defects, so the block count is exactly the sensor height.
//
// Read off a Xena 585M: 9086 defects in 26898 bytes over 2180 rows, 0.108% of the array, and the
// length arithmetic closes exactly at 4 + 2180x4 + 9086x2 + 2.
const (
	poaFlashHPCAddr   = 0x42000 // the header
	poaHPCHeaderLen   = 64
	poaHPCMaxPayload  = 0xA0000 // the SDK's own sanity bound
	poaHPCRowSentinel = 0xFFFF
)

// loadDefectMapPOA reads and parses the PlayerOne map. fullW/fullH are the geometry the map is
// indexed in, which is the vendor's full array rather than the profile's default.
func loadDefectMapPOA(c *Camera, fullW, fullH int) (*DefectMap, error) {
	head, err := c.ReadSPIFlash(poaFlashHPCAddr, poaHPCHeaderLen)
	if err != nil {
		return nil, err
	}
	if len(head) < poaHPCHeaderLen || string(head[:4]) != "HPC:" || head[4] != 0 {
		return nil, fmt.Errorf("astrocam: no HPC defect map at flash 0x%x", poaFlashHPCAddr)
	}
	sum := 0
	for _, b := range head {
		sum += int(b)
	}
	want := binary.LittleEndian.Uint16(head[20:22])
	if got := uint16(sum-int(want>>8)-int(want&0xff)) & 0xffff; got != want {
		return nil, fmt.Errorf("astrocam: HPC header checksum %#04x, want %#04x", got, want)
	}
	addr := binary.LittleEndian.Uint32(head[8:12])
	n := binary.LittleEndian.Uint32(head[12:16])
	if n == 0 || n > poaHPCMaxPayload {
		return nil, fmt.Errorf("astrocam: implausible HPC payload length %d", n)
	}
	blob, err := c.ReadSPIFlash(addr, int(n))
	if err != nil {
		return nil, err
	}
	if len(blob) < int(n) {
		return nil, fmt.Errorf("astrocam: short HPC payload: %d of %d bytes", len(blob), n)
	}
	return parseDefectMapPOA(blob, fullW, fullH, c.Color())
}

// parseDefectMapPOA expands the per-row run list into the shared DefectMap. Coordinates outside
// the declared geometry are dropped rather than trusted: the map is indexed in full-array pixels,
// and a corrupt run would otherwise write past a frame.
func parseDefectMapPOA(blob []byte, fullW, fullH int, color bool) (*DefectMap, error) {
	if len(blob) < 4 || fullW <= 0 || fullH <= 0 {
		return nil, fmt.Errorf("astrocam: HPC payload too short")
	}
	count := int(binary.LittleEndian.Uint32(blob[:4]))
	m := &DefectMap{W: fullW, H: fullH, Color: color}
	m.bitmap = make([]byte, (fullW*fullH+7)/8)
	row := -1
	for i := 4; i+1 < len(blob); i += 2 {
		v := int(binary.LittleEndian.Uint16(blob[i : i+2]))
		if v == poaHPCRowSentinel { // next two bytes are the row index
			if i+3 >= len(blob) {
				break // the closing sentinel has no row after it
			}
			i += 2
			row = int(binary.LittleEndian.Uint16(blob[i : i+2]))
			continue
		}
		if row < 0 || row >= fullH || v >= fullW {
			continue // before the first sentinel, or off the declared array
		}
		p := row*fullW + v
		m.bitmap[p>>3] |= 1 << (uint(p) & 7)
		m.Defects = append(m.Defects, p)
	}
	if count > 0 && len(m.Defects) != count {
		return nil, fmt.Errorf("astrocam: HPC map holds %d defects, header declares %d", len(m.Defects), count)
	}
	return m, nil
}
