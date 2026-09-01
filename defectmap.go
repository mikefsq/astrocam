package astrocam

import (
	"encoding/binary"
	"fmt"
)

// DefectMap is the camera's factory hot/dead-pixel map: the per-unit list of defective sensor
// pixels burned into SPI flash at manufacture. Read once from flash (LoadDefectMap) and applied
// to each captured frame by ApplyWindow, which Camera.RepairFrame calls when the vendor's map is
// trusted (Vendor.defectMapTrusted) and RepairDefects is on. ApplyRAW16 is the full-frame
// RAW16-only form.
type DefectMap struct {
	W, H    int    // full-sensor dimensions the map is indexed in
	Color   bool   // Bayer sensor → same-colour neighbour stride 2 (mono → 1)
	bitmap  []byte // packed 1-bit-per-pixel, LSB-first; set bit = defect
	Defects []int  // defect pixel linear indices (y*W+x), ascending
}

// Count is the number of flagged defect pixels.
func (m *DefectMap) Count() int { return len(m.Defects) }

// isDefect reports whether full-sensor pixel p is flagged.
func (m *DefectMap) isDefect(p int) bool {
	if p < 0 || p >= m.W*m.H {
		return false
	}
	return m.bitmap[p>>3]&(1<<(uint(p)&7)) != 0
}

// LoadDefectMap reads and decompresses the factory defect map from SPI flash (FlashHPCMapAddr).
// fullW×fullH are the full-sensor dimensions the map is indexed in. The blob is an "ASID" header
// (magic + a big-endian length that counts the header too, not just the payload: the decoder
// rejects < 9 and rounds it up to a 2048 boundary) followed by a sparse-RLE 1-bit-per-pixel
// bitmap. Returns an error if no valid "ASID" map is present.
func (c *Camera) LoadDefectMap(fullW, fullH int) (*DefectMap, error) {
	// The blob layout is the vendor's, not the die's. A vendor with its own reader takes it from
	// here; the ASID path below is ZWO's.
	if c.vend.loadDefectMap != nil {
		return c.vend.loadDefectMap(c, fullW, fullH)
	}
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

// ApplyRAW16 corrects each defect pixel in place in a full-frame little-endian RAW16 buffer
// (W*H 16-bit pixels). Each defect is replaced by the average of its in-bounds orthogonal
// neighbours at the same-Bayer-colour stride (2 colour, 1 mono), skipping still-uncorrected
// defect neighbours; if none is usable it copies the previous pixel. Defects are walked in
// ascending index order. Full-frame only (the map is in full-sensor coordinates).
func (m *DefectMap) ApplyRAW16(frame []byte) {
	W, H := m.W, m.H
	if len(frame) < W*H*2 {
		return
	}
	s := 1
	if m.Color {
		s = 2
	}
	px := func(p int) int { return int(frame[p*2]) | int(frame[p*2+1])<<8 }
	setpx := func(p, v int) { frame[p*2] = byte(v); frame[p*2+1] = byte(v >> 8) }
	for _, p := range m.Defects {
		x, y := p%W, p/W
		sum, cnt := 0, 0
		use := func(q int) {
			if q <= p || !m.isDefect(q) {
				sum += px(q)
				cnt++
			}
		}
		if y-s >= 0 {
			use(p - s*W)
		}
		if x-s >= 0 {
			use(p - s)
		}
		if x+s < W {
			use(p + s)
		}
		if y+s < H {
			use(p + s*W)
		}
		if cnt > 0 {
			setpx(p, sum/cnt)
		} else if p > 0 {
			setpx(p, px(p-1))
		}
	}
}

// ApplyWindow repairs the defects that fall inside a readout window, for either sample size. The
// map is indexed in FULL-ARRAY pixels, so (x0,y0) is where the window sits in the array and w×h
// is its size; a full frame is (0,0,W,H). Each defect takes the mean of its four same-colour
// neighbours — stride 2 on a Bayer sensor, 1 on mono — skipping neighbours that are themselves
// uncorrected defects, and skipping any that fall outside the window.
//
// Binned frames are NOT handled and must not be passed: a binned pixel already averages a block,
// so a defect is diluted rather than isolated, and the map's coordinates no longer address the
// samples being written.
func (m *DefectMap) ApplyWindow(frame []byte, bpp, x0, y0, w, h int) {
	if bpp != 1 && bpp != 2 || w <= 0 || h <= 0 || len(frame) < w*h*bpp {
		return
	}
	s := 1
	if m.Color {
		s = 2
	}
	px := func(p int) int {
		if bpp == 1 {
			return int(frame[p])
		}
		return int(frame[p*2]) | int(frame[p*2+1])<<8
	}
	setpx := func(p, v int) {
		if bpp == 1 {
			if v > 0xff {
				v = 0xff
			}
			frame[p] = byte(v)
			return
		}
		frame[p*2], frame[p*2+1] = byte(v), byte(v>>8)
	}
	for _, d := range m.Defects {
		dx, dy := d%m.W-x0, d/m.W-y0
		if dx < 0 || dy < 0 || dx >= w || dy >= h {
			continue // this defect is outside the window
		}
		p := dy*w + dx
		sum, cnt := 0, 0
		use := func(qx, qy int) {
			// A neighbour that is itself a defect is usable only once repaired, which for an
			// ascending defect list means one that sorts before this pixel.
			if qi := (qy+y0)*m.W + qx + x0; qi <= d || !m.isDefect(qi) {
				sum += px(qy*w + qx)
				cnt++
			}
		}
		if dy-s >= 0 {
			use(dx, dy-s)
		}
		if dx-s >= 0 {
			use(dx-s, dy)
		}
		if dx+s < w {
			use(dx+s, dy)
		}
		if dy+s < h {
			use(dx, dy+s)
		}
		if cnt > 0 {
			setpx(p, (sum+cnt/2)/cnt)
		}
	}
}
