package astrocam

import "testing"

// asidBlob builds a minimal "ASID" blob: 8-byte header (magic + BE length) followed by the
// given 2-byte sparse-RLE entries. length is the declared payload bound (header included),
// which the tests vary independently of the physical blob size.
func asidBlob(entries ...byte) []byte {
	blob := append([]byte("ASID\x00\x00\x00\x00"), entries...)
	return blob
}

// TestDefectMapTailBitsFiltered: with W·H not a multiple of 8, the padding bits of the last
// bitmap byte (0xFF flash fill flags them all) do not become Defects.
func TestDefectMapTailBitsFiltered(t *testing.T) {
	// 3×3 sensor: 9 pixels, 2 bitmap bytes, bits 9..15 of byte 1 are padding.
	// Entry (0x10, 0xFF): b0>>4=1, b0&0xf=0 → bitmap[1] = 0xFF → flags pixels 8..15.
	blob := asidBlob(0x10, 0xFF)
	m := parseDefectMap(blob, len(blob), 3, 3, false)
	if len(m.Defects) != 1 || m.Defects[0] != 8 {
		t.Fatalf("Defects = %v, want [8] (pixels 9..15 are bitmap padding, not defects)", m.Defects)
	}
}

// TestDefectMapOddLengthNoStraddle: a 2-byte entry needs both bytes inside the declared payload;
// with an odd length the trailing lone byte is ignored, not paired with flash junk past the
// payload.
func TestDefectMapOddLengthNoStraddle(t *testing.T) {
	// Physical blob: entry (0x00,0x01) then a lone 0x20 followed by junk 0xFF past the
	// declared length 11 (8 header + 3 payload bytes).
	blob := asidBlob(0x00, 0x01, 0x20, 0xFF)
	m := parseDefectMap(blob, 11, 5, 5, false)
	// Entry 1: b0=0x00,b1=0x01 → bitmap[0]=0x01 → pixel 0. The straddling (0x20,0xFF) pair
	// would set bitmap[2]=0xFF → pixels 16..23; none of those may appear.
	if len(m.Defects) != 1 || m.Defects[0] != 0 {
		t.Fatalf("Defects = %v, want [0] (entry straddling the odd payload end was applied)", m.Defects)
	}
}

// TestDefectMapBlockAdvance: a 00 00 entry advances the 256-byte block base.
func TestDefectMapBlockAdvance(t *testing.T) {
	blob := asidBlob(0x00, 0x00, 0x00, 0x01)            // base → 256, then bitmap[256] = 0x01
	m := parseDefectMap(blob, len(blob), 64, 64, false) // 4096 px = 512 bitmap bytes
	want := 256 * 8
	if len(m.Defects) != 1 || m.Defects[0] != want {
		t.Fatalf("Defects = %v, want [%d]", m.Defects, want)
	}
}

// TestApplyRAW16NeighbourAverage: a defect is replaced by the average of its orthogonal
// in-bounds neighbours (stride 1 mono).
func TestApplyRAW16NeighbourAverage(t *testing.T) {
	const W, H = 4, 4
	frame := make([]byte, W*H*2)
	set := func(p, v int) { frame[p*2] = byte(v); frame[p*2+1] = byte(v >> 8) }
	get := func(p int) int { return int(frame[p*2]) | int(frame[p*2+1])<<8 }
	set(1, 100)  // up
	set(4, 200)  // left
	set(6, 300)  // right
	set(9, 400)  // down
	set(5, 4095) // the defect (hot)
	m := &DefectMap{W: W, H: H, Defects: []int{5}, bitmap: make([]byte, (W*H+7)/8)}
	m.bitmap[0] = 1 << 5
	m.ApplyRAW16(frame)
	if got := get(5); got != 250 {
		t.Fatalf("defect pixel = %d, want 250 (avg of 100,200,300,400)", got)
	}
}

// TestApplyRAW16SkipsUncorrectedDefectNeighbours: a later defect (higher index) is not
// averaged into an earlier one while still uncorrected.
func TestApplyRAW16SkipsUncorrectedDefectNeighbours(t *testing.T) {
	const W, H = 4, 4
	frame := make([]byte, W*H*2)
	set := func(p, v int) { frame[p*2] = byte(v); frame[p*2+1] = byte(v >> 8) }
	get := func(p int) int { return int(frame[p*2]) | int(frame[p*2+1])<<8 }
	set(1, 100)  // up of 5
	set(4, 200)  // left of 5
	set(9, 400)  // down of 5
	set(5, 4095) // defect A
	set(6, 4095) // defect B: A's right neighbour, still uncorrected when A is fixed
	m := &DefectMap{W: W, H: H, Defects: []int{5, 6}, bitmap: make([]byte, (W*H+7)/8)}
	m.bitmap[0] = (1 << 5) | (1 << 6)
	m.ApplyRAW16(frame)
	if got := get(5); got != (100+200+400)/3 {
		t.Fatalf("defect A = %d, want %d (uncorrected defect neighbour must be skipped)",
			got, (100+200+400)/3)
	}
}

// TestApplyRAW16ShortFrameNoop: a frame smaller than W·H·2 is left untouched (no panic).
func TestApplyRAW16ShortFrameNoop(t *testing.T) {
	m := &DefectMap{W: 100, H: 100, Defects: []int{9999}, bitmap: make([]byte, 100*100/8)}
	m.ApplyRAW16(make([]byte, 16)) // must not panic or write
}

// TestDefectMapIndexPastBitmapDropped: the entry's nibble-swapped offset is flash data, so it can
// address a bitmap byte that does not exist. It must be dropped, not written.
func TestDefectMapIndexPastBitmapDropped(t *testing.T) {
	// b0 = 0x0f -> idx = (0x0f>>4) + (0x0f&0x0f)<<4 = 0 + 240, far past a 3x3 sensor's 2 bytes.
	blob := asidBlob(0x0f, 0xFF)
	m := parseDefectMap(blob, len(blob), 3, 3, false)
	if len(m.Defects) != 0 {
		t.Errorf("Defects = %v, want none: the entry indexed past the bitmap", m.Defects)
	}
}

// TestDefectMapBaseAdvancePastBitmapDropped: 00 00 entries walk the block base forward, and enough
// of them run it past the end of the bitmap. The entry that follows must still be dropped.
func TestDefectMapBaseAdvancePastBitmapDropped(t *testing.T) {
	// Two 00 00 entries put base at 512; a 3x3 sensor has a 2-byte bitmap.
	blob := asidBlob(0, 0, 0, 0, 0x10, 0xFF)
	m := parseDefectMap(blob, len(blob), 3, 3, false)
	if len(m.Defects) != 0 {
		t.Errorf("Defects = %v, want none: base had run past the bitmap", m.Defects)
	}
}

// TestDefectMapLengthPastBlobStopsAtBlob: a truncated flash read leaves the declared length longer
// than the bytes that arrived. The loop is bounded by both, so it must stop at the blob.
func TestDefectMapLengthPastBlobStopsAtBlob(t *testing.T) {
	blob := asidBlob(0x10, 0xFF) // one entry present
	// Claim a full 0x30000 payload, the largest LoadDefectMap admits.
	m := parseDefectMap(blob, 0x30000, 3, 3, false)
	if len(m.Defects) != 1 || m.Defects[0] != 8 {
		t.Fatalf("Defects = %v, want [8]: the decoder read past the blob or stopped early", m.Defects)
	}
}
