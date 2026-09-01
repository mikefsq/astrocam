package astrocam

import (
	"encoding/binary"
	"testing"
)

// hpcBlob builds a PlayerOne HPC payload: a u32 count, then one block per row — the 0xFFFF
// sentinel, the row index, that row's defect X coordinates ascending — and a closing sentinel.
func hpcBlob(h int, rows map[int][]int) []byte {
	n := 0
	for _, xs := range rows {
		n += len(xs)
	}
	out := binary.LittleEndian.AppendUint32(nil, uint32(n))
	for y := 0; y < h; y++ {
		out = binary.LittleEndian.AppendUint16(out, poaHPCRowSentinel)
		out = binary.LittleEndian.AppendUint16(out, uint16(y))
		for _, x := range rows[y] {
			out = binary.LittleEndian.AppendUint16(out, uint16(x))
		}
	}
	return binary.LittleEndian.AppendUint16(out, poaHPCRowSentinel)
}

// TestParseDefectMapPOA: the per-row run list, including the length arithmetic that closes on the
// real map read off a Xena 585M — 9086 defects in 26898 bytes over 2180 rows.
func TestParseDefectMapPOA(t *testing.T) {
	const w, h = 64, 8
	rows := map[int][]int{0: {3, 10, 61}, 2: {0}, 7: {5, 6}}
	m, err := parseDefectMapPOA(hpcBlob(h, rows), w, h, false)
	if err != nil {
		t.Fatal(err)
	}
	if m.Count() != 6 {
		t.Fatalf("count = %d, want 6", m.Count())
	}
	for y, xs := range rows {
		for _, x := range xs {
			if !m.isDefect(y*w + x) {
				t.Errorf("(%d,%d) not flagged", x, y)
			}
		}
	}
	if m.isDefect(1*w + 3) {
		t.Error("(3,1) flagged; row 1 has no defects")
	}
	// The real map's arithmetic: 4 + rows*4 + defects*2 + 2.
	if got, want := len(hpcBlob(2180, map[int][]int{})), 4+2180*4+0*2+2; got != want {
		t.Errorf("empty 2180-row blob = %d bytes, want %d", got, want)
	}
	// A count that disagrees with the blob is rejected rather than half-trusted.
	bad := hpcBlob(h, rows)
	binary.LittleEndian.PutUint32(bad[:4], 99)
	if _, err := parseDefectMapPOA(bad, w, h, false); err == nil {
		t.Error("a mismatched defect count was accepted")
	}
	// Coordinates off the declared array are dropped, not written past the frame.
	off, _ := parseDefectMapPOA(hpcBlob(h, map[int][]int{1: {w + 5}}), w, h, false)
	if off != nil && off.Count() != 0 {
		t.Errorf("out-of-array X kept: %d defects", off.Count())
	}
}

// TestApplyWindow: each defect becomes the mean of its four in-window same-colour neighbours, and
// nothing else in the frame is touched. Sub-frame windows offset the map's full-array coordinates.
func TestApplyWindow(t *testing.T) {
	const W, H = 16, 8
	m, err := parseDefectMapPOA(hpcBlob(H, map[int][]int{3: {5}}), W, H, false)
	if err != nil {
		t.Fatal(err)
	}
	for _, bpp := range []int{1, 2} {
		frame := make([]byte, W*H*bpp)
		set := func(x, y, v int) {
			p := y*W + x
			if bpp == 1 {
				frame[p] = byte(v)
			} else {
				frame[p*2], frame[p*2+1] = byte(v), byte(v>>8)
			}
		}
		get := func(x, y int) int {
			p := y*W + x
			if bpp == 1 {
				return int(frame[p])
			}
			return int(frame[p*2]) | int(frame[p*2+1])<<8
		}
		for y := 0; y < H; y++ {
			for x := 0; x < W; x++ {
				set(x, y, 100)
			}
		}
		set(5, 3, 200) // the defect: hot
		set(5, 2, 40)  // its four neighbours, deliberately uneven
		set(4, 3, 60)
		set(6, 3, 80)
		set(5, 4, 120)
		m.ApplyWindow(frame, bpp, 0, 0, W, H)
		if got, want := get(5, 3), (40+60+80+120)/4; got != want {
			t.Errorf("bpp %d: defect repaired to %d, want the neighbour mean %d", bpp, got, want)
		}
		// Nothing else moved.
		for y := 0; y < H; y++ {
			for x := 0; x < W; x++ {
				if x == 5 && y == 3 {
					continue
				}
				want := 100
				switch {
				case x == 5 && y == 2:
					want = 40
				case x == 4 && y == 3:
					want = 60
				case x == 6 && y == 3:
					want = 80
				case x == 5 && y == 4:
					want = 120
				}
				if get(x, y) != want {
					t.Fatalf("bpp %d: (%d,%d) = %d, want %d untouched", bpp, x, y, get(x, y), want)
				}
			}
		}
	}
	// A window that does not contain the defect is left entirely alone.
	frame := make([]byte, 4*4*2)
	for i := range frame {
		frame[i] = 0x55
	}
	m.ApplyWindow(frame, 2, 8, 0, 4, 4) // x0=8: the defect at x=5 is outside
	for i, b := range frame {
		if b != 0x55 {
			t.Fatalf("byte %d changed for a window that excludes the defect", i)
		}
	}
}
