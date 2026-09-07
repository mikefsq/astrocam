package astrocam

import "testing"

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
