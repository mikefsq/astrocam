//go:build linux

package astrocam

import "testing"

// The batch reap loop used to leave only on no-pending, deadline or abort. A frame that ends with
// a short transfer leaves every later slot submitted but never completing — the FX3 has stopped
// sending — so the loop waited out the whole timeout on a frame that was already in. These pin
// the predicate that now ends it: the first slot in the contiguous COMPLETED prefix that failed
// or came back short.

func TestURBReqLen(t *testing.T) {
	const chunk = 1 << 20
	// 2.5 chunks: two full transfers and a half-sized tail.
	bufLen := chunk*2 + chunk/2
	for i, want := range []int{chunk, chunk, chunk / 2} {
		if got := urbReqLen(i, chunk, bufLen); got != want {
			t.Errorf("urbReqLen(%d) = %d, want %d", i, got, want)
		}
	}
	// An exact multiple asks for a full chunk in every slot, including the last.
	if got := urbReqLen(1, chunk, chunk*2); got != chunk {
		t.Errorf("urbReqLen(last, exact multiple) = %d, want %d", got, chunk)
	}
}

func TestURBFrameEnded(t *testing.T) {
	const chunk = 1 << 20
	const bufLen = chunk * 3
	full := func(status int32) usbURB { return usbURB{actualLength: chunk, status: status} }
	short := usbURB{actualLength: chunk / 3}

	for _, tc := range []struct {
		name string
		urbs []usbURB
		done []bool
		want bool
	}{
		{"nothing completed yet", []usbURB{{}, {}, {}}, []bool{false, false, false}, false},
		{"prefix unbroken, more to come", []usbURB{full(0), {}, {}}, []bool{true, false, false}, false},
		{"every slot in and full", []usbURB{full(0), full(0), full(0)}, []bool{true, true, true}, false},
		{"short transfer ends the frame", []usbURB{full(0), short, {}}, []bool{true, true, false}, true},
		{"failed slot ends the frame", []usbURB{full(0), full(-32), {}}, []bool{true, true, false}, true},
		{"short in the first slot", []usbURB{short, {}, {}}, []bool{true, false, false}, true},
		// The gap matters: a later slot completing short does not end the frame while an
		// earlier one is still outstanding, because completions land in submission order and
		// the prefix is what the caller assembles.
		{"short behind a gap is not yet the end", []usbURB{{}, short, {}}, []bool{false, true, false}, false},
	} {
		if got := urbFrameEnded(tc.urbs, tc.done, chunk, bufLen); got != tc.want {
			t.Errorf("%s: urbFrameEnded = %v, want %v", tc.name, got, tc.want)
		}
	}
}

// The frame's last transfer asks for the exact remainder, so a full-length tail must NOT read as
// short — otherwise every complete frame would look like it ended early.
func TestURBFrameEndedExactTail(t *testing.T) {
	const chunk = 1 << 20
	bufLen := chunk + chunk/4 // one full slot plus a quarter-chunk tail
	urbs := []usbURB{{actualLength: chunk}, {actualLength: chunk / 4}}
	if urbFrameEnded(urbs, []bool{true, true}, chunk, bufLen) {
		t.Error("a tail that filled its exact remainder read as a short transfer")
	}
	urbs[1].actualLength = chunk/4 - 1
	if !urbFrameEnded(urbs, []bool{true, true}, chunk, bufLen) {
		t.Error("a tail one byte short of its remainder did not end the frame")
	}
}
