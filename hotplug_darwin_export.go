//go:build darwin

package astrocam

/*
#include <stdint.h>
*/
import "C"

// asicamHotplugCB is called from the IOKit notification callbacks in
// hotplug_darwin.go, once per device that appeared or went away. It lives in
// its own file because a file that exports a Go function may carry only
// declarations in its preamble.
//
//export asicamHotplugCB
func asicamHotplugCB(attached C.int, vid, pid, loc C.uint32_t, entry C.uint64_t, handle C.uintptr_t) {
	deliverHotplug(attached != 0, uint16(vid), uint16(pid), uint32(loc), uint64(entry), uintptr(handle))
}
