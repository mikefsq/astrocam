package astrocam

// Vendor-neutral half of the hardware Thermal seam: the write cache every vendor's backend
// shares, and the Camera entry point that hands out the backend the vendor supplies. The
// backends themselves are thermal_zwo.go and thermal_poa.go, because the cooling actuators sit
// at vendor-specific FPGA registers.

import (
	"sync"
	"time"
)

// coolRefresh bounds how long an unchanged TEC level or fan state goes without being rewritten:
// the write cache below skips a value the FPGA already holds, and this periodic rewrite covers
// a register the firmware reset behind the driver's back (a device reset the cache did not see,
// a second Thermal instance driving the same registers).
const coolRefresh = 5 * time.Second

// coolWrites is the write cache the hardware Thermal keeps on the Camera (shared by every
// HardwareThermal instance): the last TEC level and fan state put on the wire, so the 5 Hz
// regulation loop issues the reg-0x26 write and the reg-0x19 fan RMW only when a value changes
// or coolRefresh has passed. Init and a device reset invalidate it.
type coolWrites struct {
	mu       sync.Mutex
	level    uint16 // last TEC level written to fpgaCoolPower
	levelOK  bool   // level is known to be on the wire
	levelAt  time.Time
	fan      bool
	fanOK    bool
	fanAt    time.Time
	nowStamp func() time.Time // test hook; nil = time.Now
}

func (w *coolWrites) now() time.Time {
	if w.nowStamp != nil {
		return w.nowStamp()
	}
	return time.Now()
}

// invalidate forgets the cached values, so the next SetTECPower/SetFan writes unconditionally.
func (w *coolWrites) invalidate() {
	w.mu.Lock()
	w.levelOK, w.fanOK = false, false
	w.mu.Unlock()
}

// levelDue reports whether level must be written: unknown, changed, or last written more than
// coolRefresh ago.
func (w *coolWrites) levelDue(level uint16) bool {
	w.mu.Lock()
	defer w.mu.Unlock()
	return !w.levelOK || w.level != level || w.now().Sub(w.levelAt) >= coolRefresh
}

func (w *coolWrites) levelWritten(level uint16) {
	w.mu.Lock()
	w.level, w.levelOK, w.levelAt = level, true, w.now()
	w.mu.Unlock()
}

func (w *coolWrites) fanDue(on bool) bool {
	w.mu.Lock()
	defer w.mu.Unlock()
	return !w.fanOK || w.fan != on || w.now().Sub(w.fanAt) >= coolRefresh
}

func (w *coolWrites) fanWritten(on bool) {
	w.mu.Lock()
	w.fan, w.fanOK, w.fanAt = on, true, w.now()
	w.mu.Unlock()
}

// HardwareThermal returns the control-transfer Thermal backend for this (FPGA-cooled) camera:
// cam.EnableCooling(cam.HardwareThermal(), …). The backend comes from the vendor, because the
// cooling registers are not shared (see Vendor.newThermal).
func (c *Camera) HardwareThermal() Thermal { return c.vend.newThermal(c) }
