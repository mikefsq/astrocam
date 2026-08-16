package astrocam

// Snap (single-frame) data plane: arm, host-time the exposure, read one whole frame.
//
// Status word: 0 idle / 1 working / 2 success / 3 failed. A single snap arms then reads
// until FrameBytes arrives.
//
// Arm sequence: SendCMD 0xAA (stop) → FPGAStop (reg0 bit4 set: → 0x71) → sensor StreamStop
// (WriteSONYREG 0x200=1) → SendCMD 0xA9 (start) → sensor StreamStart (0x200=0) → FPGAStart
// (reg0 bit4 clear: → 0x61) → ResetEndPoint(0x81) → bulk read. The generic SendCMD/FPGA/
// endpoint parts are here; the sensor master stop/start is the Sensor.StreamStop/StreamStart hook.
//
// Delivered bulk data is raw pixels from offset 0 (the FX3 strips its internal 0xBB00AA11 frame
// delimiter); frames are validated by size, not a header.

import (
	"errors"
	"fmt"
	"time"
)

// ErrDeviceWedged is returned once the camera is dead: a short/failed readout whose
// firmware-version recheck showed the FX3 dropped its firmware. Recovery is a USB reset
// (USBDEVFS_RESET) + re-scan + re-Open. Callers match this with errors.Is.
var ErrDeviceWedged = errors.New("astrocam: device wedged (FX3 firmware crash): needs USB reset + re-scan")

// ZWO's FX3 streaming vendor commands (SendCMD); they seed ZWO.Cmds. Other vendors fill their
// own table.
const (
	cmdStreamStop  = 0xAA // stop/prepare before (re)arming
	cmdStreamStart = 0xA9 // begin streaming
	cmdFlush       = 0xAF // pipeline flush / drop recovery
	bulkEndpoint   = 0x81 // bulk-IN endpoint
)

// FPGA mode register 0: bit4 stops the readout pipeline (FPGAStop sets it, FPGAStart clears it).
const (
	fpgaModeReg0 = 0x00
	fpgaStopBit  = 0x10
)

func (c *Camera) fpgaSetReg0(mask, val uint16) error {
	v, err := c.rm.ReadFPGAReg(fpgaModeReg0)
	if err != nil {
		return fmt.Errorf("astrocam: read FPGA reg0: %w", err)
	}
	return c.rm.WriteFPGAReg(fpgaModeReg0, ((v&^mask)|val)&0xff)
}

func (c *Camera) fpgaStop() error  { return c.fpgaSetReg0(fpgaStopBit, fpgaStopBit) }
func (c *Camera) fpgaStart() error { return c.fpgaSetReg0(fpgaStopBit, 0) }

// --- WorkerCtl: generic primitives a per-sensor capture Worker (Sensor.Worker) calls. ---

// Rm returns the Camera's Regmap (sensor + FPGA register R/W).
func (c *Camera) Rm() Regmap { return c.rm }

// VendorCmd issues a SendCMD-style FX3 vendor command (FX3StreamStop / FX3StreamStart /
// FX3Flush), resolved through the vendor's command table.
func (c *Camera) VendorCmd(op FX3Op) error { return c.vendorCmd(op) }

// ResetEndpoint clears the bulk-IN pipe (EP 0x81) when the backend supports it.
func (c *Camera) ResetEndpoint() error {
	if r, ok := c.t.(EndpointResetter); ok {
		return r.ResetEndpoint(bulkEndpoint)
	}
	return nil
}

// ResetDevice performs a whole-device USB reset, the worker's last-resort recovery for a
// wedged readout. On a backend without DeviceResetter (WinUSB exposes no device-level
// reset) it returns an error rather than pretending the reset happened, so a recovery
// ladder's last rung fails loud instead of looping on a still-wedged device.
func (c *Camera) ResetDevice() error {
	if r, ok := c.t.(DeviceResetter); ok {
		return r.ResetDevice()
	}
	return fmt.Errorf("astrocam: transport has no device reset (recovery rung unavailable)")
}

// BulkRead reads from the bulk-IN endpoint with the given timeout.
func (c *Camera) BulkRead(buf []byte, to time.Duration) (int, error) { return c.t.BulkRead(buf, to) }

// BulkReadQuiet is BulkRead with the first `quiet` declared as host-timed integration: the
// transfers are armed immediately but the control-transfer gate engages only when quiet
// elapses or data arrives (see QuietBulkReader), so a sensor-timed exposure doesn't blind
// EP0 (TEC, telemetry, ST4). Falls back to a fully-gated BulkRead on backends without it.
func (c *Camera) BulkReadQuiet(buf []byte, quiet, to time.Duration) (int, error) {
	if q, ok := c.t.(QuietBulkReader); ok {
		return q.BulkReadQuiet(buf, quiet, to)
	}
	return c.t.BulkRead(buf, to)
}

// NoteStall records one readout stall (a short/failed bulk read that triggered a re-arm).
func (c *Camera) NoteStall() { c.stalls.Add(1) }

// StallCount returns the cumulative number of worker readout stalls over the camera's lifetime.
func (c *Camera) StallCount() int64 { return c.stalls.Load() }

// StreamFrame reads one frame via the continuous windowed pump when the backend provides it,
// else a single BulkRead. The windowed pump reliably pulls a large USB3 frame without
// truncating at a burst boundary.
func (c *Camera) StreamFrame(buf []byte, idle, total time.Duration) (int, error) {
	if fs, ok := c.t.(FrameStreamer); ok {
		return fs.ReadFrameStream(buf, idle, total)
	}
	return c.t.BulkRead(buf, total)
}

// StreamFramePrequeued reads one free-run frame with a continuously-in-flight URB ring (the SDK's
// async-transfer model), falling back to StreamFrame on backends without it.
func (c *Camera) StreamFramePrequeued(buf []byte, idle, total time.Duration) (int, error) {
	if p, ok := c.t.(PrequeuedFrameStreamer); ok {
		return p.ReadFrameStreamPrequeued(buf, idle, total)
	}
	return c.StreamFrame(buf, idle, total)
}

// FrameBytes is the size of one frame to read off the wire and the buffer a caller must
// allocate: width × height × bytes-per-pixel (RAW16 → 2, RAW8 → 1) of the live OUTPUT mode.
// Output depth is the readout mode (SetOutputDepth), not the sensor ADC depth. For RAW16
// software binning (SoftBin>1) the sensor reads the full bin-scaled frame, so this is SoftBin²
// larger than the delivered image, which the read averages down to roiW·roiH·bpp. SoftBin==1
// for every non-software-bin path, where this is exactly the delivered frame size.
func (c *Camera) FrameBytes() int {
	m := ModeOf(c.rm)
	bpp := m.BytesPerPx
	sb := m.SoftBin
	if sb < 1 {
		sb = 1
	}
	c.mu.Lock()
	w, h := c.roiW, c.roiH
	c.mu.Unlock()
	// Wire/read size: for RAW16 software binning the sensor reads the full bin-scaled frame (sb×
	// per axis), averaged down to the w×h output later. sb==1 for non-software-bin paths.
	return w * h * bpp * sb * sb
}

// binFrame applies RAW16 host-side binning to a freshly read full-resolution frame (these sensors
// have no hardware 16-bit binned mode). With SoftBin<=1 or RAW8 it is a no-op returning n;
// otherwise it averages bin×bin in place and returns the smaller output byte count. m.Width/Height
// are the full readout dims.
func (c *Camera) binFrame(buf []byte, n int) int {
	m := ModeOf(c.rm)
	if m.SoftBin <= 1 || m.BytesPerPx < 2 {
		return n
	}
	if want := m.Width * m.Height * 2; n < want || m.Width <= 0 || m.Height <= 0 {
		return n // not the expected full RAW16 frame; leave it for the caller to flag
	}
	if c.Color() {
		return colorBinRAW16(buf, m.Width, m.Height, m.SoftBin)
	}
	return averageBinRAW16(buf, m.Width, m.Height, m.SoftBin)
}

// colorBinRAW16 is the Bayer-preserving equivalent of averageBinRAW16: it averages only
// same-color samples so the binned output stays a valid mosaic. Output is floor dims
// (fullW/bin)×(fullH/bin). Each output pixel (oy,ox) keeps Bayer phase (oy%2,ox%2) and averages
// the same-phase pixels in the bin×bin source block at (bin·oy, bin·ox): for an even bin, the
// (bin/2)² same-color samples (bin 2 = 1 sample, bin 4 = 4). cnt clamps partial edge blocks (none
// for exact divisors, the only color bins SetROI allows). Scratch output avoids in-place aliasing.
func colorBinRAW16(buf []byte, fullW, fullH, bin int) int {
	outW, outH := fullW/bin, fullH/bin
	out := make([]byte, outW*outH*2)
	for oy := 0; oy < outH; oy++ {
		py := oy & 1 // Bayer row phase (even bin → source block starts on an even row)
		srow0 := oy * bin
		for ox := 0; ox < outW; ox++ {
			px := ox & 1
			scol0 := ox * bin
			sum, cnt := 0, 0
			for dy := py; dy < bin; dy += 2 { // same-color rows within the bin×bin block
				r := srow0 + dy
				if r >= fullH {
					break
				}
				rb := r * fullW
				for dx := px; dx < bin; dx += 2 { // same-color cols
					cc := scol0 + dx
					if cc >= fullW {
						break
					}
					p := (rb + cc) * 2
					sum += int(buf[p]) | int(buf[p+1])<<8
					cnt++
				}
			}
			v := 0
			if cnt > 0 {
				v = (sum + cnt/2) / cnt
			}
			if v > 0xffff {
				v = 0xffff
			}
			o := (oy*outW + ox) * 2
			out[o] = byte(v)
			out[o+1] = byte(v >> 8)
		}
	}
	copy(buf, out)
	return outW * outH * 2
}

// averageBinRAW16 averages bin×bin blocks of a fullW×fullH RAW16 (16-bit little-endian) frame in
// buf, writing the (fullW/bin)×(fullH/bin) result into the front of the same buffer, and returns
// the output length in bytes. Rounded mean (sum + bin²/2)/bin². In-place is safe: each output
// pixel's source bytes sit at an offset ≥ its own, so a forward scan never overwrites unread input.
func averageBinRAW16(buf []byte, fullW, fullH, bin int) int {
	outW, outH := fullW/bin, fullH/bin
	div := bin * bin
	half := div / 2
	for oy := 0; oy < outH; oy++ {
		for ox := 0; ox < outW; ox++ {
			sum := 0
			for dy := 0; dy < bin; dy++ {
				base := ((oy*bin+dy)*fullW + ox*bin) * 2
				for dx := 0; dx < bin; dx++ {
					p := base + dx*2
					sum += int(buf[p]) | int(buf[p+1])<<8
				}
			}
			v := (sum + half) / div
			if v > 0xffff {
				v = 0xffff
			}
			o := (oy*outW + ox) * 2
			buf[o] = byte(v)
			buf[o+1] = byte(v >> 8)
		}
	}
	return outW * outH * 2
}

// StartExposure arms a single (snap) exposure: it brackets the sensor's master start with the
// FX3 stream stop/start commands (or runs the profile's own Arm) and seeds the exposure status
// to Working. GetDataAfterExp reads the frame once GetExpStatus reports Success. light is the
// ASCOM/ASI light-vs-dark flag, accepted for API parity; none of the supported cameras has a
// mechanical shutter, so it has no hardware effect (cover the optics for darks).
func (c *Camera) StartExposure(light bool) error {
	if c.dead.Load() {
		return ErrDeviceWedged // dead device: refuse new frames until reset+re-Open
	}
	// Claim the exposure atomically: the busy check and the Working transition happen in ONE
	// critical section (check-then-act across an unlock let two StartExposure calls double-arm).
	// Claiming also bumps the generation and clears the abort flag, superseding any stale
	// capture still unwinding; its generation-guarded writes become no-ops.
	c.mu.Lock()
	if c.status == ExpWorking {
		c.mu.Unlock()
		return nil // already capturing (documented no-op; the new light flag is NOT applied)
	}
	c.status = ExpWorking
	c.expStart = nowFunc()
	c.expGen++
	gen := c.expGen
	c.expAborted = false
	c.mu.Unlock()
	// Re-enable frame reads: a prior StopExposure left the transport's read-abort latched
	// (level-triggered, see ReadAborter) so stale reads fail fast; this exposure owns the
	// bus again.
	if ra, ok := c.t.(ReadAborter); ok {
		ra.ArmRead()
	}
	// A sensor Worker arms, host-times, and reads inside GetDataAfterExp, so the worker path
	// seeds state only. The non-worker path arms here, unlocked.
	if c.sensor.Worker == nil {
		if err := c.arm(); err != nil {
			c.setStatusIfGen(gen, ExpIdle) // release the claim (unless superseded meanwhile)
			return err
		}
	}
	return nil
}

// setStatusIfGen stores the exposure status only if the exposure generation is still gen:
// a stale capture (superseded by a newer StartExposure) must not clobber the new exposure's
// state.
func (c *Camera) setStatusIfGen(gen uint64, s ExposureStatus) {
	c.mu.Lock()
	if c.expGen == gen {
		c.status = s
	}
	c.mu.Unlock()
}

// abortedGen reports whether the generation-gen exposure is aborted: StopExposure ran
// (expAborted), or a newer StartExposure superseded it. Independent of status: a poll deriving
// SUCCESS at window end must not read as an abort mid-integration.
func (c *Camera) abortedGen(gen uint64) bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.expAborted || c.expGen != gen
}

// Aborted reports that the current exposure was aborted (StopExposure ran). A host-timed
// worker polls this (via its generation-bound WorkerCtl) so an abort cuts the integration
// short instead of waiting out the full exposure.
func (c *Camera) Aborted() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.expAborted
}

// workerCtl is the WorkerCtl handed to a sensor Worker: the Camera plus the exposure
// generation the worker belongs to, so Aborted() also fires when the worker was superseded
// by a newer StartExposure (not just on an explicit StopExposure).
type workerCtl struct {
	*Camera
	gen uint64
}

func (w workerCtl) Aborted() bool { return w.abortedGen(w.gen) }

// expDuration returns the last-set exposure under the lock (used to size bulk read timeouts).
func (c *Camera) expDuration() time.Duration {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.expDur
}

// arm issues the arm sequence: the sensor's own Arm hook when it has one, else the generic
// SendCMD 0xAA → FPGA stop → sensor master stop → SendCMD 0xA9 → sensor master start → settle →
// FPGA start (clear reg0 bit4, un-halting the readout so frames reach EP 0x81). Shared by
// StartExposure, StartVideo and the GetDataAfterExp recovery path.
func (c *Camera) arm() error {
	if c.sensor.Arm != nil {
		return c.sensor.Arm(c) // the profile's own choreography (see Sensor.Arm)
	}
	if err := c.vendorCmd(FX3StreamStop); err != nil {
		return err
	}
	if err := c.fpgaStop(); err != nil {
		return err
	}
	if c.sensor.StreamStop != nil {
		if err := c.sensor.StreamStop(c.rm); err != nil {
			return err
		}
	}
	if err := c.vendorCmd(FX3StreamStart); err != nil {
		return err
	}
	if c.sensor.StreamStart != nil {
		if err := c.sensor.StreamStart(c.rm); err != nil {
			return err
		}
	}
	time.Sleep(10 * time.Millisecond) // usleep(0x2710) settle before FPGA start
	return c.fpgaStart()
}

// StartVideo arms the sensor for continuous free-run streaming (the video / .ser burst path):
// it runs the capture arm but does not read, so the sensor free-runs at the readout rate and
// frames are pulled back-to-back with no per-frame re-arm. It un-halts the readout (clear reg0
// bit4) but keeps WaitMode (bit6) as SetExposure left it: the 174 stream runs at reg0=0x61
// (bit4 clear, bit6 set). Short (non-trigger) exposures only; long-exp trigger stays single-shot.
func (c *Camera) StartVideo(light bool) error {
	if err := c.arm(); err != nil { // ends with FPGAStart: readout un-halted (bit4 clear), WaitMode (bit6) untouched
		return err
	}
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	c.mu.Lock()
	c.status = ExpWorking
	c.expStart = nowFunc()
	c.expGen++ // supersede any stale single-shot capture still unwinding
	c.expAborted = false
	c.mu.Unlock()
	if ra, ok := c.t.(ReadAborter); ok {
		ra.ArmRead() // clear a prior StopExposure's read-abort latch (see StartExposure)
	}
	return nil
}

// StartStream opens a resident windowed-stream session (FrameStream) sized to the current frame
// when the backend supports it; errors otherwise so the caller can fall back to per-frame reads.
func (c *Camera) StartStream(total time.Duration) (FrameStream, error) {
	ss, ok := c.t.(StreamStarter)
	if !ok {
		return nil, fmt.Errorf("astrocam: backend has no resident stream session")
	}
	return ss.StartStream(c.FrameBytes(), total)
}

// nowFunc is the clock the host-timed status poll uses; overridable in tests.
var nowFunc = time.Now

// GetExpStatus reports the snap exposure status: WORKING while the exposure is in flight,
// SUCCESS once the host-timed window has elapsed (frame ready for readout). Poll until SUCCESS,
// then call GetDataAfterExp. The SUCCESS at window end is DERIVED, not stored, so a status poll
// never changes what the worker still integrating observes.
func (c *Camera) GetExpStatus() ExposureStatus {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.status == ExpWorking && nowFunc().Sub(c.expStart) >= c.expDur {
		return ExpSuccess
	}
	return c.status
}

// frameReadAttempts is how many times GetDataAfterExp tries to read a valid frame, recovering
// and re-arming between tries (a transient pipe dropout or partial/stalled frame is fixed by
// clearing the pipe and re-arming a fresh exposure).
const frameReadAttempts = 3

// RepairDMAMarkers controls whether captured frames have their FX3 DDR frame header/footer
// marker words stripped (on by default). Set false to receive the raw frame with the marker
// pixels intact.
var RepairDMAMarkers = true

// repairFX3DMAMarkers strips the FX3 bridge's frame markers. The FX3 brackets every DDR frame
// with a fixed header word (0x00005A7E low half) and footer word (0x3CF0 high half), so the
// first and last 32-bit DMA word are not sensor data (2 pixels each in RAW16, 4 each in RAW8).
// Confirmed on IMX455 + IMX462; gated per-sensor by Sensor.FX3DMAMarkers. Signature-detected
// (0x5A7E/0x3CF0 at the start/end), so a no-op on frames without the markers; when present, each
// marker word is overwritten by edge-replicating the nearest real pixel. bpp is the output
// bytes-per-pixel (1 = RAW8, 2 = RAW16).
func repairFX3DMAMarkers(buf []byte, bpp int) {
	n := len(buf)
	if bpp < 1 || n < 8 || n%bpp != 0 {
		return
	}
	if buf[0] != 0x7E || buf[1] != 0x5A || buf[n-2] != 0xF0 || buf[n-1] != 0x3C {
		return
	}
	fill := func(lo, hi, src int) {
		for off := lo; off < hi; off += bpp {
			copy(buf[off:off+bpp], buf[src:src+bpp])
		}
	}
	fill(0, 4, 4)         // leading marker word <- first real pixel (at byte 4)
	fill(n-4, n, n-4-bpp) // trailing marker word <- last real pixel (ends at byte n-4)
}

// RepairFrame applies the FX3 DDR frame-marker repair to a delivered frame when the profile has
// the markers (Sensor.FX3DMAMarkers) and RepairDMAMarkers is on; a no-op otherwise. GetDataAfterExp
// applies it itself; callers pulling frames from the free-run paths (ReadFrame, a resident
// FrameStream) call it per frame. In free-run the header word's third byte is a frame counter,
// so an unrepaired stream shows the corner pixel stepping 1, 2, 3, ….
func (c *Camera) RepairFrame(buf []byte) {
	if RepairDMAMarkers && c.sensor.FX3DMAMarkers {
		repairFX3DMAMarkers(buf, ModeOf(c.rm).BytesPerPx)
	}
}

// Wedged reports whether the camera has been marked dead by the firmware-crash check (see
// ErrDeviceWedged / checkDead). Recovery is a USB reset + re-scan + re-Open.
func (c *Camera) Wedged() bool { return c.dead.Load() }

// checkDead re-reads the firmware version after a short/failed readout and compares it to the
// baseline cached at Init: if it changed (or won't read) the FX3 dropped its firmware and the
// camera is dead. It latches c.dead so later StartExposure/GetDataAfterExp fail fast with
// ErrDeviceWedged. A matching firmware means a transient stall, so it returns false and the
// caller surfaces the ordinary short-frame error. The firmware read is one bounded control
// transfer (no bulk in flight), so it cannot itself hang.
func (c *Camera) checkDead() bool {
	if c.dead.Load() {
		return true
	}
	fw, err := c.FirmwareVersion()
	if err != nil || (c.baseFWok && fw != c.baseFW) {
		c.dead.Store(true)
		return true
	}
	return false
}

// GetDataAfterExp reads the captured frame into buf once the exposure has succeeded (gates on
// status==SUCCESS, then resets to IDLE). buf must be at least FrameBytes. It validates by size
// and retries with recovery (re-arm) on a failed/short read.
func (c *Camera) GetDataAfterExp(buf []byte) (int, error) {
	if c.dead.Load() {
		return 0, ErrDeviceWedged // already dead: don't issue reads that would hang
	}
	// Snapshot the state the read needs (INCLUDING the exposure generation), then release the
	// lock for the duration of the (multi-second) USB read so concurrent status polls / aborts
	// aren't blocked. Every status write below is generation-guarded: if a StopExposure +
	// StartExposure supersedes this capture mid-read, its writes become no-ops instead of
	// clobbering the new exposure's state.
	c.mu.Lock()
	st := c.status
	gen := c.expGen
	worker := c.sensor.Worker
	expDur := c.expDur
	c.mu.Unlock()
	if st != ExpWorking && st != ExpSuccess {
		return 0, fmt.Errorf("astrocam: no frame ready (status %s)", st)
	}
	// Per-sensor worker path: host-timed single-shot capture, doing the whole
	// arm → expose → re-arm → fire → read itself.
	if worker != nil {
		n, err := worker(workerCtl{c, gen}, buf, expDur)
		if err != nil {
			// Clean abort (StopExposure ran, or a new StartExposure superseded us): leave the
			// status alone (StopExposure already set Idle / the new exposure owns it) and
			// don't probe the firmware (that control transfer would interleave with the new
			// capture's readout).
			if c.abortedGen(gen) {
				return n, fmt.Errorf("astrocam: capture worker: %w", err)
			}
			c.setStatusIfGen(gen, ExpFailed)
			if c.checkDead() {
				return n, ErrDeviceWedged
			}
			return n, fmt.Errorf("astrocam: capture worker: %w", err)
		}
		if n < c.FrameBytes() {
			c.setStatusIfGen(gen, ExpFailed)
			if c.checkDead() { // FX3 firmware crash? re-read firmware; if it changed, latch dead
				return n, ErrDeviceWedged
			}
			return n, fmt.Errorf("astrocam: short frame (%d of %d bytes)", n, c.FrameBytes())
		}
		if RepairDMAMarkers && c.sensor.FX3DMAMarkers {
			repairFX3DMAMarkers(buf[:n], ModeOf(c.rm).BytesPerPx)
		}
		n = c.binFrame(buf, n)         // RAW16 host-side bin (no-op unless SoftBin>1)
		c.setStatusIfGen(gen, ExpIdle) // one-shot consume
		return n, nil
	}
	var lastErr error
	for attempt := 0; attempt < frameReadAttempts; attempt++ {
		if c.abortedGen(gen) {
			return 0, fmt.Errorf("astrocam: exposure aborted")
		}
		if attempt > 0 {
			// Recover the pipe and re-arm so the retry has a fresh frame to read.
			if err := c.recoverAndRearm(attempt, gen); err != nil {
				lastErr = fmt.Errorf("re-arm: %w", err)
				continue
			}
		}
		n, err := c.readFrame(buf)
		switch {
		case err != nil:
			lastErr = err
		case n < c.FrameBytes():
			// Validate by size: a short read is a dropped/stalled frame.
			lastErr = fmt.Errorf("short frame (%d of %d bytes)", n, c.FrameBytes())
		default:
			n = c.binFrame(buf, n)         // RAW16 host-side bin (no-op unless SoftBin>1)
			c.setStatusIfGen(gen, ExpIdle) // one-shot consume
			return n, nil
		}
	}
	c.setStatusIfGen(gen, ExpFailed)
	// Last resort: bus-reset so a wedged device is left clean. The reset wipes init state
	// (this run is lost), but the next Open/Init starts fresh.
	if r, ok := c.t.(DeviceResetter); ok {
		_ = r.ResetDevice()
	}
	return 0, fmt.Errorf("astrocam: frame read failed after %d attempts: %w", frameReadAttempts, lastErr)
}

// recoverAndRearm escalates pipe recovery, then re-arms a fresh exposure for the next read
// attempt. It does not bus-reset (that wipes init state); the bus reset is GetDataAfterExp's
// final give-up action only. Generation-guarded: it refuses to resurrect an aborted or
// superseded exposure and never drops the status to Idle mid-recovery (which would open a
// window for a concurrent StartExposure to double-arm).
func (c *Camera) recoverAndRearm(attempt int, gen uint64) error {
	if c.abortedGen(gen) {
		return fmt.Errorf("exposure aborted")
	}
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint) // clear any stall / leftover data
	}
	if attempt >= 2 {
		_ = c.vendorCmd(FX3Flush) // 0xAF on ZWO: flush the FX3 pipeline
	}
	if err := c.arm(); err != nil {
		return err
	}
	c.mu.Lock()
	if c.expGen == gen {
		c.expStart = nowFunc()
		c.status = ExpWorking
	}
	c.mu.Unlock()
	return nil
}

// ReadFrame reads one frame from the already-armed, running stream into buf (no arm/re-arm) and
// returns the bytes read. reset clears the bulk pipe first (a fresh frame boundary); pass false
// for back-to-back continuous reads. A complete frame gets the FX3 marker repair (RepairFrame).
func (c *Camera) ReadFrame(buf []byte, reset bool) (int, error) {
	if reset {
		if r, ok := c.t.(EndpointResetter); ok {
			_ = r.ResetEndpoint(bulkEndpoint)
		}
	}
	to := 2*c.expDuration() + 3*time.Second
	if to < 2*time.Second {
		to = 2 * time.Second
	}
	n, err := c.t.BulkRead(buf, to)
	if err == nil && n > 0 && n == c.FrameBytes() {
		c.RepairFrame(buf[:n])
	}
	return n, err
}

// readFrame fills buf with one whole-frame BulkRead; the backend supplies the transfer
// concurrency that primes the FX3 GPIF.
func (c *Camera) readFrame(buf []byte) (int, error) {
	// Flush the bulk pipe so the read starts at a fresh frame boundary.
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	// Size the timeout for 2× the exposure plus a readout/jitter margin: a free-run frame can
	// arrive up to one frame period after the exposure ends, so the bulk read must not abort
	// before it appears.
	to := 2*c.expDuration() + 3*time.Second
	if to < 2*time.Second {
		to = 2 * time.Second
	}
	return c.t.BulkRead(buf, to)
}

// StopExposure halts streaming and leaves the device clean: stop the sensor (master stop) and
// the FPGA pipeline, then flush, so the next session's control writes don't hit a
// still-streaming device.
func (c *Camera) StopExposure() error {
	c.mu.Lock()
	c.expAborted = true // the dedicated abort signal the in-flight worker polls
	c.status = ExpIdle
	c.mu.Unlock()
	// Break any frame read blocked in the transport BEFORE issuing the stop writes: a
	// whole-frame read holds ioMu for seconds, and the master-stop control transfers below
	// would queue behind it (an AbortExposure that hangs for the rest of the read). AbortRead is
	// level-triggered, so a stale worker's follow-up read also fails fast instead of
	// re-taking the bus; the next StartExposure/StartVideo re-arms reads.
	if ra, ok := c.t.(ReadAborter); ok {
		ra.AbortRead()
	}
	_ = c.vendorCmd(FX3StreamStop) // 0xAA on ZWO
	if c.sensor.StreamStop != nil {
		_ = c.sensor.StreamStop(c.rm) // master stop (0x200=1)
	}
	_ = c.fpgaStop()
	return c.vendorCmd(FX3Flush) // 0xAF on ZWO
}
