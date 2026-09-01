package astrocam

// Snap (single-frame) data plane: arm, host-time the exposure, read one whole frame.
//
// The generic arm sequence, shown with ZWO's opcodes (Camera.arm resolves them through the
// vendor's command table, and a profile with its own Sensor.Arm or Sensor.Worker replaces it):
// SendCMD 0xAA (stop) → FPGAStop (reg0 bit4 set: → 0x71) → sensor StreamStop (WriteSONYREG
// 0x200=1) → SendCMD 0xA9 (start) → sensor StreamStart (0x200=0) → FPGAStart (reg0 bit4 clear:
// → 0x61) → ResetEndPoint(0x81) → bulk read.
//
// Delivered bulk data is raw pixels from offset 0: the FX3 strips its internal 0xBB00AA11 frame
// delimiter, so frames are validated by size, not a header. Both vendors' firmware then writes a
// fixed marker over the first pixels, which RepairFrame puts back.

import (
	"errors"
	"fmt"
	"time"
)

// ErrDeviceWedged is returned once the camera is dead: a short/failed readout whose
// firmware-version recheck showed the FX3 dropped its firmware. Recovery is a USB reset
// (USBDEVFS_RESET) + re-scan + re-Open. Callers match it with errors.Is.
var ErrDeviceWedged = errors.New("astrocam: device wedged (FX3 firmware crash): needs USB reset + re-scan")

// ErrCaptureBusy is returned by StartExposure when an exposure is already claimed, so the
// requested one was not armed. Consume the claimed one with GetDataAfterExp or drop it with
// StopExposure, then arm again. Callers match it with errors.Is to tell a busy camera from a
// device fault.
var ErrCaptureBusy = errors.New("astrocam: capture already in progress")

// ErrFrameDesynced reports a frame whose FX3 DDR marker words are not at the frame boundaries:
// the read started mid-frame, so the buffer holds the tail of an earlier frame ahead of this
// one and every pixel is offset. The content still looks like a sky frame, so the marker check
// is the only reliable detector; without it the frame is saved and stacked as if it were good.
var ErrFrameDesynced = errors.New("astrocam: frame desynced (FX3 DDR markers not at the frame boundaries)")

// ZWO's FX3 streaming vendor commands (SendCMD); they seed ZWO.Cmds.
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

// zwoFPGARun is ZWO's readout run control: bit 4 of FPGA register 0 is the STOP flag, set to
// halt and cleared to run, read-modify-written so the rest of the mode byte survives.
func zwoFPGARun(rm Regmap, start bool) error {
	return SetFPGABit(rm, fpgaModeReg0, fpgaStopBit, !start)
}

// FPGARun implements WorkerCtl: the readout run control, in the vendor's encoding.
func (c *Camera) FPGARun(start bool) error { return c.vend.fpgaRun(c.rm, start) }

// fpgaStop / fpgaStart drive the readout through the vendor's encoding (Vendor.fpgaRun).
func (c *Camera) fpgaStop() error  { return c.vend.fpgaRun(c.rm, false) }
func (c *Camera) fpgaStart() error { return c.vend.fpgaRun(c.rm, true) }

// --- WorkerCtl: generic primitives a per-sensor capture Worker (Sensor.Worker) calls. ---

// Rm returns the Camera's Regmap (sensor + FPGA register R/W).
func (c *Camera) Rm() Regmap { return c.rm }

// VendorCmd issues a SendCMD-style FX3 vendor command (FX3StreamStop / FX3StreamStart /
// FX3Flush), resolved through the vendor's command table.
func (c *Camera) VendorCmd(op FX3Op) error { return c.vendorCmd(op) }

// drainScratch sizes DrainPipe's discard buffer. The residue measured on two desynced ASI6200MC
// frames was 229 KB and 213 KB, so one pass clears a typical one.
const drainScratch = 256 << 10

// DrainPipe discards whatever the device has already queued on the bulk-IN endpoint, so the next
// frame starts at a frame boundary instead of behind an earlier frame's tail.
//
// The FX3 keeps pushing into its DMA buffers between a read returning and the readout halting,
// and ResetEndpoint does not empty them: on Linux it is a bare USBDEVFS_CLEAR_HALT, which resets
// the data toggle and clears a stall but discards nothing. The leftover then heads the next
// frame, shifting every pixel by that many bytes — measured at 14 and 13 whole 16 KiB units on
// an ASI6200MC, with the FX3 DDR header word found that far into the buffer instead of at 0.
//
// Flush the FX3 pipeline, read the endpoint dry with a short timeout, then clear the pipe. An
// empty pipe costs one timeout. budget bounds the whole thing so a free-running sensor cannot
// hold the caller here. Returns the bytes discarded.
func (c *Camera) DrainPipe(budget time.Duration) int {
	if budget <= 0 {
		budget = 250 * time.Millisecond
	}
	_ = c.vendorCmd(FX3Flush)
	buf := make([]byte, drainScratch)
	discarded := 0
	for deadline := time.Now().Add(budget); time.Now().Before(deadline); {
		n, err := c.t.BulkRead(buf, 20*time.Millisecond)
		if err != nil || n <= 0 {
			break
		}
		discarded += n
	}
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	return discarded
}

// ResetEndpoint clears the bulk-IN pipe (EP 0x81) when the backend supports it.
func (c *Camera) ResetEndpoint() error {
	if r, ok := c.t.(EndpointResetter); ok {
		return r.ResetEndpoint(bulkEndpoint)
	}
	return nil
}

// ResetDevice performs a whole-device USB reset, the worker's last-resort recovery for a wedged
// readout. A backend without DeviceResetter (WinUSB has no device-level reset) returns an error
// so a recovery ladder's last rung fails instead of looping on a still-wedged device.
func (c *Camera) ResetDevice() error {
	if r, ok := c.t.(DeviceResetter); ok {
		c.coolW.invalidate() // the reset wipes the FPGA cooling registers
		return r.ResetDevice()
	}
	return fmt.Errorf("astrocam: transport has no device reset (recovery rung unavailable)")
}

// BulkRead reads from the bulk-IN endpoint with the given timeout.
func (c *Camera) BulkRead(buf []byte, to time.Duration) (int, error) { return c.t.BulkRead(buf, to) }

// BulkReadQuiet is BulkRead with the first `quiet` declared as host-timed integration
// (QuietBulkReader). Falls back to a fully-gated BulkRead on backends without it.
func (c *Camera) BulkReadQuiet(buf []byte, quiet, to time.Duration) (int, error) {
	if q, ok := c.t.(QuietBulkReader); ok {
		return q.BulkReadQuiet(buf, quiet, to)
	}
	return c.t.BulkRead(buf, to)
}

// NoteStall records one readout stall (a short/failed bulk read that triggered a re-arm).
func (c *Camera) NoteStall() {
	c.stalls.Add(1)
	// A stalled read during a free-running capture is a frame the sensor produced and the host
	// failed to collect, which is what the SDK counts as a dropped image.
	c.dropped.Add(1)
}

// ReapplyOffset rewrites the cached offset through the profile (WorkerCtl); a no-op for a
// profile without SetOffset.
func (c *Camera) ReapplyOffset() error {
	if c.sensor.SetOffset == nil {
		return nil
	}
	c.mu.Lock()
	off := c.offset
	c.mu.Unlock()
	return c.sensor.SetOffset(c.rm, off)
}

// StallCount returns the cumulative number of worker readout stalls over the camera's lifetime.
func (c *Camera) StallCount() int64 { return c.stalls.Load() }

// StreamFrame reads one frame via the windowed pump (FrameStreamer) when the backend provides
// it, else a single BulkRead.
func (c *Camera) StreamFrame(buf []byte, idle, total time.Duration) (int, error) {
	if fs, ok := c.t.(FrameStreamer); ok {
		return fs.ReadFrameStream(buf, idle, total)
	}
	return c.t.BulkRead(buf, total)
}

// StreamFramePrequeued reads one free-run frame with a pre-queued transfer batch
// (PrequeuedFrameStreamer), falling back to StreamFrame on backends without it.
func (c *Camera) StreamFramePrequeued(buf []byte, idle, total time.Duration) (int, error) {
	if p, ok := c.t.(PrequeuedFrameStreamer); ok {
		return p.ReadFrameStreamPrequeued(buf, idle, total)
	}
	return c.StreamFrame(buf, idle, total)
}

// FrameBytes is the size of one frame to read off the wire and the buffer a caller must
// allocate: width × height × bytes-per-pixel of the live output mode (SetOutputDepth, not the
// sensor ADC depth). For software binning (SoftBin>1) the sensor reads the full bin-scaled
// frame, so this is SoftBin² larger than the delivered image, which the read bins down to
// roiW·roiH·bpp.
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
	return w * h * bpp * sb * sb
}

// binFrame applies the host-side bin to a freshly read frame and returns the output byte count.
// SoftBin <= 1 is a no-op. The conventions follow the SDK, wire-checked on the ASI290 and ASI174
// for mono and the ASI6200MC for color.
func (c *Camera) binFrame(buf []byte, n int) int {
	m := ModeOf(c.rm)
	if m.SoftBin <= 1 {
		return n
	}
	if want := m.Width * m.Height * m.BytesPerPx; n < want || m.Width <= 0 || m.Height <= 0 {
		return n // not the expected full frame; leave it for the caller to flag
	}
	switch {
	case m.BytesPerPx < 2 && c.Color():
		return colorSumBinRAW8(buf, m.Width, m.Height, m.SoftBin)
	case m.BytesPerPx < 2:
		return sumBinRAW8(buf, m.Width, m.Height, m.SoftBin)
	case c.Color():
		return colorBinRAW16(buf, m.Width, m.Height, m.SoftBin)
	}
	return averageBinRAW16(buf, m.Width, m.Height, m.SoftBin)
}

// sumBinRAW8 sums bin×bin blocks of a fullW×fullH RAW8 frame in buf, clipped at 255 (the SDK's
// 8-bit MonoBin), writing the (fullW/bin)×(fullH/bin) result into the front of the same buffer,
// and returns the output length. In-place is safe for the same reason as averageBinRAW16.
func sumBinRAW8(buf []byte, fullW, fullH, bin int) int {
	outW, outH := fullW/bin, fullH/bin
	for oy := 0; oy < outH; oy++ {
		for ox := 0; ox < outW; ox++ {
			sum := 0
			for dy := 0; dy < bin; dy++ {
				base := (oy*bin+dy)*fullW + ox*bin
				for dx := 0; dx < bin; dx++ {
					sum += int(buf[base+dx])
				}
			}
			if sum > 0xff {
				sum = 0xff
			}
			buf[oy*outW+ox] = byte(sum)
		}
	}
	return outW * outH
}

// colorBinRAW16 is the color host bin, the SDK's rule wire-matched on the ASI6200MC at bin 2 and
// 4. Every 2·bin × 2·bin block of the mosaic yields one 2×2 RGGB output cell, each output pixel
// the mean of the bin² same-color pixels of that block, so the output stays a valid mosaic at
// (fullW/bin)×(fullH/bin). cnt clamps a partial edge block. It runs in place: in scan order every
// later output pixel reads from beyond the current write, so a forward pass never overwrites
// unread input.
func colorBinRAW16(buf []byte, fullW, fullH, bin int) int {
	outW, outH := fullW/bin, fullH/bin
	for oy := 0; oy < outH; oy++ {
		row0 := 2*bin*(oy/2) + oy&1 // block origin + phase row
		for ox := 0; ox < outW; ox++ {
			col0 := 2*bin*(ox/2) + ox&1
			sum, cnt := 0, 0
			for i := 0; i < bin; i++ {
				r := row0 + 2*i
				if r >= fullH {
					break
				}
				rb := r * fullW
				for j := 0; j < bin; j++ {
					cc := col0 + 2*j
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
			buf[o] = byte(v)
			buf[o+1] = byte(v >> 8)
		}
	}
	return outW * outH * 2
}

// colorSumBinRAW8 is colorBinRAW16's RAW8 form: the same block-to-cell mapping, each output the
// sum of its bin² same-color pixels clipped at 255. Wire-matched on the ASI6200MC, where bin 2
// reads 4× the bin-1 level per phase.
func colorSumBinRAW8(buf []byte, fullW, fullH, bin int) int {
	outW, outH := fullW/bin, fullH/bin
	for oy := 0; oy < outH; oy++ {
		row0 := 2*bin*(oy/2) + oy&1
		for ox := 0; ox < outW; ox++ {
			col0 := 2*bin*(ox/2) + ox&1
			sum := 0
			for i := 0; i < bin; i++ {
				r := row0 + 2*i
				if r >= fullH {
					break
				}
				rb := r * fullW
				for j := 0; j < bin; j++ {
					cc := col0 + 2*j
					if cc >= fullW {
						break
					}
					sum += int(buf[rb+cc])
				}
			}
			if sum > 0xff {
				sum = 0xff
			}
			buf[oy*outW+ox] = byte(sum)
		}
	}
	return outW * outH
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

// StartExposure arms a single exposure and sets the status to Working. GetDataAfterExp reads the
// frame once GetExpStatus reports Success. light is accepted for API parity: none of the supported
// cameras has a mechanical shutter.
func (c *Camera) StartExposure(light bool) error {
	c.dropped.Store(0) // the count is per capture, as the SDK's is
	if c.dead.Load() {
		return ErrDeviceWedged
	}
	// The busy check and the Working transition are one critical section, so two StartExposure
	// calls cannot double-arm. Claiming bumps the generation and clears the abort flag, turning a
	// stale capture's generation-guarded writes into no-ops.
	c.mu.Lock()
	if c.status == ExpWorking {
		c.mu.Unlock()
		return ErrCaptureBusy // the requested exposure was not armed; the claimed one runs on
	}
	c.status = ExpWorking
	c.expStart = nowFunc()
	c.expGen++
	gen := c.expGen
	c.expAborted = false
	c.mu.Unlock()
	// Clear a prior StopExposure's read-abort latch (ReadAborter).
	if ra, ok := c.t.(ReadAborter); ok {
		ra.ArmRead()
	}
	// A sensor Worker arms, host-times, and reads inside GetDataAfterExp; the non-worker path
	// arms here, unlocked.
	if c.sensor.Worker == nil {
		if err := c.arm(); err != nil {
			c.setStatusIfGen(gen, ExpIdle) // release the claim (unless superseded meanwhile)
			return err
		}
	}
	return nil
}

// setStatusIfGen stores the exposure status only if the exposure generation is still gen, so a
// superseded capture cannot clobber the new exposure's state.
func (c *Camera) setStatusIfGen(gen uint64, s ExposureStatus) {
	c.mu.Lock()
	if c.expGen == gen {
		c.status = s
	}
	c.mu.Unlock()
}

// abortedGen reports whether the generation-gen exposure is aborted: StopExposure ran, or a
// newer StartExposure superseded it. It is independent of status (see expAborted).
func (c *Camera) abortedGen(gen uint64) bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.expAborted || c.expGen != gen
}

// Aborted reports that the current exposure was aborted (StopExposure ran). A host-timed worker
// polls it (via its generation-bound WorkerCtl) so an abort cuts the integration short.
func (c *Camera) Aborted() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.expAborted
}

// workerCtl is the WorkerCtl handed to a sensor Worker: the Camera plus the worker's exposure
// generation, so Aborted() also fires when a newer StartExposure superseded the worker.
type workerCtl struct {
	*Camera
	gen uint64
}

func (w workerCtl) Aborted() bool { return w.abortedGen(w.gen) }

// expDuration returns the last-set exposure under the lock (it sizes the bulk read timeouts).
func (c *Camera) expDuration() time.Duration {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.expDur
}

// arm issues the arm sequence: the sensor's own Arm hook when it has one, else SendCMD 0xAA →
// FPGA stop → sensor master stop → SendCMD 0xA9 → sensor master start → 10 ms settle → FPGA
// start (clear reg0 bit4, so frames reach EP 0x81). Shared by StartExposure, StartVideo and the
// GetDataAfterExp recovery path.
func (c *Camera) arm() error {
	if c.sensor.Arm != nil {
		return c.sensor.Arm(c)
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

// StartVideo arms the sensor for continuous free-run streaming and does not read, so frames are
// pulled back-to-back with no per-frame re-arm. It un-halts the readout and keeps WaitMode as
// SetExposure left it: the 174 stream runs at reg0=0x61. Short exposures only, since
// long-exposure trigger mode stays single-shot.
func (c *Camera) StartVideo(light bool) error {
	c.dropped.Store(0) // the count is per capture, as the SDK's is
	if c.dead.Load() {
		return ErrDeviceWedged
	}
	// A claimed single-shot may still be reading, and a gated whole-frame read holds the
	// transport's I/O gate for seconds, so the arm writes below would queue behind it. The
	// generation bump below discards its result and the ArmRead at the end clears the latch.
	c.mu.Lock()
	busy := c.status == ExpWorking
	c.mu.Unlock()
	if busy {
		if ra, ok := c.t.(ReadAborter); ok {
			ra.AbortRead()
		}
	}
	if err := c.arm(); err != nil {
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
		ra.ArmRead()
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

// GetExpStatus reports the snap exposure status: Working while the exposure is in flight, Success
// once the host-timed window has elapsed. The Success at window end is derived, not stored, so a
// poll never changes what the worker still integrating observes.
//
// It serves a client that has to report progress (Alpaca ImageReady), not a step on the way to the
// pixels. Every profile with a Worker integrates inside GetDataAfterExp, so polling to Success and
// only then reading waits out the exposure twice.
func (c *Camera) GetExpStatus() ExposureStatus {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.status == ExpWorking && nowFunc().Sub(c.expStart) >= c.expDur {
		return ExpSuccess
	}
	return c.status
}

// frameReadAttempts is how many times GetDataAfterExp tries to read a valid frame, recovering
// and re-arming between tries.
const frameReadAttempts = 3

// RepairDMAMarkers controls whether captured frames have their FX3 DDR frame header/footer
// marker words stripped (on by default). Set false to receive the raw frame with the marker
// pixels intact.
var RepairDMAMarkers = true

// repairFX3DMAMarkers strips the FX3 bridge's frame markers. The FX3 brackets every DDR frame with
// a fixed header word (0x00005A7E low half) and footer word (0x3CF0 high half), so the first and
// last 32-bit DMA word are not sensor data: 2 pixels each in RAW16, 4 each in RAW8. Confirmed on
// IMX455 and IMX462. The signature is detected at the start and end, so this is a no-op on frames
// without the markers.
//
// Each marker word takes the pixels at the same columns a whole number of rows away, holding the
// column so the horizontal phase survives. rows is the step: two on a colour sensor, the Bayer
// period, and one without a CFA. The SDK does the same, measured on four cameras: its row 0 equals
// its row 2 byte for byte on the ASI6200MC and ASI462MC, and its row 1 on the mono ASI174MM and
// ASI290MM.
//
// width is the wire frame's width in pixels, the sensor-side window, since the repair precedes any
// host bin. Without a usable width the copy stays in the row but still steps a whole marker word,
// an even number of samples, so the phase survives.
func repairFX3DMAMarkers(buf []byte, bpp, width, rows int) bool {
	n := len(buf)
	if bpp < 1 || n < 8 || n%bpp != 0 {
		return false
	}
	if buf[0] != 0x7E || buf[1] != 0x5A || buf[n-2] != 0xF0 || buf[n-1] != 0x3C {
		return false
	}
	src := rows * width * bpp
	if width <= 0 || rows <= 0 || src+4 > n || n-4-src < 0 {
		src = 4 // geometry unknown, or a frame too short for the step
	}
	copy(buf[0:4], buf[src:src+4])      // header word <- `rows` down, same columns
	copy(buf[n-4:], buf[n-4-src:n-src]) // footer word <- `rows` up, same columns
	return true
}

// fx3MarkerOffset returns the byte offset of an FX3 DDR frame boundary inside buf, or -1 when
// there is none. A boundary is the previous frame's footer word immediately followed by the next
// frame's header word, 32-bit aligned:
//
//	00 00 F0 3C | 7E 5A nn 00
//	  footer         header (third byte is the free-run frame counter)
//
// Both words are required, so a lone 0x7E 0x5A pair in sensor noise cannot trigger it. A hit at
// a non-zero offset means the read started that many bytes before the frame did: the buffer
// holds the tail of an earlier frame first. Only called after the boundary check has failed.
func fx3MarkerOffset(buf []byte) int {
	for i := 4; i+2 <= len(buf); i += 4 {
		if buf[i] == 0x7E && buf[i+1] == 0x5A && buf[i-2] == 0xF0 && buf[i-1] == 0x3C {
			return i
		}
	}
	return -1
}

// markerRows is how many rows the FX3 marker repair steps to find a replacement pixel: the Bayer
// period on a colour sensor, the nearest row without a CFA.
func (c *Camera) markerRows() int {
	if c.Color() {
		return 2
	}
	return 1
}

// BinFrame applies the host-side bin to a wire frame read from a free-run path, the step
// GetDataAfterExp performs itself. Call RepairFrame first.
func (c *Camera) BinFrame(buf []byte, n int) int { return c.binFrame(buf, n) }

// RepairFrame applies the FX3 DDR frame-marker repair when the profile has the markers and
// RepairDMAMarkers is on, then the vendor's own frame marker and the factory defect map. In
// free-run the header word's third byte is a frame counter, so an unrepaired stream shows the
// corner pixel stepping 1, 2, 3, and on.
//
// It reports whether the FX3 markers sat at the frame boundaries. False means the read started
// mid-frame; the caller locates the boundary with fx3MarkerOffset and drops the frame. A profile
// with no FX3 markers, or the repair switched off, reports true — there is nothing to check.
func (c *Camera) RepairFrame(buf []byte) bool {
	if !RepairDMAMarkers {
		return true
	}
	m := ModeOf(c.rm)
	aligned := true
	if c.sensor.FX3DMAMarkers {
		aligned = repairFX3DMAMarkers(buf, m.BytesPerPx, m.Width, c.markerRows())
	}
	if c.vend.frameMarker != nil {
		c.vend.frameMarker(buf, m.BytesPerPx, m.Width, c.markerRows())
	}
	// The factory defect map, as the vendor SDKs apply it: on every frame, not on request. It is
	// skipped for binned frames, where a defect is diluted into a block average and the map's
	// coordinates no longer address the samples in the buffer.
	if c.defects != nil && RepairDefects {
		c.mu.Lock()
		x, y, bin := c.roiX, c.roiY, c.bin
		c.mu.Unlock()
		if bin <= 1 {
			c.defects.ApplyWindow(buf, m.BytesPerPx, x, y, m.Width, m.Height)
		}
	}
	return aligned
}

// RepairDefects controls whether captured frames have their factory hot-pixel map applied. On by
// default, matching both vendor SDKs; set false for the uncorrected sensor output.
var RepairDefects = true

// Wedged reports whether the camera has been marked dead by the firmware-crash check (see
// ErrDeviceWedged / checkDead).
func (c *Camera) Wedged() bool { return c.dead.Load() }

// checkDead re-reads the firmware version after a short or failed readout and compares it to the
// Init baseline. A changed or unreadable version means the FX3 dropped its firmware and latches
// c.dead; a matching version means a transient stall. The firmware read is one bounded control
// transfer with no bulk in flight, so it cannot itself hang.
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
// status Working/Success, then resets to Idle). buf must be at least FrameBytes. It validates
// by size and retries with recovery (re-arm) on a failed/short read.
func (c *Camera) GetDataAfterExp(buf []byte) (int, error) {
	if c.dead.Load() {
		return 0, ErrDeviceWedged
	}
	// A buffer smaller than the frame is a caller error, refused before the device is touched. A
	// Worker clamps its read to len(buf), and the frame check below then reads the clamped count as
	// a short frame, fails the exposure, and fires the firmware-crash probe at a working camera.
	if fb := c.FrameBytes(); len(buf) < fb {
		return 0, fmt.Errorf("astrocam: frame buffer too small: %d bytes, need %d (FrameBytes)", len(buf), fb)
	}
	// Snapshot the state the read needs, including the exposure generation, then release the
	// lock for the multi-second USB read. Every status write below is generation-guarded.
	c.mu.Lock()
	st := c.status
	gen := c.expGen
	worker := c.sensor.Worker
	expDur := c.expDur
	c.mu.Unlock()
	if st != ExpWorking && st != ExpSuccess {
		return 0, fmt.Errorf("astrocam: no frame ready (status %s)", st)
	}
	// Per-sensor worker path: the worker does the whole arm → expose → re-arm → fire → read.
	if worker != nil {
		n, err := worker(workerCtl{c, gen}, buf, expDur)
		if err != nil {
			// Clean abort: leave the status alone, and skip the firmware probe, whose control
			// transfer would interleave with the new capture's readout.
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
			if c.checkDead() {
				return n, ErrDeviceWedged
			}
			return n, fmt.Errorf("astrocam: short frame (%d of %d bytes)", n, c.FrameBytes())
		}
		if !c.RepairFrame(buf[:n]) { // vendor frame markers, before any host-side binning
			// Only a located frame boundary proves a desync. A frame carrying no markers at
			// all says nothing (an undecoded mode, a stub), so it passes as it always did.
			if at := fx3MarkerOffset(buf[:n]); at > 0 {
				c.setStatusIfGen(gen, ExpFailed)
				return n, fmt.Errorf("%w: frame starts at byte %d of %d, so the read began %d bytes (%d x 16 KiB) early",
					ErrFrameDesynced, at, n, n-at, (n-at)/16384)
			}
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
			lastErr = fmt.Errorf("short frame (%d of %d bytes)", n, c.FrameBytes())
		default:
			n = c.binFrame(buf, n)         // RAW16 host-side bin (no-op unless SoftBin>1)
			c.setStatusIfGen(gen, ExpIdle) // one-shot consume
			return n, nil
		}
	}
	c.setStatusIfGen(gen, ExpFailed)
	// Last resort: bus-reset so a wedged device is left clean. The reset wipes init state; the
	// next Open/Init starts fresh.
	if r, ok := c.t.(DeviceResetter); ok {
		c.coolW.invalidate()
		_ = r.ResetDevice()
	}
	return 0, fmt.Errorf("astrocam: frame read failed after %d attempts: %w", frameReadAttempts, lastErr)
}

// recoverAndRearm escalates pipe recovery, then re-arms a fresh exposure for the next read
// attempt. It does not bus-reset, which is GetDataAfterExp's final give-up action. It refuses to
// resurrect an aborted or superseded exposure and never drops the status to Idle mid-recovery,
// which would let a concurrent StartExposure double-arm.
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
		if !c.RepairFrame(buf[:n]) {
			// A desynced free-run frame is reported, not returned as good: the caller retries
			// and the next read starts on a boundary. Publishing it would ship shifted pixels.
			if at := fx3MarkerOffset(buf[:n]); at > 0 {
				return n, fmt.Errorf("%w: frame starts at byte %d of %d, so the read began %d bytes early",
					ErrFrameDesynced, at, n, n-at)
			}
		}
	}
	return n, err
}

// readFrame resets the bulk pipe (a fresh frame boundary) and fills buf with one whole-frame
// BulkRead. The timeout is 2× the exposure plus a 3 s margin (minimum 2 s): a free-run frame can
// arrive up to one frame period after the exposure ends.
func (c *Camera) readFrame(buf []byte) (int, error) {
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	to := 2*c.expDuration() + 3*time.Second
	if to < 2*time.Second {
		to = 2 * time.Second
	}
	return c.t.BulkRead(buf, to)
}

// StopExposure aborts the exposure and leaves the device clean: read-abort, SendCMD stop, sensor
// master stop, FPGA stop, flush. AbortRead runs before the stop writes, so they do not queue
// behind a frame read holding ioMu.
func (c *Camera) StopExposure() error {
	c.mu.Lock()
	c.expAborted = true // the abort signal the in-flight worker polls
	c.status = ExpIdle
	c.mu.Unlock()
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
