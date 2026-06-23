package astrocam

// Snap (single-frame) data plane, built around StartCapture / the post-exposure image read (the
// status gate) and the capture worker.
//
// The SDK runs capture on a worker thread that pumps an ASYNC bulk stream
// (initAsyncXfer/startAsyncXfer, 1 MiB chunks) and advances a status word at
// the worker-state field (0 idle / 1 working / 2 success / 3 failed). For a single snap we use a
// single whole-frame bulk read (the mac backend issues N concurrent async transfers
// internally to prime the FX3 GPIF): arm, then read until FrameBytes arrives.
//
// Arm sequence (the capture worker, wire-confirmed): SendCMD 0xAA (stop)
// → FPGAStop (reg0 bit4: → 0x71) → sensor StreamStop (WriteSONYREG 0x200=1) → SendCMD
// 0xA9 (start) → sensor StreamStart (0x200=0) → FPGAStart (reg0 bit4 clear: → 0x61) →
// ResetEndPoint(0x81) → bulk read. The generic SendCMD/FPGA/endpoint parts are here;
// the sensor master stop/start is the Sensor.StreamStop/StreamStart hook.
//
// HARDWARE-VALIDATED (ASI174MM Mini): this path streamed a real 4.77MB frame. The
// delivered bulk data is RAW pixels from offset 0 — the FX3 stream's 0xBB00AA11 frame
// magic is internal and stripped before readout, so we validate by SIZE, not a header.

import (
	"fmt"
	"time"
)

// FrameMagic is the FX3 stream's INTERNAL frame delimiter (0xBB00AA11). It is NOT
// present in the bulk-delivered frame — the FX3 strips it on readout, so the data the
// host receives is raw pixels from offset 0 (wire-confirmed). Kept for reference only;
// the post-exposure data read validates frames by size, not by this header.
const FrameMagic uint32 = 0xBB00AA11

// Generic FX3 streaming vendor commands (SendCMD), from the capture worker.
const (
	cmdStreamStop  = 0xAA // stop/prepare before (re)arming
	cmdStreamStart = 0xA9 // begin streaming
	cmdFlush       = 0xAF // pipeline flush / drop recovery
	bulkEndpoint   = 0x81 // bulk-IN endpoint ("EP9")
)

// Async-pump sizing, from initAsyncXfer: 1 MiB transfers with a
// ~200 MB in-flight budget (min(0xC800000/xferLen, N) buffers). bulkChunkBytes is
// a var only so tests can shrink it; production is 1 MiB.
var bulkChunkBytes = 1 << 20 // xferLen 0x100000

const bulkBudgetBytes = 0xC800000 // 200 MB in-flight budget

// pidNeedsCapBit reports whether StartCapture's PID-gated FPGA reg-0x45 capture
// bit applies (StartCapture special-cases these PIDs).
func pidNeedsCapBit(pid uint16) bool { return pid == 0x461e || pid == 0x411e }

// FPGA mode register 0: bit4 stops the readout pipeline (FPGAStop sets
// it, FPGAStart clears it). RMW the live register.
const (
	fpgaModeReg0 = 0x00
	fpgaStopBit  = 0x10
)

func (c *Camera) fpgaSetReg0(mask, val uint16) error {
	v, err := c.rm.ReadFPGAReg(fpgaModeReg0)
	if err != nil {
		return fmt.Errorf("asicam: read FPGA reg0: %w", err)
	}
	return c.rm.WriteFPGAReg(fpgaModeReg0, ((v&^mask)|val)&0xff)
}

func (c *Camera) fpgaStop() error  { return c.fpgaSetReg0(fpgaStopBit, fpgaStopBit) }
func (c *Camera) fpgaStart() error { return c.fpgaSetReg0(fpgaStopBit, 0) }

// --- WorkerCtl: the generic primitives a per-sensor capture Worker (Sensor.Worker) calls
// around its own sensor-register choreography. ---
func (c *Camera) Rm() Regmap                { return c.rm }
func (c *Camera) VendorCmd(cmd uint8) error { return c.vendorCmd(cmd) }
func (c *Camera) ResetEndpoint() error {
	if r, ok := c.t.(EndpointResetter); ok {
		return r.ResetEndpoint(bulkEndpoint)
	}
	return nil
}

// ResetDevice performs a whole-device USB reset when the backend supports it
// (DeviceResetter); a no-op otherwise. It is the worker's last-resort recovery for a
// wedged readout.
func (c *Camera) ResetDevice() error {
	if r, ok := c.t.(DeviceResetter); ok {
		return r.ResetDevice()
	}
	return nil
}
func (c *Camera) BulkRead(buf []byte, to time.Duration) (int, error) { return c.t.BulkRead(buf, to) }

// StreamFrame reads one frame with the continuous windowed pump when the backend
// provides it (darwin readFrameStream), else via the BulkStreamer window (linux/
// windows), else a single BulkRead. The first path is the only one that reliably
// pulls a large USB3 frame without truncating at a burst boundary; see WorkerCtl.
func (c *Camera) StreamFrame(buf []byte, idle, total time.Duration) (int, error) {
	if fs, ok := c.t.(FrameStreamer); ok {
		return fs.ReadFrameStream(buf, idle, total)
	}
	// BulkStreamer fallback: cycle a window of chunks, copy contiguously to buf, stop
	// when buf is full or a chunk read stalls/ends short.
	if s, ok := c.t.(BulkStreamer); ok {
		n := (len(buf) + bulkChunkBytes - 1) / bulkChunkBytes
		if max := bulkBudgetBytes / bulkChunkBytes; n > max {
			n = max
		}
		st, err := s.BulkStream(bulkChunkBytes, n)
		if err != nil {
			return 0, fmt.Errorf("asicam: open bulk stream: %w", err)
		}
		defer st.Close()
		got := 0
		for got < len(buf) {
			b, err := st.Next(idle)
			if err != nil {
				return got, nil // stall/timeout: let the worker recover and continue
			}
			got += copy(buf[got:], b)
			if len(b) == 0 {
				break
			}
		}
		return got, nil
	}
	return c.t.BulkRead(buf, total)
}

// FrameBytes is the size of one frame to READ off the wire and the buffer a caller must
// allocate for the post-exposure data read: width × height × bytes-per-pixel (RAW16 → 2, RAW8 → 1) of the
// live OUTPUT mode. Output depth is the readout mode (SetOutputDepth), not the sensor ADC
// depth — a 12/16-bit ADC can still emit RAW8. For RAW16 software binning (SoftBin>1) the
// sensor reads the FULL bin·-scaled frame, so this is SoftBin² larger than the delivered
// image; the post-exposure data read averages it down and returns the smaller OUTPUT byte count (the value
// to use for the image itself / ImageBytes is that returned n, equivalently roiW·roiH·bpp).
// For every non-software-bin path SoftBin==1 and this is exactly the delivered frame size.
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
	// FrameBytes is the WIRE/read size. For RAW16 software binning the sensor reads the FULL
	// bin·-scaled frame (sb× per axis), which the read path then averages down to the w×h
	// output. sb==1 for every non-software-bin path, so this is unchanged there.
	return w * h * bpp * sb * sb
}

// binFrame applies RAW16 host-side binning to a freshly read FULL-resolution frame, the way
// the SDK does (GetImage reads the full 16-bit frame and calls MonoBin) because these
// sensors have no hardware 16-bit binned mode. With
// SoftBin<=1 or RAW8 it is a no-op and returns n unchanged; otherwise it averages bin×bin in
// place and returns the (smaller) output byte count. m.Width/Height are the full readout dims.
func (c *Camera) binFrame(buf []byte, n int) int {
	m := ModeOf(c.rm)
	if m.SoftBin <= 1 || m.BytesPerPx < 2 {
		return n
	}
	if want := m.Width * m.Height * 2; n < want || m.Width <= 0 || m.Height <= 0 {
		return n // not the expected full RAW16 frame — leave it for the caller to flag
	}
	if c.Color() {
		return colorBinRAW16(buf, m.Width, m.Height, m.SoftBin)
	}
	return averageBinRAW16(buf, m.Width, m.Height, m.SoftBin)
}

// colorBinRAW16 is the Bayer-preserving equivalent of averageBinRAW16 (the SDK's
// ColorRAWBin path): it averages only SAME-COLOR samples so the binned output stays
// a valid mosaic. Output is the floor dims (fullW/bin)×(fullH/bin) — same as the SDK, which for
// the 6200 emits e.g. bin4 = 2394×1597 (odd height, since 4 divides 6388 exactly). Each output
// pixel (oy,ox) keeps Bayer phase (oy%2,ox%2) and averages the same-phase pixels inside the
// bin×bin source block at (bin·oy, bin·ox) — for an even bin those are the (bin/2)² same-color
// samples in that block. bin 2 therefore takes 1 sample per output pixel (a same-color
// decimation); bin 4 averages 4. Edge clamping (cnt) guards a partial block; for exact divisors
// (the only bins SetROI allows for color) no clamping occurs. Scratch output avoids in-place
// aliasing. Validated on the 6200MC at bin 2 and bin 4 (per-pixel match to the SDK).
func colorBinRAW16(buf []byte, fullW, fullH, bin int) int {
	outW, outH := fullW/bin, fullH/bin
	out := make([]byte, outW*outH*2)
	for oy := 0; oy < outH; oy++ {
		py := oy & 1 // Bayer row phase (bin is even → source block starts on an even row)
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

// averageBinRAW16 averages bin×bin blocks of a fullW×fullH RAW16 (16-bit little-endian) frame
// stored in buf, writing the (fullW/bin)×(fullH/bin) result back into the FRONT of the same
// buffer, and returns the output length in bytes. Rounded mean (sum + bin²/2)/bin², matching
// the SDK's averaging binning (level preserved, read noise down by the box size). In place is
// safe: each output pixel's source bytes sit at an offset ≥ its own, so a forward scan never
// overwrites unread input.
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

// StartExposure arms a single (snap) exposure. Mirrors StartExposure→StartCapture(1):
// it sets the PID-gated FPGA capture bit, brackets the sensor's master start with
// the generic FX3 stream stop/start commands, and seeds the exposure status to
// Working. The frame is then read with the post-exposure data read once GetExpStatus reports
// Success. light selects a light vs dark frame (the FPGA capture-start bit).
func (c *Camera) StartExposure(light bool) error {
	c.mu.Lock()
	busy := c.status == ExpWorking
	c.mu.Unlock()
	if busy {
		return nil // already capturing — StartCapture no-ops a busy camera
	}
	// A sensor Worker (the capture worker) arms, host-times, and reads inside
	// the post-exposure data read (the SDK runs it on a thread; we run it synchronously there), so
	// the worker path seeds state only. The non-worker path arms here — USB I/O, run
	// unlocked. Either way the state transition is the locked tail.
	if c.sensor.Worker == nil {
		if err := c.arm(light); err != nil {
			return err
		}
	}
	c.mu.Lock()
	c.status = ExpWorking
	c.expStart = nowFunc()
	c.mu.Unlock()
	return nil
}

// setStatus stores the exposure status under the lock — the one-line state mutation the
// capture path uses between (unlocked) USB operations.
func (c *Camera) setStatus(s ExposureStatus) {
	c.mu.Lock()
	c.status = s
	c.mu.Unlock()
}

// Aborted reports that the exposure is no longer in flight — i.e. StopExposure ran (it sets
// ExpIdle) while a capture worker was integrating. A host-timed worker polls this so an abort
// cuts the integration short instead of waiting out the full exposure (which would pin the
// readout, the driver's AbortExposure join, and the client's abort request for the whole exposure).
func (c *Camera) Aborted() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.status != ExpWorking
}

// expDuration returns the last-set exposure under the lock — used by the read paths to
// size their bulk timeout while SetExposure may be updating it from another goroutine.
func (c *Camera) expDuration() time.Duration {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.expDur
}

// arm issues the capture worker's arm sequence: the PID-gated FPGA capture bit, then SendCMD
// 0xAA → FPGA stop → sensor master stop → SendCMD 0xA9 → sensor master start → settle →
// FPGA start. The FPGA start (clear reg0 bit4) un-halts the readout pipeline so frames
// reach EP 0x81. Shared by StartExposure and the post-exposure data read's recovery path, which
// re-arms to stream a fresh frame after a failed read.
func (c *Camera) arm(light bool) error {
	// PID-gated FPGA capture-start bit (reg 0x45 bit1) for the models that need it.
	if pidNeedsCapBit(c.pid) {
		cur, err := c.rm.ReadFPGAReg(0x45)
		if err != nil {
			return fmt.Errorf("asicam: read FPGA 0x45: %w", err)
		}
		v := cur &^ 0x2
		if light {
			v |= 0x2
		}
		if err := c.rm.WriteFPGAReg(0x45, v); err != nil {
			return fmt.Errorf("asicam: write FPGA 0x45: %w", err)
		}
	}
	if err := c.vendorCmd(cmdStreamStop); err != nil {
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
	if err := c.vendorCmd(cmdStreamStart); err != nil {
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

// StartVideo arms the sensor for CONTINUOUS free-run streaming (the video / .ser burst
// path). It runs the capture arm but does NOT read — the sensor then free-runs at the
// readout rate and frames are pulled with a FrameStream session (StartStream), so there is
// no per-frame re-arm. It also clears the FPGA WaitMode bit (reg0 bit6) that SetExposure's
// ApplyExposure sets, since a free-running stream must not be parked in wait mode (the
// single-shot worker's FPGAStart does the same 0x50 clear; c.arm's fpgaStart cleared only
// bit4). Short (non-trigger) exposures only; long-exposure trigger mode stays single-shot.
func (c *Camera) StartVideo(light bool) error {
	if err := c.arm(light); err != nil {
		return err
	}
	if err := c.fpgaSetReg0(fpgaStopBit|0x40, 0); err != nil { // clear readout-stop (bit4) + WaitMode (bit6)
		return err
	}
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	c.mu.Lock()
	c.status = ExpWorking
	c.expStart = nowFunc()
	c.mu.Unlock()
	return nil
}

// StartStream opens a resident windowed-stream session (FrameStream) sized to the current
// frame, when the backend supports it. Returns an error on backends without a stream
// session so the caller can fall back to the per-frame read path.
func (c *Camera) StartStream(total time.Duration) (FrameStream, error) {
	ss, ok := c.t.(StreamStarter)
	if !ok {
		return nil, fmt.Errorf("asicam: backend has no resident stream session")
	}
	return ss.StartStream(c.FrameBytes(), total)
}

// nowFunc is the clock the host-timed status poll uses; overridable in tests.
var nowFunc = time.Now

// GetExpStatus reports the snap exposure status. While the exposure is in flight
// the SDK forces WORKING; once the host-timed exposure window has elapsed the
// frame is on its way over bulk, so we surface SUCCESS (ready for readout). This
// mirrors the ASIGetExpStatus contract (poll until SUCCESS, then the post-exposure data read).
func (c *Camera) GetExpStatus() ExposureStatus {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.status == ExpWorking && nowFunc().Sub(c.expStart) >= c.expDur {
		c.status = ExpSuccess
	}
	return c.status
}

// frameReadAttempts is how many times the post-exposure data read tries to read a valid frame,
// recovering and re-arming between tries. A bulk read can fail recoverably two ways:
// a transient pipe dropout, or a partial/stalled frame — both are fixed by clearing the
// pipe and re-arming a fresh exposure.
const frameReadAttempts = 3

// GetDataAfterExp reads the captured frame into buf once the exposure has succeeded
// (the post-exposure image read gates on status==SUCCESS, then resets to IDLE). buf
// must be at least FrameBytes. It validates the frame by size and RETRIES with recovery
// on a failed/short read, mirroring the SDK's re-arm-and-retry rather than failing the
// whole capture on a single transient hiccup.
// RepairDMAMarkers controls whether captured frames have their FX3 DDR frame header/footer
// marker words stripped (on by default). Set false to receive the genuine raw frame with the
// marker pixels intact — for wire analysis or to handle them downstream.
var RepairDMAMarkers = true

// repairFX3DMAMarkers strips the FX3 bridge's frame markers. The FX3 brackets every DDR frame
// with a fixed header word (0x00005A7E low half) and footer word (0x3CF0 high half) — so the
// first and last 32-bit DMA word are NOT sensor data (2 pixels each in RAW16, 4 each in RAW8).
// This is an FX3-bridge artifact common to these cameras (confirmed on IMX455 + IMX462), gated
// per-sensor by Sensor.FX3DMAMarkers. It is also signature-detected (the 0x5A7E/0x3CF0 byte
// pattern at the very start/end), so it is a safe no-op on any frame that does not carry the
// markers; when present, each marker word is overwritten by edge-replicating the nearest real
// pixel. bpp is the output bytes-per-pixel (1 = RAW8, 2 = RAW16).
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

func (c *Camera) GetDataAfterExp(buf []byte) (int, error) {
	// Snapshot the state the read needs, then release the lock for the duration of the
	// (multi-second) USB read so concurrent status polls / aborts aren't blocked.
	c.mu.Lock()
	st := c.status
	worker := c.sensor.Worker
	expDur := c.expDur
	c.mu.Unlock()
	if st != ExpWorking && st != ExpSuccess {
		return 0, fmt.Errorf("asicam: no frame ready (status %s)", st)
	}
	// Per-sensor worker path (the capture worker): the host-timed single-shot
	// capture. It does the whole arm → expose → re-arm → fire → read itself.
	if worker != nil {
		n, err := worker(c, buf, expDur)
		if err != nil {
			c.setStatus(ExpFailed)
			return n, fmt.Errorf("asicam: capture worker: %w", err)
		}
		if n < c.FrameBytes() {
			c.setStatus(ExpFailed)
			return n, fmt.Errorf("asicam: short frame (%d of %d bytes)", n, c.FrameBytes())
		}
		if RepairDMAMarkers && c.sensor.FX3DMAMarkers {
			repairFX3DMAMarkers(buf[:n], ModeOf(c.rm).BytesPerPx)
		}
		n = c.binFrame(buf, n) // RAW16 host-side bin (no-op unless SoftBin>1)
		c.setStatus(ExpIdle)   // one-shot consume
		return n, nil
	}
	var lastErr error
	for attempt := 0; attempt < frameReadAttempts; attempt++ {
		if attempt > 0 {
			// Recover the pipe and re-arm so the retry has a fresh frame to read.
			if err := c.recoverAndRearm(attempt); err != nil {
				lastErr = fmt.Errorf("re-arm: %w", err)
				continue
			}
		}
		n, err := c.readFrame(buf)
		switch {
		case err != nil:
			lastErr = err
		case n < c.FrameBytes():
			// The delivered frame is RAW pixels from offset 0 (the FX3's 0xBB00AA11
			// magic is internal/stripped), so validate by size — a short read is a
			// dropped/stalled frame, not a valid one.
			lastErr = fmt.Errorf("short frame (%d of %d bytes)", n, c.FrameBytes())
		default:
			n = c.binFrame(buf, n) // RAW16 host-side bin (no-op unless SoftBin>1)
			c.setStatus(ExpIdle)   // one-shot consume (the post-exposure image read sets the worker-state field = 0)
			return n, nil
		}
	}
	c.setStatus(ExpFailed)
	// Last resort: bus-reset so a wedged device is left clean for the next capture. The
	// reset wipes init state (this run is lost), but the next Open/Init starts fresh.
	if r, ok := c.t.(DeviceResetter); ok {
		_ = r.ResetDevice()
	}
	return 0, fmt.Errorf("asicam: frame read failed after %d attempts: %w", frameReadAttempts, lastErr)
}

// recoverAndRearm escalates pipe recovery, then re-arms a fresh exposure so the next
// read attempt has a frame to capture. It deliberately does NOT bus-reset (that wipes
// init state); the bus reset is the post-exposure data read's final give-up action only.
func (c *Camera) recoverAndRearm(attempt int) error {
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint) // clear any stall / leftover data
	}
	if attempt >= 2 {
		_ = c.vendorCmd(cmdFlush) // 0xAF: harder — flush the FX3 pipeline
	}
	c.setStatus(ExpIdle) // clear the StartExposure busy-guard so the re-arm proceeds
	if err := c.arm(true); err != nil {
		return err
	}
	c.mu.Lock()
	c.expStart = nowFunc()
	c.status = ExpWorking
	c.mu.Unlock()
	return nil
}

// ReadFrame reads one frame from the already-armed, running stream into buf — no arm
// or re-arm. reset clears the bulk pipe first (a fresh frame boundary); pass false for
// back-to-back continuous reads that keep the sensor stream flowing. Exposed for
// streaming and for the cold-start/throwaway-frame diagnosis (the snap path uses
// the post-exposure data read). Returns the bytes read.
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
	return c.t.BulkRead(buf, to)
}

// readFrame fills buf with one frame. It uses the async streaming pump when the
// transport supports it (the fast path for USB3 line-rate), assembling 1 MiB
// chunks until the frame size is reached or a short packet ends it; otherwise it
// falls back to a single BulkRead.
func (c *Camera) readFrame(buf []byte) (int, error) {
	// Flush the bulk pipe so the read starts at a fresh frame boundary
	// (ResetEndPoint(0x81) before each capture session).
	if r, ok := c.t.(EndpointResetter); ok {
		_ = r.ResetEndpoint(bulkEndpoint)
	}
	// OPEN BUG: above ~0.4 s every frame currently takes ~2× the exposure to arrive
	// (1 s→2 s, 5 s→10 s, 300 s→600 s), with CORRECT brightness (one real integration —
	// it is dead-time, not a double exposure). It is NOT a one-time cold-start throwaway:
	// the -n continuous test shows all frames at 2×, not just the first. The exposure
	// programming matches the SDK on the wire (see imx174.go), so the doubling is here in
	// the readout path and is not yet identified. Until it is, we must wait out the full
	// ~2× or the backend's no-data bulk timeout aborts before the frame appears (truncated
	// read). Size the timeout for 2× the exposure plus a readout/jitter margin.
	to := 2*c.expDuration() + 3*time.Second
	if to < 2*time.Second {
		to = 2 * time.Second
	}
	s, ok := c.t.(BulkStreamer)
	if !ok {
		// One whole-frame read. The backend is responsible for the transfer
		// concurrency that primes the FX3 GPIF (the mac backend submits N
		// concurrent ReadPipeAsync transfers across the buffer).
		return c.t.BulkRead(buf, to)
	}
	want := c.FrameBytes() + 4 // pixels + the 4-byte header magic
	if want > len(buf) {
		want = len(buf)
	}
	n := (want + bulkChunkBytes - 1) / bulkChunkBytes
	if max := bulkBudgetBytes / bulkChunkBytes; n > max {
		n = max
	}
	if n < 1 {
		n = 1
	}
	st, err := s.BulkStream(bulkChunkBytes, n)
	if err != nil {
		return 0, fmt.Errorf("asicam: open bulk stream: %w", err)
	}
	defer st.Close()
	total := 0
	for total < want {
		b, err := st.Next(2 * time.Second)
		if err != nil {
			return total, fmt.Errorf("asicam: bulk stream EP 0x%02x: %w", bulkEndpoint, err)
		}
		total += copy(buf[total:], b)
		if len(b) < bulkChunkBytes || total >= len(buf) {
			break // short packet = end of frame (the +512 delimiter), or buf full
		}
	}
	return total, nil
}

// StopExposure halts streaming and leaves the device in a clean state: stop the
// sensor (master stop) and the FPGA pipeline, then flush. Important on exit so the
// next session's control writes don't hit a still-streaming device (the flaky
// 0xBD-during-init symptom).
func (c *Camera) StopExposure() error {
	c.setStatus(ExpIdle)
	_ = c.vendorCmd(cmdStreamStop) // 0xAA
	if c.sensor.StreamStop != nil {
		_ = c.sensor.StreamStop(c.rm) // master stop (0x200=1)
	}
	_ = c.fpgaStop()
	return c.vendorCmd(cmdFlush) // 0xAF
}
