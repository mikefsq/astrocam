package main

import (
	"encoding/binary"
	"fmt"
	"os"
	"sync"
	"time"
)

// SER v3 writer: one fixed-size header, every frame's raw pixels back-to-back, then a
// per-frame timestamp trailer.
//
// Header layout (all integers little-endian; 178 bytes total):
//
//	[ 0:14]  FileID            "LUCAM-RECORDER"
//	[14:18]  LuID              int32  (0)
//	[18:22]  ColorID           int32  (0 = MONO, 8 = BAYER_RGGB, ...)
//	[22:26]  LittleEndian      int32  (pixel byte order for >8-bit data)
//	[26:30]  ImageWidth        int32
//	[30:34]  ImageHeight       int32
//	[34:38]  PixelDepthPerPlane int32 (8 or 16)
//	[38:42]  FrameCount        int32  (patched on Close)
//	[42:82]  Observer          char[40]
//	[82:122] Instrument        char[40]
//	[122:162]Telescope         char[40]
//	[162:170]DateTime          int64  (.NET ticks, local)
//	[170:178]DateTimeUTC       int64  (.NET ticks, UTC)
const serHeaderSize = 178

// SER colour IDs.
const (
	serMono      = 0
	serBayerRGGB = 8
	serBayerGRBG = 9
	serBayerGBRG = 10
	serBayerBGGR = 11
)

// serLittleEndian is the LittleEndian header field. The spec text says 1 = little-endian,
// but the de-facto convention (FireCapture, SharpCap, and the readers tuned to them) is 0
// with little-endian data; a spec-literal 1 makes those readers byte-swap. RAW16 frames
// are written little-endian verbatim, so the field is 0.
const serLittleEndian = 0

// netEpochTicks is the number of 100 ns ticks from the .NET epoch (0001-01-01) to the Unix
// epoch (1970-01-01); SER timestamps are .NET DateTime ticks.
const netEpochTicks = 621355968000000000

func netTicks(t time.Time) int64 { return netEpochTicks + t.UnixNano()/100 }

// netTicksLocal renders t's local wall-clock as .NET ticks for the DateTime field
// (DateTimeUTC is the UTC one). UnixNano is zone-independent, so the zone offset is added.
func netTicksLocal(t time.Time) int64 {
	_, off := t.Zone()
	return netTicks(t) + int64(off)*10_000_000 // offset seconds to 100 ns ticks
}

type serWriter struct {
	// mu serializes writeFrame against close: the burst writes frames on its own goroutine while
	// an interrupt closes the file from the signal handler, and the trailer must not be built
	// from a half-updated stamp slice. It also makes count safe to read after a burst.
	mu         sync.Mutex
	f          *os.File
	frameBytes int
	closed     bool
	count      int
	stamps     []int64
}

// frameCount reports how many frames have been written.
func (s *serWriter) frameCount() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.count
}

// serColorID maps the color flag and Bayer pattern to a SER ColorID.
func serColorID(color bool, bayer string) int32 {
	if !color {
		return serMono
	}
	switch bayer {
	case "RGGB":
		return serBayerRGGB
	case "GRBG":
		return serBayerGRBG
	case "GBRG":
		return serBayerGBRG
	case "BGGR":
		return serBayerBGGR
	default:
		return serBayerRGGB
	}
}

// newSER creates a SER file and writes its header with a placeholder FrameCount (patched on
// close). writeFrame enforces the per-frame size w*h*bpp.
func newSER(path string, w, h, bpp int, colorID int32, instrument string) (*serWriter, error) {
	f, err := os.Create(path)
	if err != nil {
		return nil, err
	}
	hdr := make([]byte, serHeaderSize)
	copy(hdr[0:14], "LUCAM-RECORDER")
	le := binary.LittleEndian
	le.PutUint32(hdr[14:], 0) // LuID
	le.PutUint32(hdr[18:], uint32(colorID))
	le.PutUint32(hdr[22:], serLittleEndian)
	le.PutUint32(hdr[26:], uint32(w))
	le.PutUint32(hdr[30:], uint32(h))
	le.PutUint32(hdr[34:], uint32(bpp*8))
	le.PutUint32(hdr[38:], 0) // FrameCount, patched on close
	writeFixed(hdr[42:82], "")
	writeFixed(hdr[82:122], instrument)
	writeFixed(hdr[122:162], "")
	now := time.Now()
	le.PutUint64(hdr[162:], uint64(netTicksLocal(now)))
	le.PutUint64(hdr[170:], uint64(netTicks(now)))
	if _, err := f.Write(hdr); err != nil {
		f.Close()
		return nil, err
	}
	return &serWriter{f: f, frameBytes: w * h * bpp}, nil
}

// writeFrame appends one frame's raw pixels (frameBytes long) and records at, the frame's
// capture time, for the trailer. Callers stamp at read completion, not at write time: the
// async writer runs a pool-deep queue behind the reads.
func (s *serWriter) writeFrame(data []byte, at time.Time) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed {
		return fmt.Errorf("ser: file already closed")
	}
	if len(data) != s.frameBytes {
		return fmt.Errorf("ser: frame %d is %d bytes, want %d", s.count, len(data), s.frameBytes)
	}
	if _, err := s.f.Write(data); err != nil {
		return err
	}
	s.stamps = append(s.stamps, netTicks(at))
	s.count++
	return nil
}

// close writes the per-frame timestamp trailer, patches FrameCount in the header, and closes.
func (s *serWriter) close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed { // an interrupt hook and the normal return both close
		return nil
	}
	s.closed = true
	trailer := make([]byte, 8*len(s.stamps))
	for i, t := range s.stamps {
		binary.LittleEndian.PutUint64(trailer[i*8:], uint64(t))
	}
	if _, err := s.f.Write(trailer); err != nil {
		s.f.Close()
		return err
	}
	var cnt [4]byte
	binary.LittleEndian.PutUint32(cnt[:], uint32(s.count))
	if _, err := s.f.WriteAt(cnt[:], 38); err != nil { // patch FrameCount @ offset 38
		s.f.Close()
		return err
	}
	return s.f.Close()
}

// writeFixed copies s into a fixed-width field, space-padded and truncated to fit.
func writeFixed(dst []byte, s string) {
	for i := range dst {
		dst[i] = ' '
	}
	copy(dst, s)
}
