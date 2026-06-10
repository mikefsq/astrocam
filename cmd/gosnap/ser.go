package main

import (
	"encoding/binary"
	"fmt"
	"os"
	"time"
)

// SER v3 writer (the planetary / lucky-imaging video container: one fixed-size header,
// then every frame's raw pixels back-to-back, then an optional per-frame timestamp
// trailer). Written for the fast-exposure burst path — frames stream straight to disk as
// they arrive, so RAM stays flat regardless of frame count. Read by SER Player, PIPP,
// AutoStakkert!, FireCapture.
//
// Layout (all integers little-endian; total header = 178 bytes):
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

// serLittleEndian: the camera delivers RAW16 little-endian and we write it verbatim (no
// swap). The SER v3 spec says 1 = little-endian data; that is what we set and what
// spec-compliant readers (SER Player, PIPP, AutoStakkert!) expect. (8-bit data is
// byte-order-agnostic, so RAW8 SER is unaffected by this field.)
const serLittleEndian = 1

// netEpochTicks is the number of 100-ns ticks from the .NET epoch (0001-01-01) to the
// Unix epoch (1970-01-01) — SER timestamps are .NET DateTime ticks.
const netEpochTicks = 621355968000000000

func netTicks(t time.Time) int64 { return netEpochTicks + t.UnixNano()/100 }

type serWriter struct {
	f          *os.File
	frameBytes int
	count      int
	stamps     []int64
}

// serColorID maps the gosnap color flag + Bayer pattern to a SER ColorID.
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

// newSER creates a SER file and writes its header with a placeholder frame count (patched
// on Close). frameBytes is the exact per-frame size (w*h*bpp); writeFrame enforces it.
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
	le.PutUint32(hdr[38:], 0) // FrameCount — patched on Close
	writeFixed(hdr[42:82], "")
	writeFixed(hdr[82:122], instrument)
	writeFixed(hdr[122:162], "")
	now := time.Now()
	le.PutUint64(hdr[162:], uint64(netTicks(now)))
	le.PutUint64(hdr[170:], uint64(netTicks(now.UTC())))
	if _, err := f.Write(hdr); err != nil {
		f.Close()
		return nil, err
	}
	return &serWriter{f: f, frameBytes: w * h * bpp}, nil
}

// writeFrame appends one frame's raw pixels (exactly frameBytes) and records its capture
// time for the trailer.
func (s *serWriter) writeFrame(data []byte) error {
	if len(data) != s.frameBytes {
		return fmt.Errorf("ser: frame %d is %d bytes, want %d", s.count, len(data), s.frameBytes)
	}
	if _, err := s.f.Write(data); err != nil {
		return err
	}
	s.stamps = append(s.stamps, netTicks(time.Now()))
	s.count++
	return nil
}

// Close writes the per-frame timestamp trailer, patches FrameCount in the header, and closes.
func (s *serWriter) close() error {
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

// writeFixed copies s into a fixed-width field, space-padded, NUL-safe (truncates if long).
func writeFixed(dst []byte, s string) {
	for i := range dst {
		dst[i] = ' '
	}
	copy(dst, s)
}
