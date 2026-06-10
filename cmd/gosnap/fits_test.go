package main

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// TestWriteFITS checks the FITS encoding against the standard: 2880-byte blocking, the
// unsigned-16 BZERO convention, and big-endian sample order — without any hardware.
func TestWriteFITS(t *testing.T) {
	const w, h = 4, 3
	data := make([]byte, w*h*2)
	// Known little-endian UNSIGNED samples: pixel i = i*1000.
	for i := 0; i < w*h; i++ {
		v := uint16(i * 1000)
		data[2*i] = byte(v)
		data[2*i+1] = byte(v >> 8)
	}
	path := filepath.Join(t.TempDir(), "t.fits")
	if err := writeFITS(path, data, w, h, 2, "RGGB", 0.1, 3.76, 0, "ZWO ASI6200MC Pro"); err != nil {
		t.Fatal(err)
	}
	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}

	if len(b)%2880 != 0 {
		t.Fatalf("file size %d is not a multiple of 2880", len(b))
	}
	// Header is one 2880 block (17 cards < 36), data is the next block.
	hdr := string(b[:2880])
	if got := hdr[:30]; got != "SIMPLE  =                    T" {
		t.Errorf("SIMPLE card = %q", got)
	}
	for _, kw := range []string{"BITPIX", "NAXIS1", "NAXIS2", "BZERO", "BSCALE", "BAYERPAT= 'RGGB", "EXPTIME", "INSTRUME", "END     "} {
		if !strings.Contains(hdr, kw) {
			t.Errorf("header missing %q", kw)
		}
	}

	d := b[2880:]
	// pixel0 v=0 -> (0-32768) signed = 0x8000 -> big-endian 80 00
	if d[0] != 0x80 || d[1] != 0x00 {
		t.Errorf("pixel0 = %02x %02x, want 80 00", d[0], d[1])
	}
	// pixel1 v=1000=0x03E8 -> ^0x8000 = 0x83E8 -> big-endian 83 E8
	if d[2] != 0x83 || d[3] != 0xe8 {
		t.Errorf("pixel1 = %02x %02x, want 83 e8", d[2], d[3])
	}
	// Reconstruct pixel1 the way a reader does: physical = int16(be) + BZERO.
	be := int16(uint16(d[2])<<8 | uint16(d[3]))
	if phys := int(be) + 32768; phys != 1000 {
		t.Errorf("pixel1 round-trip = %d, want 1000", phys)
	}
}
