# astrocam: Go astronomy camera driver

astrocam is a Go driver for astronomy CMOS cameras built on the Cypress FX3 bridge and Sony
sensors. It speaks the cameras' USB protocol directly, with no vendor SDK in the process.

The driver is cgo-free except for the macOS USB backend. Linux and Windows build with
`CGO_ENABLED=0`; macOS uses IOUSBHost through cgo.

## Vendor independence

A `(VID,PID) → Model` registry and a `VID → Vendor` dialect map let one sensor profile, keyed by
the Sony die, serve every vendor that uses that die. ZWO and PlayerOne share the IMX455 and
IMX571 profiles. The vendors differ in their register-map opcodes (`Regmap`) and their
gain/offset unit scale, selected at call time from the regmap's VID. Each vendor registers itself
from `init()`: ZWO in `protocol.go`, PlayerOne in `protocol_poa.go`.

## Layout

```
.                      package astrocam: the driver core (flat)
  device.go            open/enumerate/lifecycle
  vendor.go            (VID,PID)->Model registry + VID->Vendor dialect map
  models.go            camera-model registry
  camera.go            camera API (identity, controls)
  capture.go           capture control: single frame, StartVideo/StartStream/ReadFrame,
                       ResetDevice, FX3 marker repair
  fps.go               readout / line-time engine, FPGA geometry helpers
  cooling.go           TEC cooling PID loop          thermal_hw.go  thermal wire ops
  gain.go  shutter.go  guide.go (ST4)  sensor.go  defectmap.go
  protocol.go          ZWO wire protocol / regmap dialect
  protocol_poa.go      PlayerOne wire protocol / regmap dialect
  transport.go         the Transport interface
  transport_darwin.go  macOS   (IOUSBHost, cgo)
  transport_linux.go   Linux   (usbfs, pure Go; needs udev access to the camera VID)
  transport_windows.go Windows (WinUSB, pure Go)
  transport_stub.go    in-memory stub transport for hardware-free tests (all platforms)
  *_test.go            unit + e2e tests (stdlib only, no hardware)
sensors/               per-die sensor profiles (data templates over the shared engine)
  imx174 imx178 imx290 imx455 imx462 imx571 imx585  + models.go, sensors_test.go
cmd/gosnap/            bring-up + capture CLI
```

macOS, Linux and Windows have a USB backend. Other platforms compile and run the stub transport
and tests; `OpenHost`, `OpenLocation` and `Enumerate` return an error there.

## Driver status

The driver covers the control and data plane, cooling PID, ST4 guiding, and snap/stream
capture, with one sensor profile per Sony die. The profiles wire HCG gain, sub-frame ROI,
binning, offset/black level, RAW8/RAW16 readout, 10-bit high-speed mode, and host-timed long
exposures.

Hardware-validated on the wire:

- **ASI6200MM/MC Pro (IMX455)**: full frame pixel-matched to the SDK, optical-black crop, FX3
  DDR frame-marker repair (on by default), factory hot-pixel map repair (off by default).
- **ASI290MM Mini (IMX290)**.
- **ASI462MC (IMX462)**: 12-bit and 10-bit high-speed; FX3 frame-marker repair (a per-sensor
  opt-in).
- **ASI174MM Mini (IMX174)**: global shutter, no hardware binning.

Decoded but not validated on hardware:

- **ASI2600 family (IMX571)**: tracks the IMX455 profile.
- **IMX178**, **IMX585**.
- **PlayerOne models** (Apollo, Sedna, Mars/Ceres, Zeus, Poseidon, Uranus): registered over the
  shared profiles through the PlayerOne regmap dialect; no PlayerOne camera has been driven.

## Build and test

```sh
go build ./...        # darwin pulls cgo for the USB backend; linux/windows are pure Go
go test ./...         # stdlib only, no hardware

# cross-build the snap tool for a deploy target (linux/windows are pure Go)
GOOS=linux GOARCH=arm64 CGO_ENABLED=0 go build -o gosnap ./cmd/gosnap
```

Opening the device on Linux needs udev access to the camera's USB vendor id (ZWO `0x03C3`,
PlayerOne `0xA0A0`).

## gosnap

`cmd/gosnap` is the bring-up and capture tool. The default mode connects, prints the camera's
identity, and disconnects. `-capture` runs the full init and one exposure. `-v` logs every USB
transfer. `-vid`/`-pid` select the camera (defaults `0x03c3`, ZWO, and `0x1749`, the ASI174MM
Mini), or `-serial` selects it by factory serial for every mode; `-list` enumerates attached
cameras with their serials.

```sh
gosnap -list                            # enumerate attached cameras
gosnap -pid 0x620a                      # detect + identify (no capture)
gosnap -serial 06118f061f090900 -capture # same camera picked by serial
gosnap -pid 0x620a -capture -exposure 100ms -gain 200 -out frame.fits
gosnap -capture -bin 2 -roi x,y,w,h -raw8 -highspeed   # readout-mode controls
gosnap -capture -n 200 -discard         # streaming: per-frame interval stats
gosnap -capture -n 200 -out run.ser     # streaming: SER video capture
# -n N only records to a .ser output; with any other -out the frames are read and discarded
# (a throughput benchmark), and a .txt/.csv -out receives the per-frame interval dump.
```
