# astrocam —- Go astronomy camera driver

A Go driver for various astronomy cmos cameras that talks the
cameras' USB protocol directly — **no vendor SDK** in the process. 

cgo-free everywhere except the macOS USB backend: **Linux and Windows build with
`CGO_ENABLED=0`**; macOS uses IOUSBHost via cgo.

## Vendor-independent by design

A `(VID,PID)→Model` registry plus a `VID→Vendor` dialect map, so a single sensor profile —
keyed by the **Sony die** — serves every vendor that uses that die. ZWO and PlayerOne share
the IMX455/IMX571 profiles; the only per-vendor difference is the register-map (`Regmap`)
opcodes and the gain/offset unit scale, selected at call time from the regmap's VID. Vendors
register themselves from `init()` (ZWO in `protocol.go`, PlayerOne in `protocol_poa.go`).

## Layout

```
.                      package asicam — the driver core (flat)
  device.go            open/enumerate/lifecycle
  vendor.go            (VID,PID)->Model registry + VID->Vendor dialect map (the seam)
  models.go            known camera models
  camera.go            camera API (identity, controls)
  capture.go           single-frame capture
  fps.go               video / streaming capture
  cooling.go           TEC cooling PID loop          thermal_hw.go  thermal wire ops
  gain.go  shutter.go  guide.go (ST4)  sensor.go  color.go
  protocol.go          ZWO wire protocol / regmap dialect
  protocol_poa.go      PlayerOne wire protocol / regmap dialect
  transport.go         the Transport seam
  transport_darwin.go  macOS   — IOUSBHost (cgo)
  transport_linux.go   Linux   — usbfs (pure Go; needs udev access to VID 0x03C3)
  transport_windows.go Windows — WinUSB (pure Go)
  transport_stub.go    other   — compile-only
  *_test.go            unit + e2e tests (stdlib only, no hardware)
sensors/               per-die sensor profiles (data templates over the shared engine)
  imx174 imx178 imx290 imx455 imx462 imx571 imx585  + models.go, sensors_test.go
cmd/gosnap/            bring-up + capture CLI (the pure-Go counterpart to the SDK asisnap)
```

## Driver status

Control + data plane, cooling PID, guiding, and snap/stream capture, with modular sensor profiles (one per Sony die). Wired across the family: HCG gain, sub-frame ROI, binning, offset/black-level, RAW8/RAW16 readout, 10-bit high-speed mode, and host-timed long exposures.

**Hardware-validated on the wire:**
- **ASI6200MM/MC Pro (IMX455)** — full-frame pixel-matched to the SDK: optical-black crop + FX3 DDR frame-marker repair (on by default), and hot-pixel map repair (off by default).
- **ASI290MM Mini (IMX290)** 
- **ASI462 (IMX462)** — 12-bit + 10-bit high-speed; shares the FX3 frame-marker repair (the repair is a per-sensor opt-in, so unverified/non-Sony cameras are never touched).
- **ASI174MM Mini (IMX174)** — global shutter (no hardware binning)

**WIP**: 
- **IMX178**
- **IMX585**


## Build & test

```sh
go build ./...        # darwin pulls cgo for the USB backend; linux/windows are pure Go
go test ./...         # stdlib only, no hardware

# cross-build the snap tool for a deploy target (linux/windows are pure Go)
GOOS=linux GOARCH=arm64 CGO_ENABLED=0 go build -o gosnap ./cmd/gosnap
```

On Linux, opening the device needs udev access to the ZWO USB vendor id (`0x03C3`).

## gosnap

`cmd/gosnap` is the bring-up + capture tool. Default mode detects the camera, connects, prints
its identity, and disconnects (control transfers only); `-capture` runs the full init + a single
exposure (the first-light test); `-v` logs every USB transfer for debugging an arm sequence
against the real device.

```sh
gosnap                                  # detect + identify (no capture)
gosnap -capture -exposure 100ms -gain 200 -out frame.raw
gosnap -capture -bin 2 -roi x,y,w,h -raw8 -highspeed   # readout-mode controls
gosnap -capture -fps -discard           # streaming: per-frame interval stats + stdev
```
