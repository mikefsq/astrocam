# astrocam: Go astronomy camera driver

astrocam is a Go driver for astronomy CMOS cameras that use the Cypress FX3 bridge and
Sony sensors. astrocam speaks their USB protocol directly, so you do not need a vendor SDK.

## Driver status

The driver covers the control plane, the data plane, the cooling PID loop, ST4 guiding, and snap
and stream capture. There is one sensor profile for each Sony die. The profiles wire these
controls:

- HCG gain
- sub-frame ROI
- binning on the host, in the FPGA and on the die, summed or averaged
- offset and black level
- RAW8 and RAW16 readout
- 10-bit high-speed mode
- alternative sensor modes such as HDR
- the frame-rate cap
- white balance
- host-timed long exposures

Hardware validated: 

- **ASI6200MM/MC Pro (IMX455)** 
- **ASI290MM Mini (IMX290)**
- **ASI462MC (IMX462)**
- **ASI174MM Mini (IMX174)**
- **Xena 585M (IMX585, PlayerOne)**

not validated yet:

- **ASI2600 family (IMX571)**: tracks the IMX455 profile.
- **IMX178**
- **ZWO half of the IMX585 profile**

## Build and test

```sh
go build ./...        # darwin pulls cgo for the USB backend. linux and windows are pure Go.
go test ./...         # stdlib only, no hardware

# cross-build the snap tool for a deploy target (linux and windows are pure Go)
GOOS=linux GOARCH=arm64 CGO_ENABLED=0 go build -o gosnap ./cmd/gosnap
```

On Linux, give udev access to the USB vendor id of the camera before you open the device: ZWO
`0x03C3`, PlayerOne `0xA0A0`.

## gosnap

`cmd/gosnap` is the bring-up and capture tool. The default mode connects, shows the identity of
the camera, and disconnects. `-capture` does the full init and one exposure. `-v` logs every USB
transfer.

`-vid` and `-pid` select the camera. The defaults are `0x03c3` for ZWO and `0x1749` for the
ASI174MM Mini. `-serial` selects the camera by its factory serial, in every mode. `-list` shows
each attached camera with its serial.

```sh
gosnap -list                            # enumerate attached cameras
gosnap -pid 0x620a                      # detect and identify, no capture
gosnap -serial 06118f061f090900 -capture # the same camera, picked by serial
gosnap -pid 0x620a -capture -exposure 100ms -gain 200 -out frame.fits
gosnap -capture -bin 2 -roi x,y,w,h -raw8 -highspeed   # readout-mode controls
gosnap -capture -bin 2 -hwbin -binsum   # bin on the die, sum instead of average
gosnap -capture -sensormode 1           # HDR (IMX585 PlayerOne, RAW16 only)
gosnap -capture -framelimit 30          # cap the frame rate, independent of -fps
gosnap -capture -n 200 -discard         # streaming: interval stats for each frame
gosnap -capture -n 200 -out run.ser     # streaming: SER video capture
# -n N records only to a .ser output. With any other -out, gosnap reads the frames
# and discards them, as a throughput benchmark. A .txt or .csv -out gets the
# interval dump for each frame.
```

Correction and inspection:

```sh
gosnap -capture -fixdefects=false        # raw sensor output. The factory map applies by default.
gosnap -capture -keepmarkers             # leave the FX3 frame markers in place
gosnap -defects                          # report the factory hot-pixel map
gosnap -guide N,250ms                    # one ST4 pulse on the given line
gosnap -dumpregs 0x301a,0x3069,f:0x14-0x18   # read registers back, write nothing
gosnap -flashat 42000,40                 # a raw SPI flash range, hex addr,len
```

Use `-dumpregs` to compare the driver against a vendor SDK. The register file survives after the
SDK closes the device. Run the vendor tool first, then dump the registers. This gives a direct
diff, and you do not need a USB analyzer.

`-usb2` drives the camera with the USB2 configuration while the wire stays at SuperSpeed. The
configuration covers the bandwidth budget, the GPIF divider, and the EP0 pacing that runs only on
USB2 readouts. The host cannot change the negotiated link, because the link is a hardware matter.
The flag therefore tests the USB2 configuration at SuperSpeed throughput. This separates a fault
in the timing model from a fault in the link.
