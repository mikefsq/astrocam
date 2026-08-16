// Package sensors holds the per-die sensor profiles (IMX174, IMX178, IMX290, IMX455, IMX462,
// IMX571, IMX585) and the camera-model registry that binds USB (VID,PID) pairs to them. Each
// profile is a data template over the shared engine in the astrocam package: an init register
// table plus gain/exposure/ROI/offset ops and a capture worker, keyed by the Sony die and shared
// by every vendor and color/mono variant that uses it. Importing the package for side effect
// (import _ "github.com/mikefsq/astrocam/sensors") registers every model.
package sensors
