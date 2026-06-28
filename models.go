package astrocam

// Model binds a camera (USB VID:PID) to its Sensor profile plus the per-model facts
// that are not sensor-intrinsic (color filter, cooler, USB speed, ST4).
type Model struct {
	Name   string
	Sensor *Sensor
	Color  bool // Bayer-filtered (MC) vs mono (MM)
	Cooled bool // has a TEC (engages the cooling PID loop)
	USB3   bool // SuperSpeed vs USB2 HighSpeed
	ST4    bool // has an ST4 guide port (CanPulseGuide)
}

// registry maps a USB VID:PID to its camera model. Sensor profiles live in the sensors
// package and register themselves from its init() (caller does
// `import _ "github.com/mikefsq/astrocam/sensors"`), so the core never imports sensor data.
var registry = map[DeviceID]Model{}

// Register adds a model under its USB vendor:product id (called by the sensors package).
func Register(vid, pid uint16, m Model) { registry[DeviceID{vid, pid}] = m }

// Lookup returns the model registered for a USB vendor:product id.
func Lookup(vid, pid uint16) (Model, bool) { m, ok := registry[DeviceID{vid, pid}]; return m, ok }
