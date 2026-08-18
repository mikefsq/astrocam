//go:build !linux && !darwin && !windows

package astrocam

import (
	"context"
	"errors"
)

// ErrNoHotplug is returned by Hotplug on a platform with no notification
// source; the caller keeps polling.
var ErrNoHotplug = errors.New("astrocam: no hotplug notification source on this platform")

func hotplug(context.Context) (<-chan HotplugEvent, error) { return nil, ErrNoHotplug }
