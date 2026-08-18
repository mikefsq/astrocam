package astrocam

import (
	"context"
	"errors"
	"fmt"
	"strconv"
	"strings"
	"syscall"
)

// ErrNoHotplug is returned by Hotplug on a platform with no notification
// source; the caller keeps polling.
var ErrNoHotplug = errors.New("astrocam: no hotplug notification source on this platform")

// netlinkKobjectUevent is NETLINK_KOBJECT_UEVENT, the kernel's uevent bus.
const netlinkKobjectUevent = 15

// hotplug reads the kernel's uevent netlink socket, the same feed udev
// consumes, and reports usb_device add and remove events on known vendor ids.
// A uevent is "ACTION@DEVPATH\0KEY=VALUE\0...", with PRODUCT=vid/pid/bcd in
// hex and BUSNUM/DEVNUM decimal, so Location and Attachment are exact. Group 1
// is the kernel's own multicast group, readable without privilege.
func hotplug(ctx context.Context) (<-chan HotplugEvent, error) {
	fd, err := syscall.Socket(syscall.AF_NETLINK, syscall.SOCK_RAW|syscall.SOCK_CLOEXEC, netlinkKobjectUevent)
	if err != nil {
		return nil, fmt.Errorf("astrocam: uevent socket: %w", err)
	}
	if err := syscall.Bind(fd, &syscall.SockaddrNetlink{Family: syscall.AF_NETLINK, Groups: 1}); err != nil {
		syscall.Close(fd)
		return nil, fmt.Errorf("astrocam: uevent bind: %w", err)
	}
	ch := make(chan HotplugEvent, 16)
	go func() {
		<-ctx.Done()
		syscall.Close(fd) // unblocks Recvfrom
	}()
	go func() {
		defer close(ch)
		buf := make([]byte, 8192)
		for {
			n, _, err := syscall.Recvfrom(fd, buf, 0)
			if err != nil {
				if ctx.Err() != nil || errors.Is(err, syscall.EBADF) {
					return
				}
				if errors.Is(err, syscall.EINTR) || errors.Is(err, syscall.EAGAIN) {
					continue
				}
				return
			}
			if ev, ok := parseUevent(buf[:n]); ok {
				select {
				case ch <- ev:
				case <-ctx.Done():
					return
				}
			}
		}
	}()
	return ch, nil
}

// parseUevent decodes one uevent datagram into an event, reporting false for
// anything that is not a usb_device add or remove on a known vendor.
func parseUevent(msg []byte) (HotplugEvent, bool) {
	var ev HotplugEvent
	fields := strings.Split(string(msg), "\x00")
	if len(fields) == 0 {
		return ev, false
	}
	action, _, _ := strings.Cut(fields[0], "@")
	switch action {
	case "add":
		ev.Attached = true
	case "remove":
	default:
		return ev, false
	}
	var subsystem, devtype string
	var bus, dev int
	for _, f := range fields[1:] {
		k, v, ok := strings.Cut(f, "=")
		if !ok {
			continue
		}
		switch k {
		case "SUBSYSTEM":
			subsystem = v
		case "DEVTYPE":
			devtype = v
		case "PRODUCT": // vid/pid/bcd, hex without padding
			parts := strings.Split(v, "/")
			if len(parts) >= 2 {
				if x, err := strconv.ParseUint(parts[0], 16, 16); err == nil {
					ev.VID = uint16(x)
				}
				if x, err := strconv.ParseUint(parts[1], 16, 16); err == nil {
					ev.PID = uint16(x)
				}
			}
		case "BUSNUM":
			bus, _ = strconv.Atoi(v)
		case "DEVNUM":
			dev, _ = strconv.Atoi(v)
		}
	}
	if subsystem != "usb" || devtype != "usb_device" || !knownVID(ev.VID) {
		return ev, false
	}
	ev.Location = uint32(bus)<<16 | uint32(dev&0xffff)
	ev.Attachment = uint64(ev.Location)
	return ev, true
}
