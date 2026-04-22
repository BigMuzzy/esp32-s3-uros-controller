# ADR-0002: micro-ROS Transport

**Status:** Proposed
**Date:** 2026-04-10

## Context

The ESP32-S3 micro-ROS node must communicate with a micro-ROS agent
running on the master PC. The `micro_ros_espidf_component` supports
several transports:

- **USB-CDC serial** — the board's USB-C port exposes a CDC-ACM device
- **Wi-Fi UDP** — the ESP32-S3 has 2.4 GHz Wi-Fi
- **Wi-Fi TCP** — same radio, stream-oriented

The robot is a differential-drive platform with an onboard master PC
connected via USB.

## Decision

**USB-CDC serial only**, implemented on top of the ESP32-S3
**USB-Serial/JTAG** peripheral as a custom micro-XRCE transport.

- Zero configuration — the USB-C cable is already present for
  flashing and monitoring.
- Lowest latency (~1 ms round-trip), no packet loss, no jitter from
  wireless retransmissions.
- No Wi-Fi stack needed — saves flash, RAM, and CPU on Core 0.
  Eliminates an entire class of connectivity issues (AP config,
  DHCP, signal strength).
- The master PC is physically mounted on the robot, so a USB cable
  is not a constraint.

Wi-Fi UDP can be added later if the platform moves to a wireless
link, but there is no current requirement for it.

### Implementation notes

The `micro_ros_espidf_component` ships built-in transports for
WLAN-UDP, Ethernet-UDP, and UART only. USB-CDC is not a shipped
option. On ESP32-S3 the native USB peripheral has two incompatible
personalities:

1. **USB-Serial/JTAG** — ROM peripheral, driven by ESP-IDF's
   `usb_serial_jtag` driver. Appears to the host as CDC-ACM
   (`/dev/ttyACM0`). No TinyUSB, no descriptors to maintain.
2. **USB-OTG** — full USB controller driven by TinyUSB. Requires
   descriptors, VID/PID, ~30 KB extra flash.

We use (1). It is simpler, uses less flash, and gives the same
host-side experience.

The client-side transport is registered as a **custom** XRCE
transport with framing enabled:

```c
rmw_uros_set_custom_transport(true /* framing */, NULL,
    open_cb, close_cb, write_cb, read_cb);
```

The four callbacks are thin wrappers around
`usb_serial_jtag_driver_install`, `usb_serial_jtag_write_bytes`,
and `usb_serial_jtag_read_bytes` (see
[`firmware/main/uros_transport_usb_jtag.c`](../../firmware/main/uros_transport_usb_jtag.c)).

Build configuration:

- `firmware/app-colcon.meta` sets `-DRMW_UXRCE_TRANSPORT=custom`
  so `libmicroros.a` is compiled with the custom transport profile.
- `CONFIG_MICRO_ROS_ESP_UART_TRANSPORT=y` in `sdkconfig` satisfies
  the component's required network-interface Kconfig choice without
  pulling in WLAN / Ethernet init. The UART pin settings in that
  menu are ignored — we don't use a UART peripheral.

Host side:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

The baud rate is ignored by the USB CDC endpoint but still required
by the agent's command line.

## Alternatives Considered

### A. Wi-Fi UDP
Enables untethered operation but adds complexity: Wi-Fi provisioning,
static IP / mDNS configuration, and latency spikes from retransmissions.
Not justified when the master PC is onboard and USB-connected.

### B. Wi-Fi TCP
Same drawbacks as UDP, plus head-of-line blocking. DDS already handles
reliability, so TCP's guarantees are redundant and harmful to latency.

### C. Dual transport (USB-CDC + Wi-Fi, build-time switch)
Adds two build configurations and conditional code. Not justified
without a concrete wireless requirement.

### D. Bluetooth serial (SPP / BLE)
Lower bandwidth, higher latency. BLE MTU limits fragment larger
messages. Not supported by the standard micro-ROS ESP-IDF component
without a custom transport.

## Consequences

- Single build configuration — no Kconfig transport menu needed.
- The USB-C port is dedicated to micro-ROS agent communication.
  Serial monitor / logging lives on UART0 mapped to GPIO1/2
  (see ADR-0008).
- Physical USB cable between ESP32 and master PC must be accounted
  for in the robot's wiring.
- Flashing via the USB-C port still works, but auto-reset into
  download mode can be unreliable once the app owns USB-Serial/JTAG;
  the BOOT+RESET button sequence is the reliable fallback.
- Failsafe (ADR-0006) detects agent link loss by cmd_vel timeout,
  which is transport-agnostic.
