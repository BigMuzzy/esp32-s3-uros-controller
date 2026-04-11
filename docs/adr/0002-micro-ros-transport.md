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

**USB-CDC serial only.**

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
  Serial monitor / logging should use UART0 or be disabled in
  production.
- Physical USB cable between ESP32 and master PC must be accounted
  for in the robot's wiring.
- Failsafe (ADR-0006) detects agent link loss by cmd_vel timeout,
  which is transport-agnostic.
