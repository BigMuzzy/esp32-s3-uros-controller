# ADR-0001: Core Allocation Strategy

**Status:** Proposed
**Date:** 2026-04-10

## Context

The ESP32-S3 has two Xtensa LX7 cores. The firmware must run:

- **micro-ROS / DDS stack** — network-heavy, variable latency, drives Wi-Fi
  or USB-CDC serial
- **CAN (TWAI) communication** — time-sensitive TX/RX with two VESC
  controllers at ~50 Hz
- **Diff-drive kinematics** — converts cmd_vel to left/right wheel speeds
  each control cycle
- **Odometry computation** — integrates VESC tachometer/RPM feedback
- **RC PWM failsafe** — reads RC receiver pulses, must respond within one
  control cycle if master link is lost

micro-ROS and the Wi-Fi stack are interrupt-heavy and can cause
unpredictable latency spikes. CAN TX/RX and PWM reading are
latency-sensitive — jitter directly affects motor control quality and
failsafe response time.

## Decision

**Partition by latency domain:**

| Core | Domain         | Tasks                                          |
|------|----------------|------------------------------------------------|
| 0    | Network / DDS  | `uros_task`, Wi-Fi, USB-CDC                    |
| 1    | Real-time I/O  | `can_tx_task`, `can_rx_task`, `rc_failsafe_task`|

- Core 0 runs micro-ROS spin and all DDS/transport work at normal
  priority. ESP-IDF pins its Wi-Fi tasks to Core 0 by default.
- Core 1 runs CAN and PWM tasks at elevated priority, with no networking
  interrupts.
- Cross-core data is passed via FreeRTOS queues or shared structs
  protected by spinlocks:
  - `cmd_vel` → Core 1 (queue, consumed by `can_tx_task`)
  - Odometry → Core 0 (shared struct, published by `uros_task`)
  - Failsafe state → Core 0 (atomic flag, published by `uros_task`)

## Alternatives Considered

### A. Single-core (everything on Core 0, Core 1 idle)
Simpler, but Wi-Fi/DDS latency spikes would directly affect CAN timing.
Unacceptable for motor control.

### B. Opposite assignment (real-time on Core 0, DDS on Core 1)
Possible, but ESP-IDF defaults Wi-Fi to Core 0. Fighting this adds
configuration complexity with no real benefit.

### C. No pinning (let FreeRTOS schedule freely across both cores)
Loses the latency isolation guarantee. A Wi-Fi interrupt on the same
core as CAN TX could delay a motor command by milliseconds.

## Consequences

- CAN and PWM tasks get deterministic scheduling, unaffected by network
  load.
- Cross-core communication adds a small amount of complexity (queues,
  shared structs).
- CPU-intensive work (e.g., odometry integration) should stay small
  enough to not starve the CAN TX cycle on Core 1.
- If Wi-Fi is not used (USB-CDC only), Core 0 will be lightly loaded —
  this is acceptable, the goal is isolation not load balancing.
