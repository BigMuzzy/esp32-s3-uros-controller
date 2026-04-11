# ADR-0003: VESC CAN Protocol

**Status:** Proposed
**Date:** 2026-04-10

## Context

The ESP32-S3 drives a differential-drive robot with two VESC-based motor
controllers over a shared CAN bus (ESP32-S3 TWAI peripheral). The target
hardware:

| Controller              | HW base  | FW     | Voltage   | Cont. A |
|-------------------------|----------|--------|-----------|---------|
| Flipsky Mini FSESC6.7 Pro 70A | VESC 6.6 | 5.2    | 14–60 V   | 70 A    |
| Flipsky 75200 Pro V2.0  | VESC 6   | 6.02   | 14–84 V   | 200 A   |

Both run standard VESC firmware and use the same CAN protocol. The
firmware must support either controller without code changes — only
VESC Tool calibration differs per motor.

VESC CAN uses **CAN 2.0B extended frames** (29-bit ID). The ID encodes
a command type in the upper bits and the target VESC ID in the lower
8 bits:

```
CAN ID [28:8] = command    CAN ID [7:0] = VESC ID
```

## Decision

### Bus configuration

| Parameter      | Value                         |
|----------------|-------------------------------|
| CAN standard   | CAN 2.0B (extended frames)    |
| Bit rate       | 500 kbit/s (VESC default)     |
| Termination    | 120 Ω jumper on Waveshare board, plus termination at last VESC |
| Left VESC ID   | 1                             |
| Right VESC ID  | 2                             |

### Command: ERPM (CAN_PACKET_SET_RPM, command 3)

The ESP32 sends **ERPM set-point** commands to each VESC. ERPM mode
provides closed-loop speed control inside the VESC — the VESC PID
loop regulates motor speed, making it a natural fit for diff-drive
kinematics where the ESP32 computes target wheel RPM from `cmd_vel`.

Frame format (TX from ESP32):

| Field     | Value                                  |
|-----------|----------------------------------------|
| CAN ID    | `(3 << 8) | vesc_id` (extended frame)  |
| DLC       | 4                                      |
| Data[0:3] | ERPM as signed 32-bit integer, big-endian |

ERPM = mechanical RPM × motor pole pairs. The pole-pair count is
configured in the VESC, so the ESP32 sends ERPM directly.

### Status: VESC status frames (RX to ESP32)

VESCs periodically broadcast status frames. The ESP32 listens for
these to compute odometry and publish telemetry. Key frames:

| Frame                    | Cmd ID | Contents                          |
|--------------------------|--------|-----------------------------------|
| `CAN_PACKET_STATUS`      | 9      | ERPM, current, duty cycle         |
| `CAN_PACKET_STATUS_4`    | 14     | Tachometer (cumulative ERPM ticks), voltage in   |
| `CAN_PACKET_STATUS_5`    | 15     | Tachometer (absolute), voltage in |

- **Tachometer** from STATUS_4/5 gives cumulative motor revolutions
  for odometry (see ADR-0005).
- **ERPM** from STATUS gives real-time speed for diagnostics.
- **Voltage/current** from STATUS frames publishes as motor telemetry.

Status broadcast rate is configured in VESC Tool (default ~50 Hz per
VESC, configurable via `CAN Status Message Rate`).

### Firmware abstraction

```c
// vesc_can.h — thin encode/decode layer
void vesc_can_set_rpm(uint8_t vesc_id, int32_t erpm);

typedef struct {
    int32_t  erpm;
    float    current;
    float    duty;
    int32_t  tachometer;
    float    voltage_in;
} vesc_status_t;

bool vesc_can_parse_status(twai_message_t *msg, uint8_t *vesc_id_out,
                           vesc_status_t *status_out);
```

The `vesc_can` module handles byte-order conversion and frame
encoding/decoding. All VESC-protocol details are isolated here so
the rest of the firmware works in SI units (RPM, amps, volts).

## Alternatives Considered

### A. Duty cycle command (CAN_PACKET_SET_DUTY, command 0)
Open-loop — no speed regulation. Wheel speed varies with load, making
accurate diff-drive kinematics impossible without an external PID loop
on the ESP32.

### B. Current command (CAN_PACKET_SET_CURRENT, command 1)
Torque control. Would require a speed-control PID loop on the ESP32
and high-rate ERPM feedback. Adds complexity without benefit — the
VESC's internal PID already does this well.

### C. UART-based VESC protocol
Would require two UART ports (one per VESC) and can't share a bus.
CAN allows both VESCs on a single bus with ID-based addressing.

### D. Custom CAN bit rate (1 Mbit/s)
Higher throughput but no benefit — VESC CAN frames are small (≤8 bytes)
and traffic is light (~100 frames/s total). 500 kbit/s is the VESC
default and avoids reconfiguring every VESC.

## Consequences

- Both VESC controllers must be configured via VESC Tool with:
  - Unique CAN IDs (1 and 2)
  - CAN status message broadcasting enabled
  - Motor pole-pair count set correctly (for ERPM ↔ RPM conversion)
  - CAN baud rate at 500 kbit/s (default)
- The `vesc_can` module must handle both firmware versions (5.2 and
  6.02) — the CAN frame format is the same across these versions.
- The Waveshare board's 120 Ω termination jumper should be installed.
  The CAN bus should also be terminated at the far end (last VESC).
- ERPM value must be range-checked before sending to prevent
  commanding unsafe speeds.
