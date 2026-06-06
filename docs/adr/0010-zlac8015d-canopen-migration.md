# ADR-0010: ZLAC8015D CANopen Migration

**Status:** Accepted
**Date:** 2026-06-06
**Supersedes:** ADR-0003 (VESC CAN Protocol) for the current robot build

## Context

The robot's drivetrain hardware changed. The previous build used two
VESC controllers on a custom CAN protocol (ADR-0003). The new build uses
a single **ZLAC8015D V4** dual-channel hub-servo drive speaking
**CANopen / CiA 402**, driving two **ZLLG65ASM250 V3.0** 6.5″
direct-drive hub motors.

Reference docs (in `docs/datasheets/`):

- ZLAC8015D V4 Series Manual V1.03
- ZLAC8015D V4 Series CANopen Communication Routine V1.07
- ZLAC8015D V4 Series CANopen Communication Quick Start Guide V1.00
- ZLLG65ASM250 V3.0 (motor)

### Motor (ZLLG65ASM250 V3.0) key specs

| Parameter        | Value                                  |
|------------------|----------------------------------------|
| Type             | 6.5″ direct-drive hub motor (1:1)      |
| Wheel diameter   | 170 mm                                 |
| Rated voltage    | 24 VDC                                 |
| Pole pairs       | 15                                     |
| Encoder          | 4096 lines (→ 16384 counts/rev quad.)  |
| Rated speed      | 500 r/min                              |
| Peak speed       | 560 r/min                              |
| Rated / peak torque | 4 / 12 N·m                          |
| Rated / peak current | 10 / 30 A                           |

## Decision

Keep the hardware-agnostic motor HAL (`main/motor_driver.h`) and add a
CANopen backend selected via Kconfig `MOTOR_DRIVER_BACKEND`. The
**ZLAC8015D backend is the default**; the VESC backend remains
selectable for legacy builds.

### Bus & node

| Parameter    | Value                                            |
|--------------|--------------------------------------------------|
| CAN standard | CANopen (CiA 301), TWAI standard frames          |
| Bit rate     | 500 kbit/s (drive default, object 0x200B = 1)    |
| Node         | Single node, default ID 1 (object 0x200A)        |
| Channels     | LEFT = sub-index 1, RIGHT = sub-index 2          |

### CiA 402 control

- Profile Velocity mode (`0x6060 = 3`).
- Single shared controlword `0x6040`; enable sequence `0x06 → 0x07 → 0x0F`.
- Target velocity `0x60FF:01/02` (i32, **raw r/min**, range ±1000; speed
  resolution object `0x2026:05 = 1` assumed, the factory default).
- Async control (object `0x200F = 0`): both targets written in one RPDO.

### Object dictionary mapping (verified against Routine V1.07)

| Object   | Meaning                 | Units / layout                          |
|----------|-------------------------|-----------------------------------------|
| `0x6040` | Controlword             | U16, shared                             |
| `0x6041` | Statusword              | U32, **high16 = LEFT, low16 = RIGHT**   |
| `0x6060` | Modes of operation      | i8, shared                              |
| `0x6064` | Position actual         | i32 sub1/sub2, encoder counts           |
| `0x606C` | Velocity actual         | i32 sub1/sub2, **0.1 r/min**            |
| `0x60FF` | Target velocity         | i32 sub1/sub2, **1 r/min**              |
| `0x6083` | Profile acceleration    | u32 sub1/sub2, **ms** (0–32767)         |
| `0x6084` | Profile deceleration    | u32 sub1/sub2, **ms** (0–32767)         |
| `0x200E` | Encoder wire (lines)    | u16 sub1/sub2 (read at bring-up)        |
| `0x1017` | Producer heartbeat      | u16, unit 0.5 ms                        |

PDOs: RPDO0 = controlword, RPDO1 = target velocity L+R; TPDO0 =
statusword, TPDO1 = position L+R, TPDO2 = velocity L+R (all async,
event-timer triggered).

### Derived parameters

- **Odometry:** counts/rev is read from `0x200E:01 × 4` at bring-up
  (16384 for the 4096-line encoder) rather than hardcoded, so it tracks
  the drive's actual configuration. Gear ratio is 1:1 (direct drive).
- **Speed clamp:** target velocity is clamped to the motor's rated speed
  (`ZLAC_MAX_MOTOR_RPM`, default 500 r/min). The drive raises a "speed
  setting error" fault if commanded above its rated speed.

## Consequences

- The HAL keeps `motor_task`, `diff_drive`, and `uros_task` unchanged —
  the backend swap is contained to `canopen.*`, `cia402.*`, and
  `motor_driver_zlac8015d.*`.
- A single CANopen node replaces two VESC nodes; CiA 402 state-machine
  bring-up and a producer-heartbeat liveness check replace VESC
  ping/pong (ADR-0009).
- The VESC-specific tuning CLI (`tune_cli`) is compiled only for the
  VESC backend; the ZLAC drive runs its own internal velocity loop.
- `WHEEL_DIAMETER_M` is 0.170 m (was 0.160 m) to match the new wheel.
