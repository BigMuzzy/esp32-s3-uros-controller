# ADR-0004: Diff-Drive Kinematics on ESP32

**Status:** Proposed
**Date:** 2026-04-10

## Context

The robot is a differential-drive platform with two independently driven
wheels. The master PC publishes `cmd_vel` (`geometry_msgs/Twist`) with
linear velocity (`v`, m/s) and angular velocity (`ω`, rad/s). These
must be converted to individual left/right wheel speeds and sent to the
VESCs as ERPM commands (ADR-0003).

The conversion could run on the master PC (publishing per-wheel commands)
or on the ESP32.

## Decision

**Diff-drive kinematics runs on the ESP32 (Core 1).**

The ESP32 subscribes to `cmd_vel` and performs the full conversion to
left/right ERPM internally. This makes the ESP32 a self-contained
diff-drive controller.

### Kinematics equations

```
v_left  = v - ω × (track_width / 2)
v_right = v + ω × (track_width / 2)
```

Where `v_left` and `v_right` are linear wheel speeds (m/s).

Conversion to motor RPM:

```
rpm = v_wheel / (π × wheel_diameter)  × 60
erpm = rpm × motor_pole_pairs
```

### Robot parameters

Defined as compile-time constants (or Kconfig values) since they are
fixed per physical robot build:

```c
// diff_drive.h
#define WHEEL_DIAMETER_M      0.100   // meters — placeholder, set per build
#define TRACK_WIDTH_M         0.300   // meters — placeholder, set per build
#define MOTOR_POLE_PAIRS      7       // placeholder, set per motor
#define MAX_WHEEL_RPM         3000    // safety clamp
```

These values differ between builds (FSESC 6.7 Pro vs 75200 Pro use
different motors and chassis), so they must be easy to change without
modifying logic.

### Data flow

```
Core 0                          Core 1
──────                          ──────
uros_task                       can_tx_task
  │                               │
  │  cmd_vel callback             │
  │  writes to shared struct ──►  reads shared struct
  │  (atomic / mutex)             │
  │                               ├─ diff_drive_cmd_vel_to_erpm()
  │                               ├─ vesc_can_set_rpm(LEFT,  erpm_l)
  │                               └─ vesc_can_set_rpm(RIGHT, erpm_r)
```

The `cmd_vel` value is passed from Core 0 to Core 1 via a shared
struct protected by a spinlock (`portMUX_TYPE`). The `can_tx_task`
on Core 1 reads the latest cmd_vel, converts to ERPM, and sends
CAN frames — all in a single 20 ms cycle.

### Safety limits

- **Max ERPM clamp**: applied after kinematics to prevent commanding
  unsafe motor speeds.
- **cmd_vel timeout**: if no `cmd_vel` is received within a
  configurable period (e.g. 500 ms), wheel commands are set to zero.
  This is independent of the RC failsafe (ADR-0006).

### API sketch

```c
// diff_drive.h
typedef struct {
    float linear_x;    // m/s
    float angular_z;   // rad/s
} cmd_vel_t;

typedef struct {
    int32_t left_erpm;
    int32_t right_erpm;
} wheel_erpm_t;

wheel_erpm_t diff_drive_cmd_vel_to_erpm(const cmd_vel_t *cmd);
```

## Alternatives Considered

### A. Kinematics on the master PC
The master PC publishes per-wheel speed commands instead of `cmd_vel`.
This simplifies the ESP32 but:
- Breaks the standard ROS `cmd_vel` interface — `nav2`, `teleop_twist`,
  and other tools publish `Twist`, not per-wheel speeds.
- Requires custom messages and a host-side node for conversion.
- RC failsafe (ADR-0006) would still need kinematics on the ESP32 for
  arcade mixing, duplicating the logic.

### B. Kinematics on Core 0 (inside uros_task)
Compute ERPM inside the `cmd_vel` callback and pass ERPM values to
Core 1. Functionally equivalent, but couples kinematics timing to
the micro-ROS spin rate. Keeping it on Core 1 ensures the conversion
and CAN send happen in the same deterministic loop.

## Consequences

- The ESP32 firmware must be configured with correct wheel diameter,
  track width, and motor pole pairs per robot build. Incorrect values
  will cause wrong velocities and curved driving.
- The `cmd_vel` interface is standard `geometry_msgs/Twist` —
  compatible with `nav2`, `teleop_twist_keyboard`, joystick nodes,
  etc. without any adaptation layer.
- The same kinematics constants are reused by odometry (ADR-0005)
  for the inverse conversion (ERPM → wheel velocity → odom).
- RC failsafe (ADR-0006) reuses the same `diff_drive_cmd_vel_to_erpm()`
  after arcade mixing, keeping one code path for motor commands.
