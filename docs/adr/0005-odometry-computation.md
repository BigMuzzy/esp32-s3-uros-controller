# ADR-0005: Odometry Computation

**Status:** Proposed
**Date:** 2026-04-10

## Context

A differential-drive robot needs wheel odometry to estimate its pose
(x, y, θ) for navigation. The two VESCs broadcast status frames on
the CAN bus that include tachometer counts (cumulative ERPM ticks)
and instantaneous ERPM (ADR-0003). The ESP32 can use this data to
compute odometry and publish `nav_msgs/Odometry` to the master PC.

Odometry could alternatively be computed on the master PC from raw
telemetry, but ADR-0004 already places kinematics on the ESP32.
Computing odometry on the same device keeps the forward and inverse
kinematics co-located and uses the same robot parameters.

## Decision

**Odometry is computed on Core 1 and published via micro-ROS on Core 0.**

### Data source

The VESC **tachometer** value from `CAN_PACKET_STATUS_4` (command 14)
is used. It provides cumulative ERPM ticks since VESC power-on —
a monotonically increasing counter that doesn't lose counts between
reads (unlike integrating instantaneous ERPM, which is sensitive to
timing jitter).

### Conversion: tachometer → wheel distance

```
delta_tach       = tach_now - tach_prev          // ERPM ticks
delta_mech_rev   = delta_tach / motor_pole_pairs // mechanical revolutions
delta_distance   = delta_mech_rev × π × wheel_diameter  // meters
```

Uses the same `WHEEL_DIAMETER_M` and `MOTOR_POLE_PAIRS` constants
defined in ADR-0004.

### Diff-drive odometry integration

Per update cycle (each time new tachometer values arrive for both
wheels):

```
d_left  = left wheel distance since last update
d_right = right wheel distance since last update

ds = (d_right + d_left) / 2          // linear displacement
dθ = (d_right - d_left) / track_width // angular displacement

θ  += dθ
x  += ds × cos(θ)
y  += ds × sin(θ)
```

This is the standard first-order dead-reckoning model. Sufficient
for short-range odometry; the master PC can fuse with IMU/lidar
for drift correction.

### Velocity estimate

Instantaneous velocity is computed from ERPM in `CAN_PACKET_STATUS`
(command 9):

```
wheel_rpm  = erpm / motor_pole_pairs
wheel_vel  = wheel_rpm × π × wheel_diameter / 60    // m/s

v     = (v_right + v_left) / 2       // linear velocity
omega = (v_right - v_left) / track_width  // angular velocity
```

Published in the `twist` field of the Odometry message.

### Data flow

```
Core 1                                Core 0
──────                                ──────
can_rx_task                           uros_task
  │                                     │
  ├─ VESC L STATUS_4 → tach_l          │
  ├─ VESC R STATUS_4 → tach_r          │
  │                                     │
  ├─ diff_drive_update_odom()           │
  │   (x, y, θ, v, ω)                  │
  │                                     │
  └─ write odom struct ──────────────► read odom struct
     (spinlock)                         │
                                        └─ publish nav_msgs/Odometry
```

The odom struct is passed Core 1 → Core 0 via a spinlock-protected
shared struct (same pattern as `cmd_vel` in ADR-0004, reverse
direction).

### Publishing

| Field                   | Value                                  |
|-------------------------|----------------------------------------|
| Topic                   | `odom`                                 |
| Message type            | `nav_msgs/Odometry`                    |
| Rate                    | ~50 Hz (matched to VESC status rate)   |
| `header.frame_id`       | `"odom"`                               |
| `child_frame_id`        | `"base_link"`                          |
| `pose.pose`             | x, y, θ (as quaternion)                |
| `twist.twist`           | v (linear.x), ω (angular.z)           |
| `pose.covariance`       | Diagonal, placeholder values           |
| `twist.covariance`      | Diagonal, placeholder values           |

**TF broadcast (`odom` → `base_link`) is NOT published by the ESP32.**
The master PC is responsible for TF — typically via `robot_localization`
or `robot_state_publisher`. This avoids adding a TF publisher on the
constrained micro-ROS side and allows the host to fuse odometry with
other sensors before broadcasting the transform.

### API sketch

```c
// diff_drive.h (extends ADR-0004)
typedef struct {
    float x;           // meters
    float y;           // meters
    float theta;       // radians
    float linear_vel;  // m/s
    float angular_vel; // rad/s
    int32_t tach_left_prev;
    int32_t tach_right_prev;
} odom_state_t;

void diff_drive_update_odom(odom_state_t *state,
                            int32_t tach_left, int32_t tach_right,
                            int32_t erpm_left, int32_t erpm_right);

void diff_drive_reset_odom(odom_state_t *state);
```

## Alternatives Considered

### A. Odometry on the master PC
Publish raw tachometer/ERPM values and let a host-side node compute
odom. This works but:
- Adds a custom message type for raw VESC data.
- Separates forward and inverse kinematics across two systems using
  the same physical constants — a maintenance risk.
- Increases latency: raw data → USB → host → odom node → publish.

### B. Integrate ERPM instead of using tachometer
Read instantaneous ERPM and multiply by `dt` to estimate distance.
Sensitive to timing jitter — if the read interval varies, distance
accumulates error. The tachometer is a hardware counter that doesn't
miss ticks, making it strictly more accurate.

### C. Publish TF from micro-ROS
micro-ROS supports TF publishing, but:
- Adds another publisher and message type on the constrained device.
- Prevents the host from fusing odom with IMU/lidar before TF.
- Standard practice in ROS is to have one node publish each TF frame.

## Consequences

- Odometry accuracy depends on correct `WHEEL_DIAMETER_M`,
  `TRACK_WIDTH_M`, and `MOTOR_POLE_PAIRS` — same constants as
  kinematics (ADR-0004). Measure carefully.
- Odometry will drift over time (no absolute reference). The master
  PC should fuse with IMU and/or lidar for corrected localization.
- The odom publish rate is coupled to VESC status broadcast rate.
  If VESC status rate is changed in VESC Tool, odom rate follows.
- `diff_drive_reset_odom()` should be callable via a micro-ROS
  service in the future if the host needs to reset the pose.
