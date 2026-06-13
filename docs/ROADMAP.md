# Roadmap & Future Work

This project has no formal product backlog. This file consolidates the
known future work that is otherwise scattered across the ADRs, the
`TBD`/`(planned)` entries in [architecture.md](architecture.md), and
`TODO`/milestone markers in the firmware source.

Status legend: 🔲 not started · 🟡 partial · ✅ done (tracked elsewhere)

## Near-term gaps

### 🔲 `motor/status` telemetry topic
The ROS 2 interface table reserves `motor/status` but no publisher
exists yet. The data is already collected — `motor_driver_get_health()`
returns `online` / `boot_passed` / `fault_bits`, and `motor_feedback_t`
carries per-wheel RPM, current, and raw fault codes. What's missing is
the publisher wiring in [`uros_task.c`](../firmware/main/uros_task.c).
This also closes the "ideally publish a health/status topic" half of the
stale-feedback review item (the firmware-side freshness gate already
landed in `motor_task`).

Suggested payload: per-wheel velocity/current, decoded `MOTOR_FAULT_*`
bits, bus voltage, and an online/stale flag.

### 🔲 ZLAC bus voltage
`motor_feedback_t.bus_voltage_v` is hard-zero in the ZLAC backend —
voltage is not in the TPDO map (see the comment at the bottom of
`motor_driver_get_feedback` in
[`motor_driver_zlac8015d.c`](../firmware/main/motor_driver_zlac8015d.c)).
Add a periodic SDO read (or map it into a TPDO) so `motor/status` and
battery telemetry have a real voltage. Prerequisite for the item below.

### 🟡 Backend-agnostic battery publishing
`publish_battery()` is VESC-only today and queries the VESC backend
directly ([`uros_task.c`](../firmware/main/uros_task.c), guarded by
`CONFIG_MOTOR_DRIVER_VESC`). Generalize it to publish from
`motor_feedback_t.bus_voltage_v` so the ZLAC build gets battery state
too — depends on the ZLAC bus-voltage item above.

### 🔲 VESC fault decode
`motor_health_t.fault_bits` is always 0 for the VESC backend
(`/* VESC fault decode is TODO */` in
[`motor_driver_vesc.c`](../firmware/main/motor_driver_vesc.c)). Map the
VESC fault codes onto the `MOTOR_FAULT_*` buckets so faults surface in
health/status telemetry.

## Milestones referenced in firmware

Named in the odometry covariance comment of
[`uros_task.c`](../firmware/main/uros_task.c):

- **M1 — odometry closure.** Bench square-drive test; tune the `odom`
  pose/twist covariance diagonals against measured drift closure. This
  is a firmware/bench task. The `reset_odom` service
  (`std_srvs/Trigger`) zeroes the pose between trials without a reboot.
  Procedure + tooling: [odometry-calibration.md](odometry-calibration.md)
  and the `scripts/calibration_drive.py` / `scripts/calibrate_constants.py`
  helpers.

(Host-side IMU/EKF fusion is intentionally *not* tracked here — the IMU
and the fusion live on the main ROS computer, so the firmware has no
deliverable beyond publishing honest `odom` covariance, which already
ships. See the covariance entry under "Done" below.)

## Larger / unscheduled

- **Multi-node CANopen.** The heartbeat table now spans node IDs 1–127,
  so a second drive / additional CiA 402 node is feasible without
  structural changes.
- **RS485 port.** Isolated RS485 hardware is present but unused (see the
  hardware table in [architecture.md](architecture.md)).

## Done (tracked in ADRs, listed for context)

- ✅ ZLAC8015D CANopen backend behind the motor HAL — [ADR-0010](adr/0010-zlac8015d-canopen-migration.md).
- ✅ Boot health check + runtime watchdog + bus-off recovery (VESC) —
  [ADR-0009](adr/0009-vesc-health-monitoring.md) (all three stages).
- ✅ Agent-synced odom timestamps + covariance — `uros_task`. Honest
  covariance diagonals so a host-side EKF will fuse the wheel `odom`
  (all-zeros reads as "infinitely certain" and breaks fusion).
- ✅ `reset_odom` service (`std_srvs/Trigger`) — zeroes the odom pose
  without a reboot ([ADR-0005](adr/0005-odometry-computation.md)).
