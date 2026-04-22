# ADR-0009: VESC Health Monitoring & Arming Policy

**Status:** Accepted
**Date:** 2026-04-21

## Context

The firmware drives two VESC motor controllers over a shared CAN bus
(ADR-0003, ADR-0007). Previously, the firmware started commanding ERPM
set-points to both VESCs immediately after `twai_start()`, with no
verification that the VESCs were present, correctly addressed, or
reporting plausible state.

Failure modes that went undetected at boot:

- VESC powered off, miswired, or unreachable (no ACK → TWAI TX errors
  but the system kept trying).
- Wrong controller ID configured in VESC Tool (`VESC_ID_LEFT` /
  `VESC_ID_RIGHT` mismatch) — commands would silently land on the
  wrong or no controller.
- CAN bitrate mismatch, missing termination (ADR-0007), or broken
  trunk — bus-off / error-passive state with no runtime visibility.
- Under-/over-voltage at the pack — motors might run briefly then
  trip internally.

The firmware also had no runtime watchdog: if a VESC dropped off the
bus mid-run (loose connector, thermal trip), the ESP32 would keep
commanding the other wheel, causing the robot to spin.

## Decision

Introduce a layered health-check policy, rolled out in three stages.
Each stage is independently shippable; this ADR covers all three for
coherence.

### Stage A — Passive presence check at boot (implemented)

After `twai_start()` and before the TX task begins commanding motion:

1. Start `can_rx_task` first.
2. Wait up to `VESC_HEALTH_BOOT_TIMEOUT_MS` (1500 ms default) for
   every configured VESC ID to broadcast both `CAN_PACKET_STATUS`
   (cmd 9) and `CAN_PACKET_STATUS_5` (cmd 27).
3. Verify `STATUS_5.voltage_in ∈ [VESC_VOLTAGE_MIN_V,
   VESC_VOLTAGE_MAX_V]`.
4. If all checks pass, set `BIT_ARMED` in the CAN event group.
5. Start `can_tx_task`. The task *always* runs, but transmits
   `ERPM=0` whenever `BIT_ARMED` is clear. Clocking the bus with
   zero set-points keeps the link active and matches the
   `FAILSAFE_STOP` semantics from ADR-0006.

Rationale for transmitting ERPM=0 rather than halting the firmware:
the system remains observable (debug console, diagnostic topics)
so the operator can fix wiring or configuration without reflashing.

Note: earlier drafts of this ADR and an earlier firmware revision
referenced `STATUS_4` (cmd 14) as the tachometer + voltage source.
That packet is actually `STATUS_2` in the VESC protocol and carries
amp-hours; the tachometer + `v_in` payload is in `STATUS_5` (cmd 27).
The decode and the VESC Tool requirement below reflect the correct
packet.

### Stage B — Active ping/pong (implemented)

VESC firmware answers `CAN_PACKET_PING` (cmd 17, 1-byte payload
= sender ID) with `CAN_PACKET_PONG` (cmd 18, 1-byte payload =
responder's controller ID). The ESP32 uses sender ID
`VESC_CAN_SENDER_ID = 0xFE` (matches VESC Tool convention; does
not collide with `VESC_ID_LEFT`/`RIGHT`).

Boot sequence:

1. Ping both VESCs in parallel, wait up to `VESC_PING_TIMEOUT_MS`
   (100 ms) for both pongs, retry `VESC_PING_RETRIES` times (2).
2. Proceed to Stage A status wait.

Ping/pong is authoritative for liveness because it does not depend
on the VESC being configured to broadcast periodic status. The
Stage A status wait still runs — it confirms the VESC is actually
producing the frames odometry needs and supplies voltage for the
plausibility check. Both must pass for `BIT_ARMED` to be set.

A missing PONG with present STATUS is theoretically possible but
only via misconfigured firmware or bus-addressing collisions; the
log in that case explicitly names the inconsistency.

### Stage C — Runtime watchdog & failsafe integration (implemented)

Each tick of `can_tx_task` (20 ms) runs `vesc_watchdog_check()`:

- Compares `esp_timer_get_time()/1000 − last_status_ms` per VESC
  against `VESC_STATUS_TIMEOUT_MS` (200 ms default — 10 missed
  frames at the 50 Hz default broadcast rate).
- Re-validates the most recent `voltage_in` against the
  `[VESC_VOLTAGE_MIN_V, VESC_VOLTAGE_MAX_V]` window, so battery
  sag under load trips the same failsafe path as a status timeout.
- On either condition, logs the event with the offending value,
  clears `vesc_health.online`, and forces `erpm = {0, 0}` in the
  TX path regardless of `drive_mode_t`.
- On recovery (status frames resume and voltage is back in range),
  logs and re-enables commands for that VESC.

The boot-time `online = false` state is **sticky**: if a VESC failed
the boot check, a later appearance of status frames does not arm it.
Runtime watchdog only downgrades a previously-armed VESC and can
re-upgrade it — the `BIT_ARMED` consent is still required.

The watchdog runs inside the existing `can_tx_task` rather than in a
separate task to avoid an additional context switch and to keep the
"do we send non-zero ERPM this tick?" decision local to one place.

### Orthogonal: TWAI bus-health alerts (implemented)

`twai_general_config_t.alerts_enabled` is configured with
`BUS_ERROR | BUS_OFF | ERR_PASS | TX_FAILED | RX_QUEUE_FULL`.
`twai_alert_handle()` runs at the top of each `can_tx_task` tick,
logs transitions, and auto-recovers from bus-off by calling
`twai_initiate_recovery()` followed by `twai_start()` (retried
with a 10 ms delay for up to 100 ms while the recovery timer
runs out).

This distinguishes "bus wiring/termination bad" from "one VESC
missing" — the former manifests as bus errors; the latter as
silent RX (no broadcasts observed).

### Orthogonal: fault code (deferred)

`vesc_health_t` carries a `fault_code` field intended for the VESC
fault state, but on FW 7.0 the periodic STATUS_1..6 packets do not
include a latched fault field; the fault code is served via the
request/response path (`CAN_PACKET_GET_VALUES` / `_SETUP`). Adding
this requires a small request/response state machine in `vesc_can.c`
and is deferred as a follow-up; the field stays as 0 at runtime until
then.

### Host visibility: `/vesc/{left,right}/battery` (implemented)

`uros_task` publishes a `sensor_msgs/BatteryState` per VESC:

- `voltage`  — `STATUS_5.voltage_in`.
- `present`  — `vesc_health.online` (reflects boot check + runtime
  watchdog verdict).
- Other fields are `NaN` / `UNKNOWN` per `sensor_msgs/BatteryState`
  convention since they are not measured by this layer.

Topic names use `left` / `right` rather than `1` / `2` because ROS 2
topic-name validation (REP 144) forbids any path segment starting
with a digit — the wheel-side label also reads more naturally.

This change raised `RMW_UXRCE_MAX_PUBLISHERS` from the default to 8
in `firmware/app-colcon.meta` to cover odom + failsafe + two battery
topics with headroom.

## Configuration

Defined as compile-time constants in `can_task.h` for now; they can
graduate to Kconfig entries if field tuning requires it.

| Symbol                          | Default | Purpose                                  |
|---------------------------------|---------|------------------------------------------|
| `VESC_HEALTH_BOOT_TIMEOUT_MS`   | 1500    | Max wait for both VESCs at boot          |
| `VESC_VOLTAGE_MIN_V`            | 18.0    | Lower plausibility bound                 |
| `VESC_VOLTAGE_MAX_V`            | 60.0    | Upper plausibility bound                 |
| `VESC_STATUS_TIMEOUT_MS` (C)    | 200     | Runtime watchdog threshold               |
| `VESC_PING_TIMEOUT_MS` (B)      | 100     | Per-attempt ping timeout                 |
| `VESC_PING_RETRIES` (B)         | 2       | Ping retry count                         |

The voltage window must be tuned to the target battery pack; the
default covers a generous range of typical LiPo/Li-ion configurations
compatible with the FSESC6.7 Pro.

## Alternatives Considered

### A. No boot check — rely on VESCs' internal safety only

Simplest, but loses all of the failure modes enumerated in Context.
The VESC protects itself; it does not protect the kinematic layer
from issuing motion commands that land nowhere.

### B. Ping-only, no passive check

Authoritative for liveness but loses voltage plausibility (which
requires `STATUS_5`) and wastes the existing status broadcasts.
Kept as Stage B alongside — not instead of — the passive check.

### C. Halt firmware on health-check failure

Reversible by reboot only; prevents diagnostics from being
publishable over micro-ROS. Rejected in favor of "stay up, stay
disarmed" — the operator learns the system is unhealthy through the
same channel they use to operate it.

### D. Clear `BIT_ARMED` on runtime watchdog trip

Would require operator intervention (reboot) to recover from a
momentary dropout. Rejected: the watchdog enforces a hard stop for
the duration of the dropout, which is the actionable safety property;
arming is a *boot-time* consent that runtime dropouts should not
invalidate.

## Consequences

- `can_task_init()` now takes up to `VESC_HEALTH_BOOT_TIMEOUT_MS`
  longer to return. Boot log clearly indicates per-VESC pass/fail.
- The system can boot without any VESC attached (bench test) — it
  stays disarmed, the bus is clocked with ERPM=0 frames, and the
  micro-ROS topics are publishable.
- Each VESC must be configured in VESC Tool to broadcast
  `STATUS` and `STATUS_5` (enum value `STATUS_1_2_3_4_5` or
  `STATUS_1_2_3_4_5_6` under *App Settings → General → Send CAN
  status*). If status broadcasts are disabled, Stage A fails;
  Stage B (ping) stays authoritative for liveness but voltage
  plausibility is lost.
- `vesc_health_t` is published over micro-ROS via
  `/vesc/{left,right}/battery` (see “Host visibility” above).
- Stages B and C require new protocol paths in `vesc_can.c` but
  no hardware changes.

## References

- ADR-0003 — VESC CAN protocol (frame formats, controller IDs).
- ADR-0006 — RC failsafe & mixing (`FAILSAFE_STOP` semantics reused).
- ADR-0007 — CAN bus topology (termination, bitrate).
- VESC bldc firmware, `comm_can.c` (ping/pong, STATUS_1–5).
- REP 144 — ROS topic and service name conventions (digit-leading
  segments rejected; drove `/vesc/{left,right}/battery` naming).
