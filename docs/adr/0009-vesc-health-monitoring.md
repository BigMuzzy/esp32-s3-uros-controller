# ADR-0009: VESC Health Monitoring & Arming Policy

**Status:** Proposed
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
   (cmd 9) and `CAN_PACKET_STATUS_4` (cmd 14).
3. Verify `STATUS_4.voltage_in ∈ [VESC_VOLTAGE_MIN_V,
   VESC_VOLTAGE_MAX_V]`.
4. If all checks pass, set `BIT_ARMED` in the CAN event group.
5. Start `can_tx_task`. The task *always* runs, but transmits
   `ERPM=0` whenever `BIT_ARMED` is clear. Clocking the bus with
   zero set-points keeps the link active and matches the
   `FAILSAFE_STOP` semantics from ADR-0006.

Rationale for transmitting ERPM=0 rather than halting the firmware:
the system remains observable (debug console, diagnostic topics)
so the operator can fix wiring or configuration without reflashing.

### Stage B — Active ping/pong (planned)

VESC firmware answers `CAN_PACKET_PING` (cmd 17, 1-byte payload
= sender ID) with `CAN_PACKET_PONG` (cmd 18). Stage B adds:

- `vesc_can_encode_ping(target_id, sender_id, out_msg)`.
- Pong decode in `can_rx_task` → sets a per-VESC event-group bit.
- A boot-time ping exchange (3× retries, 100 ms timeout each)
  that runs *before* the Stage A status wait.

Ping/pong is authoritative for liveness because it does not depend
on the VESC being configured to broadcast periodic status. Stage A's
status wait remains as a secondary check that also populates voltage.

### Stage C — Runtime watchdog & failsafe integration (planned)

Each tick of `can_tx_task` compares `esp_timer_get_time() -
last_status_ms` per VESC against `VESC_STATUS_TIMEOUT_MS`
(200 ms default). If the timeout trips:

- Clear `vesc_health.online` for that VESC.
- Force `erpm = {0, 0}` regardless of `drive_mode_t`.
- Publish the degraded state on the failsafe / diagnostics topic.

Once the VESC resumes broadcasting, `online` is set again, but
`BIT_ARMED` is **not** cleared by the watchdog — a runtime dropout
does not require a reboot to recover, it just forces a stop for its
duration.

### Orthogonal: TWAI bus-health alerts

`twai_general_config_t.alerts_enabled` is configured with
`BUS_ERROR | BUS_OFF | ERR_PASS | TX_FAILED | RX_QUEUE_FULL`.
A periodic `twai_read_alerts()` call (folded into `can_tx_task`
or a small dedicated task in Stage C) logs transitions and
invokes `twai_initiate_recovery()` on bus-off.

This distinguishes "bus wiring/termination bad" from "one VESC
missing" — the former manifests as bus errors; the latter as
silent RX (no broadcasts observed).

### Orthogonal: fault code (Stage B/C)

`CAN_PACKET_STATUS_6` (cmd 28) carries the VESC fault code. Decoded
in `vesc_can.c`, stored in `vesc_health_t.fault_code`. A non-zero
fault latches `online = false` until the VESC clears it.

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
requires `STATUS_4`) and wastes the existing status broadcasts.
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
- Each VESC must be configured in VESC Tool to broadcast `STATUS`
  and `STATUS_4` (default VESC Tool behavior). If status broadcasts
  are disabled, Stage A fails; Stage B (ping) becomes mandatory.
- The `vesc_health_t` struct is available to `uros_task` for
  publishing a diagnostics topic; the actual publication is
  out of scope for this ADR.
- Stages B and C require new protocol paths in `vesc_can.c` but
  no hardware changes.

## References

- ADR-0003 — VESC CAN protocol (frame formats, controller IDs).
- ADR-0006 — RC failsafe & mixing (`FAILSAFE_STOP` semantics reused).
- ADR-0007 — CAN bus topology (termination, bitrate).
- VESC bldc firmware, `comm_can.c` (ping/pong, STATUS/STATUS_4/STATUS_6).
