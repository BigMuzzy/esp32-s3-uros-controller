# ADR-0011: `rc_active` Heartbeat for the Host Velocity Mux

**Status:** Accepted
**Date:** 2026-09-26
**Related:** ADR-0006 (RC failsafe & mixing); host repository ADR-0005
(velocity arbitration with `twist_mux`)

## Context

The host selects its velocity source with `twist_mux` (host ADR-0005):
Nav2, local teleop and fleet teleop are mux inputs, and locks mask them.
While the RC mode switch has the robot in manual mode, the firmware already
ignores `cmd_vel` (ADR-0006), but the host does not know. Its inputs keep
streaming, and when the operator flips back to autonomous, whatever the host
was sending takes over at once.

The existing `failsafe/active` topic cannot serve as the host's RC signal.
It is `true` whenever the mode is not `DRIVE_MODE_AUTONOMOUS`, which includes
`DRIVE_MODE_FAILSAFE_STOP`: a stale `cmd_vel`. As a mux lock it would mask
every input exactly when a fresh command is needed to leave the failsafe, and
the robot would never recover.

## Decision

Publish `rc_active` (`std_msgs/Bool`) from `uros_task`, in the same spin-loop
iteration as `odom` and `failsafe/active` (every `UROS_SPIN_PERIOD_MS`,
nominally 10 ms, when the loop is not blocked on transport):

- `data` is `true` exactly when `rc_failsafe_get_mode()` returns
  `DRIVE_MODE_MANUAL`, and `false` in `DRIVE_MODE_AUTONOMOUS` and
  `DRIVE_MODE_FAILSAFE_STOP`.
- It is a heartbeat, not an event. The host lock has a timeout (0.5 s), so a
  firmware reset, a lost agent session or a dead USB link also masks every
  ROS input, which is the safe direction.
- `failsafe/active` is unchanged and keeps its meaning.

## Consequences

- The host's `/rc_active` lock (`twist_mux`, priority 200) stays off by
  default until this firmware is flashed; enabled against older firmware it
  would expire and block every input.
- One more publisher per session (limit `RMW_UXRCE_MAX_PUBLISHERS=8`; the
  ZLAC build uses 3).
- Manual-mode motor control is unchanged: RC still drives the motors
  directly (ADR-0006). The heartbeat only tells the host to stand down.
