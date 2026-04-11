# ADR-0006: RC Failsafe Behavior & Mixing

**Status:** Proposed
**Date:** 2026-04-10

## Context

The robot needs a manual override / failsafe mode when the master PC
is unreachable or misbehaving. An RC receiver connected to the ESP32-S3
provides human-in-the-loop control independent of the software stack.

The ESP32-S3 has 16 free GPIOs on the 20-pin header (GPIO 1–14, 43, 44)
after accounting for CAN (15, 16), RS485 (17, 18, 21), USB (19, 20),
I2C/RTC (38, 39), and CH1 control (47). The MCPWM capture peripheral
can read PWM on any GPIO via the GPIO matrix.

## Decision

### RC channels

Three PWM channels from a standard RC receiver:

| Channel | Function     | GPIO | Notes                              |
|---------|-------------|------|------------------------------------|
| CH1     | Steering    | 4    | Left/right, 1000–2000 µs          |
| CH2     | Throttle    | 5    | Forward/reverse, 1000–2000 µs     |
| CH3     | Mode switch | 6    | 2-position: autonomous / manual   |

GPIO 4, 5, 6 are chosen for physical proximity on the header. Any free
GPIO would work — these can be changed in a config header.

Standard RC PWM: 1000–2000 µs pulse width, ~50 Hz (20 ms period),
center at 1500 µs.

### PWM reading method

**MCPWM capture** — the ESP32-S3 MCPWM peripheral has hardware capture
units that timestamp rising and falling edges with minimal CPU usage.
Each channel uses one capture channel. An ISR records the pulse width;
the `rc_failsafe_task` reads the latest values.

### Operating modes

The system has three states, determined by mode switch position and
cmd_vel availability:

```
                    ┌─────────────────────┐
                    │    AUTONOMOUS       │
                    │  cmd_vel → motors   │
                    │  RC ignored         │
                    └────────┬────────────┘
                             │ cmd_vel timeout
                             │ (500 ms)
                             ▼
                    ┌─────────────────────┐
                    │    FAILSAFE_STOP    │
                    │  motors stopped     │
                    │  waiting for RC     │
                    │  or cmd_vel recovery│
                    └────────┬────────────┘
                             │ mode switch
                             │ → manual
                             ▼
┌──────────────┐    ┌─────────────────────┐
│ mode switch  │───►│    MANUAL           │
│ → autonomous │    │  RC sticks → motors │
│              │◄───│  cmd_vel ignored    │
└──────────────┘    └─────────────────────┘
```

| State           | Motor source       | Entry condition                          |
|-----------------|--------------------|------------------------------------------|
| AUTONOMOUS      | `cmd_vel`          | Mode switch = autonomous AND cmd_vel ok  |
| FAILSAFE_STOP   | Zero (stopped)     | Mode switch = autonomous AND cmd_vel timeout |
| MANUAL          | RC sticks          | Mode switch = manual                     |

**Mode switch always wins.** If the operator flips to manual, RC
controls the robot immediately regardless of cmd_vel state. Flipping
back to autonomous resumes cmd_vel control (or enters FAILSAFE_STOP
if cmd_vel has timed out).

### Arcade mixing

In MANUAL mode, the RC throttle and steering inputs are mixed into
left/right wheel commands using standard arcade drive:

```
throttle = (ch2_us - 1500) / 500.0    // −1.0 to +1.0
steering = (ch1_us - 1500) / 500.0    // −1.0 to +1.0

left  = throttle + steering
right = throttle - steering

// clamp to [−1.0, +1.0], then scale to cmd_vel
cmd.linear_x  = (left + right) / 2.0 × MAX_MANUAL_SPEED
cmd.angular_z = (right - left) / TRACK_WIDTH_M
```

The result is passed through the same `diff_drive_cmd_vel_to_erpm()`
function from ADR-0004, keeping a single code path to the motors.

### RC signal loss detection

If no valid PWM edges are detected on any channel for >100 ms, the
RC signal is considered lost. Behavior depends on current state:

| State      | RC signal lost                                  |
|------------|-------------------------------------------------|
| AUTONOMOUS | No effect — RC is not used                      |
| MANUAL     | Transition to FAILSAFE_STOP (motors stopped)    |

### Deadband

A ±25 µs deadband around center (1500 µs) prevents motor creep when
sticks are released. Pulse widths between 1475–1525 µs are treated
as zero.

### Safety limits

- `MAX_MANUAL_SPEED` — configurable maximum linear velocity in manual
  mode (e.g. 1.0 m/s), independent of autonomous speed limits.
- `MAX_MANUAL_ANGULAR` — configurable maximum angular velocity.
- ERPM clamp from ADR-0004 still applies as a final safety layer.

### API sketch

```c
// rc_failsafe.h
typedef enum {
    MODE_AUTONOMOUS,
    MODE_FAILSAFE_STOP,
    MODE_MANUAL,
} drive_mode_t;

typedef struct {
    uint16_t ch1_us;     // steering pulse width
    uint16_t ch2_us;     // throttle pulse width
    bool     ch3_manual; // mode switch state
    bool     signal_ok;  // true if valid PWM detected recently
} rc_input_t;

void rc_failsafe_init(void);           // configure MCPWM capture on GPIO 4,5,6
rc_input_t rc_failsafe_read(void);     // get latest RC values
drive_mode_t rc_failsafe_get_mode(void);
```

### Data flow

```
Core 1
──────
rc_failsafe_task (20 ms loop)
  │
  ├─ rc_failsafe_read()          ← MCPWM capture ISR data
  ├─ rc_failsafe_get_mode()      ← mode switch + cmd_vel timeout
  │
  ├─ if MANUAL:
  │    arcade_mix() → cmd_vel_t
  │    diff_drive_cmd_vel_to_erpm()
  │    vesc_can_set_rpm(L, R)
  │
  ├─ if FAILSAFE_STOP:
  │    vesc_can_set_rpm(0, 0)
  │
  └─ if AUTONOMOUS:
       (can_tx_task handles cmd_vel → motors)

  publish drive_mode → Core 0 → failsafe/active topic
```

## Alternatives Considered

### A. PCNT peripheral for PWM reading
PCNT counts pulses but doesn't directly measure pulse width timing.
Would require combining PCNT with a timer for width measurement —
more complex than MCPWM capture which does this natively.

### B. GPIO interrupt + `esp_timer_get_time()`
Simple software approach: ISR on rising/falling edges, compute delta
with a high-res timer. Works but is more CPU-intensive than hardware
capture and susceptible to interrupt latency jitter. MCPWM capture
timestamps edges in hardware.

### C. Automatic failsafe (no mode switch)
Use only cmd_vel timeout to trigger RC takeover. Dangerous — RC
sticks might be in a non-neutral position when failsafe engages,
causing unexpected motion. An explicit mode switch gives the
operator clear intentional control.

### D. Failsafe on the RC receiver side
Many RC receivers output a pre-configured failsafe pulse on signal
loss. This is a receiver-level feature, not a substitute for
ESP32-level failsafe. The ESP32 cannot distinguish "receiver
failsafe output" from "operator input" without a mode switch.

## Consequences

- Three GPIO pins (4, 5, 6) are reserved for RC input.
- An RC receiver must be physically connected for manual mode.
  Without it, the system operates in AUTONOMOUS / FAILSAFE_STOP
  only (RC signal loss detection handles this gracefully).
- The mode switch provides a hard override — useful for field
  testing and emergency stop scenarios.
- `failsafe/active` topic lets the master PC know when it has lost
  control, enabling higher-level responses (e.g. logging, alerts).
- The arcade mixing reuses `diff_drive_cmd_vel_to_erpm()` from
  ADR-0004, keeping motor command logic in one place.
