/*
 * rc_failsafe.h — RC PWM input, arcade mixing, and failsafe logic
 *
 * Reads 3 RC channels via MCPWM capture (hardware-timestamped,
 * minimal CPU load):
 *   CH1 (GPIO 4) — steering
 *   CH2 (GPIO 5) — throttle
 *   CH3 (GPIO 6) — mode switch (autonomous / manual)
 *
 * Operating modes (see ADR-0006):
 *
 *   AUTONOMOUS    — cmd_vel drives motors, RC ignored
 *   FAILSAFE_STOP — motors stopped (cmd_vel timeout, mode=auto)
 *   MANUAL        — RC sticks drive motors via arcade mixing,
 *                   cmd_vel ignored
 *
 * Mode switch always wins: flipping to manual gives the operator
 * immediate control. Flipping back to autonomous resumes cmd_vel
 * (or enters FAILSAFE_STOP if cmd_vel has timed out).
 *
 * In MANUAL mode, arcade mixing converts throttle + steering into
 * a cmd_vel_t, then uses diff_drive_cmd_vel_to_erpm() — same code
 * path as autonomous driving.
 *
 * All functions run on Core 1 (rc_failsafe_task), except
 * drive_mode which is read by Core 0 for status publishing.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "diff_drive.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ───────────────────────────────────────────────── */

#define RC_CH1_GPIO           4       /* steering */
#define RC_CH2_GPIO           5       /* throttle */
#define RC_CH3_GPIO           6       /* mode switch */

#define RC_PWM_CENTER_US      1500    /* neutral stick position */
#define RC_PWM_RANGE_US       500     /* half-range: 1000–2000 µs */
#define RC_DEADBAND_US        25      /* ±25 µs around center = zero */
#define RC_SIGNAL_TIMEOUT_MS  100     /* RC signal lost if no edges */
#define CMD_VEL_TIMEOUT_MS    500     /* cmd_vel stale threshold */

/* ── Types ───────────────────────────────────────────────────────── */

typedef enum {
    DRIVE_MODE_AUTONOMOUS,
    DRIVE_MODE_FAILSAFE_STOP,
    DRIVE_MODE_MANUAL,
} drive_mode_t;

/** Raw RC channel readings + derived state. */
typedef struct {
    uint16_t ch1_us;       /* steering pulse width, µs */
    uint16_t ch2_us;       /* throttle pulse width, µs */
    bool     ch3_manual;   /* true if mode switch in manual position */
    bool     signal_ok;    /* true if valid PWM edges within timeout */
} rc_input_t;

/* ── Initialization ──────────────────────────────────────────────── */

/**
 * Configure MCPWM capture on the three RC GPIO pins.
 * Must be called once from app_main() before starting rc_failsafe_task.
 *
 * @return ESP_OK on success, or an esp_err_t on MCPWM config failure.
 */
esp_err_t rc_failsafe_init(void);

/* ── Called from rc_failsafe_task (Core 1, ~20 ms loop) ─────────── */

/**
 * Read latest RC channel values from MCPWM capture ISR data.
 * Non-blocking — returns the most recent capture result.
 */
rc_input_t rc_failsafe_read(void);

/**
 * Determine current drive mode based on:
 *   - RC mode switch position
 *   - RC signal presence
 *   - Time since last cmd_vel (call rc_failsafe_notify_cmd_vel()
 *     from the cmd_vel callback to reset the timer)
 */
drive_mode_t rc_failsafe_get_mode(void);

/**
 * Arcade-mix RC inputs into a cmd_vel suitable for
 * diff_drive_cmd_vel_to_erpm(). Only valid in MANUAL mode.
 *
 * Applies deadband, normalizes to [-1, +1], scales by
 * MAX_MANUAL_SPEED_MS and MAX_MANUAL_ANGVEL (from diff_drive.h).
 *
 * @param rc  RC input from rc_failsafe_read().
 * @return    Mixed velocity command.
 */
cmd_vel_t rc_failsafe_arcade_mix(const rc_input_t *rc);

/* ── Cross-core interface ────────────────────────────────────────── */

/**
 * Notify that a valid cmd_vel was received.
 * Called from uros_task (Core 0) cmd_vel subscription callback.
 * Resets the cmd_vel timeout timer (thread-safe, uses atomic).
 */
void rc_failsafe_notify_cmd_vel(void);

#ifdef __cplusplus
}
#endif
