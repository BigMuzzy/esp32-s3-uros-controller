/*
 * motor_task.h — Upper control loop (Core 1)
 *
 * Owns the 20 ms control tick that coordinates failsafe mode,
 * cmd_vel, RC arcade mix, and tune-override sources, then issues
 * a wheel-RPM setpoint to the motor_driver HAL.  Also owns
 * odometry integration from the HAL's feedback.
 *
 * Hardware-agnostic: this module never references VESC, CANopen,
 * ERPM, or any controller-native unit — only wheel-side quantities
 * via motor_driver.h and diff_drive.h.
 *
 * Lifecycle:
 *   motor_task_init() — after motor_driver_init() and rc_failsafe_init()
 *
 * Cross-core surface:
 *   set_cmd_vel       — written by uros_task (Core 0) on each
 *                        cmd_vel subscription callback.
 *   set/clear_tune_override — written by tune_cli (Core 0).
 *   get_odom          — read by uros_task (Core 0) at the publish rate.
 *
 * All cross-core accessors are spinlock-protected and never block.
 */

#pragma once

#include "esp_err.h"
#include "diff_drive.h"   /* cmd_vel_t, odom_state_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ───────────────────────────────────────────────── */

#define MOTOR_TASK_PERIOD_MS    20
#define MOTOR_TASK_STACK        4096
#define MOTOR_TASK_PRIO         5     /* higher than uros_task */

/* Motor feedback older than this is treated as stale.  Odometry then
 * reports zero wheel velocity instead of freezing the last-known value:
 * motor_driver_get_feedback() keeps returning the most recent snapshot
 * forever once any has been seen, so a mid-mission comms dropout would
 * otherwise keep publishing a stale (possibly non-zero) velocity at the
 * loop rate.  Both backends refresh feedback well inside this window in
 * healthy operation (ZLAC TPDO event timer, VESC STATUS broadcasts). */
#define MOTOR_FEEDBACK_TIMEOUT_MS  200

/* Watchdog timeout for the tune override (ms). */
#define TUNE_OVERRIDE_TIMEOUT_MS  150

/* ── Lifecycle ───────────────────────────────────────────────────── */

/**
 * Start the upper control loop task on Core 1.  motor_driver_init()
 * and rc_failsafe_init() must have been called first.
 */
esp_err_t motor_task_init(void);

/* ── Core 0 → Core 1: cmd_vel input ─────────────────────────────── */

/**
 * Set the latest cmd_vel from the micro-ROS subscription callback.
 * Also calls rc_failsafe_notify_cmd_vel() to reset the timeout.
 * Thread-safe.
 */
void motor_task_set_cmd_vel(const cmd_vel_t *cmd);

/* ── Core 1 → Core 0: odometry output ───────────────────────────── */

/**
 * Read the latest odometry state computed by the control tick.
 * Thread-safe.
 */
void motor_task_get_odom(odom_state_t *odom_out);

/* ── Tune-mode override ─────────────────────────────────────────── */

/**
 * Install a per-wheel RPM override from the tuning CLI.
 *
 * While the override is fresh (refreshed within
 * TUNE_OVERRIDE_TIMEOUT_MS), the control tick substitutes these
 * RPM targets for the normal cmd_vel / RC-derived targets ONLY when
 * the RC failsafe drive mode is AUTONOMOUS or FAILSAFE_STOP.
 * MANUAL mode (sticks live) always wins over the override.
 *
 * The override also requires the system to be armed
 * (motor_driver_is_armed()).
 *
 * Thread-safe.  Safe to call from any task.
 */
void motor_task_set_tune_override(float left_rpm, float right_rpm);

/** Immediately clear any active tune override.  Thread-safe. */
void motor_task_clear_tune_override(void);

#ifdef __cplusplus
}
#endif
