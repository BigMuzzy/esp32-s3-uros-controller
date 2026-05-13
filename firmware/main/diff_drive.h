/*
 * diff_drive.h — Differential-drive kinematics & odometry
 *
 * Pure math module — no hardware dependencies. Converts between
 * cmd_vel (linear + angular velocity) and per-wheel ERPM, and
 * integrates wheel displacement into a 2D pose estimate.
 *
 * Forward path (cmd_vel → motors):
 *   cmd_vel_t  →  diff_drive_cmd_vel_to_erpm()  →  wheel_erpm_t
 *   Called by can_tx_task (Core 1) each cycle.
 *
 * Inverse path (motors → odometry):
 *   tachometer deltas  →  diff_drive_update_odom()  →  odom_state_t
 *   Called by can_rx_task (Core 1) on each VESC status update.
 *
 * Robot parameters are compile-time constants — set per physical
 * build (chassis dimensions, motor pole pairs).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Robot parameters (set per build) ────────────────────────────── */

#define WHEEL_DIAMETER_M     0.160f   /* meters — matches VESC MCCONF_SI_WHEEL_DIAMETER */
#define TRACK_WIDTH_M        0.600f   /* wheel-to-wheel center, meters */
#define MOTOR_POLE_PAIRS     7        /* VESC MCCONF_SI_MOTOR_POLES=14 → 7 pairs */
#define MAX_WHEEL_ERPM       1000     /* matches VESC MCCONF_L_RPM_MAX (±1000) */
#define MAX_MANUAL_SPEED_MS  1.0f     /* max linear vel in manual mode (m/s) */
#define MAX_MANUAL_ANGVEL    0.67f    /* max angular vel in manual mode (rad/s);
                                       * full stick + full steer = 1.0 + 0.67×0.3 ≈ 1.20 m/s
                                       * = MAX_WHEEL_ERPM ceiling */

/* ── Types ───────────────────────────────────────────────────────── */

/** Velocity command in robot frame. */
typedef struct {
    float linear_x;    /* m/s, positive = forward */
    float angular_z;   /* rad/s, positive = CCW */
} cmd_vel_t;

/** Per-wheel ERPM set-points, ready for vesc_can_encode_rpm(). */
typedef struct {
    int32_t left_erpm;
    int32_t right_erpm;
} wheel_erpm_t;

/** Accumulated 2D pose and instantaneous velocity. */
typedef struct {
    /* Pose (integrated from tachometer deltas) */
    float x;           /* meters */
    float y;           /* meters */
    float theta;       /* radians, normalized to [-π, π] */

    /* Velocity (from instantaneous ERPM) */
    float linear_vel;  /* m/s */
    float angular_vel; /* rad/s */

    /* Previous tachometer readings for delta computation */
    int32_t tach_left_prev;
    int32_t tach_right_prev;
    bool    tach_initialized;
} odom_state_t;

/* ── Forward kinematics (cmd_vel → wheel ERPM) ──────────────────── */

/**
 * Convert a velocity command to left/right wheel ERPM.
 *
 * Applies diff-drive kinematics:
 *   v_left  = linear_x - angular_z × (track_width / 2)
 *   v_right = linear_x + angular_z × (track_width / 2)
 *
 * Then converts m/s → RPM → ERPM and clamps to ±MAX_WHEEL_ERPM.
 *
 * @param cmd  Desired robot velocity.
 * @return     Clamped ERPM for left and right wheels.
 */
wheel_erpm_t diff_drive_cmd_vel_to_erpm(const cmd_vel_t *cmd);

/* ── Inverse kinematics / odometry ──────────────────────────────── */

/**
 * Initialize odometry state to zero pose.
 * Must be called once before diff_drive_update_odom().
 */
void diff_drive_odom_init(odom_state_t *state);

/**
 * Update odometry from new VESC tachometer and ERPM readings.
 *
 * Pose is integrated from tachometer deltas (cumulative, no lost
 * counts). Velocity is computed from instantaneous ERPM.
 *
 * The first call after init records the tachometer baseline and
 * does not update the pose.
 *
 * @param state       Odometry state (updated in place).
 * @param tach_left   Current tachometer value from left VESC (STATUS_4).
 * @param tach_right  Current tachometer value from right VESC (STATUS_4).
 * @param erpm_left   Current ERPM from left VESC (STATUS).
 * @param erpm_right  Current ERPM from right VESC (STATUS).
 */
void diff_drive_update_odom(odom_state_t *state,
                            int32_t tach_left, int32_t tach_right,
                            int32_t erpm_left, int32_t erpm_right);

/**
 * Reset pose to origin (0, 0, 0) without losing tachometer baseline.
 * Velocity is unaffected.
 */
void diff_drive_reset_odom(odom_state_t *state);

#ifdef __cplusplus
}
#endif
