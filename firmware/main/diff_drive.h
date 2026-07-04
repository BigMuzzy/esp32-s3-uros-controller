/*
 * diff_drive.h — Differential-drive kinematics & odometry
 *
 * Pure math module — no hardware dependencies. Converts between
 * cmd_vel (linear + angular velocity) and per-wheel RPM, and
 * integrates per-wheel revolutions into a 2D pose estimate.
 *
 * Forward path (cmd_vel → motors):
 *   cmd_vel_t  →  diff_drive_cmd_vel_to_wheel_rpm()  →  motor_wheel_cmd_t
 *
 * Inverse path (motors → odometry):
 *   revolutions, rpm  →  diff_drive_update_odom()  →  odom_state_t
 *
 * All quantities at this layer are *wheel-side* (mechanical) — no
 * motor-electrical units (ERPM, pole pairs, encoder counts).
 * Backends (motor_driver_*) own the conversion from controller-
 * native units.
 *
 * Robot parameters are compile-time constants — set per physical
 * build (chassis dimensions).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "motor_driver.h"   /* motor_wheel_cmd_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ── Robot parameters (set per build) ────────────────────────────── */

#define WHEEL_DIAMETER_M     0.17068f /* meters — ZLLG65ASM250 V3.0 (170 mm) */
#define TRACK_WIDTH_M        0.54481f /* wheel-to-wheel center, meters */
#define MAX_WHEEL_RPM        143.0f   /* wheel-side cap (≈ 1.27 m/s for 0.17 m
                                       * wheels).  Direct-drive hub motor, so
                                       * this is also the motor RPM — well under
                                       * the ZLLG65ASM250 500 rpm rated speed.
                                       * Backends may impose tighter limits. */
#define MAX_MANUAL_SPEED_MS  1.0f     /* max linear vel in manual mode (m/s) */
#define MAX_MANUAL_ANGVEL    0.67f    /* max angular vel in manual mode (rad/s);
                                       * full stick + full steer ≈ 1.20 m/s */

/* ── Types ───────────────────────────────────────────────────────── */

/** Velocity command in robot frame. */
typedef struct {
    float linear_x;    /* m/s, positive = forward */
    float angular_z;   /* rad/s, positive = CCW */
} cmd_vel_t;

/** Accumulated 2D pose and instantaneous velocity. */
typedef struct {
    /* Pose (integrated from wheel revolution deltas) */
    float x;           /* meters */
    float y;           /* meters */
    float theta;       /* radians, normalized to [-π, π] */

    /* Velocity (from instantaneous wheel RPM) */
    float linear_vel;  /* m/s */
    float angular_vel; /* rad/s */

    /* Previous wheel revolution readings for delta computation */
    float revs_left_prev;
    float revs_right_prev;
    bool  revs_initialized;
} odom_state_t;

/* ── Forward kinematics (cmd_vel → wheel RPM) ────────────────────── */

/**
 * Convert a velocity command to left/right wheel RPM.
 *
 * Applies diff-drive kinematics:
 *   v_left  = linear_x - angular_z × (track_width / 2)
 *   v_right = linear_x + angular_z × (track_width / 2)
 *
 * Then converts m/s → RPM (wheel side) and clamps to ±MAX_WHEEL_RPM.
 */
motor_wheel_cmd_t diff_drive_cmd_vel_to_wheel_rpm(const cmd_vel_t *cmd);

/* ── Inverse kinematics / odometry ──────────────────────────────── */

void diff_drive_odom_init(odom_state_t *state);

/**
 * Update odometry from new wheel-revolution and RPM readings.
 *
 * Pose is integrated from cumulative wheel-revolution deltas.
 * Velocity is computed from instantaneous wheel RPM.
 *
 * The first call after init records the revolution baseline and
 * does not update the pose.
 */
void diff_drive_update_odom(odom_state_t *state,
                            float revs_left, float revs_right,
                            float rpm_left,  float rpm_right);

/** Reset pose to origin without losing the revolution baseline. */
void diff_drive_reset_odom(odom_state_t *state);

#ifdef __cplusplus
}
#endif
