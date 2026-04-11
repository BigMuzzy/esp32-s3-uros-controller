/*
 * diff_drive.c — Differential-drive kinematics & odometry
 *
 * Pure math — no FreeRTOS, no hardware. Can be unit-tested on host.
 */

#include "diff_drive.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Helpers ─────────────────────────────────────────────────────── */

static inline int32_t clamp_i32(int32_t val, int32_t lo, int32_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/** Normalize angle to [-π, π]. */
static float normalize_angle(float a)
{
    while (a > M_PI)  a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

/**
 * Convert wheel linear velocity (m/s) to ERPM.
 *   rpm  = v / (π × diameter) × 60
 *   erpm = rpm × pole_pairs
 */
static int32_t vel_to_erpm(float vel_ms)
{
    float circumference = (float)M_PI * WHEEL_DIAMETER_M;
    float rpm  = (vel_ms / circumference) * 60.0f;
    float erpm = rpm * (float)MOTOR_POLE_PAIRS;
    return (int32_t)erpm;
}

/**
 * Convert ERPM to wheel linear velocity (m/s).
 *   rpm = erpm / pole_pairs
 *   v   = rpm × π × diameter / 60
 */
static float erpm_to_vel(int32_t erpm)
{
    float rpm = (float)erpm / (float)MOTOR_POLE_PAIRS;
    float circumference = (float)M_PI * WHEEL_DIAMETER_M;
    return rpm * circumference / 60.0f;
}

/**
 * Convert tachometer delta (ERPM ticks) to linear distance (meters).
 *   mech_revs = tach_delta / pole_pairs
 *   distance  = mech_revs × π × diameter
 */
static float tach_to_meters(int32_t tach_delta)
{
    float mech_revs = (float)tach_delta / (float)MOTOR_POLE_PAIRS;
    float circumference = (float)M_PI * WHEEL_DIAMETER_M;
    return mech_revs * circumference;
}

/* ── Forward kinematics ──────────────────────────────────────────── */

wheel_erpm_t diff_drive_cmd_vel_to_erpm(const cmd_vel_t *cmd)
{
    float half_track = TRACK_WIDTH_M / 2.0f;

    float v_left  = cmd->linear_x - cmd->angular_z * half_track;
    float v_right = cmd->linear_x + cmd->angular_z * half_track;

    wheel_erpm_t out;
    out.left_erpm  = clamp_i32(vel_to_erpm(v_left),  -MAX_WHEEL_ERPM, MAX_WHEEL_ERPM);
    out.right_erpm = clamp_i32(vel_to_erpm(v_right), -MAX_WHEEL_ERPM, MAX_WHEEL_ERPM);
    return out;
}

/* ── Odometry ────────────────────────────────────────────────────── */

void diff_drive_odom_init(odom_state_t *state)
{
    state->x = 0.0f;
    state->y = 0.0f;
    state->theta = 0.0f;
    state->linear_vel = 0.0f;
    state->angular_vel = 0.0f;
    state->tach_left_prev = 0;
    state->tach_right_prev = 0;
    state->tach_initialized = false;
}

void diff_drive_update_odom(odom_state_t *state,
                            int32_t tach_left, int32_t tach_right,
                            int32_t erpm_left, int32_t erpm_right)
{
    /* First call: record baseline, no pose update */
    if (!state->tach_initialized) {
        state->tach_left_prev  = tach_left;
        state->tach_right_prev = tach_right;
        state->tach_initialized = true;
        return;
    }

    /* Distance traveled by each wheel since last update */
    int32_t dt_left  = tach_left  - state->tach_left_prev;
    int32_t dt_right = tach_right - state->tach_right_prev;
    state->tach_left_prev  = tach_left;
    state->tach_right_prev = tach_right;

    float d_left  = tach_to_meters(dt_left);
    float d_right = tach_to_meters(dt_right);

    /* Diff-drive pose integration */
    float ds    = (d_right + d_left)  / 2.0f;   /* linear displacement */
    float dtheta = (d_right - d_left) / TRACK_WIDTH_M; /* angular displacement */

    state->theta = normalize_angle(state->theta + dtheta);
    state->x += ds * cosf(state->theta);
    state->y += ds * sinf(state->theta);

    /* Instantaneous velocity from ERPM */
    float v_left  = erpm_to_vel(erpm_left);
    float v_right = erpm_to_vel(erpm_right);
    state->linear_vel  = (v_right + v_left)  / 2.0f;
    state->angular_vel = (v_right - v_left) / TRACK_WIDTH_M;
}

void diff_drive_reset_odom(odom_state_t *state)
{
    state->x = 0.0f;
    state->y = 0.0f;
    state->theta = 0.0f;
    /* Keep tach baseline and velocity — only reset pose */
}
