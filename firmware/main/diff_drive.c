/*
 * diff_drive.c — Differential-drive kinematics & odometry
 *
 * Pure math — no FreeRTOS, no hardware. Can be unit-tested on host.
 * All quantities at this layer are wheel-side (m/s, RPM, revolutions).
 */

#include "diff_drive.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Helpers ─────────────────────────────────────────────────────── */

static inline float clamp_f(float val, float lo, float hi)
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
 * Convert wheel linear velocity (m/s) to wheel RPM.
 *   rpm = v / (π × diameter) × 60
 */
static float vel_to_rpm(float vel_ms)
{
    float circumference = (float)M_PI * WHEEL_DIAMETER_M;
    return (vel_ms / circumference) * 60.0f;
}

/**
 * Convert wheel RPM to linear velocity (m/s).
 *   v = rpm × π × diameter / 60
 */
static float rpm_to_vel(float rpm)
{
    float circumference = (float)M_PI * WHEEL_DIAMETER_M;
    return rpm * circumference / 60.0f;
}

/**
 * Convert wheel revolution delta to linear distance (meters).
 *   distance = revs × π × diameter
 */
static float revs_to_meters(float revs_delta)
{
    float circumference = (float)M_PI * WHEEL_DIAMETER_M;
    return revs_delta * circumference;
}

/* ── Forward kinematics ──────────────────────────────────────────── */

motor_wheel_cmd_t diff_drive_cmd_vel_to_wheel_rpm(const cmd_vel_t *cmd)
{
    float half_track = TRACK_WIDTH_M / 2.0f;

    float v_left  = cmd->linear_x - cmd->angular_z * half_track;
    float v_right = cmd->linear_x + cmd->angular_z * half_track;

    motor_wheel_cmd_t out;
    out.left_rpm  = clamp_f(vel_to_rpm(v_left),  -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
    out.right_rpm = clamp_f(vel_to_rpm(v_right), -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
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
    state->revs_left_prev = 0.0f;
    state->revs_right_prev = 0.0f;
    state->revs_initialized = false;
}

void diff_drive_update_odom(odom_state_t *state,
                            float revs_left, float revs_right,
                            float rpm_left,  float rpm_right)
{
    /* First call: record baseline, no pose update */
    if (!state->revs_initialized) {
        state->revs_left_prev  = revs_left;
        state->revs_right_prev = revs_right;
        state->revs_initialized = true;
        return;
    }

    /* Distance traveled by each wheel since last update */
    float d_revs_left  = revs_left  - state->revs_left_prev;
    float d_revs_right = revs_right - state->revs_right_prev;
    state->revs_left_prev  = revs_left;
    state->revs_right_prev = revs_right;

    float d_left  = revs_to_meters(d_revs_left);
    float d_right = revs_to_meters(d_revs_right);

    /* Diff-drive pose integration */
    float ds     = (d_right + d_left)  / 2.0f;           /* linear displacement */
    float dtheta = (d_right - d_left) / TRACK_WIDTH_M;   /* angular displacement */

    state->theta = normalize_angle(state->theta + dtheta);
    state->x += ds * cosf(state->theta);
    state->y += ds * sinf(state->theta);

    /* Instantaneous velocity from wheel RPM */
    float v_left  = rpm_to_vel(rpm_left);
    float v_right = rpm_to_vel(rpm_right);
    state->linear_vel  = (v_right + v_left)  / 2.0f;
    state->angular_vel = (v_right - v_left) / TRACK_WIDTH_M;
}

void diff_drive_reset_odom(odom_state_t *state)
{
    state->x = 0.0f;
    state->y = 0.0f;
    state->theta = 0.0f;
    /* Keep revolution baseline and velocity — only reset pose */
}
