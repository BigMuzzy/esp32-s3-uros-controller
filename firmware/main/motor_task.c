/*
 * motor_task.c — Upper control loop on Core 1
 *
 * 20 ms periodic tick that arbitrates between cmd_vel (autonomous),
 * RC arcade mix (manual), and tune-mode override sources, converts
 * the chosen velocity to per-wheel RPM via diff_drive, and pushes
 * the result into the motor_driver HAL.  Reads HAL feedback to
 * update the odometry pose published to micro-ROS.
 *
 * Hardware-agnostic: this file references only diff_drive (pure
 * kinematics) and motor_driver (HAL).  It never sees ERPM, pole
 * pairs, CAN frames, or any controller-native unit.
 */

#include "motor_task.h"
#include "motor_driver.h"
#include "diff_drive.h"
#include "rc_failsafe.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "motor_task";

/* ── Shared state ───────────────────────────────────────────────── */

static portMUX_TYPE  s_cmd_vel_mux = portMUX_INITIALIZER_UNLOCKED;
static cmd_vel_t     s_cmd_vel;

static portMUX_TYPE  s_odom_mux = portMUX_INITIALIZER_UNLOCKED;
static odom_state_t  s_odom;

/* Tune-mode override (set by tune_cli, read by motor_task tick).
 * Active only while last_refresh_ms is within TUNE_OVERRIDE_TIMEOUT_MS.
 * Cleared by motor_task_clear_tune_override (zeroes last_refresh_ms). */
static portMUX_TYPE  s_tune_mux = portMUX_INITIALIZER_UNLOCKED;
static float         s_tune_left_rpm;
static float         s_tune_right_rpm;
static uint32_t      s_tune_refresh_ms;   /* 0 = inactive */

/* ── Tick body ──────────────────────────────────────────────────── */

/* Compute the wheel-RPM setpoint for this cycle, returning the
 * commanded cmd_vel that produced it (for logging only).  The
 * decision tree mirrors the legacy can_task_tx_task arbitration:
 *
 *   !armed                          → zero
 *   DRIVE_MODE_AUTONOMOUS,
 *     tune override fresh           → tune override
 *     else                          → cmd_vel from ROS
 *   DRIVE_MODE_MANUAL                → RC arcade mix (override ignored)
 *   DRIVE_MODE_FAILSAFE_STOP,
 *     tune override fresh           → tune override (bench mode)
 *     else                          → zero
 *
 * Returns true if the tune override won (drives a different debug
 * path).  cmd_out is always populated. */
static bool select_wheel_cmd(uint32_t now_ms,
                             drive_mode_t mode,
                             bool armed,
                             motor_wheel_cmd_t *cmd_out,
                             cmd_vel_t *cmd_vel_dbg)
{
    cmd_out->left_rpm  = 0.0f;
    cmd_out->right_rpm = 0.0f;
    cmd_vel_dbg->linear_x  = 0.0f;
    cmd_vel_dbg->angular_z = 0.0f;

    if (!armed) return false;

    /* Latch the tune override snapshot once for use below.  Note the
     * override is only ever populated by tune_cli, which is VESC-only
     * (see main.c), so in the default ZLAC build tune_fresh is always
     * false and the tune branches below are inert. */
    float tune_l = 0.0f, tune_r = 0.0f;
    uint32_t tune_refresh = 0;
    taskENTER_CRITICAL(&s_tune_mux);
    tune_l       = s_tune_left_rpm;
    tune_r       = s_tune_right_rpm;
    tune_refresh = s_tune_refresh_ms;
    taskEXIT_CRITICAL(&s_tune_mux);
    bool tune_fresh = (tune_refresh != 0) &&
                      ((now_ms - tune_refresh) <= TUNE_OVERRIDE_TIMEOUT_MS);

    switch (mode) {
    case DRIVE_MODE_AUTONOMOUS: {
        if (tune_fresh) {
            cmd_out->left_rpm  = tune_l;
            cmd_out->right_rpm = tune_r;
            return true;
        }
        cmd_vel_t cmd;
        taskENTER_CRITICAL(&s_cmd_vel_mux);
        cmd = s_cmd_vel;
        taskEXIT_CRITICAL(&s_cmd_vel_mux);
        *cmd_vel_dbg = cmd;
        *cmd_out = diff_drive_cmd_vel_to_wheel_rpm(&cmd);
        return false;
    }
    case DRIVE_MODE_MANUAL: {
        rc_input_t rc = rc_failsafe_read();
        cmd_vel_t cmd = rc_failsafe_arcade_mix(&rc);
        *cmd_vel_dbg = cmd;
        *cmd_out = diff_drive_cmd_vel_to_wheel_rpm(&cmd);
        return false;
    }
    case DRIVE_MODE_FAILSAFE_STOP: {
        /* Tune override is honoured in FAILSAFE_STOP so the controller
         * is usable on the bench without an RC link.  MANUAL stick
         * deflection cannot reach this branch, so the override is
         * still safe — RC always wins. */
        if (tune_fresh) {
            cmd_out->left_rpm  = tune_l;
            cmd_out->right_rpm = tune_r;
            return true;
        }
        return false;
    }
    }
    return false;
}

static void motor_task_fn(void *arg)
{
    diff_drive_odom_init(&s_odom);

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        bool armed = motor_driver_is_armed();
        drive_mode_t mode = rc_failsafe_get_mode();

        motor_wheel_cmd_t cmd;
        cmd_vel_t cmd_vel_dbg;
        bool used_tune = select_wheel_cmd(now_ms, mode, armed, &cmd, &cmd_vel_dbg);

        motor_driver_set_cmd(&cmd);

        /* Update odometry from HAL feedback. */
        motor_feedback_t fb;
        if (motor_driver_get_feedback(&fb)) {
            /* Gate velocity on feedback freshness.  get_feedback() returns
             * the last snapshot forever once any has been seen, so without
             * this a comms dropout would freeze a stale (possibly non-zero)
             * velocity into odom at the loop rate.  Stale → report zero
             * velocity; the frozen revolution counts integrate to ~0 pose
             * delta regardless, so the pose simply holds. */
            bool fb_fresh =
                (now_ms - fb.last_update_ms) <= MOTOR_FEEDBACK_TIMEOUT_MS;
            float rpm_l = fb_fresh ? fb.left.rpm  : 0.0f;
            float rpm_r = fb_fresh ? fb.right.rpm : 0.0f;

            odom_state_t odom_local;
            taskENTER_CRITICAL(&s_odom_mux);
            odom_local = s_odom;
            taskEXIT_CRITICAL(&s_odom_mux);

            diff_drive_update_odom(&odom_local,
                                   fb.left.revolutions, fb.right.revolutions,
                                   rpm_l,               rpm_r);

            taskENTER_CRITICAL(&s_odom_mux);
            s_odom = odom_local;
            taskEXIT_CRITICAL(&s_odom_mux);
        }

        /* Rate-limited debug (~1 Hz). */
        static TickType_t s_last_dbg;
        if ((xTaskGetTickCount() - s_last_dbg) >= pdMS_TO_TICKS(1000)) {
            const char *mode_str =
                (mode == DRIVE_MODE_AUTONOMOUS)    ? "AUTO" :
                (mode == DRIVE_MODE_MANUAL)        ? "MANUAL" :
                (mode == DRIVE_MODE_FAILSAFE_STOP) ? "FAILSAFE" : "?";
            ESP_LOGI(TAG, "tick: mode=%s armed=%d src=%s "
                          "cmd=(%.2f,%.2f) rpm=(%.1f,%.1f)",
                     mode_str, armed,
                     used_tune ? "TUNE" : "CMD",
                     cmd_vel_dbg.linear_x, cmd_vel_dbg.angular_z,
                     cmd.left_rpm, cmd.right_rpm);
            s_last_dbg = xTaskGetTickCount();
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────── */

esp_err_t motor_task_init(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(
        motor_task_fn, "motor_task",
        MOTOR_TASK_STACK, NULL, MOTOR_TASK_PRIO, NULL, 1);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

void motor_task_set_cmd_vel(const cmd_vel_t *cmd)
{
    taskENTER_CRITICAL(&s_cmd_vel_mux);
    s_cmd_vel = *cmd;
    taskEXIT_CRITICAL(&s_cmd_vel_mux);

    rc_failsafe_notify_cmd_vel();
}

void motor_task_get_odom(odom_state_t *odom_out)
{
    taskENTER_CRITICAL(&s_odom_mux);
    *odom_out = s_odom;
    taskEXIT_CRITICAL(&s_odom_mux);
}

void motor_task_set_tune_override(float left_rpm, float right_rpm)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (now_ms == 0) now_ms = 1;

    taskENTER_CRITICAL(&s_tune_mux);
    s_tune_left_rpm   = left_rpm;
    s_tune_right_rpm  = right_rpm;
    s_tune_refresh_ms = now_ms;
    taskEXIT_CRITICAL(&s_tune_mux);
}

void motor_task_clear_tune_override(void)
{
    taskENTER_CRITICAL(&s_tune_mux);
    s_tune_refresh_ms = 0;
    s_tune_left_rpm   = 0.0f;
    s_tune_right_rpm  = 0.0f;
    taskEXIT_CRITICAL(&s_tune_mux);
}
