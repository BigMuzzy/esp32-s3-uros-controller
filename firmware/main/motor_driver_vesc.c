/*
 * motor_driver_vesc.c — VESC-over-CAN motor-driver backend
 *
 * Implements the motor_driver.h HAL on top of two VESC controllers
 * sharing a TWAI bus (CAN 2.0B, 500 kbit/s).  This file owns:
 *
 *   - TWAI peripheral init + bus-health alert handling (incl. BUS-OFF
 *     auto-recovery).
 *   - Boot-time health check (ADR-0009 Stages A + B): active PING/PONG
 *     plus passive STATUS + STATUS_5 wait per VESC, with input-voltage
 *     plausibility window.
 *   - Periodic motor-command TX task (CAN_TX_PERIOD_MS):
 *       * reads the latest wheel-RPM setpoint from the upper layer
 *         (motor_driver_set_cmd),
 *       * converts wheel RPM → VESC ERPM via pole pairs,
 *       * applies safety overrides: !armed / !vesc_ok / e-stop → 0,
 *       * applies the hybrid stop strategy (SET_RPM=0 above a wheel-
 *         speed threshold, SET_CURRENT_BRAKE below, sticky per wheel),
 *       * transmits SET_RPM or SET_CURRENT_BRAKE per wheel.
 *   - STATUS / STATUS_5 / PONG RX task: decodes ERPM, current, duty,
 *     tachometer, V_in; updates the shared motor_feedback_t (wheel-
 *     side units only — divides ERPM and tachometer by pole pairs)
 *     and the runtime watchdog timestamps.
 *
 * What this file does NOT do (intentional separation from the old
 * monolithic can_task.c):
 *   - No cmd_vel / RC-failsafe arbitration — the upper layer
 *     (motor_task) decides per-cycle which source drives the wheels,
 *     then calls motor_driver_set_cmd with the resulting RPM target.
 *   - No tune-mode override — tune_cli now talks to motor_task.
 *   - No pose integration — motor_task reads wheel revolutions from
 *     motor_driver_get_feedback and runs diff_drive_update_odom.
 *
 * Compiled only when CONFIG_MOTOR_DRIVER_VESC is selected; see
 * Kconfig + CMakeLists.txt.
 */

#include "motor_driver.h"
#include "motor_driver_vesc.h"
#include "vesc_can.h"

#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "motor_vesc";

/* ── Hardware configuration ──────────────────────────────────────── */

/* TWAI GPIO (Waveshare ESP32-S3 RS485-CAN board) */
#define CAN_TX_GPIO  15
#define CAN_RX_GPIO  16

/* Motor electrical-to-mechanical conversion.  Hoverboard hub motors
 * are 15-pole = 7.5 pole pairs in practice, rounded to 7 for the
 * VESC's integer field. */
#define MOTOR_POLE_PAIRS  7

/* Control-loop / tasks */
#define CAN_TX_PERIOD_MS      20
#define CAN_TX_TASK_STACK     4096
#define CAN_RX_TASK_STACK     4096
#define CAN_TX_TASK_PRIO      5    /* higher than uros_task */
#define CAN_RX_TASK_PRIO      6    /* highest — don't miss frames */

/* ── Health-check configuration (ADR-0009) ───────────────────────── */

#define VESC_HEALTH_BOOT_TIMEOUT_MS   1500
#define VESC_VOLTAGE_MIN_V            18.0f
#define VESC_VOLTAGE_MAX_V            60.0f
#define VESC_PING_TIMEOUT_MS          100
#define VESC_PING_RETRIES             2
#define VESC_STATUS_TIMEOUT_MS        200
#define VESC_BRAKE_CURRENT_MA         8000
#define VESC_BRAKE_HOLD_ERPM_MAX      100

/* ── Shared state ───────────────────────────────────────────────── */

/* Spinlocks — never blocked, safe in IRQs.  Keep critical sections
 * tiny: just memcpy a struct or update a couple of fields. */
static portMUX_TYPE   s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static vesc_status_t  s_vesc_status[2];     /* [0]=left, [1]=right */
static vesc_status5_t s_vesc_status5[2];
static bool           s_status_valid[2];
static vesc_health_t  s_vesc_health[2];

static portMUX_TYPE      s_feedback_mux = portMUX_INITIALIZER_UNLOCKED;
static motor_feedback_t  s_feedback;
static bool              s_feedback_valid;

static portMUX_TYPE      s_cmd_mux = portMUX_INITIALIZER_UNLOCKED;
static motor_wheel_cmd_t s_cmd;          /* latest setpoint from upper layer */
static bool              s_estop;        /* set by motor_driver_emergency_stop */

/* Event group used during boot health check and for runtime arming.
 * Bits mirror the legacy can_task scheme. */
#define BIT_STATUS_LEFT    (1 << 0)
#define BIT_STATUS_RIGHT   (1 << 1)
#define BIT_STATUS5_LEFT   (1 << 2)
#define BIT_STATUS5_RIGHT  (1 << 3)
#define BIT_PONG_LEFT      (1 << 4)
#define BIT_PONG_RIGHT     (1 << 5)
#define BIT_ALL_SEEN       (BIT_STATUS_LEFT | BIT_STATUS_RIGHT | \
                            BIT_STATUS5_LEFT | BIT_STATUS5_RIGHT)
#define BIT_ALL_PONG       (BIT_PONG_LEFT | BIT_PONG_RIGHT)
#define BIT_ARMED          (1 << 7)

static EventGroupHandle_t s_can_events;

/* ── Helpers ─────────────────────────────────────────────────────── */

static inline float erpm_to_wheel_rpm(int32_t erpm)
{
    return (float)erpm / (float)MOTOR_POLE_PAIRS;
}

static inline int32_t wheel_rpm_to_erpm(float rpm)
{
    /* round-half-away-from-zero */
    float scaled = rpm * (float)MOTOR_POLE_PAIRS;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

/* Tachometer ticks → wheel revolutions.
 *
 * The VESC tachometer increments by `(6 * pole_pairs)` per mechanical
 * revolution — 6 electrical commutation steps × pole-pair count.
 * For 7 pole pairs that's 42 ticks/rev. */
static inline float vesc_tach_to_wheel_revs(int32_t tach)
{
    return (float)tach / (6.0f * (float)MOTOR_POLE_PAIRS);
}

/* ── Runtime watchdog + bus-alert handling ──────────────────────── */

/* Per-VESC runtime watchdog.  Returns true if both VESCs have
 * reported STATUS recently with plausible voltage; updates
 * s_vesc_health[i].online accordingly and logs edge transitions.
 * Re-arms `online` when STATUS resumes after a transient timeout,
 * gated by the sticky boot_passed flag. */
static bool vesc_watchdog_check(uint32_t now_ms)
{
    static bool s_was_online[2] = { true, true };
    const uint8_t ids[2] = { VESC_ID_LEFT, VESC_ID_RIGHT };
    bool all_online = true;

    for (int i = 0; i < 2; i++) {
        uint32_t last_ms;
        bool boot_passed;
        float   voltage;
        taskENTER_CRITICAL(&s_status_mux);
        last_ms     = s_vesc_health[i].last_status_ms;
        boot_passed = s_vesc_health[i].boot_passed;
        voltage     = s_vesc_health[i].voltage_in;
        taskEXIT_CRITICAL(&s_status_mux);

        bool fresh = (last_ms != 0) &&
                     ((now_ms - last_ms) <= VESC_STATUS_TIMEOUT_MS);
        bool voltage_ok = (voltage >= VESC_VOLTAGE_MIN_V) &&
                          (voltage <= VESC_VOLTAGE_MAX_V);
        bool online = boot_passed && fresh && voltage_ok;

        if (online != s_was_online[i]) {
            if (online) {
                ESP_LOGW(TAG, "VESC %u: healthy again (V_in=%.1f V)",
                         ids[i], voltage);
            } else if (!fresh) {
                ESP_LOGE(TAG, "VESC %u: status timeout (last %" PRIu32
                              " ms ago) — forcing stop",
                         ids[i], now_ms - last_ms);
            } else {
                ESP_LOGE(TAG, "VESC %u: voltage %.1f V out of range "
                              "[%.1f, %.1f] — forcing stop",
                         ids[i], voltage,
                         VESC_VOLTAGE_MIN_V, VESC_VOLTAGE_MAX_V);
            }
            s_was_online[i] = online;
        }

        taskENTER_CRITICAL(&s_status_mux);
        s_vesc_health[i].online = online;
        taskEXIT_CRITICAL(&s_status_mux);

        if (!online) all_online = false;
    }

    return all_online;
}

/* Read and handle TWAI bus-health alerts.  Auto-recovers from BUS-OFF.
 * Rate-limits the non-critical summary to ≤1 line/s. */
static void twai_alert_handle(void)
{
    static uint32_t s_bus_err_count;
    static uint32_t s_err_pass_count;
    static uint32_t s_rx_full_count;
    static TickType_t s_last_summary;

    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts != 0) {
        if (alerts & TWAI_ALERT_BUS_ERROR)     s_bus_err_count++;
        if (alerts & TWAI_ALERT_ERR_PASS)      s_err_pass_count++;
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) s_rx_full_count++;

        if (alerts & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI BUS-OFF — initiating recovery");
            (void)twai_initiate_recovery();
            for (int i = 0; i < 10; i++) {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (twai_start() == ESP_OK) {
                    ESP_LOGI(TAG, "TWAI restarted after bus-off");
                    break;
                }
            }
        }
    }

    TickType_t now = xTaskGetTickCount();
    if ((s_bus_err_count || s_err_pass_count || s_rx_full_count) &&
        (now - s_last_summary) >= pdMS_TO_TICKS(1000)) {
        ESP_LOGW(TAG, "TWAI alerts in last ~1s: bus_err=%" PRIu32
                      " err_pass=%" PRIu32 " rx_full=%" PRIu32,
                 s_bus_err_count, s_err_pass_count, s_rx_full_count);
        s_bus_err_count  = 0;
        s_err_pass_count = 0;
        s_rx_full_count  = 0;
        s_last_summary   = now;
    }
}

/* ── TX task ─────────────────────────────────────────────────────── */

static void motor_vesc_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAN_TX_PERIOD_MS));

        twai_alert_handle();

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        bool vesc_ok = vesc_watchdog_check(now_ms);
        bool armed = (xEventGroupGetBits(s_can_events) & BIT_ARMED) != 0;

        /* Latch the upper-layer command + e-stop state.  When the
         * backend is disarmed, the watchdog has tripped, or e-stop
         * has been engaged, force ERPM=0 — the safety layer never
         * trusts the caller. */
        motor_wheel_cmd_t cmd_local;
        bool estop_local;
        taskENTER_CRITICAL(&s_cmd_mux);
        cmd_local   = s_cmd;
        estop_local = s_estop;
        taskEXIT_CRITICAL(&s_cmd_mux);

        int32_t erpm_left  = 0;
        int32_t erpm_right = 0;
        if (armed && vesc_ok && !estop_local) {
            erpm_left  = wheel_rpm_to_erpm(cmd_local.left_rpm);
            erpm_right = wheel_rpm_to_erpm(cmd_local.right_rpm);
        }

        /* Hybrid stop strategy (sticky per wheel).  See ADR-0014.
         *   target=0, |actual| ≥ threshold        → SET_RPM=0 (PID braking)
         *   target=0, |actual| <  threshold       → SET_CURRENT_BRAKE
         *   target=0, latched braking             → SET_CURRENT_BRAKE (sticky)
         *   target≠0                              → SET_RPM=target (clears latch)
         *
         * In the disarmed / watchdog-tripped path target_zero is false
         * (armed/vesc_ok already factored in), so we keep SET_RPM=0 —
         * the safer behaviour when a VESC has been flagged unhealthy. */
        bool target_zero = armed && vesc_ok && !estop_local &&
                           (erpm_left == 0) && (erpm_right == 0);

        int32_t actual_erpm_left  = 0;
        int32_t actual_erpm_right = 0;
        taskENTER_CRITICAL(&s_status_mux);
        actual_erpm_left  = s_vesc_status[0].erpm;
        actual_erpm_right = s_vesc_status[1].erpm;
        taskEXIT_CRITICAL(&s_status_mux);

        static bool s_braking[2];
        const int32_t target_per_side[2] = { erpm_left,  erpm_right };
        const int32_t actual_per_side[2] = { actual_erpm_left, actual_erpm_right };
        bool brake_per_side[2] = { false, false };
        for (int i = 0; i < 2; i++) {
            if (target_per_side[i] != 0 || !target_zero) {
                s_braking[i] = false;
            } else if (s_braking[i] ||
                       abs(actual_per_side[i]) < VESC_BRAKE_HOLD_ERPM_MAX) {
                s_braking[i] = true;
            }
            brake_per_side[i] = s_braking[i];
        }
        bool brake_left  = brake_per_side[0];
        bool brake_right = brake_per_side[1];

        twai_message_t msg;
        esp_err_t tx_ret;
        static uint32_t s_tx_err_count;
        static TickType_t s_last_tx_err_log;

        if (brake_left) {
            vesc_can_encode_current_brake(VESC_ID_LEFT,
                                          VESC_BRAKE_CURRENT_MA, &msg);
        } else {
            vesc_can_encode_rpm(VESC_ID_LEFT, erpm_left, &msg);
        }
        tx_ret = twai_transmit(&msg, pdMS_TO_TICKS(5));
        if (tx_ret != ESP_OK) s_tx_err_count++;

        if (brake_right) {
            vesc_can_encode_current_brake(VESC_ID_RIGHT,
                                          VESC_BRAKE_CURRENT_MA, &msg);
        } else {
            vesc_can_encode_rpm(VESC_ID_RIGHT, erpm_right, &msg);
        }
        tx_ret = twai_transmit(&msg, pdMS_TO_TICKS(5));
        if (tx_ret != ESP_OK) s_tx_err_count++;

        /* Rate-limited debug (~1 Hz). */
        static TickType_t s_last_dbg;
        if ((xTaskGetTickCount() - s_last_dbg) >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "tx: armed=%d vesc_ok=%d estop=%d "
                          "L=%s%" PRId32 "(act=%" PRId32 ") "
                          "R=%s%" PRId32 "(act=%" PRId32 ")",
                     armed, vesc_ok, estop_local,
                     brake_left  ? "BRK_mA=" : "rpm=",
                     brake_left  ? (int32_t)VESC_BRAKE_CURRENT_MA : erpm_left,
                     actual_erpm_left,
                     brake_right ? "BRK_mA=" : "rpm=",
                     brake_right ? (int32_t)VESC_BRAKE_CURRENT_MA : erpm_right,
                     actual_erpm_right);
            s_last_dbg = xTaskGetTickCount();
        }

        if (s_tx_err_count > 0 &&
            (xTaskGetTickCount() - s_last_tx_err_log) >= pdMS_TO_TICKS(1000)) {
            ESP_LOGW(TAG, "TWAI TX errors: %" PRIu32 " in last ~1s (latest: %s)",
                     s_tx_err_count, esp_err_to_name(tx_ret));
            s_tx_err_count = 0;
            s_last_tx_err_log = xTaskGetTickCount();
        }
    }
}

/* ── RX task ─────────────────────────────────────────────────────── */

/* Publish the wheel-side motor_feedback_t snapshot after a STATUS_5
 * round trip (we have fresh ERPM + tach + V_in on both sides).
 * Called only from motor_vesc_rx_task, so we can safely read the
 * VESC status caches without locking — they were just written by us. */
static void publish_feedback(uint32_t now_ms)
{
    motor_feedback_t fb;
    fb.left.rpm          = erpm_to_wheel_rpm(s_vesc_status[0].erpm);
    fb.left.revolutions  = vesc_tach_to_wheel_revs(s_vesc_status5[0].tachometer);
    fb.left.current_a    = s_vesc_status[0].current_motor;
    fb.left.fault_code_raw = 0;
    fb.left.fault_bits     = 0;

    fb.right.rpm         = erpm_to_wheel_rpm(s_vesc_status[1].erpm);
    fb.right.revolutions = vesc_tach_to_wheel_revs(s_vesc_status5[1].tachometer);
    fb.right.current_a   = s_vesc_status[1].current_motor;
    fb.right.fault_code_raw = 0;
    fb.right.fault_bits     = 0;

    /* Bus voltage: average of both VESCs (same battery in practice). */
    fb.bus_voltage_v  = 0.5f * (s_vesc_status5[0].voltage_in +
                                s_vesc_status5[1].voltage_in);
    fb.last_update_ms = now_ms;

    taskENTER_CRITICAL(&s_feedback_mux);
    s_feedback       = fb;
    s_feedback_valid = true;
    taskEXIT_CRITICAL(&s_feedback_mux);
}

static void motor_vesc_rx_task(void *arg)
{
    bool tach_left_updated = false;
    bool tach_right_updated = false;

    twai_message_t msg;

    for (;;) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) {
            continue;
        }

        uint8_t vesc_id;
        int cmd = vesc_can_get_cmd(&msg, &vesc_id);
        if (cmd < 0) continue;

        /* PONG arrives addressed to our sender ID; the responding
         * VESC's ID is in the payload. */
        if (cmd == VESC_CAN_CMD_PONG && vesc_id == VESC_CAN_SENDER_ID) {
            uint8_t responder;
            if (vesc_can_decode_pong(&msg, &responder)) {
                if (responder == VESC_ID_LEFT) {
                    xEventGroupSetBits(s_can_events, BIT_PONG_LEFT);
                } else if (responder == VESC_ID_RIGHT) {
                    xEventGroupSetBits(s_can_events, BIT_PONG_RIGHT);
                }
            }
            continue;
        }

        int idx = -1;
        if (vesc_id == VESC_ID_LEFT)  idx = 0;
        if (vesc_id == VESC_ID_RIGHT) idx = 1;
        if (idx < 0) continue;

        switch (cmd) {
        case VESC_CAN_CMD_STATUS: {
            vesc_status_t st;
            if (vesc_can_decode_status(&msg, &st)) {
                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                taskENTER_CRITICAL(&s_status_mux);
                s_vesc_status[idx]  = st;
                s_status_valid[idx] = true;
                s_vesc_health[idx].last_status_ms = now_ms;
                taskEXIT_CRITICAL(&s_status_mux);

                xEventGroupSetBits(s_can_events,
                    (idx == 0) ? BIT_STATUS_LEFT : BIT_STATUS_RIGHT);
            }
            break;
        }
        case VESC_CAN_CMD_STATUS_5: {
            vesc_status5_t st5;
            if (vesc_can_decode_status5(&msg, &st5)) {
                taskENTER_CRITICAL(&s_status_mux);
                s_vesc_status5[idx] = st5;
                s_vesc_health[idx].voltage_in = st5.voltage_in;
                taskEXIT_CRITICAL(&s_status_mux);

                xEventGroupSetBits(s_can_events,
                    (idx == 0) ? BIT_STATUS5_LEFT : BIT_STATUS5_RIGHT);

                if (idx == 0) tach_left_updated  = true;
                else          tach_right_updated = true;

                /* Publish feedback once both wheels have fresh
                 * tach + voltage.  Wheel ERPM is taken from the
                 * most recent STATUS frame (s_vesc_status[].erpm). */
                if (tach_left_updated && tach_right_updated) {
                    tach_left_updated  = false;
                    tach_right_updated = false;
                    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                    publish_feedback(now_ms);
                }
            }
            break;
        }
        default:
            break;
        }
    }
}

/* ── Boot health check ──────────────────────────────────────────── */

static EventBits_t vesc_ping_round(void)
{
    twai_message_t msg;

    xEventGroupClearBits(s_can_events, BIT_ALL_PONG);

    vesc_can_encode_ping(VESC_ID_LEFT,  VESC_CAN_SENDER_ID, &msg);
    (void)twai_transmit(&msg, pdMS_TO_TICKS(5));

    vesc_can_encode_ping(VESC_ID_RIGHT, VESC_CAN_SENDER_ID, &msg);
    (void)twai_transmit(&msg, pdMS_TO_TICKS(5));

    return xEventGroupWaitBits(s_can_events, BIT_ALL_PONG,
        pdFALSE, pdTRUE, pdMS_TO_TICKS(VESC_PING_TIMEOUT_MS));
}

static void vesc_boot_health_check(void)
{
    ESP_LOGI(TAG, "VESC boot ping (timeout %d ms, %d retries)...",
             VESC_PING_TIMEOUT_MS, VESC_PING_RETRIES);

    EventBits_t pong_bits = 0;
    for (int attempt = 0; attempt <= VESC_PING_RETRIES; attempt++) {
        pong_bits = vesc_ping_round();
        if ((pong_bits & BIT_ALL_PONG) == BIT_ALL_PONG) break;
    }

    ESP_LOGI(TAG, "VESC boot status wait (timeout %d ms)...",
             VESC_HEALTH_BOOT_TIMEOUT_MS);

    EventBits_t bits = xEventGroupWaitBits(
        s_can_events, BIT_ALL_SEEN,
        pdFALSE, pdTRUE,
        pdMS_TO_TICKS(VESC_HEALTH_BOOT_TIMEOUT_MS));

    const uint8_t ids[2] = { VESC_ID_LEFT, VESC_ID_RIGHT };
    const EventBits_t want_pong[2]    = { BIT_PONG_LEFT,    BIT_PONG_RIGHT };
    const EventBits_t want_status[2]  = { BIT_STATUS_LEFT,  BIT_STATUS_RIGHT };
    const EventBits_t want_status5[2] = { BIT_STATUS5_LEFT, BIT_STATUS5_RIGHT };

    bool all_ok = true;
    for (int i = 0; i < 2; i++) {
        bool saw_pong    = (pong_bits & want_pong[i])    != 0;
        bool saw_status  = (bits      & want_status[i])  != 0;
        bool saw_status5 = (bits      & want_status5[i]) != 0;

        float v = 0.0f;
        taskENTER_CRITICAL(&s_status_mux);
        v = s_vesc_health[i].voltage_in;
        taskEXIT_CRITICAL(&s_status_mux);

        bool voltage_ok = saw_status5 &&
                          v >= VESC_VOLTAGE_MIN_V &&
                          v <= VESC_VOLTAGE_MAX_V;

        bool ok = saw_pong && saw_status && saw_status5 && voltage_ok;
        if (ok) {
            ESP_LOGI(TAG, "VESC %u: online (PONG ok), V_in=%.1f V", ids[i], v);
        } else if (!saw_pong) {
            ESP_LOGW(TAG, "VESC %u: no PONG — not reachable on bus", ids[i]);
        } else if (!saw_status && !saw_status5) {
            ESP_LOGW(TAG, "VESC %u: PONG ok but no STATUS broadcasts — "
                          "enable 'Send CAN status = STATUS_1_2_3_4_5' in VESC Tool",
                     ids[i]);
        } else if (!voltage_ok && saw_status5) {
            ESP_LOGW(TAG, "VESC %u: voltage %.1f V out of range [%.1f, %.1f]",
                     ids[i], v, VESC_VOLTAGE_MIN_V, VESC_VOLTAGE_MAX_V);
        } else {
            ESP_LOGW(TAG, "VESC %u: partial status (PONG=%d STATUS=%d STATUS_5=%d)",
                     ids[i], saw_pong, saw_status, saw_status5);
        }

        taskENTER_CRITICAL(&s_status_mux);
        s_vesc_health[i].online      = ok;
        s_vesc_health[i].boot_passed = ok;
        taskEXIT_CRITICAL(&s_status_mux);

        if (!ok) all_ok = false;
    }

    if (all_ok) {
        xEventGroupSetBits(s_can_events, BIT_ARMED);
        ESP_LOGI(TAG, "VESC health check passed; motor commands armed");
    } else {
        ESP_LOGE(TAG, "VESC health check failed; motor commands disarmed "
                      "(ERPM=0 will be transmitted). Fix wiring/config "
                      "and reboot.");
    }
}

/* ── HAL implementation ─────────────────────────────────────────── */

esp_err_t motor_driver_init(void)
{
    s_can_events = xEventGroupCreate();
    if (s_can_events == NULL) {
        ESP_LOGE(TAG, "EventGroup alloc failed");
        return ESP_ERR_NO_MEM;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.alerts_enabled = TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF |
                              TWAI_ALERT_ERR_PASS  | TWAI_ALERT_TX_FAILED |
                              TWAI_ALERT_RX_QUEUE_FULL;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "TWAI started (500 kbit/s, TX=%d, RX=%d)",
             CAN_TX_GPIO, CAN_RX_GPIO);

    /* RX first so the health check can observe VESC broadcasts. */
    BaseType_t ok;
    ok = xTaskCreatePinnedToCore(motor_vesc_rx_task, "motor_vesc_rx",
             CAN_RX_TASK_STACK, NULL, CAN_RX_TASK_PRIO, NULL, 1);
    if (ok != pdPASS) return ESP_FAIL;

    vesc_boot_health_check();

    ok = xTaskCreatePinnedToCore(motor_vesc_tx_task, "motor_vesc_tx",
             CAN_TX_TASK_STACK, NULL, CAN_TX_TASK_PRIO, NULL, 1);
    if (ok != pdPASS) return ESP_FAIL;

    return ESP_OK;
}

bool motor_driver_is_armed(void)
{
    if (s_can_events == NULL) return false;
    return (xEventGroupGetBits(s_can_events) & BIT_ARMED) != 0;
}

void motor_driver_set_cmd(const motor_wheel_cmd_t *cmd)
{
    if (cmd == NULL) return;
    taskENTER_CRITICAL(&s_cmd_mux);
    s_cmd = *cmd;
    /* Any explicit non-emergency command clears a latched e-stop only
     * when the caller asks for zero motion AND we are still tripped —
     * keep e-stop sticky otherwise.  Today the upper layer never
     * "un-stops" implicitly: it would clear via motor_driver_set_cmd
     * with zero and the next non-zero command lifts the latch.  Match
     * the legacy behavior: e-stop is one-shot — clears on next set_cmd. */
    s_estop = false;
    taskEXIT_CRITICAL(&s_cmd_mux);
}

void motor_driver_emergency_stop(void)
{
    taskENTER_CRITICAL(&s_cmd_mux);
    s_cmd.left_rpm  = 0.0f;
    s_cmd.right_rpm = 0.0f;
    s_estop = true;
    taskEXIT_CRITICAL(&s_cmd_mux);
}

bool motor_driver_get_feedback(motor_feedback_t *fb_out)
{
    if (fb_out == NULL) return false;
    taskENTER_CRITICAL(&s_feedback_mux);
    *fb_out = s_feedback;
    bool valid = s_feedback_valid;
    taskEXIT_CRITICAL(&s_feedback_mux);
    if (!valid) {
        memset(fb_out, 0, sizeof(*fb_out));
    }
    return valid;
}

bool motor_driver_get_health(motor_health_t *health_out)
{
    if (health_out == NULL) return false;

    vesc_health_t h[2];
    taskENTER_CRITICAL(&s_status_mux);
    h[0] = s_vesc_health[0];
    h[1] = s_vesc_health[1];
    taskEXIT_CRITICAL(&s_status_mux);

    health_out->online      = h[0].online      && h[1].online;
    health_out->boot_passed = h[0].boot_passed && h[1].boot_passed;
    health_out->fault_bits  = 0;  /* VESC fault decode is TODO */
    return true;
}

/* ── VESC-specific extensions ───────────────────────────────────── */

bool motor_driver_vesc_get_status(uint8_t vesc_id, vesc_status_t *status_out)
{
    int idx = -1;
    if (vesc_id == VESC_ID_LEFT)  idx = 0;
    if (vesc_id == VESC_ID_RIGHT) idx = 1;
    if (idx < 0 || status_out == NULL) return false;

    taskENTER_CRITICAL(&s_status_mux);
    *status_out = s_vesc_status[idx];
    bool valid = s_status_valid[idx];
    taskEXIT_CRITICAL(&s_status_mux);
    return valid;
}

bool motor_driver_vesc_get_health(uint8_t vesc_id, vesc_health_t *health_out)
{
    int idx = -1;
    if (vesc_id == VESC_ID_LEFT)  idx = 0;
    if (vesc_id == VESC_ID_RIGHT) idx = 1;
    if (idx < 0 || health_out == NULL) return false;

    taskENTER_CRITICAL(&s_status_mux);
    *health_out = s_vesc_health[idx];
    taskEXIT_CRITICAL(&s_status_mux);
    return true;
}
