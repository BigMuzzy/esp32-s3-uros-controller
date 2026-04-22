/*
 * can_task.c — CAN bus orchestration on Core 1
 *
 * Two tasks:
 *   can_tx_task (20 ms periodic) — mode decision + motor commands
 *   can_rx_task (event-driven)   — VESC status + odometry
 *
 * Shared data to/from Core 0 is protected by spinlocks.
 */

#include "can_task.h"
#include "rc_failsafe.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <inttypes.h>
#include <string.h>

static const char *TAG = "can_task";

/* ── TWAI GPIO (Waveshare board: CAN TX=15, RX=16) ──────────────── */

#define CAN_TX_GPIO  15
#define CAN_RX_GPIO  16

/* ── Shared state (Core 0 ↔ Core 1) ─────────────────────────────── */

static portMUX_TYPE s_cmd_vel_mux = portMUX_INITIALIZER_UNLOCKED;
static cmd_vel_t    s_cmd_vel;

static portMUX_TYPE   s_odom_mux = portMUX_INITIALIZER_UNLOCKED;
static odom_state_t   s_odom;

static portMUX_TYPE   s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static vesc_status_t  s_vesc_status[2]; /* [0]=left, [1]=right */
static vesc_status5_t s_vesc_status5[2];
static bool           s_status_valid[2];
static vesc_health_t  s_vesc_health[2];

/* Event group used during boot health check and for runtime arming.
 * Bits 0/1: STATUS   seen from LEFT/RIGHT.
 * Bits 2/3: STATUS_5 seen from LEFT/RIGHT.
 * Bits 4/5: PONG     seen from LEFT/RIGHT.
 * Bit  7:   BIT_ARMED — can_tx_task allowed to command non-zero ERPM. */
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

/* ── Odometry state (Core 1 only) ────────────────────────────────── */

static odom_state_t s_odom_local;

/* Track whether both VESCs have reported tachometer */
static bool s_tach_left_updated;
static bool s_tach_right_updated;
static int32_t s_tach_left;
static int32_t s_tach_right;
static int32_t s_erpm_left;
static int32_t s_erpm_right;

/* ── can_tx_task ─────────────────────────────────────────────────── */

/* Per-VESC runtime watchdog.  Returns true if all VESCs have reported
 * STATUS recently; updates s_vesc_health[i].online accordingly and
 * logs edge transitions. */
static bool vesc_watchdog_check(uint32_t now_ms)
{
    static bool s_was_online[2] = { true, true };
    const uint8_t ids[2] = { VESC_ID_LEFT, VESC_ID_RIGHT };
    bool all_online = true;

    for (int i = 0; i < 2; i++) {
        uint32_t last_ms;
        bool boot_online;
        float   voltage;
        taskENTER_CRITICAL(&s_status_mux);
        last_ms     = s_vesc_health[i].last_status_ms;
        boot_online = s_vesc_health[i].online;
        voltage     = s_vesc_health[i].voltage_in;
        taskEXIT_CRITICAL(&s_status_mux);

        /* Only considered online if (a) boot check passed,
         * (b) a STATUS frame arrived within the timeout window, and
         * (c) last reported V_in is inside the plausibility window. */
        bool fresh = (last_ms != 0) &&
                     ((now_ms - last_ms) <= VESC_STATUS_TIMEOUT_MS);
        bool voltage_ok = (voltage >= VESC_VOLTAGE_MIN_V) &&
                          (voltage <= VESC_VOLTAGE_MAX_V);
        bool online = boot_online && fresh && voltage_ok;

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

            /* Publish the runtime state so /vesc/health reflects the
             * watchdog verdict, not just the boot result.  Boot-time
             * `online = false` is sticky — runtime watchdog only
             * downgrades a previously-armed VESC. */
            taskENTER_CRITICAL(&s_status_mux);
            if (boot_online) {
                s_vesc_health[i].online = online;
            }
            taskEXIT_CRITICAL(&s_status_mux);
        }

        if (!online) all_online = false;
    }

    return all_online;
}

/* Read and handle TWAI bus-health alerts.  Auto-recovers from BUS_OFF
 * by calling twai_initiate_recovery() then twai_start() after a short
 * delay.  Bus errors can occur at ~50 Hz during a fault, so they are
 * rate-limited to at most one summary log per second; BUS_OFF itself
 * is always logged immediately.  Called from can_tx_task; non-blocking. */
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
            /* Wait briefly for the recovery timer (128 * 11 recessive bits
             * @ 500 kbit/s ≈ 2.8 ms).  twai_start() fails until recovery
             * completes; retry a few times. */
            for (int i = 0; i < 10; i++) {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (twai_start() == ESP_OK) {
                    ESP_LOGI(TAG, "TWAI restarted after bus-off");
                    break;
                }
            }
        }
    }

    /* Emit a rolling summary at most once per second when any rate-
     * limited alert has been seen.  Keeps faults visible without
     * drowning the log at 50 lines/s. */
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

static void can_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAN_TX_PERIOD_MS));

        twai_alert_handle();

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        bool vesc_ok = vesc_watchdog_check(now_ms);

        /* Hard gate: until the boot-time health check arms the system,
         * only transmit ERPM=0.  This keeps the bus active (so VESCs
         * that require periodic commands don't time out) while making
         * sure we never command motion against an unhealthy VESC.
         * The runtime watchdog (vesc_ok) also forces stop on any
         * VESC that has stopped broadcasting STATUS. */
        bool armed = (xEventGroupGetBits(s_can_events) & BIT_ARMED) != 0;

        drive_mode_t mode = rc_failsafe_get_mode();
        wheel_erpm_t erpm = { .left_erpm = 0, .right_erpm = 0 };

        if (armed && vesc_ok) {
            switch (mode) {
            case DRIVE_MODE_AUTONOMOUS: {
                cmd_vel_t cmd;
                taskENTER_CRITICAL(&s_cmd_vel_mux);
                cmd = s_cmd_vel;
                taskEXIT_CRITICAL(&s_cmd_vel_mux);
                erpm = diff_drive_cmd_vel_to_erpm(&cmd);
                break;
            }
            case DRIVE_MODE_MANUAL: {
                rc_input_t rc = rc_failsafe_read();
                cmd_vel_t cmd = rc_failsafe_arcade_mix(&rc);
                erpm = diff_drive_cmd_vel_to_erpm(&cmd);
                break;
            }
            case DRIVE_MODE_FAILSAFE_STOP:
                /* erpm already zeroed */
                break;
            }
        }

        /* Send ERPM commands to both VESCs. Log TX failures at most
         * once per second so a bus-off or TX-queue-full condition is
         * visible without flooding the log. */
        twai_message_t msg;
        esp_err_t tx_ret;
        static uint32_t s_tx_err_count;
        static TickType_t s_last_tx_err_log;

        vesc_can_encode_rpm(VESC_ID_LEFT, erpm.left_erpm, &msg);
        tx_ret = twai_transmit(&msg, pdMS_TO_TICKS(5));
        if (tx_ret != ESP_OK) s_tx_err_count++;

        vesc_can_encode_rpm(VESC_ID_RIGHT, erpm.right_erpm, &msg);
        tx_ret = twai_transmit(&msg, pdMS_TO_TICKS(5));
        if (tx_ret != ESP_OK) s_tx_err_count++;

        if (s_tx_err_count > 0 &&
            (xTaskGetTickCount() - s_last_tx_err_log) >= pdMS_TO_TICKS(1000)) {
            ESP_LOGW(TAG, "TWAI TX errors: %" PRIu32 " in last ~1s (latest: %s)",
                     s_tx_err_count, esp_err_to_name(tx_ret));
            s_tx_err_count = 0;
            s_last_tx_err_log = xTaskGetTickCount();
        }
    }
}

/* ── can_rx_task ─────────────────────────────────────────────────── */

static void can_rx_task(void *arg)
{
    diff_drive_odom_init(&s_odom_local);
    twai_message_t msg;

    for (;;) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) {
            continue;
        }

        uint8_t vesc_id;
        int cmd = vesc_can_get_cmd(&msg, &vesc_id);
        if (cmd < 0) continue;

        /* PONG arrives addressed to our sender ID; the responding
         * VESC's ID is in the payload, not the frame ID. Handle
         * before the VESC_ID_* filter below. */
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
                s_vesc_status[idx] = st;
                s_status_valid[idx] = true;
                s_vesc_health[idx].last_status_ms = now_ms;
                taskEXIT_CRITICAL(&s_status_mux);

                xEventGroupSetBits(s_can_events,
                    (idx == 0) ? BIT_STATUS_LEFT : BIT_STATUS_RIGHT);

                /* Cache ERPM for odom velocity */
                if (idx == 0) s_erpm_left  = st.erpm;
                else          s_erpm_right = st.erpm;
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

                /* Track tachometer updates for odometry */
                if (idx == 0) {
                    s_tach_left = st5.tachometer;
                    s_tach_left_updated = true;
                } else {
                    s_tach_right = st5.tachometer;
                    s_tach_right_updated = true;
                }

                /* Update odom when both wheels have fresh data */
                if (s_tach_left_updated && s_tach_right_updated) {
                    s_tach_left_updated  = false;
                    s_tach_right_updated = false;

                    diff_drive_update_odom(&s_odom_local,
                                           s_tach_left, s_tach_right,
                                           s_erpm_left, s_erpm_right);

                    taskENTER_CRITICAL(&s_odom_mux);
                    s_odom = s_odom_local;
                    taskEXIT_CRITICAL(&s_odom_mux);
                }
            }
            break;
        }
        default:
            break;
        }
    }
}

/* ── Initialization ──────────────────────────────────────────────── */

/* Send a single PING to each VESC and wait for both PONGs, up to
 * VESC_PING_TIMEOUT_MS. Returns the event-group bits actually seen so
 * the caller can report per-VESC results. */
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

/* Boot-time health check (ADR-0009 Stages A + B).
 *   1. Active ping/pong: authoritative liveness, independent of the
 *      VESC being configured to broadcast periodic status.
 *   2. Passive STATUS + STATUS_5 wait: confirms the VESC is actually
 *      producing the frames odometry needs, and provides voltage for
 *      the plausibility check.
 * Both must pass per VESC to set BIT_ARMED. On failure, logs which
 * check failed but returns normally — the system stays up disarmed
 * so diagnostics remain publishable. */
static void vesc_boot_health_check(void)
{
    /* ── Stage B: active ping ─────────────────────────────────── */

    ESP_LOGI(TAG, "VESC boot ping (timeout %d ms, %d retries)...",
             VESC_PING_TIMEOUT_MS, VESC_PING_RETRIES);

    EventBits_t pong_bits = 0;
    for (int attempt = 0; attempt <= VESC_PING_RETRIES; attempt++) {
        pong_bits = vesc_ping_round();
        if ((pong_bits & BIT_ALL_PONG) == BIT_ALL_PONG) break;
    }

    /* ── Stage A: passive status wait ─────────────────────────── */

    ESP_LOGI(TAG, "VESC boot status wait (timeout %d ms)...",
             VESC_HEALTH_BOOT_TIMEOUT_MS);

    EventBits_t bits = xEventGroupWaitBits(
        s_can_events, BIT_ALL_SEEN,
        pdFALSE,   /* don't clear on exit */
        pdTRUE,    /* wait for all bits   */
        pdMS_TO_TICKS(VESC_HEALTH_BOOT_TIMEOUT_MS));

    /* ── Per-VESC verdict ─────────────────────────────────────── */

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
            ESP_LOGI(TAG, "VESC %u: online (PONG ok), V_in=%.1f V",
                     ids[i], v);
        } else if (!saw_pong) {
            ESP_LOGW(TAG, "VESC %u: no PONG — not reachable on bus",
                     ids[i]);
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
        s_vesc_health[i].online = ok;
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

esp_err_t can_task_init(void)
{
    s_can_events = xEventGroupCreate();
    if (s_can_events == NULL) {
        ESP_LOGE(TAG, "EventGroup alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* Configure TWAI: 500 kbit/s, extended frames, bus-health alerts. */
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

    /* Start RX first so the health check can observe VESC broadcasts. */
    BaseType_t ok;
    ok = xTaskCreatePinnedToCore(can_rx_task, "can_rx",
             CAN_RX_TASK_STACK, NULL, CAN_RX_TASK_PRIO, NULL, 1);
    if (ok != pdPASS) return ESP_FAIL;

    /* Boot-time passive presence + voltage check (ADR-0009 Stage A). */
    vesc_boot_health_check();

    /* TX task always runs — gated internally by BIT_ARMED. */
    ok = xTaskCreatePinnedToCore(can_tx_task, "can_tx",
             CAN_TX_TASK_STACK, NULL, CAN_TX_TASK_PRIO, NULL, 1);
    if (ok != pdPASS) return ESP_FAIL;

    return ESP_OK;
}

/* ── Cross-core accessors ────────────────────────────────────────── */

void can_task_set_cmd_vel(const cmd_vel_t *cmd)
{
    taskENTER_CRITICAL(&s_cmd_vel_mux);
    s_cmd_vel = *cmd;
    taskEXIT_CRITICAL(&s_cmd_vel_mux);

    rc_failsafe_notify_cmd_vel();
}

void can_task_get_odom(odom_state_t *odom_out)
{
    taskENTER_CRITICAL(&s_odom_mux);
    *odom_out = s_odom;
    taskEXIT_CRITICAL(&s_odom_mux);
}

bool can_task_get_vesc_status(uint8_t vesc_id, vesc_status_t *status_out)
{
    int idx = -1;
    if (vesc_id == VESC_ID_LEFT)  idx = 0;
    if (vesc_id == VESC_ID_RIGHT) idx = 1;
    if (idx < 0) return false;

    taskENTER_CRITICAL(&s_status_mux);
    *status_out = s_vesc_status[idx];
    bool valid = s_status_valid[idx];
    taskEXIT_CRITICAL(&s_status_mux);

    return valid;
}

bool can_task_get_vesc_health(uint8_t vesc_id, vesc_health_t *health_out)
{
    int idx = -1;
    if (vesc_id == VESC_ID_LEFT)  idx = 0;
    if (vesc_id == VESC_ID_RIGHT) idx = 1;
    if (idx < 0) return false;

    taskENTER_CRITICAL(&s_status_mux);
    *health_out = s_vesc_health[idx];
    taskEXIT_CRITICAL(&s_status_mux);

    return true;
}
