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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
static vesc_status4_t s_vesc_status4[2];
static bool           s_status_valid[2];

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

static void can_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAN_TX_PERIOD_MS));

        drive_mode_t mode = rc_failsafe_get_mode();
        wheel_erpm_t erpm = { .left_erpm = 0, .right_erpm = 0 };

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

        /* Send ERPM commands to both VESCs */
        twai_message_t msg;

        vesc_can_encode_rpm(VESC_ID_LEFT, erpm.left_erpm, &msg);
        twai_transmit(&msg, pdMS_TO_TICKS(5));

        vesc_can_encode_rpm(VESC_ID_RIGHT, erpm.right_erpm, &msg);
        twai_transmit(&msg, pdMS_TO_TICKS(5));
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

        int idx = -1;
        if (vesc_id == VESC_ID_LEFT)  idx = 0;
        if (vesc_id == VESC_ID_RIGHT) idx = 1;
        if (idx < 0) continue;

        switch (cmd) {
        case VESC_CAN_CMD_STATUS: {
            vesc_status_t st;
            if (vesc_can_decode_status(&msg, &st)) {
                taskENTER_CRITICAL(&s_status_mux);
                s_vesc_status[idx] = st;
                s_status_valid[idx] = true;
                taskEXIT_CRITICAL(&s_status_mux);

                /* Cache ERPM for odom velocity */
                if (idx == 0) s_erpm_left  = st.erpm;
                else          s_erpm_right = st.erpm;
            }
            break;
        }
        case VESC_CAN_CMD_STATUS_4: {
            vesc_status4_t st4;
            if (vesc_can_decode_status4(&msg, &st4)) {
                taskENTER_CRITICAL(&s_status_mux);
                s_vesc_status4[idx] = st4;
                taskEXIT_CRITICAL(&s_status_mux);

                /* Track tachometer updates for odometry */
                if (idx == 0) {
                    s_tach_left = st4.tachometer;
                    s_tach_left_updated = true;
                } else {
                    s_tach_right = st4.tachometer;
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

esp_err_t can_task_init(void)
{
    /* Configure TWAI: 500 kbit/s, extended frames */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
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

    /* Create tasks pinned to Core 1 */
    BaseType_t ok;
    ok = xTaskCreatePinnedToCore(can_tx_task, "can_tx",
             CAN_TX_TASK_STACK, NULL, CAN_TX_TASK_PRIO, NULL, 1);
    if (ok != pdPASS) return ESP_FAIL;

    ok = xTaskCreatePinnedToCore(can_rx_task, "can_rx",
             CAN_RX_TASK_STACK, NULL, CAN_RX_TASK_PRIO, NULL, 1);
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
