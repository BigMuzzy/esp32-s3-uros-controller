/*
 * main.c — app_main entry point
 *
 * Initialization order matters:
 *   1. rc_failsafe_init()      — MCPWM capture (no dependencies)
 *   2. motor_driver_init()     — backend transport + boot health check
 *   3. motor_task_init()       — Core 1 control loop (uses HAL + failsafe)
 *   4. uros_task_init()        — Core 0 task (reads from motor_task + failsafe)
 *   5. tune_cli_init()         — transport-abstracted tuning CLI
 *
 * After init, app_main has nothing left to do — all work happens
 * in the FreeRTOS tasks. app_main returns (FreeRTOS idle task
 * reclaims its stack).
 */

#include "esp_log.h"
#include "sdkconfig.h"
#include "rc_failsafe.h"
#include "motor_driver.h"
#include "motor_task.h"
#include "uros_task.h"
#ifdef CONFIG_MOTOR_DRIVER_VESC
#include "tune_cli.h"
#endif

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 diff-drive controller starting...");

    esp_err_t ret;

    /* 1. RC failsafe — MCPWM capture, no task dependencies */
    ret = rc_failsafe_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "rc_failsafe_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "RC failsafe initialized");

    /* 2. Motor driver backend — transport init + boot health check.
     * Brings up TWAI (VESC backend) and starts whatever background TX/RX
     * tasks the backend needs.  Stays running disarmed on failure so
     * diagnostics keep publishing. */
    ret = motor_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "motor_driver_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Motor driver backend started");

    /* 3. Upper control loop — cmd_vel / RC / tune arbitration. */
    ret = motor_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "motor_task_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "motor_task started on Core 1");

    /* 4. micro-ROS — USB-CDC transport + Core 0 spin task */
    ret = uros_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uros_task_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "micro-ROS task started on Core 0");

    /* 5. Tuning CLI — VESC-specific (operates in ERPM, talks to the
     * VESC speed PID).  Only built when the VESC backend is selected.
     * Non-fatal if it fails to start — the rest of the controller
     * keeps working. */
#ifdef CONFIG_MOTOR_DRIVER_VESC
    ret = tune_cli_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "tune_cli_init failed: %s (continuing)",
                 esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "tune_cli started");
    }
#endif

    /* app_main returns — FreeRTOS tasks run independently */
    ESP_LOGI(TAG, "All tasks running");
}
