/*
 * main.c — app_main entry point
 *
 * Initialization order matters:
 *   1. rc_failsafe_init()  — MCPWM capture (no dependencies)
 *   2. can_task_init()     — TWAI + Core 1 tasks (queries failsafe)
 *   3. uros_task_init()    — Core 0 task (reads from can_task + failsafe)
 *
 * After init, app_main has nothing left to do — all work happens
 * in the FreeRTOS tasks. app_main returns (FreeRTOS idle task
 * reclaims its stack).
 */

#include "esp_log.h"
#include "rc_failsafe.h"
#include "can_task.h"
#include "uros_task.h"

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

    /* 2. CAN bus — TWAI peripheral + Core 1 TX/RX tasks */
    ret = can_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "can_task_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "CAN tasks started on Core 1");

    /* 3. micro-ROS — USB-CDC transport + Core 0 spin task */
    ret = uros_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uros_task_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "micro-ROS task started on Core 0");

    /* app_main returns — FreeRTOS tasks run independently */
    ESP_LOGI(TAG, "All tasks running");
}
