#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "uros_driver";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 micro-ROS VESC Driver starting...");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
