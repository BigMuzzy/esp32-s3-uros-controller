/*
 * uros_transport_usb_jtag.c — custom micro-XRCE transport over USB-Serial/JTAG
 *
 * On ESP32-S3, USB-Serial/JTAG is an independent peripheral (not UART,
 * not TinyUSB) hardwired to GPIO19/20. The ESP-IDF `usb_serial_jtag`
 * driver exposes a byte-oriented read/write API to the CDC-ACM endpoint.
 *
 * The four callbacks below plug straight into the micro-XRCE custom
 * transport API. XRCE framing is enabled at registration time (see
 * uros_task.c), so on the host the micro-ROS agent is launched in
 * `serial` mode.
 */

#include "uros_transport_usb_jtag.h"

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "uros_usb_jtag";

#define RX_BUF_SIZE  1024
#define TX_BUF_SIZE  1024

static bool s_driver_installed;

bool uros_transport_usb_jtag_open(struct uxrCustomTransport *t)
{
    (void)t;
    if (s_driver_installed) {
        return true;
    }

    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = RX_BUF_SIZE,
        .tx_buffer_size = TX_BUF_SIZE,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "driver install failed: %s", esp_err_to_name(err));
        return false;
    }
    s_driver_installed = true;
    ESP_LOGI(TAG, "USB-Serial/JTAG transport ready");
    return true;
}

bool uros_transport_usb_jtag_close(struct uxrCustomTransport *t)
{
    (void)t;
    /* Leave the driver installed across agent reconnects. There is no
     * supported uninstall path, and reopening would just reinstall. */
    return true;
}

size_t uros_transport_usb_jtag_write(struct uxrCustomTransport *t,
                                      const uint8_t *buf, size_t len,
                                      uint8_t *err)
{
    (void)t;
    (void)err;
    int written = usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(50));
    return (written < 0) ? 0 : (size_t)written;
}

size_t uros_transport_usb_jtag_read(struct uxrCustomTransport *t,
                                     uint8_t *buf, size_t len,
                                     int timeout_ms, uint8_t *err)
{
    (void)t;
    (void)err;
    int read = usb_serial_jtag_read_bytes(buf, len,
                                           pdMS_TO_TICKS(timeout_ms));
    return (read < 0) ? 0 : (size_t)read;
}
