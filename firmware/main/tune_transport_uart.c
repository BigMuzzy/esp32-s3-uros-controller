/*
 * tune_transport_uart.c — UART0 transport for the tuning CLI
 *
 * Drives the USB-UART bridge on the Waveshare ESP32-S3-RS485-CAN
 * (CP2102 wired to UART0, GPIO43 TX / GPIO44 RX). The board enumerates
 * on the host as /dev/ttyUSB0.
 *
 * ESP_LOG output and printf are rerouted through the same UART driver
 * (see uart_vfs_dev_use_driver below), so log lines and CLI replies
 * share one TX path. A host parser must filter on the CLI line
 * prefixes ("R,", "T,", "OK", "ERR,", "E,") or simply mute logs with
 * the "quiet on" CLI command during interactive sessions.
 */

#include "tune_transport.h"

#include "driver/uart.h"
#include "esp_log.h"

/* ESP-IDF v5.3+ moved the VFS helpers to driver/uart_vfs.h; older
 * trees expose the same function via esp_vfs_dev.h. */
#if __has_include("driver/uart_vfs.h")
#  include "driver/uart_vfs.h"
#  define TUNE_UART_USE_DRIVER(p) uart_vfs_dev_use_driver(p)
#else
#  include "esp_vfs_dev.h"
#  define TUNE_UART_USE_DRIVER(p) esp_vfs_dev_uart_use_driver(p)
#endif

#define UART_PORT          UART_NUM_0
#define UART_BAUD          115200
#define UART_RX_BUF_BYTES  1024
#define UART_TX_BUF_BYTES  2048

static const char *TAG = "tune_uart";
static bool s_initialized;

static int uart_xport_read(tune_transport_t *t, uint8_t *buf,
                           size_t len, uint32_t timeout_ms)
{
    (void)t;
    /* pdMS_TO_TICKS rounds down; at 100 Hz tick rate a 5 ms timeout
     * becomes 0 and uart_read_bytes returns immediately, busy-looping
     * the caller.  Floor to one tick. */
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ms > 0 && ticks == 0) ticks = 1;
    int n = uart_read_bytes(UART_PORT, buf, len, ticks);
    return (n < 0) ? -1 : n;
}

static int uart_xport_write(tune_transport_t *t,
                            const uint8_t *buf, size_t len)
{
    (void)t;
    int n = uart_write_bytes(UART_PORT, (const char *)buf, len);
    return (n < 0) ? -1 : n;
}

static bool uart_xport_is_connected(tune_transport_t *t)
{
    (void)t;
    return true;   /* UART is always "connected" */
}

static tune_transport_t s_xport = {
    .read         = uart_xport_read,
    .write        = uart_xport_write,
    .is_connected = uart_xport_is_connected,
    .ctx          = NULL,
};

tune_transport_t *tune_transport_uart_get(void)
{
    if (s_initialized) return &s_xport;

    /* UART0 is the IDF console by default.  Install the driver, then
     * route the C stdio VFS through it so ESP_LOG / printf no longer
     * race uart_write_bytes() over the same TX FIFO (the symptom is
     * tilde-soup on the serial monitor immediately after app_main
     * starts). uart_read_bytes() is what tune_cli polls for input. */
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(UART_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err));
        return NULL;
    }

    /* Default pins (TX=43, RX=44) — no remap needed. */
    err = uart_driver_install(UART_PORT,
                              UART_RX_BUF_BYTES,
                              UART_TX_BUF_BYTES,
                              0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        return NULL;
    }

    TUNE_UART_USE_DRIVER(UART_PORT);

    s_initialized = true;
    ESP_LOGI(TAG, "UART0 transport ready (%d baud)", UART_BAUD);
    return &s_xport;
}
