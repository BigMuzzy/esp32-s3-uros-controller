/*
 * tune_transport.h — byte-stream transport interface for the tuning CLI
 *
 * The tune CLI parses lines from a transport and emits CSV/responses
 * back the same way. By isolating the transport behind this interface,
 * the same parser + experiment state machine works over UART0 today,
 * over WiFi TCP / SoftAP / BLE NUS later, or any combination.
 *
 * Conventions:
 *   - read():  non-blocking up to `timeout_ms`. Returns the number of
 *              bytes actually read (0 = no data, negative = fatal error).
 *   - write(): best-effort, may block briefly to drain TX FIFO.
 *              Returns bytes written (>= 0) or negative on fatal error.
 *   - is_connected(): for connection-oriented transports (TCP, BLE).
 *              Stream transports (UART) always return true.
 *
 * Implementations must be safe to call from a single dedicated task;
 * concurrent calls from multiple tasks are NOT required to be safe.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tune_transport {
    int  (*read)        (struct tune_transport *t,
                          uint8_t *buf, size_t len, uint32_t timeout_ms);
    int  (*write)       (struct tune_transport *t,
                          const uint8_t *buf, size_t len);
    bool (*is_connected)(struct tune_transport *t);
    void *ctx;
} tune_transport_t;

/* Concrete transport accessors.  Each transport owns its driver
 * lifecycle; calling the accessor a second time returns the same
 * singleton.  Returns NULL if init failed. */

tune_transport_t *tune_transport_uart_get(void);
tune_transport_t *tune_transport_wifi_get(void);   /* stub for now */

#ifdef __cplusplus
}
#endif
