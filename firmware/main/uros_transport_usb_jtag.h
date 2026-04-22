/*
 * uros_transport_usb_jtag.h — micro-XRCE custom transport over USB-Serial/JTAG
 *
 * Implements the four `uxrCustomTransport` callbacks on top of the
 * ESP-IDF `usb_serial_jtag` driver. The ESP32-S3 exposes USB-Serial/JTAG
 * on GPIO19 (D-) / GPIO20 (D+), which appears to the host as a CDC-ACM
 * device (e.g. /dev/ttyACM0).
 *
 * Designed to be passed to `rmw_uros_set_custom_transport(true, ...)`
 * with `framing = true` (XRCE HDLC framing, host agent in `serial` mode).
 */

#pragma once

#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

bool   uros_transport_usb_jtag_open (struct uxrCustomTransport *t);
bool   uros_transport_usb_jtag_close(struct uxrCustomTransport *t);
size_t uros_transport_usb_jtag_write(struct uxrCustomTransport *t,
                                      const uint8_t *buf, size_t len,
                                      uint8_t *err);
size_t uros_transport_usb_jtag_read (struct uxrCustomTransport *t,
                                      uint8_t *buf, size_t len,
                                      int timeout_ms, uint8_t *err);

#ifdef __cplusplus
}
#endif
