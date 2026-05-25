/*
 * motor_driver_vesc.h — VESC-backend-specific extensions
 *
 * The cross-backend interface is in motor_driver.h; that header
 * gives the upper control loop everything it needs.  This header
 * exposes additional read-only accessors that only make sense for
 * the VESC backend (per-VESC raw status and health, in controller-
 * native units).
 *
 * Callers that include this header become hardware-coupled — they
 * should be compiled only when CONFIG_MOTOR_DRIVER_VESC is selected
 * (tune_cli is the only such consumer today).
 *
 * NOTE: Tune-mode override is NOT in this header.  Tune override is
 * a backend-agnostic concept and lives in motor_task.h, with the
 * tuner CLI converting ERPM ↔ wheel-RPM at its own boundary.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "vesc_can.h"   /* vesc_status_t, VESC_ID_LEFT / VESC_ID_RIGHT */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Per-VESC health snapshot.  Populated by the runtime watchdog
 * inside motor_driver_vesc; read by tune_cli for the `status`
 * command and the auto-tuner gate.
 *
 *   online         — true when boot check passed AND the watchdog
 *                    has seen a STATUS frame within the timeout
 *                    AND the last reported V_in is in range.
 *   boot_passed    — sticky; true once the boot health check
 *                    succeeded.  Cleared only by reboot.
 *   voltage_in     — last decoded STATUS_5 input voltage (volts).
 *                    0 if no STATUS_5 has been received yet.
 *   fault_code     — reserved for future STATUS_6 decode.
 *   last_status_ms — esp_timer-based timestamp (ms) of the last
 *                    STATUS frame.  0 if never seen.
 */
typedef struct {
    bool     online;
    bool     boot_passed;
    float    voltage_in;
    uint8_t  fault_code;
    uint32_t last_status_ms;
} vesc_health_t;

/**
 * Read the latest raw VESC status for a given VESC ID.
 *
 * @param vesc_id     VESC_ID_LEFT or VESC_ID_RIGHT.
 * @return true if status has been received at least once.
 */
bool motor_driver_vesc_get_status(uint8_t vesc_id, vesc_status_t *status_out);

/**
 * Read the latest VESC health snapshot for a given VESC ID.
 */
bool motor_driver_vesc_get_health(uint8_t vesc_id, vesc_health_t *health_out);

#ifdef __cplusplus
}
#endif
