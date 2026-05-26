/*
 * motor_driver_zlac8015d.h — public diagnostics for the ZLAC8015D backend
 *
 * The motor_driver.h HAL exposes bus-voltage + fault bits at a
 * generic level; this header adds per-axis CANopen-specific state
 * (CiA 402 state name, heartbeat freshness, last statusword) that
 * micro-ROS or the CLI may want to surface for debugging.
 *
 * Compiled only when CONFIG_MOTOR_DRIVER_ZLAC8015D is selected.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cia402.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Per-axis health snapshot.  The "axis index" is 0 = LEFT, 1 = RIGHT;
 * the underlying CANopen node id is a Kconfig setting. */
typedef struct {
    bool             heartbeat_seen;   /* ever observed */
    uint32_t         last_heartbeat_ms;
    bool             pdo_fresh;        /* TPDO within timeout */
    uint32_t         last_tpdo_ms;
    uint16_t         statusword;
    cia402_state_t   state;
    int32_t          target_velocity_motor_rpm;    /* commanded */
    int32_t          actual_velocity_motor_rpm;    /* feedback */
    int32_t          position_counts;
} zlac_axis_status_t;

bool motor_driver_zlac_get_axis_status(int axis /*0=L,1=R*/,
                                       zlac_axis_status_t *out);

#ifdef __cplusplus
}
#endif
