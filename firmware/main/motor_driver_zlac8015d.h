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

/* Command-path telemetry: what was sent to the drive, what it measured
 * and its readiness, each value with the esp_timer time (us) it was
 * produced.  Index 0 = LEFT, 1 = RIGHT.  Wheel-side, forward-positive
 * rpm (axis inversion already undone).  A time of 0 means "never". */
typedef struct {
    int64_t  rpdo_us;               /* last RPDO1 (target velocity) sent  */
    float    target_wheel_rpm[2];   /* targets carried by that RPDO1      */
    int64_t  velocity_us;           /* last TPDO2 (0x606C) received      */
    float    actual_wheel_rpm[2];
    int64_t  current_us;            /* last TPDO3 (0x6077) received      */
    float    current_a[2];          /* NAN while TPDO3 is not mapped      */
    uint16_t statusword[2];
    bool     boot_passed;           /* bring-up complete, link up         */
    bool     heartbeat_fresh;
    bool     tpdo_fresh;            /* both axes within the TPDO timeout  */
    bool     operation_enabled;     /* both axes in CiA 402 OE            */
    bool     estop;                 /* latched emergency stop             */
    bool     fault;                 /* statusword fault bit on any axis   */
    uint32_t heartbeat_age_ms;      /* since the last heartbeat; UINT32_MAX = never */
    uint32_t accel_ms;              /* 0x6083 read back, both axes equal; UINT32_MAX = unknown */
    uint32_t decel_ms;              /* 0x6084 read back, both axes equal; UINT32_MAX = unknown */
    uint32_t profile_errors;        /* failed runtime profile changes     */
    uint32_t rpdo_errors;           /* RPDO1 frames the CAN driver refused */
} zlac_telemetry_t;

bool motor_driver_zlac_get_telemetry(zlac_telemetry_t *out);

/* Request new S-curve accel/decel times (ms, objects 0x6083/0x6084, both
 * axes).  Applied by SDO from the drive's TX task once both wheels are
 * commanded zero, and kept for later re-connections; a reboot returns to
 * the Kconfig values.  If the axes do not read back equal, the last
 * confirmed times are restored.  Values are clamped to 0..32767. */
void motor_driver_zlac_request_profile_ms(uint32_t accel_ms, uint32_t decel_ms);

#ifdef __cplusplus
}
#endif
