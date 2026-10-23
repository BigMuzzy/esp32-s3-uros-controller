/*
 * uros_task.h — micro-ROS task (Core 0)
 *
 * Owns the micro-ROS agent connection (USB-CDC serial) and all
 * ROS 2 pub/sub. Runs a spin loop at ~10 ms that:
 *
 *   1. Spins the micro-ROS executor (triggers subscription callbacks)
 *   2. Reads odom from motor_task_get_odom() -> publishes nav_msgs/Odometry
 *   3. Reads per-motor health from the motor driver HAL ->
 *      publishes battery/telemetry topics (VESC backend only)
 *   4. Reads drive mode from rc_failsafe_get_mode() -> publishes failsafe/active
 *   5. ZLAC backend: publishes chassis/telemetry (layout below)
 *
 * Subscription callback:
 *   cmd_vel → motor_task_set_cmd_vel()  (writes to Core 1)
 *   chassis/profile_ms (std_msgs/Int32, CONFIG_ZLAC_RUNTIME_PROFILE only)
 *     → motor_driver_zlac_request_profile_ms(ms, ms)
 *
 * Service:
 *   reset_odom (std_srvs/Trigger) → motor_task_reset_odom()
 *     zeroes the odom pose without a reboot (bench calibration).
 *
 * All TWAI and GPIO access is delegated to Core 1 modules.
 * This task only touches micro-ROS and shared data accessors.
 *
 * Agent lifecycle:
 *   - On startup, blocks until the micro-ROS agent is reachable.
 *   - Creates node, publishers, subscriptions, then enters spin loop.
 *   - On agent disconnect, destroys entities and retries connection.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ───────────────────────────────────────────────── */

#define UROS_TASK_STACK       8192
#define UROS_TASK_PRIO        3       /* below motor tasks */
#define UROS_SPIN_PERIOD_MS   10

#define UROS_NODE_NAME        "esp32_drive"
#define UROS_NODE_NAMESPACE   ""

/* ── chassis/telemetry layout (std_msgs/Float64MultiArray) ──────────
 *
 * Best effort, once per spin loop (about 50 Hz), only after the agent
 * clock sync.  layout.dim is empty; data[] holds, by index:
 *
 * Times are ROS epoch seconds on the agent-synced ESP32 clock; NAN means
 * "never happened".  Wheel speeds are rim speeds in m/s, forward-positive.
 *
 *   0  layout version (UROS_TELEMETRY_VERSION)
 *   1  sample time: when this message was assembled
 *   2  time the last cmd_vel was received
 *   3  number of cmd_vel messages received this boot
 *   4  last cmd_vel linear.x (m/s)        5  last cmd_vel angular.z (rad/s)
 *   6  time of the last RPDO1 (wheel targets) sent to the drive
 *   7  left target (m/s)                  8  right target (m/s)
 *   9  time of the last TPDO2 (measured velocity) received
 *  10  left measured (m/s)               11  right measured (m/s)
 *  12  time of the last TPDO3 (current) received
 *  13  left current (A)                  14  right current (A); NAN if unmapped
 *  15  flags, bit: 0 boot passed (drive configured, link up), 1 heartbeat
 *      fresh, 2 feedback fresh, 3 both axes operation enabled, 4 e-stop
 *      latched, 5 drive fault, 6 armed (accepting commands), 7 online
 *  16  drive mode (rc_failsafe: 0 autonomous, 1 failsafe stop, 2 manual)
 *  17  heartbeat age (s); NAN if never seen
 *  18  S-curve accel time 0x6083 read back, both axes equal (ms); NAN if not
 *  19  S-curve decel time 0x6084 read back, both axes equal (ms); NAN if not
 *  20  left statusword                   21  right statusword
 *  22  failed runtime profile changes (the last confirmed one is restored)
 *  23  RPDO1 frames the CAN driver refused
 */
#define UROS_TELEMETRY_VERSION  1
#define UROS_TELEMETRY_LEN      24

/* ── Initialization ──────────────────────────────────────────────── */

/**
 * Configure USB-CDC transport and create uros_task pinned to Core 0.
 *
 * Must be called after motor_driver_init(), motor_task_init() and
 * rc_failsafe_init(), since the spin loop reads from their shared
 * data structures.
 *
 * @return ESP_OK on success, or an esp_err_t on task creation failure.
 */
esp_err_t uros_task_init(void);

#ifdef __cplusplus
}
#endif
