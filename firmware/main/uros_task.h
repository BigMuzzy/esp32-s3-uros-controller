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
 *
 * Subscription callback:
 *   cmd_vel → motor_task_set_cmd_vel()  (writes to Core 1)
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
