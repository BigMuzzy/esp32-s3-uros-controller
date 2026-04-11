/*
 * uros_task.h — micro-ROS task (Core 0)
 *
 * Owns the micro-ROS agent connection (USB-CDC serial) and all
 * ROS 2 pub/sub. Runs a spin loop at ~10 ms that:
 *
 *   1. Spins the micro-ROS executor (triggers subscription callbacks)
 *   2. Reads odom from can_task_get_odom() → publishes nav_msgs/Odometry
 *   3. Reads VESC status from can_task_get_vesc_status() → publishes telemetry
 *   4. Reads drive mode from rc_failsafe_get_mode() → publishes failsafe/active
 *
 * Subscription callback:
 *   cmd_vel → can_task_set_cmd_vel()   (writes to Core 1)
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
#define UROS_TASK_PRIO        3       /* below CAN tasks */
#define UROS_SPIN_PERIOD_MS   10

#define UROS_NODE_NAME        "esp32_drive"
#define UROS_NODE_NAMESPACE   ""

/* ── Initialization ──────────────────────────────────────────────── */

/**
 * Configure USB-CDC transport and create uros_task pinned to Core 0.
 *
 * Must be called after can_task_init() and rc_failsafe_init(), since
 * the spin loop reads from their shared data structures.
 *
 * @return ESP_OK on success, or an esp_err_t on task creation failure.
 */
esp_err_t uros_task_init(void);

#ifdef __cplusplus
}
#endif
