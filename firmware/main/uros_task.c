/*
 * uros_task.c — micro-ROS spin task on Core 0
 *
 * Lifecycle:
 *   1. Configure USB-CDC transport
 *   2. Wait for micro-ROS agent
 *   3. Create node, publishers, subscriptions; sync session clock
 *   4. Spin loop: executor spin + publish odom/status/failsafe,
 *      periodic clock re-sync
 *   5. On agent disconnect: destroy entities, go to step 2
 *
 * Time: the ESP32 has no RTC.  micro-ROS runs an NTP-style handshake
 * with the agent (rmw_uros_sync_session) and derives the offset from
 * esp_timer's monotonic clock to the ROS epoch.  Odometry is stamped
 * with rmw_uros_epoch_nanos(); publishing is gated until the first
 * sync so consumers never see a zero-stamped frame.
 *
 * Depends on micro_ros_espidf_component — will not compile until
 * that component is added to the project.
 */

#include "uros_task.h"
#include "uros_transport_usb_jtag.h"
#include "sdkconfig.h"
#include "motor_task.h"
#ifdef CONFIG_MOTOR_DRIVER_VESC
#include "motor_driver_vesc.h"   /* VESC-backend-specific battery health */
#endif
#include "rc_failsafe.h"
#include "diff_drive.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/trigger.h>
#include <builtin_interfaces/msg/time.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <math.h>

static const char *TAG = "uros_task";

/* ── Helpers ─────────────────────────────────────────────────────── */

#define RCCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGE(TAG, "RCCHECK fail: %s line %d: %d", __FILE__, __LINE__, (int)rc); \
        return rc; \
    } \
} while(0)

/** Build a quaternion from yaw (rotation about Z). */
static void yaw_to_quaternion(float yaw,
                               geometry_msgs__msg__Quaternion *q)
{
    q->x = 0.0;
    q->y = 0.0;
    q->z = sin(yaw / 2.0);
    q->w = cos(yaw / 2.0);
}

/* ── Message instances ───────────────────────────────────────────── */

static geometry_msgs__msg__Twist  s_cmd_vel_msg;
static nav_msgs__msg__Odometry    s_odom_msg;
static std_msgs__msg__Bool        s_failsafe_msg;
static std_srvs__srv__Trigger_Request  s_reset_req;
static std_srvs__srv__Trigger_Response s_reset_res;
#ifdef CONFIG_MOTOR_DRIVER_VESC
static sensor_msgs__msg__BatteryState s_battery_msg[2]; /* [0]=LEFT, [1]=RIGHT */
#endif

/* ── cmd_vel subscription callback ───────────────────────────────── */

static void cmd_vel_cb(const void *msg_in)
{
    const geometry_msgs__msg__Twist *twist =
        (const geometry_msgs__msg__Twist *)msg_in;

    cmd_vel_t cmd = {
        .linear_x  = (float)twist->linear.x,
        .angular_z = (float)twist->angular.z,
    };

    /* Edge-triggered debug: log only when the commanded Twist changes
     * versus the previous callback.  Steady streams (e.g. teleop or
     * Nav2 publishing the same value at 20 Hz) stay quiet; every
     * transition — including hidden zero-Twist injections from a
     * second publisher — produces one log line. */
    static float    s_prev_lin = 0.0f;
    static float    s_prev_ang = 0.0f;
    static bool     s_have_prev;
    static uint32_t s_same_count;
    if (!s_have_prev ||
        cmd.linear_x != s_prev_lin || cmd.angular_z != s_prev_ang) {
        ESP_LOGI(TAG, "cmd_vel rx: lin=%.3f ang=%.3f (prev held %lu msgs)",
                 cmd.linear_x, cmd.angular_z, (unsigned long)s_same_count);
        s_prev_lin   = cmd.linear_x;
        s_prev_ang   = cmd.angular_z;
        s_have_prev  = true;
        s_same_count = 0;
    } else {
        s_same_count++;
    }

    motor_task_set_cmd_vel(&cmd);
}

/* ── reset_odom service callback ───────────────────────────── */

/* std_srvs/Trigger handler: zero the odom pose without a reboot.  Used
 * by bench odometry-closure calibration (roadmap M1) so each run can
 * re-zero between trials.  Pose-only — motor commands are untouched. */
static void reset_odom_cb(const void *req_in, void *res_out)
{
    (void)req_in;
    std_srvs__srv__Trigger_Response *res =
        (std_srvs__srv__Trigger_Response *)res_out;

    motor_task_reset_odom();

    /* Static literal — outlives the response serialization; rclc does
     * not own/free this String. */
    static const char ok_msg[] = "odom pose reset to origin";
    res->success          = true;
    res->message.data     = (char *)ok_msg;
    res->message.size     = sizeof(ok_msg) - 1;
    res->message.capacity = sizeof(ok_msg);

    ESP_LOGI(TAG, "reset_odom: pose zeroed");
}

/* ── Publish helpers ─────────────────────────────────────────────── */

/* Fill a ROS Time from the agent-synced epoch.  Cheap at 100 Hz —
 * rmw_uros_epoch_nanos() is just esp_timer + the stored offset. */
static void stamp_now(builtin_interfaces__msg__Time *stamp)
{
    if (rmw_uros_epoch_synchronized()) {
        int64_t ns = rmw_uros_epoch_nanos();
        stamp->sec     = (int32_t)(ns / 1000000000LL);
        stamp->nanosec = (uint32_t)(ns % 1000000000LL);
    } else {
        stamp->sec     = 0;
        stamp->nanosec = 0;
    }
}

static void publish_odom(rcl_publisher_t *pub)
{
    /* Gate: don't emit a zero-stamped frame before the first agent
     * time sync.  tf2 / the host EKF reject sec=0 messages, and the
     * robot is not meaningfully moving in those first few hundred ms. */
    if (!rmw_uros_epoch_synchronized()) {
        return;
    }

    odom_state_t odom;
    motor_task_get_odom(&odom);

    /* Header — frame_id / child_frame_id and covariance are set once
     * at init; stamp every publish from the synced clock. */
    stamp_now(&s_odom_msg.header.stamp);

    /* Pose */
    s_odom_msg.pose.pose.position.x = odom.x;
    s_odom_msg.pose.pose.position.y = odom.y;
    s_odom_msg.pose.pose.position.z = 0.0;
    yaw_to_quaternion(odom.theta, &s_odom_msg.pose.pose.orientation);

    /* Twist */
    s_odom_msg.twist.twist.linear.x  = odom.linear_vel;
    s_odom_msg.twist.twist.angular.z = odom.angular_vel;

    rcl_publish(pub, &s_odom_msg, NULL);
}

static void publish_failsafe(rcl_publisher_t *pub)
{
    drive_mode_t mode = rc_failsafe_get_mode();
    s_failsafe_msg.data = (mode != DRIVE_MODE_AUTONOMOUS);
    rcl_publish(pub, &s_failsafe_msg, NULL);
}

/* Per-VESC health → sensor_msgs/BatteryState.
 * VESC-specific path: queries motor_driver_vesc directly for the per-
 * controller voltage.  Backend-agnostic battery publishing (using
 * motor_feedback_t.bus_voltage_v) is a later-phase item. */
#ifdef CONFIG_MOTOR_DRIVER_VESC
static void publish_battery(rcl_publisher_t *pub, uint8_t vesc_id, int idx)
{
    vesc_health_t h;
    if (!motor_driver_vesc_get_health(vesc_id, &h)) return;

    sensor_msgs__msg__BatteryState *m = &s_battery_msg[idx];
    m->voltage  = h.voltage_in;
    m->present  = h.online;
    /* Not measured by VESC protocol at this layer: */
    m->temperature   = NAN;
    m->current       = NAN;
    m->charge        = NAN;
    m->capacity      = NAN;
    m->design_capacity = NAN;
    m->percentage    = NAN;
    m->power_supply_status     = 0; /* UNKNOWN */
    m->power_supply_health     = 0; /* UNKNOWN */
    m->power_supply_technology = 0; /* UNKNOWN */

    rcl_publish(pub, m, NULL);
}
#endif /* CONFIG_MOTOR_DRIVER_VESC */

/* ── Main task ───────────────────────────────────────────────────── */

static void uros_task_fn(void *arg)
{
    /* Allocator */
    rcl_allocator_t allocator = rcl_get_default_allocator();

    for (;;) {
        /* ── Wait for agent ─────────────────────────────────────── */
        ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
        while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGI(TAG, "Agent connected");

        /* ── Init support & node ────────────────────────────────── */
        rclc_support_t support;
        rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
        if (rc != RCL_RET_OK) {
            ESP_LOGE(TAG, "rclc_support_init failed: %d", (int)rc);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        rcl_node_t node;
        rc = rclc_node_init_default(&node, UROS_NODE_NAME,
                                     UROS_NODE_NAMESPACE, &support);
        if (rc != RCL_RET_OK) {
            ESP_LOGE(TAG, "node init failed: %d", (int)rc);
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        /* ── Time sync ──────────────────────────────────────────── */
        /* NTP-style handshake with the agent.  No RTC needed — computes
         * the offset from esp_timer's monotonic clock to the ROS epoch.
         * Non-fatal: the spin loop re-syncs periodically, and odom
         * publishing is gated until the first sync succeeds. */
        if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
            ESP_LOGW(TAG, "initial time sync failed; will retry in spin loop");
        } else {
            ESP_LOGI(TAG, "session clock synced to agent");
        }

        /* ── Publishers ─────────────────────────────────────────── */
        /* Each entity is created independently.  A failed init is logged
         * and that entity is then skipped — never published to, never
         * fini'd — instead of being used uninitialised.  The subscription
         * + executor are session-critical: if either fails we skip the
         * spin loop and reconnect. */
        rcl_publisher_t odom_pub;
        rc = rclc_publisher_init_default(&odom_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
        bool odom_ok = (rc == RCL_RET_OK);
        if (!odom_ok) {
            ESP_LOGE(TAG, "odom publisher init failed: %d", (int)rc);
        }

        rcl_publisher_t failsafe_pub;
        rc = rclc_publisher_init_default(&failsafe_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "failsafe/active");
        bool failsafe_ok = (rc == RCL_RET_OK);
        if (!failsafe_ok) {
            ESP_LOGE(TAG, "failsafe publisher init failed: %d", (int)rc);
        }

#ifdef CONFIG_MOTOR_DRIVER_VESC
        rcl_publisher_t battery_pub[2];
        bool battery_ok[2];
        rc = rclc_publisher_init_default(&battery_pub[0], &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
            "vesc/left/battery");
        battery_ok[0] = (rc == RCL_RET_OK);
        if (!battery_ok[0]) {
            ESP_LOGE(TAG, "vesc/left/battery publisher init failed: %d", (int)rc);
        }
        rc = rclc_publisher_init_default(&battery_pub[1], &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
            "vesc/right/battery");
        battery_ok[1] = (rc == RCL_RET_OK);
        if (!battery_ok[1]) {
            ESP_LOGE(TAG, "vesc/right/battery publisher init failed: %d", (int)rc);
        }
#endif

        /* ── Subscription ───────────────────────────────────────── */
        rcl_subscription_t cmd_vel_sub;
        rc = rclc_subscription_init_default(&cmd_vel_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
        bool sub_ok = (rc == RCL_RET_OK);
        if (!sub_ok) {
            ESP_LOGE(TAG, "cmd_vel subscription init failed: %d", (int)rc);
        }

        /* ── Services ───────────────────────────────────────────── */
        /* reset_odom (std_srvs/Trigger): zero the odom pose without a
         * reboot — for bench odometry-closure calibration.  Non-critical:
         * a failure here is logged but does not block driving. */
        rcl_service_t reset_odom_srv;
        rc = rclc_service_init_default(&reset_odom_srv, &node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "reset_odom");
        bool reset_srv_ok = (rc == RCL_RET_OK);   /* true ⇒ init'd, needs fini */
        if (!reset_srv_ok) {
            ESP_LOGE(TAG, "reset_odom service init failed: %d", (int)rc);
        }

        /* ── Executor ───────────────────────────────────────────── */
        /* Handles: cmd_vel subscription + reset_odom service. */
        rclc_executor_t executor;
        rc = rclc_executor_init(&executor, &support.context, 2, &allocator);
        bool exec_init_ok = (rc == RCL_RET_OK);
        if (!exec_init_ok) {
            ESP_LOGE(TAG, "executor init failed: %d", (int)rc);
        }
        bool exec_ready = false;
        if (exec_init_ok && sub_ok) {
            rc = rclc_executor_add_subscription(&executor, &cmd_vel_sub,
                                                &s_cmd_vel_msg, &cmd_vel_cb,
                                                ON_NEW_DATA);
            exec_ready = (rc == RCL_RET_OK);
            if (!exec_ready) {
                ESP_LOGE(TAG, "executor add cmd_vel failed: %d", (int)rc);
            }
        }
        if (exec_init_ok && reset_srv_ok) {
            rc = rclc_executor_add_service(&executor, &reset_odom_srv,
                                           &s_reset_req, &s_reset_res,
                                           &reset_odom_cb);
            if (rc != RCL_RET_OK) {
                /* Service stays init'd (still fini'd below); it just won't
                 * receive requests this session. */
                ESP_LOGE(TAG, "executor add reset_odom failed: %d", (int)rc);
            }
        }

        /* ── Init odom message frame IDs ────────────────────────── */
        /* micro-ROS static strings — set once */
        static const char odom_frame[] = "odom";
        static const char base_frame[] = "base_link";
        s_odom_msg.header.frame_id.data    = (char *)odom_frame;
        s_odom_msg.header.frame_id.size    = sizeof(odom_frame) - 1;
        s_odom_msg.header.frame_id.capacity = sizeof(odom_frame);
        s_odom_msg.child_frame_id.data     = (char *)base_frame;
        s_odom_msg.child_frame_id.size     = sizeof(base_frame) - 1;
        s_odom_msg.child_frame_id.capacity = sizeof(base_frame);

        /* ── Static odom covariance diagonals ───────────────────── */
        /* 6x6 row-major [x, y, z, roll, pitch, yaw].  The host EKF
         * weights inputs by covariance; all-zero reads as "infinitely
         * certain" and breaks fusion once a host-side IMU is fused in
         * (the IMU lives on the main ROS computer, not the ESP32).  The
         * drive fuses vx + vyaw, so those diagonals matter most; unused
         * 2D axes (z/roll/pitch) get a large value.  x/y/yaw measured from
         * the M1 square-drive closure (2026-06-21, smooth floor — optimistic
         * vs outdoor terrain; re-measure on the real surface). */
        s_odom_msg.pose.covariance[0]  = 0.00096; /* x     (m^2)    */
        s_odom_msg.pose.covariance[7]  = 0.00060; /* y     (m^2)    */
        s_odom_msg.pose.covariance[14] = 1e6;     /* z     (unused) */
        s_odom_msg.pose.covariance[21] = 1e6;     /* roll  (unused) */
        s_odom_msg.pose.covariance[28] = 1e6;     /* pitch (unused) */
        s_odom_msg.pose.covariance[35] = 0.00054; /* yaw   (rad^2)  */

        s_odom_msg.twist.covariance[0]  = 0.001; /* vx    (m/s)^2    */
        s_odom_msg.twist.covariance[7]  = 1e6;   /* vy    (non-holo)*/
        s_odom_msg.twist.covariance[14] = 1e6;   /* vz              */
        s_odom_msg.twist.covariance[21] = 1e6;   /* vroll           */
        s_odom_msg.twist.covariance[28] = 1e6;   /* vpitch          */
        s_odom_msg.twist.covariance[35] = 0.003; /* vyaw  (rad/s)^2 */

        /* ── Spin loop ──────────────────────────────────────────── */
        /* Seed timers so the first periodic ping / re-sync fire one
         * interval from now (we just synced + the agent is live). */
        int64_t last_sync_us = esp_timer_get_time();
        int64_t last_ping_us = last_sync_us;

        if (exec_ready) {
            ESP_LOGI(TAG, "Spinning...");
        } else {
            ESP_LOGE(TAG, "executor/subscription unavailable — reconnecting");
        }

        /* Skipped entirely when exec_ready is false (falls straight
         * through to cleanup + reconnect). */
        while (exec_ready) {
            rclc_executor_spin_some(&executor,
                                     RCL_MS_TO_NS(UROS_SPIN_PERIOD_MS));

            int64_t now_us = esp_timer_get_time();

            /* Liveness: ping the agent ~once per second instead of every
             * spin iteration.  A ping is a full XRCE round trip, so pinging
             * every ~10 ms made the loop rate (and the odom publish rate)
             * RTT-bound and roughly doubled transport traffic.  Between
             * pings the executor spin drives all traffic and would surface
             * a dead link; the periodic ping is the explicit liveness check
             * that tears down + reconnects on loss. */
            if (now_us - last_ping_us > 1000000LL) {   /* every 1 s */
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                    ESP_LOGW(TAG, "agent ping failed — assuming disconnect");
                    break;
                }
                last_ping_us = now_us;
            }

            /* Periodic re-sync for MCU/host crystal drift (~tens of ppm).
             * Non-fatal on timeout — the stored offset stays valid. */
            if (now_us - last_sync_us > 5000000LL) {   /* every 5 s */
                rmw_uros_sync_session(200);
                last_sync_us = now_us;
            }

            if (odom_ok)     publish_odom(&odom_pub);
            if (failsafe_ok) publish_failsafe(&failsafe_pub);
#ifdef CONFIG_MOTOR_DRIVER_VESC
            if (battery_ok[0]) publish_battery(&battery_pub[0], VESC_ID_LEFT,  0);
            if (battery_ok[1]) publish_battery(&battery_pub[1], VESC_ID_RIGHT, 1);
#endif
        }

        /* ── Cleanup (agent lost or session setup incomplete) ────── */
        ESP_LOGW(TAG, "Cleaning up session entities...");

        if (exec_init_ok) rclc_executor_fini(&executor);
        if (reset_srv_ok) rcl_service_fini(&reset_odom_srv, &node);
        if (sub_ok)       rcl_subscription_fini(&cmd_vel_sub, &node);
#ifdef CONFIG_MOTOR_DRIVER_VESC
        if (battery_ok[1]) rcl_publisher_fini(&battery_pub[1], &node);
        if (battery_ok[0]) rcl_publisher_fini(&battery_pub[0], &node);
#endif
        if (failsafe_ok) rcl_publisher_fini(&failsafe_pub, &node);
        if (odom_ok)     rcl_publisher_fini(&odom_pub, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ── Public init ─────────────────────────────────────────────────── */

esp_err_t uros_task_init(void)
{
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    /* Register USB-Serial/JTAG as the XRCE transport with HDLC framing.
     * The host agent must be launched in `serial` mode against the
     * enumerated CDC device (e.g. /dev/ttyACM0). */
    rmw_uros_set_custom_transport(
        true,                                   /* framing = true */
        NULL,                                   /* no per-transport args */
        uros_transport_usb_jtag_open,
        uros_transport_usb_jtag_close,
        uros_transport_usb_jtag_write,
        uros_transport_usb_jtag_read);
    ESP_LOGI(TAG, "Transport: USB-Serial/JTAG (custom, framed)");
#else
#error "micro-ROS library not built with RMW_UXRCE_TRANSPORT=custom — check app-colcon.meta"
#endif

    BaseType_t ok = xTaskCreatePinnedToCore(uros_task_fn, "uros",
                        UROS_TASK_STACK, NULL, UROS_TASK_PRIO, NULL, 0);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}
