/*
 * uros_task.c — micro-ROS spin task on Core 0
 *
 * Lifecycle:
 *   1. Configure USB-CDC transport
 *   2. Wait for micro-ROS agent
 *   3. Create node, publishers, subscriptions
 *   4. Spin loop: executor spin + publish odom/status/failsafe
 *   5. On agent disconnect: destroy entities, go to step 2
 *
 * Depends on micro_ros_espidf_component — will not compile until
 * that component is added to the project.
 */

#include "uros_task.h"
#include "uros_transport_usb_jtag.h"
#include "can_task.h"
#include "rc_failsafe.h"
#include "diff_drive.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/bool.h>

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

/* ── cmd_vel subscription callback ───────────────────────────────── */

static void cmd_vel_cb(const void *msg_in)
{
    const geometry_msgs__msg__Twist *twist =
        (const geometry_msgs__msg__Twist *)msg_in;

    cmd_vel_t cmd = {
        .linear_x  = (float)twist->linear.x,
        .angular_z = (float)twist->angular.z,
    };
    can_task_set_cmd_vel(&cmd);
}

/* ── Publish helpers ─────────────────────────────────────────────── */

static void publish_odom(rcl_publisher_t *pub)
{
    odom_state_t odom;
    can_task_get_odom(&odom);

    /* Header */
    /* frame_id and child_frame_id are set once at init */

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

        /* ── Publishers ─────────────────────────────────────────── */
        rcl_publisher_t odom_pub;
        rclc_publisher_init_default(&odom_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");

        rcl_publisher_t failsafe_pub;
        rclc_publisher_init_default(&failsafe_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "failsafe/active");

        /* ── Subscription ───────────────────────────────────────── */
        rcl_subscription_t cmd_vel_sub;
        rclc_subscription_init_default(&cmd_vel_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

        /* ── Executor ───────────────────────────────────────────── */
        rclc_executor_t executor;
        rclc_executor_init(&executor, &support.context, 1, &allocator);
        rclc_executor_add_subscription(&executor, &cmd_vel_sub,
                                        &s_cmd_vel_msg, &cmd_vel_cb,
                                        ON_NEW_DATA);

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

        /* ── Spin loop ──────────────────────────────────────────── */
        ESP_LOGI(TAG, "Spinning...");

        while (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            rclc_executor_spin_some(&executor,
                                     RCL_MS_TO_NS(UROS_SPIN_PERIOD_MS));
            publish_odom(&odom_pub);
            publish_failsafe(&failsafe_pub);
        }

        /* ── Agent lost — cleanup ───────────────────────────────── */
        ESP_LOGW(TAG, "Agent disconnected, cleaning up...");

        rclc_executor_fini(&executor);
        rcl_subscription_fini(&cmd_vel_sub, &node);
        rcl_publisher_fini(&failsafe_pub, &node);
        rcl_publisher_fini(&odom_pub, &node);
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
