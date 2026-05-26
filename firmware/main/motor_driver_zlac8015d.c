/*
 * motor_driver_zlac8015d.c — ZLAC8015D CANopen motor-driver backend
 *
 * Implements the hardware-agnostic motor_driver.h HAL on top of two
 * CANopen-CiA-402 axes, presented as separate node IDs on a shared
 * 500 kbit/s TWAI bus.
 *
 * Vendor assumptions (recorded in detail in
 * /memories/session/phase2_zlac_assumptions.md — review against the
 * ZLAC8015D manual before robot bring-up):
 *
 *   • Two CANopen nodes — node LEFT = CONFIG_ZLAC_NODE_LEFT,
 *     node RIGHT = CONFIG_ZLAC_NODE_RIGHT.
 *   • Profile Velocity mode (0x6060 = 3); target velocity via
 *     0x60FF in motor-shaft RPM (int32, signed = direction).
 *   • Encoder feedback via 0x6064 in counts; CPR is a Kconfig knob.
 *   • Velocity feedback via 0x606C in motor RPM.
 *   • PDO layout configured at init via SDO:
 *       RPDO1 (host→drive, cob = 0x200 + node):
 *           u16 controlword (0x6040,0) | i32 target_velocity (0x60FF,0)
 *       TPDO1 (drive→host, cob = 0x180 + node, transmission type 1):
 *           u16 statusword  (0x6041,0) | i32 position_actual (0x6064,0)
 *       TPDO2 (drive→host, cob = 0x280 + node, transmission type 1):
 *           i32 velocity_actual (0x606C,0) | i32 reserved
 *   • SYNC frame sent by us at 50 Hz to trigger TPDO transmission.
 *
 * Architecture
 * ────────────
 *   - canopen_init() handles TWAI bring-up.
 *   - At init we per-node:
 *       1. NMT reset_node, wait for heartbeat = BOOTUP (0)
 *       2. NMT enter pre-operational
 *       3. Disable PDOs (write valid=1 bit into cob-id), clear maps,
 *          install our mapping, re-enable PDOs.
 *       4. Configure Profile Velocity mode + accel/decel limits.
 *       5. NMT start (operational)
 *   - tx task @ 50 Hz: send SYNC + 2× RPDO1 (one per node).  Drives
 *     the CiA 402 state machine toward Operation Enabled (or
 *     Switch-On-Disabled when disarmed / e-stop).
 *   - rx via TPDO1/TPDO2 callbacks: updates per-axis caches and
 *     republishes motor_feedback_t once both axes have fresh data.
 *
 * What this file does NOT do (matches motor_driver_vesc.c):
 *   - cmd_vel arbitration, RC failsafe, tune override → motor_task.
 *   - Odometry integration → diff_drive + motor_task.
 *
 * Compiled only when CONFIG_MOTOR_DRIVER_ZLAC8015D is selected.
 */

#include "motor_driver.h"
#include "motor_driver_zlac8015d.h"
#include "canopen.h"
#include "cia402.h"
#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "motor_zlac";

/* ── Hardware configuration (Kconfig-overridable) ───────────────── */

#ifndef CONFIG_ZLAC_CAN_TX_GPIO
#define CONFIG_ZLAC_CAN_TX_GPIO  15
#endif
#ifndef CONFIG_ZLAC_CAN_RX_GPIO
#define CONFIG_ZLAC_CAN_RX_GPIO  16
#endif
#ifndef CONFIG_ZLAC_NODE_LEFT
#define CONFIG_ZLAC_NODE_LEFT    1
#endif
#ifndef CONFIG_ZLAC_NODE_RIGHT
#define CONFIG_ZLAC_NODE_RIGHT   2
#endif
#ifndef CONFIG_ZLAC_ENCODER_CPR
#define CONFIG_ZLAC_ENCODER_CPR  16384      /* 4096 lines × 4 quadrature */
#endif
#ifndef CONFIG_ZLAC_GEAR_RATIO_X100
#define CONFIG_ZLAC_GEAR_RATIO_X100  100    /* 1.00 : direct drive */
#endif
#ifndef CONFIG_ZLAC_INVERT_LEFT
#define CONFIG_ZLAC_INVERT_LEFT   0
#endif
#ifndef CONFIG_ZLAC_INVERT_RIGHT
#define CONFIG_ZLAC_INVERT_RIGHT  0
#endif
#ifndef CONFIG_ZLAC_PROFILE_ACCEL
#define CONFIG_ZLAC_PROFILE_ACCEL  500      /* RPM/s (motor) */
#endif
#ifndef CONFIG_ZLAC_PROFILE_DECEL
#define CONFIG_ZLAC_PROFILE_DECEL  500
#endif

#define ZLAC_NUM_AXES            2
#define ZLAC_AXIS_LEFT           0
#define ZLAC_AXIS_RIGHT          1

#define ZLAC_TX_PERIOD_MS        20         /* 50 Hz */
#define ZLAC_TX_TASK_STACK       4096
#define ZLAC_TX_TASK_PRIO        5
#define ZLAC_TX_TASK_CORE        1

#define ZLAC_BOOT_TIMEOUT_MS     3000
#define ZLAC_HEARTBEAT_TIMEOUT_MS  500
#define ZLAC_TPDO_TIMEOUT_MS     200
#define ZLAC_SDO_TIMEOUT_MS      200

/* CiA 402 object indices (standard) */
#define OBJ_CONTROLWORD          0x6040
#define OBJ_STATUSWORD           0x6041
#define OBJ_MODES_OF_OPERATION   0x6060
#define OBJ_POSITION_ACTUAL      0x6064
#define OBJ_VELOCITY_ACTUAL      0x606C
#define OBJ_TARGET_VELOCITY      0x60FF
#define OBJ_PROFILE_ACCEL        0x6083
#define OBJ_PROFILE_DECEL        0x6084

/* PDO communication-parameter and mapping-parameter indices */
#define OBJ_RPDO1_COMM           0x1400
#define OBJ_RPDO1_MAP            0x1600
#define OBJ_TPDO1_COMM           0x1800
#define OBJ_TPDO1_MAP            0x1A00
#define OBJ_TPDO2_COMM           0x1801
#define OBJ_TPDO2_MAP            0x1A01

/* ── Static node table ──────────────────────────────────────────── */

static const uint8_t s_node_id[ZLAC_NUM_AXES] = {
    [ZLAC_AXIS_LEFT]  = CONFIG_ZLAC_NODE_LEFT,
    [ZLAC_AXIS_RIGHT] = CONFIG_ZLAC_NODE_RIGHT,
};
static const int s_invert[ZLAC_NUM_AXES] = {
    [ZLAC_AXIS_LEFT]  = CONFIG_ZLAC_INVERT_LEFT  ? -1 : +1,
    [ZLAC_AXIS_RIGHT] = CONFIG_ZLAC_INVERT_RIGHT ? -1 : +1,
};

/* ── Shared state (spinlock-protected, never blocks) ────────────── */

static portMUX_TYPE      s_cmd_mux = portMUX_INITIALIZER_UNLOCKED;
static motor_wheel_cmd_t s_cmd;             /* latest setpoint */
static bool              s_estop;

static portMUX_TYPE     s_axis_mux = portMUX_INITIALIZER_UNLOCKED;
typedef struct {
    /* Last TPDO observations */
    uint16_t statusword;
    int32_t  position_counts;
    int32_t  velocity_motor_rpm;
    uint32_t last_tpdo_ms;
    bool     tpdo_seen;

    /* Last command we transmitted (debug only) */
    int32_t  cmd_velocity_motor_rpm;
    uint16_t cmd_controlword;
} zlac_axis_state_t;
static zlac_axis_state_t s_axis[ZLAC_NUM_AXES];

static portMUX_TYPE     s_feedback_mux = portMUX_INITIALIZER_UNLOCKED;
static motor_feedback_t s_feedback;
static bool             s_feedback_valid;

static portMUX_TYPE     s_health_mux = portMUX_INITIALIZER_UNLOCKED;
static motor_health_t   s_health;        /* updated from tx task */

#define BIT_ARMED   (1u << 0)
static EventGroupHandle_t s_events;

/* ── Helpers ─────────────────────────────────────────────────────── */

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static inline float counts_per_wheel_rev(void)
{
    /* Wheel rev = motor rev (direct drive) × gear ratio.  CPR is in
     * motor-shaft counts/rev.  Wheel counts = motor_counts × gear. */
    return (float)CONFIG_ZLAC_ENCODER_CPR *
           ((float)CONFIG_ZLAC_GEAR_RATIO_X100 / 100.0f);
}

static inline float gear_ratio_f(void)
{
    return (float)CONFIG_ZLAC_GEAR_RATIO_X100 / 100.0f;
}

/* Wheel RPM → motor-shaft RPM. */
static inline int32_t wheel_rpm_to_motor_rpm(float wheel_rpm, int axis)
{
    float motor = wheel_rpm * gear_ratio_f() * (float)s_invert[axis];
    /* Round half-away-from-zero */
    return (int32_t)(motor + (motor >= 0.0f ? 0.5f : -0.5f));
}

/* Encoder counts (signed, motor-side) → wheel revolutions. */
static inline float counts_to_wheel_revs(int32_t counts, int axis)
{
    float revs = (float)counts / counts_per_wheel_rev();
    return revs * (float)s_invert[axis];
}

/* Motor RPM → wheel RPM. */
static inline float motor_rpm_to_wheel_rpm(int32_t motor_rpm, int axis)
{
    return ((float)motor_rpm / gear_ratio_f()) * (float)s_invert[axis];
}

/* ── PDO mapping installation (SDO) ─────────────────────────────── */

/*
 * Each call sequence below mirrors the standard "disable PDO →
 * clear mapping → install new mapping → enable PDO" pattern.
 * On any error we log and return — the boot health check will
 * report the affected axis as down.
 */

#define MAP_ENTRY(idx, sub, bits)  \
    (((uint32_t)(idx) << 16) | ((uint32_t)(sub) << 8) | ((uint32_t)(bits)))

static esp_err_t configure_rpdo1(uint8_t node)
{
    esp_err_t r;
    /* 1. Disable the PDO (set MSB of COB-ID in 0x1400,1) */
    r = canopen_sdo_write_u32(node, OBJ_RPDO1_COMM, 1,
            0x80000000u | (uint32_t)(CANOPEN_COB_RPDO1_BASE + node),
            ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    /* 2. Clear the mapping count */
    r = canopen_sdo_write_u8(node, OBJ_RPDO1_MAP, 0, 0, ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    /* 3. Install entries: controlword (16 bit) then target_velocity (32 bit) */
    r = canopen_sdo_write_u32(node, OBJ_RPDO1_MAP, 1,
            MAP_ENTRY(OBJ_CONTROLWORD,    0, 16), ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    r = canopen_sdo_write_u32(node, OBJ_RPDO1_MAP, 2,
            MAP_ENTRY(OBJ_TARGET_VELOCITY, 0, 32), ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    /* 4. Set mapping count = 2 */
    r = canopen_sdo_write_u8(node, OBJ_RPDO1_MAP, 0, 2, ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    /* 5. Set transmission type = 255 (async, on receipt) */
    r = canopen_sdo_write_u8(node, OBJ_RPDO1_COMM, 2, 0xFF,
            ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    /* 6. Re-enable PDO (clear MSB of COB-ID) */
    r = canopen_sdo_write_u32(node, OBJ_RPDO1_COMM, 1,
            (uint32_t)(CANOPEN_COB_RPDO1_BASE + node),
            ZLAC_SDO_TIMEOUT_MS);
    return r;
}

static esp_err_t configure_tpdo(uint8_t node,
                                uint16_t comm_idx, uint16_t map_idx,
                                uint32_t cob_base,
                                uint32_t entry_a, uint32_t entry_b)
{
    esp_err_t r;
    /* Disable */
    r = canopen_sdo_write_u32(node, comm_idx, 1,
            0x80000000u | (uint32_t)(cob_base + node),
            ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    /* Clear map */
    r = canopen_sdo_write_u8(node, map_idx, 0, 0, ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    r = canopen_sdo_write_u32(node, map_idx, 1, entry_a, ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    r = canopen_sdo_write_u32(node, map_idx, 2, entry_b, ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    r = canopen_sdo_write_u8(node, map_idx, 0, 2, ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    /* Transmission type = 1 (every SYNC) */
    r = canopen_sdo_write_u8(node, comm_idx, 2, 1, ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) return r;
    /* Re-enable */
    r = canopen_sdo_write_u32(node, comm_idx, 1,
            (uint32_t)(cob_base + node), ZLAC_SDO_TIMEOUT_MS);
    return r;
}

/* ── PDO RX callbacks ───────────────────────────────────────────── */

static void on_tpdo1(uint32_t cob_id, const uint8_t *data, uint8_t dlc, void *ctx)
{
    int axis = (int)(intptr_t)ctx;
    if (dlc < 6) return;
    uint16_t sw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    int32_t  pos = (int32_t)((uint32_t)data[2]
                           | ((uint32_t)data[3] << 8)
                           | ((uint32_t)data[4] << 16)
                           | ((uint32_t)data[5] << 24));
    uint32_t t = now_ms();
    taskENTER_CRITICAL(&s_axis_mux);
    s_axis[axis].statusword      = sw;
    s_axis[axis].position_counts = pos;
    s_axis[axis].last_tpdo_ms    = t;
    s_axis[axis].tpdo_seen       = true;
    taskEXIT_CRITICAL(&s_axis_mux);
    (void)cob_id;
}

static void on_tpdo2(uint32_t cob_id, const uint8_t *data, uint8_t dlc, void *ctx)
{
    int axis = (int)(intptr_t)ctx;
    if (dlc < 4) return;
    int32_t v = (int32_t)((uint32_t)data[0]
                        | ((uint32_t)data[1] << 8)
                        | ((uint32_t)data[2] << 16)
                        | ((uint32_t)data[3] << 24));
    taskENTER_CRITICAL(&s_axis_mux);
    s_axis[axis].velocity_motor_rpm = v;
    taskEXIT_CRITICAL(&s_axis_mux);
    (void)cob_id;
}

static esp_err_t register_axis_callbacks(int axis)
{
    uint8_t node = s_node_id[axis];
    esp_err_t r;
    r = canopen_register_pdo_cb(CANOPEN_COB_TPDO1_BASE + node,
                                on_tpdo1, (void *)(intptr_t)axis);
    if (r != ESP_OK) return r;
    r = canopen_register_pdo_cb(CANOPEN_COB_TPDO2_BASE + node,
                                on_tpdo2, (void *)(intptr_t)axis);
    return r;
}

/* ── Feedback publishing ────────────────────────────────────────── */

static void publish_feedback(uint32_t t_ms)
{
    motor_feedback_t fb;
    zlac_axis_state_t a[ZLAC_NUM_AXES];
    taskENTER_CRITICAL(&s_axis_mux);
    a[0] = s_axis[0];
    a[1] = s_axis[1];
    taskEXIT_CRITICAL(&s_axis_mux);

    fb.left.rpm           = motor_rpm_to_wheel_rpm(a[0].velocity_motor_rpm, 0);
    fb.left.revolutions   = counts_to_wheel_revs(a[0].position_counts, 0);
    fb.left.current_a     = 0.0f;
    fb.left.fault_code_raw = a[0].statusword;
    fb.left.fault_bits     = (a[0].statusword & CIA402_SW_FAULT)
                                ? MOTOR_FAULT_OTHER : 0;

    fb.right.rpm          = motor_rpm_to_wheel_rpm(a[1].velocity_motor_rpm, 1);
    fb.right.revolutions  = counts_to_wheel_revs(a[1].position_counts, 1);
    fb.right.current_a    = 0.0f;
    fb.right.fault_code_raw = a[1].statusword;
    fb.right.fault_bits     = (a[1].statusword & CIA402_SW_FAULT)
                                ? MOTOR_FAULT_OTHER : 0;

    /* The standard CiA 402 object dictionary has no required bus-
     * voltage entry; vendor extension would map it via TPDO3 in a
     * future iteration.  Leave at 0 for now. */
    fb.bus_voltage_v  = 0.0f;
    fb.last_update_ms = t_ms;

    taskENTER_CRITICAL(&s_feedback_mux);
    s_feedback       = fb;
    s_feedback_valid = true;
    taskEXIT_CRITICAL(&s_feedback_mux);
}

/* ── Health watchdog ────────────────────────────────────────────── */

static bool axis_online(int axis, uint32_t t_ms)
{
    /* Heartbeat present and within timeout */
    canopen_nmt_state_t hb_state;
    uint32_t hb_last;
    if (!canopen_get_heartbeat(s_node_id[axis], &hb_state, &hb_last)) {
        return false;
    }
    if ((t_ms - hb_last) > ZLAC_HEARTBEAT_TIMEOUT_MS) return false;
    if (hb_state != CANOPEN_NMT_STATE_OPERATIONAL) return false;

    bool fresh; uint16_t sw;
    taskENTER_CRITICAL(&s_axis_mux);
    fresh = s_axis[axis].tpdo_seen &&
            ((t_ms - s_axis[axis].last_tpdo_ms) <= ZLAC_TPDO_TIMEOUT_MS);
    sw = s_axis[axis].statusword;
    taskEXIT_CRITICAL(&s_axis_mux);
    if (!fresh) return false;
    if (sw & CIA402_SW_FAULT) return false;
    return true;
}

/* ── TX task: CiA 402 driver + SYNC + RPDO transmit ─────────────── */

static void encode_rpdo1(uint16_t controlword, int32_t target_vel,
                         uint8_t out[6])
{
    out[0] = (uint8_t)(controlword & 0xFF);
    out[1] = (uint8_t)((controlword >> 8) & 0xFF);
    out[2] = (uint8_t)((uint32_t)target_vel & 0xFF);
    out[3] = (uint8_t)(((uint32_t)target_vel >> 8) & 0xFF);
    out[4] = (uint8_t)(((uint32_t)target_vel >> 16) & 0xFF);
    out[5] = (uint8_t)(((uint32_t)target_vel >> 24) & 0xFF);
}

static void zlac_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    /* Track previous controlword bit-7 per axis to produce a rising
     * edge for fault-reset. */
    bool prev_cw_reset[ZLAC_NUM_AXES] = { false, false };

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ZLAC_TX_PERIOD_MS));

        uint32_t t = now_ms();

        /* Latch upper-layer command + e-stop */
        motor_wheel_cmd_t cmd_local;
        bool estop_local;
        taskENTER_CRITICAL(&s_cmd_mux);
        cmd_local   = s_cmd;
        estop_local = s_estop;
        taskEXIT_CRITICAL(&s_cmd_mux);

        bool armed = (xEventGroupGetBits(s_events) & BIT_ARMED) != 0;

        /* Per-axis health (drives target-state choice) */
        bool axis_ok[ZLAC_NUM_AXES] = {
            axis_online(0, t),
            axis_online(1, t),
        };
        bool all_ok = axis_ok[0] && axis_ok[1];

        /* If anything is wrong, target is Switch On Disabled (motor
         * de-energised — safest stop).  Otherwise drive to OE. */
        cia402_state_t target = (armed && !estop_local && all_ok)
            ? CIA402_STATE_OPERATION_ENABLED
            : CIA402_STATE_SWITCH_ON_DISABLED;

        /* Always send SYNC first so TPDOs latch the prior tick's
         * statuswords before we read them next cycle. */
        (void)canopen_sync_send();

        for (int axis = 0; axis < ZLAC_NUM_AXES; axis++) {
            uint8_t node = s_node_id[axis];

            uint16_t sw;
            taskENTER_CRITICAL(&s_axis_mux);
            sw = s_axis[axis].statusword;
            taskEXIT_CRITICAL(&s_axis_mux);
            cia402_state_t cur = cia402_decode_state(sw);

            uint16_t cw = cia402_next_controlword(cur, target,
                                                  prev_cw_reset[axis]);
            prev_cw_reset[axis] = (cw & 0x80) != 0;

            int32_t target_vel = 0;
            if (target == CIA402_STATE_OPERATION_ENABLED &&
                cur == CIA402_STATE_OPERATION_ENABLED) {
                float w = (axis == ZLAC_AXIS_LEFT)
                    ? cmd_local.left_rpm
                    : cmd_local.right_rpm;
                target_vel = wheel_rpm_to_motor_rpm(w, axis);
            }

            uint8_t pdu[6];
            encode_rpdo1(cw, target_vel, pdu);
            (void)canopen_send_pdo(CANOPEN_COB_RPDO1_BASE + node, pdu, 6);

            taskENTER_CRITICAL(&s_axis_mux);
            s_axis[axis].cmd_controlword         = cw;
            s_axis[axis].cmd_velocity_motor_rpm  = target_vel;
            taskEXIT_CRITICAL(&s_axis_mux);
        }

        /* Update health snapshot */
        uint32_t fault_bits = 0;
        if (!axis_ok[0]) fault_bits |= MOTOR_FAULT_COMMUNICATION;
        if (!axis_ok[1]) fault_bits |= MOTOR_FAULT_COMMUNICATION;
        uint16_t sw0, sw1;
        taskENTER_CRITICAL(&s_axis_mux);
        sw0 = s_axis[0].statusword;
        sw1 = s_axis[1].statusword;
        taskEXIT_CRITICAL(&s_axis_mux);
        if (sw0 & CIA402_SW_FAULT) fault_bits |= MOTOR_FAULT_OTHER;
        if (sw1 & CIA402_SW_FAULT) fault_bits |= MOTOR_FAULT_OTHER;

        taskENTER_CRITICAL(&s_health_mux);
        s_health.online      = all_ok;
        /* boot_passed is sticky — set by init, never cleared here. */
        s_health.fault_bits  = fault_bits;
        taskEXIT_CRITICAL(&s_health_mux);

        /* Publish feedback once per tick (cheap; consumer reads
         * via spinlock).  Skip the lock if neither axis has data. */
        bool any_data;
        taskENTER_CRITICAL(&s_axis_mux);
        any_data = s_axis[0].tpdo_seen || s_axis[1].tpdo_seen;
        taskEXIT_CRITICAL(&s_axis_mux);
        if (any_data) publish_feedback(t);

        /* Rate-limited debug */
        static TickType_t s_last_dbg;
        if ((xTaskGetTickCount() - s_last_dbg) >= pdMS_TO_TICKS(1000)) {
            zlac_axis_state_t a[ZLAC_NUM_AXES];
            taskENTER_CRITICAL(&s_axis_mux);
            a[0] = s_axis[0]; a[1] = s_axis[1];
            taskEXIT_CRITICAL(&s_axis_mux);
            ESP_LOGI(TAG,
                "tx: armed=%d estop=%d L[%s sw=0x%04X cw=0x%04X tgt=%" PRId32
                " act=%" PRId32 "] R[%s sw=0x%04X cw=0x%04X tgt=%" PRId32
                " act=%" PRId32 "]",
                armed, estop_local,
                cia402_state_name(cia402_decode_state(a[0].statusword)),
                a[0].statusword, a[0].cmd_controlword,
                a[0].cmd_velocity_motor_rpm, a[0].velocity_motor_rpm,
                cia402_state_name(cia402_decode_state(a[1].statusword)),
                a[1].statusword, a[1].cmd_controlword,
                a[1].cmd_velocity_motor_rpm, a[1].velocity_motor_rpm);
            s_last_dbg = xTaskGetTickCount();
        }
    }
}

/* ── Per-axis init (NMT + SDO config) ───────────────────────────── */

static esp_err_t wait_heartbeat(uint8_t node, canopen_nmt_state_t want,
                                uint32_t timeout_ms)
{
    uint32_t deadline = now_ms() + timeout_ms;
    while ((int32_t)(deadline - now_ms()) > 0) {
        canopen_nmt_state_t s;
        uint32_t ts;
        if (canopen_get_heartbeat(node, &s, &ts) && s == want) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t bring_up_axis(int axis)
{
    uint8_t node = s_node_id[axis];
    esp_err_t r;

    ESP_LOGI(TAG, "axis %d: NMT reset_node (node %u)", axis, node);
    r = canopen_nmt_send(CANOPEN_NMT_RESET_NODE, node);
    if (r != ESP_OK) return r;

    r = wait_heartbeat(node, CANOPEN_NMT_STATE_BOOTUP, ZLAC_BOOT_TIMEOUT_MS);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "axis %d: no boot-up heartbeat from node %u", axis, node);
        /* Continue anyway — some drives skip the explicit boot-up
         * frame and jump straight to pre-operational. */
    }

    r = canopen_nmt_send(CANOPEN_NMT_ENTER_PREOP, node);
    if (r != ESP_OK) return r;
    vTaskDelay(pdMS_TO_TICKS(50));

    /* PDO mapping must be configured in pre-operational */
    r = configure_rpdo1(node);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "axis %d: RPDO1 mapping failed: %s",
                 axis, esp_err_to_name(r));
        return r;
    }
    r = configure_tpdo(node, OBJ_TPDO1_COMM, OBJ_TPDO1_MAP,
                       CANOPEN_COB_TPDO1_BASE,
                       MAP_ENTRY(OBJ_STATUSWORD,      0, 16),
                       MAP_ENTRY(OBJ_POSITION_ACTUAL, 0, 32));
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "axis %d: TPDO1 mapping failed: %s",
                 axis, esp_err_to_name(r));
        return r;
    }
    r = configure_tpdo(node, OBJ_TPDO2_COMM, OBJ_TPDO2_MAP,
                       CANOPEN_COB_TPDO2_BASE,
                       MAP_ENTRY(OBJ_VELOCITY_ACTUAL, 0, 32),
                       /* second entry reserved/dummy — drives that
                        * reject this can be tuned later. */
                       MAP_ENTRY(OBJ_VELOCITY_ACTUAL, 0, 32));
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "axis %d: TPDO2 mapping failed: %s (velocity feedback unavailable)",
                 axis, esp_err_to_name(r));
        /* Non-fatal — position-based velocity estimate at the upper
         * layer would be a fallback; we just lose live RPM. */
    }

    /* Mode of operation = Profile Velocity */
    r = canopen_sdo_write_i8(node, OBJ_MODES_OF_OPERATION, 0,
                              CIA402_MODE_PROFILE_VELOCITY,
                              ZLAC_SDO_TIMEOUT_MS);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "axis %d: mode-of-operation write failed: %s",
                 axis, esp_err_to_name(r));
        return r;
    }

    /* Profile acceleration / deceleration limits */
    (void)canopen_sdo_write_u32(node, OBJ_PROFILE_ACCEL, 0,
                                 CONFIG_ZLAC_PROFILE_ACCEL,
                                 ZLAC_SDO_TIMEOUT_MS);
    (void)canopen_sdo_write_u32(node, OBJ_PROFILE_DECEL, 0,
                                 CONFIG_ZLAC_PROFILE_DECEL,
                                 ZLAC_SDO_TIMEOUT_MS);

    /* NMT start → operational (PDOs become active) */
    r = canopen_nmt_send(CANOPEN_NMT_START_REMOTE, node);
    if (r != ESP_OK) return r;

    /* Wait for the first TPDO so we have a statusword to work with. */
    uint32_t deadline = now_ms() + ZLAC_BOOT_TIMEOUT_MS;
    while ((int32_t)(deadline - now_ms()) > 0) {
        bool seen;
        taskENTER_CRITICAL(&s_axis_mux);
        seen = s_axis[axis].tpdo_seen;
        taskEXIT_CRITICAL(&s_axis_mux);
        if (seen) break;
        /* Generate SYNC so sync-triggered TPDOs get sent. */
        (void)canopen_sync_send();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    bool tpdo_seen;
    taskENTER_CRITICAL(&s_axis_mux);
    tpdo_seen = s_axis[axis].tpdo_seen;
    taskEXIT_CRITICAL(&s_axis_mux);

    if (!tpdo_seen) {
        ESP_LOGE(TAG, "axis %d: no TPDO from node %u after start", axis, node);
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "axis %d (node %u): brought up", axis, node);
    return ESP_OK;
}

/* ── HAL implementation ─────────────────────────────────────────── */

esp_err_t motor_driver_init(void)
{
    s_events = xEventGroupCreate();
    if (s_events == NULL) return ESP_ERR_NO_MEM;

    /* Health defaults: not yet up. */
    taskENTER_CRITICAL(&s_health_mux);
    s_health.online      = false;
    s_health.boot_passed = false;
    s_health.fault_bits  = 0;
    taskEXIT_CRITICAL(&s_health_mux);

    esp_err_t r = canopen_init(CONFIG_ZLAC_CAN_TX_GPIO,
                               CONFIG_ZLAC_CAN_RX_GPIO);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "canopen_init failed: %s", esp_err_to_name(r));
        return r;
    }

    /* Register TPDO callbacks BEFORE NMT start so we don't miss the
     * very first sync-triggered frames. */
    for (int axis = 0; axis < ZLAC_NUM_AXES; axis++) {
        r = register_axis_callbacks(axis);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "axis %d: cb registration failed: %s",
                     axis, esp_err_to_name(r));
            return r;
        }
    }

    bool all_ok = true;
    for (int axis = 0; axis < ZLAC_NUM_AXES; axis++) {
        r = bring_up_axis(axis);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "axis %d bring-up failed: %s",
                     axis, esp_err_to_name(r));
            all_ok = false;
        }
    }

    taskENTER_CRITICAL(&s_health_mux);
    s_health.boot_passed = all_ok;
    s_health.online      = all_ok;
    taskEXIT_CRITICAL(&s_health_mux);

    /* Spawn TX task regardless of boot result — disarmed it will
     * keep sending Disable-Voltage controlwords (safe). */
    BaseType_t ok = xTaskCreatePinnedToCore(zlac_tx_task, "motor_zlac_tx",
        ZLAC_TX_TASK_STACK, NULL, ZLAC_TX_TASK_PRIO, NULL,
        ZLAC_TX_TASK_CORE);
    if (ok != pdPASS) return ESP_FAIL;

    if (all_ok) {
        xEventGroupSetBits(s_events, BIT_ARMED);
        ESP_LOGI(TAG, "ZLAC8015D armed (both axes operational)");
    } else {
        ESP_LOGE(TAG, "ZLAC8015D health check failed; motor commands disarmed. "
                      "Fix wiring/config and reboot.");
    }

    return ESP_OK;
}

bool motor_driver_is_armed(void)
{
    if (s_events == NULL) return false;
    return (xEventGroupGetBits(s_events) & BIT_ARMED) != 0;
}

void motor_driver_set_cmd(const motor_wheel_cmd_t *cmd)
{
    if (cmd == NULL) return;
    taskENTER_CRITICAL(&s_cmd_mux);
    s_cmd   = *cmd;
    s_estop = false;     /* sticky-clear: matches VESC backend semantics */
    taskEXIT_CRITICAL(&s_cmd_mux);
}

void motor_driver_emergency_stop(void)
{
    taskENTER_CRITICAL(&s_cmd_mux);
    s_cmd.left_rpm  = 0.0f;
    s_cmd.right_rpm = 0.0f;
    s_estop = true;
    taskEXIT_CRITICAL(&s_cmd_mux);
}

bool motor_driver_get_feedback(motor_feedback_t *fb_out)
{
    if (fb_out == NULL) return false;
    bool valid;
    taskENTER_CRITICAL(&s_feedback_mux);
    *fb_out = s_feedback;
    valid = s_feedback_valid;
    taskEXIT_CRITICAL(&s_feedback_mux);
    if (!valid) {
        memset(fb_out, 0, sizeof(*fb_out));
        return false;
    }
    return true;
}

bool motor_driver_get_health(motor_health_t *health_out)
{
    if (health_out == NULL) return false;
    taskENTER_CRITICAL(&s_health_mux);
    *health_out = s_health;
    taskEXIT_CRITICAL(&s_health_mux);
    return true;
}

/* ── Backend-specific diagnostics ───────────────────────────────── */

bool motor_driver_zlac_get_axis_status(int axis, zlac_axis_status_t *out)
{
    if (out == NULL || axis < 0 || axis >= ZLAC_NUM_AXES) return false;
    canopen_nmt_state_t hb_state = CANOPEN_NMT_STATE_BOOTUP;
    uint32_t hb_last = 0;
    bool hb_seen = canopen_get_heartbeat(s_node_id[axis], &hb_state, &hb_last);
    uint32_t t = now_ms();

    zlac_axis_state_t snap;
    taskENTER_CRITICAL(&s_axis_mux);
    snap = s_axis[axis];
    taskEXIT_CRITICAL(&s_axis_mux);

    out->heartbeat_seen            = hb_seen;
    out->last_heartbeat_ms         = hb_last;
    out->pdo_fresh                 = snap.tpdo_seen &&
                                     ((t - snap.last_tpdo_ms) <= ZLAC_TPDO_TIMEOUT_MS);
    out->last_tpdo_ms              = snap.last_tpdo_ms;
    out->statusword                = snap.statusword;
    out->state                     = cia402_decode_state(snap.statusword);
    out->target_velocity_motor_rpm = snap.cmd_velocity_motor_rpm;
    out->actual_velocity_motor_rpm = snap.velocity_motor_rpm;
    out->position_counts           = snap.position_counts;
    return true;
}
