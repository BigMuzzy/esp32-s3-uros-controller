/*
 * motor_driver_zlac8015d.c — CANopen backend for the ZLAC8015D V4 drive.
 *
 * Hardware topology (verified against the vendor "CANopen Communication
 * Quick Start Guide" + "Communication Routine" V1.07):
 *
 *   * A SINGLE CANopen node (default ID 1, configurable via object
 *     0x200A) provides both motor channels.  LEFT = sub-index 1,
 *     RIGHT = sub-index 2 on every per-channel object (0x6064, 0x606C,
 *     0x60FF, …).  Controlword (0x6040) and mode-of-operation (0x6060)
 *     are single u16/u8 values that govern BOTH channels at once.
 *
 *   * Statusword 0x6041 is a U32: low 16 bits = LEFT axis, high 16 bits
 *     = RIGHT axis.  Each half follows the standard CiA 402 state-word
 *     bit layout (0x21 = RTSO, 0x23 = SO, 0x27 = OE, 0x40 = SOD, bit3
 *     = fault).  See routine document §3.1 "Status word switching
 *     state" — fully matches cia402.h decoder.
 *
 *   * Profile Velocity mode (0x6060 = 3) accepts targets at 0x60FF.
 *     With speed-resolution 0x2026:05 = 1 (factory default) the raw
 *     value is in motor RPM.  Range ±1000 r/min.  We use sub-indexes
 *     1 (LEFT) and 2 (RIGHT) directly, each i32, mapped into RPDO1
 *     as 8 data bytes (4 + 4 little-endian).
 *
 *   * Velocity feedback 0x606C is signed in 0.1 r/min units (sub 1 / 2,
 *     each i32).  Firmware divides feedback by 10 before applying the
 *     gear ratio.  Target commands stay in raw RPM.
 *
 *   * Position feedback 0x6064 is encoder counts (sub 1 / 2, each i32).
 *     CONFIG_ZLAC_ENCODER_COUNTS_PER_REV is the counts/rev at the
 *     motor shaft and equals 4 × (drive PPR 0x200E setting).  Default
 *     factory PPR = 1024 → 4096 counts/rev.
 *
 *   * Accel/decel objects 0x6083/0x6084 are S-curve TIMES in
 *     milliseconds (per QSG §9 object dictionary), NOT RPM/s.
 *
 *   * Heartbeat producer (0x1017, unit 0.5 ms) is disabled at factory
 *     (value 0).  We enable it during bring-up.
 *
 *   * PDOs used here:
 *       RPDO0  (cob 0x200 + node) — controlword (0x6040, default map)
 *       RPDO1  (cob 0x300 + node) — target velocity LEFT (0x60FF:1)
 *                                 + target velocity RIGHT (0x60FF:2)
 *       TPDO0  (cob 0x180 + node) — statusword (0x6041, u32)
 *       TPDO1  (cob 0x280 + node) — position LEFT + RIGHT
 *       TPDO2  (cob 0x380 + node) — velocity LEFT + RIGHT
 *     All asynchronous (transmission type 0xFF); TPDOs are timer-
 *     triggered via 0x18xx:05 event timer (no SYNC frame needed).
 *
 * Author: outdoor-patrol firmware (auto-generated, see HISTORY.md).
 */

#include "motor_driver.h"
#include "motor_driver_zlac8015d.h"

#include "canopen.h"
#include "cia402.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define TAG "zlac_drv"

/* ---- object dictionary indexes ---------------------------------------- */

#define OD_HEARTBEAT_TIME       0x1017  /* u16, unit 0.5 ms                */
#define OD_RPDO_COMM_BASE       0x1400  /* +n */
#define OD_RPDO_MAP_BASE        0x1600  /* +n */
#define OD_TPDO_COMM_BASE       0x1800  /* +n */
#define OD_TPDO_MAP_BASE        0x1A00  /* +n */
#define OD_FAULT_CODE           0x603F  /* u32 (low16=L, high16=R)         */
#define OD_CONTROLWORD          0x6040  /* u16, shared                     */
#define OD_STATUSWORD           0x6041  /* u32 (low16=L, high16=R)         */
#define OD_MODES_OF_OPERATION   0x6060  /* i8,  shared                     */
#define OD_POSITION_ACTUAL      0x6064  /* i32, sub1=L sub2=R              */
#define OD_VELOCITY_ACTUAL      0x606C  /* i32, sub1=L sub2=R (0.1 RPM)    */
#define OD_ACCEL_TIME           0x6083  /* u32, sub1=L sub2=R (ms)         */
#define OD_DECEL_TIME           0x6084  /* u32, sub1=L sub2=R (ms)         */
#define OD_TARGET_VELOCITY      0x60FF  /* i32, sub1=L sub2=R (RPM)        */

#define MODE_PROFILE_VELOCITY   3

#define AXIS_LEFT               0
#define AXIS_RIGHT              1
#define NUM_AXES                2

/* Encode an entry for the PDO mapping table (CiA 301 §7.4.6): the U32
 * value stored in 0x1600/0x1A00 sub-indexes is
 *   bits 31..16 = object index
 *   bits 15..8  = sub-index
 *   bits  7..0  = bit length
 */
#define PDO_MAP_ENTRY(idx, sub, bits)                                          \
    (((uint32_t)(idx) << 16) | ((uint32_t)(sub) << 8) | (uint32_t)(bits))

/* ---- compile-time configuration --------------------------------------- */

#define ZLAC_NODE               CONFIG_ZLAC_NODE_ID
#define ZLAC_COUNTS_PER_REV     CONFIG_ZLAC_ENCODER_COUNTS_PER_REV
#define ZLAC_GEAR_X100          CONFIG_ZLAC_GEAR_RATIO_X100
#define ZLAC_ACCEL_MS           CONFIG_ZLAC_PROFILE_ACCEL_MS
#define ZLAC_DECEL_MS           CONFIG_ZLAC_PROFILE_DECEL_MS
#define ZLAC_HEARTBEAT_MS       CONFIG_ZLAC_HEARTBEAT_INTERVAL_MS
#define ZLAC_TPDO_EVENT_MS      CONFIG_ZLAC_TPDO_EVENT_MS
#define ZLAC_TX_GPIO            CONFIG_ZLAC_CAN_TX_GPIO
#define ZLAC_RX_GPIO            CONFIG_ZLAC_CAN_RX_GPIO

#define ZLAC_TX_TICK_MS         20U    /* RPDO refresh cadence */
#define ZLAC_HEARTBEAT_TIMEOUT  500U   /* declare offline after (ms) */
#define ZLAC_TPDO_TIMEOUT       200U   /* feedback freshness threshold */

/* ---- backend state ---------------------------------------------------- */

typedef struct {
    bool             invert;
    int32_t          target_motor_rpm;   /* setpoint after sign/clamp */
    int32_t          actual_motor_rpm;   /* feedback                  */
    int32_t          position_counts;
    uint16_t         statusword;
    cia402_state_t   state;
    uint32_t         last_tpdo_ms;
    bool             tpdo_seen;
} axis_state_t;

static SemaphoreHandle_t s_lock;
static TaskHandle_t      s_tx_task;
static bool              s_initialised;
static bool              s_boot_passed;

/* Wheel-side command set by motor_driver_set_cmd() */
static motor_wheel_cmd_t s_cmd;
static bool              s_estop;     /* sticky until next set_cmd */
static bool              s_prev_cw_had_reset;

static axis_state_t      s_axes[NUM_AXES];

/* Heartbeat snapshot maintained by tx_task */
static uint32_t            s_last_heartbeat_ms;
static bool                s_heartbeat_seen;

/* ---- utility ---------------------------------------------------------- */

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static inline int32_t motor_rpm_to_wheel_rpm(int32_t mrpm)
{
    return (int32_t)((int64_t)mrpm * 100 / ZLAC_GEAR_X100);
}

static inline int32_t wheel_rpm_to_motor_rpm(int32_t wrpm)
{
    return (int32_t)((int64_t)wrpm * ZLAC_GEAR_X100 / 100);
}

static inline int32_t counts_per_wheel_rev(void)
{
    return (int32_t)((int64_t)ZLAC_COUNTS_PER_REV * ZLAC_GEAR_X100 / 100);
}

/* ---- TPDO receive callbacks ------------------------------------------- */

static void on_tpdo_statusword(uint32_t cob_id, const uint8_t *data,
                               uint8_t dlc, void *ctx)
{
    (void)cob_id; (void)ctx;
    if (dlc < 4) return;
    uint32_t sw32 = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                  | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    uint16_t sw_l = (uint16_t)(sw32 & 0xFFFF);
    uint16_t sw_r = (uint16_t)((sw32 >> 16) & 0xFFFF);
    uint32_t t = now_ms();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_axes[AXIS_LEFT].statusword   = sw_l;
    s_axes[AXIS_LEFT].state        = cia402_decode_state(sw_l);
    s_axes[AXIS_LEFT].last_tpdo_ms = t;
    s_axes[AXIS_LEFT].tpdo_seen    = true;
    s_axes[AXIS_RIGHT].statusword   = sw_r;
    s_axes[AXIS_RIGHT].state        = cia402_decode_state(sw_r);
    s_axes[AXIS_RIGHT].last_tpdo_ms = t;
    s_axes[AXIS_RIGHT].tpdo_seen    = true;
    xSemaphoreGive(s_lock);
}

static void on_tpdo_positions(uint32_t cob_id, const uint8_t *data,
                              uint8_t dlc, void *ctx)
{
    (void)cob_id; (void)ctx;
    if (dlc < 8) return;
    int32_t pos_l, pos_r;
    memcpy(&pos_l, &data[0], 4);
    memcpy(&pos_r, &data[4], 4);
    uint32_t t = now_ms();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_axes[AXIS_LEFT].position_counts  = s_axes[AXIS_LEFT].invert  ? -pos_l : pos_l;
    s_axes[AXIS_RIGHT].position_counts = s_axes[AXIS_RIGHT].invert ? -pos_r : pos_r;
    s_axes[AXIS_LEFT].last_tpdo_ms     = t;
    s_axes[AXIS_RIGHT].last_tpdo_ms    = t;
    xSemaphoreGive(s_lock);
}

static void on_tpdo_velocities(uint32_t cob_id, const uint8_t *data,
                               uint8_t dlc, void *ctx)
{
    (void)cob_id; (void)ctx;
    if (dlc < 8) return;
    int32_t vel_l_decirpm, vel_r_decirpm;
    memcpy(&vel_l_decirpm, &data[0], 4);
    memcpy(&vel_r_decirpm, &data[4], 4);
    /* Object 0x606C unit is 0.1 r/min — convert to integer RPM. */
    int32_t vel_l = vel_l_decirpm / 10;
    int32_t vel_r = vel_r_decirpm / 10;
    uint32_t t = now_ms();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_axes[AXIS_LEFT].actual_motor_rpm  = s_axes[AXIS_LEFT].invert  ? -vel_l : vel_l;
    s_axes[AXIS_RIGHT].actual_motor_rpm = s_axes[AXIS_RIGHT].invert ? -vel_r : vel_r;
    s_axes[AXIS_LEFT].last_tpdo_ms      = t;
    s_axes[AXIS_RIGHT].last_tpdo_ms     = t;
    xSemaphoreGive(s_lock);
}

/* ---- bring-up: configure the drive once over SDO --------------------- */

static bool sdo_w16(uint16_t idx, uint8_t sub, uint16_t v)
{
    return canopen_sdo_write_u16(ZLAC_NODE, idx, sub, v, 0) == ESP_OK;
}
static bool sdo_w32(uint16_t idx, uint8_t sub, uint32_t v)
{
    return canopen_sdo_write_u32(ZLAC_NODE, idx, sub, v, 0) == ESP_OK;
}
static bool sdo_w8(uint16_t idx, uint8_t sub, uint8_t v)
{
    return canopen_sdo_write_u8(ZLAC_NODE, idx, sub, v, 0) == ESP_OK;
}
static bool sdo_wi8(uint16_t idx, uint8_t sub, int8_t v)
{
    return canopen_sdo_write_i8(ZLAC_NODE, idx, sub, v, 0) == ESP_OK;
}

static bool configure_rpdo(uint8_t pdo_idx,
                           const uint32_t *entries, uint8_t n_entries)
{
    /* CiA 301 PDO config sequence:
     *   1. disable PDO    (set comm-param sub1 high bit = 1)
     *   2. clear mapping  (write 0 to map-param sub0)
     *   3. write entries  (sub1..n)
     *   4. set sub0 = n
     *   5. set tx-type   (sub2 of comm-param)
     *   6. re-enable PDO (clear high bit in comm-param sub1)
     */
    uint16_t comm = OD_RPDO_COMM_BASE + pdo_idx;
    uint16_t map  = OD_RPDO_MAP_BASE  + pdo_idx;
    uint32_t cob  = (pdo_idx == 0 ? 0x200u :
                     pdo_idx == 1 ? 0x300u :
                     pdo_idx == 2 ? 0x400u : 0x500u) + ZLAC_NODE;
    if (!sdo_w32(comm, 1, cob | 0x80000000u)) return false;
    if (!sdo_w8 (map,  0, 0))                 return false;
    for (uint8_t i = 0; i < n_entries; ++i) {
        if (!sdo_w32(map, i + 1, entries[i])) return false;
    }
    if (!sdo_w8 (map,  0, n_entries))         return false;
    if (!sdo_w8 (comm, 2, 0xFF))              return false;  /* async */
    if (!sdo_w32(comm, 1, cob))               return false;
    return true;
}

static bool configure_tpdo(uint8_t pdo_idx,
                           const uint32_t *entries, uint8_t n_entries,
                           uint16_t event_timer_halfms)
{
    uint16_t comm = OD_TPDO_COMM_BASE + pdo_idx;
    uint16_t map  = OD_TPDO_MAP_BASE  + pdo_idx;
    uint32_t cob  = (pdo_idx == 0 ? 0x180u :
                     pdo_idx == 1 ? 0x280u :
                     pdo_idx == 2 ? 0x380u : 0x480u) + ZLAC_NODE;
    if (!sdo_w32(comm, 1, cob | 0x80000000u)) return false;
    if (!sdo_w8 (map,  0, 0))                 return false;
    for (uint8_t i = 0; i < n_entries; ++i) {
        if (!sdo_w32(map, i + 1, entries[i])) return false;
    }
    if (!sdo_w8 (map,  0, n_entries))         return false;
    if (!sdo_w8 (comm, 2, 0xFF))              return false;  /* async */
    if (!sdo_w16(comm, 5, event_timer_halfms))return false;
    if (!sdo_w32(comm, 1, cob))               return false;
    return true;
}

static bool bring_up_drive(void)
{
    ESP_LOGI(TAG, "ZLAC bring-up: NMT reset node %u", (unsigned)ZLAC_NODE);
    canopen_nmt_send(CANOPEN_NMT_RESET_NODE, ZLAC_NODE);
    vTaskDelay(pdMS_TO_TICKS(500));   /* wait for boot-up */

    /* Heartbeat producer */
    if (!sdo_w16(OD_HEARTBEAT_TIME, 0,
                 (uint16_t)(ZLAC_HEARTBEAT_MS * 2))) {
        ESP_LOGE(TAG, "set 0x1017 failed");
        return false;
    }

    /* Profile-velocity mode */
    if (!sdo_wi8(OD_MODES_OF_OPERATION, 0, MODE_PROFILE_VELOCITY)) {
        ESP_LOGE(TAG, "set 0x6060 failed");
        return false;
    }

    /* Accel / decel times (ms) — set both channels */
    if (!sdo_w32(OD_ACCEL_TIME, 1, ZLAC_ACCEL_MS) ||
        !sdo_w32(OD_ACCEL_TIME, 2, ZLAC_ACCEL_MS) ||
        !sdo_w32(OD_DECEL_TIME, 1, ZLAC_DECEL_MS) ||
        !sdo_w32(OD_DECEL_TIME, 2, ZLAC_DECEL_MS)) {
        ESP_LOGE(TAG, "set 0x6083/0x6084 failed");
        return false;
    }

    /* Zero initial targets */
    sdo_w32(OD_TARGET_VELOCITY, 1, 0);
    sdo_w32(OD_TARGET_VELOCITY, 2, 0);

    /* RPDO0 — keep default mapping (controlword 16-bit on 0x200+node) */

    /* RPDO1 — two target velocities */
    {
        const uint32_t map[2] = {
            PDO_MAP_ENTRY(OD_TARGET_VELOCITY, 1, 32),
            PDO_MAP_ENTRY(OD_TARGET_VELOCITY, 2, 32),
        };
        if (!configure_rpdo(1, map, 2)) {
            ESP_LOGE(TAG, "RPDO1 config failed");
            return false;
        }
    }

    uint16_t event_timer_halfms = (uint16_t)(ZLAC_TPDO_EVENT_MS * 2);

    /* TPDO0 — statusword (u32) */
    {
        const uint32_t map[1] = {
            PDO_MAP_ENTRY(OD_STATUSWORD, 0, 32),
        };
        if (!configure_tpdo(0, map, 1, event_timer_halfms)) {
            ESP_LOGE(TAG, "TPDO0 config failed");
            return false;
        }
    }
    /* TPDO1 — positions L/R */
    {
        const uint32_t map[2] = {
            PDO_MAP_ENTRY(OD_POSITION_ACTUAL, 1, 32),
            PDO_MAP_ENTRY(OD_POSITION_ACTUAL, 2, 32),
        };
        if (!configure_tpdo(1, map, 2, event_timer_halfms)) {
            ESP_LOGE(TAG, "TPDO1 config failed");
            return false;
        }
    }
    /* TPDO2 — velocities L/R */
    {
        const uint32_t map[2] = {
            PDO_MAP_ENTRY(OD_VELOCITY_ACTUAL, 1, 32),
            PDO_MAP_ENTRY(OD_VELOCITY_ACTUAL, 2, 32),
        };
        if (!configure_tpdo(2, map, 2, event_timer_halfms)) {
            ESP_LOGE(TAG, "TPDO2 config failed");
            return false;
        }
    }

    /* Register RX hooks on the per-PDO COB-IDs */
    canopen_register_pdo_cb(0x180 + ZLAC_NODE, on_tpdo_statusword,  NULL);
    canopen_register_pdo_cb(0x280 + ZLAC_NODE, on_tpdo_positions,   NULL);
    canopen_register_pdo_cb(0x380 + ZLAC_NODE, on_tpdo_velocities,  NULL);

    /* Enter operational */
    ESP_LOGI(TAG, "NMT start node %u", (unsigned)ZLAC_NODE);
    canopen_nmt_send(CANOPEN_NMT_START_REMOTE, ZLAC_NODE);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* CiA 402 enable sequence over SDO (single controlword) */
    static const uint16_t boot_seq[] = { 0x06, 0x07, 0x0F };
    for (size_t i = 0; i < sizeof(boot_seq)/sizeof(boot_seq[0]); ++i) {
        if (!sdo_w16(OD_CONTROLWORD, 0, boot_seq[i])) {
            ESP_LOGE(TAG, "CW=0x%02X SDO failed", boot_seq[i]);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGI(TAG, "ZLAC bring-up complete");
    return true;
}

/* ---- periodic TX task: RPDO0 (controlword) + RPDO1 (targets) --------- */

static void tx_task(void *arg)
{
    (void)arg;
    while (1) {
        xSemaphoreTake(s_lock, portMAX_DELAY);

        /* Translate wheel-side cmd_vel into motor RPM per axis. */
        float wl_rpm = s_cmd.left_rpm;
        float wr_rpm = s_cmd.right_rpm;
        if (s_estop || !s_boot_passed) {
            wl_rpm = 0.0f;
            wr_rpm = 0.0f;
        }
        int32_t tgt_l = wheel_rpm_to_motor_rpm((int32_t)wl_rpm);
        int32_t tgt_r = wheel_rpm_to_motor_rpm((int32_t)wr_rpm);
        if (tgt_l >  1000) tgt_l =  1000;
        if (tgt_l < -1000) tgt_l = -1000;
        if (tgt_r >  1000) tgt_r =  1000;
        if (tgt_r < -1000) tgt_r = -1000;
        s_axes[AXIS_LEFT].target_motor_rpm  = tgt_l;
        s_axes[AXIS_RIGHT].target_motor_rpm = tgt_r;
        int32_t bus_l = s_axes[AXIS_LEFT].invert  ? -tgt_l : tgt_l;
        int32_t bus_r = s_axes[AXIS_RIGHT].invert ? -tgt_r : tgt_r;

        /* Single shared controlword: step worst axis toward target. */
        cia402_state_t st_l = s_axes[AXIS_LEFT].state;
        cia402_state_t st_r = s_axes[AXIS_RIGHT].state;
        cia402_state_t worst = (st_l < st_r) ? st_l : st_r;
        cia402_state_t target_state =
            (s_estop || !s_boot_passed)
                ? CIA402_STATE_SWITCH_ON_DISABLED
                : CIA402_STATE_OPERATION_ENABLED;
        uint16_t cw = cia402_next_controlword(worst, target_state,
                                              s_prev_cw_had_reset);
        s_prev_cw_had_reset = (cw & 0x0080) != 0;

        xSemaphoreGive(s_lock);

        /* RPDO0: controlword (cob 0x200+node, 2 bytes) */
        uint8_t cw_frame[2] = {
            (uint8_t)(cw & 0xFF),
            (uint8_t)((cw >> 8) & 0xFF),
        };
        canopen_send_pdo(0x200 + ZLAC_NODE, cw_frame, 2);

        /* RPDO1: target velocities (cob 0x300+node, 8 bytes) */
        uint8_t v_frame[8];
        memcpy(&v_frame[0], &bus_l, 4);
        memcpy(&v_frame[4], &bus_r, 4);
        canopen_send_pdo(0x300 + ZLAC_NODE, v_frame, 8);

        /* Pull the freshest heartbeat snapshot for the offline check. */
        canopen_nmt_state_t nmt = CANOPEN_NMT_STATE_BOOTUP;
        uint32_t hb_ms = 0;
        if (canopen_get_heartbeat(ZLAC_NODE, &nmt, &hb_ms)) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_heartbeat_seen      = true;
            s_last_heartbeat_ms   = hb_ms;
            xSemaphoreGive(s_lock);
        }

        vTaskDelay(pdMS_TO_TICKS(ZLAC_TX_TICK_MS));
    }
}

/* ---- motor_driver.h HAL implementation -------------------------------- */

esp_err_t motor_driver_init(void)
{
    if (s_initialised) return ESP_OK;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;

    memset(s_axes, 0, sizeof(s_axes));
#ifdef CONFIG_ZLAC_INVERT_LEFT
    s_axes[AXIS_LEFT].invert  = true;
#endif
#ifdef CONFIG_ZLAC_INVERT_RIGHT
    s_axes[AXIS_RIGHT].invert = true;
#endif
    s_estop             = false;
    s_prev_cw_had_reset = false;
    s_cmd.left_rpm      = 0.0f;
    s_cmd.right_rpm     = 0.0f;

    esp_err_t err = canopen_init(ZLAC_TX_GPIO, ZLAC_RX_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "canopen_init failed: %s", esp_err_to_name(err));
        return err;
    }

    s_boot_passed = bring_up_drive();
    if (!s_boot_passed) {
        ESP_LOGE(TAG, "bring-up failed; backend will stay disarmed");
        /* still spawn tx task so health telemetry remains queryable */
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        tx_task, "zlac_tx", 4096, NULL, 6, &s_tx_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "tx task spawn failed");
        return ESP_FAIL;
    }

    s_initialised = true;
    return s_boot_passed ? ESP_OK : ESP_FAIL;
}

bool motor_driver_is_armed(void)
{
    if (!s_initialised) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool armed = s_boot_passed &&
                 !s_estop &&
                 s_axes[AXIS_LEFT].state  == CIA402_STATE_OPERATION_ENABLED &&
                 s_axes[AXIS_RIGHT].state == CIA402_STATE_OPERATION_ENABLED;
    xSemaphoreGive(s_lock);
    return armed;
}

void motor_driver_set_cmd(const motor_wheel_cmd_t *cmd)
{
    if (!cmd || !s_initialised) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cmd   = *cmd;
    s_estop = false;     /* match VESC backend: any set_cmd lifts latch */
    xSemaphoreGive(s_lock);
}

void motor_driver_emergency_stop(void)
{
    if (!s_initialised) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cmd.left_rpm  = 0.0f;
    s_cmd.right_rpm = 0.0f;
    s_estop         = true;
    xSemaphoreGive(s_lock);
}

static uint32_t map_fault_bits(uint16_t sw)
{
    /* Bit 3 of statusword = drive in fault.  Without 0x603F context
     * we cannot bucket further; mark as OTHER. */
    return (sw & 0x0008) ? MOTOR_FAULT_OTHER : 0;
}

bool motor_driver_get_feedback(motor_feedback_t *fb_out)
{
    if (!fb_out) return false;
    memset(fb_out, 0, sizeof(*fb_out));
    if (!s_initialised) return false;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool any = s_axes[AXIS_LEFT].tpdo_seen || s_axes[AXIS_RIGHT].tpdo_seen;
    int32_t mrpm_l = s_axes[AXIS_LEFT].actual_motor_rpm;
    int32_t mrpm_r = s_axes[AXIS_RIGHT].actual_motor_rpm;
    int32_t cnts_l = s_axes[AXIS_LEFT].position_counts;
    int32_t cnts_r = s_axes[AXIS_RIGHT].position_counts;
    uint16_t sw_l  = s_axes[AXIS_LEFT].statusword;
    uint16_t sw_r  = s_axes[AXIS_RIGHT].statusword;
    uint32_t t_l   = s_axes[AXIS_LEFT].last_tpdo_ms;
    uint32_t t_r   = s_axes[AXIS_RIGHT].last_tpdo_ms;
    xSemaphoreGive(s_lock);

    int32_t cpwr = counts_per_wheel_rev();
    fb_out->left.rpm           = (float)motor_rpm_to_wheel_rpm(mrpm_l);
    fb_out->left.revolutions   = (float)((double)cnts_l / (double)cpwr);
    fb_out->left.current_a     = 0.0f;
    fb_out->left.fault_code_raw= sw_l;
    fb_out->left.fault_bits    = map_fault_bits(sw_l);
    fb_out->right.rpm          = (float)motor_rpm_to_wheel_rpm(mrpm_r);
    fb_out->right.revolutions  = (float)((double)cnts_r / (double)cpwr);
    fb_out->right.current_a    = 0.0f;
    fb_out->right.fault_code_raw= sw_r;
    fb_out->right.fault_bits   = map_fault_bits(sw_r);
    fb_out->bus_voltage_v      = 0.0f;  /* 0x2035 not in PDO map */
    fb_out->last_update_ms     = (t_l > t_r) ? t_l : t_r;
    return any;
}

bool motor_driver_get_health(motor_health_t *health_out)
{
    if (!health_out) return false;
    memset(health_out, 0, sizeof(*health_out));
    if (!s_initialised) return true;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t t       = now_ms();
    bool hb_fresh    = s_heartbeat_seen &&
                       (t - s_last_heartbeat_ms) < ZLAC_HEARTBEAT_TIMEOUT;
    bool tpdo_fresh  = s_axes[AXIS_LEFT].tpdo_seen && s_axes[AXIS_RIGHT].tpdo_seen &&
                       (t - s_axes[AXIS_LEFT].last_tpdo_ms)  < ZLAC_TPDO_TIMEOUT &&
                       (t - s_axes[AXIS_RIGHT].last_tpdo_ms) < ZLAC_TPDO_TIMEOUT;
    uint32_t faults  = map_fault_bits(s_axes[AXIS_LEFT].statusword) |
                       map_fault_bits(s_axes[AXIS_RIGHT].statusword);
    if (!hb_fresh) faults |= MOTOR_FAULT_COMMUNICATION;
    health_out->online      = s_boot_passed && hb_fresh && tpdo_fresh;
    health_out->boot_passed = s_boot_passed;
    health_out->fault_bits  = faults;
    xSemaphoreGive(s_lock);
    return true;
}

/* ---- diagnostics (motor_driver_zlac8015d.h) -------------------------- */

bool motor_driver_zlac_get_axis_status(int axis, zlac_axis_status_t *out)
{
    if (axis < 0 || axis >= NUM_AXES || !out) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    out->heartbeat_seen    = s_heartbeat_seen;
    out->last_heartbeat_ms = s_last_heartbeat_ms;
    out->pdo_fresh         = s_axes[axis].tpdo_seen &&
                             (now_ms() - s_axes[axis].last_tpdo_ms)
                                 < ZLAC_TPDO_TIMEOUT;
    out->last_tpdo_ms      = s_axes[axis].last_tpdo_ms;
    out->statusword        = s_axes[axis].statusword;
    out->state             = s_axes[axis].state;
    out->target_velocity_motor_rpm = s_axes[axis].target_motor_rpm;
    out->actual_velocity_motor_rpm = s_axes[axis].actual_motor_rpm;
    out->position_counts   = s_axes[axis].position_counts;
    xSemaphoreGive(s_lock);
    return true;
}
