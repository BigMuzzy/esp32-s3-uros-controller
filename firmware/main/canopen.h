/*
 * canopen.h — CiA 301 primitives for ESP-IDF / TWAI
 *
 * Lightweight CANopen master implementation tailored to a single
 * application: drive 1–2 CiA 402 motor controllers over a 500 kbit/s
 * TWAI bus.  Out of scope: LSS, time stamp object, dynamic object
 * dictionary, full SDO segmented/block transfer.
 *
 * Provides
 * ────────
 *   - canopen_init()              : TWAI bring-up + internal RX task
 *   - canopen_nmt_send()          : NMT master command
 *   - canopen_sync_send()         : 0x080 SYNC frame
 *   - canopen_send_pdo()          : push an RPDO to a slave
 *   - canopen_sdo_write_*()       : SDO expedited download (1/2/4 B)
 *   - canopen_sdo_read_*()        : SDO expedited upload (2/4 B)
 *   - canopen_register_pdo_cb()   : install a TPDO/EMCY/heartbeat
 *                                   dispatch callback by COB-ID
 *   - canopen_get_heartbeat()     : poll latest heartbeat state
 *
 * Threading
 * ─────────
 *   All public calls are safe from any task.  Only ONE SDO request
 *   may be in flight at a time (enforced by an internal mutex);
 *   the call blocks until the response arrives or the timeout
 *   elapses.  PDO TX is non-blocking (5 ms TWAI tx timeout).
 *
 * Compiled only when CONFIG_MOTOR_DRIVER_ZLAC8015D is selected; the
 * VESC backend owns its own TWAI init independently.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── COB-ID layout (CiA 301) ─────────────────────────────────────── */

#define CANOPEN_COB_NMT_CMD        0x000u   /* broadcast */
#define CANOPEN_COB_SYNC           0x080u
#define CANOPEN_COB_EMCY_BASE      0x080u   /* + node id */
#define CANOPEN_COB_TPDO1_BASE     0x180u
#define CANOPEN_COB_RPDO1_BASE     0x200u
#define CANOPEN_COB_TPDO2_BASE     0x280u
#define CANOPEN_COB_RPDO2_BASE     0x300u
#define CANOPEN_COB_TPDO3_BASE     0x380u
#define CANOPEN_COB_RPDO3_BASE     0x400u
#define CANOPEN_COB_TPDO4_BASE     0x480u
#define CANOPEN_COB_RPDO4_BASE     0x500u
#define CANOPEN_COB_SDO_TX_BASE    0x580u   /* server → client */
#define CANOPEN_COB_SDO_RX_BASE    0x600u   /* client → server */
#define CANOPEN_COB_HEARTBEAT_BASE 0x700u

/* ── NMT master commands (CiA 301 §7.2.8) ────────────────────────── */

typedef enum {
    CANOPEN_NMT_START_REMOTE = 0x01,
    CANOPEN_NMT_STOP_REMOTE  = 0x02,
    CANOPEN_NMT_ENTER_PREOP  = 0x80,
    CANOPEN_NMT_RESET_NODE   = 0x81,
    CANOPEN_NMT_RESET_COMM   = 0x82,
} canopen_nmt_cmd_t;

/* ── NMT slave state (heartbeat byte) ────────────────────────────── */

typedef enum {
    CANOPEN_NMT_STATE_BOOTUP      = 0,
    CANOPEN_NMT_STATE_STOPPED     = 4,
    CANOPEN_NMT_STATE_OPERATIONAL = 5,
    CANOPEN_NMT_STATE_PRE_OP      = 127,
} canopen_nmt_state_t;

/* ── PDO / heartbeat / EMCY dispatch callback ────────────────────── */

/**
 * Called from the internal RX task whenever a frame with the
 * registered COB-ID arrives.  Keep callbacks short — they run
 * inline on the RX task and must not block.
 */
typedef void (*canopen_pdo_cb_t)(uint32_t cob_id,
                                 const uint8_t *data,
                                 uint8_t dlc,
                                 void *ctx);

/* ── Lifecycle ───────────────────────────────────────────────────── */

/**
 * Install + start the TWAI driver at 500 kbit/s and spawn the
 * internal CANopen RX task pinned to Core 1.  Must be called once
 * before any other canopen_* function.
 */
esp_err_t canopen_init(int gpio_tx, int gpio_rx);

/* ── Master frames ───────────────────────────────────────────────── */

/** Send an NMT command. `node_id == 0` broadcasts to all slaves. */
esp_err_t canopen_nmt_send(canopen_nmt_cmd_t cmd, uint8_t node_id);

/** Send the SYNC frame (COB-ID 0x080, no data). */
esp_err_t canopen_sync_send(void);

/** Transmit an RPDO (or any data frame) at an arbitrary COB-ID. */
esp_err_t canopen_send_pdo(uint32_t cob_id,
                           const uint8_t *data,
                           uint8_t dlc);

/* ── SDO expedited transfer ─────────────────────────────────────── */

/* Width is implied by the function name.  `timeout_ms == 0` means
 * "use the module default" (currently 200 ms). */
esp_err_t canopen_sdo_write_u8 (uint8_t node, uint16_t idx, uint8_t sub,
                                uint8_t  value, uint32_t timeout_ms);
esp_err_t canopen_sdo_write_i8 (uint8_t node, uint16_t idx, uint8_t sub,
                                int8_t   value, uint32_t timeout_ms);
esp_err_t canopen_sdo_write_u16(uint8_t node, uint16_t idx, uint8_t sub,
                                uint16_t value, uint32_t timeout_ms);
esp_err_t canopen_sdo_write_u32(uint8_t node, uint16_t idx, uint8_t sub,
                                uint32_t value, uint32_t timeout_ms);

esp_err_t canopen_sdo_read_u16(uint8_t node, uint16_t idx, uint8_t sub,
                               uint16_t *value, uint32_t timeout_ms);
esp_err_t canopen_sdo_read_u32(uint8_t node, uint16_t idx, uint8_t sub,
                               uint32_t *value, uint32_t timeout_ms);

/* ── PDO RX dispatch ─────────────────────────────────────────────── */

/**
 * Register a callback for the given COB-ID.  Replaces any previous
 * registration for the same COB-ID.  Pass `cb == NULL` to unregister.
 *
 * The dispatch table is small (CANOPEN_MAX_PDO_CB entries, currently
 * 16).  Returns ESP_ERR_NO_MEM if full.
 */
esp_err_t canopen_register_pdo_cb(uint32_t cob_id,
                                  canopen_pdo_cb_t cb,
                                  void *ctx);

/* ── Heartbeat poll ──────────────────────────────────────────────── */

/**
 * Snapshot the latest heartbeat observed for `node_id`.  Returns
 * false if no heartbeat has been received since init.
 *
 *  state_out    — NMT state from the heartbeat byte.
 *  last_ms_out  — esp_timer-based timestamp (ms) of the last frame.
 */
bool canopen_get_heartbeat(uint8_t node_id,
                           canopen_nmt_state_t *state_out,
                           uint32_t *last_ms_out);

#ifdef __cplusplus
}
#endif
