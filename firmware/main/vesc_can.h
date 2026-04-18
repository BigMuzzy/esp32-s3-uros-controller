/*
 * vesc_can.h — VESC CAN protocol encode/decode
 *
 * Thin layer over VESC CAN 2.0B extended-frame protocol. Encodes
 * outgoing ERPM commands and decodes incoming status broadcasts.
 * All VESC-protocol details (byte order, frame IDs, field packing)
 * are isolated here — the rest of the firmware works in SI units.
 *
 * CAN ID format (29-bit extended):
 *   ID[28:8] = command type
 *   ID[7:0]  = VESC controller ID
 *
 * Supported commands:
 *   TX: CAN_PACKET_SET_RPM  (cmd 3) — ERPM set-point, 4-byte big-endian
 *   RX: CAN_PACKET_STATUS   (cmd 9) — ERPM, current, duty
 *   RX: CAN_PACKET_STATUS_4 (cmd 14) — tachometer (cumulative ERPM ticks), voltage
 *
 * Requires TWAI driver to be initialized separately (see can_task.h).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── VESC IDs (configured in VESC Tool, must match) ──────────────── */

#define VESC_ID_LEFT   1
#define VESC_ID_RIGHT  2

/* ── CAN command IDs ─────────────────────────────────────────────── */

#define VESC_CAN_CMD_SET_RPM    3
#define VESC_CAN_CMD_STATUS     9
#define VESC_CAN_CMD_STATUS_4  14

/* ── Status data ─────────────────────────────────────────────────── */

/**
 * Decoded fields from CAN_PACKET_STATUS (cmd 9).
 * Broadcast by each VESC at ~50 Hz.
 */
typedef struct {
    int32_t erpm;          /* electrical RPM (signed) */
    float   current_motor; /* motor current, amps     */
    float   duty_cycle;    /* -1.0 .. +1.0            */
} vesc_status_t;

/**
 * Decoded fields from CAN_PACKET_STATUS_4 (cmd 14).
 * Used for odometry (tachometer) and battery monitoring (voltage).
 */
typedef struct {
    int32_t tachometer;    /* cumulative ERPM ticks (signed) */
    float   voltage_in;    /* input voltage, volts            */
} vesc_status4_t;

/* ── TX: command encoding ────────────────────────────────────────── */

/**
 * Build a TWAI message that commands a VESC to a target ERPM.
 *
 * @param vesc_id  Target VESC controller ID (VESC_ID_LEFT / VESC_ID_RIGHT).
 * @param erpm     Signed ERPM set-point.  Caller is responsible for
 *                 range-checking before calling.
 * @param out_msg  Filled with the ready-to-transmit TWAI message.
 */
void vesc_can_encode_rpm(uint8_t vesc_id, int32_t erpm,
                         twai_message_t *out_msg);

/* ── RX: status decoding ────────────────────────────────────────── */

/**
 * Identify the VESC command type from a received TWAI message.
 *
 * @param msg       Received TWAI message (must be extended frame).
 * @param vesc_id   Extracted VESC controller ID (lower 8 bits).
 * @return          Command ID (upper bits), or -1 if not extended frame.
 */
int vesc_can_get_cmd(const twai_message_t *msg, uint8_t *vesc_id);

/**
 * Decode CAN_PACKET_STATUS (cmd 9) from a received TWAI message.
 *
 * @param msg  Received message (caller should verify cmd == VESC_CAN_CMD_STATUS).
 * @param out  Decoded status fields.
 * @return     true on success, false if DLC is wrong.
 */
bool vesc_can_decode_status(const twai_message_t *msg, vesc_status_t *out);

/**
 * Decode CAN_PACKET_STATUS_4 (cmd 14) from a received TWAI message.
 *
 * @param msg  Received message (caller should verify cmd == VESC_CAN_CMD_STATUS_4).
 * @param out  Decoded status fields.
 * @return     true on success, false if DLC is wrong.
 */
bool vesc_can_decode_status4(const twai_message_t *msg, vesc_status4_t *out);

#ifdef __cplusplus
}
#endif
