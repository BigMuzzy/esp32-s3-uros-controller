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
 *   TX: CAN_PACKET_SET_RPM           (cmd 3)  — ERPM set-point, 4-byte big-endian
 *   TX: CAN_PACKET_SET_CURRENT_BRAKE (cmd 6)  — brake current (mA), 4-byte big-endian
 *   RX: CAN_PACKET_STATUS            (cmd 9)  — ERPM, current, duty
 *   RX: CAN_PACKET_STATUS_5          (cmd 27) — tachometer (cumulative ERPM ticks), v_in
 *
 * Note: STATUS_2/3/4 (cmd 14/15/16) carry amp-hours, watt-hours, and
 * temperatures respectively — not used here. Do not confuse STATUS_4
 * with STATUS_5; only STATUS_5 contains tachometer + voltage.
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

#define VESC_CAN_CMD_SET_RPM            3
#define VESC_CAN_CMD_SET_CURRENT_BRAKE  6
#define VESC_CAN_CMD_STATUS             9
#define VESC_CAN_CMD_PING              17
#define VESC_CAN_CMD_PONG              18
#define VESC_CAN_CMD_STATUS_5          27

/* Sender ID used by this ESP32 in ping requests.  Must not collide
 * with any VESC_ID_* on the bus.  VESC firmware treats IDs 0..253
 * as regular controllers; 0xFE is a conventional "external tool"
 * sender ID matching VESC Tool defaults. */
#define VESC_CAN_SENDER_ID   0xFE

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
 * Decoded fields from CAN_PACKET_STATUS_5 (cmd 27).
 * Used for odometry (tachometer) and battery monitoring (voltage).
 */
typedef struct {
    int32_t tachometer;    /* cumulative ERPM ticks (signed) */
    float   voltage_in;    /* input voltage, volts            */
} vesc_status5_t;

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

/**
 * Build a TWAI message that commands a VESC to apply a brake current.
 *
 * The VESC interprets this as a regenerative-brake current set-point:
 * while the rotor is turning it opposes motion proportionally to
 * `current_ma`; at standstill it draws ~0 A.  Combined with the VESC
 * `foc_short_ls_on_zero_duty` setting it provides an active hold
 * without requiring the speed PID to run.
 *
 * Bypasses the speed PID — use this instead of SET_RPM=0 when the
 * intent is "stop and hold", not "track zero velocity".
 *
 * @param vesc_id     Target VESC controller ID.
 * @param current_ma  Brake current in milliamps (always positive; VESC
 *                    derives the sign from rotor direction).  Clamped
 *                    by the VESC against `l_current_max` / `l_abs_max`.
 * @param out_msg     Filled with the ready-to-transmit TWAI message.
 */
void vesc_can_encode_current_brake(uint8_t vesc_id, int32_t current_ma,
                                   twai_message_t *out_msg);

/**
 * Build a TWAI message that pings a VESC.  The target VESC replies
 * with a PONG addressed to `sender_id`, payload = [target_vesc_id].
 *
 * @param target_vesc_id  VESC to ping.
 * @param sender_id       Our controller ID; the VESC sends the PONG
 *                        to (PONG << 8) | sender_id.
 *                        Normally VESC_CAN_SENDER_ID.
 * @param out_msg         Filled with the ready-to-transmit message.
 */
void vesc_can_encode_ping(uint8_t target_vesc_id, uint8_t sender_id,
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
 * Decode CAN_PACKET_STATUS_5 (cmd 27) from a received TWAI message.
 *
 * @param msg  Received message (caller should verify cmd == VESC_CAN_CMD_STATUS_5).
 * @param out  Decoded status fields.
 * @return     true on success, false if DLC is wrong.
 */
bool vesc_can_decode_status5(const twai_message_t *msg, vesc_status5_t *out);

/**
 * Decode CAN_PACKET_PONG.  The payload carries the responding VESC's
 * controller ID.  Caller should verify cmd == VESC_CAN_CMD_PONG and
 * that the lower 8 bits of the frame ID match our sender ID before
 * calling.
 *
 * @param msg            Received PONG frame (DLC must be >= 1).
 * @param responder_id   Populated with payload[0] — the VESC that
 *                       answered the ping.
 * @return               true on success, false if DLC is wrong.
 */
bool vesc_can_decode_pong(const twai_message_t *msg, uint8_t *responder_id);

#ifdef __cplusplus
}
#endif
