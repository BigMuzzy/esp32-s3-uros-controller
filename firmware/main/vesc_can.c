/*
 * vesc_can.c — VESC CAN protocol encode/decode
 *
 * VESC CAN frame format (CAN 2.0B extended, 29-bit ID):
 *   ID[28:8] = command type
 *   ID[7:0]  = VESC controller ID
 *   All multi-byte fields are big-endian (network byte order).
 */

#include "vesc_can.h"
#include <string.h>

/* ── Helpers ─────────────────────────────────────────────────────── */

/** Store a signed 32-bit integer as big-endian bytes. */
static void put_be32(uint8_t *buf, int32_t val)
{
    buf[0] = (uint8_t)(val >> 24);
    buf[1] = (uint8_t)(val >> 16);
    buf[2] = (uint8_t)(val >> 8);
    buf[3] = (uint8_t)(val);
}

/** Read a signed 32-bit integer from big-endian bytes. */
static int32_t get_be32(const uint8_t *buf)
{
    return (int32_t)(((uint32_t)buf[0] << 24) |
                     ((uint32_t)buf[1] << 16) |
                     ((uint32_t)buf[2] << 8)  |
                     ((uint32_t)buf[3]));
}

/** Read a signed 16-bit integer from big-endian bytes. */
static int16_t get_be16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

/* ── TX: command encoding ────────────────────────────────────────── */

void vesc_can_encode_rpm(uint8_t vesc_id, int32_t erpm,
                         twai_message_t *out_msg)
{
    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->extd = 1;
    out_msg->identifier = ((uint32_t)VESC_CAN_CMD_SET_RPM << 8) | vesc_id;
    out_msg->data_length_code = 4;
    put_be32(out_msg->data, erpm);
}

/* ── RX: status decoding ─────────────────────────────────────────── */

int vesc_can_get_cmd(const twai_message_t *msg, uint8_t *vesc_id)
{
    if (!msg->extd) {
        return -1;
    }
    *vesc_id = (uint8_t)(msg->identifier & 0xFF);
    return (int)(msg->identifier >> 8);
}

/*
 * CAN_PACKET_STATUS (cmd 9), 8 bytes:
 *   [0:3] ERPM          — int32, big-endian
 *   [4:5] current × 10  — int16, big-endian (amps × 10)
 *   [6:7] duty × 1000   — int16, big-endian
 */
bool vesc_can_decode_status(const twai_message_t *msg, vesc_status_t *out)
{
    if (msg->data_length_code < 8) {
        return false;
    }
    out->erpm          = get_be32(msg->data);
    out->current_motor = (float)get_be16(msg->data + 4) / 10.0f;
    out->duty_cycle    = (float)get_be16(msg->data + 6) / 1000.0f;
    return true;
}

/*
 * CAN_PACKET_STATUS_4 (cmd 14), 8 bytes:
 *   [0:3] tachometer    — int32, big-endian (cumulative ERPM ticks)
 *   [4:5] voltage × 10  — int16, big-endian (volts × 10)
 *   [6:7] reserved
 */
bool vesc_can_decode_status4(const twai_message_t *msg, vesc_status4_t *out)
{
    if (msg->data_length_code < 6) {
        return false;
    }
    out->tachometer = get_be32(msg->data);
    out->voltage_in = (float)get_be16(msg->data + 4) / 10.0f;
    return true;
}
