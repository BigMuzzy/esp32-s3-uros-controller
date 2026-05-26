/*
 * cia402.h — CiA 402 drive state-machine helpers
 *
 * Pure data-only helpers: statusword decoding and controlword
 * derivation.  No transport, no I/O.  Used by motor_driver_zlac8015d.c
 * to bring each axis from whatever state it powers up in (typically
 * "Switch on disabled") through to "Operation enabled".
 *
 * State diagram (CiA 402 §6.6 simplified)
 * ───────────────────────────────────────
 *      ┌──────────────┐
 *      │  Not Ready   │ (powered, no comm)
 *      └──────┬───────┘
 *             ▼ auto
 *      ┌──────────────┐ ◀──── any state via controlword "disable voltage"
 *      │ Switch On    │
 *      │   Disabled   │
 *      └──────┬───────┘
 *             ▼ shutdown (0x06)
 *      ┌──────────────┐
 *      │ Ready To     │
 *      │  Switch On   │
 *      └──────┬───────┘
 *             ▼ switch on (0x07)
 *      ┌──────────────┐
 *      │  Switched On │
 *      └──────┬───────┘
 *             ▼ enable operation (0x0F)
 *      ┌──────────────┐
 *      │  Operation   │
 *      │   Enabled    │
 *      └──────────────┘
 *
 * Faults: from any state, a fault transitions to "Fault Reaction
 * Active", then to "Fault".  Cleared with a rising edge on
 * controlword bit 7 (fault reset).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Statusword bit fields (object 0x6041) ──────────────────────── */

#define CIA402_SW_READY_TO_SWITCH_ON  (1u << 0)
#define CIA402_SW_SWITCHED_ON         (1u << 1)
#define CIA402_SW_OPERATION_ENABLED   (1u << 2)
#define CIA402_SW_FAULT               (1u << 3)
#define CIA402_SW_VOLTAGE_ENABLED     (1u << 4)
#define CIA402_SW_QUICK_STOP          (1u << 5)   /* 0 = active */
#define CIA402_SW_SWITCH_ON_DISABLED  (1u << 6)
#define CIA402_SW_WARNING             (1u << 7)
#define CIA402_SW_REMOTE              (1u << 9)
#define CIA402_SW_TARGET_REACHED      (1u << 10)

/* ── Controlword commands (object 0x6040) ───────────────────────── */

#define CIA402_CW_SHUTDOWN            0x0006u   /* → Ready To Switch On */
#define CIA402_CW_SWITCH_ON           0x0007u   /* → Switched On */
#define CIA402_CW_DISABLE_VOLTAGE     0x0000u   /* → Switch On Disabled */
#define CIA402_CW_QUICK_STOP          0x0002u   /* → Quick Stop Active */
#define CIA402_CW_DISABLE_OPERATION   0x0007u   /* OE → Switched On */
#define CIA402_CW_ENABLE_OPERATION    0x000Fu   /* SO → Operation Enabled */
#define CIA402_CW_FAULT_RESET         0x0080u   /* rising edge bit 7 */

/* ── Modes of operation (object 0x6060 / 0x6061) ────────────────── */

#define CIA402_MODE_PROFILE_POSITION   1
#define CIA402_MODE_PROFILE_VELOCITY   3
#define CIA402_MODE_PROFILE_TORQUE     4
#define CIA402_MODE_HOMING             6
#define CIA402_MODE_CYCLIC_SYNC_POS    8
#define CIA402_MODE_CYCLIC_SYNC_VEL    9
#define CIA402_MODE_CYCLIC_SYNC_TORQUE 10

/* ── Decoded state ──────────────────────────────────────────────── */

typedef enum {
    CIA402_STATE_UNKNOWN              = 0,
    CIA402_STATE_NOT_READY_TO_SWITCH_ON,
    CIA402_STATE_SWITCH_ON_DISABLED,
    CIA402_STATE_READY_TO_SWITCH_ON,
    CIA402_STATE_SWITCHED_ON,
    CIA402_STATE_OPERATION_ENABLED,
    CIA402_STATE_QUICK_STOP_ACTIVE,
    CIA402_STATE_FAULT_REACTION_ACTIVE,
    CIA402_STATE_FAULT,
} cia402_state_t;

/** Decode the CiA 402 state from a statusword reading. */
cia402_state_t cia402_decode_state(uint16_t statusword);

/** Human-readable name for logging. Never NULL. */
const char *cia402_state_name(cia402_state_t s);

/**
 * Compute the next controlword to send to drive `current_state`
 * toward `target_state`.  This is *one-step* — the caller is
 * expected to re-read the statusword each tick and call again until
 * `current == target`.
 *
 * `prev_cw_had_reset` should be the previous controlword's bit-7
 * value (fault-reset).  We toggle it to produce the required rising
 * edge when recovering from a FAULT state.
 *
 * Today only `CIA402_STATE_OPERATION_ENABLED` and
 * `CIA402_STATE_SWITCH_ON_DISABLED` are supported as targets — they
 * cover the run / safe-stop pair the motor backend needs.
 */
uint16_t cia402_next_controlword(cia402_state_t current_state,
                                 cia402_state_t target_state,
                                 bool prev_cw_had_reset);

#ifdef __cplusplus
}
#endif
