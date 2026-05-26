/*
 * cia402.c — CiA 402 state machine helpers
 *
 * State decoding follows the bit-pattern table in CiA 402 §6.6:
 *
 *   State                     | bits checked        | pattern
 *   ──────────────────────────┼─────────────────────┼──────────
 *   Not Ready to Switch On    | sw & 0x4F           | 0x00
 *   Switch On Disabled        | sw & 0x4F           | 0x40
 *   Ready to Switch On        | sw & 0x6F           | 0x21
 *   Switched On               | sw & 0x6F           | 0x23
 *   Operation Enabled         | sw & 0x6F           | 0x27
 *   Quick Stop Active         | sw & 0x6F           | 0x07
 *   Fault Reaction Active     | sw & 0x4F           | 0x0F
 *   Fault                     | sw & 0x4F           | 0x08
 */

#include "cia402.h"

cia402_state_t cia402_decode_state(uint16_t sw)
{
    /* Mask out higher bits (warning / remote / target-reached / etc) */
    if      ((sw & 0x4F) == 0x00) return CIA402_STATE_NOT_READY_TO_SWITCH_ON;
    else if ((sw & 0x4F) == 0x40) return CIA402_STATE_SWITCH_ON_DISABLED;
    else if ((sw & 0x6F) == 0x21) return CIA402_STATE_READY_TO_SWITCH_ON;
    else if ((sw & 0x6F) == 0x23) return CIA402_STATE_SWITCHED_ON;
    else if ((sw & 0x6F) == 0x27) return CIA402_STATE_OPERATION_ENABLED;
    else if ((sw & 0x6F) == 0x07) return CIA402_STATE_QUICK_STOP_ACTIVE;
    else if ((sw & 0x4F) == 0x0F) return CIA402_STATE_FAULT_REACTION_ACTIVE;
    else if ((sw & 0x4F) == 0x08) return CIA402_STATE_FAULT;
    return CIA402_STATE_UNKNOWN;
}

const char *cia402_state_name(cia402_state_t s)
{
    switch (s) {
    case CIA402_STATE_NOT_READY_TO_SWITCH_ON:  return "NotReady";
    case CIA402_STATE_SWITCH_ON_DISABLED:      return "SwitchOnDisabled";
    case CIA402_STATE_READY_TO_SWITCH_ON:      return "ReadyToSwitchOn";
    case CIA402_STATE_SWITCHED_ON:             return "SwitchedOn";
    case CIA402_STATE_OPERATION_ENABLED:       return "OperationEnabled";
    case CIA402_STATE_QUICK_STOP_ACTIVE:       return "QuickStopActive";
    case CIA402_STATE_FAULT_REACTION_ACTIVE:   return "FaultReactionActive";
    case CIA402_STATE_FAULT:                   return "Fault";
    default:                                   return "Unknown";
    }
}

uint16_t cia402_next_controlword(cia402_state_t current,
                                 cia402_state_t target,
                                 bool prev_cw_had_reset)
{
    /* Fault recovery is mode-independent: always route via fault
     * reset (rising edge on bit 7) to reach Switch On Disabled. */
    if (current == CIA402_STATE_FAULT) {
        return prev_cw_had_reset ? CIA402_CW_DISABLE_VOLTAGE
                                 : CIA402_CW_FAULT_RESET;
    }
    if (current == CIA402_STATE_FAULT_REACTION_ACTIVE) {
        /* Wait; the drive will transition to Fault on its own. */
        return CIA402_CW_DISABLE_VOLTAGE;
    }

    if (target == CIA402_STATE_OPERATION_ENABLED) {
        switch (current) {
        case CIA402_STATE_NOT_READY_TO_SWITCH_ON: return CIA402_CW_DISABLE_VOLTAGE; /* auto */
        case CIA402_STATE_SWITCH_ON_DISABLED:     return CIA402_CW_SHUTDOWN;
        case CIA402_STATE_READY_TO_SWITCH_ON:     return CIA402_CW_SWITCH_ON;
        case CIA402_STATE_SWITCHED_ON:            return CIA402_CW_ENABLE_OPERATION;
        case CIA402_STATE_OPERATION_ENABLED:      return CIA402_CW_ENABLE_OPERATION;
        case CIA402_STATE_QUICK_STOP_ACTIVE:      return CIA402_CW_DISABLE_VOLTAGE;
        default:                                  return CIA402_CW_DISABLE_VOLTAGE;
        }
    }

    if (target == CIA402_STATE_SWITCH_ON_DISABLED) {
        return CIA402_CW_DISABLE_VOLTAGE;
    }

    /* Unsupported target — fail safe. */
    return CIA402_CW_DISABLE_VOLTAGE;
}
