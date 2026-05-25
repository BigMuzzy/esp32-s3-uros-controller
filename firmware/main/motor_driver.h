/*
 * motor_driver.h — Hardware-agnostic motor driver interface
 *
 * Abstraction layer between the upper control loop (cmd_vel,
 * failsafe, tune override, odometry) and the underlying motor
 * controller hardware.  Lets the same firmware target multiple
 * drives (VESC over custom CAN, ZLAC8015D over CANopen, …) chosen
 * at build time via `CONFIG_MOTOR_DRIVER_*`.
 *
 * Design rules
 * ────────────
 * 1. Wheel-side units only.  All command/feedback quantities are
 *    expressed at the wheel — RPM (revolutions/min), revolutions
 *    (cumulative, signed), amperes, volts, meters.  Backend modules
 *    own any conversion to motor-electrical / encoder-count units.
 * 2. The HAL is a *device* interface, not a control task.  It
 *    exposes synchronous setters/getters and runs whatever
 *    background I/O it needs internally.  The 20 ms control loop
 *    (failsafe coordination, tune override, odometry update) lives
 *    outside this module.
 * 3. Two-channel diff-drive is baked into the contract — every call
 *    operates on a left+right pair.  Backends with a single
 *    multi-channel controller (ZLAC8015D) or two separate
 *    controllers (VESC) hide that detail.
 * 4. Thread-safety: setters and getters MUST be safe to call from
 *    any task / any core.  Internally either lock-free atomics or
 *    spinlocks; never blocking primitives that would stall the
 *    control loop.
 * 5. Hot path is statically allocated — no `malloc` after init.
 *
 * Backend selection
 * ─────────────────
 * Exactly one of:
 *   CONFIG_MOTOR_DRIVER_VESC       — 2× VESC over custom CAN
 *   CONFIG_MOTOR_DRIVER_ZLAC8015D  — 1× ZLAC8015D over CANopen
 *
 * Backend-specific configuration (CAN bitrate, node IDs, pole
 * pairs, encoder CPR, …) lives in the backend's own Kconfig menu;
 * the upper layer never sees it.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Types ───────────────────────────────────────────────────────── */

/**
 * Per-wheel velocity command, in wheel revolutions per minute.
 *
 * Sign convention: positive = forward.  Backends are expected to
 * clamp to their own physical/configured limits and silently
 * saturate (never wrap, never abort).
 */
typedef struct {
    float left_rpm;
    float right_rpm;
} motor_wheel_cmd_t;

/**
 * Motor-controller-detected fault categories.  Bitfield so multiple
 * conditions can be reported simultaneously.  Backends map their
 * native fault codes onto these buckets; the raw code is also kept
 * in `motor_feedback_t.fault_code_raw` for diagnostics.
 */
enum {
    MOTOR_FAULT_NONE          = 0,
    MOTOR_FAULT_OVERCURRENT   = 1u << 0,
    MOTOR_FAULT_OVERVOLTAGE   = 1u << 1,
    MOTOR_FAULT_UNDERVOLTAGE  = 1u << 2,
    MOTOR_FAULT_OVERTEMP      = 1u << 3,
    MOTOR_FAULT_ENCODER       = 1u << 4,
    MOTOR_FAULT_COMMUNICATION = 1u << 5,
    MOTOR_FAULT_OTHER         = 1u << 7,
};

/**
 * Per-wheel feedback snapshot.  Populated by the backend's RX path,
 * read by the upper layer for odometry + telemetry.
 *
 *  rpm                  — instantaneous wheel velocity (RPM, signed).
 *  revolutions          — cumulative wheel revolutions (signed,
 *                          float to handle long missions without
 *                          int wrap; resolution depends on backend
 *                          encoder).
 *  current_a            — phase current (amps).  Sign is backend-
 *                          defined; treat as magnitude for telemetry.
 *  fault_code_raw       — vendor-specific raw fault code, 0 = none.
 *  fault_bits           — decoded MOTOR_FAULT_* bitmask.
 */
typedef struct {
    float    rpm;
    float    revolutions;
    float    current_a;
    uint32_t fault_code_raw;
    uint32_t fault_bits;
} motor_wheel_feedback_t;

/**
 * Combined feedback for both wheels plus shared bus state.
 *
 *  bus_voltage_v   — DC bus voltage as reported by the controller.
 *                    0.0f if not yet observed.
 *  last_update_ms  — esp_timer-based timestamp (ms) of the most
 *                    recent feedback update from the hardware.  Used
 *                    by the upper layer to detect staleness.
 */
typedef struct {
    motor_wheel_feedback_t left;
    motor_wheel_feedback_t right;
    float    bus_voltage_v;
    uint32_t last_update_ms;
} motor_feedback_t;

/**
 * Liveness / health snapshot.
 *
 *  online       — true when the controller is communicating AND
 *                 (backend-defined) sanity checks pass.  Goes false
 *                 on comms timeout, bad bus voltage, etc.; goes back
 *                 true automatically when conditions clear, gated
 *                 by `boot_passed`.
 *  boot_passed  — sticky; true once the boot health check succeeded.
 *                 Never re-asserted after init failed — a bad boot
 *                 requires reboot.  The runtime online flag can
 *                 only re-arm if boot_passed is true.
 *  fault_bits   — OR of left/right fault bits + global controller
 *                 faults.
 */
typedef struct {
    bool     online;
    bool     boot_passed;
    uint32_t fault_bits;
} motor_health_t;

/* ── Lifecycle ───────────────────────────────────────────────────── */

/**
 * Initialise the motor-driver backend.  Brings up the underlying
 * transport (TWAI, RS485, …), performs the boot health check
 * (probe both wheels, verify bus voltage), and starts whatever
 * background tasks the backend needs.
 *
 * Must be called exactly once, before any other motor_driver_* call.
 *
 * On success the backend is in "armed" state — `motor_driver_set_cmd`
 * will produce motion.  On failure the backend stays disarmed:
 * `motor_health_t.boot_passed` remains false, and `set_cmd` is a
 * silent no-op until a reboot.
 *
 * @return ESP_OK on success; an `esp_err_t` describing the failure
 *         otherwise.  A failed init still leaves the module in a
 *         queryable state — callers may proceed to publish health
 *         telemetry indicating the drive is unavailable.
 */
esp_err_t motor_driver_init(void);

/**
 * Whether the backend completed init without error.  Equivalent to
 * `motor_driver_get_health().boot_passed`.
 */
bool motor_driver_is_armed(void);

/* ── Control ─────────────────────────────────────────────────────── */

/**
 * Send a new wheel-velocity setpoint.  The backend transmits this to
 * the controller at its native cadence; multiple `set_cmd` calls
 * between transmissions coalesce to the latest value.
 *
 * The HAL imposes no rate limit, but the upper control loop should
 * call this at a steady cadence (≥ 50 Hz) to keep watchdogs satisfied.
 *
 * Safe to call from any task / any core.  When the backend is not
 * armed, this call is a no-op.
 */
void motor_driver_set_cmd(const motor_wheel_cmd_t *cmd);

/**
 * Engage an immediate stop.  Equivalent to commanding zero velocity
 * plus, on backends that support it, an active braking current.
 * Used by the failsafe path and the boot sequence.
 *
 * Safe to call before `motor_driver_init` (no-op) and from interrupt
 * context (best-effort — backend may defer the actual TX to its task).
 */
void motor_driver_emergency_stop(void);

/* ── Feedback ────────────────────────────────────────────────────── */

/**
 * Snapshot current motor feedback (both wheels + bus state).  Always
 * returns the most recently observed values; the caller can detect
 * staleness via `last_update_ms`.  Never blocks.
 *
 * @return true if feedback has been received at least once since
 *         boot; false if no data is available yet (in which case
 *         the output struct is zeroed).
 */
bool motor_driver_get_feedback(motor_feedback_t *fb_out);

/**
 * Snapshot the current health/liveness state.  Never blocks.
 *
 * @return true on success.  The output struct is always populated
 *         (zeroed if the backend has not initialised yet).
 */
bool motor_driver_get_health(motor_health_t *health_out);

#ifdef __cplusplus
}
#endif
