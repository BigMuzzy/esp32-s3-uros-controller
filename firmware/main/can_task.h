/*
 * can_task.h — CAN bus orchestration (Core 1)
 *
 * Owns the TWAI peripheral and coordinates all CAN traffic plus
 * the real-time control loop on Core 1:
 *
 *   can_tx_task (20 ms periodic)
 *   ├─ rc_failsafe_get_mode()
 *   ├─ if AUTONOMOUS:  read shared cmd_vel → diff_drive_cmd_vel_to_erpm()
 *   ├─ if MANUAL:       rc_failsafe_arcade_mix() → diff_drive_cmd_vel_to_erpm()
 *   ├─ if FAILSAFE_STOP: erpm = 0, 0
 *   └─ vesc_can_set_rpm(LEFT, ...) + vesc_can_set_rpm(RIGHT, ...)
 *
 *   can_rx_task (event-driven)
 *   ├─ receive TWAI frame
 *   ├─ vesc_can_parse_status() → update per-VESC status
 *   ├─ if both VESCs updated: diff_drive_update_odom()
 *   └─ write odom + status to shared structs (→ Core 0)
 *
 * Shared data between Core 0 and Core 1 is protected by spinlocks
 * (portMUX_TYPE) for low-latency, ISR-safe access.
 */

#pragma once

#include "esp_err.h"
#include "diff_drive.h"
#include "vesc_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ───────────────────────────────────────────────── */

#define CAN_TX_PERIOD_MS      20
#define CAN_TX_TASK_STACK     4096
#define CAN_RX_TASK_STACK     4096
#define CAN_TX_TASK_PRIO      5       /* higher than uros_task */
#define CAN_RX_TASK_PRIO      6       /* highest — don't miss frames */

/* VESC_ID_LEFT / VESC_ID_RIGHT are defined in vesc_can.h (included above). */

/* ── Health-check configuration (ADR-0009) ───────────────────────── */

/* Max time to wait at boot for both VESCs to broadcast STATUS + STATUS_4. */
#define VESC_HEALTH_BOOT_TIMEOUT_MS   1500

/* Plausibility window for VESC input voltage (volts). Values outside
 * this range at boot mark the VESC unhealthy. Tune for the target pack. */
#define VESC_VOLTAGE_MIN_V            18.0f
#define VESC_VOLTAGE_MAX_V            60.0f

/* Active ping (ADR-0009 Stage B).  Each attempt sends a PING to both
 * VESCs and waits up to VESC_PING_TIMEOUT_MS for both PONGs.  If either
 * pong is missing, retries VESC_PING_RETRIES times.  A successful ping
 * is authoritative for liveness even if periodic status broadcasts are
 * disabled. */
#define VESC_PING_TIMEOUT_MS          100
#define VESC_PING_RETRIES             2

/* Runtime watchdog (ADR-0009 Stage C).  If no STATUS frame arrives
 * from a given VESC for this long, the VESC is marked offline and
 * can_tx_task forces ERPM=0 regardless of drive mode.  At 50 Hz
 * broadcast (default), 200 ms = 10 missed frames. */
#define VESC_STATUS_TIMEOUT_MS        200

/* Active-brake current (mA) commanded when both wheels' target ERPM
 * is zero and the system is armed.  Acts as a regenerative brake
 * while the wheel is turning and draws ~0 A at standstill.  Tune
 * against `l_current_max` / `l_abs_current_max` in the VESC MCCONF;
 * value must be well below those limits.  8000 mA on a ~25 A phase /
 * 38 A absolute limit is ~32 % / ~21 % — firm hold, no fault risk. */
#define VESC_BRAKE_CURRENT_MA         8000

/* Hybrid stop threshold (|actual ERPM|).  When the target ERPM is zero
 * but the wheel is still rolling faster than this, can_tx_task commands
 * SET_RPM=0 instead of SET_CURRENT_BRAKE so the VESC speed PID can use
 * the full `l_current_max` (≈25 A) for sharp deceleration — much
 * stronger than the fixed `VESC_BRAKE_CURRENT_MA` regen brake.  Once
 * |actual ERPM| drops below this threshold the firmware switches to
 * SET_CURRENT_BRAKE for a clean, PID-free hold at standstill.
 *
 * 100 ERPM ≈ 14 rpm wheel ≈ 0.12 m/s with WHEEL_DIAMETER_M=0.160 — a
 * slow crawl; below that, 8 A brake current decelerates to 0 in a
 * single 20 ms tick. */
#define VESC_BRAKE_HOLD_ERPM_MAX      100

/* ── Types ───────────────────────────────────────────────────────── */

/**
 * Per-VESC health snapshot.  Populated by can_rx_task + can_tx_task
 * watchdog, read by any core.
 *
 * Fields reflect the most recent observed state:
 *   online         — true when boot check passed AND the runtime
 *                    watchdog has seen a STATUS frame within
 *                    VESC_STATUS_TIMEOUT_MS.  Cleared by the watchdog
 *                    on timeout; re-set when STATUS resumes (boot-time
 *                    failure is sticky until reboot).
 *   voltage_in     — last decoded STATUS_5 input voltage (volts).  0 if
 *                    no STATUS_5 has been received yet.
 *   fault_code     — reserved for STATUS_6 decode.  Always 0 at present.
 *   last_status_ms — esp_timer-based timestamp (ms) of the last STATUS
 *                    frame from this VESC.  0 if never seen.
 */
typedef struct {
    bool     online;
    float    voltage_in;
    uint8_t  fault_code;
    uint32_t last_status_ms;
} vesc_health_t;

/* ── Initialization ──────────────────────────────────────────────── */

/**
 * Initialize TWAI peripheral (500 kbit/s, CAN 2.0B extended frames)
 * and create can_tx_task + can_rx_task pinned to Core 1.
 *
 * Must be called after rc_failsafe_init() (failsafe is queried
 * by can_tx_task each cycle).
 *
 * @return ESP_OK on success, or an esp_err_t on TWAI/task failure.
 */
esp_err_t can_task_init(void);

/* ── Core 0 → Core 1: cmd_vel input ─────────────────────────────── */

/**
 * Set the latest cmd_vel from the micro-ROS subscription callback.
 * Called from uros_task (Core 0). Thread-safe (spinlock).
 *
 * Also calls rc_failsafe_notify_cmd_vel() to reset the timeout.
 */
void can_task_set_cmd_vel(const cmd_vel_t *cmd);

/* ── Core 1 → Core 0: odometry & status output ──────────────────── */

/**
 * Read the latest odometry state computed by can_rx_task.
 * Called from uros_task (Core 0) to populate the Odometry message.
 * Thread-safe (spinlock).
 */
void can_task_get_odom(odom_state_t *odom_out);

/**
 * Read the latest VESC status for a given VESC ID.
 * Called from uros_task (Core 0) for telemetry publishing.
 * Thread-safe (spinlock).
 *
 * @param vesc_id   VESC_ID_LEFT or VESC_ID_RIGHT.
 * @param status_out  Populated with latest status.
 * @return true if status has been received at least once.
 */
bool can_task_get_vesc_status(uint8_t vesc_id, vesc_status_t *status_out);

/**
 * Read the latest health snapshot for a given VESC ID.
 * Thread-safe (spinlock).
 *
 * @param vesc_id     VESC_ID_LEFT or VESC_ID_RIGHT.
 * @param health_out  Populated with the latest health state.
 * @return true if vesc_id is valid; false otherwise.
 */
bool can_task_get_vesc_health(uint8_t vesc_id, vesc_health_t *health_out);

#ifdef __cplusplus
}
#endif
