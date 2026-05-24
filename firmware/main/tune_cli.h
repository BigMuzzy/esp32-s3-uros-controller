/*
 * tune_cli.h — Tuning CLI: line parser, experiment engine, telemetry
 *
 * Drives motor-tuning experiments from a host script (or a terminal
 * such as picocom / netcat) and streams telemetry back as CSV. Speaks
 * over any `tune_transport_t` (UART0 today, WiFi TCP / BLE later).
 *
 * The CLI never writes VESC config (no MCCONF writes). It only
 * commands ERPM set-points via the existing CAN tx path and streams
 * measured telemetry back. Tune VESC PID in VESC Tool; evaluate the
 * gains here.
 *
 * Safety:
 *   - All motion commands require "enable" first.
 *   - The CAN override expires automatically if not refreshed in
 *     TUNE_OVERRIDE_TIMEOUT_MS (handled by can_task) — link loss
 *     (UART unplug, TCP disconnect, task crash) ⇒ motors stop.
 *   - RC failsafe and VESC health watchdog remain authoritative.
 *
 * Wire protocol (line-oriented, '\n' terminated, ASCII):
 *
 *   ──> commands sent by host:
 *     help                          show help
 *     ping                          → "R,pong"
 *     status                        → "R,mode=… armed=… vesc_ok=… …"
 *     enable                        arm tune mode (override active @ 0,0)
 *     disable                       drop override, idle
 *     stop                          alias for disable
 *     rpm <L|R|B> <erpm>            constant ERPM set-point
 *     step <L|R|B> <e0> <e1> <ms>   hold e0, switch to e1 after <ms>;
 *                                   stops after 2×ms
 *     chirp <L|R|B> <amp> <f0> <f1> <ms>
 *                                   linear-frequency sine chirp,
 *                                   amplitude=|amp| ERPM,
 *                                   f0..f1 Hz over <ms> ms
 *     log <hz>                      telemetry rate (0 = off, max 100)
 *
 *   <── lines emitted by firmware (always prefixed):
 *     R,<text>                       command reply
 *     OK                             command accepted
 *     ERR,<text>                     command rejected
 *     T,<t_us>,<tgtL>,<tgtR>,<erpmL>,<erpmR>,
 *       <iL>,<iR>,<dutyL>,<dutyR>,<vin>
 *                                    telemetry sample
 *     E,<text>                       async event (e.g. "expired")
 *
 *  ESP_LOG output is unmodified; host scripts should filter by the
 *  T,/R,/OK/ERR,/E, prefixes.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TUNE_CLI_TASK_STACK   4096
#define TUNE_CLI_TASK_PRIO    3       /* below CAN, above idle */

/* Maximum telemetry rate accepted by `log <hz>`. Sized so that at
 * 115200 baud UART, one ~90-byte CSV line per sample fits comfortably:
 *   100 Hz × 90 B = 9 kB/s, ~78 % of line capacity, leaves headroom
 *   for replies and ESP_LOG. */
#define TUNE_CLI_LOG_HZ_MAX   100

/* Pick the active transport (currently UART0; WiFi planned) and
 * start the CLI task. Returns ESP_OK on success. */
esp_err_t tune_cli_init(void);

#ifdef __cplusplus
}
#endif
