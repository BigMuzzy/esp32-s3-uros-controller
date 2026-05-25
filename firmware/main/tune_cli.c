/*
 * tune_cli.c — Tuning CLI: line parser, experiment engine, telemetry
 *
 * One FreeRTOS task pinned to Core 0:
 *   - reads bytes from the configured transport into a line buffer,
 *   - parses commands and updates experiment state,
 *   - each tick (~5 ms): refreshes the motor_task tune override if an
 *     experiment is active,
 *   - emits CSV telemetry at the configured rate.
 *
 * No motion ever leaves this task without going through
 * `motor_task_set_tune_override()`, which is gated by RC failsafe +
 * VESC health watchdog + a 150 ms refresh timeout in motor_task.c.
 */

#include "tune_cli.h"
#include "tune_transport.h"
#include "motor_task.h"
#include "motor_driver_vesc.h"

/* Pole-pair count for the VESC backend; used to translate the wire-
 * protocol ERPM units exposed to the host tuner into wheel RPM
 * understood by motor_task_set_tune_override().  Keep in sync with
 * MOTOR_POLE_PAIRS inside motor_driver_vesc.c. */
#define TUNE_VESC_POLE_PAIRS  7

static inline float erpm_to_wheel_rpm_f(int32_t erpm)
{
    return (float)erpm / (float)TUNE_VESC_POLE_PAIRS;
}
#include "rc_failsafe.h"
#include "vesc_can.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "tune_cli";

/* ── Constants ───────────────────────────────────────────────────── */

#define LINE_BUF_MAX        160
#define OUT_BUF_MAX         192
#define TICK_PERIOD_MS      5      /* experiment-update tick (clamped to >=1 RTOS tick) */

/* Tune-CLI ERPM ceiling. Deliberately decoupled from MAX_WHEEL_RPM
 * (which is the autonomous-mode safety cap, wheel side) so we can
 * sweep across the motor's useful operating range during tuning
 * without raising the autopilot's speed limit. Stays well below the
 * mcconf l_max_erpm so the VESC's own limiter is never the gate. */
#define ERPM_LIMIT          8000

/* pdMS_TO_TICKS() rounds down: at the default 100 Hz tick rate,
 * pdMS_TO_TICKS(5) == 0, which makes xTaskDelayUntil() assert.
 * Floor to one tick so the loop always advances. */
#define MIN_DELAY_TICKS(ms_) \
    (pdMS_TO_TICKS(ms_) > 0 ? pdMS_TO_TICKS(ms_) : (TickType_t)1)

#define SIDE_L   0x1u
#define SIDE_R   0x2u
#define SIDE_B   (SIDE_L | SIDE_R)

/* ── Experiment state ────────────────────────────────────────────── */

typedef enum {
    EXP_NONE = 0,
    EXP_CONST,
    EXP_STEP,
    EXP_CHIRP,
} exp_type_t;

typedef struct {
    exp_type_t type;
    uint8_t    side_mask;   /* SIDE_L | SIDE_R */
    int64_t    t0_us;
    int32_t    erpm0;
    int32_t    erpm1;
    int64_t    dur_us;      /* total duration */
    int64_t    step_us;     /* step transition time (STEP) */
    float      amp;
    float      f0_hz;
    float      f1_hz;
} experiment_t;

/* ── CLI state ───────────────────────────────────────────────────── */

static tune_transport_t *s_xport;
static bool       s_enabled;        /* "enable" issued, override active */
static experiment_t s_exp;          /* current experiment */
static uint32_t   s_log_period_us;  /* 0 = off */
static int64_t    s_next_log_us;

/* ── Output helpers ──────────────────────────────────────────────── */

static void out_raw(const char *s, size_t n)
{
    if (s_xport) s_xport->write(s_xport, (const uint8_t *)s, n);
}

static void out_line(const char *s)
{
    size_t n = strlen(s);
    out_raw(s, n);
    out_raw("\r\n", 2);
}

static void out_printf(const char *fmt, ...)
{
    char buf[OUT_BUF_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    out_raw(buf, (size_t)n);
    out_raw("\r\n", 2);
}

static void out_ok(void)         { out_line("OK"); }
static void out_err(const char *m){ out_printf("ERR,%s", m); }

/* ── Helpers ─────────────────────────────────────────────────────── */

static int32_t clamp_erpm(int32_t v)
{
    if (v >  ERPM_LIMIT) return  ERPM_LIMIT;
    if (v < -ERPM_LIMIT) return -ERPM_LIMIT;
    return v;
}

/* Parse "L", "R", or "B" (case-insensitive). Returns 0 on failure. */
static uint8_t parse_side(const char *s)
{
    if (!s || !s[0] || s[1] != '\0') return 0;
    switch (s[0]) {
    case 'l': case 'L': return SIDE_L;
    case 'r': case 'R': return SIDE_R;
    case 'b': case 'B': return SIDE_B;
    default: return 0;
    }
}

static const char *mode_name(drive_mode_t m)
{
    switch (m) {
    case DRIVE_MODE_AUTONOMOUS:    return "AUTO";
    case DRIVE_MODE_MANUAL:        return "MANUAL";
    case DRIVE_MODE_FAILSAFE_STOP: return "FAILSAFE";
    default:                       return "?";
    }
}

/* ── Experiment evaluation ───────────────────────────────────────── */

/* Compute per-wheel target ERPM for an active experiment at `now_us`.
 * Returns false when the experiment is done (caller should stop). */
static bool exp_eval(const experiment_t *e, int64_t now_us,
                     int32_t *left_out, int32_t *right_out)
{
    int64_t dt_us = now_us - e->t0_us;
    int32_t target = 0;

    switch (e->type) {
    case EXP_NONE:
        *left_out = 0;
        *right_out = 0;
        return false;

    case EXP_CONST:
        target = e->erpm0;
        break;

    case EXP_STEP:
        if (dt_us < 0) {
            target = e->erpm0;
        } else if (dt_us < e->step_us) {
            target = e->erpm0;
        } else if (dt_us < e->dur_us) {
            target = e->erpm1;
        } else {
            *left_out = 0;
            *right_out = 0;
            return false;
        }
        break;

    case EXP_CHIRP: {
        if (dt_us < 0 || dt_us >= e->dur_us) {
            *left_out = 0;
            *right_out = 0;
            return false;
        }
        /* Linear frequency sweep, phase = 2π·(f0·t + (f1-f0)/(2·T)·t²) */
        double t = (double)dt_us * 1e-6;
        double T = (double)e->dur_us * 1e-6;
        double k = (e->f1_hz - e->f0_hz) / T;
        double phase = 2.0 * M_PI * (e->f0_hz * t + 0.5 * k * t * t);
        double v = (double)e->amp * sin(phase);
        target = (int32_t)lrint(v);
        break;
    }
    }

    target = clamp_erpm(target);
    *left_out  = (e->side_mask & SIDE_L) ? target : 0;
    *right_out = (e->side_mask & SIDE_R) ? target : 0;
    return true;
}

/* ── Command handlers ────────────────────────────────────────────── */

static void cmd_help(void)
{
    out_line("R,commands: help ping status enable disable stop");
    out_line("R,  rpm <L|R|B> <erpm>");
    out_line("R,  step <L|R|B> <e0> <e1> <ms>");
    out_line("R,  chirp <L|R|B> <amp> <f0_hz> <f1_hz> <ms>");
    out_line("R,  log <hz>   (0..100)");
    out_line("R,  quiet <on|off>   mute motor task ESP_LOG spam");
    out_line("R,  gains set <L|R|B> [kp=.. ki=.. kd=.. ramp=..]   RAM-only");
    out_line("R,  gains save <L|R|B>   commit RAM mcconf -> flash (needs enable)");
    out_line("R,  term <L|R|B> <vesc-terminal-cmd>   raw passthrough");
}

static void cmd_quiet(char *args)
{
    char *v = strtok(args, " \t");
    if (!v) { out_err("usage: quiet <on|off>"); return; }
    if (!strcmp(v, "on")) {
        esp_log_level_set("motor_vesc", ESP_LOG_WARN);
        esp_log_level_set("motor_task", ESP_LOG_WARN);
        out_ok();
    } else if (!strcmp(v, "off")) {
        esp_log_level_set("motor_vesc", ESP_LOG_INFO);
        esp_log_level_set("motor_task", ESP_LOG_INFO);
        out_ok();
    } else {
        out_err("usage: quiet <on|off>");
    }
}

static void cmd_status(void)
{
    drive_mode_t mode = rc_failsafe_get_mode();

    vesc_health_t hL = {0}, hR = {0};
    motor_driver_vesc_get_health(VESC_ID_LEFT,  &hL);
    motor_driver_vesc_get_health(VESC_ID_RIGHT, &hR);
    bool vesc_ok = hL.online && hR.online;

    vesc_status_t sL = {0}, sR = {0};
    motor_driver_vesc_get_status(VESC_ID_LEFT,  &sL);
    motor_driver_vesc_get_status(VESC_ID_RIGHT, &sR);

    out_printf("R,mode=%s enabled=%d vesc_ok=%d "
               "vinL=%.1f vinR=%.1f erpmL=%" PRId32 " erpmR=%" PRId32
               " exp=%d log_hz=%" PRIu32,
               mode_name(mode), s_enabled, vesc_ok,
               (double)hL.voltage_in, (double)hR.voltage_in,
               sL.erpm, sR.erpm, (int)s_exp.type,
               s_log_period_us ? (uint32_t)(1000000U / s_log_period_us) : 0U);
}

static void cmd_enable(void)
{
    s_enabled = true;
    s_exp.type = EXP_CONST;
    s_exp.side_mask = SIDE_B;
    s_exp.erpm0 = 0;
    s_exp.t0_us = esp_timer_get_time();
    motor_task_set_tune_override(0.0f, 0.0f);
    out_ok();
}

static void cmd_disable(void)
{
    s_enabled = false;
    s_exp.type = EXP_NONE;
    motor_task_clear_tune_override();
    out_ok();
}

/* "rpm <side> <erpm>" */
static void cmd_rpm(char *args)
{
    if (!s_enabled) { out_err("not_enabled"); return; }
    char *side_s = strtok(args, " \t");
    char *erpm_s = strtok(NULL, " \t");
    if (!side_s || !erpm_s) { out_err("usage: rpm <L|R|B> <erpm>"); return; }
    uint8_t side = parse_side(side_s);
    if (!side) { out_err("bad side"); return; }
    int32_t erpm = clamp_erpm((int32_t)strtol(erpm_s, NULL, 10));

    s_exp.type = EXP_CONST;
    s_exp.side_mask = side;
    s_exp.erpm0 = erpm;
    s_exp.t0_us = esp_timer_get_time();
    out_ok();
}

/* "step <side> <e0> <e1> <ms>" */
static void cmd_step(char *args)
{
    if (!s_enabled) { out_err("not_enabled"); return; }
    char *side_s = strtok(args, " \t");
    char *e0_s   = strtok(NULL, " \t");
    char *e1_s   = strtok(NULL, " \t");
    char *ms_s   = strtok(NULL, " \t");
    if (!side_s || !e0_s || !e1_s || !ms_s) {
        out_err("usage: step <L|R|B> <e0> <e1> <ms>");
        return;
    }
    uint8_t side = parse_side(side_s);
    if (!side) { out_err("bad side"); return; }
    int32_t e0 = clamp_erpm((int32_t)strtol(e0_s, NULL, 10));
    int32_t e1 = clamp_erpm((int32_t)strtol(e1_s, NULL, 10));
    int32_t ms = (int32_t)strtol(ms_s, NULL, 10);
    if (ms < 10 || ms > 60000) { out_err("ms out of range [10,60000]"); return; }

    s_exp.type = EXP_STEP;
    s_exp.side_mask = side;
    s_exp.erpm0 = e0;
    s_exp.erpm1 = e1;
    s_exp.step_us = (int64_t)ms * 1000;
    s_exp.dur_us  = 2 * s_exp.step_us;
    s_exp.t0_us   = esp_timer_get_time();
    out_ok();
}

/* "chirp <side> <amp> <f0> <f1> <ms>" */
static void cmd_chirp(char *args)
{
    if (!s_enabled) { out_err("not_enabled"); return; }
    char *side_s = strtok(args, " \t");
    char *amp_s  = strtok(NULL, " \t");
    char *f0_s   = strtok(NULL, " \t");
    char *f1_s   = strtok(NULL, " \t");
    char *ms_s   = strtok(NULL, " \t");
    if (!side_s || !amp_s || !f0_s || !f1_s || !ms_s) {
        out_err("usage: chirp <L|R|B> <amp> <f0> <f1> <ms>");
        return;
    }
    uint8_t side = parse_side(side_s);
    if (!side) { out_err("bad side"); return; }
    float amp = strtof(amp_s, NULL);
    float f0  = strtof(f0_s,  NULL);
    float f1  = strtof(f1_s,  NULL);
    int32_t ms = (int32_t)strtol(ms_s, NULL, 10);
    if (!(amp > 0.0f) || amp > (float)ERPM_LIMIT) {
        out_err("amp out of range (0, ERPM_LIMIT]"); return;
    }
    if (!(f0 >= 0.0f) || !(f1 >= 0.0f) || f0 > 50.0f || f1 > 50.0f) {
        out_err("freq out of range [0,50] Hz"); return;
    }
    if (ms < 100 || ms > 60000) { out_err("ms out of range [100,60000]"); return; }

    s_exp.type = EXP_CHIRP;
    s_exp.side_mask = side;
    s_exp.amp = amp;
    s_exp.f0_hz = f0;
    s_exp.f1_hz = f1;
    s_exp.dur_us = (int64_t)ms * 1000;
    s_exp.t0_us  = esp_timer_get_time();
    out_ok();
}

static void cmd_log(char *args)
{
    char *hz_s = strtok(args, " \t");
    if (!hz_s) { out_err("usage: log <hz>"); return; }
    int hz = (int)strtol(hz_s, NULL, 10);
    if (hz < 0 || hz > TUNE_CLI_LOG_HZ_MAX) {
        out_err("hz out of range");
        return;
    }
    s_log_period_us = (hz == 0) ? 0 : (uint32_t)(1000000 / hz);
    s_next_log_us = esp_timer_get_time();
    out_ok();
}

/* ── VESC terminal-command tunnel ────────────────────────────────── */

/* Send one terminal command string to the VESCs selected by `mask`.
 * Returns ESP_OK only if both sends (where applicable) succeed. */
static esp_err_t send_term_to_sides(uint8_t mask, const char *term)
{
    esp_err_t err = ESP_OK;
    if (mask & SIDE_L) {
        esp_err_t e = vesc_can_send_terminal_cmd(VESC_ID_LEFT, term,
                                                 pdMS_TO_TICKS(50));
        if (e != ESP_OK) err = e;
    }
    if (mask & SIDE_R) {
        esp_err_t e = vesc_can_send_terminal_cmd(VESC_ID_RIGHT, term,
                                                 pdMS_TO_TICKS(50));
        if (e != ESP_OK && err == ESP_OK) err = e;
    }
    /* Small pause: gives the VESC time to parse + apply before the
     * next command lands.  Terminal parsing is not the bus-bottleneck;
     * leaving 5 ms here is plenty. */
    vTaskDelay(MIN_DELAY_TICKS(5));
    return err;
}

/* `term <L|R|B> <free-text>` — passthrough escape hatch.  Sends the
 * remainder of the line verbatim as a VESC terminal command.  Useful
 * for one-off diagnostics ("faults", "kv", "tacho", "param_detect …")
 * without baking each into firmware. */
static void cmd_term(char *args)
{
    char *side_s = strtok(args, " \t");
    char *rest   = strtok(NULL, "");
    if (!side_s || !rest) {
        out_err("usage: term <L|R|B> <terminal-cmd>");
        return;
    }
    /* Trim leading whitespace on rest. */
    while (*rest == ' ' || *rest == '\t') rest++;
    if (*rest == '\0') {
        out_err("empty terminal command");
        return;
    }
    uint8_t mask = parse_side(side_s);
    if (!mask) { out_err("side must be L, R, or B"); return; }

    esp_err_t err = send_term_to_sides(mask, rest);
    if (err != ESP_OK) {
        out_printf("ERR,term twai_transmit=%d", (int)err);
        return;
    }
    out_printf("R,term %c sent: %s",
               (mask == SIDE_L) ? 'L' : (mask == SIDE_R) ? 'R' : 'B', rest);
    out_ok();
}

/* `gains set|save <L|R|B> [kp=.. ki=.. kd=.. ramp=..]`
 *
 * Wraps VESC's `set_mcconf_param` terminal command.  Writes go to
 * RAM (volatile) — survive until next VESC reset.  `gains save`
 * issues `mcconf_store` to commit the current RAM mcconf to flash;
 * gated behind `enable` to avoid accidental flash writes.
 *
 * Fields are VESC mcconf names:
 *   kp   -> s_pid_kp
 *   ki   -> s_pid_ki
 *   kd   -> s_pid_kd
 *   ramp -> s_pid_ramp_erpms_s
 */
static void cmd_gains(char *args)
{
    char *sub = strtok(args, " \t");
    if (!sub) {
        out_err("usage: gains set|save <L|R|B> [kp=.. ki=.. kd=.. ramp=..]");
        return;
    }

    if (!strcmp(sub, "save")) {
        char *side_s = strtok(NULL, " \t");
        if (!side_s) { out_err("usage: gains save <L|R|B>"); return; }
        uint8_t mask = parse_side(side_s);
        if (!mask) { out_err("side must be L, R, or B"); return; }
        if (!s_enabled) {
            out_err("gains save requires 'enable' (flash write)");
            return;
        }
        esp_err_t err = send_term_to_sides(mask, "mcconf_store");
        if (err != ESP_OK) {
            out_printf("ERR,gains save twai_transmit=%d", (int)err);
            return;
        }
        out_line("R,gains save: mcconf_store sent (RAM->flash)");
        out_ok();
        return;
    }

    if (strcmp(sub, "set") != 0) {
        out_err("usage: gains set|save <L|R|B> ...");
        return;
    }

    char *side_s = strtok(NULL, " \t");
    if (!side_s) { out_err("usage: gains set <L|R|B> kp=.. ..."); return; }
    uint8_t mask = parse_side(side_s);
    if (!mask) { out_err("side must be L, R, or B"); return; }

    struct {
        const char *key;
        const char *vesc_param;
        float       val;
        bool        set;
    } params[] = {
        { "kp",   "s_pid_kp",            0.0f, false },
        { "ki",   "s_pid_ki",            0.0f, false },
        { "kd",   "s_pid_kd",            0.0f, false },
        { "ramp", "s_pid_ramp_erpms_s",  0.0f, false },
    };
    const size_t NPARAM = sizeof(params) / sizeof(params[0]);

    char *kv;
    while ((kv = strtok(NULL, " \t")) != NULL) {
        char *eq = strchr(kv, '=');
        if (!eq) { out_printf("ERR,bad arg: %s", kv); return; }
        *eq = '\0';
        const char *key = kv;
        const char *val_s = eq + 1;
        char *endp = NULL;
        float val = strtof(val_s, &endp);
        if (endp == val_s) { out_printf("ERR,bad value: %s", val_s); return; }

        bool found = false;
        for (size_t i = 0; i < NPARAM; i++) {
            if (!strcmp(key, params[i].key)) {
                params[i].val = val;
                params[i].set = true;
                found = true;
                break;
            }
        }
        if (!found) { out_printf("ERR,unknown key: %s", key); return; }
    }

    int sent = 0;
    for (size_t i = 0; i < NPARAM; i++) {
        if (!params[i].set) continue;
        char term[80];
        int n = snprintf(term, sizeof(term),
                         "set_mcconf_param %s %g",
                         params[i].vesc_param, (double)params[i].val);
        if (n < 0 || (size_t)n >= sizeof(term)) {
            out_err("terminal command too long");
            return;
        }
        esp_err_t err = send_term_to_sides(mask, term);
        if (err != ESP_OK) {
            out_printf("ERR,gains set twai_transmit=%d (param=%s)",
                       (int)err, params[i].vesc_param);
            return;
        }
        out_printf("R,sent: %s", term);
        sent++;
    }
    if (sent == 0) {
        out_err("no gains specified (need kp=.. ki=.. kd=.. or ramp=..)");
        return;
    }
    out_line("R,note: changes are RAM-only; use 'gains save' to persist");
    out_ok();
}

/* ── Line dispatch ───────────────────────────────────────────────── */

static void dispatch(char *line)
{
    /* Trim leading whitespace */
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0' || *line == '#') return;   /* empty or comment */

    char *cmd = strtok(line, " \t");
    char *rest = strtok(NULL, "");   /* remainder of line */
    if (!cmd) return;

    /* lowercase the command word in place */
    for (char *p = cmd; *p; p++) *p = (char)tolower((unsigned char)*p);

    if      (!strcmp(cmd, "help"))    cmd_help();
    else if (!strcmp(cmd, "ping"))    out_line("R,pong");
    else if (!strcmp(cmd, "status"))  cmd_status();
    else if (!strcmp(cmd, "enable"))  cmd_enable();
    else if (!strcmp(cmd, "disable")) cmd_disable();
    else if (!strcmp(cmd, "stop"))    cmd_disable();
    else if (!strcmp(cmd, "rpm"))     cmd_rpm(rest ? rest : "");
    else if (!strcmp(cmd, "step"))    cmd_step(rest ? rest : "");
    else if (!strcmp(cmd, "chirp"))   cmd_chirp(rest ? rest : "");
    else if (!strcmp(cmd, "log"))     cmd_log(rest ? rest : "");
    else if (!strcmp(cmd, "quiet"))   cmd_quiet(rest ? rest : "");
    else if (!strcmp(cmd, "term"))    cmd_term(rest ? rest : "");
    else if (!strcmp(cmd, "gains"))   cmd_gains(rest ? rest : "");
    else                              out_err("unknown command");
}

/* ── Telemetry ───────────────────────────────────────────────────── */

static void emit_telemetry(int64_t now_us,
                           int32_t tgtL, int32_t tgtR)
{
    vesc_status_t sL = {0}, sR = {0};
    motor_driver_vesc_get_status(VESC_ID_LEFT,  &sL);
    motor_driver_vesc_get_status(VESC_ID_RIGHT, &sR);

    vesc_health_t hL = {0};
    motor_driver_vesc_get_health(VESC_ID_LEFT, &hL);

    out_printf("T,%lld,%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
               ",%.2f,%.2f,%.3f,%.3f,%.2f",
               (long long)now_us, tgtL, tgtR, sL.erpm, sR.erpm,
               (double)sL.current_motor, (double)sR.current_motor,
               (double)sL.duty_cycle,    (double)sR.duty_cycle,
               (double)hL.voltage_in);
}

/* ── Main task ───────────────────────────────────────────────────── */

static void tune_cli_task(void *arg)
{
    (void)arg;

    /* Banner so a fresh terminal can confirm the link is up. */
    out_line("R,tune_cli ready; type 'help'");

    char    line_buf[LINE_BUF_MAX];
    size_t  line_len = 0;
    uint8_t rx_buf[64];

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* 1. Drain any pending RX (short timeout — keeps us at TICK_PERIOD_MS). */
        int n = s_xport->read(s_xport, rx_buf, sizeof(rx_buf),
                              TICK_PERIOD_MS);
        for (int i = 0; i < n; i++) {
            char c = (char)rx_buf[i];
            if (c == '\r') continue;
            if (c == '\n') {
                line_buf[line_len] = '\0';
                dispatch(line_buf);
                line_len = 0;
            } else if (line_len + 1 < sizeof(line_buf)) {
                line_buf[line_len++] = c;
            } else {
                /* overflow: drop and reset */
                line_len = 0;
                out_err("line_too_long");
            }
        }

        /* 2. Experiment tick — refresh override if enabled. */
        int64_t now_us = esp_timer_get_time();
        int32_t tgtL = 0, tgtR = 0;
        if (s_enabled) {
            bool active = exp_eval(&s_exp, now_us, &tgtL, &tgtR);
            if (!active && s_exp.type != EXP_NONE && s_exp.type != EXP_CONST) {
                /* Finite experiment finished: settle at zero, stay enabled. */
                s_exp.type = EXP_CONST;
                s_exp.side_mask = SIDE_B;
                s_exp.erpm0 = 0;
                tgtL = 0;
                tgtR = 0;
                out_line("E,exp_done");
            }
            motor_task_set_tune_override(erpm_to_wheel_rpm_f(tgtL),
                                         erpm_to_wheel_rpm_f(tgtR));
        }

        /* 3. Periodic telemetry. */
        if (s_log_period_us != 0 && now_us >= s_next_log_us) {
            emit_telemetry(now_us, tgtL, tgtR);
            s_next_log_us += s_log_period_us;
            /* If we fell badly behind, resync. */
            if (s_next_log_us < now_us) s_next_log_us = now_us + s_log_period_us;
        }

        /* The blocking read above already paces the loop; vTaskDelayUntil
         * gives a hard floor in case the transport returns immediately. */
        if (n == 0) {
            vTaskDelayUntil(&last_wake, MIN_DELAY_TICKS(TICK_PERIOD_MS));
        } else {
            last_wake = xTaskGetTickCount();
        }
    }
}

/* ── Init ────────────────────────────────────────────────────────── */

esp_err_t tune_cli_init(void)
{
    /* Prefer WiFi when available; fall back to UART0. The WiFi stub
     * returns NULL until implemented. */
    s_xport = tune_transport_wifi_get();
    if (s_xport == NULL) s_xport = tune_transport_uart_get();
    if (s_xport == NULL) {
        ESP_LOGE(TAG, "no transport available");
        return ESP_FAIL;
    }

    s_enabled = false;
    s_exp.type = EXP_NONE;
    s_log_period_us = 0;

    BaseType_t ok = xTaskCreatePinnedToCore(
        tune_cli_task, "tune_cli",
        TUNE_CLI_TASK_STACK, NULL, TUNE_CLI_TASK_PRIO, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "tune_cli task started");
    return ESP_OK;
}
