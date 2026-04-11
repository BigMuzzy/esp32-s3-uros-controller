/*
 * rc_failsafe.c — RC PWM input via MCPWM capture + failsafe logic
 *
 * Three RC channels read by MCPWM capture ISRs. Each ISR records
 * the pulse width (rising → falling edge delta) using hardware
 * timestamps. The rc_failsafe_task reads these atomically.
 */

#include "rc_failsafe.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/mcpwm_cap.h"

#include <stdatomic.h>
#include <math.h>

static const char *TAG = "rc_failsafe";

/* ── Per-channel capture state ──────────────────────────────────── */

typedef struct {
    mcpwm_cap_channel_handle_t cap_chan;
    uint32_t                   rise_ticks;  /* capture value at rising edge */
    volatile uint16_t          pulse_us;    /* latest pulse width in µs */
    volatile int64_t           last_edge_us;/* esp_timer timestamp of last edge */
} rc_channel_t;

static rc_channel_t s_ch[3]; /* 0=steering, 1=throttle, 2=mode */

/* ── cmd_vel timeout tracking ───────────────────────────────────── */

static atomic_int_least64_t s_last_cmd_vel_us;

/* ── MCPWM capture callback ────────────────────────────────────── */

static bool IRAM_ATTR capture_cb(mcpwm_cap_channel_handle_t cap_chan,
                                  const mcpwm_capture_event_data_t *edata,
                                  void *user_data)
{
    rc_channel_t *ch = (rc_channel_t *)user_data;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        ch->rise_ticks = edata->cap_value;
    } else {
        /* Falling edge — compute pulse width */
        uint32_t delta = edata->cap_value - ch->rise_ticks;
        /* MCPWM capture timer runs at APB clock (typically 80 MHz) */
        uint32_t pulse_us = delta / 80; /* 80 ticks per µs at 80 MHz */
        /* Sanity check: valid RC PWM is 800–2200 µs */
        if (pulse_us >= 800 && pulse_us <= 2200) {
            ch->pulse_us    = (uint16_t)pulse_us;
            ch->last_edge_us = esp_timer_get_time();
        }
    }
    return false; /* no high-priority task woken */
}

/* ── Initialization ──────────────────────────────────────────────── */

static esp_err_t init_capture_channel(int idx, int gpio)
{
    mcpwm_cap_timer_handle_t cap_timer = NULL;

    /* Create capture timer (shared, one per group — only first call creates) */
    static mcpwm_cap_timer_handle_t s_cap_timer = NULL;
    if (s_cap_timer == NULL) {
        mcpwm_capture_timer_config_t timer_cfg = {
            .group_id = 0,
            .clk_src  = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        };
        esp_err_t ret = mcpwm_new_capture_timer(&timer_cfg, &s_cap_timer);
        if (ret != ESP_OK) return ret;

        ret = mcpwm_capture_timer_enable(s_cap_timer);
        if (ret != ESP_OK) return ret;

        ret = mcpwm_capture_timer_start(s_cap_timer);
        if (ret != ESP_OK) return ret;
    }
    cap_timer = s_cap_timer;

    /* Create capture channel on the specified GPIO */
    mcpwm_capture_channel_config_t chan_cfg = {
        .gpio_num = gpio,
        .prescale = 1,
        .flags = {
            .pos_edge  = true,
            .neg_edge  = true,
            .pull_down = true,
        },
    };
    esp_err_t ret = mcpwm_new_capture_channel(cap_timer, &chan_cfg,
                                               &s_ch[idx].cap_chan);
    if (ret != ESP_OK) return ret;

    /* Register callback */
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = capture_cb,
    };
    ret = mcpwm_capture_channel_register_event_callbacks(
              s_ch[idx].cap_chan, &cbs, &s_ch[idx]);
    if (ret != ESP_OK) return ret;

    ret = mcpwm_capture_channel_enable(s_ch[idx].cap_chan);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t rc_failsafe_init(void)
{
    /* Initialize pulse values to center (safe default) */
    for (int i = 0; i < 3; i++) {
        s_ch[i].pulse_us    = RC_PWM_CENTER_US;
        s_ch[i].last_edge_us = 0;
    }

    atomic_store(&s_last_cmd_vel_us, 0);

    static const int gpios[3] = { RC_CH1_GPIO, RC_CH2_GPIO, RC_CH3_GPIO };
    for (int i = 0; i < 3; i++) {
        esp_err_t ret = init_capture_channel(i, gpios[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init capture ch%d (GPIO %d): %s",
                     i + 1, gpios[i], esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "MCPWM capture on GPIO %d, %d, %d",
             RC_CH1_GPIO, RC_CH2_GPIO, RC_CH3_GPIO);
    return ESP_OK;
}

/* ── Reading ─────────────────────────────────────────────────────── */

rc_input_t rc_failsafe_read(void)
{
    int64_t now = esp_timer_get_time();
    rc_input_t out;

    out.ch1_us = s_ch[0].pulse_us;
    out.ch2_us = s_ch[1].pulse_us;

    /* Mode switch: >1500 µs = manual */
    out.ch3_manual = (s_ch[2].pulse_us > RC_PWM_CENTER_US);

    /* Signal OK if any channel had an edge within timeout */
    bool any_recent = false;
    for (int i = 0; i < 3; i++) {
        int64_t age = now - s_ch[i].last_edge_us;
        if (s_ch[i].last_edge_us != 0 &&
            age < (int64_t)RC_SIGNAL_TIMEOUT_MS * 1000) {
            any_recent = true;
            break;
        }
    }
    out.signal_ok = any_recent;

    return out;
}

drive_mode_t rc_failsafe_get_mode(void)
{
    rc_input_t rc = rc_failsafe_read();

    /* Mode switch to manual — immediate override */
    if (rc.ch3_manual && rc.signal_ok) {
        return DRIVE_MODE_MANUAL;
    }

    /* Autonomous mode — check cmd_vel timeout */
    int64_t last_cmd = atomic_load(&s_last_cmd_vel_us);
    int64_t now = esp_timer_get_time();

    if (last_cmd == 0 || (now - last_cmd) > (int64_t)CMD_VEL_TIMEOUT_MS * 1000) {
        return DRIVE_MODE_FAILSAFE_STOP;
    }

    return DRIVE_MODE_AUTONOMOUS;
}

/* ── Arcade mixing ───────────────────────────────────────────────── */

/** Normalize RC pulse to [-1, +1] with deadband. */
static float normalize_rc(uint16_t pulse_us)
{
    int16_t offset = (int16_t)pulse_us - RC_PWM_CENTER_US;

    /* Apply deadband */
    if (offset > -RC_DEADBAND_US && offset < RC_DEADBAND_US) {
        return 0.0f;
    }

    float norm = (float)offset / (float)RC_PWM_RANGE_US;
    if (norm > 1.0f)  norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;
    return norm;
}

cmd_vel_t rc_failsafe_arcade_mix(const rc_input_t *rc)
{
    float throttle = normalize_rc(rc->ch2_us);
    float steering = normalize_rc(rc->ch1_us);

    cmd_vel_t cmd;
    cmd.linear_x  = throttle * MAX_MANUAL_SPEED_MS;
    cmd.angular_z = steering * MAX_MANUAL_ANGVEL;
    return cmd;
}

/* ── Cross-core: cmd_vel notification ────────────────────────────── */

void rc_failsafe_notify_cmd_vel(void)
{
    atomic_store(&s_last_cmd_vel_us, esp_timer_get_time());
}
