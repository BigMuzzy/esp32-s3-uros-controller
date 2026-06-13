/*
 * canopen.c — CiA 301 primitives over ESP-IDF TWAI driver
 *
 * See canopen.h for the public contract and design rules.
 *
 * Internals
 * ─────────
 *   - One TWAI install at 500 kbit/s.  Filter accept-all (we are the
 *     master and care about heartbeats, SDO responses, TPDOs and
 *     EMCYs from many node IDs).
 *   - A single RX task pinned to Core 1 demultiplexes incoming frames:
 *       * SDO server→client response   → signal the SDO waiter
 *       * Heartbeat (0x700 + nodeid)   → update s_heartbeat[]
 *       * Anything matching a registered PDO callback → dispatch inline
 *   - SDO requests serialise through s_sdo_mutex.  A single binary
 *     semaphore signals "response received"; the response COB-ID, idx,
 *     sub are validated before the data is copied into the caller's
 *     buffer, and a per-request generation counter rejects a late reply
 *     from a previously timed-out transfer.  Aborts (SCS=0x80) are
 *     surfaced as ESP_ERR_INVALID_RESPONSE and the abort code is logged
 *     at ERROR level.
 */

#include "canopen.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <string.h>

static const char *TAG = "canopen";

/* ── Tunables ───────────────────────────────────────────────────── */

#define CANOPEN_RX_TASK_STACK     4096
#define CANOPEN_RX_TASK_PRIO      7        /* above the backend's zlac_tx (6)
                                            * so the RX demux preempts it and
                                            * never misses an incoming frame */
#define CANOPEN_RX_TASK_CORE      1

#define CANOPEN_TWAI_TX_TIMEOUT_MS  5
#define CANOPEN_SDO_DEFAULT_TO_MS   200
#define CANOPEN_MAX_PDO_CB          16
#define CANOPEN_MAX_NODES           127      /* full CiA 301 node-id range 1..127.
                                              * Table is (N+1)*12 B ≈ 1.5 KB; sized
                                              * to the whole range so heartbeat
                                              * tracking works for any ZLAC_NODE_ID
                                              * (Kconfig allows 1..127), not just
                                              * low IDs. */

/* SDO command specifier (CCS/SCS) — CiA 301 §7.2.4 */
#define SDO_CCS_DOWNLOAD_INIT     0x20  /* client → server, expedited */
#define SDO_CCS_UPLOAD_INIT       0x40  /* client → server */
#define SDO_SCS_DOWNLOAD_RESP     0x60  /* server → client, OK */
#define SDO_SCS_UPLOAD_RESP       0x40  /* server → client, expedited data */
#define SDO_ABORT                 0x80  /* either direction */

/* "expedited size indicated" flags overlaid on CCS/SCS */
#define SDO_DOWNLOAD_FLAGS_BASE   (0x20 | 0x02 | 0x01) /* expedited + size ind */
#define SDO_EXPEDITED             0x02
#define SDO_SIZE_INDICATED        0x01

/* ── Dispatch table ──────────────────────────────────────────────── */

typedef struct {
    uint32_t          cob_id;     /* 0 = empty slot */
    canopen_pdo_cb_t  cb;
    void             *ctx;
} canopen_pdo_entry_t;

static portMUX_TYPE        s_table_mux = portMUX_INITIALIZER_UNLOCKED;
static canopen_pdo_entry_t s_table[CANOPEN_MAX_PDO_CB];

/* ── Heartbeat cache (one slot per possible node id) ────────────── */

typedef struct {
    bool                valid;
    canopen_nmt_state_t state;
    uint32_t            last_ms;
} canopen_hb_t;

static portMUX_TYPE  s_hb_mux = portMUX_INITIALIZER_UNLOCKED;
static canopen_hb_t  s_heartbeat[CANOPEN_MAX_NODES + 1];   /* idx by node id */

/* ── SDO request/response handoff ───────────────────────────────── */

typedef struct {
    bool             active;
    uint32_t         seq;           /* generation, ++ on each new request */
    uint8_t          node;
    uint16_t         index;
    uint8_t          sub;
    /* Filled by RX task before giving the semaphore */
    uint32_t         rx_seq;        /* s_sdo.seq this response was matched to */
    uint8_t          rx_scs;        /* command specifier byte */
    uint16_t         rx_index;
    uint8_t          rx_sub;
    uint32_t         rx_data;       /* expedited payload (LE u32) */
    uint32_t         rx_abort_code;
} canopen_sdo_xfer_t;

static SemaphoreHandle_t s_sdo_mutex;        /* serialises requests */
static SemaphoreHandle_t s_sdo_done_sem;     /* binary, signalled by RX */
static canopen_sdo_xfer_t s_sdo;             /* current request state */

static bool s_inited = false;

/* ── Helpers ─────────────────────────────────────────────────────── */

static esp_err_t twai_tx(uint32_t cob_id, const uint8_t *data, uint8_t dlc)
{
    twai_message_t msg = {0};
    msg.identifier = cob_id;
    msg.extd       = 0;
    msg.rtr        = 0;
    msg.data_length_code = dlc;
    if (data != NULL && dlc > 0) {
        memcpy(msg.data, data, dlc);
    }
    return twai_transmit(&msg, pdMS_TO_TICKS(CANOPEN_TWAI_TX_TIMEOUT_MS));
}

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static canopen_pdo_cb_t find_cb(uint32_t cob_id, void **ctx_out)
{
    canopen_pdo_cb_t cb = NULL;
    void *ctx = NULL;
    taskENTER_CRITICAL(&s_table_mux);
    for (int i = 0; i < CANOPEN_MAX_PDO_CB; i++) {
        if (s_table[i].cob_id == cob_id && s_table[i].cb != NULL) {
            cb  = s_table[i].cb;
            ctx = s_table[i].ctx;
            break;
        }
    }
    taskEXIT_CRITICAL(&s_table_mux);
    if (ctx_out) *ctx_out = ctx;
    return cb;
}

/* ── RX task ─────────────────────────────────────────────────────── */

static void handle_sdo_response(const twai_message_t *msg)
{
    /* Expect 8 byte payload.  Some drives pad short responses with
     * zeros; tolerate any DLC and rely on size-indicated flag. */
    if (msg->data_length_code < 4) return;

    uint8_t  scs   = msg->data[0];
    uint16_t index = (uint16_t)msg->data[1] | ((uint16_t)msg->data[2] << 8);
    uint8_t  sub   = msg->data[3];
    uint32_t data  = (uint32_t)msg->data[4]
                   | ((uint32_t)msg->data[5] << 8)
                   | ((uint32_t)msg->data[6] << 16)
                   | ((uint32_t)msg->data[7] << 24);

    /* Validate against the in-flight request.  We do NOT hold the
     * SDO mutex here — the requester is blocked on s_sdo_done_sem.
     * Reads of s_sdo fields are safe because the requester populated
     * them before kicking off the TX, and no other path writes here. */
    if (!s_sdo.active) {
        ESP_LOGW(TAG, "Stray SDO response: node=%u idx=0x%04X.%02u",
                 (unsigned)(msg->identifier - CANOPEN_COB_SDO_TX_BASE),
                 index, sub);
        return;
    }
    uint8_t responder = (uint8_t)(msg->identifier - CANOPEN_COB_SDO_TX_BASE);
    if (responder != s_sdo.node || index != s_sdo.index || sub != s_sdo.sub) {
        ESP_LOGW(TAG, "Out-of-order SDO response: got %u 0x%04X.%02u, "
                      "expected %u 0x%04X.%02u",
                 responder, index, sub,
                 s_sdo.node, s_sdo.index, s_sdo.sub);
        return;
    }

    s_sdo.rx_scs   = scs;
    s_sdo.rx_index = index;
    s_sdo.rx_sub   = sub;
    s_sdo.rx_data  = data;
    s_sdo.rx_abort_code = (scs == SDO_ABORT) ? data : 0;
    s_sdo.rx_seq   = s_sdo.seq;   /* stamp with the in-flight generation */
    xSemaphoreGive(s_sdo_done_sem);
}

static void handle_heartbeat(uint8_t node, uint8_t state_byte)
{
    if (node == 0 || node > CANOPEN_MAX_NODES) return;
    taskENTER_CRITICAL(&s_hb_mux);
    s_heartbeat[node].valid   = true;
    s_heartbeat[node].state   = (canopen_nmt_state_t)state_byte;
    s_heartbeat[node].last_ms = now_ms();
    taskEXIT_CRITICAL(&s_hb_mux);
}

static void canopen_rx_task(void *arg)
{
    twai_message_t msg;
    for (;;) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) {
            continue;
        }
        uint32_t id = msg.identifier;

        /* Heartbeat: 0x700 + node, 1 byte payload */
        if (id >= CANOPEN_COB_HEARTBEAT_BASE &&
            id <  CANOPEN_COB_HEARTBEAT_BASE + 0x80 &&
            msg.data_length_code >= 1) {
            uint8_t node = (uint8_t)(id - CANOPEN_COB_HEARTBEAT_BASE);
            handle_heartbeat(node, msg.data[0]);
            /* Fall through — also offer to a registered callback if any */
        }

        /* SDO response: 0x580 + node */
        if (id >= CANOPEN_COB_SDO_TX_BASE &&
            id <  CANOPEN_COB_SDO_TX_BASE + 0x80) {
            handle_sdo_response(&msg);
            continue;
        }

        /* Dispatch to any registered callback (TPDOs, EMCYs, heartbeats). */
        void *ctx = NULL;
        canopen_pdo_cb_t cb = find_cb(id, &ctx);
        if (cb != NULL) {
            cb(id, msg.data, msg.data_length_code, ctx);
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────── */

esp_err_t canopen_init(int gpio_tx, int gpio_rx)
{
    if (s_inited) return ESP_OK;

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        gpio_tx, gpio_rx, TWAI_MODE_NORMAL);
    g.alerts_enabled = TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF |
                       TWAI_ALERT_ERR_PASS  | TWAI_ALERT_TX_FAILED |
                       TWAI_ALERT_RX_QUEUE_FULL;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g, &t, &f);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_start: %s", esp_err_to_name(ret));
        return ret;
    }

    s_sdo_mutex    = xSemaphoreCreateMutex();
    s_sdo_done_sem = xSemaphoreCreateBinary();
    if (s_sdo_mutex == NULL || s_sdo_done_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(canopen_rx_task, "canopen_rx",
        CANOPEN_RX_TASK_STACK, NULL, CANOPEN_RX_TASK_PRIO, NULL,
        CANOPEN_RX_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CANopen up (TWAI 500 kbit/s, TX=%d RX=%d)",
             gpio_tx, gpio_rx);
    s_inited = true;
    return ESP_OK;
}

/* ── Bus diagnostics & recovery ─────────────────────────────────── */

static const char *twai_state_str(twai_state_t st)
{
    switch (st) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
    }
}

void canopen_log_bus_diagnostics(const char *context)
{
    const char *ctx = context ? context : "";

    if (!s_inited) {
        ESP_LOGW(TAG, "diag(%s): TWAI not initialised", ctx);
        return;
    }

    /* Non-blocking: returns whatever alerts have latched since last read. */
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) != ESP_OK) {
        alerts = 0;   /* ESP_ERR_TIMEOUT == none pending */
    }

    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) {
        ESP_LOGW(TAG, "diag(%s): status query failed", ctx);
        return;
    }

    ESP_LOGW(TAG, "diag(%s): state=%s tx_err=%" PRIu32 " rx_err=%" PRIu32
                  " tx_failed=%" PRIu32 " bus_err=%" PRIu32 " arb_lost=%" PRIu32
                  " tx_q=%" PRIu32 " rx_q=%" PRIu32,
             ctx, twai_state_str(st.state),
             st.tx_error_counter, st.rx_error_counter,
             st.tx_failed_count, st.bus_error_count, st.arb_lost_count,
             st.msgs_to_tx, st.msgs_to_rx);

    if (alerts) {
        ESP_LOGW(TAG, "diag(%s): alerts=0x%08" PRIx32 "%s%s%s%s%s", ctx, alerts,
                 (alerts & TWAI_ALERT_TX_FAILED)     ? " TX_FAILED"   : "",
                 (alerts & TWAI_ALERT_BUS_ERROR)     ? " BUS_ERROR"   : "",
                 (alerts & TWAI_ALERT_ERR_PASS)      ? " ERR_PASSIVE" : "",
                 (alerts & TWAI_ALERT_BUS_OFF)       ? " BUS_OFF"     : "",
                 (alerts & TWAI_ALERT_RX_QUEUE_FULL) ? " RX_Q_FULL"   : "");
    }

    bool not_acked = (st.state == TWAI_STATE_BUS_OFF) ||
                     (st.tx_error_counter >= 96) ||
                     (st.tx_failed_count > 0) ||
                     (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF));

    bool rx_corrupt = (st.rx_error_counter >= 8) ||
                      (st.bus_error_count > 0) ||
                      (alerts & TWAI_ALERT_BUS_ERROR);

    if (not_acked) {
        ESP_LOGE(TAG, "diag(%s): TX not ACKed — no node is acknowledging our "
                      "frames. Check: drive powered? CAN_H/L wiring + 120 ohm "
                      "termination at both ends? bus bitrate = 500 kbit/s?", ctx);
    } else if (rx_corrupt) {
        ESP_LOGW(TAG, "diag(%s): our frames ARE ACKed (tx_err=%" PRIu32 ") but we "
                      "received corrupted/foreign frames — possible second device "
                      "at a different bitrate, electrical noise, or marginal "
                      "termination. A peer is present at 500 kbit/s.",
                 ctx, st.tx_error_counter);
    } else if (st.tx_error_counter == 0 && st.rx_error_counter == 0) {
        ESP_LOGW(TAG, "diag(%s): bus healthy and our frames are ACKed, but no SDO "
                      "reply — a node is present at 500 kbit/s but did not answer. "
                      "Likely the drive is still booting (see manual §3.2 boot-up "
                      "frame) or the node ID is wrong (run a node scan).", ctx);
    }
}

void canopen_scan_nodes(void)
{
    if (!s_inited) {
        ESP_LOGW(TAG, "scan: TWAI not initialised");
        return;
    }

    ESP_LOGW(TAG, "scanning CANopen node IDs 1..127 (SDO read 0x1000:00)...");
    int found = 0;
    for (uint8_t node = 1; node <= 127; ++node) {
        uint32_t devtype = 0;
        esp_err_t r = canopen_sdo_read_u32(node, 0x1000, 0, &devtype, 40);
        if (r == ESP_OK) {
            ESP_LOGW(TAG, "  -> node %u present: 0x1000 device type = 0x%08" PRIx32,
                     (unsigned)node, devtype);
            found++;
        } else if (r == ESP_ERR_INVALID_RESPONSE) {
            /* SDO abort: the node answered but lacks 0x1000 — still present. */
            ESP_LOGW(TAG, "  -> node %u present (SDO abort on 0x1000)",
                     (unsigned)node);
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGE(TAG, "scan: no node answered an SDO. If frames are being ACKed "
                      "(tx_err=0) the device may use a non-standard SDO COB-ID; "
                      "otherwise check power / wiring / termination / bitrate.");
    } else {
        ESP_LOGW(TAG, "scan: %d node(s) answered. Set CONFIG_ZLAC_NODE_ID to the "
                      "drive's ID above, then rebuild.", found);
    }
}

esp_err_t canopen_bus_recover(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) return ESP_FAIL;

    if (st.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGW(TAG, "bus-off detected — initiating TWAI recovery");
        esp_err_t r = twai_initiate_recovery();
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "twai_initiate_recovery: %s", esp_err_to_name(r));
            return r;
        }
        /* Recovery finishes after 128×11 recessive bits; poll for the
         * transition out of RECOVERING (driver lands in STOPPED). */
        for (int i = 0; i < 20; ++i) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (twai_get_status_info(&st) == ESP_OK &&
                st.state != TWAI_STATE_RECOVERING) {
                break;
            }
        }
    }

    if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_STOPPED) {
        esp_err_t r = twai_start();
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "twai_start after recovery: %s", esp_err_to_name(r));
            return r;
        }
        ESP_LOGI(TAG, "TWAI restarted after bus recovery");
    }
    return ESP_OK;
}

esp_err_t canopen_nmt_send(canopen_nmt_cmd_t cmd, uint8_t node_id)
{
    uint8_t payload[2] = { (uint8_t)cmd, node_id };
    return twai_tx(CANOPEN_COB_NMT_CMD, payload, 2);
}

esp_err_t canopen_sync_send(void)
{
    return twai_tx(CANOPEN_COB_SYNC, NULL, 0);
}

esp_err_t canopen_send_pdo(uint32_t cob_id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > 8) return ESP_ERR_INVALID_ARG;
    return twai_tx(cob_id, data, dlc);
}

/* ── SDO download (write) ───────────────────────────────────────── */

static esp_err_t sdo_expedited_write(uint8_t node, uint16_t idx, uint8_t sub,
                                     uint32_t value, uint8_t size,
                                     uint32_t timeout_ms)
{
    if (!s_inited)             return ESP_ERR_INVALID_STATE;
    if (node == 0 || node > 127) return ESP_ERR_INVALID_ARG;
    if (size == 0 || size > 4) return ESP_ERR_INVALID_ARG;
    if (timeout_ms == 0)       timeout_ms = CANOPEN_SDO_DEFAULT_TO_MS;

    if (xSemaphoreTake(s_sdo_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Build CCS byte: 0x23/0x27/0x2B/0x2F for 4/3/2/1 byte expedited */
    uint8_t n = (uint8_t)(4 - size);                /* unused bytes */
    uint8_t ccs = 0x20 | (n << 2) | SDO_EXPEDITED | SDO_SIZE_INDICATED;
    uint8_t payload[8] = {
        ccs,
        (uint8_t)(idx & 0xFF), (uint8_t)((idx >> 8) & 0xFF),
        sub,
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF),
    };

    /* Drain any stale "done" signal before arming this request. */
    (void)xSemaphoreTake(s_sdo_done_sem, 0);

    s_sdo.active = true;
    s_sdo.seq++;                       /* new generation for this request */
    uint32_t my_seq = s_sdo.seq;
    s_sdo.node   = node;
    s_sdo.index  = idx;
    s_sdo.sub    = sub;
    s_sdo.rx_abort_code = 0;

    esp_err_t ret = twai_tx(CANOPEN_COB_SDO_RX_BASE + node, payload, 8);
    if (ret != ESP_OK) {
        s_sdo.active = false;
        xSemaphoreGive(s_sdo_mutex);
        ESP_LOGW(TAG, "SDO write tx fail node=%u idx=0x%04X.%02u: %s",
                 node, idx, sub, esp_err_to_name(ret));
        return ret;
    }

    if (xSemaphoreTake(s_sdo_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        s_sdo.active = false;
        xSemaphoreGive(s_sdo_mutex);
        ESP_LOGW(TAG, "SDO write timeout node=%u idx=0x%04X.%02u",
                 node, idx, sub);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t  scs    = s_sdo.rx_scs;
    uint32_t abort  = s_sdo.rx_abort_code;
    bool     seq_ok = (s_sdo.rx_seq == my_seq);
    s_sdo.active = false;
    xSemaphoreGive(s_sdo_mutex);

    if (!seq_ok) {
        /* Response belonged to an earlier (timed-out) generation that
         * slipped past the drain — reject so a late reply can't satisfy
         * this request with stale data. */
        ESP_LOGW(TAG, "SDO write stale response node=%u idx=0x%04X.%02u",
                 node, idx, sub);
        return ESP_ERR_TIMEOUT;
    }
    if (scs == SDO_ABORT) {
        ESP_LOGE(TAG, "SDO write abort node=%u idx=0x%04X.%02u code=0x%08" PRIX32,
                 node, idx, sub, abort);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if ((scs & 0xE0) != SDO_SCS_DOWNLOAD_RESP) {
        ESP_LOGE(TAG, "SDO write bad scs=0x%02X node=%u idx=0x%04X.%02u",
                 scs, node, idx, sub);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t canopen_sdo_write_u8(uint8_t node, uint16_t idx, uint8_t sub,
                               uint8_t v, uint32_t to)
{
    return sdo_expedited_write(node, idx, sub, (uint32_t)v, 1, to);
}

esp_err_t canopen_sdo_write_i8(uint8_t node, uint16_t idx, uint8_t sub,
                               int8_t v, uint32_t to)
{
    /* Sign-extend into the lowest byte; upper bytes are "unused" per
     * the expedited size-indicator and are ignored by the slave. */
    return sdo_expedited_write(node, idx, sub, (uint32_t)(uint8_t)v, 1, to);
}

esp_err_t canopen_sdo_write_u16(uint8_t node, uint16_t idx, uint8_t sub,
                                uint16_t v, uint32_t to)
{
    return sdo_expedited_write(node, idx, sub, (uint32_t)v, 2, to);
}

esp_err_t canopen_sdo_write_u32(uint8_t node, uint16_t idx, uint8_t sub,
                                uint32_t v, uint32_t to)
{
    return sdo_expedited_write(node, idx, sub, v, 4, to);
}

/* ── SDO upload (read) ──────────────────────────────────────────── */

static esp_err_t sdo_expedited_read(uint8_t node, uint16_t idx, uint8_t sub,
                                    uint32_t *value_out, uint32_t timeout_ms)
{
    if (!s_inited)              return ESP_ERR_INVALID_STATE;
    if (node == 0 || node > 127) return ESP_ERR_INVALID_ARG;
    if (value_out == NULL)      return ESP_ERR_INVALID_ARG;
    if (timeout_ms == 0)        timeout_ms = CANOPEN_SDO_DEFAULT_TO_MS;

    if (xSemaphoreTake(s_sdo_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t payload[8] = {
        SDO_CCS_UPLOAD_INIT,
        (uint8_t)(idx & 0xFF), (uint8_t)((idx >> 8) & 0xFF),
        sub, 0, 0, 0, 0,
    };

    (void)xSemaphoreTake(s_sdo_done_sem, 0);

    s_sdo.active = true;
    s_sdo.seq++;                       /* new generation for this request */
    uint32_t my_seq = s_sdo.seq;
    s_sdo.node   = node;
    s_sdo.index  = idx;
    s_sdo.sub    = sub;
    s_sdo.rx_abort_code = 0;

    esp_err_t ret = twai_tx(CANOPEN_COB_SDO_RX_BASE + node, payload, 8);
    if (ret != ESP_OK) {
        s_sdo.active = false;
        xSemaphoreGive(s_sdo_mutex);
        return ret;
    }

    if (xSemaphoreTake(s_sdo_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        s_sdo.active = false;
        xSemaphoreGive(s_sdo_mutex);
        ESP_LOGW(TAG, "SDO read timeout node=%u idx=0x%04X.%02u",
                 node, idx, sub);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t  scs    = s_sdo.rx_scs;
    uint32_t data   = s_sdo.rx_data;
    uint32_t abort  = s_sdo.rx_abort_code;
    bool     seq_ok = (s_sdo.rx_seq == my_seq);
    s_sdo.active = false;
    xSemaphoreGive(s_sdo_mutex);

    if (!seq_ok) {
        /* Late reply from a prior (timed-out) generation — reject so a
         * read can't return stale data from the wrong request. */
        ESP_LOGW(TAG, "SDO read stale response node=%u idx=0x%04X.%02u",
                 node, idx, sub);
        return ESP_ERR_TIMEOUT;
    }
    if (scs == SDO_ABORT) {
        ESP_LOGE(TAG, "SDO read abort node=%u idx=0x%04X.%02u code=0x%08" PRIX32,
                 node, idx, sub, abort);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if ((scs & 0xE0) != SDO_SCS_UPLOAD_RESP) {
        ESP_LOGE(TAG, "SDO read bad scs=0x%02X node=%u idx=0x%04X.%02u",
                 scs, node, idx, sub);
        return ESP_ERR_INVALID_RESPONSE;
    }

    *value_out = data;
    return ESP_OK;
}

esp_err_t canopen_sdo_read_u16(uint8_t node, uint16_t idx, uint8_t sub,
                               uint16_t *value, uint32_t to)
{
    uint32_t raw = 0;
    esp_err_t ret = sdo_expedited_read(node, idx, sub, &raw, to);
    if (ret == ESP_OK && value) *value = (uint16_t)(raw & 0xFFFF);
    return ret;
}

esp_err_t canopen_sdo_read_u32(uint8_t node, uint16_t idx, uint8_t sub,
                               uint32_t *value, uint32_t to)
{
    return sdo_expedited_read(node, idx, sub, value, to);
}

/* ── Dispatch table mgmt ────────────────────────────────────────── */

esp_err_t canopen_register_pdo_cb(uint32_t cob_id,
                                  canopen_pdo_cb_t cb,
                                  void *ctx)
{
    if (cob_id == 0) return ESP_ERR_INVALID_ARG;
    taskENTER_CRITICAL(&s_table_mux);
    /* Unregister path */
    if (cb == NULL) {
        for (int i = 0; i < CANOPEN_MAX_PDO_CB; i++) {
            if (s_table[i].cob_id == cob_id) {
                s_table[i].cob_id = 0;
                s_table[i].cb     = NULL;
                s_table[i].ctx    = NULL;
                break;
            }
        }
        taskEXIT_CRITICAL(&s_table_mux);
        return ESP_OK;
    }
    /* Replace existing or take a free slot */
    int free_slot = -1;
    for (int i = 0; i < CANOPEN_MAX_PDO_CB; i++) {
        if (s_table[i].cob_id == cob_id) { free_slot = i; break; }
        if (s_table[i].cob_id == 0 && free_slot == -1) free_slot = i;
    }
    if (free_slot == -1) {
        taskEXIT_CRITICAL(&s_table_mux);
        return ESP_ERR_NO_MEM;
    }
    s_table[free_slot].cob_id = cob_id;
    s_table[free_slot].cb     = cb;
    s_table[free_slot].ctx    = ctx;
    taskEXIT_CRITICAL(&s_table_mux);
    return ESP_OK;
}

bool canopen_get_heartbeat(uint8_t node_id,
                           canopen_nmt_state_t *state_out,
                           uint32_t *last_ms_out)
{
    if (node_id == 0 || node_id > CANOPEN_MAX_NODES) return false;
    bool valid;
    canopen_nmt_state_t state;
    uint32_t last_ms;
    taskENTER_CRITICAL(&s_hb_mux);
    valid   = s_heartbeat[node_id].valid;
    state   = s_heartbeat[node_id].state;
    last_ms = s_heartbeat[node_id].last_ms;
    taskEXIT_CRITICAL(&s_hb_mux);
    if (!valid) return false;
    if (state_out)   *state_out   = state;
    if (last_ms_out) *last_ms_out = last_ms;
    return true;
}
