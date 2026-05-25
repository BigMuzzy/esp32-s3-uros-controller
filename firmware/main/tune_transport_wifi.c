/*
 * tune_transport_wifi.c — WiFi STA + TCP server transport for tune_cli
 *
 * When CONFIG_TUNE_WIFI_ENABLE is set, this transport replaces UART0
 * as the active CLI channel. The firmware connects to the configured
 * AP and listens on CONFIG_TUNE_WIFI_PORT (default 3334) for a single
 * TCP client. The CLI line protocol is identical on both transports.
 *
 * Safety on disconnect:
 *   - The CAN override watchdog in motor_task expires after 150 ms when
 *     tune_cli stops refreshing it, so a TCP drop, AP drop, or radio
 *     blackout stops the motors automatically.
 *   - Only one client is served at a time; a new connection displaces
 *     any in-flight session.
 *
 * The transport is intentionally simple: blocking sockets with
 * SO_RCVTIMEO-converted timeouts, one accept task, one mutex guarding
 * the client fd. No TLS, no auth beyond network-layer access — assume
 * a trusted LAN and rely on the `enable` token + watchdog for safety.
 */

#include "tune_transport.h"

#include "sdkconfig.h"

#if CONFIG_TUNE_WIFI_ENABLE

#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define TCP_PORT             CONFIG_TUNE_WIFI_PORT
#define WIFI_RETRY_DELAY_MS  3000
#define ACCEPT_TASK_STACK    4096
#define ACCEPT_TASK_PRIO     3

#define BIT_GOT_IP   (1 << 0)

static const char *TAG = "tune_wifi";

static EventGroupHandle_t s_wifi_events;
static SemaphoreHandle_t  s_sock_mux;
static int                s_client_fd  = -1;   /* -1 = no client */
static int                s_listen_fd  = -1;
static bool               s_initialized;

/* ── WiFi STA bring-up ───────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, BIT_GOT_IP);
        ESP_LOGW(TAG, "WiFi disconnected; retrying in %d ms",
                 WIFI_RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got IP " IPSTR "; tune CLI on tcp/%d",
                 IP2STR(&evt->ip_info.ip), TCP_PORT);
        xEventGroupSetBits(s_wifi_events, BIT_GOT_IP);
    }
}

static esp_err_t wifi_sta_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "nvs init: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "netif init: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop: %s", esp_err_to_name(err));
        return err;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid,     CONFIG_TUNE_WIFI_SSID,
            sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, CONFIG_TUNE_WIFI_PASSWORD,
            sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = (strlen(CONFIG_TUNE_WIFI_PASSWORD) == 0)
        ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA started, joining \"%s\"...",
             CONFIG_TUNE_WIFI_SSID);
    return ESP_OK;
}

/* ── TCP accept loop ─────────────────────────────────────────────── */

static int open_listener(void)
{
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) { ESP_LOGE(TAG, "socket: %d", errno); return -1; }

    int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind: %d", errno); close(s); return -1;
    }
    if (listen(s, 1) < 0) {
        ESP_LOGE(TAG, "listen: %d", errno); close(s); return -1;
    }
    return s;
}

static void set_client(int new_fd)
{
    int old_fd;
    xSemaphoreTake(s_sock_mux, portMAX_DELAY);
    old_fd = s_client_fd;
    s_client_fd = new_fd;
    xSemaphoreGive(s_sock_mux);
    if (old_fd >= 0) {
        shutdown(old_fd, SHUT_RDWR);
        close(old_fd);
    }
}

static void accept_task(void *arg)
{
    (void)arg;

    xEventGroupWaitBits(s_wifi_events, BIT_GOT_IP,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    s_listen_fd = open_listener();
    if (s_listen_fd < 0) {
        ESP_LOGE(TAG, "listener failed; tune WiFi transport disabled");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "listening on tcp/%d", TCP_PORT);

    for (;;) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        int fd = accept(s_listen_fd, (struct sockaddr *)&peer, &peer_len);
        if (fd < 0) {
            ESP_LOGW(TAG, "accept: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* TCP keepalives: detect silently-vanished peers without
         * the kernel's default 2-hour timeout.  15 s idle, probe
         * every 5 s, 3 misses → dropped. */
        int yes = 1, idle = 15, intvl = 5, cnt = 3;
        setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE,  &yes,   sizeof(yes));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,  sizeof(idle));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof(cnt));

        char ipbuf[16] = { 0 };
        inet_ntoa_r(peer.sin_addr, ipbuf, sizeof(ipbuf));
        ESP_LOGI(TAG, "client connected from %s:%d",
                 ipbuf, ntohs(peer.sin_port));

        set_client(fd);   /* displaces any previous session */

        /* Wait until read/write tears the client down. */
        while (true) {
            xSemaphoreTake(s_sock_mux, portMAX_DELAY);
            int cur = s_client_fd;
            xSemaphoreGive(s_sock_mux);
            if (cur != fd) break;
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

/* ── tune_transport_t callbacks ──────────────────────────────────── */

static int wifi_xport_read(tune_transport_t *t, uint8_t *buf,
                           size_t len, uint32_t timeout_ms)
{
    (void)t;

    int fd;
    xSemaphoreTake(s_sock_mux, portMAX_DELAY);
    fd = s_client_fd;
    xSemaphoreGive(s_sock_mux);
    if (fd < 0) {
        if (timeout_ms) vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        return 0;
    }

    struct timeval tv = {
        .tv_sec  = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int n = recv(fd, buf, len, 0);
    if (n > 0) return n;
    if (n == 0) {
        ESP_LOGI(TAG, "client closed");
        set_client(-1);
        return 0;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    ESP_LOGW(TAG, "recv: %d", errno);
    set_client(-1);
    return 0;
}

static int wifi_xport_write(tune_transport_t *t,
                            const uint8_t *buf, size_t len)
{
    (void)t;

    int fd;
    xSemaphoreTake(s_sock_mux, portMAX_DELAY);
    fd = s_client_fd;
    xSemaphoreGive(s_sock_mux);
    if (fd < 0) return (int)len;   /* drop silently when no client */

    int n = send(fd, buf, len, MSG_NOSIGNAL);
    if (n < 0) {
        ESP_LOGW(TAG, "send: %d", errno);
        set_client(-1);
        return -1;
    }
    return n;
}

static bool wifi_xport_is_connected(tune_transport_t *t)
{
    (void)t;
    bool ok;
    xSemaphoreTake(s_sock_mux, portMAX_DELAY);
    ok = (s_client_fd >= 0);
    xSemaphoreGive(s_sock_mux);
    return ok;
}

static tune_transport_t s_xport = {
    .read         = wifi_xport_read,
    .write        = wifi_xport_write,
    .is_connected = wifi_xport_is_connected,
    .ctx          = NULL,
};

tune_transport_t *tune_transport_wifi_get(void)
{
    if (s_initialized) return &s_xport;

    s_wifi_events = xEventGroupCreate();
    s_sock_mux    = xSemaphoreCreateMutex();
    if (!s_wifi_events || !s_sock_mux) {
        ESP_LOGE(TAG, "alloc failed");
        return NULL;
    }

    if (wifi_sta_start() != ESP_OK) return NULL;

    BaseType_t ok = xTaskCreate(accept_task, "tune_wifi_acc",
                                ACCEPT_TASK_STACK, NULL,
                                ACCEPT_TASK_PRIO, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "accept task create failed");
        return NULL;
    }

    s_initialized = true;
    return &s_xport;
}

#else  /* !CONFIG_TUNE_WIFI_ENABLE */

#include "esp_log.h"
static const char *TAG = "tune_wifi";

tune_transport_t *tune_transport_wifi_get(void)
{
    ESP_LOGD(TAG, "WiFi transport disabled at build time");
    return NULL;
}

#endif /* CONFIG_TUNE_WIFI_ENABLE */
