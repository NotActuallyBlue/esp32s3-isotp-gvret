#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "constants.h"
#include "display.h"
#include "diag_can.h"
#include "elm327.h"
#include "elm_transport.h"
#include "elm_mode.h"
#include "power_mgr.h"
#include "stackwatch.h"

#define ELM_TAG             "ElmMode"
#define IN_CHUNK            64
#define IN_QUEUE_LEN        16
#define OUT_BUF_SIZE        1024
#define LOG_BUDGET          80          // commands logged at INFO after each new connection, then DEBUG

typedef enum { SRC_BLE, SRC_WIFI } source_t;

typedef struct {
    uint8_t source;
    uint8_t len;
    uint8_t data[IN_CHUNK];
} in_msg_t;

static QueueHandle_t    in_queue;
static elm_t            elm;
static source_t         reply_to = SRC_BLE;
static char             out_buf[OUT_BUF_SIZE];
static size_t           out_len;
static char             last_cmd[24] = "";
static uint32_t         command_count;
static int              log_budget = LOG_BUDGET;
static bool             bench_mode;
static volatile bool    ble_up, wifi_up;

// ---------------------------------------------------------------- elm327 hooks
static int op_request(void *ctx, uint32_t tx_id, uint32_t lo, uint32_t hi, const uint8_t *req, uint16_t len,
                      uint32_t timeout_ms, diag_response_t *out, int out_max)
{
    diag_request_t rq = {
        .tx_id = tx_id, .rx_lo = lo, .rx_hi = hi, .request = req, .request_len = len,
        .timeout_ms = timeout_ms, .extra_ms = 60,
    };
    int n = diag_can_request(&rq, out, out_max);
    if (log_budget > 0) {
        ESP_LOGI(ELM_TAG, "OBD 0x%03lX <- %02X %02X.. (%u bytes): %d response(s)%s", (unsigned long)tx_id, req[0], len > 1 ? req[1] : 0,
                 len, n, n > 0 ? "" : " (none)");
    }
    return n;
}

static void flush_out(void)
{
    if (!out_len) return;
    if (reply_to == SRC_BLE) elm_ble_send((const uint8_t *)out_buf, out_len);
    else elm_wifi_send((const uint8_t *)out_buf, out_len);
    out_len = 0;
}

static void op_write(void *ctx, const char *text)
{
    size_t n = strlen(text);
    if (out_len + n > sizeof(out_buf)) flush_out();
    if (n > sizeof(out_buf)) return;
    memcpy(out_buf + out_len, text, n);
    out_len += n;
}

static void op_command(void *ctx, const char *line)
{
    command_count++;
    strlcpy(last_cmd, line, sizeof(last_cmd));
    if (log_budget > 0) {
        log_budget--;
        ESP_LOGI(ELM_TAG, "%s: %s", reply_to == SRC_BLE ? "BLE" : "WiFi", line);
    } else {
        ESP_LOGD(ELM_TAG, "%s", line);
    }
    power_mgr_poke();
}

static const elm_ops_t ops = { .request = op_request, .write = op_write, .on_command = op_command };

// ---------------------------------------------------------------- transports
static void queue_input(source_t source, const uint8_t *data, size_t len)
{
    for (size_t pos = 0; pos < len; pos += IN_CHUNK) {
        in_msg_t msg = { .source = source, .len = (uint8_t)(len - pos > IN_CHUNK ? IN_CHUNK : len - pos) };
        memcpy(msg.data, data + pos, msg.len);
        xQueueSend(in_queue, &msg, pdMS_TO_TICKS(100));
    }
}

static void on_ble_rx(const uint8_t *data, size_t len)  { queue_input(SRC_BLE, data, len); }
static void on_wifi_rx(const uint8_t *data, size_t len) { queue_input(SRC_WIFI, data, len); }

static void on_link(bool up)
{
    if (up) {
        log_budget = LOG_BUDGET;
        power_mgr_poke();
    }
    ble_up = elm_ble_connected();
    wifi_up = elm_wifi_connected();
}

static bool clients_connected(void)
{
    return elm_ble_connected() || elm_wifi_connected();
}

// ---------------------------------------------------------------- screen
static void update_screen(void)
{
    char ble_value[24], activity[24], count[24], bus[24];
    const char *client = elm_ble_connected() ? "BLE" : (elm_wifi_connected() ? "WI-FI" : "none");

    snprintf(ble_value, sizeof(ble_value), "%s", elm_ble_connected() ? "CONNECTED" : "ADVERTISING");
    snprintf(activity, sizeof(activity), "%s", last_cmd[0] ? last_cmd : "--");
    snprintf(count, sizeof(count), "%lu", (unsigned long)command_count);
    uint32_t quiet = diag_can_ms_since_rx();
    if (bench_mode)                snprintf(bus, sizeof(bus), "SIMULATED");
    else if (quiet == UINT32_MAX)  snprintf(bus, sizeof(bus), "no frames yet");
    else if (quiet < 2000)         snprintf(bus, sizeof(bus), "ACTIVE");
    else                           snprintf(bus, sizeof(bus), "quiet %lus", (unsigned long)(quiet / 1000));

    display_detail_t d[] = {
        { "#BLE",     "",                     COLOR_CYAN },
        { "NAME",     ELM_BLE_NAME,           COLOR_WHITE },
        { "STATE",    ble_value,              elm_ble_connected() ? COLOR_GREEN : COLOR_YELLOW },
        { "#WI-FI",   "",                     COLOR_CYAN },
        { "SSID",     elm_wifi_ssid(),        COLOR_WHITE },
        { "PASSWORD", elm_wifi_password(),    COLOR_YELLOW },
        { "IP",       ELM_WIFI_IP,            COLOR_WHITE },
        { "PORT",     "35000",                COLOR_WHITE },
        { "#ACTIVITY", "",                    COLOR_CYAN },
        { "CLIENT",   client,                 clients_connected() ? COLOR_GREEN : COLOR_MUTED },
        { "LAST",     activity,               COLOR_WHITE },
        { "COMMANDS", count,                  COLOR_WHITE },
        { "CAN BUS",  bus,                    (quiet < 2000 || bench_mode) ? COLOR_GREEN : COLOR_MUTED },
    };
    display_set_details("ELM327", COLOR_CYAN, d, sizeof(d) / sizeof(d[0]));
    display_set_status("ELM327", clients_connected() ? "CONNECTED" : "READY", clients_connected() ? COLOR_GREEN : COLOR_CYAN);
}

static void elm_task(void *arg)
{
    in_msg_t msg;
    int64_t last_screen = 0;

    while (1) {
        if (xQueueReceive(in_queue, &msg, pdMS_TO_TICKS(250)) == pdTRUE) {
            reply_to = (source_t)msg.source;
            elm_input(&elm, msg.data, msg.len);
            flush_out();
            power_mgr_poke();
        }
        int64_t now = esp_timer_get_time();
        if (now - last_screen > 500000) {
            last_screen = now;
            update_screen();
        }
    }
}

void elm_mode_start(bool bench)
{
    bench_mode = bench;
    in_queue = xQueueCreate(IN_QUEUE_LEN, sizeof(in_msg_t));
    if (!elm_init(&elm, &ops, NULL)) {
        ESP_LOGE(ELM_TAG, "Out of memory");
        return;
    }

    diag_can_start(bench);
    stackwatch_create(elm_task, "elm", 8192, NULL, 2);
    elm_ble_start(on_ble_rx, on_link);
    elm_wifi_start(on_wifi_rx, on_link);
    ESP_LOGI(ELM_TAG, "ELM327 emulation running (BLE '%s', Wi-Fi %s:%d)", ELM_BLE_NAME, ELM_WIFI_IP, ELM_WIFI_PORT);
}

bool elm_mode_clients_connected(void)
{
    return clients_connected();
}
