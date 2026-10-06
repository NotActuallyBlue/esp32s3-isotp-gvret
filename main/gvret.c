#include "gvret.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/twai.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "constants.h"
#include "display.h"
#include "canstats.h"
#include "bench_sim.h"
#include "elm_transport.h"

#define GVRET_TAG       "GVRET"
#define GVRET_SSID      "ISOTP-SAVVYCAN"
#define GVRET_PORT      23              // the port SavvyCAN's "Network connection (GVRET)" uses by default
#define BATCH_SIZE      1024

// Protocol command bytes
#define PROTO_BUILD_CAN_FRAME    0   // 0x00
#define PROTO_TIME_SYNC          1   // 0x01
#define PROTO_GET_CANBUS_PARAMS  6   // 0x06
#define PROTO_GET_DEV_INFO       7   // 0x07
#define PROTO_GET_EXT_BUSES      9   // 0x09
#define PROTO_GET_NUM_BUSES      12  // 0x0C
#define PROTO_GET_NUM_BUSES_EXT  13  // 0x0D

typedef struct {
    uint32_t    id;
    uint8_t     len;
    uint8_t     data[8];
} bench_frame_t;

static bool             bench_mode;
static QueueHandle_t    bench_queue;

// ---------------------------------------------------------------- output
// USB is only written while a computer is attached, and only while something is reading it. A computer that powers the dongle
// but has not opened the serial port leaves the transmit buffer full: every write would then wait for its timeout and stall the
// capture (and the Wi-Fi stream with it). After a few writes that do not fit, USB is skipped for half a second and then tried again.
static int64_t usb_skip_until_us;
static int     usb_failures;

static void write_usb(const uint8_t *data, size_t len)
{
    if (!usb_serial_jtag_is_connected()) return;
    int64_t now = esp_timer_get_time();
    if (now < usb_skip_until_us) return;

    int written = usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(2));
    if (written < (int)len) {
        if (++usb_failures >= 3) {
            usb_skip_until_us = now + 500000;
            usb_failures = 0;
        }
    } else {
        usb_failures = 0;
    }
}

static void write_net(const uint8_t *data, size_t len)
{
    elm_wifi_send(data, len);
}

static void write_all(const uint8_t *data, size_t len)
{
    write_usb(data, len);
    write_net(data, len);
}

// ---------------------------------------------------------------- CAN access
static void can_transmit(uint32_t id, bool extended, uint8_t dlc, const uint8_t *data)
{
    g_tx_count++;
    if (bench_mode) {
        bench_sim_send_can(id, data, dlc);
        return;
    }
    twai_message_t frame = { 0 };
    frame.extd = extended;
    frame.identifier = extended ? id : (id & 0x7FF);
    frame.data_length_code = dlc;
    memcpy(frame.data, data, dlc);
    twai_transmit(&frame, pdMS_TO_TICKS(10));
}

static volatile uint32_t bench_dropped;

static void bench_sink(uint32_t id, const uint8_t *data, uint16_t len)
{
    bench_frame_t f = { .id = id, .len = (uint8_t)(len > 8 ? 8 : len) };
    memcpy(f.data, data, f.len);
    if (xQueueSend(bench_queue, &f, 0) != pdTRUE) bench_dropped++;
}

static bool receive_frame(twai_message_t *out, TickType_t wait)
{
    if (bench_mode) {
        bench_frame_t f;
        if (xQueueReceive(bench_queue, &f, wait) != pdTRUE) return false;
        memset(out, 0, sizeof(*out));
        out->identifier = f.id;
        out->data_length_code = f.len;
        memcpy(out->data, f.data, f.len);
        return true;
    }
    return twai_receive(out, wait) == ESP_OK;
}

// One captured frame in GVRET format. Returns the number of bytes written.
static size_t pack_frame(const twai_message_t *frame, uint8_t *out)
{
    uint32_t now = (uint32_t)esp_timer_get_time();
    uint32_t id = frame->identifier | (frame->extd ? (1UL << 31) : 0);
    uint8_t dlc = frame->data_length_code & 0x0F;
    if (dlc > 8) dlc = 8;

    out[0] = 0xF1;
    out[1] = PROTO_BUILD_CAN_FRAME;
    out[2] = now & 0xFF; out[3] = (now >> 8) & 0xFF; out[4] = (now >> 16) & 0xFF; out[5] = (now >> 24) & 0xFF;
    out[6] = id & 0xFF; out[7] = (id >> 8) & 0xFF; out[8] = (id >> 16) & 0xFF; out[9] = (id >> 24) & 0xFF;
    out[10] = dlc;
    memcpy(&out[11], frame->data, dlc);
    return 11 + dlc;
}

// Frames from the bus go to every connected client, several at a time so Wi-Fi is not asked for one packet per frame
static void gvret_rx_can_task(void *arg)
{
    uint8_t batch[BATCH_SIZE];
    size_t used = 0;

    while (1) {
        twai_message_t frame;
        if (receive_frame(&frame, used ? 0 : pdMS_TO_TICKS(20))) {
            g_rx_count++;
            canstats_on_frame(frame.identifier, frame.data_length_code);
            used += pack_frame(&frame, batch + used);
            if (used + 20 < sizeof(batch)) continue;
        }
        if (used) {
            write_all(batch, used);
            used = 0;
        }
    }
}

// ---------------------------------------------------------------- command parser
// Works byte by byte, so a command split across two reads (normal over TCP) is still understood.
typedef enum { ST_IDLE, ST_CMD, ST_FRAME } parse_state_t;

typedef struct {
    parse_state_t   state;
    uint8_t         buf[16];
    int             have;
    int             need;
    void          (*write)(const uint8_t *data, size_t len);
} parser_t;

static void put_u32(uint8_t *out, uint32_t v)
{
    out[0] = v & 0xFF; out[1] = (v >> 8) & 0xFF; out[2] = (v >> 16) & 0xFF; out[3] = (v >> 24) & 0xFF;
}

static void handle_command(parser_t *p, uint8_t cmd)
{
    switch (cmd) {
    case PROTO_GET_NUM_BUSES: {
        const uint8_t resp[] = { 0xF1, PROTO_GET_NUM_BUSES, 1 };
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_GET_NUM_BUSES_EXT: {
        const uint8_t resp[] = { 0xF1, PROTO_GET_NUM_BUSES_EXT, 0 };
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_GET_DEV_INFO: {
        const uint8_t resp[] = { 0xF1, PROTO_GET_DEV_INFO, 0x20, 0x01, 0x00, 0x00 };
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_GET_CANBUS_PARAMS: {
        const uint8_t resp[] = { 0xF1, PROTO_GET_CANBUS_PARAMS, 0x01, 0x20, 0xA1, 0x07, 0x00 };   // bus 0 active, 500000 baud
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_GET_EXT_BUSES: {                 // also used as the keepalive ping
        const uint8_t resp[] = { 0xF1, PROTO_GET_EXT_BUSES, 0x01, 0x00, 0x00, 0x00 };
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_TIME_SYNC: {
        uint8_t resp[6] = { 0xF1, PROTO_TIME_SYNC };
        put_u32(resp + 2, (uint32_t)esp_timer_get_time());
        p->write(resp, sizeof(resp));
        break;
    }
    default:
        break;
    }
}

static void parser_feed(parser_t *p, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        switch (p->state) {
        case ST_IDLE:
            if (b == 0xF1) p->state = ST_CMD;           // anything else (0xE7 filler, checksums) is skipped
            break;

        case ST_CMD:
            if (b == PROTO_BUILD_CAN_FRAME) {
                p->state = ST_FRAME;
                p->have = 0;
                p->need = 6;                            // id (4), bus (1), length (1), then the data
            } else {
                handle_command(p, b);
                p->state = ST_IDLE;
            }
            break;

        case ST_FRAME:
            p->buf[p->have++] = b;
            if (p->have == 6) {
                uint8_t dlc = p->buf[5] & 0x0F;
                p->need = 6 + (dlc > 8 ? 8 : dlc);
            }
            if (p->have >= p->need) {
                uint32_t id = (uint32_t)p->buf[0] | ((uint32_t)p->buf[1] << 8) | ((uint32_t)p->buf[2] << 16) | ((uint32_t)p->buf[3] << 24);
                uint8_t dlc = p->buf[5] & 0x0F;
                if (dlc > 8) dlc = 8;
                bool extended = (id & (1UL << 31)) != 0;
                can_transmit(id & ~(1UL << 31), extended, dlc, &p->buf[6]);
                p->state = ST_IDLE;
            }
            break;
        }
    }
}

static parser_t usb_parser = { .state = ST_IDLE, .write = write_usb };
static parser_t net_parser = { .state = ST_IDLE, .write = write_net };

// Wi-Fi client sent bytes (runs in the TCP server task)
static void on_net_rx(const uint8_t *data, size_t len)
{
    parser_feed(&net_parser, data, len);
}

static void on_net_link(bool up)
{
    net_parser.state = ST_IDLE;
    ESP_LOGI(GVRET_TAG, "SavvyCAN %s over Wi-Fi", up ? "connected" : "disconnected");
}

static void gvret_usb_task(void *arg)
{
    uint8_t rx_buf[128];
    while (1) {
        int len = usb_serial_jtag_read_bytes(rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(20));
        if (len > 0) parser_feed(&usb_parser, rx_buf, len);
    }
}

// ---------------------------------------------------------------- bench traffic
// With no car there would be nothing to look at, so the bench simulator also broadcasts three test frames, the way a real bus
// has fast and slow cyclic messages: 0x100 every 10 ms, 0x200 every 20 ms and 0x300 every 100 ms. Byte 0 is a counter,
// byte 1 a slow ramp and byte 2 a triangle wave, so graphs in SavvyCAN have something to draw.
static void bench_traffic_task(void *arg)
{
    uint32_t tick = 0;
    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
        tick++;
        uint8_t tri = (uint8_t)((tick % 100) < 50 ? (tick % 100) * 5 : (100 - (tick % 100)) * 5);
        uint8_t ramp = (uint8_t)(tick / 10);
        uint8_t data[8] = { (uint8_t)tick, ramp, tri, 0, 0, 0, 0, (uint8_t)(tick >> 8) };
        bench_sink(0x100, data, 8);
        if (tick % 2 == 0)  bench_sink(0x200, data, 8);
        if (tick % 10 == 0) bench_sink(0x300, data, 8);
    }
}

// ---------------------------------------------------------------- screen
static const char *can_state_text(uint16_t *color)
{
    twai_status_info_t st;
    *color = COLOR_WHITE;
    if (bench_mode || twai_get_status_info(&st) != ESP_OK) return "simulated";
    if (st.state == TWAI_STATE_BUS_OFF || st.state == TWAI_STATE_RECOVERING) { *color = COLOR_RED; return "BUS OFF"; }
    if (st.tx_error_counter >= 128 || st.rx_error_counter >= 128) { *color = COLOR_ORANGE; return "ERROR PASSIVE"; }
    *color = COLOR_GREEN;
    return "OK";
}

static void gvret_screen_task(void *arg)
{
    while (1) {
        char rx[16], tx[16];
        snprintf(rx, sizeof(rx), "%lu", (unsigned long)g_rx_count);
        snprintf(tx, sizeof(tx), "%lu", (unsigned long)g_tx_count);
        bool net = elm_wifi_connected();
        bool usb = usb_serial_jtag_is_connected();
        char port[8];
        snprintf(port, sizeof(port), "%d", elm_wifi_port());
        uint16_t state_color;
        const char *state = can_state_text(&state_color);

        display_detail_t d[] = {
            { "#WI-FI",    "",                            COLOR_CYAN },
            { "NETWORK",   elm_wifi_ssid(),               COLOR_WHITE },
            { "PASSWORD",  elm_wifi_password(),           COLOR_YELLOW },
            { "IP",        ELM_WIFI_IP,                   COLOR_WHITE },
            { "PORT",      port,                          COLOR_WHITE },
            { "#CAPTURE",  "",                            COLOR_CYAN },
            { "CLIENT",    net ? "Wi-Fi" : (usb ? "USB" : "none"), (net || usb) ? COLOR_GREEN : COLOR_LIGHTGREY },
            { "FRAMES RX", rx,                            COLOR_WHITE },
            { "FRAMES TX", tx,                            COLOR_WHITE },
            { "CAN BUS",   state,                         state_color },
        };
        display_set_details("SAVVYCAN", COLOR_CYAN, d, sizeof(d) / sizeof(d[0]));
        display_set_status("SAVVYCAN", (net || usb) ? "CONNECTED" : "READY", (net || usb) ? COLOR_GREEN : COLOR_CYAN);
        static uint32_t last_dropped, last_tx;
        if (bench_dropped != last_dropped || g_tx_count != last_tx) {
            ESP_LOGI(GVRET_TAG, "frames sent to the bus %lu, bench queue overflows %lu", (unsigned long)g_tx_count, (unsigned long)bench_dropped);
            last_dropped = bench_dropped;
            last_tx = g_tx_count;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

bool gvret_client_connected(void)
{
    return elm_wifi_connected() || usb_serial_jtag_is_connected();
}

void gvret_start(bool bench)
{
    bench_mode = bench;
    if (bench) {
        bench_queue = xQueueCreate(256, sizeof(bench_frame_t));
        bench_sim_set_sink(bench_sink);
        bench_sim_start();
        xTaskCreate(bench_traffic_task, "gvret_bench", 3072, NULL, 3, NULL);
    }

    // USB stays available for bench use with SavvyCAN
    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 2048,
    };
    usb_serial_jtag_driver_install(&usb_cfg);

    elm_wifi_start_ex(GVRET_SSID, GVRET_PORT, on_net_rx, on_net_link);

    xTaskCreate(gvret_rx_can_task, "gvret_can_rx", 4096, NULL, 5, NULL);
    xTaskCreate(gvret_usb_task,    "gvret_usb",    4096, NULL, 4, NULL);
    xTaskCreate(gvret_screen_task, "gvret_screen", 4096, NULL, 1, NULL);
    ESP_LOGI(GVRET_TAG, "SavvyCAN (GVRET) ready: Wi-Fi '%s' port %d, or USB", GVRET_SSID, GVRET_PORT);
}
