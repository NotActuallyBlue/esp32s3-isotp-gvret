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
#include "gvret_parser.h"
#include "stackwatch.h"

#define GVRET_TAG       "GVRET"
#define GVRET_SSID      "ISOTP-SAVVYCAN"
#define GVRET_PORT      23              // the port SavvyCAN's "Network connection (GVRET)" uses by default
#define PROTO_BUILD_CAN_FRAME 0
#define BATCH_SIZE      1024

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
    frame.identifier = id;
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
static void parser_transmit(uint32_t id, bool extended, uint8_t dlc, const uint8_t *data)
{
    can_transmit(id, extended, dlc, data);
}

static uint32_t parser_now_us(void)
{
    return (uint32_t)esp_timer_get_time();
}

static gvret_parser_t usb_parser = { .write = write_usb, .transmit = parser_transmit, .now_us = parser_now_us };
static gvret_parser_t net_parser = { .write = write_net, .transmit = parser_transmit, .now_us = parser_now_us };

// Wi-Fi client sent bytes (runs in the TCP server task)
static void on_net_rx(const uint8_t *data, size_t len)
{
    gvret_parser_feed(&net_parser, data, len);
}

static void on_net_link(bool up)
{
    gvret_parser_reset(&net_parser);
    ESP_LOGI(GVRET_TAG, "SavvyCAN %s over Wi-Fi", up ? "connected" : "disconnected");
}

static void gvret_usb_task(void *arg)
{
    uint8_t rx_buf[128];
    while (1) {
        int len = usb_serial_jtag_read_bytes(rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(20));
        if (len > 0) gvret_parser_feed(&usb_parser, rx_buf, len);
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

        // Frames the controller had no room for (the reader was too slow) are the ones SavvyCAN never sees
        uint32_t lost = 0;
        twai_status_info_t st;
        if (!bench_mode && twai_get_status_info(&st) == ESP_OK) lost = st.rx_missed_count + st.rx_overrun_count;
        char lost_text[16];
        snprintf(lost_text, sizeof(lost_text), "%lu", (unsigned long)lost);

        display_detail_t d[] = {
            { "#WI-FI",    "",                            COLOR_CYAN },
            { "SSID",      elm_wifi_ssid(),               COLOR_WHITE },
            { "PASSWORD",  elm_wifi_password(),           COLOR_YELLOW },
            { "IP",        ELM_WIFI_IP,                   COLOR_WHITE },
            { "PORT",      port,                          COLOR_WHITE },
            { "#CAPTURE",  "",                            COLOR_CYAN },
            { "CLIENT",    net ? "Wi-Fi" : (usb ? "USB" : "none"), (net || usb) ? COLOR_GREEN : COLOR_MUTED },
            { "FRAMES RX", rx,                            COLOR_WHITE },
            { "FRAMES TX", tx,                            COLOR_WHITE },
            { "CAN BUS",   state,                         state_color },
            { "FRAMES LOST", lost_text,                   lost ? COLOR_RED : COLOR_GREEN },
        };
        display_set_details("SAVVYCAN", COLOR_CYAN, d, sizeof(d) / sizeof(d[0]));
        display_set_status("SAVVYCAN", (net || usb) ? "CONNECTED" : "READY", (net || usb) ? COLOR_GREEN : COLOR_CYAN);
        static uint32_t last_lost;
        if (lost != last_lost) {
            ESP_LOGW(GVRET_TAG, "CAN frames lost because the reader fell behind: %lu so far", (unsigned long)lost);
            last_lost = lost;
        }
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
        stackwatch_create(bench_traffic_task, "gvret_bench", 3072, NULL, 3);
    }

    // USB stays available for bench use with SavvyCAN
    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 2048,
    };
    usb_serial_jtag_driver_install(&usb_cfg);

    elm_wifi_start_ex(GVRET_SSID, GVRET_PORT, on_net_rx, on_net_link);

    stackwatch_create(gvret_rx_can_task, "gvret_can_rx", 4096, NULL, 5);
    stackwatch_create(gvret_usb_task,    "gvret_usb",    4096, NULL, 4);
    stackwatch_create(gvret_screen_task, "gvret_screen", 4096, NULL, 1);
    ESP_LOGI(GVRET_TAG, "SavvyCAN (GVRET) ready: Wi-Fi '%s' port %d, or USB", GVRET_SSID, GVRET_PORT);
}
