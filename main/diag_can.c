#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "constants.h"
#include "display.h"
#include "twai.h"
#include "canstats.h"
#include "bench_sim.h"
#include "diag_can.h"
#include "stackwatch.h"

#define DIAG_TAG            "DiagCAN"
#define RX_QUEUE_LEN        128
#define FC_TIMEOUT_MS       1000        // ISO 15765-2 N_Bs / N_Cr
#define PENDING_WAIT_MS     5000        // after NRC 0x78 (response pending)
#define PAD_BYTE            0x55

typedef struct {
    uint32_t    id;
    uint8_t     len;
    uint8_t     data[8];
} frame_t;

typedef enum { SLOT_EMPTY, SLOT_ASSEMBLING, SLOT_DONE } slot_state_t;

static QueueHandle_t        rx_queue    = NULL;
static SemaphoreHandle_t    req_mutex   = NULL;
static volatile bool        capturing   = false;
static volatile int64_t     last_rx_us  = 0;
static bool                 bench_mode  = false;
static bool                 started     = false;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static void queue_frame(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (!capturing || len > 8) return;
    frame_t f = { .id = id, .len = len };
    memcpy(f.data, data, len);
    xQueueSend(rx_queue, &f, 0);
}

// Frames the simulator sends to the tester (bench mode)
static void sim_sink(uint32_t id, const uint8_t *data, uint16_t len)
{
    last_rx_us = esp_timer_get_time();
    g_rx_count++;
    queue_frame(id, data, (uint8_t)len);
}

static void rx_task(void *arg)
{
    twai_message_t m;
    while (1) {
        if (twai_receive(&m, pdMS_TO_TICKS(100)) != ESP_OK) continue;
        last_rx_us = esp_timer_get_time();
        canstats_on_frame(m.identifier, m.data_length_code);
        g_rx_count++;
        if (m.extd || m.rtr || m.identifier < 0x600) continue;      // diagnostic ids only
        queue_frame(m.identifier, m.data, m.data_length_code);
    }
}

void diag_can_start(bool bench)
{
    if (started) return;
    started = true;
    bench_mode = bench;
    rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(frame_t));
    req_mutex = xSemaphoreCreateMutex();

    if (bench) {
        bench_sim_set_sink(sim_sink);
        bench_sim_start();
        ESP_LOGW(DIAG_TAG, "Using the bench simulator instead of the CAN bus");
    } else {
        twai_init();
        twai_start_raw();
        stackwatch_create(rx_task, "diag_rx", 3072, NULL, TWAI_TASK_PRIO);
    }
}

uint32_t diag_can_ms_since_rx(void)
{
    if (last_rx_us == 0) return UINT32_MAX;
    return (uint32_t)((esp_timer_get_time() - last_rx_us) / 1000);
}

static void tx_frame(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint8_t buf[8];
    memset(buf, PAD_BYTE, sizeof(buf));
    memcpy(buf, data, len);
    g_tx_count++;
    if (bench_mode) {
        bench_sim_send_can(id, buf, 8);
        return;
    }
    twai_message_t m = { .identifier = id, .data_length_code = 8 };
    memcpy(m.data, buf, 8);
    twai_send(&m);
}

void diag_default_rx_range(uint32_t tx_id, uint32_t *lo, uint32_t *hi)
{
    if (tx_id == DIAG_FUNCTIONAL_ID)               { *lo = 0x7E8; *hi = 0x7EF; }
    else if (tx_id >= 0x7E0 && tx_id <= 0x7E7)     { *lo = *hi = tx_id + 8; }
    else if (tx_id >= 0x700 && tx_id <= 0x795)     { *lo = *hi = tx_id + 0x6A; }
    else                                           { *lo = 0x600; *hi = 0x7FF; }
}

static bool wait_frame(frame_t *f, int64_t deadline_ms)
{
    int64_t left = deadline_ms - now_ms();
    if (left < 0) left = 0;
    return xQueueReceive(rx_queue, f, pdMS_TO_TICKS(left)) == pdTRUE;
}

static void separation_delay(uint8_t st)
{
    if (st == 0) return;
    if (st <= 0x7F) {
        if (st < 2) esp_rom_delay_us(st * 1000);
        else vTaskDelay(pdMS_TO_TICKS(st) > 0 ? pdMS_TO_TICKS(st) : 1);
    } else if (st >= 0xF1 && st <= 0xF9) {
        esp_rom_delay_us((st - 0xF0) * 100);
    }
}

// Send the request, handling flow control for requests that do not fit in a single frame.
static bool send_request(const diag_request_t *rq)
{
    uint8_t buf[8];
    if (rq->request_len <= 7) {
        buf[0] = (uint8_t)rq->request_len;
        memcpy(buf + 1, rq->request, rq->request_len);
        tx_frame(rq->tx_id, buf, (uint8_t)(rq->request_len + 1));
        return true;
    }

    buf[0] = 0x10 | (rq->request_len >> 8);
    buf[1] = rq->request_len & 0xFF;
    memcpy(buf + 2, rq->request, 6);
    tx_frame(rq->tx_id, buf, 8);

    uint16_t pos = 6;
    uint8_t seq = 1, block = 0, st_min = 0, sent_in_block = 0;
    bool need_fc = true;
    while (pos < rq->request_len) {
        if (need_fc) {
            int waits = 0;
            while (1) {
                frame_t f;
                if (!wait_frame(&f, now_ms() + FC_TIMEOUT_MS)) return false;
                if (f.id < rq->rx_lo || f.id > rq->rx_hi || (f.data[0] >> 4) != 3) continue;
                uint8_t fs = f.data[0] & 0xF;
                if (fs == 0) { block = f.data[1]; st_min = f.data[2]; break; }
                if (fs == 1 && ++waits < 10) continue;
                return false;                                  // overflow or too many waits
            }
            sent_in_block = 0;
            need_fc = false;
        }
        uint16_t n = rq->request_len - pos;
        if (n > 7) n = 7;
        buf[0] = 0x20 | seq;
        memcpy(buf + 1, rq->request + pos, n);
        tx_frame(rq->tx_id, buf, (uint8_t)(n + 1));
        pos += n;
        seq = (seq + 1) & 0xF;
        if (pos < rq->request_len) {
            if (block && ++sent_in_block >= block) need_fc = true;
            else separation_delay(st_min);
        }
    }
    return true;
}

static int do_request(const diag_request_t *rq, diag_response_t *out, int out_max)
{
    slot_state_t state[DIAG_MAX_RESPONSES] = { SLOT_EMPTY };
    uint16_t expected[DIAG_MAX_RESPONSES] = { 0 };
    uint8_t seqn[DIAG_MAX_RESPONSES] = { 0 };
    bool functional = rq->tx_id == DIAG_FUNCTIONAL_ID;
    int limit = rq->max_responses ? rq->max_responses : out_max;
    if (limit > out_max) limit = out_max;

    if (!send_request(rq)) {
        ESP_LOGW(DIAG_TAG, "Request to 0x%03lX could not be sent (no flow control)", (unsigned long)rq->tx_id);
        return -1;
    }

    int64_t deadline = now_ms() + rq->timeout_ms;
    int done = 0;
    frame_t f;

    while (wait_frame(&f, deadline)) {
        if (f.id < rq->rx_lo || f.id > rq->rx_hi || f.len < 1) continue;
        uint8_t pci = f.data[0] >> 4;

        int slot = -1;
        for (int i = 0; i < out_max; i++) {
            if (state[i] != SLOT_EMPTY && out[i].id == f.id) { slot = i; break; }
        }

        if (pci == 0) {                                         // single frame
            uint8_t n = f.data[0] & 0xF;
            if (n == 0 || n > 7 || n + 1 > f.len || slot >= 0) continue;
            if (n == 3 && f.data[1] == 0x7F && f.data[3] == 0x78) {   // response pending: keep waiting
                deadline = now_ms() + PENDING_WAIT_MS;
                continue;
            }
            for (slot = 0; slot < out_max && state[slot] != SLOT_EMPTY; slot++) {}
            if (slot >= out_max) continue;
            out[slot].id = f.id;
            out[slot].len = n;
            memcpy(out[slot].data, f.data + 1, n);
            state[slot] = SLOT_DONE;
            done++;
        } else if (pci == 1) {                                  // first frame
            uint16_t total = ((f.data[0] & 0xF) << 8) | f.data[1];
            if (total <= 7 || slot >= 0) continue;
            if (total > DIAG_MAX_PAYLOAD) {
                uint8_t fc[3] = { 0x32, 0, 0 };
                tx_frame(functional ? f.id - 8 : rq->tx_id, fc, 3);
                continue;
            }
            for (slot = 0; slot < out_max && state[slot] != SLOT_EMPTY; slot++) {}
            if (slot >= out_max) continue;
            out[slot].id = f.id;
            memcpy(out[slot].data, f.data + 2, 6);
            out[slot].len = 6;
            expected[slot] = total;
            seqn[slot] = 1;
            state[slot] = SLOT_ASSEMBLING;
            uint8_t fc[3] = { 0x30, 0, 0 };
            tx_frame(functional ? f.id - 8 : rq->tx_id, fc, 3);
            int64_t cf_deadline = now_ms() + FC_TIMEOUT_MS;
            if (cf_deadline > deadline) deadline = cf_deadline;
        } else if (pci == 2 && slot >= 0 && state[slot] == SLOT_ASSEMBLING) {   // consecutive frame
            if ((f.data[0] & 0xF) != seqn[slot]) { state[slot] = SLOT_EMPTY; continue; }
            seqn[slot] = (seqn[slot] + 1) & 0xF;
            uint16_t n = expected[slot] - out[slot].len;
            if (n > 7) n = 7;
            if (n + 1 > f.len) n = f.len - 1;
            memcpy(out[slot].data + out[slot].len, f.data + 1, n);
            out[slot].len += n;
            if (out[slot].len >= expected[slot]) {
                state[slot] = SLOT_DONE;
                done++;
            } else {
                int64_t cf_deadline = now_ms() + FC_TIMEOUT_MS;
                if (cf_deadline > deadline) deadline = cf_deadline;
            }
        } else {
            continue;
        }

        if (state[slot] == SLOT_DONE) {
            if (!functional || done >= limit) break;
            int64_t extra = now_ms() + rq->extra_ms;            // give the other ECUs time to answer
            if (extra < deadline) deadline = extra;
        }
    }

    // Compact the finished responses to the front
    int n = 0;
    for (int i = 0; i < out_max; i++) {
        if (state[i] != SLOT_DONE) continue;
        if (n != i) out[n] = out[i];
        n++;
    }
    return n;
}

int diag_can_request(const diag_request_t *rq, diag_response_t *out, int out_max)
{
    if (!started || !rq || !rq->request || rq->request_len == 0 || rq->request_len > DIAG_MAX_PAYLOAD) return -1;
    if (out_max > DIAG_MAX_RESPONSES) out_max = DIAG_MAX_RESPONSES;

    xSemaphoreTake(req_mutex, portMAX_DELAY);
    frame_t drop;
    while (xQueueReceive(rx_queue, &drop, 0) == pdTRUE) {}
    capturing = true;
    int result = do_request(rq, out, out_max);
    capturing = false;
    xSemaphoreGive(req_mutex);
    return result;
}

int diag_can_obd(uint32_t tx_id, const uint8_t *request, uint16_t len, diag_response_t *out, int out_max, uint32_t timeout_ms)
{
    diag_request_t rq = {
        .tx_id = tx_id, .request = request, .request_len = len,
        .timeout_ms = timeout_ms, .extra_ms = 60,
    };
    diag_default_rx_range(tx_id, &rq.rx_lo, &rq.rx_hi);
    return diag_can_request(&rq, out, out_max);
}
