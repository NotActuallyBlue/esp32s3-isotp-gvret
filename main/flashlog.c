#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "flashlog.h"

#define FLASHLOG_TAG            "FlashLog"

// The "log" partition is split into slots, one per boot, used round-robin so the
// last few sessions survive (e.g. a car session followed by a boot on the desk).
// A slot is fully erased at boot so no flash erase happens while the bridge runs.
#define FLASHLOG_SUBTYPE        0x40
#define FLASHLOG_SLOT_COUNT     6
#define FLASHLOG_MAGIC          0x474F4C42  // "BLOG"
#define FLASHLOG_HEADER_SIZE    32
#define FLASHLOG_RINGBUF_SIZE   (16 * 1024)
#define FLASHLOG_LINE_SIZE      256
#define FLASHLOG_FLUSH_MS       500
#define FLASHLOG_TASK_STACK     3072
#define FLASHLOG_TASK_PRIO      1

typedef struct {
    uint32_t magic;
    uint32_t boot_count;
    uint32_t reset_reason;
    uint32_t slot_size;
} flashlog_header_t;

static const esp_partition_t*   log_partition   = NULL;
static RingbufHandle_t          log_ringbuf     = NULL;
static SemaphoreHandle_t        log_fmt_mutex   = NULL;
static vprintf_like_t           log_orig_vprintf = NULL;
static char                     log_line[FLASHLOG_LINE_SIZE];
static uint8_t                  log_flush_buf[2048];
static uint32_t                 log_slot_start  = 0;
static uint32_t                 log_slot_size   = 0;
static uint32_t                 log_write_pos   = 0;
static volatile uint32_t        log_dropped     = 0;
static bool                     log_full        = false;
static uint32_t                 log_boot_number = 0;

static int flashlog_vprintf(const char* fmt, va_list args)
{
    va_list copy;
    va_copy(copy, args);
    int ret = log_orig_vprintf ? log_orig_vprintf(fmt, args) : vprintf(fmt, args);

    if (log_ringbuf && !xPortInIsrContext() && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        if (xSemaphoreTake(log_fmt_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            int len = vsnprintf(log_line, sizeof(log_line), fmt, copy);
            if (len > 0) {
                if (len >= (int)sizeof(log_line)) {
                    len = sizeof(log_line) - 1;
                    log_line[len - 1] = '\n';
                }
                if (xRingbufferSend(log_ringbuf, log_line, len, 0) != pdTRUE) {
                    log_dropped++;
                }
            }
            xSemaphoreGive(log_fmt_mutex);
        } else {
            log_dropped++;
        }
    }

    va_end(copy);
    return ret;
}

static void flashlog_write(const void* data, size_t size)
{
    if (log_full) {
        return;
    }

    static const char full_msg[] = "\n[FLASHLOG] slot full, logging stopped\n";
    if (log_write_pos + size > log_slot_size - sizeof(full_msg)) {
        esp_partition_write(log_partition, log_slot_start + log_write_pos, full_msg, sizeof(full_msg) - 1);
        log_full = true;
        return;
    }

    if (esp_partition_write(log_partition, log_slot_start + log_write_pos, data, size) == ESP_OK) {
        log_write_pos += size;
    }
}

static void flashlog_task(void* arg)
{
    uint32_t reported_dropped = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(FLASHLOG_FLUSH_MS));

        // Gather everything queued since the last flush into one flash write
        size_t used = 0;
        while (used < sizeof(log_flush_buf)) {
            size_t size = 0;
            void* item = xRingbufferReceiveUpTo(log_ringbuf, &size, 0, sizeof(log_flush_buf) - used);
            if (!item) {
                break;
            }
            memcpy(log_flush_buf + used, item, size);
            used += size;
            vRingbufferReturnItem(log_ringbuf, item);
        }

        uint32_t dropped = log_dropped;
        if (dropped != reported_dropped) {
            char msg[48];
            int len = snprintf(msg, sizeof(msg), "[FLASHLOG] %lu lines dropped\n", (unsigned long)(dropped - reported_dropped));
            reported_dropped = dropped;
            if (used + len <= sizeof(log_flush_buf)) {
                memcpy(log_flush_buf + used, msg, len);
                used += len;
            }
        }

        if (used) {
            flashlog_write(log_flush_buf, used);
        }
    }
}

void flashlog_init(void)
{
    log_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, FLASHLOG_SUBTYPE, "log");
    if (!log_partition) {
        ESP_LOGE(FLASHLOG_TAG, "No log partition, flash logging disabled");
        return;
    }

    log_slot_size = (log_partition->size / FLASHLOG_SLOT_COUNT) & ~(log_partition->erase_size - 1);

    // Pick the slot after the one with the highest boot count
    uint32_t max_boot = 0;
    int max_slot = -1;
    for (int i = 0; i < FLASHLOG_SLOT_COUNT; i++) {
        flashlog_header_t header;
        if (esp_partition_read(log_partition, i * log_slot_size, &header, sizeof(header)) == ESP_OK &&
            header.magic == FLASHLOG_MAGIC && header.boot_count != 0xFFFFFFFF &&
            (max_slot < 0 || header.boot_count > max_boot)) {
            max_boot = header.boot_count;
            max_slot = i;
        }
    }

    int slot = (max_slot + 1) % FLASHLOG_SLOT_COUNT;
    log_slot_start = slot * log_slot_size;
    if (esp_partition_erase_range(log_partition, log_slot_start, log_slot_size) != ESP_OK) {
        ESP_LOGE(FLASHLOG_TAG, "Erase failed, flash logging disabled");
        return;
    }

    uint8_t header_buf[FLASHLOG_HEADER_SIZE];
    memset(header_buf, 0xFF, sizeof(header_buf));
    flashlog_header_t header = {
        .magic = FLASHLOG_MAGIC,
        .boot_count = max_slot < 0 ? 1 : max_boot + 1,
        .reset_reason = esp_reset_reason(),
        .slot_size = log_slot_size,
    };
    memcpy(header_buf, &header, sizeof(header));
    esp_partition_write(log_partition, log_slot_start, header_buf, sizeof(header_buf));
    log_write_pos = FLASHLOG_HEADER_SIZE;

    log_boot_number = header.boot_count;

    log_fmt_mutex = xSemaphoreCreateMutex();
    log_ringbuf = xRingbufferCreate(FLASHLOG_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!log_fmt_mutex || !log_ringbuf) {
        ESP_LOGE(FLASHLOG_TAG, "Out of memory, flash logging disabled");
        log_ringbuf = NULL;
        return;
    }

    xTaskCreate(flashlog_task, "flashlog", FLASHLOG_TASK_STACK, NULL, FLASHLOG_TASK_PRIO, NULL);
    log_orig_vprintf = esp_log_set_vprintf(flashlog_vprintf);

    ESP_LOGI(FLASHLOG_TAG, "Boot #%lu logging to slot %d (reset reason %d)",
             (unsigned long)header.boot_count, slot, (int)header.reset_reason);
}

uint32_t flashlog_boot_number(void)
{
    return log_boot_number;
}

uint8_t flashlog_used_percent(void)
{
    if (!log_slot_size) {
        return 0;
    }
    return (uint8_t)(((uint64_t)log_write_pos * 100) / log_slot_size);
}
