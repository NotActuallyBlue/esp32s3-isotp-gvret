#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "isotp.h"
#include "ble_server.h"
#include "isotp_link_containers.h"
#include "twai.h"
#include "persist.h"
#include "constants.h"
#include "eeprom.h"
#include "uart.h"
#include "connection_handler.h"
#include "isotp_bridge.h"
#include "display.h"
#include "mode_mgr.h"
#include "gvret.h"
#include "flashlog.h"
#include "bench_sim.h"
#include "ota_update.h"
#include "diag.h"
#include "elm_mode.h"
#include "power_mgr.h"
#include "esp_sleep.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "canstats.h"
#include "driver/twai.h"

SemaphoreHandle_t sync_task_sem = NULL;

#define MAIN_TAG    "Main"

static bool ble_busy(void) {
    return ble_connected();
}

static void app_ble_connected(void) {
    display_set_status("BLE ISO-TP", "CONNECTED", COLOR_GREEN);
    bridge_connect();
}

static void app_ble_disconnected(void) {
    display_set_status("BLE ISO-TP", "DISCONNECTED", COLOR_YELLOW);
    bridge_disconnect();
}

void app_main(void)
{
    flashlog_init();
    ESP_LOGI(MAIN_TAG, "Application starting");

    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    uint32_t flash_size = 0;
    esp_flash_get_physical_size(NULL, &flash_size);
    ESP_LOGI(MAIN_TAG, "Build %s %s (IDF %s), running from %s, %lu MB flash",
             app->date, app->time, app->idf_ver, running ? running->label : "?", (unsigned long)(flash_size >> 20));

    // Initialize display immediately
    display_init();

    // Initialize non-volatile settings and read operating mode
    eeprom_init();
    mode_mgr_init();

    dongle_mode_t current_mode = mode_mgr_get();

    // Setup FreeRTOS task synchronization
    sync_task_sem = xSemaphoreCreateBinary();

    bool bench = mode_mgr_is_bench();
    display_set_bench(bench);

    if (esp_reset_reason() == ESP_RST_DEEPSLEEP) {
        uint32_t wake = esp_sleep_get_wakeup_causes();
        ESP_LOGI(MAIN_TAG, "Woken from deep sleep by %s", (wake & BIT(ESP_SLEEP_WAKEUP_EXT1)) ? "the CAN bus or the BOOT button" : "another source");
    }

    if (current_mode == OP_MODE_SIMOS_BLE) {
        if (bench) {
            ESP_LOGI(MAIN_TAG, "Booting in SIMOS BLE Mode on the BENCH SIMULATOR (virtual modules, no CAN bus)");
        } else {
            ESP_LOGI(MAIN_TAG, "Booting in SIMOS BLE ISO-TP Mode");
        }
        display_set_status("BLE ISO-TP", "READY", COLOR_CYAN);

        // Core hardware & protocol stacks for Simos BLE
        ble_server_init();
        ch_init();
        uart_init();
        if (!bench) twai_init();
        isotp_init();
        persist_init();

        // Read configured GAP name
        char* gapName = eeprom_read_str(BLE_GAP_KEY);
        if (gapName) {
            ble_set_gap_name(gapName, false);
            free(gapName);
        }

        // Setup BLE server callbacks
        ble_server_callbacks callbacks = {
            .data_received = bridge_received_ble,
            .notifications_subscribed = app_ble_connected,
            .notifications_unsubscribed = app_ble_disconnected
        };

        // Start tasks
        ble_server_start(callbacks);
        if (bench) {
            bench_sim_start();
        } else {
            twai_start_task();
        }
        isotp_start_task();
        persist_start_task();
        uart_start_task();
        ch_start_task();

        ESP_LOGI(MAIN_TAG, "Simos BLE services running.");
        if (!bench) power_mgr_start(ble_busy);
    } 
    else if (current_mode == OP_MODE_DIAG) {
        ESP_LOGI(MAIN_TAG, "Booting in DIAG Mode%s", bench ? " (bench simulator)" : "");
        display_set_status("DIAG", "READY", COLOR_CYAN);
        diag_start(bench);
        if (!bench) power_mgr_start(NULL);
    }
    else if (current_mode == OP_MODE_ELM327) {
        ESP_LOGI(MAIN_TAG, "Booting in ELM327 Mode%s", bench ? " (bench simulator)" : "");
        display_set_status("ELM327", "READY", COLOR_CYAN);
        elm_mode_start(bench);
        if (!bench) power_mgr_start(elm_mode_clients_connected);
    }
    else if (current_mode == OP_MODE_WIFI_UPDATE) {
        ESP_LOGI(MAIN_TAG, "Booting in WIFI UPDATE Mode");
        ota_update_start();
    }
    else if (current_mode == OP_MODE_SAVVYCAN_GVRET) {
        ESP_LOGI(MAIN_TAG, "Booting in SavvyCAN GVRET Mode%s", bench ? " (bench simulator)" : "");
        display_set_status("SAVVYCAN", "READY", COLOR_CYAN);

        // The GVRET task is the only reader of CAN frames here, so the receive task of the Simos bridge is not started
        if (!bench) {
            twai_init();
            twai_start_raw();
        }
        gvret_start(bench);
        if (!bench) power_mgr_start(gvret_client_connected);

        ESP_LOGI(MAIN_TAG, "SavvyCAN GVRET services running.");
    }

    // Main system heartbeat loop. A freshly updated image is only kept once it has run for 20 s;
    // if it crashes before that, the bootloader rolls back to the previous image.
    int uptime_ticks = 0;
    int heap_ticks = 0;
    int bus_ticks = 0, bus_quiet_logs = 0;
    uint32_t bus_last_frames = 0;
    uint32_t bus_last_lost = 0;
    bool bus_was_alive = true;      // so a silent bus is logged at the first check
    bool image_validated = false;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        // Bus timeline: when the car's network wakes up or goes quiet. Logged on every change, and once a minute.
        if (!bench && current_mode != OP_MODE_WIFI_UPDATE && ++bus_ticks >= 20) {
            bus_ticks = 0;
            canstats_totals_t totals;
            canstats_get_totals(&totals);
            uint32_t delta = totals.frames - bus_last_frames;
            bus_last_frames = totals.frames;
            bool alive = delta > 0;
            twai_status_info_t st;
            bool have_state = twai_get_status_info(&st) == ESP_OK;
            uint32_t lost = have_state ? st.rx_missed_count + st.rx_overrun_count : 0;
            if (alive != bus_was_alive || ++bus_quiet_logs >= 6 || lost != bus_last_lost) {
                bus_quiet_logs = 0;
                bus_was_alive = alive;
                bus_last_lost = lost;
                ESP_LOGI(MAIN_TAG, "Bus: %s, %lu frame(s) in the last 10 s, %lu unique id(s), %lu lost, CAN %s (TEC %lu, REC %lu)",
                         alive ? "ALIVE" : "SILENT", (unsigned long)delta, (unsigned long)totals.unique_ids, (unsigned long)lost,
                         !have_state ? "n/a" : st.state == TWAI_STATE_RUNNING ? "running" : st.state == TWAI_STATE_BUS_OFF ? "BUS OFF" : "other",
                         have_state ? (unsigned long)st.tx_error_counter : 0UL, have_state ? (unsigned long)st.rx_error_counter : 0UL);
            }
        }

        // Heap health, so a slow leak or a tight spot shows up in the flash log (one short line every 30 s)
        if (++heap_ticks >= 60) {
            heap_ticks = 0;
            ESP_LOGI(MAIN_TAG, "Heap: free %lu, lowest ever %lu, largest block %lu bytes", (unsigned long)esp_get_free_heap_size(),
                     (unsigned long)esp_get_minimum_free_heap_size(), (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
        }

        if (!image_validated && ++uptime_ticks >= 40) {
            image_validated = true;
            if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
                ESP_LOGI(MAIN_TAG, "Image marked valid");
            }
        }
    }
}