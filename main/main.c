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
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_flash.h"

SemaphoreHandle_t sync_task_sem = NULL;

// Override POSIX getentropy to prevent libesp_libc TLS errno linker conflict
int getentropy(void *buffer, size_t length) {
    if (length > 256) {
        return -1;
    }
    esp_fill_random(buffer, length);
    return 0;
}

#define MAIN_TAG    "Main"

static void app_ble_connected(void) {
    display_set_status("BLE ISO-TP", "CONNECTED", COLOR_GREEN);
    bridge_connect();
}

static void app_ble_disconnected(void) {
    display_set_status("BLE ISO-TP", "DISCONNECTED", COLOR_RED);
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

    bool bench = (current_mode == OP_MODE_BENCH_SIM);
    display_set_bench(bench);

    if (current_mode == OP_MODE_SIMOS_BLE || bench) {
        if (bench) {
            ESP_LOGI(MAIN_TAG, "Booting in BENCH SIMULATOR Mode (virtual ECU and TCU, no CAN bus)");
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
    } 
    else if (current_mode == OP_MODE_WIFI_UPDATE) {
        ESP_LOGI(MAIN_TAG, "Booting in WIFI UPDATE Mode");
        ota_update_start();
    }
    else if (current_mode == OP_MODE_SAVVYCAN_GVRET) {
        ESP_LOGI(MAIN_TAG, "Booting in SavvyCAN GVRET Mode");
        display_set_status("SAVVYCAN", "CONNECTING...", COLOR_YELLOW);

        // Hardware initialization for raw bus sniffing
        twai_init();
        twai_start_task();

        // Start GVRET USB bridge engine
        gvret_start();

        ESP_LOGI(MAIN_TAG, "SavvyCAN GVRET services running.");
    }

    // Main system heartbeat loop. A freshly updated image is only kept once it has run for 20 s;
    // if it crashes before that, the bootloader rolls back to the previous image.
    int uptime_ticks = 0;
    bool image_validated = false;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        if (!image_validated && ++uptime_ticks >= 40) {
            image_validated = true;
            if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
                ESP_LOGI(MAIN_TAG, "Image marked valid");
            }
        }
    }
}