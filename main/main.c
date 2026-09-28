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
#include "elm327.h"

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

static void app_elm_ble_connected(void) {
    display_set_status("ELM327 OBD2", "CONNECTED", COLOR_GREEN);
}

static void app_elm_ble_disconnected(void) {
    display_set_status("ELM327 OBD2", "WAITING BLE", COLOR_CYAN);
}

void app_main(void)
{
    ESP_LOGI(MAIN_TAG, "Application starting");

    // Initialize display immediately
    display_init();

    // Initialize non-volatile settings and read operating mode
    eeprom_init();
    mode_mgr_init();

    dongle_mode_t current_mode = mode_mgr_get();

    // Setup FreeRTOS task synchronization
    sync_task_sem = xSemaphoreCreateBinary();

    if (current_mode == OP_MODE_SIMOS_BLE) {
        ESP_LOGI(MAIN_TAG, "Booting in SIMOS BLE ISO-TP Mode");
        display_set_status("BLE ISO-TP", "READY", COLOR_CYAN);

        // Core hardware & protocol stacks for Simos BLE
        ble_server_init();
        ch_init();
        uart_init();
        twai_init();
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
        twai_start_task();
        isotp_start_task();
        persist_start_task();
        uart_start_task();
        ch_start_task();

        ESP_LOGI(MAIN_TAG, "Simos BLE services running.");
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
    else if (current_mode == OP_MODE_ELM327) {
        ESP_LOGI(MAIN_TAG, "Booting in ELM327 OBD2 Mode");
        display_set_status("ELM327 OBD2", "WAITING BLE", COLOR_CYAN);

        // Hardware initialization
        twai_init();
        twai_start_task();
        ble_server_init();

        // Optional custom advertised BLE name for standard OBD apps
        ble_set_gap_name("OBDII-BLE", false);

        // Wire BLE rx directly to the ELM ASCII engine
        ble_server_callbacks callbacks = {
            .data_received = elm327_rx_data,
            .notifications_subscribed = app_elm_ble_connected,
            .notifications_unsubscribed = app_elm_ble_disconnected
        };

        ble_server_start(callbacks);
        elm327_init();
        elm327_start();

        ESP_LOGI(MAIN_TAG, "ELM327 OBD2 services running.");
    }

    // Main system heartbeat loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}