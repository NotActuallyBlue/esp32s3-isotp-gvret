#include "mode_mgr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "display.h"

#define TAG "MODE_MGR"
#define NVS_NAMESPACE "dongle_cfg"
#define NVS_KEY_MODE  "op_mode"

static dongle_mode_t current_mode = OP_MODE_SIMOS_BLE;

static const char* get_mode_name(dongle_mode_t mode)
{
    switch (mode) {
        case OP_MODE_SIMOS_BLE:      return "SIMOS BLE";
        case OP_MODE_SAVVYCAN_GVRET: return "SAVVYCAN USB";
        default:                     return "UNKNOWN";
    }
}

static void mode_button_monitor_task(void *pvParameters)
{
    int held_counter_ms = 0;

    while (1) {
        // BOOT button is active LOW (0 when pressed)
        if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
            held_counter_ms += 50;

            // Optional: Wake the screen on initial press
            if (held_counter_ms == 50) {
                display_power(true);
            }

            // Once held for 2 full seconds (2000 ms)
            if (held_counter_ms >= 2000) {
                ESP_LOGI(TAG, "Runtime mode toggle triggered!");
                
                // Advance to the next mode
                mode_mgr_toggle();

                // Show target mode prompt on screen
                display_power(true);
                display_set_status(get_mode_name(current_mode), "REBOOTING...", COLOR_YELLOW);

                // Brief pause so the display write finishes cleanly
                vTaskDelay(pdMS_TO_TICKS(800));

                // Clean reboot into the selected mode
                esp_restart();
            }
        } else {
            held_counter_ms = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void mode_mgr_init(void)
{
    // Configure BOOT button with internal pull-up
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);

    // Read stored mode from NVS
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        uint8_t val = 0;
        if (nvs_get_u8(nvs, NVS_KEY_MODE, &val) == ESP_OK) {
            if (val < OP_MODE_COUNT) {
                current_mode = (dongle_mode_t)val;
            } else {
                current_mode = OP_MODE_SIMOS_BLE;
            }
        }
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "Active mode: %s (%d)", get_mode_name(current_mode), current_mode);

    // Launch the runtime button listener task
    xTaskCreate(mode_button_monitor_task, "mode_btn_task", 2048, NULL, 1, NULL);
}

dongle_mode_t mode_mgr_get(void)
{
    return current_mode;
}

void mode_mgr_set(dongle_mode_t mode)
{
    if (mode >= OP_MODE_COUNT) {
        mode = OP_MODE_SIMOS_BLE;
    }
    current_mode = mode;

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_KEY_MODE, (uint8_t)mode);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

void mode_mgr_toggle(void)
{
    dongle_mode_t next = (dongle_mode_t)((current_mode + 1) % OP_MODE_COUNT);
    mode_mgr_set(next);
}