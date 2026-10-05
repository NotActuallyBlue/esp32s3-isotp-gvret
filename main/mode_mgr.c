#include "mode_mgr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "display.h"

#define TAG "MODE_MGR"
#define NVS_NAMESPACE "dongle_cfg"
#define NVS_KEY_MODE  "op_mode"

// One-shot mode request. RTC memory survives a software restart but not a power cycle, so these modes can
// never stick: the dongle always comes back in its saved mode after power is removed.
#define ONESHOT_MAGIC 0x4F53484F
static RTC_NOINIT_ATTR uint32_t oneshot_magic;
static RTC_NOINIT_ATTR uint32_t oneshot_mode;

static dongle_mode_t saved_mode   = OP_MODE_SIMOS_BLE;     // what is stored in NVS
static dongle_mode_t current_mode = OP_MODE_SIMOS_BLE;     // what is running now

// Button menu: hold the BOOT button and let go when the screen shows what you want
//   tap                  next page
//   hold  2 s .. 5 s     switch between the saved modes (Simos BLE / SavvyCAN)
//   hold  5 s .. 8 s     bench simulator (this boot only)
//   hold  8 s .. 11 s    Wi-Fi firmware update (this boot only)
//   hold  11 s +         cancel (restart unchanged)
#define HOLD_SWITCH_MS   2000
#define HOLD_BENCH_MS    5000
#define HOLD_UPDATE_MS   8000
#define HOLD_CANCEL_MS   11000
#define TAP_MAX_MS       800
#define BUTTON_POLL_MS   50

typedef enum { ZONE_NONE, ZONE_SWITCH, ZONE_BENCH, ZONE_UPDATE, ZONE_CANCEL } hold_zone_t;

static const char* get_mode_name(dongle_mode_t mode)
{
    switch (mode) {
        case OP_MODE_SIMOS_BLE:      return "SIMOS BLE";
        case OP_MODE_SAVVYCAN_GVRET: return "SAVVYCAN USB";
        case OP_MODE_BENCH_SIM:      return "BENCH SIM";
        case OP_MODE_WIFI_UPDATE:    return "WIFI UPDATE";
        default:                     return "UNKNOWN";
    }
}

static hold_zone_t zone_for(int held_ms)
{
    if (held_ms < HOLD_SWITCH_MS) return ZONE_NONE;
    if (held_ms < HOLD_BENCH_MS)  return ZONE_SWITCH;
    if (held_ms < HOLD_UPDATE_MS) return ZONE_BENCH;
    if (held_ms < HOLD_CANCEL_MS) return ZONE_UPDATE;
    return ZONE_CANCEL;
}

static dongle_mode_t next_saved_mode(void)
{
    return (dongle_mode_t)((saved_mode + 1) % OP_MODE_COUNT);
}

static const char* short_mode_name(dongle_mode_t mode)
{
    return mode == OP_MODE_SAVVYCAN_GVRET ? "SAVVYCAN" : "SIMOS";
}

static void show_prompt(hold_zone_t zone)
{
    switch (zone) {
        case ZONE_SWITCH: display_set_prompt("RELEASE TO",  short_mode_name(next_saved_mode()), COLOR_YELLOW); break;
        case ZONE_BENCH:  display_set_prompt("RELEASE FOR", "BENCH SIM",   COLOR_CYAN);   break;
        case ZONE_UPDATE: display_set_prompt("RELEASE FOR", "WIFI UPDATE", COLOR_ORANGE); break;
        case ZONE_CANCEL: display_set_prompt("RELEASE TO",  "CANCEL",      COLOR_RED);    break;
        default: break;
    }
}

static void restart_with(hold_zone_t zone)
{
    switch (zone) {
        case ZONE_SWITCH:
            ESP_LOGI(TAG, "Runtime mode toggle triggered!");
            mode_mgr_toggle();
            display_set_status(get_mode_name(saved_mode), "REBOOTING...", COLOR_YELLOW);
            break;
        case ZONE_BENCH:
            ESP_LOGI(TAG, "Restarting into the bench simulator (one-shot)");
            mode_mgr_request_oneshot(OP_MODE_BENCH_SIM);
            display_set_prompt("BENCH SIM", "REBOOTING", COLOR_CYAN);
            break;
        case ZONE_UPDATE:
            ESP_LOGI(TAG, "Restarting into Wi-Fi update mode (one-shot)");
            mode_mgr_request_oneshot(OP_MODE_WIFI_UPDATE);
            display_set_prompt("WIFI UPDATE", "REBOOTING", COLOR_ORANGE);
            break;
        default:
            ESP_LOGI(TAG, "Menu cancelled, restarting");
            display_set_prompt("CANCELLED", "REBOOTING", COLOR_RED);
            break;
    }

    // Brief pause so the display write finishes cleanly
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

static void mode_button_monitor_task(void *pvParameters)
{
    int held_ms = 0;
    bool woke_screen = false;
    hold_zone_t shown = ZONE_NONE;

    while (1) {
        // BOOT button is active LOW (0 when pressed)
        if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
            if (held_ms == 0) {
                woke_screen = !display_is_awake();      // the first press only wakes a sleeping screen
                display_power(true);
            }
            held_ms += BUTTON_POLL_MS;

            hold_zone_t zone = zone_for(held_ms);
            if (zone != shown) {
                shown = zone;
                show_prompt(zone);
            }
        } else if (held_ms > 0) {
            if (shown != ZONE_NONE) {
                restart_with(shown);
            } else if (held_ms < TAP_MAX_MS && !woke_screen) {
                display_next_page();
            }
            held_ms = 0;
            shown = ZONE_NONE;
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
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
                saved_mode = (dongle_mode_t)val;
            }
        }
        nvs_close(nvs);
    }
    current_mode = saved_mode;

    // A one-shot request only counts right after a software restart, and only once
    if (esp_reset_reason() == ESP_RST_SW && oneshot_magic == ONESHOT_MAGIC &&
        (oneshot_mode == OP_MODE_BENCH_SIM || oneshot_mode == OP_MODE_WIFI_UPDATE)) {
        current_mode = (dongle_mode_t)oneshot_mode;
    }
    oneshot_magic = 0;

#ifdef FORCE_BENCH_SIM
    current_mode = OP_MODE_BENCH_SIM;      // test builds only: PLATFORMIO_BUILD_FLAGS=-DFORCE_BENCH_SIM pio run -t upload
#endif
#ifdef FORCE_WIFI_UPDATE
    current_mode = OP_MODE_WIFI_UPDATE;    // test builds only, see FORCE_BENCH_SIM
#endif

    ESP_LOGI(TAG, "Active mode: %s (%d), saved mode: %s", get_mode_name(current_mode), current_mode, get_mode_name(saved_mode));

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
    saved_mode = mode;
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
    mode_mgr_set(next_saved_mode());
}

void mode_mgr_request_oneshot(dongle_mode_t mode)
{
    oneshot_mode = (uint32_t)mode;
    oneshot_magic = ONESHOT_MAGIC;
}
