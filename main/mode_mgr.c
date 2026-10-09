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
#include "stackwatch.h"
#include "ble_access.h"
#include "key_logic.h"

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
//   hold  2 s .. 6.5 s   switch to the next saved mode, every 1.5 s further along (Simos, SavvyCAN, Diag, ELM327)
//   hold  6.5 s .. 8 s   bench simulator (this boot only)
//   hold  8 s .. 9.5 s   Wi-Fi firmware update (this boot only)
//   hold  9.5 s +        cancel (restart unchanged)
#define HOLD_SWITCH_MS   2000
#define HOLD_STEP_MS     1500
#define HOLD_BENCH_MS    (HOLD_SWITCH_MS + (OP_MODE_COUNT - 1) * HOLD_STEP_MS)
#define HOLD_UPDATE_MS   (HOLD_BENCH_MS + HOLD_STEP_MS)
#define HOLD_CANCEL_MS   (HOLD_UPDATE_MS + HOLD_STEP_MS)
#define TAP_MAX_MS       800
#define BUTTON_POLL_MS   50

// 1 .. OP_MODE_COUNT-1 = switch forward by that many saved modes
typedef enum { ZONE_NONE = 0, ZONE_BENCH = 100, ZONE_UPDATE, ZONE_CANCEL } hold_zone_t;

static bool bench_boot = false;                         // this boot runs against the simulator
static void (*tap_handler)(void) = NULL;

static const char* get_mode_name(dongle_mode_t mode)
{
    switch (mode) {
        case OP_MODE_SIMOS_BLE:      return "SIMOS BLE";
        case OP_MODE_SAVVYCAN_GVRET: return "SAVVYCAN";
        case OP_MODE_DIAG:           return "DIAG";
        case OP_MODE_ELM327:         return "ELM327";
        case OP_MODE_BENCH_SIM:      return "BENCH SIM";
        case OP_MODE_WIFI_UPDATE:    return "WIFI UPDATE";
        default:                     return "UNKNOWN";
    }
}

static hold_zone_t zone_for(int held_ms)
{
    if (held_ms < HOLD_SWITCH_MS) return ZONE_NONE;
    if (held_ms < HOLD_BENCH_MS)  return (hold_zone_t)(1 + (held_ms - HOLD_SWITCH_MS) / HOLD_STEP_MS);
    if (held_ms < HOLD_UPDATE_MS) return ZONE_BENCH;
    if (held_ms < HOLD_CANCEL_MS) return ZONE_UPDATE;
    return ZONE_CANCEL;
}

static dongle_mode_t saved_mode_ahead(int steps)
{
    return (dongle_mode_t)((saved_mode + steps) % OP_MODE_COUNT);
}

static const char* short_mode_name(dongle_mode_t mode)
{
    switch (mode) {
        case OP_MODE_SAVVYCAN_GVRET: return "SAVVYCAN";
        case OP_MODE_DIAG:           return "DIAG";
        case OP_MODE_ELM327:         return "ELM327";
        default:                     return "SIMOS";
    }
}

static void show_prompt(hold_zone_t zone)
{
    if (zone > ZONE_NONE && zone < ZONE_BENCH) {
        display_set_prompt("RELEASE TO", short_mode_name(saved_mode_ahead(zone)), COLOR_YELLOW);
        return;
    }
    switch (zone) {
        case ZONE_BENCH:  display_set_prompt("RELEASE FOR", "BENCH SIM",   COLOR_ACCENT);   break;
        case ZONE_UPDATE: display_set_prompt("RELEASE FOR", "WIFI UPDATE", COLOR_ORANGE); break;
        case ZONE_CANCEL: display_set_prompt("RELEASE TO",  "CANCEL",      COLOR_MUTED); break;
        default: break;
    }
}

static void restart_with(hold_zone_t zone)
{
    if (zone > ZONE_NONE && zone < ZONE_BENCH) {
        ESP_LOGI(TAG, "Switching saved mode forward by %d", (int)zone);
        mode_mgr_set(saved_mode_ahead(zone));
        display_set_prompt(short_mode_name(saved_mode), "REBOOTING", COLOR_YELLOW);
        vTaskDelay(pdMS_TO_TICKS(800));
        esp_restart();
    }
    switch (zone) {
        case ZONE_BENCH:
            ESP_LOGI(TAG, "Restarting into the bench simulator (one-shot)");
            mode_mgr_request_oneshot(OP_MODE_BENCH_SIM);
            display_set_prompt("BENCH SIM", "REBOOTING", COLOR_ACCENT);
            break;
        case ZONE_UPDATE:
            ESP_LOGI(TAG, "Restarting into Wi-Fi update mode (one-shot)");
            mode_mgr_request_oneshot(OP_MODE_WIFI_UPDATE);
            display_set_prompt("WIFI UPDATE", "REBOOTING", COLOR_ORANGE);
            break;
        default:
            ESP_LOGI(TAG, "Menu cancelled, restarting");
            display_set_prompt("CANCELLED", "REBOOTING", COLOR_MUTED);
            break;
    }

    // Brief pause so the display write finishes cleanly
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

// KEY (the second button) in the modes that do not use it: a 2 s hold opens Bluetooth again after it locked, a 5 s hold shows the About
// screen with the QR code. Diag mode uses KEY for its own menu (it has an ABOUT item) and the update mode keeps its progress on screen.
// The rules are in key_logic.c.
#define KEY_BUTTON_PIN   14

static void mode_button_monitor_task(void *pvParameters)
{
    int held_ms = 0;
    bool woke_screen = false;
    hold_zone_t shown = ZONE_NONE;
    key_state_t key = { 0 };
    bool about_allowed = current_mode != OP_MODE_DIAG && current_mode != OP_MODE_WIFI_UPDATE;

    if (about_allowed) {
        gpio_config_t key_cfg = {
            .pin_bit_mask = 1ULL << KEY_BUTTON_PIN,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&key_cfg);
    }

    while (1) {
        if (about_allowed) {
            key_actions_t act = key_step(&key, gpio_get_level(KEY_BUTTON_PIN) == 0, BUTTON_POLL_MS, !ble_access_open(),
                                         display_pairing_active(), display_about_active());
            if (act.hide_pairing) display_pairing_end();
            if (act.reopen_bluetooth) ble_access_poke();
            if (act.show_about) display_show_about(true);
            if (act.hide_about) display_show_about(false);
        }

        // BOOT button is active LOW (0 when pressed)
        if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
            if (held_ms == 0) {
                woke_screen = !display_is_awake();      // the first press only wakes a sleeping screen
                if (display_pairing_active()) {                 // a press hides the Pairing Mode screen and does nothing else
                    display_pairing_end();
                    woke_screen = true;
                }
                if (display_about_active()) {           // any BOOT press closes the About screen and does nothing else
                    display_show_about(false);
                    woke_screen = true;
                }
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
                if (tap_handler) tap_handler();
                else display_next_page();
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
#ifdef SET_SAVED_MODE
    // Test builds only: store this saved mode once at boot (-DSET_SAVED_MODE=0 Simos, 1 SavvyCAN, 2 Diag, 3 ELM327). Used to put the
    // dongle back in a known mode after bench testing, since a serial reset from a PC can look like a long BOOT press.
    if (saved_mode != (dongle_mode_t)SET_SAVED_MODE) {
        saved_mode = (dongle_mode_t)SET_SAVED_MODE;
        nvs_handle_t nvs_w;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_w) == ESP_OK) {
            nvs_set_u8(nvs_w, NVS_KEY_MODE, (uint8_t)saved_mode);
            nvs_commit(nvs_w);
            nvs_close(nvs_w);
        }
        ESP_LOGW(TAG, "Saved mode set to %d by the test build", (int)saved_mode);
    }
#endif
    current_mode = saved_mode;

    // A one-shot request only counts right after a software restart, and only once
    if (esp_reset_reason() == ESP_RST_SW && oneshot_magic == ONESHOT_MAGIC) {
        if (oneshot_mode == OP_MODE_BENCH_SIM) {
            bench_boot = true;                  // keep the saved mode, but talk to the simulator
        } else if (oneshot_mode == OP_MODE_WIFI_UPDATE) {
            current_mode = OP_MODE_WIFI_UPDATE;
        }
    }
    oneshot_magic = 0;

#ifdef FORCE_BENCH_SIM
    bench_boot = true;                     // test builds only: PLATFORMIO_BUILD_FLAGS=-DFORCE_BENCH_SIM pio run -t upload
#endif
#ifdef FORCE_MODE
    current_mode = (dongle_mode_t)FORCE_MODE;   // test builds only: -DFORCE_MODE=2 (diag) or 3 (ELM327), not saved
#endif
#ifdef FORCE_WIFI_UPDATE
    current_mode = OP_MODE_WIFI_UPDATE;    // test builds only, see FORCE_BENCH_SIM
#endif

    ESP_LOGI(TAG, "Active mode: %s (%d)%s, saved mode: %s", get_mode_name(current_mode), current_mode,
             bench_boot ? " on the bench simulator" : "", get_mode_name(saved_mode));

    // Launch the runtime button listener task
    stackwatch_create(mode_button_monitor_task, "mode_btn_task", 2048, NULL, 1);
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
    mode_mgr_set(saved_mode_ahead(1));
}

bool mode_mgr_is_bench(void)
{
    return bench_boot;
}

void mode_mgr_set_tap_handler(void (*handler)(void))
{
    tap_handler = handler;
}

void mode_mgr_request_oneshot(dongle_mode_t mode)
{
    oneshot_mode = (uint32_t)mode;
    oneshot_magic = ONESHOT_MAGIC;
}
