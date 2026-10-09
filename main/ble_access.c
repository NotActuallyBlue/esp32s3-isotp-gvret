#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ble_access.h"
#include "stackwatch.h"

#define ACCESS_TAG "Access"

static ble_access_connected_fn  is_connected;
static ble_access_switch_fn     adv_on, adv_off;
static ble_access_changed_fn    on_changed;
static bool                     started;
static bool                     window_open = true;
static int64_t                  deadline_ms;
static portMUX_TYPE             lock = portMUX_INITIALIZER_UNLOCKED;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

bool ble_access_open(void)
{
    return !started || window_open;
}

void ble_access_poke(void)
{
    if (!started) return;
    bool reopened = false;
    taskENTER_CRITICAL(&lock);
        deadline_ms = now_ms() + BLE_ACCESS_WINDOW_MS;
        if (!window_open) {
            window_open = true;
            reopened = true;
        }
    taskEXIT_CRITICAL(&lock);
    if (reopened) {
        ESP_LOGI(ACCESS_TAG, "Button pressed: Bluetooth open for another %d s", BLE_ACCESS_WINDOW_MS / 1000);
        if (adv_on) adv_on();
        if (on_changed) on_changed(true);
    }
}

static void access_task(void *arg)
{
#ifdef ACCESS_TEST_POKE_AT
    int64_t started_ms = now_ms();
    bool poked = false;
#endif
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
#ifdef ACCESS_TEST_POKE_AT
        if (!poked && now_ms() - started_ms >= ACCESS_TEST_POKE_AT * 1000) {      // test builds only: pretend a button was pressed
            poked = true;
            ble_access_poke();
        }
#endif
        bool close = false;
        taskENTER_CRITICAL(&lock);
            if (window_open) {
                if (is_connected && is_connected()) {
                    deadline_ms = now_ms() + BLE_ACCESS_WINDOW_MS;          // a connected app keeps it open; the minute restarts on disconnect
                } else if (now_ms() >= deadline_ms) {
                    window_open = false;
                    close = true;
                }
            }
        taskEXIT_CRITICAL(&lock);
        if (close) {
            ESP_LOGW(ACCESS_TAG, "No Bluetooth connection for %d s: advertising stopped. A button press opens it again.", BLE_ACCESS_WINDOW_MS / 1000);
            if (adv_off) adv_off();
            if (on_changed) on_changed(false);
        }
    }
}

void ble_access_start(ble_access_connected_fn connected, ble_access_switch_fn advertising_on, ble_access_switch_fn advertising_off,
                      ble_access_changed_fn changed)
{
    is_connected = connected;
    adv_on = advertising_on;
    adv_off = advertising_off;
    on_changed = changed;
    deadline_ms = now_ms() + BLE_ACCESS_WINDOW_MS;
    window_open = true;
    started = true;
    stackwatch_create(access_task, "ble_access", 2560, NULL, 1);
    ESP_LOGI(ACCESS_TAG, "Bluetooth accepts new connections for %d s after start-up and after each button press", BLE_ACCESS_WINDOW_MS / 1000);
}
