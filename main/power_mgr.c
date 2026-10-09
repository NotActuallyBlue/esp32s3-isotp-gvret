#include "power_mgr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs.h"
#include "constants.h"
#include "canstats.h"
#include "display.h"
#include "stackwatch.h"

#define POWER_TAG       "Power"
#define NVS_NAMESPACE   "dongle_cfg"
#define NVS_KEY_SLEEP   "sleep_min"
#define CHECK_MS        1000

static power_busy_fn        busy_fn;
static volatile int64_t     last_activity_ms;
static uint32_t             idle_minutes = POWER_IDLE_MINUTES_DEFAULT;
static int64_t              idle_ms;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

void power_mgr_poke(void)
{
    last_activity_ms = now_ms();
}

static void enter_deep_sleep(void)
{
    ESP_LOGW(POWER_TAG, "Idle for %lld s: entering deep sleep. A CAN frame or the BOOT button wakes the dongle.",
             (long long)(idle_ms / 1000));
    ESP_LOGI(POWER_TAG, "Pin levels going to sleep: CAN RX %d, BOOT %d", gpio_get_level(CAN_RX_PORT), gpio_get_level(BOOT_BUTTON_PIN));
    display_power(false);
    vTaskDelay(pdMS_TO_TICKS(1500));            // let the flash log write this line

    const uint64_t wake_pins = (1ULL << CAN_RX_PORT) | (1ULL << BOOT_BUTTON_PIN);
    rtc_gpio_pullup_en(BOOT_BUTTON_PIN);
    rtc_gpio_pulldown_dis(BOOT_BUTTON_PIN);
    esp_sleep_enable_ext1_wakeup_io(wake_pins, ESP_EXT1_WAKEUP_ANY_LOW);
#ifdef POWER_TEST
    esp_sleep_enable_timer_wakeup(15 * 1000000ULL);     // test builds only: wake by itself so the boot path can be checked
#endif
    esp_deep_sleep_start();
}

static void power_task(void *arg)
{
    canstats_totals_t totals;
    canstats_get_totals(&totals);
    uint32_t last_frames = totals.frames;
    last_activity_ms = now_ms();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CHECK_MS));

        canstats_get_totals(&totals);
        if (totals.frames != last_frames) {
            last_frames = totals.frames;
            power_mgr_poke();
        }
        if (busy_fn && busy_fn()) power_mgr_poke();
#ifndef POWER_TEST
        if (usb_serial_jtag_is_connected()) power_mgr_poke();       // powered from a computer, not the car
#endif

        if (now_ms() - last_activity_ms >= idle_ms) {
            enter_deep_sleep();
        }
    }
}

void power_mgr_start(power_busy_fn busy)
{
    busy_fn = busy;

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t minutes;
        if (nvs_get_u8(nvs, NVS_KEY_SLEEP, &minutes) == ESP_OK) idle_minutes = minutes;
        nvs_close(nvs);
    }

    idle_ms = (int64_t)idle_minutes * 60 * 1000;
#ifdef POWER_TEST
    idle_ms = POWER_TEST * 1000LL;                      // seconds, e.g. -DPOWER_TEST=40
    idle_minutes = 1;
#endif
    if (idle_minutes == 0) {
        ESP_LOGI(POWER_TAG, "Sleep disabled");
        return;
    }
    ESP_LOGI(POWER_TAG, "Deep sleep after %lld s without CAN traffic, clients or button presses", (long long)(idle_ms / 1000));
    stackwatch_create(power_task, "power", 3072, NULL, 1);
}
