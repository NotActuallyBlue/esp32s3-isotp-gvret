#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "isotp.h"
#include "isotp_link_containers.h"
#include "constants.h"
#include "connection_handler.h"
#include "twai.h"
#include "persist.h"
#include "canstats.h"
#include "driver/gpio.h"
#include "stackwatch.h"

#define TWAI_TAG        "TWAI"

static const twai_general_config_t g_config = {
    .mode = CAN_MODE,
    .tx_io = CAN_TX_PORT,
    .rx_io = CAN_RX_PORT,
    .clkout_io = CAN_CLK_IO,
    .bus_off_io = CAN_BUS_IO,
    .tx_queue_len = 16,                     // Enable hardware TX buffer
    .rx_queue_len = CAN_INTERNAL_BUFFER_SIZE,
    .alerts_enabled = CAN_ALERTS,
    .clkout_divider = CAN_CLK_DIVIDER,
    .intr_flags = CAN_FLAGS
};
static const twai_timing_config_t   t_config                = CAN_TIMING;
static const twai_filter_config_t   f_config                = CAN_FILTER;
static SemaphoreHandle_t            twai_receive_task_mutex = NULL;
static SemaphoreHandle_t            twai_alert_task_mutex   = NULL;
static SemaphoreHandle_t            twai_bus_off_mutex      = NULL;
static SemaphoreHandle_t            twai_settings_mutex     = NULL;
static bool16                       twai_run_task           = false;

void twai_receive_task(void *arg);
void twai_alert_task(void* arg);

void twai_set_run_task(bool16 allow)
{
    tMUTEX(twai_settings_mutex);
        twai_run_task = allow;
    rMUTEX(twai_settings_mutex);
}

bool16 twai_allow_run_task()
{
    tMUTEX(twai_settings_mutex);
        bool16 run_task = twai_run_task;
    rMUTEX(twai_settings_mutex);

    return run_task;
}

void twai_init()
{
    twai_deinit();

    twai_receive_task_mutex = xSemaphoreCreateMutex();
    twai_alert_task_mutex   = xSemaphoreCreateMutex();
    twai_bus_off_mutex      = xSemaphoreCreateMutex();
    twai_settings_mutex     = xSemaphoreCreateMutex();

    ESP_LOGI(TWAI_TAG, "Init");
}

void twai_deinit()
{
    bool16 didDeInit = false;

    if (twai_receive_task_mutex) {
        vSemaphoreDelete(twai_receive_task_mutex);
        twai_receive_task_mutex = NULL;
        didDeInit = true;
    }

    if (twai_alert_task_mutex) {
        vSemaphoreDelete(twai_alert_task_mutex);
        twai_alert_task_mutex = NULL;
        didDeInit = true;
    }

    if (twai_bus_off_mutex) {
        vSemaphoreDelete(twai_bus_off_mutex);
        twai_bus_off_mutex = NULL;
        didDeInit = true;
    }

    if (twai_settings_mutex) {
        vSemaphoreDelete(twai_settings_mutex);
        twai_settings_mutex = NULL;
        didDeInit = true;
    }

    if (didDeInit)
        ESP_LOGI(TWAI_TAG, "Deinit");
}

void twai_start_task()
{
    twai_stop_task();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TWAI_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TWAI_TAG, "Driver started");

    twai_set_run_task(true);

    ESP_LOGI(TWAI_TAG, "Tasks starting");
    xSemaphoreTake(sync_task_sem, 0);

    stackwatch_create(twai_alert_task, "TWAI_alert", TASK_STACK_SIZE, NULL, TWAI_TASK_PRIO);
    xSemaphoreTake(sync_task_sem, portMAX_DELAY);

    stackwatch_create(twai_receive_task, "TWAI_rx", TASK_STACK_SIZE, NULL, TWAI_TASK_PRIO);
    xSemaphoreTake(sync_task_sem, portMAX_DELAY);

    ESP_LOGI(TWAI_TAG, "Tasks started");
}

// Driver and alert (bus-off recovery) task only: the caller reads frames with twai_receive itself.
// Used by the diagnostics modes, where no ISO-TP links exist.
void twai_start_raw()
{
    twai_stop_task();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    twai_set_run_task(true);

    xSemaphoreTake(sync_task_sem, 0);
    stackwatch_create(twai_alert_task, "TWAI_alert", TASK_STACK_SIZE, NULL, TWAI_TASK_PRIO);
    xSemaphoreTake(sync_task_sem, portMAX_DELAY);
    ESP_LOGI(TWAI_TAG, "Driver started (raw)");
}

void twai_stop_task()
{
    if (twai_allow_run_task()) {
        twai_set_run_task(false);

        tMUTEX(twai_receive_task_mutex);
        rMUTEX(twai_receive_task_mutex);

        tMUTEX(twai_alert_task_mutex);
        rMUTEX(twai_alert_task_mutex);

        ESP_LOGI(TWAI_TAG, "Tasks stopped");

        twai_stop();
        twai_driver_uninstall();

        ESP_LOGI(TWAI_TAG, "Driver uninstalled");
    }
}

void twai_send_isotp_message(IsoTpLinkContainer* link, twai_message_t* msg)
{
    PERSIST_LOG_WINDOW(TWAI_TAG, "CAN RX <- ID: 0x%03lX, DLC: %d, %02X %02X %02X %02X", (unsigned long)msg->identifier, msg->data_length_code,
                       msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    tMUTEX(link->data_mutex);
        isotp_on_can_message(&link->link, msg->data, msg->data_length_code);
    rMUTEX(link->data_mutex);

    xSemaphoreGive(link->wait_for_isotp_data_sem);
}

void twai_receive_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    tMUTEX(twai_receive_task_mutex);
        ESP_LOGI(TWAI_TAG, "Receive task started");
        xSemaphoreGive(sync_task_sem);
        twai_message_t twai_rx_msg;
        while (twai_allow_run_task())
        {
            if (twai_receive(&twai_rx_msg, pdMS_TO_TICKS(TIMEOUT_LONG)) == ESP_OK) {
                ch_take_can_timer_sem();
                canstats_on_frame(twai_rx_msg.identifier, twai_rx_msg.data_length_code);
        
                if (twai_rx_msg.identifier < 0x500) {
                    esp_task_wdt_reset();
                    continue;
                }
                
                IsoTpLinkContainer* isotp_link_container = &isotp_link_containers[isotp_link_container_id];
                if (twai_rx_msg.identifier == isotp_link_container->link.receive_arbitration_id) {
                    twai_send_isotp_message(isotp_link_container, &twai_rx_msg);
                } else {
                    for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
                        isotp_link_container = &isotp_link_containers[i];
                        if (twai_rx_msg.identifier == isotp_link_container->link.receive_arbitration_id) {
                            twai_send_isotp_message(isotp_link_container, &twai_rx_msg);
                            break;
                        }
                    }
                }
            }

            esp_task_wdt_reset();
            taskYIELD();
        }
        ESP_LOGI(TWAI_TAG, "Receive task stopped");
    rMUTEX(twai_receive_task_mutex);

    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    vTaskDelete(NULL);
}

void twai_send(twai_message_t *twai_tx_msg)
{
    tMUTEX(twai_bus_off_mutex);
    rMUTEX(twai_bus_off_mutex);
    // Non-blocking timeout instead of infinite while loop
    esp_err_t err = twai_transmit(twai_tx_msg, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        canstats_on_tx_failure();
        ESP_LOGW(TWAI_TAG, "CAN TX failed (ID: 0x%03lX): %s", (unsigned long)twai_tx_msg->identifier, esp_err_to_name(err));
    }
}

void twai_alert_task(void* arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    tMUTEX(twai_alert_task_mutex);
        ESP_LOGI(TWAI_TAG, "Alert task started");
        xSemaphoreGive(sync_task_sem);
        uint32_t alerts;
        while (twai_allow_run_task()) {

            if (twai_read_alerts(&alerts, pdMS_TO_TICKS(TIMEOUT_LONG)) == ESP_OK) {
                if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
                    canstats_on_event(CAN_EVENT_ERROR_WARNING);
                    ESP_LOGW(TWAI_TAG, "Surpassed Error Warning Limit");
                }

                if (alerts & TWAI_ALERT_ERR_PASS) {
                    canstats_on_event(CAN_EVENT_ERROR_PASSIVE);
                    ESP_LOGW(TWAI_TAG, "Entered Error Passive state");
                }

                if (alerts & TWAI_ALERT_BUS_OFF) {
                    canstats_on_event(CAN_EVENT_BUS_OFF);
                    ESP_LOGE(TWAI_TAG, "Bus Off state detected!");
                    if (xSemaphoreTake(twai_bus_off_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL)) == pdTRUE) {
                        ESP_LOGW(TWAI_TAG, "Initiate bus recovery");
                        esp_err_t err = twai_initiate_recovery();
                        if (err != ESP_OK) {
                            // Not worth restarting the dongle over: it could be in the middle of a flash
                            ESP_LOGE(TWAI_TAG, "Bus recovery failed: %s", esp_err_to_name(err));
                            xSemaphoreGive(twai_bus_off_mutex);
                        }
                    }
                }

                if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                    esp_err_t err = twai_start();
                    if (err != ESP_OK) ESP_LOGE(TWAI_TAG, "Restart after bus recovery failed: %s", esp_err_to_name(err));
                    else ESP_LOGI(TWAI_TAG, "Bus Recovered");
                    xSemaphoreGive(twai_bus_off_mutex);
                }
            }

            esp_task_wdt_reset();
            taskYIELD();
        }
        ESP_LOGI(TWAI_TAG, "Alert task stopped");
    rMUTEX(twai_alert_task_mutex);

    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    vTaskDelete(NULL);
}