#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_timer.h"
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

#define BRIDGE_TAG                  "Bridge"

static QueueHandle_t        isotp_send_message_queue    = NULL;
static SemaphoreHandle_t    isotp_send_task_mutex       = NULL;
static SemaphoreHandle_t    isotp_settings_mutex        = NULL;
static SemaphoreHandle_t    isotp_receive_mutex         = NULL;
static bool16               isotp_run_tasks             = false;

static ble_header_t         split_header;
static uint16_t             split_enabled               = false;
static uint8_t              split_count                 = 0;
static uint16_t             split_length                = 0;
static uint8_t*             split_data                  = NULL;
static uint32_t             virtual_led_color           = 0x004000;

bool16  isotp_allow_run_tasks(void);
void    isotp_set_run_tasks(bool16 allow);

int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint16_t size)
{
    twai_message_t frame = {.identifier = arbitration_id, .data_length_code = size};
    memcpy(frame.data, data, sizeof(frame.data));

    g_tx_count++;
    ESP_LOGI(BRIDGE_TAG, "CAN TX -> ID: 0x%03lX, DLC: %d", (unsigned long)arbitration_id, size);
    twai_send(&frame);

    return ISOTP_RET_OK;                           
}

uint64_t isotp_user_get_us(void)
{
    return esp_timer_get_time();
}

void isotp_user_debug(const char* message, ...)
{
    ESP_LOGD(BRIDGE_TAG, "ISOTP: %s", message);
}

void send_packet(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size)
{
    if(ble_connected()) {
        ble_send(txID, rxID, flags, src, size);
    } else {
        uart_send(txID, rxID, flags, src, size);
    }
}

static void isotp_processing_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    IsoTpLinkContainer *isotp_link_container = (IsoTpLinkContainer*)arg;
    IsoTpLink *link_ptr = &isotp_link_container->link;
    uint8_t *payload_buf = isotp_link_container->payload_buf;

    tMUTEX(isotp_link_container->task_mutex);
        xSemaphoreGive(sync_task_sem);
        while (isotp_allow_run_tasks())
        {
            if (link_ptr->send_status != ISOTP_SEND_STATUS_INPROGRESS &&
                link_ptr->receive_status != ISOTP_RECEIVE_STATUS_INPROGRESS) {
                xSemaphoreTake(isotp_link_container->wait_for_isotp_data_sem, pdMS_TO_TICKS(TIMEOUT_LONG));
            }
            
            tMUTEX(isotp_link_container->data_mutex);
                isotp_poll(link_ptr);
                uint16_t out_size;
                int ret = isotp_receive(link_ptr, payload_buf, isotp_link_container->buffer_size, &out_size);
            rMUTEX(isotp_link_container->data_mutex);

            if (ret == ISOTP_RET_OK) {
                ESP_LOGI(BRIDGE_TAG, "Received ECU response: %d bytes (arbitration 0x%03lX)", 
                         out_size, (unsigned long)link_ptr->receive_arbitration_id);

                tMUTEX(isotp_link_container->data_mutex);
                    uint32_t txID = link_ptr->receive_arbitration_id;
                    uint32_t rxID = link_ptr->send_arbitration_id;
                rMUTEX(isotp_link_container->data_mutex);
                send_packet(txID, rxID, 0, payload_buf, out_size);
            }

            esp_task_wdt_reset();
            taskYIELD();
        }
    rMUTEX(isotp_link_container->task_mutex);

    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    vTaskDelete(NULL);
}

static void isotp_send_queue_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    tMUTEX(isotp_send_task_mutex);
        xSemaphoreGive(sync_task_sem);
        while (isotp_allow_run_tasks()) {
            send_message_t msg;
            if (xQueueReceive(isotp_send_message_queue, &msg, pdMS_TO_TICKS(TIMEOUT_LONG)) == pdTRUE) {
                if (isotp_allow_run_tasks()) {
                    ESP_LOGI(BRIDGE_TAG, "Dispatching command: %d bytes (target: 0x%04X, reply: 0x%04X)", msg.msg_length, msg.rxID, msg.txID);
                    bool16 found_container = false;

                    for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
                        IsoTpLinkContainer* isotp_link_container = &isotp_link_containers[i];
                        tMUTEX(isotp_link_container->data_mutex);
                            if ((msg.txID == isotp_link_container->link.receive_arbitration_id && msg.rxID == isotp_link_container->link.send_arbitration_id) ||
                                (msg.txID == isotp_link_container->link.send_arbitration_id && msg.rxID == isotp_link_container->link.receive_arbitration_id) ||
                                (msg.rxID == 0 && msg.txID == 0 && i == 0)) {
                                
                                ESP_LOGI(BRIDGE_TAG, "Using container [%d] (ECU 0x%03lX)", i, (unsigned long)isotp_link_container->link.send_arbitration_id);
                                isotp_link_container_id = i;
                                isotp_send(&isotp_link_container->link, msg.buffer, msg.msg_length);
                                xSemaphoreGive(isotp_link_container->wait_for_isotp_data_sem);
                                found_container = true;
                            }
                        rMUTEX(isotp_link_container->data_mutex);

                        if(found_container) break;
                    }

                    if (!found_container) {
                        ESP_LOGW(BRIDGE_TAG, "No container matched -> routing to default ECU [0]");
                        IsoTpLinkContainer* def_container = &isotp_link_containers[0];
                        tMUTEX(def_container->data_mutex);
                            isotp_send(&def_container->link, msg.buffer, msg.msg_length);
                            xSemaphoreGive(def_container->wait_for_isotp_data_sem);
                        rMUTEX(def_container->data_mutex);
                    }

                    free(msg.buffer);
                }
            }
            esp_task_wdt_reset();
            taskYIELD();
        }
    rMUTEX(isotp_send_task_mutex);

    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    vTaskDelete(NULL);
}

void isotp_init(void)
{
    isotp_deinit();
    isotp_send_task_mutex       = xSemaphoreCreateMutex();
    isotp_settings_mutex        = xSemaphoreCreateMutex();
    isotp_receive_mutex         = xSemaphoreCreateMutex();
    isotp_send_message_queue    = xQueueCreate(ISOTP_QUEUE_SIZE, sizeof(send_message_t));
    configure_isotp_links();
}

void isotp_deinit(void)
{
    if (isotp_send_task_mutex) { vSemaphoreDelete(isotp_send_task_mutex); isotp_send_task_mutex = NULL; }
    if (isotp_settings_mutex) { vSemaphoreDelete(isotp_settings_mutex); isotp_settings_mutex = NULL; }
    if (isotp_receive_mutex) { vSemaphoreDelete(isotp_receive_mutex); isotp_receive_mutex = NULL; }
    if (isotp_send_message_queue) {
        send_message_t msg;
        while (xQueueReceive(isotp_send_message_queue, &msg, 0) == pdTRUE) if (msg.buffer) free(msg.buffer);
        vQueueDelete(isotp_send_message_queue);
        isotp_send_message_queue = NULL;
    }
    disable_isotp_links();
}

void isotp_start_task(void)
{
    isotp_stop_task();
    isotp_set_run_tasks(true);
    xSemaphoreTake(sync_task_sem, 0);
    for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
        IsoTpLinkContainer* c = &isotp_link_containers[i];
        xTaskCreate(isotp_processing_task, c->name, TASK_STACK_SIZE, c, ISOTP_TSK_PRIO, NULL);
        xSemaphoreTake(sync_task_sem, portMAX_DELAY);
    }
    xTaskCreate(isotp_send_queue_task, "ISOTP_send_q", TASK_STACK_SIZE, NULL, MAIN_TSK_PRIO, NULL);
    xSemaphoreTake(sync_task_sem, portMAX_DELAY);
}

void isotp_stop_task(void)
{
    if (isotp_allow_run_tasks()) {
        isotp_set_run_tasks(false);
        for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
            IsoTpLinkContainer* c = &isotp_link_containers[i];
            xSemaphoreGive(c->wait_for_isotp_data_sem);
            xSemaphoreTake(c->task_mutex, portMAX_DELAY);
            xSemaphoreGive(c->task_mutex);
        }
        send_message_t msg;
        xQueueSend(isotp_send_message_queue, &msg, portMAX_DELAY);
        xSemaphoreTake(isotp_send_task_mutex, portMAX_DELAY);
        xSemaphoreGive(isotp_send_task_mutex);
    }
}

bool16 isotp_allow_run_tasks(void) {
    tMUTEX(isotp_settings_mutex); bool16 r = isotp_run_tasks; rMUTEX(isotp_settings_mutex); return r;
}
void isotp_set_run_tasks(bool16 allow) {
    tMUTEX(isotp_settings_mutex); isotp_run_tasks = allow; rMUTEX(isotp_settings_mutex);
}

void split_clear(void)
{
    memset(&split_header, 0, sizeof(ble_header_t));
    split_enabled = false;
    split_count = 0;
    split_length = 0;
    if(split_data) { free(split_data); split_data = NULL; }
}

bool16 parse_packet(ble_header_t* header, uint8_t* data)
{
    ESP_LOGI(BRIDGE_TAG, "Parsing Header -> ID: 0x%02X, Flags: 0x%02X, Size: %d, rxID: 0x%04X, txID: 0x%04X",
             header->hdID, header->cmdFlags, header->cmdSize, header->rxID, header->txID);

    // 1. Password Authentication Challenge (Flags == 0xC7 or BRG_SETTING_PASSWORD)
    if (header->cmdFlags == 0xC7 || 
        ((header->cmdFlags & BLE_COMMAND_FLAG_SETTINGS) && ((header->cmdFlags & 0x0F) == BRG_SETTING_PASSWORD)))
    {
        uint8_t auth_ok = 0xFF;
        ESP_LOGI(BRIDGE_TAG, "Simos Tools Auth Handshake (0x%02X) -> Sending 0xFF (ACCEPTED)", header->cmdFlags);
        send_packet(0xFF, 0xFF, 0xFF, &auth_ok, sizeof(auth_ok));
        return true;
    }

    // 2. Settings Request Handler (GET / SET)
    if(header->cmdFlags & BLE_COMMAND_FLAG_SETTINGS)
    {
        uint8_t setting_id = header->cmdFlags ^ (BLE_COMMAND_FLAG_SETTINGS | (header->cmdFlags & BLE_COMMAND_FLAG_SETTINGS_GET));

        if(header->cmdFlags & BLE_COMMAND_FLAG_SETTINGS_GET)
        {
            ESP_LOGI(BRIDGE_TAG, "Simos Tools requested setting [%d] (GET)", setting_id);
            switch(setting_id)
            {
                case BRG_SETTING_ISOTP_STMIN: {
                    uint16_t stmin = 0;
                    send_packet(header->rxID, header->txID, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_ISOTP_STMIN, &stmin, sizeof(uint16_t));
                    break;
                }
                case BRG_SETTING_LED_COLOR: {
                    send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_LED_COLOR, &virtual_led_color, sizeof(uint32_t));
                    break;
                }
                case BRG_SETTING_PERSIST_DELAY: {
                    uint16_t delay = persist_get_delay();
                    send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_PERSIST_DELAY, &delay, sizeof(uint16_t));
                    break;
                }
                case BRG_SETTING_PERSIST_Q_DELAY: {
                    uint16_t delay = persist_get_q_delay();
                    send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_PERSIST_Q_DELAY, &delay, sizeof(uint16_t));
                    break;
                }
                case BRG_SETTING_BLE_SEND_DELAY: {
                    uint16_t delay = ble_get_delay_send();
                    send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_BLE_SEND_DELAY, &delay, sizeof(uint16_t));
                    break;
                }
                case BRG_SETTING_BLE_MULTI_DELAY: {
                    uint16_t delay = ble_get_delay_multi();
                    send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_BLE_MULTI_DELAY, &delay, sizeof(uint16_t));
                    break;
                }
                case BRG_SETTING_GAP: {
                    char str[MAX_GAP_LENGTH+1];
                    ble_get_gap_name(str);
                    send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_GAP, str, strlen(str));
                    break;
                }
                default: {
                    uint32_t zero_ack = 0;
                    send_packet(0, 0, header->cmdFlags, &zero_ack, sizeof(zero_ack));
                    break;
                }
            }
            return true;
        } else {
            ESP_LOGI(BRIDGE_TAG, "Simos Tools applied setting [%d] (SET)", setting_id);
            if (setting_id == BRG_SETTING_LED_COLOR && header->cmdSize >= sizeof(uint32_t)) {
                virtual_led_color = *(uint32_t*)data;
            }
            return true;
        }
    }

    // 3. Outgoing ISO-TP Vehicle Command
    if (header->cmdSize)
    {
        ESP_LOGI(BRIDGE_TAG, "Queueing ISO-TP frame: %d bytes (rxID: 0x%04X, txID: 0x%04X)",
                 header->cmdSize, header->rxID, header->txID);

        send_message_t msg;
        msg.msg_length = header->cmdSize;
        msg.rxID = header->txID;
        msg.txID = header->rxID;
        msg.buffer = malloc(header->cmdSize);
        if (msg.buffer) {
            memcpy(msg.buffer, data, header->cmdSize);
            if (xQueueSend(isotp_send_message_queue, &msg, pdMS_TO_TICKS(TIMEOUT_NORMAL)) != pdTRUE) {
                free(msg.buffer);
            }
        }
        return true;
    }

    return false;
}

void packet_received(const void* src, size_t size)
{
    tMUTEX(isotp_receive_mutex);
        uint8_t* data = (uint8_t*)src;

        if (size < sizeof(ble_header_t)) {
            goto release_mutex;
        }

        while(size >= sizeof(ble_header_t))
        {
            ble_header_t* header = (ble_header_t*)data;
            if(header->hdID != BLE_HEADER_ID)
            {
                ESP_LOGE(BRIDGE_TAG, "Protocol Header Mismatch: expected 0x%02X, got 0x%02X", BLE_HEADER_ID, header->hdID);
                goto release_mutex;
            }

            data += sizeof(ble_header_t);
            size -= sizeof(ble_header_t);

            if(parse_packet(header, data))
            {
                data += header->cmdSize;
                size -= header->cmdSize;
            } else {
                goto release_mutex;
            }
        }
release_mutex:
    rMUTEX(isotp_receive_mutex);
}

void uart_data_received(const void* src, size_t size)
{
    if(!ble_connected()) {
        ch_reset_uart_timer();
        packet_received(src, size);
    }
}

void bridge_connect(void)
{
    persist_clear();
    display_set_status("BLE ISO-TP", "CONNECTED", COLOR_GREEN);
}

void bridge_disconnect(void)
{
    persist_clear();
    display_set_status("BLE ISO-TP", "READY", COLOR_CYAN);
}

void ch_on_uart_connect(void) { bridge_connect(); }
void ch_on_uart_disconnect(void) { bridge_disconnect(); }

void bridge_received_ble(const void* src, size_t size)
{
    g_rx_count++;
    packet_received(src, size);
}

int32_t bridge_send_isotp(send_message_t *msg)
{
    return xQueueSend(isotp_send_message_queue, msg, pdMS_TO_TICKS(TIMEOUT_SHORT));
}

uint16_t bridge_send_available(void)
{
    return uxQueueSpacesAvailable(isotp_send_message_queue);
}