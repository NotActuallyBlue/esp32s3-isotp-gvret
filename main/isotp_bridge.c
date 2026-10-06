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
#include "bench_sim.h"

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

    // Bench simulator: frames are exchanged in memory instead of going out on the bus
    if (bench_sim_active()) {
        if (!bench_sim_is_response_id(arbitration_id)) g_tx_count++;
        bench_sim_send_can(arbitration_id, data, size);
        return ISOTP_RET_OK;
    }

    g_tx_count++;
    PERSIST_LOG_WINDOW(BRIDGE_TAG, "CAN TX -> ID: 0x%03lX, DLC: %d, %02X %02X %02X %02X", (unsigned long)arbitration_id, size,
                       data[0], size > 1 ? data[1] : 0, size > 2 ? data[2] : 0, size > 3 ? data[3] : 0);
    twai_send(&frame);

    return ISOTP_RET_OK;                           
}

uint64_t isotp_user_get_us(void)
{
    return esp_timer_get_time();
}

void isotp_user_flow_control(uint32_t arbitration_id, uint8_t block_size,
                             uint32_t receiver_st_min_us, uint16_t override_us, uint32_t used_st_min_us)
{
    PERSIST_LOG_WINDOW(BRIDGE_TAG, "Flow control for 0x%03lX: BS %d, STmin %lu us (override %u) -> using %lu us",
                       (unsigned long)arbitration_id, block_size, (unsigned long)receiver_st_min_us, override_us, (unsigned long)used_st_min_us);
}

void isotp_user_debug(const char* message, ...)
{
    ESP_LOGD(BRIDGE_TAG, "ISOTP: %s", message);
}

void bridge_send_packet(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size)
{
    if(ble_connected()) {
        ble_send(txID, rxID, flags, src, size);
    } else {
        uart_send(txID, rxID, flags, src, size);
    }
}

// Requests are sent one at a time per link: the ECU ignores a new First Frame while it is still answering the
// previous request, and isotp_send() refuses to start while a send is in progress.
#define LINK_RESPONSE_WAIT_MS   60

static volatile bool    link_awaiting_response[NUM_ISOTP_LINK_CONTAINERS];
static volatile int64_t link_request_time_us[NUM_ISOTP_LINK_CONTAINERS];

static void isotp_wait_link_ready(uint16_t index)
{
    IsoTpLinkContainer* c = &isotp_link_containers[index];
    int64_t start = esp_timer_get_time();

    while (isotp_allow_run_tasks()) {
        tMUTEX(c->data_mutex);
            bool busy = c->link.send_status == ISOTP_SEND_STATUS_INPROGRESS ||
                        c->link.receive_status == ISOTP_RECEIVE_STATUS_INPROGRESS;
        rMUTEX(c->data_mutex);

        int64_t now = esp_timer_get_time();
        bool awaiting = link_awaiting_response[index] && (now - link_request_time_us[index]) < LINK_RESPONSE_WAIT_MS * 1000;

        if ((!busy && !awaiting) || (now - start) > 300 * 1000) {
            break;
        }
        vTaskDelay(1);
    }
}

// ISO-TP errors repeat when a module answers late; log the first one, then one summary per minute
#define ERROR_LOG_INTERVAL_US (60LL * 1000 * 1000)

typedef struct {
    int64_t last_log_us;
    uint32_t suppressed;
} ErrorLogState;

static void log_isotp_error(ErrorLogState *state, uint16_t number, const char *direction, int16_t code)
{
    g_error_count++;
    int64_t now = esp_timer_get_time();
    if (state->last_log_us == 0 || now - state->last_log_us >= ERROR_LOG_INTERVAL_US) {
        if (state->suppressed > 0) {
            ESP_LOGW(BRIDGE_TAG, "[%d] ISO-TP %s error: %d (%lu more since the last report)", number, direction, code,
                     (unsigned long)state->suppressed);
        } else {
            ESP_LOGW(BRIDGE_TAG, "[%d] ISO-TP %s error: %d", number, direction, code);
        }
        state->last_log_us = now;
        state->suppressed = 0;
    } else {
        state->suppressed++;
    }
}

static void isotp_processing_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    IsoTpLinkContainer *isotp_link_container = (IsoTpLinkContainer*)arg;
    IsoTpLink *link_ptr = &isotp_link_container->link;
    uint8_t *payload_buf = isotp_link_container->payload_buf;
    uint16_t number = isotp_link_container->number;
    int16_t last_send_result = ISOTP_PROTOCOL_RESULT_OK;
    int16_t last_receive_result = ISOTP_PROTOCOL_RESULT_OK;
    ErrorLogState send_errors = {0};
    ErrorLogState receive_errors = {0};

    tMUTEX(isotp_link_container->task_mutex);
        xSemaphoreGive(sync_task_sem);
        while (isotp_allow_run_tasks())
        {
            // Idle: sleep until there is work. Waiting on the ECU/TCU (flow control or consecutive frames):
            // sleep briefly, every received frame wakes us. Only spin while there are frames left to send.
            tMUTEX(isotp_link_container->data_mutex);
                bool send_active = link_ptr->send_status == ISOTP_SEND_STATUS_INPROGRESS;
                bool receive_active = link_ptr->receive_status == ISOTP_RECEIVE_STATUS_INPROGRESS;
                bool can_send = send_active && (link_ptr->send_bs_remain == ISOTP_INVALID_BS || link_ptr->send_bs_remain > 0);
            rMUTEX(isotp_link_container->data_mutex);

            if (!send_active && !receive_active) {
                xSemaphoreTake(isotp_link_container->wait_for_isotp_data_sem, pdMS_TO_TICKS(TIMEOUT_LONG));
            } else if (!can_send) {
                xSemaphoreTake(isotp_link_container->wait_for_isotp_data_sem, pdMS_TO_TICKS(2));
            }
            
            tMUTEX(isotp_link_container->data_mutex);
                isotp_poll(link_ptr);
                uint16_t out_size;
                int ret = isotp_receive(link_ptr, payload_buf, isotp_link_container->buffer_size, &out_size);
                int16_t send_result = link_ptr->send_protocol_result;
                int16_t receive_result = link_ptr->receive_protocol_result;
            rMUTEX(isotp_link_container->data_mutex);

            if (send_result != last_send_result) {
                if (send_result != ISOTP_PROTOCOL_RESULT_OK) log_isotp_error(&send_errors, number, "send", send_result);
                last_send_result = send_result;
            }
            if (receive_result != last_receive_result) {
                if (receive_result != ISOTP_PROTOCOL_RESULT_OK) log_isotp_error(&receive_errors, number, "receive", receive_result);
                last_receive_result = receive_result;
            }

            if (ret == ISOTP_RET_OK) {
                if (number < NUM_ISOTP_LINK_CONTAINERS) {
                    if (link_awaiting_response[number]) {
                        display_set_link_latency(number, (esp_timer_get_time() - link_request_time_us[number]) / 1000);
                    }
                    link_awaiting_response[number] = false;
                }
                PERSIST_LOG_WINDOW(BRIDGE_TAG, "Received ECU response: %d bytes (arbitration 0x%03lX)",
                         out_size, (unsigned long)link_ptr->receive_arbitration_id);

                // In persist mode, send a timestamp in place of rx/tx and release the next persist message
                if (number < PERSIST_COUNT && persist_enabled()) {
                    uint32_t time = (esp_timer_get_time() / 1000UL) & 0xFFFFFFFF;
                    uint16_t rxID = (time >> 16) & 0xFFFF;
                    uint16_t txID = time & 0xFFFF;
                    bridge_send_packet(txID, rxID, 0, payload_buf, out_size);
                    persist_note_reply(number, out_size);
                    persist_allow_send(number);
                } else {
                    persist_note_late_reply(number);
                    tMUTEX(isotp_link_container->data_mutex);
                        uint32_t txID = link_ptr->receive_arbitration_id;
                        uint32_t rxID = link_ptr->send_arbitration_id;
                    rMUTEX(isotp_link_container->data_mutex);
                    bridge_send_packet(txID, rxID, 0, payload_buf, out_size);
                }
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
                    PERSIST_LOG_WINDOW(BRIDGE_TAG, "Dispatching command: %d bytes (target: 0x%04X, reply: 0x%04X): %02X %02X %02X %02X %02X %02X %02X %02X", msg.msg_length, msg.rxID, msg.txID,
                                       msg.buffer[0], msg.msg_length > 1 ? msg.buffer[1] : 0, msg.msg_length > 2 ? msg.buffer[2] : 0, msg.msg_length > 3 ? msg.buffer[3] : 0,
                                       msg.msg_length > 4 ? msg.buffer[4] : 0, msg.msg_length > 5 ? msg.buffer[5] : 0, msg.msg_length > 6 ? msg.buffer[6] : 0, msg.msg_length > 7 ? msg.buffer[7] : 0);
                    bool16 found_container = false;

                    // Wait for the addressed link to finish its previous request
                    for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
                        IsoTpLinkContainer* c = &isotp_link_containers[i];
                        tMUTEX(c->data_mutex);
                            bool match = (msg.txID == c->link.receive_arbitration_id && msg.rxID == c->link.send_arbitration_id) ||
                                         (msg.txID == c->link.send_arbitration_id && msg.rxID == c->link.receive_arbitration_id) ||
                                         (msg.rxID == 0 && msg.txID == 0 && i == 0);
                        rMUTEX(c->data_mutex);
                        if (match) {
                            isotp_wait_link_ready(i);
                            break;
                        }
                    }

                    for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
                        IsoTpLinkContainer* isotp_link_container = &isotp_link_containers[i];
                        tMUTEX(isotp_link_container->data_mutex);
                            if ((msg.txID == isotp_link_container->link.receive_arbitration_id && msg.rxID == isotp_link_container->link.send_arbitration_id) ||
                                (msg.txID == isotp_link_container->link.send_arbitration_id && msg.rxID == isotp_link_container->link.receive_arbitration_id) ||
                                (msg.rxID == 0 && msg.txID == 0 && i == 0)) {
                                
                                PERSIST_LOG_WINDOW(BRIDGE_TAG, "Using container [%d] (ECU 0x%03lX)", i, (unsigned long)isotp_link_container->link.send_arbitration_id);
                                isotp_link_container_id = i;
                                int send_ret = isotp_send(&isotp_link_container->link, msg.buffer, msg.msg_length);
                                if (send_ret != ISOTP_RET_OK) {
                                    g_error_count++;
                                    ESP_LOGW(BRIDGE_TAG, "[%d] isotp_send failed: %d (%d bytes dropped)", i, send_ret, msg.msg_length);
                                } else {
                                    link_request_time_us[i] = esp_timer_get_time();
                                    link_awaiting_response[i] = true;
                                }
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
        send_message_t msg = { 0 };     // wakes the send task so it can see that it should stop
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
        char pass[17] = {0};
        size_t pass_len = header->cmdSize < sizeof(pass) - 1 ? header->cmdSize : sizeof(pass) - 1;
        for (size_t i = 0; i < pass_len; i++) pass[i] = (data[i] >= 32 && data[i] < 127) ? data[i] : '?';
        ESP_LOGI(BRIDGE_TAG, "Auth Handshake (0x%02X), password \"%s\" -> Sending 0xFF (ACCEPTED, not checked)", header->cmdFlags, pass);
        bridge_send_packet(0xFF, 0xFF, 0xFF, &auth_ok, sizeof(auth_ok));
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
                    for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
                        IsoTpLinkContainer *c = &isotp_link_containers[i];
                        if (header->rxID == c->link.receive_arbitration_id && header->txID == c->link.send_arbitration_id) {
                            uint16_t stmin = c->link.stmin_override;
                            bridge_send_packet(header->rxID, header->txID, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_ISOTP_STMIN, &stmin, sizeof(uint16_t));
                        }
                    }
                    break;
                }
                case BRG_SETTING_LED_COLOR: {
                    bridge_send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_LED_COLOR, &virtual_led_color, sizeof(uint32_t));
                    break;
                }
                case BRG_SETTING_PERSIST_DELAY: {
                    uint16_t delay = persist_get_delay();
                    bridge_send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_PERSIST_DELAY, &delay, sizeof(uint16_t));
                    break;
                }
                case BRG_SETTING_PERSIST_Q_DELAY: {
                    uint16_t delay = persist_get_q_delay();
                    bridge_send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_PERSIST_Q_DELAY, &delay, sizeof(uint16_t));
                    break;
                }
                case BRG_SETTING_BLE_SEND_DELAY: {
                    uint16_t delay = ble_get_delay_send();
                    bridge_send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_BLE_SEND_DELAY, &delay, sizeof(uint16_t));
                    break;
                }
                case BRG_SETTING_BLE_MULTI_DELAY: {
                    uint16_t delay = ble_get_delay_multi();
                    bridge_send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_BLE_MULTI_DELAY, &delay, sizeof(uint16_t));
                    break;
                }
                case BRG_SETTING_GAP: {
                    char str[MAX_GAP_LENGTH+1];
                    ble_get_gap_name(str);
                    bridge_send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_GAP, str, strlen(str));
                    break;
                }
                default: {
                    uint32_t zero_ack = 0;
                    bridge_send_packet(0, 0, header->cmdFlags, &zero_ack, sizeof(zero_ack));
                    break;
                }
            }
            return true;
        } else {
            ESP_LOGI(BRIDGE_TAG, "Simos Tools applied setting [%d] (SET)", setting_id);
            switch(setting_id)
            {
                case BRG_SETTING_ISOTP_STMIN:
                    if (header->cmdSize == sizeof(uint16_t)) {
                        for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
                            IsoTpLinkContainer *c = &isotp_link_containers[i];
                            tMUTEX(c->data_mutex);
                                if (header->rxID == c->link.receive_arbitration_id && header->txID == c->link.send_arbitration_id) {
                                    c->link.stmin_override = *(uint16_t*)data;
                                    ESP_LOGI(BRIDGE_TAG, "Set stmin override %u us on rx 0x%03lX", c->link.stmin_override, (unsigned long)c->link.receive_arbitration_id);
                                }
                            rMUTEX(c->data_mutex);
                        }
                    }
                    break;
                case BRG_SETTING_LED_COLOR:
                    if (header->cmdSize == sizeof(uint32_t)) virtual_led_color = *(uint32_t*)data;
                    break;
                case BRG_SETTING_PERSIST_DELAY:
                    if (header->cmdSize == sizeof(uint16_t)) persist_set_delay(*(uint16_t*)data);
                    break;
                case BRG_SETTING_PERSIST_Q_DELAY:
                    if (header->cmdSize == sizeof(uint16_t)) persist_set_q_delay(*(uint16_t*)data);
                    break;
                case BRG_SETTING_BLE_SEND_DELAY:
                    if (header->cmdSize == sizeof(uint16_t)) ble_set_delay_send(*(uint16_t*)data);
                    break;
                case BRG_SETTING_BLE_MULTI_DELAY:
                    if (header->cmdSize == sizeof(uint16_t)) ble_set_delay_multi(*(uint16_t*)data);
                    break;
                case BRG_SETTING_GAP:
                    if (header->cmdSize <= MAX_GAP_LENGTH) {
                        char str[MAX_GAP_LENGTH+1];
                        memcpy(str, data, header->cmdSize);
                        str[header->cmdSize] = 0;
                        ble_set_gap_name(str, true);
                        eeprom_write_str(BLE_GAP_KEY, str);
                        eeprom_commit();
                        ch_give_sleep_sem();
                    }
                    break;
            }
            return true;
        }
    }

    // 3. Persist (logging) mode control
    if (persist_enabled()) {
        if (header->cmdFlags & BLE_COMMAND_FLAG_PER_CLEAR) {
            persist_clear();
        }

        if ((header->cmdFlags & BLE_COMMAND_FLAG_PER_ENABLE) == 0) {
            ESP_LOGI(BRIDGE_TAG, "Persist mode disabled");
            persist_set(false);
        } else {
            // While persist is running only setting changes are accepted
            return false;
        }
    } else {
        // The app registers ECU and TCU requests separately, so a clear only drops the addressed link's messages
        if (header->cmdFlags & BLE_COMMAND_FLAG_PER_CLEAR) {
            persist_clear_link(header->txID, header->rxID);
        }

        if (header->cmdFlags & BLE_COMMAND_FLAG_PER_ADD) {
            persist_add(header->txID, header->rxID, data, header->cmdSize);
        }

        if (header->cmdFlags & BLE_COMMAND_FLAG_PER_ENABLE) {
            ESP_LOGI(BRIDGE_TAG, "Persist mode enabled");
            persist_set(true);
            return false;
        }
    }

    // 4. Outgoing ISO-TP Vehicle Command
    if (header->cmdSize)
    {
        PERSIST_LOG_WINDOW(BRIDGE_TAG, "Queueing ISO-TP frame: %d bytes (rxID: 0x%04X, txID: 0x%04X)",
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

        if (size < 2) {
            goto release_mutex;
        }

        // Continuation chunk of a split packet: [0xF2][count][payload...]
        if (split_enabled && data[0] == BLE_PARTIAL_ID) {
            if (data[1] != split_count || !split_data || !split_length) {
                ESP_LOGW(BRIDGE_TAG, "Split packet out of order [%02X, %02X]", data[1], split_count);
                split_clear();
                goto release_mutex;
            }

            uint8_t* new_data = malloc(split_length + size - 2);
            if (new_data == NULL) {
                split_clear();
                goto release_mutex;
            }
            memcpy(new_data, split_data, split_length);
            memcpy(new_data + split_length, data + 2, size - 2);
            free(split_data);
            split_data = new_data;
            split_length += size - 2;
            split_count++;

            if (split_length == split_header.cmdSize) {
                parse_packet(&split_header, split_data);
                split_clear();
            } else if (split_length > split_header.cmdSize) {
                ESP_LOGW(BRIDGE_TAG, "Split packet larger than command [%d, %d]", split_header.cmdSize, split_length);
                split_clear();
            }
            goto release_mutex;
        }

        split_enabled = false;

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

            if (header->cmdSize > size) {
                // First chunk of a split packet; the rest arrives as 0xF2 chunks
                if (header->cmdFlags & BLE_COMMAND_FLAG_SPLIT_PK) {
                    split_clear();
                    split_data = malloc(size ? size : 1);
                    if (split_data == NULL) {
                        goto release_mutex;
                    }
                    memcpy(split_data, data, size);
                    memcpy(&split_header, header, sizeof(ble_header_t));
                    split_enabled = true;
                    split_count = 1;
                    split_length = size;
                } else {
                    ESP_LOGW(BRIDGE_TAG, "Command size larger than packet [%d, %d]", header->cmdSize, size);
                }
                goto release_mutex;
            }

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