// Copied from "BLE SPP" example in ESP-IDF, which is Public Domain
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "ble_server.h"

#define BLE_TAG                              "BLE"

#define SPP_PROFILE_NUM                      1
#define SPP_PROFILE_APP_IDX                  0
#define ESP_SPP_APP_ID                       0x56
#define SPP_SVC_INST_ID                      0
#define DEFAULT_MTU_SIZE                     23
#define DEFAULT_DELAY_SEND                   0
#define DEFAULT_DELAY_MULTI                  0

static const uint16_t spp_service_uuid = 0xABF0;
#define ESP_GATT_UUID_SPP_DATA_RECEIVE       0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY        0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE    0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY     0xABF4

static uint8_t spp_adv_data[23] = {
    0x02,0x01,0x06,
    0x03,0x03,0xF0,0xAB,
    0x0F,0x09, 'B', 'L', 'E', '_', 'T', 'O', '_', 'I', 'S', 'O', 'T','P', '2', '0'
};

static char                 ble_gap_name[MAX_GAP_LENGTH+1]  = DEFAULT_GAP_NAME;
static uint16_t             ble_delay_send                  = DEFAULT_DELAY_SEND;
static uint16_t             ble_delay_multi                 = DEFAULT_DELAY_MULTI;
static bool16               ble_run_tasks                   = false;
static ble_server_callbacks server_callbacks;

static uint16_t             spp_mtu_size                    = DEFAULT_MTU_SIZE;
static uint16_t             spp_conn_id                     = 0xffff;
static esp_gatt_if_t        spp_gatts_if                    = ESP_GATT_IF_NONE;
static QueueHandle_t        spp_send_queue                  = NULL;
static SemaphoreHandle_t    ble_congested                   = NULL;
static SemaphoreHandle_t    ble_task_mutex                  = NULL;
static SemaphoreHandle_t    ble_settings_mutex              = NULL;

static bool16               enable_data_ntf                 = false;
static bool16               is_connected                    = false;
static bool16               allow_connection                = true;
static esp_bd_addr_t        spp_remote_bda                  = {0x0,};

static uint16_t             spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    [SPP_IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    [SPP_IDX_SPP_DATA_RECV_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    [SPP_IDX_SPP_DATA_RECV_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    [SPP_IDX_SPP_DATA_NTY_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    [SPP_IDX_SPP_DATA_NTF_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    [SPP_IDX_SPP_COMMAND_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    [SPP_IDX_SPP_COMMAND_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    [SPP_IDX_SPP_STATUS_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    [SPP_IDX_SPP_STATUS_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    [SPP_IDX_SPP_STATUS_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]) return i;
    }
    return 0xff;
}

static void disable_notification(void) {
    enable_data_ntf = false;
    server_callbacks.notifications_unsubscribed();
}

static void enable_notification(void) {
    enable_data_ntf = true;
    server_callbacks.notifications_subscribed();
}

void ble_set_run_tasks(bool16 allowed)
{
    tMUTEX(ble_settings_mutex);
        ble_run_tasks = allowed;
    rMUTEX(ble_settings_mutex);
}

bool16 ble_allow_run_tasks(void)
{
    tMUTEX(ble_settings_mutex);
        bool16 allowed = ble_run_tasks;
    rMUTEX(ble_settings_mutex);
    return allowed;
}

void send_task(void *pvParameters)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    tMUTEX(ble_task_mutex);
        send_message_t event;
        while(ble_allow_run_tasks()) {
            if (xQueueReceive(spp_send_queue, &event, pdMS_TO_TICKS(TIMEOUT_LONG)) == pdTRUE) {
                if (ble_allow_run_tasks()) {
                    if (event.msg_length) {
                        if (!enable_data_ntf) {
                            ESP_LOGW(BLE_TAG, "Notifications not enabled, discarding message");
                            free(event.buffer);
                        }
                        else
                        {
                            xSemaphoreTake(ble_congested, pdMS_TO_TICKS(BLE_CONGESTION_MAX));
                            xSemaphoreGive(ble_congested);

                            uint32_t dataLength = event.msg_length + sizeof(ble_header_t);
                            uint8_t* data = (uint8_t*)malloc(dataLength);
                            if (data == NULL) {
                                free(event.buffer);
                                break;
                            }
                            memset(data, 0, dataLength);
                            memcpy(data + sizeof(ble_header_t), event.buffer, event.msg_length);
                            free(event.buffer);

                            ble_header_t* header = (ble_header_t*)data;
                            header->hdID = BLE_HEADER_ID;
                            header->cmdFlags = event.flags;
                            header->cmdSize = event.msg_length;
                            header->rxID = event.rxID;
                            header->txID = event.txID;

                            // Always use the registered interface handle to avoid 0xFF panics
                            esp_gatt_if_t target_if = (spp_gatts_if != ESP_GATT_IF_NONE) ? 
                                                      spp_gatts_if : spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if;

                            if (target_if != ESP_GATT_IF_NONE && spp_conn_id != 0xffff) {
                                ESP_LOGI(BLE_TAG, "Transmitting BLE packet: %ld bytes (flags: 0x%02X)", (long)event.msg_length, event.flags);
                                esp_ble_gatts_send_indicate(target_if, spp_conn_id, 
                                                            spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 
                                                            dataLength, data, false);
                            } else {
                                ESP_LOGE(BLE_TAG, "Cannot send: Invalid target interface (0x%02X) or conn_id (%d)", target_if, spp_conn_id);
                            }

                            free(data);
                            vTaskDelay(pdMS_TO_TICKS(ble_get_delay_send()));
                        }
                    }
                }
            }
            esp_task_wdt_reset();
            taskYIELD();
        }
    rMUTEX(ble_task_mutex);

    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    vTaskDelete(NULL);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&spp_adv_params));
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    switch (event) {
        case ESP_GATTS_REG_EVT:
            // Lock the active registered interface handle
            spp_gatts_if = gatts_if;
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;

            tMUTEX(ble_settings_mutex);
                ESP_ERROR_CHECK(esp_ble_gap_set_device_name(ble_gap_name));
                ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, spp_adv_data[7] + 8));
            rMUTEX(ble_settings_mutex);
            ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID));
            break;
        case ESP_GATTS_WRITE_EVT: {
            res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        ESP_LOGI(BLE_TAG, "Client subscribed to notifications");
                        enable_notification();
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        ESP_LOGI(BLE_TAG, "Client unsubscribed from notifications");
                        disable_notification();
                    }
                }
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
                    char hex_dump[96] = {0};
                    int max_bytes = (p_data->write.len > 24) ? 24 : p_data->write.len;
                    for (int b = 0; b < max_bytes; b++) {
                        snprintf(hex_dump + (b * 3), 4, "%02X ", p_data->write.value[b]);
                    }
                    ESP_LOGI(BLE_TAG, "Incoming BLE write (%d bytes): %s", p_data->write.len, hex_dump);

                    server_callbacks.data_received((char *)(p_data->write.value), p_data->write.len);
                }
            }
            break;
        }
        case ESP_GATTS_MTU_EVT:
            spp_mtu_size = p_data->mtu.mtu;
            break;
        case ESP_GATTS_CONNECT_EVT:
            spp_conn_id = p_data->connect.conn_id;

            // Only update spp_gatts_if if a valid profile interface was provided
            if (gatts_if != ESP_GATT_IF_NONE) {
                spp_gatts_if = gatts_if;
            }

            tMUTEX(ble_settings_mutex);
                is_connected = true;
            rMUTEX(ble_settings_mutex);
            memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
            xSemaphoreGive(ble_congested);
            ESP_LOGI(BLE_TAG, "Simos Tools paired & connected (conn_id: %d, gatts_if: 0x%02X)", spp_conn_id, spp_gatts_if);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            tMUTEX(ble_settings_mutex);
                is_connected = false;
            rMUTEX(ble_settings_mutex);
            disable_notification();
            spp_mtu_size = DEFAULT_MTU_SIZE;
            spp_conn_id = 0xffff;
            ESP_LOGI(BLE_TAG, "Simos Tools disconnected");
            if(ble_allow_connection())
                ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&spp_adv_params));
            break;
        case ESP_GATTS_CONGEST_EVT:
            if(p_data->connect.conn_id == spp_conn_id) {
                if(p_data->congest.congested) xSemaphoreTake(ble_congested, 1);
                else xSemaphoreGive(ble_congested);
            }
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status == ESP_GATT_OK){
                memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
                ESP_ERROR_CHECK(esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]));
            }
            break;
        }
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
    }
    for (int idx = 0; idx < SPP_PROFILE_NUM; idx++) {
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == spp_profile_tab[idx].gatts_if) {
            if (spp_profile_tab[idx].gatts_cb) {
                spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

void ble_server_init(void)
{
    ble_server_deinit();
    ble_congested       = xSemaphoreCreateBinary();
    ble_task_mutex      = xSemaphoreCreateMutex();
    ble_settings_mutex  = xSemaphoreCreateMutex();
    spp_send_queue      = xQueueCreate(BLE_QUEUE_SIZE, sizeof(send_message_t));
    spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_cb = gatts_profile_event_handler;
    spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = ESP_GATT_IF_NONE;
    ESP_LOGI(BLE_TAG, "Init");
}

void ble_server_deinit(void)
{
    if (spp_send_queue) { vQueueDelete(spp_send_queue); spp_send_queue = NULL; }
    if (ble_congested) { vSemaphoreDelete(ble_congested); ble_congested = NULL; }
    if (ble_task_mutex) { vSemaphoreDelete(ble_task_mutex); ble_task_mutex = NULL; }
    if (ble_settings_mutex) { vSemaphoreDelete(ble_settings_mutex); ble_settings_mutex = NULL; }
}

void ble_server_start(ble_server_callbacks callbacks)
{
    ble_server_stop();
    server_callbacks = callbacks;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(ESP_SPP_APP_ID));

    ble_set_run_tasks(true);
    xTaskCreate(send_task, "BLE_sendTask", BLE_STACK_SIZE, NULL, BLE_TASK_PRIORITY, NULL);
    ESP_LOGI(BLE_TAG, "Started");
}

void ble_server_stop(void)
{
    if (ble_allow_run_tasks()) {
        ble_set_run_tasks(false);
        send_message_t msg = {.buffer = NULL};
        xQueueSend(spp_send_queue, &msg, portMAX_DELAY);
        tMUTEX(ble_task_mutex);
        rMUTEX(ble_task_mutex);
        while (xQueueReceive(spp_send_queue, &msg, 0) == pdTRUE) if(msg.buffer) free(msg.buffer);
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
    }
}

void ble_send(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size)
{
    send_message_t msg;
    msg.buffer = malloc(size);
    if (!msg.buffer) return;
    msg.msg_length = size;
    msg.rxID = rxID;
    msg.txID = txID;
    msg.flags = flags;
    memcpy(msg.buffer, src, size);
    if (xQueueSend(spp_send_queue, &msg, pdMS_TO_TICKS(TIMEOUT_NORMAL)) != pdTRUE) {
        free(msg.buffer);
    }
}

bool16 ble_connected(void)
{
    tMUTEX(ble_settings_mutex);
        bool16 connected = is_connected;
    rMUTEX(ble_settings_mutex);
    return connected;
}

uint16_t ble_queue_spaces(void) { return uxQueueSpacesAvailable(spp_send_queue); }
uint16_t ble_queue_waiting(void) { return uxQueueMessagesWaiting(spp_send_queue); }

void ble_set_delay_send(uint16_t delay) {
    tMUTEX(ble_settings_mutex); ble_delay_send = delay; rMUTEX(ble_settings_mutex);
}
void ble_set_delay_multi(uint16_t delay) {
    tMUTEX(ble_settings_mutex); ble_delay_multi = delay; rMUTEX(ble_settings_mutex);
}
uint16_t ble_get_delay_send(void) {
    tMUTEX(ble_settings_mutex); uint16_t d = ble_delay_send; rMUTEX(ble_settings_mutex); return d;
}
uint16_t ble_get_delay_multi(void) {
    tMUTEX(ble_settings_mutex); uint16_t d = ble_delay_multi; rMUTEX(ble_settings_mutex); return d;
}

bool16 ble_set_gap_name(char* name, bool16 set) {
    if(name && strlen(name) <= MAX_GAP_LENGTH) {
        tMUTEX(ble_settings_mutex);
            memset((char*)&spp_adv_data[9], 0, MAX_GAP_LENGTH);
            memcpy((char*)&spp_adv_data[9], name, strlen(name));
            spp_adv_data[7] = strlen(name)+1;
            strcpy(ble_gap_name, name);
            if(set) ESP_ERROR_CHECK(esp_ble_gap_set_device_name(ble_gap_name));
        rMUTEX(ble_settings_mutex);
        return true;
    }
    return false;
}

bool16 ble_get_gap_name(char* name) {
    if(name) {
        tMUTEX(ble_settings_mutex); strcpy(name, ble_gap_name); rMUTEX(ble_settings_mutex);
        return true;
    }
    return false;
}

bool16 ble_allow_connection(void) {
    tMUTEX(ble_settings_mutex); bool16 a = allow_connection; rMUTEX(ble_settings_mutex); return a;
}

void ble_stop_advertising(void) {
    tMUTEX(ble_settings_mutex); allow_connection = false; rMUTEX(ble_settings_mutex);
    ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
}

void ble_start_advertising(void) {
    tMUTEX(ble_settings_mutex); allow_connection = true; rMUTEX(ble_settings_mutex);
    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&spp_adv_params));
}