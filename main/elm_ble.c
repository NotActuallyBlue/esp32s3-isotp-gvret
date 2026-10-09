// BLE side of the ELM327 emulation. GATT layout follows the common BLE ELM327 adapters so generic apps find it:
//   service 0xFFF0  notify 0xFFF1, write 0xFFF2
//   service 0xFFE0  notify + write 0xFFE1
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_defs.h"
#include "display.h"
#include "elm_transport.h"

#define BLE_TAG             "ElmBLE"
#define ELM_APP_ID          0x57
#define DEFAULT_MTU         23
#define SERVICE_COUNT       2
#define SEND_GAP_MS         3

enum { A_SVC, A_NTF_DECL, A_NTF_VAL, A_NTF_CCC, A_WR_DECL, A_WR_VAL, A_MAX_FFF0 };     // 0xFFF0
enum { B_SVC, B_DECL, B_VAL, B_CCC, B_MAX };                                          // 0xFFE0

static const uint16_t uuid_primary = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t uuid_decl = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t uuid_ccc = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t svc_fff0 = 0xFFF0, chr_fff1 = 0xFFF1, chr_fff2 = 0xFFF2;
static const uint16_t svc_ffe0 = 0xFFE0, chr_ffe1 = 0xFFE1;
static const uint8_t prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t prop_all = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t zero_val[20] = { 0 };
static const uint8_t ccc_val[2] = { 0, 0 };

#define ATTR(uuid_ptr, perm, maxlen, curlen, valptr) \
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)(uuid_ptr), (perm), (maxlen), (curlen), (uint8_t *)(valptr)}}

static const esp_gatts_attr_db_t db_fff0[A_MAX_FFF0] = {
    [A_SVC]      = ATTR(&uuid_primary, ESP_GATT_PERM_READ, 2, 2, &svc_fff0),
    [A_NTF_DECL] = ATTR(&uuid_decl, ESP_GATT_PERM_READ, 1, 1, &prop_read_notify),
    [A_NTF_VAL]  = ATTR(&chr_fff1, ESP_GATT_PERM_READ, 512, sizeof(zero_val), zero_val),
    [A_NTF_CCC]  = ATTR(&uuid_ccc, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 2, 2, ccc_val),
    [A_WR_DECL]  = ATTR(&uuid_decl, ESP_GATT_PERM_READ, 1, 1, &prop_write),
    [A_WR_VAL]   = ATTR(&chr_fff2, ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ, 512, sizeof(zero_val), zero_val),
};

static const esp_gatts_attr_db_t db_ffe0[B_MAX] = {
    [B_SVC]  = ATTR(&uuid_primary, ESP_GATT_PERM_READ, 2, 2, &svc_ffe0),
    [B_DECL] = ATTR(&uuid_decl, ESP_GATT_PERM_READ, 1, 1, &prop_all),
    [B_VAL]  = ATTR(&chr_ffe1, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 512, sizeof(zero_val), zero_val),
    [B_CCC]  = ATTR(&uuid_ccc, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 2, 2, ccc_val),
};

static uint16_t             handles_fff0[A_MAX_FFF0];
static uint16_t             handles_ffe0[B_MAX];
static bool                 ntf_fff1, ntf_ffe1;
static esp_gatt_if_t        gatts_if_saved = ESP_GATT_IF_NONE;
static uint16_t             conn_id_saved = 0xFFFF;
static volatile bool        connected;
static volatile bool        congested;
static volatile bool        adv_allowed = true;     // false while the access window is closed
static uint16_t             mtu = DEFAULT_MTU;
static elm_rx_cb            rx_cb;
static elm_link_cb          link_cb;
static SemaphoreHandle_t    send_mutex;

// Flags, the 0xFFF0 service and the name
static uint8_t adv_data[31];
static int     adv_len;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

bool elm_ble_connected(void) { return connected; }

void elm_ble_set_advertising(bool on)
{
    adv_allowed = on;
    if (on) esp_ble_gap_start_advertising(&adv_params);
    else esp_ble_gap_stop_advertising();
}
uint16_t elm_ble_mtu(void) { return mtu; }

void elm_ble_send(const uint8_t *data, size_t len)
{
    if (!connected || gatts_if_saved == ESP_GATT_IF_NONE) return;
    uint16_t handle = ntf_fff1 ? handles_fff0[A_NTF_VAL] : (ntf_ffe1 ? handles_ffe0[B_VAL] : 0);
    if (!handle) return;

    xSemaphoreTake(send_mutex, portMAX_DELAY);
    size_t chunk = mtu > 3 ? mtu - 3 : 20;
    for (size_t pos = 0; pos < len && connected; pos += chunk) {
        for (int wait = 0; congested && wait < 500; wait++) vTaskDelay(pdMS_TO_TICKS(1));
        size_t n = len - pos > chunk ? chunk : len - pos;
        esp_ble_gatts_send_indicate(gatts_if_saved, conn_id_saved, handle, n, (uint8_t *)data + pos, false);
        vTaskDelay(pdMS_TO_TICKS(SEND_GAP_MS));
    }
    xSemaphoreGive(send_mutex);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        if (adv_allowed) esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        display_set_ble_info(mtu, param->update_conn_params.conn_int);
        ESP_LOGI(BLE_TAG, "Connection interval %d (x1.25 ms)", param->update_conn_params.conn_int);
        break;
    default:
        break;
    }
}

static void handle_write(esp_ble_gatts_cb_param_t *p)
{
    uint16_t h = p->write.handle;
    if (h == handles_fff0[A_NTF_CCC] || h == handles_ffe0[B_CCC]) {
        bool on = p->write.len == 2 && (p->write.value[0] & 0x01);
        if (h == handles_fff0[A_NTF_CCC]) ntf_fff1 = on; else ntf_ffe1 = on;
        ESP_LOGI(BLE_TAG, "Notifications %s on %s", on ? "enabled" : "disabled", h == handles_fff0[A_NTF_CCC] ? "0xFFF1" : "0xFFE1");
    } else if (h == handles_fff0[A_WR_VAL] || h == handles_ffe0[B_VAL]) {
        if (rx_cb) rx_cb(p->write.value, p->write.len);
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        gatts_if_saved = gatts_if;
        esp_ble_gap_set_device_name(ELM_BLE_NAME);
        esp_ble_gap_config_adv_data_raw(adv_data, adv_len);
        esp_ble_gatts_create_attr_tab(db_fff0, gatts_if, A_MAX_FFF0, 0);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(BLE_TAG, "Attribute table %d failed: %d", param->add_attr_tab.svc_inst_id, param->add_attr_tab.status);
            break;
        }
        if (param->add_attr_tab.svc_inst_id == 0) {
            memcpy(handles_fff0, param->add_attr_tab.handles, sizeof(handles_fff0));
            esp_ble_gatts_start_service(handles_fff0[A_SVC]);
            esp_ble_gatts_create_attr_tab(db_ffe0, gatts_if, B_MAX, 1);
        } else {
            memcpy(handles_ffe0, param->add_attr_tab.handles, sizeof(handles_ffe0));
            esp_ble_gatts_start_service(handles_ffe0[B_SVC]);
        }
        break;
    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep) handle_write(param);
        break;
    case ESP_GATTS_MTU_EVT:
        mtu = param->mtu.mtu;
        ESP_LOGI(BLE_TAG, "MTU %u", mtu);
        break;
    case ESP_GATTS_CONNECT_EVT: {
        conn_id_saved = param->connect.conn_id;
        connected = true;
        congested = false;
        esp_ble_conn_update_params_t cp = { 0 };
        memcpy(cp.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        cp.min_int = 0x06; cp.max_int = 0x0C; cp.latency = 0; cp.timeout = 400;
        esp_ble_gap_update_conn_params(&cp);
        esp_ble_gap_set_pkt_data_len(param->connect.remote_bda, 251);
        ESP_LOGI(BLE_TAG, "Client connected");
        if (link_cb) link_cb(true);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        connected = false;
        ntf_fff1 = ntf_ffe1 = false;
        mtu = DEFAULT_MTU;
        conn_id_saved = 0xFFFF;
        display_set_ble_info(0, 0);
        ESP_LOGI(BLE_TAG, "Client disconnected, advertising again");
        if (link_cb) link_cb(false);
        if (adv_allowed) esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONGEST_EVT:
        congested = param->congest.congested;
        break;
    default:
        break;
    }
}

void elm_ble_start(elm_rx_cb rx, elm_link_cb link)
{
    rx_cb = rx;
    link_cb = link;
    send_mutex = xSemaphoreCreateMutex();

    int n = 0;
    adv_data[n++] = 2; adv_data[n++] = 0x01; adv_data[n++] = 0x06;                       // flags
    adv_data[n++] = 3; adv_data[n++] = 0x03; adv_data[n++] = 0xF0; adv_data[n++] = 0xFF; // service 0xFFF0
    size_t name_len = strlen(ELM_BLE_NAME);
    adv_data[n++] = (uint8_t)(name_len + 1); adv_data[n++] = 0x09;                       // complete local name
    memcpy(adv_data + n, ELM_BLE_NAME, name_len);
    n += name_len;
    adv_len = n;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(ELM_APP_ID));
    esp_ble_gatt_set_local_mtu(247);
    ESP_LOGI(BLE_TAG, "Started as '%s'", ELM_BLE_NAME);
}
