#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "display.h"
#include "ota_update.h"

#define OTA_TAG             "OtaUpdate"
#define OTA_IDLE_TIMEOUT_MS (10 * 60 * 1000)    // give up and restart if nothing is uploaded
#define OTA_CHUNK_SIZE      1024

static char ap_ssid[32];
static char ap_password[16];

static const char upload_page[] =
    "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>ISOTP-BLE update</title>"
    "<style>body{font-family:sans-serif;max-width:28em;margin:2em auto;padding:0 1em}"
    "button{padding:.6em 1.2em;font-size:1em}progress{width:100%}</style></head><body>"
    "<h2>ISOTP-BLE firmware update</h2>"
    "<p>Choose <b>firmware.bin</b> from the build folder and upload it. The dongle restarts when it is done.</p>"
    "<input type=file id=f accept='.bin'><p><button onclick='go()'>Upload</button></p>"
    "<progress id=p value=0 max=100></progress><pre id=o></pre>"
    "<script>function go(){var f=document.getElementById('f').files[0];if(!f){return}"
    "var x=new XMLHttpRequest();x.open('POST','/update');"
    "x.upload.onprogress=function(e){document.getElementById('p').value=100*e.loaded/e.total};"
    "x.onload=function(){document.getElementById('o').textContent=x.responseText};"
    "x.onerror=function(){document.getElementById('o').textContent='Upload failed'};"
    "x.send(f)}</script></body></html>";

static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
}

static void idle_timeout_cb(TimerHandle_t timer)
{
    ESP_LOGW(OTA_TAG, "No update received, restarting");
    esp_restart();
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, upload_page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t fail(httpd_req_t *req, const char *status, const char *message)
{
    ESP_LOGE(OTA_TAG, "Update rejected: %s", message);
    display_set_status("UPDATE", "FAILED", COLOR_RED);
    display_set_detail(3, "ERROR", message);
    httpd_resp_set_status(req, status);
    httpd_resp_send(req, message, HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
}

static esp_err_t update_handler(httpd_req_t *req)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        return fail(req, "500 Internal Server Error", "no update partition");
    }

    size_t total = req->content_len;
    if (total < 512 || total > target->size) {
        return fail(req, "400 Bad Request", "file size is not valid for this dongle");
    }

    static char buf[OTA_CHUNK_SIZE];
    esp_ota_handle_t handle = 0;
    size_t received = 0;
    int last_percent = -1;
    bool started = false;

    display_set_status("UPDATE", "UPLOADING", COLOR_YELLOW);
    ESP_LOGI(OTA_TAG, "Receiving %u bytes into %s", (unsigned)total, target->label);

    while (received < total) {
        size_t want = total - received < sizeof(buf) ? total - received : sizeof(buf);
        int got = httpd_req_recv(req, buf, want);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (got <= 0) {
            if (started) esp_ota_abort(handle);
            return fail(req, "500 Internal Server Error", "connection lost during upload");
        }

        if (!started) {
            // The first chunk carries the image header and the app descriptor. Refuse anything that is not
            // this project's firmware before a single byte is written.
            const size_t desc_offset = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
            if ((size_t)got < desc_offset + sizeof(esp_app_desc_t)) {
                return fail(req, "400 Bad Request", "not a firmware image");
            }
            const esp_app_desc_t *desc = (const esp_app_desc_t *)(buf + desc_offset);
            if (desc->magic_word != ESP_APP_DESC_MAGIC_WORD ||
                strncmp(desc->project_name, esp_app_get_description()->project_name, sizeof(desc->project_name)) != 0) {
                return fail(req, "400 Bad Request", "this is not ISOTP-BLE firmware");
            }
            if (esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle) != ESP_OK) {   // erase as we write, not all at once
                return fail(req, "500 Internal Server Error", "could not start the update");
            }
            started = true;
        }

        if (esp_ota_write(handle, buf, got) != ESP_OK) {
            esp_ota_abort(handle);
            return fail(req, "500 Internal Server Error", "flash write failed");
        }
        received += got;

        int percent = (int)((uint64_t)received * 100 / total);
        if (percent != last_percent && percent % 2 == 0) {
            last_percent = percent;
            char text[16];
            snprintf(text, sizeof(text), "%d%%", percent);
            display_set_detail(3, "UPLOAD", text);
        }
    }

    if (esp_ota_end(handle) != ESP_OK) {
        return fail(req, "400 Bad Request", "image check failed, the file is damaged");
    }
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
        return fail(req, "500 Internal Server Error", "could not select the new image");
    }

    ESP_LOGI(OTA_TAG, "Update written to %s, restarting", target->label);
    display_set_status("UPDATE", "SUCCESS", COLOR_GREEN);
    display_set_detail(3, "UPLOAD", "done, rebooting");
    httpd_resp_sendstr(req, "Update written. The dongle is restarting.");
    xTaskCreate(restart_task, "ota_restart", 2048, NULL, 1, NULL);
    return ESP_OK;
}

static void start_access_point(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    // The name carries the last bytes of the MAC so several dongles can be told apart; the password is
    // random for every update session and only appears on the dongle's own screen.
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(ap_ssid, sizeof(ap_ssid), "ISOTP-BLE-%02X%02X", mac[4], mac[5]);
    snprintf(ap_password, sizeof(ap_password), "%08lu", (unsigned long)(esp_random() % 100000000UL));

    wifi_config_t ap_cfg = { 0 };
    strlcpy((char *)ap_cfg.ap.ssid, ap_ssid, sizeof(ap_cfg.ap.ssid));
    strlcpy((char *)ap_cfg.ap.password, ap_password, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.ssid_len = strlen(ap_ssid);
    ap_cfg.ap.channel = 6;
    ap_cfg.ap.max_connection = 2;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void ota_update_start(void)
{
    display_set_status("UPDATE", "STARTING", COLOR_YELLOW);

    start_access_point();

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.recv_wait_timeout = 10;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler };
    httpd_uri_t update_uri = { .uri = "/update", .method = HTTP_POST, .handler = update_handler };
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &update_uri);

    TimerHandle_t idle_timer = xTimerCreate("ota_idle", pdMS_TO_TICKS(OTA_IDLE_TIMEOUT_MS), pdFALSE, NULL, idle_timeout_cb);
    if (idle_timer) xTimerStart(idle_timer, 0);

    display_set_detail(0, "WIFI", ap_ssid);
    display_set_detail(1, "PASSWORD", ap_password);
    display_set_detail(2, "OPEN", "192.168.4.1");
    display_set_status("UPDATE", "READY", COLOR_CYAN);

    ESP_LOGW(OTA_TAG, "Update mode: join Wi-Fi '%s' (password shown on the dongle) and open http://192.168.4.1", ap_ssid);
}
