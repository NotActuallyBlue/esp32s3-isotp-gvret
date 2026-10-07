// Wi-Fi side of the ELM327 emulation: an access point and a TCP server on 192.168.0.10:35000, which is where
// generic ELM327 Wi-Fi apps look for the adapter.
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "elm_transport.h"

#define WIFI_TAG        "ElmWiFi"
#define NVS_NAMESPACE   "dongle_cfg"
#define NVS_KEY_PASS    "elm_wifi_pw"

static char                 password[12];
static const char          *ap_ssid = ELM_WIFI_SSID;
static int                  tcp_port = ELM_WIFI_PORT;
static int                  client_fd = -1;
static SemaphoreHandle_t    client_mutex;
static elm_rx_cb            rx_cb;
static elm_link_cb          link_cb;

bool elm_wifi_connected(void) { return client_fd >= 0; }
const char *elm_wifi_ssid(void) { return ap_ssid; }
int         elm_wifi_port(void) { return tcp_port; }
const char *elm_wifi_password(void) { return password; }

// Created on first use and kept, so the phone can remember the network. No look-alike characters.
static void load_password(void)
{
    static const char alphabet[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
    nvs_handle_t nvs;
    size_t size = sizeof(password);
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        strlcpy(password, "elm327pw", sizeof(password));
        return;
    }
    if (nvs_get_str(nvs, NVS_KEY_PASS, password, &size) != ESP_OK || strlen(password) < 8) {
        for (int i = 0; i < 8; i++) password[i] = alphabet[esp_random() % (sizeof(alphabet) - 1)];
        password[8] = 0;
        nvs_set_str(nvs, NVS_KEY_PASS, password);
        nvs_commit(nvs);
    }
    nvs_close(nvs);
}

void elm_wifi_send(const uint8_t *data, size_t len)
{
    xSemaphoreTake(client_mutex, portMAX_DELAY);
    int fd = client_fd;
    size_t pos = 0;
    while (fd >= 0 && pos < len) {
        int n = send(fd, data + pos, len - pos, 0);
        if (n <= 0) {
            // The client stopped reading or is gone (the send timed out or failed). Half a reply would corrupt the stream, so
            // end the connection; the reader task then notices and cleans up, and the dongle is free for a new client.
            shutdown(fd, SHUT_RDWR);
            break;
        }
        pos += n;
    }
    xSemaphoreGive(client_mutex);
}

static void drop_client(int fd)
{
    xSemaphoreTake(client_mutex, portMAX_DELAY);
    if (client_fd == fd) client_fd = -1;
    xSemaphoreGive(client_mutex);
    close(fd);
    ESP_LOGI(WIFI_TAG, "Client disconnected");
    if (link_cb) link_cb(false);
}

static void server_task(void *arg)
{
    int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int yes = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(tcp_port), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(listener, (struct sockaddr *)&addr, sizeof(addr)) != 0 || listen(listener, 1) != 0) {
        ESP_LOGE(WIFI_TAG, "Cannot listen on port %d (errno %d)", tcp_port, errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(WIFI_TAG, "Listening on %s:%d", ELM_WIFI_IP, tcp_port);

    while (1) {
        int fd = accept(listener, NULL, NULL);
        if (fd < 0) continue;

        // One phone at a time: a new connection replaces the old one
        xSemaphoreTake(client_mutex, portMAX_DELAY);
        int old = client_fd;
        client_fd = fd;
        xSemaphoreGive(client_mutex);
        if (old >= 0) close(old);

        int nodelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        struct timeval tv = { .tv_sec = 1 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        struct timeval send_tv = { .tv_sec = 0, .tv_usec = 500000 };
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &send_tv, sizeof(send_tv));

        // A phone or laptop that leaves without closing the connection is detected in about 11 s, so the dongle can sleep again
        int keepalive = 1, idle = 5, interval = 2, count = 3;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
        ESP_LOGI(WIFI_TAG, "Client connected");
        if (link_cb) link_cb(true);

        uint8_t buf[128];
        while (client_fd == fd) {
            int n = recv(fd, buf, sizeof(buf), 0);
            if (n > 0) {
                if (rx_cb) rx_cb(buf, n);
            } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
                break;
            }
        }
        if (client_fd == fd) drop_client(fd);
    }
}

void elm_wifi_start(elm_rx_cb rx, elm_link_cb link)
{
    elm_wifi_start_ex(ELM_WIFI_SSID, ELM_WIFI_PORT, rx, link);
}

void elm_wifi_start_ex(const char *ssid, int port, elm_rx_cb rx, elm_link_cb link)
{
    ap_ssid = ssid;
    tcp_port = port;
    rx_cb = rx;
    link_cb = link;
    client_mutex = xSemaphoreCreateMutex();
    load_password();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();

    // Generic ELM327 Wi-Fi adapters sit at 192.168.0.10
    esp_netif_ip_info_t ip = { 0 };
    ip.ip.addr = esp_ip4addr_aton(ELM_WIFI_IP);
    ip.gw.addr = ip.ip.addr;
    ip.netmask.addr = esp_ip4addr_aton("255.255.255.0");
    esp_netif_dhcps_stop(ap);
    esp_netif_set_ip_info(ap, &ip);
    esp_netif_dhcps_start(ap);

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    wifi_config_t ap_cfg = { 0 };
    strlcpy((char *)ap_cfg.ap.ssid, ap_ssid, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = strlen(ap_ssid);
    strlcpy((char *)ap_cfg.ap.password, password, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.channel = 6;
    ap_cfg.ap.max_connection = 2;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

#ifdef ELM_LOG_PASSWORD
    ESP_LOGW(WIFI_TAG, "TEST BUILD: Wi-Fi password is %s", password);     // never in release builds
#endif
    xTaskCreate(server_task, "elm_tcp", 4096, NULL, 2, NULL);
    ESP_LOGI(WIFI_TAG, "Access point '%s' started", ap_ssid);
}
