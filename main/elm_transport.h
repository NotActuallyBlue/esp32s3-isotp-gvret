#ifndef ELM_TRANSPORT_H
#define ELM_TRANSPORT_H

// The two ways a phone reaches the ELM327 emulation: BLE (what iPhones and most Android apps use) and
// Wi-Fi (TCP port 35000 on 192.168.0.10, the address generic ELM327 Wi-Fi adapters use).

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef void (*elm_rx_cb)(const uint8_t *data, size_t len);     // bytes from the phone (any task)
typedef void (*elm_link_cb)(bool connected);

// BLE: advertises as "ISOTP-ELM327" with the two service layouts ELM327 BLE adapters use
// (0xFFF0: notify 0xFFF1 / write 0xFFF2, and 0xFFE0: notify+write 0xFFE1).
void     elm_ble_start(elm_rx_cb rx, elm_link_cb link);
void     elm_ble_send(const uint8_t *data, size_t len);
bool     elm_ble_connected(void);
uint16_t elm_ble_mtu(void);
#define  ELM_BLE_NAME   "ISOTP-ELM327"

// Wi-Fi access point + TCP server. The password is generated once and kept in NVS; it is shown on the screen.
void        elm_wifi_start(elm_rx_cb rx, elm_link_cb link);
// Same access point and TCP server under another network name and port (SavvyCAN mode uses this)
void        elm_wifi_start_ex(const char *ssid, int port, elm_rx_cb rx, elm_link_cb link);
int         elm_wifi_port(void);
void        elm_wifi_send(const uint8_t *data, size_t len);
bool        elm_wifi_connected(void);
const char *elm_wifi_ssid(void);
const char *elm_wifi_password(void);
#define     ELM_WIFI_SSID   "ISOTP-ELM327"
#define     ELM_WIFI_IP     "192.168.0.10"
#define     ELM_WIFI_PORT   35000

#endif // ELM_TRANSPORT_H
