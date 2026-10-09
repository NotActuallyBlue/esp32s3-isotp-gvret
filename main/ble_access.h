#pragma once

#include <stdbool.h>

// Bluetooth access window. The Simos and ELM327 Bluetooth interfaces have no password (the apps cannot take one), so the dongle only
// advertises, and so only accepts new connections, for a minute after it starts and after every button press. An app that is already
// connected is never cut off; when it disconnects the minute starts again. A press of BOOT (or KEY) opens the window again.
#define BLE_ACCESS_WINDOW_MS   60000

typedef bool (*ble_access_connected_fn)(void);
typedef void (*ble_access_switch_fn)(void);
typedef void (*ble_access_changed_fn)(bool open);

void ble_access_start(ble_access_connected_fn connected, ble_access_switch_fn advertising_on, ble_access_switch_fn advertising_off,
                      ble_access_changed_fn changed);
void ble_access_poke(void);         // a button was pressed: open the window for another minute
bool ble_access_open(void);         // true while connections are accepted (always true when the feature is not running)
