#ifndef ELM_MODE_H
#define ELM_MODE_H

#include <stdbool.h>

// ELM327 emulation mode: generic OBD apps (Car Scanner, OBD Fusion, Torque, ...) connect over BLE or Wi-Fi
// and the dongle answers like an ELM327 adapter. See elm327.h for what is supported.
// bench = answer from the simulated modules instead of the CAN bus.
void elm_mode_start(bool bench);
bool elm_mode_clients_connected(void);     // for the sleep logic

#endif // ELM_MODE_H
