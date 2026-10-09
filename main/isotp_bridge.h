#ifndef ISOTP_BRIDGE_H
#define ISOTP_BRIDGE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "ble_server.h"

void        isotp_init(void);
void        isotp_deinit(void);
void        isotp_start_task(void);
void        isotp_stop_task(void);

void        bridge_connect(void);
void        bridge_disconnect(void);
void        bridge_received_ble(const void* src, size_t size);
int32_t     bridge_send_isotp(send_message_t *msg);
uint16_t    bridge_send_available(void);

// No frame has been received for BUS_QUIET_MS: the car is off or nothing answers. Never true on the bench simulator.
#define BUS_QUIET_MS 3000
bool        bridge_bus_quiet(void);

#endif // ISOTP_BRIDGE_H