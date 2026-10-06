#ifndef MODE_MGR_H
#define MODE_MGR_H

#include "constants.h"
#include <stdbool.h>

void          mode_mgr_init(void);
dongle_mode_t mode_mgr_get(void);
void          mode_mgr_set(dongle_mode_t mode);
void          mode_mgr_toggle(void);

// Select a one-shot mode (bench simulator, Wi-Fi update) for the next restart only. Not saved: a power
// cycle or any later restart goes back to the saved mode. The caller restarts.
void          mode_mgr_request_oneshot(dongle_mode_t mode);

// True when this boot runs against the simulated modules instead of the CAN bus. mode_mgr_get() then
// returns the saved mode (Simos, SavvyCAN, Diag or ELM327) that is being tested.
bool          mode_mgr_is_bench(void);

// What a short press of the BOOT button does. The default cycles the display pages; a mode with its own
// screens (diagnostics) takes it over.
void          mode_mgr_set_tap_handler(void (*handler)(void));

#endif // MODE_MGR_H
