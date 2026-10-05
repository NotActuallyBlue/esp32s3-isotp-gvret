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

#endif // MODE_MGR_H
