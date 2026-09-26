#ifndef MODE_MGR_H
#define MODE_MGR_H

#include "constants.h"
#include <stdbool.h>

void          mode_mgr_init(void);
dongle_mode_t mode_mgr_get(void);
void          mode_mgr_set(dongle_mode_t mode);
void          mode_mgr_toggle(void);
bool          mode_mgr_check_button_held(void);

#endif // MODE_MGR_H