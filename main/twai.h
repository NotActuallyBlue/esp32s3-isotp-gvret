#ifndef TWAI_H
#define TWAI_H

#include "driver/twai.h"

void twai_init(void);
void twai_deinit(void);
void twai_start_task(void);
void twai_start_raw(void);          // driver + bus-off recovery only, no receive task
void twai_stop_task(void);
void twai_send(twai_message_t *twai_tx_msg);

#endif // TWAI_H