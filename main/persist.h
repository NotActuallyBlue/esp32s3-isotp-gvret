#ifndef PERSIST_H
#define PERSIST_H

#include <stdint.h>

void         persist_init(void);
void         persist_deinit(void);
void         persist_start_task(void);
void         persist_stop_task(void);

void         persist_allow_send(uint16_t persist);
uint16_t     persist_enabled(void);
void         persist_set(uint16_t enable);
int16_t      persist_add(uint16_t rx, uint16_t tx, const void* src, size_t size);
void         persist_clear(void);
void         persist_task(void *arg);
void         persist_set_delay(uint16_t delay);
void         persist_set_q_delay(uint16_t delay);
uint16_t     persist_get_delay(void);
uint16_t     persist_get_q_delay(void);

#endif // PERSIST_H