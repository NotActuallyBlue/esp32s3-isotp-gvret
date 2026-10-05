#ifndef PERSIST_H
#define PERSIST_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"

void         persist_init(void);
void         persist_deinit(void);
void         persist_start_task(void);
void         persist_stop_task(void);

void         persist_allow_send(uint16_t persist);
uint16_t     persist_enabled(void);
void         persist_set(uint16_t enable);
int16_t      persist_add(uint16_t rx, uint16_t tx, const void* src, size_t size);
void         persist_clear(void);
bool         persist_log_window(void);
void         persist_note_reply(uint16_t link, uint16_t size);          // a reply was delivered while persist was on
void         persist_note_late_reply(uint16_t link);     // a reply arrived just after persist was switched off

// Per-frame traffic is only logged for a short window after persist starts, to keep the flash log small
#define PERSIST_LOG_WINDOW(tag, ...) ESP_LOG_LEVEL(persist_log_window() ? ESP_LOG_INFO : ESP_LOG_DEBUG, tag, __VA_ARGS__)
void         persist_clear_link(uint16_t rx, uint16_t tx);
void         persist_task(void *arg);
void         persist_set_delay(uint16_t delay);
void         persist_set_q_delay(uint16_t delay);
uint16_t     persist_get_delay(void);
uint16_t     persist_get_q_delay(void);

#endif // PERSIST_H
