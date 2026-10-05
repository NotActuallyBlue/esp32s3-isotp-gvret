#ifndef FLASHLOG_H
#define FLASHLOG_H

#include <stdint.h>

// Persistent log: mirrors every ESP_LOG line into the "log" flash partition so a
// session in the car can be read back later with tools/read_log.py.
// Call flashlog_init() first thing in app_main, before other tasks start.
void flashlog_init(void);

// For the display: this boot's number (0 if logging is disabled) and how full its log slot is
uint32_t flashlog_boot_number(void);
uint8_t  flashlog_used_percent(void);

#endif
