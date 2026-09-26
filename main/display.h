#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// RGB565 Color definitions for callers
#define COLOR_BLACK            0x0000
#define COLOR_WHITE            0xFFFF
#define COLOR_RED              0x00F8
#define COLOR_GREEN            0xE007
#define COLOR_BLUE             0x1F00
#define COLOR_DARKGREY         0x1042
#define COLOR_CYAN             0xFF07
#define COLOR_YELLOW           0xE0FF

void display_init(void);
void display_set_status(const char *transport, const char *status_msg, uint16_t color);
void display_power(bool power_on);

#ifdef __cplusplus
}
#endif