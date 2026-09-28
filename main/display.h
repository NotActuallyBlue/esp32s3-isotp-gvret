#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// RGB565 Color definitions
#define COLOR_BLACK            0x0000
#define COLOR_WHITE            0xFFFF
#define COLOR_RED              0x00F8
#define COLOR_GREEN            0xE007
#define COLOR_BLUE             0x1F00
#define COLOR_DARKGREY         0x1042
#define COLOR_LIGHTGREY        0x8410
#define COLOR_CYAN             0xFF07
#define COLOR_YELLOW           0xE0FF
#define COLOR_ORANGE           0x00FD

// Global Traffic Counters
extern volatile uint32_t g_rx_count;
extern volatile uint32_t g_tx_count;

typedef enum {
    ICON_NONE = 0,
    ICON_BLUETOOTH,
    ICON_USB,
    ICON_OBD
} display_icon_t;

void display_init(void);
void display_clear(uint16_t color);
void display_power(bool power_on);

// Portrait UI API
void display_set_mode_view(const char *mode_title, display_icon_t icon, const char *status_str, uint16_t state_color);
void display_update_traffic(uint32_t rx_count, uint32_t tx_count);

// Compatibility shim with existing calls
void display_set_status(const char *transport, const char *status_msg, uint16_t color);

#ifdef __cplusplus
}
#endif