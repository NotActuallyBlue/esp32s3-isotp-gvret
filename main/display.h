#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// RGB565 colors in the byte order the panel expects. COLOR_RGB turns 8-bit red, green and blue into that order.
#define COLOR_RGB565(r, g, b)  ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define COLOR_RGB(r, g, b)     ((uint16_t)(((COLOR_RGB565(r, g, b)) >> 8) | ((COLOR_RGB565(r, g, b)) << 8)))

#define COLOR_BLACK            0x0000
#define COLOR_WHITE            0xFFFF
#define COLOR_RED              0x00F8
#define COLOR_GREEN            0xE007
#define COLOR_BLUE             0x1F00
#define COLOR_DARKGREY         0x1042
#define COLOR_LIGHTGREY        COLOR_RGB(190, 196, 208)
#define COLOR_CYAN             0xFF07
#define COLOR_YELLOW           0xE0FF
#define COLOR_ORANGE           0x00FD

// Global Traffic Counters
extern volatile uint32_t g_rx_count;
extern volatile uint32_t g_tx_count;
extern volatile uint32_t g_notify_count;    // BLE notifications sent
extern volatile uint32_t g_error_count;     // ISO-TP send/receive failures

void display_init(void);
void display_power(bool power_on);
bool display_is_awake(void);                 // backlight currently on
void display_set_bench(bool bench);          // label Simos statuses as the bench simulator

// Portrait UI API
void display_set_mode_view(const char *mode_title, const char *status_str, uint16_t state_color);
void display_update_traffic(uint32_t rx_count, uint32_t tx_count);

// Debug panel inputs (cheap, safe to call from any task)
void display_set_ble_info(uint16_t mtu, uint16_t conn_int);     // conn_int in units of 1.25 ms, 0/0 = no link
void display_set_persist(bool enabled);
void display_set_link_latency(uint8_t link, uint32_t ms);       // link 0 = ECU, 1 = TCU

// Button menu prompt: header and status only, no data rows
void display_set_prompt(const char *title, const char *status, uint16_t color);

// Cycle to the next page (status, bus monitor, firmware info)
void display_next_page(void);

// Detail rows replace the normal rows (used by the Wi-Fi update screen); set index 0..7, clear to go back
void display_set_detail(uint8_t index, const char *label, const char *value);
void display_clear_details(void);

// Replace all detail rows at once (up to 16) under a section title. A label starting with '#' is drawn as
// a section header. Color 0 = default. Only rows that changed are redrawn, so this can be called often.
typedef struct {
    const char *label;
    const char *value;
    uint16_t    color;
} display_detail_t;
void display_set_details(const char *title, uint16_t title_color, const display_detail_t *rows, int count);

// Compatibility shim with existing calls
void display_set_status(const char *transport, const char *status_msg, uint16_t color);

#ifdef __cplusplus
}
#endif