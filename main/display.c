#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "display.h"
#include "flashlog.h"

#define TAG "DISPLAY"

// LilyGo T-Display-S3 Parallel 8080 Pinout
#define PIN_NUM_LCD_PWR         GPIO_NUM_15
#define PIN_NUM_BK_LIGHT        GPIO_NUM_38
#define PIN_NUM_LCD_CS          GPIO_NUM_6
#define PIN_NUM_LCD_DC          GPIO_NUM_7
#define PIN_NUM_LCD_WR          GPIO_NUM_8
#define PIN_NUM_LCD_RD          GPIO_NUM_9
#define PIN_NUM_LCD_RST         GPIO_NUM_5

#define PIN_NUM_LCD_D0          GPIO_NUM_39
#define PIN_NUM_LCD_D1          GPIO_NUM_40
#define PIN_NUM_LCD_D2          GPIO_NUM_41
#define PIN_NUM_LCD_D3          GPIO_NUM_42
#define PIN_NUM_LCD_D4          GPIO_NUM_45
#define PIN_NUM_LCD_D5          GPIO_NUM_46
#define PIN_NUM_LCD_D6          GPIO_NUM_47
#define PIN_NUM_LCD_D7          GPIO_NUM_48

// LilyGo T-Display-S3 Resolution
#define LCD_H_RES               170
#define LCD_V_RES               320

// Global Traffic Counters
volatile uint32_t g_rx_count = 0;
volatile uint32_t g_tx_count = 0;
volatile uint32_t g_notify_count = 0;
volatile uint32_t g_error_count = 0;

static esp_lcd_panel_handle_t panel_handle = NULL;
static TimerHandle_t display_timer = NULL;
static bool display_is_on = true;

// Everything is drawn by display_task. Other tasks only set the state below, so no two tasks ever
// draw at once and the pixel strip is never overwritten while a transfer is still using it.
typedef struct {
    char            title[16];
    display_icon_t  icon;
    char            status[16];
    uint16_t        color;
} display_view_t;

static display_view_t       view            = { "STANDBY", ICON_BLUETOOTH, "READY", COLOR_CYAN };
static bool                 view_dirty      = true;
static portMUX_TYPE         state_lock      = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t         display_task_handle = NULL;
static SemaphoreHandle_t    tx_done_sem     = NULL;

static volatile uint16_t    info_mtu        = 0;
static volatile uint16_t    info_conn_int   = 0;    // units of 1.25 ms
static volatile bool        info_persist    = false;
static volatile uint32_t    info_latency_ms[2] = { 0, 0 };

// Basic 8x8 ASCII Font
static const uint8_t font8x8_basic[95][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, 
    {0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00}, {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, 
    {0x18,0x3E,0x60,0x3C,0x06,0x7C,0x18,0x00}, {0x00,0x63,0x66,0x0C,0x18,0x33,0x63,0x00}, 
    {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, {0x30,0x30,0x10,0x20,0x00,0x00,0x00,0x00}, 
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, 
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, 
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, {0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00}, 
    {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00}, {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, 
    {0x3C,0x66,0x06,0x0C,0x18,0x30,0x7E,0x00}, {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00}, 
    {0x0C,0x1C,0x3C,0x6C,0xFE,0x0C,0x0C,0x00}, {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00}, 
    {0x1C,0x30,0x60,0x7C,0x66,0x66,0x3C,0x00}, {0x7E,0xC6,0x06,0x0C,0x18,0x18,0x18,0x00}, 
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, {0x3C,0x66,0x66,0x3E,0x06,0x0C,0x38,0x00}, 
    {0x00,0x18,0x18,0x00,0x18,0x18,0x00,0x00}, {0x00,0x18,0x18,0x00,0x18,0x18,0x30,0x00}, 
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00}, 
    {0x60,0x30,0x18,0x0C,0x18,0x30,0x60,0x00}, {0x3C,0x66,0x0C,0x18,0x18,0x00,0x18,0x00}, 
    {0x3C,0x66,0x6E,0x6E,0x60,0x62,0x3C,0x00}, {0x18,0x3C,0x66,0x7E,0x66,0x66,0x66,0x00}, 
    {0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00}, {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00}, 
    {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00}, {0x7E,0x60,0x60,0x7C,0x60,0x60,0x7E,0x00}, 
    {0x7E,0x60,0x60,0x7C,0x60,0x60,0x60,0x00}, {0x3C,0x66,0x60,0x6E,0x66,0x66,0x3A,0x00}, 
    {0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00}, {0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, 
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x6C,0x38,0x00}, {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00}, 
    {0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00}, {0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00}, 
    {0x66,0x76,0x7E,0x7E,0x6E,0x66,0x66,0x00}, {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, 
    {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00}, {0x3C,0x66,0x66,0x66,0x6E,0x3C,0x0E,0x00}, 
    {0x7C,0x66,0x66,0x7C,0x78,0x6C,0x66,0x00}, {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00}, 
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, {0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, 
    {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, 
    {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00}, 
    {0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00}, {0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00}, 
    {0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00}, {0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00}, 
    {0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, 
    {0x30,0x18,0x0C,0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x00}, 
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, {0x00,0x00,0x3C,0x66,0x60,0x66,0x3C,0x00}, 
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00}, {0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x00}, 
    {0x1C,0x30,0x78,0x30,0x30,0x30,0x30,0x00}, {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x3C}, 
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x00}, {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, 
    {0x0C,0x00,0x1C,0x0C,0x0C,0x0C,0x6C,0x38}, {0x60,0x60,0x66,0x6C,0x78,0x6C,0x66,0x00}, 
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, {0x00,0x00,0x66,0x7F,0x7F,0x6B,0x63,0x00}, 
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x00}, {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, 
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60}, {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x06}, 
    {0x00,0x00,0x7C,0x66,0x60,0x60,0x60,0x00}, {0x00,0x00,0x3E,0x60,0x3C,0x06,0x7C,0x00}, 
    {0x18,0x18,0x7E,0x18,0x18,0x18,0x0E,0x00}, {0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x00}, 
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, {0x00,0x00,0x63,0x6B,0x7F,0x36,0x36,0x00}, 
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, {0x00,0x00,0x66,0x66,0x66,0x3E,0x06,0x3C}, 
    {0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00}, {0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00}, 
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, {0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00}, 
    {0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00}
};

// Bluetooth Icon (32x32)
static const uint8_t icon_bluetooth_32x32[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x03, 0x80, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x03, 0xf0, 0x00, 
    0x00, 0x03, 0xb8, 0x00, 0x00, 0x43, 0x9c, 0x00, 0x00, 0x73, 0x9c, 0x00, 0x00, 0x7b, 0xb8, 0x00, 
    0x00, 0x3f, 0xf0, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x07, 0x80, 0x00, 
    0x00, 0x07, 0xc0, 0x00, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x3b, 0xf8, 0x00, 
    0x00, 0x73, 0xbc, 0x00, 0x00, 0x63, 0x9c, 0x00, 0x00, 0x43, 0x9c, 0x00, 0x00, 0x03, 0xb8, 0x00, 
    0x00, 0x03, 0xf0, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x03, 0x80, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// USB-C / Pill Icon (32x32)
static const uint8_t icon_usb_32x32[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x00, 
    0x00, 0x40, 0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x4e, 0x72, 0x00, 0x00, 0x4e, 0x72, 0x00, 
    0x00, 0x4e, 0x72, 0x00, 0x00, 0x4e, 0x72, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 
    0x00, 0xff, 0xff, 0x00, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 
    0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 
    0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 
    0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x80, 0x01, 0x80, 
    0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ---------------------------------------------------------------------------------------------
// Drawing core (display_task only)
// ---------------------------------------------------------------------------------------------
#define STRIP_ROWS      16
static uint16_t strip[LCD_H_RES * STRIP_ROWS];

static bool display_on_color_trans_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(tx_done_sem, &woken);
    return woken == pdTRUE;
}

// Send the first w*h pixels of the strip to the panel and wait until the transfer has finished
static void display_push(int x, int y, int w, int h)
{
    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + w, y + h, strip);
    xSemaphoreTake(tx_done_sem, pdMS_TO_TICKS(100));
}

static void display_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_H_RES) w = LCD_H_RES - x;
    if (y + h > LCD_V_RES) h = LCD_V_RES - y;
    if (w <= 0 || h <= 0) return;

    for (int cur_y = y; cur_y < y + h; cur_y += STRIP_ROWS) {
        int rows = (y + h - cur_y < STRIP_ROWS) ? (y + h - cur_y) : STRIP_ROWS;
        for (int i = 0; i < w * rows; i++) strip[i] = color;
        display_push(x, cur_y, w, rows);
    }
}

void display_clear(uint16_t color)
{
    display_fill_rect(0, 0, LCD_H_RES, LCD_V_RES, color);
}

// Draw text centered in a box [x0, x0 + box_w), padding with the background. One transfer per call.
static void display_draw_text(int x0, int y, int box_w, const char *str, uint16_t color, uint16_t bg, int scale, bool center)
{
    if (scale < 1 || 8 * scale > STRIP_ROWS) return;
    if (x0 < 0) x0 = 0;
    if (x0 + box_w > LCD_H_RES) box_w = LCD_H_RES - x0;
    if (box_w <= 0 || y < 0 || y + 8 * scale > LCD_V_RES) return;

    int len = strlen(str);
    int max_chars = box_w / (8 * scale);
    if (len > max_chars) len = max_chars;
    int text_w = len * 8 * scale;
    int text_x = center ? (box_w - text_w) / 2 : 0;
    int rows = 8 * scale;

    for (int py = 0; py < rows; py++) {
        uint16_t *line = &strip[py * box_w];
        for (int px = 0; px < box_w; px++) line[px] = bg;

        int glyph_row = py / scale;
        for (int ci = 0; ci < len; ci++) {
            char c = str[ci];
            if (c < 32 || c > 126) c = ' ';
            uint8_t bits = font8x8_basic[c - 32][glyph_row];
            int base = text_x + ci * 8 * scale;
            for (int b = 0; b < 8; b++) {
                if (bits & (0x80 >> b)) {
                    for (int s = 0; s < scale; s++) line[base + b * scale + s] = color;
                }
            }
        }
    }
    display_push(x0, y, box_w, rows);
}

// Draw a 32x32 1-bit icon scaled up, in bands of STRIP_ROWS rows
static void display_draw_icon(int x, int y, const uint8_t *bitmap, uint16_t color, uint16_t bg, int scale)
{
    int dim = 32 * scale;
    if (dim > 64 || x < 0 || x + dim > LCD_H_RES || y < 0 || y + dim > LCD_V_RES) return;

    for (int band = 0; band < dim; band += STRIP_ROWS) {
        int rows = (dim - band < STRIP_ROWS) ? (dim - band) : STRIP_ROWS;
        for (int py = 0; py < rows; py++) {
            int src_row = (band + py) / scale;
            uint16_t *line = &strip[py * dim];
            for (int sx = 0; sx < 32; sx++) {
                uint8_t byte_val = bitmap[src_row * 4 + (sx / 8)];
                uint16_t px = (byte_val & (0x80 >> (sx % 8))) ? color : bg;
                for (int s = 0; s < scale; s++) line[sx * scale + s] = px;
            }
        }
        display_push(x, y + band, dim, rows);
    }
}

// ---------------------------------------------------------------------------------------------
// Backlight
// ---------------------------------------------------------------------------------------------
void display_power(bool enable)
{
    display_is_on = enable;
    gpio_set_level(PIN_NUM_BK_LIGHT, enable ? 1 : 0);
}

static void display_timer_callback(TimerHandle_t xTimer)
{
    display_power(false);
}

static void display_bump_timer(void)
{
    if (!display_is_on) display_power(true);
    if (display_timer) xTimerReset(display_timer, 0);
}

// ---------------------------------------------------------------------------------------------
// Layout (170 x 320 portrait)
//   title 12 | icon 38-102 | status pill 110-144 | debug panel 156-212 | divider 222 | RX/TX 236-290
// ---------------------------------------------------------------------------------------------
#define DEBUG_Y         156
#define DEBUG_LINE_H    12
#define DEBUG_LINES     5
#define DIVIDER_Y       222
#define COUNTERS_Y      236

static char     info_cache[DEBUG_LINES][24];
static uint16_t info_color_cache[DEBUG_LINES];
static char     rx_cache[24];
static char     tx_cache[24];

static bool view_shows_debug(const display_view_t *v)
{
    return strcmp(v->title, "SIMOS") == 0;
}

static void display_draw_view(const display_view_t *v)
{
    display_clear(COLOR_BLACK);

    int scale = (strlen(v->title) <= 9) ? 2 : 1;
    display_draw_text(0, (scale == 2) ? 12 : 16, LCD_H_RES, v->title, COLOR_WHITE, COLOR_BLACK, scale, true);

    int icon_x = (LCD_H_RES - 64) / 2;
    if (v->icon == ICON_BLUETOOTH || v->icon == ICON_OBD) {
        display_draw_icon(icon_x, 38, icon_bluetooth_32x32, v->color, COLOR_BLACK, 2);
    } else if (v->icon == ICON_USB) {
        display_draw_icon(icon_x, 38, icon_usb_32x32, v->color, COLOR_BLACK, 2);
    }

    int pill_w = 140, pill_h = 34, pill_x = (LCD_H_RES - pill_w) / 2, pill_y = 110;
    display_fill_rect(pill_x, pill_y, pill_w, pill_h, v->color);
    int status_scale = (strlen(v->status) <= 8) ? 2 : 1;
    display_draw_text(pill_x, pill_y + (pill_h - 8 * status_scale) / 2, pill_w, v->status, COLOR_BLACK, v->color, status_scale, true);

    display_fill_rect(10, DIVIDER_Y, LCD_H_RES - 20, 1, COLOR_DARKGREY);

    // Force the info rows to redraw
    memset(info_cache, 0, sizeof(info_cache));
    memset(rx_cache, 0, sizeof(rx_cache));
    memset(tx_cache, 0, sizeof(tx_cache));
}

static void display_format_info(int line, char *out, size_t out_size, uint16_t *color, uint32_t notify_rate)
{
    *color = COLOR_LIGHTGREY;

    switch (line) {
    case 0: {
        uint16_t mtu = info_mtu, ci = info_conn_int;
        if (mtu == 0 && ci == 0) {
            snprintf(out, out_size, "BLE  no link");
        } else {
            uint32_t x100 = (uint32_t)ci * 125;
            snprintf(out, out_size, "MTU %u  INT %lu.%lums", mtu, (unsigned long)(x100 / 100), (unsigned long)((x100 % 100) / 10));
        }
        break;
    }
    case 1:
        if (info_persist) {
            snprintf(out, out_size, "STREAM ON  %lu/s", (unsigned long)notify_rate);
            *color = COLOR_GREEN;
        } else {
            snprintf(out, out_size, "STREAM OFF");
        }
        break;
    case 2: {
        char ecu[12], tcu[12];
        uint32_t e = info_latency_ms[0], t = info_latency_ms[1];
        if (e) snprintf(ecu, sizeof(ecu), "%lums", (unsigned long)(e > 999 ? 999 : e)); else snprintf(ecu, sizeof(ecu), "--");
        if (t) snprintf(tcu, sizeof(tcu), "%lums", (unsigned long)(t > 999 ? 999 : t)); else snprintf(tcu, sizeof(tcu), "--");
        snprintf(out, out_size, "ECU %s TCU %s", ecu, tcu);
        break;
    }
    case 3: {
        uint32_t errs = g_error_count;
        uint32_t secs = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        snprintf(out, out_size, "ERR %lu  UP %02lu:%02lu:%02lu", (unsigned long)(errs > 9999 ? 9999 : errs),
                 (unsigned long)(secs / 3600 % 100), (unsigned long)(secs / 60 % 60), (unsigned long)(secs % 60));
        if (errs) *color = COLOR_ORANGE;
        break;
    }
    default: {
        uint32_t boot = flashlog_boot_number();
        if (boot) snprintf(out, out_size, "LOG #%lu  %u%% used", (unsigned long)boot, flashlog_used_percent());
        else snprintf(out, out_size, "LOG off");
        break;
    }
    }
}

// Redraw only the rows whose text changed
static void display_update_info(const display_view_t *v, uint32_t notify_rate)
{
    if (view_shows_debug(v)) {
        for (int i = 0; i < DEBUG_LINES; i++) {
            char text[24];
            uint16_t color;
            display_format_info(i, text, sizeof(text), &color, notify_rate);
            if (strcmp(text, info_cache[i]) != 0 || color != info_color_cache[i]) {
                display_draw_text(0, DEBUG_Y + i * DEBUG_LINE_H, LCD_H_RES, text, color, COLOR_BLACK, 1, true);
                strcpy(info_cache[i], text);
                info_color_cache[i] = color;
            }
        }
    }

    char rx_str[24], tx_str[24];
    snprintf(rx_str, sizeof(rx_str), "RX: %lu", (unsigned long)g_rx_count);
    snprintf(tx_str, sizeof(tx_str), "TX: %lu", (unsigned long)g_tx_count);

    if (strcmp(rx_str, rx_cache) != 0) {
        int scale = (strlen(rx_str) * 16 <= LCD_H_RES) ? 2 : 1;
        display_draw_text(0, COUNTERS_Y + (scale == 2 ? 0 : 4), LCD_H_RES, rx_str, COLOR_GREEN, COLOR_BLACK, scale, true);
        strcpy(rx_cache, rx_str);
    }
    if (strcmp(tx_str, tx_cache) != 0) {
        int scale = (strlen(tx_str) * 16 <= LCD_H_RES) ? 2 : 1;
        display_draw_text(0, COUNTERS_Y + 30 + (scale == 2 ? 0 : 4), LCD_H_RES, tx_str, COLOR_CYAN, COLOR_BLACK, scale, true);
        strcpy(tx_cache, tx_str);
    }
}

static void display_task(void *pvParameters)
{
    uint32_t last_rx = 0, last_tx = 0;
    uint32_t last_notify = 0, notify_rate = 0;
    int64_t last_rate_us = esp_timer_get_time();
    display_view_t current = view;

    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));

        bool dirty;
        taskENTER_CRITICAL(&state_lock);
            dirty = view_dirty;
            if (dirty) {
                current = view;
                view_dirty = false;
            }
        taskEXIT_CRITICAL(&state_lock);

        int64_t now = esp_timer_get_time();
        if (now - last_rate_us >= 1000000) {
            uint32_t cur_notify = g_notify_count;
            notify_rate = (uint32_t)((cur_notify - last_notify) * 1000000LL / (now - last_rate_us));
            last_notify = cur_notify;
            last_rate_us = now;
        }

        uint32_t cur_rx = g_rx_count, cur_tx = g_tx_count;
        bool traffic = (cur_rx != last_rx) || (cur_tx != last_tx);
        last_rx = cur_rx;
        last_tx = cur_tx;

        if (dirty || traffic) display_bump_timer();
        if (dirty) display_draw_view(&current);
        display_update_info(&current, notify_rate);
    }
}

// ---------------------------------------------------------------------------------------------
// Public API: these only record state and wake the display task
// ---------------------------------------------------------------------------------------------
static void display_wake(void)
{
    if (display_task_handle) xTaskNotifyGive(display_task_handle);
}

void display_update_traffic(uint32_t rx_count, uint32_t tx_count)
{
    // Counters are read directly by the display task; this only wakes it
    display_wake();
}

void display_set_mode_view(const char *mode_title, display_icon_t icon, const char *status_str, uint16_t state_color)
{
    taskENTER_CRITICAL(&state_lock);
        strlcpy(view.title, mode_title, sizeof(view.title));
        strlcpy(view.status, status_str, sizeof(view.status));
        view.icon = icon;
        view.color = state_color;
        view_dirty = true;
    taskEXIT_CRITICAL(&state_lock);
    display_wake();
}

void display_set_status(const char *transport, const char *status_msg, uint16_t color)
{
    display_icon_t icon = ICON_BLUETOOTH;
    const char *header_label = transport;
    
    if (strstr(transport, "SAVVY") != NULL || strstr(transport, "USB") != NULL) {
        icon = ICON_USB;
        header_label = "SAVVYCAN";
    } else if (strstr(transport, "SIMOS") != NULL || strstr(transport, "ISO-TP") != NULL || strstr(transport, "BLE") != NULL) {
        header_label = "SIMOS";
    }

    const char *short_status = status_msg;
    if (strstr(status_msg, "CONNECTED") != NULL) short_status = "CONNECTED";
    else if (strstr(status_msg, "WAITING") != NULL) short_status = "READY";
    else if (strstr(status_msg, "ERROR") != NULL) short_status = "ERROR";
    else if (strstr(status_msg, "DISCONNECTED") != NULL) short_status = "OFFLINE";
    else if (strstr(status_msg, "REBOOTING") != NULL) short_status = "REBOOT";

    display_set_mode_view(header_label, icon, short_status, color);
}

void display_set_ble_info(uint16_t mtu, uint16_t conn_int)
{
    info_mtu = mtu;
    info_conn_int = conn_int;
}

void display_set_persist(bool enabled)
{
    info_persist = enabled;
}

void display_set_link_latency(uint8_t link, uint32_t ms)
{
    if (link < 2) info_latency_ms[link] = ms;
}

void display_init(void)
{
    // 1. Enable LilyGo onboard power supply (GPIO 15)
    gpio_reset_pin(PIN_NUM_LCD_PWR);
    gpio_set_direction(PIN_NUM_LCD_PWR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_LCD_PWR, 1);

    // 2. Keep RD (GPIO 9) HIGH so ST7789 does not stay in read cycle
    gpio_reset_pin(PIN_NUM_LCD_RD);
    gpio_set_direction(PIN_NUM_LCD_RD, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_LCD_RD, 1);

    // 3. Configure Backlight (GPIO 38)
    gpio_reset_pin(PIN_NUM_BK_LIGHT);
    gpio_set_direction(PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BK_LIGHT, 1);
    display_is_on = true;

    // 4. Hardware Reset toggle on GPIO 5
    gpio_reset_pin(PIN_NUM_LCD_RST);
    gpio_set_direction(PIN_NUM_LCD_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_NUM_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 5. Configure Intel 8080 8-bit Parallel Bus
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .wr_gpio_num = PIN_NUM_LCD_WR,
        .data_gpio_nums = {
            PIN_NUM_LCD_D0,
            PIN_NUM_LCD_D1,
            PIN_NUM_LCD_D2,
            PIN_NUM_LCD_D3,
            PIN_NUM_LCD_D4,
            PIN_NUM_LCD_D5,
            PIN_NUM_LCD_D6,
            PIN_NUM_LCD_D7,
        },
        .bus_width = 8,
        .max_transfer_bytes = LCD_H_RES * 40 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    // 6. Configure IO handle
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = 10 * 1000 * 1000, // Stable 10 MHz pixel clock
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .swap_color_bytes = 0, // Codebase color definitions are pre-swapped
        },
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    // Signal when each pixel transfer has finished so the strip buffer can be reused safely
    tx_done_sem = xSemaphoreCreateBinary();
    esp_lcd_panel_io_callbacks_t io_callbacks = {
        .on_color_trans_done = display_on_color_trans_done,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &io_callbacks, NULL));

    // 7. Instantiate ST7789 panel
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));

    // ST7789 170x320 panel memory offset
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 35, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    display_timer = xTimerCreate("DispTimer", pdMS_TO_TICKS(5000), pdFALSE, NULL, display_timer_callback);
    if (display_timer) xTimerStart(display_timer, 0);

    // The task draws the initial view (view_dirty is already set)
    xTaskCreate(display_task, "Display", 3072, NULL, tskIDLE_PRIORITY + 1, &display_task_handle);
}
