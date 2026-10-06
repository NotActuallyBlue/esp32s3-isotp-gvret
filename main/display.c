#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "display.h"
#include "flashlog.h"
#include "font_aa.h"
#include "canstats.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"

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
static bool bench_mode = false;
static volatile bool prompt_hold = false;   // the button menu is on screen: nothing else may redraw until the dongle restarts

// Everything is drawn by display_task. Other tasks only set the state below, so no two tasks ever
// draw at once and the pixel strip is never overwritten while a transfer is still using it.
typedef struct {
    char            title[16];
    display_icon_t  icon;
    char            status[16];
    uint16_t        color;
    bool            prompt;     // header and status only (used for the button menu)
} display_view_t;

static display_view_t       view            = { "STANDBY", ICON_BLUETOOTH, "READY", COLOR_CYAN, false };
static bool                 view_dirty      = true;
static portMUX_TYPE         state_lock      = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t         display_task_handle = NULL;
static SemaphoreHandle_t    tx_done_sem     = NULL;

static volatile uint16_t    info_mtu        = 0;
static volatile uint16_t    info_conn_int   = 0;    // units of 1.25 ms
static volatile bool        info_persist    = false;
static volatile uint32_t    info_latency_ms[2] = { 0, 0 };

// Detail rows replace the normal rows when set (used by the Wi-Fi update screen)
#define MAX_DETAILS     16
typedef struct {
    char        label[16];      // a leading '#' makes the row a section header
    char        value[24];
    uint16_t    color;          // 0 = default value color
} detail_t;
static detail_t             details[MAX_DETAILS];
static int                  detail_count    = 0;
static char                 details_title[16] = "UPDATE";
static uint16_t             details_color   = 0;

// Pages cycled with a short press of the BOOT button
#define PAGE_COUNT      3
static volatile int         page            = 0;


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
static int strip_w, strip_h;

// RGB565 in the byte order the panel expects (same as the COLOR_* constants)
static uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return COLOR_RGB(r, g, b);
}

#define C_VALUE         rgb(235, 238, 245)
#define C_LABEL         rgb(170, 180, 200)
#define C_LINE          rgb(40, 48, 64)
#define C_LINK          rgb(0, 200, 255)
#define C_BUS           rgb(255, 70, 200)
#define C_SYSTEM        rgb(255, 180, 0)
#define C_GOOD          rgb(0, 235, 120)
#define C_WARN          rgb(255, 170, 0)
#define C_BAD           rgb(255, 60, 60)

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

// Compose a row in the strip, then push it as one transfer
static void strip_begin(int w, int h, uint16_t bg)
{
    strip_w = w;
    strip_h = h;
    for (int i = 0; i < w * h; i++) strip[i] = bg;
}

static void strip_fill(int x, int y, int w, int h, uint16_t color)
{
    for (int py = y; py < y + h && py < strip_h; py++) {
        if (py < 0) continue;
        for (int px = x; px < x + w && px < strip_w; px++) {
            if (px >= 0) strip[py * strip_w + px] = color;
        }
    }
}

// Text is anti-aliased: scale 1 uses the 8 x 13 font, scale 2 the 16 x 16 bold font. Every character is 8 * scale wide.
static int text_width(const char *str, int scale)
{
    return (int)strlen(str) * 8 * scale;
}

static int text_height(int scale)
{
    return scale >= 2 ? FONT_LARGE_H : FONT_SMALL_H;
}

// Mix a text color over what is already in the strip (both in panel byte order)
static uint16_t blend_pixel(uint16_t under, uint16_t over, uint8_t cover)
{
    uint16_t u = (uint16_t)((under >> 8) | (under << 8));
    uint16_t o = (uint16_t)((over >> 8) | (over << 8));
    int ur = (u >> 11) & 31, ug = (u >> 5) & 63, ub = u & 31;
    int orr = (o >> 11) & 31, og = (o >> 5) & 63, ob = o & 31;
    int r = (orr * cover + ur * (255 - cover)) / 255;
    int g = (og * cover + ug * (255 - cover)) / 255;
    int b = (ob * cover + ub * (255 - cover)) / 255;
    uint16_t c = (uint16_t)((r << 11) | (g << 5) | b);
    return (uint16_t)((c >> 8) | (c << 8));
}

static void strip_text(int x, int y, const char *str, uint16_t color, int scale)
{
    bool large = scale >= 2;
    int gw = large ? FONT_LARGE_W : FONT_SMALL_W;
    int gh = large ? FONT_LARGE_H : FONT_SMALL_H;
    for (int ci = 0; str[ci]; ci++) {
        char c = str[ci];
        if (c < 32 || c > 126) c = ' ';
        const uint8_t *glyph = large ? font_large[c - 32] : font_small[c - 32];
        for (int gy = 0; gy < gh; gy++) {
            int py = y + gy;
            if (py < 0 || py >= strip_h) continue;
            for (int gx = 0; gx < gw; gx++) {
                uint8_t cover = glyph[gy * gw + gx];
                int px = x + ci * gw + gx;
                if (!cover || px < 0 || px >= strip_w) continue;
                uint16_t *dst = &strip[py * strip_w + px];
                *dst = cover == 255 ? color : blend_pixel(*dst, color, cover);
            }
        }
    }
}

static void display_draw_text(int x0, int y, int box_w, const char *str, uint16_t color, uint16_t bg, int scale, bool center)
{
    int h = text_height(scale);
    if (scale < 1 || h > STRIP_ROWS || box_w <= 0 || x0 + box_w > LCD_H_RES) return;
    strip_begin(box_w, h, bg);
    int tw = text_width(str, scale);
    strip_text(center ? (box_w - tw) / 2 : 0, 0, str, color, scale);
    display_push(x0, y, box_w, h);
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

bool display_is_awake(void)
{
    return display_is_on;
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
//   header bar 0-35 | status 42 | rows from 68 (14 px pitch, sections 4 px apart) | RX/TX 262, 286
// ---------------------------------------------------------------------------------------------
#define HEADER_H        36
#define STATUS_Y        42
#define TOP_DIVIDER_Y   63
#define ROWS_Y          68
#define ROW_PITCH       14
#define SECTION_GAP     4
#define BOTTOM_DIVIDER_Y 258
#define COUNTERS_Y      262
#define MAX_ROWS        24

typedef enum { ROW_SECTION, ROW_DATA } row_kind_t;

typedef struct {
    row_kind_t  kind;
    const char *label;
    char        label_buf[16];  // storage for labels that are built at run time
    char        value[24];
    uint16_t    color;          // section accent or value color
} row_t;

static char row_key[MAX_ROWS][64];
static char rx_key[32];
static char tx_key[32];

static bool view_shows_simos(const display_view_t *v)
{
    return strcmp(v->title, "SIMOS") == 0 || strcmp(v->title, "BENCH SIM") == 0;
}

static void add_section(row_t *rows, int *n, const char *label, uint16_t accent)
{
    rows[*n].kind = ROW_SECTION;
    rows[*n].label = label;
    rows[*n].value[0] = 0;
    rows[*n].color = accent;
    (*n)++;
}

static row_t *add_data(row_t *rows, int *n, const char *label)
{
    row_t *r = &rows[(*n)++];
    r->kind = ROW_DATA;
    r->label = label;
    r->value[0] = 0;
    r->color = C_VALUE;
    return r;
}

static row_t *add_data_label(row_t *rows, int *n, const char *label)
{
    row_t *r = add_data(rows, n, "");
    strlcpy(r->label_buf, label, sizeof(r->label_buf));
    r->label = r->label_buf;
    return r;
}

static uint16_t latency_color(uint32_t ms)
{
    return ms < 60 ? C_GOOD : (ms < 150 ? C_WARN : C_BAD);
}

static void format_latency(row_t *r, uint32_t ms)
{
    if (ms) {
        snprintf(r->value, sizeof(r->value), "%lu ms", (unsigned long)(ms > 999 ? 999 : ms));
        r->color = latency_color(ms);
    } else {
        snprintf(r->value, sizeof(r->value), "--");
        r->color = C_LABEL;
    }
}

static void format_can(row_t *r)
{
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) {
        snprintf(r->value, sizeof(r->value), "--");
        r->color = C_LABEL;
        return;
    }

    const char *state = st.state == TWAI_STATE_RUNNING ? "OK" :
                        st.state == TWAI_STATE_BUS_OFF ? "BUS OFF" :
                        st.state == TWAI_STATE_RECOVERING ? "RECOVER" : "STOPPED";
    snprintf(r->value, sizeof(r->value), "%s T%lu R%lu", state, (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter);
    if (st.state != TWAI_STATE_RUNNING) r->color = C_BAD;
    else if (st.tx_error_counter || st.rx_error_counter) r->color = C_WARN;
    else r->color = C_GOOD;
}

static int build_rows_status(const display_view_t *v, row_t *rows, uint32_t notify_rate)
{
    int n = 0;
    row_t *r;

    if (view_shows_simos(v)) {
        add_section(rows, &n, "LINK", C_LINK);
        uint16_t mtu = info_mtu, ci = info_conn_int;
        r = add_data(rows, &n, "MTU");
        if (mtu) snprintf(r->value, sizeof(r->value), "%u", mtu); else { snprintf(r->value, sizeof(r->value), "no link"); r->color = C_LABEL; }
        r = add_data(rows, &n, "INTERVAL");
        if (ci) {
            uint32_t x100 = (uint32_t)ci * 125;
            snprintf(r->value, sizeof(r->value), "%lu.%lu ms", (unsigned long)(x100 / 100), (unsigned long)((x100 % 100) / 10));
            r->color = ci <= 24 ? C_GOOD : C_WARN;
        } else { snprintf(r->value, sizeof(r->value), "--"); r->color = C_LABEL; }
        r = add_data(rows, &n, "STREAM");
        if (info_persist) { snprintf(r->value, sizeof(r->value), "ON  %lu/s", (unsigned long)notify_rate); r->color = C_GOOD; }
        else { snprintf(r->value, sizeof(r->value), "OFF"); r->color = C_LABEL; }
    }

    add_section(rows, &n, "BUS", C_BUS);
    format_can(add_data(rows, &n, "CAN"));
    if (view_shows_simos(v)) {
        format_latency(add_data(rows, &n, "ECU"), info_latency_ms[0]);
        format_latency(add_data(rows, &n, "TCU"), info_latency_ms[1]);
        r = add_data(rows, &n, "ISO-TP ERR");
        uint32_t errs = g_error_count;
        snprintf(r->value, sizeof(r->value), "%lu", (unsigned long)(errs > 9999 ? 9999 : errs));
        r->color = errs ? C_WARN : C_GOOD;
    }

    add_section(rows, &n, "SYSTEM", C_SYSTEM);
    uint32_t secs = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    r = add_data(rows, &n, "UPTIME");
    snprintf(r->value, sizeof(r->value), "%02lu:%02lu:%02lu", (unsigned long)(secs / 3600 % 100), (unsigned long)(secs / 60 % 60), (unsigned long)(secs % 60));
    r = add_data(rows, &n, "HEAP");
    uint32_t free_kb = esp_get_free_heap_size() / 1024, min_kb = esp_get_minimum_free_heap_size() / 1024;
    snprintf(r->value, sizeof(r->value), "%lu KB", (unsigned long)free_kb);
    r->color = min_kb < 20 ? C_WARN : C_VALUE;
    if (view_shows_simos(v)) {
        r = add_data(rows, &n, "FLASH LOG");
        uint32_t boot = flashlog_boot_number();
        if (boot) {
            uint8_t pct = flashlog_used_percent();
            snprintf(r->value, sizeof(r->value), "#%lu  %u%%", (unsigned long)boot, pct);
            r->color = pct < 70 ? C_GOOD : (pct < 90 ? C_WARN : C_BAD);
        } else { snprintf(r->value, sizeof(r->value), "off"); r->color = C_LABEL; }
    }
    return n;
}

static int build_rows_bus(row_t *rows)
{
    static canstats_totals_t prev;
    static int64_t prev_us;
    static uint32_t fps, load_x10;

    canstats_totals_t t;
    canstats_get_totals(&t);
    int64_t now = esp_timer_get_time();
    if (now - prev_us >= 1000000) {
        int64_t dt = now - prev_us;
        fps = (uint32_t)((int64_t)(t.frames - prev.frames) * 1000000 / dt);
        load_x10 = (uint32_t)((int64_t)(t.bits - prev.bits) * 1000000 / dt / 500);     // 500 kbit/s bus, tenths of a percent
        prev = t;
        prev_us = now;
    }

    int n = 0;
    row_t *r;
    add_section(rows, &n, "TRAFFIC", C_BUS);
    r = add_data(rows, &n, "FRAMES/s");
    snprintf(r->value, sizeof(r->value), "%lu", (unsigned long)fps);
    r = add_data(rows, &n, "BUS LOAD");
    snprintf(r->value, sizeof(r->value), "~%lu.%lu %%", (unsigned long)(load_x10 / 10), (unsigned long)(load_x10 % 10));
    r->color = load_x10 < 400 ? C_GOOD : (load_x10 < 700 ? C_WARN : C_BAD);
    r = add_data(rows, &n, "IDS SEEN");
    snprintf(r->value, sizeof(r->value), "%lu", (unsigned long)t.unique_ids);
    r = add_data(rows, &n, "FRAMES");
    snprintf(r->value, sizeof(r->value), "%lu", (unsigned long)t.frames);

    uint32_t ids[6], counts[6];
    int top = canstats_top_ids(ids, counts, 6);
    if (top) add_section(rows, &n, "BUSIEST IDS", C_LINK);
    for (int i = 0; i < top; i++) {
        char label[16];
        snprintf(label, sizeof(label), "0x%03lX", (unsigned long)ids[i]);
        r = add_data_label(rows, &n, label);
        snprintf(r->value, sizeof(r->value), "%lu", (unsigned long)counts[i]);
    }

    add_section(rows, &n, "FAULTS", C_SYSTEM);
    format_can(add_data(rows, &n, "CAN"));
    r = add_data(rows, &n, "TX FAILS");
    snprintf(r->value, sizeof(r->value), "%lu", (unsigned long)t.tx_failures);
    r->color = t.tx_failures ? C_WARN : C_GOOD;
    r = add_data(rows, &n, "WARN/PASS/OFF");
    snprintf(r->value, sizeof(r->value), "%lu/%lu/%lu", (unsigned long)t.events[0], (unsigned long)t.events[1], (unsigned long)t.events[2]);
    r->color = (t.events[1] || t.events[2]) ? C_WARN : C_VALUE;
    return n;
}

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON:   return "POWER ON";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT WDT";
    case ESP_RST_TASK_WDT:  return "TASK WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEP SLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_USB:       return "USB";
    default:                return "OTHER";
    }
}

static int build_rows_info(row_t *rows)
{
    int n = 0;
    row_t *r;
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();

    add_section(rows, &n, "FIRMWARE", C_LINK);
    r = add_data(rows, &n, "BUILD");
    snprintf(r->value, sizeof(r->value), "%.16s", app->date);
    r = add_data(rows, &n, "TIME");
    snprintf(r->value, sizeof(r->value), "%.16s", app->time);
    r = add_data(rows, &n, "IDF");
    snprintf(r->value, sizeof(r->value), "%.16s", app->idf_ver);
    r = add_data(rows, &n, "SLOT");
    snprintf(r->value, sizeof(r->value), "%.16s", running ? running->label : "?");

    add_section(rows, &n, "BOOT", C_BUS);
    esp_reset_reason_t reason = esp_reset_reason();
    r = add_data(rows, &n, "RESET");
    snprintf(r->value, sizeof(r->value), "%s", reset_reason_name(reason));
    r->color = (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT || reason == ESP_RST_BROWNOUT) ? C_BAD : C_VALUE;
    r = add_data(rows, &n, "FLASH");
    uint32_t flash_size = 0;
    esp_flash_get_physical_size(NULL, &flash_size);
    snprintf(r->value, sizeof(r->value), "%lu MB", (unsigned long)(flash_size >> 20));
    r = add_data(rows, &n, "FLASH LOG");
    uint32_t boot = flashlog_boot_number();
    if (boot) snprintf(r->value, sizeof(r->value), "#%lu  %u%%", (unsigned long)boot, flashlog_used_percent());
    else snprintf(r->value, sizeof(r->value), "off");

    add_section(rows, &n, "MEMORY", C_SYSTEM);
    r = add_data(rows, &n, "HEAP FREE");
    snprintf(r->value, sizeof(r->value), "%lu KB", (unsigned long)(esp_get_free_heap_size() / 1024));
    r = add_data(rows, &n, "HEAP MIN");
    uint32_t min_kb = esp_get_minimum_free_heap_size() / 1024;
    snprintf(r->value, sizeof(r->value), "%lu KB", (unsigned long)min_kb);
    r->color = min_kb < 20 ? C_WARN : C_VALUE;
    r = add_data(rows, &n, "LARGEST BLOCK");
    snprintf(r->value, sizeof(r->value), "%lu KB", (unsigned long)(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT) / 1024));
    return n;
}

static int build_rows_detail(row_t *rows)
{
    int n = 0;
    detail_t local[MAX_DETAILS];
    char title[sizeof(details_title)];
    uint16_t title_color;
    int count;

    taskENTER_CRITICAL(&state_lock);
        count = detail_count;
        memcpy(local, details, sizeof(local));
        strlcpy(title, details_title, sizeof(title));
        title_color = details_color;
    taskEXIT_CRITICAL(&state_lock);

    add_section(rows, &n, title, title_color ? title_color : C_SYSTEM);
    for (int i = 0; i < count && i < MAX_DETAILS; i++) {
        if (local[i].label[0] == '#') {
            row_t *s = &rows[n++];
            s->kind = ROW_SECTION;
            strlcpy(s->label_buf, local[i].label + 1, sizeof(s->label_buf));
            s->label = s->label_buf;
            s->value[0] = 0;
            s->color = local[i].color ? local[i].color : C_LINK;
            continue;
        }
        row_t *r = add_data_label(rows, &n, local[i].label);
        strlcpy(r->value, local[i].value, sizeof(r->value));
        if (local[i].color) r->color = local[i].color;
    }
    return n;
}

static int build_rows(const display_view_t *v, row_t *rows, uint32_t notify_rate)
{
    if (detail_count > 0) return build_rows_detail(rows);
    switch (page) {
    case 1:  return build_rows_bus(rows);
    case 2:  return build_rows_info(rows);
    default: return build_rows_status(v, rows, notify_rate);
    }
}

static void draw_row(const row_t *r, int y)
{
    strip_begin(LCD_H_RES, FONT_SMALL_H, COLOR_BLACK);
    if (r->kind == ROW_SECTION) {
        int tw = text_width(r->label, 1);
        strip_fill(6, 1, 3, FONT_SMALL_H - 2, r->color);
        strip_text(14, 0, r->label, r->color, 1);
        strip_fill(14 + tw + 6, 6, LCD_H_RES - (14 + tw + 6) - 8, 1, C_LINE);
    } else {
        strip_text(8, 0, r->label, C_LABEL, 1);
        strip_text(LCD_H_RES - 8 - text_width(r->value, 1), 0, r->value, r->color, 1);
    }
    display_push(0, y, LCD_H_RES, FONT_SMALL_H);
}

static void draw_counter(int y, const char *label, unsigned long value, uint16_t color)
{
    char num[16];
    snprintf(num, sizeof(num), "%lu", value);
    int scale = (text_width(num, 2) + text_width(label, 2) + 24 <= LCD_H_RES) ? 2 : 1;

    strip_begin(LCD_H_RES, 16, COLOR_BLACK);
    strip_text(8, 0, label, C_LABEL, 2);
    strip_text(LCD_H_RES - 8 - text_width(num, scale), (16 - text_height(scale)) / 2, num, color, scale);
    display_push(0, y, LCD_H_RES, 16);
}

static bool display_shows_counters(const display_view_t *v)
{
    return !v->prompt && detail_count == 0 && page == 0;
}

static void display_draw_view(const display_view_t *v)
{
    display_clear(COLOR_BLACK);

    // Header bar: icon and mode name on the state color
    display_fill_rect(0, 0, LCD_H_RES, HEADER_H, v->color);
    if (v->icon == ICON_BLUETOOTH || v->icon == ICON_OBD) {
        display_draw_icon(4, 2, icon_bluetooth_32x32, COLOR_BLACK, v->color, 1);
    } else if (v->icon == ICON_USB) {
        display_draw_icon(4, 2, icon_usb_32x32, COLOR_BLACK, v->color, 1);
    }
    int title_x = (v->icon == ICON_NONE) ? 8 : 42;
    int title_scale = (text_width(v->title, 2) <= LCD_H_RES - title_x - 4) ? 2 : 1;
    display_draw_text(title_x, (HEADER_H - text_height(title_scale)) / 2, LCD_H_RES - title_x - 4, v->title, COLOR_BLACK, v->color, title_scale, false);

    // Status line in the state color
    int status_scale = (text_width(v->status, 2) <= LCD_H_RES - 8) ? 2 : 1;
    display_draw_text(0, STATUS_Y + (status_scale == 2 ? 0 : 2), LCD_H_RES, v->status, v->color, COLOR_BLACK, status_scale, true);

    // Force every row to redraw
    memset(row_key, 0, sizeof(row_key));
    memset(rx_key, 0, sizeof(rx_key));
    memset(tx_key, 0, sizeof(tx_key));

    if (v->prompt) return;

    display_fill_rect(10, TOP_DIVIDER_Y, LCD_H_RES - 20, 1, C_LINE);
    if (display_shows_counters(v)) {
        display_fill_rect(10, BOTTOM_DIVIDER_Y, LCD_H_RES - 20, 1, C_LINE);
    }

    // Page indicator: one square per page, the current one in the state color
    if (detail_count == 0) {
        strip_begin(LCD_H_RES, 6, COLOR_BLACK);
        int x0 = (LCD_H_RES - (PAGE_COUNT * 6 + (PAGE_COUNT - 1) * 6)) / 2;
        for (int i = 0; i < PAGE_COUNT; i++) {
            strip_fill(x0 + i * 12, 0, 6, 6, i == page ? v->color : C_LINE);
        }
        display_push(0, LCD_V_RES - 10, LCD_H_RES, 6);
    }
}

// Redraw only the rows whose text or color changed. If the rows moved (a page with a different number of section
// headers has different spacing), the whole row area is wiped and redrawn so no old line is left behind.
static void display_update_info(const display_view_t *v, uint32_t notify_rate)
{
    if (v->prompt) return;

    row_t rows[MAX_ROWS];
    int n = build_rows(v, rows, notify_rate);

    static int prev_y[MAX_ROWS];
    static int prev_n = -1;
    int ys[MAX_ROWS];
    int y = ROWS_Y;
    bool moved = n != prev_n;
    for (int i = 0; i < n; i++) {
        if (rows[i].kind == ROW_SECTION && i > 0) y += SECTION_GAP;
        ys[i] = y;
        if (i >= prev_n || ys[i] != prev_y[i]) moved = true;
        if (ys[i] + FONT_SMALL_H > LCD_V_RES) { n = i; break; }
        y += ROW_PITCH;
    }
    if (moved) {
        int bottom = display_shows_counters(v) ? BOTTOM_DIVIDER_Y : LCD_V_RES - 4;
        display_fill_rect(0, ROWS_Y, LCD_H_RES, bottom - ROWS_Y, COLOR_BLACK);
        memset(row_key, 0, sizeof(row_key));
        memcpy(prev_y, ys, sizeof(int) * n);
        prev_n = n;
    }

    for (int i = 0; i < n; i++) {
        char key[64];
        snprintf(key, sizeof(key), "%d|%s|%s|%04X", rows[i].kind, rows[i].label, rows[i].value, rows[i].color);
        if (strcmp(key, row_key[i]) != 0) {
            draw_row(&rows[i], ys[i]);
            strlcpy(row_key[i], key, sizeof(row_key[i]));
        }
    }

    if (!display_shows_counters(v)) return;

    unsigned long rx = g_rx_count, tx = g_tx_count;
    char key[32];
    snprintf(key, sizeof(key), "%lu", rx);
    if (strcmp(key, rx_key) != 0) {
        draw_counter(COUNTERS_Y, "RX", rx, COLOR_CYAN);
        strlcpy(rx_key, key, sizeof(rx_key));
    }
    snprintf(key, sizeof(key), "%lu", tx);
    if (strcmp(key, tx_key) != 0) {
        draw_counter(COUNTERS_Y + 24, "TX", tx, COLOR_RGB(190, 160, 255));
        strlcpy(tx_key, key, sizeof(tx_key));
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
    if (prompt_hold) return;       // the running mode refreshes its status every few hundred ms; do not overwrite the menu
    taskENTER_CRITICAL(&state_lock);
        // Repaint only when something shown actually changed, so a caller may set the same status repeatedly
        bool changed = strncmp(view.title, mode_title, sizeof(view.title) - 1) != 0 ||
                       strncmp(view.status, status_str, sizeof(view.status) - 1) != 0 ||
                       view.icon != icon || view.color != state_color || view.prompt;
        strlcpy(view.title, mode_title, sizeof(view.title));
        strlcpy(view.status, status_str, sizeof(view.status));
        view.icon = icon;
        view.color = state_color;
        view.prompt = false;
        if (changed) view_dirty = true;
    taskEXIT_CRITICAL(&state_lock);
    display_wake();
}

void display_set_status(const char *transport, const char *status_msg, uint16_t color)
{
    display_icon_t icon = ICON_BLUETOOTH;
    const char *header_label = transport;
    
    if (strstr(transport, "SAVVY") != NULL || strstr(transport, "USB") != NULL) {
        icon = ICON_NONE;
        header_label = bench_mode ? "BENCH SAVVY" : "SAVVYCAN";
    } else if (strstr(transport, "BENCH") != NULL) {
        header_label = "BENCH SIM";
    } else if (strstr(transport, "UPDATE") != NULL) {
        icon = ICON_NONE;
        header_label = "WIFI UPDATE";
    } else if (strstr(transport, "DIAG") != NULL) {
        icon = ICON_NONE;
        header_label = bench_mode ? "BENCH DIAG" : "DIAG";
    } else if (strstr(transport, "ELM") != NULL) {
        header_label = bench_mode ? "BENCH ELM327" : "ELM327";
    } else if (strstr(transport, "SIMOS") != NULL || strstr(transport, "ISO-TP") != NULL || strstr(transport, "BLE") != NULL) {
        header_label = bench_mode ? "BENCH SIM" : "SIMOS";
    }

    const char *short_status = status_msg;
    if (strstr(status_msg, "CONNECTED") != NULL) short_status = "CONNECTED";
    else if (strstr(status_msg, "WAITING") != NULL) short_status = "READY";
    else if (strstr(status_msg, "ERROR") != NULL) short_status = "ERROR";
    else if (strstr(status_msg, "DISCONNECTED") != NULL) short_status = "OFFLINE";
    else if (strstr(status_msg, "REBOOTING") != NULL) short_status = "REBOOT";

    display_set_mode_view(header_label, icon, short_status, color);
}

void display_set_prompt(const char *title, const char *status, uint16_t color)
{
    taskENTER_CRITICAL(&state_lock);
        bool changed = !view.prompt || strncmp(view.title, title, sizeof(view.title) - 1) != 0 ||
                       strncmp(view.status, status, sizeof(view.status) - 1) != 0 || view.color != color;
        strlcpy(view.title, title, sizeof(view.title));
        strlcpy(view.status, status, sizeof(view.status));
        view.icon = ICON_NONE;
        view.color = color;
        view.prompt = true;
        prompt_hold = true;
        if (changed) view_dirty = true;
    taskEXIT_CRITICAL(&state_lock);
    display_wake();
}

void display_next_page(void)
{
    page = (page + 1) % PAGE_COUNT;
    taskENTER_CRITICAL(&state_lock);
        view_dirty = true;
    taskEXIT_CRITICAL(&state_lock);
    display_wake();
}

void display_set_detail(uint8_t index, const char *label, const char *value)
{
    if (index >= MAX_DETAILS) return;
    taskENTER_CRITICAL(&state_lock);
        strlcpy(details[index].label, label, sizeof(details[index].label));
        strlcpy(details[index].value, value, sizeof(details[index].value));
        details[index].color = 0;
        if (index >= detail_count) {
            detail_count = index + 1;
            if (!prompt_hold) view_dirty = true;      // the layout changed
        }
    taskEXIT_CRITICAL(&state_lock);
    display_wake();
}

void display_set_details(const char *title, uint16_t title_color, const display_detail_t *rows, int count)
{
    if (count > MAX_DETAILS) count = MAX_DETAILS;
    if (count < 1) {
        display_clear_details();
        return;
    }
    taskENTER_CRITICAL(&state_lock);
        // Only a change in the number of rows needs the whole screen redrawn; otherwise just the rows that differ
        if (!prompt_hold && (count != detail_count || strcmp(title, details_title) != 0)) view_dirty = true;
        strlcpy(details_title, title, sizeof(details_title));
        details_color = title_color;
        for (int i = 0; i < count; i++) {
            strlcpy(details[i].label, rows[i].label, sizeof(details[i].label));
            strlcpy(details[i].value, rows[i].value ? rows[i].value : "", sizeof(details[i].value));
            details[i].color = rows[i].color;
        }
        detail_count = count;
    taskEXIT_CRITICAL(&state_lock);
    display_wake();
}

void display_clear_details(void)
{
    taskENTER_CRITICAL(&state_lock);
        detail_count = 0;
        strlcpy(details_title, "UPDATE", sizeof(details_title));
        details_color = 0;
        if (!prompt_hold) view_dirty = true;
    taskEXIT_CRITICAL(&state_lock);
    display_wake();
}

void display_set_bench(bool bench)
{
    bench_mode = bench;
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
    xTaskCreate(display_task, "Display", 6144, NULL, tskIDLE_PRIORITY + 1, &display_task_handle);
}
