#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "constants.h"
#include "display.h"
#include "mode_mgr.h"
#include "diag_can.h"
#include "obd_codec.h"
#include "power_mgr.h"
#include "canstats.h"
#include "diag.h"
#include "stackwatch.h"

#define DIAG_TAG            "Diag"
#define KEY_PIN             14                  // second button on the T-Display-S3
#define KEY_SHORT_MAX_MS    1000
#define CONFIRM_HOLD_MS     2000
#define MAX_MODULES         16
#define MAX_MODULE_DTCS     32
#define SCAN_TIMEOUT_MS     50
#define LIVE_PID_COUNT      8

typedef struct {
    uint32_t    tx;
    uint32_t    rx;
    char        name[12];
    bool        uds;                            // answered UDS 0x19
    bool        denied;                         // answered with a negative response other than "not supported"
    bool        pending_listed;                 // OBD-only module: pending codes (mode 07) were read as well
    uint8_t     dtc_count;
    obd_dtc_t   dtcs[MAX_MODULE_DTCS];
    char        clear_result[12];
} module_t;

typedef enum { UI_MENU, UI_RESULTS, UI_CONFIRM, UI_LIVE, UI_INFO, UI_NOTICE } ui_state_t;

static const char *const menu_items[] = { "SCAN CODES", "CLEAR CODES", "LIVE DATA", "VEHICLE INFO" };
#define MENU_COUNT      4

static module_t         modules[MAX_MODULES];
static int              module_count;
static bool             scanned;
static diag_response_t  resp[DIAG_MAX_RESPONSES];
static bool             bench;
static uint32_t         scan_frames;            // frames seen on the bus during the last scan

static ui_state_t       ui = UI_MENU;
static int              menu_cursor;
static int              page;
static volatile bool    ev_next;
static volatile bool    ev_key;
static int64_t          key_down_since;
static bool             key_was_down;

// ---------------------------------------------------------------- screen helpers
typedef struct {
    char        label[16];
    char        value[24];
    uint16_t    color;
} row_buf_t;

static row_buf_t    rows[16];
static int          row_count;

static void rows_reset(void) { row_count = 0; }

static void row(const char *label, const char *value, uint16_t color)
{
    if (row_count >= 16) return;
    strlcpy(rows[row_count].label, label, sizeof(rows[row_count].label));
    strlcpy(rows[row_count].value, value ? value : "", sizeof(rows[row_count].value));
    rows[row_count].color = color;
    row_count++;
}

static void rowf(const char *label, uint16_t color, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void rowf(const char *label, uint16_t color, const char *fmt, ...)
{
    char value[24];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(value, sizeof(value), fmt, ap);
    va_end(ap);
    row(label, value, color);
}

static void show(const char *title, uint16_t title_color)
{
    display_detail_t d[16];
    for (int i = 0; i < row_count; i++) {
        d[i].label = rows[i].label;
        d[i].value = rows[i].value;
        d[i].color = rows[i].color;
    }
    display_set_details(title, title_color, d, row_count);
}

static void status(const char *text, uint16_t color)
{
    display_set_status("DIAG", text, color);
}

// ---------------------------------------------------------------- buttons
static void on_boot_tap(void)
{
    ev_next = true;
    power_mgr_poke();
}

static void poll_key(void)
{
    bool down = gpio_get_level(KEY_PIN) == 0;
    int64_t now = esp_timer_get_time() / 1000;
    if (down && !key_was_down) {
        key_down_since = now;
        display_power(true);
        power_mgr_poke();
    } else if (!down && key_was_down) {
        int64_t held = now - key_down_since;
        if (held >= 40 && held < KEY_SHORT_MAX_MS) ev_key = true;
    }
    key_was_down = down;
}

static int key_held_ms(void)
{
    return key_was_down ? (int)(esp_timer_get_time() / 1000 - key_down_since) : 0;
}

static bool take_key(void)
{
    bool e = ev_key;
    ev_key = false;
    return e;
}

static bool take_next(void)
{
    bool e = ev_next;
    ev_next = false;
    return e;
}

// ---------------------------------------------------------------- vehicle access
static const char *module_name(uint32_t tx, char *buf, size_t size)
{
    if (tx == 0x7E0)      snprintf(buf, size, "ENGINE");
    else if (tx == 0x7E1) snprintf(buf, size, "TRANS");
    else                  snprintf(buf, size, "MOD %03lX", (unsigned long)tx);
    return buf;
}

static bool is_obd_physical(uint32_t tx)
{
    return tx >= 0x7E0 && tx <= 0x7E7;
}

static int request(uint32_t tx, const uint8_t *req, uint16_t len, uint32_t timeout_ms)
{
    return diag_can_obd(tx, req, len, resp, DIAG_MAX_RESPONSES, timeout_ms);
}

static void log_dtcs(const module_t *m)
{
    for (int i = 0; i < m->dtc_count; i++) {
        char code[6];
        obd_format_dtc(m->dtcs[i].code, code);
        uint16_t vag = obd_vag_fault_number(m->dtcs[i].code);
        if (m->dtcs[i].has_status) {
            ESP_LOGI(DIAG_TAG, "  %s %s: %s-%02X status 0x%02X (VAG %u)", m->name, "UDS", code, m->dtcs[i].fault_type, m->dtcs[i].status, vag);
        } else {
            ESP_LOGI(DIAG_TAG, "  %s %s: %s (VAG %u)", m->name, "OBD", code, vag);
        }
    }
}

// Log only: what the module says through the OBD-II services, to compare with the UDS list. Mode 04 can only act on
// what shows up here.
static void log_obd_view(const module_t *m)
{
    static const uint8_t m03 = 0x03, m07 = 0x07, m0a = 0x0A;
    static const uint8_t pid01[] = { 0x01, 0x01 };
    obd_dtc_t tmp[MAX_MODULE_DTCS];
    int stored = -1, pending = -1, permanent = -1;

    int n = request(m->tx, &m03, 1, 100);
    if (n > 0 && resp[0].data[0] == 0x43) stored = obd_parse_mode_dtcs(resp[0].data, resp[0].len, tmp, MAX_MODULE_DTCS);
    n = request(m->tx, &m07, 1, 100);
    if (n > 0 && resp[0].data[0] == 0x47) pending = obd_parse_mode_dtcs(resp[0].data, resp[0].len, tmp, MAX_MODULE_DTCS);
    n = request(m->tx, &m0a, 1, 100);
    if (n > 0 && resp[0].data[0] == 0x4A) permanent = obd_parse_mode_dtcs(resp[0].data, resp[0].len, tmp, MAX_MODULE_DTCS);
    n = request(m->tx, pid01, sizeof(pid01), 100);
    int mil = -1, count = -1;
    if (n > 0 && resp[0].len >= 6 && resp[0].data[0] == 0x41) { mil = resp[0].data[2] >> 7; count = resp[0].data[2] & 0x7F; }

    ESP_LOGI(DIAG_TAG, "  %s through OBD-II: stored %d, pending %d, permanent %d (-1 = no answer), MIL %d, confirmed count %d",
             m->name, stored, pending, permanent, mil, count);
}

// Log only: the exact bytes a module answered, to see how its fault records are laid out
static void log_raw(const char *name, const char *what, const diag_response_t *r)
{
    char text[3 * 64 + 8];
    int n = r->len > 64 ? 64 : r->len;
    for (int i = 0; i < n; i++) snprintf(text + i * 3, sizeof(text) - i * 3, "%02X ", r->data[i]);
    text[n * 3] = 0;
    ESP_LOGI(DIAG_TAG, "  %s %s raw (%u bytes): %s%s", name, what, (unsigned)r->len, text, r->len > 64 ? "..." : "");
}

// Log only: ask the module for every stored entry whatever its status, and log what comes back. Shows entries the fault
// filter hides, and why a module said "no access".
static void log_unfiltered(const module_t *m)
{
    static const uint8_t read_all[] = { 0x19, 0x02, 0xFF };
    int n = request(m->tx, read_all, sizeof(read_all), SCAN_TIMEOUT_MS);
    if (n <= 0) {
        ESP_LOGI(DIAG_TAG, "  %s unfiltered (19 02 FF): no answer", m->name);
        return;
    }
    const diag_response_t *r = &resp[0];
    log_raw(m->name, "unfiltered", r);
    if (r->data[0] != 0x59) {
        ESP_LOGI(DIAG_TAG, "  %s unfiltered (19 02 FF): service 0x%02X, byte 2 0x%02X, byte 3 0x%02X", m->name, r->data[0],
                 r->len > 1 ? r->data[1] : 0, r->len > 2 ? r->data[2] : 0);
        return;
    }
    obd_dtc_t all[MAX_MODULE_DTCS];
    int count = obd_parse_uds_dtcs(r->data, r->len, all, MAX_MODULE_DTCS);
    ESP_LOGI(DIAG_TAG, "  %s unfiltered (19 02 FF): %d entr%s", m->name, count, count == 1 ? "y" : "ies");
    for (int i = 0; i < count && i < 12; i++) {
        char code[6];
        obd_format_dtc(all[i].code, code);
        ESP_LOGI(DIAG_TAG, "    %s-%02X status 0x%02X (VAG %u)", code, all[i].fault_type, all[i].status, obd_vag_fault_number(all[i].code));
    }
}

// Read trouble codes of one module that answered. Returns false if it did not answer at all.
static bool read_module(module_t *m)
{
    static const uint8_t read_uds[] = { 0x19, 0x02, DTC_STATUS_FAULT_MASK };
    int n = request(m->tx, read_uds, sizeof(read_uds), SCAN_TIMEOUT_MS);
    if (n <= 0) return false;

    const diag_response_t *r = &resp[0];
    m->rx = r->id;
    log_raw(m->name, "fault read", r);
    if (r->data[0] == 0x59) {
        m->uds = true;
        m->dtc_count = (uint8_t)obd_parse_uds_dtcs(r->data, r->len, m->dtcs, MAX_MODULE_DTCS);
        m->dtc_count = (uint8_t)obd_keep_faults(m->dtcs, m->dtc_count);      // some modules ignore the mask
        return true;
    }

    bool not_supported = r->len >= 3 && r->data[0] == 0x7F && (r->data[2] == 0x11 || r->data[2] == 0x12 || r->data[2] == 0x7F);
    if (is_obd_physical(m->tx) && (not_supported || r->data[0] != 0x7F)) {
        static const uint8_t mode03 = 0x03, mode07 = 0x07;
        n = request(m->tx, &mode03, 1, 100);
        if (n > 0 && resp[0].data[0] == 0x43) {
            m->dtc_count = (uint8_t)obd_parse_mode_dtcs(resp[0].data, resp[0].len, m->dtcs, MAX_MODULE_DTCS);
        }
        n = request(m->tx, &mode07, 1, 100);
        if (n > 0 && resp[0].data[0] == 0x47) {
            obd_dtc_t pend[MAX_MODULE_DTCS];
            int p = obd_parse_mode_dtcs(resp[0].data, resp[0].len, pend, MAX_MODULE_DTCS);
            for (int i = 0; i < p && m->dtc_count < MAX_MODULE_DTCS; i++) {
                pend[i].status = DTC_STATUS_PENDING;
                pend[i].has_status = true;
                m->dtcs[m->dtc_count++] = pend[i];
            }
            m->pending_listed = true;
        }
        return true;
    }

    // Some modules refuse the fault-status mask but answer a request for every entry
    static const uint8_t read_all[] = { 0x19, 0x02, 0xFF };
    n = request(m->tx, read_all, sizeof(read_all), SCAN_TIMEOUT_MS);
    if (n > 0 && resp[0].data[0] == 0x59) {
        m->uds = true;
        m->dtc_count = (uint8_t)obd_parse_uds_dtcs(resp[0].data, resp[0].len, m->dtcs, MAX_MODULE_DTCS);
        m->dtc_count = (uint8_t)obd_keep_faults(m->dtcs, m->dtc_count);
        return true;
    }

    m->denied = true;                                   // present, but will not tell us (needs a session or login)
    return true;
}

// What the CAN controller reports: an unacknowledged transmitter (nobody awake on the bus) ends up error passive
static const char *can_state_text(uint16_t *color)
{
    twai_status_info_t st;
    *color = COLOR_WHITE;
    if (bench || twai_get_status_info(&st) != ESP_OK) return "simulated";
    if (st.state == TWAI_STATE_BUS_OFF || st.state == TWAI_STATE_RECOVERING) { *color = COLOR_RED; return "BUS OFF"; }
    if (st.tx_error_counter >= 128 || st.rx_error_counter >= 128) { *color = COLOR_ORANGE; return "ERROR PASSIVE"; }
    if (st.tx_error_counter || st.rx_error_counter) { *color = COLOR_YELLOW; return "some errors"; }
    *color = COLOR_GREEN;
    return "OK";
}

// An entry that only says "failed at some point since the last clear" (0x20) is history: the module does not hold it as a
// stored, pending or active fault, and OBD-II does not list it. Real faults carry one of the low four status bits.
static bool dtc_is_history(const obd_dtc_t *d)
{
    return d->has_status && (d->status & 0x0F) == 0;
}

static int module_faults(const module_t *m)
{
    int n = 0;
    for (int i = 0; i < m->dtc_count; i++) if (!dtc_is_history(&m->dtcs[i])) n++;
    return n;
}

static int total_codes(void)
{
    int total = 0;
    for (int i = 0; i < module_count; i++) total += module_faults(&modules[i]);
    return total;
}

static int total_history(void)
{
    int total = 0;
    for (int i = 0; i < module_count; i++) total += modules[i].dtc_count - module_faults(&modules[i]);
    return total;
}

static void show_menu(void)
{
    rows_reset();
    for (int i = 0; i < MENU_COUNT; i++) {
        char label[16];
        snprintf(label, sizeof(label), "%c %s", i == menu_cursor ? '>' : ' ', menu_items[i]);
        row(label, i == menu_cursor ? "[KEY]" : "", COLOR_YELLOW);
    }
    row("#BUTTONS", "", COLOR_CYAN);
    row("BOOT", "next item", 0);
    row("KEY", "select", 0);
    if (scanned) {
        row("#LAST SCAN", "", COLOR_CYAN);
        rowf("MODULES", COLOR_WHITE, "%d", module_count);
        int codes = total_codes();
        rowf("CODES", codes ? COLOR_ORANGE : COLOR_GREEN, "%d", codes);
        if (total_history()) rowf("HISTORY", COLOR_MUTED, "%d", total_history());
    }
    show("DIAGNOSTICS", COLOR_CYAN);
    status("READY", COLOR_CYAN);
}

static void scan(void)
{
    module_count = 0;
    scanned = false;
    status("SCANNING", COLOR_YELLOW);

    // Physical OBD addresses first, then the VAG style 0x700 range (answers on request + 0x6A)
    uint32_t candidates[8 + 0x96];
    int total = 0;
    for (uint32_t id = 0x7E0; id <= 0x7E7; id++) candidates[total++] = id;
    for (uint32_t id = 0x700; id < 0x796; id++) candidates[total++] = id;

    canstats_totals_t before;
    canstats_get_totals(&before);
    int64_t started = esp_timer_get_time();
    for (int i = 0; i < total; i++) {
        poll_key();
        if (take_key()) {
            ESP_LOGI(DIAG_TAG, "Scan cancelled");
            break;
        }
        power_mgr_poke();

        if (i % 3 == 0) {
            rows_reset();
            rowf("PROGRESS", COLOR_WHITE, "%d%%", i * 100 / total);
            rowf("ADDRESS", COLOR_WHITE, "0x%03lX", (unsigned long)candidates[i]);
            rowf("MODULES", module_count ? COLOR_GREEN : COLOR_MUTED, "%d", module_count);
            rowf("CODES", total_codes() ? COLOR_ORANGE : COLOR_GREEN, "%d", total_codes());
            row("KEY", "cancel", 0);
            show("SCANNING", COLOR_YELLOW);
        }

        if (module_count >= MAX_MODULES) break;
        module_t *m = &modules[module_count];
        memset(m, 0, sizeof(*m));
        m->tx = candidates[i];
        module_name(m->tx, m->name, sizeof(m->name));
        if (!read_module(m)) continue;

        ESP_LOGI(DIAG_TAG, "Module 0x%03lX (%s) answered on 0x%03lX: %s, %u code(s)", (unsigned long)m->tx, m->name,
                 (unsigned long)m->rx, m->uds ? "UDS" : (m->denied ? "no access" : "OBD"), m->dtc_count);
        log_dtcs(m);
        log_unfiltered(m);
        if (is_obd_physical(m->tx) && m->uds) log_obd_view(m);
        module_count++;
    }

    canstats_totals_t after;
    canstats_get_totals(&after);
    scan_frames = after.frames - before.frames;
    uint16_t state_color;
    ESP_LOGI(DIAG_TAG, "Bus during the scan: %lu frame(s) received, CAN controller %s", (unsigned long)scan_frames, can_state_text(&state_color));

    scanned = true;
    ESP_LOGI(DIAG_TAG, "Scan finished in %lld ms: %d module(s), %d code(s)", (long long)((esp_timer_get_time() - started) / 1000),
             module_count, total_codes());
}

// ---------------------------------------------------------------- results list
static int entry_total(void)
{
    int n = 0;
    for (int i = 0; i < module_count; i++) n += 1 + (modules[i].dtc_count ? modules[i].dtc_count : 1);
    return n;
}

static void entry_at(int index, char *label, size_t lsize, char *value, size_t vsize, uint16_t *color)
{
    *color = 0;
    value[0] = 0;
    for (int i = 0; i < module_count; i++) {
        const module_t *m = &modules[i];
        if (index == 0) {
            snprintf(label, lsize, "#%s", m->name);
            *color = module_faults(m) ? COLOR_ORANGE : (m->dtc_count ? COLOR_MUTED : COLOR_GREEN);
            return;
        }
        index--;
        int n = m->dtc_count ? m->dtc_count : 1;
        if (index < n) {
            if (!m->dtc_count) {
                snprintf(label, lsize, "%s", m->denied ? "no access" : "no codes");
                snprintf(value, vsize, "%s", m->denied ? "NRC" : "OK");
                *color = m->denied ? COLOR_YELLOW : COLOR_GREEN;
                return;
            }
            const obd_dtc_t *d = &m->dtcs[index];
            char code[6];
            obd_format_dtc(d->code, code);
            if (d->has_status) {
                // Engine and transmission use the standard code format. Other VW modules report their own number, which a
                // P/B/C/U letter would only mislead, so show what the module sent.
                if (is_obd_physical(m->tx) || (d->code >> 14) != 0) snprintf(label, lsize, "%s-%02X", code, d->fault_type);
                else snprintf(label, lsize, "%04X%02X", d->code, d->fault_type);
                obd_format_status(d->status, value, vsize);
                *color = dtc_is_history(d) ? COLOR_MUTED :
                         (d->status & (DTC_STATUS_TEST_FAILED | DTC_STATUS_WARNING_LAMP)) ? COLOR_RED :
                         ((d->status & DTC_STATUS_PENDING) && !(d->status & DTC_STATUS_CONFIRMED) ? COLOR_YELLOW : COLOR_ORANGE);
            } else {
                snprintf(label, lsize, "%s", code);
                snprintf(value, vsize, "STORED");
                *color = COLOR_ORANGE;
            }
            return;
        }
        index -= n;
    }
    label[0] = 0;
}

// The list is paged by pixels, not by a fixed row count: module headers take extra spacing, and a page must never run
// off the bottom of the 320 px screen.
#define LIST_FIRST_Y    81      // below the section title
#define LIST_LAST_Y     312     // the footer row has to end before this
#define ROW_H           14
#define HEADER_H        18      // a section row plus its gap

static int entry_height(int index)
{
    char label[16], value[24];
    uint16_t color;
    entry_at(index, label, sizeof(label), value, sizeof(value), &color);
    return label[0] == '#' ? HEADER_H : ROW_H;
}

// How many entries fit on the page that starts at 'first'
static int page_size(int first, int total)
{
    int y = LIST_FIRST_Y, n = 0;
    while (first + n < total) {
        int h = entry_height(first + n);
        if (n > 0 && y + h + ROW_H > LIST_LAST_Y) break;
        y += h;
        n++;
    }
    return n;
}

static int page_start(int target, int total, int *pages)
{
    int first = 0, p = 0, start = 0;
    while (first < total) {
        if (p == target) start = first;
        first += page_size(first, total);
        p++;
    }
    if (pages) *pages = p ? p : 1;
    return target < p ? start : 0;
}

static void show_results(void)
{
    int total = entry_total();
    int pages;
    page_start(0, total, &pages);
    if (page >= pages) page = 0;
    int first = page_start(page, total, NULL);
    int count = page_size(first, total);

    rows_reset();
    if (module_count == 0) {
        uint16_t state_color;
        const char *state = can_state_text(&state_color);
        bool asleep = !bench && scan_frames < 5;            // a live bus sends frames of its own; our requests are not echoed back
        row("NO MODULE", "answered", COLOR_RED);
        row("BUS", asleep ? "SILENT" : "traffic seen", asleep ? COLOR_ORANGE : COLOR_GREEN);
        row("CAN", state, state_color);
        row("", "", 0);
        if (asleep) {
            row("BUS ASLEEP?", "", COLOR_YELLOW);
            row("TURN", "ignition on", COLOR_WHITE);
            row("OR", "start engine", COLOR_WHITE);
        } else {
            row("BUS IS ALIVE", "", COLOR_YELLOW);
            row("CHECK", "OBD cable", COLOR_WHITE);
            row("CAN", "500 kbit/s", COLOR_WHITE);
        }
    }
    for (int i = 0; i < count && row_count < 15; i++) {
        char label[16], value[24];
        uint16_t color;
        entry_at(first + i, label, sizeof(label), value, sizeof(value), &color);
        row(label, value, color);
    }
    if (pages > 1 || module_count) {
        char hint[24];
        snprintf(hint, sizeof(hint), pages > 1 ? "BOOT more  KEY menu" : "KEY menu");
        char pg[24];
        snprintf(pg, sizeof(pg), "%d/%d", page + 1, pages);
        row(pg, hint, COLOR_CYAN);
    } else {
        row("KEY", "menu", COLOR_CYAN);
    }
    show("FAULT CODES", COLOR_ORANGE);

    int codes = total_codes();
    char text[32];
    if (module_count == 0) status("NO MODULES", COLOR_RED);
    else if (codes == 0 && total_history()) { snprintf(text, sizeof(text), "%d HISTORY", total_history()); status(text, COLOR_YELLOW); }
    else if (codes == 0) status("NO CODES", COLOR_GREEN);
    else { snprintf(text, sizeof(text), "%d CODE%s", codes, codes == 1 ? "" : "S"); status(text, COLOR_ORANGE); }
}

// ---------------------------------------------------------------- clearing
static void show_confirm(int held_ms)
{
    rows_reset();
    rowf("MODULES", COLOR_WHITE, "%d", module_count);
    rowf("CODES", COLOR_ORANGE, "%d", total_codes());
    row("", "", 0);
    row("#THIS ALSO", "", COLOR_YELLOW);
    row("RESETS", "readiness", COLOR_YELLOW);
    row("ERASES", "freeze frame", COLOR_YELLOW);
    row("NEEDS", "ignition on", COLOR_WHITE);
    row("", "", 0);
    if (held_ms > 0) {
        char bar[24];
        int filled = held_ms * 10 / CONFIRM_HOLD_MS;
        if (filled > 10) filled = 10;
        for (int i = 0; i < 10; i++) bar[i] = i < filled ? '#' : '-';
        bar[10] = 0;
        row("HOLDING", bar, COLOR_CYAN);
    } else {
        row("HOLD KEY 2 s", "to clear", COLOR_CYAN);
    }
    row("BOOT", "cancel", COLOR_MUTED);
    show("CLEAR CODES?", COLOR_ORANGE);
    status("CONFIRM", COLOR_ORANGE);
}

// Short words for the answers a module gives when it refuses a request
static const char *nrc_text(uint8_t nrc, char *buf, size_t size)
{
    switch (nrc) {
    case 0x11: case 0x12: snprintf(buf, size, "NOT SUPP"); break;      // service or sub-function not supported
    case 0x22: snprintf(buf, size, "NOT NOW");  break;                // conditions not correct
    case 0x33: snprintf(buf, size, "SECURITY"); break;                // security access denied
    case 0x7E: case 0x7F: snprintf(buf, size, "SESSION"); break;      // not supported in the active session
    default:   snprintf(buf, size, "NRC %02X", nrc); break;
    }
    return buf;
}

static void log_answer(const char *what, const char *name, int n)
{
    if (n <= 0) {
        ESP_LOGW(DIAG_TAG, "%s %s: no answer", name, what);
        return;
    }
    char hex[3 * 12 + 1] = "";
    for (int i = 0; i < resp[0].len && i < 12; i++) {
        char b[4];
        snprintf(b, sizeof(b), "%02X ", resp[0].data[i]);
        strlcat(hex, b, sizeof(hex));
    }
    ESP_LOGW(DIAG_TAG, "%s %s: answered %s(%u bytes, from 0x%03lX)", name, what, hex, resp[0].len, (unsigned long)resp[0].id);
}

static void set_clear_result(module_t *m, int n, uint8_t positive)
{
    if (n <= 0) {
        strlcpy(m->clear_result, "NO REPLY", sizeof(m->clear_result));
    } else if (resp[0].data[0] == positive) {
        strlcpy(m->clear_result, "OK", sizeof(m->clear_result));
    } else if (resp[0].len >= 3 && resp[0].data[0] == 0x7F) {
        nrc_text(resp[0].data[2], m->clear_result, sizeof(m->clear_result));
    } else {
        strlcpy(m->clear_result, "UNKNOWN", sizeof(m->clear_result));
    }
}

static bool is_negative(int n, uint8_t nrc_a, uint8_t nrc_b)
{
    return n > 0 && resp[0].len >= 3 && resp[0].data[0] == 0x7F && (resp[0].data[2] == nrc_a || resp[0].data[2] == nrc_b);
}

// Clear in the extended diagnostic session (10 03), then go back to the default one. Some control units only accept the
// clear there. Returns the raw result of the clear request.
static int clear_in_extended_session(module_t *m)
{
    static const uint8_t extended[] = { 0x10, 0x03 };
    static const uint8_t clear_uds[] = { 0x14, 0xFF, 0xFF, 0xFF };
    static const uint8_t normal[] = { 0x10, 0x01 };

    int n = request(m->tx, extended, sizeof(extended), 300);
    log_answer("extended session request", m->name, n);
    if (n <= 0 || resp[0].data[0] != 0x50) {
        return n > 0 ? n : 0;
    }
    n = request(m->tx, clear_uds, sizeof(clear_uds), 500);
    log_answer("clear in the extended session", m->name, n);
    diag_response_t keep = resp[0];
    int back = request(m->tx, normal, sizeof(normal), 300);
    log_answer("return to the default session", m->name, back);
    resp[0] = keep;                                     // report the clear's answer, not the session change's
    return n;
}

static void clear_module(module_t *m)
{
    static const uint8_t clear_uds[] = { 0x14, 0xFF, 0xFF, 0xFF };
    static const uint8_t clear_obd = 0x04;
    bool use_uds = m->uds || !is_obd_physical(m->tx);
    const uint8_t *req = use_uds ? clear_uds : &clear_obd;
    uint16_t len = use_uds ? sizeof(clear_uds) : 1;
    uint8_t positive = use_uds ? 0x54 : 0x44;

    int n = request(m->tx, req, len, 500);
    log_answer(use_uds ? "UDS clear (14 FF FF FF)" : "OBD clear (mode 04)", m->name, n);

    // The engine and transmission control units of a Mk7 answer UDS 0x14 with "service not supported"; they should accept
    // the OBD-II clear (mode 04) for their emission related codes. Try it before giving up.
    if (use_uds && is_obd_physical(m->tx) && is_negative(n, 0x11, 0x12)) {
        ESP_LOGW(DIAG_TAG, "%s refused UDS clear (NRC %02X), trying OBD mode 04", m->name, resp[0].data[2]);
        n = request(m->tx, &clear_obd, 1, 500);
        log_answer("OBD clear (mode 04)", m->name, n);
        positive = 0x44;
    }

    // Last resort: the extended diagnostic session. Only after a refusal, never after silence.
    if (n > 0 && resp[0].data[0] == 0x7F && resp[0].len >= 3 && resp[0].data[2] != 0x78) {
        ESP_LOGW(DIAG_TAG, "%s refused the clear (NRC %02X), trying the extended session", m->name, resp[0].data[2]);
        n = clear_in_extended_session(m);
        positive = 0x54;
    }

    set_clear_result(m, n, positive);
    ESP_LOGW(DIAG_TAG, "Clear codes on %s (0x%03lX): %s", m->name, (unsigned long)m->tx, m->clear_result);
}

static void clear_all(void)
{
    status("CLEARING", COLOR_YELLOW);
    for (int i = 0; i < module_count; i++) {
        rows_reset();
        rowf("MODULE", COLOR_WHITE, "%d/%d", i + 1, module_count);
        row("NAME", modules[i].name, COLOR_WHITE);
        show("CLEARING", COLOR_YELLOW);
        power_mgr_poke();
        clear_module(&modules[i]);
    }

    // Read back so the result shows what is really stored now, not just that the request was accepted
    int cleared = 0, left = 0;
    for (int i = 0; i < module_count; i++) {
        module_t *m = &modules[i];
        uint8_t keep_uds = m->uds;
        bool denied = m->denied;
        char name[12], result[12];
        strlcpy(name, m->name, sizeof(name));
        strlcpy(result, m->clear_result, sizeof(result));
        uint32_t tx = m->tx;

        memset(m, 0, sizeof(*m));
        m->tx = tx;
        strlcpy(m->name, name, sizeof(m->name));
        strlcpy(m->clear_result, result, sizeof(m->clear_result));
        m->uds = keep_uds;
        m->denied = denied;
        read_module(m);
        left += module_faults(m);                                    // history-only entries are not faults
        if (module_faults(m) == 0) cleared++;
    }

    rows_reset();
    for (int i = 0; i < module_count && i < 14; i++) {
        const module_t *m = &modules[i];
        int faults = module_faults(m), hist = m->dtc_count - faults;
        bool accepted = strcmp(m->clear_result, "OK") == 0;
        char value[24];
        if (m->dtc_count == 0) {
            row(m->name, accepted ? "CLEARED" : "NO CODES", COLOR_GREEN);
        } else if (faults == 0) {
            snprintf(value, sizeof(value), "%d HISTORY", hist);          // only "failed since last clear" entries are left
            row(m->name, value, COLOR_MUTED);
        } else if (accepted) {
            snprintf(value, sizeof(value), "%d BACK", faults);
            row(m->name, value, COLOR_ORANGE);
        } else {
            row(m->name, m->clear_result, COLOR_ORANGE);
        }
    }
    row("KEY", "menu", COLOR_CYAN);
    show("CLEAR RESULT", left ? COLOR_ORANGE : COLOR_GREEN);

    char text[32];
    if (left == 0 && cleared == module_count) status("CLEARED", COLOR_GREEN);
    else { snprintf(text, sizeof(text), "%d LEFT", left); status(text, COLOR_ORANGE); }
    ESP_LOGW(DIAG_TAG, "Clear finished: %d/%d modules clear, %d code(s) remain", cleared, module_count, left);
}

// ---------------------------------------------------------------- live data and info
static void live_data(void)
{
    static const uint8_t pids[LIVE_PID_COUNT] = { 0x0C, 0x0D, 0x05, 0x04, 0x11, 0x0F, 0x0B, 0x42 };
    char values[LIVE_PID_COUNT][24];
    bool seen[LIVE_PID_COUNT] = { false };
    for (int i = 0; i < LIVE_PID_COUNT; i++) strlcpy(values[i], "--", sizeof(values[i]));

    status("LIVE", COLOR_GREEN);
    int next = 0;
    int answered_total = 0;
    int misses = 0;
    while (!take_key()) {
        poll_key();
        power_mgr_poke();
        uint8_t req[2] = { 0x01, pids[next] };
        int n = request(DIAG_FUNCTIONAL_ID, req, 2, 80);
        bool got = false;
        if (n > 0 && resp[0].len >= 3 && resp[0].data[0] == 0x41 && resp[0].data[1] == pids[next]) {
            got = obd_format_pid(pids[next], resp[0].data + 2, resp[0].len - 2, values[next], sizeof(values[next]));
            if (got) { seen[next] = true; answered_total++; }
        }
        misses = got ? 0 : misses + 1;
        if (!got && !seen[next]) strlcpy(values[next], "--", sizeof(values[next]));
        next = (next + 1) % LIVE_PID_COUNT;

        if (next == 0 || next == LIVE_PID_COUNT / 2) {
            rows_reset();
            for (int i = 0; i < LIVE_PID_COUNT; i++) {
                row(obd_pid_name(pids[i]), values[i], seen[i] ? COLOR_WHITE : COLOR_MUTED);
            }
            row("", "", 0);
            row("KEY", "back", COLOR_CYAN);
            show("LIVE DATA", COLOR_CYAN);
            status(answered_total ? "LIVE" : "NO DATA", answered_total ? COLOR_GREEN : COLOR_YELLOW);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(DIAG_TAG, "Live data ended: %d of %d values seen, %d answers", (int)(seen[0] + seen[1] + seen[2] + seen[3] + seen[4] + seen[5] + seen[6] + seen[7]),
             LIVE_PID_COUNT, answered_total);
}

static void vehicle_info(void)
{
    status("READING", COLOR_YELLOW);
    char vin[20] = "";
    obd_readiness_t ready = { 0 };
    bool have_ready = false;

    static const uint8_t vin_req[] = { 0x09, 0x02 };
    if (request(DIAG_FUNCTIONAL_ID, vin_req, 2, 300) > 0) {
        obd_parse_vin(resp[0].data, resp[0].len, vin, sizeof(vin));
    }
    static const uint8_t ready_req[] = { 0x01, 0x01 };
    if (request(DIAG_FUNCTIONAL_ID, ready_req, 2, 200) > 0 && resp[0].len >= 6 && resp[0].data[0] == 0x41 && resp[0].data[1] == 0x01) {
        obd_parse_readiness(resp[0].data + 2, &ready);
        have_ready = true;
    }

    rows_reset();
    row("#VEHICLE", "", COLOR_CYAN);
    row("", vin[0] ? vin : "VIN not available", vin[0] ? COLOR_WHITE : COLOR_MUTED);
    if (have_ready) {
        row("MIL", ready.mil_on ? "ON" : "OFF", ready.mil_on ? COLOR_RED : COLOR_GREEN);
        rowf("CONFIRMED", ready.dtc_count ? COLOR_ORANGE : COLOR_GREEN, "%u", ready.dtc_count);
        row("#READINESS", "", COLOR_CYAN);
        for (int i = 0; i < OBD_READINESS_COUNT && row_count < 15; i++) {
            if (!(ready.supported & (1u << i))) continue;
            bool incomplete = ready.incomplete & (1u << i);
            row(obd_readiness_name(i, ready.compression_ignition), incomplete ? "NOT READY" : "READY", incomplete ? COLOR_YELLOW : COLOR_GREEN);
        }
    } else {
        row("ENGINE ECU", "no answer", COLOR_YELLOW);
    }
    row("KEY", "back", COLOR_CYAN);
    show("VEHICLE INFO", COLOR_CYAN);
    status(have_ready ? "INFO" : "NO DATA", have_ready ? COLOR_CYAN : COLOR_YELLOW);
    ESP_LOGI(DIAG_TAG, "Vehicle info: VIN '%s', ready data %s", vin, have_ready ? "yes" : "no");
}

// ---------------------------------------------------------------- main task
#ifdef DIAG_SELFTEST
static void release_key_later(void *arg) { ev_key = true; }

// Test builds only (-DDIAG_SELFTEST): drive every screen without pressing buttons and log what it finds
static void selftest(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGW(DIAG_TAG, "SELFTEST scan");
    scan();
    {
        int total = entry_total(), pages;
        page_start(0, total, &pages);
        ESP_LOGW(DIAG_TAG, "SELFTEST results: %d entries, %d code(s), %d page(s)", total, total_codes(), pages);
        for (int p = 0; p < pages; p++) {
            page = p;
            show_results();
            int y = LIST_FIRST_Y - ROW_H;
            for (int i = 1; i < row_count; i++) y += (rows[i].label[0] == '#' ? HEADER_H : ROW_H);
            ESP_LOGW(DIAG_TAG, "SELFTEST page %d/%d: %d rows, list ends at y=%d (limit %d)", p + 1, pages, row_count, y + ROW_H, LIST_LAST_Y);
            vTaskDelay(pdMS_TO_TICKS(700));
        }
    }
    page = 0;
    show_results();

    ESP_LOGW(DIAG_TAG, "SELFTEST info");
    vehicle_info();

    ESP_LOGW(DIAG_TAG, "SELFTEST live (3 s)");
    const esp_timer_create_args_t args = { .callback = release_key_later, .name = "selftest" };
    esp_timer_handle_t timer;
    esp_timer_create(&args, &timer);
    esp_timer_start_once(timer, 3000000);
    live_data();

    ESP_LOGW(DIAG_TAG, "SELFTEST clear");
    clear_all();
    ESP_LOGW(DIAG_TAG, "SELFTEST rescan");
    scan();
    ESP_LOGW(DIAG_TAG, "SELFTEST done: %d module(s), %d code(s) after clearing", module_count, total_codes());
}
#endif

static void diag_task(void *arg)
{
    ESP_LOGI(DIAG_TAG, "Diagnostics ready (BOOT = next, KEY = select)");
    show_menu();
#ifdef DIAG_SELFTEST
    selftest();
#endif

    while (1) {
        poll_key();

        switch (ui) {
        case UI_MENU:
            if (take_next()) {
                menu_cursor = (menu_cursor + 1) % MENU_COUNT;
                show_menu();
            }
            if (take_key()) {
                power_mgr_poke();
                if (menu_cursor == 0) {
                    scan();
                    page = 0;
                    ui = UI_RESULTS;
                    show_results();
                } else if (menu_cursor == 1) {
                    if (!scanned || module_count == 0 || total_codes() == 0) {
                        // Nothing known to clear: scan first so the confirmation shows real numbers
                        scan();
                    }
                    if (module_count == 0) {
                        page = 0;
                        ui = UI_RESULTS;
                        show_results();
                    } else if (total_codes() == 0) {
                        rows_reset();
                        row("NO CODES", "to clear", COLOR_GREEN);
                        rowf("MODULES", COLOR_WHITE, "%d", module_count);
                        row("KEY", "menu", COLOR_CYAN);
                        show("CLEAR CODES", COLOR_GREEN);
                        status("NO CODES", COLOR_GREEN);
                        ui = UI_NOTICE;
                    } else {
                        ui = UI_CONFIRM;
                        show_confirm(0);
                    }
                } else if (menu_cursor == 2) {
                    ui = UI_LIVE;
                    live_data();
                    ui = UI_MENU;
                    show_menu();
                } else {
                    ui = UI_INFO;
                    vehicle_info();
                }
            }
            break;

        case UI_RESULTS:
            if (take_next()) {
                page++;
                show_results();
            }
            if (take_key()) {
                ui = UI_MENU;
                show_menu();
            }
            break;

        case UI_CONFIRM: {
            static int shown_held = -1;
            int held = key_held_ms();
            int bucket = held * 10 / CONFIRM_HOLD_MS;
            if (held >= CONFIRM_HOLD_MS) {
                shown_held = -1;
                ev_key = false;
                clear_all();
                ui = UI_NOTICE;
                // wait for the button to be released so it does not count as a menu press
                while (gpio_get_level(KEY_PIN) == 0) vTaskDelay(pdMS_TO_TICKS(20));
                key_was_down = false;
                ev_key = false;
            } else if (bucket != shown_held) {
                shown_held = bucket;
                show_confirm(held);
            }
            if (take_next()) {
                shown_held = -1;
                ui = UI_MENU;
                show_menu();
            }
            take_key();                                 // a short press does not confirm
            break;
        }

        case UI_NOTICE:
        case UI_INFO:
            take_next();
            if (take_key()) {
                ui = UI_MENU;
                show_menu();
            }
            break;

        default:
            ui = UI_MENU;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void diag_start(bool use_bench)
{
    bench = use_bench;

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << KEY_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&cfg);

    mode_mgr_set_tap_handler(on_boot_tap);
    diag_can_start(use_bench);
    stackwatch_create(diag_task, "diag", 8192, NULL, 2);
}
