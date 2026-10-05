#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "elm327.h"

#define ELM_VERSION         "ELM327 v1.5"
#define ELM_DEFAULT_ST      0x32        // x 4 ms = 200 ms
#define ELM_EXTRA_MS        60

static void out(elm_t *e, const char *s)
{
    e->ops->write(e->ctx, s);
}

// One answer line, terminated like the chip does
static void out_line(elm_t *e, const char *s)
{
    out(e, s);
    out(e, e->linefeed ? "\r\n" : "\r");
}

bool elm_init(elm_t *elm, const elm_ops_t *ops, void *ctx)
{
    memset(elm, 0, sizeof(*elm));
    elm->ops = ops;
    elm->ctx = ctx;
    elm->resp = calloc(DIAG_MAX_RESPONSES, sizeof(diag_response_t));
    if (!elm->resp) return false;
    elm_reset(elm);
    return true;
}

void elm_free(elm_t *elm)
{
    free(elm->resp);
    elm->resp = NULL;
}

void elm_reset(elm_t *elm)
{
    elm->echo = true;
    elm->linefeed = true;       // ELM327 default is ATL1
    elm->spaces = true;
    elm->headers = false;
    elm->auto_protocol = true;
    elm->connected = false;
    elm->tx_id = DIAG_FUNCTIONAL_ID;
    elm->cra_lo = elm->cra_hi = 0;
    elm->timeout_ms = ELM_DEFAULT_ST * 4;
    elm->last[0] = 0;
}

// ---- hex helpers ----
static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parse_hex(const char *s, uint32_t *value, int max_digits)
{
    int n = 0;
    uint32_t v = 0;
    for (; *s; s++, n++) {
        int d = hex_val(*s);
        if (d < 0 || n >= max_digits) return false;
        v = (v << 4) | (uint32_t)d;
    }
    if (n == 0) return false;
    *value = v;
    return true;
}

// ---- response formatting ----
static void append_bytes(char *buf, size_t size, const uint8_t *d, size_t n, bool spaces)
{
    size_t pos = strlen(buf);
    for (size_t i = 0; i < n && pos + 4 < size; i++) {
        pos += snprintf(buf + pos, size - pos, spaces && i ? " %02X" : "%02X", d[i]);
    }
}

static void print_response(elm_t *e, const diag_response_t *r)
{
    char line[ELM_LINE_MAX + 40];

    if (r->len <= 7) {
        line[0] = 0;
        if (e->headers) {
            snprintf(line, sizeof(line), e->spaces ? "%03lX %02X " : "%03lX%02X", (unsigned long)r->id, r->len);
        }
        append_bytes(line, sizeof(line), r->data, r->len, e->spaces);
        out_line(e, line);
        return;
    }

    // Multi-frame answer. Headers off: a length line, then numbered lines (6 bytes, then 7 per line).
    // Headers on: the raw frames, rebuilt from the payload.
    if (!e->headers) {
        snprintf(line, sizeof(line), "%03X", r->len);
        out_line(e, line);
        int idx = 0;
        for (size_t pos = 0; pos < r->len; idx++) {
            size_t n = (idx == 0) ? 6 : 7;
            if (n > r->len - pos) n = r->len - pos;
            snprintf(line, sizeof(line), "%X: ", idx & 0xF);
            append_bytes(line, sizeof(line), r->data + pos, n, e->spaces);
            out_line(e, line);
            pos += n;
        }
        return;
    }

    snprintf(line, sizeof(line), e->spaces ? "%03lX %02X %02X " : "%03lX%02X%02X", (unsigned long)r->id, 0x10 | (r->len >> 8), r->len & 0xFF);
    append_bytes(line, sizeof(line), r->data, 6, e->spaces);
    out_line(e, line);
    int seq = 1;
    for (size_t pos = 6; pos < r->len; pos += 7, seq++) {
        size_t n = r->len - pos > 7 ? 7 : r->len - pos;
        snprintf(line, sizeof(line), e->spaces ? "%03lX %02X " : "%03lX%02X", (unsigned long)r->id, 0x20 | (seq & 0xF));
        append_bytes(line, sizeof(line), r->data + pos, n, e->spaces);
        out_line(e, line);
    }
}

// ---- requests ----
static void response_range(const elm_t *e, uint32_t *lo, uint32_t *hi)
{
    if (e->cra_lo) {
        *lo = e->cra_lo;
        *hi = e->cra_hi;
    } else {
        diag_default_rx_range(e->tx_id, lo, hi);
    }
}

static int do_request(elm_t *e, const uint8_t *req, uint16_t len, uint32_t timeout_ms)
{
    uint32_t lo, hi;
    response_range(e, &lo, &hi);
    return e->ops->request(e->ctx, e->tx_id, lo, hi, req, len, timeout_ms, e->resp, DIAG_MAX_RESPONSES);
}

static void run_obd(elm_t *e, const char *hex)
{
    size_t n = strlen(hex);
    int expect = 0;
    if (n & 1) {                                // a trailing single digit is the expected response count
        expect = hex_val(hex[n - 1]);
        n--;
    }
    if (n < 2 || n > 2 * DIAG_MAX_PAYLOAD || n / 2 > 4095 || expect < 0) {
        out_line(e, "?");
        return;
    }

    uint8_t req[ELM_LINE_MAX / 2];
    if (n / 2 > sizeof(req)) {
        out_line(e, "?");
        return;
    }
    for (size_t i = 0; i < n / 2; i++) {
        int h = hex_val(hex[2 * i]), l = hex_val(hex[2 * i + 1]);
        if (h < 0 || l < 0) {
            out_line(e, "?");
            return;
        }
        req[i] = (uint8_t)(h << 4 | l);
    }

    bool probe = n == 4 && req[0] == 0x01 && req[1] == 0x00;
    if (!e->connected && e->auto_protocol && probe) {
        out_line(e, "SEARCHING...");
    }

    int count = do_request(e, req, (uint16_t)(n / 2), e->timeout_ms);
    if (count < 0) {
        out_line(e, "CAN ERROR");
    } else if (count == 0) {
        out_line(e, (!e->connected && e->auto_protocol && probe) ? "UNABLE TO CONNECT" : "NO DATA");
    } else {
        e->connected = true;
        for (int i = 0; i < count; i++) print_response(e, &e->resp[i]);
    }
}

// Battery voltage: there is no sense line on the dongle, so ask the ECU (mode 01 PID 42)
static void run_rv(elm_t *e)
{
    uint8_t req[2] = { 0x01, 0x42 };
    uint32_t saved_tx = e->tx_id;
    e->tx_id = DIAG_FUNCTIONAL_ID;
    uint32_t lo, hi;
    diag_default_rx_range(e->tx_id, &lo, &hi);
    int count = e->ops->request(e->ctx, e->tx_id, lo, hi, req, 2, 150, e->resp, DIAG_MAX_RESPONSES);
    e->tx_id = saved_tx;

    char line[16] = "0.0V";
    if (count > 0 && e->resp[0].len >= 4 && e->resp[0].data[0] == 0x41 && e->resp[0].data[1] == 0x42) {
        unsigned mv = (e->resp[0].data[2] << 8) | e->resp[0].data[3];
        snprintf(line, sizeof(line), "%u.%uV", mv / 1000, (mv % 1000) / 100);
    }
    out_line(e, line);
}

static bool flag_arg(const char *cmd, const char *prefix, bool *value)
{
    size_t n = strlen(prefix);
    if (strncmp(cmd, prefix, n) != 0 || (cmd[n] != '0' && cmd[n] != '1') || cmd[n + 1]) return false;
    *value = cmd[n] == '1';
    return true;
}

static void run_at(elm_t *e, const char *cmd)
{
    bool v;
    uint32_t num;

    if (strcmp(cmd, "Z") == 0) {
        elm_reset(e);
        out_line(e, "");
        out_line(e, ELM_VERSION);
        return;
    }
    if (strcmp(cmd, "WS") == 0) {
        out_line(e, "");
        out_line(e, ELM_VERSION);
        return;
    }
    if (strcmp(cmd, "D") == 0) {                // defaults, keeps the connection state
        bool connected = e->connected;
        elm_reset(e);
        e->connected = connected;
        out_line(e, "OK");
        return;
    }
    if (strcmp(cmd, "I") == 0)   { out_line(e, ELM_VERSION); return; }
    if (strcmp(cmd, "@1") == 0)  { out_line(e, "OBDII to RS232 Interpreter"); return; }
    if (strcmp(cmd, "@2") == 0)  { out_line(e, "ISOTP-BLE"); return; }
    if (strcmp(cmd, "RV") == 0)  { run_rv(e); return; }
    if (strcmp(cmd, "DP") == 0)  { out_line(e, e->auto_protocol ? "AUTO, ISO 15765-4 (CAN 11/500)" : "ISO 15765-4 (CAN 11/500)"); return; }
    if (strcmp(cmd, "DPN") == 0) { out_line(e, e->auto_protocol ? "A6" : "6"); return; }

    if (flag_arg(cmd, "E", &v))  { e->echo = v; out_line(e, "OK"); return; }
    if (flag_arg(cmd, "L", &v))  { out_line(e, "OK"); e->linefeed = v; return; }
    if (flag_arg(cmd, "S", &v))  { e->spaces = v; out_line(e, "OK"); return; }
    if (flag_arg(cmd, "H", &v))  { e->headers = v; out_line(e, "OK"); return; }
    if (strcmp(cmd, "CAF1") == 0) { out_line(e, "OK"); return; }

    if ((strncmp(cmd, "SP", 2) == 0 || strncmp(cmd, "TP", 2) == 0) && cmd[2]) {
        const char *p = cmd + 2;
        bool autop = false;
        if (*p == 'A') { autop = true; p++; }
        if (strcmp(p, "0") == 0)      { e->auto_protocol = true; out_line(e, "OK"); return; }
        if (strcmp(p, "6") == 0)      { e->auto_protocol = autop; out_line(e, "OK"); return; }
        out_line(e, "?");                       // other protocols are not wired up
        return;
    }
    if (strncmp(cmd, "ST", 2) == 0 && parse_hex(cmd + 2, &num, 2)) {
        e->timeout_ms = (num ? num : ELM_DEFAULT_ST) * 4;
        out_line(e, "OK");
        return;
    }
    if (strncmp(cmd, "SH", 2) == 0 && parse_hex(cmd + 2, &num, 8) && num < 0x800) {
        e->tx_id = num;
        out_line(e, "OK");
        return;
    }
    if (strcmp(cmd, "CRA") == 0) { e->cra_lo = e->cra_hi = 0; out_line(e, "OK"); return; }
    if (strncmp(cmd, "CRA", 3) == 0 && parse_hex(cmd + 3, &num, 8) && num < 0x800) {
        e->cra_lo = e->cra_hi = num;
        if (!num) e->cra_lo = e->cra_hi = 0;
        out_line(e, "OK");
        return;
    }

    // Accepted without effect: the chip settings that do not change how requests are made here
    static const char *const harmless[] = {
        "AT0", "AT1", "AT2", "AL", "NL", "D0", "D1", "M0", "M1", "V0", "V1", "R0", "R1", "CEA", "CS", "CFC0", "CFC1",
        "BI", "KW0", "KW1", "JTM1", "JTM5", "IB10", "IB48",
    };
    for (size_t i = 0; i < sizeof(harmless) / sizeof(harmless[0]); i++) {
        if (strcmp(cmd, harmless[i]) == 0) { out_line(e, "OK"); return; }
    }
    if (strncmp(cmd, "FCS", 3) == 0 || strncmp(cmd, "FCM", 3) == 0 || strncmp(cmd, "CF", 2) == 0 ||
        strncmp(cmd, "CM", 2) == 0 || strncmp(cmd, "PP", 2) == 0 || strncmp(cmd, "SW", 2) == 0) {
        out_line(e, "OK");
        return;
    }

    out_line(e, "?");
}

static void run_line(elm_t *e, char *raw)
{
    // Upper case, drop spaces
    char cmd[ELM_LINE_MAX];
    size_t n = 0;
    for (const char *p = raw; *p && n + 1 < sizeof(cmd); p++) {
        if (*p == ' ' || *p == '\t') continue;
        cmd[n++] = (char)toupper((unsigned char)*p);
    }
    cmd[n] = 0;

    if (e->echo) out_line(e, raw);

    if (n == 0) {                               // an empty line repeats the previous command
        if (e->last[0]) strlcpy(cmd, e->last, sizeof(cmd));
        else { out(e, ">"); return; }
    } else {
        strlcpy(e->last, cmd, sizeof(e->last));
    }

    if (e->ops->on_command) e->ops->on_command(e->ctx, cmd);

    if (cmd[0] == 'A' && cmd[1] == 'T') {
        run_at(e, cmd + 2);
    } else {
        bool all_hex = true;
        for (const char *p = cmd; *p; p++) if (hex_val(*p) < 0) all_hex = false;
        if (all_hex) run_obd(e, cmd);
        else out_line(e, "?");
    }

    out_line(e, "");
    out(e, ">");
}

void elm_input(elm_t *e, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t c = data[i];
        if (c == '\r' || c == '\n') {
            if (e->line_len == 0 && c == '\n') continue;        // the LF of a CR LF pair
            e->line[e->line_len] = 0;
            e->line_len = 0;
            run_line(e, e->line);
        } else if (c >= 0x20 && c < 0x7F) {
            if (e->line_len < ELM_LINE_MAX - 1) e->line[e->line_len++] = (char)c;
        }
    }
}
