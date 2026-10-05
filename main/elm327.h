#ifndef ELM327_H
#define ELM327_H

// ELM327 command interpreter. It turns the text a phone app sends ("ATZ", "0100", "ATSH7E0", ...) into
// diagnostic requests and formats the answers the way an ELM327 chip does, so generic OBD apps work.
// Transport (BLE, Wi-Fi) and CAN access are supplied through elm_ops_t, which keeps this file free of
// hardware code and covered by the host tests (pio test -e native).
//
// Supported: CAN 11 bit / 500 kbit/s (ISO 15765-4, protocol 6, also as auto). Not supported: the other
// protocols, raw frame mode (ATCAF0), monitor mode (ATMA) and custom flow control (accepted, ignored).

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "diag_can.h"

#define ELM_LINE_MAX    96

typedef struct {
    // Send a request and collect the answers. Returns the number of responses, 0 for none, -1 if the
    // request could not be sent.
    int  (*request)(void *ctx, uint32_t tx_id, uint32_t rx_lo, uint32_t rx_hi, const uint8_t *req, uint16_t len,
                    uint32_t timeout_ms, diag_response_t *out, int out_max);
    // Text for the app
    void (*write)(void *ctx, const char *text);
    // Display / logging hook, optional: called with each command line received
    void (*on_command)(void *ctx, const char *line);
} elm_ops_t;

typedef struct {
    const elm_ops_t *ops;
    void           *ctx;
    diag_response_t *resp;              // DIAG_MAX_RESPONSES entries, allocated by elm_init
    char            line[ELM_LINE_MAX];
    int             line_len;
    char            last[ELM_LINE_MAX]; // repeated by an empty line, like the real chip
    bool            echo, linefeed, spaces, headers;
    bool            auto_protocol;
    bool            connected;          // an OBD request has been answered since reset
    uint32_t        tx_id;
    uint32_t        cra_lo, cra_hi;     // response filter set with ATCRA (0 = automatic)
    uint32_t        timeout_ms;
} elm_t;

bool elm_init(elm_t *elm, const elm_ops_t *ops, void *ctx);
void elm_free(elm_t *elm);
void elm_reset(elm_t *elm);

// Feed received bytes. Lines end with CR (or LF); each finished line is executed and answered through ops.write.
void elm_input(elm_t *elm, const uint8_t *data, size_t len);

#endif // ELM327_H
