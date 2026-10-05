#ifndef DIAG_CAN_H
#define DIAG_CAN_H

// Request/response layer for the standalone diagnostics and ELM327 modes. It owns the CAN controller in
// those modes (the Simos bridge uses its own ISO-TP links instead) and implements just enough ISO-TP for a
// tester: single/multi-frame requests, flow control, reassembly of multi-frame responses from several ECUs,
// and "response pending" (NRC 0x78) handling. In bench mode frames go to the simulator instead of the bus.

#include <stdint.h>
#include <stdbool.h>

#define DIAG_FUNCTIONAL_ID      0x7DF
#define DIAG_MAX_PAYLOAD        1024
#define DIAG_MAX_RESPONSES      8

typedef struct {
    uint32_t    id;                         // CAN id the response came from
    uint16_t    len;
    uint8_t     data[DIAG_MAX_PAYLOAD];
} diag_response_t;

typedef struct {
    uint32_t    tx_id;                      // 0x7DF broadcasts to every OBD ECU
    uint32_t    rx_lo, rx_hi;               // accepted response ids (inclusive)
    const uint8_t *request;
    uint16_t    request_len;
    uint32_t    timeout_ms;                 // wait for the first response
    uint32_t    extra_ms;                   // functional requests: how long to keep listening for more ECUs
    uint8_t     max_responses;              // stop early once this many ECUs answered (0 = DIAG_MAX_RESPONSES)
} diag_request_t;

// Start (or restart) the CAN side. bench selects the in-memory simulator instead of the TWAI driver.
void diag_can_start(bool bench);

// Returns the number of responses stored in out (0 = nothing answered), or -1 if the request could not be sent.
// Not reentrant: callers are serialised internally.
int  diag_can_request(const diag_request_t *req, diag_response_t *out, int out_max);

// Convenience for the common case: physical or functional OBD request with default response ranges.
int  diag_can_obd(uint32_t tx_id, const uint8_t *request, uint16_t len, diag_response_t *out, int out_max, uint32_t timeout_ms);

// Response id range for a request id: 0x7E0..0x7E7 answer on +8, 0x700..0x77F (VAG) on +0x6A, 0x7DF on 0x7E8..0x7EF.
void diag_default_rx_range(uint32_t tx_id, uint32_t *lo, uint32_t *hi);

// Milliseconds since a frame was last received (UINT32_MAX if none yet). Lets the sleep logic and screens
// tell a live bus from a silent one.
uint32_t diag_can_ms_since_rx(void);

#endif // DIAG_CAN_H
