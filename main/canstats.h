#ifndef CANSTATS_H
#define CANSTATS_H

#include <stdint.h>

// Lightweight CAN bus statistics for the display's bus page. Counting is cheap enough to run on every frame.

typedef enum {
    CAN_EVENT_ERROR_WARNING,
    CAN_EVENT_ERROR_PASSIVE,
    CAN_EVENT_BUS_OFF,
} can_event_t;

typedef struct {
    uint32_t frames;            // frames received since boot
    uint32_t bits;              // estimated bits on the bus (including stuffing and framing)
    uint32_t unique_ids;        // distinct 11-bit IDs seen
    uint32_t tx_failures;       // frames the controller could not transmit
    uint32_t events[3];         // indexed by can_event_t
} canstats_totals_t;

void canstats_on_frame(uint32_t id, uint8_t dlc);
void canstats_on_tx_failure(void);
void canstats_on_event(can_event_t event);
void canstats_get_totals(canstats_totals_t *out);

// The busiest IDs (most frames first). Returns how many were written.
int  canstats_top_ids(uint32_t *ids, uint32_t *counts, int max);

#endif // CANSTATS_H
