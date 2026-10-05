#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "canstats.h"

#define TRACKED_IDS     16

typedef struct {
    uint32_t id;
    uint32_t count;
} tracked_id_t;

static portMUX_TYPE         stats_lock = portMUX_INITIALIZER_UNLOCKED;
static canstats_totals_t    totals;
static uint32_t             seen_bitmap[2048 / 32];
static tracked_id_t         tracked[TRACKED_IDS];

void canstats_on_frame(uint32_t id, uint8_t dlc)
{
    uint32_t std_id = id & 0x7FF;
    uint8_t len = dlc > 8 ? 8 : dlc;

    taskENTER_CRITICAL(&stats_lock);
        totals.frames++;
        totals.bits += (47 + 8 * len) * 115 / 100;      // framing plus roughly 15% bit stuffing

        uint32_t *word = &seen_bitmap[std_id / 32];
        uint32_t bit = 1u << (std_id % 32);
        if (!(*word & bit)) {
            *word |= bit;
            totals.unique_ids++;
        }

        // Keep counts for the busiest IDs; when the table is full, the least busy slot is reused
        tracked_id_t *slot = NULL;
        tracked_id_t *weakest = &tracked[0];
        for (int i = 0; i < TRACKED_IDS; i++) {
            if (tracked[i].count && tracked[i].id == id) {
                slot = &tracked[i];
                break;
            }
            if (tracked[i].count < weakest->count) {
                weakest = &tracked[i];
            }
        }
        if (slot) {
            slot->count++;
        } else {
            weakest->id = id;
            weakest->count = weakest->count ? weakest->count + 1 : 1;
        }
    taskEXIT_CRITICAL(&stats_lock);
}

void canstats_on_tx_failure(void)
{
    taskENTER_CRITICAL(&stats_lock);
        totals.tx_failures++;
    taskEXIT_CRITICAL(&stats_lock);
}

void canstats_on_event(can_event_t event)
{
    if ((int)event < 0 || (int)event > 2) {
        return;
    }
    taskENTER_CRITICAL(&stats_lock);
        totals.events[event]++;
    taskEXIT_CRITICAL(&stats_lock);
}

void canstats_get_totals(canstats_totals_t *out)
{
    taskENTER_CRITICAL(&stats_lock);
        *out = totals;
    taskEXIT_CRITICAL(&stats_lock);
}

int canstats_top_ids(uint32_t *ids, uint32_t *counts, int max)
{
    tracked_id_t copy[TRACKED_IDS];
    taskENTER_CRITICAL(&stats_lock);
        memcpy(copy, tracked, sizeof(copy));
    taskEXIT_CRITICAL(&stats_lock);

    int n = 0;
    for (int out = 0; out < max; out++) {
        int best = -1;
        for (int i = 0; i < TRACKED_IDS; i++) {
            if (copy[i].count && (best < 0 || copy[i].count > copy[best].count)) {
                best = i;
            }
        }
        if (best < 0) {
            break;
        }
        ids[n] = copy[best].id;
        counts[n] = copy[best].count;
        n++;
        copy[best].count = 0;
    }
    return n;
}
