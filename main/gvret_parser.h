#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Byte-by-byte reader for the GVRET commands SavvyCAN sends. It knows how long the commands with data are, so the data of a
// command that is not supported (for example a bus speed change) is skipped instead of being read as commands.
typedef struct {
    int         state;
    uint8_t     buf[16];
    int         have;
    int         need;
    int         skip;
    void      (*write)(const uint8_t *data, size_t len);                                        // reply to SavvyCAN
    void      (*transmit)(uint32_t id, bool extended, uint8_t dlc, const uint8_t *data);        // put a frame on the bus
    uint32_t  (*now_us)(void);
} gvret_parser_t;

void gvret_parser_reset(gvret_parser_t *p);
void gvret_parser_feed(gvret_parser_t *p, const uint8_t *data, size_t len);
