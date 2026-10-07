#include "gvret_parser.h"

// Protocol command bytes
#define PROTO_BUILD_CAN_FRAME    0
#define PROTO_TIME_SYNC          1
#define PROTO_SET_DIG_OUT        4
#define PROTO_SETUP_CANBUS       5
#define PROTO_GET_CANBUS_PARAMS  6
#define PROTO_GET_DEV_INFO       7
#define PROTO_SET_SW_MODE        8
#define PROTO_GET_EXT_BUSES      9
#define PROTO_SET_SYSTYPE        10
#define PROTO_GET_NUM_BUSES      12
#define PROTO_GET_NUM_BUSES_EXT  13
#define PROTO_SET_EXT_BUSES      14

enum { ST_IDLE, ST_CMD, ST_FRAME, ST_SKIP };

static void put_u32(uint8_t *out, uint32_t v)
{
    out[0] = v & 0xFF; out[1] = (v >> 8) & 0xFF; out[2] = (v >> 16) & 0xFF; out[3] = (v >> 24) & 0xFF;
}

void gvret_parser_reset(gvret_parser_t *p)
{
    p->state = ST_IDLE;
    p->have = p->need = p->skip = 0;
}

// Number of data bytes that follow a command we do not act on
static int ignored_payload(uint8_t cmd)
{
    switch (cmd) {
    case PROTO_SET_DIG_OUT:
    case PROTO_SET_SW_MODE:
    case PROTO_SET_SYSTYPE:   return 1;
    case PROTO_SETUP_CANBUS:  return 8;
    case PROTO_SET_EXT_BUSES: return 12;
    default:                  return 0;
    }
}

static void handle_command(gvret_parser_t *p, uint8_t cmd)
{
    switch (cmd) {
    case PROTO_GET_NUM_BUSES: {
        const uint8_t resp[] = { 0xF1, PROTO_GET_NUM_BUSES, 1 };
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_GET_NUM_BUSES_EXT: {
        const uint8_t resp[] = { 0xF1, PROTO_GET_NUM_BUSES_EXT, 0 };
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_GET_DEV_INFO: {
        const uint8_t resp[] = { 0xF1, PROTO_GET_DEV_INFO, 0x20, 0x01, 0x00, 0x00 };
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_GET_CANBUS_PARAMS: {
        const uint8_t resp[] = { 0xF1, PROTO_GET_CANBUS_PARAMS, 0x01, 0x20, 0xA1, 0x07, 0x00 };   // bus 0 active, 500000 baud
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_GET_EXT_BUSES: {                 // also used as the keepalive ping
        const uint8_t resp[] = { 0xF1, PROTO_GET_EXT_BUSES, 0x01, 0x00, 0x00, 0x00 };
        p->write(resp, sizeof(resp));
        break;
    }
    case PROTO_TIME_SYNC: {
        uint8_t resp[6] = { 0xF1, PROTO_TIME_SYNC };
        put_u32(resp + 2, p->now_us());
        p->write(resp, sizeof(resp));
        break;
    }
    default:
        break;
    }
}

void gvret_parser_feed(gvret_parser_t *p, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        switch (p->state) {
        case ST_IDLE:
            if (b == 0xF1) p->state = ST_CMD;           // anything else (0xE7 filler, checksums) is skipped
            break;

        case ST_CMD:
            if (b == PROTO_BUILD_CAN_FRAME) {
                p->state = ST_FRAME;
                p->have = 0;
                p->need = 6;                            // id (4), bus (1), length (1), then the data
            } else if (ignored_payload(b)) {
                p->state = ST_SKIP;
                p->skip = ignored_payload(b);
            } else {
                handle_command(p, b);
                p->state = ST_IDLE;
            }
            break;

        case ST_SKIP:
            if (--p->skip <= 0) p->state = ST_IDLE;
            break;

        case ST_FRAME:
            p->buf[p->have++] = b;
            if (p->have == 6) {
                uint8_t dlc = p->buf[5] & 0x0F;
                p->need = 6 + (dlc > 8 ? 8 : dlc);
            }
            if (p->have >= p->need) {
                uint32_t id = (uint32_t)p->buf[0] | ((uint32_t)p->buf[1] << 8) | ((uint32_t)p->buf[2] << 16) | ((uint32_t)p->buf[3] << 24);
                uint8_t dlc = p->buf[5] & 0x0F;
                if (dlc > 8) dlc = 8;
                bool extended = (id & (1UL << 31)) != 0;
                id &= ~(1UL << 31);
                // One bus only, and an identifier that does not fit its format is a damaged frame: do not send it
                bool valid = p->buf[4] == 0 && (extended ? id <= 0x1FFFFFFFUL : id <= 0x7FFUL);
                if (valid) p->transmit(id, extended, dlc, &p->buf[6]);
                p->state = ST_IDLE;
            }
            break;
        }
    }
}
