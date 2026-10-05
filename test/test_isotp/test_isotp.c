// Host-side tests for the ISO-TP engine (main/isotp.c). Run with: pio test -e native
#include <unity.h>
#include <string.h>
#include "isotp.h"
#include "../../main/isotp.c"

#define MAX_SENT 512

typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint16_t len;
} sent_frame_t;

static uint64_t     now_us;
static sent_frame_t sent[MAX_SENT];
static int          sent_count;
static IsoTpLink    link;
static uint8_t      send_buf[256];
static uint8_t      recv_buf[256];

// ---- hooks required by the engine ----
int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t *data, const uint16_t size)
{
    if (sent_count < MAX_SENT) {
        sent[sent_count].id = arbitration_id;
        sent[sent_count].len = size;
        memcpy(sent[sent_count].data, data, size > 8 ? 8 : size);
        sent_count++;
    }
    return ISOTP_RET_OK;
}
uint64_t isotp_user_get_us(void) { return now_us; }
void isotp_user_debug(const char *message, ...) { (void)message; }
void isotp_user_flow_control(uint32_t id, uint8_t bs, uint32_t rx_us, uint16_t override_us, uint32_t used_us)
{
    (void)id; (void)bs; (void)rx_us; (void)override_us; (void)used_us;
}

// ---- helpers ----
void setUp(void)
{
    now_us = 1000;
    sent_count = 0;
    memset(sent, 0, sizeof(sent));
    isotp_init_link(&link, 0x7E0, 0x7E8, send_buf, sizeof(send_buf), recv_buf, sizeof(recv_buf));
}
void tearDown(void) {}

static void rx(const uint8_t *data, uint16_t len)
{
    uint8_t tmp[8];
    memcpy(tmp, data, len);
    isotp_on_can_message(&link, tmp, len);
}

static void rx_flow_control(uint8_t bs, uint8_t stmin)
{
    uint8_t fc[3] = { 0x30, bs, stmin };
    rx(fc, 3);
}

static void make_payload(uint8_t *out, int len)
{
    for (int i = 0; i < len; i++) out[i] = (uint8_t)(i + 1);
}

// ---- sending ----
void test_single_frame_send(void)
{
    uint8_t payload[3] = { 0x22, 0xF1, 0x90 };
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_send(&link, payload, sizeof(payload)));

    TEST_ASSERT_EQUAL(1, sent_count);
    TEST_ASSERT_EQUAL_HEX32(0x7E0, sent[0].id);
    TEST_ASSERT_EQUAL(8, sent[0].len);              // padded to a full frame
    TEST_ASSERT_EQUAL_HEX8(0x03, sent[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, &sent[0].data[1], 3);
    TEST_ASSERT_EQUAL(ISOTP_SEND_STATUS_IDLE, link.send_status);
}

void test_multi_frame_send_waits_for_flow_control(void)
{
    uint8_t payload[20];
    make_payload(payload, sizeof(payload));
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_send(&link, payload, sizeof(payload)));

    TEST_ASSERT_EQUAL(1, sent_count);
    TEST_ASSERT_EQUAL_HEX8(0x10, sent[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8(20, sent[0].data[1]);

    for (int i = 0; i < 5; i++) isotp_poll(&link);
    TEST_ASSERT_EQUAL_MESSAGE(1, sent_count, "consecutive frames must wait for flow control");

    rx_flow_control(0, 0);
    for (int i = 0; i < 10; i++) isotp_poll(&link);

    TEST_ASSERT_EQUAL(3, sent_count);               // FF + 2 CF
    TEST_ASSERT_EQUAL_HEX8(0x21, sent[1].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x22, sent[2].data[0]);
    TEST_ASSERT_EQUAL(ISOTP_SEND_STATUS_IDLE, link.send_status);

    uint8_t rebuilt[20];
    memcpy(rebuilt, &sent[0].data[2], 6);
    memcpy(rebuilt + 6, &sent[1].data[1], 7);
    memcpy(rebuilt + 13, &sent[2].data[1], 7);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, rebuilt, 20);
}

void test_block_size_pauses_until_next_flow_control(void)
{
    uint8_t payload[60];
    make_payload(payload, sizeof(payload));
    isotp_send(&link, payload, sizeof(payload));

    rx_flow_control(2, 0);                          // allow two consecutive frames
    for (int i = 0; i < 20; i++) isotp_poll(&link);
    TEST_ASSERT_EQUAL(3, sent_count);               // FF + 2 CF
    TEST_ASSERT_EQUAL(ISOTP_SEND_STATUS_INPROGRESS, link.send_status);

    rx_flow_control(0, 0);                          // unlimited
    for (int i = 0; i < 20; i++) isotp_poll(&link);
    TEST_ASSERT_EQUAL(ISOTP_SEND_STATUS_IDLE, link.send_status);
    TEST_ASSERT_EQUAL(1 + (60 - 6 + 6) / 7, sent_count);
}

void test_send_refused_while_busy_or_too_large(void)
{
    uint8_t payload[20];
    make_payload(payload, sizeof(payload));
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_send(&link, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL(ISOTP_RET_INPROGRESS, isotp_send(&link, payload, sizeof(payload)));

    setUp();
    uint8_t big[300] = { 0 };
    TEST_ASSERT_EQUAL(ISOTP_RET_OVERFLOW, isotp_send(&link, big, sizeof(big)));
}

// ---- STmin ----
void test_stmin_override_only_lengthens_gap(void)
{
    uint8_t payload[20];
    make_payload(payload, sizeof(payload));

    // receiver asks for 10 ms, app override is 1 ms -> 10 ms
    link.stmin_override = 1000;
    isotp_send(&link, payload, sizeof(payload));
    rx_flow_control(0, 0x0A);
    TEST_ASSERT_EQUAL_UINT32(10000, link.send_st_min);

    // receiver asks for nothing, override is 1 ms -> 1 ms
    setUp();
    link.stmin_override = 1000;
    isotp_send(&link, payload, sizeof(payload));
    rx_flow_control(0, 0x00);
    TEST_ASSERT_EQUAL_UINT32(1000, link.send_st_min);

    // no override: the receiver's value is used
    setUp();
    isotp_send(&link, payload, sizeof(payload));
    rx_flow_control(0, 0x05);
    TEST_ASSERT_EQUAL_UINT32(5000, link.send_st_min);

    // sub-millisecond encoding (0xF1-0xF9 = 100-900 us)
    setUp();
    isotp_send(&link, payload, sizeof(payload));
    rx_flow_control(0, 0xF5);
    TEST_ASSERT_EQUAL_UINT32(500, link.send_st_min);
}

void test_stmin_gap_is_enforced(void)
{
    uint8_t payload[20];
    make_payload(payload, sizeof(payload));
    isotp_send(&link, payload, sizeof(payload));
    rx_flow_control(0, 0x0A);                       // 10 ms

    isotp_poll(&link);
    TEST_ASSERT_EQUAL_MESSAGE(1, sent_count, "no frame before the gap has elapsed");

    now_us += 5000;
    isotp_poll(&link);
    TEST_ASSERT_EQUAL(1, sent_count);

    now_us += 6000;                                 // 11 ms after the flow control
    isotp_poll(&link);
    TEST_ASSERT_EQUAL(2, sent_count);
}

// ---- timeouts (regression: the response timeout was 100 us instead of 100 ms) ----
void test_send_timeout_is_100ms_not_100us(void)
{
    uint8_t payload[20];
    make_payload(payload, sizeof(payload));
    isotp_send(&link, payload, sizeof(payload));

    now_us += 50 * 1000;                            // 50 ms: the ECU may still answer
    isotp_poll(&link);
    TEST_ASSERT_EQUAL(ISOTP_SEND_STATUS_INPROGRESS, link.send_status);

    now_us += 60 * 1000;                            // 110 ms: give up
    isotp_poll(&link);
    TEST_ASSERT_EQUAL(ISOTP_SEND_STATUS_ERROR, link.send_status);
    TEST_ASSERT_EQUAL(ISOTP_PROTOCOL_RESULT_TIMEOUT_BS, link.send_protocol_result);
}

void test_receive_timeout_is_100ms_not_100us(void)
{
    uint8_t ff[8] = { 0x10, 20, 1, 2, 3, 4, 5, 6 };
    rx(ff, 8);
    TEST_ASSERT_EQUAL(ISOTP_RECEIVE_STATUS_INPROGRESS, link.receive_status);

    now_us += 50 * 1000;
    isotp_poll(&link);
    TEST_ASSERT_EQUAL(ISOTP_RECEIVE_STATUS_INPROGRESS, link.receive_status);

    now_us += 60 * 1000;
    isotp_poll(&link);
    TEST_ASSERT_EQUAL(ISOTP_RECEIVE_STATUS_IDLE, link.receive_status);
    TEST_ASSERT_EQUAL(ISOTP_PROTOCOL_RESULT_TIMEOUT_CR, link.receive_protocol_result);
}

// ---- receiving ----
void test_multi_frame_receive(void)
{
    uint8_t payload[20];
    make_payload(payload, sizeof(payload));

    uint8_t ff[8] = { 0x10, 20 };
    memcpy(&ff[2], payload, 6);
    rx(ff, 8);

    TEST_ASSERT_EQUAL(1, sent_count);               // we answer with flow control
    TEST_ASSERT_EQUAL_HEX32(0x7E0, sent[0].id);
    TEST_ASSERT_EQUAL_HEX8(0x30, sent[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x20, sent[0].data[1]);  // block size 32
    TEST_ASSERT_EQUAL_HEX8(0x00, sent[0].data[2]);  // STmin 0

    uint8_t cf1[8] = { 0x21 };
    memcpy(&cf1[1], payload + 6, 7);
    rx(cf1, 8);
    TEST_ASSERT_EQUAL(ISOTP_RECEIVE_STATUS_INPROGRESS, link.receive_status);

    uint8_t cf2[8] = { 0x22 };
    memcpy(&cf2[1], payload + 13, 7);
    rx(cf2, 8);
    TEST_ASSERT_EQUAL(ISOTP_RECEIVE_STATUS_FULL, link.receive_status);

    uint8_t out[64];
    uint16_t out_size = 0;
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_receive(&link, out, sizeof(out), &out_size));
    TEST_ASSERT_EQUAL(20, out_size);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, 20);
    TEST_ASSERT_EQUAL(ISOTP_RET_NO_DATA, isotp_receive(&link, out, sizeof(out), &out_size));
}

void test_single_frame_receive(void)
{
    uint8_t sf[8] = { 0x03, 0x62, 0xF1, 0x90, 0xAA, 0xAA, 0xAA, 0xAA };
    rx(sf, 8);

    uint8_t out[16];
    uint16_t out_size = 0;
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_receive(&link, out, sizeof(out), &out_size));
    TEST_ASSERT_EQUAL(3, out_size);
    TEST_ASSERT_EQUAL_HEX8(0x62, out[0]);
}

void test_wrong_sequence_number_aborts_receive(void)
{
    uint8_t ff[8] = { 0x10, 20, 1, 2, 3, 4, 5, 6 };
    rx(ff, 8);

    uint8_t bad[8] = { 0x23, 7, 8, 9, 10, 11, 12, 13 };     // should have been 0x21
    rx(bad, 8);
    TEST_ASSERT_EQUAL(ISOTP_RECEIVE_STATUS_IDLE, link.receive_status);
    TEST_ASSERT_EQUAL(ISOTP_PROTOCOL_RESULT_WRONG_SN, link.receive_protocol_result);
}

void test_oversized_response_is_refused(void)
{
    uint8_t ff[8] = { 0x1F, 0xFF, 1, 2, 3, 4, 5, 6 };       // 4095 bytes, buffer is 256
    rx(ff, 8);
    TEST_ASSERT_EQUAL(ISOTP_RECEIVE_STATUS_IDLE, link.receive_status);
    TEST_ASSERT_EQUAL(ISOTP_PROTOCOL_RESULT_BUFFER_OVFLW, link.receive_protocol_result);
    TEST_ASSERT_EQUAL_HEX8(0x32, sent[0].data[0]);          // flow control: overflow
}


// ---- two links talking to each other: what the bench simulator does between the dongle and a virtual module ----
static IsoTpLink    module_link;
static uint8_t      module_send_buf[1024];
static uint8_t      module_recv_buf[1024];
static uint8_t      big_link_send[1024];
static uint8_t      big_link_recv[1024];

// Deliver every frame sent so far to the link it is addressed to, polling both until nothing is left to do
static void pump_links(void)
{
    int delivered = 0;
    for (int guard = 0; guard < 20000; guard++) {
        bool progress = false;
        while (delivered < sent_count) {
            sent_frame_t f = sent[delivered++];
            uint8_t data[8];
            memcpy(data, f.data, f.len);
            if (f.id == 0x7E0) isotp_on_can_message(&module_link, data, f.len);
            else if (f.id == 0x7E8) isotp_on_can_message(&link, data, f.len);
            progress = true;
        }
        isotp_poll(&link);
        isotp_poll(&module_link);
        now_us += 200;
        if (!progress && delivered == sent_count &&
            link.send_status != ISOTP_SEND_STATUS_INPROGRESS && module_link.send_status != ISOTP_SEND_STATUS_INPROGRESS &&
            link.receive_status != ISOTP_RECEIVE_STATUS_INPROGRESS && module_link.receive_status != ISOTP_RECEIVE_STATUS_INPROGRESS) {
            break;
        }
    }
}

void test_request_and_response_between_two_links(void)
{
    sent_count = 0;
    isotp_init_link(&link, 0x7E0, 0x7E8, big_link_send, sizeof(big_link_send), big_link_recv, sizeof(big_link_recv));
    isotp_init_link(&module_link, 0x7E8, 0x7E0, module_send_buf, sizeof(module_send_buf), module_recv_buf, sizeof(module_recv_buf));
    link.stmin_override = 1000;                     // the app's 1 ms setting

    uint8_t request[69];
    make_payload(request, sizeof(request));
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_send(&link, request, sizeof(request)));
    pump_links();

    uint8_t got[1024];
    uint16_t got_len = 0;
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_receive(&module_link, got, sizeof(got), &got_len));
    TEST_ASSERT_EQUAL(69, got_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(request, got, 69);

    uint8_t response[132];
    for (int i = 0; i < 132; i++) response[i] = (uint8_t)(i * 3);
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_send(&module_link, response, sizeof(response)));
    pump_links();

    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_receive(&link, got, sizeof(got), &got_len));
    TEST_ASSERT_EQUAL(132, got_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(response, got, 132);
    TEST_ASSERT_EQUAL(ISOTP_PROTOCOL_RESULT_OK, link.send_protocol_result);
    TEST_ASSERT_EQUAL(ISOTP_PROTOCOL_RESULT_OK, link.receive_protocol_result);
}

void test_large_exchange_with_block_size_limit(void)
{
    sent_count = 0;
    isotp_init_link(&link, 0x7E0, 0x7E8, big_link_send, sizeof(big_link_send), big_link_recv, sizeof(big_link_recv));
    isotp_init_link(&module_link, 0x7E8, 0x7E0, module_send_buf, sizeof(module_send_buf), module_recv_buf, sizeof(module_recv_buf));
    link.default_block_size = 4;                    // we ask the module for a flow control every 4 frames
    module_link.default_block_size = 4;

    uint8_t request[369];                           // the biggest request seen from Simos Tools
    make_payload(request, sizeof(request));
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_send(&link, request, sizeof(request)));
    pump_links();

    uint8_t got[1024];
    uint16_t got_len = 0;
    TEST_ASSERT_EQUAL(ISOTP_RET_OK, isotp_receive(&module_link, got, sizeof(got), &got_len));
    TEST_ASSERT_EQUAL(369, got_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(request, got, 369);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_single_frame_send);
    RUN_TEST(test_multi_frame_send_waits_for_flow_control);
    RUN_TEST(test_block_size_pauses_until_next_flow_control);
    RUN_TEST(test_send_refused_while_busy_or_too_large);
    RUN_TEST(test_stmin_override_only_lengthens_gap);
    RUN_TEST(test_stmin_gap_is_enforced);
    RUN_TEST(test_send_timeout_is_100ms_not_100us);
    RUN_TEST(test_receive_timeout_is_100ms_not_100us);
    RUN_TEST(test_multi_frame_receive);
    RUN_TEST(test_single_frame_receive);
    RUN_TEST(test_wrong_sequence_number_aborts_receive);
    RUN_TEST(test_oversized_response_is_refused);
    RUN_TEST(test_request_and_response_between_two_links);
    RUN_TEST(test_large_exchange_with_block_size_limit);
    return UNITY_END();
}
