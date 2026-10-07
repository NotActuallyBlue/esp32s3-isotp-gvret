// Host-side tests for the GVRET command reader (SavvyCAN protocol). Run with: pio test -e native
#include <unity.h>
#include <string.h>
#include "../../main/gvret_parser.c"

static uint8_t  out[64];
static size_t   out_len;
static int      tx_count;
static uint32_t tx_id;
static bool     tx_ext;
static uint8_t  tx_dlc;
static uint8_t  tx_data[8];

static void on_write(const uint8_t *data, size_t len) { memcpy(out + out_len, data, len); out_len += len; }
static void on_tx(uint32_t id, bool ext, uint8_t dlc, const uint8_t *data)
{
    tx_count++; tx_id = id; tx_ext = ext; tx_dlc = dlc; memcpy(tx_data, data, dlc);
}
static uint32_t on_now(void) { return 0x01020304; }

static gvret_parser_t p;

void setUp(void)
{
    memset(&p, 0, sizeof(p));
    p.write = on_write; p.transmit = on_tx; p.now_us = on_now;
    gvret_parser_reset(&p);
    out_len = 0; tx_count = 0;
}
void tearDown(void) {}

static void feed(const uint8_t *d, size_t n) { gvret_parser_feed(&p, d, n); }

void test_frame_is_transmitted(void)
{
    const uint8_t cmd[] = { 0xF1, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x00, 0x03, 0x02, 0x10, 0x03, 0xAA };
    feed(cmd, sizeof(cmd));
    TEST_ASSERT_EQUAL(1, tx_count);
    TEST_ASSERT_EQUAL_HEX32(0x7E0, tx_id);
    TEST_ASSERT_FALSE(tx_ext);
    TEST_ASSERT_EQUAL(3, tx_dlc);
    TEST_ASSERT_EQUAL_HEX8(0x10, tx_data[1]);
}

void test_frame_split_across_reads(void)
{
    const uint8_t cmd[] = { 0xF1, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x11, 0x22 };
    feed(cmd, 4);
    TEST_ASSERT_EQUAL(0, tx_count);
    feed(cmd + 4, 3);
    TEST_ASSERT_EQUAL(0, tx_count);
    feed(cmd + 7, 3);
    TEST_ASSERT_EQUAL(1, tx_count);
    TEST_ASSERT_EQUAL_HEX32(0x100, tx_id);
}

void test_extended_id(void)
{
    const uint8_t cmd[] = { 0xF1, 0x00, 0x78, 0x56, 0x34, 0x92, 0x00, 0x01, 0x55 };   // 0x12345678 with the extended flag
    feed(cmd, sizeof(cmd));
    TEST_ASSERT_EQUAL(1, tx_count);
    TEST_ASSERT_TRUE(tx_ext);
    TEST_ASSERT_EQUAL_HEX32(0x12345678, tx_id);
}

void test_standard_id_too_large_is_dropped(void)
{
    const uint8_t cmd[] = { 0xF1, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x55 };   // 0x800 does not fit 11 bits
    feed(cmd, sizeof(cmd));
    TEST_ASSERT_EQUAL(0, tx_count);
}

void test_other_bus_is_dropped(void)
{
    const uint8_t cmd[] = { 0xF1, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x55 };   // bus 1
    feed(cmd, sizeof(cmd));
    TEST_ASSERT_EQUAL(0, tx_count);
}

// The data of a bus-speed command must not be read as commands, even when it contains F1 00
void test_setup_canbus_payload_is_skipped(void)
{
    const uint8_t cmd[] = { 0xF1, 0x05, 0xF1, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00 };
    feed(cmd, sizeof(cmd));
    TEST_ASSERT_EQUAL(0, tx_count);
    TEST_ASSERT_EQUAL(0, out_len);
}

void test_command_after_skipped_payload_still_works(void)
{
    const uint8_t cmd[] = { 0xF1, 0x05, 1, 2, 3, 4, 5, 6, 7, 8, 0xF1, 0x0C };
    feed(cmd, sizeof(cmd));
    TEST_ASSERT_EQUAL(3, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x0C, out[1]);
    TEST_ASSERT_EQUAL_HEX8(1, out[2]);
}

void test_garbage_between_commands_is_ignored(void)
{
    const uint8_t cmd[] = { 0xE7, 0x13, 0x77, 0xF1, 0x0C, 0xE7, 0xE7 };
    feed(cmd, sizeof(cmd));
    TEST_ASSERT_EQUAL(3, out_len);
}

void test_time_sync_uses_clock(void)
{
    const uint8_t cmd[] = { 0xF1, 0x01 };
    feed(cmd, sizeof(cmd));
    TEST_ASSERT_EQUAL(6, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x04, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[5]);
}

void test_bus_parameters_say_500k(void)
{
    const uint8_t cmd[] = { 0xF1, 0x06 };
    feed(cmd, sizeof(cmd));
    TEST_ASSERT_EQUAL(7, out_len);
    TEST_ASSERT_EQUAL_HEX8(0x20, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0xA1, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x07, out[5]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_frame_is_transmitted);
    RUN_TEST(test_frame_split_across_reads);
    RUN_TEST(test_extended_id);
    RUN_TEST(test_standard_id_too_large_is_dropped);
    RUN_TEST(test_other_bus_is_dropped);
    RUN_TEST(test_setup_canbus_payload_is_skipped);
    RUN_TEST(test_command_after_skipped_payload_still_works);
    RUN_TEST(test_garbage_between_commands_is_ignored);
    RUN_TEST(test_time_sync_uses_clock);
    RUN_TEST(test_bus_parameters_say_500k);
    return UNITY_END();
}
