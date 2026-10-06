// Host-side tests for the OBD/UDS decoders and the ELM327 interpreter. Run with: pio test -e native
#include <unity.h>
#include <string.h>
#include <stdio.h>
#include "../../main/obd_codec.c"
#include "../../main/elm327.c"

// Same ranges as diag_can.c (that file needs the RTOS, so it is not part of the host build)
void diag_default_rx_range(uint32_t tx_id, uint32_t *lo, uint32_t *hi)
{
    if (tx_id == DIAG_FUNCTIONAL_ID)               { *lo = 0x7E8; *hi = 0x7EF; }
    else if (tx_id >= 0x7E0 && tx_id <= 0x7E7)     { *lo = *hi = tx_id + 8; }
    else if (tx_id >= 0x700 && tx_id <= 0x77F)     { *lo = *hi = tx_id + 0x6A; }
    else                                           { *lo = 0x600; *hi = 0x7FF; }
}

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------- codec
static void test_dtc_names(void)
{
    char s[8];
    obd_format_dtc(0x0300, s); TEST_ASSERT_EQUAL_STRING("P0300", s);
    obd_format_dtc(0x1234, s); TEST_ASSERT_EQUAL_STRING("P1234", s);
    obd_format_dtc(0x4035, s); TEST_ASSERT_EQUAL_STRING("C0035", s);
    obd_format_dtc(0x8123, s); TEST_ASSERT_EQUAL_STRING("B0123", s);
    obd_format_dtc(0xC100, s); TEST_ASSERT_EQUAL_STRING("U0100", s);
    obd_format_dtc(0xF3FF, s); TEST_ASSERT_EQUAL_STRING("U33FF", s);
}

static void test_vag_number(void)
{
    TEST_ASSERT_EQUAL_UINT16(16684, obd_vag_fault_number(0x0300));
    TEST_ASSERT_EQUAL_UINT16(16555, obd_vag_fault_number(0x0171));
    TEST_ASSERT_EQUAL_UINT16(18386, obd_vag_fault_number(0x2002));
    TEST_ASSERT_EQUAL_UINT16(0, obd_vag_fault_number(0xC100));      // not a P-code
    TEST_ASSERT_EQUAL_UINT16(0, obd_vag_fault_number(0x00AF));      // hex digits have no decimal number
}

static void test_mode_dtcs(void)
{
    obd_dtc_t d[8];
    const uint8_t with_count[] = { 0x43, 0x02, 0x03, 0x00, 0x01, 0x71 };
    TEST_ASSERT_EQUAL_INT(2, obd_parse_mode_dtcs(with_count, sizeof(with_count), d, 8));
    TEST_ASSERT_EQUAL_UINT16(0x0300, d[0].code);
    TEST_ASSERT_EQUAL_UINT16(0x0171, d[1].code);

    const uint8_t no_count[] = { 0x43, 0x03, 0x00 };
    TEST_ASSERT_EQUAL_INT(1, obd_parse_mode_dtcs(no_count, sizeof(no_count), d, 8));
    TEST_ASSERT_EQUAL_UINT16(0x0300, d[0].code);

    const uint8_t padded[] = { 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    TEST_ASSERT_EQUAL_INT(0, obd_parse_mode_dtcs(padded, sizeof(padded), d, 8));

    const uint8_t none[] = { 0x43, 0x00 };
    TEST_ASSERT_EQUAL_INT(0, obd_parse_mode_dtcs(none, sizeof(none), d, 8));
}

static void test_uds_dtcs(void)
{
    obd_dtc_t d[8];
    const uint8_t r[] = { 0x59, 0x02, 0xFF, 0x03, 0x00, 0x1F, 0x89, 0x01, 0x71, 0x00, 0x08 };
    TEST_ASSERT_EQUAL_INT(2, obd_parse_uds_dtcs(r, sizeof(r), d, 8));
    TEST_ASSERT_EQUAL_UINT16(0x0300, d[0].code);
    TEST_ASSERT_EQUAL_UINT8(0x1F, d[0].fault_type);
    TEST_ASSERT_EQUAL_UINT8(0x89, d[0].status);
    TEST_ASSERT_EQUAL_UINT8(0x08, d[1].status);

    char s[16];
    obd_format_uds_dtc(&d[0], s);
    TEST_ASSERT_EQUAL_STRING("P0300-1F", s);
    obd_format_status(0x89, s, sizeof(s)); TEST_ASSERT_EQUAL_STRING("ACT MIL", s);
    obd_format_status(0x08, s, sizeof(s)); TEST_ASSERT_EQUAL_STRING("STORED", s);
    obd_format_status(0x04, s, sizeof(s)); TEST_ASSERT_EQUAL_STRING("PEND", s);
    obd_format_status(0x20, s, sizeof(s)); TEST_ASSERT_EQUAL_STRING("HIST", s);          // failed since last clear, not failing now
    obd_format_status(0x02, s, sizeof(s)); TEST_ASSERT_EQUAL_STRING("THIS", s);          // failed in this drive
    obd_format_status(0x42, s, sizeof(s)); TEST_ASSERT_EQUAL_STRING("THIS", s);

    const uint8_t negative[] = { 0x7F, 0x19, 0x12 };
    TEST_ASSERT_EQUAL_INT(0, obd_parse_uds_dtcs(negative, sizeof(negative), d, 8));
    const uint8_t truncated[] = { 0x59, 0x02, 0xFF, 0x03, 0x00, 0x1F };      // incomplete record is ignored
    TEST_ASSERT_EQUAL_INT(0, obd_parse_uds_dtcs(truncated, sizeof(truncated), d, 8));
}

static void test_only_real_faults_are_kept(void)
{
    // Real statuses seen from a car: "test not completed" entries must not count as codes
    TEST_ASSERT_FALSE(obd_status_is_fault(0x10));
    TEST_ASSERT_FALSE(obd_status_is_fault(0x40));
    TEST_ASSERT_FALSE(obd_status_is_fault(0x50));
    TEST_ASSERT_TRUE(obd_status_is_fault(0x08));       // confirmed
    TEST_ASSERT_TRUE(obd_status_is_fault(0x09));       // active and confirmed
    TEST_ASSERT_TRUE(obd_status_is_fault(0x04));       // pending
    TEST_ASSERT_TRUE(obd_status_is_fault(0x28));       // failed since last clear, confirmed
    TEST_ASSERT_TRUE(obd_status_is_fault(0x89));       // lamp on

    obd_dtc_t d[4] = {
        { 0x0101, 0x07, 0x09, true }, { 0x003A, 0xFD, 0x40, true }, { 0x0053, 0x8C, 0x50, true }, { 0x0002, 0x35, 0x08, true },
    };
    TEST_ASSERT_EQUAL_INT(2, obd_keep_faults(d, 4));
    TEST_ASSERT_EQUAL_UINT16(0x0101, d[0].code);
    TEST_ASSERT_EQUAL_UINT16(0x0002, d[1].code);

    obd_dtc_t plain = { 0x0700, 0, 0, false };        // OBD mode 03 entries carry no status and are always kept
    TEST_ASSERT_EQUAL_INT(1, obd_keep_faults(&plain, 1));
}

static void test_readiness(void)
{
    const uint8_t abcd[] = { 0x82, 0x07, 0x65, 0x04 };      // MIL on, 2 DTCs, EVAP incomplete
    obd_readiness_t r;
    obd_parse_readiness(abcd, &r);
    TEST_ASSERT_TRUE(r.mil_on);
    TEST_ASSERT_EQUAL_UINT8(2, r.dtc_count);
    TEST_ASSERT_FALSE(r.compression_ignition);
    TEST_ASSERT_EQUAL_UINT16((1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | (1u << 5) | (1u << 8) | (1u << 9), r.supported);
    TEST_ASSERT_EQUAL_UINT16(1u << 5, r.incomplete);
    TEST_ASSERT_EQUAL_STRING("EVAP", obd_readiness_name(5, false));
}

static void test_pid_format_and_vin(void)
{
    char s[24];
    const uint8_t rpm[] = { 0x1A, 0xF8 };
    TEST_ASSERT_TRUE(obd_format_pid(0x0C, rpm, 2, s, sizeof(s)));
    TEST_ASSERT_EQUAL_STRING("1726 rpm", s);
    const uint8_t coolant[] = { 128 };
    TEST_ASSERT_TRUE(obd_format_pid(0x05, coolant, 1, s, sizeof(s)));
    TEST_ASSERT_EQUAL_STRING("88 C", s);
    const uint8_t volts[] = { 0x35, 0xE8 };
    TEST_ASSERT_TRUE(obd_format_pid(0x42, volts, 2, s, sizeof(s)));
    TEST_ASSERT_EQUAL_STRING("13.80 V", s);
    TEST_ASSERT_FALSE(obd_format_pid(0x0C, rpm, 1, s, sizeof(s)));       // too short
    TEST_ASSERT_FALSE(obd_format_pid(0xEE, rpm, 2, s, sizeof(s)));       // unknown

    uint8_t resp[20] = { 0x49, 0x02, 0x01 };
    memcpy(resp + 3, "SIMULATEDVIN00001", 17);
    char vin[20];
    TEST_ASSERT_TRUE(obd_parse_vin(resp, sizeof(resp), vin, sizeof(vin)));
    TEST_ASSERT_EQUAL_STRING("SIMULATEDVIN00001", vin);
    const uint8_t bad[] = { 0x7F, 0x09, 0x12 };
    TEST_ASSERT_FALSE(obd_parse_vin(bad, sizeof(bad), vin, sizeof(vin)));
}

// ---------------------------------------------------------------- ELM327
static char     output[4096];
static uint32_t last_tx, last_lo, last_hi;
static int      requests;
static uint8_t  last_request[16];
static uint16_t last_request_len;
static int      canned_count;                   // what the fake bus answers
static diag_response_t canned[DIAG_MAX_RESPONSES];

static int fake_request(void *ctx, uint32_t tx, uint32_t lo, uint32_t hi, const uint8_t *req, uint16_t len,
                        uint32_t timeout_ms, diag_response_t *out, int out_max)
{
    (void)ctx; (void)timeout_ms;
    requests++;
    last_tx = tx; last_lo = lo; last_hi = hi;
    last_request_len = len;
    memcpy(last_request, req, len < sizeof(last_request) ? len : sizeof(last_request));
    for (int i = 0; i < canned_count && i < out_max; i++) out[i] = canned[i];
    return canned_count;
}
static void fake_write(void *ctx, const char *text) { (void)ctx; strlcat(output, text, sizeof(output)); }

static const elm_ops_t ops = { .request = fake_request, .write = fake_write };
static elm_t elm;

static void start(void)
{
    output[0] = 0;
    requests = 0;
    canned_count = 0;
    memset(canned, 0, sizeof(canned));
    TEST_ASSERT_TRUE(elm_init(&elm, &ops, NULL));
}
static void stop(void) { elm_free(&elm); }

static void send(const char *text) { elm_input(&elm, (const uint8_t *)text, strlen(text)); }

static void set_canned(int i, uint32_t id, const uint8_t *data, uint16_t len)
{
    canned[i].id = id;
    canned[i].len = len;
    memcpy(canned[i].data, data, len);
    if (i >= canned_count) canned_count = i + 1;
}

static void test_elm_reset_and_settings(void)
{
    start();
    send("ATZ\r");
    TEST_ASSERT_NOT_NULL(strstr(output, "ELM327 v1.5"));
    TEST_ASSERT_EQUAL_CHAR('>', output[strlen(output) - 1]);

    output[0] = 0;
    send("ATE0\r");
    TEST_ASSERT_NOT_NULL(strstr(output, "OK"));
    output[0] = 0;
    send("ATL0\r");
    output[0] = 0;
    send("ATI\r");
    TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r\r>", output);       // echo off, linefeeds off

    output[0] = 0;
    send("ATDPN\r");
    TEST_ASSERT_EQUAL_STRING("A6\r\r>", output);
    output[0] = 0;
    send("AT SP 6\r");
    send("ATDPN\r");
    TEST_ASSERT_NOT_NULL(strstr(output, "OK\r\r>6\r\r>"));
    output[0] = 0;
    send("ATSP7\r");
    TEST_ASSERT_EQUAL_STRING("?\r\r>", output);
    output[0] = 0;
    send("ATFOO\r");
    TEST_ASSERT_EQUAL_STRING("?\r\r>", output);
    stop();
}

static void test_elm_obd_single_frame(void)
{
    start();
    send("ATE0\rATL0\r");
    output[0] = 0;
    const uint8_t pids[] = { 0x41, 0x00, 0x98, 0x3A, 0x80, 0x01 };
    set_canned(0, 0x7E8, pids, sizeof(pids));
    send("0100\r");
    TEST_ASSERT_EQUAL_STRING("SEARCHING...\r41 00 98 3A 80 01\r\r>", output);
    TEST_ASSERT_EQUAL_UINT32(0x7DF, last_tx);
    TEST_ASSERT_EQUAL_UINT32(0x7E8, last_lo);
    TEST_ASSERT_EQUAL_UINT32(0x7EF, last_hi);
    TEST_ASSERT_EQUAL_UINT16(2, last_request_len);

    output[0] = 0;                                  // no searching message once connected
    send("0100\r");
    TEST_ASSERT_EQUAL_STRING("41 00 98 3A 80 01\r\r>", output);

    output[0] = 0;
    send("ATS0\r");
    output[0] = 0;
    send("0100\r");
    TEST_ASSERT_EQUAL_STRING("4100983A8001\r\r>", output);
    stop();
}

static void test_elm_headers_and_multi_frame(void)
{
    start();
    send("ATE0\rATL0\rATS1\rATH0\r");
    uint8_t vin[20] = { 0x49, 0x02, 0x01 };
    memcpy(vin + 3, "SIMULATEDVIN00001", 17);
    set_canned(0, 0x7E8, vin, sizeof(vin));

    output[0] = 0;
    send("0902\r");
    TEST_ASSERT_EQUAL_STRING("014\r0: 49 02 01 53 49 4D\r1: 55 4C 41 54 45 44 56\r2: 49 4E 30 30 30 30 31\r\r>", output);

    output[0] = 0;
    send("ATH1\r");
    output[0] = 0;
    send("0902\r");
    TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 53 49 4D\r7E8 21 55 4C 41 54 45 44 56\r7E8 22 49 4E 30 30 30 30 31\r\r>", output);

    output[0] = 0;
    const uint8_t rpm[] = { 0x41, 0x0C, 0x1A, 0xF8 };
    set_canned(0, 0x7E8, rpm, sizeof(rpm));
    canned_count = 1;
    send("010C\r");
    TEST_ASSERT_EQUAL_STRING("7E8 04 41 0C 1A F8\r\r>", output);
    stop();
}

static void test_elm_two_ecus_and_no_data(void)
{
    start();
    send("ATE0\rATL0\r");
    const uint8_t a[] = { 0x41, 0x0D, 0x32 };
    const uint8_t b[] = { 0x41, 0x0D, 0x33 };
    set_canned(0, 0x7E8, a, sizeof(a));
    set_canned(1, 0x7E9, b, sizeof(b));
    output[0] = 0;
    send("010D\r");
    TEST_ASSERT_EQUAL_STRING("41 0D 32\r41 0D 33\r\r>", output);

    canned_count = 0;
    output[0] = 0;
    send("010D\r");
    TEST_ASSERT_EQUAL_STRING("NO DATA\r\r>", output);
    stop();
}

static void test_elm_unable_to_connect(void)
{
    start();
    send("ATE0\rATL0\r");
    output[0] = 0;
    send("0100\r");
    TEST_ASSERT_EQUAL_STRING("SEARCHING...\rUNABLE TO CONNECT\r\r>", output);
    stop();
}

static void test_elm_header_and_filter(void)
{
    start();
    send("ATE0\rATL0\r");
    output[0] = 0;
    send("ATSH7E1\r");
    TEST_ASSERT_EQUAL_STRING("OK\r\r>", output);
    send("1003\r");
    TEST_ASSERT_EQUAL_UINT32(0x7E1, last_tx);
    TEST_ASSERT_EQUAL_UINT32(0x7E9, last_lo);
    TEST_ASSERT_EQUAL_UINT32(0x7E9, last_hi);

    send("ATSH710\r");                                  // VAG style address answers on +0x6A
    send("3E00\r");
    TEST_ASSERT_EQUAL_UINT32(0x77A, last_lo);

    send("ATCRA77D\r");                                 // an explicit filter wins
    send("3E00\r");
    TEST_ASSERT_EQUAL_UINT32(0x77D, last_lo);
    TEST_ASSERT_EQUAL_UINT32(0x77D, last_hi);
    send("ATCRA\r");
    send("3E00\r");
    TEST_ASSERT_EQUAL_UINT32(0x77A, last_lo);

    output[0] = 0;
    send("ATSH12345678\r");                             // 29 bit ids are not supported
    TEST_ASSERT_EQUAL_STRING("?\r\r>", output);
    stop();
}

static void test_elm_line_handling(void)
{
    start();
    send("ATE0\rATL0\r");
    const uint8_t r[] = { 0x41, 0x05, 0x7B };
    set_canned(0, 0x7E8, r, sizeof(r));
    output[0] = 0;

    send("01");                                          // a command can arrive in pieces
    send(" 05\r");
    TEST_ASSERT_EQUAL_INT(1, requests);
    TEST_ASSERT_EQUAL_UINT8(0x01, last_request[0]);
    TEST_ASSERT_EQUAL_UINT8(0x05, last_request[1]);

    output[0] = 0;
    send("\r");                                         // an empty line repeats it
    TEST_ASSERT_EQUAL_INT(2, requests);
    TEST_ASSERT_NOT_NULL(strstr(output, "41 05 7B"));

    send("0105\r\n");                                  // CR LF does not make a second command
    TEST_ASSERT_EQUAL_INT(3, requests);

    send("0105 1\r");                                   // trailing digit = expected responses, not data
    TEST_ASSERT_EQUAL_UINT16(2, last_request_len);

    output[0] = 0;
    send("ZZZZ\r");
    TEST_ASSERT_EQUAL_STRING("?\r\r>", output);
    stop();
}

static void test_elm_battery_voltage(void)
{
    start();
    send("ATE0\rATL0\r");
    const uint8_t v[] = { 0x41, 0x42, 0x35, 0xE8 };
    set_canned(0, 0x7E8, v, sizeof(v));
    output[0] = 0;
    send("ATRV\r");
    TEST_ASSERT_EQUAL_STRING("13.8V\r\r>", output);
    TEST_ASSERT_EQUAL_UINT8(0x42, last_request[1]);

    canned_count = 0;
    output[0] = 0;
    send("ATRV\r");
    TEST_ASSERT_EQUAL_STRING("0.0V\r\r>", output);
    stop();
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_dtc_names);
    RUN_TEST(test_vag_number);
    RUN_TEST(test_mode_dtcs);
    RUN_TEST(test_uds_dtcs);
    RUN_TEST(test_only_real_faults_are_kept);
    RUN_TEST(test_readiness);
    RUN_TEST(test_pid_format_and_vin);
    RUN_TEST(test_elm_reset_and_settings);
    RUN_TEST(test_elm_obd_single_frame);
    RUN_TEST(test_elm_headers_and_multi_frame);
    RUN_TEST(test_elm_two_ecus_and_no_data);
    RUN_TEST(test_elm_unable_to_connect);
    RUN_TEST(test_elm_header_and_filter);
    RUN_TEST(test_elm_line_handling);
    RUN_TEST(test_elm_battery_voltage);
    return UNITY_END();
}
