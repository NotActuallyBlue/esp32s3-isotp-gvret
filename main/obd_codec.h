#ifndef OBD_CODEC_H
#define OBD_CODEC_H

// Pure decoding helpers for OBD-II / UDS diagnostics. No hardware or RTOS dependencies, so they are
// covered by the host tests (pio test -e native).

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define OBD_MAX_DTCS    64

// DTC status byte (ISO 14229-1), as returned by UDS 0x19
#define DTC_STATUS_TEST_FAILED      0x01
#define DTC_STATUS_PENDING          0x04
#define DTC_STATUS_CONFIRMED        0x08
#define DTC_STATUS_WARNING_LAMP     0x80

typedef struct {
    uint16_t    code;       // two-byte SAE code (first two bytes of the DTC)
    uint8_t     fault_type; // third byte of a UDS DTC (0 for OBD-II mode 03/07/0A)
    uint8_t     status;     // UDS status byte (0 for OBD-II modes, where the mode implies the state)
    bool        has_status;
} obd_dtc_t;

// "P0300", "U0100", "B1234", "C0035". out needs 6 bytes.
void obd_format_dtc(uint16_t code, char *out);

// "P0300-1F". out needs 9 bytes.
void obd_format_uds_dtc(const obd_dtc_t *dtc, char *out);

// VAG's five digit fault number for P-codes (P0300 = 16684). Returns 0 for other classes.
uint16_t obd_vag_fault_number(uint16_t code);

// Short flag text for a UDS status byte, e.g. "ACT MIL", "STORED", "PEND". out needs 16 bytes.
void obd_format_status(uint8_t status, char *out, size_t out_size);

// OBD-II mode 03 / 07 / 0A response: [mode+0x40][count?] then two bytes per DTC. CAN responses carry a
// count byte after the mode. Zero pairs (0x0000) are padding. Returns the number of DTCs written.
int obd_parse_mode_dtcs(const uint8_t *resp, size_t len, obd_dtc_t *out, int max);

// UDS status bits that mean a real fault: test failed (0x01), failed this cycle (0x02), pending (0x04), confirmed
// (0x08), failed since last clear (0x20), warning lamp requested (0x80). The "test not completed" bits (0x10, 0x40) say
// only that a monitor has not run yet: VAG modules list every code they know with those set, so they must not count.
#define DTC_STATUS_FAULT_MASK       0xAF
static inline bool obd_status_is_fault(uint8_t status) { return (status & DTC_STATUS_FAULT_MASK) != 0; }

// Drop the entries that are not faults, in place. Returns the new count.
int obd_keep_faults(obd_dtc_t *list, int count);

// UDS 0x19 0x02 response: 59 02 <availability> then (3 byte DTC + status byte) repeated.
int obd_parse_uds_dtcs(const uint8_t *resp, size_t len, obd_dtc_t *out, int max);

// Mode 01 PID 01 (monitor status): MIL, count and readiness. data points at the four bytes A B C D.
typedef struct {
    bool    mil_on;
    uint8_t dtc_count;
    bool    compression_ignition;
    uint16_t supported;         // supported monitors (bit n = monitor n of obd_readiness_name)
    uint16_t incomplete;        // supported and not yet complete
} obd_readiness_t;

#define OBD_READINESS_COUNT 11
const char *obd_readiness_name(int index, bool compression_ignition);
void obd_parse_readiness(const uint8_t *abcd, obd_readiness_t *out);

// Live data PIDs we can show (mode 01). Returns false if the PID or length is not handled.
bool obd_format_pid(uint8_t pid, const uint8_t *data, size_t len, char *out, size_t out_size);
const char *obd_pid_name(uint8_t pid);

// Extract the 17 character VIN from a mode 09 PID 02 response (49 02 01 <vin>). Returns false if absent.
bool obd_parse_vin(const uint8_t *resp, size_t len, char *vin, size_t vin_size);

#endif // OBD_CODEC_H
