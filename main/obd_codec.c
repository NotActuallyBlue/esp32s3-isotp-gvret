#include <stdio.h>
#include <string.h>
#include "obd_codec.h"

void obd_format_dtc(uint16_t code, char *out)
{
    static const char klass[4] = { 'P', 'C', 'B', 'U' };
    snprintf(out, 6, "%c%X%X%X%X", klass[(code >> 14) & 3], (code >> 12) & 3, (code >> 8) & 0xF, (code >> 4) & 0xF, code & 0xF);
}

void obd_format_uds_dtc(const obd_dtc_t *dtc, char *out)
{
    char base[6];
    obd_format_dtc(dtc->code, base);
    snprintf(out, 9, "%s-%02X", base, dtc->fault_type);
}

uint16_t obd_vag_fault_number(uint16_t code)
{
    if ((code >> 14) != 0) return 0;                        // P-codes only
    unsigned d3 = (code >> 12) & 3, d2 = (code >> 8) & 0xF, d1 = (code >> 4) & 0xF, d0 = code & 0xF;
    if (d2 > 9 || d1 > 9 || d0 > 9) return 0;               // hex digits have no decimal fault number
    return (uint16_t)(16384 + d3 * 1000 + d2 * 100 + d1 * 10 + d0);
}

void obd_format_status(uint8_t status, char *out, size_t out_size)
{
    out[0] = 0;
    // Most important first: failing now, failed in this drive, pending, confirmed, otherwise only history
    if (status & DTC_STATUS_TEST_FAILED)        strlcat(out, "ACT ", out_size);
    else if (status & 0x02)                     strlcat(out, "THIS ", out_size);
    else if (status & DTC_STATUS_PENDING)       strlcat(out, "PEND ", out_size);
    else if (status & DTC_STATUS_CONFIRMED)     strlcat(out, "STORED ", out_size);
    else if (status & 0x20)                     strlcat(out, "HIST ", out_size);
    else                                        strlcat(out, "STORED ", out_size);
    if (status & DTC_STATUS_WARNING_LAMP)       strlcat(out, "MIL", out_size);
    size_t n = strlen(out);
    if (n && out[n - 1] == ' ') out[n - 1] = 0;
}

int obd_parse_mode_dtcs(const uint8_t *resp, size_t len, obd_dtc_t *out, int max)
{
    // [mode+0x40][count] then pairs. Some ECUs (and non-CAN protocols) omit the count byte, so use the
    // length to decide: with the count byte the remaining bytes are odd.
    if (len < 1) return 0;
    size_t pos = 1;
    if (((len - 1) & 1) == 1) pos = 2;

    int n = 0;
    for (; pos + 1 < len && n < max; pos += 2) {
        uint16_t code = ((uint16_t)resp[pos] << 8) | resp[pos + 1];
        if (code == 0) continue;
        out[n].code = code;
        out[n].fault_type = 0;
        out[n].status = 0;
        out[n].has_status = false;
        n++;
    }
    return n;
}

int obd_keep_faults(obd_dtc_t *list, int count)
{
    int n = 0;
    for (int i = 0; i < count; i++) {
        if (list[i].has_status && !obd_status_is_fault(list[i].status)) continue;
        list[n++] = list[i];
    }
    return n;
}

int obd_parse_uds_dtcs(const uint8_t *resp, size_t len, obd_dtc_t *out, int max)
{
    if (len < 3 || resp[0] != 0x59) return 0;
    int n = 0;
    for (size_t pos = 3; pos + 3 < len && n < max; pos += 4) {
        out[n].code = ((uint16_t)resp[pos] << 8) | resp[pos + 1];
        out[n].fault_type = resp[pos + 2];
        out[n].status = resp[pos + 3];
        out[n].has_status = true;
        n++;
    }
    return n;
}

static const char *const readiness_spark[OBD_READINESS_COUNT] = {
    "MISFIRE", "FUEL SYSTEM", "COMPONENTS", "CATALYST", "HEATED CAT", "EVAP", "SEC AIR",
    "A/C REFRIG", "O2 SENSOR", "O2 HEATER", "EGR/VVT",
};
static const char *const readiness_diesel[OBD_READINESS_COUNT] = {
    "MISFIRE", "FUEL SYSTEM", "COMPONENTS", "NMHC CAT", "NOX/SCR", "RESERVED", "BOOST PRESS",
    "RESERVED", "EXHAUST SENS", "PM FILTER", "EGR/VVT",
};

const char *obd_readiness_name(int index, bool compression_ignition)
{
    if (index < 0 || index >= OBD_READINESS_COUNT) return "?";
    return compression_ignition ? readiness_diesel[index] : readiness_spark[index];
}

void obd_parse_readiness(const uint8_t *abcd, obd_readiness_t *out)
{
    uint8_t a = abcd[0], b = abcd[1], c = abcd[2], d = abcd[3];
    out->mil_on = (a & 0x80) != 0;
    out->dtc_count = a & 0x7F;
    out->compression_ignition = (b & 0x08) != 0;

    // Continuous monitors (byte B): support bits 0..2, "not complete" bits 4..6
    out->supported = 0;
    out->incomplete = 0;
    for (int i = 0; i < 3; i++) {
        if (b & (1 << i))        out->supported |= 1u << i;
        if (b & (1 << (i + 4)))  out->incomplete |= 1u << i;
    }
    // Non-continuous monitors (bytes C support, D "not complete"): 8 bits, mapped to indices 3..10
    for (int i = 0; i < 8; i++) {
        if (c & (1 << i)) {
            out->supported |= 1u << (3 + i);
            if (d & (1 << i)) out->incomplete |= 1u << (3 + i);
        }
    }
}

const char *obd_pid_name(uint8_t pid)
{
    switch (pid) {
    case 0x04: return "ENGINE LOAD";
    case 0x05: return "COOLANT";
    case 0x0B: return "MAP";
    case 0x0C: return "RPM";
    case 0x0D: return "SPEED";
    case 0x0F: return "INTAKE AIR";
    case 0x11: return "THROTTLE";
    case 0x42: return "MODULE VOLT";
    case 0x46: return "AMBIENT";
    case 0x5C: return "OIL TEMP";
    default:   return NULL;
    }
}

bool obd_format_pid(uint8_t pid, const uint8_t *d, size_t len, char *out, size_t out_size)
{
    switch (pid) {
    case 0x04: case 0x11:
        if (len < 1) return false;
        snprintf(out, out_size, "%u %%", (unsigned)(d[0] * 100 / 255));
        return true;
    case 0x05: case 0x0F: case 0x46: case 0x5C:
        if (len < 1) return false;
        snprintf(out, out_size, "%d C", (int)d[0] - 40);
        return true;
    case 0x0B:
        if (len < 1) return false;
        snprintf(out, out_size, "%u kPa", (unsigned)d[0]);
        return true;
    case 0x0C:
        if (len < 2) return false;
        snprintf(out, out_size, "%u rpm", (unsigned)(((d[0] << 8) | d[1]) / 4));
        return true;
    case 0x0D:
        if (len < 1) return false;
        snprintf(out, out_size, "%u km/h", (unsigned)d[0]);
        return true;
    case 0x42: {
        if (len < 2) return false;
        unsigned mv = (d[0] << 8) | d[1];
        snprintf(out, out_size, "%u.%02u V", mv / 1000, (mv % 1000) / 10);
        return true;
    }
    default:
        return false;
    }
}

bool obd_parse_vin(const uint8_t *resp, size_t len, char *vin, size_t vin_size)
{
    if (len < 3 || resp[0] != 0x49 || resp[1] != 0x02 || vin_size < 18) return false;
    size_t pos = 3;                                         // 49 02 <item count>
    size_t n = 0;
    for (; pos < len && n < 17; pos++) {
        if (resp[pos] >= 0x20 && resp[pos] < 0x7F) vin[n++] = (char)resp[pos];
    }
    vin[n] = 0;
    return n >= 11;
}
