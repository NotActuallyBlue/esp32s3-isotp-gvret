#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "isotp.h"
#include "isotp_link_containers.h"
#include "bench_sim.h"
#include "stackwatch.h"

#define SIM_TAG             "BenchSim"
#define SIM_NODE_COUNT      4
#define SIM_MAX_DTCS        24
#define SIM_FUNCTIONAL_ID   0x7DF
#define SIM_BUF_SIZE        1024
#define SIM_QUEUE_LEN       64
#define SIM_LATENCY_US      4000        // time a real module would take to produce a response

typedef struct {
    uint32_t    id;
    uint8_t     len;
    uint8_t     data[8];
} sim_frame_t;

typedef struct {
    uint16_t    code;                   // two-byte SAE code
    uint8_t     fault_type;
    uint8_t     status;                 // UDS status byte
} sim_dtc_t;

typedef struct {
    const char *name;
    uint32_t    request_id;             // what the dongle sends to
    uint32_t    response_id;            // what this node answers on
    IsoTpLink   link;
    uint8_t     send_buf[SIM_BUF_SIZE];
    uint8_t     recv_buf[SIM_BUF_SIZE];
    uint8_t     request[SIM_BUF_SIZE];
    uint8_t     response[SIM_BUF_SIZE];
    uint16_t    response_len;
    int64_t     respond_at_us;
    bool        response_pending;
    uint32_t    st_min_us;              // gap this module asks for in its flow control, and insists on
    int64_t     last_cf_us;             // when the previous consecutive frame of the current request arrived
    bool        request_dropped;        // the dongle sent frames faster than st_min_us: ignore this request
    bool        obd;                    // answers the functional OBD-II address 0x7DF as well
    bool        no_uds_dtc;             // refuses UDS 0x19 (an OBD-only module): codes come from modes 03/07
    bool        no_uds_clear;           // reads codes with UDS but refuses UDS 0x14 (NRC 11), like a Mk7 engine ECU: OBD mode 04 clears
    bool        clear_needs_extended;   // refuses every clear until the extended diagnostic session (10 03) is open
    bool        extended;               // the extended session is open
    sim_dtc_t   dtcs[SIM_MAX_DTCS];
    uint8_t     dtc_count;
} sim_node_t;

static bench_sim_sink_t sim_sink = NULL;

static bool             sim_active = false;
static QueueHandle_t    sim_queue = NULL;
static sim_node_t       sim_nodes[SIM_NODE_COUNT] = {
    { .name = "ECU", .request_id = 0x7E0, .response_id = 0x7E8, .st_min_us = 0, .obd = true, .no_uds_clear = true },
    // A DSG-like module: asks for 10 ms between consecutive frames and silently drops a request whose frames
    // come faster. This reproduces the "flow control, then silence" behaviour seen on the real TCU.
    { .name = "TCU", .request_id = 0x7E1, .response_id = 0x7E9, .st_min_us = 10000, .obd = true, .no_uds_dtc = true, .clear_needs_extended = true },
    // UDS-only modules (VAG style 0x7xx / +0x6A addressing) for the multi-module scan
    { .name = "GATEWAY", .request_id = 0x710, .response_id = 0x77A, .st_min_us = 0 },
    { .name = "ABS", .request_id = 0x713, .response_id = 0x77D, .st_min_us = 0 },
};

// Trouble codes the simulated modules start with, so the diagnostics screens have something to show and clear
static void sim_reset_dtcs(void)
{
    memset(sim_nodes[0].dtcs, 0, sizeof(sim_nodes[0].dtcs));
    sim_nodes[0].dtcs[0] = (sim_dtc_t){ 0x0300, 0x00, 0x89 };     // P0300 active, confirmed, lamp on
    sim_nodes[0].dtcs[1] = (sim_dtc_t){ 0x0171, 0x00, 0x08 };     // P0171 stored
    sim_nodes[0].dtcs[2] = (sim_dtc_t){ 0x0420, 0x00, 0x04 };     // P0420 pending only
    sim_nodes[0].dtcs[3] = (sim_dtc_t){ 0x003A, 0xFD, 0x40 };     // "test not completed": listed by real modules, not a fault
    sim_nodes[0].dtcs[4] = (sim_dtc_t){ 0x0053, 0x8C, 0x50 };
    sim_nodes[0].dtcs[5] = (sim_dtc_t){ 0x0043, 0x1A, 0x20 };     // "failed since last clear" only: history, survives a clear like on the real car
    sim_nodes[0].dtc_count = 6;
    sim_nodes[1].dtcs[0] = (sim_dtc_t){ 0x0700, 0x00, 0x08 };     // P0700 stored
    sim_nodes[1].dtc_count = 1;
    sim_nodes[2].dtcs[0] = (sim_dtc_t){ 0xC100, 0x00, 0x28 };     // U0100 stored
    sim_nodes[2].dtc_count = 1;
    sim_nodes[3].dtcs[0] = (sim_dtc_t){ 0x4035, 0x00, 0x09 };     // C0035 active
    sim_nodes[3].dtc_count = 1;
    // Enough codes to need several pages on the screen
    for (int i = 0; i < 15; i++) {
        sim_nodes[3].dtcs[1 + i] = (sim_dtc_t){ (uint16_t)(0x4040 + i), 0x00, 0x08 };
    }
    sim_nodes[3].dtc_count = 16;
}

void bench_sim_set_sink(bench_sim_sink_t sink)
{
    sim_sink = sink;
}

bool bench_sim_active(void)
{
    return sim_active;
}

bool bench_sim_is_response_id(uint32_t arbitration_id)
{
    for (int i = 0; i < SIM_NODE_COUNT; i++) {
        if (arbitration_id == sim_nodes[i].response_id) {
            return true;
        }
    }
    return false;
}

void bench_sim_send_can(uint32_t arbitration_id, const uint8_t *data, uint16_t size)
{
    if (!sim_queue || size > 8) {
        return;
    }
    sim_frame_t frame = { .id = arbitration_id, .len = (uint8_t)size };
    memcpy(frame.data, data, size);
    xQueueSend(sim_queue, &frame, 0);
}

// Values that move over time, so gauges in the app visibly change
static uint8_t sim_value(int node, uint16_t did, int index)
{
    uint32_t tick = (uint32_t)(esp_timer_get_time() / 50000);
    return (uint8_t)(tick * 3 + index * 17 + did + node * 5);
}

// Build the answer to a UDS request. Returns the response length.
static uint16_t sim_build_response(int node, const uint8_t *req, uint16_t len, uint8_t *out)
{
    uint8_t sid = req[0];
    uint8_t sub = len > 1 ? req[1] : 0;

    switch (sid) {
    case 0x10:  // DiagnosticSessionControl
        sim_nodes[node].extended = (sub == 0x03);
        out[0] = 0x50; out[1] = sub; out[2] = 0x00; out[3] = 0x32; out[4] = 0x01; out[5] = 0xF4;
        return 6;
    case 0x11:  // ECUReset
        out[0] = 0x51; out[1] = sub;
        return 2;
    case 0x3E:  // TesterPresent
        out[0] = 0x7E; out[1] = sub;
        return 2;
    case 0x27:  // SecurityAccess: odd = request seed, even = send key
        out[0] = 0x67; out[1] = sub;
        if (sub & 1) {
            out[2] = 0x12; out[3] = 0x34; out[4] = 0x56; out[5] = 0x78;
            return 6;
        }
        return 2;
    case 0x19: {  // ReadDTCInformation, reportDTCByStatusMask only
        if (sim_nodes[node].no_uds_dtc) { out[0] = 0x7F; out[1] = sid; out[2] = 0x11; return 3; }
        if (sub != 0x02) { out[0] = 0x7F; out[1] = sid; out[2] = 0x12; return 3; }
        sim_node_t *n = &sim_nodes[node];
        uint16_t pos = 0;
        out[pos++] = 0x59; out[pos++] = sub; out[pos++] = 0xFF;
        for (int i = 0; i < n->dtc_count; i++) {
            if (!(n->dtcs[i].status & (len > 2 ? req[2] : 0xFF))) continue;
            out[pos++] = n->dtcs[i].code >> 8; out[pos++] = n->dtcs[i].code & 0xFF;
            out[pos++] = n->dtcs[i].fault_type; out[pos++] = n->dtcs[i].status;
        }
        return pos;
    }
    case 0x14:  // ClearDiagnosticInformation
        if (sim_nodes[node].clear_needs_extended && !sim_nodes[node].extended) { out[0] = 0x7F; out[1] = sid; out[2] = 0x22; return 3; }
        if (sim_nodes[node].no_uds_clear) { out[0] = 0x7F; out[1] = sid; out[2] = 0x11; return 3; }
        sim_nodes[node].dtc_count = 0;
        out[0] = 0x54;
        return 1;
    case 0x04:  // OBD-II clear DTCs
        if (sim_nodes[node].clear_needs_extended && !sim_nodes[node].extended) { out[0] = 0x7F; out[1] = sid; out[2] = 0x22; return 3; }
        {   // mode 04 clears stored, pending and active faults, not history entries
            sim_node_t *n = &sim_nodes[node];
            int keep = 0;
            for (int i = 0; i < n->dtc_count; i++) if ((n->dtcs[i].status & 0x0F) == 0) n->dtcs[keep++] = n->dtcs[i];
            n->dtc_count = keep;
        }
        out[0] = 0x44;
        return 1;
    case 0x03: case 0x07: case 0x0A: {  // OBD-II stored / pending / permanent DTCs
        sim_node_t *n = &sim_nodes[node];
        uint8_t want = sid == 0x03 ? 0x08 : (sid == 0x07 ? 0x04 : 0x00);
        uint16_t pos = 2;
        uint8_t count = 0;
        for (int i = 0; i < n->dtc_count && want; i++) {
            // mode 03 lists confirmed codes, mode 07 codes that are pending and not yet confirmed
            bool match = sid == 0x03 ? (n->dtcs[i].status & 0x08) : ((n->dtcs[i].status & 0x04) && !(n->dtcs[i].status & 0x08));
            if (!match) continue;
            out[pos++] = n->dtcs[i].code >> 8; out[pos++] = n->dtcs[i].code & 0xFF;
            count++;
        }
        out[0] = sid + 0x40; out[1] = count;
        return pos;
    }
    case 0x01: {  // OBD-II current data
        uint8_t pid = len > 1 ? req[1] : 0;
        out[0] = 0x41; out[1] = pid;
        switch (pid) {
        case 0x00: out[2] = 0x98; out[3] = 0x3A; out[4] = 0x80; out[5] = 0x01; return 6;   // 01 04 05 0B 0C 0D 0F 11, then 0x20 for the next range
        case 0x01: {
            uint8_t count = 0;
            for (int i = 0; i < sim_nodes[node].dtc_count; i++) if (sim_nodes[node].dtcs[i].status & 0x08) count++;
            bool mil = false;
            for (int i = 0; i < sim_nodes[node].dtc_count; i++) if (sim_nodes[node].dtcs[i].status & 0x80) mil = true;
            out[2] = (mil ? 0x80 : 0) | count; out[3] = 0x07; out[4] = 0x65; out[5] = 0x04;
            return 6;
        }
        case 0x04: out[2] = 70 + sim_value(node, pid, 0) % 40; return 3;
        case 0x05: out[2] = 40 + 88; return 3;
        case 0x0B: out[2] = 35; return 3;
        case 0x0C: { uint16_t rpm4 = 3200 + (sim_value(node, pid, 0) % 64) * 20; out[2] = rpm4 >> 8; out[3] = rpm4 & 0xFF; return 4; }
        case 0x0D: out[2] = 40 + sim_value(node, pid, 0) % 20; return 3;
        case 0x0F: out[2] = 40 + 24; return 3;
        case 0x11: out[2] = 40 + sim_value(node, pid, 0) % 30; return 3;
        case 0x20: out[2] = 0x00; out[3] = 0x00; out[4] = 0x00; out[5] = 0x01; return 6;
        case 0x40: out[2] = 0x40; out[3] = 0x00; out[4] = 0x00; out[5] = 0x00; return 6;
        case 0x42: out[2] = 0x35; out[3] = 0xE8; return 4;                                  // 13.8 V
        default: out[0] = 0x7F; out[1] = sid; out[2] = 0x12; return 3;
        }
    }
    case 0x09: {  // OBD-II vehicle information
        uint8_t pid = len > 1 ? req[1] : 0;
        if (pid == 0x00) { out[0] = 0x49; out[1] = 0x00; out[2] = 0x50; out[3] = 0; out[4] = 0; out[5] = 0; return 6; }
        if (pid == 0x02) { out[0] = 0x49; out[1] = 0x02; out[2] = 0x01; memcpy(out + 3, "SIMULATEDVIN00001", 17); return 20; }
        if (pid == 0x04) { out[0] = 0x49; out[1] = 0x04; out[2] = 0x01; memset(out + 3, 0, 16); memcpy(out + 3, "SIM-CAL-0001", 12); return 19; }
        out[0] = 0x7F; out[1] = sid; out[2] = 0x12;
        return 3;
    }
    case 0x2E:  // WriteDataByIdentifier
        out[0] = 0x6E; out[1] = len > 1 ? req[1] : 0; out[2] = len > 2 ? req[2] : 0;
        return 3;
    case 0x2C:  // DynamicallyDefineDataIdentifier
        out[0] = 0x6C; out[1] = sub;
        if (len > 3) { out[2] = req[2]; out[3] = req[3]; return 4; }
        return 2;
    case 0x22: {  // ReadDataByIdentifier: each DID answers with 8 bytes, so replies are multi-frame
        if (len == 3 && req[1] == 0xF1 && req[2] == 0x87) {     // spare part number
            out[0] = 0x62; out[1] = 0xF1; out[2] = 0x87;
            int k = snprintf((char *)out + 3, 20, "SIM-%s-PART", sim_nodes[node].name);
            return 3 + k;
        }
        uint16_t pos = 0;
        out[pos++] = 0x62;
        for (uint16_t i = 1; i + 1 < len && pos + 10 <= SIM_BUF_SIZE; i += 2) {
            uint16_t did = ((uint16_t)req[i] << 8) | req[i + 1];
            out[pos++] = req[i];
            out[pos++] = req[i + 1];
            for (int b = 0; b < 8; b++) out[pos++] = sim_value(node, did, b);
        }
        return pos;
    }
    case 0x34: out[0] = 0x74; out[1] = 0x20; out[2] = 0x0F; out[3] = 0xFF; return 4;   // RequestDownload
    case 0x36: out[0] = 0x76; out[1] = sub; return 2;                                    // TransferData
    case 0x37: out[0] = 0x77; return 1;                                                  // TransferExit
    case 0x31: out[0] = 0x71; out[1] = sub; out[2] = len > 2 ? req[2] : 0; out[3] = len > 3 ? req[3] : 0; return 4; // RoutineControl
    default:
        break;
    }

    // Unknown service. Short requests get a negative response (serviceNotSupported). Longer ones are most
    // likely the apps' own PID/data requests: answer positively with data that grows with the request, so the
    // multi-frame paths in both directions get exercised.
    if (len < 8) {
        out[0] = 0x7F; out[1] = sid; out[2] = 0x11;
        return 3;
    }
    uint16_t n = len * 2 > 200 ? 200 : len * 2;
    out[0] = sid + 0x40;
    for (uint16_t i = 1; i < n; i++) out[i] = (i < 8) ? req[i] : sim_value(node, req[1], i);
    return n;
}

static void sim_route_frame(const sim_frame_t *frame)
{
    // Functional OBD-II request: every OBD module hears it
    if (frame->id == SIM_FUNCTIONAL_ID) {
        for (int i = 0; i < SIM_NODE_COUNT; i++) {
            if (!sim_nodes[i].obd) continue;
            uint8_t data[8];
            memcpy(data, frame->data, frame->len);
            isotp_on_can_message(&sim_nodes[i].link, data, frame->len);
        }
        return;
    }

    // A frame the dongle sent to a simulated module
    for (int i = 0; i < SIM_NODE_COUNT; i++) {
        if (frame->id == sim_nodes[i].request_id) {
            sim_node_t *node = &sim_nodes[i];
            uint8_t pci = frame->data[0] & 0xF0;

            if (pci == 0x10) {                              // first frame: a new request starts
                node->request_dropped = false;
                node->last_cf_us = 0;
            } else if (pci == 0x20 && node->st_min_us) {    // consecutive frame: check the separation time
                int64_t now = esp_timer_get_time();
                if (node->last_cf_us && (now - node->last_cf_us) < (int64_t)node->st_min_us * 8 / 10 && !node->request_dropped) {
                    ESP_LOGW(SIM_TAG, "%s: consecutive frame after %lld us, asked for %lu us: request dropped",
                             node->name, (long long)(now - node->last_cf_us), (unsigned long)node->st_min_us);
                    node->request_dropped = true;
                    node->link.receive_status = ISOTP_RECEIVE_STATUS_IDLE;
                }
                node->last_cf_us = now;
            }
            if (node->request_dropped && pci == 0x20) return;

            uint8_t data[8];
            memcpy(data, frame->data, frame->len);
            isotp_on_can_message(&node->link, data, frame->len);
            return;
        }
    }

    // A frame a simulated module sent: deliver it to the dongle's link exactly like the CAN receive task does
    for (int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
        IsoTpLinkContainer *c = &isotp_link_containers[i];
        if (c->data_mutex && frame->id == c->link.receive_arbitration_id) {
            uint8_t data[8];
            memcpy(data, frame->data, frame->len);
            xSemaphoreTake(c->data_mutex, portMAX_DELAY);
            isotp_on_can_message(&c->link, data, frame->len);
            xSemaphoreGive(c->data_mutex);
            xSemaphoreGive(c->wait_for_isotp_data_sem);
            return;
        }
    }

    // No ISO-TP link wants it: hand it to the diagnostics tester, if there is one
    if (sim_sink) {
        sim_sink(frame->id, frame->data, frame->len);
    }
}

static void sim_service_node(sim_node_t *node, int index)
{
    isotp_poll(&node->link);

    uint16_t request_len = 0;
    if (isotp_receive(&node->link, node->request, SIM_BUF_SIZE, &request_len) == ISOTP_RET_OK && request_len > 0) {
        node->response_len = sim_build_response(index, node->request, request_len, node->response);
        node->respond_at_us = esp_timer_get_time() + SIM_LATENCY_US;
        node->response_pending = true;
    }

    if (node->response_pending && esp_timer_get_time() >= node->respond_at_us &&
        node->link.send_status != ISOTP_SEND_STATUS_INPROGRESS) {
        isotp_send(&node->link, node->response, node->response_len);
        node->response_pending = false;
    }
}

static void sim_task(void *arg)
{
    sim_frame_t frame;
    while (1) {
        if (xQueueReceive(sim_queue, &frame, pdMS_TO_TICKS(1)) == pdTRUE) {
            do {
                sim_route_frame(&frame);
            } while (xQueueReceive(sim_queue, &frame, 0) == pdTRUE);
        }

        for (int i = 0; i < SIM_NODE_COUNT; i++) {
            sim_service_node(&sim_nodes[i], i);
        }
    }
}

void bench_sim_start(void)
{
    sim_queue = xQueueCreate(SIM_QUEUE_LEN, sizeof(sim_frame_t));
    sim_reset_dtcs();
    for (int i = 0; i < SIM_NODE_COUNT; i++) {
        sim_node_t *node = &sim_nodes[i];
        isotp_init_link(&node->link, node->response_id, node->request_id,
                        node->send_buf, sizeof(node->send_buf), node->recv_buf, sizeof(node->recv_buf));
        node->link.st_min = node->st_min_us;                // advertised in our flow control frames
    }
    sim_active = true;
    stackwatch_create(sim_task, "bench_sim", 4096, NULL, 3);
    ESP_LOGW(SIM_TAG, "Bench simulator running: virtual ECU 0x7E0/0x7E8, TCU 0x7E1/0x7E9, gateway 0x710/0x77A and ABS 0x713/0x77D, no CAN traffic");
}
