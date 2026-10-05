#ifndef BENCH_SIM_H
#define BENCH_SIM_H

#include <stdint.h>
#include <stdbool.h>

// Bench simulator: a virtual ECU (0x7E0 -> 0x7E8) and TCU (0x7E1 -> 0x7E9) that answer UDS and OBD-II requests
// (plus a gateway and ABS module for the multi-module scan) with plausible multi-frame responses, so Simos Tools / Simos.app can be exercised on the desk with no car and
// no CAN hardware. Frames never touch the bus: they are exchanged in memory.

void bench_sim_start(void);
bool bench_sim_active(void);

// Called instead of putting a frame on the CAN bus while the simulator is active
void bench_sim_send_can(uint32_t arbitration_id, const uint8_t *data, uint16_t size);

// True for the IDs the simulated modules answer on (frames the dongle did not send itself)
bool bench_sim_is_response_id(uint32_t arbitration_id);

// Receiver for frames the simulated modules send when no Simos ISO-TP link claims them (the diagnostics
// and ELM327 modes use this instead of a CAN driver). Called from the simulator task.
typedef void (*bench_sim_sink_t)(uint32_t arbitration_id, const uint8_t *data, uint16_t size);
void bench_sim_set_sink(bench_sim_sink_t sink);

#endif // BENCH_SIM_H
