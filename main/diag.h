#ifndef DIAG_H
#define DIAG_H

#include <stdbool.h>

// Standalone diagnostics mode: scan every control unit for trouble codes, show them on the dongle's screen,
// clear them (with a confirmation), watch live data and read vehicle info. Driven by the two buttons on the
// T-Display: BOOT (tap = next) and KEY (GPIO14, tap = select / back, hold = confirm). No phone needed.
// bench = talk to the simulated modules (see bench_sim.h) instead of the CAN bus.
void diag_start(bool bench);

#endif // DIAG_H
