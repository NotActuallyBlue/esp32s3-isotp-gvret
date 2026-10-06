#ifndef GVRET_H
#define GVRET_H

#include <stdbool.h>

// SavvyCAN (GVRET) adapter. SavvyCAN connects either over Wi-Fi (network connection, port 23, to the dongle's own
// access point, which works with the dongle in the car) or over USB serial (bench use).
// bench = answer from the simulated modules instead of the CAN bus.
void gvret_start(bool bench);
bool gvret_client_connected(void);      // a SavvyCAN session is open (Wi-Fi or USB), for the sleep logic

#endif // GVRET_H
