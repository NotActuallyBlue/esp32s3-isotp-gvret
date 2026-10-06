#ifndef POWER_MGR_H
#define POWER_MGR_H

#include <stdbool.h>

// Sleep when the car is off. The OBD port is live all the time, so a dongle that never sleeps flattens the
// battery of a parked car. After POWER_IDLE_MINUTES with no CAN frames, no phone connected and no button
// presses, the ESP32-S3 goes into deep sleep. A frame on the bus (the RX line going low) or the BOOT button
// wakes it, and it boots normally. It never sleeps while USB is connected, so flashing and log reading at the
// desk are not interrupted.
//
// What this does not cover: the buck converter and the CAN transceiver keep drawing current while the ESP32 sleeps,
// so measure the whole dongle's draw in sleep before relying on it for weeks of parking.

#define POWER_IDLE_MINUTES_DEFAULT  3       // NVS key "sleep_min" (dongle_cfg) overrides, 0 = never sleep

// busy() may be NULL. It returns true while a client is connected (BLE or Wi-Fi), which counts as activity.
typedef bool (*power_busy_fn)(void);
void power_mgr_start(power_busy_fn busy);

// Report activity (a button press, an app request) so the idle timer restarts. Cheap, any task.
void power_mgr_poke(void);

#endif // POWER_MGR_H
