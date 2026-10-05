#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

// Wi-Fi firmware update mode. The dongle starts an access point (name and one-time password are shown on
// the display) and a small web server. Open http://192.168.4.1 in a browser, or run tools/ota_update.py, to
// upload firmware.bin. The new image is written to the inactive app slot and only kept if it boots and runs
// for 20 s; otherwise the bootloader rolls back to the previous image.
//
// Runs in place of the normal services (no BLE, no CAN) and ends with a restart.

void ota_update_start(void);

#endif // OTA_UPDATE_H
