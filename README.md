# ISOTP-BLE

ISO-TP over BLE bridge firmware for the LilyGo T-Display-S3 and an Adafruit CAN Pal (ESP32-S3, built-in TWAI
controller, 500 kbit/s). It speaks the protocol used by Simos Tools and Simos.app, and doubles as a SavvyCAN
(GVRET) USB adapter. Based on [esp32-isotp-ble-bridge](https://github.com/Switchleg1/esp32-isotp-ble-bridge).

## Modes

| Mode | What it does | How you get there |
|---|---|---|
| **Simos BLE** (default) | BLE ISO-TP bridge for Simos Tools / Simos.app | saved |
| **SavvyCAN USB** | GVRET adapter over USB serial | saved |
| **Bench sim** | Same as Simos BLE, but a virtual ECU (0x7E0/0x7E8) and TCU (0x7E1/0x7E9) answer instead of a car. No CAN traffic. Handy for testing the apps at the desk. | one-shot |
| **Wi-Fi update** | Access point plus upload page for firmware updates, so USB is never needed in the car | one-shot |

One-shot modes are never saved: the dongle is back in its saved mode after any power cycle.

## BOOT button

| Press | Action |
|---|---|
| tap | next page: status, bus monitor, firmware info |
| hold 2-5 s, release | switch between the saved modes (Simos BLE / SavvyCAN) |
| hold 5-8 s, release | bench simulator |
| hold 8-11 s, release | Wi-Fi update |
| hold 11 s+, release | cancel |

The screen shows what releasing will do while the button is held.

## Updating the firmware

* USB: `pio run -t upload`
* Wi-Fi: put the dongle in Wi-Fi update mode, join the network shown on its screen (the password is random
  and only appears there), then `python3 tools/ota_update.py` (or open <http://192.168.4.1> and pick `firmware.bin`).
  A new image is kept only after it has run for 20 s, otherwise the bootloader rolls back.

## Logs and crashes

Everything the firmware logs is also written to flash (`log` partition, the last 6 boots). Read it back, with
crash reports decoded to source lines, using `python3 tools/read_log.py` while the dongle is plugged into the PC.
The car session is the one before the boot you just caused by plugging in.

## Tests

The ISO-TP engine is plain C with host-side tests: `pio test -e native`.

## Layout

16 MB flash: two 2 MB OTA slots, a core dump partition and the log partition (`partitions.csv`).
Settings (NVS) keep their address across updates.
