# ISOTP-BLE

ISO-TP over BLE bridge firmware for the LilyGo T-Display-S3 and an Adafruit CAN Pal (ESP32-S3, built-in TWAI
controller, 500 kbit/s). It speaks the protocol used by Simos Tools and Simos.app, and doubles as a SavvyCAN
(GVRET) USB adapter. Based on [esp32-isotp-ble-bridge](https://github.com/Switchleg1/esp32-isotp-ble-bridge).

## Modes

| Mode | What it does | How you get there |
|---|---|---|
| **Simos BLE** (default) | BLE ISO-TP bridge for Simos Tools / Simos.app | saved |
| **SavvyCAN USB** | GVRET adapter over USB serial | saved |
| **Diag** | Standalone trouble code tool on the dongle's screen: scan every module, clear codes, live data, vehicle info. No phone needed. | saved |
| **ELM327** | Acts like an ELM327 adapter over BLE (`ISOTP-ELM327`) and Wi-Fi (`192.168.0.10:35000`), so generic OBD apps (Car Scanner, OBD Fusion, ...) work | saved |
| **Bench sim** | The saved mode (Simos BLE, Diag or ELM327) against virtual modules instead of a car: ECU 0x7E0, TCU 0x7E1 (OBD-only, wants 10 ms between frames), gateway 0x710 and ABS 0x713, each with a few trouble codes. No CAN traffic. | one-shot |
| **Wi-Fi update** | Access point plus upload page for firmware updates, so USB is never needed in the car | one-shot |

One-shot modes are never saved: the dongle is back in its saved mode after any power cycle.

### Diag mode

Two buttons: **BOOT** (tap = next) and **KEY** (the second button, GPIO14: tap = select / back).

* **Scan codes** asks the OBD addresses 0x7E0-0x7E7 and the VAG range 0x700-0x76F (answers on request + 0x6A) for
  trouble codes, about 6 s. Modules that speak UDS are read with service 0x19; OBD-only modules fall back to
  modes 03 and 07. The screen lists codes per module with their status (ACT / PEND / STORED / MIL).
* **Clear codes** asks for confirmation: **hold KEY for 2 s**. It sends UDS 0x14 (or mode 04 to OBD-only
  modules) to every module that answered, then reads them back and shows what is really stored now. Clearing
  also resets readiness monitors and erases freeze frames, and needs ignition on.
* **Live data** polls engine speed, speed, coolant, load, throttle, intake temperature, MAP and module voltage.
* **Vehicle info** shows the VIN, MIL state and readiness monitors.

The scan is read-only. Everything is logged to the flash log, including each code's VAG fault number (P0300 = 16684).

### ELM327 mode

The screen shows the Wi-Fi network, its password (generated once, kept in NVS), and what the phone last sent.
BLE exposes both service layouts used by BLE ELM327 adapters (0xFFF0 notify 0xFFF1 / write 0xFFF2, and 0xFFE0 /
0xFFE1). Supported: ISO 15765-4 CAN 11 bit / 500 kbit/s (protocol 6, also as auto), requests to 0x7DF or a
specific address (`ATSH`), response filter (`ATCRA`), headers, spaces, multi-frame answers, `ATRV` (read from
the ECU's module voltage, there is no voltage sense line). Not supported: other protocols (`ATSP7`... answer `?`),
raw frame mode (`ATCAF0`), monitor mode (`ATMA`); custom flow control commands are accepted and ignored.
The ELM327 and Simos BLE interfaces are open to any client in range, like any dongle of this kind.

### Sleep

The OBD port is live all the time. After 10 minutes with no CAN frames, no connected phone and no button press,
the dongle goes into deep sleep; a frame on the bus or the BOOT button wakes it (it boots normally). It never
sleeps while USB is connected, and not in SavvyCAN, bench or update modes. Change the time with the NVS key
`sleep_min` in namespace `dongle_cfg` (0 = never). Only the ESP32-S3 sleeps: measure the whole dongle's current
(buck converter and CAN transceiver included) before relying on it for weeks of parking.

## BOOT button

| Press | Action |
|---|---|
| tap | next page (status, bus monitor, firmware info), or next item in Diag mode |
| hold 2 s, release | switch to the next saved mode; keep holding for every further 1.5 s to skip ahead (Simos, SavvyCAN, Diag, ELM327) |
| hold 6.5-8 s, release | bench simulator (saved mode) |
| hold 8-9.5 s, release | Wi-Fi update |
| hold 9.5 s+, release | cancel |

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

* The ISO-TP engine, the OBD/UDS decoders and the ELM327 interpreter are plain C with host-side tests: `pio test -e native`.
* End-to-end over BLE from a desktop (needs `pip install bleak` and a Bluetooth adapter):
  `python3 tools/ble_probe.py all` speaks the dongle's protocol like Simos Tools does (handshake, settings,
  single/multi-frame and 69-byte requests, split packets, persist streaming, and the "create PID" timing
  probe). Put the dongle in bench mode first (hold BOOT 6.5-8 s with Simos BLE as the saved mode). The simulated TCU asks for a 10 ms gap between
  consecutive frames and drops a request that arrives faster, so it catches a regression in frame spacing.
* `python3 tools/elm_probe.py ble` (or `tcp` after joining the dongle's Wi-Fi) runs an ELM327 session like a
  phone app would, against the bench simulator.
* Test builds (never use these in a car; flash the normal build afterwards), set with `PLATFORMIO_BUILD_FLAGS`:
  `-DFORCE_BENCH_SIM` (bench), `-DFORCE_WIFI_UPDATE`, `-DFORCE_MODE=2` (Diag) or `=3` (ELM327) without saving,
  `-DDIAG_SELFTEST` (Diag drives its own screens and logs the result), `-DELM_LOG_PASSWORD` (print the ELM Wi-Fi
  password), `-DPOWER_TEST=40` (sleep after 40 s with USB ignored, wake by timer after 15 s).

Note: an update is only kept after the new image has run for 20 s. Resetting or unplugging the dongle sooner
rolls back to the previous firmware, which is what the bootloader is meant to do with an image that may be bad.

## Layout

16 MB flash: two 2 MB OTA slots, a core dump partition and the log partition (`partitions.csv`).
Settings (NVS) keep their address across updates.
