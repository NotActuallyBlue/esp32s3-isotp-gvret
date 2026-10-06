# Development guide

Notes for people building, testing or changing the firmware. The user guide is in [README.md](README.md).

## Hardware

* **LilyGo T-Display-S3** (ESP32-S3, 16 MB flash, 170x320 screen) and an **Adafruit CAN Pal** transceiver, with its silent pin
  tied to ground so it is always in normal mode.
* CAN at **500 kbit/s**, ESP32-S3 TWAI controller: **GPIO17 = TX, GPIO18 = RX**.
* Buttons: **BOOT** (GPIO0) and **KEY** (GPIO14).
* OBD-II connector: CAN-H pin 6, CAN-L pin 14, ground pin 4/5, battery +12 V pin 16 (always live), powered through a
  DFRobot DFR0571 buck converter (5 V) with a resistor and capacitor on its input.

## Building

```bash
pio run -t upload            # build and flash over USB (PlatformIO, ESP-IDF 6.1)
pio test -e native           # host-side unit tests
```

Never connect USB while the dongle is plugged into a car. Update over Wi-Fi, capture with SavvyCAN over Wi-Fi, and read the log at
the desk.

## How the modes are built

| Mode | What it does | How you get there |
|---|---|---|
| **Simos BLE** (default) | BLE ISO-TP bridge for Simos.app / Simos Tools | saved |
| **SavvyCAN USB** | GVRET adapter over USB serial | saved |
| **Diag** | Standalone trouble code tool on the screen | saved |
| **ELM327** | ELM327 emulation over BLE (`ISOTP-ELM327`) and Wi-Fi (`192.168.0.10:35000`) | saved |
| **Bench sim** | The saved mode (Simos BLE, Diag or ELM327) against virtual modules instead of a car | one-shot |
| **Wi-Fi update** | Access point and upload page for firmware updates | one-shot |

SavvyCAN mode streams GVRET over a TCP server on port 23 of the dongle's own access point (`ISOTP-SAVVYCAN`, 192.168.0.10, same
WPA2 password as ELM327), and over USB serial for bench use. It is the only reader of CAN frames in that mode (the Simos receive
task is not started), writes to USB only while a host is attached, and batches frames per Wi-Fi packet. The command parser is a
byte-wise state machine, so commands and frames split across TCP segments work. USB writes are skipped for half a second after a few
writes that do not fit, so a PC that powers the dongle without opening the serial port cannot stall the Wi-Fi stream. SavvyCAN mode also
runs under the bench simulator, which adds three cyclic test frames (0x100 every 10 ms, 0x200 every 20 ms, 0x300 every 100 ms).

One-shot modes are never saved (RTC memory request, valid only after a software restart): after any power cycle the dongle is
back in its saved mode.

### Simos BLE

Speaks the protocol Simos.app and Simos Tools use: handshake, per-link settings (STmin, persist delays), single and
multi-frame requests, split packets for long requests, and **persist mode**, where the dongle polls the ECU and TCU
itself and streams timestamped replies for high-rate logging.

Notes from real cars: the TCU asks for 5 ms between frames and the dongle honours it (an STmin override from the app only ever
lengthens the gap); ECU replies come back in about 30-45 ms; the connection asks the phone for a 7.5-15 ms interval and a
251-byte data length.

### Diag

* **Scan** asks the OBD addresses 0x7E0-0x7E7 and the VAG range 0x700-0x76F (answers on request + 0x6A), about 6 s. Modules
  that speak UDS are read with service 0x19 and status mask 0xAF, so only real faults come back. VAG modules also list every code
  they monitor with the "test not completed" bits (0x10, 0x40) set, and those are dropped. Entries with only 0x20 ("failed since
  last clear") are history: the module does not hold them as stored, pending or active, OBD-II does not list them, and the
  headline count and clear result leave them out (shown dimmed as `HIST`). OBD-only modules fall back to modes 03 and 07. Outside
  the engine and transmission a module's three-byte code can be a VAG-specific number rather than a standard SAE code.
* **Clear** sends UDS 0x14 (or mode 04 to OBD-only modules). Engine and transmission control units on a Mk7 refuse UDS 0x14
  ("service not supported"), so the firmware retries with OBD mode 04, then with UDS 0x14 inside the extended session
  (10 03, then back to 10 01). It reads every module back and reports what is left.
* **Bus state:** if nothing answers, the screen reports `BUS SILENT` (fewer than 5 frames during the scan) or `BUS IS ALIVE`, plus
  the CAN controller state.
* The log records every module, code, raw clear answer, each OBD-II module's own view (modes 03/07/0A and PID 01) and each code's
  five-digit VAG fault number (P0300 = 16684).

### ELM327

Both BLE layouts used by ELM327 adapters are offered (service 0xFFF0 with notify 0xFFF1 and write 0xFFF2, and 0xFFE0 with 0xFFE1).
Supported: ISO 15765-4 CAN 11 bit / 500 kbit/s (protocol 6, also as auto), requests to 0x7DF or a specific address (`ATSH`),
response filter (`ATCRA`), headers and spaces, multi-frame answers in the chip's numbered-line format, `ATRV` (read from the ECU's
module voltage; there is no sense line). Not supported: other protocols (`ATSP7` and friends answer `?`), raw frame mode
(`ATCAF0`), monitor mode (`ATMA`). Custom flow control commands are accepted and ignored. The Wi-Fi password is generated once and
kept in NVS.

The ELM327 and Simos BLE interfaces have no authentication: anyone who can connect can send UDS requests to the car.

### Bench simulator

Four virtual modules replace the CAN bus: an engine ECU (0x7E0) and a TCU (0x7E1), plus a gateway (0x710) and ABS (0x713) for the
scan. The simulated TCU is OBD-only, insists on 10 ms between frames and clears only in the extended session, which catches
frame-spacing regressions; the engine refuses UDS 0x14 and keeps its history entries across a clear, like the real one.

### Sleep

After 10 minutes with no CAN frames, no connected client and no button press, the ESP32-S3 goes into deep sleep. A frame on the
bus (RX line low) or the BOOT button wakes it. It never sleeps while USB is connected, and not in bench or update modes (in SavvyCAN mode only while a client is connected).
Change the time with the NVS key `sleep_min` in namespace `dongle_cfg` (0 = never).

Only the ESP32-S3 sleeps. The buck converter and the CAN transceiver keep drawing current, so measure the whole dongle in sleep
before making battery-life claims.

## Firmware updates

* **USB:** `pio run -t upload`
* **Wi-Fi:** Wi-Fi update mode, join the network on the screen (random password, shown only there), then
  `python3 tools/ota_update.py` or open <http://192.168.4.1> and pick `firmware.bin`. A new image is kept only after it has run
  for 20 s, otherwise the bootloader rolls back to the previous one.

## Logs and crashes

Everything the firmware logs is also written to flash (`log` partition, the last 6 boots). Read it back, with crash reports
decoded to source lines, using:

```bash
python3 tools/read_log.py
```

Plugging the dongle into the PC boots it, so the car session is the one *before* the boot you just caused. Wait about 20 s after
plugging in before reading (opening the serial port resets the dongle).

Useful lines:

* `Persist: Session: persist on for X ms | ECU: ... first after a ms | TCU: ...`: timing of each persist session.
* `Flow control for 0x7E1: BS .. STmin .. -> using ..`: what a module asks for between frames.
* `Bus: ALIVE|SILENT, N frame(s) in the last 10 s ...`: when the car's network woke up or went quiet.
* `Main: Heap: free .., lowest ever ..`: memory health, every 30 s.
* `[CRASH]`: decoded to function and source line.

Per-frame lines are limited to a fixed budget per boot after the first PID is registered.

## Tests

* **Host tests:** the ISO-TP engine, the OBD/UDS decoders and the ELM327 interpreter are plain C with unit tests:
  `pio test -e native`.
* **Over BLE from a desktop** (needs `pip install bleak` and a Bluetooth adapter): `python3 tools/ble_probe.py all` speaks the
  Simos protocol like the apps do (handshake, settings, single and multi-frame and 69-byte requests, split packets, persist
  streaming and the "create PID" timing check), and `python3 tools/elm_probe.py ble` (or `tcp` after joining the dongle's
  Wi-Fi) runs an ELM327 session like a phone app would, and `python3 tools/gvret_probe.py` runs a SavvyCAN (GVRET) session over
  Wi-Fi. Put the dongle in bench mode first.
* **Test builds**, set with `PLATFORMIO_BUILD_FLAGS` (never use these in a car, flash the normal build afterwards):
  `-DFORCE_BENCH_SIM`, `-DFORCE_WIFI_UPDATE`, `-DFORCE_MODE=2` (Diag) or `=3` (ELM327) without saving, `-DDIAG_SELFTEST` (Diag
  drives its own screens and logs the result), `-DELM_LOG_PASSWORD`, and `-DPOWER_TEST=40` (sleep after 40 s with USB ignored,
  wake by timer after 15 s).

## Source layout

| Path | What |
|---|---|
| `main/main.c`, `mode_mgr.c` | start-up, saved and one-shot modes, button menu |
| `main/isotp.c`, `isotp_bridge.c` | ISO-TP engine and the Simos BLE bridge (processing, send queue, settings) |
| `main/ble_server.c`, `persist.c` | Simos GATT service (0xABF0) and persist mode |
| `main/twai.c`, `canstats.c` | CAN driver and bus statistics |
| `main/diag.c`, `diag_can.c`, `obd_codec.c` | Diag mode, the tester-side ISO-TP layer, DTC/PID decoding |
| `main/elm327.c`, `elm_mode.c`, `elm_ble.c`, `elm_wifi.c` | ELM327 interpreter and its BLE and Wi-Fi transports |
| `main/bench_sim.c` | virtual ECU, TCU, gateway and ABS |
| `main/display.c` | single-task renderer (strips, pages, detail rows) |
| `main/flashlog.c` | rotating flash log, crash summary |
| `main/ota_update.c`, `power_mgr.c` | Wi-Fi update, deep sleep |
| `main/gvret.c` | SavvyCAN (GVRET) over Wi-Fi and USB |
| `tools/` | log reader, OTA uploader, BLE and ELM327 probes |
| `test/` | host-side unit tests |

Flash is 16 MB: two 2 MB OTA slots, a core dump partition and the log partition (`partitions.csv`). Settings (NVS) keep their
address across updates.

## Verification status

What has been checked, and how.

| Area | State |
|---|---|
| Simos BLE with Simos.app (ECU and TCU, HSL and mode 22) | Works in the car |
| Simos BLE with Simos Tools, ECU HSL | Works in the car |
| Simos Tools with the TCU enabled | Shows "Failed to create PID frame" (see below) |
| Simos BLE, key on / engine off (the state needed for flashing) | Works: Simos Tools pulled ECU info over the dongle on a 2017 Mk7 GTI |
| Diag scan, live data and vehicle info | Works on a 2017 Mk7 GTI, key on / engine off: 16 modules found, VIN, live data and stored codes correct |
| Diag clear codes | Cleared 12 of 16 modules on the same car. The engine and transmission refuse every clear (UDS 0x14 in either session: "service not supported"; mode 04: "conditions not correct", because OBD-II lists no codes for them); the two engine entries were `HIST` only. The ABS module (0x713) did not answer the clear. Not tried yet on a car with real stored engine codes |
| SavvyCAN over Wi-Fi | Tested on the bench with the simulator and `tools/gvret_probe.py` (10 checks, also with the dongle powered from a PC that is not reading its serial port); not yet tried with SavvyCAN itself or on a real bus |
| ELM327, sleep, Wi-Fi update | Tested on the bench, against the simulator and desktop test tools. Screens and buttons checked by hand on the bench |
| ELM327 against a real car, and with real phone apps | Not yet verified |
| Wake from deep sleep by the BOOT button or the CAN bus | Not yet verified (a timer wake is) |

## Known issues

* **Simos Tools with the TCU enabled shows "Failed to create PID frame" in a loop.** With the same dongle and car, Simos.app polls
  the ECU and TCU without trouble, and Simos Tools works with ECU HSL alone. The logs show the TCU answering every request the app
  sent, within the app's time window, so this looks like Simos Tools' own handling of TCU frames. It is not proven. A possible
  workaround (hand the app the previous TCU reply when persist starts) has not been built because it makes the first sample up to
  a second stale.
* The Simos BLE and ELM327 interfaces have no authentication.
* Only CAN 11 bit at 500 kbit/s is supported. No CAN FD, DoIP or K-line.
* The repository has no licence file yet. Check the upstream project's terms before redistributing anything built from it.

## Credits

Built on [esp32-isotp-ble-bridge](https://github.com/Switchleg1/esp32-isotp-ble-bridge) by Switchleg1. The Simos protocol is
that of Simos Tools and Simos.app by Tycho ([TheFlashBold](https://github.com/TheFlashBold)).
