# ISOTP-BLE

A pocket CAN diagnostic dongle for VAG cars, built on a **LilyGo T-Display-S3** and an **Adafruit CAN Pal**. It is a
BLE bridge for the Simos tuning apps, a standalone trouble code reader with a screen, an ELM327 adapter for generic
OBD apps, and a SavvyCAN adapter, all in one firmware. It logs to its own flash, so you can diagnose a session
afterwards without ever plugging a laptop into the car.

Based on [Switchleg1/esp32-isotp-ble-bridge](https://github.com/Switchleg1/esp32-isotp-ble-bridge).

| | |
|---|---|
| **Simos BLE** | ISO-TP over BLE for Simos.app and Simos Tools (ECU/TCU logging, HSL, flashing) |
| **Diag** | Scan every module for trouble codes, clear them, live data, VIN and readiness. Buttons and screen only, no phone |
| **ELM327** | Looks like an ELM327 adapter over BLE and Wi-Fi, so Car Scanner, OBD Fusion and friends work |
| **SavvyCAN** | GVRET adapter over USB serial |
| **Always on** | On-device flash log with crash decoding, Wi-Fi firmware update with rollback, deep sleep when the car is off, a bench simulator to test it all without a car |

## Status

Honest summary of what has been checked and how.

| Area | State |
|---|---|
| Simos BLE with Simos.app (ECU and TCU, HSL and mode 22) | Works in the car |
| Simos BLE with Simos Tools, ECU HSL | Works in the car |
| Simos Tools with the TCU enabled | Shows "Failed to create PID frame" in the car (see [Known issues](#known-issues)) |
| Diag scan, live data and vehicle info in the car | VIN, live data and stored codes read correctly on a real car (found and fixed on that run: "test not completed" entries were counted as codes, and the code list could overlap when paging) |
| Diag clear codes | Not yet tried on a real car |
| ELM327, sleep, Wi-Fi update | Tested on the bench, against the simulator and desktop test tools. Screens and buttons checked by hand on the bench |
| ELM327 against a real car, and with real phone apps | Not yet verified |
| Wake from deep sleep by the BOOT button or the CAN bus | Not yet verified (a timer wake is) |

## Hardware

* **LilyGo T-Display-S3** (ESP32-S3, 16 MB flash, 170x320 screen), bought from the maker.
* **Adafruit CAN Pal** CAN transceiver, with its silent pin tied to ground so it is always in normal mode.
* CAN at **500 kbit/s**, ESP32-S3 TWAI controller: **GPIO17 = TX, GPIO18 = RX**.
* Buttons: **BOOT** (GPIO0) and **KEY** (GPIO14), the two on the board.
* OBD-II connector: CAN-H pin 6, CAN-L pin 14, ground pin 4/5, battery +12 V pin 16 (always live) through a buck
  converter down to 5 V. A prototype power stage is not an automotive one: if you build more than one, use proper
  reverse-polarity and load-dump protection.

Never connect USB while the dongle is plugged into a car you care about. The firmware and its tools are designed so
that you never have to: update over Wi-Fi, and read the log at the desk.

## Quick start

```bash
pio run -t upload            # build and flash over USB (PlatformIO, ESP-IDF 6.1)
pio test -e native           # host-side unit tests
```

Out of the box it starts in **Simos BLE** mode and advertises as `BLE_TO_ISOTP20`. Hold the BOOT button to change
mode (below).

## Modes

| Mode | What it does | How you get there |
|---|---|---|
| **Simos BLE** (default) | BLE ISO-TP bridge for Simos.app / Simos Tools | saved |
| **SavvyCAN USB** | GVRET adapter over USB serial | saved |
| **Diag** | Standalone trouble code tool on the screen | saved |
| **ELM327** | ELM327 emulation over BLE (`ISOTP-ELM327`) and Wi-Fi (`192.168.0.10:35000`) | saved |
| **Bench sim** | The saved mode (Simos BLE, Diag or ELM327) against virtual modules instead of a car | one-shot |
| **Wi-Fi update** | Access point and upload page for firmware updates | one-shot |

One-shot modes are never saved: after any power cycle the dongle is back in its saved mode.

### BOOT button

| Press | Action |
|---|---|
| tap | next display page (status, bus monitor, firmware info), or the next item in Diag mode |
| hold 2 s, release | switch to the next saved mode; each further 1.5 s you keep holding skips one more (Simos, SavvyCAN, Diag, ELM327) |
| hold 6.5-8 s, release | bench simulator |
| hold 8-9.5 s, release | Wi-Fi update |
| hold 9.5 s+, release | cancel |

The screen shows what releasing will do while you hold the button.

### Simos BLE

Speaks the protocol Simos.app and Simos Tools use: handshake, per-link settings (STmin, persist delays), single and
multi-frame requests, split packets for long requests, and **persist mode**, where the dongle polls the ECU and TCU
itself and streams timestamped replies for high-rate logging.

Notes from real cars: the TCU asks for 5 ms between frames and the dongle honours it (an STmin override from the
app only ever lengthens the gap); ECU replies come back in about 30-45 ms; the connection asks the phone for a 7.5-15 ms
interval and a 251-byte data length.

### Diag

Two buttons: **BOOT** (tap = next) and **KEY** (tap = select / back).

* **Scan codes** asks the OBD addresses 0x7E0-0x7E7 and the VAG range 0x700-0x76F (answers on request + 0x6A),
  about 6 s. Modules that speak UDS are read with service 0x19 and status mask 0xAF, so only real faults come back
  (failed, pending, confirmed, failed since last clear, lamp requested). VAG modules also list every code they
  monitor with the "test not completed" bits set, and those are not shown. OBD-only modules fall back to modes 03 and
  07. Codes are listed per module with their status (ACT / PEND / STORED / MIL) and paged to fit the screen. Outside the
  engine and transmission, a module's three-byte code can be a VAG-specific number rather than a standard SAE code.
* **Clear codes** needs you to **hold KEY for 2 s**. It sends UDS 0x14 (or mode 04 to OBD-only modules) to every
  module that answered, then reads them back and shows what is really stored now. Clearing also resets readiness
  monitors and erases freeze frames, and needs ignition on.
* **Live data** shows engine speed, speed, coolant, load, throttle, intake temperature, MAP and module voltage.
* **Vehicle info** shows the VIN, MIL state and readiness monitors.

If nothing answers, the screen says why: **BUS SILENT** means no frames at all were seen, so the car's network is
asleep (some cars need the engine running, not just ignition on, before the OBD port wakes up); **BUS IS ALIVE** means
traffic was seen but no module replied. The CAN controller state (`OK`, `ERROR PASSIVE`, `BUS OFF`) is shown too, and
both are written to the flash log.

The scan is read-only. Every module, code and clear result is written to the flash log, including each code's
five-digit VAG fault number (P0300 = 16684).

### ELM327

Connect over BLE (name `ISOTP-ELM327`) or join the Wi-Fi network shown on the screen (the password is generated once
and kept in NVS) and open `192.168.0.10:35000`. Both BLE layouts used by ELM327 adapters are offered (service
0xFFF0 with notify 0xFFF1 and write 0xFFF2, and 0xFFE0 with 0xFFE1).

Supported: ISO 15765-4 CAN 11 bit / 500 kbit/s (protocol 6, also as auto), requests to 0x7DF or a specific address
(`ATSH`), response filter (`ATCRA`), headers and spaces, multi-frame answers with the chip's numbered-line format,
`ATRV` (read from the ECU's module voltage; there is no sense line). Not supported: other protocols (`ATSP7` and
friends answer `?`), raw frame mode (`ATCAF0`), monitor mode (`ATMA`). Custom flow control commands are accepted and
ignored.

The ELM327 and Simos BLE interfaces are open to any client in range, as on any dongle of this kind: anyone who can
connect can send UDS requests to the car.

### Bench simulator

Hold BOOT 6.5-8 s and the saved mode runs against four virtual modules instead of the CAN bus: an engine ECU (0x7E0)
and a TCU (0x7E1), plus a gateway (0x710) and ABS (0x713) for the scan. Each has a few trouble codes that really clear.
The simulated TCU is OBD-only and insists on 10 ms between frames, which catches frame-spacing regressions. Nothing
touches the bus, so it is safe to use anywhere.

### Sleep

The OBD port is live all the time. After 10 minutes with no CAN frames, no connected phone and no button press, the
dongle goes into deep sleep. A frame on the bus or the BOOT button wakes it, and it boots normally. It never sleeps
while USB is connected, and not in SavvyCAN, bench or update modes. Change the time with the NVS key `sleep_min`
in namespace `dongle_cfg` (0 = never).

Only the ESP32-S3 sleeps. The buck converter and the CAN transceiver keep drawing current, so measure the whole
dongle in sleep before you promise anyone weeks of parking.

## Updating the firmware

* **USB:** `pio run -t upload`
* **Wi-Fi:** put the dongle in Wi-Fi update mode, join the network shown on its screen (the password is random and
  only appears there), then run `python3 tools/ota_update.py` or open <http://192.168.4.1> and pick `firmware.bin`.
  An update is kept only after the new image has run for 20 s, otherwise the bootloader rolls back to the previous
  one. Resetting or unplugging within those 20 s therefore reverts the update, by design.

## Logs and crashes

Everything the firmware logs is also written to flash (`log` partition, the last 6 boots). Read it back, with
crash reports decoded to source lines, using:

```bash
python3 tools/read_log.py
```

Plugging the dongle into the PC boots it, so the car session is the one *before* the boot you just caused. Wait about
20 s after plugging in before reading (opening the serial port resets the dongle).

Useful lines to look for:

* `Persist: Session: persist on for X ms | ECU: ... first after a ms | TCU: ...` timing of each persist session.
* `Flow control for 0x7E1: BS .. STmin .. -> using ..` what a module asks for between frames.
* `Main: Heap: free .., lowest ever ..` memory health, every 30 s.
* `[CRASH]` lines, decoded to function and source line.

Per-frame lines are plentiful, so they are limited to a fixed budget per boot after the first PID is registered.

## Tests

* **Host tests:** the ISO-TP engine, the OBD/UDS decoders and the ELM327 interpreter are plain C with unit tests:
  `pio test -e native` (28 tests).
* **Over BLE from a desktop** (needs `pip install bleak` and a Bluetooth adapter):
  `python3 tools/ble_probe.py all` speaks the Simos protocol like the apps do (handshake, settings, single and
  multi-frame and 69-byte requests, split packets, persist streaming and the "create PID" timing check), and
  `python3 tools/elm_probe.py ble` (or `tcp` after joining the dongle's Wi-Fi) runs an ELM327 session like a phone
  app would. Put the dongle in bench mode first.
* **Test builds**, set with `PLATFORMIO_BUILD_FLAGS` (never use these in a car, flash the normal build afterwards):
  `-DFORCE_BENCH_SIM`, `-DFORCE_WIFI_UPDATE`, `-DFORCE_MODE=2` (Diag) or `=3` (ELM327) without saving,
  `-DDIAG_SELFTEST` (Diag drives its own screens and logs the result), `-DELM_LOG_PASSWORD`, and
  `-DPOWER_TEST=40` (sleep after 40 s with USB ignored, wake by timer after 15 s).

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
| `main/ota_update.c`, `power_mgr.c`, `gvret.c` | Wi-Fi update, deep sleep, SavvyCAN |
| `tools/` | log reader, OTA uploader, BLE and ELM327 probes |
| `test/` | host-side unit tests |

Flash is 16 MB: two 2 MB OTA slots, a core dump partition and the log partition (`partitions.csv`). Settings (NVS)
keep their address across updates.

## Known issues

* **Simos Tools with the TCU enabled shows "Failed to create PID frame" in a loop.** With the same dongle and car,
  Simos.app polls the ECU and TCU without trouble, and Simos Tools works with ECU HSL alone. The logs show the TCU
  answering every request the app sent, within the app's time window, so this looks like Simos Tools' own handling of
  TCU frames. It is not proven. A possible workaround (hand the app the previous TCU reply when persist starts) has
  not been built because it makes the first sample up to a second stale.
* The Simos BLE and ELM327 interfaces have no authentication.
* Only CAN 11 bit at 500 kbit/s is supported. No CAN FD, DoIP or K-line.

## Credits and licence

Built on [esp32-isotp-ble-bridge](https://github.com/Switchleg1/esp32-isotp-ble-bridge) by Switchleg1, and uses the
protocol of Simos Tools and Simos.app by Tycho ([TheFlashBold](https://github.com/TheFlashBold)). This repository does
not contain a licence file yet: check the upstream project's terms before you redistribute anything built from it.

## Disclaimer

Reading trouble codes is harmless. Clearing them resets readiness monitors, and anything that writes to a control unit
can brick it or void a warranty. Use this on your own car, at your own risk.
