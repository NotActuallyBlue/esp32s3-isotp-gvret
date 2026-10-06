# ISOTP-BLE Dongle

A pocket diagnostic and logging dongle for VAG cars. It plugs into the OBD-II port, shows what it is doing on its own
screen, and connects to your phone or laptop over Bluetooth, Wi-Fi or USB.

**What it does**

* **Log and tune with the Simos apps.** A fast Bluetooth link for Simos.app and Simos Tools: live ECU and transmission
  logging, ECU info and flashing.
* **Read and clear trouble codes without a phone.** The built-in Diag mode scans every control unit, lists the faults on the
  screen, shows live data and your VIN, and clears codes behind a confirmation step.
* **Work with generic OBD apps.** ELM327 mode makes the dongle look like a standard ELM327 adapter over Bluetooth LE or Wi-Fi,
  for apps that support those adapters.
* **Capture CAN traffic on a computer.** SavvyCAN mode streams raw CAN frames over Wi-Fi, so it works with the dongle in the car.
* **Sleep when the car is off,** and update its own firmware over Wi-Fi.

**Contents:** [What you need](#what-you-need) · [Getting started](#getting-started) · [Choosing a mode](#choosing-a-mode) ·
[Simos mode](#simos-mode) · [Diag mode](#diag-mode) · [ELM327 mode](#elm327-mode) · [SavvyCAN mode](#savvycan-mode) ·
[Auto sleep](#auto-sleep) · [Updating the firmware](#updating-the-firmware) · [Troubleshooting](#troubleshooting) ·
[Specifications](#specifications) · [Safety and legal](#safety-and-legal)

---

## What you need

* A car with a standard OBD-II port that uses **CAN at 500 kbit/s**. Most cars sold in the US since 2008 do. The dongle was tested
  on a 2017 Volkswagen Golf GTI (Mk7); other models are expected to work but have not been tested yet.
* For Simos mode: **Simos.app** or **Simos Tools**, both made by their own developers.
* For ELM327 mode: an app that supports **ELM327 over Bluetooth LE** or **ELM327 over Wi-Fi**.
* For Diag mode: nothing. The dongle works on its own.

Not supported: CAN FD and DoIP (found on the newest cars), K-line, and older cars that use the TP 2.0 protocol.

## Getting started

1. **Plug the dongle into the OBD-II port,** usually under the dashboard on the driver's side. It starts as soon as it has power and shows its
   status on the screen.
2. **Turn the ignition on** so the dash lights up. For flashing, use ignition on with the engine off. Some cars need a few seconds after the dash
   lights up before the OBD port wakes.
3. **Pick a mode** (see below). Out of the box the dongle starts in Simos mode.
4. **Connect your app,** or use the buttons and screen in Diag mode.

Do not connect the dongle to a computer over USB while it is plugged into a car. Use Bluetooth or Wi-Fi in the car. USB is for
bench use.

## Choosing a mode

The dongle has two buttons: **BOOT** and **KEY**. To change mode, hold **BOOT**. The screen shows what will happen when you let go, so
release when you see the mode you want.

| Hold BOOT for | The screen offers | What happens |
|---|---|---|
| a quick tap | next page | cycles the status, bus monitor and firmware info pages |
| 2 seconds | the next mode | switches to it and restarts |
| every further 1.5 seconds | the mode after that | keep holding to move through Simos, SavvyCAN, Diag and ELM327 |
| about 6.5 to 8 seconds | **Bench sim** | demo mode, no car needed (see below) |
| about 8 to 9.5 seconds | **Wi-Fi update** | firmware update mode |
| 9.5 seconds or more | **Cancel** | restarts in the same mode |

The mode you pick is remembered. Bench sim and Wi-Fi update last for one session only: unplug the dongle and plug it back in to return
to your mode.

**Try it without a car.** Hold BOOT for about 7 seconds and release on **Bench sim**. The dongle then runs your chosen mode against
simulated control units (an engine, a transmission, a gateway and an ABS module with a few made-up trouble codes), so you can try Diag
mode or an app without a car. Nothing is sent on a car's CAN bus.

## Simos mode

The default mode. The dongle appears over Bluetooth as **BLE_TO_ISOTP20**.

1. Open Simos.app or Simos Tools and choose the dongle in its Bluetooth device list.
2. Connect. The screen shows **CONNECTED** with the connection details (link speed and ECU and transmission response times).
3. Use the app as normal: ECU info, logging and flashing.

For flashing, keep the car in ignition on with the engine off, and make sure the car's battery and your phone or laptop are well
charged.

Compatibility notes:

* **Simos.app** works with both the engine control unit (ECU) and the transmission control unit (TCU).
* **Simos Tools** works for engine control unit (ECU) monitoring. Enabling transmission control unit (TCU) monitoring in Simos Tools currently shows "Failed to create PID frame".
  Simos.app works with the TCU on the same dongle and car, so use it if you need the TCU.

## Diag mode

Diag mode turns the dongle into a standalone trouble code tool. You use the two buttons: **BOOT** moves to the next item, and **KEY**
selects (a short press of KEY also goes back).

**Scan codes** asks every control unit in the car for its trouble codes. It takes about six seconds and lists the modules that answered,
with their codes.

| What the screen shows | What it means |
|---|---|
| **ACT** | the fault is present right now |
| **THIS** | it failed during the current drive |
| **PEND** | seen once, not yet confirmed |
| **STORED** | a confirmed fault that is saved in the module |
| **MIL** | the warning lamp is requested |
| **HIST** (dimmed) | the module only remembers that it failed at some point since the last clear; it is not counted as a fault |

Some modules list codes in their own format. In the engine and transmission they are standard codes such as P0300; other modules may
show a manufacturer-specific number. If a number does not look familiar, look it up for your car.

**Clear codes** asks you to confirm: **hold KEY for 2 seconds**. The dongle then asks every module to clear, reads each one back, and shows
what is really left. A few things to know:

* Clearing also resets the readiness monitors and erases freeze-frame data, and the car must be in ignition on.
* A code that is still present comes straight back.
* Some modules refuse to clear. The screen then shows a short reason such as `NOT SUPP` (not supported), `NOT NOW` (conditions not met),
  `SECURITY` or `SESSION`.
* **HIST** entries are not faults, and a module may keep them when asked to clear.

**Live data** shows engine speed, vehicle speed, coolant temperature, engine load, throttle, intake air temperature, manifold pressure
and the control module voltage.

**Vehicle info** shows the VIN, the warning lamp state and the readiness monitors.

If the scan finds nothing, the screen says why:

* **BUS SILENT:** no traffic was seen at all, so the car's network is asleep. Turn the ignition on, wait a few seconds and try again.
  Some cars need the engine running before the OBD port wakes.
* **BUS IS ALIVE:** the car is talking but no module answered. Check that the dongle is firmly in the port.

## ELM327 mode

In this mode the dongle behaves like a standard ELM327 adapter, so apps that support those adapters can connect to it.

* **Over Bluetooth LE:** connect to **ISOTP-ELM327** from the app's adapter list.
* **Over Wi-Fi:** join the Wi-Fi network **ISOTP-ELM327** (the password is shown on the dongle's screen), then set the app to connect to
  `192.168.0.10` on port `35000`.

The screen shows the Wi-Fi password, whether a phone is connected, and the last command received.

It supports the common CAN OBD-II protocol (ISO 15765-4, 11-bit, 500 kbit/s) and the usual ELM327 commands. It does not support other
protocols, raw frame mode or monitor mode. Apps differ, so if one does not connect, check that it supports your adapter type (Bluetooth LE
or Wi-Fi ELM327).

## SavvyCAN mode

For capturing and analysing raw CAN traffic with [SavvyCAN](https://www.savvycan.com) on a computer, with the dongle in the car.

1. Switch the dongle to SavvyCAN mode.
2. On your computer, join the Wi-Fi network **ISOTP-SAVVYCAN** (the password is shown on the dongle's screen; it is the same password the ELM327
   network uses).
3. In SavvyCAN, add a new device connection of the **network connection (GVRET)** type, with the IP address `192.168.0.10` and port `23`.
4. Connect. The screen shows the connection status and counts the frames captured and sent.

The dongle reads at 500 kbit/s. Frames you send from SavvyCAN go out on the car's bus, so only send frames you understand.

**What you will see on the car.** Many modern cars (VW, Audi and others) put a gateway module between the OBD-II port and the car's internal
networks. Through the port you normally see only diagnostic traffic: the replies to requests you send, plus a few status messages. Everyday
broadcast data such as RPM or wheel speed is not forwarded, so a capture on a plain OBD-II connection can look nearly empty. To see that
data you either request it from a module by its address, or connect to the car's network behind the gateway.

**Try it without a car.** Power the dongle from any USB port or charger, hold BOOT for about 7 seconds with SavvyCAN as your mode and release on
**Bench sim**, and connect as above over Wi-Fi. SavvyCAN then shows three test messages that repeat (IDs `0x100` every 10 ms, `0x200` every
20 ms and `0x300` every 100 ms), and the simulated control units answer requests: for example, send ID `0x7DF` with the data `02 01 00` and the
engine and transmission reply on `0x7E8` and `0x7E9`.

At a desk you can also connect the dongle to the computer over USB and add it in SavvyCAN as a serial **GVRET** device (250000 baud). Do not do
this while the dongle is in a car.

## Auto sleep

The OBD-II port has power all the time, so to avoid draining the car's battery the dongle goes into a low-power sleep after **3 minutes**
with no CAN traffic, no connected phone or app, and no button presses. Driving the car, connecting an app or pressing BOOT wakes it. It does
not sleep while connected over USB, or in Bench sim or Wi-Fi update modes. In SavvyCAN mode it sleeps only when no computer is connected.

Sleep lowers the dongle's power use but does not turn it off completely. **If you leave the car parked for days or weeks, unplug the
dongle.**

## Updating the firmware

You can update over Wi-Fi without any cable.

1. Hold BOOT for about 8 to 9.5 seconds and release on **Wi-Fi update**.
2. On your phone or computer, join the Wi-Fi network shown on the dongle's screen and enter the password shown there.
3. Open `http://192.168.4.1` in a browser, choose the `firmware.bin` file you were given, and upload it.
4. The dongle restarts by itself. **Wait about 30 seconds before unplugging it.** It checks the new firmware as it starts, and goes back to
   the previous version automatically if the new one does not start properly.

## Troubleshooting

| Problem | Try this |
|---|---|
| The dongle does not show in the app | Make sure no other phone is connected to it. Unplug it, plug it back in, and look for **BLE_TO_ISOTP20** (Simos mode) or **ISOTP-ELM327**. |
| Nothing answers, or the screen says BUS SILENT | Turn the ignition on and wait a few seconds. Some cars need the engine running before the OBD port wakes. |
| A code comes straight back after clearing | The fault is still present. Fix the cause, then clear again. |
| A module says NOT SUPP, NOT NOW, SECURITY or SESSION when clearing | It refused the request. Other tools may use a method this dongle does not support. |
| The screen is dark | It switches off after a while. Press BOOT to wake it. |
| The dongle does not respond after an update | Wait 30 seconds. If it is still unresponsive, unplug it and plug it back in; it starts the previous firmware if the new one failed. |
| Simos Tools shows "Failed to create PID frame" with the TCU enabled | Use Simos.app for TCU logging, or turn the TCU off in Simos Tools. |

## Specifications

| | |
|---|---|
| Connection to the car | OBD-II port, CAN at 500 kbit/s (11-bit) |
| Power | 12 V from the OBD-II port |
| Wireless | Bluetooth LE and a 2.4 GHz Wi-Fi access point (WPA2) |
| USB | USB-C, for bench use |
| Display | 1.9 inch colour screen, 170 x 320 |
| Controls | BOOT and KEY buttons |
| Modes | Simos, Diag, ELM327, SavvyCAN, plus Bench sim and Wi-Fi update |
| Protocols | ISO-TP (ISO 15765-2), UDS (ISO 14229), OBD-II (SAE J1979), GVRET |

## Safety and legal

* Reading trouble codes and live data is safe. **Clearing codes resets readiness monitors,** and anything that writes to a control unit
  (including flashing) can damage it or void a warranty. Use the dongle on your own vehicle and at your own risk. Keep the battery
  charged and the ignition on, with the engine off, during flashing.
* **Do not connect USB to a computer while the dongle is in the car.**
* The Bluetooth connection is not password protected, so anyone within range could connect to the dongle while it is powered. The Wi-Fi
  networks use a password, which is shown on the dongle's screen. Unplug the dongle when you are not using it.
* Modifying a vehicle's emissions equipment or its emissions-related software can be illegal for road use where you live. This dongle only
  reports what a control unit says; you are responsible for complying with local law.
* Volkswagen, Audi, Golf, GTI, ELM327, Bluetooth and the other names used here belong to their owners. This product is not affiliated with or
  endorsed by them. Simos.app and Simos Tools are separate products by their own developers.

## Credits

Built on [esp32-isotp-ble-bridge](https://github.com/Switchleg1/esp32-isotp-ble-bridge) by Switchleg1.

Developers: see [DEVELOPMENT.md](DEVELOPMENT.md) for building, testing, logs and the source layout.
