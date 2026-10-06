#!/usr/bin/env python3
"""Talk to the dongle's ELM327 mode the way a phone app does, over BLE or Wi-Fi (TCP).

Put the dongle in ELM327 mode (hold BOOT to the "ELM327" prompt) and, to test without a car, in the bench
simulator as well (hold BOOT 6.5-8 s while ELM327 is the saved mode). Needs bleak for BLE (pip install bleak).

    python3 tools/elm_probe.py ble                       # all checks over BLE
    python3 tools/elm_probe.py tcp [--host 192.168.0.10]  # all checks over Wi-Fi (join the dongle's network first)
    python3 tools/elm_probe.py ble --cmd ATZ --cmd 0100   # just send commands and print the answers

The bench checks expect the simulator's data (ECU 0x7E0 with three codes, TCU 0x7E1 with one).
"""
import argparse
import asyncio
import sys

NAME = "ISOTP-ELM327"
UUID_NOTIFY = "0000fff1-0000-1000-8000-00805f9b34fb"
UUID_WRITE = "0000fff2-0000-1000-8000-00805f9b34fb"

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{PASS if ok else FAIL}] {name}" + (f"  ({detail})" if detail else ""))


class Ble:
    async def open(self, args):
        from bleak import BleakClient, BleakScanner
        dev = await BleakScanner.find_device_by_name(NAME, timeout=15)
        if not dev:
            sys.exit(f"No BLE device named {NAME} found")
        self.buf = bytearray()
        self.client = BleakClient(dev)
        await self.client.connect()
        await self.client.start_notify(UUID_NOTIFY, lambda _, d: self.buf.extend(d))
        print(f"BLE connected to {dev.address}, MTU {self.client.mtu_size}")

    async def write(self, data):
        await self.client.write_gatt_char(UUID_WRITE, data, response=False)

    async def close(self):
        await self.client.disconnect()


class Tcp:
    async def open(self, args):
        self.reader, self.writer = await asyncio.open_connection(args.host, args.port)
        self.buf = bytearray()
        asyncio.create_task(self.pump())
        print(f"TCP connected to {args.host}:{args.port}")

    async def pump(self):
        while True:
            data = await self.reader.read(512)
            if not data:
                return
            self.buf.extend(data)

    async def write(self, data):
        self.writer.write(data)
        await self.writer.drain()

    async def close(self):
        self.writer.close()


async def ask(link, cmd, timeout=4.0):
    """Send one command, return the text before the '>' prompt."""
    link.buf.clear()
    await link.write(cmd.encode() + b"\r")
    loop = asyncio.get_event_loop()
    end = loop.time() + timeout
    while loop.time() < end:
        if b">" in link.buf:
            break
        await asyncio.sleep(0.01)
    text = link.buf.decode(errors="replace")
    return text.replace(">", "").replace("\n", "").strip("\r ")


def lines(text):
    return [l.strip() for l in text.split("\r") if l.strip()]


async def checks(link):
    r = await ask(link, "ATZ")
    check("ATZ identifies as ELM327", "ELM327" in r, r.replace("\r", " | "))
    for cmd in ("ATE0", "ATL0", "ATS1", "ATH0", "ATSP0", "ATAT1"):
        r = await ask(link, cmd)
        check(f"{cmd} -> OK", lines(r)[-1:] == ["OK"], r.replace("\r", " | "))

    r = await ask(link, "ATI")
    check("ATI", "ELM327" in r, r)
    r = await ask(link, "ATDPN")
    check("ATDPN reports automatic CAN 11/500", lines(r)[-1:] == ["A6"], r)

    r = await ask(link, "0100")
    ls = lines(r)
    check("0100 searches, then answers", ls[:1] == ["SEARCHING..."] and any(l.startswith("41 00") for l in ls), r.replace("\r", " | "))
    check("0100 answered by both ECU and TCU", sum(l.startswith("41 00") for l in ls) == 2, f"{sum(l.startswith('41 00') for l in ls)} answers")

    r = await ask(link, "0105")
    check("0105 coolant temperature", any(l.startswith("41 05") for l in lines(r)), r.replace("\r", " | "))
    r = await ask(link, "010C")
    check("010C engine speed", any(l.startswith("41 0C") and len(l.split()) == 4 for l in lines(r)), r.replace("\r", " | "))

    r = await ask(link, "0902")
    ls = lines(r)
    check("0902 VIN comes as numbered lines", "014" in ls and any(l.startswith("0:") for l in ls) and any(l.startswith("2:") for l in ls), r.replace("\r", " | "))
    vin = "".join(chr(int(b, 16)) for l in ls if ":" in l for b in l.split(":")[1].split())
    check("VIN decodes", "SIMULATEDVIN00001" in vin, vin)

    r = await ask(link, "03")
    ls = lines(r)
    check("03 stored codes: ECU lists two, TCU one", "43 02 03 00 01 71" in ls and any(l.startswith("43 01 07 00") for l in ls), r.replace("\r", " | "))
    r = await ask(link, "07")
    check("07 pending code from the ECU", any(l.startswith("47 01 04 20") for l in lines(r)), r.replace("\r", " | "))

    r = await ask(link, "ATRV")
    check("ATRV reads module voltage", lines(r)[-1:] == ["13.8V"], r)

    r = await ask(link, "ATH1")
    r = await ask(link, "0105")
    check("ATH1 shows the responding id", any(l.startswith("7E8 03 41 05") for l in lines(r)), r.replace("\r", " | "))
    r = await ask(link, "ATH0")

    r = await ask(link, "ATSH7E0")
    check("ATSH7E0 -> OK", lines(r)[-1:] == ["OK"], r)
    r = await ask(link, "0105")
    check("a physical request is answered by that ECU only", len(lines(r)) == 1 and lines(r)[0].startswith("41 05"), r.replace("\r", " | "))
    r = await ask(link, "ATSH7DF")
    r = await ask(link, "ZZZZ")
    check("garbage -> ?", lines(r)[-1:] == ["?"], r)

    r = await ask(link, "04")
    ls = lines(r)
    check("04: the ECU clears, the TCU refuses with 'conditions not correct'", "44" in ls and "7F 04 22" in ls, r.replace("\r", " | "))
    r = await ask(link, "03")
    ls = lines(r)
    check("03 afterwards: the ECU has nothing stored, the TCU still has its code", "43 00" in ls and any(l.startswith("43 01 07 00") for l in ls), r.replace("\r", " | "))


async def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("transport", choices=["ble", "tcp"])
    ap.add_argument("--host", default="192.168.0.10")
    ap.add_argument("--port", type=int, default=35000)
    ap.add_argument("--cmd", action="append", help="send this command instead of the checks (repeatable)")
    args = ap.parse_args()

    link = Ble() if args.transport == "ble" else Tcp()
    await link.open(args)
    try:
        if args.cmd:
            for c in args.cmd:
                print(f"> {c}\n{await ask(link, c)}")
        else:
            await checks(link)
    finally:
        await link.close()

    if results:
        print(f"\n{sum(results)}/{len(results)} checks passed")
        return 0 if all(results) else 1
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
