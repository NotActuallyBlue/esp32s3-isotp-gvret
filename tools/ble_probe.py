#!/usr/bin/env python3
"""Talk to the dongle over BLE the way Simos Tools does, from a desktop.

Needs the bleak library (pip install bleak) and a Bluetooth adapter. Most tests need something that answers
requests: use the bench simulator (BOOT button hold 5-8 s) or a dongle on a real bus.

    python3 tools/ble_probe.py scan
    python3 tools/ble_probe.py smoke        # handshake, settings, single and multi-frame requests
    python3 tools/ble_probe.py split        # a request too long for one packet (0xF2 continuation chunks)
    python3 tools/ble_probe.py persist      # register requests, enable persist mode, measure the stream
    python3 tools/ble_probe.py probe        # what Simos Tools' "create PID" test sees: enable, wait, disable
    python3 tools/ble_probe.py all
"""
import argparse
import asyncio
import struct
import sys
import time

from bleak import BleakClient, BleakScanner

DEFAULT_NAME = "BLE_TO_ISOTP20"
UUID_WRITE = "0000abf1-0000-1000-8000-00805f9b34fb"
UUID_NOTIFY = "0000abf2-0000-1000-8000-00805f9b34fb"

HEADER = struct.Struct("<BBHHH")        # id, flags, rxID, txID, size
HEADER_ID, PARTIAL_ID = 0xF1, 0xF2
F_PER_ENABLE, F_PER_CLEAR, F_PER_ADD, F_SPLIT = 1, 2, 4, 8
F_SETTINGS_GET, F_SETTINGS = 64, 128

ECU = (0x7E8, 0x7E0)    # (rxID = where answers come from, txID = where requests go)
TCU = (0x7E9, 0x7E1)

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{PASS if ok else FAIL}] {name}" + (f"  ({detail})" if detail else ""))
    return ok


def packet(flags, ids, payload=b""):
    rx, tx = ids
    return HEADER.pack(HEADER_ID, flags, rx, tx, len(payload)) + payload


def parse(data):
    """Split a notification into (flags, rx, tx, payload) packets (several may be packed together)."""
    out, pos = [], 0
    while pos + HEADER.size <= len(data):
        hd, flags, rx, tx, size = HEADER.unpack_from(data, pos)
        if hd != HEADER_ID:
            break
        out.append((flags, rx, tx, bytes(data[pos + HEADER.size:pos + HEADER.size + size])))
        pos += HEADER.size + size
    return out


class Dongle:
    def __init__(self, client):
        self.client = client
        self.rx = []                    # (time, flags, rx, tx, payload)
        self.partial = None

    def _on_notify(self, _char, data):
        now = time.monotonic()
        data = bytes(data)
        # Long replies arrive as a header chunk (split flag) followed by 0xF2 continuation chunks
        if self.partial is not None and data[:1] == bytes([PARTIAL_ID]):
            self.partial += data[2:]
            hd, flags, rx, tx, size = HEADER.unpack_from(self.partial)
            if len(self.partial) >= HEADER.size + size:
                for p in parse(self.partial):
                    self.rx.append((now,) + p)
                self.partial = None
            return
        if data[:1] == bytes([HEADER_ID]) and len(data) >= HEADER.size:
            hd, flags, rx, tx, size = HEADER.unpack_from(data)
            if flags & F_SPLIT and len(data) < HEADER.size + size:
                self.partial = data
                return
        for p in parse(data):
            self.rx.append((now,) + p)

    async def send(self, flags, ids, payload=b""):
        await self.client.write_gatt_char(UUID_WRITE, packet(flags, ids, payload), response=False)

    async def send_raw(self, data):
        await self.client.write_gatt_char(UUID_WRITE, data, response=False)

    def clear(self):
        self.rx.clear()

    async def wait_for(self, count=1, timeout=2.0, predicate=None):
        """Wait until `count` matching packets have arrived; returns the list that arrived."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            matches = [r for r in self.rx if predicate is None or predicate(r)]
            if len(matches) >= count:
                return matches
            await asyncio.sleep(0.01)
        return [r for r in self.rx if predicate is None or predicate(r)]


async def connect(name, address=None):
    print(f"Looking for {address or name} ...")
    device = await BleakScanner.find_device_by_address(address, timeout=10) if address else \
        await BleakScanner.find_device_by_name(name, timeout=10)
    if device is None:
        sys.exit(f"Could not find '{address or name}'. Is the dongle powered and not connected to a phone?")
    client = BleakClient(device)
    await client.connect()
    dongle = Dongle(client)
    await client.start_notify(UUID_NOTIFY, dongle._on_notify)
    print(f"Connected to {device.address}, MTU {client.mtu_size}")
    return dongle


# --------------------------------------------------------------------------------------------- tests
async def test_handshake(d):
    print("handshake and settings")
    d.clear()
    await d.send_raw(HEADER.pack(HEADER_ID, 0xC7, 0, 0, 6) + b"\x01\x02\"$Fs")
    got = await d.wait_for(1, 1.5, lambda r: r[1] == 0xFF)
    check("handshake answered with 0xFF", bool(got))

    d.clear()
    await d.send(F_SETTINGS | F_SETTINGS_GET | 1, ECU)               # GET stmin
    got = await d.wait_for(1, 1.5, lambda r: r[1] == (F_SETTINGS | 1))
    check("stmin GET answered", bool(got))
    await d.send(F_SETTINGS | 1, ECU, struct.pack("<H", 1000))       # SET stmin 1000 us
    d.clear()
    await d.send(F_SETTINGS | F_SETTINGS_GET | 1, ECU)
    got = await d.wait_for(1, 1.5, lambda r: r[1] == (F_SETTINGS | 1))
    check("stmin SET took effect", bool(got) and struct.unpack("<H", got[0][4][:2])[0] == 1000,
          f"read back {struct.unpack('<H', got[0][4][:2])[0]}" if got else "no reply")

    # Both links get the app's smallest STmin: the dongle must still respect what the module asks for
    await d.send(F_SETTINGS | 1, TCU, struct.pack("<H", 500))

    for flag, label in ((3, "persist delay"), (4, "persist queue delay")):
        await d.send(F_SETTINGS | flag, ECU, struct.pack("<H", 33))
    d.clear()
    await d.send(F_SETTINGS | F_SETTINGS_GET | 3, (0, 0))
    got = await d.wait_for(1, 1.5, lambda r: r[1] == (F_SETTINGS | 3))
    check("persist delay SET/GET", bool(got) and struct.unpack("<H", got[0][4][:2])[0] == 33)


async def request(d, ids, payload, timeout=2.0):
    d.clear()
    t0 = time.monotonic()
    await d.send(0, ids, payload)
    # Replies mirror the IDs of the request: rxID holds the ID the request was sent to
    got = await d.wait_for(1, timeout, lambda r: r[2] == ids[1] and r[3] == ids[0])
    return (got[0][4], (got[0][0] - t0) * 1000) if got else (None, None)


async def test_smoke(d):
    await test_handshake(d)
    print("requests")
    for name, ids in (("ECU", ECU), ("TCU", TCU)):
        reply, ms = await request(d, ids, bytes([0x3E, 0x00]))
        check(f"{name} tester present", reply == bytes([0x7E, 0x00]), f"{ms:.0f} ms" if reply else "no reply")

        reply, ms = await request(d, ids, bytes([0x22, 0xF1, 0x90, 0xF1, 0x8C, 0x20, 0x01]))
        ok = reply is not None and reply[0] == 0x62 and len(reply) == 1 + 3 * 10
        check(f"{name} multi-frame read (3 DIDs)", ok, f"{len(reply)} bytes in {ms:.0f} ms" if reply else "no reply")

        reply, ms = await request(d, ids, bytes([0x10, 0x03]))
        check(f"{name} session control", reply is not None and reply[0] == 0x50)

        big = bytes([0x2C, 0x01]) + bytes(range(67))                       # 69 bytes: first frame plus 9 consecutive
        reply, ms = await request(d, ids, big)
        check(f"{name} 69-byte request", reply is not None and reply[0] == 0x6C, f"{ms:.0f} ms" if reply else "no reply")


async def test_split(d):
    print("split packets (request longer than one BLE packet)")
    payload = bytes([0x2C, 0x01]) + bytes(range(67))
    d.clear()
    first = packet(F_SPLIT, ECU, payload)[:20]
    rest = packet(F_SPLIT, ECU, payload)[20:]
    await d.send_raw(first)
    n = 1
    for i in range(0, len(rest), 18):
        await d.send_raw(bytes([PARTIAL_ID, n]) + rest[i:i + 18])
        n += 1
    got = await d.wait_for(1, 2.0, lambda r: r[2] == ECU[1])
    check("69-byte request sent in 20-byte chunks was reassembled", bool(got) and got[0][4][0] == 0x6C)

    d.clear()
    await d.send_raw(first)
    await d.send_raw(bytes([PARTIAL_ID, 5]) + rest[:18])               # wrong sequence number
    got = await d.wait_for(1, 0.7, lambda r: r[2] == ECU[1])
    check("out-of-order chunk is dropped without a reply", not got)

    reply, _ = await request(d, ECU, bytes([0x3E, 0x00]))
    check("dongle still answers after the bad sequence", reply == bytes([0x7E, 0x00]))


async def register(d, ids, payload, enable=False):
    flags = F_PER_ADD | (F_PER_ENABLE if enable else 0)
    await d.send(flags, ids, payload)


async def test_persist(d, seconds=3.0):
    print(f"persist mode stream ({seconds:.0f} s)")
    await d.send(F_PER_CLEAR, ECU)
    await d.send(F_PER_CLEAR, TCU)
    d.clear()
    await register(d, ECU, bytes([0x22, 0xF2, 0x00, 0xF2, 0x01]))
    await register(d, ECU, bytes([0x22, 0xF2, 0x02]))
    await register(d, TCU, bytes([0x22, 0xF3, 0x00, 0xF3, 0x01]), enable=True)
    t0 = time.monotonic()
    await asyncio.sleep(seconds)
    await d.send(F_PER_CLEAR, ECU)                                     # disables persist mode
    await asyncio.sleep(0.2)

    frames = [r for r in d.rx if r[0] - t0 >= 0 and r[1] == 0 and r[4][:1] == b"\x62"]
    ecu = [r for r in frames if len(r[4]) in (21, 11)]
    check("persist stream produced responses", len(frames) > 10, f"{len(frames)} responses, {len(frames) / seconds:.0f}/s")
    check("replies carry a timestamp in the ID fields", all((r[2], r[3]) != (0, 0) for r in frames[1:]))
    stamps = [(r[2] << 16 | r[3]) for r in frames]
    check("timestamps increase", all(b >= a for a, b in zip(stamps, stamps[1:])) if len(stamps) > 1 else False)
    lens = sorted({len(r[4]) for r in frames})
    check("both ECU and TCU requests answered", len(lens) >= 2, f"response sizes {lens}")


async def test_probe(d, attempts=8, window=0.08):
    """What Simos Tools does when it creates a PID: register, enable, wait ~80 ms for data, disable."""
    print(f"create-PID probe ({attempts} attempts, {window * 1000:.0f} ms window)")
    firsts = []
    for _ in range(attempts):
        await d.send(F_PER_CLEAR, ECU)
        await d.send(F_PER_CLEAR, TCU)
        await asyncio.sleep(0.15)
        d.clear()
        t0 = time.monotonic()
        await register(d, ECU, bytes([0x22, 0xF2, 0x00]))
        await register(d, TCU, bytes([0x22, 0xF3, 0x00]), enable=True)
        got = await d.wait_for(1, window, lambda r: r[1] == 0 and r[4][:1] == b"\x62")
        firsts.append((got[0][0] - t0) * 1000 if got else None)
        await asyncio.sleep(max(0, window - (time.monotonic() - t0)))
        await d.send(F_PER_CLEAR, ECU)
        await asyncio.sleep(0.3)
    hits = [f for f in firsts if f is not None]
    check("first persist reply arrived inside the window every time", len(hits) == attempts,
          f"{len(hits)}/{attempts}" + (f", first reply after {min(hits):.0f}-{max(hits):.0f} ms" if hits else ""))


async def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("test", choices=["scan", "smoke", "split", "persist", "probe", "all"])
    parser.add_argument("--name", default=DEFAULT_NAME, help="BLE name (default %(default)s)")
    parser.add_argument("--address", help="BLE address instead of the name")
    args = parser.parse_args()

    if args.test == "scan":
        for device, adv in (await BleakScanner.discover(timeout=8, return_adv=True)).values():
            print(f"{device.address}  {adv.rssi:4d} dBm  {device.name or adv.local_name or '-'}")
        return 0

    d = await connect(args.name, args.address)
    try:
        if args.test in ("smoke", "all"):
            await test_smoke(d)
        if args.test in ("split", "all"):
            await test_split(d)
        if args.test in ("persist", "all"):
            await test_persist(d)
        if args.test in ("probe", "all"):
            await test_probe(d)
    finally:
        await d.client.disconnect()

    print(f"\n{sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
