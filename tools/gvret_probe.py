#!/usr/bin/env python3
"""Talk to the dongle's SavvyCAN (GVRET) mode over Wi-Fi the way SavvyCAN does.

Put the dongle in SavvyCAN mode and, to test without a car, in the bench simulator as well (hold BOOT 6.5-8 s with
SavvyCAN as the saved mode). Join the dongle's Wi-Fi network (ISOTP-SAVVYCAN) first.

    python3 tools/gvret_probe.py [--host 192.168.0.10] [--port 23]
"""
import argparse
import socket
import struct
import sys
import time

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{PASS if ok else FAIL}] {name}" + (f"  ({detail})" if detail else ""))


def read_for(sock, seconds):
    sock.settimeout(0.1)
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        try:
            data = sock.recv(4096)
            if not data:
                break
            buf.extend(data)
        except socket.timeout:
            pass
    return bytes(buf)


def split_frames(buf):
    """Pull the captured CAN frames (F1 00 ...) out of a byte stream; returns (frames, remaining bytes)."""
    frames, i = [], 0
    while i + 11 <= len(buf):
        if buf[i] == 0xF1 and buf[i + 1] == 0x00:
            dlc = buf[i + 10] & 0x0F
            if i + 11 + dlc > len(buf):
                break
            ts, ident = struct.unpack_from("<II", buf, i + 2)
            frames.append((ident & 0x1FFFFFFF, bytes(buf[i + 11:i + 11 + dlc])))
            i += 11 + dlc
        else:
            i += 1
    return frames, buf[i:]


def tx_frame(ident, data):
    data = bytes(data).ljust(8, b"\x55")
    return bytes([0xF1, 0x00]) + struct.pack("<I", ident) + bytes([0, len(data)]) + data + b"\x00"


REPLY_LEN = {0x01: 6, 0x06: 7, 0x07: 6, 0x09: 6, 0x0C: 3, 0x0D: 3}


def find_reply(buf, cmd):
    """The answer to one command inside a stream that may also carry captured frames (F1 00 ...)."""
    i = 0
    while i + 2 <= len(buf):
        if buf[i] == 0xF1:
            c = buf[i + 1]
            if c == 0x00:
                if i + 11 > len(buf):
                    break
                i += 11 + (buf[i + 10] & 0x0F)
                continue
            n = REPLY_LEN.get(c)
            if n and i + n <= len(buf):
                if c == cmd:
                    return bytes(buf[i:i + n])
                i += n
                continue
        i += 1
    return b""


def command(sock, cmd, expect_len):
    sock.sendall(bytes([0xF1, cmd]))
    return find_reply(read_for(sock, 0.5), cmd)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.0.10")
    ap.add_argument("--port", type=int, default=23)
    args = ap.parse_args()

    s = socket.create_connection((args.host, args.port), timeout=5)
    print(f"Connected to {args.host}:{args.port}")

    r = command(s, 0x0C, 3)
    check("number of buses", r == bytes([0xF1, 0x0C, 1]), r.hex())
    r = command(s, 0x07, 6)
    check("device info", r[:2] == bytes([0xF1, 0x07]) and len(r) == 6, r.hex())
    r = command(s, 0x06, 7)
    check("CAN bus parameters report 500000 baud", r[:3] == bytes([0xF1, 0x06, 0x01]) and struct.unpack("<I", r[3:7])[0] == 500000, r.hex())
    r = command(s, 0x09, 6)
    check("keepalive answered", r[:2] == bytes([0xF1, 0x09]), r.hex())
    r1 = command(s, 0x01, 6)
    time.sleep(0.2)
    r2 = command(s, 0x01, 6)
    check("time sync answers and the clock advances", len(r1) == 6 and len(r2) == 6 and struct.unpack("<I", r2[2:])[0] > struct.unpack("<I", r1[2:])[0], f"{r1.hex()} then {r2.hex()}")

    # A command split over two TCP segments must still be understood
    s.sendall(b"\xF1")
    time.sleep(0.3)
    s.sendall(b"\x0C")
    r = find_reply(read_for(s, 0.5), 0x0C)
    check("a command split over two segments is understood", r[:3] == bytes([0xF1, 0x0C, 1]), r.hex())

    # The bench simulator broadcasts test frames, so a capture has something in it
    seen = {}
    buf = read_for(s, 1.0)
    frames, _ = split_frames(buf)
    for i, d in frames:
        seen[i] = seen.get(i, 0) + 1
    check("bench traffic streams: 0x100 about every 10 ms, 0x200 every 20 ms, 0x300 every 100 ms",
          seen.get(0x100, 0) >= 80 and seen.get(0x200, 0) >= 40 and seen.get(0x300, 0) >= 8,
          f"in 1 s: 0x100 x{seen.get(0x100, 0)}, 0x200 x{seen.get(0x200, 0)}, 0x300 x{seen.get(0x300, 0)}")

    # Transmit a functional OBD request; the simulated ECU and TCU answer on 0x7E8 / 0x7E9
    read_for(s, 0.3)
    s.sendall(tx_frame(0x7DF, [0x02, 0x01, 0x00]))
    buf = read_for(s, 0.8)
    frames, _ = split_frames(buf)
    ids = {i for i, d in frames if i in (0x7E8, 0x7E9) and len(d) > 2 and d[1] == 0x41}
    check("a transmitted request is answered by both simulated modules", ids == {0x7E8, 0x7E9}, f"answers from {sorted(hex(i) for i in ids)}")

    # The same request, written in three pieces
    frame = tx_frame(0x7DF, [0x02, 0x01, 0x05])
    for part in (frame[:5], frame[5:9], frame[9:]):
        s.sendall(part)
        time.sleep(0.15)
    frames, _ = split_frames(read_for(s, 0.8))
    ids = {i for i, d in frames if i in (0x7E8, 0x7E9) and len(d) > 2 and d[1] == 0x41 and d[2] == 0x05}
    check("a transmit frame split into three segments still goes out", ids == {0x7E8, 0x7E9}, f"answers from {sorted(hex(i) for i in ids)}")

    # A steady stream in both directions (one request every 15 ms, like a logger polling an ECU): nothing is lost
    read_for(s, 0.3)
    count = 60
    for _ in range(count):
        s.sendall(tx_frame(0x7E0, [0x02, 0x01, 0x0D]))
        time.sleep(0.015)
    frames, _ = split_frames(read_for(s, 1.0))
    answers = sum(1 for i, d in frames if i == 0x7E8 and len(d) > 2 and d[1] == 0x41)
    check(f"{count} requests at 15 ms spacing, answers received", answers >= count * 0.9, f"{answers} of {count} answered")

    s.close()
    print(f"\n{sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
