#!/usr/bin/env python3
"""Read the dongle's flash log (see main/flashlog.c) over USB and save it as text.

Usage:
    python3 tools/read_log.py                 # auto-detect port, save all sessions
    python3 tools/read_log.py --port /dev/ttyACM0
    python3 tools/read_log.py --bin dump.bin  # decode an existing raw dump
"""
import argparse
import datetime
import os
import struct
import subprocess
import sys
import tempfile

# Must match partitions.csv and flashlog.c
LOG_OFFSET = 0x110000
LOG_SIZE = 0xF0000
SLOT_COUNT = 6
MAGIC = 0x474F4C42
HEADER_SIZE = 32

RESET_REASONS = [
    "UNKNOWN", "POWERON", "EXT", "SW", "PANIC", "INT_WDT", "TASK_WDT", "WDT",
    "DEEPSLEEP", "BROWNOUT", "SDIO", "USB", "JTAG", "EFUSE", "PWR_GLITCH", "CPU_LOCKUP",
]


def esptool_cmd():
    pio_home = os.path.expanduser("~/.platformio")
    bundled = os.path.join(pio_home, "packages", "tool-esptoolpy", "esptool.py")
    pio_python = os.path.join(pio_home, "penv", "bin", "python")
    if os.path.exists(bundled):
        return [pio_python if os.path.exists(pio_python) else sys.executable, bundled]
    return [sys.executable, "-m", "esptool"]


def dump_flash(port, out_path):
    cmd = esptool_cmd() + ["--chip", "esp32s3"]
    if port:
        cmd += ["--port", port]
    cmd += ["read_flash", hex(LOG_OFFSET), hex(LOG_SIZE), out_path]
    print("Reading log partition from dongle...")
    subprocess.run(cmd, check=True)


def decode(raw):
    slot_size = (LOG_SIZE // SLOT_COUNT) & ~0xFFF
    sessions = []
    for i in range(SLOT_COUNT):
        slot = raw[i * slot_size:(i + 1) * slot_size]
        magic, boot, reason, _ = struct.unpack_from("<IIII", slot)
        if magic != MAGIC or boot == 0xFFFFFFFF:
            continue
        body = slot[HEADER_SIZE:]
        end = body.find(b"\xff")
        text = body[:end if end >= 0 else len(body)].decode("utf-8", errors="replace")
        reason_name = RESET_REASONS[reason] if reason < len(RESET_REASONS) else str(reason)
        sessions.append((boot, i, reason_name, text))
    sessions.sort()
    return sessions


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (default: auto-detect)")
    parser.add_argument("--bin", help="decode an existing raw dump instead of reading the dongle")
    parser.add_argument("--out", help="output text file (default: dongle-log-<time>.txt)")
    args = parser.parse_args()

    if args.bin:
        with open(args.bin, "rb") as f:
            raw = f.read()
    else:
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "log.bin")
            dump_flash(args.port, path)
            with open(path, "rb") as f:
                raw = f.read()

    sessions = decode(raw)
    if not sessions:
        print("No log sessions found (is the logging firmware flashed?)")
        return 1

    out = args.out or datetime.datetime.now().strftime("dongle-log-%Y%m%d-%H%M%S.txt")
    with open(out, "w") as f:
        for boot, slot, reason, text in sessions:
            f.write(f"===== Boot #{boot} (slot {slot}, reset reason: {reason}, {len(text)} bytes) =====\n")
            f.write(text)
            if not text.endswith("\n"):
                f.write("\n")
            f.write("\n")

    print(f"\nSaved {len(sessions)} session(s) to {out}")
    for boot, slot, reason, text in sessions:
        print(f"  Boot #{boot}: reset {reason}, {text.count(chr(10))} lines")
    print("Plugging the dongle into this PC boots it, so the newest session is usually that desk boot;\n"
          "the car session is the one before it. Check the reset reason for BROWNOUT during cranking.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
