#!/usr/bin/env python3
"""Read the dongle's flash log (see main/flashlog.c) over USB and save it as text.

Usage:
    python3 tools/read_log.py                 # auto-detect port, save all sessions
    python3 tools/read_log.py --port /dev/ttyACM0
    python3 tools/read_log.py --bin dump.bin  # decode an existing raw dump

Crash reports ([CRASH] lines) are decoded to function names and source lines using the ELF from the
latest build, so decode them with the same firmware that was running on the dongle.
"""
import argparse
import csv
import datetime
import glob
import os
import re
import struct
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SLOT_COUNT = 6          # must match FLASHLOG_SLOT_COUNT in main/flashlog.c
MAGIC = 0x474F4C42
HEADER_SIZE = 32

RESET_REASONS = [
    "UNKNOWN", "POWERON", "EXT", "SW", "PANIC", "INT_WDT", "TASK_WDT", "WDT",
    "DEEPSLEEP", "BROWNOUT", "SDIO", "USB", "JTAG", "EFUSE", "PWR_GLITCH", "CPU_LOCKUP",
]


def parse_int(text):
    text = text.strip()
    mult = 1
    if text[-1:] in "KkMm":
        mult = 1024 if text[-1] in "Kk" else 1024 * 1024
        text = text[:-1]
    return int(text, 0) * mult


def log_partition():
    """Offset and size of the 'log' partition, read from partitions.csv so the two never disagree."""
    with open(os.path.join(ROOT, "partitions.csv")) as f:
        for row in csv.reader(line for line in f if not line.lstrip().startswith("#")):
            row = [c.strip() for c in row]
            if len(row) >= 5 and row[0] == "log":
                return parse_int(row[3]), parse_int(row[4])
    raise SystemExit("No 'log' partition found in partitions.csv")


def esptool_cmd():
    pio_home = os.path.expanduser("~/.platformio")
    bundled = os.path.join(pio_home, "packages", "tool-esptoolpy", "esptool.py")
    pio_python = os.path.join(pio_home, "penv", "bin", "python")
    if os.path.exists(bundled):
        return [pio_python if os.path.exists(pio_python) else sys.executable, bundled]
    return [sys.executable, "-m", "esptool"]


def dump_flash(port, out_path, offset, size, baud=921600):
    cmd = esptool_cmd() + ["--chip", "esp32s3", "--baud", str(baud)]
    if port:
        cmd += ["--port", port]
    cmd += ["read_flash", hex(offset), hex(size), out_path]
    print("Reading log partition from dongle...")
    subprocess.run(cmd, check=True)


def decode(raw, size):
    slot_size = (size // SLOT_COUNT) & ~0xFFF
    sessions = []
    for i in range(SLOT_COUNT):
        slot = raw[i * slot_size:(i + 1) * slot_size]
        if len(slot) < HEADER_SIZE:
            continue
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


def find_tool(pattern):
    matches = glob.glob(os.path.expanduser(pattern))
    return matches[0] if matches else None


def decode_crashes(text):
    """Append function/file:line for every address on a [CRASH] line."""
    addr2line = find_tool("~/.platformio/packages/toolchain-xtensa-esp*/bin/xtensa-esp32s3-elf-addr2line")
    elf = find_tool(os.path.join(ROOT, ".pio", "build", "*", "firmware.elf"))
    if not addr2line or not elf:
        return text

    out = []
    for line in text.split("\n"):
        out.append(line)
        if "[CRASH]" not in line or "sha256" in line:
            continue
        if "backtrace:" in line:
            addrs = re.findall(r"0x[0-9A-Fa-f]{8}", line.split("backtrace:")[1])
        else:
            m = re.search(r"PC (0x[0-9A-Fa-f]{8})", line)      # skip vaddr: it is data, not code
            addrs = [m.group(1)] if m else []
        if not addrs:
            continue
        try:
            result = subprocess.run([addr2line, "-pfiaC", "-e", elf] + addrs, capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            continue
        for decoded in result.stdout.strip().split("\n"):
            if decoded:
                out.append("        -> " + decoded)
    return "\n".join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (default: auto-detect)")
    parser.add_argument("--bin", help="decode an existing raw dump instead of reading the dongle")
    parser.add_argument("--out", help="output text file (default: dongle-log-<time>.txt)")
    args = parser.parse_args()

    offset, size = log_partition()

    if args.bin:
        with open(args.bin, "rb") as f:
            raw = f.read()
    else:
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "log.bin")
            dump_flash(args.port, path, offset, size)
            with open(path, "rb") as f:
                raw = f.read()

    sessions = decode(raw, size)
    if not sessions:
        print("No log sessions found (is the logging firmware flashed?)")
        return 1

    out = args.out or datetime.datetime.now().strftime("dongle-log-%Y%m%d-%H%M%S.txt")
    crashes = 0
    with open(out, "w") as f:
        for boot, slot, reason, text in sessions:
            text = decode_crashes(text)
            crashes += text.count("[CRASH] task")
            f.write(f"===== Boot #{boot} (slot {slot}, reset reason: {reason}, {len(text)} bytes) =====\n")
            f.write(text)
            if not text.endswith("\n"):
                f.write("\n")
            f.write("\n")

    print(f"\nSaved {len(sessions)} session(s) to {out}")
    for boot, slot, reason, text in sessions:
        print(f"  Boot #{boot}: reset {reason}, {text.count(chr(10))} lines")
    if crashes:
        print(f"\n{crashes} crash report(s) found: search the file for [CRASH].")
    print("Plugging the dongle into this PC boots it, so the newest session is usually that desk boot;\n"
          "the car session is the one before it. Check the reset reason for BROWNOUT during cranking.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
