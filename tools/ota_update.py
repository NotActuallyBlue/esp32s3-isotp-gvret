#!/usr/bin/env python3
"""Upload firmware to the dongle over Wi-Fi (see main/ota_update.c).

1. Hold the dongle's BOOT button for 8-11 seconds and release while the screen says WIFI UPDATE.
2. Join the Wi-Fi network shown on the dongle's screen (password is on the screen too).
3. Run:  python3 tools/ota_update.py            (uploads the latest build)
         python3 tools/ota_update.py path/to/firmware.bin

The dongle checks that the file is ISOTP-BLE firmware, writes it to the inactive slot and restarts. If the new
firmware does not run for 20 seconds, the bootloader goes back to the old one.
"""
import argparse
import glob
import os
import sys
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class ProgressFile:
    """File wrapper so urllib streams the upload and we can print progress."""

    def __init__(self, path):
        self.f = open(path, "rb")
        self.size = os.path.getsize(path)
        self.sent = 0
        self.last_percent = -1

    def read(self, n=-1):
        data = self.f.read(n)
        self.sent += len(data)
        percent = self.sent * 100 // self.size
        if percent != self.last_percent:
            self.last_percent = percent
            print(f"\r  uploading... {percent}%", end="", flush=True)
        return data

    def __len__(self):
        return self.size


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("firmware", nargs="?", help="firmware.bin (default: the latest PlatformIO build)")
    parser.add_argument("--url", default="http://192.168.4.1", help="dongle address (default: %(default)s)")
    args = parser.parse_args()

    path = args.firmware
    if not path:
        matches = glob.glob(os.path.join(ROOT, ".pio", "build", "*", "firmware.bin"))
        if not matches:
            sys.exit("No firmware.bin found. Build first with: pio run")
        path = matches[0]
    if not os.path.isfile(path):
        sys.exit(f"File not found: {path}")

    size = os.path.getsize(path)
    print(f"Firmware: {path} ({size / 1024:.0f} KB)")

    try:
        urllib.request.urlopen(args.url + "/", timeout=5).read()
    except (urllib.error.URLError, OSError) as e:
        sys.exit(f"Cannot reach the dongle at {args.url} ({e}).\nIs it in WIFI UPDATE mode and is this computer on its Wi-Fi network?")

    request = urllib.request.Request(args.url + "/update", data=ProgressFile(path), method="POST",
                                     headers={"Content-Type": "application/octet-stream", "Content-Length": str(size)})
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            print("\n" + response.read().decode(errors="replace"))
    except urllib.error.HTTPError as e:
        sys.exit("\nThe dongle rejected the update: " + e.read().decode(errors="replace"))
    except (urllib.error.URLError, OSError) as e:
        sys.exit(f"\nUpload failed: {e}")

    print("Done. The dongle restarts into the new firmware; it is kept after 20 seconds of stable running.")


if __name__ == "__main__":
    main()
