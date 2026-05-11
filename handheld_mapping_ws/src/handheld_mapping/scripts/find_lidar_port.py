#!/usr/bin/env python3
"""Auto-detect YDLidar serial port (supports wireless ESP8266 passthrough).

Sends a stop + start scan command on each candidate port and checks for a
valid scan data response. Prints the port path to stdout. Exits 0 on success.
"""

import serial
import glob
import sys
import time
import struct


BAUD = 230400
SCAN_PORTS = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))


def probe(port: str, timeout: float = 1.5) -> bool:
    """Return True if a YDLidar (or wireless bridge) is found on *port*."""
    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except (serial.SerialException, OSError):
        return False

    try:
        # flush any stale data
        ser.reset_input_buffer()
        # send stop command twice (robust stop)
        ser.write(b'\xA5\x65')
        time.sleep(0.05)
        ser.write(b'\xA5\x65')
        time.sleep(0.05)
        ser.reset_input_buffer()

        # send start scan
        ser.write(b'\xA5\x60')

        # try to read a valid scan frame (aa 55 sync word + header)
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            waiting = ser.in_waiting
            if waiting:
                buf += ser.read(waiting)
            else:
                time.sleep(0.02)
            # look for scan frame header: aa 55 <ph> <ct> <lsn>
            idx = buf.find(b'\xaa\x55')
            if idx >= 0 and len(buf) - idx >= 7:
                ph, ct, lsn = struct.unpack_from('<HBB', buf, idx)
                if ph == 0x55AA and 1 <= lsn <= 1024:
                    return True
                # bad sync — discard and keep looking
                buf = buf[idx + 1:]
    finally:
        try:
            ser.write(b'\xA5\x65')  # stop scan
            ser.close()
        except Exception:
            pass

    return False


if __name__ == "__main__":
    if not SCAN_PORTS:
        print("/dev/ttyUSB0", file=sys.stderr)
        sys.stderr.write("[find_port] No /dev/ttyUSB* or /dev/ttyACM* found, "
                         "falling back to /dev/ttyUSB0\n")
        print("/dev/ttyUSB0")
        sys.exit(0)

    for port in SCAN_PORTS:
        sys.stderr.write(f"[find_port] probing {port} @ {BAUD}...\n")
        if probe(port):
            sys.stderr.write(f"[find_port] FOUND YDLidar on {port}\n")
            print(port)
            sys.exit(0)

    # nothing found — print default so launch doesn't break
    sys.stderr.write("[find_port] No YDLidar found, falling back to /dev/ttyUSB0\n")
    print("/dev/ttyUSB0")
    sys.exit(1)