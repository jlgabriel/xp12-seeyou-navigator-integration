#!/usr/bin/env python3
"""
NMEA Sniffer - Captures and logs NMEA sentences from a COM port.
Used to reverse-engineer what Condor 3 sends to the Naviter dongle.

Setup:
  1. Install com0com virtual serial port pair (e.g., COM3 <-> COM4)
  2. Set Condor 3 NMEA output to COM3
  3. Run this script reading from COM4
"""

import serial
import time
import sys
import argparse
from datetime import datetime
from collections import Counter

def main():
    parser = argparse.ArgumentParser(description="NMEA Sniffer for Condor 3 / Naviter analysis")
    parser.add_argument("--port", default="COM4", help="COM port to read from (default: COM4)")
    parser.add_argument("--baud", type=int, default=4800, help="Baud rate (default: 4800)")
    parser.add_argument("--log", default=None, help="Save raw output to file (optional)")
    args = parser.parse_args()

    print(f"=== NMEA Sniffer ===")
    print(f"Port: {args.port} @ {args.baud} baud")
    print(f"Waiting for data... (Ctrl+C to stop)\n")

    sentence_counts = Counter()
    total_lines = 0
    start_time = None
    log_file = None

    if args.log:
        log_file = open(args.log, "w", encoding="utf-8")
        print(f"Logging to: {args.log}\n")

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        print(f"Port {args.port} opened successfully.\n")
    except serial.SerialException as e:
        print(f"ERROR: Could not open {args.port}: {e}")
        print("Make sure com0com is configured and the port exists.")
        sys.exit(1)

    try:
        while True:
            line = ser.readline()
            if not line:
                continue

            try:
                text = line.decode("ascii", errors="replace").strip()
            except Exception:
                text = repr(line)

            if not text:
                continue

            if start_time is None:
                start_time = time.time()

            total_lines += 1
            now = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            # Extract sentence type (e.g., $GPRMC, $GPGGA, $LXWP0)
            sentence_type = "UNKNOWN"
            if text.startswith("$") or text.startswith("!"):
                parts = text.split(",", 1)
                sentence_type = parts[0]
            sentence_counts[sentence_type] += 1

            # Display
            print(f"[{now}] {text}")

            # Log to file
            if log_file:
                log_file.write(f"[{now}] {text}\n")
                log_file.flush()

    except KeyboardInterrupt:
        elapsed = time.time() - start_time if start_time else 0
        print(f"\n\n=== Capture Summary ===")
        print(f"Duration: {elapsed:.1f} seconds")
        print(f"Total sentences: {total_lines}")
        if elapsed > 0:
            print(f"Rate: {total_lines / elapsed:.1f} sentences/sec")
        print(f"\nSentence types:")
        for stype, count in sentence_counts.most_common():
            print(f"  {stype}: {count}")
        print()
    finally:
        ser.close()
        if log_file:
            log_file.close()
            print(f"Log saved to: {args.log}")


if __name__ == "__main__":
    main()
