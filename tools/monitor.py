#!/usr/bin/env python3
"""Read the ESP32-S3 UART log, resetting the board first so boot messages show.

Usage: monitor.py [port] [duration_seconds]
"""

import sys
import time

import serial  # pyserial, part of the ESP-IDF python env

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem5AE80117911"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
BAUD = 115200

s = serial.Serial()
s.port = PORT
s.baudrate = BAUD
s.timeout = 0.5
s.rts = False   # avoid holding EN low
s.dtr = False   # avoid pulling IO0 low (download mode)
s.open()

# Pulse EN (via RTS on CH343 auto-reset boards) to reboot the board
s.rts = True
time.sleep(0.1)
s.rts = False

start = time.time()
try:
    while time.time() - start < DURATION:
        data = s.read(4096)
        if data:
            sys.stdout.write(data.decode("utf-8", "replace"))
            sys.stdout.flush()
finally:
    s.close()
