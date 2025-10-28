#!/usr/bin/env python3
"""Dump exact termios settings used by pyserial"""

import serial
import termios
import sys

port = serial.Serial('/dev/ttyUSB0', 19200, timeout=0.2)

# Get the termios settings
[iflag, oflag, cflag, lflag, ispeed, ospeed, cc] = termios.tcgetattr(port.fd)

print("=== Termios Settings from Python pyserial ===")
print(f"iflag:  0x{iflag:08x}")
print(f"oflag:  0x{oflag:08x}")
print(f"cflag:  0x{cflag:08x}")
print(f"lflag:  0x{lflag:08x}")
print(f"ispeed: 0x{ispeed:08x} ({ispeed})")
print(f"ospeed: 0x{ospeed:08x} ({ospeed})")

# Test communication
port.write(bytes([0xAA, 0xFF, 0x0F, 0x0E]))
print(f"\nSent HARD_RESET command")

port.close()
