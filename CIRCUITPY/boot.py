# SPDX-FileCopyrightText: 2021 Kattni Rembor for Adafruit Industries
# SPDX-License-Identifier: Unlicense
"""
CircuitPython Essentials Storage CP Filesystem boot.py file
"""
import board
from digitalio import DigitalInOut, Direction, Pull
import storage

pin = DigitalInOut(board.A2)
pin.switch_to_input(pull=Pull.UP)
print ("DipSW1=",pin.value)
# If the pin is connected to ground, the filesystem is writable by CircuitPython
storage.remount("/", readonly=pin.value)

