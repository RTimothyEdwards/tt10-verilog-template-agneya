#!/usr/bin/env python3
#
# mmult_comm.py:
#
# Script to communicate with the matrix multiplier through UART
# through the Arty A7 FPGA board.

from pyftdi.ftdi import Ftdi
import time
import sys, os
import pyftdi.serialext

import binascii
from io import StringIO

SR_WIP = 0b00000001  # Busy/Work-in-progress bit

def is_busy(device):
    return get_status(device) & SR_WIP


# This is roundabout but works. . .
s = StringIO()
Ftdi.show_devices(out=s)
devlist = s.getvalue().splitlines()[1:-1]
gooddevs = []
for dev in devlist:
    url = dev.split('(')[0].strip()
    name = '(' + dev.split('(')[1]
    if name == '(Digilent USB Device)' and url.endswith('/2'):
        gooddevs.append(url)

if len(gooddevs) == 0:
    print('Error:  No matching FTDI devices on USB bus!')
    sys.exit(1)

elif len(gooddevs) > 1:
    print('Error:  Too many matching FTDI devices on USB bus!')
    Ftdi.show_devices()
    sys.exit(1)

else:
    print('Success: Found one matching FTDI device at ' + gooddevs[0])

# The project is configured to run at 120000 Baud
print('Setting communcation baud rate to 120000')
port = pyftdi.serialext.serial_for_url(gooddevs[0], baudrate=120000)
port.flush()

# Example data transmission and read-back.
print('Sending test data 01 02 03 04 05 06 07 08')
message = b'\x01\x02\x03\x04\x05\x06\x07\x08'
port.write(message)
value = port.read(4)
print("value = {:04x}".format(int.from_bytes(value, byteorder='big')))

print('Sending test data 10 20 30 40 50 60 70 80')
message = b'\x10\x20\x30\x40\x50\x60\x70\x80'
port.write(message)
value = port.read(4)
print("value = {:04x}".format(int.from_bytes(value, byteorder='big')))

print('Sending test data 08 07 06 05 04 03 02 01')
message = b'\x08\x07\x06\x05\x04\x03\x02\x01'
port.write(message)
value = port.read(4)
print("value = {:04x}".format(int.from_bytes(value, byteorder='big')))

print('Re-sending test data 01 02 03 04 05 06 07 08')
message = b'\x01\x02\x03\x04\x05\x06\x07\x08'
port.write(message)
value = port.read(4)
print("value = {:04x}".format(int.from_bytes(value, byteorder='big')))

print('Done with test')
port.close()

