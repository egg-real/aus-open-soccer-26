# SPDX-FileCopyrightText: 2018 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""CircuitPython Essentials UART Serial example"""
import board
import busio
import digitalio
import serial

uart = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=3000)
while True:
    data = uart.read(1)  # read up to 32 bytes
    #print(data)  # this is a bytearray type
    bitstring = ''.join(f"{byte:08b}" for byte in data)
    print(bitstring)
#    if data is not None:
        # convert bytearray to string
#       data_string = ''.join([chr(b) for b in data])
        #print(data_string, end="")
