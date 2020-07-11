#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail
Date Created: 07/10/2020
Description: A Test script for communicating 4 16-bit PWM values to the
arduino.
'''
import serial
import time
import struct

#Initialize serial communication
ser = serial.Serial('COM12', 115200)
test_data = [0, 40, -65, 255]
start_byte = 0xDB
end_byte = 0xBD

while True:

    #Write the start byte
    b = bytearray()
    b.append(start_byte)
    ser.write(b)

    #Write the 4 pwm values (16-bit)
    for i in range(4):
        pwm = struct.pack('h', test_data[i])
        ser.write(pwm)

    #Write the end byte
    b = bytearray()
    b.append(end_byte)
    ser.write(b)

    time.sleep(0.010)
