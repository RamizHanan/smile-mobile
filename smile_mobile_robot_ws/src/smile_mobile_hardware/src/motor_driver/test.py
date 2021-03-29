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
test_data = [0, 0, 0, 0]
start_byte = 0xDB
end_byte = 0xBD
sending_period = 0.005 #Time inbetween sending commands

while True:
    #This method of the utility timer makes for more accurate
    #and datarates. Rather than doing 1 simple time.sleep()
    #This more accurately controls the rate of the loop.
    #However, this technique does fail at higher frequencies
    #because I think that the time.sleep() can't do well
    #below a certain resolution. My best was sending_period
    #of 0.005 s
    start_time = time.time()

    #Write the start byte
    b = bytearray()
    b.append(start_byte)
    ser.write(b)

    #Write the 4 pwm values (16-bit)
    for i in range(4):
        pwm = struct.pack('h', test_data[i])
        #print(pwm)
        #print(pwm)
        ser.write(pwm)

    #Write the end byte
    b = bytearray()
    b.append(end_byte)
    ser.write(b)

    end_time = time.time()
    elapsed = (end_time - start_time)
    if(elapsed < sending_period):
        time.sleep(sending_period - elapsed)
    #time.sleep(0.005)
