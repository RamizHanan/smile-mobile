#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 07/12/2020
Description: This module communicates the PWM values to the hardware on the
             vehicle.
'''
import rospy
from std_msgs.msg import Int16MultiArray
import serial
import time
import struct

#Global varialbes
com_port = '/dev/ttyUSB0'
motor_control_serial = serial.Serial(com_port, 115200)
START_BYTE = 0xDB
END_BYTE = 0xBD
SENDING_PERIOD = 0.010 #Write values ten times a second

#The PWM values to update and continously write to the motor driving hardware
pwm_values = [0, 0, 0, 0]

def pwm_subscriber_callback(pwm_msg):
    '''
    Subscriber callback for the pwm values received over the ROS network.

    Parameters:
        pwm_msg: A 4 value Int16MultiArray containing the pwm values for PWM values
            that must be bounded between -255, 255.
            [FL_motor, FR_motor, BR_motor, BL_motor]
    Returns:
        N/A
    '''
    global pwm_values
    pwm_values = pwm_msg.data


def main():
    '''
    Main loop to run to start receiving PWM values and sending them to
    the vehicle.

    Parameters:
        N/A
    Returns:
        N/A
    '''
    global pwm_values

    rospy.init_node("motor_driver")

    #Initialize a subscriber for the PWM values
    pwm_topic = rospy.get_namespace() + 'pwm'
    pwm_subscriber = rospy.Subscriber(pwm_topic, Int16MultiArray, pwm_subscriber_callback)

    #Main loop
    while not rospy.is_shutdown():
        try:
            start_time = time.time()

            #Write the start byte
            b = bytearray()
            b.append(START_BYTE)
            motor_control_serial.write(b)

            #Write the 4 pwm values (16-bit)
            for i in range(4):
                pwm = struct.pack('h', pwm_values[i])
                motor_control_serial.write(pwm)

            #Write the end byte
            b = bytearray()
            b.append(END_BYTE)
            motor_control_serial.write(b)

            #Hold out the remaining time to ensure consistent sending rate
            end_time = time.time()
            elapsed = (end_time - start_time)
            if(elapsed < SENDING_PERIOD):
                time.sleep(SENDING_PERIOD - elapsed)


        except rospy.ROSInterruptException:
            rospy.loginfo("Exiting Motor Driver Script")

if __name__ == "__main__":
    main()
