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
from std_srvs.srv import SetBool, SetBoolResponse

rospy.init_node("motor_driver")

#Global varialbes

#Load the com_port that the motor driver is connected to. This is from the parameter server
if rospy.has_param('/motor_driver_serial_port'):
    com_port = rospy.get_param('/motor_driver_serial_port')
else:
    rospy.logerr("PARAMETER 'motor_driver_serial' NOT LOADED IN PARAMETER SERVER. CANNOT ATTEMPT TO CONNECT TO SERIAL.")
    raise

serial_port = rospy.get_param('/motor_driver_serial_port')
motor_driver_serial = serial.Serial(serial_port, 115200)
START_BYTE = 0xDB
END_BYTE = 0xBD
SENDING_PERIOD = 0.010 #Write values ten times a second

#The PWM values to update and continously write to the motor driving hardware
pwm_values = [0, 0, 0, 0]
motors_enabled = True

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
    global motors_enabled
    if(motors_enabled):
        pwm_values = pwm_msg.data
    else:
        pwm_values = [0, 0, 0, 0]

def motor_state_callback(req):
    '''
    The motor enable/disable service will use this for the callback to enable/disable
    if the motors will send PWMs other than [0, 0, 0, 0] to it.

    Parameters:
        req: The request message (bool) to enable or disable the motor.
    Returns:
        SetBoolResponse: A response notifiying that the motors were enabled or disabled.
    '''
    global motors_enabled
    motors_enabled = req.data

    response = SetBoolResponse()
    response.success = True
    if(motors_enabled):
        response.message = "Motors Enabled"
    else:
        response.message = "Motors Disabled"

    return response
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
    global motors_enabled

    #Initialize a subscriber for the PWM values
    pwm_topic = rospy.get_namespace() + 'pwm'
    pwm_subscriber = rospy.Subscriber(pwm_topic, Int16MultiArray, pwm_subscriber_callback)

    motors_enabled = True
    rospy.Service('motors_enabled', SetBool, motor_state_callback)

    #Main loop
    while not rospy.is_shutdown():
        try:
            start_time = time.time()

            #Write the start byte
            b = bytearray()
            b.append(START_BYTE)
            motor_driver_serial.write(b)

            #Write the 4 pwm values (16-bit)
            for i in range(4):
                if(motors_enabled):
                    pwm = struct.pack('h', pwm_values[i])
                else:
                    pwm = struct.pack('h', 0)
                motor_driver_serial.write(pwm)

            #Write the end byte
            b = bytearray()
            b.append(END_BYTE)
            motor_driver_serial.write(b)

            #Hold out the remaining time to ensure consistent sending rate
            end_time = time.time()
            elapsed = (end_time - start_time)
            if(elapsed < SENDING_PERIOD):
                time.sleep(SENDING_PERIOD - elapsed)


        except rospy.ROSInterruptException:
            rospy.loginfo("Exiting Motor Driver Script")

if __name__ == "__main__":
    main()
