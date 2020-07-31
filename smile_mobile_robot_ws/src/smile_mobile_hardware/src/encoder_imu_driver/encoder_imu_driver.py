#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell
Date Created: 07/12/2020
Description: This module communicates with the arduino nano collecting
        IMU data and encoder data.
'''
import rospy
from std_msgs.msg import Float32MultiArray
import serial
import time
import struct
import math
from sensor_msgs.msg import Imu

rospy.init_node("enc_imu_driver")

#Global variables

#Load the com_port that the motor driver is connected to. This is from the parameter server
if rospy.has_param('/enc_imu_serial_port'):
    com_port = rospy.get_param('/enc_imu_serial_port')
else:
    rospy.logerr("PARAMETER 'enc_imu_serial_port' NOT LOADED IN PARAMETER SERVER. CANNOT ATTEMPT TO CONNECT TO SERIAL.")
    raise

serial_port = rospy.get_param('/enc_imu_serial_port')
enc_imu_serial = serial.Serial(serial_port, 115200)
START_BYTE = hex(0xDA)
END_BYTE = hex(0xAD)
RECEIVING_PERIOD = 0.0050 #Needs to be at least twice as high of a freq as the sender

def main():
    '''
    Main loop to get IMU and encoder data from the Arduino.

    Parameters:
        N/A
    Returns:
        N/A
    '''

    #Initialize a publisher for the encoder data and imu data
    imu_topic = rospy.get_namespace() + 'imu'
    enc_topic = rospy.get_namespace() + 'encoders'
    imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=10)
    encoder_pub = rospy.Publisher(enc_topic, Float32MultiArray, queue_size=10)
    imu_msg = Imu()
    encoder_msg = Float32MultiArray()

    while not rospy.is_shutdown():
        try:
            start_time = time.time()

            if(enc_imu_serial.in_waiting > 0):

                #Read the start byte
                read_byte = hex(ord(enc_imu_serial.read(1)))

                #Read the data for imu and encoder
                if(read_byte == START_BYTE):

                    motor1_freq = enc_imu_serial.read(4)
                    motor1_freq = struct.unpack('<f', motor1_freq)[0]
                    motor2_freq = enc_imu_serial.read(4)
                    motor2_freq = struct.unpack('<f', motor2_freq)[0]
                    accel_x = enc_imu_serial.read(4)
                    accel_x = struct.unpack('<f', accel_x)[0]
                    accel_y = enc_imu_serial.read(4)
                    accel_y = struct.unpack('<f', accel_y)[0]
                    accel_z = enc_imu_serial.read(4)
                    accel_z = struct.unpack('<f', accel_z)[0]
                    gyro_x = enc_imu_serial.read(4)
                    gyro_x = struct.unpack('<f', gyro_x)[0]
                    gyro_y = enc_imu_serial.read(4)
                    gyro_y = struct.unpack('<f', gyro_y)[0]
                    gyro_z = enc_imu_serial.read(4)
                    gyro_z = struct.unpack('<f', gyro_z)[0]
                    roll = enc_imu_serial.read(4)
                    roll = struct.unpack('<f', roll)[0]
                    pitch = enc_imu_serial.read(4)
                    pitch = struct.unpack('<f', pitch)[0]
                    heading = enc_imu_serial.read(4)
                    heading = struct.unpack('<f', heading)[0]

                    #Convert heading to radians and measure it between -pi and pi
                    if(roll >= 180.0):
                        roll = (roll - 360.0) * (math.pi / 180.0)
                    else:
                        roll = roll * (math.pi / 180.0)
                    
                    if(pitch >= 180.0):
                        pitch = (pitch - 360.0) * (math.pi / 180.0)
                    else:
                        pitch = pitch * (math.pi / 180.0)
                        
                    if(heading  >= 180.0):       
                        heading = (heading - 360.0) * (math.pi / 180.0)
                    else:
                        heading = heading * (math.pi / 180.0)

                    end_byte = hex(ord(enc_imu_serial.read(1)))

                    if(end_byte == END_BYTE):
                        #Print statement for debugging purposes.
                        #print("accel X: ",accel_x)
                        #print("accel Y: ",accel_y)
                        #print("accel Z: ",accel_z)
                        #print("gyro X: ",gyro_x)
                        #print("gyro Y: ",gyro_y)
                        #print("gyro Z: ",gyro_z)
                        #print("roll:", roll)
                        #print("pitch:", pitch)
                        #print("heading: ",heading)
                        #print("---------------------- \n")

                        #Publish the encoder and IMU data
                        encoder_msg.data = [-1*motor1_freq, motor2_freq, -1*motor1_freq, motor2_freq]

                        imu_msg.linear_acceleration.x = accel_x # accel
                        imu_msg.linear_acceleration.y = accel_y
                        imu_msg.linear_acceleration.z = accel_z

                        imu_msg.angular_velocity.x = gyro_x # gyro
                        imu_msg.angular_velocity.y = gyro_y
                        imu_msg.angular_velocity.z = gyro_z

                        #The imu data is NOT in quaternion anymore.
                        #x --> roll, y --> pitch, z --> yaw(heading)
                        imu_msg.orientation.x = roll
                        imu_msg.orientation.y = pitch
                        imu_msg.orientation.z = heading

                        encoder_pub.publish(encoder_msg)
                        imu_pub.publish(imu_msg)

            end_time = time.time()
            elapsed = (end_time - start_time)
            if(elapsed < RECEIVING_PERIOD):
                time.sleep(RECEIVING_PERIOD - elapsed)

        except rospy.ROSInterruptException:
            rospy.loginfo("Exiting Encoder Imu Driver.")

if __name__ == "__main__":
    main()
