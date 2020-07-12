#!/usr/bin/env python
import serial
import time
import struct
import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Float32MultiArray  # dont use
import tf

# def writePWM(pwmList):
#     global ser
#     startWrite = 0xDB
#     endWrite = 0xBD
#     b = bytearray()
#     b.append(startWrite)
#     ser.write(b)
#     for i in range(4): # 4 pwms in custom message
#         pwm_bytes = struct.pack('i', pwmList.data[i])

#         ser.write(pwm_bytes)

#     b=bytearray()
#     b.append(endWrite)
#     ser.write(b)

def writeJoyPWM(joy_msg):
    global ser
    start_byte = 0xDB
    end_byte = 0xBD
    int16pwm = Int16MultiArray()

    FWBW_pwmFL = (joy_msg.axes[1] * 75) 
    FWBW_pwmBL = (joy_msg.axes[1] * 75) 
    FWBW_pwmFR = (joy_msg.axes[1] * 75) 
    FWVW_pwmBR = (joy_msg.axes[1] * 75) 
    
    #1 is left, -1 is right
    if(joy_msg.axes[3] >= 0.0): 
        LR_pwmFL = 0
        LR_pwmBL = 0
        LR_pwmFR = (joy_msg.axes[3] * 50) 
        LR_pwmBR = (joy_msg.axes[3] * 50) 
    else: 
        LR_pwmFL = 0 - (joy_msg.axes[3] * 50) 
        LR_pwmBL = 0 - (joy_msg.axes[3] * 50) 
        LR_pwmFR = 0 #- (joy_msg.axes[3] * 50) 
        LR_pwmBR = 0 #- (joy_msg.axes[3] * 50) 

    total_pwmFL = FWBW_pwmFL + LR_pwmFL
    total_pwmBL = FWBW_pwmBL + LR_pwmBL
    total_pwmFR = FWBW_pwmFR + LR_pwmFR
    total_pwmBR = FWVW_pwmBR + LR_pwmBR

    if (joy_msg.buttons[1]): #kill pwms with B Button
        total_pwmFL = 0
        total_pwmBL = 0
        total_pwmFR = 0 
        total_pwmBR = 0

    if (joy_msg.buttons[5]): #RB for right rotate
        total_pwmFR = -100
        total_pwmBR = -100
        total_pwmFL = 100
        total_pwmBL = 100

    if (joy_msg.buttons[4]): #LB for left rotate
        total_pwmFR = 100
        total_pwmBR = 100
        total_pwmFL = -100 
        total_pwmBL = -100

    print("FL ", total_pwmFL)
    print("BL ", total_pwmBL)
    print("FR ", total_pwmFR)
    print("BR ", total_pwmBR)

    int16pwm.data = [total_pwmFL, total_pwmFR, total_pwmBR, total_pwmBL]
    b = bytearray()
    b.append(start_byte)
    ser.write(b)

    for i in range(4): # 4 pwms in custom message
        pwm_bytes = struct.pack('h', int16pwm.data[i])

        ser.write(pwm_bytes)

    b=bytearray()
    b.append(end_byte)
    ser.write(b)

rospy.init_node("ControlNode")
IMU_topic = rospy.get_namespace() + 'imu'
Enc_topic = rospy.get_namespace() + 'encoders'
PWM_topic = rospy.get_namespace() + 'pwm'
joy_topic = 'joy'
IMU_raw_pub = rospy.Publisher(IMU_topic, Imu, queue_size=10)
encoder_pub = rospy.Publisher(Enc_topic, Float32MultiArray, queue_size = 10)
# pwm_sub = rospy.Subscriber(PWM_topic, Int16MultiArray, writePWM)
joy_sub = rospy.Subscriber(joy_topic, Joy, writeJoyPWM)
imu_msg = Imu()
encoder_msg = Float32MultiArray()

#Get the COM port of the master arduino
com_port = rospy.get_param('/com_ports/arduino_1')

ser = serial.Serial(com_port,115200)


while not rospy.is_shutdown():
    if(ser.in_waiting):
        SYNC = hex(ord(ser.read()))
        if (SYNC == '0xda'):
            motor1_freq = ser.read(4)
            motor1_freq = struct.unpack('<f', motor1_freq)[0]
            motor2_freq = ser.read(4)
            motor2_freq = struct.unpack('<f', motor2_freq)[0]
            accel_x = ser.read(4)
            accel_x = struct.unpack('<f', accel_x)[0]
            accel_y = ser.read(4)
            accel_y = struct.unpack('<f', accel_y)[0]
            accel_z = ser.read(4)
            accel_z = struct.unpack('<f', accel_z)[0]
            heading = ser.read(4)
            heading = struct.unpack('<f', heading)[0]
            gyro_x = ser.read(4)
            gyro_x = struct.unpack('<f', gyro_x)[0]
            gyro_y = ser.read(4)
            gyro_y = struct.unpack('<f', gyro_y)[0]
            gyro_z = ser.read(4)
            gyro_z = struct.unpack('<f', gyro_z)[0]

            STOP = hex(ord(ser.read(1)))
            if (STOP == "0xad"):
                print("accel X: ",accel_x)
                print("accel Y: ",accel_y)
                print("accel Z: ",accel_z)
                print("gyro X: ",gyro_x)
                print("gyro Y: ",gyro_y)
                print("gyro Z: ",gyro_z)
                print("heading: ",heading)
                print("---------------------- \n")

                #encoder_msg.header.stamp = rospy.Time.now()
                encoder_msg.data = [-1*motor1_freq,motor2_freq,-1*motor1_freq,motor2_freq]
                #imu_msg.header.stamp = rospy.Time.now()
                imu_msg.linear_acceleration.x = accel_x # accel
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z

                imu_msg.angular_velocity.x = gyro_x # gyro
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z

                #b = bytearray(struct.pack('f',gyro_z))
                #print(["0x%02x" % p for p in b])
                yaw = (heading - 180) * (np.pi / 180.0) #For some reason 0 degrees on IMU points to West 270

                quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]

                encoder_pub.publish(encoder_msg)
                IMU_raw_pub.publish(imu_msg)

    time.sleep(0.01)

ser.close()
    #check in_waiting
    #check sync start byte
    #read data (floats 4 bytes)
    #check stop byte for validation
