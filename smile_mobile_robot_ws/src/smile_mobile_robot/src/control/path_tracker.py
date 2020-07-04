#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 07/03/2020
Description: A queue system with a timer for managing and executing tracking the tracjectory.
'''
from trajectory_planner import Trajectory
import rospy
import actionlib
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32
import math
import numpy as np
from tf.transformations import euler_from_quaternion
import time
import matplotlib.pyplot as plt
from pid_controller import PID_Controller



class Cross_Track_PID():
    '''
    '''
    def __init__(self):
        '''
        '''
        rospy.init_node("trajectory_controller")
        odom_topic = rospy.get_namespace() + 'raw/odometry'
        desired_movement_topic = rospy.get_namespace() + 'desired/movement'

        rospy.Subscriber(odom_topic, Odometry, self._odom_data_callback)
        self.velocity = 0.0
        self.yaw = 0.0
        self.X = 0.0
        self.Y = 0.0

        #Publisher to the movement controller to drive the robot between waypoints
        #Send angle and velocity commands
        self.desired_movement_pub = rospy.Publisher(desired_movement_topic, Float32MultiArray, queue_size=10)
        self.desired_movement_msg = Float32MultiArray()

        #FOR DEBUGGING PURPOSES
        cross_track_error_topic = rospy.get_namespace() + 'cross_track_error'
        self.cross_track_error_msg = Float32()
        self.cross_track_error_pub = rospy.Publisher(cross_track_error_topic, Float32, queue_size=1)

        #Initialize the PID controller for the change of angle
        self.delta_angle_controller = PID_Controller(k_p=0.3,
                                                   k_i=0.0,
                                                   k_d=0.0,
                                                   max_control_effort=1.0,
                                                   min_control_effort=-1.0,
                                                   integral_min=-10,
                                                   integral_max=10)

        #FOR TESTING PURPOSES, GENERATE A RANDOM TRAJECTORY.
        x_ref = np.arange(0, 10, 1/2.0)
        y_ref = x_ref

        #Testing trajectory. (NOT FINAL FORM AT ALL)
        self.trajectory = Trajectory(x_ref, y_ref, self.X, self.Y, self.yaw, 0.0, 1.0, 0.0, 0.0)
        self.trajectory_index = 0
        self.max_trajectory_index = len(self.trajectory.x_path)
        print(self.max_trajectory_index)
        self.rate = rospy.Rate(10)

    def _get_closest_point(self):
        '''
        Get's the closest point between the robots current position and that of the trajectory.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        closest_point_calc_region_tol = 50
        lower_index = 0
        upper_index = self.max_trajectory_index
        if(self.trajectory_index >= closest_point_calc_region_tol):
            lower_index = self.trajectory_index - closest_point_calc_region_tol

        if(self.trajectory_index <= self.max_trajectory_index - closest_point_calc_region_tol):
            upper_index = self.trajectory_index + closest_point_calc_region_tol

        shortest_distance = 1e6
        x = self.X
        y = self.Y

        #Find the closest points within the index range
        for index in range(lower_index, upper_index):

            distance = np.sqrt((x - self.trajectory.x_path[index])**2 + (y - self.trajectory.y_path[index])**2)
            if(distance < shortest_distance):
                shortest_distance = distance
                self.trajectory_index = index #update the index

        #Determine if it is to the left or right of the vehicle
        dir_angle = np.arctan2((y - self.trajectory.y_path[self.trajectory_index]), (x - self.trajectory.x_path[self.trajectory_index]))

        if(dir_angle < 0):
            shortest_distance *= -1

        return shortest_distance, self.trajectory_index

    def _odom_data_callback(self, odom_msg):
        '''
        Callback function for the measured odometry data estimated.
        Unpack the data

        Parameters:
            odom_msg: Odometry data message type nav_msgs/Odometry
        Returns:
            N/A
        '''
        self.odom = odom_msg
        self.velocity= self.odom.twist.twist.linear.x

        #Orientation is the direction the robot faces
        #Convert from quaternion to euler
        quaternion = [self.odom.pose.pose.orientation.x,
                      self.odom.pose.pose.orientation.y,
                      self.odom.pose.pose.orientation.z,
                      self.odom.pose.pose.orientation.w]
        [roll, pitch, yaw] = euler_from_quaternion(quaternion)

        self.yaw = yaw
        self.X = self.odom.pose.pose.position.x
        self.Y = self.odom.pose.pose.position.y


    def run(self):
        '''
        '''
        try:
            while(not rospy.is_shutdown()):

                #Get the cross track error
                cross_track_error, index = self._get_closest_point()

                if(index == self.max_trajectory_index - 1):
                    self.desired_movement_msg.data = [0, 0]
                    self.desired_movement_pub.publish(self.desired_movement_msg)
                    print("Done")
                    break
                delta_angle, error = self.delta_angle_controller.update(0, cross_track_error)

                #Get the desired steering angle
                adjustment_angle = self.yaw + delta_angle
                if(adjustment_angle < -1*math.pi):
                    desired_orientation = 2.0*math.pi + adjustment_angle

                elif(adjustment_angle > math.pi):
                    desired_orientation = adjustment_angle - 2.0*math.pi

                else:
                    desired_orientation = adjustment_angle

                self.desired_movement_msg.data = [0.5, desired_orientation]
                self.desired_movement_pub.publish(self.desired_movement_msg)

                #FOR DEBUGGING PURPOSES
                self.cross_track_error_msg.data = cross_track_error
                self.cross_track_error_pub.publish(self.cross_track_error_msg)


                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Trajectory Controller Interurupted. Shutting Down")

if __name__ == "__main__":
    cross_track_pid = Cross_Track_PID()
    cross_track_pid.run()
