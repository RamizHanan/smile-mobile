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
from Queue import Queue



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
        self.delta_angle_controller = PID_Controller(k_p=0.50,
                                                   k_i=0.00,
                                                   k_d=0.00,
                                                   max_control_effort=0.10,
                                                   min_control_effort=-0.10,
                                                   integral_min=-10,
                                                   integral_max=10)

        #Trajectory Queue for queueing up paths to follow.
        self.trajectory_queue = Queue(maxsize=20)

        #FOR TESTING PURPOSES, GENERATE A RANDOM TRAJECTORY.
        x_ref = np.arange(0, 10, 1/2.0)
        y_ref = np.sqrt(x_ref)

        #Testing trajectory. (NOT FINAL FORM AT ALL)
        trajectory = Trajectory(x_ref, y_ref, self.X, self.Y, self.yaw, 0.0, 1.0, 0.0, 0.0)
        self.trajectory_queue.put(trajectory)

        self.rate = rospy.Rate(100)

    def _get_closest_point(self, trajectory, curr_index):
        '''
        Get's the closest point between the robots current position and that of the trajectory.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        max_trajectory_index = len(trajectory.x_path)

        closest_point_calc_region_tol = 20
        lower_index = 0
        upper_index = max_trajectory_index

        if(curr_index >= closest_point_calc_region_tol):
            lower_index = curr_index - closest_point_calc_region_tol

        if(curr_index <= max_trajectory_index - closest_point_calc_region_tol):
            upper_index = curr_index + closest_point_calc_region_tol

        shortest_distance = 1e6
        x = self.X
        y = self.Y

        #Find the closest points within the index range
        for index in range(lower_index, upper_index):

            distance = np.sqrt((x - trajectory.x_path[index])**2 + (y - trajectory.y_path[index])**2)
            if(distance < shortest_distance):
                shortest_distance = distance
                curr_index = index #update the index

        #Determine if it is to the left or right of the vehicle
        dir_angle = np.arctan2((y - trajectory.y_path[curr_index]), (x - trajectory.x_path[curr_index]))

        if(dir_angle < 0):
            shortest_distance *= -1

        return shortest_distance, curr_index

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
        executing_trajectory = False
        current_trajectory = None
        trajectory_index = 0
        max_trajectory_index = 0

        #FOR TESTING PURPOSES
        mean_square_error = 0.0
        try:
            while(not rospy.is_shutdown()):

                #Executing a trajectory
                if(executing_trajectory):

                    #Get the cross track error
                    cross_track_error, trajectory_index = self._get_closest_point(current_trajectory, trajectory_index)

                    #When the maximum index is reached (end of path)
                    if(trajectory_index == max_trajectory_index - 1):
                        self.desired_movement_msg.data = [0, 0]
                        self.desired_movement_pub.publish(self.desired_movement_msg)
                        executing_trajectory = False
                        print("Done")
                        continue

                    delta_angle, error = self.delta_angle_controller.update(0, cross_track_error)
                    #delta_angle = np.arctan2((self.Y - current_trajectory.y_path[trajectory_index]), (self.X - current_trajectory.x_path[trajectory_index]))

                    #Get the desired steering angle, limit the steering angle between -pi, pi
                    adjustment_angle = self.yaw + delta_angle
                    if(adjustment_angle < -1*math.pi):
                        desired_orientation = 2.0*math.pi + adjustment_angle

                    elif(adjustment_angle > math.pi):
                        desired_orientation = adjustment_angle - 2.0*math.pi

                    else:
                        desired_orientation = adjustment_angle


                    #Send the desired movememnt to the movement controller (velocity, angle)
                    self.desired_movement_msg.data = [0.25, desired_orientation]
                    self.desired_movement_pub.publish(self.desired_movement_msg)

                    #FOR DEBUGGING PURPOSES
                    #self.cross_track_error_msg.data = cross_track_error
                    #self.cross_track_error_pub.publish(self.cross_track_error_msg)
                    #mean_square_error = (mean_square_error + cross_track_error**2) / 2.0
                    #print(cross_track_error)
                    trajectory_index += 1
                else:

                    #Check if the the queue is empty
                    if(self.trajectory_queue.empty()):
                        rospy.loginfo("Trajectory Queue Empty!")
                        self.desired_movement_msg.data = [0, 0]
                        self.desired_movement_pub.publish(self.desired_movement_msg)
                    else:
                        current_trajectory = self.trajectory_queue.get()
                        max_trajectory_index = len(current_trajectory.x_path)
                        executing_trajectory = True
                        trajectory_index = 0


                self.rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo("Trajectory Controller Interurupted. Shutting Down")


if __name__ == "__main__":
    cross_track_pid = Cross_Track_PID()
    cross_track_pid.run()
