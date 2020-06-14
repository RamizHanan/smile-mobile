#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 06/11/2020
Description: Takes waypoints and instructs the motion of the vehicle
            to follow those waypoints. These are for close range waypoints
            where the robot will drive as the crow flies between each. For smooth
            movement it is suggested that desired waypoints are grouped closely to
            make more complex motions.
'''

import rospy
import actionlib
from smile_mobile_robot.msg import motionAction, motionFeedback, motionResult
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
from tf.transformations import euler_from_quaternion

class Motion_Planner:
    '''
    Motion planning given waypoints.
    '''

    def __init__(self):
        '''
        Initialize the motion planner node.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        rospy.init_node('motion_planner')
        #Action to receive the desired waypoint
        self.action_name = rospy.get_namespace() + 'motion_action'

        #Receive the action to move do a desired point
        self.motion_action_server = actionlib.SimpleActionServer(self.action_name,
                                motionAction, execute_cb=self._motion_action_callback,
                                auto_start=False)
        self.motion_action_server.start()
        self.update_rate = rospy.Rate(10)


        #Subscriber to the odometry of the vehicle
        #TOPIC
        odom_topic = rospy.get_namespace() + 'raw/odometry'
        rospy.Subscriber(odom_topic, Odometry, self._odom_data_callback)
        #Initialize odometry variables
        self.velocity=0.0; self.yaw=0.0; self.X=0.0; self.Y=0.0

        #TOPIC
        desired_movement_topic = rospy.get_namespace() + 'desired/movement'

        #Publisher to the movement controller to drive the robot between waypoints
        #Send angle and velocity commands
        self.desired_movement_pub = rospy.Publisher(desired_movement_topic, Float32MultiArray, queue_size=10)
        self.desired_movement_msg = Float32MultiArray()

        #Tunable variables that should be in parameter server
        param_path = '/motion_planner_params/'
        self.max_velocity = rospy.get_param(param_path + 'max_velocity')
        self.success_pos_tolerance = rospy.get_param(param_path + 'success_pos_tolerance')
        self.success_pos_timeout = rospy.get_param(param_path + 'success_pos_timeout')
        self.relative_angle_limit = rospy.get_param(param_path + 'relative_angle_limit')

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

    def _motion_action_callback(self, goal):
        '''
        Callback function for receiving a motion action. Begin execution if motion
        action. Feedback will be sent to back

        Parameters:
            motion_action_goal: The motion action goal to attempt.
        Returns:
            N/A
        '''
        rospy.loginfo("Received Waypoint (X:%0.2f, Y:%0.2f, vel:%0.2f)" % (goal.x, goal.y, goal.velocity))

        dist = 10000.0
        timer = 0.0
        timer_start_time = (rospy.Time.now()).secs

        #Since this module receives the relative x and y to go to, translate those into the
        #global X and Y.
        goal_X = self.X + (goal.x * math.cos(self.yaw) - goal.y*math.sin(self.yaw))
        goal_Y = self.Y + (goal.x * math.sin(self.yaw) + goal.y* math.cos(self.yaw))
        print("(%0.2f, %0.2f)", goal_X, goal_Y)

        while(True):
            timer = (rospy.Time.now()).secs - timer_start_time

            if(timer > self.success_pos_timeout):
                rospy.loginfo("FAIL: DIDN'T REACH WAYPOINT. TIMEOUT")
                success = False
                break

            #Get the distance between the two points (as the crow flies)
            dist = math.sqrt((self.X - goal_X)**2 + (self.Y - goal_Y)**2)

            #TODO: Add a catch here if the distance is too far for the short
            #movement.

            if(abs(dist) < self.success_pos_tolerance):
                rospy.loginfo("SUCCESS: REACHED WAYPOINT")
                success = True
                break

            #A unit vector with angle of yaw
            robot_direction_vect = np.array([math.cos(self.yaw), math.sin(self.yaw)])

            #Get the vector representing the
            dest_direction_vect = np.array([(goal_X - self.X), (goal_Y - self.Y)])
            dot_prod = np.dot(robot_direction_vect, dest_direction_vect)
            norm_mult = np.linalg.norm(robot_direction_vect) * np.linalg.norm(dest_direction_vect)

            #TODO: Catch if the angle is to large
            relative_angle = math.acos(dot_prod / norm_mult)
            if(goal.y < 0):
                relative_angle *= -1

            if(abs(relative_angle) > self.relative_angle_limit):
                rospy.loginfo("FAIL: RELATIVE ANGLE ADJUSTMENT BEYONE SET LIMIT")
                success = False
                break

            desired_angle = self._get_yaw_angle(relative_angle, self.yaw)

            #Publish the velocity and angle to the movement controller
            self.desired_movement_msg.data = [goal.velocity, desired_angle]
            self.desired_movement_pub.publish(self.desired_movement_msg)

            #Publish the feedback
            feedback = motionFeedback()
            feedback.dist = dist
            feedback.time = timer
            self.motion_action_server.publish_feedback(feedback)

            #TODO: Add a catch here if the relative anlge is not within limit
            #if(abs(relative_angle) > self.relative_angle_limit):
            self.update_rate.sleep()


        result = motionResult()
        result.success = success

        #TODO: ADD IN TIMER
        result.time = timer
        self.motion_action_server.set_succeeded(result)

    def _get_velocity_at_time_step(self, time_step, rho, scale, offset):
        '''
        Adjusts from the current velocity to the desired velocity
        '''

        return (scale / (1+np.exp(-rho*time_step)))*scale + offset
    def _get_yaw_angle(self, relative_angle_adjustment, curr_yaw):
        '''
        Get the yaw angle need between -pi and pi with the addition of
        the angle adjustment to the current read yaw of the vehicle

        Parameters:
            relative_angle_adjustment: The angle increment to change the yaw to.
            curr_yaw: The current yaw of the vehicle
        Returns:
            desired_angle: The new yaw with the added angle adjustment
        '''
        #Set the yaw angle for the robot to point at the desired waypoint
        adjustment_angle = curr_yaw + relative_angle_adjustment
        if(adjustment_angle < -1*math.pi):
            desired_angle = 2.0*math.pi + adjustment_angle

        elif(adjustment_angle > math.pi):
            desired_angle = adjustment_angle - 2.0*math.pi

        else:
            desired_angle = adjustment_angle
        return desired_angle

if __name__ == "__main__":
    motion_planner = Motion_Planner()
    rospy.spin()
