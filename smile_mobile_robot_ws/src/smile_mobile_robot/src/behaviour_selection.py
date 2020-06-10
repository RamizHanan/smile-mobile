#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 06/08/2020
Descripion: Main decision module for controlling the vehcile to reach goals given
            environment perception.
'''
import rospy

class Behaviour_Selector:
    '''
    Finite state machine for choosing the behaviour of the vehicle
    '''
    def __init__(self):
        '''
        Initialize the behaviour selector.
        '''
        #TOPICS
        lane_detection_topic = rospy.get_namespace() + 'lane_detection'

        #Subscribe to the lane detector. Have a flag for if a lane is detected.
        self.valid_lane_detected = False
        self.center_lane_poly_coeffs = [0.0, 0.0, 0.0] #The coefficients describing the center lane polynomial
        rospy.Subscriber(lane_detection_topic, Float32MultiArray, self._lane_detection_subscriber_callback)

    def get_lane_path(self, lane_detection_params):
        '''
        Get the lane path (positions and orientations) from the lane detection
        results.

        Parameters:
            lane_detection_params: The parameters received from the lane detector.
                    About the trajector of the lane.
        Return
            A tuple of the (x, y, yaw) Path to drive on the lane.
        '''

    def _lane_detection_subscriber_callback(self, lane_detection_msg):
        '''
        Callback function for the lane detection message from the lane
        detector.

        Parameters:
            lane_detection_msg: A lane detection message of type Float32MultiArray
        Returns:
            N/A
        '''

        if(lane_detection_msg.data[0] == 1):
            self.valid_lane_detected = True
        else:
            self.valid_lane_detected = False

        #The polynomial coefficents [ax^2, bx, c] describing center of the lane
        self.center_lane_poly_coeffs = lane_detection_msg.data[1:4]

        
