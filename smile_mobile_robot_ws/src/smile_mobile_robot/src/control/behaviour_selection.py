#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created: 06/08/2020
Descripion: Main decision module for controlling the vehcile to reach goals given
            environment perception.
'''
import rospy
from smile_mobile_robot.srv import mode_select, mode_selectResponse
from std_msgs.msg import Float32MultiArray

class Behaviour_Selector:
    '''
    Finite state machine for choosing the behaviour of the vehicle
    '''
    def __init__(self):
        '''
        Initialize the behaviour selector.
        '''
        rospy.init_node('behaviour_selector')

        #MODE OF OPERATION
        #There are multiple modes of operation of the vehicle based on the environment
        #and user preference.
        #Mode Types:
        #   disabled/killed (0): The robot is inactive and cannot drive.
        #   paved_navigation (1): Follow the roads and primarily use lane detection.
        #   unpaved_navigation (2): Use reinforcement learning the navigate unpaced environment
        #   manual 1 (3): Manual control of the vehicle through raw PWM values.
        #   manual 2 (4): Manual control with movement controller PIDs.
        self.mode = 0

        #Service for a request to change modes.
        rospy.Service('mode_select', mode_select, self._mode_select_server)

        #TOPICS
        lane_detection_topic = rospy.get_namespace() + 'lane_detection'

        #Subscribe to the lane detector. Have a flag for if a lane is detected.
        self.valid_lane_detected = False
        self.center_lane_poly_coeffs = [0.0, 0.0, 0.0] #The coefficients describing the center lane polynomial
        rospy.Subscriber(lane_detection_topic, Float32MultiArray, self._lane_detection_subscriber_callback)

        #Behaviour_Selector looping frequnecy
        self.rate = rospy.Rate(100)


    def _mode_select_server(self, req):
        '''
        The mode selection service that can be called to select the vehicles
        mode of operation.

        Parameters:
            req: The request message of type mode_select.srv

        Returns:
            mode_selectResponse: A response noting if the mode selection was
                            successful or not.
        '''
        return_msg = mode_selectResponse()
        if(req.mode == 0):
            rospy.loginfo("MODE SELECTED: KILLED")
            self.mode = 0
            return_msg.success = True; return_msg.message = "KILLED"
            return return_msg
        elif(req.mode == 1):
            rospy.loginfo("MODE SELECTED: PAVED_NAV")
            self.mode = 1
            return_msg.success = True; return_msg.message = "PAVED_NAV"
            return return_msg
        elif(req.mode == 2):
            rospy.loginfo("MODE SELECTED: UNPAVED_NAV")
            self.mode = 2
            return_msg.success = True; return_msg.message = "UNPAVED_NAV"
            return return_msg
        elif(req.mode == 3):
            rospy.loginfo("MODE SELECTED: MANUAL_1")
            self.mode = 3
            return_msg.success = True; return_msg.message = "MANUAL_1"
            return return_msg
        elif(req.mode == 4):
            rospy.loginfo("MODE SELECTED: MANUAL_2")
            self.mode = 4
            return_msg.success = True; return_msg.message = "MANUAL_2"
            return return_msg
        else:
            rospy.loginfo("[WARN]: MODE SELECTION FAILED. DEFAULT KILLED")
            rospy.logwarn("[WARN]: MODE SELECTION FAILED. DEFAULT KILLED")
            self.mode = 0
            return_msg.success = False; return_msg.message = "FAILED"
            return return_msg


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

        
    def run(self):
        '''
        Main loop to run the behaviour selector module

        Parameters:
            N/A
        Returns:
            N/A
        '''
        try:
            while not rospy.is_shutdown():

                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("[WARNING]: Behaviour Selector Interurupted.Shutting Down")

if __name__ == "__main__":
    behaviour_selector = Behaviour_Selector()
    behaviour_selector.run()
