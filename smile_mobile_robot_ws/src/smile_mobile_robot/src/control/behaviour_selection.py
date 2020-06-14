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
import numpy as np
from Queue import Queue
import actionlib
from smile_mobile_robot.msg import motionAction, motionActionGoal, motionActionFeedback, motionActionResult

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
        self.mode = 1

        #Service for a request to change modes.
        rospy.Service('mode_select', mode_select, self._mode_select_server)

        #TOPICS
        lane_detection_topic = rospy.get_namespace() + 'lane_detection'

        #Subscribe to the lane detector. Have a flag for if a lane is detected.
        self.valid_lane_detected = False
        self.center_lane_poly_coeffs = [0.0, 0.0, 0.0] #The coefficients describing the center lane polynomial
        rospy.Subscriber(lane_detection_topic, Float32MultiArray, self._lane_detection_subscriber_callback)
        self.num_trajectory_points = rospy.get_param('/vision_params/lane_detector/num_trajectory_points')

        self.relative_pos_queue = Queue(1)

        #Motion action msg. Send desired relative motion to the motion controller.
        self.motion_action_goal = motionActionGoal()
        self.action_name = rospy.get_namespace() + 'motion_action'
        self.motion_action_client = actionlib.SimpleActionClient(self.action_name, motionAction)

        #Wait for the action server to get started
        self.motion_action_client.wait_for_server()

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

    def _lane_detection_subscriber_callback(self, lane_detection_msg):
        '''
        Callback function for the lane detection message from the lane
        detector.

        Parameters:
            lane_detection_msg: A lane detection message of type Float32MultiArray
        Returns:
            N/A
        '''
        #Read the lane detection message
        center_lane_points_flattened = np.array(lane_detection_msg.data)
        center_lane_points = center_lane_points_flattened.reshape(self.num_trajectory_points, 2)

        motion_goal = motionActionGoal()
        motion_goal.goal.x = center_lane_points[1, 0]
        motion_goal.goal.y = -1*center_lane_points[1, 1]
        motion_goal.goal.velocity = 1.0

        #Place as an action goal
        self.relative_pos_queue.put(motion_goal.goal)


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

                if(True):
                    if( not self.relative_pos_queue.empty()):
                        motion_goal = self.relative_pos_queue.get()
                        print(motion_goal)
                        self.motion_action_client.send_goal(motion_goal)

                        motion_action_result = self.motion_action_client.get_result()
                        print(motion_action_result)
                else:
                    self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("[WARNING]: Behaviour Selector Interurupted.Shutting Down")

if __name__ == "__main__":
    behaviour_selector = Behaviour_Selector()
    behaviour_selector.run()
