#!/usr/bin/env python
'''
Author: David Pierce Walker-Howeell<piercedhowell@gmail.com>
Date Created: 05/20/2020
Description: This python module controls the movement of the robot using
              pid controllers.
'''
import rospy
import rosparam
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int16MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
import time
from pid_controller import PID_Controller
from smile_mobile_robot.srv import pid_gains, pid_gainsResponse
from std_srvs.srv import SetBool, SetBoolResponse

class Movement_Controller:
    """
    Control the robot using PID controllers for linear velocity and
    direction orientation.
    """

    def __init__(self, node_name="movement_controller"):
        '''
        Initialize the movement controller as a ros node.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        self.node_name = node_name
        rospy.init_node(node_name)

        #TOPICS - This format of name space is given for ability to simulate multiple
        #by providing different names

        measured_odom_topic = rospy.get_namespace() + "raw/odometry"
        pwm_topic = rospy.get_namespace() + "pwm"
        desired_movement_topic = rospy.get_namespace() + "desired/movement"

        #Initialize subscriber for raw odometry
        rospy.Subscriber(measured_odom_topic, Odometry, self._odom_data_callback)

        #Initialize subscriber for desired_odometry
        rospy.Subscriber(desired_movement_topic, Float32MultiArray, self._desired_movement_callback)

        #Initialize PWM controllers
        self._initialize_pid_controlers()

        self.pwm_pub = rospy.Publisher(pwm_topic, Int16MultiArray, queue_size=10)
        self.pwm_msg = Int16MultiArray()

        self.pid_timer = rospy.Rate(100) #100Hz

        #Service to enable/disable the movement controller...only sends 0 pwm value.
        self.movement_controller_enabled = True
        rospy.Service('movement_controller/enabled', SetBool, self._movement_controller_state_callback)

        #Get the maximum allowable PWM to send to the motors
        self.max_pwm = rospy.get_param('/max_pwm')

        #Initialize publisher for writing PWM
        self.measured_velocity = 0.0
        self.desired_velocity= 0.0
        self.measured_orientation = 0.0
        self.desired_orientation = 0.0
        self.velocity_control = 0.0
        self.steering_control = 0.0

    def _initialize_pid_controlers(self):
        '''
        Initialize communication with the pid controllers.

        Parameters:
            N/A
        Returns:
            N/A
        '''

        pid_params = rospy.get_param("/pid")
        velocity_pid_params = pid_params['velocity']
        steering_pid_params = pid_params['steering']

        velocity_k_p = velocity_pid_params['k_p']
        velocity_k_i = velocity_pid_params['k_i']
        velocity_k_d = velocity_pid_params['k_d']

        self.velocity_pid_controller = PID_Controller(k_p=velocity_k_p,
                                                      k_i=velocity_k_i,
                                                      k_d=velocity_k_d,
                                                      integral_min=-10,
                                                      integral_max=10,
                                                      max_control_effort=255,
                                                      min_control_effort=-255)

        steering_k_p = steering_pid_params['k_p']
        steering_k_i = steering_pid_params['k_i']
        steering_k_d = steering_pid_params['k_d']
        self.steering_pid_controller = PID_Controller(k_p=steering_k_p,
                                                      k_i=steering_k_i,
                                                      k_d=steering_k_d,
                                                      max_control_effort=80,
                                                      min_control_effort=-80,
                                                      integral_min=-1,
                                                      integral_max=1,
                                                      angle_error=True)

        #Initialize the service for updating the pid controller gains
        rospy.Service('update_pid_gains', pid_gains, self.handle_pid_gains_update)

        return

    def handle_pid_gains_update(self, req):
        '''
        Handler function for when a PID gain upate is requested. This will update
        the PIDS in the parameter server.

        Parameters:
            reg: The request message containing the desired gains
        Returns:
            N/A
        '''
        self.velocity_pid_controller.set_gains(req.velocity_k_p, req.velocity_k_i, req.velocity_k_d)
        self.steering_pid_controller.set_gains(req.steering_k_p, req.steering_k_i, req.steering_k_d)

        #Update the parameter server on the correct pid values
        rospy.set_param("/pid/velocity", {'k_p': req.velocity_k_p, 'k_i': req.velocity_k_i, 'k_d': req.velocity_k_d})
        rospy.set_param("/pid/steering", {'k_p': req.steering_k_p, 'k_i': req.steering_k_i, 'k_d': req.steering_k_d})


        return pid_gainsResponse()

    def _movement_controller_state_callback(self, req):
        '''
        Callback function for the movement controller enable/disable service. If true,
        the movement controller will make corrections to meet the desired speed and
        steering direction with PID controller. Else if disabled, the controller will
        simply send PWM values of [0, 0, 0, 0] to the motor driver.

        Parameters:
            req: The request message (bool) to enable/disable the motor. Type std_srvs/SetBool.
        Returns:
            response: A response with a string & bool notifiying if the request was valid. Type std_srvs/SetBoolResponse
        '''
        self.movement_controller_enabled = req.data
        response = SetBoolResponse()
        response.success = True

        if(self.movement_controller_enabled):
            response.message = "Movement Controller Enabled"
        else:
            response.message = "Movement Controller Disabled"
            pwm_msg = Int16MultiArray(); pwm_msg.data = [0, 0, 0, 0]
            self.pwm_pub.publish(pwm_msg)

        return  response


    def _odom_data_callback(self, odom_msg):
        '''
        Callback function for the measured odometry data estimated.
        Unpack the data

        Parameters:
            odom_msg: Odometry data message type nav_msgs/Odometry
        Returns:
            N/A
        '''
        self.measured_odom = odom_msg
        self.measured_velocity= self.measured_odom.twist.twist.linear.x

        #Orientation is the direction the robot faces
        #Convert from quaternion to euler
        [roll, pitch, yaw] =  [self.measured_odom.pose.pose.orientation.x,
                      self.measured_odom.pose.pose.orientation.y,
                      self.measured_odom.pose.pose.orientation.z]
        
        self.measured_orientation = yaw


    def _desired_movement_callback(self, desired_movement_msg):
        '''
        Callback function for the desired movement for the robot to hold. The
        message contains the desired velocity and orientation.

        Parameters:
            desired_movement_msg: The desired velocity (m/s) and orientation (rad/s).
                                  The message is of type std_msgs/Float32MultiArray.
                                  [desired_velocity, desired_orientation]
        Returns:
            N/A
        '''
        self.desired_velocity = desired_movement_msg.data[0]

        #Orientation is the direction the robot faces
        self.desired_orientation = desired_movement_msg.data[1]

    def map_control_efforts_to_pwms(self, vel_control_effort, steering_control_effort):
        '''
        Maps the control efforts for velocity and steering to individual motor
        PWMS.

        Parameters:
            vel_control_effort: Velocity Control effort output from PID.
            steering_control_effort: sterring control effor output from PID
        Returns:
            pwms: [pwm_1, pwm_2, pwm_3, pwm_4]
        '''
        vel_control = vel_control_effort
        steering_control = steering_control_effort

        pwm_1 = vel_control - steering_control
        pwm_2 = vel_control + steering_control
        pwm_3 = vel_control + steering_control
        pwm_4 = vel_control - steering_control

        pwms = [pwm_1, pwm_2, pwm_3, pwm_4]
        for i in range(4):
            if(pwms[i] < -self.max_pwm):
                pwms[i] = -self.max_pwm
            elif(pwms[i] > self.max_pwm):
                pwms[i] = self.max_pwm
        return pwms

    def run(self):
        '''
        Run the main loop of the movement controller. Receive desired and
        measured odometry data and control.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        #Publish errror for debug
        error_pub = rospy.Publisher('pid_errors', Float32MultiArray, queue_size=1)
        error_msg = Float32MultiArray()

        try:
            while not rospy.is_shutdown():

                if self.movement_controller_enabled:
                    #Take the control effort output from PID controller and map to
                    #PID's of individual motor
                    self.velocity_control, velocity_error = self.velocity_pid_controller.update(self.desired_velocity, self.measured_velocity)
                    self.steering_control, steering_error = self.steering_pid_controller.update(self.desired_orientation, self.measured_orientation)
                    error_msg.data = [velocity_error, steering_error]
                    error_pub.publish(error_msg)

                    motor_pwms = self.map_control_efforts_to_pwms(self.velocity_control, -1*self.steering_control)


                    self.pwm_msg.data = motor_pwms#Send PWM command    
                    self.pwm_pub.publish(self.pwm_msg)
                else:
                    self.pwm_msg.data = [0, 0, 0, 0] #Movement Controller disabled state

                

                #100Hz
                self.pid_timer.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    movement_controller = Movement_Controller()
    movement_controller.run()
