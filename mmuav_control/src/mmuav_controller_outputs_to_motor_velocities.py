#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped, PoseStamped, Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray
import math
from math import sin, cos
from mav_msgs.msg import Actuators
from datetime import datetime
from rosgraph_msgs.msg import Clock
import time
import DualManipulatorPassiveJointKinematics as ArmsKinematics
import copy
from control_msgs.msg import JointControllerState

class MergeControllerOutputs:

    def __init__(self):
        self.rate = rospy.get_param('rate', 50)
        self.ros_rate = rospy.Rate(self.rate)

        self.roll_command = 0.0
        self.pitch_command = 0.0
        self.yaw_command = 0.0
        self.vpc_roll_command = 0.0
        self.vpc_pitch_command = 0.0

        self.mot_vel_ref = 0.0

        self.attitude_command_received_flag = False
        self.mot_vel_ref_received_flag = False
        self.q1_left = 2.493
        self.q2_left = -2.56
        self.q3_left = -1.08
        self.q1_right = -0.645
        self.q2_right = -2.567
        self.q3_right = -1.08

        self.q1_left_pub = rospy.Publisher('joint1_left_controller/command', Float64, queue_size=1)
        self.q2_left_pub = rospy.Publisher('joint2_left_controller/command', Float64, queue_size=1)
        self.q3_left_pub = rospy.Publisher('joint3_left_controller/command', Float64, queue_size=1)
        self.q1_right_pub = rospy.Publisher('joint1_right_controller/command', Float64, queue_size=1)
        self.q2_right_pub = rospy.Publisher('joint2_right_controller/command', Float64, queue_size=1)
        self.q3_right_pub = rospy.Publisher('joint3_right_controller/command', Float64, queue_size=1)

        rospy.Subscriber('payload_position', Point, self.payload_pos_cb, queue_size=1)
        rospy.Subscriber('joint1_left_controller/state', JointControllerState, self.joint1_left_cb)
        rospy.Subscriber('joint2_left_controller/state', JointControllerState, self.joint2_left_cb)
        rospy.Subscriber('joint3_left_controller/state', JointControllerState, self.joint3_left_cb)
        rospy.Subscriber('joint1_right_controller/state', JointControllerState, self.joint1_right_cb)
        rospy.Subscriber('joint2_right_controller/state', JointControllerState, self.joint2_right_cb)
        rospy.Subscriber('joint3_right_controller/state', JointControllerState, self.joint3_right_cb)

        self.dx = 0
        self.dy = 0

        self.x_right_offset = -0.23499987352
        self.y_right_offset = 0.000136305972549

        self.x_left_offset = -0.234999660588
        self.y_left_offset = 0.000345747003094

        self.L1 = 0.094
        self.L2 = 0.061
        self.L3 = 0.08
        
    def run(self):

         # Sleep for few seconds then publish initial arms q
        print("Setting initial pos")
        rospy.sleep(5)
        self.q1_left_pub.publish(Float64(2.493))
        self.q2_left_pub.publish(Float64(-2.56))
        self.q3_left_pub.publish(Float64(-1.08))
        self.q1_right_pub.publish(Float64(-0.645))
        self.q2_right_pub.publish(Float64(-2.567))
        self.q3_right_pub.publish(Float64(-1.08))
        rospy.sleep(5.0)

        while not rospy.is_shutdown():
            self.ros_rate.sleep()
            
            self.get_new_dqR(self.dx, self.dy, 0.15)
            self.get_new_dqL(self.dx, self.dy, 0.15)

    def payload_pos_cb(self, msg):
        self.dx = msg.x
        self.dy = msg.y

    def get_new_dqR(self, dx_pitch, dy_roll, limit):
        # Add initial offset - right manipulator
        x_right_target = self.x_right_offset - dx_pitch
        y_right_target = self.y_right_offset - dy_roll

        q1 = self.q1_right
        q2 = self.q2_right
        q3 = self.q3_right
        l1 = self.L1
        l2 = self.L2
        l3 = self.L3

        # Get current position of the right manipulator 
        x_right_curr = - l1 * cos(q1) \
                        - l2 * cos(q1 + q2) \
                        - l3 * cos(q1 + q2 + q3) 
        y_right_curr = - l1 * sin(q1) \
                        - l2 * sin(q1 + q2) \
                        - l3 * sin(q1 + q2 + q3) 

        print 'x_right_target: ', x_right_target, ' y_right_target: ', y_right_target
        print 'x_right_curr: ', x_right_curr, ' y_right_curr: ', y_right_curr
        print 'dx: ', dx_pitch, ' dy: ', dy_roll

        dx_right_pitch = x_right_target - x_right_curr
        dy_right_roll = y_right_target - y_right_curr

        # Limit dx / dy
        #dx_right_pitch = self.saturation(dx_right_pitch, limit)
        #dy_right_roll = self.saturation(dy_right_roll, limit)

        print 'dx_new: ', dx_right_pitch, ' dy_new: ', dy_right_roll

        # New angle increments (right)
        try: 
            dq1R =  ( (l3 * cos(q1 + q2 + q3) + l2 * cos(q1 + q2)) * dx_right_pitch + \
                (l3 * sin(q1 + q2 + q3) + l2 * sin(q1 + q2)) * dy_right_roll ) / \
                (l1 * l3 * sin(q2 + q3) + l1 * l2 * sin(q2)) 
            dq2R = ( (l3 * cos(q1 + q2 + q3) + l2 * cos(q1 + q2) + l1 * cos(q1)) * dx_right_pitch +  \
                (l3 * sin(q1 + q2 + q3) + l2 * sin(q1 + q2) + l1 * sin(q1)) * dy_right_roll ) /  \
                (l1 * l3 * sin(q2 + q3) + l1 * l2 * sin(q2))
        except ZeroDivisionError:
            print "ZeroDivisionError"
            dq1R = 0
            dq2R = 0
        
        print 'dq1R: ', dq1R, ' dq2R: ', dq2R
        print ''

        return (dq1R, dq2R)

    '''
    Calculate new joint increments for the left manipulator.
    @param dx_pitch 
    @param dy_roll
    @return Tuple containing (dq1L, dq2L)
    '''
    def get_new_dqL(self, dx_pitch, dy_roll, limit):
        # Add initial offset - left manipulator
        x_left_target = self.x_left_offset + dx_pitch
        y_left_target = self.y_left_offset + dy_roll

        # Get joint offsets of the left manipulator
        q1 = self.q1_left
        q2 = self.q2_left
        q3 = self.q3_left
        l1 = self.L1
        l2 = self.L2
        l3 = self.L3

        print ''
        # Get current position of the right manipulator 
        x_left_curr = - l1 * cos(q1) \
                        - l2 * cos(q1 + q2) \
                        - l3 * cos(q1 + q2 + q3) 
        y_left_curr = - l1 * sin(q1) \
                        - l2 * sin(q1 + q2) \
                        - l3 * sin(q1 + q2 + q3) 

        print 'x_left_target: ', x_left_curr, ' y_left_target: ', y_left_target
        print 'x_left_curr: ', x_left_curr, ' y_left_curr: ', y_left_curr
        print 'dx: ', dx_pitch, ' dy: ', dy_roll

        # Calculate distance to the target position
        dx_left_pitch = x_left_target - x_left_curr
        dy_left_roll = y_left_target - y_left_curr

        # Limit dx / dy
        #dx_left_pitch = self.saturation(dx_left_pitch, limit)
        #dy_left_roll = self.saturation(dy_left_roll, limit)

        print 'dx_new: ', dx_left_pitch, ' dy_new: ', dy_left_roll

        # New angle increments (left)
        try:
            dq1L =  ( (l3 * cos(q1 + q2 + q3) + l2 * cos(q1 + q2)) * dx_left_pitch + \
                (l3 * sin(q1 + q2 + q3) + l2 * sin(q1 + q2)) * dy_left_roll ) / \
                (l1 * l3 * sin(q2 + q3) + l1 * l2 * sin(q2)) 
            dq2L = ( (l3 * cos(q1 + q2 + q3) + l2 * cos(q1 + q2) + l1 * cos(q1)) * dx_left_pitch +  \
                (l3 * sin(q1 + q2 + q3) + l2 * sin(q1 + q2) + l1 * sin(q1)) * dy_left_roll ) /  \
                (l1 * l3 * sin(q2 + q3) + l1 * l2 * sin(q2))
        except ZeroDivisionError:
            print "ZeroDivisionError"
            # TODO Handle zero division

            #dq2r - any value
            dq2L = 0
            dq1L = 0

        print 'dq1L: ', dq1L, ' dq2L: ', dq2L
        print ''
        
        return (dq1L, dq2L) 



    def joint1_left_cb(self, msg):
        self.q1_left = msg.process_value
    def joint2_left_cb(self, msg):
        self.q2_left = msg.process_value
    def joint3_left_cb(self, msg):
        self.q3_left = msg.process_value
    
    def joint1_right_cb(self, msg):
        self.q1_right = msg.process_value
    def joint2_right_cb(self, msg):
        self.q2_right = msg.process_value
    def joint3_right_cb(self, msg):
        self.q3_right = msg.process_value

    def saturation(self, x, limit):
        if x > limit:
            return limit

        if x < (- limit):
            return -limit

        return x

if __name__ == "__main__":
    rospy.init_node('mmuav_merge_controller_outputs')
    merge_controller_outputs = MergeControllerOutputs()
    merge_controller_outputs.run()
