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

class GripperControl:

    def __init__(self):
        self.rate = rospy.get_param('rate', 100)
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

        # Topics adapted for double joint controllers
        self.q1_left_pub = rospy.Publisher('joint1_left_controller/command', Float64, queue_size=1)
        self.q2_left_pub = rospy.Publisher('joint2_left_controller/command', Float64, queue_size=1)
        self.q3_left_pub = rospy.Publisher('joint3_left_controller/command', Float64, queue_size=1)
        self.q1_right_pub = rospy.Publisher('joint1_right_controller/command', Float64, queue_size=1)
        self.q2_right_pub = rospy.Publisher('joint2_right_controller/command', Float64, queue_size=1)
        self.q3_right_pub = rospy.Publisher('joint3_right_controller/command', Float64, queue_size=1)

        self.left_gripper_pub = rospy.Publisher('left_gripper_pos', Point, queue_size=1)
        self.right_gripper_pub = rospy.Publisher('right_gripper_pos', Point, queue_size=1)

        rospy.Subscriber('payload_position', Point, self.payload_pos_cb, queue_size=1)
        rospy.Subscriber('joint1_left_controller/state', JointControllerState, self.joint1_left_cb)
        rospy.Subscriber('joint2_left_controller/state', JointControllerState, self.joint2_left_cb)
        rospy.Subscriber('joint3_left_controller/state', JointControllerState, self.joint3_left_cb)
        rospy.Subscriber('joint1_right_controller/state', JointControllerState, self.joint1_right_cb)
        rospy.Subscriber('joint2_right_controller/state', JointControllerState, self.joint2_right_cb)
        rospy.Subscriber('joint3_right_controller/state', JointControllerState, self.joint3_right_cb)

        self.dx = 0
        self.dy = 0

        self.x_right_offset = 0.158342
        self.y_right_offset = 0

        self.x_left_offset = 0.158342
        self.y_left_offset = 0

        self.L1 = 0.094
        self.L2 = 0.061
        self.L3 = 0.08

        self.dx_old = 0
        self.dy_old = 0
        
    def run(self):

         # Sleep for few seconds then publish initial arms q
        print("Setting initial pos")
        rospy.sleep(5.0)
        self.q1_left_pub.publish(Float64(2.493))
        self.q2_left_pub.publish(Float64(-2.56))
        self.q3_left_pub.publish(Float64(-1.08))
        self.q1_right_pub.publish(Float64(-0.645))
        self.q2_right_pub.publish(Float64(-2.567))
        self.q3_right_pub.publish(Float64(-1.08))
        rospy.sleep(5.0)

        while not rospy.is_shutdown():
            self.ros_rate.sleep()
            
            dx_curr = self.dx_old + 1 * (self.dx - self.dx_old)
            dy_curr = self.dy_old + 1 * (self.dy - self.dy_old)
            
            dqR = self.get_new_dqR(dx_curr, dy_curr, 0.01)
            dqL = self.get_new_dqL(dx_curr, dy_curr, 0.01)

            self.dx_old = dx_curr
            self.dy_old = dy_curr

            self.q1_left_pub.publish(Float64(self.q1_left + dqL[0] + 1.57))
            self.q2_left_pub.publish(Float64(self.q2_left + dqL[1] - 1.57))
            self.q3_left_pub.publish(Float64(self.q3_left + dqL[2]))
            self.q1_right_pub.publish(Float64(self.q1_right + dqR[0] - 1.57))
            self.q2_right_pub.publish(Float64(self.q2_right + dqR[1] - 1.57))
            self.q3_right_pub.publish(Float64(self.q3_right + dqL[2]))

    def payload_pos_cb(self, msg):
        self.dx = - msg.y
        self.dy = - msg.x

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
        x_right_curr = l1 * cos(q1) + l2 * cos(q2 + q1) + l3 * cos(q3 + q2 + q1)
        y_right_curr = l1 * sin(q1) + l2 * sin(q2 + q1) + l3 * sin(q3 + q2 + q1)

        right_point = Point()
        right_point.y = (x_right_curr - self.x_right_offset)
        right_point.x = y_right_curr
        right_point.z = -0.07349
        self.right_gripper_pub.publish(right_point)

        #print 'x_right_target: ', x_right_target, ' y_right_target: ', y_right_target
        #print 'x_right_curr: ', x_right_curr, ' y_right_curr: ', y_right_curr
        #print 'dx: ', dx_pitch, ' dy: ', dy_roll

        dx_right_pitch = x_right_target - x_right_curr
        dy_right_roll = y_right_target - y_right_curr

        # Limit dx / dy
        dx_right_pitch = self.saturation(dx_right_pitch, limit)
        dy_right_roll = self.saturation(dy_right_roll, limit)

        #print 'dx_new: ', dx_right_pitch, ' dy_new: ', dy_right_roll

        # New angle increments (right)
        try: 
            (dq1R, dq2R, dq3R) = self.func(self.L1, self.L2, self.L3, q1, q2, q3, -dx_right_pitch, -dy_right_roll)
        except ZeroDivisionError:
            print "ZeroDivisionError"
            dq1R = 0
            dq2R = 0
            dq3R = 0
        
        #print 'dq1R: ', dq1R, ' dq2R: ', dq2R
        #print ''

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

        #print ''
        # Get current position of the right manipulator 
        x_left_curr = l1 * cos(q1) + l2 * cos(q2 + q1) + l3 * cos(q3 + q2 + q1)
        y_left_curr = l1 * sin(q1) + l2 * sin(q2 + q1) + l3 * sin(q3 + q2 + q1)

        left_point = Point()
        left_point.y = - (x_left_curr - self.x_left_offset)
        left_point.x = - y_left_curr
        left_point.z = -0.07349
        self.left_gripper_pub.publish(left_point)

        #print 'x_left_target: ', x_left_target, ' y_left_target: ', y_left_target
        #print 'x_left_curr: ', x_left_curr, ' y_left_curr: ', y_left_curr
        #print 'dx: ', dx_pitch, ' dy: ', dy_roll

        # Calculate distance to the target position
        dx_left_pitch = x_left_target - x_left_curr
        dy_left_roll = y_left_target - y_left_curr

        # Limit dx / dy
        dx_left_pitch = self.saturation(dx_left_pitch, limit)
        dy_left_roll = self.saturation(dy_left_roll, limit)

        #print 'dx_new: ', dx_left_pitch, ' dy_new: ', dy_left_roll

        # New angle increments (left)
        try:
            (dq1L, dq2L, dq3L) = self.func(self.L1, self.L2, self.L3, q1, q2, q3, -dx_left_pitch, -dy_left_roll)
        except ZeroDivisionError:
            print "ZeroDivisionError"
            dq2L = 0
            dq1L = 0
            dq3L = 0

        #print 'dq1L: ', dq1L, ' dq2L: ', dq2L
        #print ''
        
        return (dq1L, dq2L, dq3L) 


    def func(self, L_C1, L_C2, L_C3, q1, q2, q3, dx, dy):
        # Pseudoinverse
        dq1 = (L_C2*L_C3**2*sin(q1 + q2) + L_C1*L_C2**2*sin(q1) + 2*L_C1*L_C3**2*sin(q1) - 2*L_C1*L_C3**2*sin(q1 + 2*q2 + 2*q3) - L_C1*L_C2**2*sin(q1 + 2*q2) \
                - L_C2*L_C3**2*sin(q1 + q2 + 2*q3) + L_C1*L_C2*L_C3*sin(q1 - q3) - 2*L_C1*L_C2*L_C3*sin(q1 + 2*q2 + q3) + L_C1*L_C2*L_C3*sin(q1 + q3)) * dx / \
                (L_C1**2*L_C2**2 + 2*L_C1**2*L_C3**2 + 2*L_C2**2*L_C3**2 - 2*L_C1**2*L_C3**2*cos(2*q2 + 2*q3) - L_C1**2*L_C2**2*cos(2*q2) - 2*L_C2**2*L_C3**2*cos(2*q3) + \
                2*L_C1*L_C2*L_C3**2*cos(q2) + 2*L_C1**2*L_C2*L_C3*cos(q3) - 2*L_C1*L_C2*L_C3**2*cos(q2 + 2*q3) - 2*L_C1**2*L_C2*L_C3*cos(2*q2 + q3)) \
                - (L_C1*L_C2**2*cos(q1) + 2*L_C1*L_C3**2*cos(q1) - 2*L_C1*L_C3**2*cos(q1 + 2*q2 + 2*q3) - L_C1*L_C2**2*cos(q1 + 2*q2) - L_C2*L_C3**2*cos(q1 + q2 + 2*q3) \
                + L_C2*L_C3**2*cos(q1 + q2) + L_C1*L_C2*L_C3*cos(q1 - q3) - 2*L_C1*L_C2*L_C3*cos(q1 + 2*q2 + q3) + L_C1*L_C2*L_C3*cos(q1 + q3)) * dy / \
                (L_C1**2*L_C2**2 + 2*L_C1**2*L_C3**2 + 2*L_C2**2*L_C3**2 - 2*L_C1**2*L_C3**2*cos(2*q2 + 2*q3) - L_C1**2*L_C2**2*cos(2*q2) - 2*L_C2**2*L_C3**2*cos(2*q3) \
                + 2*L_C1*L_C2*L_C3**2*cos(q2) + 2*L_C1**2*L_C2*L_C3*cos(q3) - 2*L_C1*L_C2*L_C3**2*cos(q2 + 2*q3) - 2*L_C1**2*L_C2*L_C3*cos(2*q2 + q3) )

        dq2 = (L_C1**2*L_C2*sin(q1 + q2) + L_C2*L_C3**2*sin(q1 + q2) - L_C1*L_C2**2*sin(q1) - L_C1*L_C3**2*sin(q1) + L_C1*L_C3**2*sin(q1 + 2*q2 + 2*q3) -  \
                L_C1**2*L_C2*sin(q1 - q2) + L_C1*L_C2**2*sin(q1 + 2*q2) + L_C1**2*L_C3*sin(q1 + q2 + q3) + L_C1**2*L_C3*sin(q2 - q1 + q3) - L_C2*L_C3**2*sin(q1 + q2 + 2*q3) \
                - L_C1*L_C2*L_C3*sin(q1 - q3) + 2*L_C1*L_C2*L_C3*sin(q1 + 2*q2 + q3) - L_C1*L_C2*L_C3*sin(q1 + q3)) * dx/ \
                (L_C1**2*L_C2**2 + 2*L_C1**2*L_C3**2 + 2*L_C2**2*L_C3**2 - 2*L_C1**2*L_C3**2*cos(2*q2 + 2*q3) - L_C1**2*L_C2**2*cos(2*q2) - 2*L_C2**2*L_C3**2*cos(2*q3) \
                + 2*L_C1*L_C2*L_C3**2*cos(q2) + 2*L_C1**2*L_C2*L_C3*cos(q3) - 2*L_C1*L_C2*L_C3**2*cos(q2 + 2*q3) - 2*L_C1**2*L_C2*L_C3*cos(2*q2 + q3)) \
                + (L_C1*L_C2**2*cos(q1) + L_C1*L_C3**2*cos(q1) - L_C1*L_C3**2*cos(q1 + 2*q2 + 2*q3) + L_C1**2*L_C2*cos(q1 - q2) - L_C1*L_C2**2*cos(q1 + 2*q2) \
                - L_C1**2*L_C3*cos(q1 + q2 + q3) + L_C1**2*L_C3*cos(q2 - q1 + q3) + L_C2*L_C3**2*cos(q1 + q2 + 2*q3) - L_C1**2*L_C2*cos(q1 + q2) \
                - L_C2*L_C3**2*cos(q1 + q2) + L_C1*L_C2*L_C3*cos(q1 - q3) - 2*L_C1*L_C2*L_C3*cos(q1 + 2*q2 + q3) + L_C1*L_C2*L_C3*cos(q1 + q3)) * dy / \
                (L_C1**2*L_C2**2 + 2*L_C1**2*L_C3**2 + 2*L_C2**2*L_C3**2 - 2*L_C1**2*L_C3**2*cos(2*q2 + 2*q3) - L_C1**2*L_C2**2*cos(2*q2) - 2*L_C2**2*L_C3**2*cos(2*q3) + \
                2*L_C1*L_C2*L_C3**2*cos(q2) + 2*L_C1**2*L_C2*L_C3*cos(q3) - 2*L_C1*L_C2*L_C3**2*cos(q2 + 2*q3) - 2*L_C1**2*L_C2*L_C3*cos(2*q2 + q3))

        dq3 = (L_C3*(L_C1**2*sin(q1 + q2 + q3) + 2*L_C2**2*sin(q1 + q2 + q3) + L_C1**2*sin(q2 - q1 + q3) - 2*L_C2**2*sin(q1 + q2 - q3) - 2*L_C1*L_C2*sin(q1 - q3) + L_C1*L_C2*sin(q1 + 2*q2 + q3) \
                + 2*L_C2*L_C3*sin(q1 + q2 + 2*q3) + L_C1*L_C2*sin(q1 + q3) - 2*L_C2*L_C3*sin(q1 + q2) - L_C1*L_C3*sin(q1) + L_C1*L_C3*sin(q1 + 2*q2 + 2*q3))) * dx \
                /(L_C1**2*L_C2**2 + 2*L_C1**2*L_C3**2 + 2*L_C2**2*L_C3**2 - 2*L_C1**2*L_C3**2*cos(2*q2 + 2*q3) - L_C1**2*L_C2**2*cos(2*q2) - 2*L_C2**2*L_C3**2*cos(2*q3) \
                + 2*L_C1*L_C2*L_C3**2*cos(q2) + 2*L_C1**2*L_C2*L_C3*cos(q3) - 2*L_C1*L_C2*L_C3**2*cos(q2 + 2*q3) - 2*L_C1**2*L_C2*L_C3*cos(2*q2 + q3)) \
                -(L_C3*(L_C1**2*cos(q1 + q2 + q3) + 2*L_C2**2*cos(q1 + q2 + q3) - L_C1**2*cos(q2 - q1 + q3) - 2*L_C2**2*cos(q1 + q2 - q3) + L_C1*L_C2*cos(q1 + 2*q2 + q3) + 2*L_C2*L_C3*cos(q1 + q2 + 2*q3) \
                + L_C1*L_C2*cos(q1 + q3) - 2*L_C2*L_C3*cos(q1 + q2) - L_C1*L_C3*cos(q1) + L_C1*L_C3*cos(q1 + 2*q2 + 2*q3) - 2*L_C1*L_C2*cos(q1 - q3))) * dy / \
                (L_C1**2*L_C2**2 + 2*L_C1**2*L_C3**2 + 2*L_C2**2*L_C3**2 - 2*L_C1**2*L_C3**2*cos(2*q2 + 2*q3) - L_C1**2*L_C2**2*cos(2*q2) - 2*L_C2**2*L_C3**2*cos(2*q3) \
                + 2*L_C1*L_C2*L_C3**2*cos(q2) + 2*L_C1**2*L_C2*L_C3*cos(q3) - 2*L_C1*L_C2*L_C3**2*cos(q2 + 2*q3) - 2*L_C1**2*L_C2*L_C3*cos(2*q2 + q3))

        return (dq1, dq2, dq3)

    def joint1_left_cb(self, msg):
        self.q1_left = msg.process_value - 1.57
    def joint2_left_cb(self, msg):
        self.q2_left = msg.process_value + 1.57
    def joint3_left_cb(self, msg):
        self.q3_left = msg.process_value
    
    def joint1_right_cb(self, msg):
        self.q1_right = msg.process_value + 1.57
    def joint2_right_cb(self, msg):
        self.q2_right = msg.process_value + 1.57
    def joint3_right_cb(self, msg):
        self.q3_right = msg.process_value

    def saturation(self, x, limit):
        if x > limit:
            return limit

        if x < (- limit):
            return -limit

        return x

if __name__ == "__main__":
    rospy.init_node('gripper_control')
    gc = GripperControl()
    gc.run()