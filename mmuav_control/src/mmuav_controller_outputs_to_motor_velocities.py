#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped, PoseStamped, Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray
import math
from mav_msgs.msg import Actuators
from datetime import datetime
from rosgraph_msgs.msg import Clock
import time
import DualManipulatorPassiveJointKinematics as ArmsKinematics
import copy

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

        # Publisher for motor velocities
        self.mot_vel_pub = rospy.Publisher('/gazebo/command/motor_speed', 
            Actuators, queue_size=1)
        self.q1_left_pub = rospy.Publisher('joint1_left_controller/command', Float64, queue_size=1)
        self.q2_left_pub = rospy.Publisher('joint2_left_controller/command', Float64, queue_size=1)
        self.q3_left_pub = rospy.Publisher('joint3_left_controller/command', Float64, queue_size=1)
        self.q1_right_pub = rospy.Publisher('joint1_right_controller/command', Float64, queue_size=1)
        self.q2_right_pub = rospy.Publisher('joint2_right_controller/command', Float64, queue_size=1)
        self.q3_right_pub = rospy.Publisher('joint3_right_controller/command', Float64, queue_size=1)

        rospy.Subscriber('payload_position', Point, self.payload_pos_cb, queue_size=1)
        self.dx = 0
        self.dy = 0
        self.q_left = [2.463-1.57, -2.4846+1.57, -1.117]
        self.q_right = [-0.6769896+1.57, -2.4846+1.57, -1.117]

        # Sleep for few seconds then publish initial arms q
        rospy.sleep(5.0)
        self.q1_left_pub.publish(Float64(self.q1_left))
        self.q2_left_pub.publish(Float64(self.q2_left))
        self.q3_left_pub.publish(Float64(self.q3_left))
        self.q1_right_pub.publish(Float64(self.q1_right))
        self.q2_right_pub.publish(Float64(self.q2_right))
        self.q3_right_pub.publish(Float64(self.q3_right))


    def run(self):

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            L1 = 0.094
            L2 = 0.061
            L3 = 0.08
            arms_pos = [- self.dx, - self.dy]
            
            #print arms_pos
            t0 = time.time()
            q = ArmsKinematics.ik_both_arms(self.q_right, 
                self.q_left, arms_pos, L1, L2, L3)
            #print time.time() - t0
            self.q_right = copy.deepcopy(q[0])
            self.q_left = copy.deepcopy(q[1])
            q_left = self.q_left
            q_right = self.q_right
            attitude_output = [q_left[0]+1.57, q_left[1]-1.57, q_left[2], \
                q_right[0]-1.57, q_right[1]-1.57, q_right[2]]
            
            self.q1_left_pub.publish(Float64(attitude_output[0]))
            self.q2_left_pub.publish(Float64(attitude_output[1]))
            self.q3_left_pub.publish(Float64(- attitude_output[2]))
            self.q1_right_pub.publish(Float64(attitude_output[3]))
            self.q2_right_pub.publish(Float64(attitude_output[4]))
            self.q3_right_pub.publish(Float64(-attitude_output[5]))

    def payload_pos_cb(self, msg):
        self.dx = msg.x
        self.dy = msg.y

if __name__ == "__main__":
    rospy.init_node('mmuav_merge_controller_outputs')
    merge_controller_outputs = MergeControllerOutputs()
    merge_controller_outputs.run()
