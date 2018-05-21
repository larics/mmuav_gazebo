#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray
import math
from mav_msgs.msg import Actuators
from datetime import datetime
from rosgraph_msgs.msg import Clock
import time

class MergeControllerOutputs:

    def __init__(self):
        self.rate = rospy.get_param('~rate', 100)
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

        # Subscribers to height and attitude controllers
        rospy.Subscriber('attitude_command', Float64MultiArray, 
            self.attitude_command_cb, queue_size=1)
        rospy.Subscriber('mot_vel_ref', Float64, 
            self.motor_velocity_ref_cb, queue_size=1)

        # Sleep for few seconds then publish initial arms q
        rospy.sleep(2.0)
        self.q1_left_pub.publish(Float64(self.q1_left))
        self.q2_left_pub.publish(Float64(self.q2_left))
        self.q3_left_pub.publish(Float64(self.q3_left))
        self.q1_right_pub.publish(Float64(self.q1_right))
        self.q2_right_pub.publish(Float64(self.q2_right))
        self.q3_right_pub.publish(Float64(self.q3_right))


    def run(self):
        while (not self.attitude_command_received_flag) and (not rospy.is_shutdown()):
            print "Waiting for attitude controller to start"
            rospy.sleep(0.5)
        print "Attitude control started."

        while (not self.mot_vel_ref_received_flag) and (not rospy.is_shutdown()):
            print "Waiting for height controller to start"
            rospy.sleep(0.5)
        print "Height control started."

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            # Compute motor velocities, + configuration
            mot1 = self.mot_vel_ref - self.vpc_pitch_command + self.yaw_command
            mot2 = self.mot_vel_ref + self.vpc_roll_command - self.yaw_command
            mot3 = self.mot_vel_ref + self.vpc_pitch_command + self.yaw_command
            mot4 = self.mot_vel_ref - self.vpc_roll_command - self.yaw_command

            mot_speed_msg = Actuators()
            mot_speed_msg.header.stamp = rospy.Time.now()
            mot_speed_msg.angular_velocities = [mot1, mot2, mot3, mot4]
            self.mot_vel_pub.publish(mot_speed_msg)

            self.q1_left_pub.publish(Float64(self.q1_left))
            self.q2_left_pub.publish(Float64(self.q2_left))
            self.q3_left_pub.publish(Float64(self.q3_left))
            self.q1_right_pub.publish(Float64(self.q1_right))
            self.q2_right_pub.publish(Float64(self.q2_right))
            self.q3_right_pub.publish(Float64(self.q3_right))


    def attitude_command_cb(self, msg):
        if len(msg.data) < 9:
            print "Not enough data, should be 9. Length: ", len(msg.data)
        else:
            self.attitude_command_received_flag = True
            self.q1_left = msg.data[0]
            self.q2_left = msg.data[1]
            self.q3_left = -msg.data[2]
            self.q1_right = msg.data[3]
            self.q2_right = msg.data[4]
            self.q3_right = -msg.data[5]
            self.yaw_command = msg.data[6]
            self.vpc_roll_command = msg.data[7]
            self.vpc_pitch_command = msg.data[8]


    def motor_velocity_ref_cb(self, msg):
        self.mot_vel_ref = msg.data
        self.mot_vel_ref_received_flag = True


if __name__ == "__main__":
    rospy.init_node('mmuav_merge_controller_outputs')
    merge_controller_outputs = MergeControllerOutputs()
    merge_controller_outputs.run()
