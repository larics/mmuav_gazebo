#!/usr/bin/env python

__author__ = 'mcar'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray
import math
from mav_msgs.msg import Actuators
from datetime import datetime
from rosgraph_msgs.msg import Clock

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

        # Publisher for motor velocities
        self.mot_vel_pub = rospy.Publisher('command/motors', 
            Actuators, queue_size=1)

        # Publishers for moving masses
        self.tilt_front_pub = rospy.Publisher('tilt_0_position_controller/command', Float64, queue_size=1)
        self.tilt_left_pub = rospy.Publisher('tilt_1_position_controller/command', Float64, queue_size=1)
        self.tilt_back_pub = rospy.Publisher('tilt_2_position_controller/command', Float64, queue_size=1)
        self.tilt_right_pub = rospy.Publisher('tilt_3_position_controller/command', Float64, queue_size=1)
        self.tilt_all_pub = rospy.Publisher('tilt_all/command', Float64MultiArray, queue_size=1)

        # Subscribers to height and attitude controllers
        rospy.Subscriber('attitude_command', Float64MultiArray, 
            self.attitude_command_cb, queue_size=1)
        rospy.Subscriber('mot_vel_ref', Float64, 
            self.motor_velocity_ref_cb, queue_size=1)

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

            # Compute motor velocities, + configuration-0.1
            mot1 = self.mot_vel_ref + self.yaw_command - self.vpc_pitch_command
            mot2 = self.mot_vel_ref - self.yaw_command + self.vpc_roll_command
            mot3 = self.mot_vel_ref + self.yaw_command + self.vpc_pitch_command
            mot4 = self.mot_vel_ref - self.yaw_command - self.vpc_roll_command

            tilt_front = -0.25 - self.pitch_command # mm1
            tilt_back = -0.25 + self.pitch_command # mm3
            tilt_left = 0.25 - self.roll_command # mm2
            tilt_right = 0.25 + self.roll_command # mm4

            # Publish everything
            mot_speed_msg = Actuators()
            mot_speed_msg.header.stamp = rospy.Time.now()
            mot_speed_msg.angular_velocities = [mot1, mot2, mot3, mot4]
            self.mot_vel_pub.publish(mot_speed_msg)

            self.tilt_front_pub.publish(Float64(tilt_front))
            self.tilt_back_pub.publish(Float64(tilt_back))
            self.tilt_left_pub.publish(Float64(tilt_left))
            self.tilt_right_pub.publish(Float64(tilt_right))
            all_tilt_msg = Float64MultiArray()
            all_tilt_msg.data = [tilt_front, tilt_left, tilt_back, tilt_right]
            self.tilt_all_pub.publish(all_tilt_msg)


    def attitude_command_cb(self, msg):
        try:
            self.roll_command = msg.data[0]
            self.pitch_command = msg.data[1]
            self.yaw_command = msg.data[2]
            self.vpc_roll_command = msg.data[3]
            self.vpc_pitch_command = msg.data[4]
            self.attitude_command_received_flag = True
        except:
            print "Not enough data. Length of data array: ", len(msg.data)

    def motor_velocity_ref_cb(self, msg):
        self.mot_vel_ref = msg.data
        self.mot_vel_ref_received_flag = True


if __name__ == "__main__":
    rospy.init_node('vpc_mmcuav_merge_controller_outputs')
    merge_controller_outputs = MergeControllerOutputs()
    merge_controller_outputs.run()
