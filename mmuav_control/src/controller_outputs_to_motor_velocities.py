#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
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

        self.mot_vel_ref = 0.0

        self.attitude_command_received_flag = False
        self.mot_vel_ref_received_flag = False

        # Publisher for motor velocities
        self.mot_vel_pub = rospy.Publisher('/gazebo/command/motor_speed', 
            Actuators, queue_size=1)

        # Subscribers to height and attitude controllers
        rospy.Subscriber('attitude_command', Vector3Stamped, 
            self.attitude_command_cb, queue_size=1)
        rospy.Subscriber('mot_vel_ref', Float64, 
            self.motor_velocity_ref_cb, queue_size=1)

    def run(self):
        while (not self.attitude_command_received_flag) and (not rospy.is_shutdown()):
            print("Waiting for attitude controller to start")
            rospy.sleep(0.5)
        print("Attitude control started.")

        while (not self.mot_vel_ref_received_flag) and (not rospy.is_shutdown()):
            print("Waiting for height controller to start")
            rospy.sleep(0.5)
        print("Height control started.")

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            # Compute motor velocities, + configuration
            mot1 = self.mot_vel_ref - self.pitch_command + self.yaw_command
            mot2 = self.mot_vel_ref + self.roll_command - self.yaw_command
            mot3 = self.mot_vel_ref + self.pitch_command + self.yaw_command
            mot4 = self.mot_vel_ref - self.roll_command - self.yaw_command

            mot_speed_msg = Actuators()
            mot_speed_msg.header.stamp = rospy.Time.now()
            mot_speed_msg.angular_velocities = [mot1, mot2, mot3, mot4]
            self.mot_vel_pub.publish(mot_speed_msg)


    def attitude_command_cb(self, msg):
        self.roll_command = msg.vector.x
        self.pitch_command = msg.vector.y
        self.yaw_command = msg.vector.z
        self.attitude_command_received_flag = True

    def motor_velocity_ref_cb(self, msg):
        self.mot_vel_ref = msg.data
        self.mot_vel_ref_received_flag = True


if __name__ == "__main__":
    rospy.init_node('uav_merge_controller_outputs')
    merge_controller_outputs = MergeControllerOutputs()
    merge_controller_outputs.run()
