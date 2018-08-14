#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class CalculateDesired:
    """
    This class uses bebop_launch node to guide bebop in a
    circle around the windmill. It will not start flying until
    a message on /bebop/start_flight is published.
    """

    def __init__(self):

        # Reference publishers
        self.pos_ref_pub = rospy.Publisher(
            "bebop/pos_ref",
            Vector3,
            queue_size=10)
        self.pos_ref_msg = Vector3()

        self.ang_ref_pub = rospy.Publisher(
            "bebop/angle_ref",
            Vector3,
            queue_size=10)
        self.ang_ref_msg = Vector3()

        # Odometry subscriber
        self.odom_subscriber = rospy.Subscriber(
            "bebop/odometry",
            Odometry,
            self.odometry_callback)
        self.first_measurement = False

        # Flight control signal
        self.fc_subscriber = rospy.Subscriber(
            "bebop/flight_control",
            Vector3,
            self.fc_callback)
        self.forward = 1    # 1 Is forward, 0 is backward
        self.start = 0      # 1 is start, 0 is stop
        self.i = 0

        # Sleep time if no measurement found
        self.sleep_sec = 2

        self.x_list = []
        self.y_list = []
        self.theta_list = []
        self.d_theta = 1  # angle discretisation

        # Crontroller rate
        self.controller_rate = 10
        self.rate = rospy.Rate(self.controller_rate)

        # ~Windmill height(?)
        self.windmill_height = 2.16
        self.back_dist = - 1.5
        self.angle_delta = math.pi / 180 * 5
        self.windmill_radius = 2 - self.back_dist

        self.fc_sleep = 0

    def odometry_callback(self, data):
        """Callback function for odometry subscriber"""

        self.first_measurement = True

        self.x_mv = data.pose.pose.position.x
        self.y_mv = data.pose.pose.position.y
        self.z_mv = data.pose.pose.position.z

        self.vx_mv = data.twist.twist.linear.x
        self.vy_mv = data.twist.twist.linear.y
        self.vz_mv = data.twist.twist.linear.z

        self.p = data.twist.twist.angular.x
        self.q = data.twist.twist.angular.y
        self.r = data.twist.twist.angular.z

        self.qx = data.pose.pose.orientation.x
        self.qy = data.pose.pose.orientation.y
        self.qz = data.pose.pose.orientation.z
        self.qw = data.pose.pose.orientation.w

    def quaternion2euler(self, qx, qy, qz, qw):
        """
        Calculate roll, pitch and yaw angles/rates with quaternions.

        :returns:
            This function returns following information:
                pitch, roll, yaw,
                pitch_rate, roll_rate, yaw_rate
        """
        # conversion quaternion to euler (yaw - pitch - roll)
        roll = math.atan2(2 * (qw * qx + qy * qz), qw * qw
                          - qx * qx - qy * qy + qz * qz)
        pitch = math.asin(2 * (qw * qy - qx * qz))
        yaw = math.atan2(2 * (qw * qz + qx * qy), qw * qw
                         + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = self.p
        q = self.q
        r = self.r

        sx = math.sin(roll)  # sin(roll)
        cx = math.cos(roll)  # cos(roll)
        cy = math.cos(pitch)  # cos(pitch)
        ty = math.tan(pitch)  # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        roll_rate = p + sx * ty * q + cx * ty * r
        pitch_rate = cx * q - sx * r
        yaw_rate = sx / cy * q + cx / cy * r

        return roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate

    def run(self):

        while not self.first_measurement:
            print("BebopCircleFlight.run() - Waiting for first measurement.")
            rospy.sleep(self.sleep_sec)

        while self.start == 0:
            print("BebopCircleFlight.run() - Waiting for flight control.")
            rospy.sleep(self.sleep_sec)

        # Takeoff
        self.get_current_position()
        self.takeoff()

        # Get initial position
        self.get_current_position()
        self.initial_x = self.curr_x
        self.initial_y = self.curr_y
        self.initial_z = self.curr_z
        self.initial_yaw = self.curr_yaw

        self.windmill_x = self.initial_x + math.cos(self.initial_yaw) * (self.windmill_radius + self.back_dist)
        self.windmill_y = self.initial_y + math.sin(self.initial_yaw) * (self.windmill_radius + self.back_dist)

        for i in range(360 // self.d_theta):
            x, y = self.windmill_x + self.windmill_radius * np.cos(i * np.deg2rad(self.d_theta)), \
                   self.windmill_y + self.windmill_radius * np.sin(i * np.deg2rad(self.d_theta))
            theta_p = -(180 - i * self.d_theta)
            self.x_list.append(x)
            self.y_list.append(y)
            self.theta_list.append(np.deg2rad(theta_p))

        distances = []

        for i in zip(self.x_list, self.y_list):
            distances.append(abs(i[0] - self.initial_x) + abs(i[1] - self.initial_y))

        dis_array = np.array(distances)
        i_min = np.argmin(dis_array)
        list_len = len(self.x_list)

        self.circle_x = self.x_list[i_min:list_len] + self.x_list[0:i_min]
        self.circle_y = self.y_list[i_min:list_len] + self.y_list[0:i_min]
        self.circle_theta = self.theta_list[i_min:list_len] + self.theta_list[0:i_min]

        while not rospy.is_shutdown():
            self.rate.sleep()
            self.get_current_position()

            if self.start == 0:
                print("CircleFlight.run() - Flight control - Stop.")
                continue

            self.pos_ref_msg.y = self.circle_y[self.i]
            self.pos_ref_msg.x = self.circle_x[self.i]
            self.ang_ref_msg.z = self.circle_theta[self.i]

            print("X_off: {}, y_off: {}, yaw_off: {}"
                  .format(self.pos_ref_msg.x, self.pos_ref_msg.y, self.ang_ref_msg.z))

            # Publish position
            self.pos_ref_pub.publish(self.pos_ref_msg)

            # Publish angle
            self.ang_ref_pub.publish(self.ang_ref_msg)
            print(self.fc_sleep)
            rospy.sleep(self.fc_sleep)

            self.i += int(self.forward)
            print(self.i)
            if self.i > (list_len - 1):
                self.i = 0
            elif self.i < 0:
                self.i = (list_len - 1)

    def get_current_position(self):
        """
        Get current bebop position
        """
        # Get current position
        _, _, self.curr_yaw, _, _, _ = self.quaternion2euler(
            self.qx, self.qy, self.qz, self.qw)
        self.curr_x = self.x_mv
        self.curr_y = self.y_mv
        self.curr_z = self.z_mv

    def takeoff(self):
        # Take - off
        print("BebopCircleFlight() - takeoff ready.")

        self.pos_ref_msg.x = self.curr_x
        self.pos_ref_msg.y = self.curr_y
        self.pos_ref_msg.z = self.windmill_height
        self.pos_ref_pub.publish(self.pos_ref_msg)

        print("BebopCircleFlight() - takeoff completed.")
        rospy.sleep(self.sleep_sec)

    def fc_callback(self, data):
        self.forward = data.x
        self.start = data.y
        self.fc_sleep = data.z

if __name__ == '__main__':
    rospy.init_node('calculate_desired', anonymous=True)
    try:
        cf = CalculateDesired()
        cf.run()
    except rospy.ROSInterruptException:
        pass