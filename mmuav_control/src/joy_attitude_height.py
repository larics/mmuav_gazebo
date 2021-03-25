#!/usr/bin/env python3

__author__ = 'aivanovic'

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import copy

class JoyToEuler:

    def __init__(self):
        self.euler_joy_ref = Vector3()
        self.euler_joy_ref_old = Vector3()
        self.pos_ref = Vector3()
        self.pos_ref_old = Vector3()
        self.pos_ref_old.z = 1.0
        self.rate = rospy.get_param('~rate', 100)
        self.Ts = 1.0/float(self.rate)
        self.max_roll = rospy.get_param('~max_roll', 0.5)
        self.max_pitch = rospy.get_param('~max_pitch', 0.5)
        self.max_yaw_rate = rospy.get_param('~max_yaw_rate', 1.0)
        self.max_velocity_z = rospy.get_param('~max_velocity_z', 1.0)
        self.joy = Joy()
        self.first_joy_message_received = False

        self.pub_euler_ref = rospy.Publisher('euler_ref', Vector3,
            queue_size=1)
        self.pub_pos_ref = rospy.Publisher('pos_ref', Vector3, 
            queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        print("Starting joy control.")


    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(self.Ts)

            if self.first_joy_message_received == True:
                self.euler_joy_ref.x = -self.joy.axes[0]*self.max_roll
                self.euler_joy_ref.y = self.joy.axes[1]*self.max_pitch
                self.euler_joy_ref.z = self.joy.axes[2]*self.max_yaw_rate*self.Ts + self.euler_joy_ref_old.z

                self.pos_ref.z = self.joy.axes[3]*self.max_velocity_z*self.Ts + self.pos_ref_old.z

                self.pub_euler_ref.publish(self.euler_joy_ref)
                self.pub_pos_ref.publish(self.pos_ref)
                self.euler_joy_ref_old = copy.deepcopy(self.euler_joy_ref)
                self.pos_ref_old = copy.deepcopy(self.pos_ref)

    def joy_callback(self,msg):
        self.first_joy_message_received = True
        self.joy = msg

if __name__ == '__main__':

    rospy.init_node('joy_attitude_control')
    joy = JoyToEuler()
    joy.run()