#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy

class JoyToEuler:

    def __init__(self):
        self.euler_joy_ref = Vector3()
        self.rate = rospy.get_param('rate', 100)
        self.Ts = 1.0/float(self.rate)

        self.pub_euler_ref = rospy.Publisher('euler_ref', Vector3,
            queue_size=1)
        rospy.Subscriber('joy', Joy, self.joy_callback)

        print "Starting joy control."


    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(self.Ts)

            self.pub_euler_ref.publish(self.euler_joy_ref)

    def joy_callback(self,msg):
        self.euler_joy_ref.x = -msg.axes[0]*0.2
        self.euler_joy_ref.y = msg.axes[1]*0.2
        #self.euler_joy_ref.z = msg.buttons[3] - msg.buttons[4]


if __name__ == '__main__':

    rospy.init_node('joy_ref_pub')
    joy = JoyToEuler()
    joy.run()