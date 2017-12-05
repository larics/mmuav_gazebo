#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from std_msgs.msg import Empty
from dynamixel_hr_ros.msg import CommandPosition

class PulleyReference:

    def __init__(self):

        self.id_fast = rospy.get_param('~id_fast', 12)
        self.id_slow = rospy.get_param('~id_slow', 15)
        self.dt1 = rospy.get_param('~dt1', 0.5)
        self.v1 = rospy.get_param('~v1', 0)
        self.dt2 = rospy.get_param('~dt2', 0.5)
        self.v2 = rospy.get_param('~v2', 0)
        self.dt3 = rospy.get_param('~dt3', 0.5)

        self.pub_dxl_ref = rospy.Publisher('/dxl/command_position', CommandPosition, queue_size=1)

        rospy.Subscriber('start', Empty, self.start_callback)

        print "Starting pulley control."

    def run(self):
        rospy.spin()

    def start_callback(self, msg):
        pub_msg = CommandPosition()
        pub_msg.id = [self.id_fast, self.id_slow]
        pub_msg.angle = [0, 0]
        pub_msg.speed = [self.v1, 0]
        self.pub_dxl_ref.publish(pub_msg)

        rospy.sleep(self.dt1)
        pub_msg.speed = [self.v1, self.v1]
        self.pub_dxl_ref.publish(pub_msg)

        rospy.sleep(self.dt2)
        pub_msg.speed = [self.v2, self.v2]
        self.pub_dxl_ref.publish(pub_msg)

        rospy.sleep(self.dt3)
        pub_msg.speed = [0, 0]
        self.pub_dxl_ref.publish(pub_msg)


if __name__ == '__main__':

    rospy.init_node('pulley_reference')
    pulley = PulleyReference()
    pulley.run()