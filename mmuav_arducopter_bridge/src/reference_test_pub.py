#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from geometry_msgs.msg import Vector3

if __name__ == '__main__':

    rospy.init_node('vpc_mmcuav_ref_pub')
    pub_euler_ref = rospy.Publisher('euler_ref', Vector3, queue_size=1)
    euler_ref_msg = Vector3()
    euler_ref_msg.y = 0
    pub_euler_ref.publish(euler_ref_msg)
    ref_seq = [0, 0.2, 0, -0.2]# 0.2, 0, -0.2, 0]
    i = 0
    #for ref in ref_seq:
    while not rospy.is_shutdown():
        euler_ref_msg.y = ref_seq[i]
        i = (i + 1) % 4
        pub_euler_ref.publish(euler_ref_msg)
        print "Published {}. reference".format(i)
        rospy.sleep(10)