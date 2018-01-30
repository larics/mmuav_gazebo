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
    ref_seq_roll =  [0.0, 0.0, 0.0, -0.2, 0.0, -0.2, 0.0]
    ref_seq_pitch = [0.0, 0.2, 0.0, 0.0, 0.0, 0.2, 0.0]
    #ref_seq_roll = [0.0, 0.2, 0.0, -0.2]
    #ref_seq_pitch = [0.0, 0.2, 0.0, -0.2]
    i = 0
    #for ref in ref_seq:
    while not rospy.is_shutdown():
        euler_ref_msg.x = ref_seq_roll[i]
        euler_ref_msg.y = ref_seq_pitch[i]
        i = (i + 1) % len(ref_seq_pitch)
        pub_euler_ref.publish(euler_ref_msg)
        print "Published {}. reference".format(i)
        rospy.sleep(10)