#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float64MultiArray
import math

if __name__ == '__main__':

    rospy.init_node('klackalica_mm_sine_ref_pub')
    pub_mm_ref = rospy.Publisher('movable_mass_all/command', Float64MultiArray, queue_size=1)
    cmd_msg = Float64()
    T = 0.5
    w = 2.0*3.14/T
    #for ref in ref_seq:
    t = 0
    dt = 0.05
    while not rospy.is_shutdown():
        rospy.sleep(dt)
        x = 0.05 * math.sin(w*t)
        message = Float64MultiArray()
        message.data = [x, 0, -x, 0]
        pub_mm_ref.publish(message)
        #print("Published {}. reference".format(cmd_msg.data))
        t += dt