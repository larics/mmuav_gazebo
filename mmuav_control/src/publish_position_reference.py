#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from geometry_msgs.msg import Vector3


class PublishReference():

    def __init__(self):
        self.position_ref_pub = rospy.Publisher('/mmuav/pos_ref', Vector3, queue_size=1)

    def run(self):
        time.sleep(1)
        msg = Vector3()
        msg.x = 1.0
        msg.y = 0.0
        msg.z = 1.0
        self.position_ref_pub.publish(msg)

        time.sleep(20)
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 1.0
        self.position_ref_pub.publish(msg)

        time.sleep(20)
        msg.x = -1.0
        msg.y = 0.0
        msg.z = 1.0
        self.position_ref_pub.publish(msg)

        time.sleep(20)
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 1.0
        self.position_ref_pub.publish(msg)

    def trajectory_callback(self, msg):
        trajectory = msg

        print("Trajectory received, length:", len(trajectory.points))
        rate = rospy.Rate(100)
        position_ref = Vector3()
        for i in range(len(trajectory.points)):
            rate.sleep()
            position_ref.x = trajectory.points[i].transforms[0].translation.x
            position_ref.y = trajectory.points[i].transforms[0].translation.y
            position_ref.z = trajectory.points[i].transforms[0].translation.z
            self.position_ref_pub.publish(position_ref)

if __name__=="__main__":
    rospy.init_node("PublishReference")
    traj = PublishReference()
    traj.run()