#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Vector3


class TrajectoryToPositionReference():

    def __init__(self):
        rospy.Subscriber('/euroc3/command/trajectory', MultiDOFJointTrajectory, 
            self.trajectory_callback, queue_size=1)
        self.position_ref_pub = rospy.Publisher('pos_ref', Vector3, queue_size=1)

    def run(self):
        rospy.spin()

    def trajectory_callback(self, msg):
        trajectory = msg

        print "Trajectory received, length:", len(trajectory.points)
        rate = rospy.Rate(100)
        position_ref = Vector3()
        for i in range(len(trajectory.points)):
            rate.sleep()
            position_ref.x = trajectory.points[i].transforms[0].translation.x
            position_ref.y = trajectory.points[i].transforms[0].translation.y
            position_ref.z = trajectory.points[i].transforms[0].translation.z
            self.position_ref_pub.publish(position_ref)

if __name__=="__main__":
    rospy.init_node("TrajectoryToPositionReference")
    traj = TrajectoryToPositionReference()
    traj.run()