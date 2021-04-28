#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Pose


class TrajectoryToPositionReference():

    def __init__(self):
        self.rate = rospy.get_param("~rate", 100)
        print("----------------------RATE-------------------_", self.rate)
        rospy.Subscriber('multi_dof_trajectory', MultiDOFJointTrajectory, 
            self.trajectory_callback, queue_size=1)
        self.pose_ref_pub = rospy.Publisher('pose_ref', Pose, queue_size=1)

    def run(self):
        rospy.spin()

    def trajectory_callback(self, msg):
        trajectory = msg

        print("Trajectory received, length:", len(trajectory.points))
        rate = rospy.Rate(float(self.rate))
        pose_ref = Pose()
        for i in range(len(trajectory.points)):
            rate.sleep()
            pose_ref.position.x = trajectory.points[i].transforms[0].translation.x
            pose_ref.position.y = trajectory.points[i].transforms[0].translation.y
            pose_ref.position.z = trajectory.points[i].transforms[0].translation.z
            pose_ref.orientation.x = trajectory.points[i].transforms[0].rotation.x
            pose_ref.orientation.y = trajectory.points[i].transforms[0].rotation.y
            pose_ref.orientation.z = trajectory.points[i].transforms[0].rotation.z
            pose_ref.orientation.w = trajectory.points[i].transforms[0].rotation.w

            self.pose_ref_pub.publish(pose_ref)

if __name__=="__main__":
    rospy.init_node("TrajectoryToPositionReference")
    traj = TrajectoryToPositionReference()
    traj.run()