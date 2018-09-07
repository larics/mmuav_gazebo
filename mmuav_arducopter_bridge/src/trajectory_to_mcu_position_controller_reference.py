#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Pose
from morus_uav_ros_msgs.msg import PositionControllerReference


class TrajectoryToPositionReference():

    def __init__(self):
        self.rate = rospy.get_param("~rate", 20)
        print "----------------------RATE-------------------", self.rate
        rospy.Subscriber('multi_dof_trajectory', MultiDOFJointTrajectory, 
            self.trajectory_callback, queue_size=1)
        self.position_controller_reference_pub = rospy.Publisher(
            'position_controller_reference', PositionControllerReference, 
            queue_size=1)

    def run(self):
        rospy.spin()

    def trajectory_callback(self, msg):
        trajectory = msg

        print "Trajectory received, length:", len(trajectory.points)
        rate = rospy.Rate(float(self.rate))
        ref = PositionControllerReference()
        for i in range(len(trajectory.points)):
            rate.sleep()
            ref.position_x = trajectory.points[i].transforms[0].translation.x
            ref.position_y = trajectory.points[i].transforms[0].translation.y
            ref.position_z = trajectory.points[i].transforms[0].translation.z

            ref.velocity_x = trajectory.points[i].velocities[0].linear.x
            ref.velocity_y = trajectory.points[i].velocities[0].linear.y
            ref.velocity_z = trajectory.points[i].velocities[0].linear.z

            ref.acceleration_x = trajectory.points[i].accelerations[0].linear.x
            ref.acceleration_y = trajectory.points[i].accelerations[0].linear.y
            ref.acceleration_z = trajectory.points[i].accelerations[0].linear.z

            qx = trajectory.points[i].transforms[0].rotation.x
            qy = trajectory.points[i].transforms[0].rotation.y
            qz = trajectory.points[i].transforms[0].rotation.z
            qw = trajectory.points[i].transforms[0].rotation.w
            ref.yaw = math.atan2(
                2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)
            ref.yaw_rate = trajectory.points[i].velocities[0].angular.z

            self.position_controller_reference_pub.publish(ref)

        print "Trajectory finished."

if __name__=="__main__":
    rospy.init_node("TrajectoryToMcuPositionReference")
    traj = TrajectoryToPositionReference()
    traj.run()
