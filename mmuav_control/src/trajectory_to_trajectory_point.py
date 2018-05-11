#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from pid import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, \
    TwistStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
import copy

class TrajectoryToTrajectoryPoint:

    def __init__(self):
        
        self.trajectory_point_pub = rospy.Publisher('trajectory_point_ref', 
            MultiDOFJointTrajectoryPoint, queue_size=1)
        rospy.Subscriber('multi_dof_trajectory', MultiDOFJointTrajectory, 
            self.multi_dof_trajectory_cb, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        self.t_start = rospy.Time.now()
        self.executing_trajectory_flag = False
        self.trajectory = MultiDOFJointTrajectory()
        self.current_trajectory_point = MultiDOFJointTrajectoryPoint()

    def run(self):
        
        sample_time = 1.0/float(self.rate)
        while not rospy.is_shutdown():
            rospy.sleep(sample_time)

            if self.executing_trajectory_flag == True:
                # Take first point from trajectory, publish it and remove it
                # from trajectory
                self.current_trajectory_point = self.trajectory.points[0]
                self.trajectory_point_pub.publish(self.current_trajectory_point)
                self.trajectory.points.pop(0)
                
                if len(self.trajectory.points) == 0:
                    self.executing_trajectory_flag = False


    def multi_dof_trajectory_cb(self, msg):
        print "Received a trajectory."
        if self.executing_trajectory_flag == False and \
            len(msg.points) > 0:
            self.trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
        else:
            print "Currently executing a trajectory."


if __name__ == '__main__':

    rospy.init_node('trajectory_to_trajectory_point')
    trajectory_to_ref = TrajectoryToTrajectoryPoint()
    trajectory_to_ref.run()

