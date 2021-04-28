#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from pid import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, \
    TwistStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
import copy

class TrajectoryToTrajectoryPoint:

    def __init__(self):
        
        self.trajectory_point_pub = rospy.Publisher('trajectory_point_ref', 
            MultiDOFJointTrajectoryPoint, queue_size=1)
        self.executing_trajectory_pub = rospy.Publisher('executing_trajectory', 
            Int32, queue_size=1)
        rospy.Subscriber('multi_dof_trajectory', MultiDOFJointTrajectory, 
            self.multi_dof_trajectory_cb, queue_size=1)
        rospy.Subscriber('stop_trajectory_execution', Empty, 
            self.stop_trajectory_execution_cb, queue_size=1)
        rospy.Subscriber('trajectory_type', Int32, 
            self.trajectory_type_cb, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.trajectory_type = rospy.get_param('~trajectory_type', 0)
        self.split_start = rospy.get_param('~split_start', 5.21) #11.13
        self.split_end = rospy.get_param('~split_end', 9.59) #16.85
        self.ros_rate = rospy.Rate(self.rate) 
        self.t_start = rospy.Time.now()
        self.executing_trajectory_flag = False
        self.trajectory = MultiDOFJointTrajectory()
        self.current_trajectory_point = MultiDOFJointTrajectoryPoint()
        self.endless_trajectory = MultiDOFJointTrajectory()

    def run(self):
        
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.executing_trajectory_flag == True:
                self.executing_trajectory_pub.publish(1)
                # Take first point from trajectory, publish it and remove it
                # from trajectory
                self.current_trajectory_point = self.trajectory.points[0]
                self.trajectory_point_pub.publish(self.current_trajectory_point)
                self.trajectory.points.pop(0)
                
                if len(self.trajectory.points) == 0:
                    if self.trajectory_type == 1:
                        self.trajectory = copy.deepcopy(self.endless_trajectory)
                    else:
                        self.executing_trajectory_flag = False
            else:
                self.executing_trajectory_pub.publish(0)


    def multi_dof_trajectory_cb(self, msg):
        print("Received a trajectory.")
        #self.executing_trajectory_flag = False
        if self.executing_trajectory_flag == False and \
            len(msg.points) > 0:
            if self.trajectory_type == 1:
                self.splitTrajectory(msg)
            else:
                self.trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
        else:
            print("Currently executing a trajectory.")

    def stop_trajectory_execution_cb(self, msg):
        self.executing_trajectory_flag = False

    def trajectory_type_cb(self, msg):
        self.trajectory_type = msg.data

    def splitTrajectory(self, traj):
        start_point = int(self.split_start*self.rate)
        end_point = int(self.split_end*self.rate)

        self.trajectory = MultiDOFJointTrajectory()

        for i in range(start_point, end_point+1):
            self.trajectory.points.append(traj.points[i])
            self.trajectory.points[i-start_point].time_from_start = rospy.Duration(float(i-start_point)/float(self.rate))

        self.endless_trajectory = copy.deepcopy(self.trajectory)
        

if __name__ == '__main__':

    rospy.init_node('trajectory_to_trajectory_point')
    trajectory_to_ref = TrajectoryToTrajectoryPoint()
    trajectory_to_ref.run()

