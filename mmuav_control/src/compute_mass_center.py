#!/usr/bin/env python3

__author__ = 'aivanovic'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, PoseStamped
import copy

class ComputeMassCenter:

    def __init__(self):
        self.left_pose = PoseStamped()
        self.right_pose = PoseStamped()
        self.mass_center = Vector3()
        self.rate = 50.0
        self.Ts = 1.0/self.rate
        self.arms_reference = Vector3()

        self.pub_mass_center = rospy.Publisher('arms_mass_center', 
            Vector3, queue_size=1)
        self.pub_arms_reference = rospy.Publisher('arms_reference_continuous', 
            Vector3, queue_size=1)

        rospy.Subscriber('link_gripper2_left/pose', PoseStamped, 
            self.left_mass_callback, queue_size=1)
        rospy.Subscriber('link_gripper2_right/pose', PoseStamped, 
            self.right_mass_callback, queue_size=1)
        rospy.Subscriber('arms_reference', Vector3, 
            self.arms_reference_callback, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(self.Ts)

            self.mass_center.x = (self.left_pose.pose.position.x + 
                self.right_pose.pose.position.x)/2.0
            self.mass_center.y = (self.left_pose.pose.position.y + 
                self.right_pose.pose.position.y)/2.0

            self.pub_mass_center.publish(self.mass_center)
            self.pub_arms_reference.publish(self.arms_reference)


    def left_mass_callback(self,msg):
        self.left_pose = copy.deepcopy(msg)

    def right_mass_callback(self,msg):
        self.right_pose = copy.deepcopy(msg)

    def arms_reference_callback(self,msg):
        self.arms_reference = copy.deepcopy(msg)


if __name__ == "__main__":
    rospy.init_node('compute_mass_center')
    compute_center = ComputeMassCenter()
    compute_center.run()