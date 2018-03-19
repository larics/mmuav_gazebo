#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from geometry_msgs.msg import PoseStamped, WrenchStamped

class exp1():

    def __init__(self):

        self.pose_ref_pub_ = rospy.Publisher('/mmuav/impedance_control/pose_ref', PoseStamped, queue_size=1)
        self.force_torque_ref_pub = rospy.Publisher('/mmuav/impedance_control/force_torque_ref', WrenchStamped, queue_size=1)
        # Services for requesting trajectory interpolation
        rospy.sleep(5.)

        self.tempPose = PoseStamped()
        self.tempForce = WrenchStamped()

        self.tempPose.header.stamp = rospy.Time.now() 
        self.tempPose.pose.position.x = 0;
        self.tempPose.pose.position.y = 0;
        self.tempPose.pose.position.z = 2;
        self.tempPose.pose.orientation.x = 0;
        self.tempPose.pose.orientation.y = 0;
        self.tempPose.pose.orientation.z = 0;
        self.tempPose.pose.orientation.w = 1;
        self.pose_ref_pub_.publish(self.tempPose)
        rospy.sleep(10.)

        self.tempPose.header.stamp = rospy.Time.now() 
        self.tempPose.pose.position.x = 2;
        self.pose_ref_pub_.publish(self.tempPose)
        rospy.sleep(5.)

        self.tempPose.header.stamp = rospy.Time.now() 
        self.tempPose.pose.position.z = 1.20;
        self.pose_ref_pub_.publish(self.tempPose)
        rospy.sleep(30.)

        self.tempPose.header.stamp = rospy.Time.now() 
        self.tempPose.pose.position.z = 1.1768;
        self.pose_ref_pub_.publish(self.tempPose)
        rospy.sleep(5.)

        #self.tempPose.pose.position.z = 1.1768;
        #self.pose_ref_pub_.publish(self.tempPose)
        #rospy.sleep(10.)

        #self.tempPose.pose.position.z = 1.2;
        #self.pose_ref_pub_.publish(self.tempPose)
        #rospy.sleep(30.)

        self.tempForce.header.stamp = rospy.Time.now() 
        self.tempForce.wrench.force.z = 2.0
        self.force_torque_ref_pub.publish(self.tempForce)
        rospy.sleep(30.)

        #self.tempForce.header.stamp = rospy.Time.now() 
        #self.tempForce.wrench.force.z = 5
        #self.force_torque_ref_pub.publish(self.tempForce)
        #rospy.sleep(6.)

        #self.tempForce.header.stamp = rospy.Time.now() 
        #self.tempForce.wrench.force.z = 10
        #self.force_torque_ref_pub.publish(self.tempForce)
        #rospy.sleep(6.)

        #self.tempForce.header.stamp = rospy.Time.now() 
        #self.tempForce.wrench.force.z = 5
        #self.force_torque_ref_pub.publish(self.tempForce)
        #rospy.sleep(6.)

        #self.tempForce.header.stamp = rospy.Time.now() 
        #self.tempForce.wrench.force.z = 2.0
        #self.force_torque_ref_pub.publish(self.tempForce)
        #rospy.sleep(6.)

        #self.tempForce.header.stamp = rospy.Time.now() 
        #self.tempForce.wrench.force.z = 0.0
        #self.force_torque_ref_pub.publish(self.tempForce)
        #rospy.sleep(6.)


if __name__=="__main__":
    rospy.init_node("exp1")
    exp1()