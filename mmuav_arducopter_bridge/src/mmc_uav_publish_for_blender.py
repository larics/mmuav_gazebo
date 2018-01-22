#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from control_msgs.msg import JointControllerState
import copy

odom = Odometry()
mm0_pos = 0
mm1_pos = 0
mm2_pos = 0
mm3_pos = 0

def odometry_cb(msg):
    global odom
    odom = msg

def mm0_state_cb(msg):
    global mm0_pos
    mm0_pos = msg.process_value

def mm1_state_cb(msg):
    global mm1_pos
    mm1_pos = msg.process_value

def mm2_state_cb(msg):
    global mm2_pos
    mm2_pos = msg.process_value

def mm3_state_cb(msg):
    global mm3_pos
    mm3_pos = msg.process_value

if __name__ == '__main__':

    rospy.init_node('mmc_uav_publish')

    pub_odom_joints = rospy.Publisher('odom_joints', Odometry, queue_size=1)
    rospy.Subscriber('odometry', Odometry, odometry_cb)
    rospy.Subscriber('movable_mass_0_position_controller/state', JointControllerState, mm0_state_cb)
    rospy.Subscriber('movable_mass_1_position_controller/state', JointControllerState, mm1_state_cb)
    rospy.Subscriber('movable_mass_2_position_controller/state', JointControllerState, mm2_state_cb)
    rospy.Subscriber('movable_mass_3_position_controller/state', JointControllerState, mm3_state_cb)
    
    odom_msg = Odometry()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()
        odom_msg.pose.pose = copy.deepcopy(odom.pose.pose)
        odom_msg.pose.covariance[0] = mm0_pos
        odom_msg.pose.covariance[1] = mm1_pos
        odom_msg.pose.covariance[2] = mm2_pos
        odom_msg.pose.covariance[3] = mm3_pos
        odom_msg.twist = copy.deepcopy(odom.twist)
        odom_msg.header.stamp = rospy.Time.now()
        pub_odom_joints.publish(odom_msg)    