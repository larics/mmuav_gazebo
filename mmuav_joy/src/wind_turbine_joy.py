#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
import tf
import math

import time

def saturation(value, min, max):
    if (value < min):
        return min  
    elif (value > max):
        return max
    else:
        return value

class Commander():
    #carrot following controller for z axis requires knowing the position of the UAV 

    def __init__(self):
        # Create a publisher for roll pitch yaw cmnds
        self.uav_current_pose = PoseStamped()
        self.uav_pose_ref = Pose()
        self.uav_pose_ref.position.z = 1.0
        self.uav_pose_ref.orientation.w = 1.0
        self.yaw = 0
        self.mode = Int32()
        self.mode.data = 0
        self.current_joy = Joy()
        self.pose_pub = rospy.Publisher("pose_ref", Pose, queue_size=1)
        self.mode_pub = rospy.Publisher("mode", Int32, queue_size=1)

        # Initialize message variables.

        # Create a subscriber for color msg
        rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("joy", Joy, self.joy_callback)

        time.sleep(0.2)
        

    # Subscribe to teleop msgs

    def run(self):
        rate = rospy.Rate(20) # 20hz
        # Main while loop.
        while not rospy.is_shutdown():
            #self.uav_pose_ref.position.x = self.uav_pose_ref.position.x + self.current_joy.axes[3]*0.005
            self.uav_pose_ref.position.y = -self.current_joy.axes[2]*0.25
            self.uav_pose_ref.position.z = self.uav_pose_ref.position.z + self.current_joy.axes[1]*0.05
            self.yaw = self.yaw + self.current_joy.axes[0]*0.05
            self.uav_pose_ref.orientation.z = math.sin(self.yaw)
            self.uav_pose_ref.orientation.w = math.cos(self.yaw)
            if (self.current_joy.buttons[4] == 1 or self.current_joy.buttons[5] == 1):
                self.mode.data = 1
                self.uav_pose_ref.position.x = self.uav_pose_ref.position.x + self.current_joy.axes[3]*0.05
            else:
                self.mode.data = 0
                self.uav_pose_ref.position.x = self.current_joy.axes[3]*0.25
            self.mode_pub.publish(self.mode)
            self.pose_pub.publish(self.uav_pose_ref)
            rate.sleep()
    
    def pose_callback(self,data):
        self.uav_current_pose = data

    def joy_callback(self,data):
        self.current_joy = data

        if data.buttons[0] == 1:
            self.uav_pose_ref = self.uav_current_pose.pose
            euler = tf.transformations.euler_from_quaternion((
                self.uav_current_pose.pose.orientation.x, \
                self.uav_current_pose.pose.orientation.y, \
                self.uav_current_pose.pose.orientation.z, \
                self.uav_current_pose.pose.orientation.w), 'szyx')
            self.yaw = euler[2] 

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('wind_turbine_joy')
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        ne = Commander()
        ne.run()
    except rospy.ROSInterruptException:
        pass