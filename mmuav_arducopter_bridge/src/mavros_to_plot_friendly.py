#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, \
    TwistStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from dynamic_reconfigure.server import Server
import tf
from rospkg import RosPack
import yaml
from mavros_msgs.msg import RCIn

class PlotFriendly:

    def __init__(self):
        '''
        Initialization of the class.
        '''
        # Params
        self.rate = rospy.get_param('rate', 200)
        # 0 for simulation, 1 for optitrack
        self.plot_imu_euler = Vector3()
        self.plot_imu_euler_ref = Vector3()

        self.plot_imu_euler_pub = rospy.Publisher('/plot_friendly/imu/euler', 
            Vector3, queue_size=1)
        self.plot_imu_euler_ref_pub = rospy.Publisher('/plot_friendly/euler_ref', 
            Vector3, queue_size=1)

        rospy.Subscriber('/jeti/mavros/imu/data', Imu, self.imu_cb)
        rospy.Subscriber('/jeti/mavros/rc/in', RCIn, self.rcin_cb)


    def run(self):
        while not rospy.is_shutdown():

            rospy.sleep(1.0/float(self.rate))

            self.plot_imu_euler_pub.publish(self.plot_imu_euler)
            self.plot_imu_euler_ref_pub.publish(self.plot_imu_euler_ref)

    def imu_cb(self, msg):
        plot_imu_euler = tf.transformations.euler_from_quaternion((
            msg.orientation.x, msg.orientation.y, msg.orientation.z, 
            msg.orientation.w))
        self.plot_imu_euler.x = plot_imu_euler[0]
        self.plot_imu_euler.y = -plot_imu_euler[1]
        self.plot_imu_euler.z = plot_imu_euler[2]

    def rcin_cb(self, msg):
        self.plot_imu_euler_ref.x = (msg.channels[1]-1500)*0.3/500
        self.plot_imu_euler_ref.y = (msg.channels[0]-1500)*0.3/500
        self.plot_imu_euler_ref.z = (msg.channels[3]-1500)*0.3/500

if __name__ == '__main__':

    rospy.init_node('mavros_to_plot_friendly')
    plot = PlotFriendly()
    plot.run()

