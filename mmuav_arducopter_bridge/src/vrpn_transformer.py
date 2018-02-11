#!/usr/bin/env python

__author__ = 'thaus'

import rospy
import copy
import math
#from pid import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped, Quaternion
from std_msgs.msg import Float32
import tf

class VrpnTransform:

    def __init__(self):
        '''
        Initialization of the class.
        '''
        self.medfilt_len = rospy.get_param("~median_filter_length", 5);

        topic_name = rospy.get_param('~topic_name', '/vrpn_client_node/uav/pose')
        rospy.Subscriber(topic_name, PoseStamped, self.pose_cb)
        
        self.pose_pub = rospy.Publisher('optitrack/pose', PoseStamped, queue_size=1)
        self.vel_pub = rospy.Publisher('optitrack/velocity', TwistStamped, queue_size=1)
        self.first_pass = True
        self.t_old = rospy.Time.now()

        self.tranform_matrix = []

        self.pos = Vector3()
        self.pos_old = Vector3()
        self.vel = Vector3()
        self.vel_old = Vector3()
        self.quat = Quaternion()
        self.euler = Quaternion()

        self.filt_const = 1.0

        self.vel_list_x = [0]*self.medfilt_len
        self.vel_list_y = [0]*self.medfilt_len
        self.vel_list_z = [0]*self.medfilt_len
        

    def run(self):

        rospy.spin()
       

    def pose_cb(self, msg):
        #[0 0 -1]
        #[-1 0 0]
        #[0  1 0]
        self.pos.x = -msg.pose.position.z
        self.pos.y = -msg.pose.position.x
        self.pos.z = msg.pose.position.y

        self.quat.x = msg.pose.orientation.x
        self.quat.y = msg.pose.orientation.y
        self.quat.z = msg.pose.orientation.z
        self.quat.w = msg.pose.orientation.w

        if self.first_pass == True:
            self.first_pass = False
            self.pos_old = copy.deepcopy(self.pos)
            self.t_old = msg.header.stamp
        else:
            # compute velocity
            dt = msg.header.stamp.to_sec() - self.t_old.to_sec()
            self.t_old = msg.header.stamp
            dt = 0.01

            if dt < 0.001:
                dt = 0.001

            self.vel.x = self.filt_const * (self.pos.x - self.pos_old.x ) / dt #+ #(1.0 - self.filt_const) * self.vel_old.x
            self.vel.y = self.filt_const * (self.pos.y - self.pos_old.y ) / dt #+ #(1.0 - self.filt_const) * self.vel_old.y
            self.vel.z = self.filt_const * (self.pos.z - self.pos_old.z ) / dt #+ #(1.0 - self.filt_const) * self.vel_old.z

            # Median filter
            for i in range(self.medfilt_len-1):
                self.vel_list_x[i] = self.vel_list_x[i+1]
                self.vel_list_y[i] = self.vel_list_y[i+1]
                self.vel_list_z[i] = self.vel_list_z[i+1]
            self.vel_list_x[self.medfilt_len-1] = self.vel.x
            self.vel_list_y[self.medfilt_len-1] = self.vel.y
            self.vel_list_z[self.medfilt_len-1] = self.vel.z
            vel_list_sorted_x = copy.deepcopy(self.vel_list_x)
            vel_list_sorted_y = copy.deepcopy(self.vel_list_y)
            vel_list_sorted_z = copy.deepcopy(self.vel_list_z)
            vel_list_sorted_x.sort()
            vel_list_sorted_y.sort()
            vel_list_sorted_z.sort()
            self.vel.x = vel_list_sorted_x[int(math.floor(self.medfilt_len/2))]
            self.vel.y = vel_list_sorted_y[int(math.floor(self.medfilt_len/2))]
            self.vel.z = vel_list_sorted_z[int(math.floor(self.medfilt_len/2))]

            # Legacy, save old values
            self.vel_old = copy.deepcopy(self.vel)
            self.pos_old = copy.deepcopy(self.pos)
            
            # quaternion to euler
            qx = self.quat.x
            qy = self.quat.y
            qz = self.quat.z
            qw = self.quat.w
            self.euler.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
            self.euler.z = -math.asin(2*(qx * qz - qw * qy))
            self.euler.y = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.pose.position = copy.deepcopy(self.pos)
            # Rotate quaternion received from message for [-0.5 0.5 0.5 -0.5] -> matrix above
            #quaternion_multiply(q_rot, q_orig)
            final_orientation = tf.transformations.quaternion_multiply(
                [-0.5, 0.5, 0.5, -0.5], [msg.pose.orientation.x,
                msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            pose_msg.pose.orientation.x = -msg.pose.orientation.z
            pose_msg.pose.orientation.y = -msg.pose.orientation.x
            pose_msg.pose.orientation.z = msg.pose.orientation.y
            pose_msg.pose.orientation.w = msg.pose.orientation.w
            #pose_msg.pose.orientation = copy.deepcopy(msg.pose.orientation)

            vel_msg = TwistStamped()
            vel_msg.header.stamp = msg.header.stamp
            vel_msg.twist.linear = copy.deepcopy(self.vel)

            self.pose_pub.publish(pose_msg)
            self.vel_pub.publish(vel_msg)


        

   
if __name__ == '__main__':

    rospy.init_node('vrpn_transformer')
    vrpn = VrpnTransform()
    vrpn.run()
