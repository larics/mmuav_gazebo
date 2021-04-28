#!/usr/bin/env python

import rospy
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import OverrideRCIn
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray, Float64


class RCIn2OverrideRCIn():
    def __init__(self):

        self.overrideRCIn_pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1)
        self.received_rc_msg = OverrideRCIn()

        self.euler_ref_pub = rospy.Publisher('euler_ref', Vector3, queue_size=1)
        self.euler_ref_msg = Vector3()

        self.roll_command = 0.0
        self.pitch_command = 0.0
        self.yaw_command = 0.0

        self.vpc_roll = 0.0
        self.vpc_pitch = 0.0
        self.motor_velocity = 0.0

        self.rate = rospy.get_param("~rate", 50)

        rospy.Subscriber('/jeti/mavros/rc/in', RCIn, self.rcin_callback)
        rospy.Subscriber('attitude_command', Float64MultiArray, 
            self.attitude_command_cb, queue_size=1)
        rospy.Subscriber('mot_vel_ref', Float64, self.mot_vel_ref_cb)

    def run(self):
        
        while not rospy.is_shutdown():
            rospy.sleep(1.0/float(self.rate))
            mode = self.check_mode()

            if mode == 0:
                # attitude only, overriding message
                rc_channels = OverrideRCIn()
                for i in range(8):
                    rc_channels.channels[i] = self.received_rc_msg.channels[i]

                #rc_channels.channels[1] = 1500 + 2.5*self.vpc_roll
                rc_channels.channels[0] = 1500 + 2.5*self.vpc_pitch
                #rc_channels.channels[3] = 1500 + 0.5*(rc_channels.channels[3] - 1500.0)
                #pitch_int = int(500.0*self.pitch_command)
                #if pitch_int < -500:
                #    pitch_int = -500
                #elif pitch_int > 500:
                #    pitch_int = 500
                #rc_channels.channels[0] = 1500 + pitch_int
                self.overrideRCIn_pub.publish(rc_channels)

    def check_mode(self):
        return 0

    def rcin_callback(self,msg):
        #self.overrideRCIn_msg.channels.clear()
        #for i in range(8):
        if len(msg.channels) >=8:
            self.received_rc_msg = msg

        self.euler_ref_msg.x = (self.received_rc_msg.channels[1] - 1500)*0.15/500.0
        self.euler_ref_msg.y = (self.received_rc_msg.channels[0] - 1500)*0.15/500.0
        self.euler_ref_pub.publish(self.euler_ref_msg)

    def attitude_command_cb(self,msg):
        if len(msg.data) < 5:
            print("Not enough data, should be 5. Length: ", len(msg.data))
        else:
            self.roll_command = msg.data[0]
            self.pitch_command = msg.data[1]
            self.yaw_command = msg.data[2]
            self.vpc_roll = msg.data[3]
            self.vpc_pitch = msg.data[4]

    def mot_vel_ref_cb(self,msg):
        self.motor_velocity = 0.0

if __name__ == '__main__':

    rospy.init_node('RCIn_to_OverrideRCIn')
    conversion = RCIn2OverrideRCIn()
    conversion.run()
