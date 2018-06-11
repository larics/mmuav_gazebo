#!/usr/bin/python
# -*- coding: utf-8 -*-

#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int8
from math import sin, cos, pi

class TestTrajectory:
    """
    Trajectory used for geometric controller.
    """

    def __init__(self):

        # Reference publishers
        self.pos_ref_pub = rospy.Publisher(
            "uav/x_desired",
            Vector3,
            queue_size=10)
        self.pos_ref_msg = Vector3()

        self.heading_ref_pub = rospy.Publisher(
            "uav/b1_desired",
            Vector3,
            queue_size=10)
        self.ang_ref_msg = Vector3()
        
        self.control_mode_pub = rospy.Publisher(
            "uav/control_mode",
            Int8,
            queue_size=10)
        self.mode_ref_msg = Int8()

        # Crontroller rate
        self.controller_rate = 5
        self.rate = rospy.Rate(self.controller_rate)
        
        self.pos_ref_msg = Vector3()
        self.heading_ref_msg = Vector3()
        
    def run(self):

        end_time = 10
        t_list = np.linspace(0, end_time, 60)
        
        # Position control
        self.mode_ref_msg.data = 1;
        self.control_mode_pub.publish(self.mode_ref_msg)
        
        for t in t_list:
            self.rate.sleep();
            print(t, "/", end_time)
            self.pos_ref_msg.x = 0.4 * t
            self.pos_ref_msg.y = 0.5 * sin(pi * t)
            self.pos_ref_msg.z = 0.6 * cos(pi * t) + 1
            
            self.heading_ref_msg.x = cos(pi * t)
            self.heading_ref_msg.y = sin(pi * t)
            self.heading_ref_msg.z = 0
            
            self.heading_ref_pub.publish(self.heading_ref_msg)     
            #self.pos_ref_pub.publish(self.pos_ref_msg)     
           
                
if __name__ == '__main__':
    rospy.init_node('test_flight', anonymous=True)
    try:
        tf = TestTrajectory()
        tf.run()
    except rospy.ROSInterruptException:
        pass
