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
            "morus/x_desired",
            Vector3,
            queue_size=10)
        self.pos_ref_msg = Vector3()
        
        self.v_ref_pub = rospy.Publisher(
            "morus/v_desired",
            Vector3,
            queue_size=10)
        self.v_ref_msg = Vector3()
        
        self.a_ref_pub = rospy.Publisher(
            "morus/a_desired",
            Vector3,
            queue_size=10)
        self.a_ref_msg = Vector3()
        
        self.heading_ref_pub = rospy.Publisher(
            "morus/b1_desired",
            Vector3,
            queue_size=10)
        self.ang_ref_msg = Vector3()
        
        self.euler_ref_pub = rospy.Publisher(
            "mmcuav/euler_desired",
            Vector3, 
            queue_size=10)
        self.euler_msg = Vector3()
        
        self.control_mode_pub = rospy.Publisher(
            "mmcuav/control_mode",
            Int8,
            queue_size=10)
        self.mode_ref_msg = Int8()

        # Crontroller rate
        self.controller_rate = 50
        self.rate = rospy.Rate(self.controller_rate)
        
        self.pos_ref_msg = Vector3()
        self.heading_ref_msg = Vector3()
        
    def run(self):

        end_time = 10
        scale = 3.25
        t_list = np.linspace(0, end_time, 500 * scale)
        ang_list = np.linspace(0, pi, 500 * scale)
        easy = np.linspace(3, 1, 500)
        easy = np.append(easy, np.ones(int(500*scale) - 500))
        
        # Position control
        #self.mode_ref_msg.data = 1;
        #self.control_mode_pub.publish(self.mode_ref_msg)
        
        for t, ang, _easy in zip(t_list, ang_list, easy):
            self.rate.sleep();
            print(t, "/", end_time)
            self.pos_ref_msg.x = 0.4 * t / _easy
            self.pos_ref_msg.y = 0.5 * sin(pi * t / _easy) 
            self.pos_ref_msg.z = 0.6 * cos(pi * t / _easy) + 2
                
            self.v_ref_msg.x = 0.4 / (scale * _easy)
            self.v_ref_msg.y = 0.5 * pi * cos(pi * t / _easy) / ( scale )
            self.v_ref_msg.z = - 0.6 * pi * sin(pi * t / _easy) / ( scale )
            
            self.a_ref_msg.x = 0
            self.a_ref_msg.y = - 0.5 * pi * pi * sin(pi * t / _easy) / (scale * scale )
            self.a_ref_msg.y = - 0.6 * pi * pi * cos(pi * t / _easy) / (scale * scale )
            
            self.heading_ref_msg.x = cos(pi * t / (5 * _easy))
            self.heading_ref_msg.y = sin(pi * t / (5 * _easy))
            self.heading_ref_msg.z = 0
            
            self.euler_msg.z =  ang
                
            self.heading_ref_pub.publish(self.heading_ref_msg)     
            self.pos_ref_pub.publish(self.pos_ref_msg)
            self.v_ref_pub.publish(self.v_ref_msg)            
            self.a_ref_pub.publish(self.a_ref_msg)
            
            #self.euler_ref_pub.publish(self.euler_msg)    
            
                
if __name__ == '__main__':
    rospy.init_node('test_flight', anonymous=True)
    try:
        tf = TestTrajectory()
        tf.run()
    except rospy.ROSInterruptException:
        pass
