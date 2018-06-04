#!/usr/bin/env python

import rospy
import math
from math import pi, exp, sqrt, atan2
from cmath import acos, asin, cos, sin
from std_msgs.msg import Float64
from std_msgs.msg import String
from control_msgs.msg import JointControllerState
from geometry_msgs.msg import Point


class Kinematics():
    def Forward_kinematics(self, q2, q3, q4):
        self.x = 67.5*math.sin(q4) + 117.5
        self.y = -210*math.cos(q2 + q3) - 205*math.cos(q2) - 67.5*math.cos(q2 + q3)*math.cos(q4) - 250
        self.z = 210*math.sin(q2 + q3) + 205*math.sin(q2) + 67.5*math.sin(q2 + q3)*math.cos(q4)
        self.print_screen(1, 0, 0, 0, 0)

    def Inverse_kinematics(self, w1, w2, w3):
        q2 = [0, 0, 0, 0]
        q3 = [0, 0, 0, 0]
        q4 = [0, 0]

        q4[0] = asin((2*w1/135) - float(47)/27)
        q4[1] = pi - q4[0]

        q2[0] = atan2(w3, (-w2 - 250)) + acos((42025 + w3**2 + (w2 + 250)**2 - (210 + 67.5*cos(q4[0]))**2) / (2*205*sqrt(w3**2 + (w2 + 250)**2)))
        q2[1] = atan2(w3, (-w2 - 250)) - acos((42025 + w3**2 + (w2 + 250)**2 - (210 + 67.5*cos(q4[0]))**2) / (2*205*sqrt(w3**2 + (w2 + 250)**2)))
        q2[2] = atan2(w3, (-w2 - 250)) + acos((42025 + w3**2 + (w2 + 250)**2 - (210 + 67.5*cos(q4[1]))**2) / (2*205*sqrt(w3**2 + (w2 + 250)**2)))
        q2[3] = atan2(w3, (-w2 - 250)) - acos((42025 + w3**2 + (w2 + 250)**2 - (210 + 67.5*cos(q4[1]))**2) / (2*205*sqrt(w3**2 + (w2 + 250)**2)))

        q3[0] = pi + acos((42025 + (210 + 67.5*cos(q4[0]))**2 - w3**2 - (w2 + 250)**2) / (2*205*(210 + 67.5*cos(q4[0]))))
        q3[1] = pi - acos((42025 + (210 + 67.5*cos(q4[0]))**2 - w3**2 - (w2 + 250)**2) / (2*205*(210 + 67.5*cos(q4[0]))))
        q3[2] = pi + acos((42025 + (210 + 67.5*cos(q4[1]))**2 - w3**2 - (w2 + 250)**2) / (2*205*(210 + 67.5*cos(q4[1]))))
        q3[3] = pi - acos((42025 + (210 + 67.5*cos(q4[1]))**2 - w3**2 - (w2 + 250)**2) / (2*205*(210 + 67.5*cos(q4[1]))))

        m = 0

        if q4[0].imag == 0:
            for i in range(0, 1):
                if q2[i].imag == 0:
                    if q3[i].imag == 0:
                        self.joint_2_ref = q2[i].real
                        self.joint_3_ref = q3[i].real
                        self.joint_4_ref = q4[0].real
                        m = 1
                        break
        elif q4[1].imag == 0:
            for i in range(0, 1):
                if q2[i].imag == 0:
                    if q3[i].imag == 0:
                        self.joint_2_ref = q2[i].real
                        self.joint_3_ref = q3[i].real
                        self.joint_4_ref = q4[1].real
                        m = 1
                        break
        if m == 0:
           self.print_screen(0, 1, w1, w2, w3)




    def print_screen(self, k, i, x, y, z):
        if k:
            rospy.loginfo('Position of the arm: x = ' + str(round(self.x,2)) + '    y = ' + str(round(self.y,2)) + '    z = ' + str(round(self.z,2)))
        else:
            rospy.loginfo('The arm is folded')
        if i:
            rospy.loginfo('There is no solution for point x = ' + str(x) + '    y = ' + str(y) + '  z = ' + str(z))


    def filter(self, x_ref, x):
        x_next = x + (x_ref - x)*(self.dt/(self.dt + self.t))
        return x_next


    def callback_joint_1(self, data):
        self.q1 = data.process_value + 0.7

    def callback_joint_2(self, data):
        self.q2 = data.process_value - 2.9

    def callback_joint_3(self, data):
        self.q3 = data.process_value + 2.9

    def callback_joint_4(self, data):
        self.q4 = data.process_value + 1.5

    def callback_folding_arm(self, data):
        self.fold = data.data
        if self.fold:
            self.joint_1_ref = 0
            self.joint_2_ref = 0
            self.joint_3_ref = 0
            self.joint_4_ref = 0
        else:
            self.down = 1
            self.joint_1_ref = -0.7
            self.joint_2_ref = 2.9
            self.joint_3_ref = -2.9
            self.joint_4_ref = -1.5

    def callback_point(self, data):
        w1 = data.x
        w2 = data.y
        w3 = data.z
        if self.fold == 0:
            self.Inverse_kinematics(w1, w2, w3)



    def __init__(self):
        self.fold = 1
        self.down = 0

        self.dt = 0.005
        self.t = 1

        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q4 = 0


        self.joint_1_command = Float64
        self.joint_2_command = Float64
        self.joint_3_command = Float64
        self.joint_4_command = Float64

        self.joint_1_command = 0.0
        self.joint_2_command = 0.0
        self.joint_3_command = 0.0
        self.joint_4_command = 0.0

        self.joint_1_ref = 0.0
        self.joint_2_ref = 0.0
        self.joint_3_ref = 0.0
        self.joint_4_ref = 0.0

        self.joint_1_pub = rospy.Publisher('ruka/joint_1_position_controller/command', Float64, queue_size = 1)
        self.joint_2_pub = rospy.Publisher('ruka/joint_2_position_controller/command', Float64, queue_size = 1)
        self.joint_3_pub = rospy.Publisher('ruka/joint_3_position_controller/command', Float64, queue_size = 1)
        self.joint_4_pub = rospy.Publisher('ruka/joint_4_position_controller/command', Float64, queue_size = 1)

        self.joint_1_sub = rospy.Subscriber('ruka/joint_1_position_controller/state', JointControllerState, self.callback_joint_1, queue_size = 1)
        self.joint_2_sub = rospy.Subscriber('ruka/joint_2_position_controller/state', JointControllerState, self.callback_joint_2, queue_size = 1)
        self.joint_3_sub = rospy.Subscriber('ruka/joint_3_position_controller/state', JointControllerState, self.callback_joint_3, queue_size = 1)
        self.joint_4_sub = rospy.Subscriber('ruka/joint_4_position_controller/state', JointControllerState, self.callback_joint_4, queue_size = 1)
        self.folding_arm_sub = rospy.Subscriber('ruka/folding_arm', Float64, self.callback_folding_arm, queue_size = 1)
        self.set_point_sub = rospy.Subscriber('ruka/set_point', Point, self.callback_point, queue_size = 1)




    def run(self):
        while not rospy.is_shutdown():
            if self.fold:
                self.print_screen(0, 0, 0, 0, 0)
                self.joint_4_command =  self.filter(self.joint_4_ref, self.joint_4_command)
                self.joint_4_pub.publish(self.joint_4_command)
                self.joint_3_command = self.filter(self.joint_3_ref, self.joint_3_command)
                self.joint_3_pub.publish(self.joint_3_command)
                self.joint_2_command = self.filter(self.joint_2_ref, self.joint_2_command)
                self.joint_2_pub.publish(self.joint_2_command)
                if (round(self.joint_4_command,1) == 0) and (round(self.joint_3_command,1) == 0) and (round(self.joint_2_command,1) == 0):
                    self.joint_1_command = self.filter(self.joint_1_ref, self.joint_1_command)
                    self.joint_1_pub.publish(self.joint_1_command)
            else:
                if self.down:
                    self.joint_1_command = self.filter(self.joint_1_ref, self.joint_1_command)
                    self.joint_1_pub.publish(self.joint_1_command)
                if (round(self.joint_1_command,2) == -0.7):
                    self.down = 0
                    self.joint_2_command = self.filter(self.joint_2_ref, self.joint_2_command)
                    self.joint_2_pub.publish(self.joint_2_command)
                    self.joint_3_command = self.filter(self.joint_3_ref, self.joint_3_command)
                    self.joint_3_pub.publish(self.joint_3_command)
                    self.joint_4_command = self.filter(self.joint_4_ref, self.joint_4_command)
                    self.joint_4_pub.publish(self.joint_4_command)
                    self.Forward_kinematics(self.q2, self.q3, self.q4)
            rospy.sleep(0.01)





if __name__=='__main__':
    rospy.init_node('robotska_ruka_control', anonymous = True)

    try:
        kinem = Kinematics()
        kinem.run()
    except rospy.ROSInterruptException:
        pass
