#!/usr/bin/env python

import numpy as np
import math as m
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import time as t


def spin_transform(theta,d,alpha,a):
    # Calculate transfer matrix w.r.t. DH table row |theta | d | alpha | a |
    T = np.matrix(((m.cos(theta), -m.cos(alpha)*m.sin(theta),m.sin(alpha)*m.sin(theta), a*m.cos(theta)),(m.sin(theta), m.cos(alpha)*m.cos(theta), -m.sin(alpha)*m.cos(theta), a*m.sin(theta)),(0, m.sin(alpha), m.cos(alpha), d),(0, 0,0,1)))
    return T


def base2tool(t_01,t_12):
    # Calculate transformation matrix T01*T12 = T02
    t = np.dot(t_01, t_12)
    return t


def get_theta(x, y, theta1, theta2):
    # Calculate angle q1 and q2 with inverse kinematics of planar manipulator
    l1 = 0.28
    l2 = 0.30

    if (m.sqrt(pow(x,2)+pow(y,2))>(l1+l2)) | (np.abs((pow(x,2)+pow(y,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2))>1.0):
        print 'Out of workspace'
        return [theta1, theta2]

    q2_1 = np.real(m.acos((pow(x,2)+pow(y,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2)))
    q2_2 = np.real(-m.acos((pow(x,2)+pow(y,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2)))

    k1 = (l1+l2*m.cos(q2_1))*y - l2*m.sin(q2_1)*x
    k2 = (l1+l2*m.cos(q2_1))*x + l2*m.sin(q2_1)*y
    q1_1 = m.atan2(k1,k2)

    k1 = (l1+l2*m.cos(q2_2))*y - l2*m.sin(q2_2)*x
    k2 = (l1+l2*m.cos(q2_2))*x + l2*m.sin(q2_2)*y
    q1_2 = m.atan2(k1,k2)

    if (np.abs(q1_1) > 40*m.pi/180) & (np.abs(q1_2) > 40*m.pi/180):
        print 'Out of workspace'
        return [theta1, theta2]
    elif (np.abs(q1_1) > 40*m.pi/180):
        return [q1_2, q2_2]
    elif (np.abs(q1_2) > 40*m.pi/180):
        return [q1_1, q2_1]
    else:

        # Closest function

        if m.sqrt(pow(q1_1-theta1,2)+pow(q2_1-theta2,2))<m.sqrt(pow(q1_2-theta1,2)+pow(q2_2-theta2,2)):
            return [q1_1, q2_1]
        else:
            return [q1_2, q2_2]


class CameraManipulator:


    def pointSet_callback(self,data):
        self.ptSet = data

    def __init__(self):

        self.ptSet = Point()

        self.pub1 = rospy.Publisher('/uav/joint1_controller/command', Float64, queue_size=1)
        self.pub2 = rospy.Publisher('/uav/joint2_controller/command', Float64, queue_size=1)

        rospy.Subscriber("uav/joint_position", Point, self.pointSet_callback)

        self.x = 0.58
        self.y = 0.0
        self.z = -0.14
        self.theta1 = 0.0
        self.theta2 = 0.0


    def run(self):

        while not rospy.is_shutdown():

            t.sleep(0.1)

            newx = self.ptSet.x
            newy = self.ptSet.y
            newz = self.ptSet.z

            if (newx != self.x) | (newy != self.y):

                self.x = newx
                self.y = newy

                thetaNew = get_theta(self.x,self.y,self.theta1, self.theta2)

                self.theta1 = thetaNew[0]
                self.theta2 = thetaNew[1]

            # Print for reference: transformation matrix, angles, coordiantes
            # print (base2tool(spin_transform(self.theta1,-0.09,0.0,0.28),spin_transform(self.theta2,-0.05,0.0,0.3)))
            # print (self.theta1*180/m.pi), (self.theta2*180/m.pi)
            # print self.x, self.y

            self.pub1.publish(self.theta1)
            self.pub2.publish(self.theta2)



if __name__ == '__main__':
    rospy.init_node('CameraManipulator')
    jointctl = CameraManipulator()
    jointctl.run()
