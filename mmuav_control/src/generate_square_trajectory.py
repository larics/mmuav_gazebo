#!/usr/bin/env python3
import sys, os
import rospy
import copy
import math
import time

from geometry_msgs.msg import Pose, Point, Quaternion
from mav_path_trajectory.msg import WaypointArray
from mav_path_trajectory.srv import WaypointArraySrv

class SquareTrajectory():

    def __init__(self):
        self.waypointArray = WaypointArray()
        # Services for requesting trajectory interpolation
        rospy.wait_for_service("positionWaypointArray", timeout=30)
        self.waypointArrayService = rospy.ServiceProxy("positionWaypointArray", 
            WaypointArraySrv)

        self.tempPose = Pose()
        self.tempPose.orientation = copy.deepcopy(Quaternion(0,0,0,1))

        n = 9
        for i in range(0,n):
            self.waypointArray.waypoints.append(copy.deepcopy(self.tempPose))

        # Square
        #self.waypointArray.waypoints[0].position = copy.deepcopy(Point(0.0,0.0,2))
        #self.waypointArray.waypoints[1].position = copy.deepcopy(Point(1.0,0.0,2))
        #self.waypointArray.waypoints[2].position = copy.deepcopy(Point(1.0,1.0,2))
        #self.waypointArray.waypoints[3].position = copy.deepcopy(Point(0.0,1.0,2))
        #self.waypointArray.waypoints[4].position = copy.deepcopy(Point(0.0,0.0,2))

        # Up and down squarish trajectory
        self.waypointArray.waypoints[0].position = copy.deepcopy(Point(0.0,0.0,2))
        self.waypointArray.waypoints[1].position = copy.deepcopy(Point(0.5,0.0,2.5))
        self.waypointArray.waypoints[2].position = copy.deepcopy(Point(1.0,0.0,2))
        self.waypointArray.waypoints[3].position = copy.deepcopy(Point(1.0,0.5,2.5))
        self.waypointArray.waypoints[4].position = copy.deepcopy(Point(1.0,1.0,2))
        self.waypointArray.waypoints[5].position = copy.deepcopy(Point(0.5,1.0,2.5))
        self.waypointArray.waypoints[6].position = copy.deepcopy(Point(0.0,1.0,2))
        self.waypointArray.waypoints[7].position = copy.deepcopy(Point(0.0,0.5,2.5))
        self.waypointArray.waypoints[8].position = copy.deepcopy(Point(0.0,0.0,2))

        self.waypointArrayService(self.waypointArray)
        #print self.waypointArray
        print("Service called")


if __name__=="__main__":
    rospy.init_node("SquareTrajectory")
    SquareTrajectory()


