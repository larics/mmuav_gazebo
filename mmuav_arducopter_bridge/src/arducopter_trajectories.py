#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from mav_path_trajectory.msg import WaypointArray
from mav_path_trajectory.srv import WaypointArraySrv, SendHelixParams, SendHelixParamsRequest
from mav_path_trajectory.srv import ChangePlanningConfig, ChangePlanningConfigRequest
from std_msgs.msg import String, Float64


class ArducopterTrajectoryExamples():

    def __init__(self):
        #self.waypointArray = WaypointArray()
        # Services for requesting trajectory interpolation
        rospy.wait_for_service("positionWaypointArray", timeout=30)
        self.waypointArrayService = rospy.ServiceProxy("positionWaypointArray", 
            WaypointArraySrv)
        self.sendHelixParamsService = rospy.ServiceProxy("helix_params",
            SendHelixParams)
        self.changePlanningConfigService = rospy.ServiceProxy("change_planning_config", 
            ChangePlanningConfig)

        rospy.Subscriber("arducopter_trajectory_type", String, self.typeCallback, 
            queue_size=1)

    def run(self):
        rospy.spin()

    def typeCallback(self, msg):
        print(msg)
        split_msg = msg.data.split(" ")
        if split_msg[0] == "square":
            if len(split_msg) < 2:
                print("Please specify side length of the square.")
            else:
                self.squareTrajectory(float(split_msg[1]))

        elif split_msg[0] == "helix":
            if split_msg[1] == "big":
                self.helixTrajectoryBig()
            else:
                self.helixTrajectorySmall()

        else:
            print("No such trajectory type.")

    def squareTrajectory(self, a):
        print("Generating square trajectory with side length ", a, "m")

        config = ChangePlanningConfigRequest()
        config.max_speed = 0.8
        config.max_acceleration = 0.3
        config.sampling_frequency = 100
        self.changePlanningConfigService(config)

        waypointArray = WaypointArray()
        n = 9
        tempPose = Pose()
        tempPose.orientation = copy.deepcopy(Quaternion(0,0,0,1))
        for i in range(0,n):
            waypointArray.waypoints.append(copy.deepcopy(tempPose))

        waypointArray.waypoints[0].position = copy.deepcopy(Point(-a/2,-a/2,1))
        waypointArray.waypoints[1].position = copy.deepcopy(Point(0,-a/2,1))
        waypointArray.waypoints[2].position = copy.deepcopy(Point(a/2,-a/2,1))
        waypointArray.waypoints[3].position = copy.deepcopy(Point(a/2,0,1))
        waypointArray.waypoints[4].position = copy.deepcopy(Point(a/2,a/2,1))
        waypointArray.waypoints[5].position = copy.deepcopy(Point(0,a/2,1))
        waypointArray.waypoints[6].position = copy.deepcopy(Point(-a/2,a/2,1))
        waypointArray.waypoints[7].position = copy.deepcopy(Point(-a/2,0,1))
        waypointArray.waypoints[8].position = copy.deepcopy(Point(-a/2,-a/2,1))
        self.waypointArrayService(waypointArray)


    def helixTrajectorySmall(self):
        print("Generating helix trajectory with radius 0.5m.")
        temp = SendHelixParamsRequest()
        temp.r = 0.5
        temp.angleStep = 0.7
        temp.x0 = 0.0
        temp.y0 = 0.0
        temp.z0 = 0.5
        temp.zf = 1.5
        temp.deltaZ = 0.05
        temp.epsilon = 0.0
        self.sendHelixParamsService(temp)

    def helixTrajectoryBig(self):
        print("Generating helix trajectory with radius 1.0m.")
        temp = SendHelixParamsRequest()
        temp.r = 1.0
        temp.angleStep = 0.7
        temp.x0 = 0.0
        temp.y0 = 0.0
        temp.z0 = 0.5
        temp.zf = 1.5
        temp.deltaZ = 0.05
        temp.epsilon = 0.0
        self.sendHelixParamsService(temp)


if __name__=="__main__":
    rospy.init_node("ArducopterTrajectoryExamples")
    traj = ArducopterTrajectoryExamples()
    traj.run()