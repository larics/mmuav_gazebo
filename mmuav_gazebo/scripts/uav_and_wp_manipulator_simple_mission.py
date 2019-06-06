#!/usr/bin/env python

import copy, time

# Ros imports
import rospy
from larics_motion_planning.srv import MultiDofTrajectory, \
    MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist

class RequestTrajectory():

    def __init__(self):
        # First set up service
        self.request_trajectory_service = rospy.ServiceProxy(
            "multi_dof_trajectory", MultiDofTrajectory)

        # We will publish to joint_trajectory topic for uav with manipulator
        self.trajectory_pub = rospy.Publisher('joint_trajectory', 
            JointTrajectory, queue_size=1)
        time.sleep(0.5)

        # Example of a simple square trajectory for quadcopter. All vectors
        # must be of same length
        points = []
        points.append([[ 0.00,  4.61], [ 0.00, -4.30], [1.000, 1.000], [ 0.00,  1.57], [ 0.00,  0.00], [ 0.00,  0.00], [ 0.00,  0.00], [ 0.00,  0.00], [ 0.00,  0.00]])
        points.append([[ 4.61,  6.10], [-4.30, -9.58], [1.000, 1.048], [ 1.57,  0.00], [ 0.00,  0.71], [ 0.00,  0.99], [ 0.00, -0.06], [ 0.00,  0.00], [ 0.00,  0.00]])
        points.append([[ 6.10,  6.16], [-9.58,-11.92], [1.048, 1.044], [ 0.00,  2.74], [ 0.71,  0.71], [ 0.99,  0.99], [-0.06, -0.06], [ 0.00,  0.00], [ 0.00,  0.00]])
        points.append([[ 6.16,  6.07],[-11.92,-12.91], [1.044, 1.044], [ 2.74,  2.51], [ 0.71,  0.71], [ 0.99,  0.99], [-0.06, -0.06], [ 0.00,  0.00], [ 0.00,  0.00]])
        points.append([[ 6.07,  3.25], [-12.91,-9.55], [1.044, 1.048], [ 2.51,  3.13], [ 0.71,  0.71], [ 0.99,  0.99], [-0.06, -0.06], [ 0.00,  0.00], [ 0.00,  0.00]])
        points.append([[ 3.25,  0.00], [-9.55,  0.00], [1.048, 1.000], [ 3.13,  0.00], [ 0.71,  0.00], [ 0.99,  0.00], [-0.06,  0.00], [ 0.00,  0.00], [ 0.00,  0.00]])

        self.ExecuteSegment(points[0], False, True)
        print "Press enter to continue"
        input()
        for i in range(1,5):
            self.ExecuteSegment(points[i], True, True)
            print "Press enter to continue"
            input()

        self.ExecuteSegment(points[5], False, True)


    def JointTrajectory2MultiDofTrajectory(self, joint_trajectory):
        multi_dof_trajectory = MultiDOFJointTrajectory()

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[0]
            temp_transform.translation.y = joint_trajectory.points[i].positions[1]
            temp_transform.translation.z = joint_trajectory.points[i].positions[2]
            temp_transform.rotation.w = 1.0

            temp_vel = Twist()
            temp_vel.linear.x = joint_trajectory.points[i].velocities[0]
            temp_vel.linear.y = joint_trajectory.points[i].velocities[1]
            temp_vel.linear.z = joint_trajectory.points[i].velocities[2]

            temp_acc = Twist()
            temp_acc.linear.x = joint_trajectory.points[i].accelerations[0]
            temp_acc.linear.y = joint_trajectory.points[i].accelerations[1]
            temp_acc.linear.z = joint_trajectory.points[i].accelerations[2]

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory

    def ExecuteSegment(self, points, plan_path, plan_trajectory):
        # Create a service request which will be filled with waypoints
        request = MultiDofTrajectoryRequest()

        # Add waypoints in request
        for i in range(0, len(points[0])):
            waypoint = JointTrajectoryPoint()
            # Positions are defined above
            for j in range(0, len(points)):
                waypoint.positions.append(points[j][i])
            # Also add constraints for velocity and acceleration. These
            # constraints are added only on the first waypoint since the
            # TOPP-RA reads them only from there.
            #if i==0:
            #    waypoint.velocities = [2, 2, 2, 0.5, 0.5, 0.5, 0.5, 0.5]
            #    waypoint.accelerations = [1.25, 1.25, 1.25, 0.5, 0.5, 0.5, 0.5, 0.5]

            # Append all waypoints in request
            request.waypoints.points.append(copy.deepcopy(waypoint))

        # Set up joint names. This step is not necessary
        request.waypoints.joint_names = ["x", "y", "z", "q1", "q2", "q3", "q4", "q5"]
        # Set up flags
        request.publish_path = False
        request.publish_trajectory = False
        request.plan_path = plan_path
        request.plan_trajectory = plan_trajectory
        #print request        

        response = self.request_trajectory_service(request)

        # Response will have trajectory and bool variable success. If for some
        # reason the trajectory was not able to be planned or the configuration
        # was incomplete or wrong it will return False.

        #print response
        joint_trajectory = response.trajectory
        #multi_dof_trajectory = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)
        self.trajectory_pub.publish(joint_trajectory)

if __name__ == "__main__":
    rospy.init_node("topp_trajectory_call_example")
    RequestTrajectory()