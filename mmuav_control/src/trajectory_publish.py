#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from rospkg import RosPack
import copy


class reference_gen():
    def pose_callback(self, data):
        self.pose=data

    def __init__(self):
        self.pub = rospy.Publisher('multi_dof_trajectory', MultiDOFJointTrajectory, queue_size=1)
        kvat=Quaternion(0,0,0,1)
        self.acc=Twist()
        self.acc.angular=Vector3(0,0,0)
        self.vel=Twist()
        self.vel.angular=Vector3(0,0,0)
        self.pos=Transform()
        self.pos.rotation=kvat
        self.traj_point=MultiDOFJointTrajectoryPoint()
        self.traj=MultiDOFJointTrajectory()

        self.pose=PoseStamped()
        self.positions=[]
        self.trajectory=[]
        self.time=[]


        self.traj_point.transforms=[]
        self.traj_point.accelerations=[]
        self.traj_point.velocities=[]
        self.traj.joint_names=['UAV']

        h=Header()
        h.stamp=rospy.Time.now()
        self.traj.header=h
        self.traj.points=[]
        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate=rospy.Rate(self.rate)

    def pubtrajectory(self,file_loc):
        with open(file_loc,'r') as fajl:
            for line in fajl:
                points=list(map(float, line.split()))

                self.traj_point=MultiDOFJointTrajectoryPoint()
                self.traj_point.transforms=[]
                self.traj_point.velocities=[]
                self.traj_point.accelerations=[]
                self.pos.translation=Vector3(points[0],points[1],points[2])
                self.vel.linear=Vector3(points[3],points[4],points[5])
                self.acc.linear=Vector3(points[6],points[7],points[8])
                tfs=rospy.Duration.from_sec(points[9])

                self.traj_point.transforms.append(self.pos)
                self.traj_point.velocities.append(self.vel)
                self.traj_point.accelerations.append(self.acc)
                self.traj_point.time_from_start=tfs
                B=copy.deepcopy(self.traj_point)
                self.traj.points.append(B)
        self.pub.publish(self.traj)


if __name__== '__main__':
    rospy.init_node('trajectory_publish')


    try:
        print("generating trajectory")
        rospy.sleep(5)
        generate=reference_gen()
        file_name = RosPack().get_path('mmuav_control') + '/resources/4seg.txt'
        generate.pubtrajectory(file_name)

    except rospy.ROSInterruptException:
        pass
