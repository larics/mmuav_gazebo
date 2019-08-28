#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, JointState
import time, math

def saturation(value, min, max):
    if (value < min):
        return min  
    elif (value > max):
        return max
    else:
        return value

class Commander():
    #carrot following controller for z axis requires knowing the position of the UAV 

    def __init__(self):
        self.uav_pose_ref = Pose()
        self.uav_first_ref_received = False
        self.manipulator_position_ref = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.manipulator_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        self.first_manipulator_reference_received = False
        self.joy = Joy()
        self.joy.buttons = [0]*12
        self.joy.axes = [0]*8
        self.uav_pose_ref_pub = rospy.Publisher('pose_ref', Pose, queue_size=1)
        self.manipulator_pub_list = []
        self.mode = 0
        self.mode_sw = 0
        self.mode_sw_prev = 0
        #joint1_position_controller/command
        for i in range(1,6):
            topic = "joint" + str(i) + "_position_controller/command"
            self.manipulator_pub_list.append(
                rospy.Publisher(topic, Float64, queue_size=1))

        self.joint_state_pub = rospy.Publisher("manipulator_joint_states", JointState, queue_size=1)
        # Create a subscriber for color msg
        time.sleep(1)
        rospy.Subscriber("pose", PoseStamped, self.uav_pose_callback)
        rospy.Subscriber("joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("joy", Joy, self.joy_callback)


    def run(self):
        rate = rospy.Rate(100) # 1hz
        # Main while loop.
        while not rospy.is_shutdown():
            self.compute_references()
            self.publish_all()
            rate.sleep()

    def uav_pose_callback(self, msg):
        if self.joy.buttons[6] == 1:
            self.uav_pose_ref = msg.pose
            self.uav_first_ref_received = True
    
    def joint_states_callback(self, msg):
        if self.joy.buttons[6] == 1:
            for i in range(len(msg.name)):
                for j in range(len(self.manipulator_joint_names)):
                    if (msg.name[i] == self.manipulator_joint_names[j]):
                        self.manipulator_position_ref[j] = msg.position[i]
                        self.first_manipulator_reference_received = True

    def joy_callback(self, msg):
        self.joy = msg
        self.mode_sw = msg.buttons[0]
        if ((self.mode_sw == 1) and (self.mode_sw_prev == 0)):
            self.mode = self.mode + 1
            if self.mode > 5:
                self.mode = 0
            print "Mode switch. New mode is: ", self.mode
        self.mode_sw_prev = self.mode_sw

    def publish_all(self):
        if self.uav_first_ref_received == True and self.mode == 0:
            self.uav_pose_ref_pub.publish(self.uav_pose_ref)

        if self.first_manipulator_reference_received == True and self.mode != 0:
            for i in range(0,5):
                self.manipulator_pub_list[i].publish(self.manipulator_position_ref[i])

        joint_state = JointState()
        joint_state.name = self.manipulator_joint_names
        joint_state.position = self.manipulator_position_ref
        self.joint_state_pub.publish(joint_state)

    def compute_references(self):
        factor_uav = 0.02
        if self.uav_first_ref_received == True:
            if self.mode == 0:
                q0 = self.uav_pose_ref.orientation.w
                q1 = self.uav_pose_ref.orientation.x
                q2 = self.uav_pose_ref.orientation.y
                q3 = self.uav_pose_ref.orientation.z
                yaw = math.atan2(2.0*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
                yaw = yaw + self.joy.axes[0]*factor_uav
                self.uav_pose_ref.orientation.x = 0.0
                self.uav_pose_ref.orientation.y = 0.0
                self.uav_pose_ref.orientation.z = math.sin(yaw/2.0)
                self.uav_pose_ref.orientation.w = math.cos(yaw/2.0)
                
                dx = self.joy.axes[3]*factor_uav
                dy = self.joy.axes[2]*factor_uav
                self.uav_pose_ref.position.x = self.uav_pose_ref.position.x + math.cos(-yaw)*dx + math.sin(-yaw)*dy
                self.uav_pose_ref.position.y = self.uav_pose_ref.position.y - math.sin(-yaw)*dx + math.cos(-yaw)*dy
                self.uav_pose_ref.position.z = self.uav_pose_ref.position.z + self.joy.axes[1]*factor_uav

        if ((self.first_manipulator_reference_received == True) and (self.mode != 0)):
            self.manipulator_position_ref[self.mode-1] = self.manipulator_position_ref[self.mode-1] + self.joy.axes[1]*0.01

    

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('uav_manipulator_joy_commander')
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        ne = Commander()
        ne.run()
    except rospy.ROSInterruptException:
        pass