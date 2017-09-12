#!/usr/bin/env python
import sys, os, copy
#print os.path.abspath(__file__ + '/../../resources')
#parentDir = os.path.abspath(__file__ + '/../../') # get parent directory
# abspath will get absolute path to file /blah/blah/file.py

#sys.path.insert(0, os.path.abspath(parentDir + '/resources')) # from parent
# directory get to resources

#print sys.path.append(os.path.realpath('../resources'))
#from PyQt4.QtGui import QApplication, QDialog
from PyQt4 import QtCore, QtGui, uic
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from geometry_msgs.msg import Vector3
import DualManipulatorPassiveJointKinematics as arms_kinematics_screw
from math import sin, cos, pi
import time


class ArmsControlGUI():

	def __init__(self):
		app = QtGui.QApplication(sys.argv)
		parentDir = os.path.abspath(__file__ + '/../../')
		self.window = uic.loadUi(parentDir + "/resources/ArmsControlGUI.ui")
		self.window.show()

		# setting up callbacks
		self.window.ComboBoxControllerSelect.currentIndexChanged.connect(self.ComboBoxControllerSelect)
		self.window.PanicButton.clicked.connect(self.PanicButtonCallback)
		self.window.StartControlButton.clicked.connect(self.StartControlButtonCallback)
		self.window.PositionLockButton.clicked.connect(self.PositionLockButtonCallback)
		self.window.ResetTimerButton.clicked.connect(self.ResetTimerButtonCallback)

		# Dials for direct arms control
		self.window.LeftArmShoulderDial.sliderMoved.connect(self.LeftArmShoulderDialCallback)
		self.window.LeftArmElbowDial.sliderMoved.connect(self.LeftArmElbowDialCallback)
		self.window.RightArmShoulderDial.sliderMoved.connect(self.RightArmShoulderDialCallback)
		self.window.RightArmElbowDial.sliderMoved.connect(self.RightArmElbowDialCallback)

		# spin boxes for inverse arms control
		self.window.ArmsHSpinBox.valueChanged.connect(self.ArmsHSpinBoxCallback)
		self.window.ArmsWSpinBox.valueChanged.connect(self.ArmsWSpinBoxCallback)
		self.window.ArmsL3SpinBox.valueChanged.connect(self.ArmsL3SpinBoxCallback)

		self.armsQRight = [-1.0+1.57, -2.0+1.57, -1.2]
		self.armsQLeft = [2.0-1.57, -2.0+1.57, -1.2]
		self.L3 = 0.05

		self.armsPosition = [0, 0] # x,y

		# Create publishers for arms 
		self.JointRight1Pub = rospy.Publisher("/mmuav/joint1_right_controller/command", 
			Float64, queue_size=1)
		self.JointRight2Pub = rospy.Publisher("/mmuav/joint2_right_controller/command", 
			Float64, queue_size=1)
		self.JointRight3Pub = rospy.Publisher("/mmuav/joint3_right_controller/command", 
			Float64, queue_size=1)
		self.JointLeft1Pub = rospy.Publisher("/mmuav/joint1_left_controller/command", 
			Float64, queue_size=1)
		self.JointLeft2Pub = rospy.Publisher("/mmuav/joint2_left_controller/command", 
			Float64, queue_size=1)
		self.JointLeft3Pub = rospy.Publisher("/mmuav/joint3_left_controller/command", 
			Float64, queue_size=1)
		self.ReferencePub = rospy.Publisher("/mmuav/arms_reference", Vector3, queue_size=1)
		# Sleep to initialize publishers and publish initial pose
		time.sleep(2.0)
		self.PublishData()




		# Subscribers to arducopter pose and screw pose
		self.arducopterPoseSub = rospy.Subscriber("vrpn_client_node/arducopter/pose", PoseStamped, 
			self.arducopterPoseCallback, queue_size=1)
		self.screwPoseSub = rospy.Subscriber("vrpn_client_node/screw/pose", PoseStamped, 
			self.screwPoseCallback, queue_size=1)
		self.arducopterPose = Pose()
		self.screwPose = Pose()

		# exit gui stuff
		sys.exit(app.exec_())

	def ComboBoxControllerSelect(self):
		controller = str(self.window.ComboBoxControllerSelect.currentText())

	def PanicButtonCallback(self):
		pass

	def StartControlButtonCallback(self):
		print "Start"

	def PositionLockButtonCallback(self):
		print "Lock"

	def LeftArmShoulderDialCallback(self):
		self.ArmsAngles.x = float(self.window.LeftArmShoulderDial.value())

	def LeftArmElbowDialCallback(self):
		self.ArmsAngles.z = float(self.window.LeftArmElbowDial.value())

	def RightArmShoulderDialCallback(self):
		self.ArmsAngles.y = float(self.window.RightArmShoulderDial.value())

	def RightArmElbowDialCallback(self):
		self.ArmsAngles.w = float(self.window.RightArmElbowDial.value())

	def ArmsHSpinBoxCallback(self):
		self.armsPosition[0] = float(self.window.ArmsHSpinBox.value())
		self.Inverse()

	def ArmsWSpinBoxCallback(self):
		self.armsPosition[1] = float(self.window.ArmsWSpinBox.value())
		self.Inverse()

	def ArmsL3SpinBoxCallback(self):
		self.L3 = float(self.window.ArmsL3SpinBox.value())
		self.Inverse()

	def ResetTimerButtonCallback(self):
		pass

	def Inverse(self):
		L1 = 0.094
		L2 = 0.061
		L3 = self.L3

		#arms_pos = [self.armsPosition[0]*cos(pi/4) - self.armsPosition[1]*sin(pi/4), \
		#	self.armsPosition[0]*sin(pi/4) + self.armsPosition[1]*cos(pi/4)]
		arms_pos = [self.armsPosition[1], -self.armsPosition[0]]
		tempV3 = Vector3()
		tempV3.x = -arms_pos[1]
		tempV3.y = arms_pos[0]
		self.ReferencePub.publish(tempV3)

		q = arms_kinematics_screw.ik_both_arms(self.armsQRight, self.armsQLeft, 
			arms_pos, L1, L2, L3)
		self.armsQRight = copy.deepcopy(q[0])
		self.armsQLeft = copy.deepcopy(q[1])
		#print q
		#print self.armsQRight, self.armsQLeft

		self.PublishData()
		#print "Data"
		#print self.armsPosition
		#print self.armsQRight
		#print self.armsQLeft

	def PublishData(self):
		self.JointRight1Pub.publish(Float64(self.armsQRight[0]-1.57))
		self.JointRight2Pub.publish(Float64(self.armsQRight[1]-1.57))
		self.JointRight3Pub.publish(Float64(-self.armsQRight[2]))
		self.JointLeft1Pub.publish(Float64(self.armsQLeft[0]+1.57))
		self.JointLeft2Pub.publish(Float64(self.armsQLeft[1]-1.57))
		self.JointLeft3Pub.publish(Float64(-self.armsQLeft[2]))

	def arducopterPoseCallback(self, msg):
		self.arducopterPose = msg.pose

		L1 = 0.093
		L2 = 0.061
		L3 = self.L3
		self.armsPosition[0] = -self.arducopterPose.position.z - (-self.screwPose.position.z)
		self.armsPosition[1] = -self.arducopterPose.position.x - (-self.screwPose.position.x)

		if self.armsPosition[0] > 0.03: self.armsPosition[0] = 0.03
		if self.armsPosition[0] < -0.03: self.armsPosition[0] = -0.03
		if self.armsPosition[1] > 0.03: self.armsPosition[1] = 0.03
		if self.armsPosition[1] < -0.03: self.armsPosition[1] = -0.03

		self.armsPosition[1] = -self.armsPosition[1]
		print self.armsPosition


		arms_pos = [self.armsPosition[0]*cos(-pi/4) - self.armsPosition[1]*sin(-pi/4), \
			self.armsPosition[0]*sin(-pi/4) + self.armsPosition[1]*cos(-pi/4)]

		q = arms_kinematics_screw.ik_both_arms(self.armsQRight, self.armsQLeft, 
			arms_pos, L1, L2, L3)
		self.armsQRight = copy.deepcopy(q[0])
		self.armsQLeft = copy.deepcopy(q[1])
		self.PublishData()


	def screwPoseCallback(self, msg):
		self.screwPose = msg.pose

if __name__ == "__main__":
	rospy.init_node("ArmsScrewControlGUI")
	Gui = ArmsControlGUI()