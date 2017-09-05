#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from struct import *
import serial
import time

class StepperInterface():
    def __init__(self):

        ##Setup serial port 
        self.serial_device = serial.Serial()

        self.serial_device.port = '/dev/ttyUSB0'
        self.serial_device.baudrate = 115200
        self.serial_device.parity = serial.PARITY_NONE
        self.serial_device.stopbits = serial.STOPBITS_ONE
        self.serial_device.bytesize = serial.EIGHTBITS

        self.mass1 = 0
        self.mass2 = 0
        self.mass3 = 0
        self.mass4 = 0

        self.r = rospy.Rate(60)

        rospy.Subscriber('movable_mass_0_position_controller/command', Float64, self.mm1_cb)
        rospy.Subscriber('movable_mass_1_position_controller/command', Float64, self.mm2_cb)
        rospy.Subscriber('movable_mass_2_position_controller/command', Float64, self.mm3_cb)
        rospy.Subscriber('movable_mass_3_position_controller/command', Float64, self.mm4_cb)

    def run(self):

        ##Open communication
        print 'Opening serial communication'
        self.serial_device.open()

        while not self.serial_device.is_open:
            print 'Waiting...'

        print 'Serial communication opened!'

        while not rospy.is_shutdown():
            ##Check for messages with 60 Hz

            s = pack('iiiicxxx', self.mass1, self.mass2, self.mass3, self.mass4, 'C')

            ##Debug print
            print self.mass1

            self.serial_device.write(s)
            self.r.sleep()


    def mm1_cb(self,msg):
        if (msg.data > 0.08):
            msg.data = 0.08
        elif (msg.data < -0.08):
            msg.data = -0.08

        self.mass1 = round(5000*msg.data)

    def mm2_cb(self,msg):
        if (msg.data > 0.08):
            msg.data = 0.08
        elif (msg.data < -0.08):
            msg.data = -0.08

        self.mass2 = round(5000*msg.data)

    def mm3_cb(self,msg):
        if (msg.data > 0.08):
            msg.data = 0.08
        elif (msg.data < -0.08):
            msg.data = -0.08

        self.mass3 = round(5000*msg.data)

    def mm4_cb(self,msg):
        if (msg.data > 0.08):
            msg.data = 0.08
        elif (msg.data < -0.08):
            msg.data = -0.08

        self.mass4 = round(5000*msg.data)


if __name__ == '__main__':

    rospy.init_node('Serial_communication')
    sercom = StepperInterface()
    sercom.run()