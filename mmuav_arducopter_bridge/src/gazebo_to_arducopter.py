#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float64MultiArray
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

        rospy.Subscriber('movable_mass_all/command', Float64MultiArray, self.all_mass_cb, queue_size=1)
        self.mass_all_array = [0.0, 0.0, 0.0, 0.0]

    def run(self):

        ##Open communication
        print('Opening serial communication')
        self.serial_device.open()

        while not self.serial_device.is_open:
            print('Waiting...')

        print('Serial communication opened!')

        rospy.spin()

    def all_mass_cb(self,msg):
        if len(msg.data) < 4:
            print("Not enough data, should be at least 4. Length: ", len(msg.data))
        else:
            for i in range(len(msg.data)):
                self.mass_all_array[i] = msg.data[i]
                if (self.mass_all_array[i] > 0.08):
                    self.mass_all_array[i] = 0.08
                elif (self.mass_all_array[i] < -0.08):
                    self.mass_all_array[i] = -0.08

        # Write immediately
        s = pack('iiiicxxx',    round(5000*self.mass_all_array[0]), 
                                round(5000*self.mass_all_array[1]*0),
                                round(5000*self.mass_all_array[2]),
                                round(5000*self.mass_all_array[3]*0),
                                'C')
        self.serial_device.write(s)


if __name__ == '__main__':

    rospy.init_node('Serial_communication')
    sercom = StepperInterface()
    sercom.run()