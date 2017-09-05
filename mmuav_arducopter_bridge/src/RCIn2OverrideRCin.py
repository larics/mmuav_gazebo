#!/usr/bin/env python

import rospy
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import OverrideRCIn


class RCIn2OverrideRCIn():
	def __init__(self):

		rospy.Subscriber('/jeti/mavros/rc/in', RCIn, self.rcin_callback)
		self.overrideRCIn_pub = rospy.Publisher('/arducopter/mavros/rc/override', OverrideRCIn, queue_size=10)
		self.overrideRCIn_msg = OverrideRCIn()

	def run(self):

		rospy.spin()


	def rcin_callback(self,msg):
		#self.overrideRCIn_msg.channels.clear()
		#for i in range(8):
		if len(msg.channels) >=8:
			self.overrideRCIn_msg.channels = msg.channels[0:8]	
			self.overrideRCIn_pub.publish(self.overrideRCIn_msg)

if __name__ == '__main__':

	rospy.init_node('RCIn_to_OverrideRCIn')
	conversion = RCIn2OverrideRCIn()
	conversion.run()
