/******************************************************************************
File name: gazeboToArducopterSerialNode.cpp
Description: ROS serial interface for arducopter stepper board
Author: Antun Ivanovic
******************************************************************************/

#include <GazeboToArducopterSerial.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gazeboToArducopterSerialNode");
	GazeboToArducopterSerial serialInterface;
	serialInterface.run();
	return 0;
}