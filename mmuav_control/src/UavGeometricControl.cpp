/*
 * UavGeometricControl.cpp
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#include <mmuav_control/UavGeometricControl.h>

UavGeometricControl::UavGeometricControl(int rate)
{
	// Initialize variables
	controller_rate_ = rate;
	sleep_duration_ = 0.5;
}

UavGeometricControl::~UavGeometricControl()
{
	// TODO(lmark): Destructor..
}

void UavGeometricControl::run()
{
	ros::Rate loop_rate(controller_rate_);

	// Wait for the ROS time server
	while (ros::Time::now().toSec() == 0 && ros::ok())
	{
		ROS_INFO("UavGeometricControl::run() - "
				"Waiting for clock server to start");
	}
	ROS_INFO("UavGeometricControl::run() - "
			"Received first clock message");

	// TODO(lmark): Activate start flag first callback
	// Wait for start flag from IMU callback
	while (!start_flag_ && ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("UavGeometricControl::run() - "
				"Waiting for first measurement");
		ros::Duration(sleep_duration_).sleep();
	}
	ROS_INFO("UavGeometricControl::run() - "
			"Starting geometric control.");

	// Start the control loop.
	while (ros::ok())
	{
		// Do the control...
	}
}

int main(int argc, char** argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "geometric_control_node");

	// Initialize controller rate
	int rate;
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("rate", rate, int(100));

	// Start the control algorithm
	UavGeometricControl geometric_control(rate);
	geometric_control.run();

	return 0;
}

