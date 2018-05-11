/*
 * UavGeometricControl.h
 *
 *	UavGeometricControl class implements geometric control algorithm for
 *	moving-mass UAVs.
 *	Geometric control will handle both height and attitude control
 *	simultaneously.
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#ifndef UAV_GEOMETRY_CONTROL_H
#define UAV_GEOMETRY_CONTROL_H

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>

class UavGeometryControl
{

	public:

		/**
		 * Class constructor.
		 *
		 * @param rate - Controller rate.
		 */
		UavGeometryControl(int rate);

		/**
		 * Class destructor.
		 */
		virtual ~UavGeometryControl();

		/**
		 * Runs the geometric control algorithm until ROS shuts down.
		 */
		void run();

	private:

		/**
		 * IMU topic callback function.
		 * Following convention is used:
		 * 	1)yaw, 2) pitch, 3) roll
		 */
		void imu_cb(const sensor_msgs::Imu &msg);

		/**
		 * Perform quaternion to euler transformation.
		 *
		 * @param quaternion: 4 dimensional float array.
		 * @param euler: 3 dimensional float array.
		 */
		void quaternion2euler(float *quaternion, float *euler);

		/**
		 * Node handle used for setting up subscribers and publishers.
		 */
		ros::NodeHandle node_handle_;

		/**
		 * Subscriber handle for the IMU topic.
		 */
		ros::Subscriber imu_ros_sub_;

		/**
		 * Messages containing angle measured values, angle setpoints
		 * and angle rate measured values respectively.
		 */
		geometry_msgs::Vector3 euler_mv_, euler_sp_, euler_rate_mv_;

		/**
		 * Controller rate. Frequency at which the loop in run method
		 * will be executed.
		 */
		int controller_rate_ = -1;

		/**
		 * Sleep duration used while performing checks before starting the
		 * run() loop.
		 */
		float sleep_duration_ = -1.0;

		/**
		 * True when first callback function occurred otherwise false.
		 */
		bool start_flag_ = false;
};

#endif /* UAV_GEOMETRY_CONTROL_H */
