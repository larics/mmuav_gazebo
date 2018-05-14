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
#include <eigen3/Eigen/Dense>

using namespace Eigen;

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

		void imu_cb(const sensor_msgs::Imu &msg);
		void xd_cb(const geometry_msgs::Vector3 &msg);
		void b1d_cb(const geometry_msgs::Vector3 &msg);

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
		 * Subscriber handle for the IMU topic,
		 * desired position (x_D) topic and
		 * desired direction of the first body axis b1_D respectively.
		 */
		ros::Subscriber imu_ros_sub_, xd_ros_sub_, b1d_ros_sub_;

		/**
		 * Messages containing angle measured values and
		 * angle rate measured values respectively.
		 */
		geometry_msgs::Vector3 euler_mv_, euler_rate_mv_;

		/**
		 * CONTROLLER INPUT REFERENCES(initialized from appropriate subscriber
		 * callback functions):
		 * 	- desired position x_D
		 * 	- desired direction of the first body axis b1_D
		 */
		Matrix<float, 3, 1> x_d_, b1_d_;

		/**
		 * Controller rate. Frequency at which the loop in run method
		 * will be executed.
		 */
		int controller_rate_;

		/**
		 * Sleep duration used while performing checks before starting the
		 * run() loop.
		 */
		float sleep_duration_;

		/**
		 * True when first callback function occurred otherwise false.
		 */
		bool start_flag_;
};

#endif /* UAV_GEOMETRY_CONTROL_H */
