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
#include <rosgraph_msgs/Clock.h>
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

		// Callback functions
		void imu_cb(const sensor_msgs::Imu &msg);
		void xd_cb(const geometry_msgs::Vector3 &msg);
		void vd_cb(const geometry_msgs::Vector3 &msg);
		void ad_cb(const geometry_msgs::Vector3 &msg);
		void omegad_cb(const geometry_msgs::Vector3 &msg);
		void alphad_cb(const geometry_msgs::Vector3 &msg);
		void b1d_cb(const geometry_msgs::Vector3 &msg);

		/**
		 * Perform quaternion to euler transformation.
		 *
		 * @param quaternion: 4 dimensional float array.
		 * @param euler: 3 dimensional float array.
		 */
		void quaternion2euler(float *quaternion, float *euler);

		/**
		 * Perform hat operator on given vector components.
		 *
		 * @param x - x vector component
		 * @param y - y vector component
		 * @param z - z vector component
		 * @param hatMatrixs - Matrix of the following form:
		 * 	[ 0  -z,  y]
		 * 	[ z,  0, -x]
		 * 	[-y,  x,  0]
		 */
		void hatOperator(double x, double y, double z, Matrix<double, 3, 3> &hatMatrix);

		/**
		 * Node handle used for setting up subscribers and publishers.
		 */
		ros::NodeHandle node_handle_;

		/**
		 * Subscriber handle for the IMU topic.
		 */
		ros::Subscriber imu_ros_sub_;

		/**
		 * Subscriber handle for:
		 * - desired position reference
		 * - desired linear velocity reference
		 * - desired linear acceleration reference
		 * - desired heading reference
		 */
		ros::Subscriber xd_ros_sub_, vd_ros_sub_, ad_ros_sub_, b1d_ros_sub_,
						omega_d_ros_sub_, alpha_d_ros_sub_;

		/**
		 * Messages containing angle measured values and
		 * angle rate measured values respectively.
		 */
		geometry_msgs::Vector3 euler_mv_, euler_rate_mv_;

		/**
		 * CONTROLLER INPUT REFERENCES - position
		 * 	- desired position x_d_
		 * 	- desired linear velocity v_d_
		 * 	- desired linear acceleration a_d_
		 */
		Matrix<double, 3, 1> x_d_, v_d_, a_d_;
		Matrix<double, 3, 1>  x_mv_, v_mv_, a_mv_;

		/**
		 * 	CONTROLLER INPUT REFERENCES - attitude
		 * 	- desired angular velocity omega_d_
		 * 	- desired angular acceleration alpha_d_
		 * 	- desired direction of the first body axis b1_D
		 */
		Matrix<double, 3, 3> omega_d_, alpha_d_;
		Matrix<double, 3, 3> omega_mv_, alpha_mv_;
		Matrix<double, 3, 1> b1_d_, b1_mv_;

		/**
		 *	MEASURED VALUES - position
		 */

		/**
		 *	MEASURED VALUES - attitude
		 */
		Matrix<double, 3, 3> omega_mv_, alpha_mv_;

		/**
		 * CONTROLLER PARAMETERS:
		 *	- k_x: position tracking error gain (eX)
		 *	- k_v: velocity tracking error gain (eV)
		 *	- k_R: orientation matrix error gain (eR)
		 *	- k_omega: angular velocity error gain (eOmega)
		 */
		double k_x_, k_v_, k_R_, k_omega_;

		/**
		 * Variable used for calculating time intervals in the controller loop.
		 */
		ros::Time t_old_;

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
