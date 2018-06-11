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
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>

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
		void odom_cb(const nav_msgs::Odometry &msg);
		void xd_cb(const geometry_msgs::Vector3 &msg);
		void vd_cb(const geometry_msgs::Vector3 &msg);
		void ad_cb(const geometry_msgs::Vector3 &msg);
		void omegad_cb(const geometry_msgs::Vector3 &msg);
		void alphad_cb(const geometry_msgs::Vector3 &msg);
		void b1d_cb(const geometry_msgs::Vector3 &msg);
		void rd_cb(const std_msgs::Float64MultiArray &msg);
		void ctl_mode_cb(const std_msgs::Int8 &msg);

		/**
		 * Calculate b3_d and f_u as position tracking control inputs.
		 *
		 * @param pos_desired - desired position reference
		 * 						(may change due to prefilter)
		 * @param pos_old - old position
		 * @param dt - time interval
		 * @param b3_d - thrust heading reference, assigned in method
		 * @param f_u - thrust magnitude value, assigned in method
		 */
		void trajectoryTracking(
				const Matrix<double, 3, 1> pos_desired,
				const Matrix<double, 3, 1> pos_old,
				const double dt,
				Matrix<double, 3, 1> &b3_d,
				double &f_u);

		/**
		 * Calculate control moments M_u used for attitude tracking.
		 *
		 * @param b1_d - desired heading
		 * @param b3_d - desired thrust vector
		 * @param dt - time interval
		 * @param R_c_old - reference for old calculated rotation matrix
		 * 					(Position tracking)
		 * @param omega_c_old - reference for old calculated angular velocity
		 * 						(Position tracking)
		 * @param M_u - control moments, assigned in method
		 */
		void attitudeTracking(
				const Matrix<double, 3, 1> b1_desired,
				const Matrix<double, 3, 1> b3_desired,
				const double dt,
				Matrix<double, 3, 3> &R_c_old,
				Matrix<double, 3, 3> &omega_c_old,
				Matrix<double, 3, 1> &M_u);

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
		void hatOperator(
				const double x,
				const double y,
				const double z,
				Matrix<double, 3, 3> &hatMatrix);

		/**
		 * Perform a vee( V ) operator on a given hatMatrix.
		 * It is implied that the given hatMatrix is a skew matrix.
		 * It will decompose the skew matrix into a given veeVector reference.
		 *
		 * @param hatMatrx
		 * @param veeVector
		 */
		void veeOperator(
				const Matrix<double, 3, 3> hatMatrix,
				Matrix<double, 3, 1> &veeVector);
		/**
		 * Euler angles represented as a rotation matrix.
		 *
		 * @param roll
		 * @param pitch
		 * @param yaw
		 * @param rotMatrix - Rotation matrix will be stored here
		 */
		void euler2RotationMatrix(
				const double roll,
				const double pitch,
				const double yaw,
				Matrix<double, 3, 3> &rotMatrix);

		/**
		 * Perform saturation filter on the given value;
		 *
		 * @param value
		 * @param lowLimit
		 * @param highLimit
		 *
		 * @return saturated value
		 */
		double saturation(
				double value,
				double lowLimit,
				double highLimit);

		/**
		 * Node handle used for setting up subscribers and publishers.
		 */
		ros::NodeHandle node_handle_;

		/**
		 * Subscriber handle for the IMU and odometry topics.
		 */
		ros::Subscriber imu_ros_sub_, odom_ros_sub_;

		/**
		 * - Motor velocities publisher
		 * - Attitude error publisher
		 */
		ros::Publisher rotor_ros_pub_, att_err_ros_pub_;

		/**
		 * Subscriber handle for:
		 * - desired position reference
		 * - desired linear velocity reference
		 * - desired linear acceleration reference
		 * - desired heading reference
		 * - desired angular velocity
		 * - desired angular acceleration
		 * - desired control mode ( position / attitude )
		 * - orientation: roll, pitch, yaw
		 */
		ros::Subscriber xd_ros_sub_, vd_ros_sub_, ad_ros_sub_,
						b1d_ros_sub_, omega_d_ros_sub_, rd_ros_sub_,
						alpha_d_ros_sub_, ctl_mode_ros_sub_,
						orientation_ros_sub_;

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
		Matrix<double, 3, 1> omega_d_, alpha_d_;
		Matrix<double, 3, 1> omega_mv_, alpha_mv_;
		Matrix<double, 3, 1> b1_d_;

		/**
		 * R_mv_ - Measured rotation matrix.
		 * R_d_  - Desired rotation matrix
		 */
		Matrix<double, 3, 3> R_mv_, R_d_;

		/**
		 * CONTROLLER PARAMETERS:
		 *	- k_x: position tracking error gain (eX)
		 *	- k_v: velocity tracking error gain (eV)
		 *	- k_R: orientation matrix error gain (eR)
		 *	- k_omega: angular velocity error gain (eOmega)
		 */
		Matrix<double, 3, 3> k_x_, k_v_, k_R_, k_omega_;

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
		 * True when imu callback function occurred otherwise false.
		 */
		bool imu_start_flag_;

		/**
		 * True when odometry callback function occured otherwise false.
		 */
		bool odometry_start_flag_;

		/**
		 * Current control mode:
		 * 	- position control
		 * 	- attitude control
		 * 	- linear velocity control
		 */
		int current_control_mode_;
};

#endif /* UAV_GEOMETRY_CONTROL_H */
