/*
 * UavGeometricControl.cpp
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#include <mmuav_control/UavGeometryControl.hpp>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Float64.h>

using namespace std;

const Matrix<double, 3, 1> E3(0, 0, 1);
const double UAV_MASS = 2.1;
const double G = 9.81;
const double ARM_LENGTH = 0.314;
const double MOMENT_CONSTANT = 0.016;
const double MOTOR_CONSTANT = 8.54858e-06;
const double ROTOR_MASS = 0.01;
const double ROTOR_VELOCITY_SLOWDOWN_SIM = 15;
const double ROTOR_RADIUS = 0.1524;
Matrix<double, 3, 3> INERTIA;
Matrix<double, 4, 4> THRUST_TRANSFORM;
Matrix<double, 3, 3> EYE;

// Define control modes
const int POSITION_CONTROL = 1;
const int ATTITUDE_CONTROL = 2;
const int VELOCITY_CONTROL = 3;

// Controller rate
const int CONTROLLER_RATE = 100;

UavGeometryControl::UavGeometryControl(int rate)
{
	// Initialize controller variables
	controller_rate_ = rate;
	sleep_duration_ = 0.5;
	imu_start_flag_ = false;
	odometry_start_flag_ = false;
	current_control_mode_ = POSITION_CONTROL;

	// Initialize inertia matrix
	INERTIA.setZero(3, 3);
	INERTIA(0, 0) = 0.0826944;
	INERTIA(1, 1) = 0.0826944;
	INERTIA(2, 2) = 0.0104;

	Matrix<double, 3, 3> rotor_inertia;
	rotor_inertia.setZero(3, 3);
	rotor_inertia(0, 0) = 1/12 * ROTOR_MASS
			* (0.031 * 0.031 + 0.005 * 0.005)
			* ROTOR_VELOCITY_SLOWDOWN_SIM;
	rotor_inertia(1, 1) = 1/12 * ROTOR_MASS
			* (4 * ROTOR_RADIUS * ROTOR_RADIUS + 0.005 * 0.005)
			* ROTOR_VELOCITY_SLOWDOWN_SIM;
	rotor_inertia(2, 2) = 1/12 * ROTOR_MASS
			* (4 * ROTOR_RADIUS * ROTOR_RADIUS + 0.031 * 0.031)
			* ROTOR_VELOCITY_SLOWDOWN_SIM
			+ ROTOR_MASS * ARM_LENGTH * ARM_LENGTH;

	INERTIA = INERTIA + 4 * rotor_inertia;

	// Initialize thrust transform matrix
	THRUST_TRANSFORM.setZero(4, 4);

	// First row
	THRUST_TRANSFORM(0, 0) = 1;
	THRUST_TRANSFORM(0, 1) = 1;
	THRUST_TRANSFORM(0, 2) = 1;
	THRUST_TRANSFORM(0, 3) = 1;

	// Second row
	THRUST_TRANSFORM(1, 1) = ARM_LENGTH;
	THRUST_TRANSFORM(1, 3) = - ARM_LENGTH;

	// Third row
	THRUST_TRANSFORM(2, 0) = - ARM_LENGTH;
	THRUST_TRANSFORM(2, 2) = ARM_LENGTH;

	// Fourth row
	THRUST_TRANSFORM(3, 0) = MOMENT_CONSTANT;
	THRUST_TRANSFORM(3, 1) = - MOMENT_CONSTANT;
	THRUST_TRANSFORM(3, 2) = MOMENT_CONSTANT;
	THRUST_TRANSFORM(3, 3) = - MOMENT_CONSTANT;

	// Invert the matrix
	THRUST_TRANSFORM = THRUST_TRANSFORM.inverse().eval();

	// Initialize eye(3) matrix
	EYE.setZero(3, 3);
	EYE(0, 0) = 1;
	EYE(1, 1) = 1;
	EYE(2, 2) = 1;

	cout << "Thrust transform: \n";
	cout << THRUST_TRANSFORM << "\n";
	cout << endl;

	// Initialize desired position values
	x_d_.setZero(3,1);
	v_d_.setZero(3,1);
	a_d_.setZero(3,1);

	// Initial measured position values
	x_mv_.setZero(3,1);
	v_mv_.setZero(3,1);
	a_mv_.setZero(3,1);

	// Initialize desired attitude
	omega_d_.setZero(3, 1);
	alpha_d_.setZero(3, 1);
	b1_d_.setZero(3,1);
	b1_d_(0,0) = 1;
	R_d_.setZero(3, 3);

	// Initialize measured
	omega_mv_.setZero(3, 1);
	alpha_mv_.setZero(3, 1);
	R_mv_.setZero(3,3);

	// Initialize controller parameters
	// Parameters initialized according to 2010-extended.pdf
	k_x_.setZero(3, 3);
	k_x_(0, 0) = 5 * UAV_MASS;
	k_x_(1, 1) = 5 * UAV_MASS;
	k_x_(2, 2) = 12 * UAV_MASS;

	k_v_.setZero(3, 3);
	k_v_(0, 0) = 3 * UAV_MASS;
	k_v_(1, 1) = 3 * UAV_MASS;
	k_v_(2, 2) = 5 * UAV_MASS;

	k_R_.setZero(3, 3);
	k_R_(0, 0) = 6;
	k_R_(1, 1) = 6;
	k_R_(2, 2) = 2.5;

	k_omega_.setZero(3, 3);
	k_omega_(0, 0) = 2;
	k_omega_(1, 1) = 2;
	k_omega_(2, 2) = 0.5;

	// Initialize subscribers and publishers
	imu_ros_sub_ = node_handle_.subscribe(
			"/uav/imu", 1,
			&UavGeometryControl::imu_cb, this);
	odom_ros_sub_ = node_handle_.subscribe(
			"/uav/odometry", 1,
			&UavGeometryControl::odom_cb, this);
	rotor_ros_pub_ = node_handle_.advertise<mav_msgs::Actuators>(
			"/gazebo/command/motor_speed", 1);
	att_err_ros_pub_ = node_handle_.advertise<std_msgs::Float64>(
			"/uav/att_err", 10);

	// Initialize position reference subscribers
	xd_ros_sub_ = node_handle_.subscribe(
			"/uav/x_desired", 1,
			&UavGeometryControl::xd_cb, this);
	vd_ros_sub_ = node_handle_.subscribe(
			"/uav/v_desired", 1,
			&UavGeometryControl::vd_cb, this);
	ad_ros_sub_ = node_handle_.subscribe(
			"/uav/a_desired", 1,
			&UavGeometryControl::ad_cb, this);

	// Initialize attitude reference subscribers
	b1d_ros_sub_ = node_handle_.subscribe(
			"/uav/b1_desired", 1,
			&UavGeometryControl::b1d_cb, this);
	omega_d_ros_sub_ = node_handle_.subscribe(
			"/uav/omega_desired", 1,
			&UavGeometryControl::omegad_cb, this);
	alpha_d_ros_sub_ = node_handle_.subscribe(
			"/uav/alpha_desired", 1,
			&UavGeometryControl::alphad_cb, this);
	rd_ros_sub_ = node_handle_.subscribe(
			"/uav/R_desired", 1,
			&UavGeometryControl::rd_cb, this);

	// Control mode subscriber
	ctl_mode_ros_sub_ = node_handle_.subscribe(
			"/uav/control_mode", 1,
			&UavGeometryControl::ctl_mode_cb, this);
}

UavGeometryControl::~UavGeometryControl()
{
	// TODO(lmark): Destructor..
}

void UavGeometryControl::run()
{
	// Loop time interval check
	double dt;

	// Wait for the ROS time server
	while (ros::Time::now().toSec() == 0 && ros::ok())
	{
		ROS_INFO("UavGeometricControl::run() - "
				"Waiting for clock server to start");
	}
	ROS_INFO("UavGeometricControl::run() - "
			"Received first clock message");

	// Wait for start flag from IMU callback
	while (!imu_start_flag_ && ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("UavGeometricControl::run() - "
				"Waiting for first IMU measurement");
		ros::Duration(sleep_duration_).sleep();
	}

	// Wait for start flag from odometry callback
	while (!odometry_start_flag_ && ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("UavGeometricControl::run() - "
				"Waiting for first odometry measurement");
		ros::Duration(sleep_duration_).sleep();
	}

	ROS_INFO("UavGeometricControl::run() - "
			"Starting geometric control in 5...");

	t_old_ = ros::Time::now();

	// Position errors
	Matrix<double, 3, 1> e_x, e_v;

	// Attitude errors
	Matrix<double, 3, 1> e_omega, e_R;

	// Attitude error - scalar (PSI)
	Matrix<double, 3, 3> att_err;
	std_msgs::Float64 att_err_msg;

	// Auxiliary skew matrices
	Matrix<double, 3, 3> e_R_skew, omega_mv_skew, omega_c_skew,
						 alpha_c_skew;

	/*
	 * A 	- desired control force for the translational dynamics
	 * b3_d - desired thrust vector
	 * b2_d - desired y-direction body vector
	 * M_u 	- control moment
	 * b1_c - b1_c = Proj[b1_d] b1_d projection on the plane with normal b3_d
	 */
	Matrix<double, 3, 1> A, b3_d, M_u,
						 b2_d, b1_c, x_old,
						 b1_old, x_des, b1_des,
						 v_d_old;

	/*
	 * R_d 	- desired rotation matrix
	 * R_d_old - used for matrix differentiation
	 * R_d_dot - Desired matrix derivative
	 */
	Matrix<double, 3, 3> R_c, R_c_old, R_c_dot, omega_c_old;
	R_c_old = EYE;
	omega_c_old = EYE;

	// Rotor velocities control vector
	Matrix<double, 4, 1> rotor_velocities;
	Matrix<double, 4, 1> rotor_signs;

	// Initialize rotor velocity publisher msg
	mav_msgs::Actuators rotor_vel_msg;
	vector<double> velocity_vector(4);
	rotor_vel_msg.angular_velocities = velocity_vector;

	// Total thrust control value
	double f_u;

	// Wait for gazebo to start up
	sleep(5);

	// Start the control loop.
	while (ros::ok())
	{
		// Do 1 round of callbacks
		ros::spinOnce();

		// Calculate time difference
		double current_time = ros::Time::now().toSec();
		dt = current_time - t_old_.toSec();

		// Check if time is right
		if (dt < 1.0 / controller_rate_)
			continue;

		// Update old time
		t_old_ = ros::Time::now();

		// Construct current rotation matrix - R
		euler2RotationMatrix(
				euler_mv_.x,
				euler_mv_.y,
				euler_mv_.z,
				R_mv_);

		// Construct angular velocity vector
		omega_mv_(0, 0) = euler_rate_mv_.x;
		omega_mv_(1, 0) = euler_rate_mv_.y;
		omega_mv_(2, 0) = euler_rate_mv_.z;

		// Position and heading prefilter
		x_des = x_old + 0.025 * (x_d_ - x_old);
		b1_des = b1_old + 0.025 * (b1_d_ - b1_old);

		// ####################################################################
		// TRAJECTORY TRACKING
		// Calculate total thrust and b3_d (desired thrust vector)
		if (current_control_mode_ == POSITION_CONTROL)
		{
			e_x = - (x_mv_ - x_des);
			e_v = - (v_mv_ - v_d_);
		}
		else if (current_control_mode_ == ATTITUDE_CONTROL)
		{
			/**
			 * During Attitude control only take z - component of
			 * position and linear velocity.
			 */

			v_d_ = (x_d_ - x_old) / dt;
			e_x = - (x_mv_(2, 0) - x_d_(2, 0)) * E3;
			e_v = - (v_mv_(2, 0) - v_d_(2, 0)) * E3;
		}
		else
		{
			ROS_ERROR("Invalid control mode given.");
			break;
		}

		// Update old position
		b1_old = b1_des;
		x_old = x_des;
		v_d_old = v_d_;

		// Calculate control thrust
		A =  k_x_ * e_x
			+ k_v_ * e_v
			+ UAV_MASS * G * E3
			- UAV_MASS * a_d_;
		f_u = A.dot( R_mv_ * E3 );
		b3_d = A / A.norm();

		if (current_control_mode_ == POSITION_CONTROL)
		{
			/**
			 * During position control desired rotation, angular velocity
			 * and angular acceleration matrices will be CALCULATED.
			 * R_c, omega_c, alpha_c
			 */

			/*
			 * b13_normal - Normal of plane spanned by b3_d and b1_d.
			 *
			 * Note: b1_d will not necessarily lie in the plane with b3_d normal,
			 * it is needed to calculate it's projection to that plane.
			 */
			Matrix<double, 3, 1> b13_normal = b3_d.cross(b1_des);
			b13_normal = b13_normal / b13_normal.norm();

			// Compute b1_c = Proj[b1_d] onto the plane with normal b3_d
			b1_c = - b3_d.cross(b13_normal);

			// Construct desired rotation matrix
			b2_d = b3_d.cross(b1_c);
			b2_d = b2_d.array() / b2_d.norm();
			R_c.setZero(3, 3);
			R_c << b1_c, b2_d, b3_d;

			// Calculate angular velocity based on R_c[k] and R_c[k-1]
			Matrix<double, 3, 3> AA = R_c * R_c_old.adjoint();
			double theta = acos((AA.trace() - 1 ) / 2);
			omega_c_skew = (AA - AA.adjoint()) * theta / (2 * dt * sin(theta));
			veeOperator(omega_c_skew, omega_d_);

			// Check if omega_c_skew is NAN
			if ((double)omega_c_skew(0, 0) != (double)omega_c_skew(0, 0)
					|| omega_d_.norm() > 15)
			{
				// If current value is NAN take the old value do not update
				omega_c_skew = omega_c_old;
				alpha_c_skew.setZero(3, 3);
			}
			else
			{
				// If it's valid update
				alpha_c_skew = (omega_c_skew - omega_c_old);
				omega_c_old = omega_c_skew;
			}

			// Remap calculated values to desired
			veeOperator(omega_c_skew, omega_d_);
			veeOperator(alpha_c_skew, alpha_d_);
			R_d_ = R_c;
		}
		else if (current_control_mode_ == ATTITUDE_CONTROL)
		{
			// Do nothing here - read desired attitude values from
			// callback functions.
		}
		else
		{
			ROS_ERROR("Invalid control mode given.");
			break;
		}

		// Update old R_c
		R_c_old = R_c;


		// ###################################################################
		// ATTITUDE TRACKING
		// Calculate control moment M
		e_R_skew = (R_d_.adjoint() * R_mv_ - R_mv_.adjoint() * R_d_) / 2;
		veeOperator(e_R_skew, e_R);
		e_omega = omega_mv_ - R_mv_.adjoint() * R_d_ * omega_d_;
		hatOperator(
				(double)omega_mv_(0, 0),
				(double)omega_mv_(1, 0),
				(double)omega_mv_(2, 0),
				omega_mv_skew);
		M_u = 	- k_R_ * e_R
				- k_omega_ * e_omega
				+ omega_mv_.cross(INERTIA * omega_mv_)
				- INERTIA *
				(
					omega_mv_skew * R_mv_.adjoint() * R_d_ * omega_d_
					- R_mv_.adjoint() * R_d_ * alpha_d_
				);

		// Calculate thrust velocities
		Matrix<double, 4, 1> thrust_moment_vec(
				f_u,
				M_u(0, 0),
				M_u(1, 0),
				M_u(2, 0));

		// Convert force vector - THRUST_TRANSFORM * thrus_moment_vec ...
		// ...to angular velocity -> fi = MOTOR_CONSTANT * ang_vel_i^2
		rotor_velocities = THRUST_TRANSFORM * thrust_moment_vec;
		// cout << "Forces: \n" << rotor_velocities << "\n";
		rotor_signs = rotor_velocities.array().sign();
		rotor_velocities = rotor_velocities.array().abs();
		rotor_velocities = rotor_velocities / MOTOR_CONSTANT;
		rotor_velocities = rotor_velocities.array().sqrt();

		// Fill and publish rotor message
		rotor_vel_msg.angular_velocities[0] =
				rotor_signs(0, 0) * rotor_velocities(0, 0);
		rotor_vel_msg.angular_velocities[1] =
				rotor_signs(1, 0) * rotor_velocities(1, 0);
		rotor_vel_msg.angular_velocities[2] =
				rotor_signs(2, 0) * rotor_velocities(2, 0);
		rotor_vel_msg.angular_velocities[3] =
				rotor_signs(3, 0) * rotor_velocities(3, 0);
		rotor_ros_pub_.publish(rotor_vel_msg);

		// Calculate attitude error
		att_err = (EYE - R_d_.adjoint() * R_mv_);
		att_err_msg.data = att_err.trace() / 2;
		att_err_ros_pub_.publish(att_err_msg);

		//cout << "e_x: \n" << e_x << "\n";
		//cout << "e_v: \n" << e_v << "\n";
		cout << "e_R: \n" << e_R << "\n";
		cout << "e_omega: \n" << e_omega << "\n";
		//cout << "f_u: \n" << f_u << "\n";
		//cout << "M_u: \n" << M_u << "\n";
		cout << "Rotor_vel: \n" << rotor_velocities << "\n";
		//cout << "R_d: \n" << R_d << "\n";
		//cout << "R_mv: \n" << R_mv_ << "\n";
		//cout << "b3_d: \n" << b3_d << "\n";
		//cout << "\n\n";
		cout << endl;
	}
}

void UavGeometryControl::ctl_mode_cb(const std_msgs::Int8 &msg)
{
	current_control_mode_ = msg.data;
}

void UavGeometryControl::xd_cb(const geometry_msgs::Vector3 &msg)
{
	x_d_(0, 0) = msg.x;
	x_d_(1, 0) = msg.y;
	x_d_(2, 0) = msg.z;
}

void UavGeometryControl::vd_cb(const geometry_msgs::Vector3 &msg)
{
	v_d_(0, 0) = msg.x;
	v_d_(1, 0) = msg.y;
	v_d_(2, 0) = msg.z;
}

void UavGeometryControl::ad_cb(const geometry_msgs::Vector3 &msg)
{
	a_d_(0, 0) = msg.x;
	a_d_(1, 0) = msg.y;
	a_d_(2, 0) = msg.z;
}

void UavGeometryControl::b1d_cb(const geometry_msgs::Vector3 &msg)
{
	b1_d_(0, 0) = msg.x;
	b1_d_(1, 0) = msg.y;
	b1_d_(2, 0) = msg.z;
	b1_d_ = b1_d_.array() / b1_d_.norm();
}

void UavGeometryControl::omegad_cb(const geometry_msgs::Vector3 &msg)
{
	omega_d_(0, 0) = msg.x;
	omega_d_(1, 0) = msg.y;
	omega_d_(2, 0) = msg.z;
}

void UavGeometryControl::alphad_cb(const geometry_msgs::Vector3 &msg)
{
	alpha_d_(0, 0) = msg.x;
	alpha_d_(1, 0) = msg.y;
	alpha_d_(2, 0) = msg.z;
}

void UavGeometryControl::rd_cb(const std_msgs::Float64MultiArray &msg)
{
	// Unpack 9x1 array into a 3x3 rotation matrix by ROWS
	R_d_(0, 0) = msg.data[0];
	R_d_(0, 1) = msg.data[1];
	R_d_(0, 2) = msg.data[2];

	R_d_(1, 0) = msg.data[3];
	R_d_(1, 1) = msg.data[4];
	R_d_(1, 2) = msg.data[5];

	R_d_(2, 0) = msg.data[6];
	R_d_(2, 1) = msg.data[7];
	R_d_(2, 2) = msg.data[8];
}

void UavGeometryControl::odom_cb(const nav_msgs::Odometry &msg)
{
    if (!odometry_start_flag_) odometry_start_flag_ = true;

	x_mv_(0, 0) = msg.pose.pose.position.x;
	x_mv_(1, 0) = msg.pose.pose.position.y;
	x_mv_(2, 0) = msg.pose.pose.position.z;

	v_mv_(0, 0) = msg.twist.twist.linear.x;
	v_mv_(1, 0) = msg.twist.twist.linear.y;
	v_mv_(2, 0) = msg.twist.twist.linear.z;
}

void UavGeometryControl::imu_cb (const sensor_msgs::Imu &msg)
{
    float quaternion[4], euler[3];
    float p, q, r, sx, cx, cy, ty;

    if (!imu_start_flag_) imu_start_flag_ = true;

    quaternion[1] = msg.orientation.x;
    quaternion[2] = msg.orientation.y;
    quaternion[3] = msg.orientation.z;
    quaternion[0] = msg.orientation.w;

    quaternion2euler(quaternion, euler);
    euler_mv_.x = euler[0];
    euler_mv_.y = euler[1];
    euler_mv_.z = euler[2];

    // gyro measurements (p,q,r)
    p = msg.angular_velocity.x;
    q = msg.angular_velocity.y;
    r = msg.angular_velocity.z;

    sx = sin(euler_mv_.x);     // sin(roll)
    cx = cos(euler_mv_.x);     // cos(roll)
    cy = cos(euler_mv_.y);     // cos(pitch)
    ty = tan(euler_mv_.y);     // cos(pitch)

    // conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
    euler_rate_mv_.x = p + sx * ty * q + cx * ty * r;
    euler_rate_mv_.y = cx * q - sx * r;
    euler_rate_mv_.z = sx / cy * q + cx / cy * r;
}

void UavGeometryControl::euler2RotationMatrix(
		const double roll,
		const double pitch,
		const double yaw,
		Matrix<double, 3, 3> &rotMatrix)
{
	rotMatrix.setZero(3, 3);
	rotMatrix(0, 0) = cos(yaw) * cos(pitch);
	rotMatrix(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
	rotMatrix(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
	rotMatrix(1, 0) = sin(yaw) * cos(pitch);
	rotMatrix(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
	rotMatrix(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
	rotMatrix(2, 0) = - sin(pitch);
	rotMatrix(2, 1) = cos(pitch) * sin(roll);
	rotMatrix(2, 2) = cos(pitch) * cos(roll);
}

void UavGeometryControl::hatOperator(
		const double x,
		const double y,
		const double z,
		Matrix<double, 3, 3> &hatMatrix)
{
	hatMatrix.setZero(3,3);
	hatMatrix(0, 1) = -z;
	hatMatrix(0, 2) =  y;
	hatMatrix(1, 0) =  z;
	hatMatrix(1, 2) = -x;
	hatMatrix(2, 0) = -y;
	hatMatrix(2, 1) =  x;
}

void UavGeometryControl::veeOperator(
		Matrix<double, 3, 3> hatMatrix,
		Matrix<double, 3, 1> &veeVector)
{
	veeVector.setZero(3, 1);
	veeVector(0, 0) = hatMatrix(2, 1); 			// x component
	veeVector(1, 0) = hatMatrix(0, 2);			// y component
	veeVector(2, 0) = hatMatrix(1, 0);			// z component
}

void UavGeometryControl::quaternion2euler(float *quaternion, float *euler)
{
  euler[0] = atan2(2 * (quaternion[0] * quaternion[1] +
    quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1]
    + quaternion[2] * quaternion[2]));

  euler[1] = asin(2 * (quaternion[0] * quaternion[2] -
    quaternion[3] * quaternion[1]));

  euler[2] = atan2(2 * (quaternion[0]*quaternion[3] +
    quaternion[1]*quaternion[2]), 1 - 2 * (quaternion[2]*quaternion[2] +
    quaternion[3] * quaternion[3]));
}

int main(int argc, char** argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "geometry_control");

	// Start the control algorithm
	UavGeometryControl geometric_control(CONTROLLER_RATE);
	geometric_control.run();

	return 0;
}

