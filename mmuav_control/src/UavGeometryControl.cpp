/*
 * UavGeometricControl.cpp
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#include <mmuav_control/UavGeometryControl.hpp>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <time.h>

using namespace std;

const double G = 9.81;

// UAV constants
const double UAV_MASS = 2.1;
const double ARM_LENGTH = 0.314;
const double MOMENT_CONSTANT = 0.016;
const double MOTOR_CONSTANT = 8.54858e-06;

// ROTOR constants
const double ROTOR_MASS = 0.01;
const double ROTOR_VELOCITY_SLOWDOWN_SIM = 15;
const double ROTOR_RADIUS = 0.1524;
const double MIN_ROTOR_VELOCITY = 0;
const double MAX_ROTOR_VELOCITY = 1475;
Matrix<double, 3, 3> INERTIA;
const double D =  ARM_LENGTH + ROTOR_RADIUS / 2;

const Matrix<double, 3, 1> E3(0, 0, 1);
Matrix<double, 4, 4> THRUST_TRANSFORM;
Matrix<double, 3, 3> EYE;

const double EPS = 0.01;

// Define control modes
const int POSITION_CONTROL = 1;
const int ATTITUDE_CONTROL = 2;
const int VELOCITY_CONTROL = 3;

// Controller rate
const int CONTROLLER_RATE = 200;

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
	THRUST_TRANSFORM(1, 1) = D;
	THRUST_TRANSFORM(1, 3) = - D;

	// Third row
	THRUST_TRANSFORM(2, 0) = - D;
	THRUST_TRANSFORM(2, 2) = D;

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

	sleep(3);

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
	euler_d_.setZero(3, 1);

	// Initialize measured
	omega_mv_.setZero(3, 1);
	alpha_mv_.setZero(3, 1);
	R_mv_.setZero(3,3);

	// Initialize controller parameters
	// Parameters initialized according to 2010-extended.pdf
	k_x_.setZero(3, 3);
	k_x_(0, 0) = 12.5;
	k_x_(1, 1) = 12.5;
	k_x_(2, 2) = 50;

	k_v_.setZero(3, 3);
	k_v_(0, 0) = 9;
	k_v_(1, 1) = 9;
	k_v_(2, 2) = 20;

	k_R_.setZero(3, 3);
	k_R_(0, 0) = 8;
	k_R_(1, 1) = 8;
	k_R_(2, 2) = 8;

	k_omega_.setZero(3, 3);
	k_omega_(0, 0) = 1.5;
	k_omega_(1, 1) = 1.5;
	k_omega_(2, 2) = 1.5;

	// Initialize subscribers and publishers
	imu_ros_sub_ = node_handle_.subscribe(
			"/uav/imu", 1,
			&UavGeometryControl::imu_cb, this);
	odom_ros_sub_ = node_handle_.subscribe(
			"/uav/odometry", 1,
			&UavGeometryControl::odom_cb, this);
	rotor_ros_pub_ = node_handle_.advertise<mav_msgs::Actuators>(
			"/gazebo/command/motor_speed", 1);
	status_ros_pub_ = node_handle_.advertise<mmuav_msgs::GeomCtlStatus>(
			"/uav/uav_status", 1);

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
	euler_ros_sub_ = node_handle_.subscribe(
			"/uav/euler_desired", 1,
			&UavGeometryControl::euler_cb, this);

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
	}

	// Wait for start flag from odometry callback
	while (!odometry_start_flag_ && ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("UavGeometricControl::run() - "
				"Waiting for first odometry measurement");
	}

	ROS_INFO("UavGeometricControl::run() - "
			"Starting geometric control in 5...");

	t_old_ = ros::Time::now();

	// Attitude error - scalar (PSI)
	Matrix<double, 3, 3> att_err;
	std_msgs::Float64 att_err_msg;

	/*
	 * b3_d - desired thrust vector
	 * M_u 	- control moment
	 * x_des, x_old
	 * b1_des, b1_old
	 * v_d_old
	 */
	Matrix<double, 3, 1> b3_d, M_u, x_old, b1_old, x_des, b1_des, v_d_old;

	/*
	 * R_d 	- desired rotation matrix
	 * R_d_old - used for matrix differentiation
	 * R_d_dot - Desired matrix derivative
	 */
	Matrix<double, 3, 3> R_c, R_c_old, omega_c_old;
	R_c_old = EYE;
	omega_c_old.setZero(3, 3);

	// Rotor velocities control vector
	Matrix<double, 4, 1> rotor_velocities, rotor_signs;

	// Initialize rotor velocity publisher msg
	mav_msgs::Actuators rotor_vel_msg;
	vector<double> velocity_vector(4);
	rotor_vel_msg.angular_velocities = velocity_vector;

	// Total thrust control value
	double f_u;

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

		// Construct current rotation matrix - R
		euler2RotationMatrix(
				euler_mv_.x,
				euler_mv_.y,
				euler_mv_.z,
				R_mv_);
		//cout << R_mv_ << "\n";

		// Construct angular velocity vector
		omega_mv_(0, 0) = euler_rate_mv_.x;
		omega_mv_(1, 0) = euler_rate_mv_.y;
		omega_mv_(2, 0) = euler_rate_mv_.z;

		// Update old time
		t_old_ = ros::Time::now();

		// Position and heading prefilter
		x_des = x_d_; // x_old + 0.025 * (x_d_ - x_old);
		b1_des = b1_old + 0.04 * (b1_d_ - b1_old);

		// TRAJECTORY TRACKING BLOCK
		trajectoryTracking(
				x_des,		// Input - desired position
				x_old,		// Input - old position
				dt,			// Input - time interval
				b3_d,		// OUTPUT - thrust vector
				f_u);		// OUTPUT - total thrust

		// Update old position
		b1_old = b1_des;
		x_old = x_des;
		v_d_old = v_d_;

		// ATTITUDE TRACKING BLOCK
		attitudeTracking(
				b1_des,			// Input - desired heading
				b3_d,			// Input - desired thrust vector
				dt,				// Input - time interval
				R_c_old,
				omega_c_old,
				M_u);			// OUTPUT - control moments

		// Calculate thrust velocities
		Matrix<double, 4, 1> thrust_moment_vec(
				f_u,
				M_u(0, 0),
				M_u(1, 0),
				M_u(2, 0));

		// Convert force vector - THRUST_TRANSFORM * thrus_moment_vec ...
		// ...to angular velocity -> fi = MOTOR_CONSTANT * ang_vel_i^2
		rotor_velocities = THRUST_TRANSFORM * thrust_moment_vec;
		rotor_signs = rotor_velocities.array().sign();
		rotor_velocities = rotor_velocities.array().abs();
		rotor_velocities = rotor_velocities / MOTOR_CONSTANT;
		rotor_velocities = rotor_velocities.array().sqrt();

		// Fill and publish rotor message
		rotor_vel_msg.angular_velocities[0] =
				rotor_signs(0, 0) *
				saturation(
						(double)rotor_velocities(0, 0),
						- MAX_ROTOR_VELOCITY,
						MAX_ROTOR_VELOCITY);
		rotor_vel_msg.angular_velocities[1] =
				rotor_signs(1, 0) *
				saturation(
						(double)rotor_velocities(1, 0),
						- MAX_ROTOR_VELOCITY,
						MAX_ROTOR_VELOCITY);
		rotor_vel_msg.angular_velocities[2] =
				rotor_signs(2, 0) *
				saturation(
						(double)rotor_velocities(2, 0),
						- MAX_ROTOR_VELOCITY,
						MAX_ROTOR_VELOCITY);
		rotor_vel_msg.angular_velocities[3] =
				rotor_signs(3, 0) *
				saturation(
						(double)rotor_velocities(3, 0),
						- MAX_ROTOR_VELOCITY,
						MAX_ROTOR_VELOCITY);

		rotor_ros_pub_.publish(rotor_vel_msg);

		// Calculate attitude error
		att_err = (EYE - R_d_.adjoint() * R_mv_);

		// Construct status msg
		status_msg_.roll_mv = euler_mv_.x;
		status_msg_.roll_sp = euler_d_(0, 0);
		status_msg_.pitch_mv = euler_mv_.y;
		status_msg_.pitch_sp = euler_d_(1, 0);
		status_msg_.yaw_mv = euler_mv_.z;
		status_msg_.yaw_sp = euler_d_(2, 0);
		status_msg_.att_err = att_err.trace() / 2;
		status_msg_.pos_err =sqrt((double)(x_d_ - x_mv_).dot(x_d_ - x_mv_));
		status_msg_.rotor_velocities[0] = rotor_vel_msg.angular_velocities[0];
		status_msg_.rotor_velocities[1] = rotor_vel_msg.angular_velocities[1];
		status_msg_.rotor_velocities[2] = rotor_vel_msg.angular_velocities[2];
		status_msg_.rotor_velocities[3] = rotor_vel_msg.angular_velocities[3];
		status_msg_.moments[0] = M_u(0, 0);
		status_msg_.moments[1] = M_u(1, 0);
		status_msg_.moments[2] = M_u(2, 0);
		status_msg_.x_mv = x_mv_(0, 0);
		status_msg_.y_mv = x_mv_(1, 0);
		status_msg_.z_mv = x_mv_(2, 0);
		status_msg_.x_sp = x_des(0, 0);
		status_msg_.y_sp = x_des(1, 0);
		status_msg_.z_sp = x_des(2, 0);
		std_msgs::Header head;
		head.stamp = ros::Time::now();
		status_msg_.header = head;
		status_ros_pub_.publish(status_msg_);

		//cout << "f_u: \n" << f_u << "\n";
		//cout << "M_u: \n" << M_u << "\n";
		//cout << "Rotor_vel: \n" << rotor_velocities << "\n";
		//cout << "\n\n";
		//cout << endl;
	}
}

void UavGeometryControl::trajectoryTracking(
		const Matrix<double, 3, 1> pos_desired,
		const Matrix<double, 3, 1> pos_old,
		const double dt,
		Matrix<double, 3, 1> &b3_d,
		double &f_u)
{

	// Position errors
	Matrix<double, 3, 1> e_x, e_v;

	// TRAJECTORY TRACKING
	// Calculate total thrust and b3_d (desired thrust vector)
	if (current_control_mode_ == POSITION_CONTROL)
	{
		//cout << "Trajectory: POSITION" << "\n";
		e_x = (x_mv_ - pos_desired);
		e_v = (v_mv_ - v_d_);
	}
	else if (current_control_mode_ == ATTITUDE_CONTROL)
	{
		//cout << "Trajectory: ATTITUDE" << "\n";
		/**
		 * During Attitude control only take z - component of
		 * position and linear velocity.
		 */

		//v_d_ = (x_d_ - pos_old);
		e_x = (x_mv_(2, 0) - x_d_(2, 0)) * E3;
		e_v = (v_mv_(2, 0) - v_d_(2, 0)) * E3;
	}
	else
	{
		ROS_ERROR("Invalid control mode given.");
		throw runtime_error("Invalid control mode given.");
	}

	/*
	 * Transform position and velocity errors.
	Matrix<double, 3, 3> a;
	cout << euler_mv_.z << "\n";
	euler2RotationMatrix(0, 0, euler_mv_.z, a);
	cout << a << "\n";
	*/

	e_x = a * e_x;
	e_v = a * e_v;

	// desired control force for the translational dynamics
	Matrix<double, 3, 1> A =
		- k_x_ * e_x
		- k_v_ * e_v
		+ UAV_MASS * G * E3
		+ UAV_MASS * a_d_;
	f_u = A.dot( R_mv_ * E3 );
	b3_d = A / A.norm();

	b3_d = b3_d / b3_d.norm();
	//cout << b3_d << "\n";

	status_msg_.e_x[0] = (double)e_x(0, 0);
	status_msg_.e_x[1] = (double)e_x(1, 0);
	status_msg_.e_x[2] = (double)e_x(2, 0);

	status_msg_.e_v[0] = (double)e_v(0, 0);
	status_msg_.e_v[1] = (double)e_v(1, 0);
	status_msg_.e_v[2] = (double)e_v(2, 0);
}


void UavGeometryControl::attitudeTracking(
		const Matrix<double, 3, 1> b1_desired,
		const Matrix<double, 3, 1> b3_desired,
		const double dt,
		Matrix<double, 3, 3> &R_c_old,
		Matrix<double, 3, 3> &omega_c_old,
		Matrix<double, 3, 1> &M_u)
{

	// Attitude errors
	Matrix<double, 3, 1> e_omega, e_R;

	// Auxiliary skew matrices
	Matrix<double, 3, 3> e_R_skew, omega_mv_skew, omega_c_skew,
						 alpha_c_skew;

	if (current_control_mode_ == POSITION_CONTROL)
	{
		//cout << "Attitude: POSITION" << "\n";
		/**
		 * During position control desired rotation, angular velocity
		 * and angular acceleration matrices will be CALCULATED.
		 * R_c, omega_c, alpha_c
		 */
		Matrix<double, 3, 3> R_c;
		Matrix<double, 3, 1> b1_c, b2_c;

		/*
		 * b13_normal - Normal of plane spanned by b3_d and b1_d.
		 *
		 * Note: b1_d will not necessarily lie in the plane with b3_d normal,
		 * it is needed to calculate it's projection to that plane.
		 */
		Matrix<double, 3, 1> b13_normal = b3_desired.cross(b1_desired);

		// Compute b1_c = Proj[b1_d] onto the plane with normal b3_d
		b1_c = - b3_desired.cross(b13_normal) / b13_normal.norm();

		// Construct desired rotation matrix
		b2_c = b3_desired.cross(b1_c);
		b2_c = b2_c / b2_c.norm();
		R_c.setZero(3, 3);
		R_c << b1_c, b2_c, b3_desired;

		// calculateDesiredAngVelAcc(R_c, R_c_old, R_mv_, omega_c_old, dt);

		// Remap calculated to desired
		R_d_ = R_c;

		cout << "R_d:\n" << R_d_ << "\n";
		cout << "R_mv:\n" << R_mv_ << "\n";

		// Update old R_c
		R_c_old = R_c;
	}
	else if (current_control_mode_ == ATTITUDE_CONTROL)
	{
		//cout << "Attitude: ATTITUDE" << "\n";
		// Do nothing here - read desired attitude values from
		// callback functions.
		euler2RotationMatrix(
				(double)euler_d_(0,0),
				(double)euler_d_(1,0),
				(double)euler_d_(2,0),
				R_d_);
	}
	else
	{
		ROS_ERROR("Invalid control mode given.");
		throw std::runtime_error("Invalid control mode given.");
	}

	// ATTITUDE TRACKING
	// Calculate control moment M
	e_R_skew = (R_d_.adjoint() * R_mv_ - R_mv_.adjoint() * R_d_) / 2;
	veeOperator(e_R_skew, e_R);
	e_omega = (omega_mv_ - R_mv_.adjoint() * R_d_ * omega_d_);
	if (e_omega(0, 0) != e_omega(0, 0))
	{
		throw std::runtime_error("STOP");
	}
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

	double sat = 100;
	M_u(0, 0) = saturation((double)M_u(0, 0), -sat, sat);
	M_u(1, 0) = saturation((double)M_u(1, 0), -sat, sat);
	M_u(2, 0) = saturation((double)M_u(2, 0), -sat, sat);

	status_msg_.e_R[0] = e_R(0, 0);
	status_msg_.e_R[1] = e_R(1, 0);
	status_msg_.e_R[2] = e_R(2, 0);

	status_msg_.e_omega[0] = e_omega(0, 0);
	status_msg_.e_omega[1] = e_omega(1, 0);
	status_msg_.e_omega[2] = e_omega(2, 0);
}

void UavGeometryControl::calculateDesiredAngVelAcc(
		const Matrix<double, 3, 3> R_c,
		const Matrix<double, 3, 3> R_c_old,
		const Matrix<double, 3, 3> R_mv,
		Matrix<double, 3, 3> &omega_c_old,
		double dt)
{
	Matrix<double, 3, 3> omega_c_skew, alpha_c_skew;


	// Calculate angular velocity based on R_c[k] and R_c[k-1]
	Matrix<double, 3, 3> AA = R_c * R_c_old.adjoint();
	double theta = acos((AA.trace() - 1 ) / 2);
	omega_c_skew = (AA - AA.adjoint()) * theta / (2 * sin(theta));
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
		alpha_c_skew = (omega_c_skew - omega_c_old) / dt;
		omega_c_old = omega_c_skew;
	}


	/*
	 * Alternate way
	Matrix<double, 3, 3> R_c_dot = (R_c - R_c_old);
	omega_c_skew = R_c_dot * R_c.adjoint();
	 */

	// Remap calculated values to desired
	veeOperator(omega_c_skew, omega_d_);
	//veeOperator(alpha_c_skew, alpha_d_);
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
	b1_d_ = b1_d_ / b1_d_.norm();
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

void UavGeometryControl::euler_cb(const geometry_msgs::Vector3 &msg)
{
	euler_d_(0,0) = msg.x;
	euler_d_(1,0) = msg.y;
	euler_d_(2,0) = msg.z;
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

    //if (euler_mv_.z < 0) { euler_mv_.z += 2 * 3.14; }

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

double UavGeometryControl::saturation(
		double value,
		double lowLimit,
		double highLimit)
{
	if (value > highLimit) { return highLimit; }
	else if (value < lowLimit) { return lowLimit; }
	else { return value; }
}

double UavGeometryControl::deadzone(
		double value,
		double lowLimit,
		double highLimit)
{
	if (value < highLimit && value > lowLimit) { return 0; }
	else { return value; }
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

