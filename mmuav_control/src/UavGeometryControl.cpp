/*
 * UavGeometricControl.cpp
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#include <mmuav_control/UavGeometryControl.hpp>
#include <mmuav_control/MmuavParameters.hpp>

#include <mav_msgs/Actuators.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <time.h>

using namespace std;

Matrix<double, 3, 3> INERTIA;
Matrix<double, 3, 3> MASS_INERTIA;
double UAV_MASS = 2.083;

// Transform matrix with roll / pitch corrections
Matrix<double, 4, 4> THRUST_TRANSFORM_FULL;

// Transform matrix without roll / pitch corrections
Matrix<double, 4, 4> THRUST_TRANSFORM_YAW;

const Matrix<double, 3, 1> E3(0, 0, 1);
Matrix<double, 3, 3> EYE3;
Matrix<double, 4, 4> EYE4;

// Define control modes
const int POSITION_CONTROL = 1;
const int ATTITUDE_CONTROL = 2;
const int VELOCITY_CONTROL = 3;

// Controller rate
const int CONTROLLER_RATE = 100;

UavGeometryControl::UavGeometryControl(int rate, std::string uav_ns)
{
	// Initialize controller variables
	controller_rate_ = rate;
	sleep_duration_ = 0.5;
	imu_start_flag_ = false;
	pose_start_flag_ = false;
	velocity_start_flag_ = false;
	param_start_flag_ = false;
	enable_mass_control_ = false;
	current_control_mode_ = POSITION_CONTROL;
	this->uav_ns = uav_ns;

	// Initialize inertia matrix
	INERTIA.setZero(3, 3);
	INERTIA(0, 0) = 0.0826944;
	INERTIA(1, 1) = 0.0826944;
	INERTIA(2, 2) = 0.0104;

	MASS_INERTIA.setZero(3, 3);
	MASS_INERTIA(0, 0) = MM_MASS * 0.085 * 0.085;
	MASS_INERTIA(1, 1) = MM_MASS * 0.085 * 0.085;
	MASS_INERTIA(2, 2) = MM_MASS * 0.085 * 0.085;

	// Initialize eye(3) matrix
	EYE3.setZero(3, 3);
	EYE3(0, 0) = 1;
	EYE3(1, 1) = 1;
	EYE3(2, 2) = 1;

	// Initialize eye(4) matrix
	EYE4.setZero(4, 4);
	EYE4(0, 0) = 1;
	EYE4(3, 3) = 1;

	// Initialize thrust transform matrix
	THRUST_TRANSFORM_FULL.setZero(4, 4);

	// First row
	THRUST_TRANSFORM_FULL(0, 0) = 1;
	THRUST_TRANSFORM_FULL(0, 1) = 1;
	THRUST_TRANSFORM_FULL(0, 2) = 1;
	THRUST_TRANSFORM_FULL(0, 3) = 1;

	// Second row
	THRUST_TRANSFORM_FULL(1, 1) = D;
	THRUST_TRANSFORM_FULL(1, 3) = - D;

	// Third row
	THRUST_TRANSFORM_FULL(2, 0) = - D;
	THRUST_TRANSFORM_FULL(2, 2) = D;

	// Fourth row
	THRUST_TRANSFORM_FULL(3, 0) = MOMENT_CONSTANT;
	THRUST_TRANSFORM_FULL(3, 1) = - MOMENT_CONSTANT;
	THRUST_TRANSFORM_FULL(3, 2) = MOMENT_CONSTANT;
	THRUST_TRANSFORM_FULL(3, 3) = - MOMENT_CONSTANT;

	// Invert the matrix
	THRUST_TRANSFORM_FULL = THRUST_TRANSFORM_FULL.inverse().eval();
	THRUST_TRANSFORM_YAW = THRUST_TRANSFORM_FULL * EYE4;

	cout << "Thrust transform: \n";
	cout << THRUST_TRANSFORM_FULL << "\n";
	cout << THRUST_TRANSFORM_YAW << "\n";
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
	euler_d_.setZero(3, 1);

	// Initialize measured
	omega_mv_.setZero(3, 1);
	alpha_mv_.setZero(3, 1);
	R_mv_.setZero(3,3);

	// Initialize controller parameters
	// Parameters initialized according to 2010-extended.pdf
	k_x_.setZero(3, 3);
	k_x_(0, 0) = 7.2;
	k_x_(1, 1) = 7.2;
	k_x_(2, 2) = 50;

	k_v_.setZero(3, 3);
	k_v_(0, 0) = 2.6;
	k_v_(1, 1) = 2.6;
	k_v_(2, 2) = 20;

	k_R_.setZero(3, 3);
	k_R_(0, 0) = 1.52;
	k_R_(1, 1) = 1.52;
	k_R_(2, 2) = 12;

	k_omega_.setZero(3, 3);
	k_omega_(0, 0) = 0.65;
	k_omega_(1, 1) = 0.65;
	k_omega_(2, 2) = 1.54;

	// Initialize subscribers and publishers
	imu_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/imu", 1,
			&UavGeometryControl::imu_cb, this);
	pose_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/pose", 1,
			&UavGeometryControl::pose_cb, this);
	velocity_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/velocity_relative", 1,
			&UavGeometryControl::vel_cb, this);
	rotor_ros_pub_ = node_handle_.advertise<mav_msgs::Actuators>(
			"/gazebo/command/motor_speed", 1);
	status_ros_pub_ = node_handle_.advertise<mmuav_msgs::GeomCtlStatus>(
			"/" + uav_ns + "/uav_status", 1);

	// Initialize position reference subscribers
	xd_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/x_desired", 1,
			&UavGeometryControl::xd_cb, this);
	vd_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/v_desired", 1,
			&UavGeometryControl::vd_cb, this);
	ad_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/a_desired", 1,
			&UavGeometryControl::ad_cb, this);

	// Initialize attitude reference subscribers
	b1d_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/b1_desired", 1,
			&UavGeometryControl::b1d_cb, this);
	omega_d_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/omega_desired", 1,
			&UavGeometryControl::omegad_cb, this);
	alpha_d_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/alpha_desired", 1,
			&UavGeometryControl::alphad_cb, this);
	rd_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/R_desired", 1,
			&UavGeometryControl::rd_cb, this);
	euler_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/euler_desired", 1,
			&UavGeometryControl::euler_cb, this);

	// Control mode subscriber
	ctl_mode_ros_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/control_mode", 1,
			&UavGeometryControl::ctl_mode_cb, this);

	// Initialize dynamic reconfigure
	param_callback_ = boost::bind(
			&UavGeometryControl::param_cb, this, _1, _2);
	server_.setCallback(param_callback_);
}

UavGeometryControl::~UavGeometryControl()
{
	// TODO(lmark): Destructor..
}

void UavGeometryControl::runControllerLoop()
{
	/**
	 * Initialize time variables used for calculating
	 * time intervals.
	 */
	double dt;
	ros::Time t_old = ros::Time::now();

	/*
	 * b3_d 			- desired thrust vector
	 * b1_des, b1_old 	- desired heading, old heading
	 */
	Matrix<double, 3, 1> b3_d, b1_old, b1_des;
	b1_old = b1_d_;

	/*
	 * R_d 		- desired rotation matrix
	 * R_d_old 	- used for matrix differentiation
	 * R_d_dot 	- Desired matrix derivative
	 */
	Matrix<double, 3, 3> R_c;
	R_c_old = EYE3;
	R_c_dot_old.setZero(3, 3);
	omega_c_old.setZero(3, 3);
	x_dot_old.setZero(3, 1);

	/**
	 * Controller outputs:
	 * 	- f_u: Total control force
	 * 	- M_u: Total control moment
	 */
	double f_u;
	Matrix<double, 3, 1> M_u;

	// Perform sensor checks
	blockingSensorChecks();
	ROS_INFO("UavGeometricControl::run() - "
			"Starting geometric control in 5...");
	sleep(5);

	R_mv_ = EYE3;
	Matrix<double, 3, 3> R_mv_old = EYE3;

	// Initialize helper time variables
	double dt_help = 0.1;
	double counter = 0.0;

	// Start the control loop.
	while (ros::ok())
	{
		// Do 1 round of callbacks
		ros::spinOnce();

		// Calculate time difference
		double current_time = ros::Time::now().toSec();
		dt = current_time - t_old.toSec();

		// Check if time is right
		if (dt < 1.0 / controller_rate_)
			continue;

		/*
		 *	Initialize auxiliary counter used
		 *	for calculations happening at a different
		 *	rate than the control loop.
		 */
		counter += dt;
		if (counter >= dt_help)
		{
			calc_desired = true;
			counter = 0.0;
		}

		// Update old time
		t_old = ros::Time::now();

		// Assign angular velocities
		omega_mv_(0, 0) = euler_rate_mv_.x;
		omega_mv_(1, 0) = euler_rate_mv_.y;
		omega_mv_(2, 0) = euler_rate_mv_.z;

		// Construct current rotation matrix - R
		euler2RotationMatrix(
				euler_mv_.x,
				euler_mv_.y,
				euler_mv_.z,
				R_mv_);

		// Calculate center of mass
		calculateCenterOfMass();

		// Position and heading prefilter
		b1_des = b1_old + 0.05 * (b1_d_ - b1_old);

		// TRAJECTORY TRACKING BLOCK
		trajectoryTracking(
				x_d_,		// Input - desired position
				b3_d,		// OUTPUT - thrust vector
				f_u);		// OUTPUT - total thrust

		// Update old position
		b1_old = b1_des;

		// ATTITUDE TRACKING BLOCK
		attitudeTracking(
				b1_des,			// Input - desired heading
				b3_d,			// Input - desired thrust vector
				dt,				// Input - time interval
				R_c_old,
				omega_c_old,
				M_u);			// OUTPUT - control moments

		// Disable calculating desired velocities / accelerations
		calc_desired = false;

		// Publish control inputs
		publishControlInputs(f_u, M_u);

		// Publish controller status
		publishStatusMessage(f_u, M_u);
	}
}

void UavGeometryControl::publishStatusMessage(
		double f_u,
		Matrix<double, 3, 1> M_u)
{
	// Attitude error - scalar (PSI)
	Matrix<double, 3, 3> att_err = (EYE3 - R_d_.adjoint() * R_mv_);

	// Construct status msg
	std_msgs::Header head;
	head.stamp = ros::Time::now();
	status_msg_.header = head;
	status_msg_.force = f_u;
	status_msg_.roll_mv = euler_mv_.x;
	status_msg_.roll_sp = euler_d_(0, 0);
	status_msg_.pitch_mv = euler_mv_.y;
	status_msg_.pitch_sp = euler_d_(1, 0);
	status_msg_.yaw_mv = euler_mv_.z;
	status_msg_.yaw_sp = euler_d_(2, 0);
	status_msg_.att_err = att_err.trace() / 2;
	status_msg_.pos_err =sqrt((double)(x_d_ - x_mv_).dot(x_d_ - x_mv_));
	status_msg_.moments[0] = M_u(0, 0);
	status_msg_.moments[1] = M_u(1, 0);
	status_msg_.moments[2] = M_u(2, 0);
	status_msg_.x_mv = x_mv_(0, 0);
	status_msg_.y_mv = x_mv_(1, 0);
	status_msg_.z_mv = x_mv_(2, 0);
	status_msg_.x_sp = x_d_(0, 0);
	status_msg_.y_sp = x_d_(1, 0);
	status_msg_.z_sp = x_d_(2, 0);
	status_msg_.a_d[0] = a_d_(0, 0);
	status_msg_.a_d[1] = a_d_(1, 0);
	status_msg_.a_d[2] = a_d_(2, 0);

	status_msg_.v_d[0] = v_d_(0, 0);
	status_msg_.v_d[1] = v_d_(1, 0);
	status_msg_.v_d[2] = v_d_(2, 0);

	status_msg_.b1_mv[0] = R_mv_(0,0);
	status_msg_.b1_mv[1] = R_mv_(1,0);
	status_msg_.b1_mv[2] = R_mv_(2,0);
	status_msg_.omega_d[0] = omega_d_(0, 0);
	status_msg_.omega_d[1] = omega_d_(1, 0);
	status_msg_.omega_d[2] = omega_d_(2, 0);
	status_msg_.alpha_d[0] = alpha_d_(0, 0);
	status_msg_.alpha_d[1] = alpha_d_(1, 0);
	status_msg_.alpha_d[2] = alpha_d_(2, 0);

	status_msg_.omega_mv[0] = omega_mv_(0, 0);
	status_msg_.omega_mv[1] = omega_mv_(1, 0);
	status_msg_.omega_mv[2] = omega_mv_(2, 0);

	if (enable_mass_control_)
	{
		status_msg_.mass_offset[0] = mass0_mv_;
		status_msg_.mass_offset[1] = mass1_mv_;
		status_msg_.mass_offset[2] = mass2_mv_;
		status_msg_.mass_offset[3] = mass3_mv_;
	}

	status_msg_.r_cm[0] = ro_cm_(0, 0);
	status_msg_.r_cm[1] = ro_cm_(1, 0);
	status_msg_.r_cm[2] = ro_cm_(2, 0);

	status_ros_pub_.publish(status_msg_);
}

void UavGeometryControl::publishControlInputs(
		double f_u,
		Matrix<double, 3, 1> M_u)
{
	// Calculate thrust velocities
	Matrix<double, 4, 1> thrust_moment_vec(
			f_u,
			M_u(0, 0),
			M_u(1, 0),
			M_u(2, 0));

	// Initialize mass control messages
	std_msgs::Float64 mass0_msg, mass1_msg, mass2_msg, mass3_msg;

	// Rotor velocities
	Matrix<double, 4, 1> rotor_velocities;

	// Calculate control inputs
	if (enable_mass_control_)
	{
		// Calculate height and yaw control
		calculateRotorVelocities(
				thrust_moment_vec,
				THRUST_TRANSFORM_YAW,
				rotor_velocities);

		// Roll and pitch control with masses
		double dx = (double)M_u(1, 0) / (2 * MM_FORCE * E3.dot(R_mv_ * E3));
		double dy = (double)M_u(0, 0) / (2 * MM_FORCE * E3.dot(R_mv_ * E3));

		dx = saturation(dx, -ARM_LENGTH / 2, ARM_LENGTH / 2);
		dy = saturation(dy, -ARM_LENGTH / 2, ARM_LENGTH / 2);

		// cout << "dx: " << dx << "\n";
		// cout << "dy: " << dy << "\n";

		mass0_msg.data = dx;
		mass1_msg.data = - dy;
		mass2_msg.data = - dx;
		mass3_msg.data = dy;

		// publish mass command values
		mass0_cmd_pub_.publish(mass0_msg);
		mass1_cmd_pub_.publish(mass1_msg);
		mass2_cmd_pub_.publish(mass2_msg);
		mass3_cmd_pub_.publish(mass3_msg);
	}
	else if (enable_manipulator_control_)
	{
		// TODO: Manipulator control - publish dx and dy somewhere...
		// Calculate height and yaw control
		calculateRotorVelocities(
				thrust_moment_vec,
				THRUST_TRANSFORM_YAW,
				rotor_velocities);

		// Roll and pitch control with masses
		double dx = (double)M_u(1, 0) / (2 * PAYLOAD_FORCE * E3.dot(R_mv_ * E3));
		double dy = (double)M_u(0, 0) / (2 * PAYLOAD_FORCE * E3.dot(R_mv_ * E3));
		dx = saturation(dx, -0.15, 0.15);
		dy = saturation(dy, -0.15, 0.15);

		geometry_msgs::Point msg;
		msg.x = dx;
		msg.y = - dy;
		payload_pos_pub_.publish(msg);

	}
	else
	{
		// Calculate full rotor control
		calculateRotorVelocities(
				thrust_moment_vec,
				THRUST_TRANSFORM_FULL,
				rotor_velocities);
	}

	// Initialize rotor velocity publisher msg
	mav_msgs::Actuators rotor_vel_msg;
	vector<double> velocity_vector(4);
	rotor_vel_msg.angular_velocities = velocity_vector;

	// Fill and publish rotor message
	rotor_vel_msg.angular_velocities[0] = (double)rotor_velocities(0, 0);
	rotor_vel_msg.angular_velocities[1] = (double)rotor_velocities(1, 0);
	rotor_vel_msg.angular_velocities[2] = (double)rotor_velocities(2, 0);
	rotor_vel_msg.angular_velocities[3] = (double)rotor_velocities(3, 0);
	rotor_ros_pub_.publish(rotor_vel_msg);

	status_msg_.rotor_velocities[0] = rotor_vel_msg.angular_velocities[0];
	status_msg_.rotor_velocities[1] = rotor_vel_msg.angular_velocities[1];
	status_msg_.rotor_velocities[2] = rotor_vel_msg.angular_velocities[2];
	status_msg_.rotor_velocities[3] = rotor_vel_msg.angular_velocities[3];
}


void UavGeometryControl::calculateCenterOfMass()
{
	if (enable_mass_control_)
	{
		// Calculate center of mass relative to the body frame
		ro_cm_(0, 0) = (MM_MASS * mass0_mv_ + MM_MASS * ( -mass2_mv_)) / UAV_MASS;
		ro_cm_(1, 0) = (MM_MASS * mass1_mv_ + MM_MASS * (- mass3_mv_)) / UAV_MASS;
		ro_cm_(2, 0) = 0;
	}
	else if (enable_manipulator_control_)
	{
		ro_cm_ = (
				PAYLOAD_MASS * gripperLeft_mv_ +
				PAYLOAD_MASS * gripperRight_mv_
				) / UAV_MASS;
	}
	else
	{
		ro_cm_(0, 0) = 0;
		ro_cm_(1, 0) = 0;
		ro_cm_(2, 0) = 0;
	}

}

void UavGeometryControl::calculateRotorVelocities(
		Matrix<double, 4, 1> thrust_moment_vec,
		Matrix<double, 4, 4> transform_matrix,
		Matrix<double, 4, 1>& rotor_velocities)
{
	Matrix<double, 4, 1> rotor_signs;

	// Convert force vector - THRUST_TRANSFORM * thrus_moment_vec ...
	// ...to angular velocity -> fi = MOTOR_CONSTANT * ang_vel_i^2
	rotor_velocities.setZero(4, 1);
	rotor_velocities = transform_matrix * thrust_moment_vec;
	rotor_signs = rotor_velocities.array().sign();
	rotor_velocities = rotor_velocities.array().abs();
	rotor_velocities = rotor_velocities / MOTOR_CONSTANT;
	rotor_velocities = rotor_velocities.array().sqrt();

	rotor_velocities(0, 0) =
			rotor_signs(0, 0) *
			saturation(
					(double)rotor_velocities(0, 0),
					- MAX_ROTOR_VELOCITY,
					MAX_ROTOR_VELOCITY);
	rotor_velocities(1, 0) =
			rotor_signs(1, 0) *
			saturation(
					(double)rotor_velocities(1, 0),
					- MAX_ROTOR_VELOCITY,
					MAX_ROTOR_VELOCITY);
	rotor_velocities(2, 0) =
			rotor_signs(2, 0) *
			saturation(
					(double)rotor_velocities(2, 0),
					- MAX_ROTOR_VELOCITY,
					MAX_ROTOR_VELOCITY);

	rotor_velocities(3, 0) =
			rotor_signs(3, 0) *
			saturation(
					(double)rotor_velocities(3, 0),
					- MAX_ROTOR_VELOCITY,
					MAX_ROTOR_VELOCITY);

}

void UavGeometryControl::trajectoryTracking(
		const Matrix<double, 3, 1> pos_desired,
		Matrix<double, 3, 1> &b3_d,
		double &f_u)
{

	// Position errors
	Matrix<double, 3, 1> e_x, e_v;

	// TRAJECTORY TRACKING
	// Calculate total thrust and b3_d (desired thrust vector)
	if (current_control_mode_ == POSITION_CONTROL)
	{
		e_x = (x_mv_ - pos_desired);
		e_v = (v_mv_ - v_d_);
	}
	else if (current_control_mode_ == ATTITUDE_CONTROL)
	{
		/**
		 * During Attitude control only take z - component of
		 * position and linear velocity.
		 */
		e_x = (x_mv_(2, 0) - pos_desired(2, 0)) * E3;
		e_v = (v_mv_(2, 0) - v_d_(2, 0)) * E3;
	}
	else
	{
		ROS_ERROR("Invalid control mode given.");
		throw runtime_error("Invalid control mode given.");
	}

	// desired control force for the translational dynamics
	Matrix<double, 3, 1> A =
		- k_x_ * e_x
		- k_v_ * e_v
		+ UAV_MASS * G * E3
		+ UAV_MASS * a_d_;

	if (enable_mass_control_ || enable_manipulator_control_)
	{
		Matrix<double, 3, 3> skew_omega, skew_ro;
		hatOperator(
			(double)omega_mv_(0, 0),
			(double)omega_mv_(1, 0),
			(double)omega_mv_(2, 0),
			skew_omega);
		hatOperator(
			(double)ro_cm_(0, 0),
			(double)ro_cm_(1, 0),
			(double)ro_cm_(2, 0),
			skew_ro);
		Matrix<double, 3, 1> additionalDynamics =
				- UAV_MASS * (R_mv_ * ro_cm_).cross(alpha_d_)
				- UAV_MASS * R_mv_ * skew_omega * skew_ro * omega_mv_;
		// cout << "Add to A: \n" << add << "\n";
		A = A + additionalDynamics;
	}
	f_u = A.dot( R_mv_ * E3 );
	b3_d = A / A.norm();

	status_msg_.e_x[0] = (double)e_x(0, 0);
	status_msg_.e_x[1] = (double)e_x(1, 0);
	status_msg_.e_x[2] = (double)e_x(2, 0);

	status_msg_.e_v[0] = (double)e_v(0, 0);
	status_msg_.e_v[1] = (double)e_v(1, 0);
	status_msg_.e_v[2] = (double)e_v(2, 0);
}

void UavGeometryControl::blockingSensorChecks()
{
	ROS_INFO("UavGeometricControl::blockingSensorChecks() - "
			"Waiting for first clock message");
	// Wait for the ROS time server
	while (ros::Time::now().toSec() == 0 && ros::ok())
	{
		ros::spinOnce();
	}
	ROS_INFO("UavGeometricControl::blockingSensorChecks() - "
			"Received first clock message");

	ROS_INFO("UavGeometricControl::blockingSensorChecks() - "
			"Waiting for first IMU message");
	// Wait for start flag from IMU callback
	while (!imu_start_flag_ && ros::ok())
	{
		ros::spinOnce();
	}
	ROS_INFO("UavGeometricControl::blockingSensorChecks() - "
			"Received first IMU message");

	ROS_INFO("UavGeometricControl::blockingSensorChecks() - "
			"Waiting for first Pose message");
	// Wait for start flag from pose callback
	while (!pose_start_flag_ && ros::ok())
	{
		ros::spinOnce();
	}
	ROS_INFO("UavGeometricControl::blockingSensorChecks() - "
			"Received first Pose message");

	ROS_INFO("UavGeometricControl::blockingSensorChecks() - "
			"Waiting for first Velocity message");
	// Wait for start flag from velocity callback
	while (!velocity_start_flag_ && ros::ok())
	{
		ros::spinOnce();
	}
	ROS_INFO("UavGeometricControl::blockingSensorChecks() - "
			"Received first Velocity message");
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

		status_msg_.b1_d[0] = b1_c(0, 0);
		status_msg_.b1_d[1] = b1_c(1, 0);
		status_msg_.b1_d[2] = b1_c(2, 0);

		// Construct desired rotation matrix
		b2_c = b3_desired.cross(b1_c);
		b2_c = b2_c / b2_c.norm();
		R_c.setZero(3, 3);
		R_c << b1_c, b2_c, b3_desired;

		calculateDesiredAngularVelAndAcc(R_c);

		// Remap calculated to desired
		R_d_ = R_c;

		// Update old R_c
		// R_c_old = R_c;
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

	// cout << "R_d:\n" << R_d_ << "\n";
	// cout << "R_mv:\n" << R_mv_ << "\n";

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

	Matrix<double, 3,3> adjustedInertia;
	Matrix<double, 3, 1> additionalDynamics;
	adjustedInertia = INERTIA;

	// Adjust inertia matrix
	if (enable_mass_control_)
	{
		adjustedInertia(0, 0) = adjustedInertia(0, 0)
				+ mass1_mv_ * mass1_mv_ * MM_MASS
				+ mass3_mv_ * mass3_mv_ * MM_MASS
				+ MASS_INERTIA(0, 0);

		adjustedInertia(1, 1) = adjustedInertia(1, 1)
				+ mass0_mv_ * mass0_mv_ * MM_MASS
				+ mass2_mv_ * mass2_mv_ * MM_MASS
				+ MASS_INERTIA(1, 1);

		adjustedInertia(2, 2) = adjustedInertia(2, 2)
				+ mass1_mv_ * mass1_mv_ * MM_MASS
				+ mass3_mv_ * mass3_mv_ * MM_MASS
				+ mass0_mv_ * mass0_mv_ * MM_MASS
				+ mass2_mv_ * mass2_mv_ * MM_MASS
				+ 4 * MASS_INERTIA(2,2);

		additionalDynamics =
				UAV_MASS * ro_cm_.cross(R_mv_.adjoint()*a_d_);
	}
	else if (enable_manipulator_control_)
	{
		adjustedInertia(0, 0) = adjustedInertia(0, 0)
				+ (gripperLeft_mv_(1,0) * gripperLeft_mv_(1,0) +
						gripperLeft_mv_(2,0) * gripperLeft_mv_(2,0)
						) * PAYLOAD_MASS
				+ (gripperRight_mv_(1,0) * gripperRight_mv_(1,0) +
						gripperLeft_mv_(2,0) * gripperRight_mv_(2,0)
						) * PAYLOAD_MASS;

		adjustedInertia(1, 1) = adjustedInertia(1, 1)
						+ (gripperLeft_mv_(0,0) * gripperLeft_mv_(0,0) +
								gripperLeft_mv_(2,0) * gripperLeft_mv_(2,0)
								) * PAYLOAD_MASS * 2
						+ (gripperRight_mv_(0,0) * gripperRight_mv_(0,0) +
								gripperLeft_mv_(2,0) * gripperRight_mv_(2,0)
								) * PAYLOAD_MASS;

		adjustedInertia(2, 2) = adjustedInertia(2, 2)
						+ (gripperLeft_mv_(0,0) * gripperLeft_mv_(0,0) +
								gripperLeft_mv_(0,0) * gripperLeft_mv_(0,0)
								) * PAYLOAD_MASS * 2
						+ (gripperRight_mv_(2,0) * gripperRight_mv_(2,0) +
								gripperLeft_mv_(2,0) * gripperRight_mv_(2,0)
								) * PAYLOAD_MASS;

		additionalDynamics =
						UAV_MASS * ro_cm_.cross(R_mv_.adjoint()*a_d_);
	}


	M_u = 	- k_R_ * e_R
		- k_omega_ * e_omega
		+ omega_mv_.cross(adjustedInertia * omega_mv_)
		- adjustedInertia *
		(
			omega_mv_skew * R_mv_.adjoint() * R_d_ * omega_d_
			- R_mv_.adjoint() * R_d_ * alpha_d_
		) + additionalDynamics;

	M_u(0, 0) = saturation((double)M_u(0, 0), -5, 5);
	M_u(1, 0) = saturation((double)M_u(1, 0), -5, 5);
	M_u(2, 0) = saturation((double)M_u(2, 0), -2.5, 2.5);

	status_msg_.e_R[0] = e_R(0, 0);
	status_msg_.e_R[1] = e_R(1, 0);
	status_msg_.e_R[2] = e_R(2, 0);

	status_msg_.e_omega[0] = e_omega(0, 0);
	status_msg_.e_omega[1] = e_omega(1, 0);
	status_msg_.e_omega[2] = e_omega(2, 0);
}

void UavGeometryControl::calculateDesiredAngularVelAndAcc(
		const Matrix<double, 3, 3> R_c)
{
	Matrix<double, 3, 3> omega_c_skew, alpha_c_skew;

	if (!calc_desired)	{ return; }

	Matrix<double, 3, 3> R_c_dot = (R_c - R_c_old) / 0.1;
	omega_c_skew = R_c.adjoint() * R_c_dot;

	Matrix<double, 3, 3> R_c_ddot = (R_c_dot - R_c_dot_old) / 0.1;
	alpha_c_skew = - omega_c_skew * omega_c_skew + R_c.adjoint() * R_c_ddot;

	// Remap calculated values to desired
	veeOperator(omega_c_skew, omega_d_);
	veeOperator(alpha_c_skew, alpha_d_);

	/*
	omega_d_(0, 0) = saturation((double)omega_d_(0, 0), -1, 1);
	omega_d_(1, 0) = saturation((double)omega_d_(1, 0), -1, 1);
	omega_d_(2, 0) = saturation((double)omega_d_(2, 0), -1, 1);
	*/
	alpha_d_(0, 0) = saturation((double)alpha_d_(0, 0), -0.5, 0.5);
	alpha_d_(1, 0) = saturation((double)alpha_d_(1, 0), -0.5, 0.5);
	alpha_d_(2, 0) = saturation((double)alpha_d_(2, 0), -0.5, 0.5);

	// cout << "omega_d: " << "\n" << omega_d_ << "\n";
	// cout << "alpha_d: " << "\n" << alpha_d_ << "\n";

	R_c_old = R_c;
	R_c_dot_old = R_c_dot;
}

void UavGeometryControl::param_cb(
		mmuav_control::UavGeometryControlParamsConfig &config,
		uint32_t level)
{

	cout << "hello";
	if (!param_start_flag_)
	{
		// Set parameters for the first time

		config.kx_xy = k_x_(0, 0);
		config.kx_z = k_x_(2, 2);
		config.kv_xy = k_v_(0, 0);
		config.kv_z = k_v_(2, 2);
		config.kR_xy = k_R_(0, 0);
		config.kR_z = k_R_(2, 2);
		config.kOm_xy = k_omega_(0, 0);
		config.kOm_z = k_omega_(2, 2);

		param_start_flag_ = true;
		server_.updateConfig(config);
	}
	else
	{
		// Update parameters

		k_x_(0, 0) = config.kx_xy;
		k_x_(1, 1) = config.kx_xy;
		k_x_(2, 2) = config.kx_z;

		k_v_(0, 0) = config.kv_xy;
		k_v_(1, 1) = config.kv_xy;
		k_v_(2, 2) = config.kv_z;

		k_R_(0, 0) = config.kR_xy;
		k_R_(1, 1) = config.kR_xy;
		k_R_(2, 2) = config.kR_z;

		k_omega_(0, 0) = config.kOm_xy;
		k_omega_(1, 1) = config.kOm_xy;
		k_omega_(2, 2) = config.kOm_z;
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

void UavGeometryControl::pose_cb(const geometry_msgs::PoseStamped &msg)
{
	if (!pose_start_flag_) pose_start_flag_ = true;

	x_mv_(0, 0) = msg.pose.position.x;
	x_mv_(1, 0) = msg.pose.position.y;
	x_mv_(2, 0) = msg.pose.position.z;
}

void UavGeometryControl::vel_cb(const geometry_msgs::TwistStamped &msg)
{
	if (!velocity_start_flag_) velocity_start_flag_ = true;

	v_mv_(0, 0) = cos(euler_mv_.z) * msg.twist.linear.x
				- sin(euler_mv_.z) * msg.twist.linear.y;
	v_mv_(1, 0) = sin(euler_mv_.z) * msg.twist.linear.x
				+ cos(euler_mv_.z) * msg.twist.linear.y;
	v_mv_(2, 0) = msg.twist.linear.z;
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

void UavGeometryControl::mass0_cb(
		const control_msgs::JointControllerState &msg)
{
	mass0_mv_ = ARM_LENGTH / 2.0 + msg.process_value;
}

void UavGeometryControl::mass1_cb(
		const control_msgs::JointControllerState &msg)
{
	mass1_mv_ = ARM_LENGTH / 2.0 + msg.process_value;
}

void UavGeometryControl::mass2_cb(
		const control_msgs::JointControllerState &msg)
{
	mass2_mv_ = ARM_LENGTH / 2.0 + msg.process_value;
}

void UavGeometryControl::mass3_cb(
		const control_msgs::JointControllerState &msg)
{
	mass3_mv_ = ARM_LENGTH / 2.0 + msg.process_value;
}

void UavGeometryControl::gripperLeft_cb(
		const geometry_msgs::Point &msg)
{
	gripperLeft_mv_(0, 0) = msg.x;
	gripperLeft_mv_(1, 0) = msg.y;
	gripperLeft_mv_(2, 0) = msg.z;
}

void UavGeometryControl::gripperRight_cb(
		const geometry_msgs::Point &msg)
{
	gripperRight_mv_(0, 0) = msg.x;
	gripperRight_mv_(1, 0) = msg.y;
	gripperRight_mv_(2, 0) = msg.z;
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

void UavGeometryControl::enableMassControl()
{
	ROS_INFO("UavGeometryControl::enableMassControl() - "
			"Mass control enabled.");
	enable_mass_control_ = true;
	enable_manipulator_control_ = false;

	UAV_MASS += 4*MM_MASS;
	// Add mass publishers if available
	mass0_cmd_pub_ = node_handle_.advertise<std_msgs::Float64>(
			"/" + uav_ns + "/movable_mass_0_position_controller/command", 1);
	mass1_cmd_pub_ = node_handle_.advertise<std_msgs::Float64>(
			"/" + uav_ns + "/movable_mass_1_position_controller/command", 1);
	mass2_cmd_pub_ = node_handle_.advertise<std_msgs::Float64>(
			"/" + uav_ns + "/movable_mass_2_position_controller/command", 1);
	mass3_cmd_pub_ = node_handle_.advertise<std_msgs::Float64>(
			"/" + uav_ns + "/movable_mass_3_position_controller/command", 1);

	// Mass state subscribers
	mass0_state_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/movable_mass_0_position_controller/state", 1,
			&UavGeometryControl::mass0_cb, this);
	mass1_state_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/movable_mass_1_position_controller/state", 1,
			&UavGeometryControl::mass1_cb, this);
	mass2_state_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/movable_mass_2_position_controller/state", 1,
			&UavGeometryControl::mass2_cb, this);
	mass3_state_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/movable_mass_3_position_controller/state", 1,
			&UavGeometryControl::mass3_cb, this);
}

void UavGeometryControl::disableMassControl()
{
	enable_mass_control_ = false;
}

void UavGeometryControl::enableManipulatorControl()
{
	ROS_INFO("UavGeometryControl::enableManipulatorControl() - "
			"Manipulator control enabled.");
	enable_manipulator_control_ = true;
	enable_mass_control_ = false;

	UAV_MASS += 2 * PAYLOAD_MASS + TOTAL_LINK_MASS;
	// Add gripper publishers if available
	gripperLeft_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/left_gripper_pos", 1,
			&UavGeometryControl::gripperLeft_cb, this);
	gripperRight_sub_ = node_handle_.subscribe(
			"/" + uav_ns + "/right_gripper_pos", 1,
			&UavGeometryControl::gripperRight_cb, this);

	payload_pos_pub_ = node_handle_.advertise<geometry_msgs::Point>(
			"/" + uav_ns + "/payload_position", 1);
}

void UavGeometryControl::disableManipulatorControl()
{
	enable_manipulator_control_ = false;
}

int main(int argc, char** argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "geometry_control");

	// Fetch parameters
	double rate;
	std::string uav_namespace;
	bool mass_ctl;
	bool manipulator_ctl;

	ros::NodeHandle nh_("~");
	nh_.getParam("rate", rate);
	nh_.getParam("type", uav_namespace);
	nh_.getParam("mass_ctl", mass_ctl);
	nh_.getParam("manipulator_ctl", manipulator_ctl);

	cout << "Rate: " << rate << "\n";
	cout << "Type: " << uav_namespace << "\n";
	cout << "Mass_ctl: " << mass_ctl << "\n";
	cout << "Manipulator_ctl: " << manipulator_ctl << "\n";

	// Start the control algorithm
	UavGeometryControl geometric_control(rate, uav_namespace);

	// Check if masses are enabled
	if (mass_ctl) { geometric_control.enableMassControl(); }

	// Check if manipulator control is enabled
	if (manipulator_ctl) { geometric_control.enableManipulatorControl(); }

	geometric_control.runControllerLoop();

	return 0;
}

