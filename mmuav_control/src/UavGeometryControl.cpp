/*
 * UavGeometricControl.cpp
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#include <mmuav_control/UavGeometryControl.hpp>
#include <mav_msgs/Actuators.h>

using namespace std;

const Matrix<double, 3, 1> E3(0, 0, 1); 		// Z - unit vector
const double UAV_MASS = 2.1;					// Mass constant
const double G = 9.81;							// Gravity acceleration
const double ARM_LENGTH = 0.314;				// Motor offset
const double MOMENT_CONSTANT = 0.016;			// Moment constant [m]
const double MOTOR_CONSTANT = 8.54858e-06;		// Motor constant
Matrix<double, 3, 3> INERTIA;					// Inertia matrix
Matrix<double, 4, 4> THRUST_TRANSFORM;			// Thrust transform matrix

UavGeometryControl::UavGeometryControl(int rate)
{
	// Initialize controller variables
	controller_rate_ = rate;
	sleep_duration_ = 0.5;
	imu_start_flag_ = false;
	odometry_start_flag_ = false;

	// Initialize inertia matrix
	INERTIA.setZero(3, 3);
	INERTIA(0, 0) = 0.0826944;
	INERTIA(1, 1) = 0.0826944;
	INERTIA(2, 2) = 0.0104;

	// Initialize thrust transform matrix
	THRUST_TRANSFORM.setZero(4, 4);

	// First row
	THRUST_TRANSFORM(0, 0) = 1;
	THRUST_TRANSFORM(0, 1) = 1;
	THRUST_TRANSFORM(0, 2) = 1;
	THRUST_TRANSFORM(0, 3) = 1;

	// Second row
	THRUST_TRANSFORM(1, 1) = - ARM_LENGTH;
	THRUST_TRANSFORM(1, 3) = ARM_LENGTH;

	// Third row
	THRUST_TRANSFORM(2, 0) = ARM_LENGTH;
	THRUST_TRANSFORM(2, 2) = - ARM_LENGTH;

	// Fourth row
	THRUST_TRANSFORM(3, 0) = MOMENT_CONSTANT;
	THRUST_TRANSFORM(3, 1) = - MOMENT_CONSTANT;
	THRUST_TRANSFORM(3, 2) = MOMENT_CONSTANT;
	THRUST_TRANSFORM(3, 3) = - MOMENT_CONSTANT;

	// Invert the matrix
	THRUST_TRANSFORM = THRUST_TRANSFORM.inverse().eval();

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

	// Initialize measured
	omega_mv_.setZero(3, 1);
	alpha_mv_.setZero(3, 1);
	R_mv_.setZero(3,3);

	// Initialize controller parameters
	// Parameters initialized according to 2010-extended.pdf
	k_x_ = 1 * UAV_MASS;
	k_v_ = 0.5 * UAV_MASS;
	k_R_ = 1;
	k_omega_ = 1;

	// Initialize subscribers and publishers
	imu_ros_sub_ = node_handle_.subscribe(
			"/uav/imu", 1, &UavGeometryControl::imu_cb, this);
	odom_ros_sub_ = node_handle_.subscribe(
			"/uav/odometry", 1, &UavGeometryControl::odom_cb, this);
	rotor_ros_pub_ = node_handle_.advertise<mav_msgs::Actuators>(
			"/gazebo/command/motor_speed", 10);

	// Initialize position reference subscribers
	xd_ros_sub_ = node_handle_.subscribe(
			"/uav/x_desired", 1, &UavGeometryControl::xd_cb, this);
	vd_ros_sub_ = node_handle_.subscribe(
			"/uav/v_desired", 1, &UavGeometryControl::vd_cb, this);
	ad_ros_sub_ = node_handle_.subscribe(
			"/uav/a_desired", 1, &UavGeometryControl::ad_cb, this);

	// Initialize attitude reference subscribers
	b1d_ros_sub_ = node_handle_.subscribe(
				"/uav/b1_desired", 1, &UavGeometryControl::b1d_cb, this);
	omega_d_ros_sub_ = node_handle_.subscribe(
				"/uav/omega_desired", 1,
				&UavGeometryControl::omegad_cb, this);
	alpha_d_ros_sub_ = node_handle_.subscribe(
				"/uav/alpha_desired", 1,
				&UavGeometryControl::alphad_cb, this);
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
			"Starting geometric control.");

	t_old_ = ros::Time::now();

	// Position errors
	Matrix<double, 3, 1> e_x, e_v;

	// Attitude errors
	Matrix<double, 3, 1> e_omega, e_R;

	// Auxiliary matrices
	Matrix<double, 3, 3> e_R_skew, omega_mv_skew;

	// A - desired control force for the translational dynamics
	// b3_d - desired thrust vector
	// b2_d - desired y-direction body vector
	// M_u - control moment
	Matrix<double, 3, 1> A, b3_d, M_u, b2_d;

	// Desired rotation matrix
	Matrix<double, 3, 3> R_d;

	// Total thrust control value
	double f_u;

	// Rotor velocities control vector
	Matrix<double, 4, 1> rotor_velocities;

	// Initialize rotor velocity publisher msg
	mav_msgs::Actuators rotor_vel_msg;
	vector<double> velocity_vector(4);
	rotor_vel_msg.angular_velocities = velocity_vector;

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

		// TRAJECTORY TRACKING
		// Calculate total thrust and b3_d (desired thrust vector)
		e_x = x_mv_ - x_d_;
		e_v = v_mv_ - v_d_;
		A = - k_x_ * e_x
			- k_v_ * e_v
			- UAV_MASS * G * E3
			+ UAV_MASS * a_d_;
		f_u = - A.dot( R_mv_ * E3 );
		b3_d =  - A / A.norm();

		// Construct desired rotation matrix
		b2_d = b3_d.cross(b1_d_);
		R_d.setZero(3, 3);
		R_d << b1_d_, b2_d, b3_d;

		// ATTITUDE TRACKING
		// Calculate control moment M
		e_R_skew = (R_d.adjoint() * R_mv_ - R_mv_.adjoint() * R_d) / 2;
		veeOperator(e_R_skew, e_R);
		e_omega = omega_mv_ - R_mv_.adjoint() * R_d * omega_d_;
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
					omega_mv_skew * R_mv_.adjoint() * R_d * omega_d_
					- R_mv_.adjoint() * R_d * alpha_d_
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
		cout << "Forces: \n" << rotor_velocities << "\n";
		rotor_velocities = rotor_velocities / MOTOR_CONSTANT;
		rotor_velocities = rotor_velocities.array().sqrt();

		// Fill and publish rotor message
		/**
		 * Note: Added negative signs to the 2nd and 4th rotor.
		 */
		rotor_vel_msg.angular_velocities[0] = rotor_velocities(0, 0);
		rotor_vel_msg.angular_velocities[1] = - rotor_velocities(1, 0);
		rotor_vel_msg.angular_velocities[2] = rotor_velocities(2, 0);
		rotor_vel_msg.angular_velocities[3] = - rotor_velocities(3, 0);
		rotor_ros_pub_.publish(rotor_vel_msg);

		cout << "e_x: \n" << e_x << "\n";
		cout << "e_v: \n" << e_v << "\n";
		cout << "e_R: \n" << e_R << "\n";
		cout << "e_omega: \n" << e_omega << "\n";
		cout << "f_u: \n" << f_u << "\n";
		cout << "M_u: \n" << M_u << "\n";
		cout << "Thrust_moment_vec: \n" << thrust_moment_vec << "\n";
		cout << "Rotor_vel: \n" << rotor_velocities << "\n";
		cout << "R_d: \n" << R_d << "\n";
		cout << "R_mv: \n" << R_mv_ << "\n";
		cout << "A: \n" << A << "\n";
		cout << "\n";
		cout << endl;
	}
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

void UavGeometryControl::odom_cb(const nav_msgs::Odometry &msg)
{
    if (!odometry_start_flag_) odometry_start_flag_ = true;

	x_mv_(0, 0) = msg.pose.pose.position.x;
	x_mv_(1, 0) = msg.pose.pose.position.y;
	x_mv_(2, 0) = msg.pose.pose.position.z;

	v_mv_(0, 0) = msg.twist.twist.linear.x;
	v_mv_(1, 0) = msg.twist.twist.linear.y;
	v_mv_(1, 0) = msg.twist.twist.linear.z;
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
}

void UavGeometryControl::euler2RotationMatrix(
		const double roll,
		const double pitch,
		const double yaw,
		Matrix<double, 3, 3> &rotMatrix)
{
	rotMatrix.setZero(3, 3);

	AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
	AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
	AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

	// TODO(lmark): Check if correct.
	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
	rotMatrix = q.matrix();
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

	// Initialize controller rate
	int rate;
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("rate", rate, int(50));

	// Start the control algorithm
	UavGeometryControl geometric_control(rate);
	geometric_control.run();

	return 0;
}

