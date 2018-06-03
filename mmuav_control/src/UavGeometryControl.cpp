/*
 * UavGeometricControl.cpp
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#include "mmuav_control/UavGeometryControl.hpp"

using namespace std;

const Matrix<double, 3, 1> e3(0, 0, 1);

UavGeometryControl::UavGeometryControl(int rate)
{
	// Initialize controller variables
	controller_rate_ = rate;
	sleep_duration_ = 0.5;
	start_flag_ = false;

	// Initialize desired position values
	x_d_.setZero(3,1);
	v_d_.setZero(3,1);
	a_d_.setZero(3,1);

	// Initial measured position values
	x_mv_.setZero(3,1);
	v_mv_.setZero(3,1);
	a_mv_.setZero(3,1);

	// Initialize desired attitude
	omega_d_.setZero(3, 3);
	alpha_d_.setZero(3, 3);
	b1_d_.setZero(3,1);
	b1_d_(0,0) = 1;

	// Initialize measured
	omega_mv_.setZero(3, 3);
	alpha_mv_.setZero(3, 3);
	R_mv_.setZero(3,3);

	// Initialize controller parameters
	k_x_ = 2;
	k_v_ = 1;
	k_R_ = 1;
	k_omega_ = 1;

	// Initialize subscribers and publishers
	imu_ros_sub_ = node_handle_.subscribe(
			"/mmuav/imu", 1, &UavGeometryControl::imu_cb, this);
	//odom_ros_sub_ = node_handle_.subscribe(
	//		"/mmuav/odometry", 1, &UavGeometryControl::odom_cb, this);

	// Initialize position reference subscribers
	xd_ros_sub_ = node_handle_.subscribe(
			"/mmuav/x_desired", 1, &UavGeometryControl::xd_cb, this);
	vd_ros_sub_ = node_handle_.subscribe(
			"/mmuav/v_desired", 1, &UavGeometryControl::vd_cb, this);
	ad_ros_sub_ = node_handle_.subscribe(
			"/mmuav/a_desired", 1, &UavGeometryControl::ad_cb, this);

	// Initialize attitude reference subscribers
	b1d_ros_sub_ = node_handle_.subscribe(
				"/mmuav/b1_desired", 1, &UavGeometryControl::b1d_cb, this);
	omega_d_ros_sub_ = node_handle_.subscribe(
				"/mmuav/omega_desired", 1,
				&UavGeometryControl::omegad_cb, this);
	alpha_d_ros_sub_ = node_handle_.subscribe(
				"/mmuav/alpha_desired", 1,
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
	while (!start_flag_ && ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("UavGeometricControl::run() - "
				"Waiting for first measurement");
		ros::Duration(sleep_duration_).sleep();
	}
	ROS_INFO("UavGeometricControl::run() - "
			"Starting geometric control.");

	t_old_ = ros::Time::now();

	// Position errors
	Matrix<double, 3, 1> e_x, e_v;

	// Attitude errors
	Matrix<double, 3, 1> e_omega, e_R;

	// A - desired control force for the translational dynamics
	Matrix<double, 3, 1> A;

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

		cout << "e_x: \n" << e_x << "\n";
		cout << "e_v: \n" << e_v << "\n";
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
	hatOperator(msg.x, msg.y, msg.z, omega_d_);
}

void UavGeometryControl::alphad_cb(const geometry_msgs::Vector3 &msg)
{
	hatOperator(msg.x, msg.y, msg.z, alpha_d_);
}

void UavGeometryControl::odom_cb(const nav_msgs::Odometry &msg)
{
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

    if (!start_flag_) start_flag_ = true;

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

    // Construct angular velocity skew matrix
    hatOperator(
    		euler_rate_mv_.x,
			euler_rate_mv_.y,
			euler_rate_mv_.z,
			omega_mv_);

    // Initialize position
}

void UavGeometryControl::euler2RotationMatrix(const double roll,
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
		double x, double y, double z, Matrix<double, 3, 3> &hatMatrix)
{
	hatMatrix.setZero(3,3);
	hatMatrix(0, 1) = -z;
	hatMatrix(0, 2) =  y;
	hatMatrix(1, 0) =  z;
	hatMatrix(1, 2) = -x;
	hatMatrix(2, 0) = -y;
	hatMatrix(2, 1) =  x;
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
	private_node_handle_.param("rate", rate, int(10));

	// Start the control algorithm
	UavGeometryControl geometric_control(rate);
	geometric_control.run();

	return 0;
}

