/*
 * UavGeometricControl.cpp
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#include <mmuav_control/UavGeometryControl.h>

UavGeometryControl::UavGeometryControl(int rate)
{
	// Initialize controller variables
	controller_rate_ = rate;
	sleep_duration_ = 0.5;
	start_flag_ = false;

	// Initialize desired position and orientation values
	x_d_(0, 0) = 0;
	x_d_(1, 0) = 0;
	x_d_(2, 0) = 0;

	b1_d_(0, 0) = 0;
	b1_d_(1, 0) = 0;
	b1_d_(2, 0) = 1;

	// Initialize subscribers and publishers
	imu_ros_sub_ = node_handle_.subscribe(
			"/mmuav/imu", 1, &UavGeometryControl::imu_cb, this);
	xd_ros_sub_ = node_handle_.subscribe(
			"/mmuav/x_desired", 1, &UavGeometryControl::xd_cb, this);
	b1d_ros_sub_ = node_handle_.subscribe(
			"/mmuav/b1_desired", 1, &UavGeometryControl::b1d_cb, this);
}

UavGeometryControl::~UavGeometryControl()
{
	// TODO(lmark): Destructor..
}

void UavGeometryControl::run()
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
	// TODO(lmark): IMU plugin missing
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

void UavGeometryControl::xd_cb(const geometry_msgs::Vector3 &msg)
{
	x_d_(0, 0) = msg.x;
	x_d_(1, 0) = msg.y;
	x_d_(2, 0) = msg.z;
}

void UavGeometryControl::b1d_cb(const geometry_msgs::Vector3 &msg)
{
	b1_d_(0, 0) = msg.x;
	b1_d_(1, 0) = msg.y;
	b1_d_(2, 0) = msg.z;
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
	private_node_handle_.param("rate", rate, int(100));

	// Start the control algorithm
	UavGeometryControl geometric_control(rate);
	geometric_control.run();

	return 0;
}

