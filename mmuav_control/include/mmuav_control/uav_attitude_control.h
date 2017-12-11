#ifndef UAV_ATTITUDE_CONTROL_H
#define UAV_ATTITUDE_CONTROL_H

#include <mmuav_control/PID.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <mmuav_control/UavAttitudeCtlParamsConfig.h>

class AttitudeControl
{
	public:
		AttitudeControl(int rate);
		void run();
	private:
		void clock_cb(const rosgraph_msgs::Clock &msg);
		void ahrs_cb(const sensor_msgs::Imu &msg);
		void euler_ref_cb(const geometry_msgs::Vector3 &msg);
		void quaternion2euler(float *quaternion, float *euler);
		void configCallback(mmuav_control::UavAttitudeCtlParamsConfig &config, uint32_t level);

		volatile bool start_flag_, config_start_;
		float w_sp_;
		int rate_;

		geometry_msgs::Vector3 euler_mv_, euler_sp_, euler_rate_mv_;
		rosgraph_msgs::Clock clock_;
		ros::Time t_old_;

		PID pid_roll_, pid_roll_rate_;
		PID pid_pitch_, pid_pitch_rate_;
		PID pid_yaw_, pid_yaw_rate_;

		ros::NodeHandle n_;
		ros::Subscriber imu_ros_sub_, euler_ref_ros_sub_;
		ros::Subscriber clock_ros_sub_;

		ros::Publisher attitude_pub_ros_, pid_roll_pub_ros_, pid_roll_rate_pub_ros_;
		ros::Publisher pid_pitch_pub_ros_, pid_pitch_rate_pub_ros_;
		ros::Publisher pid_yaw_pub_ros_, pid_yaw_rate_pub_ros_;

		dynamic_reconfigure::Server<mmuav_control::UavAttitudeCtlParamsConfig> dr_srv;
  		dynamic_reconfigure::Server<mmuav_control::UavAttitudeCtlParamsConfig>::CallbackType cb;

};

#endif