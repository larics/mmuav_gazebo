#ifndef UAV_POSITION_CONTROL_H
#define UAV_POSITION_CONTROL_H

#include <mmuav_control/PID.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <mmuav_control/UavPositionCtlParamsConfig.h>

class PositionControl
{
	public:
		PositionControl(int rate);
		void run();
	private:
		void clock_cb(const rosgraph_msgs::Clock &msg);
		void odometry_cb(const nav_msgs::Odometry &msg);
		void position_ref_cb(const geometry_msgs::Vector3Stamped &msg);
		void configCallback(mmuav_control::UavPositionCtlParamsConfig &config, uint32_t level);
		void yaw_ref_cb(const std_msgs::Float64 &msg);
		void quaternion2euler(float *quaternion, float *euler);

		volatile bool start_flag_, config_start_;
		float yaw_sp_;
		float orientation_mv_[3];
		int rate_;

		geometry_msgs::Vector3Stamped position_mv_, position_sp_, velocity_mv_;
		rosgraph_msgs::Clock clock_;
		ros::Time t_old_;

		PID pid_x_, pid_vx_;
		PID pid_y_, pid_vy_;
		PID pid_z_, pid_vz_;

		ros::NodeHandle n_;
		ros::Subscriber odometry_ros_sub_, position_ref_ros_sub_;
		ros::Subscriber clock_ros_sub_, yaw_ros_sub_;

		ros::Publisher euler_ref_pub_ros_, pid_y_pub_ros_, pid_vy_pub_ros_;
		ros::Publisher pid_x_pub_ros_, pid_vx_pub_ros_, height_pub_ros_;
		ros::Publisher pid_z_pub_ros_, pid_vz_pub_ros_;

		dynamic_reconfigure::Server<mmuav_control::UavPositionCtlParamsConfig> dr_srv;
  		dynamic_reconfigure::Server<mmuav_control::UavPositionCtlParamsConfig>::CallbackType cb;

};

#endif