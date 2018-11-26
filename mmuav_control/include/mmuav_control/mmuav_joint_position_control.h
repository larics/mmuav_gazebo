#ifndef MMUAV_JOINT_POSITION_CONTROL_H
#define MMUAV_JOINT_POSITION_CONTROL_H

#include "ros/ros.h"
#include <string>
#include <uav_ros_control/PID.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <mmuav_control/JointCtlParamsConfig.h>
#include <uav_ros_control_msgs/PIDController.h>

class JointControl
{
	private:
		void configCallback(mmuav_control::JointCtlParamsConfig &config, uint32_t level);
		float ref_;
		bool config_start_;
		PID joint_pid_;

		dynamic_reconfigure::Server<mmuav_control::JointCtlParamsConfig> *dr_srv;
		dynamic_reconfigure::Server<mmuav_control::JointCtlParamsConfig>::CallbackType cb;
	public:
		JointControl();
		~JointControl();
		void joint_ref_cb_ros(const std_msgs::Float64 &msg);
		void setReconfigure(ros::NodeHandle n);
		void set_kp(float kp);
		void set_ki(float ki);
		void set_kd(float kd);
		void set_lim_low(float lim_low);
		void set_lim_high(float lim_high);
		float get_ref(void);
		float compute(float ref, float meas, float dt);
		void create_msg(uav_ros_control_msgs::PIDController &msg);
};

class JointPositionControl{
	private:
		void joint_state_cb_ros(const sensor_msgs::JointState &msg);
		void clock_cb(const rosgraph_msgs::Clock &msg);

		int rate_, sampling_reduction_;
		bool is_init_;

		rosgraph_msgs::Clock clock_;

		ros::Subscriber joint_states_sub_ros_, clock_ros_sub_;
		std::vector<ros::Subscriber> joint_ref_ros_sub_;
		std::vector<ros::Publisher> joint_command_pub_ros_, pid_state_pub_ros_;
		std::vector<std::string> joint_name_;
		std::vector<double> joint_meas_;
		JointControl* joint_control_;
		ros::NodeHandle n_;

	public:
		JointPositionControl(void);
		~JointPositionControl(void);
		void LoadParameters(std::string file, std::vector<std::string> &controllers);
		void run(void);

};

#endif