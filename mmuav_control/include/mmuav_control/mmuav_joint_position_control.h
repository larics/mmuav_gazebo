#ifndef MMUAV_JOINT_POSITION_CONTROL_H
#define MMUAV_JOINT_POSITION_CONTROL_H

#include "ros/ros.h"
#include <string>
#include <sensor_msgs/JointState.h>

class JointPositionControl{
	private:
		void joint_state_cb_ros(const sensor_msgs::JointState &msg);

		int rate_;

		ros::Subscriber joint_states_sub_ros_;
		ros::NodeHandle n_;
	public:
		JointPositionControl(void);
		void LoadParameters(std::string file, std::vector<std::string> &controllers);
		void run(void);

};

#endif