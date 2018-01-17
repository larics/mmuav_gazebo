#include <mmuav_control/mmuav_joint_position_control.h>
#include <ros/package.h>

JointPositionControl::JointPositionControl(void)
{
	rate_ = 100;

	joint_states_sub_ros_ = n_.subscribe("joint_states/state", 1, &JointPositionControl::joint_state_cb_ros, this);
}

void JointPositionControl::joint_state_cb_ros(const sensor_msgs::JointState &msg)
{

}

void JointPositionControl::LoadParameters(std::string file, std::vector<std::string> &controllers)
{

}

void JointPositionControl::run(void)
{
	ros::Rate loop_rate(rate_);

	while(ros::ok())
	{
		ros::spinOnce();

		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_position_control_node");

	int i;

	ros::NodeHandle private_node_handle_("~");
	std::string path = ros::package::getPath("mmuav_control");
	std::string controller_params_file;

	if (argc < 2)
	{
		ROS_WARN("Missing argument controler name!");
		return 0;
	}

	std::vector<std::string> controllers;
	for (i = 1; i < argc; i++) controllers.push_back(std::string(argv[i])); 

	private_node_handle_.param("param_file", controller_params_file, std::string("/config/dual_arm_manipulator_position_control.yaml"));

	JointPositionControl joint_control;
	joint_control.LoadParameters(path+controller_params_file, controllers);
	joint_control.run();

	return 0;
}