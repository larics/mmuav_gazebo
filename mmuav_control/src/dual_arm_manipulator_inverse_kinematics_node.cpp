#include <mmuav_control/dual_arm_manipulator_inverse_kinematics_node.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "inverse_kinematics_node");
	ros::NodeHandle private_node_handle_("~");
	ros::NodeHandle n;

	float a[2], b[2], c[2];

	std::string path = ros::package::getPath("mmuav_control");
	std::string dh_parameters_file;
	DualArmManipulatorInverseKinematics manipulator_inverse;
	DualArmManipulatorDirectKinematics manipulator_direct;

	private_node_handle_.param("dh_parameters_file", dh_parameters_file, std::string("/config/dual_arm_manipulator_dh_parameters.yaml"));

	manipulator_direct.LoadParameters(path+dh_parameters_file);
	manipulator_inverse.LoadParameters(path+dh_parameters_file);

	manipulator_direct.dk_calculate(-4.3,-2.264817,0);

	manipulator_inverse.ik_calculate(-0.116239,-0.1440838,-1.853981115, a, b, c);


	return 0;
}