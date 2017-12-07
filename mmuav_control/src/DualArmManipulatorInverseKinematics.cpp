#include <mmuav_control/DualArmManipulatorInverseKinematics.h>

DualArmManipulatorInverseKinematics::DualArmManipulatorInverseKinematics(void)
{
	isInit = false;
}

void DualArmManipulatorInverseKinematics::LoadParameters(std::string file)
{
	YAML::Node config = YAML::LoadFile(file);
	std::vector<double> theta, a, left_arm_origin, right_arm_origin;

	theta = config["theta"].as<std::vector<double> >();
	a = config["a"].as<std::vector<double> >();
	left_arm_origin = config["origin"]["left_arm"].as<std::vector<double> >();
	right_arm_origin = config["origin"]["right_arm"].as<std::vector<double> >();

	for (int i=0; i<3; i++)
	{
		dhParams_.theta[i] = theta[i];
		dhParams_.alpha[i] = 0;
		dhParams_.d[i] = 0;
		dhParams_.a[i] = a[i];
	}

	isInit = true;
}