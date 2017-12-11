#include <mmuav_control/DualArmManipulatorDirectKinematics.h>


DualArmManipulatorDirectKinematics::DualArmManipulatorDirectKinematics(void)
{
	isInit = false;

	/*T01 << -1,  0,  0,  0,
		    0, -1,  0,  0,
		    0,  0,  1,  0,
		    0,  0,  0,  1;*/
}

Eigen::Matrix4d DualArmManipulatorDirectKinematics::dk_calculate(float q1, float q2, float q3)
{
	if (isInit)
	{
		float theta[3], alpha[3], d[3], a[3];
		Eigen::Matrix4d T13;

		theta[0] = dhParams_.theta[0] + q1;
		theta[1] = dhParams_.theta[1] + q2;
		theta[2] = dhParams_.theta[2] + q3;

		alpha[0] = dhParams_.alpha[0];
		alpha[1] = dhParams_.alpha[1];
		alpha[2] = dhParams_.alpha[2];

		d[0] = dhParams_.d[0];
		d[1] = dhParams_.d[1];
		d[2] = dhParams_.d[2];

		a[0] = dhParams_.a[0];
		a[1] = dhParams_.a[1];
		a[2] = dhParams_.a[2];

		T13 << cos(theta[2])*(cos(theta[0])*cos(theta[1]) - cos(alpha[0])*sin(theta[0])*sin(theta[1])) - sin(theta[2])*(cos(alpha[1])*cos(theta[0])*sin(theta[1]) - sin(alpha[0])*sin(alpha[1])*sin(theta[0]) + cos(alpha[0])*cos(alpha[1])*cos(theta[1])*sin(theta[0])),   sin(alpha[2])*(cos(alpha[1])*sin(alpha[0])*sin(theta[0]) + sin(alpha[1])*cos(theta[0])*sin(theta[1]) + cos(alpha[0])*sin(alpha[1])*cos(theta[1])*sin(theta[0])) - cos(alpha[2])*sin(theta[2])*(cos(theta[0])*cos(theta[1]) - cos(alpha[0])*sin(theta[0])*sin(theta[1])) - cos(alpha[2])*cos(theta[2])*(cos(alpha[1])*cos(theta[0])*sin(theta[1]) - sin(alpha[0])*sin(alpha[1])*sin(theta[0]) + cos(alpha[0])*cos(alpha[1])*cos(theta[1])*sin(theta[0])), cos(alpha[2])*(cos(alpha[1])*sin(alpha[0])*sin(theta[0]) + sin(alpha[1])*cos(theta[0])*sin(theta[1]) + cos(alpha[0])*sin(alpha[1])*cos(theta[1])*sin(theta[0])) + sin(alpha[2])*sin(theta[2])*(cos(theta[0])*cos(theta[1]) - cos(alpha[0])*sin(theta[0])*sin(theta[1])) + sin(alpha[2])*cos(theta[2])*(cos(alpha[1])*cos(theta[0])*sin(theta[1]) - sin(alpha[0])*sin(alpha[1])*sin(theta[0]) + cos(alpha[0])*cos(alpha[1])*cos(theta[1])*sin(theta[0])), a[0]*cos(theta[0]) + d[2]*(cos(alpha[1])*sin(alpha[0])*sin(theta[0]) + sin(alpha[1])*cos(theta[0])*sin(theta[1]) + cos(alpha[0])*sin(alpha[1])*cos(theta[1])*sin(theta[0])) + a[1]*cos(theta[0])*cos(theta[1]) + d[1]*sin(alpha[0])*sin(theta[0]) - a[2]*sin(theta[2])*(cos(alpha[1])*cos(theta[0])*sin(theta[1]) - sin(alpha[0])*sin(alpha[1])*sin(theta[0]) + cos(alpha[0])*cos(alpha[1])*cos(theta[1])*sin(theta[0])) + a[2]*cos(theta[2])*(cos(theta[0])*cos(theta[1]) - cos(alpha[0])*sin(theta[0])*sin(theta[1])) - a[1]*cos(alpha[0])*sin(theta[0])*sin(theta[1]),
	 		   cos(theta[2])*(cos(theta[1])*sin(theta[0]) + cos(alpha[0])*cos(theta[0])*sin(theta[1])) - sin(theta[2])*(sin(alpha[0])*sin(alpha[1])*cos(theta[0]) + cos(alpha[1])*sin(theta[0])*sin(theta[1]) - cos(alpha[0])*cos(alpha[1])*cos(theta[0])*cos(theta[1])), - sin(alpha[2])*(cos(alpha[1])*sin(alpha[0])*cos(theta[0]) - sin(alpha[1])*sin(theta[0])*sin(theta[1]) + cos(alpha[0])*sin(alpha[1])*cos(theta[0])*cos(theta[1])) - cos(alpha[2])*sin(theta[2])*(cos(theta[1])*sin(theta[0]) + cos(alpha[0])*cos(theta[0])*sin(theta[1])) - cos(alpha[2])*cos(theta[2])*(sin(alpha[0])*sin(alpha[1])*cos(theta[0]) + cos(alpha[1])*sin(theta[0])*sin(theta[1]) - cos(alpha[0])*cos(alpha[1])*cos(theta[0])*cos(theta[1])), sin(alpha[2])*sin(theta[2])*(cos(theta[1])*sin(theta[0]) + cos(alpha[0])*cos(theta[0])*sin(theta[1])) - cos(alpha[2])*(cos(alpha[1])*sin(alpha[0])*cos(theta[0]) - sin(alpha[1])*sin(theta[0])*sin(theta[1]) + cos(alpha[0])*sin(alpha[1])*cos(theta[0])*cos(theta[1])) + sin(alpha[2])*cos(theta[2])*(sin(alpha[0])*sin(alpha[1])*cos(theta[0]) + cos(alpha[1])*sin(theta[0])*sin(theta[1]) - cos(alpha[0])*cos(alpha[1])*cos(theta[0])*cos(theta[1])), a[0]*sin(theta[0]) - d[2]*(cos(alpha[1])*sin(alpha[0])*cos(theta[0]) - sin(alpha[1])*sin(theta[0])*sin(theta[1]) + cos(alpha[0])*sin(alpha[1])*cos(theta[0])*cos(theta[1])) - d[1]*sin(alpha[0])*cos(theta[0]) - a[2]*sin(theta[2])*(sin(alpha[0])*sin(alpha[1])*cos(theta[0]) + cos(alpha[1])*sin(theta[0])*sin(theta[1]) - cos(alpha[0])*cos(alpha[1])*cos(theta[0])*cos(theta[1])) + a[1]*cos(theta[1])*sin(theta[0]) + a[2]*cos(theta[2])*(cos(theta[1])*sin(theta[0]) + cos(alpha[0])*cos(theta[0])*sin(theta[1])) + a[1]*cos(alpha[0])*cos(theta[0])*sin(theta[1]),
	           sin(theta[2])*(cos(alpha[0])*sin(alpha[1]) + cos(alpha[1])*sin(alpha[0])*cos(theta[1])) + sin(alpha[0])*cos(theta[2])*sin(theta[1]),                                                                                                                         sin(alpha[2])*(cos(alpha[0])*cos(alpha[1]) - sin(alpha[0])*sin(alpha[1])*cos(theta[1])) + cos(alpha[2])*cos(theta[2])*(cos(alpha[0])*sin(alpha[1]) + cos(alpha[1])*sin(alpha[0])*cos(theta[1])) - cos(alpha[2])*sin(alpha[0])*sin(theta[1])*sin(theta[2]),                                                                                                                                                                                               cos(alpha[2])*(cos(alpha[0])*cos(alpha[1]) - sin(alpha[0])*sin(alpha[1])*cos(theta[1])) - sin(alpha[2])*cos(theta[2])*(cos(alpha[0])*sin(alpha[1]) + cos(alpha[1])*sin(alpha[0])*cos(theta[1])) + sin(alpha[0])*sin(alpha[2])*sin(theta[1])*sin(theta[2]),                                                                                                                                                                                               d[0] + d[1]*cos(alpha[0]) + d[2]*(cos(alpha[0])*cos(alpha[1]) - sin(alpha[0])*sin(alpha[1])*cos(theta[1])) + a[1]*sin(alpha[0])*sin(theta[1]) + a[2]*sin(theta[2])*(cos(alpha[0])*sin(alpha[1]) + cos(alpha[1])*sin(alpha[0])*cos(theta[1])) + a[2]*sin(alpha[0])*cos(theta[2])*sin(theta[1]),
	           0,                                                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                       1;
		
		return T13;
	}

}

void DualArmManipulatorDirectKinematics::LoadParameters(std::string file)
{
	YAML::Node config = YAML::LoadFile(file);
	std::vector<double> theta, alpha, d, a, left_arm_origin, right_arm_origin;

	theta = config["theta"].as<std::vector<double> >();
	alpha = config["alpha"].as<std::vector<double> >();
	d = config["d"].as<std::vector<double> >();
	a = config["a"].as<std::vector<double> >();
	//left_arm_origin = config["origin"]["left_arm"].as<std::vector<double> >();
	//right_arm_origin = config["origin"]["right_arm"].as<std::vector<double> >();

	for (int i=0; i<3; i++)
	{
		dhParams_.theta[i] = theta[i];
		dhParams_.alpha[i] = alpha[i];
		dhParams_.d[i] = d[i];
		dhParams_.a[i] = a[i];
	}

	/*Torigin_left <<  cos(left_arm_origin[5])*cos(left_arm_origin[4]),  cos(left_arm_origin[5])*sin(left_arm_origin[4])*sin(left_arm_origin[3])-sin(left_arm_origin[5])*cos(left_arm_origin[3]),  cos(left_arm_origin[5])*sin(left_arm_origin[4])*cos(left_arm_origin[3])+sin(left_arm_origin[5])*sin(left_arm_origin[3]), left_arm_origin[0],
					 sin(left_arm_origin[5])*cos(left_arm_origin[4]),  sin(left_arm_origin[5])*sin(left_arm_origin[4])*sin(left_arm_origin[3])+cos(left_arm_origin[5])*cos(left_arm_origin[3]),  sin(left_arm_origin[5])*sin(left_arm_origin[4])*cos(left_arm_origin[3])-cos(left_arm_origin[5])*sin(left_arm_origin[3]), left_arm_origin[1],
					-sin(left_arm_origin[4]),                          cos(left_arm_origin[4])*sin(left_arm_origin[3]),                                      									 cos(left_arm_origin[4])*cos(left_arm_origin[3]),                   													  left_arm_origin[2],
					 0,                                                0,                                                																	     0,                           																							  1;
	Torigin_right <<  cos(right_arm_origin[5])*cos(right_arm_origin[4]),  cos(right_arm_origin[5])*sin(right_arm_origin[4])*sin(right_arm_origin[3])-sin(right_arm_origin[5])*cos(right_arm_origin[3]),  cos(right_arm_origin[5])*sin(right_arm_origin[4])*cos(right_arm_origin[3])+sin(right_arm_origin[5])*sin(right_arm_origin[3]), right_arm_origin[0],
					  sin(right_arm_origin[5])*cos(right_arm_origin[4]),  sin(right_arm_origin[5])*sin(right_arm_origin[4])*sin(right_arm_origin[3])+cos(right_arm_origin[5])*cos(right_arm_origin[3]),  sin(right_arm_origin[5])*sin(right_arm_origin[4])*cos(right_arm_origin[3])-cos(right_arm_origin[5])*sin(right_arm_origin[3]), right_arm_origin[1],
					 -sin(right_arm_origin[4]),                           cos(right_arm_origin[4])*sin(right_arm_origin[3]),                                      									     cos(right_arm_origin[4])*cos(right_arm_origin[3]),                   													       right_arm_origin[2],
					  0,                                                  0,                                                																	         0,                           																							       1;
	*/
	isInit = true;
}