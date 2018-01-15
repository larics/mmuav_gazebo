#include <mmuav_control/MmuavDirectKinematics.h>


MmuavDirectKinematics::MmuavDirectKinematics(void)
{
	isInit = false;

	/*T01 << -1,  0,  0,  0,
		    0, -1,  0,  0,
		    0,  0,  1,  0,
		    0,  0,  0,  1;*/
}

Eigen::Matrix4d MmuavDirectKinematics::dk_calculate(float q1, float q2, float q3, float q4, float q5, float q6, float q7)
{
	if (isInit)
	{
		float theta[10], alpha[10], d[10], a[10];
		Eigen::Matrix4d T010;

		theta[0] = dhParams_.theta[0] + q1;
		theta[1] = dhParams_.theta[1];
		theta[2] = dhParams_.theta[2];
		theta[3] = dhParams_.theta[3];
		theta[4] = dhParams_.theta[4];
		theta[5] = dhParams_.theta[5];
		theta[6] = dhParams_.theta[6];
		theta[7] = dhParams_.theta[7] + q5;
		theta[8] = dhParams_.theta[8] + q6;
		theta[9] = dhParams_.theta[9] + q7;

		alpha[0] = dhParams_.alpha[0];
		alpha[1] = dhParams_.alpha[1];
		alpha[2] = dhParams_.alpha[2];
		alpha[3] = dhParams_.alpha[3];
		alpha[4] = dhParams_.alpha[4];
		alpha[5] = dhParams_.alpha[5];
		alpha[6] = dhParams_.alpha[6];
		alpha[7] = dhParams_.alpha[7];
		alpha[8] = dhParams_.alpha[8];
		alpha[9] = dhParams_.alpha[9];

		d[0] = dhParams_.d[0];
		d[1] = dhParams_.d[1] + q2;
		d[2] = dhParams_.d[2] + q3;
		d[3] = dhParams_.d[3] + q4;
		d[4] = dhParams_.d[4];
		d[5] = dhParams_.d[5];
		d[6] = dhParams_.d[6];
		d[7] = dhParams_.d[7];
		d[8] = dhParams_.d[8];
		d[9] = dhParams_.d[9];

		a[0] = dhParams_.a[0];
		a[1] = dhParams_.a[1];
		a[2] = dhParams_.a[2];
		a[3] = dhParams_.a[3];
		a[4] = dhParams_.a[4];
		a[5] = dhParams_.a[5];
		a[6] = dhParams_.a[6];
		a[7] = dhParams_.a[7];
		a[8] = dhParams_.a[8];
		a[9] = dhParams_.a[9];

 
		T010 << -sin(theta[0] + theta[7] + theta[8] + theta[9]), cos(theta[0] + theta[7] + theta[8] + theta[9]),  0, d[3]*cos(theta[0]) - a[7]*sin(theta[0] + theta[7]) - a[9]*sin(theta[0] + theta[7] + theta[8] + theta[9]) + d[6]*cos(theta[0]) - d[2]*sin(theta[0]) - d[5]*sin(theta[0]) - a[8]*sin(theta[0] + theta[7] + theta[8]),
		 		cos(theta[0] + theta[7] + theta[8] + theta[9]), sin(theta[0] + theta[7] + theta[8] + theta[9]),  0, a[9]*cos(theta[0] + theta[7] + theta[8] + theta[9]) + a[7]*cos(theta[0] + theta[7]) + d[2]*cos(theta[0]) + d[5]*cos(theta[0]) + d[3]*sin(theta[0]) + d[6]*sin(theta[0]) + a[8]*cos(theta[0] + theta[7] + theta[8]),
	             		                              0,                                       0, -1,                                                                                                                                                                     													d[1] - d[4],
	             		                              0,                                       0,  0,                                                                                                                                                                            										    		  1;
 

		return T010;
	}

}

void MmuavDirectKinematics::LoadParameters(std::string file)
{
	YAML::Node config = YAML::LoadFile(file);
	std::vector<double> theta, alpha, d, a, left_arm_origin, right_arm_origin;

	theta = config["theta"].as<std::vector<double> >();
	alpha = config["alpha"].as<std::vector<double> >();
	d = config["d"].as<std::vector<double> >();
	a = config["a"].as<std::vector<double> >();

	for (int i=0; i<10; i++)
	{
		dhParams_.theta[i] = theta[i];
		dhParams_.alpha[i] = alpha[i];
		dhParams_.d[i] = d[i];
		dhParams_.a[i] = a[i];
	}
	
	isInit = true;
}
