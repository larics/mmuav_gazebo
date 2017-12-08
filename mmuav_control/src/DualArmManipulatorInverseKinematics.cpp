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

void DualArmManipulatorInverseKinematics::ik_calculate(float x, float y, float rot_z, float *q1, float *q2, float *q3)
{
	if (isInit)
	{
		float t1, t2, w1, w2, w6, temp1, temp2;

		w1 = x;
		w2 = y;
		w6 = rot_z;

		t1 = w1 - dhParams_.a[2]*cos(w6);
		t2 = w2 - dhParams_.a[2]*sin(w6);

		temp1 = (t1*t1 + t2*t2 - dhParams_.a[0]*dhParams_.a[0] - dhParams_.a[1]*dhParams_.a[1])/(2*dhParams_.a[0]*dhParams_.a[1]);

		q2[0] = acos(temp1);
		q2[1] = -acos(temp1);

		temp1 = t2*dhParams_.a[0] + cos(q2[0])*t2*dhParams_.a[1] - sin(q2[0])*t1*dhParams_.a[1];
		temp2 = t1*dhParams_.a[0] + sin(q2[0])*t2*dhParams_.a[1] + cos(q2[0])*t1*dhParams_.a[1];

		q1[0] = atan2(temp1, temp2);

		temp1 = t2*dhParams_.a[0] + cos(q2[1])*t2*dhParams_.a[1] - sin(q2[1])*t1*dhParams_.a[1];
		temp2 = t1*dhParams_.a[0] + sin(q2[1])*t2*dhParams_.a[1] + cos(q2[1])*t1*dhParams_.a[1];

		q1[1] = atan2(temp1, temp2);

		q3[0] = w6 - q1[0] - q2[0];
		q3[1] = w6 - q1[1] - q2[1];

		q1[0] = q1[0] - dhParams_.theta[0];
		q2[0] = q2[0] - dhParams_.theta[1];
		q3[0] = q3[0] - dhParams_.theta[2];

		q1[1] = q1[1] - dhParams_.theta[0];
		q2[1] = q2[1] - dhParams_.theta[1];
		q3[1] = q3[1] - dhParams_.theta[2];
	}

}