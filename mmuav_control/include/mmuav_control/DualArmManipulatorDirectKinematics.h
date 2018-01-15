#ifndef DUALARMMANIPULATORDIRECTKINEMATICS_H
#define DUALARMMANIPULATORDIRECTKINEMATICS_H

#include <math.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>

class DualArmManipulatorDirectKinematics
{
	public:
		typedef struct
		{
			float theta[3];
			float alpha[3];
			float d[3];
			float a[3];
		} DH_Parameters_TypeDef;

		DualArmManipulatorDirectKinematics(void);
		Eigen::Matrix4d dk_calculate(float q1, float q2, float q3);
		void LoadParameters(std::string file);

	private:
		DualArmManipulatorDirectKinematics::DH_Parameters_TypeDef dhParams_;
		bool isInit;
};

#endif