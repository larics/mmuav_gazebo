#ifndef DUALARMMANIPULATORINVERSEINEMATICS_H
#define DUALARMMANIPULATORINVERSEINEMATICS_H

#include <math.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>
#include <mmuav_control/DualArmManipulatorDirectKinematics.h>

class DualArmManipulatorInverseKinematics
{
	public:
		DualArmManipulatorInverseKinematics(void);
		void LoadParameters(std::string file);
		void ik_calculate(float x, float y, float rot_z, float *q1, float *q2, float *q3);
		void setDHparams(DH_Parameters_TypeDef dhParams);
	private:
		DH_Parameters_TypeDef dhParams_;
		bool isInit;
};


#endif