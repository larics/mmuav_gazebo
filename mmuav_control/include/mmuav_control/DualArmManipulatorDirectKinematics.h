#ifndef DUALARMMANIPULATORDIRECTKINEMATICS_H
#define DUALARMMANIPULATORDIRECTKINEMATICS_H

#include "ros/ros.h"
#include <math.h>
#include "yaml-cpp/yaml.h"
#include <ros/package.h>
#include <eigen3/Eigen/Eigen>

typedef struct
{
	float theta[3];
	float alpha[3];
	float d[3];
	float a[3];
} DH_Parameters_TypeDef;


class DualArmManipulatorDirectKinematics
{
	public:
		DualArmManipulatorDirectKinematics(void);
		void dk_calculate(float q1, float q2, float q3);
		void LoadParameters(std::string file);

	private:
		DH_Parameters_TypeDef dhParams_;
		Eigen::Matrix4d T13, T01, Torigin_right, Torigin_left;

		bool isInit;
};

#endif