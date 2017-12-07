#ifndef DUALARMMANIPULATORINVERSEINEMATICS_H
#define DUALARMMANIPULATORINVERSEINEMATICS_H


#include "ros/ros.h"
#include <math.h>
#include "yaml-cpp/yaml.h"
#include <ros/package.h>
#include <eigen3/Eigen/Eigen>
#include <mmuav_control/DualArmManipulatorDirectKinematics.h>

class DualArmManipulatorInverseKinematics
{
	public:
		DualArmManipulatorInverseKinematics(void);
		void LoadParameters(std::string file);
	private:
		DH_Parameters_TypeDef dhParams_;
		bool isInit;
};


#endif