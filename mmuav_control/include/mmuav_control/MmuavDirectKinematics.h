#ifndef MMUAVDIRECTKINEMATICS_H
#define MMUAVDIRECTKINEMATICS_H

#include <math.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>

class MmuavDirectKinematics
{
	public:
		typedef struct
		{
			float theta[10];
			float alpha[10];
			float d[10];
			float a[10];
		} DH_Parameters_TypeDef;

		MmuavDirectKinematics(void);
		Eigen::Matrix4d dk_calculate(float q1, float q2, float q3, float q4, float q5, float q6, float q7);
		void LoadParameters(std::string file);

	private:
		MmuavDirectKinematics::DH_Parameters_TypeDef dhParams_;
		bool isInit;
};

#endif