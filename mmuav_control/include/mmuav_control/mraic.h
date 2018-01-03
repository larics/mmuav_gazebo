#ifndef MRAIC_H
#define MRAIC_H

#include <mmuav_control/diff2.h>
#include <mmuav_control/Tf1.h>
#include <stdint.h>

class mraic{
	private:
		float getAdaptiveProportionalGainKp();
		float getAdaptiveDerivativeGainKp();
		float getReferencePositionSignal(float q);

		float time_, ym0_, dym0_, samplingTime_;
		float a_[2], b_[2], c_[2], sigma_[2];
		float g0_, kp0_, kd0_, wp_, wd_, N_;
		uint8_t rm_type_;
		bool reference_model_init_;
		diff2 Ym_;
		Tf1 Gg, Gd, Gp;


	public:
		mraic(void);
		void initializeReferenceModel(float zeta, float omega);
		void referenceModelReset(void);
		void setReferenceModelInitialConditions(float ym0, float dym0);
		void setAdaptiveParameterInitialValues(float g0, float kp0, float kd0);
		void initializeAdaptationLaws(float *a, float *b, float *c, float *sigma, float T);
		void setWeightingFactors(float wp, float wd);
		float compute(float dt, float e);
};

#endif