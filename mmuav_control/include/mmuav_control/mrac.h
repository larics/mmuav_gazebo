#ifndef MRAC_H
#define MRAC_H

#include <mmuav_control/diff2.h>
#include <mmuav_control/Tf2.h>
#include <string>

class mrac{
	private:
		enum MracType { PARAMETRIC_ADAPTATION, SIGNAL_SYNTHESIS_ADAPTATION};

		float time_, ym0_, dym0_;
		uint8_t rm_type_;
		bool mrac_init_, reference_model_init_;
		diff2 Ym_;
		Tf2 Gm_;
		mrac::MracType mrac_type_;

	public:
		mrac(void);
		bool setType(std::string mrac_type, uint8_t rm_type);
		void initializeReferenceModel(float zeta, float omega);
		void initializeReferenceModel(float *numerator, float *denominator, float samplingTime);
		void referenceModelReset(void);
		void setReferenceModelInitialConditions(float ym0, float dym0);
		float compute(float dt, float e);
};

#endif