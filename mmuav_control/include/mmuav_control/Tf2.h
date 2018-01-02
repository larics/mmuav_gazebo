#ifndef TF2_H
#define TF2_H

#include <string>

enum PoleType { DOUBLE_REAL, SINGLE_REAL};

class Tf2{
	private:
		bool zohTransform(void);
		float d0_, d1_, d2_;
		float n0_, n1_, n2_;
		float dc0_, dc1_, dc2_;
		float nc0_, nc1_, nc2_;
		float T_, a_, b_;
		float x_[3];
		float y_[3];
		PoleType poletype_;
		bool numeratorInit_, denominatorInit_;

	public:
		Tf2(void);
		bool setDenominator(float d0, float d1, float d2);
		bool setNumerator(float n0, float n1, float n2);
		float getDiscreteOutput(float input);
		void setSamplingTime(float samplingTime);
		bool c2d(float samplingTime, std::string method);
		void getDiscreteDenominator(float *d0, float *d1, float *d2);
		void getDiscreteNumerator(float *n0, float *n1, float *n2);
		void setInitialValues(float *y0, float *x0);
		void reset(void);
	
};

#endif