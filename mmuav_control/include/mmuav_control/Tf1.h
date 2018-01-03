#ifndef TF1_H
#define TF1_H

#include <string>

class Tf1{
	private:
		bool zohTransform(void);
		float d0_, d1_;
		float n0_, n1_;
		float dc0_, dc1_;
		float nc0_, nc1_;
		float T_;
		float x_[2];
		float y_[2];
		bool numeratorInit_, denominatorInit_;

	public:
		Tf1(void);
		bool setDenominator(float d0, float d1);
		bool setNumerator(float n0, float n1);
		float getDiscreteOutput(float input);
		void setSamplingTime(float samplingTime);
		bool c2d(float samplingTime, std::string method);
		void getDiscreteDenominator(float *d0, float *d1);
		void getDiscreteNumerator(float *n0, float *n1);
		void setInitialValues(float *y0, float *x0);
		void reset(void);
	
};

#endif