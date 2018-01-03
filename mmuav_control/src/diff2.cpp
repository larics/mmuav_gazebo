#include <mmuav_control/diff2.h>
#include <stdlib.h>
#include <cmath>

diff2::diff2(void)
{
	a1_ = 0;
	a0_ = 0;

	r1_ = 0;
	r2_ = 0;

	alpha_ = 0;
	beta_ = 0;

	diff2_init_ = false;
}

float* diff2::dsolve(float x, float *cond)
{
	float *y = (float * )malloc(sizeof(float)*2);
	float c1, c2;

	if (diff2_init_)
	{
		if (diff2_type_ == COMPLEX_CONJUGATE)
		{
			c1 = cond[0];
			c2 = (cond[1] - alpha_ * cond[0]) / beta_;

			y[0] = c1 * exp(alpha_ * x) * cos(beta_ * x) + c2 * exp(alpha_ * x) * sin(beta_ * x);
			y[1] = c1 * (alpha_ * exp(alpha_ * x) * cos(beta_ * x) - beta_ * exp(alpha_ * x) * sin(beta_ * x)) + c2 * (alpha_ * exp(alpha_ * x) * sin(beta_ * x) + beta_ * exp(alpha_ * x) * cos(beta_ * x));
		}
		else if (diff2_type_ == DOUBLE_REAL)
		{
			c1 = cond[0];
			c2 = cond[1] - r1_ * cond[0];

			y[0] = c1 * exp(r1_ * x) + c2 * x * exp(r1_ * x);
			y[1] = r1_ * c1 * exp(r1_ * x) + c2 * exp(r1_ * x) + r1_ * c2 * x * exp(r1_ * x);
		}
		else if (diff2_type_ == SINGLE_REAL)
		{
			c1 = (r2_ * cond[0] + cond[1]) / (r2_ - r1_);
			c2 = (r1_ * cond[0] - cond[1]) / (r1_ - r2_);

			y[0] = c1 * exp(r1_ * x) + c2 * exp(r2_ * x);
			y[1] = r1_ * c1 * exp(r1_ * x) + r2_ * c2 * exp(r2_ * x);
		}
	}

	return y; 

}

void diff2::init(float a0, float a1)
{
	float discriminant;

	a1_ = a1;
	a0_ = a0;

	discriminant = a1_*a1_ - 4*a0_;

	if ((fabs(discriminant)*100000) > 1.0) discriminant = 0.0; 

	if (discriminant < 0.0)
	{
		diff2_type_ = COMPLEX_CONJUGATE;

		alpha_ = - a1_ / 2.0;
		beta_ = sqrt(fabs(discriminant)) / 2.0;

	}
	else if (discriminant == 0.0)
	{
		diff2_type_ = DOUBLE_REAL;

		r1_ = - a1_ / 2.0;
	}
	else
	{
		diff2_type_ = SINGLE_REAL;

		r1_ = (-a1_ + sqrt(discriminant)) / 2.0;
		r2_ = (-a1_ - sqrt(discriminant)) / 2.0;
	}


	diff2_init_ = true;
}
