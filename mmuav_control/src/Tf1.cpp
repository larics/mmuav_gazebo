#include <mmuav_control/Tf1.h>
#include <cmath>

Tf1::Tf1(void)
{
	T_ = 1.0;
	nc0_ = 0.0;
	nc1_ = 0.0;
	dc0_ = 0.0;
	dc1_ = 0.0;

	n0_ = 0.0;
	n1_ = 0.0;
	d0_ = 0.0;
	d1_ = 0.0;

	for (int i = 0; i < 2; i++)
	{
		x_[i] = 0;
		y_[i] = 0;
	}

	numeratorInit_ = false;
	denominatorInit_ = false;
}

bool Tf1::setNumerator(float n0, float n1) //n0 bez s-a
{
	nc0_ = n0;
	nc1_ = n1;

	numeratorInit_ = true;

	return numeratorInit_;
}

bool Tf1::setDenominator(float d0, float d1)
{
	float discriminant;

	dc0_ = d0;
	dc1_ = d1;

	if (dc1_ == 0.0)
	{
		denominatorInit_ = false;
	}
	else 
	{
		denominatorInit_ = true;
	}

	return denominatorInit_;
}

bool Tf1::c2d(float samplingTime, std::string method)
{
	if (samplingTime > 0.0) T_ = samplingTime;
	else return false;

	if (method == "zoh")
	{
		return zohTransform();
	}
	else
	{
		return false;
	}
}

bool Tf1::zohTransform(void)
{
	if (numeratorInit_ && denominatorInit_)
	{
		if (dc0_ == 0.0)
		{
			n0_ = - nc1_ / dc1_ + nc0_ * T_ / dc1_;
			n1_ = nc1_ / dc1_;
			d0_ = - exp( - dc0_ * T_ / dc1_);
			d1_ = 1;

			return true;
		}
		else if (dc0_ != 0.0)
		{
			n0_ = - nc1_ / dc1_ + nc0_ * (1 - exp(- dc0_ * T_ / dc1_)) / dc0_;
			n1_ = nc1_ / dc1_;
			d0_ = - exp( - dc0_ * T_ / dc1_);
			d1_ = 1;

			return true;
		}
	}

	return false;
}

void Tf1::setInitialValues(float *y0, float *x0)
{
	for (int i = 0; i < 2; i++)
	{
		x_[i] = x0[i];
		y_[i] = y0[i];
	}
}

void Tf1::getDiscreteDenominator(float *d0, float *d1)
{
	*d0 = d0_;
	*d1 = d1_;
}

void Tf1::getDiscreteNumerator(float *n0, float *n1)
{
	*n0 = n0_;
	*n1 = n1_;
}

void Tf1::reset(void)
{
	for (int i = 0; i < 2; i++)
	{
		x_[i] = 0;
		y_[i] = 0;
	}
}

float Tf1::getDiscreteOutput(float input)
{
	int i;

	x_[1] = x_[0];
	y_[1] = y_[0];

	x_[0] = input;
	y_[0] = (n1_/d1_)*x_[0] + (n0_/d1_)*x_[1] - (d0_/d1_)*y_[1];

	return y_[0];
}