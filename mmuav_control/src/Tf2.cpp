#include <mmuav_control/Tf2.h>
#include <cmath>

Tf2::Tf2(void)
{
	T_ = 1.0;
	nc0_ = 0.0;
	nc1_ = 0.0;
	nc2_ = 0.0;
	dc0_ = 0.0;
	dc1_ = 0.0;
	dc2_ = 1.0;

	n0_ = 0.0;
	n1_ = 0.0;
	n2_ = 0.0;
	d0_ = 0.0;
	d1_ = 0.0;
	d2_ = 1.0;

	for (int i = 0; i < 3; i++)
	{
		x_[i] = 0;
		y_[i] = 0;
	}

	numeratorInit_ = false;
	denominatorInit_ = false;

	a_ = 0.0;
	b_ = 0.0;
}

bool Tf2::setNumerator(float n0, float n1, float n2) //n0 bez s-a
{
	nc0_ = n0;
	nc1_ = n1;
	nc2_ = n2;

	numeratorInit_ = true;

	return numeratorInit_;
}

bool Tf2::setDenominator(float d0, float d1, float d2)
{
	float discriminant;

	dc0_ = d0;
	dc1_ = d1;
	dc2_ = d2;

	if (dc2_ == 0.0)
	{
		denominatorInit_ = false;
		return denominatorInit_;
	}

	discriminant = dc1_*dc1_ - 4*dc0_*dc2_;

	if ((fabs(discriminant)*100000) > 1.0) discriminant = 0.0; 

	if (discriminant < 0)
	{
		denominatorInit_ = false;
	}
	else
	{
		a_ = (dc1_ + sqrt(discriminant))/(2*dc2_);
		b_ = (dc1_ - sqrt(discriminant))/(2*dc2_);

		if (discriminant == 0.0) poletype_ = DOUBLE_REAL;
		else poletype_ = SINGLE_REAL;

		denominatorInit_ = true;
	}

	return denominatorInit_;
}

bool Tf2::c2d(float samplingTime, std::string method)
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

bool Tf2::zohTransform(void)
{
	if (numeratorInit_ && denominatorInit_)
	{
		if (poletype_ == SINGLE_REAL)
		{
			n0_ = nc0_ * (b_ * exp(-b_ * T_) - a_ * exp(-a_ * T_) + (a_ - b_) * exp(-T_ * dc1_ / dc2_)) / (dc0_ * (a_ - b_)) + nc1_ * (- (exp(-a_ * T_) - exp(-b_ * T_))) / (dc2_ * (b_ - a_)) + nc2_ * ((a_ / (a_ - b_)) * exp(-b_ * T_) + (b_ / (b_ - a_)) * exp(-a_ * T_)) / (dc2_);
			n1_ = nc0_ * (a_ * (1 + exp(-a_ * T_)) - b_ * (1 + exp(-b_ * T_)) - (a_ - b_) * (exp(-b_ * T_) + exp(-a_ * T_))) / (dc0_ * (a_ - b_)) + nc1_ * (exp(-a_ * T_) - exp(-b_ * T_)) / (dc2_ * (b_ - a_)) + nc2_ * (- (1 + (a_ / (a_ - b_)) * exp(-b_ * T_) + (b_ / (b_ - a_)) * exp(-a_ * T_))) / (dc2_);
			n2_ = nc2_/dc2_;
			d0_ = exp(-T_ * dc1_ / dc2_);
			d1_ = -(exp(-b_ * T_) + exp(-a_ * T_));
			d2_ = 1.0;

			return true;
		}
		else if (poletype_ == DOUBLE_REAL)
		{
			n0_ = nc0_ * (exp(-T_ * a_) * (T_ * a_ - 1 + exp(-T_*a_))) / (dc2_ * a_ * a_) + nc1_ * ( - T_ * exp(-a_ * T_)) / (dc2_) + nc2_ * ((T_ * a_) * exp(-T_ * a_)) / (dc2_);
			n1_ = nc0_ * (1 - exp(-T_ * a_) * (1 + T_ * a_)) / (dc2_ * a_ * a_) + nc1_ * (T_ * exp(-a_ * T_)) / (dc2_) + nc2_ * (- (1 + (T_ * a_ + 1) * exp(-T_ * a_))) / (dc2_);
			n2_ = nc2_/dc2_;
			d0_ = exp(-2 * T_ * a_);
			d1_ = -2*exp(-a_ * T_);
			d2_ = 1.0;

			return true;
		}
	}

	return false;
}

void Tf2::setInitialValues(float *y0, float *x0)
{
	for (int i = 0; i < 3; i++)
	{
		x_[i] = x0[i];
		y_[i] = y0[i];
	}
}

void Tf2::getDiscreteDenominator(float *d0, float *d1, float *d2)
{
	*d0 = d0_;
	*d1 = d1_;
	*d2 = d2_;
}

void Tf2::getDiscreteNumerator(float *n0, float *n1, float *n2)
{
	*n0 = n0_;
	*n1 = n1_;
	*n2 = n2_;
}

void Tf2::reset(void)
{
	for (int i = 0; i < 3; i++)
	{
		x_[i] = 0;
		y_[i] = 0;
	}
}

float Tf2::getDiscreteOutput(float input)
{
	int i;

	for (i = 2; i >= 1; i--)
	{
		x_[i] = x_[i-1];
		y_[i] = y_[i-1];
	}

	x_[0] = input;

	y_[0] = (n2_/d2_)*x_[0] + (n1_/d2_)*x_[1] + (n0_/d2_)*x_[2] - (d1_/d2_)*y_[1] - (d0_/d2_)*y_[2];

	return y_[0];
}