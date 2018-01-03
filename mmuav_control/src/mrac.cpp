#include <mmuav_control/mrac.h>

mrac::mrac(void)
{
	time_ = 0.0;
	ym0_ = 0.0;
    dym0_ = 0.0;

	mrac_init_ = false;
	reference_model_init_ = false;
}

bool mrac::setType(std::string mrac_type, uint8_t rm_type)
{

	if (rm_type < 2)
	{
		rm_type_ = rm_type;

		if (mrac_type == "PA")
		{
			mrac_type_ = PARAMETRIC_ADAPTATION;
			mrac_init_ = true;
		}
		else if (mrac_type == "SSA")
		{
			mrac_type_ = SIGNAL_SYNTHESIS_ADAPTATION;
			mrac_init_ = true;
		}
	}

	return mrac_init_;
}

void mrac::initializeReferenceModel(float zeta, float omega)
{
	Ym_.init(omega*omega, 2*zeta*omega);

	reference_model_init_ = true;
}

void mrac::initializeReferenceModel(float *numerator, float *denominator, float samplingTime)
{
	Gm_.reset();
    Gm_.setNumerator(numerator[0], numerator[1], numerator[2]);
    Gm_.setDenominator(denominator[0], denominator[1], denominator[2]);
    Gm_.c2d(samplingTime, "zoh");

    reference_model_init_ = true;
}

void mrac::setReferenceModelInitialConditions(float ym0, float dym0)
{
    ym0_ = ym0;
    dym0_ = dym0;
}

void mrac::referenceModelReset(void)
{
	time_ = 0.0;
}

float mrac::compute(float dt, float e)
{
    float cond[2], u = 0.0;
    float *ym;

    if (reference_model_init_ && mrac_init_)
    {
    	time_ += dt;

    	if (mrac_type_ == PARAMETRIC_ADAPTATION)
    	{
    		if (rm_type_ == 0)
    		{
        		cond[0] = ym0_;
        		cond[1] = dym0_;
        		ym = Ym_.dsolve(time_, cond);
    		}
    		else if (rm_type_ == 1)
    		{

    		}
    	}
    }

    free(ym);

    return u;
}