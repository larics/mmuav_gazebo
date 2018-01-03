#include <mmuav_control/mraic.h>
#include <stdlib.h>

mraic::mraic(void)
{
	time_ = 0.0;
	ym0_ = 0.0;
    dym0_ = 0.0;

    g0_ = 0.0;
    kp0_ = 0.0;
    kd0_ = 0.0;
    wp_ = 0.0;
    wd_ = 0.0;

    N_ = 100.0;

    samplingTime_ = 1.0;

	reference_model_init_ = false;
}

void mraic::initializeReferenceModel(float zeta, float omega)
{
	Ym_.init(omega*omega, 2*zeta*omega);

	reference_model_init_ = true;
}

void mraic::initializeAdaptationLaws(float *a, float *b, float *c, float *sigma, float T)
{
    samplingTime_ = T;

    Gg.reset();
    Gg.setNumerator(-1.0, -1.0);
    Gg.setDenominator(0.0, 1.0);
    Gg.c2d(samplingTime_, "zoh");
}

void mraic::setReferenceModelInitialConditions(float ym0, float dym0)
{
    ym0_ = ym0;
    dym0_ = dym0;
}

void mraic::referenceModelReset(void)
{
	time_ = 0.0;
}

void mraic::setAdaptiveParameterInitialValues(float g0, float kp0, float kd0)
{
    g0_ = g0;
    kp0_ = kp0;
    kd0_ = kd0;
}

void mraic::setWeightingFactors(float wp, float wd)
{
    wp_ = wp;
    wd_ = wd;
}

/*float mraic::qt(float* e)
{
    float q;

    q = wp_ * e[0] + wd_ * e[1];

    return q;
}*/

float mraic::getAdaptiveProportionalGainKp()
{
    float kp;

    kp = kp0_ + Gp.getDiscreteOutput(0);

    return kp;
}

float mraic::getAdaptiveDerivativeGainKp()
{
    float kd;

    kd = kd0_ + Gd.getDiscreteOutput(0);

    return kd;
}

float mraic::getReferencePositionSignal(float q)
{
    float g;

    g = g0_ + Gg.getDiscreteOutput(q);

    return g;
}


float mraic::compute(float dt, float e)
{
    float cond[2], u = 0.0;
    float *ym;

    if (reference_model_init_)
    {
    	time_ += dt;

    	cond[0] = ym0_;
        cond[1] = dym0_;
        ym = Ym_.dsolve(time_, cond);
    }

    free(ym);

    return u;
}