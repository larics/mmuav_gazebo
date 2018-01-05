#include <mmuav_control/mraic.h>
#include <stdlib.h>
#include <iostream>

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

    kp_ = 0.0;
    kd_ = 0.0;
    g_ = 0.0;

    e_[0] = 0.0;
    e_[1] = 0.0;
    de_ = 0.0;

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
    Gg.setNumerator(a[0], a[1]);
    Gg.setDenominator(0.0, 1.0);
    Gg.c2d(samplingTime_, "zoh");

    Gp.reset();
    Gp.setNumerator(b[0], b[1]);
    Gp.setDenominator(0.0, 1.0);
    Gp.c2d(samplingTime_, "zoh");

    Gd.reset();
    Gd.setNumerator(c[0], c[1]);
    Gd.setDenominator(0.0, 1.0);
    Gd.c2d(samplingTime_, "zoh");
}

void mraic::setReferenceModelInitialConditions(float ym0, float dym0)
{
    ym0_ = ym0;
    dym0_ = dym0;
}

void mraic::setImpact(bool impact)
{
    if (impact)
    {
        time_ = 0.0;
        setReferenceModelInitialConditions(e_[0], de_);
    }
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

float mraic::calculateAdaptiveProportionalGainKp(float q, float e, float de)
{
    float kp, p;

    p = q * e;

    kp = kp0_ + Gp.getDiscreteOutput(p);

    return kp;
}

float mraic::calculateAdaptiveDerivativeGainKd(float q, float e, float de)
{
    float kd, d;

    d = q * de;
    
    kd = kd0_ + Gd.getDiscreteOutput(d);

    return kd;
}

float mraic::calculateReferencePositionSignal(float q)
{
    float g;

    g = g0_ + Gg.getDiscreteOutput(q);

    return g;
}

float mraic::getAdaptiveProportionalGainKp(void)
{
    return kp_;
}

float mraic::getAdaptiveDerivativeGainKd(void)
{
    return kd_;
}

float mraic::getReferencePositionSignal(void)
{
    return g_;
}


float mraic::compute(float dt, float e)
{
    float cond[2], u = 0.0;
    float q;
    float *ym;

    if (reference_model_init_)
    {
        e_[1] = e_[0];
        e_[0] = e;

    	time_ += dt;

        de_ = (e_[0] - e_[1]) / dt;

    	cond[0] = ym0_;
        cond[1] = dym0_;
        ym = Ym_.dsolve(time_, cond);

        q = wp_ * (e - ym[0]) + wd_ * (de_ - ym[1]);
        kp_ = calculateAdaptiveProportionalGainKp(q, e_[0], de_);
        kd_ = calculateAdaptiveDerivativeGainKd(q, e_[0], de_);
        g_ = calculateReferencePositionSignal(q);

        u = g_ + kp_ * e_[0] + kd_ * de_;
    }

    free(ym);

    return u;
}