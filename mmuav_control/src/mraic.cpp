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
    q_ = 0.0;
    u_ = 0.0;

    e_[0] = 0.0;
    e_[1] = 0.0;
    de_ = 0.0;

    ym_[0] = 0.0;
    ym_[1] = 0.0;

    error_median.init(5);

    samplingTime_ = 1.0;

	reference_model_init_ = false;
    impact_ = false;
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
    Gg.setNumerator(-a[0], -a[1]);
    Gg.setDenominator(0.0, 1.0);
    Gg.c2d(samplingTime_, "zoh");

    Gp.reset();
    Gp.setNumerator(-b[0], -b[1]);
    Gp.setDenominator(0.0, 1.0);
    Gp.c2d(samplingTime_, "zoh");

    Gd.reset();
    Gd.setNumerator(-c[0], -c[1]);
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
    impact_ = impact;
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
    float cond[2];
    float *ym;

    if (reference_model_init_)
    {
        e_[1] = e_[0];
        e_[0] = e;

    	time_ += dt;

        de_ = error_median.filter((e_[0] - e_[1]) / dt);

        if (impact_)
        {
            time_ = 0.0;
            setReferenceModelInitialConditions(e_[0], 0);
        }

    	cond[0] = ym0_;
        cond[1] = dym0_;
        ym = Ym_.dsolve(time_, cond);
        ym_[0] = ym[0];
        ym_[1] = ym[1];

        q_ = wp_ * (ym[0] - e) + wd_ * (ym[1] - de_);
        kp_ = calculateAdaptiveProportionalGainKp(q_, e_[0], de_);
        kd_ = calculateAdaptiveDerivativeGainKd(q_, e_[0], de_);
        g_ = calculateReferencePositionSignal(q_);

        u_ = g_ + kp_ * e_[0] + kd_ * de_;
    }

    free(ym);

    return u_;
}

void mraic::create_msg(mmuav_msgs::MRAIController &msg)
{
    /* Returns ros message of type MARIController */
    msg.q = q_;
    msg.kp = kp_;
    msg.kd = kd_;
    msg.g = g_;
    msg.ym = ym_[0];
    msg.dym = ym_[1];
    msg.e = e_[0];
    msg.de = de_;
    msg.u = u_;
    msg.header.stamp = ros::Time::now();
}