#include <mmuav_control/PID.h>
#include <limits>

PID::PID()
{
    /*
        Initializes PID gains (proportional - kp, integral - ki, derivative - kd) and control values to zero.
    */

    // initialize gains
    kp = 0;     // proportional gain
    ki = 0;     // integral gain
    kd = 0;     // derivative gain

    // initialize control values
    up = 0;                                              // P part
    ui = 0;                                              // I part
    ui_old = 0;                                          // I part from previous step
    ud = 0;                                              // D part
    u = 0;                                               // total control value
    lim_high = std::numeric_limits<float>::max();       // control value upper limit
    lim_low = -std::numeric_limits<float>::max();        // control value lower limit

    // init referent control value (set-value)
    ref = 0;

    // init measure control value
    meas = 0;

    // init error from the previous algorithm step
    error_old = 0;

    // flag indicates first step of the algorithm
    firstPass = true;
}

void PID::reset()
{
    // Resets pid algorithm by setting all P,I,D parts to zero
    up = 0;
    ui = 0;
    ui_old = 0;
    ud = 0;
    u = 0;
}

void PID::set_kp(float invar)
{
    /* Set proportional gain. */
    kp = invar;
}

float PID::get_kp()
{
    /* Returns proportional gain */
    return kp;
}

void PID::set_ki(float invar)
{
    /* Set integral gain. */
    ki = invar;
}

float PID::get_ki()
{
    /* Returns integral gain */
    return ki;
}

void PID::set_kd(float invar)
{
    /* Set derivative gain. */
    kd = invar;
}

float PID::get_kd()
{
    /* Returns derivative gain */
    return kd;
}

void PID::set_lim_high(float invar)
{
    /* Set PID upper limit value */
    lim_high = invar;
}

float PID::get_lim_high()
{
    /* Returns PID upper limit value */
    return lim_high;
}

void PID::set_lim_low(float invar)
{
    /* Set PID lower limit value */
    lim_low = invar;
}

float PID::get_lim_low()
{
    /* Returns PID lower limit value */
    return lim_low;
}

float PID::compute(float ref_, float meas_, float dt_)
{
    /*
    Performs a PID computation and returns a control value based on
    the elapsed time (dt) and the error signal.
        :param ref: referent value
        :param meas: measured value
        :return: control value
    */

    float error, de, dt;

    ref = ref_;
    meas = meas_;
    dt = dt_;
        
    if (firstPass)
    {    
        // This is the first step of the algorithm
        // Init time stamp and error
        error_old = ref - meas;
        firstPass = false;
    }
    else
    {
        /*//////////////////////////////////////////////
        //////////////////////////////////////////////*/
        // Add your code here
        // You should compute elapsed time from the previous step (dt, in seconds),
        // compute proportional part (self.up),
        // compute integral part (self.ui),
        // compute derivative part (self.ud),
        // compute total control value (self.u),
        // implement saturation function and antiwind-up,
        
        error = ref - meas;

                            
        de = error - error_old;                // diff error
        up = kp * error;                       // proportional term

                            
        if (ki == 0)
            ui = 0;
        else
            ui = ui_old + ki * error * dt;    // integral term
                            
        ud = kd * de / dt;                    // derivative term
                            
        u = up + ui + ud;
                            
        if (u > lim_high)
        {
            u =  lim_high;
            ui = ui_old;                      // antiwind up
        }
        else if (u < lim_low)
        {        
            u =  lim_low;
            ui = ui_old;                      // antiwind up
        }
                            
        ui_old = ui;                          // save ui for next step

        error_old = error;
                            
        /* End of added code
        ///////////////////////////////////////////////
        ///////////////////////////////////////////////
        */                       
    }

    return u;
}

void PID::get_pid_values(float *up_, float *ui_, float *ud_, float *u_)
{
    /*Returns P, I, D components and total control value */
    
    *up_ = up;
    *ui_ = ui;
    *ud_ = ud;
    *u_ = u;
}

void PID::create_msg(mmuav_msgs::PIDController &msg)
{
    /* Returns ros message of type PIDController */

    msg.ref = ref;
    msg.meas = meas;
    msg.P = up;
    msg.I = ui;
    msg.D = ud;
    msg.U = u;
    msg.header.stamp = ros::Time::now();
}
