#ifndef PID_H
#define PID_H

#include "ros/ros.h"
#include <mmuav_msgs/PIDController.h>

class PID
{
	private:
		float kp, ki, kd, up, ui, ui_old, ud, u;
		float lim_high, lim_low, ref, meas, error_old;

		bool firstPass;
		
	public:
		PID();
		void reset();
		void set_kp(float invar);
		float get_kp();
		void set_ki(float invar);
		float get_ki();
		void set_kd(float invar);
		float get_kd();
		void set_lim_high(float invar);
		float get_lim_high();
		void set_lim_low(float invar);
		float get_lim_low();
		float compute(float ref_, float meas_, float dt_);
		void get_pid_values(float *up_, float *ui_, float *ud_, float *u_);
		void create_msg(mmuav_msgs::PIDController &msg);

};

#endif