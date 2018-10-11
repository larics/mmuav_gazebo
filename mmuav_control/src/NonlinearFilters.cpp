/*
 * NonlinearFilters.cpp
 *
 *  Created on: Oct 11, 2018
 *      Author: lmark
 */

#include <mmuav_control/NonlinearFilters.hpp>

double nonlinear_filters::saturation(
		double value,
		double lowLimit,
		double highLimit)
{
	if (value > highLimit) { return highLimit; }
	else if (value < lowLimit) { return lowLimit; }
	else { return value; }
}

double nonlinear_filters::deadzone(
		double value,
		double lowLimit,
		double highLimit)
{
	if (value < highLimit && value > lowLimit) { return 0; }
	else { return value; }
}


