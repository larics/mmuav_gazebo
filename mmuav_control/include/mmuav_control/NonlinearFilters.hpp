/*
 * NonlinearFilters.hpp
 *
 *  Created on: Oct 11, 2018
 *      Author: lmark
 */

#ifndef NONLINEAR_FILTERS_H
#define NONLINEAR_FILTERS_H

namespace nonlinear_filters
{
	/**
	 * Perform saturation filter on the given value;
	 *
	 * @param value
	 * @param lowLimit
	 * @param highLimit
	 *
	 * @return saturated value
	 */
	double saturation(
			double value,
			double lowLimit,
			double highLimit);

	/**
	 * Perform deadzone filter on given value.
	 *
	 * @param value
	 * @param lowLimit
	 * @param highLimit
	 */
	double deadzone(
			double value,
			double lowLimit,
			double highLimit);
}

#endif /* NONLINEAR_FILTERS_H */
