/*
 * UavGeometricControl.h
 *
 *	UavGeometricControl class implements geometric control algorithm for
 *	moving-mass UAVs.
 *	Geometric control will handle both height and attitude control
 *	simultaneously.
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#ifndef UAV_GEOMETRIC_CONTROL_H
#define UAV_GEOMETRIC_CONTROL_H

#include "ros/ros.h"

class UavGeometricControl
{

	public:

		/**
		 * Class constructor.
		 *
		 * @param rate - Controller rate.
		 */
		UavGeometricControl(int rate);

		/**
		 * Class destructor.
		 */
		virtual ~UavGeometricControl();

		/**
		 * Runs the geometric control algorithm until ROS shuts down.
		 */
		void run();

	private:

		/**
		 * Controller rate. Frequency at which the loop in run method
		 * will be executed.
		 */
		int controller_rate_ = -1;

		/**
		 * Sleep duration used while performing checks before starting the
		 * run() loop.
		 */
		float sleep_duration_ = -1.0;

		/**
		 * True when first callback function occurred otherwise false.
		 */
		bool start_flag_ = false;
};

#endif /* UAV_GEOMETRIC_CONTROL_H */
