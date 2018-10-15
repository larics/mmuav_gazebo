/*
 * MmuavParameters.hpp
 *
 *	Contains parameter constants for \mu Morus UAV.
 *	Additional mass and manipulator constants are also defined.
 *
 *  Created on: Oct 11, 2018
 *      Author: lmark
 */

#ifndef MMUAV_PARAMETERS_H
#define MMUAV_PARAMETERS_H

#include <eigen3/Eigen/Dense>

const double G = 9.81;

// UAV constants
const double UAV_MASS = 2.083;
const double ARM_LENGTH = 0.314;
const double MOMENT_CONSTANT = 0.016;
const double MOTOR_CONSTANT = 8.54858e-06;

// ROTOR constants
const double ROTOR_MASS = 0.01;
const double ROTOR_VELOCITY_SLOWDOWN_SIM = 15;
const double ROTOR_RADIUS = 0.1524;
const double ROTOR_OFFSET_TOP = 0.04579;
const double MIN_ROTOR_VELOCITY = 0;
const double MAX_ROTOR_VELOCITY = 1475;
const double D =  ARM_LENGTH + ROTOR_RADIUS / 2;
const double MAXIMUM_MOMENT =
		MAX_ROTOR_VELOCITY * MAX_ROTOR_VELOCITY * MOTOR_CONSTANT // MAX FORCE
		* D;

// Moving mass constants
const double MM_MASS = 0.208;
const double MM_FORCE = MM_MASS * G;

// Payload constant
const double PAYLOAD_MASS = 0.25;
const double TOTAL_LINK_MASS = 0.13 * 2;
const double PAYLOAD_FORCE = PAYLOAD_MASS * G;

using namespace Eigen;

namespace mmuav_params
{
	/**
	 * Set initial values for UAV body inertia matrix.
	 */
	void initializeBodyInertia(Matrix<double, 3, 3> &inertia);

	/**
	 * Set initial values for movable mass inertia matrix.
	 */
	void initializeMovableMassInertia(Matrix<double, 3, 3> &inertia);

	/**
	 * Set initial values for payload inertia.
	 */
	void initializePayloadInertia(Matrix<double, 3, 3> &inertia);

	/**
	 * Initialize transformation matrix which converts
	 * total force and moments to forces produced by each rotor.
	 */
	void initializeThrustTransform(Matrix<double, 4, 4> &transform);
}

#endif /* MMUAV_PARAMETERS_H */
