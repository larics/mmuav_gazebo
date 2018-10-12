/*
 * MmuavParameters.cpp
 *
 *  Created on: Oct 12, 2018
 *      Author: lmark
 */


#include <mmuav_control/MmuavParameters.hpp>

void mmuav_params::initializeBodyInertia(Matrix<double, 3, 3> &inertia)
{
	inertia.setZero(3, 3);
	inertia(0, 0) = 0.0826944;
	inertia(1, 1) = 0.0826944;
	inertia(2, 2) = 0.0104;
}

void mmuav_params::initializeMovableMassInertia(Matrix<double, 3, 3> &inertia)
{
	inertia.setZero(3, 3);
	inertia(0, 0) = MM_MASS * 0.085 * 0.085;
	inertia(1, 1) = MM_MASS * 0.085 * 0.085;
	inertia(2, 2) = MM_MASS * 0.085 * 0.085;

}

void mmuav_params::initializeThrustTransform(Matrix<double, 4, 4> &transform)
{
	// Initialize thrust transform matrix
	transform.setZero(4, 4);

	// First row
	transform(0, 0) = 1;
	transform(0, 1) = 1;
	transform(0, 2) = 1;
	transform(0, 3) = 1;

	// Second row
	transform(1, 1) = D;
	transform(1, 3) = - D;

	// Third row
	transform(2, 0) = - D;
	transform(2, 2) = D;

	// Fourth row
	transform(3, 0) = MOMENT_CONSTANT;
	transform(3, 1) = - MOMENT_CONSTANT;
	transform(3, 2) = MOMENT_CONSTANT;
	transform(3, 3) = - MOMENT_CONSTANT;

	// Invert the matrix
	transform = transform.inverse().eval();
}
