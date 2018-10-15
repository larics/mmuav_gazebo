/*
 * GeometricHelper.hpp
 *
 *	Auxiliary class used by Geometric controller.
 *
 *  Created on: Oct 11, 2018
 *      Author: lmark
 */

#ifndef GEOMETRIC_HELPER_H
#define GEOMETRIC_HELPER_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;
/**
 * Set of eigen vector constants.
 */
const Matrix<double, 3, 1> E1(1, 0, 0);
const Matrix<double, 3, 1> E2(0, 1, 0);
const Matrix<double, 3, 1> E3(0, 0, 1);

// Eigen matrix
const Matrix<double, 3, 3> EYE3 = Matrix3d::Identity(3, 3);

namespace geom_helper
{
	/**
	 * Perform quaternion to euler transformation.
	 *
	 * @param quaternion: 4 dimensional float array.
	 * @param euler: 3 dimensional float array.
	 */
	void quaternion2euler(float *quaternion, float *euler);

	/**
	 * Perform hat operator on given vector components.
	 *
	 * @param x - x vector component
	 * @param y - y vector component
	 * @param z - z vector component
	 * @param hatMatrixs - Matrix of the following form:
	 * 	[ 0  -z,  y]
	 * 	[ z,  0, -x]
	 * 	[-y,  x,  0]
	 */
	void hatOperator(
			const double x,
			const double y,
			const double z,
			Matrix<double, 3, 3> &hatMatrix);

	/**
	 * Perform a vee( V ) operator on a given hatMatrix.
	 * It is implied that the given hatMatrix is a skew matrix.
	 * It will decompose the skew matrix into a given veeVector reference.
	 *
	 * @param hatMatrx
	 * @param veeVector
	 */
	void veeOperator(
			const Matrix<double, 3, 3> hatMatrix,
			Matrix<double, 3, 1> &veeVector);
	/**
	 * Euler angles represented as a rotation matrix.
	 *
	 * @param roll
	 * @param pitch
	 * @param yaw
	 * @param rotMatrix - Rotation matrix will be stored here
	 */
	void euler2RotationMatrix(
			const double roll,
			const double pitch,
			const double yaw,
			Matrix<double, 3, 3> &rotMatrix);
}

#endif /* GEOMETRIC_HELPER_H */
