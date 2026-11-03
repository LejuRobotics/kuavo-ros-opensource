/*! @file orientation_tools.h
 *  @brief Utility functions for 3D rotations
 *
 *  This file contains rotation utilities.  We generally use "coordinate
 * transformations" as opposed to the displacement transformations that are
 * commonly found in graphics.  To describe the orientation of a body, we use a
 * rotation matrix which transforms from world to body coordinates. This is the
 * transpose of the matrix which would rotate the body itself into the correct
 * orientation.
 *
 *  This follows the convention of Roy Featherstone's excellent book, Rigid Body
 * Dynamics Algorithms and the spatial_v2 MATLAB library that comes with it.
 * Note that we don't use the spatial_v2 convention for quaternions!
 */

#ifndef LIBBIOMIMETICS_ORIENTATION_TOOLS_H
#define LIBBIOMIMETICS_ORIENTATION_TOOLS_H

#include <cmath>
#include <iostream>
#include <type_traits>

#include "MathUtilities.h"
#include "cppTypes.h"

Mat3 rpyToRotMat(const Vec3& v);
Quat rpyToQuat(const Vec3& rpy);
Quat quatProduct(const Quat& q1, const Quat& q2);
Vec3 quatToRPY(const Quat& q);
Quat rotationMatrixToQuaternion( const Mat3& r1);
Vec3 rotationMatrixToRPY(const Mat3& R);
Mat3 quaternionToRotationMatrix(const Quat& q);
Mat3 coordinateRotation(CoordinateAxis axis, double theta);
Vec3 rotMatToExp(const Mat3& rm);
Quat quatunify(const Quat &quat);
double anglelimit_pi(double angle);

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);
Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);

Quat negativeYaw_Quat2(Quat &quat);
Quat negativeYaw_R2Q(Mat3 &Rbody);
Mat3 negativeYaw_R2R(Mat3 &Rbody);
Quat removeYaw_Q2Q(Quat &quat);
Vec4 RemoveYaw4ik(Eigen::VectorXd &x_d, Eigen::VectorXd &xd_d, Eigen::VectorXd &xdd_d);
void prevRemoveYaw4ik(Eigen::VectorXd &preq, Eigen::VectorXd &prev, Vec4 &posY);
void AddYaw4ik(Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &a, Vec4 &posY);
double RemoveYaw4pb(Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &vd);
double RemoveYaw4pb(Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &vd, double &yaw);
void AddYaw4pb(Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &vd, double &yaw);
Vec4 RemoveYaw4se(Eigen::VectorXd &r_vec_, Vec3 &p_st, const Quat &quat);
Vec4 AddYaw4se(Eigen::VectorXd &r_vec_, Vec3 &p_st, const Vec4 &posY);

#endif  // LIBBIOMIMETICS_ORIENTATION_TOOLS_H
