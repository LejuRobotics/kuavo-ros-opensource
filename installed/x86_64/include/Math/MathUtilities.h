/*! @file MathUtilities.h
 *  @brief Utility functions for math
 *
 */

#ifndef PROJECT_MATHUTILITIES_H
#define PROJECT_MATHUTILITIES_H

#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <cmath>
#include "cppTypes.h"

using namespace std;
using namespace Eigen;

#define rad2deg  57.295779513082320876798154814105	    // (180.0/pi)
#define deg2rad  0.017453292519943295769236907684886	// (pi/180.0)
/*!
 * Square a number
 */
double square(double a);
double min(double a, double b);
double max(double a, double b);
Mat3 skew(const Vec3 &v);
double saturation(const double a, Vec2 limits);
Eigen::Matrix3d LieGamma(const Eigen::Vector3d& w, int n);
Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);
MatrixXd LieAdjoint_SEK3(const MatrixXd& X);
MatrixXd LieAdjoint(Eigen::MatrixXd X);
Vec4 QuaterniondExp(Vec3 w);
Vec4 Quaternion_Multiply(const Vec4& p, const Vec4& q);


Vec2 Contactsmooth(Vec2 contact, bool a, bool b);
Eigen::Matrix4d quat_rate(Eigen::Vector3d v);
Eigen::MatrixXd getFk( const Eigen::Vector3d& a, 
                        const Eigen::Vector3d& g, 
                        const Eigen::MatrixXd& R, 
                        const Eigen::Vector3d& v, 
                        const Eigen::Vector3d& p, 
                        const Eigen::Vector3d& dL, 
                        const Eigen::Vector3d& dR, 
                        const double& dt);
Eigen::MatrixXd getPhi(const Eigen::Vector3d& w, 
                        const Eigen::Vector3d& a, 
                    const Eigen::Vector3d& g, 
                        const Eigen::MatrixXd& R, 
                        const Eigen::Vector3d& v_pred, 
                        const Eigen::Vector3d& p_pred, 
                        const Eigen::Vector3d &dL_pred, 
                        const Eigen::Vector3d &dR_pred, 
                        const double& dt);

#endif  // PROJECT_MATHUTILITIES_H
