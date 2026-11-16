/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <vector>
#include <eigen3/Eigen/Dense>


// Rotation Matrix
using RotMat = typename Eigen::Matrix<double, 3, 3>;

// 2x1 Vector
using Vec2 = typename Eigen::Matrix<double, 2, 1>;

// 3x1 Vector
using Vec3 = typename Eigen::Matrix<double, 3, 1>;

// 4x1 Vector
using Vec4 = typename Eigen::Matrix<double, 4, 1>;

// 6x1 Vector
using Vec6 = Eigen::Matrix<double, 6, 1>;

// 10x1 Vector
using Vec10 = Eigen::Matrix<double, 10, 1>;

// 12x1 Vector
using Vec12 = Eigen::Matrix<double, 12, 1>;

// 18x1 Vector
using Vec18 = Eigen::Matrix<double, 18, 1>;

// 28x1 vector
using Vec28 = Eigen::Matrix<double, 28, 1>;

// 3x3 Matrix
using Mat3 = Eigen::Matrix<double, 3, 3>;

// 4x1 Vector
using Quat = Eigen::Matrix<double, 4, 1>;

// Spatial Vector (6x1, all subspaces)
using SVec = Eigen::Matrix<double, 6, 1>;

// Spatial Transform (6x6)
using SXform = typename Eigen::Matrix<double, 6, 6>;

// 4x4 Matrix
using Mat4 = typename Eigen::Matrix<double, 4, 4>;

// 6x6 Matrix
using Mat6 = typename Eigen::Matrix<double, 6, 6>;

enum class CoordinateAxis { X, Y, Z };

#endif  // PROJECT_CPPTYPES_H
