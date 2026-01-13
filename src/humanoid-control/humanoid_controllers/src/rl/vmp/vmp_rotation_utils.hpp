#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace humanoid_controller
{
namespace vmp
{
namespace rotation_utils
{

/**
 * @brief 将 6D 旋转表示（旋转矩阵的前两行）转换为完整的 3x3 正交旋转矩阵
 * @param mat_6d 2x3 矩阵，包含旋转矩阵的前两行
 * @return 完整的 3x3 旋转矩阵
 */
inline Eigen::Matrix3d complete_orthogonal(const Eigen::Matrix<double, 2, 3>& mat_6d)
{
  Eigen::Vector3d a1 = mat_6d.row(0).normalized();
  Eigen::Vector3d a2 = mat_6d.row(1);

  Eigen::Vector3d b1 = a1;
  Eigen::Vector3d b2 = a2 - (a2.dot(b1)) * b1;
  b2.normalize();
  Eigen::Vector3d b3 = b1.cross(b2);

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix.row(0) = b1;
  rotation_matrix.row(1) = b2;
  rotation_matrix.row(2) = b3;

  return rotation_matrix;
}

/**
 * @brief 将旋转矩阵转换为四元数 [w, x, y, z]
 * @param matrix 3x3 旋转矩阵
 * @return 四元数向量 [w, x, y, z]
 */
inline Eigen::Vector4d mat_to_quat(const Eigen::Matrix3d& matrix)
{
  Eigen::Vector4d quat;
  double trace = matrix.trace();

  if (trace > 0) {
    double s = sqrt(trace + 1.0) * 2;
    quat[0] = 0.25 * s;
    quat[1] = (matrix(2, 1) - matrix(1, 2)) / s;
    quat[2] = (matrix(0, 2) - matrix(2, 0)) / s;
    quat[3] = (matrix(1, 0) - matrix(0, 1)) / s;
  } else if ((matrix(0, 0) > matrix(1, 1)) && (matrix(0, 0) > matrix(2, 2))) {
    double s = sqrt(1.0 + matrix(0, 0) - matrix(1, 1) - matrix(2, 2)) * 2;
    quat[0] = (matrix(2, 1) - matrix(1, 2)) / s;
    quat[1] = 0.25 * s;
    quat[2] = (matrix(0, 1) + matrix(1, 0)) / s;
    quat[3] = (matrix(0, 2) + matrix(2, 0)) / s;
  } else if (matrix(1, 1) > matrix(2, 2)) {
    double s = sqrt(1.0 + matrix(1, 1) - matrix(0, 0) - matrix(2, 2)) * 2;
    quat[0] = (matrix(0, 2) - matrix(2, 0)) / s;
    quat[1] = (matrix(0, 1) + matrix(1, 0)) / s;
    quat[2] = 0.25 * s;
    quat[3] = (matrix(1, 2) + matrix(2, 1)) / s;
  } else {
    double s = sqrt(1.0 + matrix(2, 2) - matrix(0, 0) - matrix(1, 1)) * 2;
    quat[0] = (matrix(1, 0) - matrix(0, 1)) / s;
    quat[1] = (matrix(0, 2) + matrix(2, 0)) / s;
    quat[2] = (matrix(1, 2) + matrix(2, 1)) / s;
    quat[3] = 0.25 * s;
  }

  return quat;
}

/**
 * @brief 将四元数转换为欧拉角 (roll, pitch, yaw)
 * @param quat 四元数向量 [w, x, y, z]
 * @return 欧拉角向量 [roll, pitch, yaw]
 */
inline Eigen::Vector3d get_euler_xyz(const Eigen::Vector4d& quat)
{
  double w = quat[0], x = quat[1], y = quat[2], z = quat[3];

  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  double roll = atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (w * y - z * x);
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp);
  else
    pitch = asin(sinp);

  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  double yaw = atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

/**
 * @brief 将欧拉角转换为旋转矩阵
 * @param euler 欧拉角向量 [roll, pitch, yaw]
 * @return 3x3 旋转矩阵
 */
inline Eigen::Matrix3d euler_to_rotation_matrix(const Eigen::Vector3d& euler)
{
  double roll = euler[0], pitch = euler[1], yaw = euler[2];

  double cr = cos(roll), sr = sin(roll);
  double cp = cos(pitch), sp = sin(pitch);
  double cy = cos(yaw), sy = sin(yaw);

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix(0, 0) = cy * cp;
  rotation_matrix(0, 1) = cy * sp * sr - sy * cr;
  rotation_matrix(0, 2) = cy * sp * cr + sy * sr;
  rotation_matrix(1, 0) = sy * cp;
  rotation_matrix(1, 1) = sy * sp * sr + cy * cr;
  rotation_matrix(1, 2) = sy * sp * cr - cy * sr;
  rotation_matrix(2, 0) = -sp;
  rotation_matrix(2, 1) = cp * sr;
  rotation_matrix(2, 2) = cp * cr;

  return rotation_matrix;
}

/**
 * @brief 对参考运动中的 6D 旋转表示进行 yaw 角归一化（将 yaw 设为 0）
 * @param ref_motion 参考运动向量
 * @param theta_start_id theta 数据起始索引
 * @param theta_end_id theta 数据结束索引（不包含）
 */
inline void normalize_ref_motion_yaw(Eigen::VectorXd& ref_motion, int theta_start_id, int theta_end_id)
{
  if (theta_start_id < 0 || theta_end_id > static_cast<int>(ref_motion.size()) ||
      theta_start_id >= theta_end_id) {
    return;
  }

  int theta_length = theta_end_id - theta_start_id;
  if (theta_length != 6) {
    return;
  }

  Eigen::Matrix<double, 2, 3> mat_6d;
  for (int i = 0; i < 6; ++i) {
    int row = i / 3;
    int col = i % 3;
    mat_6d(row, col) = ref_motion[theta_start_id + i];
  }

  Eigen::Matrix3d rotation_matrix = complete_orthogonal(mat_6d);
  Eigen::Vector4d quat = mat_to_quat(rotation_matrix);
  Eigen::Vector3d euler = get_euler_xyz(quat);

  euler[2] = 0.0;  // Set yaw to 0

  Eigen::Matrix3d normalized_rotation = euler_to_rotation_matrix(euler);

  for (int i = 0; i < 6; ++i) {
    int row = i / 3;
    int col = i % 3;
    ref_motion[theta_start_id + i] = normalized_rotation(row, col);
  }
}

} // namespace rotation_utils
} // namespace vmp
} // namespace humanoid_controller
