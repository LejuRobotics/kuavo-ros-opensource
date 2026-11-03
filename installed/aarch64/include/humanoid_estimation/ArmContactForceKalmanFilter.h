//
// Created for arm contact force estimation with Kalman filtering
// Date: 2025-10-27
//

#pragma once

#include <Eigen/Dense>

namespace ocs2 {
namespace humanoid {

/**
 * @brief 基础卡尔曼滤波器，用于手臂接触力估计
 * 
 * 状态向量：x = [F_left, F_right]^T (12维)
 * - F_left: 左手6DOF力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz]
 * - F_right: 右手6DOF力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz]
 * 
 * 过程模型：x(k+1) = x(k) + w(k)  (随机游走模型)
 * 观测模型：z(k) = x(k) + v(k)
 */
class ArmContactForceKalmanFilter {
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;
  
  /**
   * @brief 构造函数
   * @param state_dim 状态维度（默认12维：左右手各6DOF）
   */
  explicit ArmContactForceKalmanFilter(int state_dim = 12);
  
  /**
   * @brief 预测步骤
   * @param dt 时间步长（秒）
   */
  void predict(double dt);
  
  /**
   * @brief 更新步骤
   * @param measurement 观测值（12维力/力矩向量）
   */
  void update(const VectorXd& measurement);
  
  /**
   * @brief 更新步骤（带自定义观测噪声）
   * @param measurement 观测值
   * @param R_custom 自定义观测噪声协方差矩阵
   */
  void update(const VectorXd& measurement, const MatrixXd& R_custom);
  
  /**
   * @brief 获取当前状态估计
   * @return 状态估计向量（12维）
   */
  VectorXd getEstimate() const { return x_est_; }
  
  /**
   * @brief 设置过程噪声协方差矩阵Q
   * @param Q 过程噪声协方差（12x12）
   */
  void setProcessNoise(const MatrixXd& Q) { Q_ = Q; }
  
  /**
   * @brief 设置观测噪声协方差矩阵R
   * @param R 观测噪声协方差（12x12）
   */
  void setMeasurementNoise(const MatrixXd& R) { R_ = R; }
  
  /**
   * @brief 重置滤波器到初始状态
   */
  void reset();
  
  /**
   * @brief 计算自适应观测噪声协方差R
   * @param jacobian_condition 雅可比矩阵条件数（奇异性指标）
   * @param contact_confidence 接触置信度（0-1）
   * @return 自适应的R矩阵
   */
  MatrixXd computeAdaptiveR(double jacobian_condition, double contact_confidence) const;
  
  /**
   * @brief 获取当前协方差估计P的迹（用于评估估计不确定性）
   */
  double getEstimateUncertainty() const;
  
  /**
   * @brief 获取新息（innovation）的范数（用于评估观测质量）
   */
  double getInnovationNorm() const { return innovation_norm_; }
  
private:
  int state_dim_;
  
  VectorXd x_est_;      // 状态估计 x(k|k)
  VectorXd x_pred_;     // 状态预测 x(k|k-1)
  MatrixXd P_est_;      // 协方差估计 P(k|k)
  MatrixXd P_pred_;     // 协方差预测 P(k|k-1)
  MatrixXd Q_;          // 过程噪声协方差（基准值）
  MatrixXd R_;          // 观测噪声协方差（基准值）
  MatrixXd I_;          // 单位矩阵
  
  double innovation_norm_;  // 新息范数（用于监测）
  
  // 自适应参数
  VectorXd Q_base_;     // 基准过程噪声（对角元素）
  VectorXd R_base_;     // 基准观测噪声（对角元素）
};

}  // namespace humanoid
}  // namespace ocs2

