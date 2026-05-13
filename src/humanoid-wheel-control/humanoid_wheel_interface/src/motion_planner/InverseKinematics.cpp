/********************************************************************************
Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "humanoid_wheel_interface/motion_planner/InverseKinematics.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace mobile_manipulator {

vector_t InverseKinematics::computeHandOnlyIK(vector_t init_q, int hand, vector3_t des_hand_linear_xyz,
                                              vector3_t des_hand_eular_zyx, bool isMix)
{
  const vector_t initial_q = init_q;
  switch(hand)   // 根据输入的手索引设置IK类型
  {
    case 0: ikType_ = IKType::LEFT_HAND_ONLY_IK; break;
    case 1: ikType_ = IKType::RIGHT_HAND_ONLY_IK; break;
    default: 
    {
      std::cerr << "Invalid hand index for IK computation. Returning initial configuration." << std::endl;
      return init_q;
    }
  }
  changeJointActiveIndices(ikType_);    // 修改雅可比矩阵中参与计算的关节索引列表

  const auto& model = pinocchio_interface_->getModel();
  auto& data = pinocchio_interface_->getData();
  auto FRAME_ID = model.getFrameId(info_->eeFrames[hand]);

  if(isMix == false)    // 非混合策略的调用
  {
    bestLinearError_ = std::numeric_limits<scalar_t>::max();
    bestAngularError_ = std::numeric_limits<scalar_t>::max();

    vector_t compute_q = computeTranslationIK(init_q, des_hand_linear_xyz, FRAME_ID);
    vector_t solution = computeRotationIK(compute_q, des_hand_eular_zyx, FRAME_ID);

    // 构建目标位姿
    pinocchio::framesForwardKinematics(model, data, solution);
    matrix3_t des_hand_R_des = getRotationMatrixFromZyxEulerAngles(des_hand_eular_zyx);
    pinocchio::SE3 goal_tform(des_hand_R_des, des_hand_linear_xyz);

    vector_t err = pinocchio::log6(goal_tform.actInv(data.oMf[FRAME_ID])).toVector();
    bestLinearError_ = err.head<3>().norm();
    bestAngularError_ = err.tail<3>().norm();

    return solution;
  }
  else
  {
    return computeSimpleIK(init_q, des_hand_linear_xyz, des_hand_eular_zyx, FRAME_ID);
  }

  return initial_q;
}

vector_t InverseKinematics::computeWholeBodyIK(vector_t init_q, int hand, vector3_t des_hand_linear_xyz,
                                               vector3_t des_hand_eular_zyx, bool isMix)
{
  const vector_t initial_q = init_q;
  switch(hand)
  {
    case 0: ikType_ = IKType::LEFT_WHOLE_BODY_IK; break;
    case 1: ikType_ = IKType::RIGHT_WHOLE_BODY_IK; break;
    default: 
    {
      std::cerr << "Invalid hand index for IK computation. Returning initial configuration." << std::endl;
      return init_q;
    }
  }
  changeJointActiveIndices(ikType_);    // 修改雅可比矩阵中参与计算的关节索引列表

  const auto& model = pinocchio_interface_->getModel();
  auto& data = pinocchio_interface_->getData();
  auto FRAME_ID = model.getFrameId(info_->eeFrames[hand]);

  if(isMix == false)    // 非混合策略
  {
    bestLinearError_ = std::numeric_limits<scalar_t>::max();
    bestAngularError_ = std::numeric_limits<scalar_t>::max();

    vector_t compute_q = computeTranslationIK(init_q, des_hand_linear_xyz, FRAME_ID);
    vector_t solution = computeRotationIK(compute_q, des_hand_eular_zyx, FRAME_ID);

    // 构建目标位姿
    pinocchio::framesForwardKinematics(model, data, solution);
    matrix3_t des_hand_R_des = getRotationMatrixFromZyxEulerAngles(des_hand_eular_zyx);
    pinocchio::SE3 goal_tform(des_hand_R_des, des_hand_linear_xyz);

    vector_t err = pinocchio::log6(goal_tform.actInv(data.oMf[FRAME_ID])).toVector();
    bestLinearError_ = err.head<3>().norm();
    bestAngularError_ = err.tail<3>().norm();

    return solution;
  }
  else
  {
    return computeSimpleIK(init_q, des_hand_linear_xyz, des_hand_eular_zyx, FRAME_ID);
  }

  return initial_q;
}

vector_t InverseKinematics::computeTranslationIK(vector_t init_q, vector3_t des_hand_linear_xyz, size_t frameId)
{
  const scalar_t err_tol = linearErrorMax_;
  const scalar_t converage_tol = 1e-30;
  const scalar_t damp = 1e-6;
  const scalar_t dt = 0.1;
  const int max_it = 5000;
  const bool verbose = false;

  const auto& des_hand_p = des_hand_linear_xyz;
  const auto& model = pinocchio_interface_->getModel();
  auto& data = pinocchio_interface_->getData();

  const vector_t q_min = model.lowerPositionLimit.tail(info_->armDim);
  const vector_t q_max = model.upperPositionLimit.tail(info_->armDim);

  Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_pos;   // 只取位置部分的雅可比矩阵
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> Jac_full;
  Jac_pos.setZero(3, model.nv);
  Jac_full.setZero(6, model.nv);
  vector_t v(model.nv);
  v.setZero();

  pinocchio::framesForwardKinematics(model, data, init_q);
  const vector3_t init_hand_pos = data.oMf[frameId].translation();

  vector3_t err = init_hand_pos - des_hand_p;
  scalar_t last_err_norm = err.norm();

  if(last_err_norm < err_tol)
  {
    if (verbose)
      printf("[translation IK] small err at init, err norm: %f\n", last_err_norm);
    return std::move(init_q);
  }
  else
  {
    int i = 0;
    while (true)
    {
      pinocchio::computeFrameJacobian(model, data, init_q, frameId, pinocchio::LOCAL_WORLD_ALIGNED, Jac_full);
      Jac_pos = Jac_full.topRows(3);  // 只取位置部分的雅可比矩阵
      
      /*************************** 选取使能的列 ************************************/
      int activeCols = ikJointIndices_.size();
      Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_tail = Jac_pos.rightCols(info_->armDim);
      Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_active(3, activeCols);
      for (int col = 0; col < activeCols; ++col) {
        Jac_active.col(col) = Jac_tail.col(ikJointIndices_[col]);
      }
      /**************************************************************************/

      // DLS (Damped Least Squares) 替代 QR 分解
      // 计算 J * J^T + damp * I
      Eigen::Matrix<scalar_t, 3, 3> J_Jt = Jac_active * Jac_active.transpose();
      J_Jt.diagonal().array() += damp;
      
      // 求解 v = -J^T * (J*J^T + damp*I)^{-1} * err
      Eigen::VectorXd v_selected = -Jac_active.transpose() * J_Jt.ldlt().solve(err);

      // 将求解得到的速度填充到完整的速度向量中，未使能的部分保持为0
      v.setZero();
      for(int idx = 0; idx < activeCols; ++idx) {
        int global_col = (Jac_pos.cols() - info_->armDim) + ikJointIndices_[idx];  // 全局列索引
        v(global_col) = v_selected(idx);  // 注意索引偏移
      }

      // 积分得到新的关节角度
      vector_t new_q = pinocchio::integrate(model, init_q, v * dt);

      // 添加关节限位
      for (int j = 0; j < info_->armDim; j++)
      {
        int global_idx = (Jac_pos.cols() - info_->armDim) + j;
        new_q[global_idx] = std::max(q_min[j], new_q[global_idx]);
        new_q[global_idx] = std::min(q_max[j], new_q[global_idx]);
      }

      // 更新运动学
      pinocchio::framesForwardKinematics(model, data, new_q);
      vector3_t cur_hand_pos = data.oMf[frameId].translation();
      err = cur_hand_pos - des_hand_p;

      scalar_t err_norm = err.norm();

      // 检查发散
      if (err_norm > last_err_norm)
      {
        if (verbose)
          printf("[translation IK] local min at %d, last err norm: %f, new err norm: %f\n", i, last_err_norm, err_norm);
        break;
      }

      // 检查收敛
      if (abs(err_norm - last_err_norm) < converage_tol)
      {
        if (verbose)
          printf("[translation IK] converage at %d, last err norm: %f, new err norm: %f\n", i, last_err_norm, err_norm);
        break;
      }

      last_err_norm = err_norm;
      init_q = new_q;
      
      // 检查是否达到精度
      if (err_norm < err_tol)
      {
        if (verbose)
          printf("[translation IK] small err at %d, err norm: %f\n", i, err_norm);
        break;
      }
      
      i++;
      if (i >= max_it)
      {
        if (verbose)
          printf("[translation IK] max iter at %d, err norm: %f\n", i, err_norm);
        break;
      }
    }
  }

  return std::move(init_q);
}

vector_t InverseKinematics::computeRotationIK(vector_t init_q, vector3_t des_hand_eular_zyx, size_t frameId)
{
  matrix3_t R_des = getRotationMatrixFromZyxEulerAngles(des_hand_eular_zyx);
  return computeRotationIK(init_q, R_des, frameId);
}

vector_t InverseKinematics::computeRotationIK(vector_t init_q, matrix3_t des_hand_R_des, size_t frameId)
{
  const scalar_t err_tol = angularErrorMax_;
  const scalar_t converage_tol = 1e-30;
  const scalar_t damp = 1e-6;
  const scalar_t dt = 0.001;
  const int max_it = 20000;
  const bool verbose = false;
  const scalar_t pos_err_tol = 1e-3;  // 位置误差容差

  const auto& model = pinocchio_interface_->getModel();
  auto& data = pinocchio_interface_->getData();

  // 获取期望的末端位置（从初始位置获取）
  pinocchio::framesForwardKinematics(model, data, init_q);
  const vector3_t des_hand_pos = data.oMf[frameId].translation();

  const vector_t q_min = model.lowerPositionLimit.tail(info_->armDim);
  const vector_t q_max = model.upperPositionLimit.tail(info_->armDim);

  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> Jac_full;
  Jac_full.setZero(6, model.nv);
  vector_t v(model.nv);
  v.setZero();

  pinocchio::framesForwardKinematics(model, data, init_q);
  matrix3_t R_cur = data.oMf[frameId].rotation();
  vector3_t pos_cur = data.oMf[frameId].translation();

  vector3_t err_ori = pinocchio::log3(des_hand_R_des.transpose() * R_cur);
  vector3_t err_pos = pos_cur - des_hand_pos;  // 初始位置误差
  scalar_t last_err_ori_norm = err_ori.norm();
  scalar_t init_err_pos_norm = err_pos.norm();  // 记录初始位置误差范数

  if (last_err_ori_norm < err_tol)
  {
    if (verbose)
      printf("[rotation IK] small err at init, err norm: %f\n", last_err_ori_norm);
    return std::move(init_q);
  }
  else
  {
    int i = 0;
    while (true)
    {
      pinocchio::computeFrameJacobian(model, data, init_q, frameId, pinocchio::LOCAL_WORLD_ALIGNED, Jac_full);
      
      /*************************** 提取位置和姿态雅可比 ************************************/
      // 位置雅可比（前3行）
      Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_pos = Jac_full.topRows(3);
      // 姿态雅可比（后3行）
      Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_ori = Jac_full.bottomRows(3);
      
      // 提取活跃关节对应的列（尾部 armDim 列中选中的部分）
      int activeCols = ikJointIndices_.size();
      Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_pos_tail = Jac_pos.rightCols(info_->armDim);
      Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_ori_tail = Jac_ori.rightCols(info_->armDim);
      
      Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_pos_active(3, activeCols);
      Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_ori_active(3, activeCols);
      
      for (int col = 0; col < activeCols; ++col) {
        Jac_pos_active.col(col) = Jac_pos_tail.col(ikJointIndices_[col]);
        Jac_ori_active.col(col) = Jac_ori_tail.col(ikJointIndices_[col]);
      }
      /**************************************************************************/

      // 计算位置雅可比零空间（用于姿态控制，避免影响位置）
      Eigen::FullPivLU<Eigen::Matrix<scalar_t, 3, Eigen::Dynamic>> lu_decomp(Jac_pos_active);
      Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> null_space = lu_decomp.kernel();
      
      // 在零空间中求解姿态误差
      // Jac_ori_active * null_space * v_null = err_ori
      Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Jac_ori_null = Jac_ori_active * null_space;
      
      // DLS (Damped Least Squares) 替代 QR 分解
      // 计算 J * J^T + damp * I
      Eigen::Matrix<scalar_t, 3, 3> J_Jt = Jac_ori_null * Jac_ori_null.transpose();
      J_Jt.diagonal().array() += damp;
      
      // 求解 v_null = -J^T * (J*J^T + damp*I)^{-1} * err_ori
      Eigen::VectorXd v_null = -Jac_ori_null.transpose() * J_Jt.ldlt().solve(err_ori);
      
      // 计算选中的关节速度
      Eigen::VectorXd v_selected = null_space * v_null;

      // 将求解得到的速度填充到完整的速度向量中
      v.setZero();
      for(int idx = 0; idx < activeCols; ++idx) {
        int global_col = (Jac_full.cols() - info_->armDim) + ikJointIndices_[idx];
        v(global_col) = v_selected(idx);
      }

      // 积分得到新的关节角度
      vector_t new_q = pinocchio::integrate(model, init_q, v * dt);

      // 添加关节限位
      for (int j = 0; j < info_->armDim; j++)
      {
        int global_idx = (Jac_full.cols() - info_->armDim) + j;
        new_q[global_idx] = std::max(q_min[j], new_q[global_idx]);
        new_q[global_idx] = std::min(q_max[j], new_q[global_idx]);
      }

      // 更新运动学并获取新的位姿
      pinocchio::framesForwardKinematics(model, data, new_q);
      R_cur = data.oMf[frameId].rotation();
      pos_cur = data.oMf[frameId].translation();
      err_ori = pinocchio::log3(des_hand_R_des.transpose() * R_cur);
      err_pos = pos_cur - des_hand_pos;  // 更新位置误差

      scalar_t err_ori_norm = err_ori.norm();
      scalar_t err_pos_norm = err_pos.norm();
      
      // 检查位置误差是否超过初始位置误差 + 容差
      if (err_pos_norm > init_err_pos_norm + pos_err_tol)
      {
        if (verbose)
          printf("[rotation IK] position error exceeded at %d, init pos err: %f, current pos err: %f, tolerance: %f\n", 
                 i, init_err_pos_norm, err_pos_norm, pos_err_tol);
        // 位置误差超过初始值+容差，返回变化前的关节角度
        return std::move(init_q);
      }

      // 检查发散（姿态误差发散）
      if (err_ori_norm > last_err_ori_norm)
      {
        if (verbose)
          printf("[rotation IK] local min at %d, last ori err: %f, new ori err: %f\n", 
                 i, last_err_ori_norm, err_ori_norm);
        break;
      }

      // 检查收敛
      if (abs(err_ori_norm - last_err_ori_norm) < converage_tol)
      {
        if (verbose)
          printf("[rotation IK] converage at %d, last ori err: %f, new ori err: %f\n", 
                 i, last_err_ori_norm, err_ori_norm);
        break;
      }

      last_err_ori_norm = err_ori_norm;
      init_q = new_q;

      // 检查是否达到精度
      if (err_ori_norm < err_tol)
      {
        if (verbose)
          printf("[rotation IK] small err at %d, ori err norm: %f\n", i, err_ori_norm);
        break;
      }

      i++;
      if (i >= max_it)
      {
        if (verbose)
          printf("[rotation IK] max iter at %d, ori err norm: %f\n", i, err_ori_norm);
        break;
      }
    }
  }

  return std::move(init_q);
}

vector_t InverseKinematics::computeSimpleIK(vector_t init_q, vector3_t des_hand_linear_xyz, 
                                            vector3_t des_hand_eular_zyx, size_t frameId)
{
  const vector_t initial_q = init_q;
  // ========== 初始化随机种子（只执行一次） ==========
  static int random_seeded = (std::srand(static_cast<unsigned int>(std::time(nullptr))), 0);
  // =============================================

  const scalar_t damp = 1e-6;
  const scalar_t dt = 0.1;
  const int max_it = 300000;
  const bool verbose = false;

  double totalTimeDesired = totalTimeDesired_;  // 期望的总计算时间（秒）
  int maxAttempts = maxAttempts_;  // 最大尝试次数
  double timePerAttempt = totalTimeDesired / maxAttempts;  // 每次尝试的时间限制
  const std::chrono::duration<double> timeout(timePerAttempt);  // 设置单次迭代的超时时间
  scalar_t linear_error_max = linearErrorMax_;  // 线性误差阈值
  scalar_t angular_error_max = angularErrorMax_;  // 角度误差阈值（弧度）

  // ========== 分片参数配置 ==========
  const int numSegments = 10;  // 每个关节分成10个分片
  const int minSegmentDistance = 3;  // 最小分片距离（至少间隔2个分片）
  // =================================

  const auto& model = pinocchio_interface_->getModel();
  auto& data = pinocchio_interface_->getData();

  const vector_t q_min = model.lowerPositionLimit.tail(info_->armDim);
  const vector_t q_max = model.upperPositionLimit.tail(info_->armDim);

  // 构建目标位姿
  matrix3_t des_hand_R_des = getRotationMatrixFromZyxEulerAngles(des_hand_eular_zyx);
  pinocchio::SE3 goal_tform(des_hand_R_des, des_hand_linear_xyz);

  // 构建初始变量
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> Jac_full;
  Jac_full.setZero(6, model.nv);
  vector_t v(model.nv);
  v.setZero();
  
  pinocchio::framesForwardKinematics(model, data, init_q);
  vector_t err(6);
  err = pinocchio::log6(goal_tform.actInv(data.oMf[frameId])).toVector();

  scalar_t last_err_norm = err.norm();
  const auto linear_error = err.head<3>().norm();
  const auto angular_error = err.tail<3>().norm();

  // ========== 计算原始初始角度的分片索引 ==========
  std::vector<int> original_segments(ikJointIndices_.size());
  for(size_t idx = 0; idx < ikJointIndices_.size(); ++idx) {
    int joint_idx = ikJointIndices_[idx];
    scalar_t normalized_pos = (init_q[(Jac_full.cols() - info_->armDim) + joint_idx] - q_min[joint_idx]) / 
                               (q_max[joint_idx] - q_min[joint_idx]);
    original_segments[idx] = static_cast<int>(normalized_pos * numSegments);
    // 确保分片索引在有效范围内
    original_segments[idx] = std::max(0, std::min(numSegments - 1, original_segments[idx]));
  }
  // ===============================================

  // ========== 追踪最佳解的缓存变量 ==========
  vector_t best_q = init_q;  // 存储最佳关节角度
  scalar_t best_linear_error = linear_error;
  scalar_t best_angular_error = angular_error;
  scalar_t best_err_norm = err.norm();
  bestLinearError_ = std::numeric_limits<scalar_t>::max();    // 重置最佳误差
  bestAngularError_ = std::numeric_limits<scalar_t>::max();
  // =======================================

  if (linear_error <= linear_error_max && 
      angular_error <= angular_error_max)
  {
    if (verbose)
      printf("[simple IK] small err at init, linear error: %f, angular error: %f\n", linear_error, angular_error);
    return std::move(init_q);
  }
  else
  {
    int attempt = 0;
    while(attempt <= maxAttempts)
    {
      const auto start_time = std::chrono::steady_clock::now();
      if (attempt > 0) 
      {
        // 采用分片随机初始角度策略，确保每次尝试都在原始分片的基础上有足够的扰动
        if (verbose)
          printf("[simple IK] retrying with extreme segment perturbation, attempt %d\n", attempt);
        
        for(int idx = 0; idx < ikJointIndices_.size(); ++idx)
        {
          int global_idx = (Jac_full.cols() - info_->armDim) + ikJointIndices_[idx];
          int joint_idx = ikJointIndices_[idx];
          
          // 获取当前关节的范围
          scalar_t joint_min = q_min[joint_idx];
          scalar_t joint_max = q_max[joint_idx];
          scalar_t joint_range = joint_max - joint_min;
          
          // ========== 极端分片策略 ==========
          int target_segment;
          bool valid_segment = false;
          int max_attempts_per_joint = 10;  // 每个关节最多尝试10次
          int segment_attempt = 0;
          
          do 
          {
              if(joint_idx <= 3) 
              {
                target_segment = original_segments[idx];
                valid_segment = true;
                continue;
              }

              // 随机选择任意分片（0 到 numSegments-1）
              target_segment = rand() % numSegments;

              // 检查距离原始分片是否足够远
              int distance = std::abs(target_segment - original_segments[idx]);
              if (distance >= minSegmentDistance) 
              {
                valid_segment = true;
              }

              segment_attempt++;
              if (segment_attempt >= max_attempts_per_joint) 
              {
                // 如果无法找到满足距离要求的分片，就使用随机分片（不再检查距离）
                target_segment = rand() % numSegments;
                valid_segment = true;
                if (verbose) 
                {
                  printf("[simple IK] forced random segment for joint %d: segment %d (original: %d, distance: %d)\n", 
                         joint_idx, target_segment, original_segments[idx], 
                         std::abs(target_segment - original_segments[idx]));
                }
              }
          } while (!valid_segment);
          
          // 在选定的极端分片内随机采样
          scalar_t segment_start = joint_min + (static_cast<scalar_t>(target_segment) / numSegments) * joint_range;
          scalar_t segment_end = joint_min + (static_cast<scalar_t>(target_segment + 1) / numSegments) * joint_range;
          
          // 在分片内均匀随机采样
          scalar_t t = static_cast<scalar_t>(rand()) / RAND_MAX;
          scalar_t random_value = segment_start + t * (segment_end - segment_start);
          
          init_q[global_idx] = random_value;

          for(int j = 0; j <= 3; j++)
          {
            int legJointIdx = info_->stateDim - info_->armDim + j;
            init_q[legJointIdx] = initial_q[legJointIdx];
          }
          
          if (verbose) {
            printf("[simple IK] joint %d: original segment %d -> extreme segment %d (distance %d, %s end)\n", 
                   joint_idx, original_segments[idx], target_segment, 
                   std::abs(target_segment - original_segments[idx]),
                   target_segment == 0 ? "lower" : "upper");
          }
          // ==============================================
        }
      }

      int i = 0;
      vector_t attempt_best_q = init_q;  // 本次尝试中的最佳解
      scalar_t attempt_best_err_norm = err.norm();
      
      // 更新当前尝试的初始误差
      pinocchio::framesForwardKinematics(model, data, init_q);
      err = pinocchio::log6(goal_tform.actInv(data.oMf[frameId])).toVector();
      attempt_best_err_norm = err.norm();
      
      while (true)
      {
        pinocchio::computeFrameJacobian(model, data, init_q, frameId, pinocchio::LOCAL_WORLD_ALIGNED, Jac_full);
      
        // 提取活跃关节对应的列
        int activeCols = ikJointIndices_.size();
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> Jac_tail = Jac_full.rightCols(info_->armDim);
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> Jac_active(6, activeCols);
        for (int col = 0; col < activeCols; ++col) {
          Jac_active.col(col) = Jac_tail.col(ikJointIndices_[col]);
        }

        // DLS (Damped Least Squares) 替代 QR 分解
        // 计算 J * J^T + damp * I
        Eigen::Matrix<scalar_t, 6, 6> J_Jt = Jac_active * Jac_active.transpose();
        J_Jt.diagonal().array() += damp;
      
        // 求解 v = -J^T * (J*J^T + damp*I)^{-1} * err
        Eigen::VectorXd v_selected = -Jac_active.transpose() * J_Jt.ldlt().solve(err);

        // 将求解得到的速度填充到完整的速度向量中
        v.setZero();
        for(int idx = 0; idx < activeCols; ++idx) {
          int global_col = (Jac_full.cols() - info_->armDim) + ikJointIndices_[idx];
          v(global_col) = v_selected(idx);
        }

        // 积分得到新的关节角度
        vector_t new_q = pinocchio::integrate(model, init_q, v * dt);

        // 添加关节限位
        for (int j = 0; j < info_->armDim; j++)
        {
          int global_idx = (Jac_full.cols() - info_->armDim) + j;
          new_q[global_idx] = std::max(q_min[j], new_q[global_idx]);
          new_q[global_idx] = std::min(q_max[j], new_q[global_idx]);
        }

        // 更新运动学
        pinocchio::framesForwardKinematics(model, data, new_q);
        err = pinocchio::log6(goal_tform.actInv(data.oMf[frameId])).toVector();

        scalar_t err_norm = err.norm();

        const auto linear_error = err.head<3>().norm();
        const auto angular_error = err.tail<3>().norm();
        
        // ========== 更新本次尝试的最佳解 ==========
        if (err_norm < attempt_best_err_norm) {
          attempt_best_q = new_q;
          attempt_best_err_norm = err_norm;
        }
        // =======================================
        
        if (linear_error > linear_error_max || 
            angular_error > angular_error_max) 
        {
          init_q = new_q;
        }
        else
        {
          if (verbose)
            printf("[simple IK] error within threshold at %d, linear error: %f, angular error: %f\n", i, linear_error, angular_error);
          
          // 保存误差信息到成员变量（如果需要在外部访问）
          bestLinearError_ = linear_error;
          bestAngularError_ = angular_error;
          return std::move(new_q);
        }

        last_err_norm = err_norm;

        i++;
        if (i >= max_it)
        {
          if (verbose)
            printf("[simple IK] max iter at %d, linear error: %f, angular error: %f\n", i, linear_error, angular_error);
          break;
        }

        if (std::chrono::steady_clock::now() - start_time > timeout) {
          if (verbose)
            printf("[simple IK] timeout at %d, linear error: %f, angular error: %f\n", i, linear_error, angular_error);
          break;
        }
      }
      
      // ========== 尝试结束，更新全局最佳解 ==========
      if (attempt_best_err_norm < best_err_norm) {
        best_q = attempt_best_q;
        best_err_norm = attempt_best_err_norm;
        
        // 重新计算最佳解的详细误差（用于输出）
        pinocchio::framesForwardKinematics(model, data, best_q);
        vector_t best_err = pinocchio::log6(goal_tform.actInv(data.oMf[frameId])).toVector();
        best_linear_error = best_err.head<3>().norm();
        best_angular_error = best_err.tail<3>().norm();
        
        if (verbose)
          printf("[simple IK] attempt %d finished, best error in this attempt: %f (linear: %f, angular: %f)\n", 
                 attempt, attempt_best_err_norm, best_linear_error, best_angular_error);
      }
      // ==========================================
      
      ++attempt;
    }
  }

  // 所有尝试结束，返回全局最佳解
  if (verbose) {
    printf("[simple IK] all attempts finished, returning best solution with error: %f (linear: %f, angular: %f)\n", 
           best_err_norm, best_linear_error, best_angular_error);
    if (best_linear_error > linear_error_max || best_angular_error > angular_error_max) {
      printf("[simple IK] warning: best solution still exceeds threshold\n");
    }
  }

  // 保存最佳的误差信息到成员变量（如果需要在外部访问）
  bestLinearError_ = best_linear_error;
  bestAngularError_ = best_angular_error;
  
  return std::move(best_q);
}

void InverseKinematics::changeJointActiveIndices(IKType ikType)    // 选择的起点为最后 armDim 列的第一个关节索引
{
  ikJointIndices_.clear(); // 不参与IK计算
  switch(ikType)
  {
    case IKType::LEFT_HAND_ONLY_IK:
    {
      for(int i = 0; i < singleArmDim_; i++)
      {
        ikJointIndices_.push_back(legDim_ + i); // 左臂关节索引
      }
      break;
    }
    case IKType::RIGHT_HAND_ONLY_IK:
    {
      for(int i = 0; i < singleArmDim_; i++)
      {
        ikJointIndices_.push_back(legDim_ + singleArmDim_ + i); // 右臂关节索引
      }
      break;
    }
    case IKType::LEFT_WHOLE_BODY_IK:
    {
      for(int i = 0; i < legDim_; ++i)
      {
        ikJointIndices_.push_back(i); // 下肢关节索引
      }
      for(int i = 0; i < singleArmDim_; i++)
      {
        ikJointIndices_.push_back(legDim_ + i); // 左臂关节索引
      }
      break;
    }
    case IKType::RIGHT_WHOLE_BODY_IK:
    {
      for(int i = 0; i < legDim_; ++i)
      {
        ikJointIndices_.push_back(i); // 下肢关节索引
      }
      for(int i = 0; i < singleArmDim_; i++)
      {
        ikJointIndices_.push_back(legDim_ + singleArmDim_ + i); // 左臂关节索引
      }
      break;
    }
    default:
    {
      break;
    }
  }
}

}  // namespace mobile_manipulator
}  // namespace ocs2
