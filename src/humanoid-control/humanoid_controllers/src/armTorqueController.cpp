#include <pinocchio/fwd.hpp> // forward declarations must be included first.
// #include "pinocchio/algorithm/gravity.hpp"

#include "humanoid_controllers/armTorqueController.h"
#include <stdexcept>
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include <functional>
#include <iostream>  // 用于 std::cout





ArmTorqueController::ArmTorqueController(const std::string& urdf_path,
                                         const Eigen::VectorXd& kp,
                                         const Eigen::VectorXd& kd)
{
    std::cout << "[ArmTorqueController] urdf_path: " << urdf_path << std::endl;
    std::cout << "[ArmTorqueController] kp: " << kp.transpose() << std::endl;
    std::cout << "[ArmTorqueController] kd: " << kd.transpose() << std::endl;
    // 加载 URDF 模型
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    // 定义固定的14个手臂关节名称列表（所有版本都一样）
    std::vector<std::string> arm_joint_names = {
        "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
        "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
        "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
        "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint"
    };

    std::cout << "[ArmTorqueController] model.nq: " << model_.nq << std::endl;
    std::cout << "[ArmTorqueController] model.nv: " << model_.nv << std::endl;

    // 初始化期望状态
    q_measured_ = Eigen::VectorXd::Zero(model_.nq);
    v_measured_ = Eigen::VectorXd::Zero(model_.nv);

    // 在URDF中查找手臂关节，统计数量并找到索引
    left_arm_q_start_idx_ = -1;
    left_arm_v_start_idx_ = -1;
    right_arm_q_start_idx_ = -1;
    right_arm_v_start_idx_ = -1;
    n_arm_ = 0;
    int n_left_arm_local = 0;
    int n_right_arm_local = 0;
    
    int current_q_idx = 0;
    int current_v_idx = 0;
    
    for (size_t i = 1; i < model_.names.size(); ++i) { // 从1开始，跳过root关节
        const std::string& joint_name = model_.names[i];
        
        // 检查是否是手臂关节
        for (const auto& arm_joint_name : arm_joint_names) {
            if (joint_name == arm_joint_name) {
                n_arm_++;
                // 记录左、右手臂的起始索引
                if (joint_name == "zarm_l1_joint") {
                    left_arm_q_start_idx_ = current_q_idx;
                    left_arm_v_start_idx_ = current_v_idx;
                }
                if (joint_name == "zarm_r1_joint") {
                    right_arm_q_start_idx_ = current_q_idx;
                    right_arm_v_start_idx_ = current_v_idx;
                }
                // 动态统计每侧关节数
                if (joint_name.find("zarm_l") != std::string::npos) {
                    n_left_arm_local++;
                } else if (joint_name.find("zarm_r") != std::string::npos) {
                    n_right_arm_local++;
                }
                break;
            }
        }
        
        current_q_idx += model_.joints[i].nq();
        current_v_idx += model_.joints[i].nv();
    }
    
    // 根据URDF实际关节数设置，不再硬编码为7
    n_left_arm_ = n_left_arm_local;
    n_right_arm_ = n_right_arm_local;
    
    std::cout << "[ArmTorqueController] 在URDF中找到的手臂关节数量: " << n_arm_
              << " (左: " << n_left_arm_ << ", 右: " << n_right_arm_ << ")" << std::endl;
    
    // 校验增益矩阵维度
    if (kp.rows() != n_arm_  || kd.rows() != n_arm_ ) {
        throw std::invalid_argument("KP/KD 矩阵维度与手臂关节数不匹配");
    }
    
    // 验证是否找到了左、右手臂关节
    if (left_arm_q_start_idx_ == -1 || right_arm_q_start_idx_ == -1) {
        std::cerr << "[ArmTorqueController] 错误：在URDF中未找到手臂关节！" << std::endl;
        std::cerr << "[ArmTorqueController] left_arm_q_start_idx_: " << left_arm_q_start_idx_ << std::endl;
        std::cerr << "[ArmTorqueController] right_arm_q_start_idx_: " << right_arm_q_start_idx_ << std::endl;
        throw std::invalid_argument("在URDF中未找到手臂关节");
    }
    
    // kp kd - 只设置手臂关节的增益
    kp_ = Eigen::VectorXd::Zero(model_.nq);
    kd_ = Eigen::VectorXd::Zero(model_.nv);
    
    // 设置左手臂的增益
    kp_.segment(left_arm_q_start_idx_, n_left_arm_) = kp.segment(0, n_left_arm_);
    kd_.segment(left_arm_v_start_idx_, n_left_arm_) = kd.segment(0, n_left_arm_);
    
    // 设置右手臂的增益
    kp_.segment(right_arm_q_start_idx_, n_right_arm_) = kp.segment(n_left_arm_, n_right_arm_);
    kd_.segment(right_arm_v_start_idx_, n_right_arm_) = kd.segment(n_left_arm_, n_right_arm_);
    
    std::cout << "[ArmTorqueController] 左手臂关节起始索引 (q/v): " << left_arm_q_start_idx_ << "/" << left_arm_v_start_idx_ << std::endl;
    std::cout << "[ArmTorqueController] 右手臂关节起始索引 (q/v): " << right_arm_q_start_idx_ << "/" << right_arm_v_start_idx_ << std::endl;
    std::cout << "[ArmTorqueController] 手臂关节数量: " << n_arm_ << std::endl;
}

void ArmTorqueController::setMeasuredState(const Eigen::VectorXd& q_measured,
                                            const Eigen::VectorXd& v_measured) {
    assert(q_measured.rows() == n_arm_ && v_measured.rows() == n_arm_);
    setArmData(q_measured_, v_measured_, q_measured, v_measured);
}

void ArmTorqueController::setArmData(Eigen::VectorXd& target_q, Eigen::VectorXd& target_v,
                                      const Eigen::VectorXd& source_q, const Eigen::VectorXd& source_v) const {
    // 设置左手臂
    target_q.segment(left_arm_q_start_idx_, n_left_arm_) = source_q.segment(0, n_left_arm_);
    target_v.segment(left_arm_v_start_idx_, n_left_arm_) = source_v.segment(0, n_left_arm_);
    
    // 设置右手臂
    target_q.segment(right_arm_q_start_idx_, n_right_arm_) = source_q.segment(n_left_arm_, n_right_arm_);
    target_v.segment(right_arm_v_start_idx_, n_right_arm_) = source_v.segment(n_left_arm_, n_right_arm_);
}

void ArmTorqueController::setArmData(Eigen::VectorXd& target_q, Eigen::VectorXd& target_v, Eigen::VectorXd& target_a,
                                      const Eigen::VectorXd& source_q, const Eigen::VectorXd& source_v, const Eigen::VectorXd& source_a) const {
    // 设置左手臂
    target_q.segment(left_arm_q_start_idx_, n_left_arm_) = source_q.segment(0, n_left_arm_);
    target_v.segment(left_arm_v_start_idx_, n_left_arm_) = source_v.segment(0, n_left_arm_);
    target_a.segment(left_arm_v_start_idx_, n_left_arm_) = source_a.segment(0, n_left_arm_);
    
    // 设置右手臂
    target_q.segment(right_arm_q_start_idx_, n_right_arm_) = source_q.segment(n_left_arm_, n_right_arm_);
    target_v.segment(right_arm_v_start_idx_, n_right_arm_) = source_v.segment(n_left_arm_, n_right_arm_);
    target_a.segment(right_arm_v_start_idx_, n_right_arm_) = source_a.segment(n_left_arm_, n_right_arm_);
}

Eigen::VectorXd ArmTorqueController::computeTorque(
    const Eigen::VectorXd& q_desired,
    const Eigen::VectorXd& v_desired,
    const Eigen::VectorXd& a_desired) {
    
    assert(q_desired.rows() == n_arm_ && v_desired.rows() == n_arm_ && a_desired.rows() == n_arm_);
    
    // V51版本专用：输入数据NaN检查
    if (q_desired.hasNaN() || v_desired.hasNaN() || a_desired.hasNaN()) {
        std::cerr << "[ArmTorqueController] 错误：输入数据包含NaN值" << std::endl;
        std::cerr << "q_desired.hasNaN(): " << q_desired.hasNaN() << std::endl;
        std::cerr << "v_desired.hasNaN(): " << v_desired.hasNaN() << std::endl;
        std::cerr << "a_desired.hasNaN(): " << a_desired.hasNaN() << std::endl;
        return Eigen::VectorXd::Zero(n_arm_);
    }
    
    if (q_measured_.hasNaN() || v_measured_.hasNaN()) {
        std::cerr << "[ArmTorqueController] 错误：测量数据包含NaN值" << std::endl;
        return Eigen::VectorXd::Zero(n_arm_);
    }
    
    Eigen::VectorXd q_desired_full = Eigen::VectorXd::Zero(model_.nq);
    Eigen::VectorXd v_desired_full = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd a_desired_full = Eigen::VectorXd::Zero(model_.nv);
    
    setArmData(q_desired_full, v_desired_full, a_desired_full, q_desired, v_desired, a_desired);
    
    // 计算完整动力学项
    pinocchio::computeAllTerms(model_, data_, q_measured_, v_measured_);
    const Eigen::MatrixXd& M = pinocchio::crba(model_, data_, q_measured_);  // 惯性矩阵
    const Eigen::VectorXd& Cv = data_.nle;                     // 科氏力 + 离心力
    const Eigen::VectorXd& G = data_.g;                        // 重力项

    // 前馈扭矩：M*a_desired + Cv + G
    // std::cout << "[ArmTorqueController] M*a_desired_full: " << M * a_desired_full.transpose() << std::endl;
    // std::cout << "[ArmTorqueController] Cv: " << Cv.segment(n_leg_joints_, n_arm_joints_).transpose() << std::endl;
    // std::cout << "[ArmTorqueController] G: " << G.segment(n_leg_joints_, n_arm_joints_).transpose() << std::endl;
    // Eigen::VectorXd tau_ff = M * a_desired_full + Cv + G;

    // PD 反馈扭矩：KP*(q_desired - q_measured) + KD*(v_desired - v_measured)
    Eigen::VectorXd q_error = q_desired_full - q_measured_;
    Eigen::VectorXd v_error = v_desired_full - v_measured_;
    Eigen::VectorXd tau_fb = kp_.asDiagonal() * q_error + kd_.asDiagonal() * v_error;
    // std::cout << "[ArmTorqueController] kp_: " << kp_.transpose() << std::endl;
    // std::cout << "[ArmTorqueController] kd_: " << kd_.transpose() << std::endl;
    // std::cout << "[ArmTorqueController] tau_ff: " << tau_ff.segment(n_leg_joints_, n_arm_joints_).transpose() << std::endl;
    // std::cout << "[ArmTorqueController] tau_fb: " << tau_fb.segment(n_leg_joints_, n_arm_joints_).transpose() << std::endl;
    // Eigen::VectorXd tau = tau_ff + tau_fb;
    Eigen::VectorXd tau = G + tau_fb;
    // std::cout << "[ArmTorqueController] tau: " << tau.transpose() << std::endl;
    // std::cout << "[ArmTorqueController] tau.size(): " << tau.size() << std::endl;

    // 提取左、右手臂的扭矩
    Eigen::VectorXd arm_tau = Eigen::VectorXd::Zero(n_arm_);
    arm_tau.segment(0, n_left_arm_) = tau.segment(left_arm_v_start_idx_, n_left_arm_);
    arm_tau.segment(n_left_arm_, n_right_arm_) = tau.segment(right_arm_v_start_idx_, n_right_arm_);
    
    return arm_tau;
}

void ArmTorqueController::setGains(const Eigen::VectorXd& kp, const Eigen::VectorXd& kd)
{
    if (kp.rows() != n_arm_ || kd.rows() != n_arm_)
    {
        throw std::invalid_argument("setGains: KP/KD 矩阵维度与手臂关节数不匹配");
    }
    // 只覆盖手臂段的增益，腿部/腰部保持不变（保持零，不产生力矩）
    kp_.segment(left_arm_q_start_idx_, n_left_arm_) = kp.segment(0, n_left_arm_);
    kd_.segment(left_arm_v_start_idx_, n_left_arm_) = kd.segment(0, n_left_arm_);
    kp_.segment(right_arm_q_start_idx_, n_right_arm_) = kp.segment(n_left_arm_, n_right_arm_);
    kd_.segment(right_arm_v_start_idx_, n_right_arm_) = kd.segment(n_left_arm_, n_right_arm_);
}

Eigen::Isometry3d ArmTorqueController::getEndEffectorPose(bool left) const
{
    const std::string frame_name = left ? "zarm_l7_end_effector" : "zarm_r7_end_effector";
    const pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
    // 前向运动学：基于最近一次 setMeasuredState 的实测状态
    pinocchio::framesForwardKinematics(model_, data_, q_measured_);
    // pinocchio::SE3 -> Eigen::Isometry3d（构造时强制单位旋转保证类型一致）
    return Eigen::Isometry3d(data_.oMf[frame_id].toHomogeneousMatrix());
}

Eigen::MatrixXd ArmTorqueController::getEndEffectorJacobian(bool left) const
{
    const std::string frame_name = left ? "zarm_l7_end_effector" : "zarm_r7_end_effector";
    const pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

    // 计算完整模型的关节雅可比（含基座，但基座固定，忽略其列）
    pinocchio::computeJointJacobians(model_, data_, q_measured_);
    pinocchio::updateFramePlacements(model_, data_);
    // Position errors and estimated forces use world axes at the end effector.
    Eigen::MatrixXd J_frame = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

    // 提取手臂关节对应的列：左臂 [left_arm_v_start_idx_, +n_left_arm_)，右臂 [right_arm_v_start_idx_, +n_right_arm_)
    Eigen::MatrixXd J_arm = Eigen::MatrixXd::Zero(6, n_arm_);
    J_arm.leftCols(n_left_arm_) = J_frame.middleCols(left_arm_v_start_idx_, n_left_arm_);
    J_arm.rightCols(n_right_arm_) = J_frame.middleCols(right_arm_v_start_idx_, n_right_arm_);
    return J_arm;
}

Eigen::VectorXd ArmTorqueController::computeGravityArm()
{
    // 计算重力项（需要完整的模型状态）
    pinocchio::computeAllTerms(model_, data_, q_measured_, v_measured_);
    const Eigen::VectorXd& G = data_.g;
    // 提取手臂段的重力项
    Eigen::VectorXd G_arm = Eigen::VectorXd::Zero(n_arm_);
    G_arm.segment(0, n_left_arm_) = G.segment(left_arm_v_start_idx_, n_left_arm_);
    G_arm.segment(n_left_arm_, n_right_arm_) = G.segment(right_arm_v_start_idx_, n_right_arm_);
    return G_arm;
}

Eigen::VectorXd ArmTorqueController::computeCartesianImpedanceTorque(const Eigen::Vector3d& x_desired,
                                                                     const Eigen::Vector3d& v_desired,
                                                                     bool left,
                                                                     double kx,
                                                                     double dx)
{
    // 末端位姿与雅可比均使用世界系轴向。
    const Eigen::Isometry3d x_cur = getEndEffectorPose(left);
    const Eigen::MatrixXd J = getEndEffectorJacobian(left);  // 6 x n_arm_

    // 平移误差（世界系）
    const Eigen::Vector3d dx_pos = x_desired - x_cur.translation();
    // 末端速度：J * 手臂关节速度（v_measured_ 为完整状态，需提取手臂段）
    Eigen::VectorXd arm_v(n_arm_);
    arm_v.segment(0, n_left_arm_) = v_measured_.segment(left_arm_v_start_idx_, n_left_arm_);
    arm_v.segment(n_left_arm_, n_right_arm_) = v_measured_.segment(right_arm_v_start_idx_, n_right_arm_);
    const Eigen::Vector3d v_ee = J.topRows(3) * arm_v;

    // 笛卡尔力：F = Kx·Δx + Dx·(v_d − v_ee)
    const Eigen::Vector3d F = kx * dx_pos + dx * (v_desired - v_ee);

    // τ_imp = Jᵀ·F
    const Eigen::VectorXd tau_imp = J.topRows(3).transpose() * F;

    // 加重力补偿，得到最终手臂力矩
    const Eigen::VectorXd G_arm = computeGravityArm();
    return tau_imp + G_arm;
}

Eigen::Vector3d ArmTorqueController::estimateEndEffectorForce(const Eigen::VectorXd& tau_measured_arm,
                                                              bool left) const
{
    if (tau_measured_arm.size() != n_arm_)
    {
        return Eigen::Vector3d::Zero();
    }
    // 模型力矩：重力项（忽略科氏与惯性，低速站立近似）
    pinocchio::computeAllTerms(model_, data_, q_measured_, v_measured_);
    Eigen::VectorXd G_arm = Eigen::VectorXd::Zero(n_arm_);
    G_arm.segment(0, n_left_arm_) = data_.g.segment(left_arm_v_start_idx_, n_left_arm_);
    G_arm.segment(n_left_arm_, n_right_arm_) = data_.g.segment(right_arm_v_start_idx_, n_right_arm_);

    // 外力矩 = 实测 − 模型
    const Eigen::VectorXd tau_ext = tau_measured_arm - G_arm;

    // 雅可比（平移部分，仅单臂列有效）
    const Eigen::MatrixXd J = getEndEffectorJacobian(left);
    const Eigen::MatrixXd Jt = J.topRows(3);  // 3 x n_arm_

    // 最小二乘外力：F = (J·Jᵀ)⁻¹·J·τ_ext
    const Eigen::Matrix3d JJt = Jt * Jt.transpose();
    Eigen::Matrix3d JJt_inv;
    if (JJt.determinant() < 1e-8)
    {
        return Eigen::Vector3d::Zero();
    }
    JJt_inv = JJt.inverse();
    return JJt_inv * (Jt * tau_ext);
}
