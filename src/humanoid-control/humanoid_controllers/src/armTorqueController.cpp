#include <pinocchio/fwd.hpp> // forward declarations must be included first.
// #include "pinocchio/algorithm/gravity.hpp"

#include "humanoid_controllers/armTorqueController.h"
#include <stdexcept>
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/model.hpp"
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
                break;
            }
        }
        
        current_q_idx += model_.joints[i].nq();
        current_v_idx += model_.joints[i].nv();
    }
    
    std::cout << "[ArmTorqueController] 在URDF中找到的手臂关节数量: " << n_arm_ << std::endl;
    
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
