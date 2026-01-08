#include <pinocchio/fwd.hpp> // forward declarations must be included first.
// #include "pinocchio/algorithm/gravity.hpp"

#include "humanoid_controllers/armTorqueController.h"
#include <stdexcept>
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/model.hpp"
#include <functional>
#include <cstdlib>  // 添加这个头文件用于 getenv
#include <iostream> // 添加这个头文件用于 std::cout
#include <set>      // 添加这个头文件用于 std::set

// 静态辅助方法：获取机器人版本
std::string ArmTorqueController::getRobotVersion() {
    std::string robot_version = "46";
    const char* robot_version_env = std::getenv("ROBOT_VERSION");
    if (robot_version_env != nullptr && strlen(robot_version_env) > 0) {
        robot_version = robot_version_env;
    }
    return robot_version;
}

// 静态辅助方法：计算手臂关节的起始索引（考虑waist关节）
int ArmTorqueController::getArmStartIndex() {
    static const std::set<std::string> versions_with_waist_yaw = {"50", "51", "52"};
    std::string robot_version = getRobotVersion();
    int arm_start_idx = 12; // n_leg_joints_
    if (versions_with_waist_yaw.find(robot_version) != versions_with_waist_yaw.end()) {
        arm_start_idx += 1; // 跳过waist关节
    }
    return arm_start_idx;
}



ArmTorqueController::ArmTorqueController(const std::string& urdf_path,
                                         const Eigen::VectorXd& kp,
                                         const Eigen::VectorXd& kd)
{
    std::cout << "[ArmTorqueController] 构造函数" << std::endl;
    std::cout << "[ArmTorqueController] urdf_path: " << urdf_path << std::endl;
    std::cout << "[ArmTorqueController] kp: " << kp.transpose() << std::endl;
    std::cout << "[ArmTorqueController] kd: " << kd.transpose() << std::endl;
    // 加载 URDF 模型
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    // 获取机器人版本
    std::string robot_version = getRobotVersion();
    std::cout << "从环境变量获取到机器人版本: " << robot_version << std::endl;
    
    // 根据机器人版本决定固定关节列表
    std::vector<std::string> fixed_joint_names;
    
    // 定义包含 waist_yaw 关节的版本集合
    static const std::set<std::string> versions_with_waist_yaw = {"50", "51", "52"};
    
    // 基础固定关节列表（所有版本都包含）
    fixed_joint_names = {
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint",
        "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint",
        "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
        "zhead_1_joint", "zhead_2_joint"
    };
    
    // 如果版本包含 waist_yaw 关节，则在腿部关节后添加
    if (versions_with_waist_yaw.find(robot_version) != versions_with_waist_yaw.end())
    {
        fixed_joint_names.push_back("waist_yaw_joint");
    }

    
    // TODO: 固定关节修改

    std::cout << "[ArmTorqueController] model.nq: " << model_.nq << std::endl;
    std::cout << "[ArmTorqueController] model.nv: " << model_.nv << std::endl;

    // 初始化期望状态
    q_measured_ = Eigen::VectorXd::Zero(model_.nq);
    v_measured_ = Eigen::VectorXd::Zero(model_.nv);

    const int n_fixed_joint = fixed_joint_names.size();
    int n_arm = model_.nq - n_fixed_joint;
    std::cout << "[ArmTorqueController] n_arm: " << n_arm << std::endl;
    
    // 校验增益矩阵维度
    if (kp.rows() != n_arm  || kd.rows() != n_arm ) {
        throw std::invalid_argument("KP/KD 矩阵维度与手臂关节数不匹配");
    }
    
    // 计算手臂关节的起始索引（使用统一方法）
    int arm_start_idx = getArmStartIndex();
    
    // kp kd - 只设置手臂关节的增益
    kp_ = Eigen::VectorXd::Zero(model_.nq);
    kp_.segment(arm_start_idx, n_arm) = kp;
    kd_ = Eigen::VectorXd::Zero(model_.nq);
    kd_.segment(arm_start_idx, n_arm) = kd;
    
    std::cout << "[ArmTorqueController] 手臂关节起始索引: " << arm_start_idx << std::endl;
    std::cout << "[ArmTorqueController] 手臂关节数量: " << n_arm << std::endl;
}

void ArmTorqueController::setMeasuredState(const Eigen::VectorXd& q_measured,
                                            const Eigen::VectorXd& v_measured) {
    // 使用统一方法获取手臂关节起始索引
    int arm_start_idx = getArmStartIndex();
    
    int n_arm = q_measured.rows();
    assert(q_measured.rows() == n_arm && v_measured.rows() == n_arm);
    
    q_measured_.segment(arm_start_idx, n_arm) = q_measured;
    v_measured_.segment(arm_start_idx, n_arm) = v_measured;
}

Eigen::VectorXd ArmTorqueController::computeTorque(
    const Eigen::VectorXd& q_desired,
    const Eigen::VectorXd& v_desired,
    const Eigen::VectorXd& a_desired) {
    
    // 使用统一方法获取手臂关节起始索引
    int arm_start_idx = getArmStartIndex();
    
    int n_arm = q_desired.rows();
    assert(q_desired.rows() == n_arm && v_desired.rows() == n_arm && a_desired.rows() == n_arm);
    
    // V51版本专用：输入数据NaN检查
    if (q_desired.hasNaN() || v_desired.hasNaN() || a_desired.hasNaN()) {
        std::cerr << "[ArmTorqueController] 错误：输入数据包含NaN值" << std::endl;
        std::cerr << "q_desired.hasNaN(): " << q_desired.hasNaN() << std::endl;
        std::cerr << "v_desired.hasNaN(): " << v_desired.hasNaN() << std::endl;
        std::cerr << "a_desired.hasNaN(): " << a_desired.hasNaN() << std::endl;
        return Eigen::VectorXd::Zero(n_arm);
    }
    
    if (q_measured_.hasNaN() || v_measured_.hasNaN()) {
        std::cerr << "[ArmTorqueController] 错误：测量数据包含NaN值" << std::endl;
        return Eigen::VectorXd::Zero(n_arm);
    }
    
    Eigen::VectorXd q_desired_full = Eigen::VectorXd::Zero(model_.nq);
    q_desired_full.segment(arm_start_idx, n_arm) = q_desired;
    Eigen::VectorXd v_desired_full = Eigen::VectorXd::Zero(model_.nv);
    v_desired_full.segment(arm_start_idx, n_arm) = v_desired;
    Eigen::VectorXd a_desired_full = Eigen::VectorXd::Zero(model_.nv);
    a_desired_full.segment(arm_start_idx, n_arm) = a_desired;
    
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


    return tau.segment(arm_start_idx, n_arm);
}
