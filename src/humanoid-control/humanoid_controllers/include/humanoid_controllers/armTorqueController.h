#pragma once
// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>

class ArmTorqueController {
public:
    ArmTorqueController(){}
    // 构造函数：加载 URDF 并初始化模型
    ArmTorqueController(const std::string& urdf_path, 
                        const Eigen::VectorXd& kp, 
                        const Eigen::VectorXd& kd);

    // 设置目标位置和速度
    void setMeasuredState(const Eigen::VectorXd& q_measured, 
                          const Eigen::VectorXd& v_measured);

    // 计算控制扭矩（完整动力学补偿 + PD 反馈）
    Eigen::VectorXd computeTorque(const Eigen::VectorXd& q_desired, 
                                  const Eigen::VectorXd& v_desired,
                                  const Eigen::VectorXd& a_desired);

    // ==================== 阻抗控制扩展（Phase 1/2） ====================

    /**
     * @brief 运行时设置手臂关节的 PD 增益（阻抗刚度/阻尼）
     * @param kp 比例增益（arm-sized，14 维，左臂在前右臂在后）
     * @param kd 微分增益（arm-sized，14 维）
     *
     * 阻抗控制需要在进入/退出阻抗模式时切换增益：
     *  - 进入：切换为低增益（柔顺）
     *  - 退出：恢复原始 RL 增益（刚性）
     */
    void setGains(const Eigen::VectorXd& kp, const Eigen::VectorXd& kd);

    // ---- Phase 2 笛卡尔阻抗/外力估计支持 ----

    /**
     * @brief 计算末端执行器位姿（基于最近一次 setMeasuredState 的实测状态）
     * @param left true=左手(zarm_l7_end_effector)，false=右手(zarm_r7_end_effector)
     * @return 末端在世界系（=基座默认位）下的齐次变换
     */
    Eigen::Isometry3d getEndEffectorPose(bool left) const;

    /**
     * @brief 计算末端执行器雅可比（6 x n_arm_，末端原点、世界系轴向，忽略基座运动列）
     * @param left true=左手，false=右手
     * @return 6xn_arm 雅可比矩阵，列顺序与手臂关节一致（左臂在前）
     */
    Eigen::MatrixXd getEndEffectorJacobian(bool left) const;

    /**
     * @brief 计算手臂关节的重力补偿项（基于最近一次 setMeasuredState 的实测状态）
     * @return arm-sized 重力力矩（左臂在前）
     */
    Eigen::VectorXd computeGravityArm();

    /**
     * @brief 笛卡尔空间阻抗力矩（Phase 2）：τ = Jᵀ[Kx·Δx + Dx·Δẋ] + G_arm
     * @param x_desired 目标末端位置（世界系）
     * @param v_desired 目标末端速度（世界系，通常为零向量）
     * @param left true=左手，false=右手
     * @param kx 平移刚度 (N/m)
     * @param dx 平移阻尼 (N·s/m)
     * @return arm-sized 力矩（左臂在前）
     */
    Eigen::VectorXd computeCartesianImpedanceTorque(const Eigen::Vector3d& x_desired,
                                                    const Eigen::Vector3d& v_desired,
                                                    bool left,
                                                    double kx,
                                                    double dx);

    /**
     * @brief 估计末端外力（动量观测器简化版，Phase 2）
     *        根据实测关节力矩与模型力矩之差反解：F_ext = (J·Jᵀ)⁻¹·J·(τ_meas − G_arm)
     * @param tau_measured_arm 实测关节力矩（arm-sized，左臂在前）
     * @param left true=左手，false=右手
     * @return 末端外力（世界系，3 维）
     */
    Eigen::Vector3d estimateEndEffectorForce(const Eigen::VectorXd& tau_measured_arm,
                                             bool left) const;

    // ---- 索引查询 ----
    int getLeftArmQStartIdx() const { return left_arm_q_start_idx_; }
    int getLeftArmVStartIdx() const { return left_arm_v_start_idx_; }
    int getRightArmQStartIdx() const { return right_arm_q_start_idx_; }
    int getRightArmVStartIdx() const { return right_arm_v_start_idx_; }
    int getNArm() const { return n_arm_; }
    int getNLeftArm() const { return n_left_arm_; }
    int getNRightArm() const { return n_right_arm_; }
    int getModelNq() const { return model_.nq; }
    int getModelNv() const { return model_.nv; }

private:
    // 获取机器人版本
    static std::string getRobotVersion();
    
    // 计算手臂关节的起始索引（考虑waist关节）
    static int getArmStartIndex();
    
    pinocchio::Model model_;        // 机器人模型
    mutable pinocchio::Data data_;  // 模型数据（const 方法内 FK/Jacobian 计算会修改）
    Eigen::VectorXd kp_;            // 比例增益
    Eigen::VectorXd kd_;            // 微分增益
    Eigen::VectorXd q_measured_;    // 测量关节位置
    Eigen::VectorXd v_measured_;    // 测量关节速度
    const int n_leg_joints_{12};    // 腿关节数量
    const int n_arm_joints_{14};    // 臂关节数量（最大值，实际由URDF决定）
    int n_left_arm_{0};             // 左手臂实际关节数量（由URDF动态确定）
    int n_right_arm_{0};            // 右手臂实际关节数量（由URDF动态确定）
    
    int left_arm_q_start_idx_;      // 左手臂关节在q中的起始索引
    int left_arm_v_start_idx_;      // 左手臂关节在v中的起始索引
    int right_arm_q_start_idx_;     // 右手臂关节在q中的起始索引
    int right_arm_v_start_idx_;     // 右手臂关节在v中的起始索引
    int n_arm_;                     // 实际找到的手臂关节数量
    
    // 辅助函数：设置左、右手臂的数据
    void setArmData(Eigen::VectorXd& target_q, Eigen::VectorXd& target_v,
                    const Eigen::VectorXd& source_q, const Eigen::VectorXd& source_v) const;
    void setArmData(Eigen::VectorXd& target_q, Eigen::VectorXd& target_v, Eigen::VectorXd& target_a,
                    const Eigen::VectorXd& source_q, const Eigen::VectorXd& source_v, const Eigen::VectorXd& source_a) const;
};
