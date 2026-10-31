#pragma once

// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/AmpFamilyMemberController.h"
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <ros/ros.h>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace humanoid_controller
{
  /**
   * @brief SOFT 模型控制器：屏蔽速度指令通道，支持手臂阻抗交互
   *
   * 基于 amp_wild 的模型与配置（amp_wild_imp_rl_param.info），叠加手臂阻抗能力：
   *  - /change_arm_ctrl_mode 服务：切换手臂控制模式（0=锁定, 1=自动摆臂, 2=外部控制,
   *    3=关节空间阻抗, 4=笛卡尔空间阻抗）
   *  - /get_arm_ctrl_mode 服务：查询当前手臂模式
   *  - RUIWO 驱动级增益切换（进入阻抗模式 3/4 时调低 RUIWO 手臂增益，退出时恢复 normal）
   *
   * 速度指令约束不等于物理静止：模型仍可根据外力或手臂角度变化产生运动。
   *  - 配置 ignoreTwistCmd=true：不接收任何 twist（含旋转）与 scalar command，
   *    观测中二者恒为 0（详见 RlGaitReceiver 的 ignore_twist_cmd_ 处理）；
   *  - 用户速度指令由家族 WALK/RUN 角色处理。
   */
  class AmpWildImpController : public AmpFamilyMemberController
  {
  public:
    AmpWildImpController(const std::string& name,
                         const std::string& config_file,
                         ros::NodeHandle& nh,
                         ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    ~AmpWildImpController() override;

    /// 复用 AmpFamilyMemberController::initialize()，并额外广告手臂阻抗模式切换服务 + RUIWO 增益切换
    bool initialize() override;
    /// 暂停（包括紧急中止）时恢复正常驱动增益，拒绝晚到的阻抗增益请求。
    void pause() override;

    /// 激活（切进本控制器）时按 armImpedance.enterOnActivate 自动进入手臂阻抗模式
    void resume() override;
    void resumeWarm() override;

  protected:
    /// 主循环：自动进阻抗的有限重试挂在这里（与主循环同频）
    bool updateImpl(const ros::Time& time,
                    const SensorData& sensor_data,
                    const Eigen::VectorXd& measuredRbdState,
                    kuavo_msgs::jointCmd& joint_cmd) override;

  private:
    /**
     * @brief 手臂阻抗模式切换服务回调
     *
     * 请求 control_mode：
     *   0 = 锁定，1 = 自动摆臂，2 = 外部控制，
     *   3 = 关节空间阻抗（柔顺），4 = 笛卡尔空间阻抗（末端柔顺）
     * 仅在当前控制器为活动控制器（isActive）且手臂控制器存在时生效。
     */
    bool changeArmModeCallback(kuavo_msgs::changeArmCtrlMode::Request& req,
                               kuavo_msgs::changeArmCtrlMode::Response& res);

    /// 手臂模式查询服务回调
    bool getArmModeCallback(kuavo_msgs::changeArmCtrlMode::Request& req,
                            kuavo_msgs::changeArmCtrlMode::Response& res);

    ros::ServiceServer arm_mode_service_;   ///< 手臂阻抗模式切换服务
    ros::ServiceServer arm_mode_get_service_;  ///< 手臂模式查询服务

    /// RUIWO 驱动级电机参数切换服务客户端（下位机 lejulib，仅实机）
    ros::ServiceClient srv_change_motor_param_;

    /// 是否启用 RUIWO 驱动级增益切换（由配置文件 use_amp_ruiwo_kpkd 控制，仿真默认关闭）
    bool use_amp_ruiwo_kpkd_{false};

    /// 切进本控制器时自动进入的手臂模式：0=不自动（保持模式1），3=关节阻抗，4=笛卡尔阻抗
    /// 由 armImpedance.enterOnActivate 配置；默认配置了阻抗时取 3
    int arm_enter_mode_{0};
    /// 自动进阻抗待生效标记：激活瞬间手臂可能在归位/插值，ArmController 会拒绝，需在窗口内重试
    bool arm_enter_pending_{false};
    ros::Time arm_enter_deadline_;

    /// 尝试进入 arm_enter_mode_；成功、未配置或超过重试窗口后清除待生效标记
    void tryEnterArmImpedance();

    /**
     * @brief 异步切换 RUIWO 驱动级电机参数（镜像 MoREController 的驱动级参数切换机制）
     * @param param_name kuavo.json 中的参数段名，如 "impedance_kpkd" / "normal_kpkd"
     */
    void changeRuiwoMotorParamAsync(const std::string& param_name);
    void resumeMotorParamSwitching();
    void motorParamWorker();
    std::thread motor_param_worker_;
    std::mutex motor_param_mutex_;
    std::condition_variable motor_param_cv_;
    bool motor_param_stop_{false};
    bool motor_param_active_{false};  ///< guarded by motor_param_mutex_
    std::string pending_motor_param_;
  };

} // namespace humanoid_controller
