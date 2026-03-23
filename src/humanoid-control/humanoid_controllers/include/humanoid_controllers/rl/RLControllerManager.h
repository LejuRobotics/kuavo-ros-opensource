#pragma once

#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/rl_controller_types.h"
#include "humanoid_interface/common/TopicLogger.h"
#include "kuavo_msgs/switchController.h"
#include "kuavo_msgs/getControllerList.h"
#include "kuavo_msgs/switchToNextController.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include <map>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <functional>
#include <ros/ros.h>

namespace humanoid_controller
{
  /**
   * @brief RL控制器管理类
   * 提供控制器的加载、切换、查询等功能
   */
  class RLControllerManager
  {
  public:
    /**
     * @brief 构造函数
     */
    RLControllerManager();

    /**
     * @brief 析构函数
     */
    ~RLControllerManager() = default;

    /**
     * @brief 添加控制器
     * @param name 控制器名称
     * @param controller 控制器指针（所有权转移）
     * @return 是否添加成功
     */
    bool addController(const std::string& name, std::unique_ptr<RLControllerBase> controller);

    /**
     * @brief 根据名称获取控制器
     * @param name 控制器名称
     * @return 控制器指针，如果不存在返回nullptr
     */
    RLControllerBase* getControllerByName(const std::string& name);

    /**
     * @brief 根据索引获取控制器
     * @param index 控制器索引（从0开始）
     * @return 控制器指针，如果索引无效返回nullptr
     */
    RLControllerBase* getControllerByIndex(size_t index);

    /**
     * @brief 根据类型获取控制器
     * @param type 控制器类型
     * @return 控制器指针，如果不存在返回nullptr
     */
    RLControllerBase* getControllerByType(RLControllerType type);

    /**
     * @brief 根据名称获取控制器类型
     * @param name 控制器名称
     * @return 控制器类型，如果不存在返回RLControllerType::MPC
     */
    RLControllerType getControllerTypeByName(const std::string& name);

    /**
     * @brief 根据索引获取控制器类型
     * @param index 控制器索引
     * @return 控制器类型，如果索引无效返回RLControllerType::MPC
     */
    RLControllerType getControllerTypeByIndex(size_t index);

    /**
     * @brief 切换当前控制器
     * @param name 控制器名称（空字符串表示切换到MPC控制器）
     * @return 是否切换成功
     */
    bool switchController(const std::string& name);

    /**
     * @brief 切换当前控制器（根据类型）
     * @param type 控制器类型
     * @return 是否切换成功
     */
    bool switchController(RLControllerType type);

    /**
     * @brief 获取当前控制器指针
     * @return 当前控制器指针，如果当前是MPC控制器则返回nullptr
     */
    RLControllerBase* getCurrentController();

    RLControllerBase* getLastController()
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      return last_controller_ptr_;
    }

    /**
     * @brief 获取当前控制器类型
     * @return 当前控制器类型
     */
    RLControllerType getCurrentControllerType();

    /**
     * @brief 获取当前控制器名称
     * @return 当前控制器名称，如果是MPC控制器则返回空字符串
     */
    std::string getCurrentControllerName();

    /**
     * @brief 检查控制器是否存在
     * @param name 控制器名称
     * @return 是否存在
     */
    bool hasController(const std::string& name);

    /**
     * @brief 检查指定类型的控制器是否存在
     * @param type 控制器类型
     * @return 是否存在
     */
    bool hasController(RLControllerType type);

    /**
     * @brief 获取所有控制器名称列表
     * @return 控制器名称列表
     */
    std::vector<std::string> getControllerNames();

    /**
     * @brief 获取控制器数量
     * @return 控制器数量
     */
    size_t getControllerCount();

    /**
     * @brief 切换到BASE控制器（MPC控制器）
     */
    void switchToBaseController();

    /**
     * @brief 检查当前是否为BASE控制器
     * @return 是否为BASE控制器
     */
    bool isBaseController();

    /**
     * @brief 从配置文件加载控制器列表
     * @param config_file 控制器列表配置文件路径（YAML格式）
     * @param version_config_dir 版本配置目录路径（用于解析相对路径）
     * @param nh ROS节点句柄
     * @param ros_logger ROS日志发布器（可选，可以为nullptr）
     * @return 是否加载成功
     */
    bool loadControllersFromConfig(const std::string& config_file, 
                                   const std::string& version_config_dir,
                                   ros::NodeHandle& nh,
                                   ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    /**
     * @brief 获取倒地起身控制器的名称
     * @return 倒地起身控制器的名称，如果不存在返回空字符串
     */
    std::string getFallStandControllerName();

    /**
     * @brief 检查当前是否为BASE控制器（用于更新is_rl_controller_状态）
     * @return 如果当前是BASE控制器返回true，否则返回false
     */
    bool isBaseControllerActive();

    /**
     * @brief 初始化ROS服务
     * @param nh ROS节点句柄
     * @return 是否初始化成功
     */
    bool initializeRosServices(ros::NodeHandle& nh);
    
    /**
     * @brief 设置是否直接切换到RL控制器（跳过MPC过渡）
     */
    void setDirectSwitchToRL(bool direct)
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      direct_switch_to_rl_ = direct;
    }

    /**
     * @brief 获取当前是否直接切换到RL控制器（跳过MPC过渡）
     */
    bool getDirectSwitchToRL() const
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      return direct_switch_to_rl_;
    }

    /**
     * @brief 更新MPC控制器的stance状态（由humanoidController调用）
     * @param is_stance 是否处于stance模式
     * @param gait_name 当前步态名称
     */
    void setMpcStanceState(bool is_stance, const std::string& gait_name);

    /**
     * @brief 获取WALK_CONTROLLER类型的控制器列表（包括BASE控制器）
     * @return WALK_CONTROLLER类型的控制器名称列表
     */
    std::vector<std::string> getWalkControllerList();

    /**
     * @brief 注册倒地状态回调函数
     * @param callback 回调函数，参数为FallStandState枚举值（0=STANDING, 1=FALL_DOWN）
     */
    void registerFallDownStateCallback(std::function<void(int)> callback)
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      fall_down_state_callback_ = callback;
    }

    /**
     * @brief 注册躯干速度回调函数
     * @param callback 回调函数，返回12维躯干状态向量 [x, y, z, yaw, pitch, roll, vx, vy, vz, angularVx, angularVy, angularVz]
     */
    void registerTorsoStabilityCallback(std::function<bool()> callback)
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      torso_stability_callback_ = callback;
    }

    /**
     * @brief 检查躯干速度是否稳定（从状态估计器获取）
     * @return 如果速度稳定返回true，否则返回false
     */
    bool isTorsoVelocityStable();

  private:
    /**
     * @brief 异步切换手臂控制模式
     * @param mode 目标手臂控制模式
     */
    void changeArmCtrlModeAsync(int mode);

    /**
     * @brief ROS服务回调：切换控制器
     */
    bool switchControllerCallback(kuavo_msgs::switchController::Request &req, 
                                   kuavo_msgs::switchController::Response &res);

    /**
     * @brief ROS服务回调：获取控制器列表
     */
    bool getControllerListCallback(kuavo_msgs::getControllerList::Request &req, 
                                   kuavo_msgs::getControllerList::Response &res);

    /**
     * @brief ROS服务回调：切换到下一个控制器
     */
    bool switchToNextControllerCallback(kuavo_msgs::switchToNextController::Request &req, 
                                        kuavo_msgs::switchToNextController::Response &res);

    /**
     * @brief ROS服务回调：切换到上一个控制器
     */
    bool switchToPreviousControllerCallback(kuavo_msgs::switchToNextController::Request &req, 
                                            kuavo_msgs::switchToNextController::Response &res);
    
    /**
     * @brief ROS服务回调：设置RL切换模式（是否直接切换）
     */
    bool setRLSwitchModeCallback(std_srvs::SetBool::Request &req,
                                 std_srvs::SetBool::Response &res);

    /**
     * @brief ROS服务回调：设置倒地状态
     */
    bool setFallDownStateCallback(std_srvs::SetBool::Request &req,
                                  std_srvs::SetBool::Response &res);

    /**
     * @brief ROS服务回调：切换到VMP控制器
     */
    bool switchToVMPControllerCallback(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res);

    /**
     * @brief ROS服务回调：切换到Dance控制器
     */
    bool switchToDanceControllerCallback(std_srvs::Trigger::Request &req,
                                         std_srvs::Trigger::Response &res);

    /**
     * @brief 更新按类型分组的控制器列表
     */
    void updateControllerListsByType();


  private:
    std::map<std::string, std::unique_ptr<RLControllerBase>> controllers_;  ///< 控制器映射表
    RLControllerBase* last_controller_ptr_ = nullptr;                       ///< 上一个控制器指针（不拥有所有权，仅做记录）
    std::vector<std::string> controller_names_;                             ///< 控制器名称列表（保持顺序）
    std::string current_controller_name_;                                    ///< 当前控制器名称（空字符串表示BASE）
    mutable std::recursive_mutex mutex_;                                    ///< 线程安全锁（递归锁，允许同一线程重入）

    // 按类型分组的控制器列表（不包括BASE）
    std::map<RLControllerType, std::vector<std::string>> controllers_by_type_;  ///< 按类型分组的控制器列表
    std::vector<std::string> walk_controllers_;                                ///< WALK_CONTROLLER类型列表（包括BASE，BASE在索引0）

    // ROS服务
    ros::ServiceServer switch_controller_srv_;      ///< 切换控制器服务
    ros::ServiceServer get_controller_list_srv_;     ///< 获取控制器列表服务
    ros::ServiceServer switch_to_next_controller_srv_;  ///< 切换到下一个控制器服务
    ros::ServiceServer switch_to_previous_controller_srv_;  ///< 切换到上一个控制器服务
    ros::ServiceServer set_rl_switch_mode_srv_;       ///< 设置RL切换模式服务
    ros::ServiceServer set_fall_down_state_srv_;     ///< 设置倒地状态服务
    ros::ServiceServer switch_to_vmp_controller_srv_; ///< 切换到VMP控制器服务
    ros::ServiceServer switch_to_dance_controller_srv_; ///< 切换到Dance控制器服务
    ros::NodeHandle* nh_ptr_;                       ///< ROS节点句柄指针

    // RL切换模式：true 直接切换到RL；false 使用MPC过渡
    bool direct_switch_to_rl_ = true;
    // 倒地状态回调函数
    std::function<void(int)> fall_down_state_callback_;  ///< 设置倒地状态的回调函数
    // 躯干速度检查相关
    std::function<bool()> torso_stability_callback_;          ///< 获取躯干稳定性状态的回调函数

    bool mpc_is_stance_mode_ = false;               ///< MPC控制器是否处于stance模式
    std::string mpc_current_gait_name_ = "stance";  ///< MPC控制器当前步态名称
  };

} // namespace humanoid_controller

