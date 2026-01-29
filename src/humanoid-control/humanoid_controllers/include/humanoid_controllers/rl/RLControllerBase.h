#pragma once

// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/rl_controller_types.h"
#include "humanoid_controllers/sensor_data_types.h"
#include "humanoid_controllers/LowPassFilter.h"
#include "kuavo_msgs/jointCmd.h"
#include "humanoid_interface/common/TopicLogger.h"
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <Eigen/Dense>
#include "humanoid_controllers/rl/armController.h"
#include "humanoid_controllers/rl/waistController.h"

namespace humanoid_controller
{
 /**
  * @brief
  * 将Eigen::VectorXd中indexA位置的值移动到indexB，并将两者之间的区间向indexA方向平移。
  *
  * 例如：vec = [a, b, c, d], indexA = 0, indexB = 2
  * 过程：取出a，区间[b, c]整体向前移动得到[b, c, d,
  * d]，最后将a放到indexB处，结果为[b, c, a, d]。
  *
  * @param vec      需要操作的向量
  * @param indexA   源位置
  * @param indexB   目标位置
  */
  static void moveVectorEntry(Eigen::VectorXd &vec, int indexA, int indexB) {
    if (indexA == indexB)
      return;

    const int size = static_cast<int>(vec.size());
    if (indexA < 0 || indexA >= size || indexB < 0 || indexB >= size)
      throw std::out_of_range("moveVectorEntry: index out of range");

    const double value = vec(indexA);
    if (indexA < indexB) {
      for (int i = indexA; i < indexB; ++i) {
        vec(i) = vec(i + 1);
      }
      vec(indexB) = value;
    } else {
      for (int i = indexA; i > indexB; --i) {
        vec(i) = vec(i - 1);
      }
      vec(indexB) = value;
    }
  }
  /**
  * @brief 将std::vector中indexA位置的元素移动到indexB位置，其余元素相应移位。
  *
  * @param vec      需要操作的std::vector
  * @param indexA   源位置
  * @param indexB   目标位置
  */
  template<typename T>
  static void moveStdVectorEntry(std::vector<T> &vec, int indexA, int indexB)
  {
    if (indexA == indexB)
      return;

    const int size = static_cast<int>(vec.size());
    if (indexA < 0 || indexA >= size || indexB < 0 || indexB >= size)
      throw std::out_of_range("moveStdVectorEntry: index out of range");

    const T value = vec[indexA];
    if (indexA < indexB)
    {
      for (int i = indexA; i < indexB; ++i)
      {
        vec[i] = vec[i + 1];
      }
      vec[indexB] = value;
    }
    else
    {
      for (int i = indexA; i > indexB; --i)
      {
        vec[i] = vec[i - 1];
      }
      vec[indexB] = value;
    }
  }

/**
 * @brief RL控制器基类
 * 提供统一的接口和基础功能，所有RL控制器都应继承此类
 */
class RLControllerBase {
public:
  /**
   * @brief 构造函数
   * @param name 控制器名称
   * @param type 控制器类型
   * @param config_file 配置文件路径
   * @param nh ROS节点句柄
   * @param ros_logger ROS日志发布器（可选，可以为nullptr）
   */
  RLControllerBase(const std::string &name, RLControllerType type,
                   const std::string &config_file, ros::NodeHandle &nh,
                   ocs2::humanoid::TopicLogger *ros_logger = nullptr);

  /**
   * @brief 虚析构函数
   */
  virtual ~RLControllerBase() = default;

  /**
   * @brief 初始化控制器
   * @return 是否初始化成功
   */
  virtual bool initialize() = 0;

  /**
   * @brief 更新控制器（主接口，基类负责存储数据，派生类实现具体逻辑）
   * @param time 当前时间
   * @param sensor_data 传感器数据
   * @param measuredRbdState 测量的刚体状态
   * @param joint_cmd 输出的关节命令（输出参数）
   * @return 是否更新成功
   */
  bool update(const ros::Time &time, const SensorData &sensor_data,
              const Eigen::VectorXd &measuredRbdState,
              kuavo_msgs::jointCmd &joint_cmd);

  void applyBaseState(const Eigen::VectorXd &baseState){baseStateRL_ = baseState;};
  void applyFeetPositions(const Eigen::VectorXd &feetPositions){feetPositionsRL_ = feetPositions;};

  /**
   * @brief 重置控制器状态
   */
  virtual void reset() = 0;

  // Getter方法（公开接口）
  std::string getName() const { return name_; }
  RLControllerType getType() const { return type_; }
  ControllerState getState() const { return state_; }
  bool isActive() const { return state_ == ControllerState::RUNNING; }
  bool isInitialized() const { return initialized_; }

  /**
   * @brief 检查控制器是否完成任务并准备好退出
   * @return 如果控制器已完成任务并准备好退出，返回true
   */
  virtual bool isReadyToExit() const { return false; }

  /**
   * @brief 检查控制器当前是否处于 stance（站立）模式
   * @return 如果控制器处于 stance 模式返回 true，否则返回 false
   */
  virtual bool isInStanceMode() const { return true; }

  /**
   * @brief 获取控制器的初始状态（用于设置仿真/机器人初始状态）
   * @return 初始状态向量 initialStateRL_ 的引用

   * 派生类应该在 loadConfig 中加载 defaultBaseStateRL_ 并设置 initialStateRL_
   */
  virtual const Eigen::VectorXd& getInitialState() const { return initialStateRL_; }
    
  Eigen::VectorXd getDefaultJointPos() const { return defalutJointPosRL_; }
  Eigen::VectorXd getDefaultBaseState() const { return defaultBaseStateRL_; }
  double getDefaultBaseHeightControl() const { return defaultBaseHeightControl_; }
  double getDefaultBaseXOffsetControl() const { return defaultBaseXOffsetControl_; }
  
  /**
   * @brief 获取是否从MPC切换时使用插值过渡
   * @return true表示使用插值过渡，false表示直接切换
   */
  bool getUseInterpolateFromMPC() const { return use_interpolate_from_mpc_; }

  /**
   * @brief 获取是否使用默认的电机CSP kp/kd（从kuavo.json）
   * @return true表示使用默认的kp/kd，false表示使用info文件中定义的motor_kp/motor_kd
   */
  bool getUseDefaultMotorCspKpkd() const { return use_default_motor_csp_kpkd_; }

  /**
   * @brief 更新速度限制到rosparam（虚函数，派生类可重写）
   * 基类默认实现：使用MPC默认速度限制
   * 派生类可以重写此方法以设置自己的速度限制
   * @param nh ROS节点句柄
   */
  virtual void updateVelocityLimitsParam(ros::NodeHandle& nh);

  /**
   * @brief 重新加载配置文件
   * 自动调用派生类的loadConfig方法重新加载配置文件
   * @return 是否重新加载成功
   */
  virtual bool reload();

  /**
   * @brief 启动控制器（启动推理线程）
   */
  virtual void start();

  /**
   * @brief 停止控制器（停止推理线程）
   */
  virtual void stop();

  /**
   * @brief 暂停控制器（暂停推理过程）
   */
  virtual void pause();

  /**
   * @brief 恢复控制器（恢复推理过程）
   */
  virtual void resume();

  /**
   * @brief 等待下一个控制周期（用于控制频率管理）
   * 派生类可重写此方法以实现自定义的频率控制
   */
  virtual void waitForNextCycle();

  /**
   * @brief 获取控制器的控制频率
   * @return 控制频率（Hz）
   */
  double getControlFrequency() const { return control_frequency_; }

  /**
   * @brief 获取手臂控制器（如果存在）
   * @return 手臂控制器指针，如果不存在则返回 nullptr
   */
  ArmController* getArmController() { return arm_controller_.get(); }
  
  /**
   * @brief 获取手臂控制器（const版本）
   * @return 手臂控制器指针，如果不存在则返回 nullptr
   */
  const ArmController* getArmController() const { return arm_controller_.get(); }

protected:
  /**
   * @brief 更新控制器实现（派生类需要重写）
   * @param time 当前时间
   * @param sensor_data 传感器数据
   * @param measuredRbdState 测量的刚体状态
   * @param joint_cmd 输出的关节命令（输出参数）
   * @return 是否更新成功
   */
  virtual bool updateImpl(const ros::Time &time, const SensorData &sensor_data,
                          const Eigen::VectorXd &measuredRbdState,
                          kuavo_msgs::jointCmd &joint_cmd) = 0;

  /**
   * @brief 获取机器人传感器数据（线程安全）
   * @return 传感器数据
   */
  SensorData getRobotSensorData() const;


  /**
   */
  void setRobotSensorData(const SensorData &sensor_data);

  void setRobotState(const Eigen::VectorXd &state);

  /**
   * @brief 获取机器人状态（线程安全）
   * @return 机器人状态
   */
  Eigen::VectorXd getRobotState() const;

  /**
   * @brief 获取当前最新的动作（线程安全）
   * @return 当前动作
   */
  Eigen::VectorXd getCurrentAction() const;

  void setCurrentAction(Eigen::VectorXd action);

protected:
  /**
   * @brief 裁剪向量值到指定范围
   * @param a 要裁剪的向量（输入输出参数）
   * @param num 要处理的元素数量
   * @param limit 限制值（正负对称）
   */
  static void clip(Eigen::VectorXd &a, double limit);

  /**
   * @brief 加载配置文件
   * @param config_file 配置文件路径
   * @return 是否加载成功
   */
  virtual bool loadConfig(const std::string &config_file) = 0;

  /**
   * @brief 传感器数据预处理（例如RL相关的IMU滤波等）
   * @param sensor_data 传感器数据（输入输出参数）
   *
   * 默认实现会在基类中根据是否已初始化滤波器，自动对 IMU
   * 数据进行与原 `humanoidController` 中 RL 滤波逻辑一致的处理。
   * 派生类可以在重写时先调用 RLControllerBase::preprocessSensorData，
   * 再追加自己的特殊处理（例如倒地起身中腰关节方向的修正等）。
   */
  virtual void preprocessSensorData(SensorData &sensor_data);

  /**
   * @brief 从RL配置文件中加载并初始化IMU滤波参数
   * @param config_file RL配置文件路径（与各控制器传入的一致）
   * @return 是否加载并成功初始化滤波器
   *
   * 该函数会尝试从配置中读取
   *  - accFilterCutoffFreq
   *  - freeAccFilterCutoffFreq
   *  - gyroFilterCutoffFreq
   *  - accFilterState
   *  - freeAccFilterState
   *  - gyroFilterState
   * 并使用 ROS 参数 `/controller_dt` 作为采样时间（若不存在则使用 0.002）。
   */
  bool loadRLFilterParams(const std::string &config_file);

  /**
   * @brief 基础推理过程（可在派生类中重写）
   * @param observation 观测数据
   * @param action 输出的动作（输出参数）
   * @return 是否推理成功
   */
  virtual bool inference(const Eigen::VectorXd &observation,
                         Eigen::VectorXd &action);

  /**
   * @brief 更新观测数据（可在派生类中重写）
   * @param state_est 状态估计
   * @param sensor_data 传感器数据
   */
  virtual void updateObservation(const Eigen::VectorXd &state_est,
                                 const SensorData &sensor_data);

  /**
   * @brief 将动作转换为关节命令（可在派生类中重写）
   * @param actuation 执行器输出（actuation）
   * @param measuredRbdState 测量的刚体状态
   * @param joint_cmd 关节命令（输出参数）
   */
  virtual void actionToJointCmd(const Eigen::VectorXd &actuation,
                                const Eigen::VectorXd &measuredRbdState,
                                kuavo_msgs::jointCmd &joint_cmd);

  /**
   * @brief 更新手臂指令（可选功能，用于替换jointCmdMsg中的手臂部分）
   * @param time 当前时间
   * @param sensor_data 传感器数据
   * @param joint_cmd 关节命令（输入输出参数，手臂部分将被替换）
   * @return 如果使用了外部手臂指令替换，返回true；否则返回false
   * 
   * 此函数在actionToJointCmd之后调用，用于可选地替换手臂部分的指令。
   * 派生类可以重写此函数以实现自定义的手臂控制逻辑。
   */
  virtual bool updateArmCommand(const ros::Time &time,
                                const SensorData &sensor_data,
                                kuavo_msgs::jointCmd &joint_cmd);

  /**
   * @brief 设置是否启用外部手臂指令替换
   * @param enabled 是否启用
   */
  void use_external_arm_controller(bool enabled) { arm_command_replacement_enabled_ = enabled; }

  /**
   * @brief 获取是否启用外部手臂指令替换
   * @return 是否启用
   */
  bool isArmCommandReplacementEnabled() const { return arm_command_replacement_enabled_; }

  /**
   * @brief 更新腰部指令（可选功能，用于替换jointCmdMsg中的腰部部分）
   * @param time 当前时间
   * @param sensor_data 传感器数据
   * @param joint_cmd 关节命令（输入输出参数，腰部部分将被替换）
   * @return 如果使用了外部腰部指令替换，返回true；否则返回false
   * 
   * 此函数在actionToJointCmd之后调用，用于可选地替换腰部部分的指令。
   * 派生类可以重写此函数以实现自定义的腰部控制逻辑。
   */
  virtual bool updateWaistCommand(const ros::Time &time,
                                  const SensorData &sensor_data,
                                  kuavo_msgs::jointCmd &joint_cmd);

  /**
   * @brief 设置是否启用外部腰部指令替换
   * @param enabled 是否启用
   */
  void use_external_waist_controller(bool enabled) { waist_command_replacement_enabled_ = enabled; }

  /**
   * @brief 获取是否启用外部腰部指令替换
   * @return 是否启用
   */
  bool isWaistCommandReplacementEnabled() const { return waist_command_replacement_enabled_; }

  /**
   * @brief 初始化ROS服务（在构造函数中自动调用）
   * 同时初始化关节数量和RL相关变量
   */
  void initializeServices();

  /**
   * @brief 初始化RL相关变量（在initializeServices中调用）
   * 从ROS参数获取关节数量，并初始化相关向量
   */
  void initializeRLVariables();

  /**
   * @brief 服务回调函数
   */
  bool reloadServiceCallback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res);
  bool isActiveServiceCallback(std_srvs::Trigger::Request &req,
                               std_srvs::Trigger::Response &res);
  bool getStateServiceCallback(std_srvs::Trigger::Request &req,
                               std_srvs::Trigger::Response &res);
  bool getTypeServiceCallback(std_srvs::Trigger::Request &req,
                              std_srvs::Trigger::Response &res);
  bool resetServiceCallback(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res);

  /**
   * @brief 推理线程函数（可在派生类中重写以添加特定条件）
   */
  virtual void inferenceThreadFunc();

  /**
   * @brief 检查是否应该执行推理（可在派生类中重写以添加特定条件）
   * @return 如果应该执行推理，返回true
   */
  virtual bool shouldRunInference() const;

  // 成员变量
  std::string name_;        ///< 控制器名称
  RLControllerType type_;   ///< 控制器类型
  ControllerState state_;   ///< 控制器状态
  bool initialized_;        ///< 是否已初始化
  std::string config_file_; ///< 配置文件路径
  ros::NodeHandle &nh_;     ///< ROS节点句柄引用

  // ROS服务
  ros::ServiceServer reload_srv_;    ///< 重新加载配置服务
  ros::ServiceServer is_active_srv_; ///< 是否激活服务
  ros::ServiceServer get_state_srv_; ///< 获取状态服务
  ros::ServiceServer get_type_srv_;  ///< 获取类型服务
  ros::ServiceServer reset_srv_;     ///< 重置控制器服务

  // 推理线程相关
  std::thread inference_thread_;                    ///< 推理线程
  std::atomic<bool> inference_thread_created_{false}; ///< 推理线程是否已创建
  double inference_frequency_{100.0};               ///< 推理频率（Hz）

  // 控制频率相关
  double control_frequency_{500.0};                 ///< 控制频率（Hz），从配置文件读取，默认使用 /wbc_frequency
  std::unique_ptr<ros::Rate> control_rate_;         ///< 控制频率 Rate 对象
  Eigen::VectorXd
      networkInputDataRL_; ///< 将所有历史长度的数据接成一个向量（用于推理）
  int num_actions_ = 0; ///< 动作维度（用于基类inference的默认实现）

  // 关节数量（从ROS参数获取）
  int jointNum_ = 0;    ///< 腿部关节数量
  int jointArmNum_ = 0; ///< 手臂关节数量
  int waistNum_ = 0;    ///< 腰部关节数量
  int headNum_ = 0;     ///< 头部关节数量

  // RL控制相关参数（在initializeServices/各控制器loadConfig中初始化）
  Eigen::VectorXd defalutJointPosRL_;  ///< 默认关节位置
  Eigen::VectorXd JointControlModeRL_; ///< 关节控制模式
  Eigen::VectorXd JointPDModeRL_;      ///< 关节PD模式
  Eigen::VectorXd jointKpRL_;          ///< 关节Kp参数
  Eigen::VectorXd jointKdRL_;          ///< 关节Kd参数
  Eigen::VectorXd torqueLimitsRL_;     ///< 力矩限制
  Eigen::VectorXd actionScaleTestRL_;  ///< 动作缩放测试
  Eigen::VectorXd defaultBaseStateRL_; ///< 默认基座状态 [vel(6) + pos(3) + angular(3)] = 12维
  double defaultBaseHeightControl_ = 0.64; // 使用该控制器站立时的高度
  double defaultBaseXOffsetControl_ = 0.0; // 使用该控制器站立时, base 在 x 方向相对于足端中心(0)的偏移
  Eigen::VectorXd initialStateRL_;    ///< 初始状态 [defaultBaseStateRL_(12) + defalutJointPosRL_]
  Eigen::VectorXd feetPositionsRL_;   ///< 脚位置
  Eigen::VectorXd baseStateRL_;       ///< 基座状态
  bool use_interpolate_from_mpc_ = false; ///< 从MPC切换时是否使用插值过渡（true:使用插值，false:直接切换）
  bool use_default_motor_csp_kpkd_ = true; ///< 是否使用默认的电机CSP kp/kd（true:使用kuavo.json中的默认值，false:使用info文件中的motor_kp/motor_kd）
  Eigen::VectorXd motorPdoKp_;         ///< 电机Kp参数（从info文件加载，用于替换control_modes==2的关节）
  Eigen::VectorXd motorPdoKd_;         ///< 电机Kd参数（从info文件加载，用于替换control_modes==2的关节）

  // RL 相关 IMU 滤波参数与滤波器
  bool rl_filter_initialized_ = false; ///< RL IMU 滤波器是否已初始化
  Eigen::Vector3d accFilterCutoffFreqRL_; ///< 加速度滤波截止频率
  Eigen::Vector3d freeAccFilterCutoffFreqRL_; ///< 无重力加速度滤波截止频率
  Eigen::Vector3d gyroFilterCutoffFreqRL_; ///< 角速度滤波截止频率
  Eigen::Vector3d accFilterStateRL_;       ///< 加速度滤波混合系数
  Eigen::Vector3d freeAccFilterStateRL_; ///< 无重力加速度滤波混合系数
  Eigen::Vector3d gyroFilterStateRL_;    ///< 角速度滤波混合系数
  LowPassFilter2ndOrder accFilterRL_;    ///< RL用线加速度滤波器
  LowPassFilter2ndOrder freeAccFilterRL_; ///< RL用 free 加速度滤波器
  LowPassFilter2ndOrder gyroFilterRL_;    ///< RL用角速度滤波器

  // 动作数据（线程安全）
  mutable std::mutex action_mtx_; ///< 动作数据互斥锁
  Eigen::VectorXd actions_;       ///< 当前动作

  // 存储的传感器数据和状态（线程安全）
  mutable std::recursive_mutex sensor_data_mtx_;    ///< 传感器数据互斥锁
  SensorData stored_sensor_data_;         ///< 存储的传感器数据
  Eigen::VectorXd stored_measured_state_; ///< 存储的测量状态
  std::atomic<bool> sensor_data_updated_{false}; ///< 传感器数据是否已更新

  // ROS日志发布器
  ocs2::humanoid::TopicLogger *ros_logger_; ///< ROS日志发布器指针

  // 手臂指令替换相关（可选功能）
  bool arm_command_replacement_enabled_{false}; ///< 是否启用外部手臂指令替换
  std::unique_ptr<ArmController> arm_controller_; ///< 手臂控制器（可选，统一管理手臂插值和控制）

  // 腰部指令替换相关（可选功能）
  bool waist_command_replacement_enabled_{false}; ///< 是否启用外部腰部指令替换
};

} // namespace humanoid_controller







