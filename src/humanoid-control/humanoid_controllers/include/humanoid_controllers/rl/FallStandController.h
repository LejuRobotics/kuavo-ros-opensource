#pragma once

#include "humanoid_controllers/rl/RLControllerBase.h"
#include <openvino/openvino.hpp>
#include <memory>
#include "kuavo_solver/ankle_solver.h"
#include <Eigen/Dense>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

namespace humanoid_controller
{
  /**
   * @brief 运动轨迹数据结构
   * 用于存储和访问预定义的机器人运动轨迹
   */
  class MotionTrajectoryData
  {
  public:
    Eigen::MatrixXd joint_pos;      // (time_steps, num_joints) - joint positions
    Eigen::MatrixXd joint_vel;      // (time_steps, num_joints) - joint velocities
    Eigen::MatrixXd body_pos_w;     // (time_steps, 3) - body positions in world frame (base link)
    Eigen::MatrixXd body_quat_w;    // (time_steps, 4) - body quaternions in world frame (base link)
    int time_step_total;           // total number of time steps
    int current_time_step;         // current time step index
    double reference_yaw;          // reference yaw angle from first frame of trajectory
    
    MotionTrajectoryData() : current_time_step(0), reference_yaw(0.0) {}
    
    // Get command (joint_pos + joint_vel) for current time step
    Eigen::VectorXd getCurrentCommand() const {
      if (time_step_total == 0 || joint_pos.rows() == 0) {
        return Eigen::VectorXd::Zero(joint_pos.cols() * 2);
      }
      int step = std::min(current_time_step, time_step_total - 1);
      
      Eigen::VectorXd command(joint_pos.cols() * 2);
      command.head(joint_pos.cols()) = joint_pos.row(step);
      command.tail(joint_vel.cols()) = joint_vel.row(step);
      return command;
    }
    // Get target anchor position (body position from trajectory)
    Eigen::Vector3d getTargetAnchorPos() const {
      if (time_step_total == 0) {
        return Eigen::Vector3d::Zero();
      }      
      int step = std::min(current_time_step, time_step_total - 1);
      // std::cout << "Step: " << step <<"body_pos_w " << body_pos_w.row(step).transpose() << std::endl;
      return body_pos_w.row(step).transpose();
    }
    
    // Get target anchor orientation (body quaternion from trajectory)
    Eigen::Quaterniond getTargetAnchorQuat() const {      
      int step = std::min(current_time_step, time_step_total - 1);
      Eigen::Vector4d quat_vec = body_quat_w.row(step).transpose();
      // Assuming quaternion is stored as [w, x, y, z]
      return Eigen::Quaterniond(quat_vec(0), quat_vec(1), quat_vec(2), quat_vec(3));
    }
    
    // Update time step (stop at end, no wrap-around)
    void updateTimeStep() {
      if (time_step_total > 0 && current_time_step < time_step_total - 1) {
        current_time_step++;
      }
      printf("Current time step: %d / %d\n", current_time_step, time_step_total);
      // current_time_step = time_step_total * 0.7;
      // current_time_step = 392;
    }
    bool isFinish() {
      return current_time_step >= time_step_total-1;
    }
    void resetTimeStep() {
      current_time_step = 0;
    }
    void setTimeStep(int time_step) {
      current_time_step = time_step;
    }
    int getTimeStep() const {
      return current_time_step;
    }
    int getTimeStepTotal() const {
      return time_step_total;
    }
  };

  /**
   * @brief 倒地起身控制器
   * 实现倒地起身相关的推理和控制逻辑
   */
  class FallStandController : public RLControllerBase
  {
  public:
    /**
     * @brief 构造函数
     * @param name 控制器名称
     * @param config_file 配置文件路径
     * @param nh ROS节点句柄
     * @param ros_logger ROS日志发布器（可选，可以为nullptr）
     */
    FallStandController(const std::string& name, const std::string& config_file,
                       ros::NodeHandle& nh,
                       ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    /**
     * @brief 析构函数
     */
    virtual ~FallStandController();

    /**
     * @brief 初始化控制器
     * @return 是否初始化成功
     */
    bool initialize() override;

    /**
     * @brief 加载配置文件
     * @param config_file 配置文件路径
     * @return 是否加载成功
     */
    bool loadConfig(const std::string& config_file) override;

    /**
     * @brief 重置控制器状态
     */
    void reset() override;

    void resume() override;

    /**
     * @brief 检查控制器是否完成任务并准备好退出
     * @return 如果控制器已完成任务并准备好退出，返回true
     */
    bool isReadyToExit() const override;

  protected:
    /**
     * @brief 更新控制器实现（重写基类方法）
     * @param time 当前时间
     * @param sensor_data 传感器数据
     * @param measuredRbdState 测量的刚体状态
     * @param joint_cmd 输出的关节命令（输出参数）
     * @return 是否更新成功
     */
    bool updateImpl(const ros::Time& time, 
                    const SensorData& sensor_data,
                    const Eigen::VectorXd& measuredRbdState,
                    kuavo_msgs::jointCmd& joint_cmd) override;

    /**
     * @brief 传感器预处理（包含基类的 IMU 滤波以及倒地起身相关关节方向修正）
     * @param sensor_data 传感器数据（输入输出参数）
     *
     * 先调用基类的 RLControllerBase::preprocessSensorData 进行通用滤波，
     * 再对 roban 机型的腰部关节方向进行修正，使其行为与原 humanoidController 保持一致。
     */
    void preprocessSensorData(SensorData& sensor_data) override;

    /**
     * @brief 触发倒地起身回调函数
     * @param req 服务请求
     * @param res 服务响应
     * @return 是否成功
     */
    bool triggerFallStandUpCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  

  protected:
    /**
     * @brief 推理过程（重写基类方法）
     * @param observation 观测数据
     * @param action 输出的动作（输出参数）
     * @return 是否推理成功
     */
    bool inference(const Eigen::VectorXd& observation, Eigen::VectorXd& action) override;

    /**
     * @brief 更新观测数据（重写基类方法）
     * @param state_est 状态估计
     * @param sensor_data 传感器数据
     */
    void updateObservation(const Eigen::VectorXd& state_est, const SensorData& sensor_data) override;

    /**
     * @brief 将动作转换为关节命令（重写基类方法）
     * @param actuation 执行器输出（actuation）
     * @param measuredRbdState 测量的刚体状态
     * @param joint_cmd 关节命令（输出参数）
     */
    void actionToJointCmd(const Eigen::VectorXd& actuation, 
                          const Eigen::VectorXd& measuredRbdState,
                          kuavo_msgs::jointCmd& joint_cmd) override;

  private:
    // 内部状态管理
    enum class FallStandState
    {
      FALL_DOWN = 0,          ///< 倒地状态
      READY_FOR_STAND_UP,     ///< 准备起身
      STAND_UP,               ///< 执行起身
      STANDING                ///< 站立状态
    };

    /**
     * @brief 倒地起身模型类型
     */
    enum class FallStandModelType
    {
      PRONE = 0,              ///< 趴着倒地起身模型
      SUPINE                  ///< 躺着倒地起身模型
    };


    /**
     * @brief 加载轨迹数据（loadMotionTrajectory的别名）
     * @param trajectory_file 轨迹文件路径
     * @return 是否加载成功
     */
    bool loadTrajectory(const std::string& trajectory_file);

    /**
     * @brief 启动倒地起身插值
     * @param time 当前时间
     * @param sensor_data 传感器数据
     */
    void startFallStandInterpolation(const ros::Time& time, const SensorData& sensor_data);

    /**
     * @brief 更新倒地起身插值
     * @param time 当前时间
     * @param sensor_data 传感器数据
     * @param joint_cmd 输出的关节命令
     */
    void updateFallStandInterpolation(const ros::Time& time, 
                                      const SensorData& sensor_data,
                                      const Eigen::VectorXd& measuredRbdState,
                                      kuavo_msgs::jointCmd& joint_cmd);

    /**
     * @brief 加载运动轨迹（从CSV文件）
     * @param trajectoryFile 轨迹文件路径
     * @param trajectory 轨迹数据对象（输出参数）
     * @return 是否加载成功
     */
    bool loadMotionTrajectory(const std::string& trajectoryFile, MotionTrajectoryData& trajectory);

    /**
     * @brief 获取轨迹命令（使用当前轨迹）
     * @return 轨迹命令向量（位置+速度）
     */
    Eigen::VectorXd getTrajectoryCommand();
    
    /**
     * @brief 获取轨迹命令（指定轨迹）
     * @param trajectory 轨迹数据
     * @return 轨迹命令向量（位置+速度）
     */
    Eigen::VectorXd getTrajectoryCommand(const MotionTrajectoryData& trajectory);

    /**
     * @brief 获取轨迹锚点位置
     * @return 锚点位置
     */
    Eigen::Vector3d getTrajectoryAnchorPos();

    /**
     * @brief 获取轨迹锚点姿态
     * @return 锚点四元数
     */
    Eigen::Quaterniond getTrajectoryAnchorQuat();

    /**
     * @brief 获取运动锚点位置差（在body frame中）
     * @param currentBasePos 当前基座位置
     * @param currentBaseQuat 当前基座姿态
     * @return 位置差
     */
    Eigen::Vector3d getMotionAnchorPosB(const Eigen::Vector3d& currentBasePos, 
                                         const Eigen::Quaterniond& currentBaseQuat);

    /**
     * @brief 获取运动锚点姿态差（在body frame中）
     * @param currentBaseQuat 当前基座姿态
     * @return 姿态差（6维向量）
     */
    Eigen::VectorXd getMotionAnchorOriB(const Eigen::Quaterniond& currentBaseQuat);

    /**
     * @brief 检查是否应该执行推理（重写基类方法，添加fall_stand_state_检查）
     * @return 如果应该执行推理，返回true
     */
    bool shouldRunInference() const override;

    /**
     * @brief 切换使用的模型类型
     * @param model_type 要切换到的模型类型
     * @return 是否切换成功
     */
    bool switchModel(FallStandModelType model_type);

    /**
     * @brief 获取当前使用的模型类型
     * @return 当前模型类型
     */
    FallStandModelType getCurrentModelType() const { return current_model_type_; }

    /**
     * @brief 根据当前机体姿态自动判断并切换趴着/躺着模型
     * @return 是否成功切换模型（如果已经是正确的模型，也返回true）
     * 
     * 判断逻辑：
     * - 趴着（PRONE）：机体x轴指向地面（与重力方向同向），cos_angle > 0
     * - 仰着躺（SUPINE）：机体x轴指向天花板（与重力方向反向），cos_angle < 0
     */
    bool autoSelectAndSwitchModel();



    /**
     * @brief 更新RL命令（类似humanoidController::updateRLcmd）
     * @param state 状态
     * @return 执行器输出
     */
    Eigen::VectorXd updateRLcmd(const Eigen::VectorXd& state);

    // 成员变量
    ov::Core core_;                                 ///< OpenVINO核心对象
    ov::CompiledModel compiled_model_;              ///< 当前使用的编译后的模型（指向prone或supine）
    ov::InferRequest infer_request_;                ///< 当前使用的推理请求（指向prone或supine）
    
    // 两个模型的编译对象和推理请求
    ov::CompiledModel compiled_model_prone_;        ///< 趴着倒地起身模型
    ov::InferRequest infer_request_prone_;          ///< 趴着模型推理请求
    bool model_loaded_prone_ = false;               ///< 趴着模型是否已加载
    ov::CompiledModel compiled_model_supine_;       ///< 躺着倒地起身模型
    ov::InferRequest infer_request_supine_;         ///< 躺着模型推理请求
    bool model_loaded_supine_ = false;               ///< 躺着模型是否已加载
    
    FallStandState fall_stand_state_ = FallStandState::FALL_DOWN;  ///< 倒地起身状态
    FallStandModelType current_model_type_ = FallStandModelType::PRONE;  ///< 当前使用的模型类型
    MotionTrajectoryData motion_trajectory_;       ///< 当前使用的运动轨迹数据（指向prone或supine）
    MotionTrajectoryData motion_trajectory_prone_; ///< 趴着模型的运动轨迹数据
    MotionTrajectoryData motion_trajectory_supine_; ///< 躺着模型的运动轨迹数据
    
    // 配置参数
    std::string network_model_file_;                ///< 当前使用的网络模型文件路径（向后兼容）
    std::string trajectory_file_;                  ///< 当前使用的轨迹文件路径（向后兼容）
    std::string network_model_file_prone_;         ///< 趴着模型文件路径
    std::string trajectory_file_prone_;           ///< 趴着模型轨迹文件路径
    std::string network_model_file_supine_;        ///< 躺着模型文件路径
    std::string trajectory_file_supine_;           ///< 躺着模型轨迹文件路径
    int num_obs_ = 0;                              ///< 观测维度
    // num_actions_ 使用基类的成员变量（RLControllerBase::num_actions_）
    // 关节数量（jointNum_, jointArmNum_, waistNum_, headNum_）使用基类的成员变量
    
    // 状态相关
    bool trajectory_loaded_ = false;                ///< 轨迹是否已加载（当前使用的）
    bool trajectory_loaded_prone_ = false;         ///< 趴着模型轨迹是否已加载
    bool trajectory_loaded_supine_ = false;         ///< 躺着模型轨迹是否已加载
    bool request_for_stand_up_ = false;             ///< 是否请求起身
    
    // 动作和观测数据（与humanoidController命名一致）
    // actions_ 和 action_mtx_ 已移到基类
    int frameStackRL_ = 1;                          ///< 帧堆叠数量（多长时间步的obs）
    int numSingleObsRL_ = 0;                        ///< 单一时刻的obs的维度
    std::map<std::string, std::array<double, 3>> singleInputDataRLID_; ///< 存储singleInputData的id、numIdx、obsScale
    std::vector<std::string> singleInputDataRLKeys; ///< singleInputData的键列表
    std::deque<Eigen::VectorXd> input_deque;        ///< 用于多个历史的存储输入
    Eigen::VectorXd singleInputDataRL_;             ///< 单一时间的输入向量
    
    // 插值相关
    Eigen::VectorXd fall_stand_init_joints_;       ///< 倒地起身初始关节目标（当前使用的）
    Eigen::VectorXd fall_stand_init_joints_prone_; ///< 趴着模型初始关节目标
    Eigen::VectorXd fall_stand_init_joints_supine_; ///< 躺着模型初始关节目标
    Eigen::VectorXd fall_stand_start_pos_;         ///< 插值起始位置
    double fall_stand_max_joint_velocity_ = 1.0;   ///< 最大关节速度
    double fall_stand_required_time_ = 0.0;        ///< 插值所需时间
    double fall_stand_interp_start_time_ = 0.0;     ///< 插值开始时间
    bool is_fall_stand_interpolating_ = false;     ///< 是否正在插值
    bool is_fall_stand_interpolating_complete_ = false;  ///< 插值是否完成
    
    // RL控制相关参数（defalutJointPosRL_, JointControlModeRL_, JointPDModeRL_, 
    // jointKpRL_, jointKdRL_, torqueLimitsRL_, actionScaleTestRL_）使用基类的成员变量
    double actionScaleRL_ = 0.25;                  ///< 动作缩放
    double clipActionsRL_ = 1.0;                  ///< 动作裁剪限制
    double clipObservationsRL_ = 10.0;            ///< 观测裁剪限制
    bool is_real_ = false;                         ///< 是否真实机器人
    bool is_roban_ = false;                         ///< 是否是roban机器人
    bool residualAction_ = false;                  ///< 是否使用残差动作
    bool withArmRL_ = true;                        ///< 是否包含手臂
    
    // Yaw偏移
    double my_yaw_offset_ = 0.0;                   ///< Yaw角度偏移
    
    // AnkleSolver
    AnkleSolver ankleSolver_;                      ///< 脚踝解算器
    
    // ROS服务
    ros::ServiceServer trigger_fall_stand_up_srv_;  ///< 触发倒地起身服务
    ros::ServiceServer set_fall_down_state_srv_;    ///< 设置倒地状态服务
    
  };

} // namespace humanoid_controller







