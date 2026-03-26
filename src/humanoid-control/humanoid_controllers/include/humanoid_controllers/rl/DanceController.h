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
   * @brief 舞蹈轨迹数据结构
   * 用于存储和访问预定义的舞蹈运动轨迹（从CSV加载）
   */
  class DanceTrajectoryData
  {
  public:
    Eigen::MatrixXd joint_pos;      // (time_steps, num_joints) - joint positions
    Eigen::MatrixXd joint_vel;      // (time_steps, num_joints) - joint velocities
    Eigen::MatrixXd body_pos_w;     // (time_steps, 3) - body positions in world frame (base link)
    Eigen::MatrixXd body_quat_w;    // (time_steps, 4) - body quaternions in world frame (base link)
    int time_step_total;            // total number of time steps
    int current_time_step;          // current time step index
    int hold_frame_index;           // frame index to hold after trajectory finishes (-1 means last frame)
    double reference_yaw;           // reference yaw angle from first frame of trajectory
    
    DanceTrajectoryData() : time_step_total(0), current_time_step(0), hold_frame_index(-1), reference_yaw(0.0) {}
    
    /**
     * @brief 获取当前时间步的关节指令（位置+速度）
     * @return 关节指令向量 [positions; velocities]
     */
    Eigen::VectorXd getCurrentCommand() const {
      if (time_step_total == 0 || joint_pos.rows() == 0) {
        return Eigen::VectorXd::Zero(joint_pos.cols() * 2);
      }
      
      // 确定使用哪一帧
      int step;
      if (isFinish()) {
        // 轨迹完成后使用指定的保持帧
        if (hold_frame_index < 0) {
          // -1 表示使用最后一帧
          step = time_step_total - 1;
        } else {
          // 使用指定帧（确保在有效范围内）
          step = std::min(hold_frame_index, time_step_total - 1);
        }
      } else {
        // 轨迹执行中使用当前时间步
        step = std::min(current_time_step, time_step_total - 1);
      }
      Eigen::VectorXd command(joint_pos.cols() * 2);
      command.head(joint_pos.cols()) = joint_pos.row(step);
      command.tail(joint_vel.cols()) = joint_vel.row(step);
      std::cout << "Getting command at step " << step <<std::endl;
      return command;
    }
    
    /**
     * @brief 获取目标基座位置（从轨迹）
     * @return 3D位置向量
     */
    Eigen::Vector3d getTargetBasePos() const {
      if (time_step_total == 0) {
        return Eigen::Vector3d::Zero();
      }      
      int step = std::min(current_time_step, time_step_total - 1);
      return body_pos_w.row(step).transpose();
    }
    
    /**
     * @brief 获取目标基座姿态（从轨迹）
     * @return 四元数
     */
    Eigen::Quaterniond getTargetBaseQuat() const {      
      int step = std::min(current_time_step, time_step_total - 1);
      Eigen::Vector4d quat_vec = body_quat_w.row(step).transpose();
      // 假设四元数存储为 [w, x, y, z]
      return Eigen::Quaterniond(quat_vec(0), quat_vec(1), quat_vec(2), quat_vec(3));
    }
    
    /**
     * @brief 更新时间步（推进到下一步）
     */
    void updateTimeStep() {
      if (time_step_total > 0 && current_time_step < time_step_total - 1) {
        current_time_step++;
      }
    }
    
    /**
     * @brief 检查轨迹是否执行完成
     * @return true表示已完成
     */
    bool isFinish() const {
      return current_time_step >= time_step_total - 1;
    }
    
    /**
     * @brief 重置到轨迹起始点
     */
    void resetTimeStep() {
      current_time_step = 0;
    }
    
    /**
     * @brief 设置当前时间步
     * @param time_step 时间步索引
     */
    void setTimeStep(int time_step) {
      current_time_step = std::min(time_step, time_step_total - 1);
    }
    
    /**
     * @brief 获取当前时间步
     * @return 当前时间步索引
     */
    int getTimeStep() const {
      return current_time_step;
    }
    
    /**
     * @brief 获取轨迹总时间步数
     * @return 总时间步数
     */
    int getTimeStepTotal() const {
      return time_step_total;
    }
  };

  /**
   * @brief Dance控制器
   * 实现从CSV加载舞蹈轨迹并跟踪执行的逻辑
   */
  class DanceController : public RLControllerBase
  {
  public:
    /**
     * @brief 构造函数
     * @param name 控制器名称
     * @param config_file 配置文件路径
     * @param nh ROS节点句柄
     * @param ros_logger ROS日志发布器（可选，可以为nullptr）
     */
    DanceController(const std::string& name, const std::string& config_file,
                    ros::NodeHandle& nh,
                    ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    /**
     * @brief 析构函数
     */
    virtual ~DanceController();

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

    /**
     * @brief 恢复控制器
     */
    void resume() override;

    /**
     * @brief 暂停控制器
     */
    void pause() override;

    /**
     * @brief 检查控制器是否准备好退出
     * @return 如果舞蹈轨迹执行完成，返回true
     */
    bool isReadyToExit() const override;
    
    /**
     * @brief 检查是否应该执行推理（添加轨迹就绪检查）
     * @return 如果状态为RUNNING且轨迹已加载，返回true
     */
    bool shouldRunInference() const override;

    /**
     * @brief 检查控制器当前是否处于 stance（站立）模式
     * @return Dance控制器始终返回false（因为在执行动作）
     */
    bool isInStanceMode() const override;

  protected:
    /**
     * @brief 更新控制器实现
     * @param time 当前时间
     * @param sensor_data 传感器数据
     * @param measuredRbdState 测量的刚体状态
     * @param joint_cmd 输出的关节命令
     * @return 是否更新成功
     */
    bool updateImpl(const ros::Time& time,
                    const SensorData& sensor_data,
                    const Eigen::VectorXd& measuredRbdState,
                    kuavo_msgs::jointCmd& joint_cmd) override;

    /**
     * @brief 推理过程
     * @param observation 观测数据
     * @param action 输出的动作
     * @return 始终返回true
     */
    bool inference(const Eigen::VectorXd& observation,
                   Eigen::VectorXd& action) override;

    /**
     * @brief 更新观测数据
     * @param state_est 状态估计
     * @param sensor_data 传感器数据
     */
    void updateObservation(const Eigen::VectorXd& state_est,
                           const SensorData& sensor_data) override;

    /**
     * @brief 更新RL命令（与FallStandController架构对齐）
     * @param state 状态向量
     * @return 执行器命令向量
     */
    Eigen::VectorXd updateRLcmd(const Eigen::VectorXd& state);

    /**
     * @brief 将轨迹数据映射到关节命令
     * @param actuation 动作向量（从轨迹获取）
     * @param measuredRbdState 测量的刚体状态
     * @param joint_cmd 输出的关节命令
     */
    void actionToJointCmd(const Eigen::VectorXd& actuation,
                          const Eigen::VectorXd& measuredRbdState,
                          kuavo_msgs::jointCmd& joint_cmd) override;

    /**
     * @brief 传感器数据预处理
     * @param sensor_data 传感器数据（输入输出参数）
     */
    void preprocessSensorData(SensorData& sensor_data) override;

  private:
    /**
     * @brief 从CSV文件加载舞蹈轨迹
     * @param csv_file CSV文件路径
     * @return 是否加载成功
     */
    bool loadTrajectoryFromCSV(const std::string& csv_file);

    /**
     * @brief 初始化Dance控制器特定的服务接口（在基类服务之外）
     */
    void initializeDanceServices();

    /**
     * @brief 获取轨迹相关的观测（位置差、姿态差等）
     * @param currentBasePos 当前基座位置
     * @param currentBaseQuat 当前基座姿态
     * @return 观测向量
     */
    Eigen::VectorXd getTrajectory(const Eigen::Vector3d& currentBasePos,
                                const Eigen::Quaterniond& currentBaseQuat);

    /**
     * @brief 重新开始舞蹈服务回调
     */
    bool restartDanceCallback(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res);

    // ===== 控制参数 =====
    double dt_{0.002};                    // 控制周期（从/wbc_frequency获取）
    double actionScale_{0.25};             // 动作缩放因子
    double clipActions_{100.0};           // 动作裁剪限制
    
    // ===== 舞蹈轨迹数据 =====
    DanceTrajectoryData dance_trajectory_; // 舞蹈轨迹数据
    std::string trajectory_csv_file_;      // 轨迹CSV文件路径
    double trajectory_dt_{0.02};           // 轨迹时间步长（秒），默认50Hz
    double trajectory_time_accumulator_{0.0}; // 时间累加器，用于控制轨迹更新频率
    
    // ===== 神经网络推理 =====
    ov::Core core_;                        // OpenVINO核心
    ov::CompiledModel compiled_model_;     // 编译后的模型
    ov::InferRequest infer_request_;       // 推理请求
    std::string network_model_file_;       // 模型文件路径
    
    // ===== 观测和动作 =====
    int numSingleObsRL_{0};                // 单帧观测维度
    int frameStackRL_{1};                  // 帧堆叠数量
    Eigen::VectorXd singleInputDataRL_;    // 单帧输入数据
    Eigen::VectorXd networkInputDataRL_;   // 网络输入数据（包含帧堆叠）
    std::deque<Eigen::VectorXd> input_deque; // 输入历史队列（用于帧堆叠）
    
    std::map<std::string, Eigen::Vector3d> singleInputDataID_; // 观测数据索引映射
    std::vector<std::string> singleInputDataKeys_;              // 观测数据键列表
    bool residualAction_ = false;                  // 是否使用残差动作

    // ===== 踝关节求解器 =====
    AnkleSolver ankleSolver_;
    
    // ===== 机器人配置 =====
    bool is_real_{false};               // 默认false（与FallStandController一致）
    bool is_roban_{false};
    bool first_run_ = true;
    double my_yaw_offset_{0.0};              // Yaw角度偏移（与FallStandController一致）
    
    // ===== ROS服务 =====
    ros::ServiceServer restart_dance_srv_;  // 重新开始舞蹈服务
  };

} // namespace humanoid_controller
