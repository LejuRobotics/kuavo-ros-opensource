// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/DanceController.h"
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <cmath>

namespace humanoid_controller
{
  using namespace ocs2;

  DanceController::DanceController(const std::string& name, 
                                   const std::string& config_file,
                                   ros::NodeHandle& nh,
                                   ocs2::humanoid::TopicLogger* ros_logger)
    : RLControllerBase(name, RLControllerType::DANCE_CONTROLLER, config_file, nh, ros_logger)
  {
  }

  DanceController::~DanceController()
  {
    // 基类的stop()会自动停止推理线程
    stop();
  }

  bool DanceController::initialize()
  {
    std::cout << "[" << name_ << "] Initializing DanceController...\n";
    // 初始化Dance控制器特定的ROS服务（基类服务已在构造函数中初始化）
    initializeDanceServices();

    // 从ROS参数获取控制周期
    double wbc_frequency = 500.0;
    if (!nh_.getParam("/wbc_frequency", wbc_frequency))
    {
      ROS_WARN("[%s] /wbc_frequency not found in ROS params, using default: %.1f Hz", name_.c_str(), wbc_frequency);
    }
    if (wbc_frequency <= 0.0)
    {
      ROS_WARN("[%s] Invalid /wbc_frequency (%.3f), fallback to 500 Hz", name_.c_str(), wbc_frequency);
      wbc_frequency = 500.0;
    }
    dt_ = 1.0 / wbc_frequency;

    // 加载配置文件
    if (!loadConfig(config_file_))
    {
      ROS_ERROR("[%s] Failed to load config file: %s", name_.c_str(), config_file_.c_str());
      return false;
    }

    // 加载舞蹈轨迹
    if (!trajectory_csv_file_.empty())
    {
      std::filesystem::path configPath(config_file_);
      std::string trajectoryFilePath = (configPath.parent_path() / trajectory_csv_file_).string();
      
      if (!loadTrajectoryFromCSV(trajectoryFilePath))
      {
        ROS_ERROR("[%s] Failed to load dance trajectory file: %s", name_.c_str(), trajectoryFilePath.c_str());
        return false;
      }
      else
      {
        ROS_INFO("[%s] Dance trajectory loaded successfully: %s", name_.c_str(), trajectoryFilePath.c_str());
        ROS_INFO("[%s] Trajectory has %d time steps", name_.c_str(), dance_trajectory_.time_step_total);
      }
    }
    else
    {
      ROS_ERROR("[%s] No trajectory CSV file specified in config", name_.c_str());
      return false;
    }

    // 读取机器人类型参数
    if (!nh_.getParam("/is_real", is_real_))
    {
      ROS_WARN("[%s] /is_real not found in ROS params, using default: %d", name_.c_str(), static_cast<int>(is_real_));
    }
    if (!nh_.getParam("/is_roban", is_roban_))
    {
      ROS_WARN("[%s] /is_roban not found in ROS params, using default: %d", name_.c_str(), static_cast<int>(is_roban_));
    }
    // 初始化踝关节求解器
    int ankle_solver_type = 0;
    if (!nh_.getParam("/ankle_solver_type", ankle_solver_type))
    {
      ROS_WARN("[%s] ankle_solver_type not found in ROS params, using default: %d", name_.c_str(), ankle_solver_type);
    }
    else
    {
      ROS_INFO("[%s] AnkleSolver type loaded from ROS params: %d", name_.c_str(), ankle_solver_type);
    }
    ankleSolver_.getconfig(ankle_solver_type);
    ROS_INFO("[%s] AnkleSolver initialized with type: %d", name_.c_str(), ankle_solver_type);

    // 加载神经网络模型
    try
    {
      compiled_model_ = core_.compile_model(network_model_file_, "CPU");
      infer_request_ = compiled_model_.create_infer_request();
      ROS_INFO("[%s] Neural network model loaded successfully: %s", name_.c_str(), network_model_file_.c_str());
      
      // 打印模型输入输出维度信息
      const auto input_port = compiled_model_.input();
      const auto output_port = compiled_model_.output();
      const auto input_shape = input_port.get_shape();
      const auto output_shape = output_port.get_shape();
      
      ROS_INFO("[%s] Model input shape: [%ld, %ld]", name_.c_str(), 
               input_shape.size() > 0 ? input_shape[0] : 0,
               input_shape.size() > 1 ? input_shape[1] : 0);
      ROS_INFO("[%s] Model output shape: [%ld, %ld]", name_.c_str(),
               output_shape.size() > 0 ? output_shape[0] : 0,
               output_shape.size() > 1 ? output_shape[1] : 0);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[%s] Failed to load neural network model: %s", name_.c_str(), e.what());
      return false;
    }

    // 初始化观测和动作数据（与FallStandController一致）
    const int num_actions = jointNum_ + jointArmNum_ + waistNum_;
    singleInputDataRL_.resize(numSingleObsRL_);
    networkInputDataRL_.resize(numSingleObsRL_ * frameStackRL_);
    actions_.resize(num_actions);  // 初始化基类的actions_
    singleInputDataRL_.setZero();
    networkInputDataRL_.setZero();
    actions_.setZero();  // 初始化为零，防止第一次获取action时size为0
    
    ROS_INFO("[%s] Observation initialized: numSingleObs=%d, frameStack=%d, total input size=%d",
             name_.c_str(), numSingleObsRL_, frameStackRL_, numSingleObsRL_ * frameStackRL_);
    ROS_INFO("[%s] Actions initialized: num_actions=%d", name_.c_str(), num_actions);
    
    // 初始化input_deque（帧堆叠）
    for (int i = 0; i < frameStackRL_; i++)
    {
      input_deque.push_back(singleInputDataRL_);
    }

    initialized_ = true;
    // 初始化时自动启动推理线程
    start();
    ROS_INFO("[%s] DanceController initialized and started successfully (state: RUNNING)", name_.c_str());
    return true;
  }

  bool DanceController::loadConfig(const std::string& config_file)
  {
    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(config_file, pt);

    auto loadEigenMatrix = [&](const std::string &key, auto &matrix)
    {
      loadData::loadEigenMatrix(config_file, key, matrix);
    };

    // 加载基础关节配置
    loadEigenMatrix("defaultJointState", defalutJointPosRL_);
    loadEigenMatrix("defaultBaseState", defaultBaseStateRL_);
    loadEigenMatrix("JointControlMode", JointControlModeRL_);
    loadEigenMatrix("JointPDMode", JointPDModeRL_);
    loadEigenMatrix("jointKp", jointKpRL_);
    loadEigenMatrix("jointKd", jointKdRL_);
    loadEigenMatrix("torqueLimits", torqueLimitsRL_);
    loadEigenMatrix("actionScaleTest", actionScaleTestRL_);

    // 设置初始状态
    initialStateRL_.resize(12 + defalutJointPosRL_.size());
    initialStateRL_ << defaultBaseStateRL_, defalutJointPosRL_;
    // 加载Dance特定参数
    loadData::loadCppDataType(config_file, "defaultBaseHeightControl", defaultBaseHeightControl_);
    loadData::loadCppDataType(config_file, "actionScale", actionScale_);
    loadData::loadCppDataType(config_file, "clipActions", clipActions_);
    
    // 加载轨迹CSV文件路径（相对于配置文件目录）
    loadData::loadCppDataType(config_file, "trajectoryCSVFile", trajectory_csv_file_);
    
    // 加载轨迹时间步长（秒），默认0.02s (50Hz)
    trajectory_dt_ = 0.02;
    loadData::loadCppDataType(config_file, "trajectoryTimeStep", trajectory_dt_);
    trajectory_time_accumulator_ = 0.0;
    
    // 加载轨迹完成后保持的帧索引（默认-1表示最后一帧）
    int hold_frame_idx = -1;
    loadData::loadCppDataType(config_file, "holdFrameIndex", hold_frame_idx);
    dance_trajectory_.hold_frame_index = hold_frame_idx;
    
    // 加载神经网络模型路径
    std::string networkModelFile,network_model_root_path;
    loadData::loadCppDataType(config_file, "networkModelFile", networkModelFile);
    nh_.getParam("/network_model_file", network_model_root_path);
    network_model_file_ = network_model_root_path + networkModelFile;
    
    // 加载观测空间配置
    loadData::loadCppDataType(config_file, "frameStack", frameStackRL_);
    loadData::loadCppDataType(config_file, "numSingleObs", numSingleObsRL_);
    
    // 加载推理频率并设置基类的 inference_frequency_（与 FallStandController 保持一致）
    double inference_freq = 50.0;  // 默认值
    loadData::loadCppDataType(config_file, "inferenceFrequency", inference_freq);
    inference_frequency_ = inference_freq;
    ROS_INFO("[%s] Inference frequency set to: %.1f Hz", name_.c_str(), inference_frequency_);
    
    // 加载单输入数据配置（与humanoidController::loadRLSettings一致）
    const std::string prefixSingleInputData_ = "singleInputData";
    double startIdx_ = 0, mumIdx_ = 0, obsScale_ = 0;
    int num_ = 0;
    singleInputDataKeys_.clear();
    singleInputDataID_.clear();
    
    for (const auto &pair : pt)
    {
    if (pair.first == prefixSingleInputData_)
    {
        for (const auto &pair2 : pair.second)
        {
        singleInputDataKeys_.push_back(pair2.first);
        loadData::loadPtreeValue(pt, startIdx_, prefixSingleInputData_ + "." + pair2.first + ".startIdx", false);
        loadData::loadPtreeValue(pt, mumIdx_, prefixSingleInputData_ + "." + pair2.first + ".numIdx", false);
        loadData::loadPtreeValue(pt, obsScale_, prefixSingleInputData_ + "." + pair2.first + ".obsScales", false);
        num_ += static_cast<int>(mumIdx_);
        singleInputDataID_[pair2.first] = {startIdx_, mumIdx_, obsScale_};
        }
    }
    }

    // 加载residualAction参数（与FallStandController一致）
    loadData::loadCppDataType(config_file, "residualAction", residualAction_);
    
    // 加载 RL 相关 IMU 滤波器参数（与 FallStandController 保持一致）
    // motor_pdo_kp/kd 由 dance_param.info 配置；use_default_motor_csp_kpkd 为 false 时由 RLControllerBase::update 下发 CSP
    loadRLFilterParams(config_file);

    ROS_INFO("[%s] Configuration loaded successfully", name_.c_str());
    ROS_INFO("[%s]   - Residual action mode: %s", name_.c_str(), residualAction_ ? "enabled" : "disabled");
    ROS_INFO("[%s]   - Trajectory CSV file: %s", name_.c_str(), trajectory_csv_file_.c_str());
    ROS_INFO("[%s]   - Trajectory time step: %.4f s (%.1f Hz)", name_.c_str(), trajectory_dt_, 1.0/trajectory_dt_);
    ROS_INFO("[%s]   - Hold frame index: %d (after trajectory completion)", name_.c_str(), dance_trajectory_.hold_frame_index);
    ROS_INFO("[%s]   - Network model file: %s", name_.c_str(), network_model_file_.c_str());
    ROS_INFO("[%s]   - Observation dimension: %d (frame stack: %d)", name_.c_str(), numSingleObsRL_, frameStackRL_);

    return true;
  }

  bool DanceController::loadTrajectoryFromCSV(const std::string& csv_file)
  {
    std::ifstream file(csv_file);
    if (!file.is_open())
    {
      ROS_ERROR("[%s] Failed to open trajectory CSV file: %s", name_.c_str(), csv_file.c_str());
      return false;
    }

    std::vector<std::vector<double>> csvData;
    std::string line;
    bool firstLine = true;
    int lineNum = 0;
    const int numJoints = jointNum_ + jointArmNum_ + waistNum_;
    // CSV格式支持两种：
    // 格式1: [joint_pos(numJoints), joint_vel(numJoints)] - 42列
    // 格式2: [body_pos(3), body_quat(4), joint_pos(numJoints), joint_vel(numJoints)] - 49列
    const int expectedColsWithBody = 3 + 4 + numJoints * 2;  // 49列
    const int expectedColsWithoutBody = numJoints * 2;       // 42列
    const int expectedColsWithoutpose = 4 + numJoints * 2;       // 54列

    
    // 自动检测分隔符（读取第一行非空行来判断）
    char delimiter = ',';  // 默认使用逗号
    std::string first_data_line;
    std::streampos file_start = file.tellg();
    while (std::getline(file, first_data_line))
    {
      if (!first_data_line.empty())
      {
        // 统计逗号、tab和空格的数量
        size_t comma_count = std::count(first_data_line.begin(), first_data_line.end(), ',');
        size_t tab_count = std::count(first_data_line.begin(), first_data_line.end(), '\t');
        size_t space_count = std::count(first_data_line.begin(), first_data_line.end(), ' ');
        
        // 选择数量最多的分隔符
        if (tab_count >= comma_count && tab_count >= space_count)
          delimiter = '\t';
        else if (space_count >= comma_count && space_count >= tab_count)
          delimiter = ' ';
        else
          delimiter = ',';
          
        ROS_INFO("[%s] Auto-detected CSV delimiter: '%s' (commas=%zu, tabs=%zu, spaces=%zu)", 
                 name_.c_str(), delimiter == '\t' ? "tab" : (delimiter == ' ' ? "space" : ","), 
                 comma_count, tab_count, space_count);
        break;
      }
    }
    file.clear();
    file.seekg(file_start);  // 重置文件指针到开头

    while (std::getline(file, line))
    {
      lineNum++;
      if (line.empty())
        continue;

      std::vector<double> row;
      std::stringstream ss(line);
      std::string value;
      bool parseError = false;

      // 使用检测到的分隔符解析
      if (delimiter == ' ')
      {
        // 空格分隔符特殊处理（跳过连续空格）
        while (ss >> value)
        {
          try
          {
            row.push_back(std::stod(value));
          }
          catch (const std::exception &e)
          {
            parseError = true;
            break;
          }
        }
      }
      else
      {
        // 逗号或tab分隔符
        while (std::getline(ss, value, delimiter))
        {
          // 去除空白字符
          value.erase(0, value.find_first_not_of(" \t\r\n"));
          value.erase(value.find_last_not_of(" \t\r\n") + 1);
          
          if (value.empty())
            continue;
          try
          {
            row.push_back(std::stod(value));
          }
          catch (const std::exception &e)
          {
            // 解析失败，可能是表头
            parseError = true;
            break;
          }
        }
      }

      // 如果是第一行且解析失败，认为是表头，跳过
      if (firstLine && parseError)
      {
        ROS_INFO("[%s] Detected header line, skipping: %s...", name_.c_str(), 
                 line.substr(0, std::min(80, (int)line.size())).c_str());
        firstLine = false;
        continue;
      }
      firstLine = false;

      if (row.size() == expectedColsWithBody || row.size() == expectedColsWithoutBody || row.size() == expectedColsWithoutpose)
      {
        csvData.push_back(row);
      }
      else if (row.size() > 0 && !parseError)
      {
        ROS_WARN("[%s] Line %d has %zu columns, expected %d or %d or %d", 
                 name_.c_str(), lineNum, row.size(), expectedColsWithBody, expectedColsWithoutBody, expectedColsWithoutpose);
      }
    }
    file.close();

    int timeSteps = csvData.size();
    if (timeSteps == 0)
    {
      ROS_ERROR("[%s] No valid data found in CSV file", name_.c_str());
      return false;
    }

    // 分配轨迹数据矩阵
    dance_trajectory_.time_step_total = timeSteps;
    dance_trajectory_.current_time_step = 0;
    dance_trajectory_.body_pos_w.resize(timeSteps, 3);
    dance_trajectory_.body_quat_w.resize(timeSteps, 4);
    dance_trajectory_.joint_pos.resize(timeSteps, numJoints);
    dance_trajectory_.joint_vel.resize(timeSteps, numJoints);

    // 解析CSV数据到轨迹矩阵
    for (int i = 0; i < timeSteps; ++i)
    {
      const auto &row = csvData[i];

      // 基座位置 (x, y, z)
      dance_trajectory_.body_pos_w(i, 0) = row[0];
      dance_trajectory_.body_pos_w(i, 1) = row[1];
      dance_trajectory_.body_pos_w(i, 2) = row[2];

      // 基座姿态 (qw, qx, qy, qz)
      dance_trajectory_.body_quat_w(i, 0) = row[3]; // w
      dance_trajectory_.body_quat_w(i, 1) = row[4]; // x
      dance_trajectory_.body_quat_w(i, 2) = row[5]; // y
      dance_trajectory_.body_quat_w(i, 3) = row[6]; // z
      if(row.size() == expectedColsWithoutpose)
      {
        // 如果没有基座位置数据，假设位置为0
        dance_trajectory_.body_pos_w(i, 0) = 0.0;
        dance_trajectory_.body_pos_w(i, 1) = 0.0;
        dance_trajectory_.body_pos_w(i, 2) = 0.0;
        dance_trajectory_.body_quat_w(i, 0) = row[0]; // w
        dance_trajectory_.body_quat_w(i, 1) = row[1]; // x
        dance_trajectory_.body_quat_w(i, 2) = row[2]; // y
        dance_trajectory_.body_quat_w(i, 3) = row[3]; // z
      }

      // 从CSV最后一列往前读取：
      // 最后27列 = 关节速度
      // 倒数28-54列 = 关节位置
      int totalCols = row.size();
      
      // 关节位置 - 从倒数第(2*numJoints)列开始读取
      for (int j = 0; j < numJoints; ++j)
      {
        dance_trajectory_.joint_pos(i, j) = row[totalCols - 2*numJoints + j];
      }

      // 关节速度 - 从最后numJoints列读取（CSV最后27列）
      for (int j = 0; j < numJoints; ++j)
      {
        dance_trajectory_.joint_vel(i, j) = row[totalCols - numJoints + j];
      }
    }

    // 计算参考yaw角（从第一帧的姿态）
    if (timeSteps > 0)
    {
      Eigen::Quaterniond first_quat(
        dance_trajectory_.body_quat_w(0, 0), // w
        dance_trajectory_.body_quat_w(0, 1), // x
        dance_trajectory_.body_quat_w(0, 2), // y
        dance_trajectory_.body_quat_w(0, 3)  // z
      );

      Eigen::Matrix3d ref_mat = first_quat.toRotationMatrix();
      dance_trajectory_.reference_yaw = std::atan2(ref_mat(1, 0), ref_mat(0, 0));
      
      // 打印第一帧全部 轨迹信息用于调试
      ROS_INFO("[%s] First frame body position (world): [%.3f, %.3f, %.3f] m", 
               name_.c_str(), 
               dance_trajectory_.body_pos_w(0, 0), 
               dance_trajectory_.body_pos_w(0, 1), 
               dance_trajectory_.body_pos_w(0, 2));
      ROS_INFO("[%s] First frame body orientation (world): [qw=%.3f, qx=%.3f, qy=%.3f, qz=%.3f]", 
               name_.c_str(), 
               dance_trajectory_.body_quat_w(0, 0), 
               dance_trajectory_.body_quat_w(0, 1), 
               dance_trajectory_.body_quat_w(0, 2), 
               dance_trajectory_.body_quat_w(0, 3));
      ROS_INFO("[%s] First frame joint positions: ", name_.c_str());
      std::string joint_pos_str;
      for (int j = 0; j < numJoints; ++j)
      {
        joint_pos_str += std::to_string(dance_trajectory_.joint_pos(0, j)) + (j < numJoints - 1 ? ", " : "");
      }
      ROS_INFO("[%s]   %s", name_.c_str(), joint_pos_str.c_str());
      ROS_INFO("[%s] First frame joint velocities: ", name_.c_str());
      std::string joint_vel_str;
      for (int j = 0; j < numJoints; ++j)
      {
        joint_vel_str += std::to_string(dance_trajectory_.joint_vel(0, j)) + (j < numJoints - 1 ? ", " : "");
      }
      ROS_INFO("[%s]   %s", name_.c_str(), joint_vel_str.c_str());



      ROS_INFO("[%s] Reference yaw from first frame: %.3f rad", 
               name_.c_str(), dance_trajectory_.reference_yaw);
    }

    return true;
  }

  void DanceController::initializeDanceServices()
  {
    // 重新开始舞蹈服务
    std::string service_name = "/humanoid_controller/" + name_ + "/restart_dance";
    restart_dance_srv_ = nh_.advertiseService(service_name, 
                                               &DanceController::restartDanceCallback, this);
    ROS_INFO("[%s] Service registered: %s", name_.c_str(), service_name.c_str());
  }

  bool DanceController::restartDanceCallback(std_srvs::Trigger::Request& req,
                                             std_srvs::Trigger::Response& res)
  {
    dance_trajectory_.resetTimeStep();
    res.success = true;
    res.message = "Dance trajectory reset to beginning";
    ROS_INFO("[%s] Dance trajectory restarted", name_.c_str());
    return true;
  }

  void DanceController::reset()
  {
    dance_trajectory_.resetTimeStep();
    trajectory_time_accumulator_ = 0.0;
    // 注意：reset() 仅在控制器暂停时被调用，不应改变状态
    // state_ 由 resume() 控制
    ROS_INFO("[%s] Controller reset", name_.c_str());
  }

  void DanceController::pause()
  {
    RLControllerBase::pause();
  }

  void DanceController::resume()
  {
    RLControllerBase::resume();    
    // 计算yaw偏移（与FallStandController一致）
    dance_trajectory_.resetTimeStep();
    trajectory_time_accumulator_ = 0.0;
    actions_.setZero();
    first_run_ = true;

    ROS_INFO("[%s] Controller resumed, waiting for first update to set yaw offset", name_.c_str());
  }

  bool DanceController::isReadyToExit() const
  {
    // DanceController不会自动退出，舞蹈完成后会保持指定帧
    if (dance_trajectory_.isFinish() && (dance_trajectory_.hold_frame_index == -2))
    {
        return true;
    }
    // 需要手动切换到其他控制器
    return false;
  }
  
  bool DanceController::shouldRunInference() const
  {
    bool base_check = RLControllerBase::shouldRunInference();
    bool trajectory_ready = (dance_trajectory_.time_step_total > 0);
    
    return base_check && trajectory_ready;
  }

  bool DanceController::isInStanceMode() const
  {
    // Dance控制器在执行动作时不在stance模式
    return false;
  }

  bool DanceController::updateImpl(const ros::Time& time,
                                    const SensorData& sensor_data,
                                    const Eigen::VectorXd& measuredRbdState,
                                    kuavo_msgs::jointCmd& joint_cmd)
  {
    // 状态检查：只在RUNNING状态下执行主逻辑
    if (state_ != ControllerState::RUNNING)
    {
      ROS_WARN_THROTTLE(1.0, "[%s] Controller not in RUNNING state (current: %d), skipping update",
                        name_.c_str(), static_cast<int>(state_));
      return false;
    }

    if (first_run_)
    {
        first_run_ = false;
        auto mat = sensor_data.quat_.toRotationMatrix();
        double current_yaw = std::atan2(mat(1, 0), mat(0, 0));
        my_yaw_offset_ = current_yaw - dance_trajectory_.reference_yaw;
        while (my_yaw_offset_ > M_PI) my_yaw_offset_ -= 2 * M_PI;
        while (my_yaw_offset_ < -M_PI) my_yaw_offset_ += 2 * M_PI;
        ROS_INFO("[%s] First update: yaw_offset=%.3f rad (current=%.3f, ref=%.3f)", 
             name_.c_str(), my_yaw_offset_, current_yaw, dance_trajectory_.reference_yaw);
    }

    // 调用updateRLcmd获取执行器命令
    Eigen::VectorXd actuation = updateRLcmd(measuredRbdState);
    
    // 将动作映射到关节命令（与FallStandController一致）
    actionToJointCmd(actuation, measuredRbdState, joint_cmd);
    joint_cmd.header.stamp = time;

    //在time时候下发的关节命令
    // std::cout << "[" << name_ << "] Joint command prepared at time: " << joint_cmd.header.stamp.toSec() << " s\n";
    
    // 累积时间，只有达到轨迹时间步长时才更新到下一帧
    // 这样可以控制轨迹执行速度与CSV采样率匹配
    // 添加安全检查：确保轨迹已初始化
    if (dance_trajectory_.time_step_total > 0)
    {
      // 只有在轨迹未完成时才更新时间步
      if (!dance_trajectory_.isFinish())
      {
        trajectory_time_accumulator_ += dt_;
        if (trajectory_time_accumulator_ >= trajectory_dt_)
        {
          dance_trajectory_.updateTimeStep();
          trajectory_time_accumulator_ -= trajectory_dt_;  // 保留余数，避免累积误差
        }
      }
      else
      {
        // 轨迹完成后，保持在指定帧，直到手动切换控制器
        int hold_frame = dance_trajectory_.hold_frame_index < 0 ? 
                         dance_trajectory_.getTimeStepTotal() - 1 : 
                         std::min(dance_trajectory_.hold_frame_index, dance_trajectory_.getTimeStepTotal() - 1);
        ROS_INFO_THROTTLE(5.0, "[%s] Dance trajectory completed, holding frame %d (%d/%d steps)", 
                         name_.c_str(), 
                         hold_frame,
                         dance_trajectory_.getTimeStep(),
                         dance_trajectory_.getTimeStepTotal());
      }
    }
    else
    {
      ROS_WARN_THROTTLE(5.0, "[%s] Dance trajectory not initialized, skipping time update", name_.c_str());
    }

    return true;
  }

  Eigen::VectorXd DanceController::updateRLcmd(const Eigen::VectorXd& state)
  {
    // 获取传感器数据（与FallStandController一致）
    SensorData sensor_data = getRobotSensorData();
    Eigen::VectorXd jointPos_ = sensor_data.jointPos_;
    Eigen::VectorXd jointVel_ = sensor_data.jointVel_;
    Eigen::VectorXd jointKpLocal = jointKpRL_;
    Eigen::VectorXd jointKdLocal = jointKdRL_;
    Eigen::VectorXd torqueLimitsLocal = torqueLimitsRL_;

    Eigen::VectorXd actuation(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd cmd(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd torque(jointNum_ + jointArmNum_ + waistNum_);
    
    Eigen::VectorXd motorPos_ = jointPos_;
    Eigen::VectorXd motorVel_ = jointVel_;
    // 使用基类的线程安全方法获取当前动作
    Eigen::VectorXd local_action = getCurrentAction();
    
    /*****************************************舞蹈轨迹跟踪*****************************************************************/ 
    Eigen::VectorXd temp;
    if (residualAction_ == true) {
      temp = defalutJointPosRL_;
      defalutJointPosRL_ = dance_trajectory_.getCurrentCommand().head(jointNum_ + jointArmNum_ + waistNum_);
    }    
    /*****************************************舞蹈轨迹跟踪*****************************************************************/
    
    // 安全检查：确保动作向量大小正确
    int expected_size = jointNum_ + jointArmNum_ + waistNum_;
    if (local_action.size() != expected_size)
    {
      ROS_WARN_THROTTLE(1.0, "[%s] Action size mismatch: %ld vs expected %d, returning zero vector",
                        name_.c_str(), local_action.size(), expected_size);
      return Eigen::VectorXd::Zero(expected_size);
    }
    
    // 计算关节扭矩
    Eigen::VectorXd jointTor_(jointNum_ + jointArmNum_ + waistNum_);
    if(is_roban_) {
      motorPos_.segment(waistNum_, jointNum_) = ankleSolver_.joint_to_motor_position(jointPos_.segment(waistNum_, jointNum_));
      motorVel_.segment(waistNum_, jointNum_) = ankleSolver_.joint_to_motor_velocity(jointPos_.segment(waistNum_, jointNum_), motorPos_.segment(waistNum_, jointNum_), jointVel_.segment(waistNum_, jointNum_));
      jointTor_ = -(jointKdLocal.cwiseProduct(motorVel_));
      jointTor_.segment(waistNum_, jointNum_) = ankleSolver_.motor_to_joint_torque(jointPos_.segment(waistNum_, jointNum_), motorPos_.segment(waistNum_, jointNum_), jointTor_.segment(waistNum_, jointNum_));
    } else {
      motorPos_.head(jointNum_) = ankleSolver_.joint_to_motor_position(jointPos_.head(jointNum_));
      motorVel_.head(jointNum_) = ankleSolver_.joint_to_motor_velocity(jointPos_.head(jointNum_), motorPos_.head(jointNum_), jointVel_.head(jointNum_));
      jointTor_ = -(jointKdLocal.cwiseProduct(motorVel_));
      jointTor_.head(jointNum_) = ankleSolver_.motor_to_joint_torque(jointPos_.head(jointNum_), motorPos_.head(jointNum_), jointTor_.head(jointNum_));
    }
    
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
    {
      jointTor_(i) = jointTor_(i) + jointKpLocal(i) * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]);
    }
    
    if (is_real_)
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
      {
        if (i < JointControlModeRL_.size() && JointControlModeRL_(i) == 0)
        {
          if (i < JointPDModeRL_.size() && JointPDModeRL_(i) == 0)
          {
            cmd[i] = jointKpLocal[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]) - jointKdLocal[i] * jointVel_[i];
            cmd[i] = std::clamp(cmd[i], -torqueLimitsLocal[i], torqueLimitsLocal[i]);
            torque[i] = cmd[i];
          }
          else
          {
            cmd[i] = (local_action[i] * actionScale_ * actionScaleTestRL_[i] + defalutJointPosRL_[i]);
            torque[i] = jointKpLocal[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]) - jointKdLocal[i] * jointVel_[i];
          }
        }
        else if (i < JointControlModeRL_.size() && JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointKpLocal[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]);
          torque[i] = jointTor_[i];
        }
        else
        {
          cmd[i] = 0.0;
          torque[i] = 0.0;
        }
      }
    }
    else
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
      {
        if (i < JointControlModeRL_.size() && JointControlModeRL_(i) == 0)
        {
          cmd[i] = jointKpLocal[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]) - jointKdLocal[i] * jointVel_[i];
        }
        else if (i < JointControlModeRL_.size() && JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointTor_[i];
        }
        else
        {
          cmd[i] = 0.0;
        }
        cmd[i] = std::clamp(cmd[i], -torqueLimitsLocal[i], torqueLimitsLocal[i]);
        torque[i] = cmd[i];
      }
    }

    /*****************************************舞蹈轨迹跟踪*****************************************************************/ 
    if (residualAction_ == true) {
      defalutJointPosRL_ = temp;
    }    
    /*****************************************舞蹈轨迹跟踪*****************************************************************/

    actuation = cmd;

    return actuation;
  }

  bool DanceController::inference(const Eigen::VectorXd& observation,
                                   Eigen::VectorXd& action)
  {
    ROS_INFO_THROTTLE(2.0, "[%s] inference() called, obs_size=%ld, networkInputDataRL_size=%ld", 
                     name_.c_str(), observation.size(), networkInputDataRL_.size());
    
    // 注意：参数observation保留用于接口兼容，但实际使用networkInputDataRL_（与FallStandController一致）
    const int expected_output_length = jointNum_ + jointArmNum_ + waistNum_;
    
    try
    {
      infer_request_ = compiled_model_.create_infer_request();
      
      // std::cout << "[" << name_ << "] Starting inference...\n";
      const auto input_port = compiled_model_.input();
      
      const auto expected_input_shape = input_port.get_shape();
      const size_t expected_input_length = expected_input_shape[1]; // 假设形状为 [batch_size, input_dim]
      const size_t actual_input_length = networkInputDataRL_.size();
      
      if (actual_input_length != expected_input_length)
      {
        ROS_ERROR_THROTTLE(1.0, "[%s] networkInputDataRL_ size mismatch: actual=%ld vs expected=%ld", 
                           name_.c_str(), actual_input_length, expected_input_length);
        action = Eigen::VectorXd::Zero(expected_output_length);
        return false;
      }

      // 准备输入数据
      Eigen::VectorXf float_network_input = networkInputDataRL_.cast<float>();
      ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), float_network_input.data());
      infer_request_.set_input_tensor(input_tensor);
      //   std::cout << "[" << name_ << "] Input tensor set, starting inference...\n";
      // 打输入张tensor信息用于调试
    //   std::cout << "[" << name_ << "] Inference input tensor data (first 10 values): ";
    //   for (size_t i = 0; i < std::min(size_t(10), actual_input_length); ++i)
    //   {
    //     std::cout << float_network_input[i] << (i < std::min(size_t(10), actual_input_length) - 1 ? ", " : "\n");
    //   }

      // 执行推理
      infer_request_.start_async();
      infer_request_.wait();
      
      // 获取输出
      const auto output_tensor = infer_request_.get_output_tensor();
      const size_t output_buf_length = output_tensor.get_size();
      // 注意：data<float>() 在 OpenVINO 2026.0 中将返回 const T*，这里使用 const_cast 消除废弃警告
      // 由于我们只是读取数据，使用 const float* 是安全的
      const float* output_buf = const_cast<const float*>(output_tensor.data<float>());
      
      // 检查输出维度是否匹配
      if (static_cast<int>(output_buf_length) != expected_output_length)
      {
        ROS_ERROR_THROTTLE(1.0, "[%s] Output size mismatch: actual=%ld vs expected=%d", 
                           name_.c_str(), output_buf_length, expected_output_length);
        action = Eigen::VectorXd::Zero(expected_output_length);
        return false;
      }
      
      // 复制输出到action参数
      action.resize(output_buf_length);
      for (size_t i = 0; i < output_buf_length; ++i)
      {
        action[i] = output_buf[i];
      }
      
      // 使用基类的clip函数裁剪动作（与FallStandController一致）
      clip(action, clipActions_);
      
      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] Inference failed: %s", name_.c_str(), e.what());
      action = Eigen::VectorXd::Zero(expected_output_length);
      return false;
    }
  }

  void DanceController::updateObservation(const Eigen::VectorXd& state_est,
                                          const SensorData& sensor_data)
  {
    ROS_INFO_THROTTLE(2.0, "[%s] updateObservation() called (inference thread)", name_.c_str());
    
    const int numJoints = jointNum_ + jointArmNum_ + waistNum_;
    
    // 提取状态数据
    const Eigen::Vector3d baseLineVel = state_est.segment(9 + jointNum_ + waistNum_ + jointArmNum_, 3);
    
    // 提取和处理传感器数据
    Eigen::VectorXd currentJointPos = sensor_data.jointPos_ - defalutJointPosRL_;
    Eigen::VectorXd currentJointVel = sensor_data.jointVel_;
    const Eigen::Vector3d bodyAngVel = sensor_data.angularVel_;
    
    // 变换基座线速度
    auto quat_offset_ = Eigen::AngleAxisd(-my_yaw_offset_, Eigen::Vector3d::UnitZ()) * sensor_data.quat_;
    const Eigen::Matrix3d R = quat_offset_.matrix();
    
    Eigen::Quaterniond currentBaseQuat = Eigen::Quaterniond(quat_offset_.w(),
                                                           quat_offset_.x(),
                                                           quat_offset_.y(),
                                                           quat_offset_.z());
    
    // 计算 projected_gravity
    const Eigen::Vector3d gravity_world(0, 0, -1);
    const Eigen::Vector3d projected_gravity = R.transpose() * gravity_world;
    
    // 获取轨迹目标
    Eigen::VectorXd trajectory_cmd = dance_trajectory_.getCurrentCommand();
    if (trajectory_cmd.size() != numJoints * 2)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] trajectory_cmd size mismatch: %ld vs expected %d", 
                         name_.c_str(), trajectory_cmd.size(), numJoints * 2);
      trajectory_cmd = Eigen::VectorXd::Zero(numJoints * 2);
    }
    Eigen::VectorXd target_joint_pos = trajectory_cmd.head(numJoints);
    Eigen::VectorXd target_joint_vel = trajectory_cmd.tail(numJoints);
    // 注意：CSV轨迹数据来自RL训练环境，已经是RL策略的关节顺序 [waist(1), leg(12), arm(8)]    
    // 计算 motion_target_pos（与FallStandController一致：目标基座高度）
    Eigen::Vector3d rawTrajectoryPos = dance_trajectory_.getTargetBasePos();
    Eigen::VectorXd motion_target_pos(1);
    motion_target_pos << rawTrajectoryPos[2];
    
    // 计算 motion_anchor_ori_b（与FallStandController一致：姿态差）
    Eigen::Quaterniond targetBaseQuat = dance_trajectory_.getTargetBaseQuat();
    Eigen::Quaterniond currentQuatInv = currentBaseQuat.inverse();
    Eigen::Quaterniond quatDiff = currentQuatInv * targetBaseQuat;
    Eigen::Matrix3d rotMat = quatDiff.toRotationMatrix();
    Eigen::VectorXd motion_anchor_ori_b(6);
    motion_anchor_ori_b << rotMat(0, 0), rotMat(0, 1), 
                           rotMat(1, 0), rotMat(1, 1),
                           rotMat(2, 0), rotMat(2, 1);
    
    // 获取本地动作（使用基类的线程安全方法，如果没有上一次动作则使用零）
    Eigen::VectorXd local_action = getCurrentAction();
    if (local_action.size() != numJoints)
    {
      local_action = Eigen::VectorXd::Zero(numJoints);
    }
    
    // 构建motion_command：[target_joint_pos, target_joint_vel]
    Eigen::VectorXd motion_command(numJoints * 2);
    motion_command.head(numJoints) = target_joint_pos;
    motion_command.tail(numJoints) = target_joint_vel;
    
    // 构建观测数据映射（与FallStandController一致）
    const std::map<std::string, Eigen::VectorXd> singleInputDataMap_ = {
        {"motion_command", motion_command},
        {"motion_target_pos", motion_target_pos},
        {"motion_anchor_ori_b", motion_anchor_ori_b},
        {"projected_gravity", projected_gravity},
        {"base_ang_vel", bodyAngVel},
        {"joint_pos", currentJointPos},
        {"joint_vel", currentJointVel},
        {"actions", local_action}};

    // Fill singleInputData（与FallStandController一致）
    if (!singleInputDataKeys_.empty())
    {
      // 如果配置了singleInputData，使用配置的方式填充
      int index = 0;
      for (const auto &key : singleInputDataKeys_)
      {
        auto it = singleInputDataID_.find(key);
        if (it == singleInputDataID_.end())
        {
          ROS_ERROR_THROTTLE(1.0, "[%s] Key '%s' not found in singleInputDataID_", name_.c_str(), key.c_str());
          continue;
        }
        
        const auto &value = it->second;
        int srcStartIdx = static_cast<int>(value[0]);
        int numIdx = static_cast<int>(value[1]);
        double obsScale = value[2];
        
        auto mapIt = singleInputDataMap_.find(key);
        if (mapIt == singleInputDataMap_.end())
        {
          ROS_ERROR_THROTTLE(1.0, "[%s] Key '%s' not found in singleInputDataMap_", name_.c_str(), key.c_str());
          continue;
        }
        
        const Eigen::VectorXd& srcData = mapIt->second;
        
        // 边界检查
        if (srcStartIdx + numIdx > srcData.size())
        {
          ROS_ERROR_THROTTLE(1.0, "[%s] Key '%s': srcStartIdx(%d) + numIdx(%d) > srcData.size(%ld)", 
                             name_.c_str(), key.c_str(), srcStartIdx, numIdx, srcData.size());
          continue;
        }
        if (index + numIdx > singleInputDataRL_.size())
        {
          ROS_ERROR_THROTTLE(1.0, "[%s] Key '%s': index(%d) + numIdx(%d) > singleInputDataRL_.size(%ld)", 
                             name_.c_str(), key.c_str(), index, numIdx, singleInputDataRL_.size());
          continue;
        }
        
        singleInputDataRL_.segment(index, numIdx) = srcData.segment(srcStartIdx, numIdx) * obsScale;
        index += numIdx;
        
        if (ros_logger_)
        {
          ros_logger_->publishVector("/dance_controller/InputData/" + key, srcData.segment(srcStartIdx, numIdx) * obsScale);
        }
      }
    }
    else
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] singleInputDataKeys_ is empty, cannot build observation", name_.c_str());
      singleInputDataRL_.setZero();
    }

    // 更新帧堆叠（与FallStandController一致：先push_back再pop_front）
    input_deque.push_back(singleInputDataRL_);
    input_deque.pop_front();

    // 组装网络输入
    for (int i = 0; i < frameStackRL_; i++)
    {
      networkInputDataRL_.segment(i * numSingleObsRL_, numSingleObsRL_) = input_deque[i];
    }
    
    // 发布观测数据（如果ros_logger_可用）
    if (ros_logger_)
    {
      ros_logger_->publishVector("/dance_controller/singleInputData", singleInputDataRL_);
    }
  }

  void DanceController::actionToJointCmd(const Eigen::VectorXd& actuation,
                                          const Eigen::VectorXd& measuredRbdState,
                                          kuavo_msgs::jointCmd& joint_cmd)
  {
    // std::cout << "[" << name_ << "] Mapping actions to joint commands\n";
    int total_joints = jointNum_ + jointArmNum_ + waistNum_;

    if (actuation.size() != total_joints)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] actuation size mismatch: %ld vs expected %d", 
                         name_.c_str(), actuation.size(), total_joints);
      return;
    }

    if (jointKpRL_.size() < total_joints || jointKdRL_.size() < total_joints ||
        torqueLimitsRL_.size() < total_joints || JointControlModeRL_.size() < total_joints ||
        JointPDModeRL_.size() < total_joints)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] joint config size mismatch: kp=%ld kd=%ld torque=%ld mode=%ld pd=%ld expected=%d",
                         name_.c_str(), jointKpRL_.size(), jointKdRL_.size(), torqueLimitsRL_.size(),
                         JointControlModeRL_.size(), JointPDModeRL_.size(), total_joints);
      return;
    }

    joint_cmd.joint_q.clear();
    joint_cmd.joint_v.clear();
    joint_cmd.joint_kp.clear();
    joint_cmd.joint_kd.clear();
    joint_cmd.tau.clear();
    joint_cmd.tau_ratio.clear();
    joint_cmd.tau_max.clear();
    joint_cmd.control_modes.clear();

    if (!is_real_)
    {
      for (int i = 0; i < total_joints; ++i)
      {
        joint_cmd.joint_q.push_back(0.0);
        joint_cmd.joint_v.push_back(0.0);
        joint_cmd.joint_kp.push_back(jointKpRL_[i]);
        joint_cmd.joint_kd.push_back(jointKdRL_[i]);
        joint_cmd.tau.push_back(actuation(i));
        joint_cmd.tau_ratio.push_back(1.0);
        joint_cmd.tau_max.push_back(torqueLimitsRL_[i]);
        joint_cmd.control_modes.push_back(JointControlModeRL_(i));
      }
    }
    else
    {
      int total_body_joints = jointNum_ + jointArmNum_ + waistNum_;
      Eigen::VectorXd current_jointPos, current_jointVel;
      
      {
        // 如果state结构不同，尝试从传感器数据获取
        SensorData sensor_data = getRobotSensorData();
        if (sensor_data.jointPos_.size() < total_body_joints || sensor_data.jointVel_.size() < total_body_joints)
        {
          ROS_ERROR_THROTTLE(1.0, "[%s] sensor joint size mismatch: pos=%ld vel=%ld expected=%d",
                             name_.c_str(), sensor_data.jointPos_.size(), sensor_data.jointVel_.size(), total_body_joints);
          return;
        }
        current_jointPos = sensor_data.jointPos_.head(total_body_joints);
        current_jointVel = sensor_data.jointVel_.head(total_body_joints);
      }
      
      for (int i1 = 0; i1 < total_body_joints; ++i1)
      {
        if (JointControlModeRL_(i1) == 0)
        {
          if (JointPDModeRL_(i1) == 0)
          {
            joint_cmd.joint_q.push_back(0.0);
            joint_cmd.joint_v.push_back(0.0);
            joint_cmd.joint_kp.push_back(0);
            joint_cmd.joint_kd.push_back(0);
            joint_cmd.tau.push_back(actuation(i1));
            joint_cmd.tau_ratio.push_back(1);
            joint_cmd.tau_max.push_back(torqueLimitsRL_[i1]);
            joint_cmd.control_modes.push_back(JointControlModeRL_(i1));
          }
          else
          {
            joint_cmd.joint_q.push_back(actuation(i1));
            joint_cmd.joint_v.push_back(0.0);
            joint_cmd.joint_kp.push_back(jointKpRL_[i1]);
            joint_cmd.joint_kd.push_back(jointKdRL_[i1]);
            joint_cmd.tau.push_back(0.0);
            joint_cmd.tau_ratio.push_back(1);
            joint_cmd.tau_max.push_back(torqueLimitsRL_[i1]);
            joint_cmd.control_modes.push_back(JointControlModeRL_(i1));
          }
        }
        else
        {
          joint_cmd.joint_q.push_back(current_jointPos(i1));
          joint_cmd.joint_v.push_back(0.0);
          joint_cmd.joint_kp.push_back(jointKpRL_[i1]);
          joint_cmd.joint_kd.push_back(jointKdRL_[i1]);
          joint_cmd.tau.push_back(actuation(i1));
          joint_cmd.tau_ratio.push_back(1);
          joint_cmd.tau_max.push_back(torqueLimitsRL_[i1]);
          joint_cmd.control_modes.push_back(JointControlModeRL_(i1));
        }
      }
    }
    
    // 设置头部关节（保持零位）
    for (int i = 0; i < headNum_; ++i)
    {
      joint_cmd.joint_q.push_back(0.0);
      joint_cmd.joint_v.push_back(0.0);
      joint_cmd.tau.push_back(0.0);
      joint_cmd.tau_ratio.push_back(1.0);
      joint_cmd.tau_max.push_back(10.0);
      joint_cmd.joint_kp.push_back(0.0);
      joint_cmd.joint_kd.push_back(0.0);
      joint_cmd.control_modes.push_back(0);
    }

    if (is_roban_)
    {
      // 将腰部关节命令从index 0移动到index 12
      // std::cout << "[" << name_ << "] Adjusting joint command for Roban model\n";
      joint_cmd.joint_q[0] = -joint_cmd.joint_q[0];
      joint_cmd.joint_v[0] = -joint_cmd.joint_v[0];
      joint_cmd.tau[0] = -joint_cmd.tau[0];
      moveStdVectorEntry(joint_cmd.joint_q, 0, 12);
      moveStdVectorEntry(joint_cmd.joint_v, 0, 12);
      moveStdVectorEntry(joint_cmd.joint_kp, 0, 12);
      moveStdVectorEntry(joint_cmd.joint_kd, 0, 12);
      moveStdVectorEntry(joint_cmd.tau, 0, 12);
      moveStdVectorEntry(joint_cmd.tau_ratio, 0, 12);
      moveStdVectorEntry(joint_cmd.tau_max, 0, 12);
      moveStdVectorEntry(joint_cmd.control_modes, 0, 12);
    }
  }

  void DanceController::preprocessSensorData(SensorData& sensor_data)
  {
    // 先执行基类中的通用滤波逻辑（RL IMU 滤波）
    RLControllerBase::preprocessSensorData(sensor_data);
    
    // 如果是Roban机型，需要调整腰部关节数据索引
    if (is_roban_)
    {
      // 将腰部关节数据从index 12移动到index 0
      moveVectorEntry(sensor_data.jointPos_, 12, 0);
      moveVectorEntry(sensor_data.jointVel_, 12, 0);
      moveVectorEntry(sensor_data.jointAcc_, 12, 0);
      moveVectorEntry(sensor_data.jointCurrent_, 12, 0);
      sensor_data.jointPos_[0] = -sensor_data.jointPos_[0];
      sensor_data.jointVel_[0] = -sensor_data.jointVel_[0];
      sensor_data.jointAcc_[0] = -sensor_data.jointAcc_[0];
      sensor_data.jointCurrent_[0] = -sensor_data.jointCurrent_[0];
    }
  }

} // namespace humanoid_controller
