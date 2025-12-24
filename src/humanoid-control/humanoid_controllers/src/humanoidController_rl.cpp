#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <cmath>  // 添加数学函数支持
#include "std_srvs/SetBool.h"
#include "humanoid_controllers/humanoidController.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <std_srvs/Trigger.h>
#include <angles/angles.h>
#include <humanoid_estimation/FromTopiceEstimate.h>
#include <humanoid_estimation/LinearKalmanFilter.h>
#ifdef KUAVO_CONTROL_LIB_FOUND
#include <kuavo_estimation/base_filter/InEkfBaseFilter.h>
#endif
#include <humanoid_wbc/WeightedWbc.h>
#include <humanoid_wbc/StandUpWbc.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <humanoid_wbc/HierarchicalWbc.h>
#include "kuavo_common/common/utils.h"
#include "kuavo_common/common/sensor_data.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include <memory> // 添加此行以支持智能指针
#include <iomanip> // 添加此行以支持输出格式控制
#include <cmath> // 添加此行以支持数学函数
#include <filesystem>
#include <fstream>
// #include "../../cnpy/cnpy.h"

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace humanoid;
  using Duration = std::chrono::duration<double>;
  using Clock = std::chrono::high_resolution_clock;
  std::mutex head_mtx;

  /**
  * @brief 将Eigen::VectorXd中indexA位置的值移动到indexB，并将两者之间的区间向indexA方向平移。
  *
  * 例如：vec = [a, b, c, d], indexA = 0, indexB = 2
  * 过程：取出a，区间[b, c]整体向前移动得到[b, c, d, d]，最后将a放到indexB处，结果为[b, c, a, d]。
  *
  * @param vec      需要操作的向量
  * @param indexA   源位置
  * @param indexB   目标位置
  */
  static void moveVectorEntry(Eigen::VectorXd &vec, int indexA, int indexB)
  {
    if (indexA == indexB)
      return;

    const int size = static_cast<int>(vec.size());
    if (indexA < 0 || indexA >= size || indexB < 0 || indexB >= size)
      throw std::out_of_range("moveVectorEntry: index out of range");

    const double value = vec(indexA);
    if (indexA < indexB)
    {
      for (int i = indexA; i < indexB; ++i)
      {
        vec(i) = vec(i + 1);
      }
      vec(indexB) = value;
    }
    else
    {
      for (int i = indexA; i > indexB; --i)
      {
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

  static void mujocoSimStart(ros::NodeHandle &nh_)
  {
    std_srvs::SetBool srv;
    srv.request.data = true;
    // 等待服务可用
    bool service_available = ros::service::waitForService("sim_start", ros::Duration(5.0)); // 5秒超时
    if (service_available)
    {
      ros::ServiceClient sim_start_client = nh_.serviceClient<std_srvs::SetBool>("sim_start");
      if (sim_start_client.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("sim_start Service call succeeded with message: %s", srv.response.message.c_str());
          return;
        }
        else
        {
          ROS_ERROR("sim_start Service call failed");
        }
      }
      else
      {
        ROS_ERROR("Failed to call sim_start service");
      }
    }
    else
    {
      ROS_ERROR("sim_start Service not available");
    }
    exit(1);
  }
  void humanoidController::keyboard_thread_func()
  {
    usleep(100000);
    struct sched_param param;
    param.sched_priority = 0;
    auto result = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    if (result != 0)
    {
      std::cerr << "Failed to set keyboard_thread_func's scheduling parameters. Error: " << strerror(result) << std::endl;
    }
    ros::NodeHandle nh;
    char Walk_Command = '\0';
    while (ros::ok())
    {
      CommandData CommandData_;
      CommandData_ = getCommandData();
      if (hardware_status_ != 1)
      {
        usleep(100000);
        continue;
      }
      if (kbhit())
      {
        Walk_Command = getchar();
        if (Walk_Command == '\n' || Walk_Command == '\0')
        {
          continue;
        }
        // 首先，将光标移动到上一行的开始
        std::cout << "\033[F";
        // 然后，清除当前行（从光标位置到行尾）
        std::cout << "\033[K";
        std::cout << "[keyboard command]: " << Walk_Command << std::endl;
        if (Walk_Command == 'x')
        {
          std::cout << "x" << std::endl;
          for (int i = 0; i < 5; i++)
          {
            std::cout << "publish stop message" << std::endl;
            std_msgs::Bool stop_msg;
            stop_msg.data = true;
            stop_pub.publish(stop_msg);
            ros::Duration(0.1).sleep();
          }
        }
        // else if (Walk_Command == 'f')
        // {
        //   wbc_only_ = !wbc_only_;
        //   std::cout << "start using mpc: " << !wbc_only_ << std::endl;
        // }
        // else if (Walk_Command == 'v')
        // {
        //   std::cout << "v" << std::endl;
        //   control_mode_ = (control_mode_ == 1) ? 2 : 1;
        //   std::cout << "control mode:" << control_mode_ << std::endl;
        // }
        // else if (Walk_Command == 't')
        // {
        //   control_mode_ = (control_mode_ == 0) ? 2 : 0;
        //   std::cout << "control mode:" << control_mode_ << std::endl;
        // }
        else if (Walk_Command == 'l')
        {
          // CommandData_ = initialCommandData_;
        //   is_rl_controller_ = true;
          printRLparam();
        }
        else if (Walk_Command == 'w')
        {
          CommandData_.cmdStance_ = 0;
          CommandData_.cmdVelLineX_ += 0.1;
          ROS_INFO_STREAM("Velocity linear x: " << CommandData_.cmdVelLineX_);
        }
        else if (Walk_Command == 's')
        {
          CommandData_.cmdStance_ = 0;
          CommandData_.cmdVelLineX_ -= 0.1;
          ROS_INFO_STREAM("Velocity linear x: " << CommandData_.cmdVelLineX_);
        }
        else if (Walk_Command == 'a')
        {
          CommandData_.cmdStance_ = 0;
          CommandData_.cmdVelLineY_ += 0.1;
          ROS_INFO_STREAM("Velocity linear y: " << CommandData_.cmdVelLineY_);
        }
        else if (Walk_Command == 'd')
        {
          CommandData_.cmdStance_ = 0;
          CommandData_.cmdVelLineY_ -= 0.1;
          ROS_INFO_STREAM("Velocity linear y: " << CommandData_.cmdVelLineY_);
        }
        else if (Walk_Command == 'q')
        {
          CommandData_.cmdStance_ = 0;
          CommandData_.cmdVelAngularZ_ += 0.1;
          ROS_INFO_STREAM("Velocity angular z: " << CommandData_.cmdVelAngularZ_);
        }
        else if (Walk_Command == 'e')
        {
          CommandData_.cmdStance_ = 0;
          CommandData_.cmdVelAngularZ_ -= 0.1;
          ROS_INFO_STREAM("Velocity angular z: " << CommandData_.cmdVelAngularZ_);
        }
        else if (Walk_Command == ' ')
        {
          CommandData_.setzero();
          CommandData_.cmdStance_ = initialCommandData_.cmdStance_;
          ROS_INFO_STREAM("Velocity linear x: " << CommandData_.cmdVelLineX_);
          ROS_INFO_STREAM("Velocity linear y: " << CommandData_.cmdVelLineY_);
          ROS_INFO_STREAM("Velocity angular z: " << CommandData_.cmdVelAngularZ_);
        }
        else if (Walk_Command == 'm')
        {
          CommandData_.setzero();
          ROS_INFO_STREAM("Velocity linear x: " << CommandData_.cmdVelLineX_);
          ROS_INFO_STREAM("Velocity linear y: " << CommandData_.cmdVelLineY_);
          ROS_INFO_STREAM("Velocity angular z: " << CommandData_.cmdVelAngularZ_);
        }
        setCommandData(CommandData_);
        Walk_Command = '\0';
      }
      usleep(50000);
    }
  }
  void humanoidController::printRLparam()
  {
    std::cout << "[RL param]:Inital Command: " << initialCommandData_.getCommand().transpose() << std::endl;
    // std::cout << "[RL param]:Start Using RL: " << is_rl_controller_ << std::endl;
    std::cout << "[RL param]:Joint Control Mode(0:CST, 1:CSV, 2:CSP):" << JointControlMode_.transpose() << std::endl;
    std::cout << "[RL param]:Joint PD mode:" << JointPDMode_.transpose() << std::endl;
    std::cout << "[RL param]:Joint Kp:" << jointKp_.transpose() << std::endl;
    std::cout << "[RL param]:Joint Kd:" << jointKd_.transpose() << std::endl;
    std::cout << "[RL param]:Initial State:" << initialState_.transpose() << std::endl;
    std::cout << "[RL param]:Torque Limits:" << torqueLimits_.transpose() << std::endl;
    std::cout << "[RL param]:Action Scale Test:" << actionScaleTest_.transpose() << std::endl;
    std::cout << "=============================================================================" << std::endl;
  }

  bool humanoidController::init(HybridJointInterface *robot_hw, ros::NodeHandle &controller_nh, bool is_nodelet_node)
  {
    int robot_version_int;
    
    controllerNh_ = controller_nh;
    ros_logger_ = new TopicLogger(controllerNh_);
    if (controllerNh_.hasParam("/robot_version"))
    {
      controllerNh_.getParam("/robot_version", robot_version_int);
      int major = robot_version_int / 10;
      int minor = robot_version_int % 10;
      robot_version = RobotVersion(major, minor);
    }
    is_nodelet_node_ = is_nodelet_node;
    drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(robot_version, true, 2e-3);
    kuavo_settings_ = drake_interface_->getKuavoSettings();

    auto &motor_info = kuavo_settings_.hardware_settings;
    headNum_ = motor_info.num_head_joints;
    armNumReal_ = motor_info.num_arm_joints;
    waistNum_ = motor_info.num_waist_joints;
    jointNumReal_ = motor_info.num_joints - headNum_ - armNumReal_ - waistNum_;
    actuatedDofNumReal_ = jointNumReal_ + armNumReal_ + waistNum_ + headNum_;
    ros::param::set("/armRealDof", static_cast<int>(armNumReal_));
    ros::param::set("/legRealDof", static_cast<int>(jointNumReal_));
    ros::param::set("/waistRealDof", static_cast<int>(waistNum_));
    ros::param::set("/headRealDof", static_cast<int>(headNum_));
    output_tau_ = vector_t::Zero(actuatedDofNumReal_);
    output_pos_ = vector_t::Zero(actuatedDofNumReal_);
    output_vel_ = vector_t::Zero(actuatedDofNumReal_);

    // vector_t drake_q = drake_interface_->getDrakeSquatState();
    // vector_t mujoco_q = vector_t::Zero(drake_q.size());
    // mujoco_q << drake_q.segment(4, 3), drake_q.head(4), drake_q.tail(drake_q.size() - 7);
    // std::vector<double> mujoco_init_state;
    // for (int i = 0; i < drake_q.size(); i++)
    // {
    //   mujoco_init_state.push_back(mujoco_q(i));
    // }
    // ros::param::set("mujoco_init_state", mujoco_init_state);

    auto robot_config = drake_interface_->getRobotConfig();
    ruiwo_motor_velocities_factor_ = robot_config->getValue<double>("motor_velocities_factor");
    AnkleSolverType ankleSolverType = static_cast<AnkleSolverType>(robot_config->getValue<int>("ankle_solver_type"));
    ankleSolver.getconfig(ankleSolverType);

    // Initialize OCS2
    inferenceFrequency_ = 100.0;      // 默认100Hz
    double controlFrequency_ = 500.0; // 默认500Hz
    std::string urdfFile;
    std::string taskFile;
    std::string referenceFile;
    std::string gaitCommandFile;
    std::string rlParamFile;
    controllerNh_.getParam("/network_model_file", networkModelPath_);
    controllerNh_.getParam("/rl_param", rlParamFile);
    controllerNh_.getParam("/urdfFile", urdfFile);
    controllerNh_.getParam("/taskFile", taskFile);
    controllerNh_.getParam("/referenceFile", referenceFile);
    controllerNh_.getParam("/gaitCommandFile", gaitCommandFile);
    controllerNh_.getParam("/use_external_mpc", use_external_mpc_);
    controllerNh_.getParam("/wbc_frequency", controlFrequency_);
    std::cout << "urdfFile: " << urdfFile << std::endl;
    dt_ = 1.0 / controlFrequency_;
    if (controllerNh_.hasParam("/real"))
    {
      controllerNh_.getParam("/real", is_real_);
      controllerNh_.getParam("/cali", is_cali_);
      if (is_real_)
      {
        std::cout << "real robot controller" << std::endl;
        ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(controllerNh_);
        hardware_interface_ptr_ = new KuavoHardwareInterface(nh_ptr, jointNum_);
      }
    }
    if (controllerNh_.hasParam("wbc_only"))
    {
      controllerNh_.getParam("/wbc_only", wbc_only_);
    }
    if (controllerNh_.hasParam("play_back"))
    {
      controllerNh_.getParam("/play_back", is_play_back_mode_);
    }
    if (controllerNh_.hasParam("channel_map_path"))
    {
      std::string channel_map_path;
      controllerNh_.getParam("channel_map_path", channel_map_path);
      ROS_INFO_STREAM("Loading joystick mapping from " << channel_map_path);
      loadJoyJsonConfig(channel_map_path);
    }
    else
    {
      ROS_WARN_STREAM("No channel_map_path parameter found, using default joystick mapping.");
    }
    if (controllerNh_.hasParam("joystick_sensitivity"))
    {
      controllerNh_.getParam("joystick_sensitivity", joystickSensitivity);
      ROS_INFO_STREAM("Loading joystick sensitivity: " << joystickSensitivity);
    }
    else
    {
      ROS_WARN_STREAM("No input sensitivity parameter found, using default joystick sensitivity.");
    }
    Eigen::Vector4d joystickFilterCutoffFreq_(joystickSensitivity, joystickSensitivity,
                                              joystickSensitivity, joystickSensitivity);
    joystickFilter_.setParams(0.01, joystickFilterCutoffFreq_);
    oldJoyMsg_.axes = std::vector<float>(8, 0.0); // 假设有 8 个轴，默认值为 0.0
    oldJoyMsg_.buttons = std::vector<int32_t>(12, 0);
    size_t buffer_size = (is_play_back_mode_) ? 20 : 5;
    sensors_data_buffer_ptr_ = new KuavoDataBuffer<SensorData>("humanoid_sensors_data_buffer", buffer_size, dt_);

    bool verbose = false;
    loadData::loadCppDataType(taskFile, "humanoid_interface.verbose", verbose);
    setupHumanoidInterface(taskFile, urdfFile, referenceFile, gaitCommandFile, verbose, robot_version_int);
    ros::NodeHandle nh;
    CentroidalModelPinocchioMapping pinocchioMapping(HumanoidInterface_->getCentroidalModelInfo());
    std::cout << "HumanoidInterface_->getCentroidalModelInfo().robotMass:" << HumanoidInterface_->getCentroidalModelInfo().robotMass << std::endl;
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(HumanoidInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        HumanoidInterface_->modelSettings().contactNames3DoF);

    pinocchioInterface_ptr_ = new PinocchioInterface(HumanoidInterface_->getPinocchioInterface());
    eeKinematicsPtr_->setPinocchioInterface(*pinocchioInterface_ptr_);
    auto &info = HumanoidInterface_->getCentroidalModelInfo();
    centroidalModelInfoWBC_ = info;
    centroidalModelInfo_ = info;// TODO:适配旧的代码
    std::cout << "info.actuatedDofNum:" << info.actuatedDofNum << std::endl;
    jointArmNum_ = info.actuatedDofNum - jointNum_ - waistNum_;
    jointTorqueCmd_.resize(jointNum_ + jointArmNum_ + waistNum_);
    jointTorqueCmd_.setZero();
    initialState_.resize(12 + jointNum_ + jointArmNum_ + waistNum_);
    defaultJointPos_.resize(jointNum_ + jointArmNum_ + waistNum_);
    JointControlMode_.resize(jointNum_ + jointArmNum_ + waistNum_);
    JointControlMode_.setZero();
    JointPDMode_.resize(jointNum_ + jointArmNum_ + waistNum_);
    JointPDMode_.setZero();
    jointKp_.resize(jointNum_ + jointArmNum_ + waistNum_);
    jointKd_.resize(jointNum_ + jointArmNum_ + waistNum_);
    torqueLimits_.resize(jointNum_ + jointArmNum_ + waistNum_);
    actionScaleTest_.resize(jointNum_ + jointArmNum_ + waistNum_);
    jointCmdFilterState_.resize(jointNum_ + jointArmNum_ + waistNum_);
    sensor_data_head_.resize_joint(headNum_);
    head_kp_.resize(headNum_);
    head_kd_.resize(headNum_);
    output_tau_.resize(jointNum_ + jointArmNum_ + waistNum_);
    output_tau_.setZero();
    jointPos_ = vector_t::Zero(jointNum_ + jointArmNum_ + waistNum_);
    jointVel_ = vector_t::Zero(jointNum_ + jointArmNum_ + waistNum_);
    jointAcc_ = vector_t::Zero(jointNum_ + jointArmNum_ + waistNum_);
    quat_ = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);
    loadSettings(rlParamFile, verbose, dt_);
    
    // 使用initialState_作为数据源，它来自配置文件中的defaultBaseState_和defaultJointPos_
    // initialState_结构：[vel(6) + pos(3) + angular(3) + joints] = [defaultBaseState_(12) + defaultJointPos_]
    int num_joints = initialState_.size() - 12;  // 关节数量
    vector_t mujoco_q = vector_t::Zero(7 + num_joints);  // MuJoCo格式：[pos(3) + quat(4) + joints]
    vector3_t init_root_pos = initialState_.segment(6, 3); // 基座位置 [x, y, z]
    Eigen::Vector4d init_root_quat; // 基座四元数 [qw, qx, qy, qz]
    vector_t init_joints = initialState_.tail(num_joints); // 关节角度
    // 将姿态角转换为四元数
    double roll = initialState_(9);  // angular_x
    double pitch = initialState_(10); // angular_y  
    double yaw = initialState_(11);    // angular_z
    
    // 计算四元数 (ZYX顺序) !!!!
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;
//    moveVectorEntry(init_joints,0,12);
    // 将initialState_转换为MuJoCo格式：[pos(3) + quat(4) + joints]
    // mujoco_q << initialState_.segment(6, 3),  // 基座位置 [x, y, z]
    //             qw, qx, qy, qz,               // 四元数 [qw, qx, qy, qz]
    //             initialState_.tail(num_joints); // 所有关节角度
    // init_root_pos << -0.1974,  0.3601,  0.3772;
    // init_root_quat <<  0.5435, -0.4402,  0.4714,  0.5371;
    // init_joints << -5.6453e-02,
    // -2.3636e+00, -8.4597e-01, -2.2839e-01,  1.7854e+00,
    //      -8.7265e-01, -3.6734e-03,  2.0005e+00,  6.9555e-01,  5.0070e-01,
    //       2.1167e+00, -8.7266e-01, -2.6180e-01, -1.2843e+00,  2.1031e-01,
    //      -1.1310e+00, -3.5465e-01, -1.4375e+00, -4.8355e-02,  1.3260e+00,
    //      -2.5873e-06;
    // mujoco_q << init_root_pos,init_root_quat, init_joints;
    mujoco_q << init_root_pos,init_root_quat, init_joints;
    std::vector<double> mujoco_init_state;
    for (int i = 0; i < mujoco_q.size(); i++)
    {
      mujoco_init_state.push_back(mujoco_q(i));
    }

    ros::param::set("mujoco_init_state", mujoco_init_state);
    
    if (headNum_ > 0)
    {
      loadData::loadEigenMatrix(referenceFile, "head_kp_", head_kp_);
      loadData::loadEigenMatrix(referenceFile, "head_kd_", head_kd_);
    }
    auto squat_initial_state_ =  drake_interface_->getSquatInitialState();
    std::vector<double> squat_initial_state_vector(squat_initial_state_.data(), squat_initial_state_.data() + squat_initial_state_.size());
    std::cout << "controller squat_initial_state_:" << squat_initial_state_.transpose() << std::endl;

    std::cout << "controller initialState: " << initialState_.transpose() << std::endl;
    std::vector<double> initial_state_vector(initialState_.data(), initialState_.data() + initialState_.size());
    nh.setParam("/initial_state", initial_state_vector);
    nh.setParam("/squat_initial_state", squat_initial_state_vector);

    sensorsDataSub_ = controllerNh_.subscribe<kuavo_msgs::sensorsData>("/sensors_data_raw", 10, &humanoidController::sensorsDataCallback, this);
    head_sub_ = controllerNh_.subscribe("/robot_head_motion_data", 10, &humanoidController::headCmdCallback, this);
    joy_sub_ = controllerNh_.subscribe<sensor_msgs::Joy>("/joy", 10, &humanoidController::joyCallback, this);
    // cmd_vel_sub_ = controllerNh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &humanoidController::cmdVelCallback, this);
    jointCmdPub_ = controllerNh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
    targetTorquePub_ = controllerNh_.advertise<std_msgs::Float32MultiArray>("/targetTorque", 10);
    wbcFrequencyPub_ = controllerNh_.advertise<std_msgs::Float64>("/monitor/frequency/wbc", 10);
    wbcTimeCostPub_ = controllerNh_.advertise<std_msgs::Float64>("/monitor/time_cost/wbc", 10);
    stop_pub = controllerNh_.advertise<std_msgs::Bool>("/stop_robot", 10);
    rl_control_service_ = controllerNh_.advertiseService("/humanoid_controller/walkenable", &humanoidController::WalkenableCallback, this);
    humanoidStatePublisher_ = controllerNh_.advertise<std_msgs::Float64MultiArray>("/humanoid/mm_state", 10);

    // State estimation
    setupStateEstimate(taskFile, verbose);
    sensors_data_buffer_ptr_->waitForReady();
    singleInputData.resize(numSingleObs_);
    networkInputData_.resize(numSingleObs_ * frameStack_);
    commandPhase_.resize(2);
    actions_.resize(jointNum_ + jointArmNum_ + waistNum_);
    singleInputData.setZero();
    networkInputData_.setZero();
    actions_.setZero();
    humanoidState_.resize(6 + jointArmNum_ + waistNum_); // base + arm + waist, for kmpc
    for (int i = 0; i < frameStack_; i++)
    {
      input_deque.push_back(singleInputData);
    }
    compiled_model_ =
        core_.compile_model(networkModelPath_, "CPU"); // 创建编译模型

    keyboardThread_ = std::thread(&humanoidController::keyboard_thread_func, this);
    if (!keyboardThread_.joinable())
    {
      std::cerr << "Failed to start keyboard thread" << std::endl;
      exit(1);
    }
    // 在初始化之前打印
    std::cout << "jointKp_ size: " << jointKp_.size() << std::endl;
    std::cout << "jointKd_ size: " << jointKd_.size() << std::endl;
    std::cout << "jointNum_: " << jointNum_ << std::endl;
    std::cout << "waistNum_: " << waistNum_ << std::endl;
    std::cout << "jointArmNum_: " << jointArmNum_ << std::endl;

    return true;
  }
  void humanoidController::loadJoyJsonConfig(const std::string &config_file)
  {
    std::ifstream ifs(config_file);
    if (!ifs.is_open())
    {
      std::cerr << "Failed to open config file: " << config_file << std::endl;
      return;
    }
    nlohmann::json data_;
    ifs >> data_;
    auto updateMap = [](auto &map, const auto &items)
    {
      for (const auto &[key, value] : items)
      {
        std::cout << "button:" << key << " value:" << value << std::endl;
        map[key] = value;
      }
    };
    updateMap(joyButtonMap, data_["JoyButton"].items());
    updateMap(joyAxisMap, data_["JoyAxis"].items());
  }
  void humanoidController::loadSettings(const std::string &rlParamFile, bool verbose, double dt)
  {
    bool enable_ = false;
    double index_ = 0, startIdx_ = 0, mumIdx_ = 0, obsScale_ = 0;
    int num_ = 0;
    std::string networkModelFile_;
    Eigen::Vector3d accFilterCutoffFreq_, freeAccFilterCutoffFreq_, gyroFilterCutoffFreq_;
    Eigen::VectorXd defaultBaseState_(jointNum_), jointCmdFilterCutoffFreq_(jointNum_ + jointArmNum_);
    CommandData commandData_;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(rlParamFile, pt);
    // 使用新的 helper 函数来简化数据加载
    auto loadEigenMatrix = [&](const std::string &key, auto &matrix)
    {
      loadData::loadEigenMatrix(rlParamFile, key, matrix);
    };
    loadEigenMatrix("defaultJointState", defaultJointPos_);
    loadEigenMatrix("defaultBaseState", defaultBaseState_);
    loadEigenMatrix("JointControlMode", JointControlMode_);
    loadEigenMatrix("JointPDMode", JointPDMode_);
    loadEigenMatrix("jointKp", jointKp_);
    loadEigenMatrix("jointKd", jointKd_);
    loadEigenMatrix("torqueLimits", torqueLimits_);
    loadEigenMatrix("actionScaleTest", actionScaleTest_);
    loadEigenMatrix("accFilterCutoffFreq", accFilterCutoffFreq_);
    loadEigenMatrix("freeAccFilterCutoffFreq", freeAccFilterCutoffFreq_);
    loadEigenMatrix("gyroFilterCutoffFreq", gyroFilterCutoffFreq_);
    loadEigenMatrix("jointCmdFilterCutoffFreq", jointCmdFilterCutoffFreq_);
    loadEigenMatrix("jointCmdFilterState", jointCmdFilterState_);
    loadEigenMatrix("accFilterState", accFilterState_);
    loadEigenMatrix("freeAccFilterState", freeAccFilterState_);
    loadEigenMatrix("gyroFilterState", gyroFilterState_);
    loadEigenMatrix("velocityLimits", velocityLimits_);
    ros::param::set("/humanoid_controller/velocityLimits", std::vector<double>(velocityLimits_.data(), velocityLimits_.data() + velocityLimits_.size()));
    // loadEigenMatrix("initalCommand", initalCommand_);
    // loadEigenMatrix("commandScale", commandScale_);
    loadData::loadCppDataType(rlParamFile, "actionScale", actionScale_);
    loadData::loadCppDataType(rlParamFile, "frameStack", frameStack_);
    loadData::loadCppDataType(rlParamFile, "numSingleObs", numSingleObs_);
    loadData::loadCppDataType(rlParamFile, "cycleTime", cycleTime_);
    loadData::loadCppDataType(rlParamFile, "cycleTime_short", cycleTime_short_);  
    loadData::loadCppDataType(rlParamFile, "switch_ratio", switch_ratio_); 
    loadData::loadCppDataType(rlParamFile, "phase", phase_);
    loadData::loadCppDataType(rlParamFile, "episodeLength", episodeLength_);
    loadData::loadCppDataType(rlParamFile, "clipObservations", clipObservations_);
    loadData::loadCppDataType(rlParamFile, "clipActions", clipActions_);
    loadData::loadCppDataType(rlParamFile, "withArm", withArm_);
    loadData::loadCppDataType(rlParamFile, "inferenceFrequency", inferenceFrequency_);
    loadData::loadCppDataType(rlParamFile, "networkModelFile", networkModelFile_);

    // 加载手臂插值配置
    if (pt.find("armInterpolationDuration") != pt.not_found()) {
      loadData::loadCppDataType(rlParamFile, "armInterpolationDuration", interpolation_duration_);
    } else {
      interpolation_duration_ = 1.0; // 默认2秒，更平滑
    }
    // loadData::loadCppDataType(rlParamFile, "ankleSolverType", ankleSolverType_);

    // ankleSolver.getconfig(ankleSolverType_);
    auto defaultJointPosTemp = defaultJointPos_;
    moveVectorEntry(defaultJointPosTemp, 0, 12);
    initialState_ << defaultBaseState_, defaultJointPosTemp;
    networkModelPath_ = networkModelPath_ + networkModelFile_;
    accFilter_.setParams(dt, accFilterCutoffFreq_);
    freeAccFilter_.setParams(dt, freeAccFilterCutoffFreq_);
    gyroFilter_.setParams(dt, gyroFilterCutoffFreq_);
    jointCmdFilter_.setParams(dt, jointCmdFilterCutoffFreq_);

    // 加载命令数据
    const std::string prefixCommandData_ = "commandData";
    const std::vector<std::pair<std::string, double CommandData::*>> cmdInitalList = {
        {"cmdVelLineX", &CommandData::cmdVelLineX_},
        {"cmdVelLineY", &CommandData::cmdVelLineY_},
        {"cmdVelLineZ", &CommandData::cmdVelLineZ_},
        {"cmdVelAngularX", &CommandData::cmdVelAngularX_},
        {"cmdVelAngularY", &CommandData::cmdVelAngularY_},
        {"cmdVelAngularZ", &CommandData::cmdVelAngularZ_},
        {"cmdStance", &CommandData::cmdStance_},
    };
    const std::vector<std::pair<std::string, double CommandData::*>> cmdScaleList = {
        {"cmdVelLineX", &CommandData::cmdVelScaleLineX_},
        {"cmdVelLineY", &CommandData::cmdVelScaleLineY_},
        {"cmdVelLineZ", &CommandData::cmdVelScaleLineZ_},
        {"cmdVelAngularX", &CommandData::cmdVelScaleAngularX_},
        {"cmdVelAngularY", &CommandData::cmdVelScaleAngularY_},
        {"cmdVelAngularZ", &CommandData::cmdVelScaleAngularZ_},
        {"cmdStance", &CommandData::cmdScaleStance_}};
    for (const auto &[cmdName, cmdMember] : cmdInitalList)
    {
      loadData::loadPtreeValue(pt, commandData_.*cmdMember, prefixCommandData_ + ".inital." + cmdName, verbose);
    }
    for (const auto &[cmdName, cmdMember] : cmdScaleList)
    {
      loadData::loadPtreeValue(pt, commandData_.*cmdMember, prefixCommandData_ + ".scale." + cmdName, verbose);
    }
    initialCommandData_ = commandData_;
    setCommandData(initialCommandData_);
    // 加载单输入数据
    const std::string prefixSingleInputData_ = "singleInputData";
    if (verbose)
    {
      std::cerr << "\n #### singleInputData:";
      std::cerr << "\n #### =============================================================================\n";
    }
    for (const auto &pair : pt)
    {
      if (pair.first == prefixSingleInputData_)
      {
        for (const auto &pair2 : pair.second)
        {
          singleInputDataKeys.push_back(pair2.first);
          loadData::loadPtreeValue(pt, startIdx_, prefixSingleInputData_ + "." + pair2.first + ".startIdx", verbose);
          loadData::loadPtreeValue(pt, mumIdx_, prefixSingleInputData_ + "." + pair2.first + ".numIdx", verbose);
          loadData::loadPtreeValue(pt, obsScale_, prefixSingleInputData_ + "." + pair2.first + ".obsScales", verbose);
          num_ += mumIdx_;
          singleInputDataID_[pair2.first] = {startIdx_, mumIdx_, obsScale_};
        }
      }
    }
    if (num_ != numSingleObs_)
    {
      std::cerr << "Error: singleInputData number is not equal to 'numSingleObs_'" << std::endl;
      std::cerr << "Single Obs Number: " << num_ << std::endl;
      std::cerr << "'numSingleObs_' Number: " << numSingleObs_ << std::endl;
      exit(1);
    }
  }
  void humanoidController::headCmdCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr &msg)
  {
      if (msg->joint_data.size() == 2)
      {
          if (msg->joint_data[0] < head_joint_limits_[0].first || msg->joint_data[0] > head_joint_limits_[0].second 
            || msg->joint_data[1] < head_joint_limits_[1].first || msg->joint_data[1] > head_joint_limits_[1].second)
          {
              std::cout << "\033[1;31m[headCmdCallback] Invalid robot head motion data. Head joints must be in the range [" 
                << head_joint_limits_[0].first << ", " << head_joint_limits_[0].second << "] and [" 
                << head_joint_limits_[1].first << ", " << head_joint_limits_[1].second << "].\033[0m" << std::endl;
              return;
          }
          head_mtx.lock();
          desire_head_pos_[0] = msg->joint_data[0]*M_PI/180.0;
          desire_head_pos_[1] = msg->joint_data[1]*M_PI/180.0;
          head_mtx.unlock();
      }
      else
      {
          ROS_WARN("Invalid robot head motion data. Expected 2 elements, but received %lu elements.", msg->joint_data.size());
      }
  }

  void humanoidController::sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg)
  {
    auto &joint_data = msg->joint_data;
    auto &imu_data = msg->imu_data;
    auto &end_effector_data = msg->end_effector_data; // TODO: add end_effector_data to the observation
    SensorData sensor_data;
    sensor_data.resize_joint(jointNum_ + jointArmNum_ + waistNum_);
    // JOINT DATA
    for (size_t i = 0; i < jointNum_ + jointArmNum_ + waistNum_; ++i)
    {
      sensor_data.jointPos_(i) = joint_data.joint_q[i];
      sensor_data.jointVel_(i) = joint_data.joint_v[i];
      sensor_data.jointAcc_(i) = joint_data.joint_vd[i];
      sensor_data.jointCurrent_(i) = joint_data.joint_torque[i];
    }
    // 将腰部关节数据从index 12移动到index 0
    moveVectorEntry(sensor_data.jointPos_, 12, 0);
    moveVectorEntry(sensor_data.jointVel_, 12, 0);
    moveVectorEntry(sensor_data.jointAcc_, 12, 0);
    moveVectorEntry(sensor_data.jointCurrent_, 12, 0);
    sensor_data.jointPos_[0] = -sensor_data.jointPos_[0];
    sensor_data.jointVel_[0] = -sensor_data.jointVel_[0];
    sensor_data.jointAcc_[0] = -sensor_data.jointAcc_[0];
    sensor_data.jointCurrent_[0] = -sensor_data.jointCurrent_[0];

    ros::Time ros_time = msg->header.stamp;
    sensor_data.timeStamp_ = msg->sensor_time;
    double sensor_time_diff = (ros::Time::now() - ros_time).toSec() * 1000;
    ros_logger_->publishValue("/monitor/time_cost/sensor_to_controller", sensor_time_diff);
    // IMU
    sensor_data.quat_.coeffs().w() = imu_data.quat.w;
    sensor_data.quat_.coeffs().x() = imu_data.quat.x;
    sensor_data.quat_.coeffs().y() = imu_data.quat.y;
    sensor_data.quat_.coeffs().z() = imu_data.quat.z;
    sensor_data.angularVel_ << imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z;
    sensor_data.linearAccel_ << imu_data.acc.x, imu_data.acc.y, imu_data.acc.z;
    sensor_data.freeLinearAccel_ << imu_data.free_acc.x, imu_data.free_acc.y, imu_data.free_acc.z;
    sensor_data.pos_w_ << imu_data.pos.x, imu_data.pos.y, imu_data.pos.z;
    sensor_data.orientationCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.angularVelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.linearAccelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    Eigen::Vector3d acc_filtered = accFilter_.update(sensor_data.linearAccel_);
    Eigen::Vector3d free_acc_filtered = freeAccFilter_.update(sensor_data.freeLinearAccel_);
    Eigen::Vector3d gyro_filtered = gyroFilter_.update(sensor_data.angularVel_);
    for (int i = 0; i < 3; i++)
    {
      sensor_data.linearAccel_(i) = accFilterState_(i) * acc_filtered(i) + (1 - accFilterState_(i)) * sensor_data.linearAccel_(i);
      sensor_data.freeLinearAccel_(i) = freeAccFilterState_(i) * free_acc_filtered(i) + (1 - freeAccFilterState_(i)) * sensor_data.freeLinearAccel_(i);
      sensor_data.angularVel_(i) = gyroFilterState_(i) * gyro_filtered(i) + (1 - gyroFilterState_(i)) * sensor_data.angularVel_(i);
    }
    ros_logger_->publishVector("/imu_data_filtered/linearAccel", acc_filtered);
    ros_logger_->publishVector("/imu_data_filtered/angularVel", gyro_filtered);
    ros_logger_->publishVector("/imu_data_filtered/freeLinearAccel", free_acc_filtered);
    sensors_data_buffer_ptr_->addData(sensor_data.timeStamp_.toSec(), sensor_data);
    if (headNum_ > 0 && joint_data.joint_q.size() == jointNumReal_ + armNumReal_ + waistNum_ + headNum_)
    {
      int head_start_index = joint_data.joint_q.size() - headNum_;
      for (size_t i = 0; i < headNum_; ++i)
      {
        sensor_data_head_.jointPos_(i) = joint_data.joint_q[i + head_start_index];
        sensor_data_head_.jointVel_(i) = joint_data.joint_v[i + head_start_index];
        sensor_data_head_.jointAcc_(i) = joint_data.joint_vd[i + head_start_index];
        sensor_data_head_.jointCurrent_(i) = joint_data.joint_torque[i + head_start_index];
      }
    }
    if (!is_initialized_)
      is_initialized_ = true;
  }
  
 
 
  void humanoidController::starting(const ros::Time &time)
  {
    initial_status_ = initialState_;
    currentObservation_.state = initial_status_;
    while (!is_initialized_)
    {
      if (!is_nodelet_node_)
        ros::spinOnce();
      usleep(1000);
    }
    if (is_real_)
    {
      SensorData_t intial_sensor_data;
      hardware_interface_ptr_->init(intial_sensor_data);
      real_init_wait();
    }
    else
    {
      hardware_status_ = 1;
    }
    last_time_ = current_time_;
    updateStateEstimation(time, true);
    currentObservation_.mode = ModeNumber::SS;
    intail_input_ = vector_t::Zero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
    for (int i = 0; i < 8; i++)
      intail_input_(3 * i + 2) = HumanoidInterface_->getCentroidalModelInfo().robotMass * 9.81 / 8; // 48.7*g/8
    optimizedInput_mrt_ = intail_input_;
    
    last_time_ = current_time_;
    if (!is_play_back_mode_)
      sensors_data_buffer_ptr_->sync();
    // 网络推理线程
    inferenceThread_ = std::thread(&humanoidController::inference_thread_func, this);
    if (!inferenceThread_.joinable())
    {
      ROS_ERROR_STREAM("Failed to start inference thread");
      exit(1);
    }
    if (!is_real_ && !is_play_back_mode_)
      mujocoSimStart(controllerNh_);
  }
  void humanoidController::real_init_wait()
  {
    while (ros::ok())
    {
      if (ros::param::get("/hardware/is_ready", hardware_status_))
      {
        if (hardware_status_ == 1)
        {
          std::cerr << "real robot is ready\n";
          break;
        }
      }
      usleep(1000);
    }
  }

  void humanoidController::update(const ros::Time &time, const ros::Duration &dfd)
  {
    const auto t1 = Clock::now();
    ros::Duration period = ros::Duration(dt_);
    updateStateEstimation(time, false);
    vector_t measuredRbdState_;
    measuredRbdState_ = getRobotState();
    const auto t2 = Clock::now();
    vector_t optimizedState_mrt, optimizedInput_mrt;
    auto &info = HumanoidInterface_->getCentroidalModelInfo();
    optimizedState_mrt_ = initial_status_;
    optimizedInput_mrt_ = intail_input_;
    // optimized_mode_ = plannedMode_;
    currentObservation_.input = optimizedInput_mrt_;
    std::chrono::time_point<std::chrono::high_resolution_clock> t3;
    std::chrono::time_point<std::chrono::high_resolution_clock> t4;
    std::chrono::time_point<std::chrono::high_resolution_clock> t5;
    kuavo_msgs::jointCmd jointCmdMsg;
    // if (!is_rl_controller_)
    // {
    //   if (currentObservation_.mode == ModeNumber::SS)
    //   {
    //     wbc_->setStanceMode(true);
    //   }
    //   else
    //   {
    //     wbc_->setStanceMode(false);
    //   }
    //   t3 = Clock::now();
    //   ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt_", optimizedInput_mrt_);
    //   ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt_", optimizedState_mrt_);
    //   ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt_/linear_vel_xyz", optimizedState_mrt_.head<3>());
    //   ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt_/angular_vel_xyz", optimizedState_mrt_.segment<3>(3));
    //   ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt_/pos_xyz", optimizedState_mrt_.segment<3>(6));
    //   ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt_/angular_zyx", optimizedState_mrt_.segment<3>(9));
    //   ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt_/joint_pos", optimizedState_mrt_.segment(12, info.actuatedDofNum));
    //   // ros_logger_->publishValue("/humanoid_controller/optimized_mode_", static_cast<double>(optimized_mode_));
    //   vector_t x = wbc_->update(optimizedState_mrt_, optimizedInput_mrt_, measuredRbdState_, plannedMode_, period.toSec());
    //   t4 = Clock::now();
    //   // 决策变量, 6*body_acc + 12*joint_acc + 3x4*contact_force + 12*torque = 42
    //   const vector_t &torque = x.tail(jointNum_ + jointArmNum_);
    //   const vector_t &wbc_planned_joint_acc = x.segment(6, jointNum_ + jointArmNum_);
    //   const vector_t &wbc_planned_body_acc = x.head(6);
    //   const vector_t &wbc_planned_contact_force = x.segment(6 + jointNum_ + jointArmNum_, wbc_->getContactForceSize());
    //   ros_logger_->publishVector("/humanoid_controller/torque", torque);
    //   ros_logger_->publishVector("/humanoid_controller/wbc_planned_joint_acc", wbc_planned_joint_acc);
    //   ros_logger_->publishVector("/humanoid_controller/wbc_planned_body_acc/linear", wbc_planned_body_acc.head<3>());
    //   ros_logger_->publishVector("/humanoid_controller/wbc_planned_body_acc/angular", wbc_planned_body_acc.tail<3>());
    //   ros_logger_->publishVector("/humanoid_controller/wbc_planned_contact_force/left_foot", wbc_planned_contact_force.head<12>());
    //   ros_logger_->publishVector("/humanoid_controller/wbc_planned_contact_force/right_foot", wbc_planned_contact_force.tail<12>());
    //   vector_t posDes = centroidal_model::getJointAngles(optimizedState_mrt_, HumanoidInterface_->getCentroidalModelInfo());
    //   vector_t velDes = centroidal_model::getJointVelocities(optimizedInput_mrt_, HumanoidInterface_->getCentroidalModelInfo());
    //   scalar_t dt = period.toSec();
    //   posDes = posDes + 0.5 * wbc_planned_joint_acc * dt * dt;
    //   velDes = velDes + wbc_planned_joint_acc * dt;
    //   auto current_jointPos = measuredRbdState_.segment(6, info.actuatedDofNum);
    //   auto current_jointVel = measuredRbdState_.segment(6 + info.generalizedCoordinatesNum, info.actuatedDofNum);
    //   output_tau_ = torque;
    //   t5 = Clock::now();
    //   // TODO: send the controller command to hardware interface
    //   std_msgs::Float32MultiArray targetTorqueMsg;
    //   for (int i1 = 0; i1 < jointNum_ + jointArmNum_; ++i1)
    //   {
    //     targetTorqueMsg.data.push_back(output_tau_(i1));
    //   }
    //   // targetTorquePub_.publish(targetTorqueMsg);
    //   jointCmdMsg.header.stamp = time;
    //   for (int i1 = 0; i1 < jointNum_ + jointArmNum_; ++i1)
    //   {
    //     jointCmdMsg.joint_q.push_back(posDes(i1));
    //     jointCmdMsg.joint_v.push_back(velDes(i1));
    //     jointCmdMsg.tau.push_back(output_tau_(i1));
    //     jointCmdMsg.joint_kp.push_back(0);
    //     jointCmdMsg.joint_kd.push_back(0);
    //     jointCmdMsg.tau_ratio.push_back(1);
    //     jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[i1]);
    //     jointCmdMsg.control_modes.push_back(control_mode_);
    //   }
    // }
    // else
    // {
      auto current_jointPos = measuredRbdState_.segment(6, info.actuatedDofNum);
      // ROS_INFO("info.actuatedDofNum = %d",int(info.actuatedDofNum));
      auto current_jointVel = measuredRbdState_.segment(12 + jointNum_ + waistNum_ + jointArmNum_, info.actuatedDofNum);
      t3 = Clock::now();


      {
        const auto &model = pinocchioInterface_ptr_->getModel();
        auto &data = pinocchioInterface_ptr_->getData();
        
        vector_t qMeasured_pino(info.generalizedCoordinatesNum);
        vector_t vMeasured_pino(info.generalizedCoordinatesNum);

        qMeasured_pino.head<3>() = measuredRbdState_.segment<3>(3); // xyz linear pos
        qMeasured_pino.segment<3>(3) = measuredRbdState_.head<3>(); // zyx angles  
        qMeasured_pino.tail(info.actuatedDofNum) = measuredRbdState_.segment(6, info.actuatedDofNum); // joint positions
        
        vMeasured_pino.head<3>() = measuredRbdState_.segment<3>(info.generalizedCoordinatesNum + 3); // xyz linear velocity
        vMeasured_pino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qMeasured_pino.segment<3>(3), measuredRbdState_.segment<3>(info.generalizedCoordinatesNum)); // converted angular velocity
        vMeasured_pino.tail(info.actuatedDofNum) = measuredRbdState_.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum); // joint velocities
        
        pinocchio::forwardKinematics(model, data, qMeasured_pino, vMeasured_pino);
        pinocchio::centerOfMass(model, data, qMeasured_pino, vMeasured_pino);
        
        vector3_t com_pos_global = data.com[0];
        vector3_t com_vel_global = data.vcom[0];
        
        vector3_t base_pos = qMeasured_pino.head<3>();        // 基座位置 (xyz)
        vector3_t base_euler = qMeasured_pino.segment<3>(3);  // 基座姿态 (zyx欧拉角)
        vector3_t base_vel = vMeasured_pino.head<3>();        // 基座线速度
        
        matrix3_t R_global_to_base = getRotationMatrixFromZyxEulerAngles<scalar_t>(base_euler).transpose();
        
        vector3_t com_relative_global = com_pos_global - base_pos;
        
        vector3_t com_pos_local = R_global_to_base * com_relative_global;
        vector3_t com_vel_relative_global = com_vel_global - base_vel;
        vector3_t com_vel_local = R_global_to_base * com_vel_relative_global;
        
        ros_logger_->publishVector("/humanoid_controller/com/r", com_pos_local);
        ros_logger_->publishVector("/humanoid_controller/com/rd", com_vel_local);
        
        ros_logger_->publishVector("/humanoid_controller/com/r_global", com_pos_global);
        ros_logger_->publishVector("/humanoid_controller/com/rd_global", com_vel_global);
      }
      
      Eigen::VectorXd actuation(jointNum_ + jointArmNum_ + waistNum_);
      actuation.setZero();
      actuation = updateRLcmd(measuredRbdState_);
      t4 = Clock::now();
      jointCmdMsg.header.stamp = time;
      if (!is_real_)
      {
        for (int i1 = 0; i1 < jointNum_ + jointArmNum_ + waistNum_; ++i1)
        {
            jointCmdMsg.joint_q.push_back(0.0);
            jointCmdMsg.joint_v.push_back(0.0);
            jointCmdMsg.joint_kp.push_back(jointKp_[i1]);
            jointCmdMsg.joint_kd.push_back(jointKd_[i1]);
            jointCmdMsg.tau.push_back(actuation(i1));
            jointCmdMsg.tau_ratio.push_back(1);
            jointCmdMsg.tau_max.push_back(torqueLimits_[i1]);
            jointCmdMsg.control_modes.push_back(JointControlMode_(i1)); 
          // std::cout << "joint_kp: " << jointKp_[i1] << " joint_kd: " << jointKd_[i1] << std::endl;
        }
      }
      else
      {
        for (int i1 = 0; i1 < jointNum_ + jointArmNum_ + waistNum_; ++i1)
        {
          if (JointControlMode_(i1) == 0)
          {
            if (JointPDMode_(i1) == 0)
            {
              jointCmdMsg.joint_q.push_back(0.0);
              jointCmdMsg.joint_v.push_back(0.0);
              jointCmdMsg.joint_kp.push_back(0);
              jointCmdMsg.joint_kd.push_back(0);
              jointCmdMsg.tau.push_back(actuation(i1));
              jointCmdMsg.tau_ratio.push_back(1);
              jointCmdMsg.tau_max.push_back(torqueLimits_[i1]);
              jointCmdMsg.control_modes.push_back(JointControlMode_(i1));
            }
            else
            {
              jointCmdMsg.joint_q.push_back(actuation(i1));
              jointCmdMsg.joint_v.push_back(0.0);
              jointCmdMsg.joint_kp.push_back(jointKp_[i1]);
              jointCmdMsg.joint_kd.push_back(jointKd_[i1]);
              jointCmdMsg.tau.push_back(0.0);
              jointCmdMsg.tau_ratio.push_back(1);
              jointCmdMsg.tau_max.push_back(torqueLimits_[i1]);
              jointCmdMsg.control_modes.push_back(JointControlMode_(i1));
            }
          }
          else
          {
            jointCmdMsg.joint_q.push_back(current_jointPos(i1));
            jointCmdMsg.joint_v.push_back(0.0);
            jointCmdMsg.joint_kp.push_back(jointKp_[i1]);
            jointCmdMsg.joint_kd.push_back(jointKd_[i1]);
            jointCmdMsg.tau.push_back(actuation(i1));
            jointCmdMsg.tau_ratio.push_back(1);
            jointCmdMsg.tau_max.push_back(torqueLimits_[i1]);
            jointCmdMsg.control_modes.push_back(JointControlMode_(i1));
          }
        }
      }
      t5 = Clock::now();
    // }

    // 补充头部维度
    // 计算头部反馈力
    if (headNum_ > 0)
    {
      vector_t get_head_pos = vector_t::Zero(headNum_);
      head_mtx.lock();
      get_head_pos = desire_head_pos_;
      head_mtx.unlock();
      auto &hardware_settings = kuavo_settings_.hardware_settings;
      vector_t head_feedback_tau = vector_t::Zero(headNum_);
      vector_t head_feedback_vel = vector_t::Zero(headNum_);
      if (!is_real_) // 实物不需要头部反馈力，来自kuavo仓库的移植
        head_feedback_tau = head_kp_.cwiseProduct(get_head_pos - sensor_data_head_.jointPos_) + head_kd_.cwiseProduct(-sensor_data_head_.jointVel_);
      for (int i3 = 0; i3 < headNum_; ++i3)
      {
        auto cur_head_pos = sensor_data_head_.jointPos_ * TO_DEGREE;
        auto vel = (get_head_pos[i3] - sensor_data_head_.jointPos_[i3]) * TO_DEGREE / dt_ * ruiwo_motor_velocities_factor_;
        double head_limit_vel = hardware_settings.joint_velocity_limits[jointNum_ + armNumReal_ + i3];

        vel = std::clamp(vel, -head_limit_vel, head_limit_vel) * TO_RADIAN;
        jointCmdMsg.joint_q.push_back(get_head_pos(i3));
        jointCmdMsg.joint_v.push_back(0);
        jointCmdMsg.joint_kp.push_back(0);
        jointCmdMsg.joint_kd.push_back(0);  
        jointCmdMsg.tau.push_back(head_feedback_tau(i3));
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.tau_max.push_back(10);
        jointCmdMsg.control_modes.push_back(2);
      }
    }

    // 将腰部关节命令从index 0移动到index 12
    jointCmdMsg.joint_q[0] = -jointCmdMsg.joint_q[0];
    jointCmdMsg.joint_v[0] = -jointCmdMsg.joint_v[0];
    jointCmdMsg.tau[0] = -jointCmdMsg.tau[0];

    moveStdVectorEntry(jointCmdMsg.joint_q, 0, 12);
    moveStdVectorEntry(jointCmdMsg.joint_v, 0, 12);
    moveStdVectorEntry(jointCmdMsg.joint_kp, 0, 12);
    moveStdVectorEntry(jointCmdMsg.joint_kd, 0, 12);
    moveStdVectorEntry(jointCmdMsg.tau, 0, 12);
    moveStdVectorEntry(jointCmdMsg.tau_ratio, 0, 12);
    moveStdVectorEntry(jointCmdMsg.tau_max, 0, 12);
    moveStdVectorEntry(jointCmdMsg.control_modes, 0, 12);


    jointCmdPub_.publish(jointCmdMsg);
    {
      std::lock_guard<std::mutex> lock(joint_cmd_mutex_);
      for (int i1 = 0; i1 < jointNum_ + jointArmNum_ + waistNum_; ++i1)
      {
        jointTorqueCmd_(i1) = jointCmdMsg.tau[i1];
      }
    }
    const auto t6 = Clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t1).count() > 1000)
    {
      std::cout << "t1-t2: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;
      std::cout << "t2-t3: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << " ms" << std::endl;
      std::cout << "t3-t4: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count() << " ms" << std::endl;
      std::cout << "t4-t5: " << std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count() << " ms" << std::endl;
      std::cout << "t5-t6: " << std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count() << " ms" << std::endl;
    }
    // publish time cost
    std_msgs::Float64 msg;
    wbcFrequencyPub_.publish(msg);
    wbcTimeCostPub_.publish(msg);
    static double last_ros_time = ros::Time::now().toSec();
    ros_logger_->publishValue("/monitor/time_cost/controller_loop_time", (ros::Time::now().toSec() - last_ros_time) * 1000);
    last_ros_time = ros::Time::now().toSec();
  }
  void humanoidController::applySensorData(const SensorData &data)
  {
    SensorData sensor_data_copy = data;
    jointPos_ = data.jointPos_;
    jointVel_ = data.jointVel_;
    jointAcc_ = data.jointAcc_;
    jointCurrent_ = data.jointCurrent_;
    quat_ = data.quat_;
    angularVel_ = data.angularVel_;
    linearAccel_ = data.linearAccel_;
    freeLinearAccel_ = data.freeLinearAccel_;
    orientationCovariance_ = data.orientationCovariance_;
    angularVelCovariance_ = data.angularVelCovariance_;
    linearAccelCovariance_ = data.linearAccelCovariance_;
    current_time_ = data.timeStamp_;
    stateEstimate_->updateImu(quat_, angularVel_, linearAccel_, orientationCovariance_, angularVelCovariance_, linearAccelCovariance_);
    sensor_data_copy.jointPos_ = jointPos_;
    sensor_data_copy.jointVel_ = jointVel_;
    sensor_data_copy.jointAcc_ = jointAcc_;
    sensor_data_copy.jointCurrent_ = jointCurrent_;
    sensor_data_copy.quat_ = quat_;
    sensor_data_copy.angularVel_ = angularVel_;
    sensor_data_copy.linearAccel_ = linearAccel_;
    sensor_data_copy.freeLinearAccel_ = freeLinearAccel_;
    sensor_data_copy.quat_offset_ = stateEstimate_->getImuOrientation();
    setRobotSensorData(sensor_data_copy);
  }
  void humanoidController::updateStateEstimation(const ros::Time &time, bool is_init)
  {
    // vector_t jointPos(jointNum_), jointVel(jointNum_), jointCurrent(jointNum_);
    contact_flag_t contacts;
    Eigen::Quaternion<scalar_t> quat;
    // contact_flag_t contactFlag;
    vector3_t angularVel, linearAccel;
    matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;
    SensorData sensors_data;
    vector_t measuredRbdState_;
    measuredRbdState_ = getRobotState();
    if (is_init)
      sensors_data = sensors_data_buffer_ptr_->getLastData();
    else
      sensors_data = sensors_data_buffer_ptr_->getLastData();
    applySensorData(sensors_data);
    if (is_init)
    {
      last_time_ = current_time_ - ros::Duration(0.001);
      stateEstimate_->updateJointStates(jointPos_, jointVel_);
      stateEstimate_->updateIntialEulerAngles(quat_);
      applySensorData(sensors_data);
      stateEstimate_->set_intial_state(currentObservation_.state);
      measuredRbdState_ = stateEstimate_->getRbdState();
      auto mat = sensors_data.quat_.toRotationMatrix();
      
      // 计算yaw偏移
      double current_yaw = std::atan2(mat(1, 2), mat(0, 2)); 
      my_yaw_offset_ = 0.0 - current_yaw;
      std::cout << "sensors_data.quat_: [" << sensors_data.quat_.w() << ", " 
                << sensors_data.quat_.x() << ", " << sensors_data.quat_.y() << ", " << sensors_data.quat_.z() << "]" << std::endl;
      std::cout << "Current yaw: " << current_yaw << std::endl;
      // 归一化到[-π, π]范围
      while (my_yaw_offset_ > M_PI) my_yaw_offset_ -= 2 * M_PI;
      while (my_yaw_offset_ < -M_PI) my_yaw_offset_ += 2 * M_PI;
    }
    double diff_time = (current_time_ - last_time_).toSec();
    // auto est_mode = stateEstimate_->ContactDetection(plannedMode_, jointVel_, jointCurrent_, diff_time);
    // if (is_rl_controller_)
    plannedMode_ = rl_plannedMode_;
    last_time_ = current_time_;
    ros::Duration period = ros::Duration(diff_time);
    vector_t activeTorque_ = jointCurrent_;
    stateEstimate_->setCmdTorque(activeTorque_);
    stateEstimate_->estContactForce(period);
    auto est_contact_force = stateEstimate_->getEstContactForce();
    ros_logger_->publishVector("/state_estimate/contactForce", est_contact_force);
    stateEstimate_->updateMode(plannedMode_);
    ros_logger_->publishValue("/rl_controller/rl_optimized_mode_", static_cast<double>(plannedMode_));
    if (diff_time > 0.00005 || is_init)
    {
      Eigen::VectorXd updated_joint_pos = jointPos_;
      Eigen::VectorXd updated_joint_vel = jointVel_;
      Eigen::VectorXd updated_joint_current = jointCurrent_;
#ifdef KUAVO_CONTROL_LIB_FOUND
      if (use_joint_filter_)
      {
        joint_filter_ptr_->update(measuredRbdState_, updated_joint_pos, updated_joint_vel, updated_joint_current, output_tau_, plannedMode_);
        printf("use_joint_filter_!!\n");
        exit(0);
      }
#endif
      stateEstimate_->updateJointStates(updated_joint_pos, updated_joint_vel); // 使用关节滤波之后的jointPos和jointVel更新状态估计器
      contactTrotgait_ = stateEstimate_->updateKinematics(current_time_, period);
      measuredRbdState_ = stateEstimate_->update(time, period);                // angle(zyx),pos(xyz),jointPos[info_.actuatedDofNum],angularVel(zyx),linervel(xyz),jointVel[info_.actuatedDofNum]
      currentObservation_.time += period.toSec();
    }
    setRobotState(measuredRbdState_);
    ros_logger_->publishVector("/state_estimate/measuredRbdState", measuredRbdState_);
    ros_logger_->publishVector("/state_estimate/measuredRbdState/angle_zyx", measuredRbdState_.segment(0, 3));
    ros_logger_->publishVector("/state_estimate/measuredRbdState/pos_xyz", measuredRbdState_.segment(3, 3));
    auto &info = HumanoidInterface_->getCentroidalModelInfo();
    ros_logger_->publishVector("/state_estimate/measuredRbdState/joint_pos", measuredRbdState_.segment(6, info.actuatedDofNum));
    ros_logger_->publishVector("/state_estimate/measuredRbdState/angular_vel_xyz", measuredRbdState_.segment(6 + info.actuatedDofNum, 3));
    ros_logger_->publishVector("/state_estimate/measuredRbdState/linear_vel_xyz", measuredRbdState_.segment(9 + info.actuatedDofNum, 3));
    ros_logger_->publishVector("/state_estimate/measuredRbdState/joint_vel", measuredRbdState_.tail(info.actuatedDofNum));
    scalar_t yawLast = currentObservation_.state(9);
    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
    currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
    std_msgs::Float32MultiArray state;
    for (int i1 = 0; i1 < currentObservation_.state.rows(); ++i1)
    {
      state.data.push_back(currentObservation_.state(i1));
    }
    currentObservation_.mode = plannedMode_;
    publishHumanoidState(measuredRbdState_);
  }
  
  void humanoidController::publishHumanoidState(const vector_t& rbdState)
  {
    humanoidState_ << rbdState.segment(3, 3), rbdState.segment(0, 3), rbdState.segment(6 + jointNum_, jointArmNum_);
    humanoidState_.head(2) *= -1; // 坐标系转换为跟kuavo-ros-control仓库一致
    humanoidState_(2) -= initialState_(8); // 减去默认的高度
    std_msgs::Float64MultiArray state_msg;
    state_msg.data.resize(humanoidState_.size());
    for (int i = 0; i < humanoidState_.size(); ++i)
    {
      state_msg.data[i] = humanoidState_(i);
    }
    humanoidStatePublisher_.publish(state_msg);
  }

  void humanoidController::inference_thread_func()
  {
    ros::Rate inference_rate(inferenceFrequency_);
    while (ros::ok())
    {
      const auto sensors_data = getRobotSensorData();
      const auto measuredRbdState_ = getRobotState();
      Eigen::VectorXd local_measure_state = measuredRbdState_;
      updateObservation(local_measure_state, sensors_data);
      inference();
      inference_rate.sleep();
    }
  }
  void humanoidController::inference()
  {
    infer_request_ = compiled_model_.create_infer_request();
    const auto input_port = compiled_model_.input();
    // 检查输入维度是否匹配
    const auto expected_input_shape = input_port.get_shape();
    const size_t expected_input_length = expected_input_shape[1]; // 假设形状为 [batch_size, input_dim]
    const size_t actual_input_length = networkInputData_.size();
    
    if (actual_input_length != expected_input_length)
    {
      std::cout << "❌ 神经网络输入维度错误！！" << std::endl;
      std::cout << "   期望输入维度: " << expected_input_length << std::endl;
      std::cout << "   实际输入维度: " << actual_input_length << std::endl;
      std::cout << "   输入形状: [";
      for (size_t i = 0; i < expected_input_shape.size(); ++i) {
        std::cout << expected_input_shape[i];
        if (i < expected_input_shape.size() - 1) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
      return;
    }
    
    // std::cout << "networkInputData_: " << networkInputData_.transpose() << std::endl;
    Eigen::VectorXf float_network_input = networkInputData_.cast<float>();
    ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), float_network_input.data());
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.start_async();
    infer_request_.wait();
    const auto output_tensor = infer_request_.get_output_tensor();
    const size_t output_buf_length = output_tensor.get_size();
    const auto output_buf = output_tensor.data<float>();
    const size_t expected_output_length = withArm_ ? jointNum_ + jointArmNum_ + waistNum_ : jointNum_;
    
    // 检查输出维度是否匹配
    if (output_buf_length != expected_output_length)
    {
      std::cout << "❌ 神经网络输出维度错误！！" << std::endl;
      std::cout << "   期望输出维度: " << expected_output_length << std::endl;
      std::cout << "   实际输出维度: " << output_buf_length << std::endl;
      std::cout << "   关节配置: jointNum_=" << jointNum_ 
                << ", jointArmNum_=" << jointArmNum_ 
                << ", waistNum_=" << waistNum_ << std::endl;
      std::cout << "   withArm_: " << (withArm_ ? "true" : "false") << std::endl;
      return;
    }
    Eigen::VectorXd actionTargetPos_(output_buf_length);
    {
      std::lock_guard<std::mutex> lock(action_mtx_);
      for (int i = 0; i < output_buf_length; ++i)
      {
        actions_[i] = output_buf[i];
        // actionTargetPos_[i] = output_buf[i] * actionScale_ + defaultJointPos_[i];
      }
      clip(actions_, clipActions_);

    // std::cout << "actions_: " << actions_.transpose() << std::endl;
    /*******************************************************************/
    // 生成sin曲线作为local_action
    // actions_ *= 0.0;
    // static double time_counter = 0.0;
    // time_counter += 1;  // 时间步长

    // for (int i = 1; i < 7; ++i) {
    //   // 为每个关节生成不同频率的sin曲线
    //   double frequency = 0.5 + i * 0.1;  // 不同关节不同频率
    //   double amplitude = 0.5;  // 振幅
    //   double phase = i * 0.3;  // 相位偏移
    //   actions_[i] = amplitude * sin(frequency * time_counter + phase);
    // }
    // for (int i = 0; i < 1; ++i) {
    //   // 为每个关节生成 sin曲线
    //   double frequency = 0.02;
    //   double amplitude = 0.5;  // 振幅
    //   actions_[i] = amplitude * sin(2 * 3.14 * frequency * time_counter);
    // }

      // double frequency = 0.04;
      // double amplitude = 0.3;  // 振幅
      // actions_[5] = amplitude * sin(2 * 3.14 * frequency * time_counter);
      // actions_[11] = amplitude * sin(2 * 3.14 * frequency * time_counter);
    /*******************************************************************/

      // static int cnt = 0;
      // cnt++;
      // static int action_id = 0;

      // double action_i = sin(cnt / 50. );
      // if (cnt / 50. >= M_PI * 2.) {
      //   cnt = 0;
      //   action_id = (action_id + 1) % 21;
      //   std::cout << "[HumanoidController::inference] switch action_id to " << action_id << std::endl;
      // }
      // std::cout << "actions_: " << actions_.transpose() << std::endl;
      // actions_.setZero();
      // actions_(action_id) = action_i * M_PI / 2.;
    /*******************************************************************/

      ros_logger_->publishVector("/rl_controller/actions", actions_);
      // ros_logger_->publishVector("/rl_controller/actionTargetPos", actionTargetPos_);
    }
  }
  void humanoidController::updatePhase(const CommandData &CommandData_)
  {
    float ratio = CommandData_.cmdVelLineX_ / velocityLimits_(0);
    double nearest_int_phase = std::round(phase_);
    double targetCycleTime = (ratio > switch_ratio_) ? cycleTime_short_ : cycleTime_;
    if (targetCycleTime != currentCycleTime_)
    {
      float temp_phase = episodeLength_ * dt_ / currentCycleTime_;
      episodeLength_ = int((targetCycleTime/dt_)*(temp_phase - int(temp_phase)));
      // episodeLength_ = targetCycleTime / dt_ / 2;   //将周期置为0.5的位置
    }
    double alpha = 1;
    currentCycleTime_ = (1.0 - alpha) * currentCycleTime_ + alpha * targetCycleTime;
    
    // 基于 episode 进度计算 phase
    phase_ = CommandData_.cmdStance_ == 1 ? 0.0 : episodeLength_ * dt_ / currentCycleTime_;

    commandPhase_(0) = sin(2 * M_PI * phase_);
    commandPhase_(1) = cos(2 * M_PI * phase_);
    rl_plannedMode_ = (commandPhase_(0) > 0) ? ModeNumber::SF : (commandPhase_(0) < 0) ? ModeNumber::FS
                                                                                       : ModeNumber::SS;
  }
  void humanoidController::updateObservation(const Eigen::VectorXd &state_est, const SensorData &sensor_data)
  {
    CommandData CommandData_;
    CommandData_ = getCommandData();
    updatePhase(CommandData_);

    // 提取速度命令 [vx, vy, omega_z]
    Eigen::Vector3d velocity_commands;
    velocity_commands << CommandData_.cmdVelLineX_,
                         CommandData_.cmdVelLineY_,
                         CommandData_.cmdVelAngularZ_;
    // int angularVelStart = 6 + adjNum_;  // 角速度开始位置
    // std::cout << "angular velocity start index: " << angularVelStart << "\n";
    
    // int linearVelStart = 9 + adjNum_;     // 线速度开始位置  
    // std::cout << "linear velocity start index: " << linearVelStart << "\n";
    
    // int jointVelStart = linearVelStart + 3;  // 关节速度开始位置
    // std::cout << "joint velocity start index: " << jointVelStart << "\n";
    
    const Eigen::Vector3d baseEuler(state_est(2), state_est(1), state_est(0));
    const Eigen::Vector3d baseAngVel(state_est(6 + waistNum_ + jointNum_ + jointArmNum_),
                                     state_est(6 + waistNum_ + jointNum_ + jointArmNum_ + 1),
                                     state_est(6 + waistNum_ + jointNum_ + jointArmNum_ + 2));
    const Eigen::Vector3d baseLineVel = state_est.segment(9 + waistNum_ + jointNum_ + jointArmNum_, 3);
    // const Eigen::Vector3d basePos = state_est.segment(3, 3);
    // Extract and process sensor data
    Eigen::VectorXd jointPos = sensor_data.jointPos_ - defaultJointPos_;
    Eigen::VectorXd jointVel = sensor_data.jointVel_ ;
    Eigen::VectorXd jointTorque = sensor_data.jointCurrent_;
    Eigen::Vector3d bodyAngVel = sensor_data.angularVel_;
    const Eigen::Vector3d &bodyLineAcc = sensor_data.linearAccel_;
    const Eigen::Vector3d &bodyLineFreeAcc = sensor_data.freeLinearAccel_;
    // Normalize joint torques
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; ++i)
    {
      jointTorque[i] /= torqueLimits_[i];
    }
    // Transform base linear velocity
    auto quat_offset_ = Eigen::AngleAxisd(-my_yaw_offset_, Vector3d::UnitZ()) * sensor_data.quat_;
    const Eigen::Matrix3d R = quat_offset_.matrix();
    const Eigen::Vector3d bodyLineVel = R.transpose() * baseLineVel;
    
    Eigen::Vector3d currentBasePos = sensor_data.pos_w_;
    Eigen::Quaterniond currentBaseQuat = Eigen::Quaterniond(quat_offset_.w(),
                                                           quat_offset_.x(),
                                                           quat_offset_.y(),
                                                           quat_offset_.z());
    // std::cout << "quat_offset_: " << quat_offset_.w() << " " 
    //                                     << quat_offset_.x() << " " 
    //                                     << quat_offset_.y() << " " 
    //                                     << quat_offset_.z() << std::endl;
    // std::cout << "sensor_data.quat_: " << sensor_data.quat_.w() << " " 
    //                                     << sensor_data.quat_.x() << " " 
    //                                     << sensor_data.quat_.y() << " " 
    //                                     << sensor_data.quat_.z() << std::endl;
    const Eigen::Vector3d gravity_world(0, 0, -1);
    const Eigen::Vector3d projected_gravity = R.transpose() * gravity_world;
    Eigen::VectorXd local_action;
    {
      std::lock_guard<std::mutex> lock(action_mtx_);
      local_action = actions_;
    }
    // Print raw trajectory data in real-time
    static int print_counter = 0;
    // if (print_counter % 50 == 0) { // Print every 100 iterations (~0.2s at 500Hz)
        // std::cout << "\n=== Raw Trajectory Data ===" << std::endl;
        // std::cout << "Time step: " << motionTrajectory_.current_time_step << "/" << motionTrajectory_.time_step_total << std::endl;
        // std::cout << "Raw target position: " << rawTrajectoryPos.transpose() << std::endl;
        // std::cout << "Raw target quaternion: [w=" << rawTrajectoryQuat.w()
        //           << ",x=" << rawTrajectoryQuat.x() << ", y=" << rawTrajectoryQuat.y() << ", z=" << rawTrajectoryQuat.z() << "]" << std::endl;
        // std::cout << "Current robot position: " << currentBasePos.transpose() << std::endl;
        // std::cout << "Current robot quaternion: [wxyz=" << currentBaseQuat.w() << "," << currentBaseQuat.x() << "," << currentBaseQuat.y() << "," << currentBaseQuat.z() << "]" << std::endl;
        // std::cout << "sensorjointPos: " << sensor_data.jointPos_.transpose() << std::endl;
        // std::cout << "sensorjointVel: " << sensor_data.jointVel_.transpose() << std::endl;
        // std::cout << "actions_: " << actions_.transpose() << std::endl;
        // std::cout << "--- Observation Data ---" << std::endl;
        // std::cout << "projected_gravity: " << projected_gravity.transpose() << std::endl;
        // std::cout << "bodyAngVel: " << bodyAngVel.transpose() << std::endl;
        // std::cout << "jointPos: " << jointPos.transpose() << std::endl;
        // std::cout << "joint_vel (first 6): " << jointVel.head(6).transpose() << std::endl;
        // std::cout << "local_action: " << local_action.transpose() << std::endl;
        // std::cout << "==============================\n" << std::endl;
        // if (print_counter > 30){
        //   exit(0);
        // }
    // }
    // print_counter++;

    // Populate singleInputDataMap_
    const std::map<std::string, Eigen::VectorXd> singleInputDataMap_ = {
        {"base_ang_vel", bodyAngVel},
        {"projected_gravity", projected_gravity},
        {"velocity_commands", velocity_commands},
        {"joint_pos", jointPos},
        {"joint_vel", jointVel},
        {"actions", local_action}};
    
    // Fill singleInputData
    // int index = 0;
    // for (const auto &key : singleInputDataKeys)
    // {
    //   const auto &value = singleInputDataID_[key];
    //   singleInputData.segment(index, value[1]) = singleInputDataMap_.at(key).segment(value[0], value[1]) * value[2];
    //   index += value[1];
    // }

    int index = 0;
    for (const auto &key : singleInputDataKeys) {
      const auto &value = singleInputDataID_[key];
      for (int i=0; i<frameStack_; i++) {
        // std::cout << "value[0] " << value[0] << "value[1]" << value[1] << "key" << key <<  std::endl; 
        networkInputData_.segment(index*frameStack_ + value[1]*i, value[1]) = singleInputDataMap_.at(key).segment(value[0], value[1]) * value[2];
      }
      index += value[1];
      ros_logger_->publishVector("/rl_controller/InputData/" + key, singleInputDataMap_.at(key).segment(value[0], value[1]) * value[2]);

    }
    singleInputData = networkInputData_;
    // Publish data
    ros_logger_->publishVector("/rl_controller/singleInputData", singleInputData);
  }
  Eigen::VectorXd humanoidController::updateRLcmd(const vector_t &state)
  {
    Eigen::VectorXd jointPos_ = getRobotSensorData().jointPos_;
    Eigen::VectorXd jointVel_ = getRobotSensorData().jointVel_;
    Eigen::VectorXd actuation(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd cmd_out(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd cmd(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd cmd_filter(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd torque(jointNum_ + jointArmNum_ + waistNum_);
    // const Eigen::VectorXd jointPos_ = state.segment(6, jointNum_ + jointArmNum_ + waistNum_);
    // const Eigen::VectorXd jointVel_ = state.tail(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd motorPos_ = jointPos_;
    Eigen::VectorXd motorVel_ = jointVel_;
    Eigen::VectorXd local_action;
    {
      std::lock_guard<std::mutex> lock(action_mtx_);
      local_action = actions_;
    }
    /*******************************************************************/
    // 生成sin曲线作为local_action
    // local_action *= 0.0;
    // static double time_counter = 0.0;
    // time_counter += 1;  // 时间步长
    
    // for (int i = 1; i < 7; ++i) {
    //   // 为每个关节生成不同频率的sin曲线
    //   double frequency = 0.5 + i * 0.1;  // 不同关节不同频率
    //   double amplitude = 0.5;  // 振幅
    //   double phase = i * 0.3;  // 相位偏移
    //   actions_[i] = amplitude * sin(frequency * time_counter + phase);
    // }
    // for (int i = 0; i < 1; ++i) {
    //   // 为每个关节生成 sin曲线
    //   double frequency = 0.02;
    //   double amplitude = 0.5;  // 振幅
    //   actions_[i] = amplitude * sin(2 * 3.14 * frequency * time_counter);
    // }

      // double frequency = 0.0005;
      // double amplitude = 0.3;  // 振幅
      // local_action[3] = amplitude * sin(2 * 3.14 * frequency * time_counter);
      // local_action[9] = amplitude * sin(2 * 3.14 * frequency * time_counter);
    /*******************************************************************/ 
    // std::cout << "Generated sin curve local_action: " << local_action.transpose() << std::endl;
    if (!withArm_)
    {
      local_action.tail(jointArmNum_ + waistNum_).setZero();
    }
    Eigen::VectorXd jointTor_(jointNum_ + jointArmNum_ + waistNum_);
    if(robot_version.version_number() == 11 || robot_version.version_number() == 13 || robot_version.version_number() == 14) {
      motorPos_.segment(waistNum_, jointNum_) = ankleSolver.joint_to_motor_position(jointPos_.segment(waistNum_, jointNum_));
      motorVel_.segment(waistNum_, jointNum_) = ankleSolver.joint_to_motor_velocity(jointPos_.segment(waistNum_, jointNum_), motorPos_, jointVel_.segment(waistNum_, jointNum_));
      jointTor_ = -(jointKd_.cwiseProduct(motorVel_));
      jointTor_.segment(waistNum_, jointNum_) = ankleSolver.motor_to_joint_torque(jointPos_.segment(waistNum_, jointNum_), motorPos_.segment(waistNum_, jointNum_), jointTor_.segment(waistNum_, jointNum_));
    } else {
      motorPos_.head(jointNum_) = ankleSolver.joint_to_motor_position(jointPos_.head(jointNum_));
      motorVel_.head(jointNum_) = ankleSolver.joint_to_motor_velocity(jointPos_.head(jointNum_), motorPos_.head(jointNum_), jointVel_.head(jointNum_));
      jointTor_ = -(jointKd_.cwiseProduct(motorVel_));
      jointTor_.head(jointNum_) = ankleSolver.motor_to_joint_torque(jointPos_.head(jointNum_), motorPos_.head(jointNum_), jointTor_.head(jointNum_));
    }
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
    {
      jointTor_(i) = jointTor_(i) + jointKp_(i) * (local_action[i] * actionScale_ * actionScaleTest_[i] - jointPos_[i] + defaultJointPos_[i]);
    }
    if (is_real_)
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
      {
        if (JointControlMode_(i) == 0)
        {
          if (JointPDMode_(i) == 0)
          {
            cmd[i] = jointKp_[i] * (local_action[i] * actionScale_ * actionScaleTest_[i] - jointPos_[i] + defaultJointPos_[i]) - jointKd_[i] * jointVel_[i];
            cmd[i] = std::clamp(cmd[i], -torqueLimits_[i], torqueLimits_[i]);
            torque[i] = cmd[i];
          }
          else
          {
            cmd[i] = (local_action[i] * actionScale_ * actionScaleTest_[i] + defaultJointPos_[i]);
            torque[i] = jointKp_[i] * (local_action[i] * actionScale_ * actionScaleTest_[i] - jointPos_[i] + defaultJointPos_[i]) - jointKd_[i] * jointVel_[i];
          }
        }
        else if (JointControlMode_(i) == 2)
        {
          cmd[i] = jointKp_[i] * (local_action[i] * actionScale_ * actionScaleTest_[i] - jointPos_[i] + defaultJointPos_[i]);
          // cmd[i] = local_action[i] + defaultJointPos_[i];
          // std::cout << "cmd[" << i << "] = " << cmd[i] << "jointKp_:" << jointKp_[i] << std::endl;
          // cmd[i] = defaultJointPos_[i];
          torque[i] = jointTor_[i];
        }
      }
      // std::cout << "local_action: " << local_action.transpose() << std::endl;
    }
    else
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
      {
        if (JointControlMode_(i) == 0)
        {
          cmd[i] = jointKp_[i] * (local_action[i] * actionScale_ * actionScaleTest_[i] - jointPos_[i] + defaultJointPos_[i]) - jointKd_[i] * jointVel_[i];
        }
        else if (JointControlMode_(i) == 2)
        {
          cmd[i] = jointTor_[i];
        }
        cmd[i] = std::clamp(cmd[i], -torqueLimits_[i], torqueLimits_[i]);
        // torque[i] = cmd[i];
      }
      // static int count = 0;
      // if (count % 250 == 0) {
      //   std::cout << "cmd: [";
      //   for (size_t i = 0; i < cmd.size(); i++) {
      //       std::cout << cmd[i];
      //       if (i < cmd.size() - 1) std::cout << ", ";
      //   }
      //   std::cout << "]" << std::endl;
      // } 
      // if (count % 250 == 0) {
      //   std::cout << "actionScaleTest_: [";
      //   for (size_t i = 0; i < actionScaleTest_.size(); i++) {
      //       std::cout << actionScaleTest_[i];
      //       if (i < actionScaleTest_.size() - 1) std::cout << ", ";
      //   }
      //   std::cout << "]" << std::endl;
      // } 
      // if (count % 250 == 0) {
      //   std::cout << "ddjointVel: [";
      //   for (size_t i = 0; i < ddjointVel.size(); i++) {
      //       std::cout << ddjointVel[i];
      //       if (i < ddjointVel.size() - 1) std::cout << ", ";
      //   }
      //   std::cout << "]" << std::endl;
      // } 
      // if (count % 250 == 0) {
      //   std::cout << "ddjointPos: [";
      //   for (size_t i = 0; i < ddjointPos.size(); i++) {
      //       std::cout << ddjointPos[i];
      //       if (i < ddjointPos.size() - 1) std::cout << ", ";
      //   }
      //   std::cout << "]" << std::endl;
      // } 
      // if (count % 250 == 0) {
      //   std::cout << "cmd: [";
      //   for (size_t i = 0; i < cmd.size(); i++) {
      //       std::cout << cmd[i];
      //       if (i < cmd.size() - 1) std::cout << ", ";
      //   }
      //   std::cout << "]" << std::endl;

      // } 
      // count++;
    }
    actuation = cmd;
    episodeLength_++;
    ros_logger_->publishVector("/rl_controller/torque", torque);
    ros_logger_->publishVector("/rl_controller/cmd", cmd);
    return actuation;
  }

  void humanoidController::clip(Eigen::VectorXd &a, double limit)
  {
    a = a.cwiseMax(-limit).cwiseMin(limit);
  }

  humanoidController::~humanoidController()
  {
    controllerRunning_ = false;
    // 等待并清理键盘线程
    if (keyboardThread_.joinable())
    {
      ROS_INFO("Waiting for keyboard thread to finish...");
      try {
        keyboardThread_.join();
        ROS_INFO("Keyboard thread joined successfully");
      } catch (const std::system_error& e) {
        ROS_ERROR("Error joining keyboard thread: %s", e.what());
      }
    }
    
    // 等待并清理推理线程
    if (inferenceThread_.joinable())
    {
      ROS_INFO("Waiting for inference thread to finish...");
      try {
        inferenceThread_.join();
        ROS_INFO("Inference thread joined successfully");
      } catch (const std::system_error& e) {
        ROS_ERROR("Error joining inference thread: %s", e.what());
      }
    }
  }

  void humanoidController::setupHumanoidInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, const std::string &gaitCommandFile,
                                                  bool verbose, int robot_version_int)
  {
    HumanoidInterface_ = std::make_shared<HumanoidInterface>(taskFile, urdfFile, referenceFile, gaitCommandFile, robot_version_int);
    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(HumanoidInterface_->getPinocchioInterface(),
                                                                      HumanoidInterface_->getCentroidalModelInfo());
  }

  void humanoidController::setupStateEstimate(const std::string &taskFile, bool verbose)
  {
    stateEstimate_ = std::make_shared<KalmanFilterEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose);
    currentObservation_.time = 0;
  }

  void humanoidCheaterController::setupStateEstimate(const std::string & /*taskFile*/, bool /*verbose*/)
  {
    stateEstimate_ = std::make_shared<FromTopicStateEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                              HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controllerNh_);
  }

  void humanoidKuavoController::setupStateEstimate(const std::string &taskFile, bool verbose)
  {
#ifdef KUAVO_CONTROL_LIB_FOUND
    // auto [plant, context] = drake_interface_->getPlantAndContext();
    stateEstimate_ = std::make_shared<InEkfBaseFilter>(HumanoidInterface_->getPinocchioInterface(),
                                                       HumanoidInterface_->getCentroidalModelInfo(),
                                                       *eeKinematicsPtr_,
                                                       drake_interface_,
                                                       dt_,
                                                       ros_logger_);
    std::cout << "InEkfBaseFilter stateEstimate_ initialized" << std::endl;
#endif
  }

  bool humanoidController:: WalkenableCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    if (req.data)
    {
      // 启用RL控制器和行走功能
    //   is_rl_controller_ = true;
      Walkenable_ = true;
      cmdTrotgait_ = false;
      
      res.success = true;
      res.message = "RL controller enabled and walking enabled";
      
      ROS_INFO("[RL-Control Service] RL controller enabled and walking enabled");
      printRLparam();
    }
    else
    {
    //   is_rl_controller_ = true;
      Walkenable_ = false;
      
      CommandData cmdData = getCommandData();
      cmdData.setzero();
      cmdData.cmdStance_ = 1;
      setCommandData(cmdData);
      
      res.success = true;
      res.message = "walking disabled, RL controller is still running, switched to stance mode";
      
      ROS_INFO("[RL-Control Service] RL controller is still running, walking disabled, switched to stance mode");
    }
    
    return true;
  }
} // namespace humanoid_controller
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidController)
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidCheaterController)
