#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "humanoid_controllers/humanoidWheelController.h"
#include <iostream>
#include <cmath>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>


namespace humanoid_wheel_controller
{
  using namespace ocs2;
  using Duration = std::chrono::duration<double>;
  using Clock = std::chrono::high_resolution_clock;

  static void callSimStartSrv(ros::NodeHandle &nh_)
  {
    std_srvs::SetBool srv;
    srv.request.data = true;

    // 等待服务可用
    std::cout << "Waiting for sim_start service..." << std::endl;
    bool service_available = ros::service::waitForService("sim_start", ros::Duration(100.0)); // 5秒超时

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

  void humanoidWheelController::sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg)
  {
    SensorData sensor_data;
    auto &joint_data = msg->joint_data;
    auto &imu_data = msg->imu_data;
    int num_joint = lowJointNum_ + armNum_ + 2;
    sensor_data.resize_joint(num_joint);
    // 关节数据
    for(int i=0; i < num_joint; i++)
    {
      sensor_data.jointPos_(i) = joint_data.joint_q[i];
      sensor_data.jointVel_(i) = joint_data.joint_v[i];
      sensor_data.jointAcc_(i) = joint_data.joint_vd[i];
      sensor_data.jointTorque_(i) = joint_data.joint_torque[i];
    }
    // IMU
    sensor_data.quat_.coeffs().w() = imu_data.quat.w;
    sensor_data.quat_.coeffs().x() = imu_data.quat.x;
    sensor_data.quat_.coeffs().y() = imu_data.quat.y;
    sensor_data.quat_.coeffs().z() = imu_data.quat.z;
    sensor_data.angularVel_ << imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z;
    sensor_data.linearAccel_ << imu_data.acc.x, imu_data.acc.y, imu_data.acc.z;
    sensor_data.orientationCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.angularVelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.linearAccelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    // 对齐时间戳
    sensor_data.timeStamp_ = msg->sensor_time;
    // 输入到 buffer 等待读出
    sensors_data_buffer_ptr_->addData(sensor_data.timeStamp_.toSec(), sensor_data);
  }

  double rosQuaternionToYaw(const geometry_msgs::Quaternion& ros_quat) {
      // 将ROS四元数转换为Eigen四元数
      Eigen::Quaterniond eigen_quat(
          ros_quat.w,
          ros_quat.x,
          ros_quat.y,
          ros_quat.z
      );

      // 转换为yaw角
      Eigen::Matrix3d R = eigen_quat.toRotationMatrix();
      return std::atan2(R(1, 0), R(0, 0));   // 结果在 (−π, π];
  }

  void humanoidWheelController::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    odom_mtx_.lock();
    odomData_[0] = msg->pose.pose.position.x;
    odomData_[1] = msg->pose.pose.position.y;
    odomData_[2] = rosQuaternionToYaw(msg->pose.pose.orientation);
    odomData_[3] = msg->twist.twist.linear.x;
    odomData_[4] = msg->twist.twist.linear.y;
    odomData_[5] = msg->twist.twist.angular.z;
    odom_mtx_.unlock();
  }

  bool humanoidWheelController::init(ros::NodeHandle &controller_nh, bool is_nodelet_node)
  {
    controllerNh_ = controller_nh;
    ros_logger_ = new humanoid::TopicLogger(controllerNh_);
    /************** Initialize OCS2 *********************/
    std::string taskFile;
    std::string libFolder;
    std::string urdfFile;
    bool verbose = true;

    controllerNh_.getParam("/taskFile", taskFile);
    controllerNh_.getParam("/libFolder", libFolder);
    controllerNh_.getParam("/urdfFile", urdfFile);

    setupHumanoidWheelInterface(taskFile, libFolder, urdfFile);

    observation_wheel_.state.setZero(manipulatorModelInfo_.stateDim);
    observation_wheel_.input.setZero(manipulatorModelInfo_.inputDim);
    observation_wheel_.time = 0;
    observation_wheel_.mode = 0;
    /****************************************************/
    /************load param from task.info***************/
    loadData::loadCppDataType(taskFile, "model_settings.verbose", verbose);
    loadData::loadCppDataType(taskFile, "model_settings.mpcArmsDof", armNum_);
    lowJointNum_ = manipulatorModelInfo_.armDim - armNum_;
    /****************************************************/
    /************** Initialize WBC **********************/
    wheel_wbc_ = std::make_shared<mobile_manipulator::WeightedWbc>(*pinocchioInterface_ptr_, manipulatorModelInfo_);
    wheel_wbc_->setArmNums(armNum_);
    wheel_wbc_->loadTasksSetting(taskFile, verbose, is_real_);
    /****************************************************/

    if(controllerNh_.hasParam("/real"))
    {
      controllerNh_.getParam("/real", is_real_);
    }
    double controlFrequency = 500.0; // 1000Hz
    controllerNh_.getParam("/wbc_frequency", controlFrequency);
    dt_ = 1.0 / controlFrequency;

    /*************底盘插补参数设置**********************/
    int vel_num = 3;
    velLimiter_ = std::make_shared<mobile_manipulator::VelocityLimiter>(vel_num);
    Eigen::VectorXd max_acceleration, max_deceleration;
    max_acceleration.setZero(vel_num);
    max_deceleration.setZero(vel_num);
    max_acceleration << 0.3, 0.3, 0.3;  //x, y, yaw 顺序加速度
    max_deceleration << 0.3, 0.3, 0.3;  // 减速度
    velLimiter_->setAccelerationLimits(max_acceleration, max_deceleration);
    velLimiter_->setAccelerationDt(dt_);
    /****************************************************/

    // 浮动基 7 + 全向轮 8 + 底盘下肢电机 4 + 双臂 7*2 + 头部 2
    vector_t mujoco_q = vector_t::Zero(7 + 8 + 4 + 7*2 + 2);
    mujoco_q[2] = 0.195;
    std::vector<double> robot_init_state_param;
    for (int i = 0; i < mujoco_q.size(); i++)
    {
      robot_init_state_param.push_back(mujoco_q(i));
    }
    size_t buffer_size = 5;
    sensors_data_buffer_ptr_ = new KuavoDataBuffer<SensorData>("humanoid_sensors_data_buffer", buffer_size, dt_);

    ros::param::set("robot_init_state_param", robot_init_state_param);
    cmdVelPub_ = controllerNh_.advertise<geometry_msgs::Twist>("/filter_cmd_vel", 10, true);
    jointCmdPub_ = controllerNh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
    odomSub_ = controllerNh_.subscribe<nav_msgs::Odometry>("/odom", 10, &humanoidWheelController::odomCallback, this);
    sensorsDataSub_ = controllerNh_.subscribe<kuavo_msgs::sensorsData>("/sensors_data_raw", 10, &humanoidWheelController::sensorsDataCallback, this);

    sensors_data_buffer_ptr_->waitForReady();

    return true;
  }

  void humanoidWheelController::starting(const ros::Time &time)
  {
    if (!is_real_)
      callSimStartSrv(controllerNh_);
    
    sensors_data_buffer_ptr_->sync();
  }

  bool humanoidWheelController::preUpdate(const ros::Time &time)
  {
    static int cnt;
    std::cout << "preUpdate is running !" << std::endl;
    cnt++;
    bool bIsHardwareReady{false};
    if(controllerNh_.hasParam("/hardware/is_ready"))
    {
      controllerNh_.getParam("/hardware/is_ready", bIsHardwareReady);
    }
    if(!bIsHardwareReady)
    {
      return false;
    }
    std::cout << "Hardware is ready." << std::endl;
    // 获取关节数据，并更新 Observation
    SensorData sensors_data_new;
    sensors_data_new = sensors_data_buffer_ptr_->getLastData();
    odom_mtx_.lock();
    vector6_t odomData_new = odomData_;
    odom_mtx_.unlock();
    computeObservationFromSensorData(sensors_data_new, odomData_new);
    if(cnt == 5)
    {
      setupMrt();
      initMPC();
      isPreUpdateComplete = true;
    }
    return true;
  }

  void humanoidWheelController::update(const ros::Time &time, const ros::Duration &dfd)
  {
    static auto timeInit = time.toSec();
    auto& info = manipulatorModelInfo_;
    static int cnt = 0;
    if(cnt % 500 == 0)
    {
      std::cout << "update is running, time is " << time.toSec() - timeInit << std::endl;
    }
    if(reset_mpc_) // 重置mpc
    {
      // use pinocchio 
      std::vector<Eigen::Vector3d> init_ee_pos(info.eeFrames.size());
      std::vector<Eigen::Matrix3d> init_ee_rot(info.eeFrames.size());
      getEEPose(observation_wheel_.state, init_ee_pos, init_ee_rot);

      vector_t initTarget(info.eeFrames.size() * 7);
      for(int eef_inx = 0; eef_inx < info.eeFrames.size(); eef_inx++)
      {
        initTarget.segment(eef_inx*7, 3) = init_ee_pos[eef_inx];
        initTarget.segment(eef_inx*7+3, 4) = Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
      }
      auto target_trajectories = TargetTrajectories({observation_wheel_.time}, 
                                                    {initTarget}, 
                                                    {observation_wheel_.input});
      mrtRosInterface_->resetMpcNode(target_trajectories);
      reset_mpc_ = false;
      std::cout << "reset MPC node at " << observation_wheel_.time << "\n";
    }
    // 获取关节数据，并更新 Observation
    SensorData sensors_data_new;
    sensors_data_new = sensors_data_buffer_ptr_->getLastData();
    odom_mtx_.lock();
    vector6_t odomData_new = odomData_;
    odom_mtx_.unlock();
    computeObservationFromSensorData(sensors_data_new, odomData_new);

    // 更新 mpc 数据
    {
      vector_t optimizedState_mrt, optimizedInput_mrt;
      // Update the current state of the system
      mrtRosInterface_->setCurrentObservation(observation_wheel_);

      // Trigger MRT callbacks
      mrtRosInterface_->spinMRT();
      // Update the policy if a new on was received
      if (mrtRosInterface_->updatePolicy())
      {
      }

      mrtRosInterface_->evaluatePolicy(observation_wheel_.time, observation_wheel_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
      if(enable_mpc_)
      {
        optimizedState_mrt_ = optimizedState_mrt;
        optimizedInput_mrt_ = optimizedInput_mrt;
      }
      if(std::fabs(optimizedInput_mrt_[0]) < 0.01) optimizedInput_mrt_[0] = 0;
      if(std::fabs(optimizedInput_mrt_[1]) < 0.01) optimizedInput_mrt_[1] = 0;
    }
    // 更新可视化数据
    // robotVisualizer_->update_obs(observation_wheel_);
    robotVisualizer_->update(observation_wheel_, mrtRosInterface_->getPolicy(), mrtRosInterface_->getCommand());

    vector_t x = wheel_wbc_->update(optimizedState_mrt_, optimizedInput_mrt_, observation_wheel_);

    vector_t bodyAcc = x.head(info.stateDim-info.armDim);
    vector_t jointAcc = x.segment(info.stateDim-info.armDim, info.armDim);
    vector_t torque = x.tail(info.armDim);

    ros_logger_->publishVector("/humanoid_wheel/bodyAcc", bodyAcc);
    ros_logger_->publishVector("/humanoid_wheel/jointAcc", jointAcc);
    ros_logger_->publishVector("/humanoid_wheel/torque", torque);

    // 更新关节指令
    kuavo_msgs::jointCmd jointCmdMsg;
    for (int i1 = 0; i1 < lowJointNum_; ++i1)
    {
      jointCmdMsg.joint_q.push_back(0);
      jointCmdMsg.joint_v.push_back(0);
      jointCmdMsg.tau.push_back(torque.head(lowJointNum_)[i1]);
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
      jointCmdMsg.tau_max.push_back(0);
      jointCmdMsg.control_modes.push_back(2);
    }
    for (int i2 = 0; i2 < armNum_; ++i2)
    {
      jointCmdMsg.joint_q.push_back(0);
      jointCmdMsg.joint_v.push_back(0);
      jointCmdMsg.tau.push_back(torque.tail(14)[i2]);
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
      jointCmdMsg.tau_max.push_back(0);
      jointCmdMsg.control_modes.push_back(2);
    }
    for (int i3 = 0; i3 < 2; ++i3)
    {
      jointCmdMsg.joint_q.push_back(0);
      jointCmdMsg.joint_v.push_back(0);
      jointCmdMsg.tau.push_back(0);
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.tau_max.push_back(10);
      jointCmdMsg.control_modes.push_back(2);
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
    }
    jointCmdPub_.publish(jointCmdMsg);
    
    // 更新底盘速度
    geometry_msgs::Twist velCmdMsg;
    cmd_vel_[0] = optimizedInput_mrt_[0];
    cmd_vel_[1] = optimizedInput_mrt_[1];
    cmd_vel_[2] = optimizedInput_mrt_[2];
    cmd_vel_ = velLimiter_->limitAcceleration(cmd_vel_);  // 进行梯形插补

    Eigen::Vector3d cmdVelBody = cmdVelWorldToBody(cmd_vel_, observation_wheel_.state[2]);

    velCmdMsg.linear.x = cmdVelBody[0];
    velCmdMsg.linear.y = cmdVelBody[1];
    velCmdMsg.linear.z = 0;
    velCmdMsg.angular.x = 0;
    velCmdMsg.angular.y = 0;
    velCmdMsg.angular.z = cmdVelBody[2];
    cmdVelPub_.publish(velCmdMsg);

    cnt++;
  }

  humanoidWheelController::~humanoidWheelController()
  {
  }

  void humanoidWheelController::setupHumanoidWheelInterface(const std::string &taskFile, const std::string &libFolder, const std::string &urdfFile)
  {
    HumanoidWheelInterface_ = std::make_shared<mobile_manipulator::HumanoidWheelInterface>(taskFile, libFolder, urdfFile);
    manipulatorModelInfo_ = HumanoidWheelInterface_->getManipulatorModelInfo();
    pinocchioInterface_ptr_ = new PinocchioInterface(HumanoidWheelInterface_->getPinocchioInterface());
    robotVisualizer_ = std::make_shared<mobile_manipulator::MobileManipulatorDummyVisualization>(controllerNh_, *HumanoidWheelInterface_);

    std::cout << "info.stateDim " << manipulatorModelInfo_.stateDim << std::endl;
    std::cout << "info.inputDim " << manipulatorModelInfo_.inputDim << std::endl;
    std::cout << "info.armDim " << manipulatorModelInfo_.armDim << std::endl;
    std::cout << "info.baseFrame " << manipulatorModelInfo_.baseFrame << std::endl;
    std::cout << "info.eeFrame: ";
    for(int eef_inx = 0; eef_inx < manipulatorModelInfo_.eeFrames.size(); eef_inx++)
    {
      std::cout << manipulatorModelInfo_.eeFrames[eef_inx] << std::endl;
    }
    std::cout << "info.dofNames " << std::endl;
    for(int i=0; i<manipulatorModelInfo_.dofNames.size(); i++)
    {
      std::cout << manipulatorModelInfo_.dofNames[i] << std::endl;
    }
    std::cout << "info.manipulatorModelType " << static_cast<int>(manipulatorModelInfo_.manipulatorModelType) << std::endl;
    
  }

  void humanoidWheelController::computeObservationFromSensorData(const SensorData& sensorData, const vector6_t& odomData)
  {
    // obs 的顺序： 
    // state：世界系x, y里程计(2), 机器人的yaw角度(1)，关节角度(下肢，上肢)(4+7*2)
    // input: forward velocity(1)，turning velocity(1)，关节速度(下肢，上肢)(4+7*2)

    current_time_ = sensorData.timeStamp_;
    static bool firstRun = true;
    if(firstRun){
      last_time_ = current_time_;
      firstRun = false;
    }
    double diff_time = (current_time_ - last_time_).toSec();
    ros::Duration period = ros::Duration(diff_time);

    last_time_ = current_time_;

    Eigen::Vector3d velWorld = cmdVelBodyToWorld(Eigen::Vector3d(odomData[3], odomData[4], odomData[5]), 
                                                observation_wheel_.state[2]);

    observation_wheel_.state.head(3) = odomData.head(3);
    observation_wheel_.state.tail(4 + 7*2) = sensorData.jointPos_.head(4 + 7*2);
    observation_wheel_.input[0] = velWorld[0];
    observation_wheel_.input[1] = velWorld[1];
    observation_wheel_.input[2] = velWorld[2];
    observation_wheel_.input.tail(4 + 7*2) = sensorData.jointVel_.head(4 + 7*2);
    observation_wheel_.time += period.toSec();
  }

  void humanoidWheelController::setupMrt()
  {
    mrtRosInterface_ = std::make_shared<MRT_ROS_Interface>(robotName_);
    mrtRosInterface_->initRollout(&HumanoidWheelInterface_->getRollout());
    mrtRosInterface_->launchNodes(controllerNh_);
  }

  void humanoidWheelController::initMPC()
  {
    SystemObservation initial_observation = observation_wheel_;
    // initial_observation.state = initial_state;

    // use pinocchio 
    std::vector<Eigen::Vector3d> init_ee_pos(manipulatorModelInfo_.eeFrames.size());
    std::vector<Eigen::Matrix3d> init_ee_rot(manipulatorModelInfo_.eeFrames.size());
    getEEPose(initial_observation.state, init_ee_pos, init_ee_rot);

    Eigen::Vector3d init_torso_pos;
    Eigen::Matrix3d init_torso_rot;
    getTorsoPose(initial_observation.state, init_torso_pos, init_torso_rot);

    // initial command
    int base_nums = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
    vector_t initTarget(base_nums + 7 + manipulatorModelInfo_.eeFrames.size() * 7);
    initTarget.head(base_nums) = vector_t::Zero(base_nums);
    initTarget.segment(base_nums, 3) = init_torso_pos;
    initTarget.segment(base_nums+3, 4) = Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
    for(int eef_inx = 0; eef_inx < manipulatorModelInfo_.eeFrames.size(); eef_inx++)
    {
      initTarget.tail(manipulatorModelInfo_.eeFrames.size() * 7).segment(eef_inx*7, 3) = init_ee_pos[eef_inx];
      initTarget.tail(manipulatorModelInfo_.eeFrames.size() * 7).segment(eef_inx*7+3, 4) = Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
    }

    TargetTrajectories initial_target({initial_observation.time},
                                      {initTarget},
                                      {initial_observation.input});
    
    // Set the first observation and command and wait for optimization to finish
    ROS_INFO_STREAM("Waiting for the initial policy ...");

    // Reset MPC node
    mrtRosInterface_->resetMpcNode(initial_target);
    std::cout << "reset MPC node\n";

    // Wait for the initial policy
    while (!mrtRosInterface_->initialPolicyReceived() && ros::ok() && ros::master::check())
    {
      mrtRosInterface_->spinMRT();
      mrtRosInterface_->setCurrentObservation(initial_observation);
      ros::Rate(HumanoidWheelInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    }
  }

  void humanoidWheelController::getEEPose(const vector_t& init_q, std::vector<Eigen::Vector3d>& ee_pos, std::vector<Eigen::Matrix3d>& ee_rot)
  {
    if(ee_pos.size() > manipulatorModelInfo_.eeFrames.size())
    {
      throw std::invalid_argument("[getEEPose] ee_pos is out of range.");
    }
    if(ee_rot.size() > manipulatorModelInfo_.eeFrames.size())
    {
      throw std::invalid_argument("[getEEPose] ee_rot is out of range.");
    }
    auto model = pinocchioInterface_ptr_->getModel();
    auto data = pinocchioInterface_ptr_->getData();
    pinocchio::framesForwardKinematics(model, data, init_q);

    for(int eef_inx = 0; eef_inx < manipulatorModelInfo_.eeFrames.size(); eef_inx++)
    {
      int ee_id = model.getBodyId(manipulatorModelInfo_.eeFrames[eef_inx]);
      ee_pos[eef_inx] = data.oMf[ee_id].translation();
      ee_rot[eef_inx] = data.oMf[ee_id].rotation();
    }
  }

  void humanoidWheelController::getTorsoPose(const vector_t& init_q, Eigen::Vector3d& torso_pos, Eigen::Matrix3d& torso_rot)
  {
    auto model = pinocchioInterface_ptr_->getModel();
    auto data = pinocchioInterface_ptr_->getData();
    pinocchio::framesForwardKinematics(model, data, init_q);

    int torso_id = model.getBodyId(manipulatorModelInfo_.torsoFrame);
    torso_pos = data.oMf[torso_id].translation();
    torso_rot = data.oMf[torso_id].rotation();
  }

  Eigen::Vector3d humanoidWheelController::cmdVelWorldToBody(const Eigen::Vector3d& cmd_vel_world, double yaw)
  {
    Eigen::Matrix3d R_world_to_body;
    R_world_to_body << std::cos(-yaw), -std::sin(-yaw), 0,
                       std::sin(-yaw),  std::cos(-yaw), 0,
                       0,               0,              1;
    return R_world_to_body * cmd_vel_world;
  }

  Eigen::Vector3d humanoidWheelController::cmdVelBodyToWorld(const Eigen::Vector3d& cmd_vel_body, double yaw)
  {
    Eigen::Matrix3d R_body_to_world;
    R_body_to_world << std::cos(yaw), -std::sin(yaw), 0,
                       std::sin(yaw),  std::cos(yaw), 0,
                       0,               0,              1;
    return R_body_to_world * cmd_vel_body;
  }

} // namespace humanoid_wheel_controller

