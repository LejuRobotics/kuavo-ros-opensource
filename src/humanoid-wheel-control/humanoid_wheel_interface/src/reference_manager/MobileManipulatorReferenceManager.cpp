#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "humanoid_wheel_interface/reference_manager/MobileManipulatorReferenceManager.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

namespace ocs2 {
namespace mobile_manipulator {
  MobileManipulatorReferenceManager::MobileManipulatorReferenceManager(const ManipulatorModelInfo& info, const PinocchioInterface& pinocchioInterface)
  : ReferenceManager(TargetTrajectories(), ModeSchedule())
  , info_(info)
  , pinocchioInterface_(pinocchioInterface)
  , stateInputTargetTrajectories(TargetTrajectories({0}, {vector_t::Zero(info_.stateDim)}, {vector_t::Zero(info_.inputDim)}))
  , torsoTargetTrajectories(TargetTrajectories({0}, {vector_t::Zero(7)}, {vector_t::Zero(7)}))
  , eeTargetTrajectories(TargetTrajectories({0}, {vector_t::Zero(info_.eeFrames.size())}, {vector_t::Zero(info_.eeFrames.size())}))
  {
    baseDim_ = info_.stateDim-info_.armDim;
    left_arm_traj_pose_ = vector_t::Zero(7);  // x,y,z,qx,qy,qz,qw
    right_arm_traj_pose_ = vector_t::Zero(7); // x,y,z,qx,qy,qz,qw
    currentLeftArmPose_ = vector_t::Zero(7);  // x,y,z,qx,qy,qz,qw
    currentRightArmPose_ = vector_t::Zero(7); // x,y,z,qx,qy,qz,qw
    
    // 初始化相对位置缓存
    cachedLeftArmRelativePose_ = vector_t::Zero(7);   // 左臂相对躯干位姿
    cachedRightArmRelativePose_ = vector_t::Zero(7);  // 右臂相对躯干位姿
    cachedTorsoPose_ = vector_t::Zero(7);             // 躯干位姿
    relativePoseCached_ = false;                      // 未缓存

    left_arm_joint_traj_ = vector_t::Zero(7);
    right_arm_joint_traj_ = vector_t::Zero(7);
    
    // 初始化MPC控制模式为NoControl（完全接收上层下发的 TargetTrajectory, 其余话题无法接收）
    currentMpcControlMode_ = 2;  // NoControl

    // 手臂锁住位置初始化
    lockArmState.setZero(info_.armDim - 4);

    /*************底盘插补参数设置**********************/
    double dt = 1 / 50.0;
    int vel_num = 3;
    velLimiter_ = std::make_shared<mobile_manipulator::VelocityLimiter>(vel_num);
    Eigen::VectorXd max_acceleration, max_deceleration;
    max_acceleration.setZero(vel_num);
    max_deceleration.setZero(vel_num);
    max_acceleration << 0.3, 0.3, 0.3;  //x, y, yaw 顺序加速度
    max_deceleration << 0.3, 0.3, 0.3;  // 减速度
    velLimiter_->setAccelerationLimits(max_acceleration, max_deceleration);
    std::cout << "dt: " << dt << std::endl;
    velLimiter_->setAccelerationDt(dt);
    /****************************************************/

    // 注册日志记录器
    ros_logger_ = new humanoid::TopicLogger(nodeHandle_);

    // 从参数服务器中获取所需参数
    if(nodeHandle_.hasParam("/wheel_move_spd"))
    {
      nodeHandle_.getParam("/wheel_move_spd", wheel_move_spd_);
    }

    // 设置wheelReachThreshold默认值
    wheelReachThreshold_.setZero(3);
    wheelReachThreshold_ << 0.03, 0.03, 0.03;
    std::vector<double> threshold;
    if(nodeHandle_.hasParam("/wheel_reach_threshold"))
    {
      nodeHandle_.getParam("/wheel_reach_threshold", threshold);
      if(threshold.size() != 3)
      {
        ROS_WARN("wheel_reach_threshold size must be 3, using default values [0.03, 0.03, 0.03]");
        return;
      }

      bool isReachThValid{true};
      for(int i = 0; i < threshold.size(); i++)
      {
        if(threshold[i] < 0)
        {
          ROS_WARN("wheel_reach_threshold[%d] must be greater than 0, using default value", i);
          isReachThValid = false;
          break;
        }
      }

      if(isReachThValid)
      {
        wheelReachThreshold_ << threshold[0], threshold[1], threshold[2];
      }
    }
    
  }

  void MobileManipulatorReferenceManager::setupSubscriptions(std::string nodeHandleName)
  {

    // 设置服务服务器
    controlModeServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_mpc_control", 
                                                           &MobileManipulatorReferenceManager::controlModeService, this);
    getMpcControlModeServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_get_mpc_control_mode",
                                                           &MobileManipulatorReferenceManager::getMpcControlModeService, this);
                                          
    auto targetVelocityCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdvel_mtx_.lock();
      isCmdVelUpdated_ = true;
      cmdVel_[0] = msg->linear.x;
      cmdVel_[1] = msg->linear.y;
      cmdVel_[2] = msg->angular.z;
      cmdvel_mtx_.unlock();
    };
    targetVelocitySubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, targetVelocityCallback);
    
    auto targetPoseCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdPose_mtx_.lock();
      isCmdPoseUpdated_ = true;
      cmdPose_[0] = msg->linear.x;
      cmdPose_[1] = msg->linear.y;
      cmdPose_[2] = msg->angular.z;
      std::cout << "Received cmdPose: [" << cmdPose_[0] << ", " << cmdPose_[1] << ", " << cmdPose_[2] << std::endl;
      cmdPose_mtx_.unlock();
    };
    targetPoseSubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_pose_world", 1, targetPoseCallback);
    
    targetPoseFlagPub_ = nodeHandle_.advertise<std_msgs::Bool>("/cmd_pose_world_flag", 10, false);

    // 订阅双臂末端执行器位姿指令
    auto armEndEffectorCallback = [this](const kuavo_msgs::twoArmHandPoseCmd::ConstPtr &msg)
    {
      armPose_mtx_.lock();
      
      // 解析左臂位姿 (x,y,z,qx,qy,qz,qw)
      left_arm_traj_pose_ = vector_t::Zero(7);
      left_arm_traj_pose_[0] = msg->hand_poses.left_pose.pos_xyz[0];  // x
      left_arm_traj_pose_[1] = msg->hand_poses.left_pose.pos_xyz[1];  // y
      left_arm_traj_pose_[2] = msg->hand_poses.left_pose.pos_xyz[2];  // z
      left_arm_traj_pose_[3] = msg->hand_poses.left_pose.quat_xyzw[0]; // qx
      left_arm_traj_pose_[4] = msg->hand_poses.left_pose.quat_xyzw[1]; // qy
      left_arm_traj_pose_[5] = msg->hand_poses.left_pose.quat_xyzw[2]; // qz
      left_arm_traj_pose_[6] = msg->hand_poses.left_pose.quat_xyzw[3]; // qw
      
      // 解析右臂位姿 (x,y,z,qx,qy,qz,qw)
      right_arm_traj_pose_ = vector_t::Zero(7);
      right_arm_traj_pose_[0] = msg->hand_poses.right_pose.pos_xyz[0];  // x
      right_arm_traj_pose_[1] = msg->hand_poses.right_pose.pos_xyz[1];  // y
      right_arm_traj_pose_[2] = msg->hand_poses.right_pose.pos_xyz[2];  // z
      right_arm_traj_pose_[3] = msg->hand_poses.right_pose.quat_xyzw[0]; // qx
      right_arm_traj_pose_[4] = msg->hand_poses.right_pose.quat_xyzw[1]; // qy
      right_arm_traj_pose_[5] = msg->hand_poses.right_pose.quat_xyzw[2]; // qz
      right_arm_traj_pose_[6] = msg->hand_poses.right_pose.quat_xyzw[3]; // qw

      if(msg->hand_poses.left_pose.joint_angles.size() != 7 || msg->hand_poses.right_pose.joint_angles.size() != 7)
      {
        ROS_ERROR("Left or right arm joint angles size is not 7");
        return;
      }

      for (size_t i = 0; i < msg->hand_poses.left_pose.joint_angles.size(); ++i)
      {
        left_arm_joint_traj_[i] = msg->hand_poses.left_pose.joint_angles[i] * M_PI / 180.0; // 转换为弧度
        right_arm_joint_traj_[i] = msg->hand_poses.right_pose.joint_angles[i] * M_PI / 180.0; // 转换为弧度
      }

      desire_mode_ = msg->desire_mode;
      
      armPose_mtx_.unlock();
    };
    armEndEffectorSubscriber_ =
        nodeHandle_.subscribe<kuavo_msgs::twoArmHandPoseCmd>("/mm/two_arm_hand_pose_cmd", 1, armEndEffectorCallback);
    
    // 添加订阅/kuavo_arm_traj话题
    auto armJointTrajCallback = [this](const sensor_msgs::JointState::ConstPtr &msg)
    {
      armJoint_mtx_.lock();
      
      // 解析关节角度数据
      arm_joint_traj_ = vector_t::Zero(msg->position.size());
      for (size_t i = 0; i < msg->position.size(); ++i)
      {
        arm_joint_traj_[i] = msg->position[i] * M_PI / 180.0; // 转换为弧度
      }
      
      armJoint_mtx_.unlock();
    };
    arm_joint_traj_sub_ = nodeHandle_.subscribe<sensor_msgs::JointState>("/kuavo_arm_traj", 10, armJointTrajCallback);
  }

  // 获取第一次的目标轨迹，并分配到不同的约束轨迹，后续添加额外约束, 也需要在此初始化
  void MobileManipulatorReferenceManager::getFirstTargetTrajectories(const TargetTrajectories& targetTrajectories)
  {
    // 第一次的轨迹, 包括 state, input, 躯干相对底座位姿, 所有手臂末端位姿
    // stateInputTargetTrajectories 的状态不需要初始化, 全零且维度统一即可
    stateInputTargetTrajectories.timeTrajectory.front() = targetTrajectories.timeTrajectory.front();

    torsoTargetTrajectories.timeTrajectory.front() = targetTrajectories.timeTrajectory.front();
    torsoTargetTrajectories.stateTrajectory.front() = targetTrajectories.stateTrajectory.front().segment(baseDim_, 7);

    eeTargetTrajectories.timeTrajectory.front() = targetTrajectories.timeTrajectory.front();
    eeTargetTrajectories.stateTrajectory.front() = targetTrajectories.stateTrajectory.front().segment(baseDim_ + 7, info_.eeFrames.size() * 7);
  }

  // 获取所有的目标轨迹，并分配到不同的约束轨迹
  void MobileManipulatorReferenceManager::getAllTargetTrajectories(const TargetTrajectories& targetTrajectories)
  {

    for(int i=0; i<targetTrajectories.timeTrajectory.size(); i++)
    {
      stateInputTargetTrajectories.timeTrajectory[i] = targetTrajectories.timeTrajectory[i];
      stateInputTargetTrajectories.stateTrajectory[i].head(baseDim_) = targetTrajectories.stateTrajectory[i].head(baseDim_);

      torsoTargetTrajectories.timeTrajectory[i] = targetTrajectories.timeTrajectory[i];
      torsoTargetTrajectories.stateTrajectory[i] = targetTrajectories.stateTrajectory[i].segment(baseDim_, 7);

      eeTargetTrajectories.timeTrajectory[i] = targetTrajectories.timeTrajectory[i];
      eeTargetTrajectories.stateTrajectory[i] = targetTrajectories.stateTrajectory[i].segment(baseDim_ + 7, info_.eeFrames.size() * 7);
    }
  }

  // 删除 TargetTrajectories 中 initTime 之前的所有帧，保留 initTime 前一个关键帧及之后的所有帧
  void MobileManipulatorReferenceManager::trimTargetTrajectoriesBeforeTime(scalar_t startTime)
  {
    // 辅助函数：修剪单个轨迹
    auto trimTrajectory = [startTime](TargetTrajectories& trajectory) {
      if (trajectory.timeTrajectory.empty() || trajectory.timeTrajectory.front() >= startTime) 
      {
        return;
      }

      // 如果只有一个元素，不做删减
      if (trajectory.timeTrajectory.size() <= 1) {
        return;
      }

      // 查找第一个大于或等于 startTime 的元素
      auto index = std::lower_bound(trajectory.timeTrajectory.begin(), 
                                trajectory.timeTrajectory.end(), 
                                startTime);
      
      // 计算要删除的元素数量, 保留 startTime 的之前一个及之后所有
      size_t eraseCount = std::distance(trajectory.timeTrajectory.begin(), index) - 1;

      // 如果存在需要删除的轨迹, 执行删除
      if (eraseCount > 0) {
        trajectory.timeTrajectory.erase(trajectory.timeTrajectory.begin(),  trajectory.timeTrajectory.begin() + eraseCount);
        trajectory.stateTrajectory.erase(trajectory.stateTrajectory.begin(), trajectory.stateTrajectory.begin() + eraseCount);
        trajectory.inputTrajectory.erase(trajectory.inputTrajectory.begin(), trajectory.inputTrajectory.begin() + eraseCount);
      }
    };

    // 对所有轨迹应用修剪
    trimTrajectory(stateInputTargetTrajectories);
    trimTrajectory(torsoTargetTrajectories);
    trimTrajectory(eeTargetTrajectories);
  }

  void MobileManipulatorReferenceManager::publishTargetTrajectoriesLast(void)
  {
    ros_logger_->publishVector("mobile_manipulator/state_input_target_trajectories/state", stateInputTargetTrajectories.stateTrajectory.back());
    ros_logger_->publishVector("mobile_manipulator/state_input_target_trajectories/input", stateInputTargetTrajectories.inputTrajectory.back());
    ros_logger_->publishVector("mobile_manipulator/torso_target_trajectories", torsoTargetTrajectories.stateTrajectory.back());
    ros_logger_->publishVector("mobile_manipulator/ee_target_trajectories", eeTargetTrajectories.stateTrajectory.back());
  }

  void MobileManipulatorReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule)
  {
    // 第一次进入，需要对原始轨迹数据进行初始化
    static bool firstRun{true};
    if(firstRun)
    {
      getFirstTargetTrajectories(targetTrajectories);
      firstRun = false;
    }
    
    // 获取当前控制模式
    controlMode_mtx_.lock();
    int currentMode = currentMpcControlMode_;
    controlMode_mtx_.unlock();

    // 判断模式是否发生切换，切换则使能切换标志
    bool isChange = getControlModeIsChange(currentMode);

    switch(currentMode) // 0: NoControl, 1: ArmOnly, 2: BaseOnly, 3: BaseArm
    {
      case 0: updateNoControl(initTime, targetTrajectories, isChange); break;               // 模式0: 使用上层下发的 targetTrajectories, 
      case 1: updateArmOnlyControl(initTime, initState, isChange); break;                   // 模式1: 关节可动, 底盘锁住
      case 2: updateBaseOnlyControl(initTime, finalTime, initState, isChange); break;       // 模式2: 底盘可动, 下肢和手臂锁住
      case 3: updateBaseArmControl(initTime, finalTime, initState, isChange); break;        // 模式3: 必须控制底盘, 手臂支持局部系和世界系笛卡尔和关节两种轨迹
      case 4: updateArmEeOnlyControl(initTime, initState, isChange); break;                 // 模式4: 底盘随末端移动, 不可控制, 手臂支持世界系笛卡尔轨迹
      default: std::cout << "设置了错误的控制模式, 请检查!!" << std::endl;
    }

    // 对多个轨迹进行裁剪, 删除之前无效的轨迹
    trimTargetTrajectoriesBeforeTime(initTime);

    // 发布时间最近的目标轨迹
    publishTargetTrajectoriesLast();
  }

  void MobileManipulatorReferenceManager::updateArmEndEffectorTrajectory(scalar_t initTime, scalar_t finalTime, TargetTrajectories& targetTrajectories)
  {
    // 获取当前目标状态并更新末端执行器位姿
    vector_t targetState = targetTrajectories.stateTrajectory[0];
    
    // 更新左臂末端执行器位姿 (索引: 10-16)
    if(targetState.size() >= 17)
    {
      targetState.segment(10, 7) = currentLeftArmPose_;
    }
    
    // 更新右臂末端执行器位姿 (索引: 17-23)
    if(targetState.size() >= 24)
    {
      targetState.segment(17, 7) = currentRightArmPose_;
    }
    
    // 计算轨迹时间点
    const scalar_t timeHorizon = finalTime - initTime;
    const scalar_t endTrajTime = finalTime + timeHorizon;
    
    // 构建新的轨迹：当前状态 -> 目标状态 -> 保持目标状态
    targetTrajectories.timeTrajectory = {initTime, finalTime, endTrajTime};
    targetTrajectories.stateTrajectory = {targetState, targetState, targetState};
    targetTrajectories.inputTrajectory = std::vector<vector_t>(3, vector_t::Zero(info_.inputDim));
  }

  void MobileManipulatorReferenceManager::generateTargetwithArmJointCmd(scalar_t initTime, double dt, const vector_t &initState,
                                                                        const vector_t& armJointCmd)
  {
    // 断言检查维度一致性
    assert(armJointCmd.size() == info_.armDim - 4 && 
           "armJointCmd dimension must be equal to info_.armDim - 4");

    // 一直做赋值方式的更新, 时间差为 dt, 单位: s, 初值为当前状态, 终值为 armJointCmd
    vector_t targetState = vector_t::Zero(info_.stateDim);
    targetState.head(baseDim_) = initState.head(baseDim_);
    targetState.tail(info_.armDim - 4) = armJointCmd; 

    double endTime = initTime + dt;
    stateInputTargetTrajectories.timeTrajectory = {initTime, endTime};
    stateInputTargetTrajectories.stateTrajectory = {initState, targetState};
    stateInputTargetTrajectories.inputTrajectory = std::vector<vector_t>(2, vector_t::Zero(info_.inputDim));
  }

  void MobileManipulatorReferenceManager::generateTargetwithVelcmd(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                                   const vector_t &cmdVel)
  {
    // 创建默认的3个时间点轨迹
    const auto timeHorizon = finalTime - initTime;
    const auto endTrajTime = finalTime + timeHorizon;
    
    vector_t targetState = stateInputTargetTrajectories.getDesiredState(initTime);
    targetState.head(baseDim_) = initState.head(baseDim_);
    targetState.tail(info_.armDim - 4) = lockArmState;
    stateInputTargetTrajectories.timeTrajectory = {initTime, finalTime, endTrajTime};
    stateInputTargetTrajectories.stateTrajectory = {targetState, targetState, targetState};
    stateInputTargetTrajectories.inputTrajectory = std::vector<vector_t>(3, vector_t::Zero(info_.inputDim));
    
    // 然后应用速度指令
    updateBaseTrajectory(initTime, finalTime, initState, stateInputTargetTrajectories, cmdVel);
  }

  void MobileManipulatorReferenceManager::generateTargetwithVelcmdLimited(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                                    vector_t cmdVel)
  {
    // 创建默认的3个时间点轨迹
    const auto timeHorizon = finalTime - initTime;
    const auto endTrajTime = finalTime + timeHorizon;
    
    vector_t targetState = stateInputTargetTrajectories.getDesiredState(initTime);
    targetState.head(baseDim_) = initState.head(baseDim_);
    targetState.tail(info_.armDim - 4) = lockArmState;
    stateInputTargetTrajectories.timeTrajectory = {initTime, finalTime, endTrajTime};
    stateInputTargetTrajectories.stateTrajectory = {targetState, targetState, targetState};
    stateInputTargetTrajectories.inputTrajectory = std::vector<vector_t>(3, vector_t::Zero(info_.inputDim));
    
    // 进行梯形加减速调节
    static double prev_initTime = initTime;
    velLimiter_->setAccelerationDt(initTime - prev_initTime);
    cmdVel = velLimiter_->limitAcceleration(cmdVel);  
    prev_initTime = initTime;

    // 然后应用速度指令
    updateBaseTrajectory(initTime, finalTime, initState, stateInputTargetTrajectories, cmdVel);
  }

  void MobileManipulatorReferenceManager::generateTargetwithPoseCmd(double initTime, const vector_t &initState,
                                                                    double cmdPoseTime, const vector_t &cmdPose)
  {
    // 创建2个时间点轨迹，初值为当前的期望状态，终值为 cmdPose 指定的位置

    vector_t initTargetState = stateInputTargetTrajectories.getDesiredState(initTime);
    initTargetState.tail(info_.armDim - 4) = lockArmState;

    vector_t targetState = initTargetState;
    targetState.head(baseDim_) = cmdPose; // 只更新底盘位置部分
    stateInputTargetTrajectories.timeTrajectory = {initTime, cmdPoseTime};
    stateInputTargetTrajectories.stateTrajectory = {initTargetState, targetState};
    stateInputTargetTrajectories.inputTrajectory = std::vector<vector_t>(2, vector_t::Zero(info_.inputDim));
  }

  void MobileManipulatorReferenceManager::updateBaseTrajectory(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                             TargetTrajectories &targetTrajectories, vector_t cmdVel)
  {
    // 应用速度指令到现有轨迹的底盘部分（前baseDim_个元素）
    // vector_t cmdVelRot = cmdVel;
    // const Eigen::Matrix<scalar_t, 3, 1> targetZyx = {initState[2], 0, 0};
    // cmdVelRot.head(3) = getRotationMatrixFromZyxEulerAngles(targetZyx) * cmdVel.head(3);
    
    // 更新3个时间点的底盘状态，保持手臂部分不变
    for (size_t i = 0; i < targetTrajectories.timeTrajectory.size(); ++i) 
    {
      auto& state = targetTrajectories.stateTrajectory[i];
      auto& input = targetTrajectories.inputTrajectory[i];
      scalar_t timeOffset = targetTrajectories.timeTrajectory[i] - initTime;
      
      state.head(3) = state.head(3) + cmdVel.head(3) * timeOffset;  // x, y, yaw
      input[0] = cmdVel[0];
      input[1] = cmdVel[1];
      input[2] = cmdVel[2];
    }
  }

  bool MobileManipulatorReferenceManager::controlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, 
                                                           kuavo_msgs::changeTorsoCtrlMode::Response& res)
  {
    controlMode_mtx_.lock();
    
    // 验证控制模式的有效性
    if (req.control_mode < 0 || req.control_mode > 4) {
      res.result = false;
      res.mode = currentMpcControlMode_;
      res.message = "Invalid control mode. Valid modes: 0(NoControl), 1(ArmOnly), 2(BaseOnly), 3(BaseArm), 4(ArmEeOnly)";
      controlMode_mtx_.unlock();
      return true;
    }
    
    // 更新控制模式
    int previousMode = currentMpcControlMode_;
    currentMpcControlMode_ = req.control_mode;
    
    // 根据控制模式设置相应的行为
    switch (currentMpcControlMode_) {
      case 0:  // NoControl
        res.message = "Switched to NoControl mode - no active control";
        break;
      case 1:  // ArmOnly
        res.message = "Switched to ArmOnly mode - controlling arms only, base fixed";
        break;
      case 2:  // BaseOnly
        res.message = "Switched to BaseOnly mode - controlling base only, arms fixed";
        break;
      case 3:  // BaseArm
        res.message = "Switched to BaseArm mode - controlling both base and arms";
        break;
      case 4: //ArmEeOnly
        res.message = "Switched to ArmEeOnly mode - controlling arms Ee only";
        break;
      default:
        res.message = "Unknown control mode";
        break;
    }
    
    res.result = true;
    res.mode = currentMpcControlMode_;
    
    // 打印模式切换信息
    std::cout << "[MobileManipulatorReferenceManager] MPC Control mode changed from " 
              << previousMode << " to " << currentMpcControlMode_ << ": " << res.message << std::endl;
    
    controlMode_mtx_.unlock();
    return true;
  }

  bool MobileManipulatorReferenceManager::getMpcControlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req,
                                                                   kuavo_msgs::changeTorsoCtrlMode::Response& res)
  {
    std::lock_guard<std::mutex> lock(controlMode_mtx_);
    res.result = true;
    res.mode = currentMpcControlMode_;
    res.message = "Success";
    return true;
  }

  void MobileManipulatorReferenceManager::getCurrentEeWorldPose(vector_t& EeState, const vector_t& initState)
  {
    assert(EeState.size() == info_.eeFrames.size() * 7 && "EeState dimension must be info_.eeFrames.size()*7!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 遍历每个末端执行器
    for (size_t ee_idx = 0; ee_idx < info_.eeFrames.size(); ++ee_idx) 
    {
      // 获取末端执行器帧ID（这里需要您根据实际情况获取）
      pinocchio::FrameIndex frameId = model.getFrameId(info_.eeFrames[ee_idx]);
      // 获取末端在世界坐标系中的位姿
      const pinocchio::SE3& ee_pose = data.oMf[frameId];

      EeState.segment<3>(ee_idx*7) = ee_pose.translation();
      EeState.segment<4>(ee_idx*7+3) = Eigen::Quaterniond(ee_pose.rotation()).coeffs();
    }
  }

  void MobileManipulatorReferenceManager::transformLocalToWorldFrame(vector_t& localPose, const vector_t& baseState) 
  {
    // 获取基座在世界坐标系中的位姿
    Eigen::Vector3d base_position(baseState[0], baseState[1], 0.0);
    double base_yaw = baseState[2];
    
    // 创建基座的旋转矩阵
    Eigen::Matrix3d base_rotation = Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Quaterniond base_quat(base_rotation);
    
    // 提取局部位姿
    Eigen::Vector3d local_pos = localPose.head(3);
    Eigen::Quaterniond local_quat(localPose[6],  // w
                                  localPose[3],  // x
                                  localPose[4],  // y
                                  localPose[5]); // z
    
    // 转换为世界坐标系
    localPose.head(3) = base_position + base_rotation * local_pos;
    Eigen::Quaterniond world_quat = base_quat * local_quat;
    world_quat.normalize();
    
    // 更新到位姿向量
    localPose.tail(4) = world_quat.coeffs();
    
  }

  void MobileManipulatorReferenceManager::updateNoControl(double initTime, const TargetTrajectories& targetTrajectories, bool isChange)
  {
    static bool firstRun{true};
    if(isChange || firstRun)
    {
      setEnableEeTargetTrajectories(true); // 开启末端笛卡尔跟踪
      setEnableArmJointTrack(false); // 关闭手臂跟踪
      setEnableBaseTrack(false);   // 关闭底盘跟踪

      firstRun = false;
      return;
    }

    getAllTargetTrajectories(targetTrajectories);
  }

  void MobileManipulatorReferenceManager::updateArmOnlyControl(double initTime, const vector_t& initState, bool isChange)
  {
    if(isChange)
    {
      // 重置状态
      stateInputTargetTrajectories.timeTrajectory = {initTime};
      stateInputTargetTrajectories.stateTrajectory = {initState};
      stateInputTargetTrajectories.inputTrajectory = {vector_t::Zero(info_.inputDim)};

      arm_joint_traj_ = initState.tail(info_.armDim - 4); 
      setEnableEeTargetTrajectories(false); // 关闭末端笛卡尔跟踪
      setEnableArmJointTrack(true); // 开启手臂跟踪
      setEnableBaseTrack(true);   // 开启底盘跟踪

      return;
    }

    static vector_t armJointTarget = vector_t::Zero(info_.armDim - 4);
    armJoint_mtx_.lock();
    armJointTarget = arm_joint_traj_;
    armJoint_mtx_.unlock();
    
    setLockArmJointTarget(armJointTarget);
    // generateTargetwithArmJointCmd(initTime, 0.01, initState, armJointTarget);

    generateTargetwithVelcmdLimited(initTime, initTime, initState, Eigen::Vector3d::Zero());
  }

  void MobileManipulatorReferenceManager::updateBaseOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  {
    if(isChange)
    {
      vector_t resetArmState = initState;
      resetArmState.segment(baseDim_, 4) = vector_t::Zero(4);   // 下肢关节需要复位到零点
      // 重置状态
      stateInputTargetTrajectories.timeTrajectory = {initTime};
      stateInputTargetTrajectories.stateTrajectory = {resetArmState};
      stateInputTargetTrajectories.inputTrajectory = {vector_t::Zero(info_.inputDim)};

      setLockArmJointTarget(initState.tail(info_.armDim - 4));

      setEnableEeTargetTrajectories(false); // 关闭末端笛卡尔跟踪
      setEnableArmJointTrack(true); // 开启手臂跟踪
      setEnableBaseTrack(true);   // 开启底盘跟踪

      return;
    }

    // 生成底盘指令轨迹（包含速度指令和位置指令处理）
    if(isCmdPoseUpdated_)
    {
      // 计算期望位置
      cmdPose_mtx_.lock();
      currentCmdPose_ = cmdPose_;
      // currentCmdPoseTime_ = initTime + cmdPoseTime_;
      currentCmdPose_[2] = targetYawPreProcess(initState[2], cmdPose_[2]);
      currentCmdPoseTime_ = initTime + calcExpectedTime(initState.head(3), currentCmdPose_);
      cmdPose_mtx_.unlock();

      // 发布开始移动标志
      isCmdPoseReach_ = false;
      std_msgs::Bool start_msg;
      start_msg.data = false;
      targetPoseFlagPub_.publish(start_msg);

      isCmdVelUpdated_ = false;
      isCmdPoseUpdated_ = false;
    }

    static int cmd_vel_cnt;
    if (!cmdVel_.isZero(1e-6)) 
    {
      cmdvel_mtx_.lock();
      currentCmdVel_ = cmdVel_;
      cmdvel_mtx_.unlock();
      cmd_vel_cnt = 0;
    }
    else
    {
      cmd_vel_cnt++;
      if (cmd_vel_cnt > 5) 
      {
        currentCmdVel_ = Eigen::Vector3d::Zero();
        cmd_vel_cnt = 0;
      }
    }

    if(isCmdVelUpdated_ && currentCmdPoseTime_ < initTime)
    {
      // 进行梯形加减速调节
      static double prev_initTime = initTime;
      velLimiter_->setAccelerationDt(initTime - prev_initTime);
      Eigen::Vector3d cmdVelFilter = velLimiter_->limitAcceleration(currentCmdVel_);  
      prev_initTime = initTime;

      // 判断速度为0，跳出速度控制
      static int zero_vel_cnt;
      if(zero_vel_cnt < 5)
      {
        if(cmdVelFilter.isZero(1e-6)) zero_vel_cnt++;
        else zero_vel_cnt = 0;
      }
      else
      {
        zero_vel_cnt = 0;
        isCmdVelUpdated_ = false;
      }

      generateTargetwithVelcmd(initTime, finalTime, initState, cmdVelFilter);

      currentCmdPose_ = initState.head(3);
      isCmdPoseReach_ = false;
    }
    else if(currentCmdPoseTime_ > initTime)
    {
      // 计算期望位置
      generateTargetwithPoseCmd(initTime, initState, currentCmdPoseTime_, currentCmdPose_);
      // vector_t cmdVel = (currentCmdPose_ - initState.head(3)) / (currentCmdPoseTime_ - initTime);
      // generateTargetwithVelcmd(initTime, finalTime, initState, cmdVel);
    }
    else if(currentCmdPoseTime_ <= initTime)
    {
      generateTargetwithPoseCmd(initTime, currentCmdPose_, initTime, currentCmdPose_);
      // 发布结束移动标志
      if(!isCmdPoseReach_ && isVectorClose(initState.head(3), currentCmdPose_, wheelReachThreshold_))
      {
        isCmdPoseReach_ = true;
        std_msgs::Bool end_msg;
        end_msg.data = true;
        targetPoseFlagPub_.publish(end_msg);
      }
    }

  }

  void MobileManipulatorReferenceManager::updateBaseArmControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  {
    if(isChange)
    {
      // 重置状态
      stateInputTargetTrajectories.timeTrajectory = {initTime};
      stateInputTargetTrajectories.stateTrajectory = {initState};
      stateInputTargetTrajectories.inputTrajectory = {vector_t::Zero(info_.inputDim)};

      vector_t eeState;
      eeState.setZero(info_.eeFrames.size() * 7);
      getCurrentEeWorldPose(eeState, initState);
      left_arm_traj_pose_ = eeState.head(7);
      right_arm_traj_pose_ = eeState.tail(7);
      eeTargetTrajectories.timeTrajectory = {initTime};
      eeTargetTrajectories.stateTrajectory = {eeState};
      eeTargetTrajectories.inputTrajectory = {vector_t::Zero(info_.eeFrames.size() * 7)};

      setLockArmJointTarget(initState.tail(info_.armDim - 4));
      left_arm_joint_traj_ = initState.tail(info_.armDim - 4).head((info_.armDim - 4)/2);
      right_arm_joint_traj_ = initState.tail(info_.armDim - 4).tail((info_.armDim - 4)/2);

      setEnableEeTargetTrajectories(false); // 关闭末端笛卡尔跟踪
      setEnableArmJointTrack(true); // 开启手臂跟踪
      setEnableBaseTrack(true);   // 开启底盘跟踪

      return;
    }

    // base_arm 模式接受三种收发逻辑: 
    // 1. 发手臂末端局部系轨迹, 加底盘轨迹;
    // 2. 发手臂末端世界系轨迹, 加底盘轨迹; //有些危险, 上层需关注清楚
    // 2. 发手臂关节轨迹, 加底盘轨迹;
    
    static vector_t armJointTarget = vector_t::Zero(info_.armDim - 4); // 双臂关节轨迹
    static vector_t armEeTarget = vector_t::Zero(info_.eeFrames.size() * 7); // 双臂末端轨迹
    
    if(desire_mode_ == 0 || desire_mode_ == 1) // 世界系的笛卡尔末端控制
    {
      setEnableArmJointTrack(false); // 关闭手臂跟踪
      setEnableEeTargetTrajectories(true); // 开启末端笛卡尔跟踪
      armPose_mtx_.lock();
      armEeTarget << left_arm_traj_pose_, right_arm_traj_pose_;
      armPose_mtx_.unlock();
    
      if(desire_mode_ == 1)
      {
        for(int eeIndex = 0; eeIndex < info_.eeFrames.size(); eeIndex++)
        {
          vector_t eePose = armEeTarget.segment(eeIndex*7, 7);
          transformLocalToWorldFrame(eePose, initState);
          armEeTarget.segment(eeIndex*7, 7) = eePose;
        }
      }
      vector_t eeState = vector_t::Zero(info_.eeFrames.size() * 7); // 双臂末端轨迹
      getCurrentEeWorldPose(eeState, initState);
      eeTargetTrajectories.timeTrajectory = {initTime, initTime+0.1};
      eeTargetTrajectories.stateTrajectory = {eeState, armEeTarget};
      eeTargetTrajectories.inputTrajectory = std::vector<vector_t>(2, vector_t::Zero(info_.eeFrames.size() * 7));
    }
    else if(desire_mode_ == 2)  // 关节控制
    {
      setEnableArmJointTrack(true); // 开启手臂跟踪
      setEnableEeTargetTrajectories(false); // 开启末端笛卡尔跟踪
      armPose_mtx_.lock();
      armJointTarget << left_arm_joint_traj_, right_arm_joint_traj_;
      armPose_mtx_.unlock();
      setLockArmJointTarget(armJointTarget);
    }

    // 生成底盘指令轨迹（包含速度指令和位置指令处理）
    if(isCmdPoseUpdated_)
    {
      // 计算期望速度
      cmdPose_mtx_.lock();
      currentCmdPose_ = cmdPose_;
      // currentCmdPoseTime_ = initTime + cmdPoseTime_;
      currentCmdPose_[2] = targetYawPreProcess(initState[2], cmdPose_[2]);
      currentCmdPoseTime_ = initTime + calcExpectedTime(initState.head(3), currentCmdPose_);
      cmdPose_mtx_.unlock();

      // 发布开始移动标志
      isCmdPoseReach_ = false;
      std_msgs::Bool start_msg;
      start_msg.data = false;
      targetPoseFlagPub_.publish(start_msg);

      isCmdVelUpdated_ = false;
      isCmdPoseUpdated_ = false;
    }

    static int cmd_vel_cnt;
    if (!cmdVel_.isZero(1e-6)) 
    {
      cmdvel_mtx_.lock();
      currentCmdVel_ = cmdVel_;
      cmdvel_mtx_.unlock();
      cmd_vel_cnt = 0;
    }
    else
    {
      cmd_vel_cnt++;
      if (cmd_vel_cnt > 5) 
      {
        currentCmdVel_ = Eigen::Vector3d::Zero();
        cmd_vel_cnt = 0;
      }
    }

    if(isCmdVelUpdated_ && currentCmdPoseTime_ < initTime)
    {
      // 进行梯形加减速调节
      static double prev_initTime = initTime;
      velLimiter_->setAccelerationDt(initTime - prev_initTime);
      Eigen::Vector3d cmdVelFilter = velLimiter_->limitAcceleration(currentCmdVel_);  
      prev_initTime = initTime;

      // 判断速度为0，跳出速度控制
      static int zero_vel_cnt;
      if(zero_vel_cnt < 5)
      {
        if(cmdVelFilter.isZero(1e-6)) zero_vel_cnt++;
        else zero_vel_cnt = 0;
      }
      else
      {
        zero_vel_cnt = 0;
        isCmdVelUpdated_ = false;
      }

      generateTargetwithVelcmd(initTime, finalTime, initState, cmdVelFilter);

      currentCmdPose_ = initState.head(3);
      isCmdPoseReach_ = false;
    }
    else if(currentCmdPoseTime_ > initTime)   // 通过时间判断是否存在位置指令需要执行
    {
      // 计算期望位置
      generateTargetwithPoseCmd(initTime, initState, currentCmdPoseTime_, currentCmdPose_);
      // vector_t cmdVel = (currentCmdPose_ - initState.head(3)) / (currentCmdPoseTime_ - initTime);
      // generateTargetwithVelcmd(initTime, finalTime, initState, cmdVel);
    }
    else if(currentCmdPoseTime_ <= initTime)
    {
      generateTargetwithPoseCmd(initTime, currentCmdPose_, initTime, currentCmdPose_);
      // 发布结束移动标志
      if(!isCmdPoseReach_ && isVectorClose(initState.head(3), currentCmdPose_, wheelReachThreshold_))
      {
        isCmdPoseReach_ = true;
        std_msgs::Bool end_msg;
        end_msg.data = true;
        targetPoseFlagPub_.publish(end_msg);
      }
    }
  }

  void MobileManipulatorReferenceManager::updateArmEeOnlyControl(double initTime, const vector_t& initState, bool isChange)
  {
    if(isChange)
    {
      // 重置状态
      stateInputTargetTrajectories.timeTrajectory = {initTime};
      stateInputTargetTrajectories.stateTrajectory = {initState};
      stateInputTargetTrajectories.inputTrajectory = {vector_t::Zero(info_.inputDim)};

      vector_t eeState;
      eeState.setZero(info_.eeFrames.size() * 7);
      getCurrentEeWorldPose(eeState, initState);
      left_arm_traj_pose_ = eeState.head(7);
      right_arm_traj_pose_ = eeState.tail(7);
      eeTargetTrajectories.timeTrajectory = {initTime};
      eeTargetTrajectories.stateTrajectory = {eeState};
      eeTargetTrajectories.inputTrajectory = {vector_t::Zero(info_.eeFrames.size() * 7)};

      setEnableEeTargetTrajectories(true); // 开启末端笛卡尔跟踪
      setEnableArmJointTrack(false); // 关闭手臂跟踪
      setEnableBaseTrack(false);   // 关闭底盘跟踪

      return;
    }

    static vector_t armEeTarget = vector_t::Zero(info_.eeFrames.size() * 7); // 双臂末端轨迹
    
    armPose_mtx_.lock();
    armEeTarget << left_arm_traj_pose_, right_arm_traj_pose_;
    armPose_mtx_.unlock();

    vector_t eeState = vector_t::Zero(info_.eeFrames.size() * 7); // 双臂末端轨迹
    getCurrentEeWorldPose(eeState, initState);
    eeTargetTrajectories.timeTrajectory = {initTime, initTime+0.1};
    eeTargetTrajectories.stateTrajectory = {eeState, armEeTarget};
    eeTargetTrajectories.inputTrajectory = std::vector<vector_t>(2, vector_t::Zero(info_.eeFrames.size() * 7));
  }

  double MobileManipulatorReferenceManager::targetYawPreProcess(double currentYaw, double targetYaw)
{
    // 规范化角度到 [-π, π]
    auto normalize = [](double angle) {
        angle = std::fmod(angle, 2.0 * M_PI);
        if (angle > M_PI) angle -= 2.0 * M_PI;
        if (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    };

    // 规范化角度用于计算最短路径
    double normalizedCurrent = normalize(currentYaw);
    double normalizedTarget = normalize(targetYaw);

    // std::cout << "Original currentYaw: " << currentYaw << std::endl;
    // std::cout << "Original targetYaw: " << targetYaw << std::endl;
    // std::cout << "Normalized currentYaw: " << normalizedCurrent << std::endl;
    // std::cout << "Normalized targetYaw: " << normalizedTarget << std::endl;
    
    // 计算标准化后的角度差（考虑最短路径）
    double rawDiff = normalizedTarget - normalizedCurrent;
    
    // std::cout << "rawDiff: " << rawDiff << std::endl;
    // 找到最短路径的目标角度
    double bestNormalizedTarget = 0.0;
    if(rawDiff > M_PI)
    {
      // 正转路径太长, 选择反转(减去2π)
      bestNormalizedTarget = normalizedTarget - 2.0 * M_PI;
    }
    else if(rawDiff < -M_PI)
    {
      // 反转路径太长，选择正转(加上2π)
      bestNormalizedTarget = normalizedTarget + 2.0 * M_PI;
    }
    else
    {
      bestNormalizedTarget = normalizedTarget;
    }

    // 计算 currentYaw 所在的圈数
    auto cycle = currentYaw / (2.0 * M_PI);
    int currentCycle = std::round(cycle);
    // std::cout << "Current cycle: " << currentCycle << "[" << cycle << "]" << std::endl;

    // 将目标角度锁定在当前圈数
    double newTargetYaw = currentCycle * 2.0 * M_PI + bestNormalizedTarget;
    // std::cout << "Best normalized target: " << bestNormalizedTarget << std::endl;
    // std::cout << "Cur targetYaw: " << currentYaw << ", New targetYaw: " << newTargetYaw << std::endl;
    // std::cout << "Final rotation: " << (newTargetYaw - currentYaw) << " radians" << std::endl;
    return newTargetYaw;
}

  double MobileManipulatorReferenceManager::calcExpectedTime(const vector_t& currentPose, const vector_t& cmdPose)
  {
    // 计算位置差（绝对值）
    std::vector<double> position_diff(3);
    position_diff[0] = std::abs(cmdPose[0] - currentPose[0]); // X轴位置差
    position_diff[1] = std::abs(cmdPose[1] - currentPose[1]); // Y轴位置差  
    position_diff[2] = std::abs(cmdPose[2] - currentPose[2]); // 角度差

    // 计算各轴所需时间
    std::vector<double> axis_times(3);
    for (int i = 0; i < 3; ++i) {
        if (wheel_move_spd_[i] > 0.001) { // 避免除零
            axis_times[i] = position_diff[i] / wheel_move_spd_[i];
        } else {
            axis_times[i] = 0.0;
        }
    }

    // 取最大时间作为期望时间
    double cmdPoseExpectedTime = *std::max_element(axis_times.begin(), axis_times.end());
    
    ROS_INFO("Position differences: [%.3f, %.3f, %.3f]", 
             position_diff[0], position_diff[1], position_diff[2]);
    ROS_INFO("Axis times: [%.3f, %.3f, %.3f] seconds", 
             axis_times[0], axis_times[1], axis_times[2]);
    ROS_INFO("Expected time: %.3f seconds", cmdPoseExpectedTime);

    return cmdPoseExpectedTime;
  }

  bool MobileManipulatorReferenceManager::isVectorClose(const vector_t& v1, const vector_t& v2, const vector_t& threshold)
  {
    // 参数有效性检查
    if (v1.size() != v2.size() || v1.size() != threshold.size()) {
        std::cerr << "[isVectorClose] 错误: 向量尺寸不匹配! v1: " << v1.size() 
                  << ", v2: " << v2.size() << ", threshold: " << threshold.size() << std::endl;
        return false;
    }

    for (int i = 0; i < threshold.size(); ++i) {
        if (std::abs(v1[i] - v2[i]) > threshold[i]) {
            std::cout << "[isVectorClose] "<< "第 " << i << " 个元素超出阈值, 误差: " << v1[i] - v2[i] 
            << ", 阈值: " << threshold[i] << std::endl;
            return false;
        }
    }
    return true;
  }

}  // namespace mobile_manipulator
}  // namespace ocs2