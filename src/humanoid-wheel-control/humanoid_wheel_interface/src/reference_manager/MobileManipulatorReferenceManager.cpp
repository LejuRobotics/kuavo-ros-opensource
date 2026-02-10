#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "humanoid_wheel_interface/reference_manager/MobileManipulatorReferenceManager.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <angles/angles.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <chrono>

namespace ocs2 {
namespace mobile_manipulator {

  template <typename T>
  T square(T a)
  {
    return a * a;
  }
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q)
  {
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    zyx(0) =
        std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) =
        std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
  }

  MobileManipulatorReferenceManager::MobileManipulatorReferenceManager(const ManipulatorModelInfo& info, const PinocchioInterface& pinocchioInterface, const std::string& taskFile)
  : ReferenceManager(TargetTrajectories(), ModeSchedule())
  , info_(info)
  , pinocchioInterface_(pinocchioInterface)
  , taskFile_(taskFile)
  , stateInputTargetTrajectories_(TargetTrajectories({0}, {vector_t::Zero(info_.stateDim)}, {vector_t::Zero(info_.inputDim)}))
  , torsoTargetTrajectories_(TargetTrajectories({0}, {vector_t::Zero(7)}, {vector_t::Zero(7)}))
  , eeTargetTrajectories_(TargetTrajectories({0}, {vector_t::Zero(info_.eeFrames.size())}, {vector_t::Zero(info_.eeFrames.size())}))
  {

    loadParamFromTaskFile();  // 加载配置参数

    baseDim_ = info_.stateDim-info_.armDim;
    left_arm_traj_pose_ = vector_t::Zero(7);  // x,y,z,qx,qy,qz,qw
    right_arm_traj_pose_ = vector_t::Zero(7); // x,y,z,qx,qy,qz,qw

    left_arm_joint_traj_ = vector_t::Zero(7);
    right_arm_joint_traj_ = vector_t::Zero(7);
    
    // 初始化MPC控制模式为NoControl（完全接收上层下发的 TargetTrajectory, 其余话题无法接收）
    currentMpcControlMode_ = 2;  // NoControl

    /*************cmdPose相关初始化********************/
    currentCmdPose_.setZero();
    /************************************************/

    // 躯干相对底盘位姿指令初始化
    cmdTorsoPose_.setZero(6);
    currentTorsoPose_.setZero(6);

    // 注册日志记录器
    ros_logger_ = new humanoid::TopicLogger(nodeHandle_);

    /****************************ruckig位置规划器初始化*********************************/ 
    auto createCmdPosePlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig>
    {
      int dofPose = baseDim_;
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(dofPose);
      Eigen::VectorXd max_velocity_ruckig, max_acceleration_ruckig, max_jerk_ruckig;
      max_velocity_ruckig.setZero(dofPose);
      max_acceleration_ruckig.setZero(dofPose);
      max_jerk_ruckig.setZero(dofPose);
      for(int i = 0; i < dofPose; i++)
      {
        max_velocity_ruckig[i] = wheel_move_spd_[i];
        max_acceleration_ruckig[i] = wheel_move_acc_[i];
        max_jerk_ruckig[i] = wheel_move_jerk_[i];
      }
      plannerPtr->setVelocityLimits(max_velocity_ruckig, -max_velocity_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_ruckig, -max_acceleration_ruckig);
      plannerPtr->setJerkLimits(max_jerk_ruckig);
      return plannerPtr;
    };
    cmdPosePlannerRuckigPtr_ = createCmdPosePlanner();
    timedPlannerScheduler_.addTimedPlannerPosePtr(createCmdPosePlanner());  // 一个世界系
    timedPlannerScheduler_.addTimedPlannerPosePtr(createCmdPosePlanner());  // 一个局部系
    prevTargetPose_.setZero(baseDim_);
    prevTargetVel_.setZero(baseDim_);
    prevTargetAcc_.setZero(baseDim_);
    /*********************************************************************************/ 

    /****************************ruckig速度规划器初始化**********************************/ 
    Eigen::VectorXd max_acceleration_ruckig, max_jerk_ruckig;
    max_acceleration_ruckig.setZero(baseDim_);
    max_jerk_ruckig.setZero(baseDim_);
    for(int i = 0; i < baseDim_; i++)
    {
      max_acceleration_ruckig[i] = wheel_move_acc_[i];
      max_jerk_ruckig[i] = wheel_move_jerk_[i];
    }
    cmdVelPlannerRuckigPtr_ = std::make_shared<cmdVelPlannerWithRuckig>(baseDim_);
    cmdVel_prevTargetPose_.setZero(baseDim_);
    cmdVel_prevTargetVel_.setZero(baseDim_);
    cmdVel_prevTargetAcc_.setZero(baseDim_);

    cmdVelPlannerRuckigPtr_->setAccelerationLimits(max_acceleration_ruckig, -max_acceleration_ruckig);
    cmdVelPlannerRuckigPtr_->setJerkLimits(max_jerk_ruckig);
    /*********************************************************************************/

    /*******************ruckig躯干笛卡尔规划器初始化 (自由度:x, z, yaw, pitch)***************************/ 
    auto createTorsoPosePlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig> 
    {
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(4, true);
      Eigen::VectorXd max_velocity_torsoPose_ruckig, max_acceleration_torsoPose_ruckig, max_jerk_torsoPose_ruckig;
      max_velocity_torsoPose_ruckig.setZero(4);
      max_acceleration_torsoPose_ruckig.setZero(4);
      max_jerk_torsoPose_ruckig.setZero(4);
      for(int i=0; i<4; i++)
      {
        max_velocity_torsoPose_ruckig[i] = torsoPose_move_spd_[i];
        max_acceleration_torsoPose_ruckig[i] = torsoPose_move_acc_[i];
        max_jerk_torsoPose_ruckig[i] = torsoPose_move_jerk_[i];
      }
      plannerPtr->setVelocityLimits(max_velocity_torsoPose_ruckig, -max_velocity_torsoPose_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_torsoPose_ruckig, -max_acceleration_torsoPose_ruckig);
      plannerPtr->setJerkLimits(max_jerk_torsoPose_ruckig);
      return plannerPtr;
    };
    torsoPosePlannerRuckigPtr_ = createTorsoPosePlanner();
    timedPlannerScheduler_.addTimedPlannerPosePtr(createTorsoPosePlanner());
    torsoPose_prevTargetPose_.setZero(4);
    torsoPose_prevTargetVel_.setZero(4);
    torsoPose_prevTargetAcc_.setZero(4);
    /*********************************************************************************/

    /********************ruckig下肢关节规划器初始化 (单位: 弧度)***************************/
    auto createLegJointPlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig> 
    {
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(4, true);
      Eigen::VectorXd max_velocity_legJoint_ruckig, max_acceleration_legJoint_ruckig, max_jerk_legJoint_ruckig;
      max_velocity_legJoint_ruckig.setZero(4);
      max_acceleration_legJoint_ruckig.setZero(4);
      max_jerk_legJoint_ruckig.setZero(4);
      for(int i=0; i<4; i++)
      {
        max_velocity_legJoint_ruckig.segment(i, 1) = legJoint_move_spd_.head<1>();
        max_acceleration_legJoint_ruckig.segment(i, 1) = legJoint_move_acc_.head<1>();
        max_jerk_legJoint_ruckig.segment(i, 1) = legJoint_move_jerk_.head<1>();
      }
      plannerPtr->setVelocityLimits(max_velocity_legJoint_ruckig, -max_velocity_legJoint_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_legJoint_ruckig, -max_acceleration_legJoint_ruckig);
      plannerPtr->setJerkLimits(max_jerk_legJoint_ruckig);
      return plannerPtr;
    };
    legJointPlannerRuckigPtr_ = createLegJointPlanner();
    timedPlannerScheduler_.addTimedPlannerPosePtr(createLegJointPlanner());
    legJoint_prevTargetPose_.setZero(4);
    legJoint_prevTargetVel_.setZero(4);
    legJoint_prevTargetAcc_.setZero(4);
    /*********************************************************************************/

    /*******************ruckig双臂笛卡尔规划器初始化 (Zyx欧拉角)***************************/ 
    auto createDualArmEePlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig> 
    {
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(info_.eeFrames.size() * 6, true);
      Eigen::VectorXd max_velocity_dualArm_ruckig, max_acceleration_dualArm_ruckig, max_jerk_dualArm_ruckig;
      max_velocity_dualArm_ruckig.setZero(info_.eeFrames.size() * 6);
      max_acceleration_dualArm_ruckig.setZero(info_.eeFrames.size() * 6);
      max_jerk_dualArm_ruckig.setZero(info_.eeFrames.size() * 6);
      for(int i=0; i<info_.eeFrames.size(); i++)
      {
        max_velocity_dualArm_ruckig.segment(i*6, 6) = dualArm_move_spd_.head<6>();
        max_acceleration_dualArm_ruckig.segment(i*6, 6) = dualArm_move_acc_.head<6>();
        max_jerk_dualArm_ruckig.segment(i*6, 6) = dualArm_move_jerk_.head<6>();
      }
      plannerPtr->setVelocityLimits(max_velocity_dualArm_ruckig, -max_velocity_dualArm_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_dualArm_ruckig, -max_acceleration_dualArm_ruckig);
      plannerPtr->setJerkLimits(max_jerk_dualArm_ruckig);
      return plannerPtr;
    };
    cmdDualArmEePlannerRuckigPtr_ = createDualArmEePlanner();
    timedPlannerScheduler_.addTimedPlannerPosePtr(createDualArmEePlanner());  // 一个世界系
    timedPlannerScheduler_.addTimedPlannerPosePtr(createDualArmEePlanner());  // 一个局部系
    cmdDualArm_prevTargetPose_.setZero(info_.eeFrames.size() * 6);
    cmdDualArm_prevTargetVel_.setZero(info_.eeFrames.size() * 6);
    cmdDualArm_prevTargetAcc_.setZero(info_.eeFrames.size() * 6);
    /*********************************************************************************/

    /********************ruckig上肢关节规划器初始化 (单位: 弧度)***************************/
    auto createArmJointPlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig> 
    {
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(info_.armDim - 4, true);
      Eigen::VectorXd max_velocity_armJoint_ruckig, max_acceleration_armJoint_ruckig, max_jerk_armJoint_ruckig;
      max_velocity_armJoint_ruckig.setZero(info_.armDim - 4);
      max_acceleration_armJoint_ruckig.setZero(info_.armDim - 4);
      max_jerk_armJoint_ruckig.setZero(info_.armDim - 4);
      for(int i=0; i<info_.armDim - 4; i++)
      {
        max_velocity_armJoint_ruckig.segment(i, 1) = armJoint_move_spd_.head<1>();
        max_acceleration_armJoint_ruckig.segment(i, 1) = armJoint_move_acc_.head<1>();
        max_jerk_armJoint_ruckig.segment(i, 1) = armJoint_move_jerk_.head<1>();
      }
      plannerPtr->setVelocityLimits(max_velocity_armJoint_ruckig, -max_velocity_armJoint_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_armJoint_ruckig, -max_acceleration_armJoint_ruckig);
      plannerPtr->setJerkLimits(max_jerk_armJoint_ruckig);
      return plannerPtr;
    };
    armJointPlannerRuckigPtr_ = createArmJointPlanner();
    timedPlannerScheduler_.addTimedPlannerPosePtr(createArmJointPlanner());
    armJoint_prevTargetPose_.setZero(info_.armDim - 4);
    armJoint_prevTargetPose_.setZero(info_.armDim - 4);
    armJoint_prevTargetPose_.setZero(info_.armDim - 4);
    /*********************************************************************************/

    /******************** 时间调度器初始化相关 ***************************/
    isTimedPlannerUpdated_.resize(timedPlannerScheduler_.getPlannersNum(), false);
    desireTime_.resize(timedPlannerScheduler_.getPlannersNum(), 0.0);
    timedCmdVec_.resize(timedPlannerScheduler_.getPlannersNum());
    for (size_t i = 0; i < timedPlannerScheduler_.getPlannersNum(); ++i) {
        timedCmdVecMtx_.push_back(std::make_unique<std::mutex>());
    }
    timedPlannerScheduler_.setDiffDt(ruckigDt_);
    /******************************************************************/

    /************************躯干重置的参数相关*******************************/
    torsoResetMaxVel_.setZero(6);
    torsoResetMaxVel_ << 0.2, 0.2, 0.2, 1.047, 1.047, 1.047;
    /**********************************************************************/
  }

  void MobileManipulatorReferenceManager::loadParamFromTaskFile(void)
  {
    // 参数初始化
    wheel_move_spd_ .setZero(3);
    wheel_move_acc_.setZero(3);
    wheel_move_jerk_.setZero(3);

    torsoPose_move_spd_.setZero(4);
    torsoPose_move_acc_.setZero(4);
    torsoPose_move_jerk_.setZero(4);

    dualArm_move_spd_.setZero(6);
    dualArm_move_acc_.setZero(6);
    dualArm_move_jerk_.setZero(6);

    legJoint_move_spd_.setZero(1);
    legJoint_move_acc_.setZero(1);
    legJoint_move_jerk_.setZero(1);

    armJoint_move_spd_.setZero(1);
    armJoint_move_acc_.setZero(1);
    armJoint_move_jerk_.setZero(1);

    // 从任务文件中加载参数
    std::string prefix = "referencekinematicLimit.";

    loadData::loadEigenMatrix(taskFile_, prefix + "wheel_move.max_vel", wheel_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "wheel_move.max_acc", wheel_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "wheel_move.max_jerk", wheel_move_jerk_);

    loadData::loadEigenMatrix(taskFile_, prefix + "torsoPose_move.max_vel", torsoPose_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "torsoPose_move.max_acc", torsoPose_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "torsoPose_move.max_jerk", torsoPose_move_jerk_);

    loadData::loadEigenMatrix(taskFile_, prefix + "dualArm_move.max_vel", dualArm_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "dualArm_move.max_acc", dualArm_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "dualArm_move.max_jerk", dualArm_move_jerk_);

    loadData::loadEigenMatrix(taskFile_, prefix + "legJoint_move.max_vel", legJoint_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "legJoint_move.max_acc", legJoint_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "legJoint_move.max_jerk", legJoint_move_jerk_);

    loadData::loadEigenMatrix(taskFile_, prefix + "armJoint_move.max_vel", armJoint_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "armJoint_move.max_acc", armJoint_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "armJoint_move.max_jerk", armJoint_move_jerk_);

    // 打印加载的参数
    std::cout << "[MobileManipulatorReferenceManager] Loaded Parameters from Task File:" << std::endl;
    std::cout << "  wheel_move_spd_: " << wheel_move_spd_.transpose() << std::endl;
    std::cout << "  wheel_move_acc_: " << wheel_move_acc_.transpose() << std::endl;
    std::cout << "  wheel_move_jerk_: " << wheel_move_jerk_.transpose() << std::endl;

    std::cout << "  torsoPose_move_spd_: " << torsoPose_move_spd_.transpose() << std::endl;
    std::cout << "  torsoPose_move_acc_: " << torsoPose_move_acc_.transpose() << std::endl;
    std::cout << "  torsoPose_move_jerk_: " << torsoPose_move_jerk_.transpose() << std::endl;

    std::cout << "  dualArm_move_spd_: " << dualArm_move_spd_.transpose() << std::endl;
    std::cout << "  dualArm_move_acc_: " << dualArm_move_acc_.transpose() << std::endl;
    std::cout << "  dualArm_move_jerk_: " << dualArm_move_jerk_.transpose() << std::endl;

    std::cout << "  legJoint_move_spd_: " << legJoint_move_spd_ << std::endl;
    std::cout << "  legJoint_move_acc_: " << legJoint_move_acc_ << std::endl;
    std::cout << "  legJoint_move_jerk_: " << legJoint_move_jerk_ << std::endl;

    std::cout << "  armJoint_move_spd_: " << armJoint_move_spd_ << std::endl;
    std::cout << "  armJoint_move_acc_: " << armJoint_move_acc_ << std::endl;
    std::cout << "  armJoint_move_jerk_: " << armJoint_move_jerk_ << std::endl;

    /**************************ruckig 时间周期获取************************************/
    double desiredFreq = 0.0;
    loadData::loadCppDataType(taskFile_, "mpc.mpcDesiredFrequency", desiredFreq);
    ruckigDt_ = 1 / desiredFreq;
    /*******************************************************************************/
  }

  void MobileManipulatorReferenceManager::setRobotInitialArmJointTarget(ros::NodeHandle& input_nh)
  {
    std::vector<double> initialStateVector;
    while (!input_nh.hasParam("/robot_init_state_param"))
    {
        static bool first = true;
        if(first)
        {
          ROS_INFO("Waiting for '/robot_init_state_param' parameter to be set...");
          first = false;
        } 
        ros::Duration(0.2).sleep(); // 等待1秒后再次尝试
    }
    input_nh.getParam("/robot_init_state_param", initialStateVector);
    ROS_INFO("Set '/robot_init_state_param' parameter success !!!");

    Eigen::VectorXd initialState(initialStateVector.size());
    for (size_t i = 0; i < initialStateVector.size(); ++i)
    {
        initialState(i) = initialStateVector[i];
    }

    std::cout << "[MobileManipulatorReferenceManager] robot_init_state_param: " << initialState.transpose() << std::endl;

    arm_init_joint_traj_ = initialState.segment(7 + 4, info_.armDim - 4);   // 从初始获取手臂期望

  }

  void MobileManipulatorReferenceManager::setupSubscriptions(std::string nodeHandleName)
  {
    // 从参数服务器中更新初始期望
    setRobotInitialArmJointTarget(nodeHandle_);

    // 设置服务服务器
    controlModeServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_mpc_control", 
                                                           &MobileManipulatorReferenceManager::controlModeService, this);
    getMpcControlModeServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_get_mpc_control_mode",
                                                           &MobileManipulatorReferenceManager::getMpcControlModeService, this);
    changeArmControlService_ = nodeHandle_.advertiseService("wheel_arm_change_arm_ctrl_mode", 
                                                           &MobileManipulatorReferenceManager::armControlModeSrvCallback, this);
    get_arm_control_mode_service_ = nodeHandle_.advertiseService("/humanoid_get_arm_ctrl_mode", 
                                                           &MobileManipulatorReferenceManager::getArmControlModeCallback, this);
    // 设置各 ruckigPlanner 的参数的服务
    setRuckigPlannerParamsServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_set_ruckig_planner_params", 
                                                           &MobileManipulatorReferenceManager::setRuckigPlannerParamsService, this);
    // 设置获取躯干初始位姿服务
    getLbTorsoInitialPoseServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_get_torso_initial_pose", 
                                                           &MobileManipulatorReferenceManager::getLbTorsoInitialPoseService, this);
    
    // 设置基于时间的指令服务
    setLbTimedPosCmdServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_timed_single_cmd", 
                                                           &MobileManipulatorReferenceManager::setLbTimedPosCmdService, this);
    
    // 设置重置躯干指令
    resetTorsoStatusServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_reset_torso", 
                                                           &MobileManipulatorReferenceManager::setLbResetTorsoService, this);


    auto targetVelocityCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdvel_mtx_.lock();
      isCmdVelUpdated_ = true;
      isCmdVelTimeUpdate_ = true;
      cmdVel_[0] = msg->linear.x;
      cmdVel_[1] = msg->linear.y;
      cmdVel_[2] = msg->angular.z;
      cmdvel_mtx_.unlock();
    };
    targetVelocitySubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, targetVelocityCallback);
    
    auto targetVelocityWorldCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdvelWorld_mtx_.lock();
      isCmdVelWorldUpdated_ = true;
      isCmdVelTimeUpdate_ = true;
      cmdVelWorld_[0] = msg->linear.x;
      cmdVelWorld_[1] = msg->linear.y;
      cmdVelWorld_[2] = msg->angular.z;
      cmdvelWorld_mtx_.unlock();
    };
    targetVelocityWorldSubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_vel_world", 1, targetVelocityWorldCallback);
    
    auto targetLbTorsoPoseCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdTorsoPose_mtx_.lock();
      isCmdTorsoPoseUpdated_ = true;
      cmdTorsoPoseDesiredTime_ = 0.0;
      cmdTorsoPose_[0] = msg->linear.x;
      cmdTorsoPose_[1] = msg->linear.y;
      cmdTorsoPose_[2] = msg->linear.z;
      cmdTorsoPose_[3] = msg->angular.z;
      cmdTorsoPose_[4] = msg->angular.y;
      cmdTorsoPose_[5] = msg->angular.x;
      std::cout << "Received cmdTorsoPose: "<< cmdTorsoPose_.transpose() << std::endl;
      cmdTorsoPose_mtx_.unlock();
    };
    targetTorsoPoseSubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_lb_torso_pose", 1, targetLbTorsoPoseCallback);
    
    targetTorsoPoseReachTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/lb_torso_pose_reach_time", 10, false);
    
    auto targetPoseCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdPose_mtx_.lock();
      isCmdPoseUpdated_ = true;
      cmdPoseDesiredTime_ = 0.0;
      cmdPose_[0] = msg->linear.x;
      cmdPose_[1] = msg->linear.y;
      cmdPose_[2] = msg->angular.z;
      std::cout << "Received cmdPose: [" << cmdPose_[0] << ", " << cmdPose_[1] << ", " << cmdPose_[2] << std::endl;
      cmdPose_mtx_.unlock();
    };
    targetPoseSubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_pose", 1, targetPoseCallback);

    auto targetPoseWorldCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdPoseWorld_mtx_.lock();
      isCmdPoseWorldUpdated_ = true;
      cmdPoseDesiredTime_ = 0.0;
      cmdPoseWorld_[0] = msg->linear.x;
      cmdPoseWorld_[1] = msg->linear.y;
      cmdPoseWorld_[2] = msg->angular.z;
      std::cout << "Received cmdPoseWorld: [" << cmdPoseWorld_[0] << ", " << cmdPoseWorld_[1] << ", " << cmdPoseWorld_[2] << std::endl;
      cmdPoseWorld_mtx_.unlock();
    };
    targetPoseWorldSubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_pose_world", 1, targetPoseWorldCallback);
    
    targetCmdPoseReachTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/lb_cmd_pose_reach_time", 10, false);

    // 订阅双臂末端执行器位姿指令
    auto armEndEffectorCallback = [this](const kuavo_msgs::twoArmHandPoseCmd::ConstPtr &msg)
    {
      desireMode_ = handPoseCmdFrameToLbArmMode(msg->frame);
      if(desireMode_ == LbArmControlMode::FalseMode) 
      {
        desireMode_ = LbArmControlMode::JointSpace;
        return;
      }

      armPose_mtx_.lock();
      
      if(desireMode_ == LbArmControlMode::WorldFrame || desireMode_ == LbArmControlMode::LocalFrame)
      {
        isCmdDualArmPoseUpdated_ = true;
        cmdDualArmPoseDesiredTime_ = 0.0;
        
        if(msg->hand_poses.left_pose.joint_angles.size() != 7 || msg->hand_poses.right_pose.joint_angles.size() != 7)
        {
          ROS_ERROR("Left or right arm joint angles size is not 7");
          armPose_mtx_.unlock();
          return;
        }

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
      }

      if(desireMode_ == LbArmControlMode::JointSpace) //进行关节控制时, 进行慢速稳定控制
      {
        for (size_t i = 0; i < msg->hand_poses.left_pose.joint_angles.size(); ++i)
        {
          left_arm_joint_traj_[i] = msg->hand_poses.left_pose.joint_angles[i] * M_PI / 180.0; // 转换为弧度
          right_arm_joint_traj_[i] = msg->hand_poses.right_pose.joint_angles[i] * M_PI / 180.0; // 转换为弧度
        }

        enableQuickJointControl_ = false;
        isCmdArmJointUpdated_ = true;   // 触发关节指令规划
        cmdArmJointDesiredTime_ = 0.0;
      }
      
      armPose_mtx_.unlock();
    };
    armEndEffectorSubscriber_ =
        nodeHandle_.subscribe<kuavo_msgs::twoArmHandPoseCmd>("/mm/two_arm_hand_pose_cmd", 1, armEndEffectorCallback);
    
    armEndEffectorReachTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/lb_arm_ee_reach_time", 10, false);
    
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

      desireMode_ = LbArmControlMode::JointSpace;  // 切换模式
      enableQuickJointControl_ = true;  // 使用该话题时，可以选择触发wbc独立控制的服务
      
      armJoint_mtx_.unlock();

      cmdArmJointDesiredTime_ = 0.0;
      isCmdArmJointUpdated_ = true;   // 触发关节指令规划
    };
    arm_joint_traj_sub_ = nodeHandle_.subscribe<sensor_msgs::JointState>("/kuavo_arm_traj", 10, armJointTrajCallback);

    targetArmJointReachTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/lb_arm_joint_reach_time", 10, false);

    // 添加订阅/lb_leg_traj话题
    auto lbLegJointTrajCallback = [this](const sensor_msgs::JointState::ConstPtr &msg)
    {
      if(msg->position.size() != 4)  // 数据维度检查
      {
        std::cout << "[MobileManipulatorReferenceManager] 下肢关节轨迹维度错误! 期望4, 实际 " 
                  << msg->position.size() << std::endl;
        return;
      }
      
      // 解析关节角度数据
      lbLegJoint_mtx_.lock();
      lb_leg_traj_ = vector_t::Zero(msg->position.size());
      for (size_t i = 0; i < msg->position.size(); ++i)
      {
        lb_leg_traj_[i] = msg->position[i] * M_PI / 180.0; // 转换为弧度
      }
      cmdLegJointDesiredTime_ = 0.0;
      lbLegJoint_mtx_.unlock();

      isCmdLegJointUpdated_ = true;
    };
    lb_leg_joint_traj_sub_ = nodeHandle_.subscribe<sensor_msgs::JointState>("/lb_leg_traj", 10, lbLegJointTrajCallback);

    // 设置笛卡尔跟踪 focus 躯干还是末端
    auto setFocusEeCallback = [this](const std_msgs::Bool::ConstPtr &msg)
    {
      bool flag = msg->data;
      setIsFocusEeStatus(flag);
      ROS_INFO_STREAM("[setFocusEeCallback] focus_ee: [ " << flag << " ]");
    };
    set_focus_ee_sub_ = nodeHandle_.subscribe<std_msgs::Bool>("/mobile_manipulator_focus_ee", 10, setFocusEeCallback);

    targetLegJointReachTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/lb_leg_joint_reach_time", 10, false);

    // 发布轮臂MPC当前控制模式
    mpcControlModePub_ = nodeHandle_.advertise<std_msgs::Int8>("/mobile_manipulator/lb_mpc_control_mode", 10, false);

    // 发布轮臂MPC的约束使用情况
    mpcConstraintUsagePub_ = nodeHandle_.advertise<std_msgs::Int8MultiArray>("/mobile_manipulator/lb_mpc_constraint_usage", 10, false);

    // 发布在modifyReference函数消耗的时间
    modifyReferenceTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/mobile_manipulator/lb_mpc_modify_ref_use_time", 10, false);
  }

  // 获取第一次的目标轨迹，并分配到不同的约束轨迹，后续添加额外约束, 也需要在此初始化
  void MobileManipulatorReferenceManager::getFirstTargetTrajectories(const TargetTrajectories& targetTrajectories)
  {
    // 第一次的轨迹, 包括 state, input, 躯干相对底座位姿, 所有手臂末端位姿
    stateInputTargetTrajectories_.stateTrajectory.front()= targetTrajectories.stateTrajectory.front();
    stateInputTargetTrajectories_.inputTrajectory.front() = targetTrajectories.inputTrajectory.front();
    stateInputTargetTrajectories_.timeTrajectory.front() = targetTrajectories.timeTrajectory.front();

    torsoTargetTrajectories_.timeTrajectory.front() = targetTrajectories.timeTrajectory.front();
    torsoTargetTrajectories_.stateTrajectory.front() = targetTrajectories.stateTrajectory.front().segment(baseDim_, 7);

    eeTargetTrajectories_.timeTrajectory.front() = targetTrajectories.timeTrajectory.front();
    eeTargetTrajectories_.stateTrajectory.front() = targetTrajectories.stateTrajectory.front().segment(baseDim_ + 7, info_.eeFrames.size() * 7);
  }

  // 获取所有的目标轨迹，并分配到不同的约束轨迹
  void MobileManipulatorReferenceManager::getAllTargetTrajectories(const TargetTrajectories& targetTrajectories)
  {

    for(int i=0; i<targetTrajectories.timeTrajectory.size(); i++)
    {
      stateInputTargetTrajectories_.timeTrajectory[i] = targetTrajectories.timeTrajectory[i];
      stateInputTargetTrajectories_.stateTrajectory[i].head(baseDim_) = targetTrajectories.stateTrajectory[i].head(baseDim_);

      torsoTargetTrajectories_.timeTrajectory[i] = targetTrajectories.timeTrajectory[i];
      torsoTargetTrajectories_.stateTrajectory[i] = targetTrajectories.stateTrajectory[i].segment(baseDim_, 7);

      eeTargetTrajectories_.timeTrajectory[i] = targetTrajectories.timeTrajectory[i];
      eeTargetTrajectories_.stateTrajectory[i] = targetTrajectories.stateTrajectory[i].segment(baseDim_ + 7, info_.eeFrames.size() * 7);
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
    trimTrajectory(stateInputTargetTrajectories_);
    trimTrajectory(torsoTargetTrajectories_);
    trimTrajectory(eeTargetTrajectories_);
  }

  vector_t MobileManipulatorReferenceManager::targetTrajToPose6D(const TargetTrajectories& Traj, scalar_t initTime)
  {
    const auto& timeTraj = Traj.timeTrajectory;
    const auto& stateTraj = Traj.stateTrajectory;

    vector_t position;
    Eigen::Quaterniond orientation;

    if (stateTraj.size() > 1) {
      // Normal interpolation case
      int index;
      scalar_t alpha;
      std::tie(index, alpha) = LinearInterpolation::timeSegment(initTime, timeTraj);

      const auto& lhs = stateTraj[index].head(7);
      const auto& rhs = stateTraj[index + 1].head(7);
      const Eigen::Quaterniond q_lhs(lhs.tail<4>());
      const Eigen::Quaterniond q_rhs(rhs.tail<4>());

      position = alpha * lhs.head(3) + (1.0 - alpha) * rhs.head(3);
      orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {  // stateTrajectory.size() == 1
      position = stateTraj.front().head(7).head(3);
      orientation = Eigen::Quaterniond(stateTraj.front().head(7).tail<4>());
    }

    vector_t zyx = quatToZyx(orientation);
    vector_t pose6D = vector_t::Zero(6);
    pose6D << position, zyx;

    return pose6D;
  }

  vector_t MobileManipulatorReferenceManager::targetTorsoTrajToPose6DContinous(const TargetTrajectories& Traj, scalar_t initTime)
  {
    const auto& timeTraj = Traj.timeTrajectory;
    const auto& stateTraj = Traj.stateTrajectory;

    vector_t position;
    Eigen::Quaterniond orientation;

    if (stateTraj.size() > 1) {
      // Normal interpolation case
      int index;
      scalar_t alpha;
      std::tie(index, alpha) = LinearInterpolation::timeSegment(initTime, timeTraj);

      const auto& lhs = stateTraj[index].head(7);
      const auto& rhs = stateTraj[index + 1].head(7);
      const Eigen::Quaterniond q_lhs(lhs.tail<4>());
      const Eigen::Quaterniond q_rhs(rhs.tail<4>());

      position = alpha * lhs.head(3) + (1.0 - alpha) * rhs.head(3);
      orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {  // stateTrajectory.size() == 1
      position = stateTraj.front().head(7).head(3);
      orientation = Eigen::Quaterniond(stateTraj.front().head(7).tail<4>());
    }

    // 使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
    static ContinuousEulerAnglesFromMatrix eulerUnwrapper;
    Eigen::Vector3d zyx = eulerUnwrapper.update(orientation);
    
    vector_t torsoPose6D = vector_t::Zero(6);
    torsoPose6D << position, zyx;

    return torsoPose6D;
  }

  vector_t MobileManipulatorReferenceManager::targetEeTrajToPose6DContinous(const TargetTrajectories& Traj, scalar_t initTime, int eeInx)
  {
    const auto& timeTraj = Traj.timeTrajectory;
    const auto& stateTraj = Traj.stateTrajectory;

    vector_t position;
    Eigen::Quaterniond orientation;

    if (stateTraj.size() > 1) {
      // Normal interpolation case
      int index;
      scalar_t alpha;
      std::tie(index, alpha) = LinearInterpolation::timeSegment(initTime, timeTraj);

      const auto& lhs = stateTraj[index].head(7);
      const auto& rhs = stateTraj[index + 1].head(7);
      const Eigen::Quaterniond q_lhs(lhs.tail<4>());
      const Eigen::Quaterniond q_rhs(rhs.tail<4>());

      position = alpha * lhs.head(3) + (1.0 - alpha) * rhs.head(3);
      orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {  // stateTrajectory.size() == 1
      position = stateTraj.front().head(7).head(3);
      orientation = Eigen::Quaterniond(stateTraj.front().head(7).tail<4>());
    }

    // 使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
    static std::vector<ContinuousEulerAnglesFromMatrix> eulerUnwrapper(info_.eeFrames.size());
    Eigen::Vector3d zyx = eulerUnwrapper[eeInx].update(orientation);

    vector_t pose6D = vector_t::Zero(6);
    pose6D << position, zyx;

    return pose6D;
  }

  void MobileManipulatorReferenceManager::publishTargetTrajectoriesNear(scalar_t initTime)
  {
    ros_logger_->publishVector("mobile_manipulator/currentMpcTarget/state", stateInputTargetTrajectories_.getDesiredState(initTime));
    ros_logger_->publishVector("mobile_manipulator/currentMpcTarget/input", stateInputTargetTrajectories_.getDesiredInput(initTime));

    vector_t torsoTraj = targetTorsoTrajToPose6DContinous(torsoTargetTrajectories_, initTime);
    ros_logger_->publishVector("mobile_manipulator/torso_target_6D", torsoTraj);
 
    std::vector<TargetTrajectories> dualArmEeTraj;
    dualArmEeTraj.resize(info_.eeFrames.size());
    for(int i=0; i < info_.eeFrames.size(); i++)
    {
      dualArmEeTraj[i].timeTrajectory = eeTargetTrajectories_.timeTrajectory;
      dualArmEeTraj[i].stateTrajectory.resize(eeTargetTrajectories_.stateTrajectory.size());
      for(int j=0; j < eeTargetTrajectories_.stateTrajectory.size(); j++)
      {
        dualArmEeTraj[i].stateTrajectory[j] = eeTargetTrajectories_.stateTrajectory[j].segment<7>(i*7);
      }
      vector_t armEeTraj = targetEeTrajToPose6DContinous(dualArmEeTraj[i], initTime, i);
      ros_logger_->publishVector("mobile_manipulator/ee_target_6D/point" + std::to_string(i), armEeTraj);
    }
  }

  void MobileManipulatorReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule)
  {
    // 更新形参作为全局变量
    initTime_ = initTime;
    finalTime_ = finalTime;
    initState_ = initState;
    // 底盘状态采用期望, 避免模式切换时滑动
    initState_.head(baseDim_) = stateInputTargetTrajectories_.getDesiredState(initTime).head(baseDim_); 
    
    /************************************ 更新状态差分 **************************************/
    Eigen::VectorXd dState, ddState;
    static CentralDifferenceDifferentiator stateDiff;
    static double stateDiffTime;
    stateDiffTime += ruckigDt_;   // 采用稳定的时间步长进行差分
    stateDiff.differentiate(initState_, stateDiffTime, dState, ddState);
    /**************************************************************************************/

    // 获取开始时间点
    auto startTime = std::chrono::system_clock::now();

    // 第一次进入，需要对原始轨迹数据进行初始化
    static bool firstRun{true};
    if(firstRun)
    {
      // 获取最初的躯干位姿期望
      initialTorsoPos_ = targetTrajectories.stateTrajectory.front().segment(baseDim_, 3);
      initialTorsoQuat_ = targetTrajectories.stateTrajectory.front().segment(baseDim_ + 3, 4);
      getFirstTargetTrajectories(targetTrajectories);
      firstRun = false;
    }

    // 更新当前状态
    updateTimedSchedulerCurrentState(initTime_, initState_);

    // 更新当前期望
    updateTimedSchedulerTargetTraj();
    
    // 获取当前控制模式
    // controlMode_mtx_.lock();
    // int currentMode = currentMpcControlMode_;
    // controlMode_mtx_.unlock();

    // 判断模式是否发生切换，切换则使能切换标志
    static bool isChange{false}; // = getControlModeIsChange(currentMode);

    // switch(currentMode) // 0: NoControl, 1: ArmOnly, 2: BaseOnly, 3: BaseArm
    // {
    //   case MpcControlMode::NoControl: 
    //     // updateNoControl(initTime, targetTrajectories, isChange); break;           // 模式0: 使用上层下发的 targetTrajectories, 

    //   case MpcControlMode::ArmOnly:   
    //     // updateArmOnlyControl(initTime, finalTime, initState, isChange); break;               // 模式1: 关节可动, 底盘锁住

    //   case MpcControlMode::BaseOnly:  
    //     // updateBaseOnlyControl(initTime, finalTime, initState, isChange); break;   // 模式2: 底盘可动, 下肢和手臂锁住

      // 确认是否需要重置
      if(isResetTorso_)
      {
        resetAllMpcTrajAndTarget(initTime, initState_);
        isResetTorso_ = false;
      }
      // case MpcControlMode::BaseArm:   
      updateBaseArmControl(initTime, finalTime, initState_, isChange);   // 模式3: 必须控制底盘, 手臂支持局部系和世界系笛卡尔和关节两种轨迹
        
      // case MpcControlMode::ArmEeOnly: 
      //   updateArmEeOnlyControl(initTime, finalTime, initState, isChange); break;             // 模式4: 底盘随末端移动, 不可控制, 手臂支持世界系笛卡尔轨迹
        
    //   default: std::cout << "设置了错误的控制模式, 请检查!!" << std::endl;
    // }

    // 对多个轨迹进行裁剪, 删除之前无效的轨迹
    trimTargetTrajectoriesBeforeTime(initTime);

    // 发布时间最近的目标轨迹
    publishTargetTrajectoriesNear(initTime);

    // 发布当前MPC控制模式
    std_msgs::Int8 modeMsg;
    modeMsg.data = MpcControlMode::BaseArm;
    mpcControlModePub_.publish(modeMsg);

    // 发布mpc使能各约束的标志, 按底盘, 下肢关节, 躯干, 手臂关节, 手臂末端轨迹顺序
    std_msgs::Int8MultiArray constraintUsageMsg;
    std::vector<int8_t> constraintUsageVec;
    constraintUsageVec.push_back(getEnableBaseTrack() ? 1 : 0);
    constraintUsageVec.push_back(getEnableLegJointTrack() ? 1 : 0);
    constraintUsageVec.push_back(getEnableTorsoPoseTargetTrajectories() ? 1 : 0);
    constraintUsageVec.push_back(getEnableArmJointTrack() ? 1 : 0);
    constraintUsageVec.push_back(getEnableEeTargetTrajectories() ? 1 : 0);
    constraintUsageVec.push_back(getEnableEeTargetLocalTrajectories() ? 1 : 0);
    constraintUsageMsg.data = constraintUsageVec;
    mpcConstraintUsagePub_.publish(constraintUsageMsg);

    // 获取结束时间点
    auto endTime = std::chrono::system_clock::now();

    // state
    ros_logger_->publishVector("/mobile_manipulator/initState_state", initState_);
    ros_logger_->publishVector("/mobile_manipulator/initState_dState", dState);
    ros_logger_->publishVector("/mobile_manipulator/initState_ddState", ddState);

    publishMultiPointPose_World(initState_);
    publishMultiPointPose_Local(initState_);

    // 发布 modifyReference 消耗时间，单位: 毫秒
    std_msgs::Float32 cntTimeMsg;
    cntTimeMsg.data = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    modifyReferenceTimePub_.publish(cntTimeMsg);
  }

  // 本体系的末端ruckig轨迹生成
  void MobileManipulatorReferenceManager::calcRuckigTrajWithEePose(double initTime, const vector_t &targetArmEePose, double desiredTime)
  {
    assert(targetArmEePose.size() == info_.eeFrames.size() * 7 && "dualArmPose dimension must be info_.eeFrames.size() * 7!");

    vector_t target6DEePose = vector_t::Zero(info_.eeFrames.size() * 6);
    for(int i=0; i<info_.eeFrames.size(); i++)
    {
      // 目标末端执行器位姿转换为6D表示
      target6DEePose.segment(i*6, 3) = targetArmEePose.segment(i*7, 3);
      target6DEePose.segment(i*6 + 3, 3) = quatToZyx(Eigen::Quaterniond(targetArmEePose.segment<4>(i*7 + 3)));
    }

    cmdDualArm_plannerInitialTime_ = initTime;
    cmdDualArmEePlannerRuckigPtr_->setCurrentPose(cmdDualArm_prevTargetPose_);
    cmdDualArmEePlannerRuckigPtr_->setCurrentVelocity(cmdDualArm_prevTargetVel_);
    cmdDualArmEePlannerRuckigPtr_->setCurrentAcceleration(cmdDualArm_prevTargetAcc_);

    cmdDualArmEePlannerRuckigPtr_->setTargetPose(target6DEePose);
    double durationTime = cmdDualArmEePlannerRuckigPtr_->calcTrajectory(desiredTime);
   
    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    armEndEffectorReachTimePub_.publish(time_msg); // 发布到达时间
  }

  // 从 ruckig 中取出对应时间戳的6D位姿, 转换为约束可接收的 target 格式, 下发
  void MobileManipulatorReferenceManager::generateDualArmEeTargetWithRuckig(double initTime, double finalTime, double dt)
  {
    // 使用 Ruckig 库生成平滑的手臂末端位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::VectorXd currentTargetPose, currentTargetVel, currentTargetAcc;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(info_.eeFrames.size() * 7);
    vector_t targetInput = vector_t::Zero(info_.eeFrames.size() * 7);

    for(int i=0; i<timeIncrement; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度（双臂轨迹）
      double currentTime = initTime + (i+1) * dt;

      cmdDualArmEePlannerRuckigPtr_->getTrajectoryAtTime(currentTime - cmdDualArm_plannerInitialTime_, 
                                                         currentTargetPose, currentTargetVel, currentTargetAcc);

      for(int i=0; i<info_.eeFrames.size(); i++)
      {
        targetState.segment(i*7, 3) = currentTargetPose.segment(i*6, 3);
        
        // 使用 Eigen 将 ZYX 欧拉角转换为四元数
        Eigen::Quaterniond quat = Eigen::AngleAxisd(currentTargetPose(i*6+3), Eigen::Vector3d::UnitZ())   // yaw (Z)
                                * Eigen::AngleAxisd(currentTargetPose(i*6+4), Eigen::Vector3d::UnitY()) // pitch (Y)
                                * Eigen::AngleAxisd(currentTargetPose(i*6+5), Eigen::Vector3d::UnitX()); // roll (X)
        targetState.segment(i*7 + 3, 4) = quat.coeffs();
      }

      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    eeTargetTrajectories_.timeTrajectory = timeTraj;
    eeTargetTrajectories_.stateTrajectory = stateTraj;
    eeTargetTrajectories_.inputTrajectory = inputTraj;

    // 保存当前时间的规划期望
    cmdDualArmEePlannerRuckigPtr_->getTrajectoryAtTime(initTime - cmdDualArm_plannerInitialTime_ + dt, 
                                                       cmdDualArm_prevTargetPose_,
                                                       cmdDualArm_prevTargetVel_,
                                                       cmdDualArm_prevTargetAcc_);
  }

  // 重置双臂末端Zyx插值器的初值
  void MobileManipulatorReferenceManager::resetDualArmRuckig(double initTime, const vector_t& initState, bool rePlanning, LbArmControlMode desireMode)
  {
    Eigen::VectorXd currentEePose, currentEeVel, currentEeAcc;

    switch (desireMode)
    {
      case LbArmControlMode::WorldFrame: 
        timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::DUAL_ARM_WORLD_CMD, 
                                                     currentEePose, currentEeVel, currentEeAcc);
        break;
      case LbArmControlMode::LocalFrame: 
        timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::DUAL_ARM_LOCAL_CMD, 
                                                     currentEePose, currentEeVel, currentEeAcc);
        break;
      case LbArmControlMode::JointSpace: return;
      default:
        std::cerr << "[resetDualArmRuckig] 不支持该模式的末端轨迹生成, 返回" << std::endl;
        return;
    }

    cmdDualArm_prevTargetPose_ = currentEePose;
    cmdDualArm_prevTargetVel_ = currentEeVel;
    cmdDualArm_prevTargetAcc_ = currentEeAcc;

    if(rePlanning)
    {
      cmdDualArm_plannerInitialTime_ = initTime;
      cmdDualArmEePlannerRuckigPtr_->setCurrentPose(cmdDualArm_prevTargetPose_);
      cmdDualArmEePlannerRuckigPtr_->setCurrentVelocity(cmdDualArm_prevTargetVel_);
      cmdDualArmEePlannerRuckigPtr_->setCurrentAcceleration(cmdDualArm_prevTargetAcc_);

      cmdDualArmEePlannerRuckigPtr_->setTargetPose(cmdDualArm_prevTargetPose_);
      double durationTime = cmdDualArmEePlannerRuckigPtr_->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithTorsoPose(double initTime, const vector_t &targetTorsoPose, double desiredTime)
  {
    assert(targetTorsoPose.size() == 4 && "torsoPose dimension must be 4!");

    torsoPose_plannerInitialTime_ = initTime;
    torsoPosePlannerRuckigPtr_->setCurrentPose(torsoPose_prevTargetPose_);
    torsoPosePlannerRuckigPtr_->setCurrentVelocity(torsoPose_prevTargetVel_);
    torsoPosePlannerRuckigPtr_->setCurrentAcceleration(torsoPose_prevTargetAcc_);

    torsoPosePlannerRuckigPtr_->setTargetPose(targetTorsoPose);
    double durationTime = torsoPosePlannerRuckigPtr_->calcTrajectory(desiredTime);

    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    targetTorsoPoseReachTimePub_.publish(time_msg); // 发布到达时间
  }

  void MobileManipulatorReferenceManager::generateTorsoPoseTargetWithRuckig(double initTime, double finalTime, double dt)
  {
    // 使用 Ruckig 库生成平滑的躯干位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::VectorXd currentTargetPose_torso, currentTargetVel_torso, currentTargetAcc_torso;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(7);
    vector_t targetInput = vector_t::Zero(7);

    for(int i=0; i<timeIncrement; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度
      double currentTime = initTime + (i+1) * dt;

      torsoPosePlannerRuckigPtr_->getTrajectoryAtTime(currentTime - torsoPose_plannerInitialTime_, 
                                                      currentTargetPose_torso, 
                                                      currentTargetVel_torso, 
                                                      currentTargetAcc_torso);
      
      targetState[0] = currentTargetPose_torso[0];
      targetState[1] = initialTorsoPos_[1];
      targetState[2] = currentTargetPose_torso[1];

      // 使用 Eigen 将 ZYX 欧拉角转换为旋转矩阵
      Eigen::Matrix3d torsoRot = getRotationMatrixFromZyxEulerAngles(Eigen::Vector3d(currentTargetPose_torso[2], currentTargetPose_torso[3], 0.0));
      
      // 通过只有pitch的旋转矩阵转换到base坐标系
      Eigen::Matrix3d rotPitchOnly = getRotationMatrixFromZyxEulerAngles(Eigen::Vector3d(0, currentTargetPose_torso[3], 0));
      Eigen::Vector3d baseZyx = quatToZyx(Eigen::Quaterniond(rotPitchOnly * torsoRot));

      Eigen::Quaterniond baseQuat = getQuaternionFromEulerAnglesZyx(Eigen::Vector3d(baseZyx[0], currentTargetPose_torso[3], baseZyx[2]));
      
      targetState.tail(4) = baseQuat.coeffs();

      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    torsoTargetTrajectories_.timeTrajectory = timeTraj;
    torsoTargetTrajectories_.stateTrajectory = stateTraj;
    torsoTargetTrajectories_.inputTrajectory = inputTraj;

    // 保存当前时间的规划期望
    torsoPosePlannerRuckigPtr_->getTrajectoryAtTime(initTime - cmdDualArm_plannerInitialTime_ + dt, 
                                                    torsoPose_prevTargetPose_,
                                                    torsoPose_prevTargetVel_,
                                                    torsoPose_prevTargetAcc_);

  }

  void MobileManipulatorReferenceManager::resetTorsoPoseRuckig(double initTime, const vector_t& initState, bool rePlanning)
  {
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::TORSO_POSE_CMD, 
                                                 torsoPose_prevTargetPose_, 
                                                 torsoPose_prevTargetVel_, 
                                                 torsoPose_prevTargetAcc_);

    if(rePlanning)
    {
      torsoPose_plannerInitialTime_ = initTime;
      torsoPosePlannerRuckigPtr_->setCurrentPose(torsoPose_prevTargetPose_);
      torsoPosePlannerRuckigPtr_->setCurrentVelocity(torsoPose_prevTargetVel_);
      torsoPosePlannerRuckigPtr_->setCurrentAcceleration(torsoPose_prevTargetAcc_);

      torsoPosePlannerRuckigPtr_->setTargetPose(torsoPose_prevTargetPose_);
      double durationTime = torsoPosePlannerRuckigPtr_->calcTrajectory();
    }

  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithCmdPose(double initTime, const vector_t &targetBasePose, double desiredTime)
  {
    assert(targetBasePose.size() == baseDim_ && "cmdPose dimension must be baseDim_!");

    plannerInitialTime_ = initTime;
    cmdPosePlannerRuckigPtr_->setCurrentPose(prevTargetPose_);
    cmdPosePlannerRuckigPtr_->setCurrentVelocity(prevTargetVel_);
    cmdPosePlannerRuckigPtr_->setCurrentAcceleration(prevTargetAcc_);

    cmdPosePlannerRuckigPtr_->setTargetPose(targetBasePose);
    double durationTime = cmdPosePlannerRuckigPtr_->calcTrajectory(desiredTime);

    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    targetCmdPoseReachTimePub_.publish(time_msg); // 发布到达时间
  }

  void MobileManipulatorReferenceManager::generatePoseTargetWithRuckig(double initTime, double finalTime, double dt)
  {
    // 使用 Ruckig 库生成平滑的底盘位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::VectorXd currentTargetPose, currentTargetVel, currentTargetAcc;
    Eigen::VectorXd currentTargetPose_legJoint, currentTargetVel_legJoint, currentTargetAcc_legJoint;
    Eigen::VectorXd currentTargetPose_armJoint, currentTargetVel_armJoint, currentTargetAcc_armJoint;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(info_.stateDim);
    vector_t targetInput = vector_t::Zero(info_.inputDim);
    
    for(int i=0; i<timeIncrement; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度
      double currentTime = initTime + (i+1) * dt;

      cmdPosePlannerRuckigPtr_->getTrajectoryAtTime(currentTime - plannerInitialTime_, currentTargetPose, currentTargetVel, currentTargetAcc);
      legJointPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - legJoint_plannerInitialTime_, currentTargetPose_legJoint, currentTargetVel_legJoint, currentTargetAcc_legJoint);
      armJointPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - armJoint_plannerInitialTime_, currentTargetPose_armJoint, currentTargetVel_armJoint, currentTargetAcc_armJoint);

      targetState.head(baseDim_) = currentTargetPose; // [x, y, yaw]
      targetInput.head(baseDim_) = currentTargetVel;  // [vx, vy, wz]

      targetState.segment(baseDim_, 4) = currentTargetPose_legJoint;  // 下肢4自由度
      targetInput.segment(baseDim_, 4) = currentTargetVel_legJoint;

      targetState.tail(info_.armDim - 4) = currentTargetPose_armJoint;  // 上肢14自由度
      targetInput.tail(info_.armDim - 4) = currentTargetVel_armJoint;

      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    stateInputTargetTrajectories_.timeTrajectory = timeTraj;
    stateInputTargetTrajectories_.stateTrajectory = stateTraj;
    stateInputTargetTrajectories_.inputTrajectory = inputTraj;

    // 保存当前时间的规划期望
    cmdPosePlannerRuckigPtr_->getTrajectoryAtTime(initTime - plannerInitialTime_ + dt, 
                                                  prevTargetPose_,
                                                  prevTargetVel_,
                                                  prevTargetAcc_);
    legJointPlannerRuckigPtr_->getTrajectoryAtTime(initTime - legJoint_plannerInitialTime_ + dt, 
                                                   legJoint_prevTargetPose_, 
                                                   legJoint_prevTargetVel_, 
                                                   legJoint_prevTargetAcc_);
    armJointPlannerRuckigPtr_->getTrajectoryAtTime(initTime - armJoint_plannerInitialTime_ + dt, 
                                                   armJoint_prevTargetPose_, 
                                                   armJoint_prevTargetVel_, 
                                                   armJoint_prevTargetAcc_);

  }

  void MobileManipulatorReferenceManager::resetCmdPoseRuckig(double initTime, const vector_t& initState, bool rePlanning)
  {
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::BASE_POS_WORLD_CMD, 
                                                 prevTargetPose_, prevTargetVel_, prevTargetAcc_);

    if(rePlanning)
    {
      plannerInitialTime_ = initTime;
      cmdPosePlannerRuckigPtr_->setCurrentPose(prevTargetPose_);
      cmdPosePlannerRuckigPtr_->setCurrentVelocity(prevTargetVel_);
      cmdPosePlannerRuckigPtr_->setCurrentAcceleration(prevTargetAcc_);

      cmdPosePlannerRuckigPtr_->setTargetPose(prevTargetPose_);
      double durationTime = cmdPosePlannerRuckigPtr_->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithCmdVel(double initTime, const vector_t &targetBaseVel)
  {
    cmdVel_plannerInitialTime_ = initTime;
    cmdVelPlannerRuckigPtr_->setCurrentPose(cmdVel_prevTargetPose_);
    cmdVelPlannerRuckigPtr_->setCurrentVelocity(cmdVel_prevTargetVel_);
    cmdVelPlannerRuckigPtr_->setCurrentAcceleration(cmdVel_prevTargetAcc_);
    cmdVelPlannerRuckigPtr_->setTargetVelocity(targetBaseVel);

    cmdVelPlannerRuckigPtr_->calcTrajectory();
  }

  void MobileManipulatorReferenceManager::generateVelTargetBaseWithRuckig(double initTime, double finalTime, double dt, 
                                                                          const vector_t &initState)
  {
    // 提取初始位姿 [x, y, yaw]
    Eigen::Vector2d currentPos = initState.head(2);  // [x, y]
    scalar_t currentYaw = initState(2);              // yaw角度

    // 使用 Ruckig 库生成平滑的底盘位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::Vector3d velWorld = Eigen::Vector3d::Zero();   // 世界系速度

    Eigen::VectorXd currentTargetPose, currentTargetVel, currentTargetAcc;
    Eigen::VectorXd currentTargetPose_legJoint, currentTargetVel_legJoint, currentTargetAcc_legJoint;
    Eigen::VectorXd currentTargetPose_armJoint, currentTargetVel_armJoint, currentTargetAcc_armJoint;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(info_.stateDim);
    vector_t targetInput = vector_t::Zero(info_.inputDim);

    for(int i=1; i<timeIncrement; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度
      double currentTime = initTime + (i+1) * dt;

      cmdVelPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - cmdVel_plannerInitialTime_, 
                                                   currentTargetPose, 
                                                   currentTargetVel, 
                                                   currentTargetAcc);
      legJointPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - legJoint_plannerInitialTime_, 
                                                     currentTargetPose_legJoint, 
                                                     currentTargetVel_legJoint, 
                                                     currentTargetAcc_legJoint);
      armJointPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - armJoint_plannerInitialTime_, 
                                                     currentTargetPose_armJoint, 
                                                     currentTargetVel_armJoint, 
                                                     currentTargetAcc_armJoint);

      if (i > 0) 
      {
          // 从第二个时间点开始才进行积分更新
          currentYaw += currentTargetVel[2] * dt;  // 更新偏航角期望

          Eigen::Matrix3d rotMat = Eigen::AngleAxisd(currentYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
          velWorld = rotMat * currentTargetVel;

          currentPos(0) += velWorld(0) * dt ;  // x
          currentPos(1) += velWorld(1) * dt;  // y

          // 记录当前下一帧的位置期望
          static bool isFirst = true;
          if(isFirst)
          {
            cmdVel_prevTargetPose_.head(2) = currentPos;
            cmdVel_prevTargetPose_(2) = currentYaw;
            isFirst = false;
          }
      }

      targetState.head(2) = currentPos.head(2); // [x, y]
      targetState(2) = currentYaw;

      targetInput.head(2) = velWorld.head(3);  // [vx, vy, vyaw]
      targetInput(2) = currentTargetVel[2];

      targetState.segment(baseDim_, 4) = currentTargetPose_legJoint;  // 下肢4自由度
      targetInput.segment(baseDim_, 4) = currentTargetVel_legJoint;

      targetState.tail(info_.armDim - 4) = currentTargetPose_armJoint;  // 上肢14自由度
      targetInput.tail(info_.armDim - 4) = currentTargetVel_armJoint;

      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    stateInputTargetTrajectories_.timeTrajectory = timeTraj;
    stateInputTargetTrajectories_.stateTrajectory = stateTraj;
    stateInputTargetTrajectories_.inputTrajectory = inputTraj;

    // 保存下一帧的规划期望
    Eigen::VectorXd dummy_position;
    cmdVelPlannerRuckigPtr_->getTrajectoryAtTime(initTime - cmdVel_plannerInitialTime_ + dt, 
                                                  dummy_position,
                                                  cmdVel_prevTargetVel_,
                                                  cmdVel_prevTargetAcc_);
    legJointPlannerRuckigPtr_->getTrajectoryAtTime(initTime - legJoint_plannerInitialTime_ + dt, 
                                                   legJoint_prevTargetPose_, 
                                                   legJoint_prevTargetVel_, 
                                                   legJoint_prevTargetAcc_);
    armJointPlannerRuckigPtr_->getTrajectoryAtTime(initTime - armJoint_plannerInitialTime_ + dt, 
                                                   armJoint_prevTargetPose_, 
                                                   armJoint_prevTargetVel_, 
                                                   armJoint_prevTargetAcc_);
  }

  void MobileManipulatorReferenceManager::generateVelTargetWithRuckig(double initTime, double finalTime, double dt)
  {
    // 使用 Ruckig 库生成平滑的底盘位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::VectorXd currentTargetPose, currentTargetVel, currentTargetAcc;
    Eigen::VectorXd currentTargetPose_legJoint, currentTargetVel_legJoint, currentTargetAcc_legJoint;
    Eigen::VectorXd currentTargetPose_armJoint, currentTargetVel_armJoint, currentTargetAcc_armJoint;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(info_.stateDim);
    vector_t targetInput = vector_t::Zero(info_.inputDim);

    for(int i=0; i<timeIncrement; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度
      double currentTime = initTime + (i+1) * dt;

      cmdVelPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - cmdVel_plannerInitialTime_, 
                                                   currentTargetPose, 
                                                   currentTargetVel, 
                                                   currentTargetAcc);
      legJointPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - legJoint_plannerInitialTime_, 
                                                     currentTargetPose_legJoint, 
                                                     currentTargetVel_legJoint, 
                                                     currentTargetAcc_legJoint);
      armJointPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - armJoint_plannerInitialTime_, 
                                                     currentTargetPose_armJoint, 
                                                     currentTargetVel_armJoint, 
                                                     currentTargetAcc_armJoint);

      targetState.head(3) = currentTargetPose; // [x, y, yaw]
      targetInput.head(3) = currentTargetVel;  // [vx, vy, wz]

      targetState.segment(baseDim_, 4) = currentTargetPose_legJoint;  // 下肢4自由度
      targetInput.segment(baseDim_, 4) = currentTargetVel_legJoint;

      targetState.tail(info_.armDim - 4) = currentTargetPose_armJoint;  // 上肢14自由度
      targetInput.tail(info_.armDim - 4) = currentTargetVel_armJoint;

      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    stateInputTargetTrajectories_.timeTrajectory = timeTraj;
    stateInputTargetTrajectories_.stateTrajectory = stateTraj;
    stateInputTargetTrajectories_.inputTrajectory = inputTraj;

    // 保存当前时间的规划期望
    cmdVelPlannerRuckigPtr_->getTrajectoryAtTime(initTime - cmdVel_plannerInitialTime_ + dt, 
                                                  cmdVel_prevTargetPose_,
                                                  cmdVel_prevTargetVel_,
                                                  cmdVel_prevTargetAcc_);
    legJointPlannerRuckigPtr_->getTrajectoryAtTime(initTime - legJoint_plannerInitialTime_ + dt, 
                                                   legJoint_prevTargetPose_, 
                                                   legJoint_prevTargetVel_, 
                                                   legJoint_prevTargetAcc_);
    armJointPlannerRuckigPtr_->getTrajectoryAtTime(initTime - armJoint_plannerInitialTime_ + dt, 
                                                   armJoint_prevTargetPose_, 
                                                   armJoint_prevTargetVel_, 
                                                   armJoint_prevTargetAcc_);
  }

  void MobileManipulatorReferenceManager::resetCmdVelRuckig(double initTime, const vector_t& initState, bool rePlanning)
  {
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::BASE_POS_WORLD_CMD, 
                                                 cmdVel_prevTargetPose_, cmdVel_prevTargetVel_, cmdVel_prevTargetAcc_);

    if(rePlanning)
    {
      cmdVel_plannerInitialTime_ = initTime;
      cmdVelPlannerRuckigPtr_->setCurrentPose(cmdVel_prevTargetPose_);
      cmdVelPlannerRuckigPtr_->setCurrentVelocity(cmdVel_prevTargetVel_);
      cmdVelPlannerRuckigPtr_->setCurrentAcceleration(cmdVel_prevTargetAcc_);
      cmdVelPlannerRuckigPtr_->setTargetVelocity(cmdVel_prevTargetVel_);

      cmdVelPlannerRuckigPtr_->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithLegJoint(double initTime, const vector_t &targetLegJoint, double desiredTime)
  {
    assert(targetLegJoint.size() == 4 && "armJoint dimension must be 4!");

    legJoint_plannerInitialTime_ = initTime;
    legJointPlannerRuckigPtr_->setCurrentPose(legJoint_prevTargetPose_);
    legJointPlannerRuckigPtr_->setCurrentVelocity(legJoint_prevTargetVel_);
    legJointPlannerRuckigPtr_->setCurrentAcceleration(legJoint_prevTargetAcc_);

    legJointPlannerRuckigPtr_->setTargetPose(targetLegJoint);
    double durationTime = legJointPlannerRuckigPtr_->calcTrajectory(desiredTime);

    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    targetLegJointReachTimePub_.publish(time_msg); // 发布到达时间
  }

  void MobileManipulatorReferenceManager::resetLegJointRuckig(double initTime, const vector_t& initState, bool rePlanning)
  {
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::LEG_JOINT_CMD, 
                                                 legJoint_prevTargetPose_, 
                                                 legJoint_prevTargetVel_, 
                                                 legJoint_prevTargetAcc_);

    if(rePlanning)
    {
      legJoint_plannerInitialTime_ = initTime;
      legJointPlannerRuckigPtr_->setCurrentPose(legJoint_prevTargetPose_);
      legJointPlannerRuckigPtr_->setCurrentVelocity(legJoint_prevTargetVel_);
      legJointPlannerRuckigPtr_->setCurrentAcceleration(legJoint_prevTargetAcc_);

      legJointPlannerRuckigPtr_->setTargetPose(legJoint_prevTargetPose_);
      double durationTime = legJointPlannerRuckigPtr_->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithArmJoint(double initTime, const vector_t &targetArmJoint, double desiredTime)
  {
    assert(targetArmJoint.size() == 4 && "armJoint dimension must be 4!");

    armJoint_plannerInitialTime_ = initTime;
    armJointPlannerRuckigPtr_->setCurrentPose(armJoint_prevTargetPose_);
    armJointPlannerRuckigPtr_->setCurrentVelocity(armJoint_prevTargetVel_);
    armJointPlannerRuckigPtr_->setCurrentAcceleration(armJoint_prevTargetAcc_);

    armJointPlannerRuckigPtr_->setTargetPose(targetArmJoint);
    double durationTime = armJointPlannerRuckigPtr_->calcTrajectory(desiredTime);

    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    targetArmJointReachTimePub_.publish(time_msg); // 发布到达时间
  }

  void MobileManipulatorReferenceManager::resetArmJointRuckig(double initTime, const vector_t& initState, bool rePlanning)
  {
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::ARM_JOINT_CMD, 
                                                 armJoint_prevTargetPose_, 
                                                 armJoint_prevTargetVel_, 
                                                 armJoint_prevTargetAcc_);

    if(rePlanning)
    {
      armJoint_plannerInitialTime_ = initTime;
      armJointPlannerRuckigPtr_->setCurrentPose(armJoint_prevTargetPose_);
      armJointPlannerRuckigPtr_->setCurrentVelocity(armJoint_prevTargetVel_);
      armJointPlannerRuckigPtr_->setCurrentAcceleration(armJoint_prevTargetAcc_);

      armJointPlannerRuckigPtr_->setTargetPose(armJoint_prevTargetPose_);
      double durationTime = armJointPlannerRuckigPtr_->calcTrajectory();
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

  LbArmControlMode MobileManipulatorReferenceManager::handPoseCmdFrameToLbArmMode(int frame)
  {
    static LbArmControlMode lastMode = LbArmControlMode::JointSpace;
    LbArmControlMode mode = lastMode;
    switch(frame)
    {
      case 0:   break;                                // keep current frame            
      case 1: 
        mode = LbArmControlMode::WorldFrame; break;   // world frame (based on odom)
      case 2: 
        mode = LbArmControlMode::LocalFrame; break;   // local frame
      case 5: 
        mode = LbArmControlMode::JointSpace; break;   // joint space
      default:
        mode = LbArmControlMode::FalseMode; 
        // std::cerr << "[MobileManipulatorReferenceManager] Unsupported frame type '" 
        // << frame << "' in handPoseCmdFrameToLbArmMode, change To JointSpace." << std::endl;
    }
    return mode;
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

  bool MobileManipulatorReferenceManager::armControlModeSrvCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {

    res.result = true;
    switch (req.control_mode)
    {
    case 0:
      res.message = "Arm control mode 0: keep current control position";
      break;
    case 1:
      res.message = "Arm control mode 1: reset arm to initial Target";
      break;
    case 2:
      res.message = "Arm control mode 2: using external controller";
      break;
    default:
      res.result = false;
      res.message = "Invalid control mode :" + std::to_string(req.control_mode);
      break;
    }
    if (res.result)
    {
      currentArmControlMode_ = static_cast<LbArmControlServiceMode>(req.control_mode);

      std::cout << "currentArmControlMode_:"<< currentArmControlMode_ << std::endl;
      res.mode = currentArmControlMode_;
      ROS_INFO_STREAM(res.message);
      vector_t arm_control_mode_vec(1);
      arm_control_mode_vec << currentArmControlMode_;
      ros_logger_->publishVector("/humanoid/mpc/arm_control_mode", arm_control_mode_vec);

      isCmdArmJointUpdated_ = true; // 触发关节指令规划
      desireMode_ = LbArmControlMode::JointSpace;
    }
    else
    {
      ROS_ERROR_STREAM(res.message);
    }

    return true;
  }

  bool MobileManipulatorReferenceManager::getArmControlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    res.result = true;
    res.mode = currentArmControlMode_;
    return true;
  };

  bool MobileManipulatorReferenceManager::setRuckigPlannerParamsService(kuavo_msgs::setRuckigPlannerParams::Request &req, 
                                                                        kuavo_msgs::setRuckigPlannerParams::Response &res)
  {
    res.result = true;
    res.message = "Set Ruckig Planner Limits Success";

    // 确认规划器类型
    std::shared_ptr<cmdPosePlannerWithRuckig> plannerPosePtr;
    std::shared_ptr<cmdVelPlannerWithRuckig> plannerVelPtr;
    bool is_pos_planner = true; // true: pos planner, false: vel planner
    switch(req.planner_index)
    {
      case 0: // Base Pose Planner
        plannerPosePtr = cmdPosePlannerRuckigPtr_;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Base Pose Planner Params.");
        break;
      case 1: // Base Velocity Planner
        plannerVelPtr = cmdVelPlannerRuckigPtr_;
        is_pos_planner = false;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Base Velocity Planner Params.");
        break;
      case 2: // Torso Pose Planner
        plannerPosePtr = torsoPosePlannerRuckigPtr_;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Torso Pose Planner Params.");
        break;
      case 3: // Leg Joint Planner
        plannerPosePtr = legJointPlannerRuckigPtr_;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Leg Joint Planner Params.");
        break;
      case 4: // Dual Arm EE Pose Planner
        plannerPosePtr = cmdDualArmEePlannerRuckigPtr_;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Dual Arm EE Pose Planner Params.");
        break;
      case 5: // Arm Joint Planner
        plannerPosePtr = armJointPlannerRuckigPtr_;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Arm Joint Pose Planner Params.");
        break;
      default:
        res.result = false;
        res.message = "Invalid planner index: " + std::to_string(req.planner_index);
        ROS_ERROR_STREAM(res.message);
        break;
    }

    // 设置同步模式
    if (is_pos_planner)
    {
      plannerPosePtr->setPlannerSyncMode(req.is_sync);

      int cmdType = SrvRequestIndexToCmdType(req.planner_index);
      if(cmdType != -1)
      {
        timedPlannerScheduler_.setTimedPlannerSyncMode(cmdType, req.is_sync);
      }
      if(cmdType == static_cast<int>(LbTimedPosCmdType::DUAL_ARM_WORLD_CMD) || 
         cmdType == static_cast<int>(LbTimedPosCmdType::BASE_POS_WORLD_CMD))  // 世界系时需要同步更新局部系设置
      {
        timedPlannerScheduler_.setTimedPlannerSyncMode(cmdType+1, req.is_sync);
      }

      ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Planner to " 
                   << (req.is_sync ? "Sync" : "Async") << " Mode.");
    }
    
    // 转换数据到Eigen::VectorXd并处理可选的负向限制
    Eigen::VectorXd vel_max, vel_min, acc_max, acc_min, jerk_max;
    bool has_velocity_limits = false;
    bool has_acceleration_limits = false;
    bool has_jerk_limits = false;
    
    if (!req.velocity_max.empty())
    {
      vel_max = Eigen::Map<Eigen::VectorXd>(req.velocity_max.data(), req.velocity_max.size());
      if (!req.velocity_min.empty() && 
          req.velocity_min.size() == req.velocity_max.size())
      {
        vel_min = Eigen::Map<Eigen::VectorXd>(req.velocity_min.data(), req.velocity_min.size());
      } 
      else 
      {
        ROS_WARN_STREAM("[setRuckigPlannerParamsService] velocity_min size mismatch, using -velocity_max");
        vel_min = -vel_max;
      }
      has_velocity_limits = true;
    }

    if (!req.acceleration_max.empty())
    {
      acc_max = Eigen::Map<Eigen::VectorXd>(req.acceleration_max.data(), req.acceleration_max.size());
      if (!req.acceleration_min.empty() && 
          req.acceleration_min.size() == req.acceleration_max.size())
      {
        acc_min = Eigen::Map<Eigen::VectorXd>(req.acceleration_min.data(), req.acceleration_min.size());
      } 
      else 
      {
        ROS_WARN_STREAM("[setRuckigPlannerParamsService] acceleration_min size mismatch, using -acceleration_max");
        acc_min = -acc_max;
      }
      has_acceleration_limits = true;
    }

    if (!req.jerk_max.empty())
    {
      jerk_max = Eigen::Map<Eigen::VectorXd>(req.jerk_max.data(), req.jerk_max.size());
      has_jerk_limits = true;
    }
    
    if (is_pos_planner)
    {
      if (has_velocity_limits && vel_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Pose Planner's velocity DOF size mismatch! input size: " + std::to_string(vel_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }
      if (has_acceleration_limits && acc_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Pose Planner's acceleration DOF size mismatch! input size: " + std::to_string(acc_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }
      if (has_jerk_limits && jerk_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Pose Planner's jerk DOF size mismatch! input size: " + std::to_string(jerk_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }
      if (has_velocity_limits)
      {
        plannerPosePtr->setVelocityLimits(vel_max, vel_min);
        int cmdType = SrvRequestIndexToCmdType(req.planner_index);
        if(cmdType != -1)
        {
          timedPlannerScheduler_.updateTimedPlannerVelocityLimits(cmdType, vel_max, vel_min);
        }
        if(cmdType == static_cast<int>(LbTimedPosCmdType::DUAL_ARM_WORLD_CMD) || 
           cmdType == static_cast<int>(LbTimedPosCmdType::BASE_POS_WORLD_CMD))  // 世界系时需要同步更新局部系设置
        {
          timedPlannerScheduler_.updateTimedPlannerVelocityLimits(cmdType+1, vel_max, vel_min);
        }
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Pose Planner Velocity Limits.");
        ROS_INFO_STREAM("  vel_max: [" << vel_max.transpose() << "]");
        ROS_INFO_STREAM("  vel_min: [" << vel_min.transpose() << "]");
      }
      if (has_acceleration_limits)
      {
        plannerPosePtr->setAccelerationLimits(acc_max, acc_min);
        int cmdType = SrvRequestIndexToCmdType(req.planner_index);
        if(cmdType != -1)
        {
          timedPlannerScheduler_.updateTimedPlannerAccelerationLimits(cmdType, acc_max, acc_min);
        }
        if(cmdType == static_cast<int>(LbTimedPosCmdType::DUAL_ARM_WORLD_CMD) || 
           cmdType == static_cast<int>(LbTimedPosCmdType::BASE_POS_WORLD_CMD))  // 世界系时需要同步更新局部系设置
        {
          timedPlannerScheduler_.updateTimedPlannerAccelerationLimits(cmdType+1, acc_max, acc_min);
        }
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Pose Planner Acceleration Limits.");
        ROS_INFO_STREAM("  acc_max: [" << acc_max.transpose() << "]");
        ROS_INFO_STREAM("  acc_min: [" << acc_min.transpose() << "]");
      }
      if (has_jerk_limits)
      {
        plannerPosePtr->setJerkLimits(jerk_max);
        int cmdType = SrvRequestIndexToCmdType(req.planner_index);
        if(cmdType != -1)
        {
          timedPlannerScheduler_.updateTimedPlannerJerkLimits(cmdType, jerk_max);
        }
        if(cmdType == static_cast<int>(LbTimedPosCmdType::DUAL_ARM_WORLD_CMD)|| 
           cmdType == static_cast<int>(LbTimedPosCmdType::BASE_POS_WORLD_CMD))  // 世界系时需要同步更新局部系设置
        {
          timedPlannerScheduler_.updateTimedPlannerJerkLimits(cmdType+1, jerk_max);
        }
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Pose Planner Jerk Limits.");
        ROS_INFO_STREAM("  jerk_max: [" << jerk_max.transpose() << "]");
      }
    }
    else
    {
      if (has_acceleration_limits && acc_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Vel Planner's acceleration DOF size mismatch! input size: " + std::to_string(acc_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }
      if (has_jerk_limits && jerk_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Vel Planner's jerk DOF size mismatch! input size: " + std::to_string(jerk_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }

      if (has_acceleration_limits)
      {
        plannerVelPtr->setAccelerationLimits(acc_max, acc_min);
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Vel Planner Acceleration Limits.");
        ROS_INFO_STREAM("  acc_max: [" << acc_max.transpose() << "]");
        ROS_INFO_STREAM("  acc_min: [" << acc_min.transpose() << "]");
      }
      if (has_jerk_limits)
      {
        plannerVelPtr->setJerkLimits(jerk_max);
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Vel Planner Jerk Limits.");
        ROS_INFO_STREAM("  jerk_max: [" << jerk_max.transpose() << "]");
      }
    }

    return true;
  }

  bool MobileManipulatorReferenceManager::getLbTorsoInitialPoseService(kuavo_msgs::getLbTorsoInitialPose::Request &req, 
                                                                       kuavo_msgs::getLbTorsoInitialPose::Response &res)
  {
    if (req.isNeed)
    {
      Eigen::Vector3d zyx = quatToZyx(Eigen::Quaterniond(initialTorsoQuat_.head<4>()));

      res.linear.x = initialTorsoPos_(0);
      res.linear.y = initialTorsoPos_(1);
      res.linear.z = initialTorsoPos_(2);
      res.angular.z = zyx(0);
      res.angular.y = zyx(1);
      res.angular.x = zyx(2);
      
      res.result = true;
      res.message = "Get torso initial pose success.";
      ROS_INFO_STREAM("[MobileManipulatorReferenceManager] Get torso initial pose: "
                      << "Position [" << initialTorsoPos_.transpose() << "], "
                      << "Orientation (ZYX) [" << zyx.transpose() << "]");
    }
    else
    {
      res.result = false;
      res.message = "No need to get torso initial pose.";
    }

    return true;
  }

  bool MobileManipulatorReferenceManager::setLbTimedPosCmdService(kuavo_msgs::lbTimedPosCmd::Request &req, 
                                                                  kuavo_msgs::lbTimedPosCmd::Response &res)
  {
    res.isSuccess = true;
    res.message = "Set Lb Timed Position Command Success.";

    Eigen::VectorXd eigenCmdVec = Eigen::Map<Eigen::VectorXd>(req.cmdVec.data(), req.cmdVec.size());
    // 1. 处理不同的 planner_index
    switch (static_cast<LbTimedPosCmdType>(req.planner_index)) 
    {
      case LbTimedPosCmdType::BASE_POS_WORLD_CMD:
        eigenCmdVec[2] = targetYawPreProcess(initState_[2], eigenCmdVec[2]);
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Base World Pose Command.");
        break;
      case LbTimedPosCmdType::BASE_POS_LOCAL_CMD:
      {
        // 1. 获取当前的位姿
        Eigen::Vector2d currentPos = initState_.head(2);  // 当前世界系位置 [x, y]
        scalar_t currentYaw = initState_[2];              // 当前世界系偏航角
        // 2. 构建世界系的位姿增量
        Eigen::Matrix2d currentRot = Eigen::Rotation2D<scalar_t>(initState_[2]).toRotationMatrix();
        Eigen::Vector2d displacementWorld = currentRot * eigenCmdVec.head(2);
        // 3. 进行增量处理
        eigenCmdVec[0] = currentPos[0] + displacementWorld[0];
        eigenCmdVec[1] = currentPos[1] + displacementWorld[1];
        eigenCmdVec[2] = currentYaw + eigenCmdVec[2];
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Base Local Pose Command.");
        break;
      }
      case LbTimedPosCmdType::TORSO_POSE_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Torso Pose Command.");
        break;
      case LbTimedPosCmdType::LEG_JOINT_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Leg Joint Command.");
        break;
      case LbTimedPosCmdType::DUAL_ARM_WORLD_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Dual Arm World Frame EE Command.");
        break;
      case LbTimedPosCmdType::DUAL_ARM_LOCAL_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Dual Arm Local Frame EE Command.");
        break;
      case LbTimedPosCmdType::ARM_JOINT_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Arm Joint Command.");
        break;
      default:
        res.isSuccess = false;
        res.message = "Invalid planner index.";
        ROS_ERROR_STREAM("[setLbTimedPosCmdService] " + res.message);
        return true;
    }
    
    // 调用对应的轨迹规划器计算所需时间
    double desiredTime = timedPlannerScheduler_.calcTimedTrajectory(req.planner_index, eigenCmdVec, req.desireTime);

    if(desiredTime == -1)
    {
      int desiredSize = timedPlannerScheduler_.getTimedPlannerDofNum(req.planner_index);
      res.isSuccess = false;
      std::string strReturn = "Command vector size mismatch. Input size: " 
                              + std::to_string(eigenCmdVec.size()) + 
                              ", required size: " + std::to_string(desiredSize) + ".";
      res.message = strReturn;
      ROS_ERROR_STREAM("[setLbTimedPosCmdService] " + res.message);
      return true;
    }
  
    // 保存对应轨迹和时间
    res.actualTime = desiredTime;
    desireTime_[req.planner_index] = desiredTime;

    timedCmdVecMtx_[req.planner_index]->lock();
    timedCmdVec_[req.planner_index] = Eigen::Map<Eigen::VectorXd>(req.cmdVec.data(), req.cmdVec.size());
    timedCmdVecMtx_[req.planner_index]->unlock();

    isTimedPlannerUpdated_[req.planner_index] = true;

    return true;
  }

  bool MobileManipulatorReferenceManager::setLbResetTorsoService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    if(req.data)
    {
      /***********************根据期望速度, 设置切换时间************************/
      vector_t torsoPose = vector_t::Zero(6);
      getCurrentTorsoPoseInBaseContinuous(torsoPose, initState_);
      Eigen::VectorXd err = Eigen::VectorXd::Zero(6);
      err[0] = std::fabs(initialTorsoPos_[0] - torsoPose[0]);
      err[2] = std::fabs(initialTorsoPos_[2] - torsoPose[2]);
      err[3] = std::fabs(torsoPose[3] - 0.0);
      err[4] = std::fabs(torsoPose[4] - 0.0);
      Eigen::ArrayXd validTimes = err.array() / torsoResetMaxVel_.array();
      double actualTime = validTimes.maxCoeff();
      /*********************************************************************/
      res.success = true;
      res.message = std::to_string(actualTime);  // 直接转换
      isResetTorso_ = true;
      return true;
    }

    res.success = false;
    res.message = "0.0";
    return true;
  }

  void MobileManipulatorReferenceManager::getCurrentTorsoPoseInBase(vector_t& torsoPose, const vector_t& initState)
  {
    assert(torsoPose.size() == 7 && "torsoPose dimension must be 7!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取躯干坐标系帧ID
    pinocchio::FrameIndex torsoFrameId = model.getFrameId(info_.torsoFrame);
    // 获取躯干在世界坐标系中的位姿
    const pinocchio::SE3& worldToTorso = data.oMf[torsoFrameId];

    // 获取基坐标系帧ID
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    // 获取基座在世界坐标系中的位姿
    const pinocchio::SE3& worldToBase = data.oMf[baseFrameId];

    // 计算躯干在基坐标系中的位姿: baseToTorso = worldToBase.inverse() * worldToTorso
    pinocchio::SE3 baseToTorso = worldToBase.actInv(worldToTorso);

    torsoPose.segment<3>(0) = baseToTorso.translation();
    torsoPose.segment<4>(3) = Eigen::Quaterniond(baseToTorso.rotation()).coeffs();
  }

  void MobileManipulatorReferenceManager::getCurrentEeWorldPoseContinuous(vector_t& EeState, const vector_t& initState)
  {
    // 输出6D位姿: [x, y, z, yaw, pitch, roll] (ZYX顺序)
    assert(EeState.size() == info_.eeFrames.size() * 6 && "EeState dimension must be info_.eeFrames.size()*6!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 初始化连续欧拉角跟踪器
    static std::vector<ContinuousEulerAnglesFromMatrix> eeUnwrappers(info_.eeFrames.size());

    // 遍历每个末端执行器
    for (size_t ee_idx = 0; ee_idx < info_.eeFrames.size(); ++ee_idx) 
    {
      // 获取末端执行器帧ID（这里需要您根据实际情况获取）
      pinocchio::FrameIndex frameId = model.getFrameId(info_.eeFrames[ee_idx]);
      // 获取末端在世界坐标系中的位姿
      const pinocchio::SE3& ee_pose = data.oMf[frameId];

      EeState.segment<3>(ee_idx*6) = ee_pose.translation();

      // 使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
      Eigen::Vector3d eulerZyx = eeUnwrappers[ee_idx].update(ee_pose.rotation());
      EeState.segment<3>(ee_idx*6+3) = eulerZyx;
    }
  }

  void MobileManipulatorReferenceManager::getCurrentEeBasePoseContinuous(vector_t& EeState, const vector_t& initState)
  {
    // 输出6D位姿: [x, y, z, yaw, pitch, roll] (ZYX顺序，基座坐标系下)
    assert(EeState.size() == info_.eeFrames.size() * 6 && "EeState dimension must be info_.eeFrames.size()*6!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取基座的世界系位姿
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    const pinocchio::SE3& base_pose_world = data.oMf[baseFrameId];

    // 计算世界到基座的变换矩阵
    pinocchio::SE3 world_to_base = base_pose_world.inverse();

    // 初始化连续欧拉角跟踪器
    static std::vector<ContinuousEulerAnglesFromMatrix> eeBaseUnwrappers(info_.eeFrames.size());

    // 遍历每个末端执行器
    for (size_t ee_idx = 0; ee_idx < info_.eeFrames.size(); ++ee_idx) 
    {
      // 获取末端执行器帧ID（这里需要您根据实际情况获取）
      pinocchio::FrameIndex frameId = model.getFrameId(info_.eeFrames[ee_idx]);
      // 获取末端在世界坐标系中的位姿
      const pinocchio::SE3& ee_pose_world = data.oMf[frameId];

      // 将末端位姿从世界坐标系转换到基座坐标系
      pinocchio::SE3 ee_pose_base = world_to_base * ee_pose_world;

      EeState.segment<3>(ee_idx*6) = ee_pose_base.translation();

      // 使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
      Eigen::Vector3d eulerZyx = eeBaseUnwrappers[ee_idx].update(ee_pose_base.rotation());
      EeState.segment<3>(ee_idx*6+3) = eulerZyx;
    }
  }

  void MobileManipulatorReferenceManager::getCurrentTorsoPoseInBaseContinuous(vector_t& torsoPose, const vector_t& initState)
  {
    assert(torsoPose.size() == 6 && "torsoPose dimension must be 6!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取躯干坐标系帧ID
    pinocchio::FrameIndex torsoFrameId = model.getFrameId(info_.torsoFrame);
    // 获取躯干在世界坐标系中的位姿
    const pinocchio::SE3& worldToTorso = data.oMf[torsoFrameId];

    // 获取基坐标系帧ID
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    // 获取基座在世界坐标系中的位姿
    const pinocchio::SE3& worldToBase = data.oMf[baseFrameId];

    // 计算躯干在基坐标系中的位姿: baseToTorso = worldToBase.inverse() * worldToTorso
    pinocchio::SE3 baseToTorso = worldToBase.actInv(worldToTorso);

    torsoPose.segment<3>(0) = baseToTorso.translation();
    
    // 姿态：使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
    static ContinuousEulerAnglesFromMatrix torsoUnwrapper;
    Eigen::Vector3d eulerZyx = torsoUnwrapper.update(baseToTorso.rotation());
    torsoPose.segment<3>(3) = eulerZyx;  // yaw, pitch, roll
  }

  void MobileManipulatorReferenceManager::publishMultiPointPose_World(const vector_t& initState)
  {
    Eigen::VectorXd eeState_world, d_eeState_world, dd_eeState_world;
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::DUAL_ARM_WORLD_CMD, 
                                                 eeState_world, d_eeState_world, dd_eeState_world);

    ros_logger_->publishVector("/mobile_manipulator/eeStateWorld_initState", eeState_world);
    ros_logger_->publishVector("/mobile_manipulator/d_eeStateWorld_initState", d_eeState_world);
    ros_logger_->publishVector("/mobile_manipulator/dd_eeStateWorld_initState", dd_eeState_world);
  }

  void MobileManipulatorReferenceManager::publishMultiPointPose_Local(const vector_t& initState)
  {
    vector_t eeState_local, d_eeState_local, dd_eeState_local;
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::DUAL_ARM_LOCAL_CMD, 
                                                 eeState_local, d_eeState_local, dd_eeState_local);

    ros_logger_->publishVector("/mobile_manipulator/eeStateLocal_initState", eeState_local);
    ros_logger_->publishVector("/mobile_manipulator/d_eeStateLocal_initState", d_eeState_local);
    ros_logger_->publishVector("/mobile_manipulator/dd_eeStateLocal_initState", dd_eeState_local);
    /********************************************************************************************************************/
    
    vector_t torsoState_local, d_torsoState_local, dd_torsoState_local;
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::TORSO_POSE_CMD, 
                              torsoState_local, d_torsoState_local, dd_torsoState_local);
    
    ros_logger_->publishVector("/mobile_manipulator/torsoStateLocal_initState", torsoState_local);
    ros_logger_->publishVector("/mobile_manipulator/d_torsoStateLocal_initState", d_torsoState_local);
    ros_logger_->publishVector("/mobile_manipulator/dd_torsoStateLocal_initState", dd_torsoState_local);
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

  void MobileManipulatorReferenceManager::getCurrentEeBasePose(vector_t& EeState, const vector_t& initState)
  {
    assert(EeState.size() == info_.eeFrames.size() * 7 && "EeState dimension must be info_.eeFrames.size()*7!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取基座的世界系位姿
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    const pinocchio::SE3& base_pose_world = data.oMf[baseFrameId];

    // 计算世界到基座的变换矩阵
    pinocchio::SE3 world_to_base = base_pose_world.inverse();

    // 遍历每个末端执行器
    for (size_t ee_idx = 0; ee_idx < info_.eeFrames.size(); ++ee_idx) 
    {
      // 获取末端执行器帧ID（这里需要您根据实际情况获取）
      pinocchio::FrameIndex frameId = model.getFrameId(info_.eeFrames[ee_idx]);
      // 获取末端在世界坐标系中的位姿
      const pinocchio::SE3& ee_pose_world = data.oMf[frameId];

      // 将末端位姿从世界坐标系转换到基座坐标系
      pinocchio::SE3 ee_pose_base = world_to_base * ee_pose_world;

      EeState.segment<3>(ee_idx*7) = ee_pose_base.translation();
      EeState.segment<4>(ee_idx*7+3) = Eigen::Quaterniond(ee_pose_base.rotation()).coeffs();
    }
  }

  void MobileManipulatorReferenceManager::updateNoControl(double initTime, const TargetTrajectories& targetTrajectories, bool isChange)
  {
    static bool firstRun{true};
    if(isChange || firstRun)
    {
      setEnableEeTargetTrajectories(true); // 开启末端笛卡尔跟踪
      setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
      setEnableArmJointTrack(false); // 关闭手臂跟踪
      setEnableBaseTrack(false);   // 关闭底盘跟踪

      firstRun = false;
      // return;
    }

    getAllTargetTrajectories(targetTrajectories);
  }

  void MobileManipulatorReferenceManager::updateArmOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  {
    if(isChange)
    {
      vector_t eeState = vector_t::Zero(info_.eeFrames.size() * 7);
      getCurrentEeWorldPose(eeState, initState);
      left_arm_traj_pose_ = eeState.head(7);
      right_arm_traj_pose_ = eeState.tail(7);

      lb_leg_traj_ = initState.segment(baseDim_, 4);
      arm_joint_traj_ = initState.tail(info_.armDim - 4);
      left_arm_joint_traj_ = initState.tail(info_.armDim - 4).head((info_.armDim - 4)/2);
      right_arm_joint_traj_ = initState.tail(info_.armDim - 4).tail((info_.armDim - 4)/2);

      resetAllMpcTraj(initTime_, initState_);

      // return;
    }

    if(initTime >= resetTorsoTime_ + resetTorsoInitTime_)
    {
      setArmControl(initTime, finalTime, initState);
      setTorsoControl(initTime, finalTime, initState);
    }

    generatePoseTargetWithRuckig(initTime, finalTime, ruckigDt_);
  }

  void MobileManipulatorReferenceManager::updateBaseOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  {
    if(isChange)
    {
      resetAllMpcTraj(initTime_, initState_);
      // return;
    }

    setChassisControl(initTime, finalTime, initState);
  }

  void MobileManipulatorReferenceManager::updateBaseArmControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  {
    static bool isFirstRun = true;
    if(isFirstRun)
    {
      resetAllMpcTrajAndTarget(initTime_, initState_);

      isFirstRun = false;
      // return;
    }

    if(initTime >= resetTorsoTime_ + resetTorsoInitTime_)
    {
      setArmControl(initTime, finalTime, initState);
      setTorsoControl(initTime, finalTime, initState);
    }
    setChassisControl(initTime, finalTime, initState);
  }

  void MobileManipulatorReferenceManager::updateArmEeOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  {
    if(isChange)
    {
      vector_t eeState = vector_t::Zero(info_.eeFrames.size() * 7);
      getCurrentEeWorldPose(eeState, initState);
      left_arm_traj_pose_ = eeState.head(7);
      right_arm_traj_pose_ = eeState.tail(7);

      resetAllMpcTraj(initTime, initState);

      // return;
    }

    static vector_t armEeTarget = vector_t::Zero(info_.eeFrames.size() * 7); // 双臂末端轨迹
    

    if(isCmdDualArmPoseUpdated_)
    {
      bool isChange = getLbArmControlModeIsChange(desireMode_);
      // TODO: 如果 isChange 为 true, 则根据当前模式重置一次初值
      if(isChange)  resetDualArmRuckig(initTime, initState, false, desireMode_);
      if(desireMode_ == LbArmControlMode::WorldFrame)
      {
        armPose_mtx_.lock();
        armEeTarget << left_arm_traj_pose_, right_arm_traj_pose_;
        calcRuckigTrajWithEePose(initTime, armEeTarget);
        armPose_mtx_.unlock();
      }
      isCmdDualArmPoseUpdated_ = false;
    }
    else
    {
      resetDualArmRuckig(initTime, initState, false, desireMode_);
    }

    if(desireMode_ == LbArmControlMode::WorldFrame) // 世界系的笛卡尔末端控制
    {
      setEnableArmJointTrack(false); // 关闭手臂跟踪
      setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
      setEnableBaseTrack(false);   // 关闭底盘跟踪
      setEnableEeTargetTrajectories(true); // 开启末端笛卡尔
      setEnableTorsoPoseTargetTrajectories(true); // 开启躯干笛卡尔

      generateDualArmEeTargetWithRuckig(initTime, finalTime, ruckigDt_);
    }
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

  void MobileManipulatorReferenceManager::setChassisControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState)
  {
    // 生成底盘指令轨迹（包含速度指令和位置指令处理）
    if(isCmdPoseUpdated_)
    {
      // 计算期望位置
      cmdPose_mtx_.lock();
      Eigen::Vector3d cmdPoseBase = cmdPose_;
      cmdPose_mtx_.unlock();

      // 将本体系的位置期望，转换成世界系的
      // 1. 获取当前的位姿
      Eigen::Vector2d currentPos = initState.head(2);  // 当前世界系位置 [x, y]
      scalar_t currentYaw = initState[2];              // 当前世界系偏航角
      // 2. 构建世界系的位姿增量
      Eigen::Matrix2d currentRot = Eigen::Rotation2D<scalar_t>(initState[2]).toRotationMatrix();
      Eigen::Vector2d displacementWorld = currentRot * cmdPoseBase.head(2);
      // 3. 进行增量处理
      currentCmdPose_[0] = currentPos[0] + displacementWorld[0];
      currentCmdPose_[1] = currentPos[1] + displacementWorld[1];
      currentCmdPose_[2] = currentYaw + cmdPoseBase[2];
    }

    if(isCmdPoseWorldUpdated_)
    {
      // 计算期望位置
      cmdPoseWorld_mtx_.lock();
      currentCmdPose_ = cmdPoseWorld_;
      currentCmdPose_[2] = targetYawPreProcess(initState[2], cmdPoseWorld_[2]);
      cmdPoseWorld_mtx_.unlock();
    }

    if(isCmdPoseUpdated_ || isCmdPoseWorldUpdated_)
    {
      /*****************************更新ruckig规划器所需实时数据************************************/
      calcRuckigTrajWithCmdPose(initTime, currentCmdPose_, cmdPoseDesiredTime_);
      /*****************************************************************************************/

      // 清空位置指令更新标志
      isCmdPoseUpdated_ = false;
      isCmdPoseWorldUpdated_ = false;
    }
    else  // 未收到位置指令
    {
      resetCmdPoseRuckig(initTime, initState, false);
    }

    // 更新速度指令时间，超时0.3s后未更新指令则清0
    if(isCmdVelTimeUpdate_)
    {
      lastCmdVelTime_ = initTime;
      isCmdVelTimeUpdate_ = false;
    }
    if((initTime - lastCmdVelTime_) > 0.3)
    {
      cmdVel_.setZero();
      cmdVelWorld_.setZero();
    }

    if(!isCmdVelUpdated_ && !isCmdVelWorldUpdated_)
    {
      resetCmdVelRuckig(initTime, initState, false);
    }
  
    // 更新速度指令，通过互斥锁维护数据一致性
    if(cmdVel_.isZero(1e-6) && cmdVel_prevTargetVel_.isZero(1e-6))
    {
      isCmdVelUpdated_ = false;
    }
    else
    {
      cmdvel_mtx_.lock();
      currentCmdVel_ = cmdVel_;
      cmdvel_mtx_.unlock();
    }

    if(cmdVelWorld_.isZero(1e-6) && cmdVel_prevTargetVel_.isZero(1e-6))
    {
      isCmdVelWorldUpdated_ = false;
    }
    else
    {
      cmdvelWorld_mtx_.lock();
      currentCmdVelWorld_ = cmdVelWorld_;
      cmdvelWorld_mtx_.unlock();
    }

    // 跟踪上一次使用的速度控制模式，用于检测模式切换
    static bool lastWasCmdVel = false;
    static bool lastWasCmdVelWorld = false;
    if(isCmdVelUpdated_)
    {
      // 如果之前使用的是 cmd_vel_world，现在切换到 cmd_vel，更新标志位
      lastWasCmdVel = true;
      lastWasCmdVelWorld = false;
      /*****************************更新ruckig规划器所需实时数据************************************/
      calcRuckigTrajWithCmdVel(initTime, currentCmdVel_);

      generateVelTargetBaseWithRuckig(initTime, finalTime, ruckigDt_, initState);
      /*****************************************************************************************/

      // 判断速度为0，跳出速度控制
      static int zero_vel_cnt;
      if(zero_vel_cnt < 5)
      {
        if(cmdVel_prevTargetVel_.isZero(1e-6)) zero_vel_cnt++;
        else zero_vel_cnt = 0;
      }
      else
      {
        zero_vel_cnt = 0;
        isCmdVelUpdated_ = false;
      }

      resetCmdPoseRuckig(initTime, initState, true);
    }
    else if(isCmdVelWorldUpdated_)
    {
      // 如果之前使用的是 cmd_vel，现在切换到 cmd_vel_world，需要重置 prevTargetPose 为当前实际位置
      if(lastWasCmdVel && !lastWasCmdVelWorld)
      {
        cmdVel_prevTargetPose_ = initState.head(3);  // 重置为当前实际位置，避免返回原点
        cmdVel_prevTargetVel_.setZero(3);
        cmdVel_prevTargetAcc_.setZero(3);
      }
      lastWasCmdVel = false;
      lastWasCmdVelWorld = true;
      /*****************************更新ruckig规划器所需实时数据************************************/
      calcRuckigTrajWithCmdVel(initTime, currentCmdVelWorld_);

      generateVelTargetWithRuckig(initTime, finalTime, ruckigDt_);
      /*****************************************************************************************/

      // 判断速度为0，跳出速度控制
      static int zero_vel_cnt;
      if(zero_vel_cnt < 5)
      {
        if(cmdVel_prevTargetVel_.isZero(1e-6)) zero_vel_cnt++;
        else zero_vel_cnt = 0;
      }
      else
      {
        zero_vel_cnt = 0;
        isCmdVelWorldUpdated_ = false;
      }

      resetCmdPoseRuckig(initTime, initState, true);
    }
    else    // 默认跟踪位置
    {
      generatePoseTargetWithRuckig(initTime, finalTime, ruckigDt_);
    }
  }

  void MobileManipulatorReferenceManager::setArmControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState)
  {
    // 手臂控制模式接受三种收发逻辑（desireMode_用于选择）: 
    // 0. 发手臂末端世界系轨迹; //有些危险, 上层需关注清楚
    // 1. 发手臂末端局部系轨迹;
    // 2. 发手臂关节轨迹;
    
    static vector_t armJointTarget = arm_init_joint_traj_; // 双臂关节轨迹
    static vector_t armEeTarget = vector_t::Zero(info_.eeFrames.size() * 7); // 双臂末端轨迹
    
    if(isCmdDualArmPoseUpdated_)
    {
      bool isChange = getLbArmControlModeIsChange(desireMode_);
      // TODO: 如果 isChange 为 true, 则根据当前模式重置一次初值
      if(isChange)  resetDualArmRuckig(initTime, initState, false, desireMode_);
      if(desireMode_ == LbArmControlMode::WorldFrame || desireMode_ == LbArmControlMode::LocalFrame)
      {
        armPose_mtx_.lock();
        armEeTarget << left_arm_traj_pose_, right_arm_traj_pose_;
        calcRuckigTrajWithEePose(initTime, armEeTarget, cmdDualArmPoseDesiredTime_);
        armPose_mtx_.unlock();
      }
      isCmdDualArmPoseUpdated_ = false;
    }

    if(isCmdArmJointUpdated_)
    {
      bool isChange = getLbArmControlModeIsChange(desireMode_);
      if(isChange && desireMode_ == LbArmControlMode::JointSpace)
      {
        resetArmJointRuckig(initTime, initState, false);
      }
    }
    
    if(currentArmControlMode_  == LbArmControlServiceMode::EXTERN_CONTROL)
    {
      if(desireMode_ == LbArmControlMode::WorldFrame) // 世界系的笛卡尔末端控制
      {
        setEnableArmJointTrack(false); // 关闭手臂跟踪
        setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
        setEnableEeTargetTrajectories(true); // 开启末端笛卡尔

        generateDualArmEeTargetWithRuckig(initTime, finalTime, ruckigDt_);

        resetArmJointRuckig(initTime, initState, false);  // 笛卡尔控制影响手臂关节, 重置关节轨迹初值
      }
      else if(desireMode_ == LbArmControlMode::LocalFrame)  // 局部系的笛卡尔末端控制
      {
        setEnableArmJointTrack(false); // 关闭手臂跟踪
        setEnableEeTargetTrajectories(false); // 关闭末端笛卡尔跟踪
        setEnableEeTargetLocalTrajectories(true); // 开启末端笛卡尔局部跟踪
      
        generateDualArmEeTargetWithRuckig(initTime, finalTime, ruckigDt_);

        resetArmJointRuckig(initTime, initState, false);  // 笛卡尔控制影响手臂关节, 重置关节轨迹初值
      }
      else if(desireMode_ == LbArmControlMode::JointSpace)  // 关节控制
      {
        setEnableEeTargetTrajectories(false); // 关闭末端笛卡尔跟踪
        setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
        setEnableArmJointTrack(true); // 开启手臂跟踪

        if(enableQuickJointControl_)  // true 为快速模式
        {
          armJoint_mtx_.lock();
          armJointTarget = arm_joint_traj_;
          armJoint_mtx_.unlock();
        }
        else
        {
          armPose_mtx_.lock();
          armJointTarget << left_arm_joint_traj_, right_arm_joint_traj_;
          armPose_mtx_.unlock();
        }
        if(isCmdArmJointUpdated_)
        {
          calcRuckigTrajWithArmJoint(initTime, armJointTarget, cmdArmJointDesiredTime_);
          isCmdArmJointUpdated_ = false;
        }

        // resetLegJointRuckig(initTime, initState, false);  // 笛卡尔控制影响下肢关节, 重置关节轨迹初值
      }
    }
    else if(currentArmControlMode_  == LbArmControlServiceMode::KEEP)
    {
      setEnableEeTargetTrajectories(false); // 关闭末端笛卡尔跟踪
      setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
      setEnableArmJointTrack(true); // 开启手臂跟踪

      if(isCmdArmJointUpdated_)
      {
        calcRuckigTrajWithArmJoint(initTime, armJointTarget);
        isCmdArmJointUpdated_ = false;
      }
    }
    else if(currentArmControlMode_ == LbArmControlServiceMode::AUTO_SWING)
    {
      setEnableEeTargetTrajectories(false); // 关闭末端笛卡尔跟踪
      setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
      setEnableArmJointTrack(true); // 开启手臂跟踪

      if(isCmdArmJointUpdated_)
      {
        calcRuckigTrajWithArmJoint(initTime, arm_init_joint_traj_, 1.0);
        isCmdArmJointUpdated_ = false;
      }

      armJointTarget = arm_init_joint_traj_;
      arm_joint_traj_ = arm_init_joint_traj_;
      left_arm_joint_traj_ = arm_init_joint_traj_.head((info_.armDim - 4)/2);
      right_arm_joint_traj_ = arm_init_joint_traj_.tail((info_.armDim - 4)/2);
    }
    else
    {
      ROS_ERROR_STREAM("[MobileManipulatorReferenceManager] 错误设置 currentArmControlMode_");
    }
  }

  void MobileManipulatorReferenceManager::resetTorsoControlPoseWithRuckig(scalar_t initTime, const vector_t& initState)
  {
    setEnableLegJointTrack(false); // 关闭下肢关节跟踪
    isCmdLegJointUpdated_ = false;  // 关闭关节控制标志位
    setEnableTorsoPoseTargetTrajectories(true); // 开启躯干

    vector_t resetPose = vector_t::Zero(4);
    resetPose[0] = initialTorsoPos_[0];
    resetPose[1] = initialTorsoPos_[2];
    resetTorsoPoseRuckig(initTime, initState, false);

    /*************************根据期望速度, 设置切换时间*******************************/
    vector_t torsoPose = vector_t::Zero(6);
    getCurrentTorsoPoseInBaseContinuous(torsoPose, initState);
    Eigen::VectorXd err = Eigen::VectorXd::Zero(6);
    err[0] = std::fabs(initialTorsoPos_[0] - torsoPose[0]);
    err[2] = std::fabs(initialTorsoPos_[2] - torsoPose[2]);
    err[3] = std::fabs(torsoPose[3] - 0.0);
    err[4] = std::fabs(torsoPose[4] - 0.0);

    Eigen::ArrayXd validTimes = err.array() / torsoResetMaxVel_.array();
    resetTorsoTime_ = validTimes.maxCoeff();
    resetTorsoInitTime_ = initTime;
    ROS_INFO_STREAM("[MobileManipulatorReferenceManager] reset torso require time: " << resetTorsoTime_ << " s, err: " << err.transpose());
    /*****************************************************************************/

    calcRuckigTrajWithTorsoPose(initTime, resetPose, resetTorsoTime_);

    cmdTorsoPose_.setZero();
    cmdTorsoPose_[0] = initialTorsoPos_[0];
    cmdTorsoPose_[1] = initialTorsoPos_[1];
    cmdTorsoPose_[2] = initialTorsoPos_[2];
    cmdTorsoPose_[3] = 0;
    cmdTorsoPose_[4] = 0;

    generateTorsoPoseTargetWithRuckig(initTime, initTime + resetTorsoTime_ + 0.5, ruckigDt_);
  }
  
  void MobileManipulatorReferenceManager::setTorsoControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState)
  {
    if(isCmdTorsoPoseUpdated_)
    {
      resetTorsoPoseRuckig(initTime, initState, false);

      std::cout << "[MobileManipulatorReferenceManager] 进入躯干笛卡尔控制 " << std::endl;

      cmdTorsoPose_mtx_.lock();
      currentTorsoPose_ = cmdTorsoPose_;
      cmdTorsoPose_mtx_.unlock();

      vector_t torsoPose4Dof = vector_t::Zero(4);
      torsoPose4Dof << currentTorsoPose_[0],  // x, z, yaw, pitch
                       currentTorsoPose_[2], 
                       currentTorsoPose_[3], 
                       currentTorsoPose_[4];

      calcRuckigTrajWithTorsoPose(initTime, torsoPose4Dof, cmdTorsoPoseDesiredTime_);

      isCmdTorsoPoseUpdated_ = false;
      torsoModeFlag_ = true;
    }

    if(isCmdLegJointUpdated_)
    {
      setEnableLegJointTrack(true); // 开启下肢关节跟踪
      setEnableTorsoPoseTargetTrajectories(false); // 关闭躯干

      static vector_t legJointTarget = vector_t::Zero(4); // 下肢关节轨迹

      lbLegJoint_mtx_.lock();
      legJointTarget = lb_leg_traj_;
      lbLegJoint_mtx_.unlock();

      calcRuckigTrajWithLegJoint(initTime, legJointTarget, cmdLegJointDesiredTime_);

      isCmdLegJointUpdated_ = false;
      torsoModeFlag_ = false;
    }
    
    if(torsoModeFlag_)
    {
      setEnableLegJointTrack(false); // 关闭下肢关节跟踪
      setEnableTorsoPoseTargetTrajectories(true); // 开启躯干

      generateTorsoPoseTargetWithRuckig(initTime, finalTime, ruckigDt_);

      resetLegJointRuckig(initTime, initState, true); // 躯干笛卡尔控制影响下肢关节, 重置关节轨迹初值
    }
    else
    {
      resetTorsoPoseRuckig(initTime, initState, false);
    }
  }

  void MobileManipulatorReferenceManager::resetAllMpcTraj(scalar_t initTime, const vector_t& initState)
  {
    // 默认以 baseArm 关节控制切换
      setEnableEeTargetTrajectories(false); // 关闭末端笛卡尔跟踪
      setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
      setEnableArmJointTrack(true); // 关闭手臂跟踪
      setEnableBaseTrack(true);   // 关闭底盘跟踪

      resetTorsoControlPoseWithRuckig(initTime, initState); // 重置躯干位置

      resetCmdPoseRuckig(initTime, initState, true);    // 重置底盘轨迹
      resetLegJointRuckig(initTime, initState, true);   // 重置下肢关节轨迹
      resetArmJointRuckig(initTime, initState, true);   // 重置上肢关节轨迹
      resetDualArmRuckig(initTime, initState, true, desireMode_);
      desireMode_ = LbArmControlMode::JointSpace;
  }

  void MobileManipulatorReferenceManager::resetAllMpcTrajAndTarget(scalar_t initTime, const vector_t& initState)
  {
    vector_t eeState = vector_t::Zero(info_.eeFrames.size() * 7);
    getCurrentEeWorldPose(eeState, initState);
    left_arm_traj_pose_ = eeState.head(7);
    right_arm_traj_pose_ = eeState.tail(7);

    lb_leg_traj_ = initState.segment(baseDim_, 4);
    arm_joint_traj_ = initState.tail(info_.armDim - 4);
    left_arm_joint_traj_ = initState.tail(info_.armDim - 4).head((info_.armDim - 4)/2);
    right_arm_joint_traj_ = initState.tail(info_.armDim - 4).tail((info_.armDim - 4)/2);
      
    resetAllMpcTraj(initTime, initState);
  }

  void MobileManipulatorReferenceManager::updateTimedSchedulerCurrentState(scalar_t initTime, const vector_t& initState)
  {
    std::vector<Eigen::VectorXd> currentPos;

    Eigen::VectorXd tmpPos;

    tmpPos.setZero(baseDim_);                                       // 底盘世界系, 自由度3: x, y, yaw
    tmpPos = initState.head(baseDim_);
    currentPos.push_back(tmpPos);

    tmpPos.setZero(baseDim_);                                       // 底盘局部系, 采用世界系做反馈, 自由度3: x, y, yaw
    tmpPos = initState.head(baseDim_);
    currentPos.push_back(tmpPos);

    vector_t torsoPose = vector_t::Zero(6);                         // 躯干, 自由度4: x, z, yaw, pitch
    getCurrentTorsoPoseInBaseContinuous(torsoPose, initState);
    tmpPos.setZero(4);   
    if(torsoModeFlag_ == false)   // 关节控制反馈当前实际位姿, 躯干控制反馈指令位姿
    {
      vector_t torsoState = vector_t::Zero(7);
      getCurrentTorsoPoseInBase(torsoState, initState);
      torsoTargetTrajectories_.stateTrajectory = {torsoState};
      tmpPos << torsoPose[0], torsoPose[2], torsoPose[3], torsoPose[4];
    }
    else
    {
      vector_t torsoTargetPose = targetTorsoTrajToPose6DContinous(torsoTargetTrajectories_, initTime);
      tmpPos << torsoTargetPose[0], torsoTargetPose[2], torsoTargetPose[3], torsoTargetPose[4];
    }
    
    currentPos.push_back(tmpPos);

    tmpPos.setZero(4);                                              // 下肢, 自由度4
    tmpPos = initState.segment(baseDim_, 4);
    currentPos.push_back(tmpPos);
    
    vector_t eePose = vector_t::Zero(info_.eeFrames.size() * 6);    // 双臂世界系, 自由度(末端数*6)
    getCurrentEeWorldPoseContinuous(eePose, initState);
    tmpPos.setZero(info_.eeFrames.size() * 6);
    for(int i=0; i<info_.eeFrames.size(); i++)
    {
      tmpPos.segment(i*6, 3) = eePose.segment(i*6, 3);
      tmpPos.segment(i*6 + 3, 3) = eePose.segment(i*6 + 3, 3);
    }
    currentPos.push_back(tmpPos);

    getCurrentEeBasePoseContinuous(eePose, initState);                        // 双臂局部系, 自由度(末端数*6)
    tmpPos.setZero(info_.eeFrames.size() * 6);
    for(int i=0; i<info_.eeFrames.size(); i++)
    {
      tmpPos.segment(i*6, 3) = eePose.segment(i*6, 3);
      tmpPos.segment(i*6 + 3, 3) = eePose.segment(i*6 + 3, 3);
    }
    currentPos.push_back(tmpPos);

    tmpPos.setZero(info_.armDim - 4);                               // 上肢, 自由度(全身关节数-4)
    tmpPos = initState.tail(info_.armDim - 4);
    currentPos.push_back(tmpPos);

    timedPlannerScheduler_.setTimedPlannerStates(currentPos);
  }

  void MobileManipulatorReferenceManager::updateTimedSchedulerTargetTraj(void)
  {
    for(int i = 0; i < timedPlannerScheduler_.getPlannersNum(); i++)
    {
      if(isTimedPlannerUpdated_[i])
      {
        timedCmdVecMtx_[i]->lock();
        Eigen::VectorXd targetPos = timedCmdVec_[i];
        timedCmdVecMtx_[i]->unlock();

        double duration = timedPlannerScheduler_.calcTimedTrajectory(i, targetPos, desireTime_[i]);
        updateIndexRuckigPlanner(i, desireTime_[i], targetPos);

        std::cout << "planner " << i << " duration: " << duration << std::endl;
        isTimedPlannerUpdated_[i] = false;
      }
    }
  }

  void MobileManipulatorReferenceManager::updateIndexRuckigPlanner(int plannerIndex, double desireTime, 
                                                                   const Eigen::VectorXd& cmd_vec)
  {
    int desiredSize = timedPlannerScheduler_.getTimedPlannerDofNum(plannerIndex);
  
    switch(static_cast<LbTimedPosCmdType>(plannerIndex))
    {
      case LbTimedPosCmdType::BASE_POS_WORLD_CMD:
      {
        isCmdPoseWorldUpdated_ = true;
        cmdPoseDesiredTime_ = desireTime;
        cmdPoseWorld_ = cmd_vec.head(desiredSize);
        std::cout << "Received cmdPoseWorld: " << cmdPoseWorld_.transpose() << std::endl;
        break;
      }
      case LbTimedPosCmdType::BASE_POS_LOCAL_CMD:
      {
        isCmdPoseUpdated_ = true;
        cmdPoseDesiredTime_ = desireTime;
        cmdPose_ = cmd_vec.head(desiredSize);
        std::cout << "Received cmdPose: " << cmdPose_.transpose() << std::endl;
        break;
      }
      case LbTimedPosCmdType::TORSO_POSE_CMD:
      {
        isCmdTorsoPoseUpdated_ = true;
        cmdTorsoPoseDesiredTime_ = desireTime;
        /*******************************赋值躯干指令********************************/
        vector_t tmpCmdTorsoPose = cmd_vec.head(desiredSize);
        cmdTorsoPose_[0] = tmpCmdTorsoPose[0];
        cmdTorsoPose_[2] = tmpCmdTorsoPose[1];
        cmdTorsoPose_[3] = tmpCmdTorsoPose[2];
        cmdTorsoPose_[4] = tmpCmdTorsoPose[3];
        /*************************************************************************/
        std::cout << "desiredSize: "<< desiredSize << std::endl;
        std::cout << "Received cmdTorsoPose: "<< cmdTorsoPose_.transpose() << std::endl;
        break;
      }
      case LbTimedPosCmdType::LEG_JOINT_CMD:
      {
        isCmdLegJointUpdated_ = true;
        cmdLegJointDesiredTime_ = desireTime;
        lb_leg_traj_ = cmd_vec.head(desiredSize);
        break;
      }
      case LbTimedPosCmdType::DUAL_ARM_WORLD_CMD:
      {
        isCmdDualArmPoseUpdated_ = true;
        desireMode_ = LbArmControlMode::WorldFrame;
        cmdDualArmPoseDesiredTime_ = desireTime;
        /*******************************赋值双臂指令********************************/
        Eigen::VectorXd tmpDualArmCmd = cmd_vec.head(desiredSize);
        left_arm_traj_pose_.head(3) = tmpDualArmCmd.head(3);
        Eigen::Quaterniond leftQuat = Eigen::AngleAxisd(tmpDualArmCmd(3), Eigen::Vector3d::UnitZ())   // yaw (Z)
                                    * Eigen::AngleAxisd(tmpDualArmCmd(4), Eigen::Vector3d::UnitY()) // pitch (Y)
                                    * Eigen::AngleAxisd(tmpDualArmCmd(5), Eigen::Vector3d::UnitX()); // roll (X)
        left_arm_traj_pose_.tail(4) = leftQuat.coeffs();
        right_arm_traj_pose_.head(3) = tmpDualArmCmd.tail(desiredSize/2).head(3);
        Eigen::Quaterniond rightQuat = Eigen::AngleAxisd(tmpDualArmCmd.tail(desiredSize/2)(3), Eigen::Vector3d::UnitZ())   // yaw (Z)
                                     * Eigen::AngleAxisd(tmpDualArmCmd.tail(desiredSize/2)(4), Eigen::Vector3d::UnitY()) // pitch (Y)
                                     * Eigen::AngleAxisd(tmpDualArmCmd.tail(desiredSize/2)(5), Eigen::Vector3d::UnitX()); // roll (X)
        right_arm_traj_pose_.tail(4) = rightQuat.coeffs();
        /*************************************************************************/
        break;
      }
      case LbTimedPosCmdType::DUAL_ARM_LOCAL_CMD:
      {
        isCmdDualArmPoseUpdated_ = true;
        desireMode_ = LbArmControlMode::LocalFrame;
        cmdDualArmPoseDesiredTime_ = desireTime;
        /*******************************赋值双臂指令********************************/
        Eigen::VectorXd tmpDualArmCmd = cmd_vec.head(desiredSize);
        left_arm_traj_pose_.head(3) = tmpDualArmCmd.head(3);
        Eigen::Quaterniond leftQuat = Eigen::AngleAxisd(tmpDualArmCmd(3), Eigen::Vector3d::UnitZ())   // yaw (Z)
                                    * Eigen::AngleAxisd(tmpDualArmCmd(4), Eigen::Vector3d::UnitY()) // pitch (Y)
                                    * Eigen::AngleAxisd(tmpDualArmCmd(5), Eigen::Vector3d::UnitX()); // roll (X)
        left_arm_traj_pose_.tail(4) = leftQuat.coeffs();
        right_arm_traj_pose_.head(3) = tmpDualArmCmd.tail(desiredSize/2).head(3);
        Eigen::Quaterniond rightQuat = Eigen::AngleAxisd(tmpDualArmCmd.tail(desiredSize/2)(3), Eigen::Vector3d::UnitZ())   // yaw (Z)
                                     * Eigen::AngleAxisd(tmpDualArmCmd.tail(desiredSize/2)(4), Eigen::Vector3d::UnitY()) // pitch (Y)
                                     * Eigen::AngleAxisd(tmpDualArmCmd.tail(desiredSize/2)(5), Eigen::Vector3d::UnitX()); // roll (X)
        right_arm_traj_pose_.tail(4) = rightQuat.coeffs();
        /*************************************************************************/
        break;
      }
      case LbTimedPosCmdType::ARM_JOINT_CMD:
      {
        isCmdArmJointUpdated_ = true;
        desireMode_ = LbArmControlMode::JointSpace;  // 切换模式
        cmdArmJointDesiredTime_ = desireTime;
        enableQuickJointControl_ = true;  // 采用 kuavo_arm_traj 一样的发送方式
        arm_joint_traj_ = cmd_vec.head(desiredSize);
        break;
      }
    }
  }

}  // namespace mobile_manipulator
}  // namespace ocs2