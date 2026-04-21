#include "mobile_manipulator_controllers/mobileManipulatorIkTarget.h"

namespace mobile_manipulator_controller
{
using namespace ocs2;

MobileManipulatorIkTarget::MobileManipulatorIkTarget(ros::NodeHandle& nodeHandle, const std::string& robotName)
    : nodeHandle_(nodeHandle), robotName_(robotName)
{
    loadParameters();
    setupMobileManipulatorInterface();
    initializeSubscribers();
    initializePublishers();
    initializeServices();
    initializeTimers();
}

void MobileManipulatorIkTarget::run()
{
    running_ = true;
    ros::spin();
}

void MobileManipulatorIkTarget::stop()
{
    running_ = false;
    localFrameData_.isValid = false;
    targetTrajectoriesReceived_ = false;
}

bool MobileManipulatorIkTarget::setQuest3Utils(bool useQuest3Utils)
{
    use_quest3_utils_ = useQuest3Utils;
    ROS_INFO("use_quest3_utils set to: %d", use_quest3_utils_);
    if(use_quest3_utils_)
        frameType_ = FrameType::VRFrame;
    return true;
}

bool MobileManipulatorIkTarget::setFrameType(FrameType frameType)
{
    if(frameType == FrameType::CurrentFrame)
    {
        ROS_INFO("[setFrameType]: Keep CurrentFrame");
    }
    else 
    {
        frameType_ = frameType;
        ROS_INFO("[setFrameType]: Set frameType_ to %d", static_cast<int>(frameType_));
    }
    return true;
}

FrameType MobileManipulatorIkTarget::getFrameType() const
{
    return frameType_;
}

bool MobileManipulatorIkTarget::isObservationReceived() const
{
    return observationReceived_;
}

bool MobileManipulatorIkTarget::isHumanoidObservationReceived() const
{
    return humanoidObservationReceived_;
}

int MobileManipulatorIkTarget::getEffTrajReceived() const
{
    return effTrajReceived_;
}

void MobileManipulatorIkTarget::loadParameters()
{
    // Get use_quest3_utils parameter
    if (!nodeHandle_.getParam("use_quest3_utils", use_quest3_utils_)) {
        ROS_WARN("Parameter 'use_quest3_utils' not found, using default value: false");
    }
    
    ROS_WARN("Waiting for parameter 'com_height'...");
    while (!nodeHandle_.getParam("com_height", comHeight_)) {
        ros::Duration(0.5).sleep();  // 每0.5秒检查一次
    }
    ROS_INFO(">>>>>>>>>>>>>>>>>>>>> Got com_height: %f <<<<<<<<<<<<<<<<<<<<<<", comHeight_);

    ROS_INFO("use_quest3_utils: %d", use_quest3_utils_); 
    if(use_quest3_utils_)
        frameType_ = FrameType::VRFrame;
}

void MobileManipulatorIkTarget::setupMobileManipulatorInterface()
{
    std::string taskFile, libFolder, urdfFile;
    nodeHandle_.getParam("/mm/taskFile", taskFile);
    nodeHandle_.getParam("/mm/libFolder", libFolder);
    nodeHandle_.getParam("/mm/urdfFile", urdfFile);

    mobileManipulatorInterface_ = std::make_shared<ocs2::mobile_manipulator::MobileManipulatorInterface>(taskFile, libFolder, urdfFile);
    info_ = mobileManipulatorInterface_->getManipulatorModelInfo();
    
    // 初始化 latestHumanoidObservation_.state 的大小
    // 12(躯干动量+躯干6dof) + 12(leg) + armDim
    latestHumanoidObservation_.state.setZero(12 + 12 + info_.armDim);
    
    pinocchioInterface_ptr_.reset(new PinocchioInterface(mobileManipulatorInterface_->getPinocchioInterface()));
    pinocchioMappingPtr_ = std::make_unique<ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping>(info_);
    eeSpatialKinematicsPtr_ = std::make_shared<PinocchioEndEffectorSpatialKinematics>(mobileManipulatorInterface_->getPinocchioInterface(), *pinocchioMappingPtr_.get(),
                                                                                    info_.eeFrames);
    eeSpatialKinematicsPtr_->setPinocchioInterface(*pinocchioInterface_ptr_.get());
}

void MobileManipulatorIkTarget::initializeSubscribers()
{
    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
        this->observationCallback(msg);
    };
    auto humanoidObservationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
        this->humanoidObservationCallback(msg);
    };

    observationSubscriber_ = nodeHandle_.subscribe<ocs2_msgs::mpc_observation>(robotName_ + "_mpc_observation", 1, observationCallback);
    humanoidObservationSubscriber_ = nodeHandle_.subscribe<ocs2_msgs::mpc_observation>("humanoid_wbc_observation", 1, humanoidObservationCallback);
    ikCmdSubscriber_ = nodeHandle_.subscribe<kuavo_msgs::twoArmHandPoseCmd>("/mm/two_arm_hand_pose_cmd", 10, &MobileManipulatorIkTarget::ikCmdCallback, this);
    basePoseCmdSubscriber_ = nodeHandle_.subscribe<std_msgs::Float64MultiArray>(robotName_ + "/base_pose_cmd", 10, &MobileManipulatorIkTarget::basePoseCmdCallback, this);
    mmEndEffectorTrajectorySubscriber_ = nodeHandle_.subscribe<kuavo_msgs::armTargetPoses>("/mm/end_effector_trajectory", 10, &MobileManipulatorIkTarget::mmEndEffectorTrajectoryCallback, this);
}

void MobileManipulatorIkTarget::initializePublishers()
{
    effTrajReceivedPublisher_ = nodeHandle_.advertise<std_msgs::Int32>("/mm/eff_traj_received", 10);
}

void MobileManipulatorIkTarget::initializeServices()
{
    setQuest3UtilsService_ = nodeHandle_.advertiseService("set_quest3_utils", &MobileManipulatorIkTarget::setQuest3UtilsCallback, this);
    setFrameTypeService_ = nodeHandle_.advertiseService("set_mm_ctrl_frame", &MobileManipulatorIkTarget::setFrameTypeCallback, this);
    getFrameTypeService_ = nodeHandle_.advertiseService("get_mm_ctrl_frame", &MobileManipulatorIkTarget::getFrameTypeCallback, this);
}

void MobileManipulatorIkTarget::initializeTimers()
{
    status_publish_timer_ = nodeHandle_.createTimer(ros::Duration(0.01), &MobileManipulatorIkTarget::publishStatus, this);
}

void MobileManipulatorIkTarget::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg)
{
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    if(!observationReceived_)
        observationReceived_ = true;
}

void MobileManipulatorIkTarget::humanoidObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg)
{
    // 如果enableHumanoidObservationCallback_为false，则不在回调里更新humanoidObservation_
    if(!enableHumanoidObservationCallback_)
        return;
    latestHumanoidObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    if(!humanoidObservationReceived_)
        humanoidObservationReceived_ = true;
}

bool MobileManipulatorIkTarget::setQuest3UtilsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    use_quest3_utils_ = req.data;
    ROS_INFO("use_quest3_utils set to: %d", use_quest3_utils_);
    res.success = true;
    res.message = "Successfully set use_quest3_utils to " + std::to_string(use_quest3_utils_);
    frameType_ = FrameType::VRFrame;
    return true;
}

bool MobileManipulatorIkTarget::setFrameTypeCallback(kuavo_msgs::setMmCtrlFrame::Request& req, kuavo_msgs::setMmCtrlFrame::Response& res)
{
    FrameType frameType = static_cast<FrameType>(req.frame);
    if(frameType == FrameType::CurrentFrame)
    {
        ROS_INFO("[setFrameTypeCallback]: Keep CurrentFrame");
    }
    else 
    {
        frameType_ = frameType;
        ROS_INFO("[setFrameTypeCallback]: Set frameType_ to %d", static_cast<int>(frameType_));
    }
    res.result = true;
    res.message = "Successfully set frameType_ to " + std::to_string(static_cast<int>(frameType_));
    res.currentFrame = static_cast<int>(frameType_);
    return true;
}

bool MobileManipulatorIkTarget::getFrameTypeCallback(kuavo_msgs::setMmCtrlFrame::Request& req, kuavo_msgs::setMmCtrlFrame::Response& res)
{
    res.result = true;
    res.message = "Successfully get frameType_ to " + std::to_string(static_cast<int>(frameType_));
    res.currentFrame = static_cast<int>(frameType_);
    return true;
}

void MobileManipulatorIkTarget::publishStatus(const ros::TimerEvent& event)
{
    std_msgs::Int32 msg;
    msg.data = effTrajReceived_;
    effTrajReceivedPublisher_.publish(msg);
}

void MobileManipulatorIkTarget::basePoseCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    newBasePoseReceived_ = true;
    const auto& pose = msg->data;
    if(pose.size() > 6)
    {
        ROS_ERROR_STREAM("Invalid base pose command size: " << pose.size());
        return;
    }
    basePoseCmd_.dim = pose.size();
    for(int i = 0; i < pose.size(); i++)
        basePoseCmd_.pose(i) = pose[i];
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> MobileManipulatorIkTarget::transTargetToLocalFrame(const FrameType& frameType, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat, const SystemObservation& observation)
{
    switch(frameType)
    {
        case FrameType::CurrentFrame: // TODO: 暂时保留CurrentFrame
            return transPoseFromVRFrameToLocalFrame(pos, quat, observation);
        case FrameType::VRFrame:
            return transPoseFromVRFrameToLocalFrame(pos, quat, observation);
        case FrameType::WorldFrame:
            return transPoseFromWorldFrameToLocalFrame(pos, quat, observation);
        case FrameType::LocalFrame:
            return std::make_pair(pos, quat);
        case FrameType::MmWorldFrame:
            return transPoseFromMmWorldFrameToLocalFrame(pos, quat, observation);
        default:
            ROS_ERROR("[transTargetToLocalFrame]: Invalid frame type");
            return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    }
}

void MobileManipulatorIkTarget::ikCmdCallback(const kuavo_msgs::twoArmHandPoseCmd::ConstPtr& msg)
{
    auto &hand_poses = msg->hand_poses; // cppcheck-suppress variableScope
    FrameType frameType = static_cast<FrameType>(msg->frame);

    // if (frameType != FrameType::CurrentFrame)
    // {
    //     frameType_ = frameType;
    //     std::cout << "frameType_ set to: " << static_cast<int>(frameType_) << std::endl;
    // }

    // 存储局部坐标系下的原始数据
    {
        std::lock_guard<std::mutex> lock(localDataMutex_);
        Eigen::Vector3d pos_l, pos_r;
        Eigen::Quaterniond quat_l, quat_r;
        pos_l << hand_poses.left_pose.pos_xyz[0], hand_poses.left_pose.pos_xyz[1], hand_poses.left_pose.pos_xyz[2];
        quat_l = Eigen::Quaterniond(hand_poses.left_pose.quat_xyzw[3],
                                    hand_poses.left_pose.quat_xyzw[0], hand_poses.left_pose.quat_xyzw[1], hand_poses.left_pose.quat_xyzw[2]);
        pos_r << hand_poses.right_pose.pos_xyz[0], hand_poses.right_pose.pos_xyz[1], hand_poses.right_pose.pos_xyz[2];
        quat_r = Eigen::Quaterniond(hand_poses.right_pose.quat_xyzw[3],
                                    hand_poses.right_pose.quat_xyzw[0], hand_poses.right_pose.quat_xyzw[1], hand_poses.right_pose.quat_xyzw[2]);
        std::tie(localFrameData_.leftHandPose.pos, localFrameData_.leftHandPose.quat) = transTargetToLocalFrame(frameType, pos_l, quat_l, latestHumanoidObservation_);
        std::tie(localFrameData_.rightHandPose.pos, localFrameData_.rightHandPose.quat) = transTargetToLocalFrame(frameType, pos_r, quat_r, latestHumanoidObservation_);
        localFrameData_.isValid = true;
        localFrameData_.timestamp = ros::Time::now();
        // std::cout << "localFrameData_.timestamp: " << localFrameData_.timestamp << std::endl;
        
        // 清除轨迹数据，因为现在有新的单点数据
        targetTrajectoriesReceived_ = false;
    }
}

void MobileManipulatorIkTarget::mmEndEffectorTrajectoryCallback(const kuavo_msgs::armTargetPoses::ConstPtr& msg)
{
    std::cout << "mmEndEffectorTrajectoryCallback" << std::endl;
    effTrajReceived_++;

    if (msg->values.empty() || msg->times.empty() || msg->values.size() != msg->times.size() * dof_target_pose_) {
        ROS_WARN("[MobileManipulatorIkTarget]: Invalid armTargetPoses data. Empty values or mismatched sizes.");
        return;
    }
    if (!observationReceived_) {
        ROS_WARN("[MobileManipulatorIkTarget]: Current observation not received yet. Skipping trajectory.");
        return;
    }
    
    // 从消息中获取frame类型
    FrameType msgFrameType = static_cast<FrameType>(msg->frame);
    
    const auto& values = msg->values;
    const auto& times = msg->times;
    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    ocs2::vector_t targetState;
    targetState.setZero(dof_target_pose_);
    scalar_t currentTime = latestObservation_.time;

    vector_t currentEefPoses = getMMEefPose(latestObservation_.state);
    timeTrajectory.push_back(currentTime);
    stateTrajectory.push_back(currentEefPoses);

    if (msg->times[0] == 0 && msg->times.size() > 1) {
        currentTime += 0.01;
    }
    
    for (size_t i = 0; i < msg->times.size(); ++i) {
        vector_t current_target_pose(dof_target_pose_);
        for(int j=0; j<dof_target_pose_; ++j)
        {
            current_target_pose(j) = msg->values[i * dof_target_pose_ + j];
        }

        Eigen::Vector3d pos_l = current_target_pose.head<3>();
        Eigen::Quaterniond quat_l(current_target_pose(6), current_target_pose(3), current_target_pose(4), current_target_pose(5)); // w, x, y, z
        
        Eigen::Vector3d pos_r = current_target_pose.segment<3>(7);
        Eigen::Quaterniond quat_r(current_target_pose(13), current_target_pose(10), current_target_pose(11), current_target_pose(12)); // w, x, y, z
        
        if (humanoidObservationReceived_)
        {
            FrameType frameType_Eef_Traj = (msgFrameType != FrameType::CurrentFrame) ? msgFrameType : frameType_;
            // std::cout << "msgFrameType: " << static_cast<int>(msgFrameType) << std::endl;
            // std::cout << "frameType_: " << static_cast<int>(frameType_) << std::endl;
            // std::cout << "frameType_Eef_Traj: " << static_cast<int>(frameType_Eef_Traj) << std::endl;
            if(frameType_Eef_Traj == FrameType::VRFrame)
            {
                std::tie(pos_l, quat_l) = transPoseFromVRFrameToMmWorld(pos_l, quat_l, latestHumanoidObservation_);
                std::tie(pos_r, quat_r) = transPoseFromVRFrameToMmWorld(pos_r, quat_r, latestHumanoidObservation_);
            }
            else if(frameType_Eef_Traj == FrameType::WorldFrame)
            {
                std::tie(pos_l, quat_l) = transPoseFromWorldFrameToMmWorld(pos_l, quat_l, latestHumanoidObservation_);
                std::tie(pos_r, quat_r) = transPoseFromWorldFrameToMmWorld(pos_r, quat_r, latestHumanoidObservation_);
            }
            else if(frameType_Eef_Traj == FrameType::LocalFrame)
            {
                std::tie(pos_l, quat_l) = transPoseFromLocalFrameToMmWorld(pos_l, quat_l, latestHumanoidObservation_);
                std::tie(pos_r, quat_r) = transPoseFromLocalFrameToMmWorld(pos_r, quat_r, latestHumanoidObservation_);
            }
        }
        
        vector_t transformed_pose(dof_target_pose_);
        transformed_pose.head<3>() = pos_l;
        transformed_pose.segment<4>(3) = quat_l.coeffs();
        transformed_pose.segment<3>(7) = pos_r;
        transformed_pose.segment<4>(10) = quat_r.coeffs();
        
        // Adjust the time relative to the current observation time
        scalar_t adjustedTime = currentTime + msg->times[i];

        timeTrajectory.push_back(adjustedTime);
        stateTrajectory.push_back(transformed_pose);
    }
    
    auto targetTrajectories = generateTwoHandTargetTrajectories(stateTrajectory, timeTrajectory);
    {
      std::lock_guard<std::mutex> lock(targetTrajectoriesMutex_);
      targetTrajectories_ = targetTrajectories;
      targetTrajectoriesReceived_ = true;
      localFrameData_.isValid = false;
    }
}

TargetTrajectories MobileManipulatorIkTarget::goalPoseToTargetTrajectories(const IkCmd& cmd_l, const IkCmd& cmd_r, const SystemObservation& observation) {
    // time trajectory
    const scalar_array_t timeTrajectory{observation.time};
    // state trajectory: 3 + 4 for desired position vector and orientation quaternion
    auto l_pose = (vector_t(7) << cmd_l.pos, cmd_l.quat.coeffs()).finished(); 
    auto r_pose = (vector_t(7) << cmd_r.pos, cmd_r.quat.coeffs()).finished();

    vector_t target;
    if(newBasePoseReceived_)
    {
        target = (vector_t(14 + basePoseCmd_.dim) << basePoseCmd_.pose, l_pose, r_pose).finished();
        newBasePoseReceived_ = false;
    }
    else
    {
        target = (vector_t(14) << l_pose, r_pose).finished();        
    }
    const vector_array_t stateTrajectory{target};
    // input trajectory
    const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}

ocs2::TargetTrajectories MobileManipulatorIkTarget::generateTwoHandTargetTrajectories(const ocs2::vector_array_t& poses, const ocs2::scalar_array_t& times) {
    // time trajectory
    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    // input trajectory
    ocs2::vector_array_t inputTrajectory;
    
    for(int i = 0; i < poses.size(); i++)
    {
        timeTrajectory.push_back(times[i]);
        stateTrajectory.push_back(poses[i]);
        inputTrajectory.push_back(ocs2::vector_t::Zero(20));
    }

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}

vector_t MobileManipulatorIkTarget::getMMEefPose(const vector_t& state)
{
    vector_t eefPoses;// pos(x,y,z) + quat(x,y,z,w)
    const auto& model = pinocchioInterface_ptr_->getModel();
    auto& data = pinocchioInterface_ptr_->getData();
    const auto q = pinocchioMappingPtr_->getPinocchioJointPosition(state);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    const auto eefPositions = eeSpatialKinematicsPtr_->getPosition(state);
    const auto eefOrientations = eeSpatialKinematicsPtr_->getOrientation(state);

    if(eefPositions.size() != eefOrientations.size())
        std::cerr << "[MobileManipulatorIkTarget] eefPositions.size() != eefOrientations.size()" << std::endl;
    eefPoses.resize(7*eefPositions.size());
    for(int i = 0; i < eefPositions.size(); i++)
    {
        eefPoses.segment<7>(7*i).head(3) = eefPositions[i];
        eefPoses.segment<7>(7*i).tail(4) = eefOrientations[i].coeffs();
    }
    return std::move(eefPoses);
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> MobileManipulatorIkTarget::transPoseFromVRFrameToMmWorld(
    const Eigen::Vector3d& p_be, const Eigen::Quaterniond& quat_be, const SystemObservation& observation)
{
    Eigen::Vector3d p_mw_b = (Eigen::Vector3d() << observation.state.segment<2>(6), 0).finished();
    Eigen::Vector3d zyx = (Eigen::Vector3d() << observation.state(9), 0, 0).finished();
    Eigen::Matrix3d R_mw_b = ocs2::getRotationMatrixFromZyxEulerAngles(zyx);
    Eigen::Vector3d p_mw_e = p_mw_b + R_mw_b * p_be;
    Eigen::Matrix3d R_mw_e = R_mw_b * quat_be.toRotationMatrix();
    Eigen::Quaterniond quat_mw_e(R_mw_e);
    return std::make_pair(p_mw_e, quat_mw_e);
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> MobileManipulatorIkTarget::transPoseFromLocalFrameToMmWorld(
    const Eigen::Vector3d& p_le, const Eigen::Quaterniond& quat_le, const SystemObservation& observation)
{
    Eigen::Vector3d p_mw_l = (Eigen::Vector3d() << observation.state.segment<2>(6), -comHeight_).finished();
    Eigen::Vector3d zyx = (Eigen::Vector3d() << observation.state(9), 0, 0).finished();
    Eigen::Matrix3d R_mw_l = ocs2::getRotationMatrixFromZyxEulerAngles(zyx);
    Eigen::Vector3d p_mw_e = p_mw_l + R_mw_l * p_le;
    Eigen::Matrix3d R_mw_e = R_mw_l * quat_le.toRotationMatrix();
    Eigen::Quaterniond quat_mw_e(R_mw_e);
    return std::make_pair(p_mw_e, quat_mw_e);
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> MobileManipulatorIkTarget::transPoseFromWorldFrameToMmWorld(
    const Eigen::Vector3d& p_we, const Eigen::Quaterniond& quat_we, const SystemObservation& observation)
{
    Eigen::Vector3d p_mw_w = (Eigen::Vector3d() << 0.0, 0.0, -comHeight_).finished();
    Eigen::Vector3d p_mw_e = p_mw_w + p_we;
    Eigen::Quaterniond quat_mw_e = quat_we;
    return std::make_pair(p_mw_e, quat_mw_e);
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> MobileManipulatorIkTarget::transPoseFromVRFrameToLocalFrame(
    const Eigen::Vector3d& p_ve, const Eigen::Quaterniond& quat_ve, const SystemObservation& observation)
{
    Eigen::Vector3d p_lv_v = (Eigen::Vector3d() << 0, 0, comHeight_).finished();
    auto p_le = p_lv_v + p_ve; //v跟l的旋转一致
    return std::make_pair(p_le, quat_ve);
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> MobileManipulatorIkTarget::transPoseFromWorldFrameToLocalFrame(
    const Eigen::Vector3d& p_we, const Eigen::Quaterniond& quat_we, const SystemObservation& observation)
{
    Eigen::Vector3d p_wl = (Eigen::Vector3d() << observation.state.segment<2>(6), 0).finished();
    Eigen::Matrix3d R_wl = ocs2::getRotationMatrixFromZyxEulerAngles((Eigen::Vector3d() << observation.state(9), 0, 0).finished());
    Eigen::Vector3d p_le = R_wl.transpose() * (p_we - p_wl);
    Eigen::Matrix3d R_le = R_wl.transpose() * quat_we.toRotationMatrix();
    Eigen::Quaterniond quat_le(R_le);
    return std::make_pair(p_le, quat_le);
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> MobileManipulatorIkTarget::transPoseFromMmWorldFrameToLocalFrame(
    const Eigen::Vector3d& p_me, const Eigen::Quaterniond& quat_me, const SystemObservation& observation)
{
    Eigen::Vector3d p_wm = (Eigen::Vector3d() << 0, 0, comHeight_).finished();
    auto p_we = p_wm + p_me; //m跟w的旋转一致
    return transPoseFromWorldFrameToLocalFrame(p_we, quat_me, observation);
}

// std::pair<Eigen::Vector3d, Eigen::Quaterniond> MobileManipulatorIkTarget::transPoseFromLocalFrameToMmWorld(
//     const Eigen::Vector3d& p_le, const Eigen::Quaterniond& quat_le, const SystemObservation& observation)
// {
//     Eigen::Vector3d p_lv = (Eigen::Vector3d() << 0, 0, comHeight_).finished();
//     auto p_we = p_le - p_lv; //l跟v的旋转一致
//     return transPoseFromVRFrameToMmWorld(p_we, quat_le, observation);
// }


bool MobileManipulatorIkTarget::getTargetTrajectories(TargetTrajectories& targetTrajectories)
{
  if(!observationReceived_)
  {
    ROS_WARN_STREAM("No observation received yet, cannot compute target trajectories");
    return false;
  }
  if(!humanoidObservationReceived_)
  {
    ROS_WARN_STREAM("No humanoid observation received yet, cannot compute target trajectories");
    return false;
  }

  std::lock_guard<std::mutex> lock(localDataMutex_);

  // 根据数据类型生成目标轨迹
  if (localFrameData_.isValid) {
    // 处理单点数据
    IkCmd transformed_left = localFrameData_.leftHandPose;
    IkCmd transformed_right = localFrameData_.rightHandPose;
    
    // 根据当前观测进行坐标变换
    std::tie(transformed_left.pos, transformed_left.quat) = transPoseFromLocalFrameToMmWorld(
        transformed_left.pos, transformed_left.quat, latestHumanoidObservation_);
    std::tie(transformed_right.pos, transformed_right.quat) = transPoseFromLocalFrameToMmWorld(
        transformed_right.pos, transformed_right.quat, latestHumanoidObservation_);
    
    auto target = goalPoseToTargetTrajectories(transformed_left, transformed_right, latestObservation_);
    {
      std::lock_guard<std::mutex> lock(targetTrajectoriesMutex_);
      targetTrajectories = target;
    }
    return true;
  }
  else if (targetTrajectoriesReceived_) {
    std::lock_guard<std::mutex> lock(targetTrajectoriesMutex_);
    targetTrajectories = targetTrajectories_;
    targetTrajectoriesReceived_ = false;
    return true;
  }
  return false;
}

bool MobileManipulatorIkTarget::setHumanoidObservationByMmState(const vector_t& mmState)
{
    if(mmState.size() != 6+info_.armDim)
    {
        ROS_ERROR("[MobileManipulatorIkTarget]: Invalid mmState size");
        return false;
    }
    latestHumanoidObservation_.state.segment<6>(6) = mmState.segment<6>(0);
    latestHumanoidObservation_.state.segment(24, info_.armDim) = mmState.segment(6, info_.armDim);
    latestHumanoidObservation_.time = ros::Time::now().toSec();
    humanoidObservationReceived_ = true;
    return true;
}

}  // namespace mobile_manipulator_controller 