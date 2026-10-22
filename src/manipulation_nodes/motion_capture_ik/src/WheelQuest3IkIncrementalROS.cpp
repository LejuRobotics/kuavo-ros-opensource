#include "motion_capture_ik/WheelQuest3IkIncrementalROS.h"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/changeLbQuickModeSrv.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <cmath>
#include <sstream>

#include <leju_utils/define.hpp>
#include <time.h>
#include <leju_utils/math.hpp>
#include <leju_utils/RosMsgConvertor.hpp>
#include <ocs2_core/thread_support/SetThreadPriority.h>

#include "motion_capture_ik/WheelArmControlBaseROS.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"
#include "motion_capture_ik/WheelIncrementalControlModule.h"
#include "motion_capture_ik/WheelJoyStickHandler.h"

namespace HighlyDynamic {
using namespace leju_utils::ros_msg_convertor;

namespace {

void updateHandConstraintUnlocked(std::vector<PoseData>& poseList,
                                  int handIndex,
                                  const Eigen::Vector3d& handPos,
                                  const Eigen::Quaterniond& handQuat) {
  if (poseList.size() <= static_cast<size_t>(handIndex)) {
    return;
  }
  poseList[handIndex].position = handPos;
  poseList[handIndex].rotation_matrix = handQuat.toRotationMatrix();
}

void updateElbowConstraintUnlocked(std::vector<PoseData>& poseList, int elbowIndex, const Eigen::Vector3d& elbowPos) {
  if (poseList.size() <= static_cast<size_t>(elbowIndex)) {
    return;
  }
  poseList[elbowIndex].position = elbowPos;
}
}  // namespace

void WheelQuest3IkIncrementalROS::applyWorkerThreadScheduling(const char* threadName, int priority) const {
  if (priority > 0) {
    ocs2::setThisThreadPriority(priority);
    ROS_INFO("[WheelQuest3IkIncrementalROS] %s: SCHED_FIFO priority %d", threadName, priority);
  } else {
    ROS_INFO("[WheelQuest3IkIncrementalROS] %s: priority=0, keep SCHED_OTHER", threadName);
  }

  if (vrIkThreadCpus_.empty()) {
    return;
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  std::ostringstream cpuList;
  for (size_t i = 0; i < vrIkThreadCpus_.size(); ++i) {
    const int cpu = vrIkThreadCpus_[i];
    if (cpu < 0 || cpu >= CPU_SETSIZE) {
      ROS_WARN("[WheelQuest3IkIncrementalROS] %s: skip invalid CPU id %d", threadName, cpu);
      continue;
    }
    CPU_SET(cpu, &cpuset);
    if (!cpuList.str().empty()) {
      cpuList << ", ";
    }
    cpuList << cpu;
  }

  if (CPU_COUNT(&cpuset) == 0) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] %s: no valid CPU in affinity list, skip binding", threadName);
    return;
  }

  ocs2::setThreadAffinity(cpuset);
  ROS_INFO("[WheelQuest3IkIncrementalROS] %s: CPU affinity -> [%s]", threadName, cpuList.str().c_str());
}

void WheelQuest3IkIncrementalROS::publishSolveLoopTimingMs(const ros::Publisher& publisher, double ms) const {
  if (!enableSolveLoopTimingLog_ || !publisher) {
    return;
  }
  std_msgs::Float64 msg;
  msg.data = ms;
  publisher.publish(msg);
}

void WheelQuest3IkIncrementalROS::publishLockWaitTimingMs(const ros::Publisher& publisher, double ms) const {
  if (!enableLockWaitTimingLog_ || !publisher) {
    return;
  }
  std_msgs::Float64 msg;
  msg.data = ms;
  publisher.publish(msg);
}

void WheelQuest3IkIncrementalROS::logArmTrajPublishStampPeriod(const ros::Time& stamp) {
  if (enableLockWaitTimingLog_ && hasLastArmTrajPublishStamp_) {
    publishLockWaitTimingMs(pubArmTrajStampPeriodMsPublisher_,
                            (stamp - lastArmTrajPublishStamp_).toSec() * 1000.0);
  }
  lastArmTrajPublishStamp_ = stamp;
  hasLastArmTrajPublishStamp_ = true;
}

WheelQuest3IkIncrementalROS::WheelQuest3IkIncrementalROS(ros::NodeHandle& nodeHandle,
                                               double publishRate,
                                               bool debugPrint,
                                               ArmIdx ctrlArmIdx)
    : WheelArmControlBaseROS(nodeHandle, publishRate, debugPrint), ctrlArmIdx_(ctrlArmIdx) {}

WheelQuest3IkIncrementalROS::~WheelQuest3IkIncrementalROS() {
  shouldStop_ = true;

  if (ikSolveThread_.joinable()) {
    ikSolveThread_.join();
  }
  if (jointStatePublishThread_.joinable()) {
    jointStatePublishThread_.join();
  }
  arm_traj_writer_.shutdown();
}

void WheelQuest3IkIncrementalROS::run() {
  ikSolveThread_ = std::thread(&WheelQuest3IkIncrementalROS::solveIkHandElbowThreadFunction, this);
  jointStatePublishThread_ = std::thread(&WheelQuest3IkIncrementalROS::publishJointStatesThreadFunction, this);

  // 标定后调 FSM 初始化手臂 mode 1（等价于 X+A 到跟随态的主路径）
  std::thread bootstrapArmModeThread = std::thread([this]() {
    // Dual VR（input_mode=dual_vr）：标定完成判定必须等左右两个按侧实例都完成，
    // 否则单侧先完成就 bootstrap，另一侧尚未标定。single_vr 下两侧指向同一实例，
    // 语义与改造前完全一致。
    // 注意：按侧实例的“标定自动完成”由各自按侧回调触发（见
    // WheelArmControlBaseROS::bonePosesLeft/RightCallback），此处只做只读判定，
    // 但标定状态本身是 ×bone 回调线程在 transformerDataMutex_ 内写的可变状态，
    // 读取必须持同一把锁（ArmLengthMeasurement 内部无锁）。
    auto allSideTransformersReady = [this]() {
      std::lock_guard<std::mutex> lock(transformerDataMutex_);
      const auto* leftTransformer = getArmInfoTransformerForSide("Left");
      const auto* rightTransformer = getArmInfoTransformerForSide("Right");
      if (!leftTransformer || !rightTransformer) return false;
      return leftTransformer->isArmLengthMeasurementComplete() &&
             rightTransformer->isArmLengthMeasurementComplete();
    };
    while (ros::ok() && !allSideTransformersReady()) {
      ros::Duration(0.05).sleep();
    }
    ros::Duration(0.1).sleep();

    ros::ServiceClient client = nodeHandle_.serviceClient<std_srvs::Trigger>("/quest3/bootstrap_wheel_arm_mode");
    std_srvs::Trigger srv;
    while (ros::ok()) {
      if (client.call(srv) && srv.response.success) {
        ROS_INFO("[WheelQuest3IkIncrementalROS] bootstrap_wheel_arm_mode ok");
        break;
      }
      if (srv.response.message.find("Not legacy wheel VR") != std::string::npos) {
        ROS_WARN("[WheelQuest3IkIncrementalROS] bootstrap_wheel_arm_mode skipped: %s",
                 srv.response.message.c_str());
        break;
      }
      ROS_WARN_THROTTLE(2.0, "[WheelQuest3IkIncrementalROS] bootstrap_wheel_arm_mode failed: %s",
                        srv.response.message.c_str());
      ros::Duration(0.5).sleep();
    }
  });

  std::cout << "\033[32m[WheelQuest3IkIncrementalROS] spinning start\033[0m" << std::endl;
  ros::spin();
}

void WheelQuest3IkIncrementalROS::solveIkHandElbowThreadFunction() {
  // applyWorkerThreadScheduling("ik_solve_thread", ikSolveThreadPriority_);
  ros::Rate rate(publishRate_);
  // 用于统计时间差的静态变量
  static int loopCount = 0;
  static double totalTimeDiff = 0.0;

  while (!shouldStop() && ros::ok()) {
    loopSyncCount_++;
    recordTimestamp("SolveLoopStart", loopSyncCount_);
    updateSensorArmJointMeanFromSensorData();
    updateSensorArmJointFromSensorData();
    updateFkCacheFromSensorData();

    // 实物：硬件未就绪则不进入后续（放在未激活判断与 fsm 之前，少做无效状态维护）
    {
      bool is_real_robot = false;
      if (ros::param::getCached("/is_real", is_real_robot) && is_real_robot) {
        int hardware_is_ready = 0;
        if (!ros::param::getCached("/hardware/is_ready", hardware_is_ready) || hardware_is_ready == 0) {
          ROS_INFO_THROTTLE(3.0,
                            "[WheelQuest3IkIncrementalROS] Waiting /hardware/is_ready != 0 before incremental FSM (real robot)");
          reset();
          rate.sleep();
          continue;
        }
      }
    }

    const bool chestPoseUpdateEnabled = joyStickHandlerPtr_->getRightJoyStickYHoldWithX();
    bool chestPositionUpdateEnable = false;
    if (chestPoseUpdateEnabled) {
      chestPositionUpdateEnable = joyStickHandlerPtr_->getRightJoyStickYHold();
    }
    if (chestPoseUpdateEnabled != chestIncrementalUpdateEnabled_) {
      if (!chestPoseUpdateEnabled) {
        frozenChestQuat_ = latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST
                               ? Eigen::Quaterniond(latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix).normalized()
                               : getRobotChestQuatRef();
        if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
          frozenRobotChestPos_ = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position;
        } else {
          frozenRobotChestPos_ = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_;
        }
      } else {
        // 恢复时从冻结期间最后的机器人目标重新建立人体胸部锚点。
        // 不补执行总开关关闭期间的人体位移，首个有效采样的胸部增量归零。
        chestAnchorNeedsRefresh_ = true;
        // 保留关闭前最后的朝向；恢复后只跟随新的 VR 朝向变化。
        const Eigen::Quaterniond rawChestQuat = getTorsoOwnerChestQuat();
        const Eigen::Quaterniond rawChestYawPitch =
            computeYawPitchOnlyQuatFromRotationMatrix(rawChestQuat.toRotationMatrix());
        chestFollowRecoveryOffset_ =
            (frozenChestQuat_ * rawChestYawPitch.conjugate()).normalized();
      }
      {
        std::lock_guard<std::mutex> commandLock(lbLegCommandMutex_);
        lbLegMasterPublishEnabled_ = chestPoseUpdateEnabled;
        lowerBodyMasterFreezeActive_ = !chestPoseUpdateEnabled && hasPublishedLbCommand_;
        if (lowerBodyMasterFreezeActive_) {
          lowerBodyMasterFreezeAnchor_ = lastPublishedLbCommand_;
        }
      }
      chestIncrementalUpdateEnabled_ = chestPoseUpdateEnabled;
    }
    chestPositionUpdateEnable_ = chestPoseUpdateEnabled && chestPositionUpdateEnable;
    drakeSolveUpdateChestPosition_ =
        drakeSolveUpdateChestPositionConfig_ && chestIncrementalUpdateEnabled_ && chestPositionUpdateEnable_;
    // Drake chest orientation 每帧基准恢复（上一帧的 ownership 收紧不能累积到本帧；
    // dual 下 position 那行的写法同理，只是它写的是 _ 而不是 Config_）。
    drakeSolveUpdateChestOrientation_ = drakeSolveUpdateChestOrientationConfig_;

    // Transformer 的可变状态只由骨骼回调在 transformerDataMutex_ 内写入，
    // 因此手部/肩肘快照在本临界区内完成拷贝，锁外不再触碰 Transformer。
    // Dual VR 躯干 ownership：owner 侧 Chest 的"新鲜度判定 + Quaternion 快照"也必须在本临界区内完成，
    // 使同一次解算的手部/肩肘/肘部、owner Chest 四元数、owner 数据有效性来自同一个受保护快照。
    // 判定口径与 Chest Position 完全一致（同一侧实例 + 同一 kChestStaleTimeoutMs），不新增第二套超时。
    bool ownerChestFreshSnapshot = false;
    Eigen::Quaterniond ownerChestQuatSnapshot = Eigen::Quaterniond::Identity();
    {
      std::lock_guard<std::mutex> lock(transformerDataMutex_);
      // Dual VR（input_mode=dual_vr）：Left 的状态只允许来自 Quest-A 的实例，
      // Right 的只允许来自 Quest-B 的实例，绝不允许某侧 Chest 参与对侧解算。
      // single_vr 下 getArmInfoTransformerForSide 两侧都返回主实例，行为与改造前完全一致。
      const auto* leftTransformer = getArmInfoTransformerForSide("Left");
      const auto* rightTransformer = getArmInfoTransformerForSide("Right");
      if (leftTransformer) {
        latestLeftHandPose_vr_ =
            leftTransformer->getLeftHandPoseRelativeToChest();
        latestHumanLeftShoulderPos_ =
            leftTransformer->getLeftShoulderPose().position;
        latestHumanLeftElbowPos_ =
            leftTransformer->getLeftElbowPose().position;
      }
      if (rightTransformer) {
        latestRightHandPose_vr_ =
            rightTransformer->getRightHandPoseRelativeToChest();
        latestHumanRightShoulderPos_ =
            rightTransformer->getRightShoulderPose().position;
        latestHumanRightElbowPos_ =
            rightTransformer->getRightElbowPose().position;
      }
      // dual_vr：owner 侧（left -> Quest-A，right -> Quest-B）Chest 的新鲜度判断与四元数快照。
      // 绝不在锁外调用 getFreshSideChestPose()：该函数自身要取 transformerDataMutex_，会自锁死。
      if (isDualVrMode()) {
        Quest3ArmInfoTransformer* ownerTransformer =
            getArmInfoTransformerForSide(torsoSideRight() ? "Right" : "Left");
        const std::chrono::steady_clock::time_point& ownerRecvTime =
            torsoSideRight() ? rightBoneRecvTime_ : leftBoneRecvTime_;
        if (ownerTransformer && ownerRecvTime.time_since_epoch().count() != 0) {
          // 时间戳只在骨骼帧被成功处理后才刷新（见 bonePosesLeft/RightCallback），
          // 因此超过阈值即代表 owner 侧 Chest 已停止更新。
          const int64_t ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - ownerRecvTime).count();
          if (ageMs <= kChestStaleTimeoutMs && ownerTransformer->hasValidChestPose()) {
            ownerChestQuatSnapshot = ownerTransformer->getChestQuaternion();
            ownerChestFreshSnapshot = true;
          }
        }
      }
    }
    // Dual VR 躯干 ownership：owner 侧 Chest 朝向的有效性直接取自上方的同一临界区快照，
    // 与本周期的手部/肩肘快照严格对齐（不再单独取锁判定）。
    //   left  -> Left 实例（Quest-A）的 Chest；right -> Right 实例（Quest-B）的 Chest；
    // single_vr 恒视为新鲜，行为与改造前完全一致（该缓存只在 dual 下使用）。
    torsoOwnerChestFreshThisCycle_ = !isDualVrMode() || ownerChestFreshSnapshot;
    if (isDualVrMode()) {
      const Eigen::Quaterniond& ownerChestQuat = ownerChestQuatSnapshot;
      if (torsoOwnerChestFreshThisCycle_) {
        if (!chestIncrementalUpdateEnabled_) {
          // 躯干朝向跟随未开启：该缓存不参与任何控制（调用方一律走 getRobotChestQuatRef()），
          // 因此这里只做透传，且必须把恢复锚点清零。否则启动阶段"第一次收到 owner Chest 数据"
          // 会被当成"失效→恢复首帧"而把绝对跟随变成增量跟随，改变正常启动行为。
          torsoOwnerChestQuatRecoveryOffset_.setIdentity();
          torsoOwnerChestFreshPrevCycle_ = true;
          std::lock_guard<std::mutex> lock(chestPoseMutex_);
          latestTorsoOwnerChestQuat_ = ownerChestQuat.normalized();
        } else {
          // 失效→恢复的首帧必须与机器人当前姿态参考连续：以当帧机器人参考为基准重建锚点偏移，
          // 令本帧目标恰等于当前机器人姿态（不跳变），后续按人体朝向增量正常跟随。
          // 连续有效期间该偏移恒为单位阵，退化为原来的绝对跟随，行为不变。
          if (!torsoOwnerChestFreshPrevCycle_) {
            torsoOwnerChestQuatRecoveryOffset_ =
                (getRobotChestQuatRef().normalized() * ownerChestQuat.normalized().conjugate()).normalized();
            ROS_WARN("[WheelQuest3IkIncrementalROS] Torso owner chest orientation recovered: re-anchor to "
                     "current robot chest reference to avoid waist jump");
          }
          torsoOwnerChestFreshPrevCycle_ = true;
          std::lock_guard<std::mutex> lock(chestPoseMutex_);
          latestTorsoOwnerChestQuat_ =
              (torsoOwnerChestQuatRecoveryOffset_ * ownerChestQuat.normalized()).normalized();
        }
      } else {
        // 无效时不刷新缓存：latestTorsoOwnerChestQuat_ 保持失效前已下发的参考（绝不用过期
        // 四元数、也绝不用 Identity 顶替）；本周期 Drake 的 chest orientation 更新权限
        // 由 torsoOwnerChestFreshThisCycle_ 收紧（见 fsmProcess）。
        torsoOwnerChestFreshPrevCycle_ = false;
      }
    }
    // 捕获当前骨骼数据接收时刻和序列号，随IK结果一起传播
    {
      std::lock_guard<std::mutex> lock(boneRecvTimeMutex_);
      ikResultBoneRecvTime_ = boneRecvTime_;
      currentBoneSeq_ = boneDataSeq_;
    }

    // 【三点跳变检测】验证并过滤 VR 数据中的异常跳变
    bool currentLeftGripPressed = joyStickHandlerPtr_ ? joyStickHandlerPtr_->isLeftGrip() : false;
    bool currentRightGripPressed = joyStickHandlerPtr_ ? joyStickHandlerPtr_->isRightGrip() : false;
    validateVrPose(latestLeftHandPose_vr_, latestLeftHandPose_vr_, "Left", currentLeftGripPressed);
    validateVrPose(latestRightHandPose_vr_, latestRightHandPose_vr_, "Right", currentRightGripPressed);
    auto humanArmPoseValid = [](const Eigen::Vector3d& shoulder,
                                const Eigen::Vector3d& elbow,
                                const Eigen::Vector3d& hand) {
      if (!shoulder.allFinite() || !elbow.allFinite() || !hand.allFinite()) return false;
      const Eigen::Vector3d axis = hand - shoulder;
      const double axisNorm = axis.norm();
      if (axisNorm < 1.0e-4) return false;
      const Eigen::Vector3d radial =
          (elbow - shoulder) - axis * ((elbow - shoulder).dot(axis) / axis.squaredNorm());
      return radial.norm() > 1.0e-4;
    };
    latestHumanLeftArmPoseValid_ = humanArmPoseValid(
        latestHumanLeftShoulderPos_, latestHumanLeftElbowPos_, latestLeftHandPose_vr_.position);
    latestHumanRightArmPoseValid_ = humanArmPoseValid(
        latestHumanRightShoulderPos_, latestHumanRightElbowPos_, latestRightHandPose_vr_.position);

    if (armControlMode_ == 0 || armControlMode_ == 1) {
      if (lastArmControlMode_ == 2) {
        fsmExit();
      }
      reset();  // 机器人未激活（含 2→0/1），持续重置各类状态，确保进入系统时正常
      rate.sleep();
      continue;  // 机器人未激活，不进行后续流程
    }

    const auto loopWallStart = std::chrono::steady_clock::now();
    if (enableSolveLoopTimingLog_ && hasLastSolveLoopWallStart_) {
      const double loopPeriodMs =
          std::chrono::duration<double, std::milli>(loopWallStart - lastSolveLoopWallStart_).count();
      publishSolveLoopTimingMs(solveLoopPeriodMsPublisher_, loopPeriodMs);
    }

    const auto fsmBlockStart = std::chrono::steady_clock::now();
    auto measureStage = [&](const ros::Publisher& publisher, auto&& fn) {
      if (!enableSolveLoopTimingLog_) {
        fn();
        return;
      }
      const auto stageStart = std::chrono::steady_clock::now();
      fn();
      const double stageMs =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - stageStart).count();
      publishSolveLoopTimingMs(publisher, stageMs);
    };

    measureStage(solveLoopFsmEnterMsPublisher_, [&]() { fsmEnter(); });
    measureStage(solveLoopFsmChangeMsPublisher_, [&]() { fsmChange(); });
    measureStage(solveLoopFsmProcessMsPublisher_, [&]() { fsmProcess(); });
    measureStage(solveLoopFsmExitMsPublisher_, [&]() { fsmExit(); });
    // 跟随中双手松开且未开躯干增量时：若实测 chest FK 相对冻结锚点已漂（例如外部 TUI 改过躯干），
    // 再按 FK 重锚一次，避免映射目标继续钉在旧胸位上。
    if (armControlMode_.load() == 2 && hasLatestWaistYawFk_ && !chestIncrementalUpdateEnabled_ &&
        joyStickHandlerPtr_ && !joyStickHandlerPtr_->isLeftGrip() && !joyStickHandlerPtr_->isRightGrip()) {
      const double chestDrift = (latestWaistYawFkPos_ - frozenRobotChestPos_).norm();
      if (chestDrift > kExternalTorsoReanchorPosTol) {
        reanchorMappedUpperBodyFromMeasuredFk("external_torso_fk_drift");
      }
    }
    measureStage(solveLoopPublishEeMsPublisher_, [&]() { publishEndEffectorControlData(); });
    measureStage(solveLoopPublishAuxMsPublisher_, [&]() { publishAuxiliaryStates(); });
    measureStage(solveLoopPublishMarkersMsPublisher_, [&]() { publishWholeBodyRefMarkers(); });

    if (enableSolveLoopTimingLog_) {
      const double fsmBlockTotalMs =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - fsmBlockStart).count();
      publishSolveLoopTimingMs(solveLoopFsmBlockTotalMsPublisher_, fsmBlockTotalMs);
    }

    lastSolveLoopWallStart_ = loopWallStart;
    hasLastSolveLoopWallStart_ = true;

    rate.sleep();
  }
}

void WheelQuest3IkIncrementalROS::publishJointStatesThreadFunction() {
  applyWorkerThreadScheduling("arm_traj_publish_thread", armTrajPublishThreadPriority_);
  ros::param::getCached("/reset_joint_to_default", resetJointToDefaultWheel_);
  // 不用 ros::Rate：落后时会追赶连发，header.stamp≈同一时刻 → PlotJuggler/录包呈“堆在一起”
  const double frequency = std::max(jointStatePublishRateHz_, 1.0);
  const double periodSec = 1.0 / frequency;
  struct timespec next_time {};
  clock_gettime(CLOCK_MONOTONIC, &next_time);

  auto advanceNextTime = [&](struct timespec& t) {
    t.tv_nsec += static_cast<long>(periodSec * 1e9);
    while (t.tv_nsec >= 1000000000L) {
      t.tv_sec += 1;
      t.tv_nsec -= 1000000000L;
    }
  };

  while (!shouldStop() && ros::ok()) {
    const auto pubLoopStart = std::chrono::steady_clock::now();
    if (enableLockWaitTimingLog_ && hasLastPubArmTrajWallStart_) {
      const double periodMs = std::chrono::duration<double, std::milli>(pubLoopStart - lastPubArmTrajWallStart_).count();
      publishLockWaitTimingMs(pubArmTrajPeriodMsPublisher_, periodMs);
    }

    if (armControlMode_ == 2) {
      publishJointStates();
    } else {
      if(resetJointToDefaultWheel_)
      {
        publishDefaultJointStates();
      }
    }

    if (enableLockWaitTimingLog_) {
      const double totalMs =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pubLoopStart).count();
      publishLockWaitTimingMs(pubArmTrajTotalMsPublisher_, totalMs);
      lastPubArmTrajWallStart_ = pubLoopStart;
      hasLastPubArmTrajWallStart_ = true;
    }

    advanceNextTime(next_time);
    struct timespec now {};
    clock_gettime(CLOCK_MONOTONIC, &now);
    const double lagSec =
        static_cast<double>(now.tv_sec - next_time.tv_sec) +
        static_cast<double>(now.tv_nsec - next_time.tv_nsec) * 1e-9;
    if (lagSec > periodSec) {
      // 严重落后：对齐到 now+period，丢弃追赶补发，避免 stamp 成簇
      next_time = now;
      advanceNextTime(next_time);
      ROS_WARN_THROTTLE(1.0,
                        "[WheelQuest3IkIncrementalROS] arm_traj publish lag=%.1f ms, skip Rate catch-up",
                        lagSec * 1e3);
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
  }
}

void WheelQuest3IkIncrementalROS::reanchorMappedUpperBodyFromMeasuredFk(const char* reason) {
  const Eigen::Vector3d chestPos = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_;
  const Eigen::Quaterniond chestQuat = getRobotChestQuatRef();
  const Eigen::Vector3d leftShoulderPos =
      hasLatestLeftShoulderFk_ ? latestLeftShoulderFkPos_ : robotLeftFixedShoulderPos_;
  const Eigen::Vector3d rightShoulderPos =
      hasLatestRightShoulderFk_ ? latestRightShoulderFkPos_ : robotRightFixedShoulderPos_;
  const Eigen::Vector3d leftElbowPos = computeElbow(leftLink6Position_, leftEndEffectorPosition_, l2_);
  const Eigen::Vector3d rightElbowPos = computeElbow(rightLink6Position_, rightEndEffectorPosition_, l2_);

  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
    latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = chestPos;
    latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestQuat.toRotationMatrix();
  }
  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_SHOULDER) {
    latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_SHOULDER].position = leftShoulderPos;
  }
  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_SHOULDER) {
    latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_SHOULDER].position = rightShoulderPos;
  }
  updateHandConstraintUnlocked(
      latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, leftLink6Position_, leftLink6Quat_);
  updateHandConstraintUnlocked(
      latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, rightLink6Position_, rightLink6Quat_);
  updateElbowConstraintUnlocked(latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_ELBOW, leftElbowPos);
  updateElbowConstraintUnlocked(latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_ELBOW, rightElbowPos);

  frozenRobotChestPos_ = chestPos;
  frozenChestQuat_ = chestQuat;
  frozenLeftHandHeightOffset_ = leftLink6Position_.z() - frozenRobotChestPos_.z();
  frozenRightHandHeightOffset_ = rightLink6Position_.z() - frozenRobotChestPos_.z();
  frozenLeftElbowHeightOffset_ = leftElbowPos.z() - frozenRobotChestPos_.z();
  frozenRightElbowHeightOffset_ = rightElbowPos.z() - frozenRobotChestPos_.z();

  // 人体 chest 锚点下次推进前按当前人体 Chest 重建，避免把暂停前的 VR 位移一次性打进恢复后的增量。
  chestAnchorNeedsRefresh_ = true;
  chestFollowRecoveryOffset_.setIdentity();
  hasLeftActiveChestAnchor_ = false;
  hasRightActiveChestAnchor_ = false;
  hasLeftHandPoseInChest_ = false;
  hasRightHandPoseInChest_ = false;
  hasLeftElbowPosInChest_ = false;
  hasRightElbowPosInChest_ = false;
  hasLeftGripReleaseSnapshot_ = false;
  hasRightGripReleaseSnapshot_ = false;

  latestChestPosBeforeOpt_ = chestPos;
  latestChestPosAfterOpt_ = chestPos;
  latestLeftShoulderPosBeforeOpt_ = leftShoulderPos;
  latestLeftShoulderPosAfterOpt_ = leftShoulderPos;
  latestRightShoulderPosBeforeOpt_ = rightShoulderPos;
  latestRightShoulderPosAfterOpt_ = rightShoulderPos;
  latestLeftHandPosBeforeOpt_ = leftLink6Position_;
  latestLeftHandPosAfterOpt_ = leftLink6Position_;
  latestRightHandPosBeforeOpt_ = rightLink6Position_;
  latestRightHandPosAfterOpt_ = rightLink6Position_;
  latestLeftElbowPosBeforeOpt_ = leftElbowPos;
  latestLeftElbowPosAfterOpt_ = leftElbowPos;
  latestRightElbowPosBeforeOpt_ = rightElbowPos;
  latestRightElbowPosAfterOpt_ = rightElbowPos;

  if (chestIncrementalUpdateEnabled_) {
    enterOwnerChestIncrementalMode(chestPos);
  }

  ROS_WARN("[WheelQuest3IkIncrementalROS] Re-anchored upper-body map from measured FK (%s): "
           "chest=[%.3f, %.3f, %.3f] Lhand=[%.3f, %.3f, %.3f] Rhand=[%.3f, %.3f, %.3f]",
           reason ? reason : "unspecified",
           chestPos.x(),
           chestPos.y(),
           chestPos.z(),
           leftLink6Position_.x(),
           leftLink6Position_.y(),
           leftLink6Position_.z(),
           rightLink6Position_.x(),
           rightLink6Position_.y(),
           rightLink6Position_.z());
}

void WheelQuest3IkIncrementalROS::fsmEnter() {
  auto updateChestConstraintFromFk = [&]() {
    // 进入准备动作时保持机器人当前胸部 FK 位姿，不跟随 VR 人体躯干
    const Eigen::Vector3d chestPos = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_;
    const Eigen::Quaterniond chestQuat = getRobotChestQuatRef();
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = chestPos;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestQuat.toRotationMatrix();
    }
    frozenRobotChestPos_ = chestPos;
    frozenChestQuat_ = chestQuat;
  };
  // 正常工作模式 Case 2: (0→2 或 1→2)
  if ((armControlMode_ == 2 && lastArmControlMode_ == 1) || (armControlMode_ == 2 && lastArmControlMode_ == 0)) {
    // S^0 → S^3 顶层状态切换
    exitMode2Counter_ = 0;
    if (!mode2Initialized_) {
      justEnteredMode2_ = true;  // 只在第一次进入mode 2时标记
      mode2Initialized_ = true;  // 标记mode 2已初始化
      mode2EnterTime_ = ros::Time::now();  // 记录进入mode 2的时间戳
    }
    // X+B 恢复 / 从复位进跟随：按当前实测 FK 重锚胸肩手肘，覆盖暂停期间 TUI 改躯干造成的旧锚点。
    if (pendingResumeUpperBodyReanchor_.exchange(false)) {
      reanchorMappedUpperBodyFromMeasuredFk("mode_resume_0_or_1_to_2");
    }
    auto resetMode2State = [&](bool resetIkSolution) {
      {
        std::lock_guard<std::mutex> jointLock(jointStateMutex_);
        // q_ = Eigen::VectorXd::Zero(14);
        dq_ = Eigen::VectorXd::Zero(14);
        // latest_q_ = Eigen::VectorXd::Zero(14);
        latest_dq_ = Eigen::VectorXd::Zero(14);
        lowpass_dq_ = Eigen::VectorXd::Zero(14);
        lb_q_ = Eigen::VectorXd::Zero(4);
        lb_dq_ = Eigen::VectorXd::Zero(4);
        latest_lb_q_ = Eigen::VectorXd::Zero(4);
        latest_lb_dq_ = Eigen::VectorXd::Zero(4);
        lowpass_lb_dq_ = Eigen::VectorXd::Zero(4);
      }
      {
        std::lock_guard<std::mutex> lock(lbLegMoveTimeMutex_);
        lbLegMoveStartTime_ = ros::Time(0);
      }

      {
        std::lock_guard<std::mutex> lock(ikResultMutex_);
        if (resetIkSolution) {
          if (latestIkSolution_.size() == drakeJointStateSize_) {
            latestIkSolution_.setZero();
          } else {
            // roserror latestIkSolution_ size is not equal to
            ROS_ERROR("[WheelQuest3IkIncrementalROS] latestIkSolution_ size is not equal to drakeJointStateSize");
          }
          hasValidIkSolution_ = false;
        }
        // 同步置零 ikLowerBodyJointCommand_
        ikLowerBodyJointCommand_.setZero();
      }
    };

    if (enterMode2ResetCounter_ < ENTER_MODE_2_RESET_COUNT) {
      // NOTES：确保进入mode 2时，左右手位置和姿态都为初始值
      // NOTES: 状态描述：默认机器人在5sec内已经回归到零位，因此进入mode 2时，左右手位置和姿态都为初始值
      Eigen::Vector3d currentLeftHandPos = initZeroLeftLink6Position_;
      Eigen::Vector3d currentRightHandPos = initZeroRightLink6Position_;
      Eigen::Quaterniond currentLeftHandQuat = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond currentRightHandQuat = Eigen::Quaterniond::Identity();

      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, currentLeftHandPos, currentLeftHandQuat);
      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, currentRightHandPos, currentRightHandQuat);

      if (incrementalController_) {
        incrementalController_->reset();
        incrementalController_->setHandQuatSeeds(
            currentLeftHandQuat, currentRightHandQuat, useIncrementalHandOrientation_);
      }

      resetMode2State(true);

      latestIncrementalResult_ = WheelIncrementalPoseResult();

      enterMode2ResetCounter_++;
    }

    const bool shouldEnterLeft =
        incrementalController_->shouldEnterIncrementalModeLeftArm(joyStickHandlerPtr_->isLeftGrip());
    const bool shouldEnterRight =
        incrementalController_->shouldEnterIncrementalModeRightArm(joyStickHandlerPtr_->isRightGrip());

    if (shouldEnterLeft || shouldEnterRight) {
      // 任意手进入增量模式时，同步激活 chest 增量（第二只手进入时不重复激活）。
      // owner 侧 Chest 尚无有效帧时不用零向量建立锚点（否则首次有效 Chest 会被算成
      // 巨大增量、导致腰部目标跳变），改用机器人当前 chest 目标做占位锚点（增量为 0），
      // 之后 owner Chest 首次有效时由 advanceOwnerChestIncremental 重建锚点。
      updateChestConstraintFromFk();
      if (chestIncrementalUpdateEnabled_) {
        const Eigen::Vector3d chestPosForAnchor =
            (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST)
                ? latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position
                : (hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_);
        enterOwnerChestIncrementalMode(chestPosForAnchor);
      }

      // 处理左臂：计算FK -> 更新约束列表 -> 进入增量模式
      if (shouldEnterLeft) {
        // FK 已在主循环更新到缓存

        // 【核心修复】在进入增量模式前，先更新 latestPoseConstraintList_ 为当前 FK 计算的 Link6 位置
        // 避免使用上次退出时保存的旧位置，导致跳变
        updateHandConstraintUnlocked(
            latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, leftLink6Position_, leftLink6Quat_);

        incrementalController_->enterIncrementalModeLeftArm(latestLeftHandPose_vr_,
                                                            latestPoseConstraintList_,
                                                            leftEndEffectorPosition_,
                                                            leftEndEffectorQuat_,
                                                            leftLink4Quat_);
      }

      // 处理右臂：计算FK -> 更新约束列表 -> 进入增量模式
      if (shouldEnterRight) {
        // FK 已在主循环更新到缓存

        // 【核心修复】在进入增量模式前，先更新 latestPoseConstraintList_ 为当前 FK 计算的 Link6 位置
        // 避免使用上次退出时保存的旧位置，导致跳变
        updateHandConstraintUnlocked(
            latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, rightLink6Position_, rightLink6Quat_);

        incrementalController_->enterIncrementalModeRightArm(latestRightHandPose_vr_,
                                                             latestPoseConstraintList_,
                                                             rightEndEffectorPosition_,
                                                             rightEndEffectorQuat_,
                                                             rightLink4Quat_);
      }
    }

    // 超时机制：0→2 和 1→2 都需要超时保护
    ros::Time currentTime = ros::Time::now();
    ros::Time enterTime;
    {
      std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
      enterTime = mode2EnterTime_;
    }

    if (enterTime.isZero()) {  // 如果时间戳未设置，可能是回调函数还未执行，先记录当前时间作为容错机制，避免出现异常
      std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
      if (mode2EnterTime_.isZero()) {
        mode2EnterTime_ = currentTime;
        enterTime = currentTime;
      } else {
        enterTime = mode2EnterTime_;
      }
    }

    double elapsedTime = (currentTime - enterTime).toSec();

    if (elapsedTime <= MODE_2_TIMEOUT_DURATION) {
      // print mode2 timeout duration
      // std::cout << "[WheelQuest3IkIncrementalROS] Mode 2 timeout duration: " << elapsedTime << "s" << std::endl;
      forceDeactivateAllArmCtrlMode();

      updateHandConstraintUnlocked(latestPoseConstraintList_,
                                   POSE_DATA_LIST_INDEX_LEFT_HAND,
                                   initZeroLeftLink6Position_,
                                   Eigen::Quaterniond::Identity());
      updateHandConstraintUnlocked(latestPoseConstraintList_,
                                   POSE_DATA_LIST_INDEX_RIGHT_HAND,
                                   initZeroRightLink6Position_,
                                   Eigen::Quaterniond::Identity());

      // 重置增量控制模块，清除可能被 fsmChange/fsmProcess 更新的 ruckig 滤波状态
      if (incrementalController_) {
        incrementalController_->reset();
        incrementalController_->setHandQuatSeeds(
            Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(), useIncrementalHandOrientation_);
      }

      resetMode2State(false);

      // 在超时时间内，执行进入增量模式（0→2 和 1→2 都需要）
      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, leftLink6Position_, leftLink6Quat_);
      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, rightLink6Position_, rightLink6Quat_);
      // 进入增量模式前，同步激活 chest 增量（避免腰部目标跳变）。
      // owner 侧 Chest 无效时同 fsmEnter：不用零向量建锚点，改钉在当前机器人 chest 约束上。
      updateChestConstraintFromFk();
      if (chestIncrementalUpdateEnabled_) {
        const Eigen::Vector3d chestPosForAnchor =
            (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST)
                ? latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position
                : (hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_);
        enterOwnerChestIncrementalMode(chestPosForAnchor);
      }

      incrementalController_->enterIncrementalModeLeftArm(latestLeftHandPose_vr_,
                                                          latestPoseConstraintList_,
                                                          leftEndEffectorPosition_,
                                                          leftEndEffectorQuat_,
                                                          leftLink4Quat_);

      incrementalController_->enterIncrementalModeRightArm(latestRightHandPose_vr_,
                                                           latestPoseConstraintList_,
                                                           rightEndEffectorPosition_,
                                                           rightEndEffectorQuat_,
                                                           rightLink4Quat_);
    }
  }
}

void WheelQuest3IkIncrementalROS::fsmChange() {
  modeChangeCycle_.resetAll();
  if (armControlMode_ != 2) return;
  modeChangeCycle_.leftHandCtrlModeChanged = joyStickHandlerPtr_->hasLeftArmCtrlModeChanged();
  modeChangeCycle_.rightHandCtrlModeChanged = joyStickHandlerPtr_->hasRightArmCtrlModeChanged();
}

void WheelQuest3IkIncrementalROS::fsmProcess() {
  if (armControlMode_ != 2) return;
  activateController();
  // mode2 下仅在开启躯干控制时激活 chest 增量；进入准备动作时保持机器人当前胸部 FK 姿态
  if (chestIncrementalUpdateEnabled_ && !incrementalController_->isIncrementalModeChest()) {
    // 躯干 owner 侧 Chest：dual 下真正取 A/B 实例自己的 Chest；single 下仍是 /robot_chest_pose。
    // owner Chest 无效时不用零向量建锚点，而是把锚点钉在当前机器人 chest FK 姿态上
    // （增量为 0，机器人 chest 保持当前目标）；之后有效帧由
    // advanceOwnerChestIncremental 用真实 Chest 重建锚点。
    const Eigen::Vector3d chestPos = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_;
    const Eigen::Quaterniond chestQuat = getRobotChestQuatRef();
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = chestPos;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestQuat.toRotationMatrix();
    }
    frozenRobotChestPos_ = chestPos;
    frozenChestQuat_ = chestQuat;
    enterOwnerChestIncrementalMode(chestPos);
  }
  // 0) 在 fsmProcess 统一更新 smoother 状态，确保后续 getModeChangingState() 是最新的
  {
    auto [leftChangingMaintainUpdated, leftChangingInstantUpdated] =
        leftHandSmoother_->updateModeChangingStateIfNeeded(modeChangeCycle_.leftHandCtrlModeChanged);
    auto [rightChangingMaintainUpdated, rightChangingInstantUpdated] =
        rightHandSmoother_->updateModeChangingStateIfNeeded(modeChangeCycle_.rightHandCtrlModeChanged);

    modeChangeCycle_.leftChangingMaintainUpdated = leftChangingMaintainUpdated;
    modeChangeCycle_.rightChangingMaintainUpdated = rightChangingMaintainUpdated;
    modeChangeCycle_.leftChangingInstantUpdated = leftChangingInstantUpdated;
    modeChangeCycle_.rightChangingInstantUpdated = rightChangingInstantUpdated;
  }

  auto [leftMaintainProcess, leftInstantProcess] = leftHandSmoother_->getModeChangingState();
  auto [rightMaintainProcess, rightInstantProcess] = rightHandSmoother_->getModeChangingState();

  // 模式切换过渡分支会在 grip 边沿处理逻辑之前提前 return，因此必须在这里
  // 先锁存切换前参考；否则该分支的首帧仍可能使用已推进的姿态滤波结果。
  const bool currentLeftGripForTransfer = joyStickHandlerPtr_->isLeftGrip();
  const bool currentRightGripForTransfer = joyStickHandlerPtr_->isRightGrip();
  captureGripReleaseSnapshot(!currentLeftGripForTransfer && lastLeftGripPressed_,
                             !currentRightGripForTransfer && lastRightGripPressed_);
  latchGripTransferPose(currentLeftGripForTransfer && !lastLeftGripPressed_,
                        currentRightGripForTransfer && !lastRightGripPressed_,
                        leftMaintainProcess, rightMaintainProcess);

  // ---------------------------------------------------------------------------
  // Dual VR 躯干 ownership（torso_control_side: left / right）
  // ---------------------------------------------------------------------------
  // Dual 下机器人躯干（chest position + chest orientation / waist yaw & pitch）的
  // ownership 只属于 torso_control_side 指定的那一侧：
  //   left （默认）→ Left Grip  允许驱动躯干；Right Grip 仍完整控制右臂，
  //                  但禁止通过 Incremental chest 或 Drake whole-body optimization
  //                  改变 pChest（腰部高度）与 waist yaw/pitch
  //   right        → 对称地改为 Right Grip 驱动躯干，Left Grip 不再带动躯干
  // single_vr 下 chestPositionOwnerActive 恒为 true，且 drake 朝向不收紧，
  // 保持历史行为逐字不变。
  const bool torsoOwnerGripForTransfer =
      torsoSideRight() ? currentRightGripForTransfer : currentLeftGripForTransfer;
  const bool chestPositionOwnerActive = isDualVrMode() ? torsoOwnerGripForTransfer : true;
  // Drake chest-position 求解权限跟随 ownership：Dual 下非 owner 单独 Grip 必须关闭，
  // 否则该侧手部目标仍会经 sol.pChest 带动腰部上下。single_vr 下不做任何收紧。
  drakeSolveUpdateChestPosition_ = drakeSolveUpdateChestPosition_ && chestPositionOwnerActive;
  // Drake chest-orientation 求解权限同样跟随 ownership：Dual 下非 owner 单独 Grip
  // 不得经 sol.qChest 改变 waist yaw / waist pitch。single_vr 下保持配置值不变。
  // Dual 下再叠加 owner Chest 朝向的新鲜度：owner Quest 断连/失效时本周期禁止 Drake
  // 主动更新 Chest Orientation，保持失效前已下发的机器人 chest 朝向参考，避免拿过期
  // 四元数当目标。该标志每周期在本函数内重新计算，不做永久关闭。
  drakeSolveUpdateChestOrientation_ = drakeSolveUpdateChestOrientation_ && chestPositionOwnerActive &&
                                      (isDualVrMode() ? torsoOwnerChestFreshThisCycle_ : true);

  struct FrozenRefs {
    Eigen::Vector3d leftHandPos = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightHandPos = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftElbowPos = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightElbowPos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond leftHandQuat = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond rightHandQuat = Eigen::Quaterniond::Identity();
  };

  auto computeElbowRef = [&](const char* side,
                             bool active,
                             WheelNaturalElbowGuide* guide,
                             const Eigen::Vector3d& shoulderPos,
                             const Eigen::Vector3d& handTarget,
                             const Eigen::Vector3d& currentElbow,
                             const Eigen::Vector3d& humanShoulder,
                             const Eigen::Vector3d& humanElbow,
                             const Eigen::Vector3d& humanHand,
                             bool humanPoseValid,
                             const Eigen::Vector3d& link6Pos,
                             const Eigen::Vector3d& endEffectorPos,
                             const Eigen::Vector3d& torsoPosition,
                             const Eigen::Vector3d& torsoOutwardDirection,
                             const Eigen::Vector3d& frozenElbow,
                             double& trackingActivation) -> Eigen::Vector3d {
    if (!active) {
      trackingActivation = 1.0;
      return frozenElbow;
    }

    if (enableWheelNaturalElbowGuide_ && guide) {
      WheelNaturalElbowGuideInput guideInput;
      guideInput.shoulderPosition = shoulderPos;
      guideInput.handPosition = handTarget;
      guideInput.currentElbowPosition = currentElbow;
      guideInput.humanShoulderPosition = humanShoulder;
      guideInput.humanElbowPosition = humanElbow;
      guideInput.humanHandPosition = humanHand;
      guideInput.humanPoseValid = humanPoseValid;
      guideInput.torsoPosition = torsoPosition;
      guideInput.torsoOutwardDirection = torsoOutwardDirection;
      guideInput.torsoFrameValid = true;
      const WheelNaturalElbowGuideOutput output = guide->update(guideInput);
      trackingActivation = wheelNaturalElbowSoftTrackingScale_ * output.elbowTrackingActivation;
      // Near the torso the elbow/waist keep-out starts fighting the hand
      // target and folds the circle. Fade the soft elbow cost so the hand wins.
      const double handFromTorso = (handTarget - torsoPosition).norm();
      constexpr double kNearBodyFadeStart = 0.45;
      constexpr double kNearBodyFadeFull = 0.22;
      const double t = std::clamp(
          (handFromTorso - kNearBodyFadeFull) / (kNearBodyFadeStart - kNearBodyFadeFull), 0.0, 1.0);
      trackingActivation *= t * t * (3.0 - 2.0 * t);

      LivePathElbowState& liveState = (side[0] == 'l') ? leftLivePathElbow_ : rightLivePathElbow_;
      double liveFade = livePathElbowFadeEnable_ ? liveState.speedFade : 0.0;
      const double lateral = (handTarget - torsoPosition).dot(torsoOutwardDirection);
      constexpr double kLateralFadeStart = 0.28;
      constexpr double kLateralFadeFull = 0.48;
      if (livePathElbowFadeEnable_) {
        double lateralFade = 0.0;
        if (lateral >= kLateralFadeFull) {
          lateralFade = 1.0;
        } else if (lateral > kLateralFadeStart) {
          lateralFade = (lateral - kLateralFadeStart) / (kLateralFadeFull - kLateralFadeStart);
        }
        liveFade = std::max(liveFade, lateralFade);
      }
      if (liveFade >= liveState.fade) {
        liveState.fade = liveFade;
      } else {
        liveState.fade = std::max(liveFade, liveState.fade - 0.05);
      }
      const Eigen::Vector3d onCircleElbow = WheelNaturalElbowGuide::projectElbowToCircle(
          shoulderPos, handTarget, currentElbow, l1_, l2_);
      // Add the hand step after projection so a vertical circle can lift the
      // elbow. Projecting first used to wipe that Z carry at the top.
      const Eigen::Vector3d continuityElbow =
          onCircleElbow + liveState.fade * liveState.handDelta;
      const Eigen::Vector3d blendedElbow =
          (1.0 - liveState.fade) * output.elbowPosition + liveState.fade * continuityElbow;
      // Keep a tracking floor on the live path. Extension fade otherwise hits
      // zero at the far/top of the circle and the elbow stops following.
      trackingActivation = std::max(
          trackingActivation, wheelNaturalElbowSoftTrackingScale_ * liveState.fade);

      ROS_INFO_THROTTLE(
          1.0,
          "[WheelNaturalElbow] %s radius=%.4f m, gravity_valid=%s, human_valid=%s, "
          "human_activation=%.2f, elbow_tracking_activation=%.3f, "
          "waist_safety_activation=%.3f, waist_clearance=%.3f m, live_path_fade=%.2f, "
          "hand_reachable=%s",
          side,
          output.circleRadius,
          output.gravityDirectionValid ? "true" : "false",
          output.humanDirectionValid ? "true" : "false",
          output.humanActivation,
          trackingActivation,
          output.waistSafetyActivation,
          output.waistSignedClearance,
          liveState.fade,
          output.handTargetReachable ? "true" : "false");
      return blendedElbow;
    }

    // Compatibility fallback when the new guide is explicitly disabled.
    trackingActivation = 1.0;
    const Eigen::Vector3d eeToWristVec = link6Pos - endEffectorPos;
    const double norm = eeToWristVec.norm();
    return norm > 1.0e-6
               ? link6Pos + eeToWristVec * (l2_ / norm)
               : currentElbow;
  };

  auto buildWholeBodyInput = [&](bool leftActive, bool rightActive, FrozenRefs& frozen) -> WholeBodyRefInput {
    WholeBodyRefInput input;
    input.leftRefActive = leftActive;
    input.rightRefActive = rightActive;
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
      input.chestPosRef = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position;
    } else {
      input.chestPosRef = robotFixedWaistYawPos_;
    }
    if (!chestIncrementalUpdateEnabled_) {
      input.chestPosRef = frozenRobotChestPos_;
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
      frozen.leftHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
      frozen.leftHandQuat =
          Eigen::Quaterniond(latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
      if (!leftActive && hasLeftGripReleaseSnapshot_) {
        frozen.leftHandPos = leftGripReleaseHandPos_;
        frozen.leftHandQuat = leftGripReleaseHandQuat_;
      }
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
      frozen.rightHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
      frozen.rightHandQuat =
          Eigen::Quaterniond(latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
      if (!rightActive && hasRightGripReleaseSnapshot_) {
        frozen.rightHandPos = rightGripReleaseHandPos_;
        frozen.rightHandQuat = rightGripReleaseHandQuat_;
      }
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
      frozen.leftElbowPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
      if (!leftActive && hasLeftGripReleaseSnapshot_) {
        frozen.leftElbowPos = leftGripReleaseElbowPos_;
      }
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
      frozen.rightElbowPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
      if (!rightActive && hasRightGripReleaseSnapshot_) {
        frozen.rightElbowPos = rightGripReleaseElbowPos_;
      }
    }
    {
      // 躯干朝向来源随 torso_control_side 切换：dual 下取 owner 侧 Chest（left=A / right=B），
      // single_vr 下依旧是 /robot_chest_pose 写入的 chestRotationQuaternion_（历史行为）。
      Eigen::Matrix3d chestR = getTorsoOwnerChestQuat().toRotationMatrix();
      input.chestQuatRef = computeYawPitchOnlyQuatFromRotationMatrix(chestR);
      input.chestQuatRef = (chestFollowRecoveryOffset_ * input.chestQuatRef).normalized();
      if (!chestIncrementalUpdateEnabled_) {
        input.chestQuatRef = frozenChestQuat_;
      }
    }
    return input;
  };

  auto applyWholeBodyAndSolve = [&](WholeBodyRefInput& input,
                                    Eigen::Vector3d leftHandPos,
                                    Eigen::Quaterniond leftHandQuat,
                                    Eigen::Vector3d rightHandPos,
                                    Eigen::Quaterniond rightHandQuat,
                                    const FrozenRefs& frozen) -> bool {
    const Eigen::Vector3d chestPosForFk = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : input.chestPosRef;
    auto updateHandPoseInChest = [&](bool isActive,
                                     const Eigen::Vector3d& handFkPos,
                                     const Eigen::Quaterniond& handFkQuat,
                                     const Eigen::Vector3d& fallbackWorldPos,
                                     const Eigen::Quaterniond& fallbackWorldQuat,
                                     Eigen::Vector3d& handPosInChest,
                                     Eigen::Quaterniond& handQuatInChest,
                                     bool& hasPoseInChest) {
      if (isActive) {
        auto [handQuatChest, handPosChest] = transformPose(input.chestQuatRef, chestPosForFk, handFkQuat, handFkPos);
        handPosInChest = handPosChest;
        handQuatInChest = handQuatChest.normalized();
        hasPoseInChest = true;
        return;
      }
      if (!hasPoseInChest) {
        auto [handQuatChest, handPosChest] =
            transformPose(input.chestQuatRef, chestPosForFk, fallbackWorldQuat, fallbackWorldPos);
        handPosInChest = handPosChest;
        handQuatInChest = handQuatChest.normalized();
        hasPoseInChest = true;
      }
    };
    auto updateElbowPosInChest = [&](bool isActive,
                                     const Eigen::Vector3d& currentElbowPos,
                                     const Eigen::Vector3d& fallbackWorldPos,
                                     Eigen::Vector3d& elbowPosInChest,
                                     bool& hasElbowPosInChest) {
      if (isActive) {
        auto [elbowQuatChest, elbowPosChest] =
            transformPose(input.chestQuatRef, chestPosForFk, Eigen::Quaterniond::Identity(), currentElbowPos);
        (void)elbowQuatChest;
        elbowPosInChest = elbowPosChest;
        hasElbowPosInChest = true;
        return;
      }
      if (!hasElbowPosInChest) {
        auto [elbowQuatChest, elbowPosChest] =
            transformPose(input.chestQuatRef, chestPosForFk, Eigen::Quaterniond::Identity(), fallbackWorldPos);
        (void)elbowQuatChest;
        elbowPosInChest = elbowPosChest;
        hasElbowPosInChest = true;
      }
    };

    // 仅在手部增量控制激活时缓存手部在胸坐标下的位姿；grip 按下但未进入增量时不走此路径，避免
    // chestPosRef 与 chestPosForFk 混用导致 handTarget = handFK + offset 的正反馈上漂。
    // 【松手 freeze 不跳变】active 缓存放「上一帧约束列表值」（frozen.leftHandPos/Quat = 上一帧 solve 输出），
    // 而非 FK 实测（leftLink6Position_/leftEndEffectorQuat_）。原因：active 时 solve 的手目标 = 增量目标，
    // 而 FK 实测含滤波滞后 + 求解器软约束残差，两者有差；若缓存 FK 实测，松手瞬间 freeze 重建会跳到 FK 值
    // → input_pos 跳变。缓存上一帧 solve 输出则松手瞬间 freeze 重建 = 上一帧 solve 输出，连续
    // （等价于人形“松手后继承上一帧约束列表值”）。
    // 姿态语义仍用 link7（含第7关节手腕 pitch），与 leftEE2Link6Offset_ 匹配，避免松手 freeze 抹掉 R7 导致手腕回正。
    updateHandPoseInChest(input.leftRefActive,
                          frozen.leftHandPos,
                          frozen.leftHandQuat,
                          frozen.leftHandPos,
                          frozen.leftHandQuat,
                          leftHandPosInChest_,
                          leftHandQuatInChest_,
                          hasLeftHandPoseInChest_);
    updateHandPoseInChest(input.rightRefActive,
                          frozen.rightHandPos,
                          frozen.rightHandQuat,
                          frozen.rightHandPos,
                          frozen.rightHandQuat,
                          rightHandPosInChest_,
                          rightHandQuatInChest_,
                          hasRightHandPoseInChest_);
    updateElbowPosInChest(input.leftRefActive,
                          leftLink4Position_,
                          frozen.leftElbowPos,
                          leftElbowPosInChest_,
                          hasLeftElbowPosInChest_);
    updateElbowPosInChest(input.rightRefActive,
                          rightLink4Position_,
                          frozen.rightElbowPos,
                          rightElbowPosInChest_,
                          hasRightElbowPosInChest_);

    if (!input.leftRefActive) {
      if (hasLeftHandPoseInChest_) {
        leftHandPos = chestPosForFk + input.chestQuatRef * leftHandPosInChest_;
        leftHandQuat = (input.chestQuatRef * leftHandQuatInChest_).normalized();
      } else {
        leftHandPos = frozen.leftHandPos;
        leftHandQuat = frozen.leftHandQuat;
      }
    }
    if (!input.rightRefActive) {
      if (hasRightHandPoseInChest_) {
        rightHandPos = chestPosForFk + input.chestQuatRef * rightHandPosInChest_;
        rightHandQuat = (input.chestQuatRef * rightHandQuatInChest_).normalized();
      } else {
        rightHandPos = frozen.rightHandPos;
        rightHandQuat = frozen.rightHandQuat;
      }
    }

    // The incremental controller returns active hand targets in the world frame
    // captured at grip entry. Re-express those targets through the current robot
    // chest frame so torso translation/rotation moves the hands in world space
    // while preserving the commanded hand-to-chest relative pose.
    auto followChestForActiveHand = [&](bool isActive,
                                        bool hasChestAnchor,
                                        const Eigen::Vector3d& chestAnchorPos,
                                        const Eigen::Quaterniond& chestAnchorQuat,
                                        Eigen::Vector3d& handPos,
                                        Eigen::Quaterniond& handQuat) {
      if (!isActive || !hasChestAnchor) return;
      const Eigen::Quaterniond chestFrameDelta =
          (input.chestQuatRef.normalized() * chestAnchorQuat.normalized().conjugate()).normalized();
      const Eigen::Vector3d chestPosition =
          chestIncrementalUpdateEnabled_ ? input.chestPosRef : frozenRobotChestPos_;
      handPos = chestPosition + chestFrameDelta * (handPos - chestAnchorPos);
      handQuat = (chestFrameDelta * handQuat).normalized();
    };

    // grip 上升沿的首个周期不能把 FK 胸部锚点与当前 commanded chest 混用。
    // handleGripRisingEdge() 发生在本函数之前，此时 active chest anchor 仍可能
    // 来自 latestWaistYawFkPos_；而 followChestForActiveHand() 使用的是
    // input.chestPosRef/input.chestQuatRef。腰部控制开启时两者存在残差，
    // 会把静止手部通过胸部坐标变换成明显的 x/z 跳变。
    // 首帧直接用本周期的 commanded chest 建立锚点，使变换为单位变换；
    // 从下一帧开始再正常跟随胸部运动。
    if (leftGripTransferPending_ && input.leftRefActive) {
      leftActiveChestAnchorPos_ = input.chestPosRef;
      leftActiveChestAnchorQuat_ = input.chestQuatRef.normalized();
      hasLeftActiveChestAnchor_ = true;
    }
    if (rightGripTransferPending_ && input.rightRefActive) {
      rightActiveChestAnchorPos_ = input.chestPosRef;
      rightActiveChestAnchorQuat_ = input.chestQuatRef.normalized();
      hasRightActiveChestAnchor_ = true;
    }
    followChestForActiveHand(input.leftRefActive,
                             hasLeftActiveChestAnchor_,
                             leftActiveChestAnchorPos_,
                             leftActiveChestAnchorQuat_,
                             leftHandPos,
                             leftHandQuat);
    followChestForActiveHand(input.rightRefActive,
                             hasRightActiveChestAnchor_,
                             rightActiveChestAnchorPos_,
                             rightActiveChestAnchorQuat_,
                             rightHandPos,
                             rightHandQuat);

    // Waist/chest follow rotates the grip-world incremental target into the
    // current commanded chest.  Clip relative to measured EE only after that
    // remap; doing it beforehand puts the full waist yaw into Euler ZYX and
    // rebuilds a discontinuous command (wrist twitch at ~28 deg/frame).
    // Near pitch gimbal the safe clip keeps the command instead of reconstructing yaw/roll.
    // 总开关关闭后仍使用冻结的胸部变换，限幅必须同样放在变换之后。
    if (input.leftRefActive) {
      leftHandQuat = latestIncrementalResult_.clipHandQuatAroundMeasuredEE(true, leftHandQuat);
    }
    if (input.rightRefActive) {
      rightHandQuat = latestIncrementalResult_.clipHandQuatAroundMeasuredEE(false, rightHandQuat);
    }

    // 上面的 active/inactive 分支和胸部重映射都可能重新生成 handQuat，
    // 因此必须在最终写入 whole-body input 前再次做连续性检查。
    stabilizeGripQuaternion(true, joyStickHandlerPtr_->isLeftGrip(), leftHandQuat);
    stabilizeGripQuaternion(false, joyStickHandlerPtr_->isRightGrip(), rightHandQuat);

    // Incremental position is the commanded end-effector. Shoulder-elbow
    // geometry and point-opt still operate on link6 / wrist.
    if (input.leftRefActive) {
      leftHandPos = leftHandPos - leftHandQuat.normalized() * leftEE2Link6Offset_;
    }
    if (input.rightRefActive) {
      rightHandPos = rightHandPos - rightHandQuat.normalized() * rightEE2Link6Offset_;
    }

    updateLivePathElbowFade(input.leftRefActive, true, leftHandPos);
    updateLivePathElbowFade(input.rightRefActive, false, rightHandPos);

    // Active elbow references come from the current robot FK.  Map that point
    // from the current robot chest frame into the commanded chest frame before
    // giving it to the whole-body solver.  Otherwise the hand/shoulder targets
    // follow the commanded chest while the elbow remains in the world frame,
    // forcing the arm to articulate during a rigid torso movement.
    auto mapCurrentFkPointToChestTarget =
        [&](bool isActive, const Eigen::Vector3d& currentPoint) -> Eigen::Vector3d {
      if (!isActive || !hasLatestWaistYawFk_) return currentPoint;
      const Eigen::Quaterniond currentChestQuat = getRobotChestQuatRef().normalized();
      const Eigen::Quaterniond chestFrameDelta =
          (input.chestQuatRef.normalized() * currentChestQuat.conjugate()).normalized();
      const Eigen::Vector3d chestPosition =
          chestIncrementalUpdateEnabled_ ? input.chestPosRef : frozenRobotChestPos_;
      return chestPosition + chestFrameDelta * (currentPoint - latestWaistYawFkPos_);
    };

    const Eigen::Matrix3d chestRRef = input.chestQuatRef.normalized().toRotationMatrix();
    Eigen::Matrix3d waistSafetyR = chestRRef;
    if (hasLatestWaistYawFk_) {
      waistSafetyR = latestWaistYawFkQuat_.normalized().toRotationMatrix();
    }
    const Eigen::Vector3d vLeftShoulderInChest = robotLeftFixedShoulderPos_ - robotFixedWaistYawPos_;
    const Eigen::Vector3d vRightShoulderInChest = robotRightFixedShoulderPos_ - robotFixedWaistYawPos_;
    const Eigen::Vector3d leftShoulderRef = input.chestPosRef + chestRRef * vLeftShoulderInChest;
    const Eigen::Vector3d rightShoulderRef = input.chestPosRef + chestRRef * vRightShoulderInChest;

    input.leftHandRef = leftHandPos;
    input.rightHandRef = rightHandPos;
    input.leftHandQuat = leftHandQuat.normalized();
    input.rightHandQuat = rightHandQuat.normalized();
    if (input.leftRefActive) {
      const Eigen::Vector3d currentLeftElbowRef =
          computeElbowRef("left",
                          input.leftRefActive,
                          leftNaturalElbowGuide_.get(),
                          leftShoulderRef,
                          input.leftHandRef,
                          leftLink4Position_,
                          latestHumanLeftShoulderPos_,
                          latestHumanLeftElbowPos_,
                          latestLeftHandPose_vr_.position,
                          latestHumanLeftArmPoseValid_,
                          leftLink6Position_,
                          leftEndEffectorPosition_,
                          chestPosForFk,
                          waistSafetyR * Eigen::Vector3d::UnitY(),
                          frozen.leftElbowPos,
                          input.leftElbowTrackingActivation);
      input.leftElbowRef =
          mapCurrentFkPointToChestTarget(input.leftRefActive, currentLeftElbowRef);
    } else if (hasLeftElbowPosInChest_) {
      input.leftElbowRef = chestPosForFk + input.chestQuatRef * leftElbowPosInChest_;
      input.leftElbowTrackingActivation = 1.0;
    } else {
      input.leftElbowRef = frozen.leftElbowPos;
      input.leftElbowTrackingActivation = 1.0;
    }
    if (input.rightRefActive) {
      const Eigen::Vector3d currentRightElbowRef =
          computeElbowRef("right",
                          input.rightRefActive,
                          rightNaturalElbowGuide_.get(),
                          rightShoulderRef,
                          input.rightHandRef,
                          rightLink4Position_,
                          latestHumanRightShoulderPos_,
                          latestHumanRightElbowPos_,
                          latestRightHandPose_vr_.position,
                          latestHumanRightArmPoseValid_,
                          rightLink6Position_,
                          rightEndEffectorPosition_,
                          chestPosForFk,
                          -(waistSafetyR * Eigen::Vector3d::UnitY()),
                          frozen.rightElbowPos,
                          input.rightElbowTrackingActivation);
      input.rightElbowRef =
          mapCurrentFkPointToChestTarget(input.rightRefActive, currentRightElbowRef);
    } else if (hasRightElbowPosInChest_) {
      input.rightElbowRef = chestPosForFk + input.chestQuatRef * rightElbowPosInChest_;
      input.rightElbowTrackingActivation = 1.0;
    } else {
      input.rightElbowRef = frozen.rightElbowPos;
      input.rightElbowTrackingActivation = 1.0;
    }

    if (!updateWholeBodyConstraintList(input)) return false;
    recordTimestamp("solveIkStart", loopSyncCount_);
    solveIk();
    // 记录时间戳
    recordTimestamp("solveIkFinish", loopSyncCount_);
    return true;
  };

  // 1) 优先处理“模式切换过渡期”：由 fsmProcess 统一更新约束并求解 IK（避免与正常 grip 跟随混杂）
  if (modeChangeCycle_.leftChangingMaintainUpdated || modeChangeCycle_.rightChangingMaintainUpdated) {
    // print leftHandCtrlModeChanged and rightHandCtrlModeChanged
    // std::cout << "[fsmProcess][modeChange]:"
    //           << "leftChangingMaintainUpdated: " << modeChangeCycle_.leftChangingMaintainUpdated << ", "
    //           << "rightChangingMaintainUpdated: " << modeChangeCycle_.rightChangingMaintainUpdated << std::endl;

    if (!updateLatestIncrementalResult()) return;  // check update success

    // 任意手处于增量更新期时，同步更新 chest 位置增量（位置跟随细分关闭时不更新）。
    // Dual 下 chest position ownership 归 Left：Right-only mode change 不允许更新 chest position，
    // 但整个 mode-changing 分支（含右臂 smoother）仍然照常执行。
    if (chestIncrementalUpdateEnabled_ && chestPositionUpdateEnable_ && chestPositionOwnerActive) {
      // owner 侧 Chest 本帧无效时不推进 chest 增量：机器人 chest 位置/朝向参考保持上一帧，
      // 机械臂与 Drake 求解照常执行；恢复首帧由 advanceOwnerChestIncremental 重建锚点。
      advanceOwnerChestIncremental();
    }

    // -----------------------------
    // Whole-body reference update (chest + L/R elbow/hand) with yaw/pitch-only chest orientation.
    // - For hands not in this mode-changing cycle, freeze elbow_ref/hand_ref using the latest constraints.
    // -----------------------------
    FrozenRefs frozen;
    WholeBodyRefInput input = buildWholeBodyInput(
        modeChangeCycle_.leftChangingMaintainUpdated, modeChangeCycle_.rightChangingMaintainUpdated, frozen);

    auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
        latestIncrementalResult_.getLatestIncrementalHandPose(
            true, useIncrementalHandOrientation_, true, false);

    // 增量模块在 grip 上升沿可能已经把姿态滤波器推进到新的 VR 姿态。
    // 首帧仍使用切换前的参考，保证最终 EE 位置（尤其是旋转后的 offset）连续。
    if (leftGripTransferPending_) {
      scaledLeftHandPos = leftGripTransferHandPos_;
    }
    if (leftGripTransferPending_ || leftGripOrientationHoldFrames_ > 0) {
      incrementalLeftQuat = leftGripTransferHandQuat_;
    }
    if (rightGripTransferPending_) {
      scaledRightHandPos = rightGripTransferHandPos_;
    }
    if (rightGripTransferPending_ || rightGripOrientationHoldFrames_ > 0) {
      incrementalRightQuat = rightGripTransferHandQuat_;
    }
    // grip-world 姿态只在胸部变换之后做限幅和连续性检查，与腰部总开关无关。

    // Apply hand smoother in mode-changing cycle (it updates the position by reference).
    if (input.leftRefActive && modeChangeCycle_.leftHandCtrlModeChanged) {
      const bool isLeftArmCtrlModeActive = joyStickHandlerPtr_->isLeftArmCtrlModeActive();
      auto [leftMaintain, leftInstant] = leftHandSmoother_->getModeChangingState();
      bool leftInstantCopy = leftInstant;
      if (isLeftArmCtrlModeActive) {
        leftHandSmoother_->processActiveModeInterpolation(
            scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
      } else {
        leftHandSmoother_->processInactiveModeInterpolation(
            scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
      }
      leftHandSmoother_->setModeChangingState(leftMaintain, leftInstantCopy);
    }
    if (input.rightRefActive && modeChangeCycle_.rightHandCtrlModeChanged) {
      const bool isRightArmCtrlModeActive = joyStickHandlerPtr_->isRightArmCtrlModeActive();
      auto [rightMaintain, rightInstant] = rightHandSmoother_->getModeChangingState();
      bool rightInstantCopy = rightInstant;
      if (isRightArmCtrlModeActive) {
        rightHandSmoother_->processActiveModeInterpolation(
            scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
      } else {
        rightHandSmoother_->processInactiveModeInterpolation(
            scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
      }
      rightHandSmoother_->setModeChangingState(rightMaintain, rightInstantCopy);
    }
    if (!applyWholeBodyAndSolve(
            input, scaledLeftHandPos, incrementalLeftQuat, scaledRightHandPos, incrementalRightQuat, frozen)) {
      return;
    }

    updateLeftHandChangingMode(leftHandSmoother_->getDefaultPosOnExit());
    updateRightHandChangingMode(rightHandSmoother_->getDefaultPosOnExit());

    // 重置激活计数器，为下次 mode changing 结束后重新激活做准备
    activateAllArmCtrlModeCounter_ = 0;
    return;
  }

  // 2) 非模式切换：在 fsmProcess 中执行一次“激活全部 arm ctrl mode”的收尾动作（增强鲁棒性）
  if (activateAllArmCtrlModeCounter_ < ACTIVATE_ALL_ARM_CTRL_MODE_COUNT) {
    forceActivateAllArmCtrlMode();
    kuavo_msgs::changeArmCtrlMode srv3;
    srv3.request.control_mode = static_cast<int>(kuavo_msgs::changeArmCtrlMode::Request::ik_ultra_fast_mode);
    enableWbcArmTrajectoryControlClient_.call(srv3);
    activateAllArmCtrlModeCounter_++;

    setControlMode(3);
  }

  bool currentLeftGripPressed = joyStickHandlerPtr_->isLeftGrip();
  bool currentRightGripPressed = joyStickHandlerPtr_->isRightGrip();
  const bool isLeftActive = joyStickHandlerPtr_->isLeftArmCtrlModeActive();
  const bool isRightActive = joyStickHandlerPtr_->isRightArmCtrlModeActive();

  // 移动检测仅用于 alpha 渐变与 grip 超时，不再 gate IK 增量参考点。
  if (currentLeftGripPressed) {
    incrementalController_->detectLeftArmMove(latestLeftHandPose_vr_.position);
  }
  if (currentRightGripPressed) {
    incrementalController_->detectRightArmMove(latestRightHandPose_vr_.position);
  }

  auto updateGripTimeout = [&](bool currentGripPressed,
                               bool lastGripPressed,
                               bool armMoved,
                               std::mutex& gripTimeMutex,
                               ros::Time& gripStartTime,
                               std::atomic<bool>& gripTimeoutReached,
                               const char* logLabel) {
    ros::Time currentTime = ros::Time::now();
    if (currentGripPressed) {
      // 只有在检测到移动时才开始计时
      if (armMoved) {
        ros::Time startTime;
        {
          std::lock_guard<std::mutex> lock(gripTimeMutex);
          // 如果还没有开始计时，则开始计时
          if (gripStartTime.isZero()) {
            gripStartTime = currentTime;
            gripTimeoutReached.store(false);
          }
          startTime = gripStartTime;
        }

        // 检查是否达到超时
        if (!startTime.isZero()) {
          double elapsedTime = (currentTime - startTime).toSec();
          if (elapsedTime >= GRIP_TIMEOUT_DURATION && !gripTimeoutReached.load()) {
            gripTimeoutReached.store(true);
            ROS_INFO("[WheelQuest3IkIncrementalROS] %s grip timeout reached (%.3f seconds)", logLabel, elapsedTime);
            setLbArmQuickMode(2);
          }
        }
      } else {
        // 未检测到移动，重置时间戳（不开始计时）
        std::lock_guard<std::mutex> lock(gripTimeMutex);
        if (!gripStartTime.isZero()) {
          gripStartTime = ros::Time(0);
          gripTimeoutReached.store(false);
        }
      }
    } else {
      // grip 释放，重置时间戳和布尔值
      if (lastGripPressed) {
        std::lock_guard<std::mutex> lock(gripTimeMutex);
        gripStartTime = ros::Time(0);
        gripTimeoutReached.store(false);
      }
    }
  };

  updateGripTimeout(currentLeftGripPressed,
                    lastLeftGripPressed_,
                    incrementalController_->hasLeftArmMoved(),
                    leftGripTimeMutex_,
                    leftGripStartTime_,
                    leftGripTimeoutReached_,
                    "Left");

  updateGripTimeout(currentRightGripPressed,
                    lastRightGripPressed_,
                    incrementalController_->hasRightArmMoved(),
                    rightGripTimeMutex_,
                    rightGripStartTime_,
                    rightGripTimeoutReached_,
                    "Right");

  bool leftGripRisingEdge = currentLeftGripPressed && !lastLeftGripPressed_;
  bool rightGripRisingEdge = currentRightGripPressed && !lastRightGripPressed_;

  // 双扳机同时松开的下降沿：捕获当前 lb 关节命令快照
  // 目的：胸部增量模式激活时，松开扳机后仅允许 waist_yaw（关节4）跟随 VR 旋转，
  //       冻结 knee/leg/waist_pitch（关节1~3），避免 IK 优化目标切换引起手臂漂移上升
  const bool bothGripsJustReleased = (lastLeftGripPressed_ || lastRightGripPressed_) &&
                                     !currentLeftGripPressed && !currentRightGripPressed;
  if (bothGripsJustReleased && chestIncrementalUpdateEnabled_) {
    std::lock_guard<std::mutex> lock(ikResultMutex_);
    if (ikLowerBodyJointCommand_.size() == 4) {
      frozenLbJointCommand_ = ikLowerBodyJointCommand_;
      hasLbJointCommandFrozen_ = true;
    }
  }

  // 松手下降沿：把 freezeHeightEnabled 的高度冻结基准更新为「松手瞬间」的手/肘高度。
  // 否则 freezeHeightEnabled 在松手瞬间恒为 true，会把 leftHandRef.z()/leftElbowRef.z() 强制
  // 设为 frozenRobotChestPos_.z() + frozen*HeightOffset_，而 offset 只在 fsmExit（退出 mode2）时更新，
  // 松手时手已移动 → Z 被强制跳回退出 mode2 时的旧高度 → input_pos 阶跃（且每次松手必发生）。
  // frozen*HeightOffset_ 是相对 frozenRobotChestPos_ 的偏移，更新后 freezeHeightEnabled 目标
  // = frozenRobotChestPos_.z() + offset = 松手瞬间手/肘 Z（连续），之后每帧保持该高度（防腰 pitch 漂移）。
  if (bothGripsJustReleased) {
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
      frozenLeftHandHeightOffset_ =
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position.z() - frozenRobotChestPos_.z();
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
      frozenRightHandHeightOffset_ =
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position.z() - frozenRobotChestPos_.z();
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
      frozenLeftElbowHeightOffset_ =
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position.z() - frozenRobotChestPos_.z();
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
      frozenRightElbowHeightOffset_ =
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position.z() - frozenRobotChestPos_.z();
    }
  }

  // 更新上一帧的 grip 状态（必须在使用完之后更新）
  lastLeftGripPressed_ = currentLeftGripPressed;
  lastRightGripPressed_ = currentRightGripPressed;

  // 在更新增量锚点前保存切换前一帧真正送入 whole-body IK 的 hand 参考。
  latchGripTransferPose(leftGripRisingEdge, rightGripRisingEdge,
                        leftMaintainProcess, rightMaintainProcess);
  handleGripRisingEdge(leftGripRisingEdge, rightGripRisingEdge, leftMaintainProcess, rightMaintainProcess);

  // 由胸部更新开关控制增量更新（与 grip 解耦）；位置跟随细分关闭时冻结位置。
  // Dual：chest position 只由 torso_control_side 指定的那一侧 Grip 驱动，非 owner Grip
  //       不会带动腰部上下（left 默认 = 原有 Left ownership，与真机验证行为一致）。
  // Single：与历史行为完全一致（左右任一 Grip 均可驱动）。
  // 同一求解周期内 chest 增量只允许推进一次：下面 gripPressed 与 gripReady 两个分支的
  // 条件在同一帧可能同时成立（gripReady ⊆ gripPressed），重复调用会把 chest 位置低通
  // 滤波器多推进一个采样步、并用错误的 dt 重算 dotChestDeltaPos_。这里只做去重，
  // 两个分支各自的 gate 条件（owner 侧 / maintain / active）保持原样不变。
  bool chestIncrementalAdvancedThisCycle = false;
  const bool chestPositionGripPressed = torsoOwnerGripPressed(lastLeftGripPressed_, lastRightGripPressed_);
  if (chestIncrementalUpdateEnabled_ && chestPositionUpdateEnable_ && chestPositionGripPressed) {
    // owner 侧 Chest 本帧无效时不推进 chest 增量（保持上一帧机器人 chest 参考），
    // 机械臂与 Drake 求解照常执行；恢复首帧由 advanceOwnerChestIncremental 重建锚点。
    // 只有真正推进了才置位：本帧未推进时（owner Chest 无效）后面的 gripReady 分支
    // 仍需照常尝试，否则会出现"首个有效 Chest 帧当周期不建锚点"的行为变化。
    chestIncrementalAdvancedThisCycle = advanceOwnerChestIncremental();
  }

  const bool leftGripReady = !leftMaintainProcess && currentLeftGripPressed && isLeftActive;
  const bool rightGripReady = !rightMaintainProcess && currentRightGripPressed && isRightActive;

  if (leftGripReady) {
    latestIncrementalResult_ = incrementalController_->computeIncrementalPoseLeftArm(
        latestLeftHandPose_vr_, true, leftEndEffectorQuat_);
  }
  if (rightGripReady) {
    latestIncrementalResult_ = incrementalController_->computeIncrementalPoseRightArm(
        latestRightHandPose_vr_, true, rightEndEffectorQuat_);
  }

  // 任意手 grip 就绪时，同步更新 chest 位置增量，并写入约束列表（位置跟随细分关闭时不更新）。
  // Dual：只有 torso owner 侧 Grip 就绪才允许更新 chest position（另一侧仍完整控制自己的手臂，
  //       只是不带腰）。Single：leftGripReady || rightGripReady，与历史行为一致。
  const bool chestPositionGripReady = torsoOwnerGripReady(leftGripReady, rightGripReady);
  // 低频诊断：真机确认 Dual 下躯干 ownership 实际跟随哪一侧、
  // 以及 Drake 位置/朝向求解权限是否已随 ownership 收紧。
  logDualTorsoOwnerDiagnostics(leftGripReady, rightGripReady);
  if (chestPositionGripReady) {
    if (chestIncrementalUpdateEnabled_ && chestPositionUpdateEnable_ && !chestIncrementalAdvancedThisCycle) {
      // 同上：owner Chest 无效时整块跳过，arm 控制与 solveIk 不受影响。
      advanceOwnerChestIncremental();
    }
  }

  latestIncrementalResult_ = incrementalController_->getLatestIncrementalResult();

  // Whole-body reference update (chest + L/R elbow/hand) and then solve IK once.
  FrozenRefs frozen;
  WholeBodyRefInput input = buildWholeBodyInput(leftGripReady, rightGripReady, frozen);

  auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
      latestIncrementalResult_.getLatestIncrementalHandPose(
          true, useIncrementalHandOrientation_, true, false);

  if (leftGripTransferPending_) {
    scaledLeftHandPos = leftGripTransferHandPos_;
  }
  if (leftGripTransferPending_ || leftGripOrientationHoldFrames_ > 0) {
    incrementalLeftQuat = leftGripTransferHandQuat_;
  }
  if (rightGripTransferPending_) {
    scaledRightHandPos = rightGripTransferHandPos_;
  }
  if (rightGripTransferPending_ || rightGripOrientationHoldFrames_ > 0) {
    incrementalRightQuat = rightGripTransferHandQuat_;
  }
  // grip-world 姿态只在胸部变换之后做限幅和连续性检查，与腰部总开关无关。

  recordTimestamp("applyWholeBodyAndSolveStart", loopSyncCount_);
  applyWholeBodyAndSolve(
      input, scaledLeftHandPos, incrementalLeftQuat, scaledRightHandPos, incrementalRightQuat, frozen);
  leftGripTransferPending_ = false;
  rightGripTransferPending_ = false;
  if (leftGripOrientationHoldFrames_ > 0) {
    --leftGripOrientationHoldFrames_;
  }
  if (rightGripOrientationHoldFrames_ > 0) {
    --rightGripOrientationHoldFrames_;
  }
  if (leftGripTransferLockFrames_ > 0) {
    --leftGripTransferLockFrames_;
  }
  if (rightGripTransferLockFrames_ > 0) {
    --rightGripTransferLockFrames_;
  }
  recordTimestamp("applyWholeBodyAndSolveFinish", loopSyncCount_);
  
  // 进入mode 2约2秒后重置标志位
  if (justEnteredMode2_ && (ros::Time::now() - mode2EnterTime_).toSec() >= 2.0) {
    justEnteredMode2_ = false;
  }
}

void WheelQuest3IkIncrementalROS::handleGripRisingEdge(bool leftGripRisingEdge,
                                                  bool rightGripRisingEdge,
                                                  bool leftMaintainProcess,
                                                  bool rightMaintainProcess) {
  // 如果边沿发生在切换锁期间，latchGripTransferPose 已忽略它，这里也必须
  // 同时忽略，避免重复调用 update*ArmPoseAnchor 重置增量状态。
  if (!leftGripRisingEdge) {
    leftGripTransferAccepted_ = false;
  }
  if (!rightGripRisingEdge) {
    rightGripTransferAccepted_ = false;
  }

  // 处理左臂 grip 上升沿：更新锚点，使增量归零
  if (leftGripRisingEdge && leftGripTransferAccepted_ && !leftMaintainProcess) {
    // 【对齐人形】按下 grip 时不硬切 FK 覆盖约束列表。松手期间约束列表 hand = freeze 重建值
    // （= 上一帧 solve 输出/命令，连续），若这里用 FK 实测（leftLink6Position_）硬覆盖，
    // 会因「命令(略高) → 实际(略低，重力下电机跟踪滞后)」切换产生向下阶跃
    // （input_pos 按下瞬间跳变 + 先往下再跟手 + 手不动时自动往下漂）。
    // 人形用速度 IK 从上一帧位置平滑逼近 FK；轮臂无速度 IK，等价做法 = 保留上一帧约束列表值
    // （freeze 重建值），锚点直接继承，按下瞬间连续、随后增量从 0 跟手。
    hasLeftHandPoseInChest_ = false;
    hasLeftElbowPosInChest_ = false;

    incrementalController_->updateLeftArmPoseAnchor(latestLeftHandPose_vr_,
                                                    latestPoseConstraintList_,
                                                    leftEndEffectorPosition_,
                                                    leftEndEffectorQuat_,
                                                    leftLink4Quat_);
    leftActiveChestAnchorPos_ = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : frozenRobotChestPos_;
    leftActiveChestAnchorQuat_ = getRobotChestQuatRef().normalized();
    hasLeftActiveChestAnchor_ = true;
    leftGripTransferAccepted_ = false;
    resetLivePathElbowFade(true);
  }

  if (rightGripRisingEdge && rightGripTransferAccepted_ && !rightMaintainProcess) {
    // 【对齐人形】同上：按下 grip 不硬切 FK，保留上一帧约束列表值（freeze 重建值），锚点直接继承。
    hasRightHandPoseInChest_ = false;
    hasRightElbowPosInChest_ = false;

    incrementalController_->updateRightArmPoseAnchor(latestRightHandPose_vr_,
                                                     latestPoseConstraintList_,
                                                     rightEndEffectorPosition_,
                                                     rightEndEffectorQuat_,
                                                     rightLink4Quat_);
    rightActiveChestAnchorPos_ = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : frozenRobotChestPos_;
    rightActiveChestAnchorQuat_ = getRobotChestQuatRef().normalized();
    hasRightActiveChestAnchor_ = true;
    rightGripTransferAccepted_ = false;
    resetLivePathElbowFade(false);
  }
}

void WheelQuest3IkIncrementalROS::latchGripTransferPose(bool leftGripRisingEdge,
                                                        bool rightGripRisingEdge,
                                                        bool leftMaintainProcess,
                                                        bool rightMaintainProcess) {
  if (leftGripRisingEdge && !leftMaintainProcess &&
      latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    if (!leftGripTransferAccepted_ && leftGripTransferLockFrames_ <= 0) {
      if (hasLeftGripReleaseSnapshot_) {
        // 松开期间一直使用同一个约束快照。这里同时写回约束列表，保证
        // updateLeftArmPoseAnchor() 读取到的也是该连续值，而不是松开期间
        // 被 whole-body IK 逐步推移的旧优化结果。
        leftGripTransferHandQuat_ = leftGripReleaseHandQuat_;
        leftGripTransferHandPos_ =
            leftGripReleaseHandPos_ + leftGripTransferHandQuat_ * leftEE2Link6Offset_;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position =
            leftGripReleaseHandPos_;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix =
            leftGripReleaseHandQuat_.toRotationMatrix();
        if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position =
              leftGripReleaseElbowPos_;
        }
      } else {
        const auto& pose = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND];
        leftGripTransferHandQuat_ = Eigen::Quaterniond(pose.rotation_matrix).normalized();
        leftGripTransferHandPos_ = pose.position + leftGripTransferHandQuat_ * leftEE2Link6Offset_;
      }
      leftGripTransferPending_ = true;
      leftGripOrientationHoldFrames_ = kGripOrientationHoldFrames;
      leftGripTransferLockFrames_ = kGripTransferLockFrames;
      leftGripTransferAccepted_ = true;
    }
  }
  if (rightGripRisingEdge && !rightMaintainProcess &&
      latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    if (!rightGripTransferAccepted_ && rightGripTransferLockFrames_ <= 0) {
      if (hasRightGripReleaseSnapshot_) {
        rightGripTransferHandQuat_ = rightGripReleaseHandQuat_;
        rightGripTransferHandPos_ =
            rightGripReleaseHandPos_ + rightGripTransferHandQuat_ * rightEE2Link6Offset_;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position =
            rightGripReleaseHandPos_;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix =
            rightGripReleaseHandQuat_.toRotationMatrix();
        if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position =
              rightGripReleaseElbowPos_;
        }
      } else {
        const auto& pose = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND];
        rightGripTransferHandQuat_ = Eigen::Quaterniond(pose.rotation_matrix).normalized();
        rightGripTransferHandPos_ = pose.position + rightGripTransferHandQuat_ * rightEE2Link6Offset_;
      }
      rightGripTransferPending_ = true;
      rightGripOrientationHoldFrames_ = kGripOrientationHoldFrames;
      rightGripTransferLockFrames_ = kGripTransferLockFrames;
      rightGripTransferAccepted_ = true;
    }
  }
}

void WheelQuest3IkIncrementalROS::captureGripReleaseSnapshot(bool leftGripFallingEdge,
                                                             bool rightGripFallingEdge) {
  if (leftGripFallingEdge &&
      latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    const auto& hand = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND];
    leftGripReleaseHandPos_ = hand.position;
    leftGripReleaseHandQuat_ = Eigen::Quaterniond(hand.rotation_matrix).normalized();
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
      leftGripReleaseElbowPos_ =
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
    }
    hasLeftGripReleaseSnapshot_ = true;
  }

  if (rightGripFallingEdge &&
      latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    const auto& hand = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND];
    rightGripReleaseHandPos_ = hand.position;
    rightGripReleaseHandQuat_ = Eigen::Quaterniond(hand.rotation_matrix).normalized();
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
      rightGripReleaseElbowPos_ =
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
    }
    hasRightGripReleaseSnapshot_ = true;
  }
}

void WheelQuest3IkIncrementalROS::stabilizeGripQuaternion(bool leftArm,
                                                          bool gripPressed,
                                                          Eigen::Quaterniond& quat) {
  Eigen::Quaterniond& previousQuat = leftArm ? previousLeftGripQuat_ : previousRightGripQuat_;
  bool& hasPreviousQuat = leftArm ? hasPreviousLeftGripQuat_ : hasPreviousRightGripQuat_;

  // 松开 grip 后下一次重新按下应从新的冻结姿态重新建立基准。
  if (!gripPressed) {
    hasPreviousQuat = false;
    return;
  }

  quat.normalize();

  // Quaternion 的 q 与 -q 表示同一旋转。统一到上一帧所在半球，避免
  // input_pos 中出现数值上的 2 倍阶跃，进而误判为姿态跳变。
  if (hasPreviousQuat && previousQuat.dot(quat) < 0.0) {
    quat.coeffs() *= -1.0;
  }

  // 绝对姿态模式下，保持窗口结束后必须允许机器人直接跳到当前 VR
  // 绝对姿态；否则如果人手与机器人切入时相差超过角度阈值，旧姿态会
  // 一直被当作“上一帧”保留，表现为绝对姿态无法跟踪。
  const int holdFrames = leftArm ? leftGripOrientationHoldFrames_ : rightGripOrientationHoldFrames_;
  const bool transferActive = leftArm ? leftGripTransferPending_ : rightGripTransferPending_;
  if (!useIncrementalHandOrientation_ && !transferActive && holdFrames <= 0) {
    previousQuat = quat;
    hasPreviousQuat = true;
    return;
  }

  if (!hasPreviousQuat) {
    previousQuat = quat;
    hasPreviousQuat = true;
    return;
  }

  const double dot = std::min(1.0, std::abs(previousQuat.dot(quat)));
  const double angle = 2.0 * std::acos(dot);
  // 正常 50Hz 手部运动不会在一帧内旋转几十度；Quest 丢跟踪/姿态翻转时
  // 会出现接近 180 度的跳变。保留上一帧，等待输入回到连续范围。
  constexpr double kMaxSingleFrameRotationRad = 0.5;
  if (angle > kMaxSingleFrameRotationRad) {
    quat = previousQuat;
    return;
  }
  previousQuat = quat;
}

void WheelQuest3IkIncrementalROS::resetLivePathElbowFade(bool leftArm) {
  LivePathElbowState& state = leftArm ? leftLivePathElbow_ : rightLivePathElbow_;
  state.hasPos = false;
  state.lastPos.setZero();
  state.vel.setZero();
  state.handDelta.setZero();
  state.lastStamp = ros::Time();
  state.speedFade = 0.0;
  state.fade = 0.0;
}

void WheelQuest3IkIncrementalROS::updateLivePathElbowFade(bool active, bool leftArm,
                                                         const Eigen::Vector3d& handPos) {
  LivePathElbowState& state = leftArm ? leftLivePathElbow_ : rightLivePathElbow_;
  if (!livePathElbowFadeEnable_ || !active) {
    resetLivePathElbowFade(leftArm);
    return;
  }

  const ros::Time now = ros::Time::now();
  if (!state.hasPos) {
    state.lastPos = handPos;
    state.lastStamp = now;
    state.vel.setZero();
    state.handDelta.setZero();
    state.hasPos = true;
    state.speedFade = 0.0;
    return;
  }

  const double dt = (now - state.lastStamp).toSec();
  if (dt < 1.0e-4 || dt > 0.05) {
    state.lastPos = handPos;
    state.lastStamp = now;
    state.vel.setZero();
    state.handDelta.setZero();
    state.speedFade = 0.0;
    return;
  }

  Eigen::Vector3d handDelta = handPos - state.lastPos;
  const double deltaNorm = handDelta.norm();
  constexpr double kMaxHandDeltaM = 0.04;
  if (deltaNorm > kMaxHandDeltaM && deltaNorm > 1.0e-9) {
    handDelta *= kMaxHandDeltaM / deltaNorm;
  }
  state.handDelta = handDelta;
  const Eigen::Vector3d rawVel = handDelta / dt;
  constexpr double kVelAlpha = 0.40;
  state.vel = (1.0 - kVelAlpha) * state.vel + kVelAlpha * rawVel;
  state.lastPos = handPos;
  state.lastStamp = now;

  const double speed = state.vel.norm();
  const double vmin = livePathElbowFadeVMin_;
  const double vmax = std::max(vmin + 1.0e-3, livePathElbowFadeVMax_);
  if (speed >= vmax) {
    state.speedFade = 1.0;
  } else if (speed <= vmin) {
    state.speedFade = 0.0;
  } else {
    state.speedFade = (speed - vmin) / (vmax - vmin);
  }
}

void WheelQuest3IkIncrementalROS::applyLivePathKeepOutRelax() {
  const double fade = std::max(leftLivePathElbow_.fade, rightLivePathElbow_.fade);
  const double minP1 =
      livePathElbowKeepOutMinP1Nom_ * (1.0 - fade) + livePathElbowKeepOutMinP1_ * fade;
  const double clearance =
      livePathElbowKeepOutClearanceNom_ * (1.0 - fade) + livePathElbowKeepOutClearance_ * fade;
  if (chestElbowHandPointOptSolverPtr_) {
    DrakeChestElbowHandBoundsConfig bounds = chestElbowHandBoundsConfig_;
    bounds.minP1XyNorm = std::max(0.0, minP1);
    chestElbowHandPointOptSolverPtr_->setBounds(bounds);
  }
  if (oneStageIkEndEffectorPtr_) {
    oneStageIkEndEffectorPtr_->setWaistElbowLateralClearance(std::max(0.0, clearance));
  }
}

void WheelQuest3IkIncrementalROS::fsmExit() {
  if ((armControlMode_ == 1 && lastArmControlMode_ == 2) || (armControlMode_ == 0 && lastArmControlMode_ == 2)) {
    enterMode2ResetCounter_ = 0;
    justEnteredMode2_ = false;  // 退出mode 2时重置标志位
    mode2Initialized_ = false;  // 退出mode 2时重置初始化标志位

    if (exitMode2Counter_ < EXIT_MODE_2_EXECUTION_COUNT) {
      forceDeactivateAllArmCtrlMode();
      setLbArmQuickMode(0);
      // setControlMode(3);
      // sleep(2);
      // setControlMode(2);
      {
        std::lock_guard<std::mutex> jointLock(jointStateMutex_);
        lb_dq_.setZero();
        latest_lb_dq_.setZero();
        lowpass_lb_dq_.setZero();
      }
      {
        std::lock_guard<std::mutex> lock(lbLegMoveTimeMutex_);
        lbLegMoveStartTime_ = ros::Time(0);
      }

      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, leftLink6Position_, leftLink6Quat_);
      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, rightLink6Position_, rightLink6Quat_);

      incrementalController_->enterIncrementalModeLeftArm(latestLeftHandPose_vr_,
                                                          latestPoseConstraintList_,
                                                          leftEndEffectorPosition_,
                                                          leftEndEffectorQuat_,
                                                          leftLink4Quat_);

      incrementalController_->enterIncrementalModeRightArm(latestRightHandPose_vr_,
                                                           latestPoseConstraintList_,
                                                           rightEndEffectorPosition_,
                                                           rightEndEffectorQuat_,
                                                           rightLink4Quat_);
      exitMode2Counter_++;
    }
  }

  const bool leftGripPressed = joyStickHandlerPtr_->isLeftGrip();
  const bool rightGripPressed = joyStickHandlerPtr_->isRightGrip();
  const bool bothGripReleased = !leftGripPressed && !rightGripPressed;

  if (bothGripReleased && incrementalController_ && incrementalController_->isIncrementalModeChest()) {
    Eigen::Vector3d humanChestPos;
    // 躯干 owner 侧 Chest：dual 下真正取 A/B 实例自己的 Chest；single 下仍是 /robot_chest_pose。
    // owner Chest 无效时不用零向量做人体锚点，改为与机器人 chest 当前值同值（增量为 0），
    // 避免下次恢复时把失效期间的累计运动一次性算成增量。
    const bool hasValidOwnerChest = getTorsoOwnerHumanChestPos(humanChestPos);
    const Eigen::Vector3d chestPos = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_;
    const Eigen::Quaterniond chestQuat = getRobotChestQuatRef();
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = chestPos;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestQuat.toRotationMatrix();
      frozenRobotChestPos_ = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position;
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
        frozenLeftHandHeightOffset_ =
            latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position.z() - frozenRobotChestPos_.z();
      }
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
        frozenRightHandHeightOffset_ =
            latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position.z() - frozenRobotChestPos_.z();
      }
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
        frozenLeftElbowHeightOffset_ =
            latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position.z() - frozenRobotChestPos_.z();
      }
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
        frozenRightElbowHeightOffset_ =
            latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position.z() - frozenRobotChestPos_.z();
      }
    }
    incrementalController_->updateChestAnchorOnExit(hasValidOwnerChest ? humanChestPos : chestPos,
                                                    latestPoseConstraintList_);
    // 无效输入建立的只是占位锚点：下次拿到有效 Chest 时必须重建锚点再算增量
    chestAnchorNeedsRefresh_ = !hasValidOwnerChest;
  }

  bool shouldExitIncrementalLeftArm = incrementalController_->shouldExitIncrementalModeLeftArm(leftGripPressed);
  bool shouldExitIncrementalRightArm = incrementalController_->shouldExitIncrementalModeRightArm(rightGripPressed);

  if (shouldExitIncrementalLeftArm) {
    incrementalController_->exitIncrementalModeLeftArm(latestLeftHandPose_vr_,
                                                       latestPoseConstraintList_,
                                                       leftEndEffectorPosition_,
                                                       leftEndEffectorQuat_,
                                                       leftLink4Quat_);
    {
      std::lock_guard<std::mutex> jointLock(jointStateMutex_);
      dq_.head(7).setZero();
      latest_dq_.head(7).setZero();
      lowpass_dq_.head(7).setZero();
    }
  }

  if (shouldExitIncrementalRightArm) {
    incrementalController_->exitIncrementalModeRightArm(latestRightHandPose_vr_,
                                                        latestPoseConstraintList_,
                                                        rightEndEffectorPosition_,
                                                        rightEndEffectorQuat_,
                                                        rightLink4Quat_);
    {
      std::lock_guard<std::mutex> jointLock(jointStateMutex_);
      dq_.tail(7).setZero();
      latest_dq_.tail(7).setZero();
      lowpass_dq_.tail(7).setZero();
    }
  }
  if (!shouldExitIncrementalLeftArm && !shouldExitIncrementalRightArm) return;
  // 松 grip 仅退出增量手臂；mode 1/2 下保持 WBC/MPC 外部控制，由 deactivateController 内 guard 双重保护
  if (armControlMode_.load() != 0) return;
  deactivateController();
}

void WheelQuest3IkIncrementalROS::solveIk() {
  std::vector<PoseData> poseConstraintListCopy;
  poseConstraintListCopy = latestPoseConstraintList_;

  // 支持 rosparam 动态切换（getCached 自动感知参数服务器更新）
  {
    bool lockKneeLegParam = lockKneeLegEnabled_.load();
    if (ros::param::getCached("/ik_ros_uni_cpp_node/quest3/lock_knee_leg", lockKneeLegParam)) {
      lockKneeLegEnabled_.store(lockKneeLegParam);
    }
  }

  updateChestPositionFreezeState(!chestPositionUpdateEnable_);

  // 锁下肢前两关节（knee=q[0], leg=q[1]）到当前滤波值，只留 waist_pitch/waist_yaw 随动。
  // reset_joint_to_default:=false 时胸部增量关闭，solve 会冻结 q0-q2；若此处再用
  // 实时滤波值锁 q0/q1，会和冻结快照冲突（快照常在全零初值上捕获），SNOPT 无解、手臂失控。
  // 胸部冻结生效时必须把 lock 钉在同一组 q0/q1 上。
  if (lockKneeLegEnabled_.load()) {
    Eigen::Vector3d freezeAnchor;
    if (lowerBodyMasterFreezeActive_) {
      oneStageIkEndEffectorPtr_->setKneeLegLock(lowerBodyMasterFreezeAnchor_(0), lowerBodyMasterFreezeAnchor_(1));
    } else if (copyChestPositionFreezeAnchor(freezeAnchor)) {
      oneStageIkEndEffectorPtr_->setKneeLegLock(freezeAnchor(0), freezeAnchor(1));
    } else if (filterJointDataForDrakeFK_.size() == drakeJointStateSize_ && drakeJointStateSize_ == 18) {
      oneStageIkEndEffectorPtr_->setKneeLegLock(filterJointDataForDrakeFK_(0), filterJointDataForDrakeFK_(1));
    }
  } else {
    oneStageIkEndEffectorPtr_->disableKneeLegLock();
  }

  auto startTime = std::chrono::high_resolution_clock::now();
  oneStageIkEndEffectorPtr_->setElbowTrackingActivations(
      latestLeftElbowTrackingActivation_, latestRightElbowTrackingActivation_);
  // 总开关关闭冻结完整 q0-q3；仅位置子开关关闭冻结 q0-q2，保留 yaw 跟随。
  Eigen::Vector3d chestPositionFreezeAnchor;
  if (lowerBodyMasterFreezeActive_) {
    oneStageIkEndEffectorPtr_->activateLowerBodyFreeze(lowerBodyMasterFreezeAnchor_);
  } else if (copyChestPositionFreezeAnchor(chestPositionFreezeAnchor)) {
    oneStageIkEndEffectorPtr_->activateChestPositionFreeze(chestPositionFreezeAnchor);
  } else {
    oneStageIkEndEffectorPtr_->deactivateChestPositionFreeze();
  }
  auto ikResult = oneStageIkEndEffectorPtr_->solveIK(poseConstraintListCopy, ctrlArmIdx_, jointMidValues_);
  auto endTime = std::chrono::high_resolution_clock::now();
  const auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
  const double durationMs = static_cast<double>(durationUs) / 1000.0;
  ROS_INFO_THROTTLE(1.0, "[WheelQuest3IkIncrementalROS] one-stage IK latest duration: %.3f ms (%ld us)", durationMs, static_cast<long>(durationUs));

  if (ikResult.isSuccess) {
    {
      std::lock_guard<std::mutex> lock(ikResultMutex_);
      latestIkSolution_ = ikResult.solution;
      hasValidIkSolution_ = true;
      // 同步骨骼接收时刻到IK结果，供发布线程计算全链路延迟
      ikResultBoneRecvTime_ = ikResultBoneRecvTime_;

      // 当IK成功且size == 18时，保存前4个关节角度到ikLowerBodyJointCommand_
      if (latestIkSolution_.size() == 18) {
        // print extract ik command into lower body and upper body
        // ROS_INFO("[WheelQuest3IkIncrementalROS] extract ik command into lower body and upper body");
        ikLowerBodyJointCommand_ = latestIkSolution_.head(4);
        ikUpperBodyJointCommand_ = latestIkSolution_.tail(14);
      }
    }
    // 在IK线程内直接测量并发布延迟：骨骼接收 → FSM处理 → IK求解
    // 只在收到新VR数据的那一轮测量，避免无数据时延迟虚增（锯齿现象）
    if (armTrajLatencyPublisher_ && currentBoneSeq_ != lastProcessedBoneSeq_) {
      lastProcessedBoneSeq_ = currentBoneSeq_;
      const auto latencyMs = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - ikResultBoneRecvTime_).count();
      std_msgs::Float64 latencyMsg;
      latencyMsg.data = latencyMs;
      armTrajLatencyPublisher_.publish(latencyMsg);
    }
  } else {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] solveIk failed: %s", ikResult.solverLog.c_str());
  }
}

bool WheelQuest3IkIncrementalROS::validateVrPose(const ::ArmPose& currentPose, ::ArmPose& validatedPose, const std::string& side, bool isArmActive) {
  Eigen::Vector3d currentPos = currentPose.position;
  
  // 【关键修改】如果手臂未激活，直接通过，不进行跳变检测
  if (!isArmActive) {
    validatedPose = currentPose;
    return true;
  }
  
  // 选择对应的缓冲区和计数器
  Eigen::Vector3d* prev1 = nullptr;
  Eigen::Vector3d* prev2 = nullptr;
  int* count = nullptr;
  int* spikeCount = nullptr;
  ros::Time* spikeStartTime = nullptr;
  
  if (side == "Left") {
    prev1 = &leftHandPrev1_;
    prev2 = &leftHandPrev2_;
    count = &leftHandCount_;
    spikeCount = &leftHandSpikeCount_;
    spikeStartTime = &leftHandSpikeStartTime_;
  } else if (side == "Right") {
    prev1 = &rightHandPrev1_;
    prev2 = &rightHandPrev2_;
    count = &rightHandCount_;
    spikeCount = &rightHandSpikeCount_;
    spikeStartTime = &rightHandSpikeStartTime_;
  } else {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] Invalid side parameter: %s", side.c_str());
    validatedPose = currentPose;
    return false;
  }
  
  (*count)++;
  
  // 初始化阶段：前3个点直接通过
  if (*count < 3) {
    if (*count == 1) {
      *prev1 = currentPos;
    } else if (*count == 2) {
      *prev2 = *prev1;
      *prev1 = currentPos;
    }
    validatedPose = currentPose;
    *spikeCount = 0;  // 重置跳变计数
    return true;
  }
  
  // 核心检测逻辑：检查当前点是否异常跳变
  // 规则：如果当前点同时偏离前两点（使用欧几里得距离），且前两点相近，则认为是异常跳变
  Eigen::Vector3d diff_prev1_vec = currentPos - *prev1;
  Eigen::Vector3d diff_prev2_vec = currentPos - *prev2;
  Eigen::Vector3d diff_prev_prev_vec = *prev1 - *prev2;
  
  // 使用欧几里得距离（3D空间距离）来判断跳变
  double dist_prev1 = diff_prev1_vec.norm();
  double dist_prev2 = diff_prev2_vec.norm();
  double dist_prev_prev = diff_prev_prev_vec.norm();
  
  // 如果当前点同时偏离前两点，且前两点相近，则认为是跳变
  bool isSpike = (dist_prev1 > SPIKE_THRESHOLD && 
                  dist_prev2 > SPIKE_THRESHOLD &&
                  dist_prev_prev < SPIKE_THRESHOLD * 0.2);
  
  ros::Time currentTime = ros::Time::now();
  
  // 【超时检测】如果跳变持续超过阈值时间，强制恢复
  bool forceRecover = false;
  if (isSpike) {
    if (spikeStartTime->isZero()) {
      // 第一次检测到跳变，记录开始时间
      *spikeStartTime = currentTime;
    } else {
      // 检查是否超时
      double elapsedTime = (currentTime - *spikeStartTime).toSec();
      if (elapsedTime > SPIKE_TIMEOUT_DURATION) {
        forceRecover = true;
        ROS_WARN_THROTTLE(1.0, "[WheelQuest3IkIncrementalROS] %s hand VR pose: timeout recovery triggered (%.3f seconds)",
                          side.c_str(), elapsedTime);
      }
    }
  } else {
    // 数据正常，重置跳变开始时间
    *spikeStartTime = ros::Time(0);
  }
  
  // 恢复机制：如果连续N帧都被判定为跳变，可能是快速正常运动，应该恢复
  // 或者超时恢复机制触发
  if (isSpike && !forceRecover) {
    (*spikeCount)++;
    if (*spikeCount >= SPIKE_RECOVERY_COUNT) {
      // 连续跳变次数达到阈值，认为是快速正常运动，恢复使用当前数据
      isSpike = false;
      *spikeCount = 0;  // 重置计数
      *spikeStartTime = ros::Time(0);  // 重置时间戳
      ROS_INFO_THROTTLE(1.0, "[WheelQuest3IkIncrementalROS] %s hand VR pose: continuous spikes detected, recovering (likely fast normal motion)",
                        side.c_str());
    } else {
      // 检测到跳变，使用前一个点替代
      validatedPose.position = *prev1;
      validatedPose.quaternion = currentPose.quaternion;  // 保持当前姿态
      ROS_WARN_THROTTLE(1.0, "[WheelQuest3IkIncrementalROS] %s hand VR pose spike detected (%d/%d), using previous position",
                        side.c_str(), *spikeCount, SPIKE_RECOVERY_COUNT);
    }
  } else {
    // 数据正常，或者超时恢复触发，使用当前数据
    if (forceRecover) {
      isSpike = false;
      *spikeCount = 0;  // 重置计数
      *spikeStartTime = ros::Time(0);  // 重置时间戳
    } else {
      // 数据正常，重置跳变计数
      *spikeCount = 0;
    }
    validatedPose = currentPose;
  }
  
  // 更新缓冲区
  *prev2 = *prev1;
  *prev1 = validatedPose.position;
  
  return !isSpike;
}



HighlyDynamic::SG100VrInput WheelQuest3IkIncrementalROS::makeSg100VrInput() {
  HighlyDynamic::SG100VrInput in;
  in.fresh = [this] {
    return joyStickHandlerPtr_ && joyStickHandlerPtr_->hasFreshSg100Input();
  };
  in.left_trigger = [this] {
    return joyStickHandlerPtr_ ? static_cast<float>(joyStickHandlerPtr_->getLeftTrigger()) : 0.0f;
  };
  in.right_trigger = [this] {
    return joyStickHandlerPtr_ ? static_cast<float>(joyStickHandlerPtr_->getRightTrigger()) : 0.0f;
  };
  in.left_first_touched = [this] {
    return joyStickHandlerPtr_ && joyStickHandlerPtr_->isLeftFirstButtonTouched();
  };
  in.left_first_pressed = [this] {
    return joyStickHandlerPtr_ && joyStickHandlerPtr_->isLeftFirstButtonPressed();
  };
  in.right_first_touched = [this] {
    return joyStickHandlerPtr_ && joyStickHandlerPtr_->isRightFirstButtonTouched();
  };
  in.right_first_pressed = [this] {
    return joyStickHandlerPtr_ && joyStickHandlerPtr_->isRightFirstButtonPressed();
  };
  in.right_second_touched = [this] {
    return joyStickHandlerPtr_ && joyStickHandlerPtr_->isRightSecondButtonTouched();
  };
  in.right_second_pressed = [this] {
    return joyStickHandlerPtr_ && joyStickHandlerPtr_->isRightSecondButtonPressed();
  };
  return in;
}

}  // namespace HighlyDynamic
