#include "motion_capture_ik/WheelHandSmoother.h"
#include "motion_capture_ik/BaseIKSolver.h"
#include "motion_capture_ik/WheelOneStageIKEndEffector.h"
#include <leju_utils/math.hpp>
#include <ros/ros.h>

namespace HighlyDynamic {

WheelHandSmoother::WheelHandSmoother(const std::string& handName,
                           const std::string& fkFrameName,
                           const Eigen::Vector3d& defaultPosOnExit)
    : handName_(handName),
      fkFrameName_(fkFrameName),
      defaultPosOnExit_(defaultPosOnExit),
      ctrlModeChangingMaintain_(false),
      ctrlModeChangingInstant_(false) {
  // 初始化平滑插值状态
  smootheIntermidiatePos_ = defaultPosOnExit;
  dotSmootheIntermidiatePos_.setZero();
}

void WheelHandSmoother::resetSmoothState(const Eigen::Vector3d& currentPos) {
  smootheIntermidiatePos_ = currentPos;
  dotSmootheIntermidiatePos_.setZero();
}

void WheelHandSmoother::setSmoothState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) {
  smootheIntermidiatePos_ = pos;
  dotSmootheIntermidiatePos_ = vel;
}

void WheelHandSmoother::setModeChangingState(bool changingMaintain, bool changingInstant) {
  ctrlModeChangingMaintain_ = changingMaintain;
  ctrlModeChangingInstant_ = changingInstant;
}

void WheelHandSmoother::reset() {
  ctrlModeChangingMaintain_ = false;
  ctrlModeChangingInstant_ = false;
  // 重置平滑插值状态为构造函数中的初始值
  smootheIntermidiatePos_ = defaultPosOnExit_;
  dotSmootheIntermidiatePos_.setZero();
}

std::pair<bool, bool> WheelHandSmoother::getModeChangingState() const {
  return {ctrlModeChangingMaintain_, ctrlModeChangingInstant_};
}

std::pair<bool, bool> WheelHandSmoother::updateModeChangingStateIfNeeded(bool ctrlModeChanged) {
  if (!ctrlModeChangingMaintain_) {
    setModeChangingState(ctrlModeChanged, ctrlModeChanged);
  }
  return getModeChangingState();
}

void WheelHandSmoother::processActiveModeInterpolation(Eigen::Vector3d& scaledHandPos,
                                                  bool& isChangingInstant,
                                                  const Eigen::Vector3d& defaultPos,
                                                  const std::string& handName) {
  if (isChangingInstant) {
    smootheIntermidiatePos_ = scaledHandPos;  // 总是从当前位置开始，区别在于插值目标点不一致
    dotSmootheIntermidiatePos_.setZero();
    isChangingInstant = false;

    // std::cout << "[WheelQuest3IkIncrementalROS] \033[32m" << handName
    //           << "启动中\033[0m，已更新锚点，保持当前位置等待增量计算" << std::endl;
  }
  scaledHandPos = defaultPos;
}

void WheelHandSmoother::processInactiveModeInterpolation(Eigen::Vector3d& scaledHandPos,
                                                    bool& isChangingInstant,
                                                    const Eigen::Vector3d& defaultPos,
                                                    const std::string& handName) {
  // 从抬手状态，恢复到初始位置
  if (isChangingInstant) {
    smootheIntermidiatePos_ = scaledHandPos;  // 插值应该从当前位置开始，直到接近默认值
    dotSmootheIntermidiatePos_.setZero();
    isChangingInstant = false;
    // 用红色打印关闭信息
    // std::cout << "[WheelQuest3IkIncrementalROS] \033[31m" << handName << "关闭中\033[0m，从当前位置，恢复到默认位置"
    //           << std::endl;
  }

  for (int i = 0; i < 3; i++) {
    leju_utils::fhanStepForward(
        smootheIntermidiatePos_(i), dotSmootheIntermidiatePos_(i), defaultPos(i), 0.3, 0.01, 0.04);
  }
  scaledHandPos = smootheIntermidiatePos_;
}

bool WheelHandSmoother::updateChangingMode(const Eigen::Vector3d& targetPos,
                                      BaseIKSolver* ikSolverPtr,
                                      const Eigen::VectorXd& armJoints,
                                      int jointStateSize,
                                      double threshold) {
  (void)jointStateSize;

  if (!ctrlModeChangingMaintain_) {
    return false;
  }

  // wheel_ik_ros_uni_cpp_node uses WheelOneStageIKEndEffector as the only IK solver.
  auto* oneStageIk = dynamic_cast<WheelOneStageIKEndEffector*>(ikSolverPtr);
  if (oneStageIk) {
    auto [posReached, quatTmp] = oneStageIk->FK(armJoints, fkFrameName_);
    double err = (posReached - targetPos).norm();
    if (err < threshold) {
      if (ctrlModeChangingMaintain_) {
        // ROS_INFO("[WheelQuest3IkIncrementalROS] %s target pose reached (err=%.3f), exit ctrlModeChanging",
        //          handName_.c_str(),
        //          err);
      }
      ctrlModeChangingMaintain_ = false;
      return true;
    } else {
      // std::cout << handName_ << " err: " << err << std::endl;
      // std::cout << handName_ << " posReached: " << posReached.transpose() << std::endl;
      // std::cout << handName_ << " targetPos: " << targetPos.transpose() << std::endl;
      return false;
    }
  }

  ROS_WARN("[WheelHandSmoother] Unknown IK solver type");
  return false;
}

void WheelHandSmoother::updateWithFhan(const Eigen::Vector3d& targetPos, double r, double h, double h0) {
  for (int i = 0; i < 3; i++) {
    leju_utils::fhanStepForward(smootheIntermidiatePos_(i), dotSmootheIntermidiatePos_(i), targetPos(i), r, h, h0);
  }
}

}  // namespace HighlyDynamic
