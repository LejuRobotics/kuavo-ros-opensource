#include "motion_capture_ik/JoyStickHandler.h"

#include <ros/ros.h>

#include <iostream>

namespace HighlyDynamic {

JoyStickHandler::JoyStickHandler() : isInitialized_(false), endEffectorType_(EndEffectorType::LEJUCLAW) {}

void JoyStickHandler::initialize() {
  if (isInitialized_.load()) {
    std::cout << "\033[93m[JoyStickHandler] Already initialized, skipping\033[0m" << std::endl;
    return;
  }

  handPositionData_.hasValidData = false;
  clawCommandData_.hasValidData = false;

  leftJoystick_.resize(2, 0.0);
  rightJoystick_.resize(2, 0.0);
  leftFingerData_.resize(6, 0.0);
  rightFingerData_.resize(6, 0.0);
  leftHandPosition_.resize(6, 0);
  rightHandPosition_.resize(6, 0);
  clawPosition_.resize(2, 0);

  leftSecondButtonPressed_ = false;
  leftFirstButtonTouched_ = false;
  leftFirstButtonPressed_ = false;
  rightSecondButtonPressed_ = false;
  rightFirstButtonTouched_ = false;
  rightFirstButtonPressed_ = false;

  buttonYLast_ = false;
  freezeFinger_ = false;
  frozenLeftHandPosition_.resize(6, 0);
  frozenRightHandPosition_.resize(6, 0);
  frozenClawPosition_.resize(2, 0);

  controlFingerType_ = 0;

  loadHandControlParameters();

  isInitialized_.store(true);
  std::cout << "\033[92m[JoyStickHandler] Initialization completed\033[0m" << std::endl;
}

void JoyStickHandler::updateJoyStickData(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg) {
  if (!isInitialized_.load()) {
    std::cout << "\033[91m[JoyStickHandler] Not initialized, call initialize() first\033[0m" << std::endl;
    return;
  }
  // 更新手柄数据
  leftJoystick_[0] = msg->left_trigger;
  leftJoystick_[1] = msg->left_grip;
  rightJoystick_[0] = msg->right_trigger;
  rightJoystick_[1] = msg->right_grip;

  // 更新按钮状态
  leftSecondButtonPressed_ = msg->left_second_button_pressed;
  leftFirstButtonTouched_ = msg->left_first_button_touched;
  leftFirstButtonPressed_ = msg->left_first_button_pressed;
  rightSecondButtonPressed_ = msg->right_second_button_pressed;
  rightFirstButtonTouched_ = msg->right_first_button_touched;
  rightFirstButtonPressed_ = msg->right_first_button_pressed;

  // 检测Y按钮（left_second_button）的边沿触发，实现冻结功能切换
  if (leftSecondButtonPressed_ && !buttonYLast_) {
    freezeFinger_ = !freezeFinger_;
    std::cout << "\033[91mButton Y is pressed. Freeze finger: " << (freezeFinger_ ? "ON" : "OFF") << "\033[0m"
              << std::endl;
  }
  buttonYLast_ = leftSecondButtonPressed_;
}

void JoyStickHandler::processHandEndEffectorData() {
  if (!isInitialized_.load()) {
    std::cout << "\033[91m[JoyStickHandler] Not initialized, call initialize() first\033[0m" << std::endl;
    return;
  }

  EndEffectorType endEffectorType = endEffectorType_.load();
  // QIANGNAO, QIANGNAO_TOUCH, REVO2
  if (isHandEndEffectorType(endEffectorType)) {
    processHandFingerDataWithJoystick();
    handPositionData_.leftHandPosition = leftHandPosition_;
    handPositionData_.rightHandPosition = rightHandPosition_;
    handPositionData_.hasValidData = true;
    clawCommandData_.hasValidData = false;
  } else if (isClawEndEffectorType(endEffectorType)) {
    processClawDataWithJoystick();
    std::vector<double> clawPosDouble(clawPosition_.begin(), clawPosition_.end());
    clawCommandData_.positions = clawPosDouble;
    clawCommandData_.velocities = {90.0, 90.0};  // 统一为Python版本的参数配置
    clawCommandData_.efforts = {0.0, 0.0};
    clawCommandData_.hasValidData = true;
    handPositionData_.hasValidData = false;
  }
}

void JoyStickHandler::processHandEndEffectorDataWithFingerTracking() {
  if (!isInitialized_.load()) {
    std::cout << "\033[91m[JoyStickHandler] Not initialized, call initialize() first\033[0m" << std::endl;
    return;
  }

  EndEffectorType endEffectorType = endEffectorType_.load();
  // QIANGNAO, QIANGNAO_TOUCH, REVO2
  if (isHandEndEffectorType(endEffectorType)) {
    processHandFingerDataWithHandTracking();
    handPositionData_.leftHandPosition = leftHandPosition_;
    handPositionData_.rightHandPosition = rightHandPosition_;
    handPositionData_.hasValidData = true;
    clawCommandData_.hasValidData = false;
  } else if (isClawEndEffectorType(endEffectorType)) {
    processClawDataWithHandTracking();
    std::vector<double> clawPosDouble(clawPosition_.begin(), clawPosition_.end());
    clawCommandData_.positions = clawPosDouble;
    clawCommandData_.velocities = {90.0, 90.0};  // 统一为Python版本的参数配置
    clawCommandData_.efforts = {0.0, 0.0};
    clawCommandData_.hasValidData = true;
    handPositionData_.hasValidData = false;
  }
}

void JoyStickHandler::loadHandControlParameters() {
  try {
    ros::NodeHandle nh;
    // NOTES: 禁用default 没有明确指定的参数就不让启动节点
    while (!nh.hasParam("/control_finger_type") && !nh.hasParam("/end_effector_type") && ros::ok()) {
      std::cout << "\033[93m[JoyStickHandler] Waiting for required parameters...\033[0m" << std::endl;
      ros::Duration(0.1).sleep();
    }

    nh.getParam("/control_finger_type", controlFingerType_);

    std::string endEffectorTypeStr;
    nh.getParam("/end_effector_type", endEffectorTypeStr);

    if (endEffectorTypeStr != "qiangnao" && endEffectorTypeStr != "qiangnao_touch" && endEffectorTypeStr != "revo2" &&
        endEffectorTypeStr != "lejuclaw") {
      throw std::invalid_argument("Unknown end_effector_type: " + endEffectorTypeStr);
    }
    endEffectorType_ = stringToEndEffectorType(endEffectorTypeStr);
    ROS_INFO("[JoyStickHandler] End effector type: %s", endEffectorTypeStr.c_str());
  } catch (const std::exception& e) {
    std::cout << "\033[91m[JoyStickHandler] Error reading ROS parameters: " << e.what()
              << ", using default qiangnao\033[0m" << std::endl;
    endEffectorType_ = EndEffectorType::QIANGNAO;
  }
}

void JoyStickHandler::processRobotEndHandWithFingerData() {
  if (!isInitialized_.load()) {
    std::cout << "\033[91m[JoyStickHandler] Not initialized, call initialize() first\033[0m" << std::endl;
    return;
  }

  EndEffectorType endEffectorType = endEffectorType_.load();
  // QIANGNAO, QIANGNAO_TOUCH, REVO2
  if (isHandEndEffectorType(endEffectorType)) {
    processHandFingerDataWithHandTracking();
    handPositionData_.leftHandPosition = leftHandPosition_;
    handPositionData_.rightHandPosition = rightHandPosition_;
    handPositionData_.hasValidData = true;
    clawCommandData_.hasValidData = false;

  } else if (isClawEndEffectorType(endEffectorType)) {  // LEJUCLAW
    processClawDataWithHandTracking();
    std::vector<double> clawPosDouble(clawPosition_.begin(), clawPosition_.end());
    clawCommandData_.positions = clawPosDouble;
    clawCommandData_.velocities = {90.0, 90.0};  // 统一为Python版本的参数配置
    clawCommandData_.efforts = {0.0, 0.0};
    clawCommandData_.hasValidData = true;
    handPositionData_.hasValidData = false;
  }
}

HandPositionData JoyStickHandler::getHandPositionData() {
  if (!isInitialized_.load()) {
    std::cout << "\033[91m[JoyStickHandler] Not initialized, call initialize() first\033[0m" << std::endl;
    return HandPositionData();  // 返回默认的空数据
  }
  return handPositionData_;
}

ClawCommandData JoyStickHandler::getClawCommandData() {
  if (!isInitialized_.load()) {
    std::cout << "\033[91m[JoyStickHandler] Not initialized, call initialize() first\033[0m" << std::endl;
    return ClawCommandData();  // 返回默认的空数据
  }
  return clawCommandData_;
}

EndEffectorType JoyStickHandler::getEndEffectorType() const { return endEffectorType_.load(); }

double JoyStickHandler::limitValue(double value, double minVal, double maxVal) {
  return std::max(minVal, std::min(maxVal, value));
}

int JoyStickHandler::limitIntValue(int value, int minVal, int maxVal) const {
  return std::max(minVal, std::min(value, maxVal));
}

void JoyStickHandler::processHandFingerDataWithJoystick() {
  // ROS_INFO("[JoyStickHandler] Processing hand finger data with joystick");

  if (freezeFinger_) {
    // 冻结模式：使用冻结的值
    // ROS_INFO("[JoyStickHandler] Finger is frozen, using frozen values");
    leftHandPosition_ = frozenLeftHandPosition_;
    rightHandPosition_ = frozenRightHandPosition_;
  } else {
    // 非冻结模式：计算新的位置值并存储用于冻结
    for (int i = 0; i < 6; ++i) {
      int idx = (controlFingerType_ == 0) ? 6 : 2;
      if (i <= idx) {
        leftHandPosition_[i] = limitIntValue(static_cast<int>(100.0 * leftJoystick_[0]), 0, 100);
        rightHandPosition_[i] = limitIntValue(static_cast<int>(100.0 * rightJoystick_[0]), 0, 100);
      } else {
        leftHandPosition_[i] = limitIntValue(static_cast<int>(100.0 * leftJoystick_[1]), 0, 100);
        rightHandPosition_[i] = limitIntValue(static_cast<int>(100.0 * rightJoystick_[1]), 0, 100);
      }
    }

    // 处理第一个按钮的触摸状态 - 移植自Python版本
    leftHandPosition_[1] = leftFirstButtonTouched_ ? 100 : 0;
    rightHandPosition_[1] = rightFirstButtonTouched_ ? 100 : 0;

    // 存储当前值用于冻结
    frozenLeftHandPosition_ = leftHandPosition_;
    frozenRightHandPosition_ = rightHandPosition_;
  }
}

void JoyStickHandler::processHandFingerDataWithHandTracking() {
  ROS_INFO("[JoyStickHandler] Processing hand finger data with hand tracking");

  if (freezeFinger_) {
    // 冻结模式：使用冻结的值
    leftHandPosition_ = frozenLeftHandPosition_;
    rightHandPosition_ = frozenRightHandPosition_;
  } else {
    // 非冻结模式：计算新的位置值并存储用于冻结
    for (int i = 0; i < 6; ++i) {
      leftHandPosition_[i] = limitIntValue(static_cast<int>(100.0 * leftFingerData_[i] / 1.70), 0, 100);
      rightHandPosition_[i] = limitIntValue(static_cast<int>(100.0 * rightFingerData_[i] / 1.70), 0, 100);
    }

    // 存储当前值用于冻结
    frozenLeftHandPosition_ = leftHandPosition_;
    frozenRightHandPosition_ = rightHandPosition_;
  }
}

void JoyStickHandler::processClawDataWithJoystick() {
  if (freezeFinger_) {
    // 冻结模式：使用冻结的值
    clawPosition_ = frozenClawPosition_;
  } else {
    // 非冻结模式：计算新的位置值并存储用于冻结
    clawPosition_[0] = limitIntValue(static_cast<int>(100.0 * leftJoystick_[0]), 0, 100);
    clawPosition_[1] = limitIntValue(static_cast<int>(100.0 * rightJoystick_[0]), 0, 100);

    // 存储当前值用于冻结
    frozenClawPosition_ = clawPosition_;
  }
}

void JoyStickHandler::processClawDataWithHandTracking() {
  if (freezeFinger_) {
    // 冻结模式：使用冻结的值
    clawPosition_ = frozenClawPosition_;
  } else {
    // 非冻结模式：计算新的位置值并存储用于冻结
    clawPosition_[0] = limitIntValue(static_cast<int>(100.0 * leftFingerData_[2] / 1.70), 0, 100);
    clawPosition_[1] = limitIntValue(static_cast<int>(100.0 * rightFingerData_[2] / 1.70), 0, 100);

    // 存储当前值用于冻结
    frozenClawPosition_ = clawPosition_;
  }
}

}  // namespace HighlyDynamic
