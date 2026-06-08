#include "motion_capture_ik/JoyStickHandler.h"

#include <ros/ros.h>

#include <cmath>
#include <iostream>

namespace HighlyDynamic {

JoyStickHandler::JoyStickHandler(double threshold, double alpha)
    : isInitialized_(false),
      endEffectorType_(EndEffectorType::LEJUCLAW),
      joyStickThreshold_(threshold),
      joyStickAlpha_(alpha) {}

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

  leftJoyStickX_ = 0.0;
  leftJoyStickY_ = 0.0;
  rightJoyStickX_ = 0.0;
  rightJoyStickY_ = 0.0;

  RightJoyStickYHold_ = true;
  rightJoyStickYHoldCount_ = 0;

  leftGrip_ = false;
  rightGrip_ = false;
  leftSecondButtonPressed_ = false;
  leftSecondButtonTouched_ = false;
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

  // 初始化手臂控制模式相关变量
  leftArmCtrlModeActive_ = false;
  rightArmCtrlModeActive_ = false;
  leftButtonXLastPressed_ = false;
  rightButtonALastPressed_ = false;
  leftButtonXFirstPressTime_ = ros::Time(0);
  rightButtonAFirstPressTime_ = ros::Time(0);
  lastArmCtrlModeChangeTime_ = ros::Time(0);

  // 初始化状态变化查询相关变量
  lastQueryLeftArmCtrlModeActive_ = false;
  lastQueryRightArmCtrlModeActive_ = false;

  controlFingerType_ = 0;

  loadHandControlParameters();

  isInitialized_.store(true);
  std::cout << "\033[92m[JoyStickHandler] Initialization completed\033[0m" << std::endl;
}

void JoyStickHandler::reset() {
  std::lock_guard<std::mutex> lock(dataMutex_);

  handPositionData_.hasValidData = false;
  clawCommandData_.hasValidData = false;

  leftJoystick_.resize(2, 0.0);
  rightJoystick_.resize(2, 0.0);
  leftFingerData_.resize(6, 0.0);
  rightFingerData_.resize(6, 0.0);
  leftHandPosition_.resize(6, 0);
  rightHandPosition_.resize(6, 0);
  clawPosition_.resize(2, 0);

  leftJoyStickX_ = 0.0;
  leftJoyStickY_ = 0.0;
  rightJoyStickX_ = 0.0;
  rightJoyStickY_ = 0.0;

  RightJoyStickYHold_ = true;
  rightJoyStickYHoldCount_ = 0;

  leftGrip_ = false;
  rightGrip_ = false;
  leftSecondButtonPressed_ = false;
  leftSecondButtonTouched_ = false;
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

  // 重置手臂控制模式相关变量
  leftArmCtrlModeActive_ = false;
  rightArmCtrlModeActive_ = false;
  leftButtonXLastPressed_ = false;
  rightButtonALastPressed_ = false;
  leftButtonXFirstPressTime_ = ros::Time(0);
  rightButtonAFirstPressTime_ = ros::Time(0);
  lastArmCtrlModeChangeTime_ = ros::Time(0);

  // 重置状态变化查询相关变量
  lastQueryLeftArmCtrlModeActive_ = false;
  lastQueryRightArmCtrlModeActive_ = false;

  // 重置控制手指类型（与initialize()中一致）
  controlFingerType_ = 0;
}

double JoyStickHandler::getLeftJoyStickX() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  // print leftJoyStickX_
  // std::cout << "\033[92m[JoyStickHandler] leftJoyStickX_: " << leftJoyStickX_ << "\033[0m" << std::endl;
  return leftJoyStickX_;
}

double JoyStickHandler::getLeftJoyStickY() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  // std::cout << "\033[92m[JoyStickHandler] leftJoyStickY_: " << leftJoyStickY_ << "\033[0m" << std::endl;
  return leftJoyStickY_;
}

double JoyStickHandler::getRightJoyStickX() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  // std::cout << "\033[92m[JoyStickHandler] rightJoyStickX_: " << rightJoyStickX_ << "\033[0m" << std::endl;
  return rightJoyStickX_;
}

double JoyStickHandler::getRightJoyStickY() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  // std::cout << "\033[92m[JoyStickHandler] rightJoyStickY_: " << rightJoyStickY_ << "\033[0m" << std::endl;
  return rightJoyStickY_;
}

bool JoyStickHandler::getRightJoyStickYHold() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return RightJoyStickYHold_;
}

void JoyStickHandler::updateJoyStickData(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(dataMutex_);
  if (!isInitialized_.load()) {
    std::cout << "\033[91m[JoyStickHandler] Not initialized, call initialize() first\033[0m" << std::endl;
    return;
  }

  double rightJoyStickYValue = msg->right_y;
  bool isCounting = false;

  if (!RightJoyStickYHold_) {
    if (rightJoyStickYValue > 0.8) {
      // print count
      ROS_INFO_STREAM_THROTTLE(
          0.5, "\033[92m[JoyStickHandler] rightJoyStickYHoldCount_: " << rightJoyStickYHoldCount_ << "\033[0m");
      rightJoyStickYHoldCount_++;
      isCounting = true;
      if (rightJoyStickYHoldCount_ > 100) {
        RightJoyStickYHold_ = true;
        rightJoyStickYHoldCount_ = 0;
        std::cout << "\033[92m[JoyStickHandler] RightJoyStickYHold_ 状态切换为: true\033[0m" << std::endl;
      }
    } else {
      rightJoyStickYHoldCount_ = 0;
    }
  } else {
    if (rightJoyStickYValue < -0.8) {
      ROS_INFO_STREAM_THROTTLE(
          0.5, "\033[92m[JoyStickHandler] rightJoyStickYHoldCount_: " << rightJoyStickYHoldCount_ << "\033[0m");
      rightJoyStickYHoldCount_++;
      isCounting = true;
      if (rightJoyStickYHoldCount_ > 100) {
        RightJoyStickYHold_ = false;
        rightJoyStickYHoldCount_ = 0;
        std::cout << "\033[92m[JoyStickHandler] RightJoyStickYHold_ 状态切换为: false\033[0m" << std::endl;
      }
    } else {
      rightJoyStickYHoldCount_ = 0;
    }
  }

  if (std::abs(rightJoyStickYValue) > 0.8) {
    leftJoyStickX_ = 0.0;
    leftJoyStickY_ = 0.0;
    rightJoyStickX_ = 0.0;
    // 对 rightJoyStickY_ 应用阈值判断和低通滤波
    if (std::abs(rightJoyStickYValue) > joyStickThreshold_) {
      rightJoyStickY_ = joyStickAlpha_ * rightJoyStickY_ + (1.0 - joyStickAlpha_) * rightJoyStickYValue;
    } else {
      rightJoyStickY_ = 0.0;
    }
  } else {
    // 对 leftJoyStickX_ 应用阈值判断和低通滤波
    if (std::abs(msg->left_x) > joyStickThreshold_) {
      leftJoyStickX_ = joyStickAlpha_ * leftJoyStickX_ + (1.0 - joyStickAlpha_) * msg->left_x;
    } else {
      leftJoyStickX_ = 0.0;
    }
    // 对 leftJoyStickY_ 应用阈值判断和低通滤波
    if (std::abs(msg->left_y) > joyStickThreshold_) {
      leftJoyStickY_ = joyStickAlpha_ * leftJoyStickY_ + (1.0 - joyStickAlpha_) * msg->left_y;
    } else {
      leftJoyStickY_ = 0.0;
    }
    // 对 rightJoyStickX_ 应用阈值判断和低通滤波
    if (std::abs(msg->right_x) > joyStickThreshold_) {
      rightJoyStickX_ = joyStickAlpha_ * rightJoyStickX_ + (1.0 - joyStickAlpha_) * msg->right_x;
    } else {
      rightJoyStickX_ = 0.0;
    }
    // 对 rightJoyStickY_ 应用阈值判断和低通滤波
    if (std::abs(rightJoyStickYValue) > joyStickThreshold_) {
      rightJoyStickY_ = joyStickAlpha_ * rightJoyStickY_ + (1.0 - joyStickAlpha_) * rightJoyStickYValue;
    } else {
      rightJoyStickY_ = 0.0;
    }
  }
  leftJoystick_[0] = msg->left_trigger;
  leftJoystick_[1] = msg->left_grip;
  rightJoystick_[0] = msg->right_trigger;
  rightJoystick_[1] = msg->right_grip;

  // 存储摇杆坐标用于腰部控制
  leftStickX_ = msg->left_x;
  leftStickY_ = msg->left_y;
  rightStickX_ = msg->right_x;
  rightStickY_ = msg->right_y;

  leftGrip_ = msg->left_grip > 0.75;
  rightGrip_ = msg->right_grip > 0.75;

  // 更新按钮状态
  leftSecondButtonPressed_ = msg->left_second_button_pressed;
  leftSecondButtonTouched_ = msg->left_second_button_touched;
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

  // X键双击检测（左手控制模式切换）
  if (leftFirstButtonPressed_ && !leftButtonXLastPressed_) {
    // 检测到按下边沿
    ros::Time now = ros::Time::now();
    // 修复：检查时间戳是否在未来（冷却期），如果是则忽略这次按下
    if (!leftButtonXFirstPressTime_.isZero() && now < leftButtonXFirstPressTime_) {
      // 在冷却期内，忽略这次按下，但仍需更新 lastPressed 状态
      leftButtonXLastPressed_ = leftFirstButtonPressed_;
    } else if (leftButtonXFirstPressTime_.isZero() ||
               (now - leftButtonXFirstPressTime_).toSec() > DOUBLE_CLICK_TIMEOUT) {
      // 第一次按下或超时，记录时间戳
      leftButtonXFirstPressTime_ = now;
      leftButtonXLastPressed_ = leftFirstButtonPressed_;
    } else {
      // 在超时时间内第二次按下，检查是否允许切换模式
      // 检查任意手是否在5秒内有过模式变化
      bool anyHandRecentlyChanged = !lastArmCtrlModeChangeTime_.isZero() &&
                                    (now - lastArmCtrlModeChangeTime_).toSec() < MODE_CHANGE_BLOCK_DURATION;

      if (anyHandRecentlyChanged) {
        // 任意手最近有变化，阻止左手模式切换
        std::cout << "\033[93m[JoyStickHandler] 左手控制模式切换被阻止：在 "
                  << (now - lastArmCtrlModeChangeTime_).toSec() << " 秒前发生了模式变化\033[0m" << std::endl;
        // 重置双击检测状态
        leftButtonXFirstPressTime_ = now + ros::Duration(DOUBLE_CLICK_TIMEOUT);
        leftButtonXLastPressed_ = leftFirstButtonPressed_;
      } else {
        // 允许切换，触发双击
        leftArmCtrlModeActive_ = !leftArmCtrlModeActive_;
        lastArmCtrlModeChangeTime_ = now;  // 记录统一模式变化时间
        // 修复：使用一个未来的时间戳，防止立即被当作新的第一次按下
        // 这样可以避免快速三次点击触发两次双击的问题
        leftButtonXFirstPressTime_ = now + ros::Duration(DOUBLE_CLICK_TIMEOUT);
        leftButtonXLastPressed_ = leftFirstButtonPressed_;
        std::cout << "\033[92mX键双击检测: 左手控制模式 " << (leftArmCtrlModeActive_ ? "激活" : "停用") << "\033[0m"
                  << std::endl;
      }
    }
  } else {
    leftButtonXLastPressed_ = leftFirstButtonPressed_;
  }

  // A键双击检测（右手控制模式切换）
  if (rightFirstButtonPressed_ && !rightButtonALastPressed_) {
    // 检测到按下边沿
    ros::Time now = ros::Time::now();
    // 修复：检查时间戳是否在未来（冷却期），如果是则忽略这次按下
    if (!rightButtonAFirstPressTime_.isZero() && now < rightButtonAFirstPressTime_) {
      // 在冷却期内，忽略这次按下，但仍需更新 lastPressed 状态
      rightButtonALastPressed_ = rightFirstButtonPressed_;
    } else if (rightButtonAFirstPressTime_.isZero() ||
               (now - rightButtonAFirstPressTime_).toSec() > DOUBLE_CLICK_TIMEOUT) {
      // 第一次按下或超时，记录时间戳
      rightButtonAFirstPressTime_ = now;
      rightButtonALastPressed_ = rightFirstButtonPressed_;
    } else {
      // 在超时时间内第二次按下，检查是否允许切换模式
      // 检查任意手是否在5秒内有过模式变化
      bool anyHandRecentlyChanged = !lastArmCtrlModeChangeTime_.isZero() &&
                                    (now - lastArmCtrlModeChangeTime_).toSec() < MODE_CHANGE_BLOCK_DURATION;

      if (anyHandRecentlyChanged) {
        // 任意手最近有变化，阻止右手模式切换
        std::cout << "\033[93m[JoyStickHandler] 右手控制模式切换被阻止：在 "
                  << (now - lastArmCtrlModeChangeTime_).toSec() << " 秒前发生了模式变化\033[0m" << std::endl;
        // 重置双击检测状态
        rightButtonAFirstPressTime_ = now + ros::Duration(DOUBLE_CLICK_TIMEOUT);
        rightButtonALastPressed_ = rightFirstButtonPressed_;
      } else {
        // 允许切换，触发双击
        rightArmCtrlModeActive_ = !rightArmCtrlModeActive_;
        lastArmCtrlModeChangeTime_ = now;  // 记录统一模式变化时间
        // 修复：使用一个未来的时间戳，防止立即被当作新的第一次按下
        // 这样可以避免快速三次点击触发两次双击的问题
        rightButtonAFirstPressTime_ = now + ros::Duration(DOUBLE_CLICK_TIMEOUT);
        rightButtonALastPressed_ = rightFirstButtonPressed_;
        std::cout << "\033[92mA键双击检测: 右手控制模式 " << (rightArmCtrlModeActive_ ? "激活" : "停用") << "\033[0m"
                  << std::endl;
      }
    }
  } else {
    rightButtonALastPressed_ = rightFirstButtonPressed_;
  }
}

void JoyStickHandler::processHandEndEffectorData() {
  std::lock_guard<std::mutex> lock(dataMutex_);
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
  std::lock_guard<std::mutex> lock(dataMutex_);
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
  std::lock_guard<std::mutex> lock(dataMutex_);
  if (!isInitialized_.load()) {
    std::cout << "\033[91m[JoyStickHandler] Not initialized, call initialize() first\033[0m" << std::endl;
    return HandPositionData();  // 返回默认的空数据
  }
  return handPositionData_;
}

ClawCommandData JoyStickHandler::getClawCommandData() {
  std::lock_guard<std::mutex> lock(dataMutex_);
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

bool JoyStickHandler::hasLeftArmCtrlModeChanged() {
  std::lock_guard<std::mutex> lock(dataMutex_);
  // 比较当前左手状态和上次查询状态
  bool changed = (leftArmCtrlModeActive_ != lastQueryLeftArmCtrlModeActive_);

  // 如果有变化，更新上次查询状态并返回true
  if (changed) {
    lastQueryLeftArmCtrlModeActive_ = leftArmCtrlModeActive_;
    return true;
  }

  // 状态未变化，返回false
  return false;
}

bool JoyStickHandler::hasRightArmCtrlModeChanged() {
  std::lock_guard<std::mutex> lock(dataMutex_);
  // 比较当前右手状态和上次查询状态
  bool changed = (rightArmCtrlModeActive_ != lastQueryRightArmCtrlModeActive_);

  // 如果有变化，更新上次查询状态并返回true
  if (changed) {
    lastQueryRightArmCtrlModeActive_ = rightArmCtrlModeActive_;
    return true;
  }

  // 状态未变化，返回false
  return false;
}

bool JoyStickHandler::isLeftFirstButtonPressed() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return leftFirstButtonPressed_;
}

bool JoyStickHandler::isRightFirstButtonPressed() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return rightFirstButtonPressed_;
}

bool JoyStickHandler::isLeftSecondButtonPressed() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return leftSecondButtonPressed_;
}

bool JoyStickHandler::isRightSecondButtonPressed() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return rightSecondButtonPressed_;
}

bool JoyStickHandler::isLeftFirstButtonTouched() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return leftFirstButtonTouched_;
}

bool JoyStickHandler::isLeftSecondButtonTouched() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return leftSecondButtonTouched_;
}

bool JoyStickHandler::isLeftRightFirstButtonTouched() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return leftFirstButtonTouched_ && rightFirstButtonTouched_;
}

bool JoyStickHandler::isLeftRightFirstButtonPressed() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return leftFirstButtonPressed_ && rightFirstButtonPressed_;
}

bool JoyStickHandler::isLeftGrip() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return leftGrip_;
}

bool JoyStickHandler::isRightGrip() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return rightGrip_;
}

bool JoyStickHandler::isLeftRightGrip() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return leftGrip_ && rightGrip_;
}

bool JoyStickHandler::isLeftArmCtrlModeActive() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return leftArmCtrlModeActive_;
}

bool JoyStickHandler::isRightArmCtrlModeActive() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return rightArmCtrlModeActive_;
}

void JoyStickHandler::forceSetLeftArmCtrlMode(bool active) {
  std::lock_guard<std::mutex> lock(dataMutex_);
  // 强制设置状态，并更新上次查询状态以确保 hasLeftArmCtrlModeChanged() 能检测到变化
  if (leftArmCtrlModeActive_ != active) {
    leftArmCtrlModeActive_ = active;
    // 重置上次查询状态，确保下次调用 hasLeftArmCtrlModeChanged() 时返回 true
    lastQueryLeftArmCtrlModeActive_ = !active;
    // 更新模式变化时间戳，设置5秒超时保护，防止其他按键切换
    lastArmCtrlModeChangeTime_ = ros::Time::now();
    // std::cout << "\033[92m[JoyStickHandler] 强制设置左手控制模式: " << (active ? "激活" : "停用")
    //           << "，已设置5秒超时保护\033[0m" << std::endl;
  }
}

void JoyStickHandler::forceSetRightArmCtrlMode(bool active) {
  std::lock_guard<std::mutex> lock(dataMutex_);
  // 强制设置状态，并更新上次查询状态以确保 hasRightArmCtrlModeChanged() 能检测到变化
  if (rightArmCtrlModeActive_ != active) {
    rightArmCtrlModeActive_ = active;
    // 重置上次查询状态，确保下次调用 hasRightArmCtrlModeChanged() 时返回 true
    lastQueryRightArmCtrlModeActive_ = !active;
    // 更新模式变化时间戳，设置5秒超时保护，防止其他按键切换
    lastArmCtrlModeChangeTime_ = ros::Time::now();
    // std::cout << "\033[92m[JoyStickHandler] 强制设置右手控制模式: " << (active ? "激活" : "停用")
    //           << "，已设置5秒超时保护\033[0m" << std::endl;
  }
}

}  // namespace HighlyDynamic
