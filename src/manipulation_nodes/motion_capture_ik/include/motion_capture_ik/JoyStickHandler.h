#pragma once

#include <noitom_hi5_hand_udp_python/JoySticks.h>

#include <atomic>
#include <leju_utils/define.hpp>
#include <vector>

namespace HighlyDynamic {

// 手部位置数据结构
struct HandPositionData {
  std::vector<int> leftHandPosition;
  std::vector<int> rightHandPosition;
  bool hasValidData = false;
};

// 夹爪命令数据结构
struct ClawCommandData {
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  bool hasValidData = false;
};

class JoyStickHandler {
 public:
  JoyStickHandler();

  void initialize();

  void updateJoyStickData(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg);
  void processHandEndEffectorData();
  void processHandEndEffectorDataWithFingerTracking();

  HandPositionData getHandPositionData();
  ClawCommandData getClawCommandData();
  EndEffectorType getEndEffectorType() const;

  bool isLeftFirstButtonTouched() const { return leftFirstButtonTouched_; }
  bool isRightFirstButtonTouched() const { return rightFirstButtonTouched_; }
  bool isLeftFirstButtonPressed() const { return leftFirstButtonPressed_; }
  bool isRightFirstButtonPressed() const { return rightFirstButtonPressed_; }
  bool isLeftSecondButtonPressed() const { return leftSecondButtonPressed_; }
  bool isRightSecondButtonPressed() const { return rightSecondButtonPressed_; }

  bool isLeftRightFirstButtonTouched() const { return leftFirstButtonTouched_ && rightFirstButtonTouched_; }
  bool isLeftRightFirstButtonPressed() const { return leftFirstButtonPressed_ && rightFirstButtonPressed_; }

 private:
  void processRobotEndHandWithFingerData();

  double limitValue(double value, double minVal, double maxVal);

  void processHandFingerDataWithJoystick();
  void processHandFingerDataWithHandTracking();
  void processClawDataWithJoystick();
  void processClawDataWithHandTracking();

  std::atomic<bool> isInitialized_;

  // 按钮状态
  bool leftSecondButtonPressed_;
  bool leftFirstButtonTouched_;
  bool leftFirstButtonPressed_;  // 左手第一个按键按下状态
  bool rightSecondButtonPressed_;
  bool rightFirstButtonTouched_;
  bool rightFirstButtonPressed_;  // 右手第一个按键按下状态

  // 冻结功能相关变量
  bool buttonYLast_;                          // 上一次Y按钮状态，用于边沿检测
  bool freezeFinger_;                         // 冻结状态标志
  std::vector<int> frozenLeftHandPosition_;   // 冻结的左手位置
  std::vector<int> frozenRightHandPosition_;  // 冻结的右手位置
  std::vector<int> frozenClawPosition_;       // 冻结的夹爪位置

  std::atomic<EndEffectorType> endEffectorType_;
  int controlFingerType_;

  std::vector<double> leftJoystick_;     // [left_trigger, left_grip]
  std::vector<double> rightJoystick_;    // [right_trigger, right_grip]
  std::vector<double> leftFingerData_;   // 左手手指关节数据
  std::vector<double> rightFingerData_;  // 右手手指关节数据

  // 处理后的手指位置数据
  std::vector<int> leftHandPosition_;   // 左手位置 [0-100]
  std::vector<int> rightHandPosition_;  // 右手位置 [0-100]
  std::vector<int> clawPosition_;       // 爪子位置 [0-100]

  HandPositionData handPositionData_;
  ClawCommandData clawCommandData_;

  void loadHandControlParameters();
  int limitIntValue(int value, int minVal, int maxVal) const;
};

}  // namespace HighlyDynamic
