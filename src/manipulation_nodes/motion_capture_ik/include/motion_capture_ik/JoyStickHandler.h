#pragma once

#include <noitom_hi5_hand_udp_python/JoySticks.h>

#include <atomic>
#include <leju_utils/define.hpp>
#include <vector>
#include <mutex>

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
  JoyStickHandler(double threshold = 0.0, double alpha = 0.0);

  void initialize();

  void reset();

  double getLeftJoyStickX() const;
  double getLeftJoyStickY() const;
  double getRightJoyStickX() const;
  double getRightJoyStickY() const;
  bool getRightJoyStickYHold() const;

  void updateJoyStickData(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg);
  void processHandEndEffectorData();
  void processHandEndEffectorDataWithFingerTracking();

  HandPositionData getHandPositionData();
  ClawCommandData getClawCommandData();
  EndEffectorType getEndEffectorType() const;

  bool isLeftFirstButtonPressed() const;
  bool isRightFirstButtonPressed() const;
  bool isLeftSecondButtonPressed() const;
  bool isRightSecondButtonPressed() const;
  bool isLeftFirstButtonTouched() const;
  bool isLeftSecondButtonTouched() const;

  bool isLeftRightFirstButtonTouched() const;
  bool isLeftRightFirstButtonPressed() const;

  bool isLeftGrip() const;
  bool isRightGrip() const;
  bool isLeftRightGrip() const;

  bool isLeftArmCtrlModeActive() const;
  bool isRightArmCtrlModeActive() const;

  bool hasLeftArmCtrlModeChanged();
  bool hasRightArmCtrlModeChanged();

  void forceSetLeftArmCtrlMode(bool active);
  void forceSetRightArmCtrlMode(bool active);

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
  bool leftSecondButtonTouched_;
  bool leftFirstButtonTouched_;
  bool leftFirstButtonPressed_;  // 左手第一个按键按下状态
  bool rightSecondButtonPressed_;
  bool rightFirstButtonTouched_;
  bool rightFirstButtonPressed_;  // 右手第一个按键按下状态

  bool leftGrip_;
  bool rightGrip_;

  // 冻结功能相关变量
  bool buttonYLast_;                          // 上一次Y按钮状态，用于边沿检测
  bool freezeFinger_;                         // 冻结状态标志
  std::vector<int> frozenLeftHandPosition_;   // 冻结的左手位置
  std::vector<int> frozenRightHandPosition_;  // 冻结的右手位置
  std::vector<int> frozenClawPosition_;       // 冻结的夹爪位置

  // 手臂控制模式相关变量
  bool leftArmCtrlModeActive_;            // 左手控制模式是否激活
  bool rightArmCtrlModeActive_;           // 右手控制模式是否激活
  bool leftButtonXLastPressed_;           // X键上一次按下状态（用于边沿检测）
  bool rightButtonALastPressed_;          // A键上一次按下状态（用于边沿检测）
  ros::Time leftButtonXFirstPressTime_;   // X键第一次按下的时间戳
  ros::Time rightButtonAFirstPressTime_;  // A键第一次按下的时间戳
  ros::Time lastArmCtrlModeChangeTime_;  // 任意手控制模式最后一次状态变化的时间戳（用于统一保护机制）
  static constexpr double DOUBLE_CLICK_TIMEOUT = 0.5;        // 双击检测超时时间（秒）
  static constexpr double MODE_CHANGE_BLOCK_DURATION = 5.0;  // 模式切换后阻止所有手的时间（秒）

  // 状态变化查询相关变量
  bool lastQueryLeftArmCtrlModeActive_;   // 上次查询时的左手控制模式状态
  bool lastQueryRightArmCtrlModeActive_;  // 上次查询时的右手控制模式状态

  std::atomic<EndEffectorType> endEffectorType_;
  int controlFingerType_;

  std::vector<double> leftJoystick_;     // [left_trigger, left_grip]
  std::vector<double> rightJoystick_;    // [right_trigger, right_grip]
  double leftStickX_;                    // left stick X coordinate
  double leftStickY_;                    // left stick Y coordinate
  double rightStickX_;                   // right stick X coordinate
  double rightStickY_;                   // right stick Y coordinate
  std::vector<double> leftFingerData_;   // 左手手指关节数据
  std::vector<double> rightFingerData_;  // 右手手指关节数据

  // 处理后的手指位置数据
  std::vector<int> leftHandPosition_;   // 左手位置 [0-100]
  std::vector<int> rightHandPosition_;  // 右手位置 [0-100]
  std::vector<int> clawPosition_;       // 爪子位置 [0-100]

  // 手柄摇杆缓存数据
  double leftJoyStickX_;
  double leftJoyStickY_;
  double rightJoyStickX_;
  double rightJoyStickY_;

  // 摇杆阈值和低通滤波参数
  double joyStickThreshold_;
  double joyStickAlpha_;

  // RightJoyStickY 按下时间检查相关变量
  bool RightJoyStickYHold_;      // RightJoyStickY 保持状态，默认值为 true
  int rightJoyStickYHoldCount_;  // 连续满足条件的计数

  HandPositionData handPositionData_;
  ClawCommandData clawCommandData_;

  void loadHandControlParameters();
  int limitIntValue(int value, int minVal, int maxVal) const;

  mutable std::mutex dataMutex_;
};

}  // namespace HighlyDynamic
