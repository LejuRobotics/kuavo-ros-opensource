#pragma once

#include <ros/ros.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <kuavo_msgs/SG100AddGestures.h>
#include <kuavo_msgs/SG100DeleteGesture.h>
#include <kuavo_msgs/SG100GetGestures.h>
#include <kuavo_msgs/SG100HandCommand.h>
#include <kuavo_msgs/SG100StepGesture.h>
#include <kuavo_msgs/SG100SwitchGesture.h>
#include <kuavo_msgs/SG100UpdateGesture.h>

#include "motion_capture_ik/SG100GestureLibrary.h"
#include "motion_capture_ik/SG100JointLimits.h"

namespace HighlyDynamic {

/// VR(Quest3)输入注入:人形与轮臂各自持有不同的 joystick 类,用回调组解耦
struct SG100VrInput {
  std::function<float()> left_trigger;
  std::function<float()> right_trigger;
  std::function<bool()> left_first_touched;
  std::function<bool()> left_first_pressed;
  std::function<bool()> right_first_touched;
  std::function<bool()> right_first_pressed;
  std::function<bool()> right_second_touched;
  std::function<bool()> right_second_pressed;
};

/**
 * @brief SG100(黑漫)手 ROS 桥接:手势库 + /sg100/* service + /sg100_hand_command 发布线程
 *
 * 人形 Quest3IkIncrementalROS 与轮臂 WheelQuest3IkIncrementalROS 共用本实现,
 * 避免两侧各维护一份约 500 行重复代码。
 * 调用方在门控 end_effector_type == heiman 后构造并 start(),析构时自动停线程。
 */
class SG100HandBridge {
 public:
  SG100HandBridge(ros::NodeHandle& nh, SG100VrInput vr);
  ~SG100HandBridge();

  /// 加载手势库与 URDF 限位、注册 6 个 service、advertise 命令并启动 30Hz 发布线程
  void start();

  /// 停止发布线程(幂等;析构时自动调用)
  void stop();

 private:
  /// VR 输入回调是否已注入
  bool vrInputReady() const;

  void publishLoop();
  void checkVrGestureSwitch();
  void clampKeyModeLocked(std::size_t mode_count);

  /// hand_side 解析:0→left, 1→right, 2→both;其余返回 false
  bool applyHandSide(int hand_side, bool& do_left, bool& do_right);

  bool onGetGestures(kuavo_msgs::SG100GetGestures::Request& req,
                     kuavo_msgs::SG100GetGestures::Response& res);
  bool onAddGestures(kuavo_msgs::SG100AddGestures::Request& req,
                     kuavo_msgs::SG100AddGestures::Response& res);
  bool onDeleteGesture(kuavo_msgs::SG100DeleteGesture::Request& req,
                       kuavo_msgs::SG100DeleteGesture::Response& res);
  bool onUpdateGesture(kuavo_msgs::SG100UpdateGesture::Request& req,
                       kuavo_msgs::SG100UpdateGesture::Response& res);
  bool onSwitchGesture(kuavo_msgs::SG100SwitchGesture::Request& req,
                       kuavo_msgs::SG100SwitchGesture::Response& res);
  bool onStepGesture(kuavo_msgs::SG100StepGesture::Request& req,
                     kuavo_msgs::SG100StepGesture::Response& res);

  /// VR 手势切换确认延迟(秒):摸 X+A/B 后需持续按住超过该时长才切换
  static constexpr double SG100_VR_SWITCH_CONFIRM_DELAY = 0.3;

  ros::NodeHandle nh_;
  SG100VrInput vr_;

  SG100GestureLibrary left_lib_;
  SG100GestureLibrary right_lib_;
  SG100JointLimits left_limits_;
  SG100JointLimits right_limits_;

  ros::Publisher cmd_pub_;
  ros::ServiceServer srv_get_;
  ros::ServiceServer srv_add_;
  ros::ServiceServer srv_delete_;
  ros::ServiceServer srv_update_;
  ros::ServiceServer srv_switch_;
  ros::ServiceServer srv_step_;

  std::thread thread_;
  std::mutex mutex_;
  std::atomic<int> key_mode_{0};
  std::atomic<bool> ready_{false};
  std::atomic<bool> stop_{false};

  std::string left_yaml_path_;
  std::string right_yaml_path_;

  int vr_switch_pending_dir_ = 0;
  bool vr_switch_debounce_ = false;
  ros::Time vr_switch_pending_start_;
};

}  // namespace HighlyDynamic
