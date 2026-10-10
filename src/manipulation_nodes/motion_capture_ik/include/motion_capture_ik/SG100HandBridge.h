#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <std_msgs/Float32MultiArray.h>

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

/// 扳机话题:/sg100/trigger,Float32MultiArray data[0]=左手 data[1]=右手,范围 0~1。
/// 没有发布者时按 0 解算(手势起点)。
inline constexpr const char* kSg100TriggerTopic = "/sg100/trigger";

/**
 * @brief SG100(黑漫)手 ROS 桥接:手势库 + /sg100/* service + /sg100_hand_command 发布线程
 *
 * 挂在跟机器人一起起来的手节点上(真机 SG100HandROSNode、仿真 DexHandMujocoRosNode)。
 * service / trigger 走独立 CallbackQueue + AsyncSpinner,避免被 nodelet_manager
 * 公共回调队列堵十几秒。同进程内还可 setCommandHandler 直送驱动,不绕订阅。
 */
class SG100HandBridge {
 public:
  using CommandHandler = std::function<void(const kuavo_msgs::SG100HandCommand&)>;

  explicit SG100HandBridge(ros::NodeHandle& nh);
  ~SG100HandBridge();

  /// 同进程直送手驱动/仿真(在 start 前设置)。仍会发布 /sg100_hand_command 给外部。
  void setCommandHandler(CommandHandler handler);

  /// 加载手势库与 URDF 限位、注册 6 个 service、advertise 命令并启动 30Hz 发布线程
  void start();

  /// 停止发布线程与专用 spinner(幂等;析构时自动调用)
  void stop();

 private:
  void publishLoop();
  void onTrigger(const std_msgs::Float32MultiArray::ConstPtr& msg);
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

  ros::NodeHandle nh_;
  ros::NodeHandle gesture_nh_;
  ros::CallbackQueue callback_queue_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;

  SG100GestureLibrary left_lib_;
  SG100GestureLibrary right_lib_;
  SG100JointLimits left_limits_;
  SG100JointLimits right_limits_;

  ros::Publisher cmd_pub_;
  ros::Subscriber trigger_sub_;
  ros::ServiceServer srv_get_;
  ros::ServiceServer srv_add_;
  ros::ServiceServer srv_delete_;
  ros::ServiceServer srv_update_;
  ros::ServiceServer srv_switch_;
  ros::ServiceServer srv_step_;

  std::thread thread_;
  std::mutex mutex_;
  std::mutex handler_mutex_;
  CommandHandler command_handler_;
  std::atomic<int> key_mode_{0};
  std::atomic<bool> ready_{false};
  std::atomic<bool> stop_{false};

  std::string left_yaml_path_;
  std::string right_yaml_path_;

  std::atomic<float> left_trigger_{0.0f};
  std::atomic<float> right_trigger_{0.0f};
};

}  // namespace HighlyDynamic
