#pragma once

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include "motion_capture_ik/SG100BridgeGate.h"
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

/// /sg100/trigger：data[0] 为左手，data[1] 为右手，范围 0~1。
/// 空 data 表示输入断流。有效输入默认自动开始/恢复；人工禁用后不自动启用。
inline constexpr const char* kSg100TriggerTopic = "/sg100/trigger";

/**
 * @brief SG100(黑漫)手 ROS 桥接:手势库 + /sg100/ services + /sg100_hand_command 发布线程
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

  /// 在 start 前设置；直送驱动的同时仍发布命令话题。
  void setCommandHandler(CommandHandler handler);

  /// 在 start 前设置；输入恢复/人工重新启用时通知驱动从反馈位置重置滤波。
  void setInputResumeHandler(std::function<void()> handler);

  /// 真机在 start 前传入端口配置；未配置的手不生成使能掩码。
  void setConfiguredHands(bool left, bool right);

  /// 加载手势库和 URDF/JSON 限位，注册服务并启动 100Hz 发布线程。
  void start();

  /// 停止发布线程与专用 spinner(幂等;析构时自动调用)
  void stop();

 private:
  /// 最近的有效 trigger 是否仍然新鲜
  bool vrInputReady() const;

  bool onSetEnabled(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  void publishLoop();
  void requestFilterReseed();
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

  static constexpr uint32_t kVrSourceId = 1;
  bool left_enabled_ = true;
  bool right_enabled_ = true;
  ros::NodeHandle nh_;
  ros::NodeHandle gesture_nh_;
  ros::CallbackQueue callback_queue_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;

  SG100GestureLibrary left_lib_;
  SG100GestureLibrary right_lib_;
  SG100JointLimits left_limits_;
  SG100JointLimits right_limits_;

  ros::Publisher cmd_pub_;
  ros::ServiceServer srv_enable_;
  ros::Subscriber trigger_sub_;
  ros::ServiceServer srv_get_;
  ros::ServiceServer srv_add_;
  ros::ServiceServer srv_delete_;
  ros::ServiceServer srv_update_;
  ros::ServiceServer srv_switch_;
  ros::ServiceServer srv_step_;

  std::thread thread_;
  std::mutex mutex_;
  std::mutex gate_mutex_;
  SG100BridgeGate gate_;
  std::mutex handler_mutex_;
  CommandHandler command_handler_;
  std::function<void()> input_resume_handler_;
  std::atomic<int> key_mode_{0};
  std::atomic<bool> ready_{false};
  std::atomic<bool> stop_{false};

  std::string left_yaml_path_;
  std::string right_yaml_path_;

  std::atomic<float> left_trigger_{0.0f};
  std::atomic<float> right_trigger_{0.0f};

  std::atomic<double> last_trigger_received_{0.0};
};

}  // namespace HighlyDynamic
