#pragma once

#include <ros/ros.h>

#include <atomic>
#include <functional>
#include <thread>

namespace HighlyDynamic {

/// Quest3 手柄输入。人形与轮臂的 joystick 类不同,用回调注入。
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
 * @brief VR 侧黑漫手势客户端。
 *
 * 不持有手势库,也不注册 /sg100/*。30Hz 发布 /sg100/trigger,
 * 摸 X+A / X+B 满 300ms 后调用 /sg100/step_gesture。
 * 服务由真机或仿真的手节点提供。
 */
class SG100VrGestureClient {
 public:
  SG100VrGestureClient(ros::NodeHandle& nh, SG100VrInput vr);
  ~SG100VrGestureClient();

  void start();
  void stop();

 private:
  bool vrInputReady() const;
  void loop();
  void checkVrGestureSwitch();

  static constexpr double kConfirmDelaySec = 0.3;

  ros::NodeHandle nh_;
  SG100VrInput vr_;
  ros::Publisher trigger_pub_;
  ros::ServiceClient step_client_;
  std::thread thread_;
  std::atomic<bool> stop_{false};

  int pending_dir_ = 0;
  bool debounce_ = false;
  ros::WallTime pending_start_;
};

}  // namespace HighlyDynamic
