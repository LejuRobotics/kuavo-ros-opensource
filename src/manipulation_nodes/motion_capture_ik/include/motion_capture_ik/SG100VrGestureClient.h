#pragma once

#include <ros/ros.h>

#include <atomic>
#include <functional>
#include <thread>

namespace HighlyDynamic {

/// Quest3 手柄输入。人形与轮臂的 joystick 类不同,用回调注入。
struct SG100VrInput {
  std::function<bool()> fresh;
  std::function<float()> left_trigger;
  std::function<float()> right_trigger;
  std::function<bool()> left_first_touched;
  std::function<bool()> left_first_pressed;
  std::function<bool()> left_second_touched;
  std::function<bool()> left_second_pressed;
  std::function<bool()> sticks_neutral;
  std::function<bool()> right_first_touched;
  std::function<bool()> right_first_pressed;
  std::function<bool()> right_second_touched;
  std::function<bool()> right_second_pressed;
};

/**
 * @brief VR 侧黑漫手势客户端。
 *
 * 不持有手势库,也不注册 /sg100/ 服务。仅在原始输入新鲜时 30Hz 发布 /sg100/trigger,
 * 断流时发送一次空 data 通知暂停,不会重复发送缓存的扳机值。
 * 按下左手 Y 锁定当前扳机：锁定期间持续发布按下瞬间的值，切手势不再响应，再按一次解除。
 * 未锁定时，仅轻触 Y+A / Y+B 满 800ms 后调用 /sg100/vr_step_gesture。
 * 四个摇杆轴必须回中（原始值绝对值 <= 0.1）；按压、触摸 X、A/B 同触或摇杆偏转取消。
 * 切换或取消后必须完全松开 X/Y/A/B 再试。
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

  static constexpr double kConfirmDelaySec = 0.8;

  ros::NodeHandle nh_;
  SG100VrInput vr_;
  ros::Publisher trigger_pub_;
  ros::ServiceClient step_client_;
  std::thread thread_;
  std::atomic<bool> stop_{false};

  int pending_dir_ = 0;
  bool debounce_ = false;
  ros::WallTime pending_start_;
  bool finger_locked_ = false;
  bool y_pressed_last_ = false;
  float frozen_left_ = 0.f;
  float frozen_right_ = 0.f;
};

}  // namespace HighlyDynamic
