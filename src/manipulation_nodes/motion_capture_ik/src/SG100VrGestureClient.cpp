#include "motion_capture_ik/SG100VrGestureClient.h"

#include <utility>
#include <cmath>

#include <std_msgs/Float32MultiArray.h>
#include <kuavo_msgs/SG100StepGesture.h>

#include "motion_capture_ik/SG100HandBridge.h"

namespace HighlyDynamic {

SG100VrGestureClient::SG100VrGestureClient(ros::NodeHandle& nh, SG100VrInput vr)
    : nh_(nh), vr_(std::move(vr)) {}

SG100VrGestureClient::~SG100VrGestureClient() { stop(); }

bool SG100VrGestureClient::vrInputReady() const {
  return vr_.fresh && vr_.fresh() && vr_.left_trigger && vr_.right_trigger;
}

void SG100VrGestureClient::stop() {
  stop_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void SG100VrGestureClient::start() {
  trigger_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(kSg100TriggerTopic, 10);
  step_client_ = nh_.serviceClient<kuavo_msgs::SG100StepGesture>("/sg100/step_gesture");
  stop_.store(false);
  thread_ = std::thread(&SG100VrGestureClient::loop, this);
  ROS_INFO("[SG100VrGestureClient] publishing %s, stepping via /sg100/step_gesture",
           kSg100TriggerTopic);
}

void SG100VrGestureClient::loop() {
  ros::WallRate rate(30.0);
  bool had_fresh_input = false;
  while (!stop_.load() && ros::ok()) {
    if (!vrInputReady()) {
      if (had_fresh_input) {
        trigger_pub_.publish(std_msgs::Float32MultiArray{});
        ROS_WARN("[SG100VrGestureClient] VR input stale; pausing commands, last target held");
      }
      had_fresh_input = false;
      pending_dir_ = 0;
      debounce_ = false;
      rate.sleep();
      continue;
    }
    std_msgs::Float32MultiArray trigger;
    trigger.data.resize(2);
    trigger.data[0] = vr_.left_trigger();
    trigger.data[1] = vr_.right_trigger();
    if (!std::isfinite(trigger.data[0]) || !std::isfinite(trigger.data[1])) {
      trigger_pub_.publish(std_msgs::Float32MultiArray{});
      had_fresh_input = false;
      pending_dir_ = 0;
      debounce_ = false;
      ROS_WARN_THROTTLE(1.0, "[SG100VrGestureClient] invalid trigger; pausing commands");
      rate.sleep();
      continue;
    }
    trigger_pub_.publish(trigger);
    had_fresh_input = true;
    checkVrGestureSwitch();
    rate.sleep();
  }
  if (had_fresh_input && ros::ok()) {
    trigger_pub_.publish(std_msgs::Float32MultiArray{});
  }
}

void SG100VrGestureClient::checkVrGestureSwitch() {
  if (!vrInputReady()) {
    return;
  }
  const ros::WallTime now = ros::WallTime::now();

  const bool x_touched = vr_.left_first_touched();
  if (!x_touched) {
    pending_dir_ = 0;
    debounce_ = false;
    return;
  }

  int direction = 0;
  bool btn_pressed = false;
  if (vr_.right_first_touched()) {
    direction = 1;
    btn_pressed = vr_.right_first_pressed();
  } else if (vr_.right_second_touched()) {
    direction = -1;
    btn_pressed = vr_.right_second_pressed();
  } else {
    pending_dir_ = 0;
    debounce_ = false;
    return;
  }

  if (vr_.left_first_pressed() && btn_pressed) {
    pending_dir_ = 0;
    return;
  }

  if (debounce_) {
    return;
  }

  if (pending_dir_ != direction) {
    pending_dir_ = direction;
    pending_start_ = now;
    return;
  }

  if ((now - pending_start_).toSec() < kConfirmDelaySec) {
    return;
  }

  if (!step_client_.exists()) {
    ROS_WARN_THROTTLE(5.0,
                      "[SG100VrGestureClient] /sg100/step_gesture not available "
                      "(hand node not up); gesture switch skipped");
    pending_dir_ = 0;
    return;
  }

  kuavo_msgs::SG100StepGesture srv;
  srv.request.direction = direction;
  srv.request.hand_side = 2;
  if (!step_client_.call(srv) || !srv.response.success) {
    ROS_WARN_THROTTLE(2.0, "[SG100VrGestureClient] step_gesture failed: %s",
                      srv.response.message.c_str());
    pending_dir_ = 0;
    return;
  }

  debounce_ = true;
  ROS_INFO("[SG100VrGestureClient] VR gesture switch %s -> id %d",
           direction > 0 ? "next" : "prev", srv.response.current_id);
}

}  // namespace HighlyDynamic
