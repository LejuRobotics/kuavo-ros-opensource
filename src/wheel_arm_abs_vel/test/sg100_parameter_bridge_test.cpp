// Software-only integration test: links the gesture bridge, never the driver.
#include "wheel_arm_abs_vel/sg100/SG100HandBridge.h"
#include <ros/ros.h>
#include <atomic>
#include <mutex>
#include <cstdio>
#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "sg100_parameter_bridge_test");
  ros::NodeHandle nh;
  nh.setParam("robot_version", 400062);
  nh.setParam("sg100_kp", 10.0);
  nh.setParam("sg100_kd", 0.2);
  nh.setParam("sg100_output_limit", 0.6);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  HighlyDynamic::SG100HandBridge bridge(nh);
  bridge.setConfiguredHands(false, true);
  std::mutex mutex;
  kuavo_msgs::SG100HandCommand last;
  std::atomic<unsigned> count{0};
  bridge.setCommandHandler([&](const kuavo_msgs::SG100HandCommand& cmd) {
    std::lock_guard<std::mutex> lock(mutex); last = cmd; ++count;
  });
  bridge.start();
  auto client = nh.serviceClient<kuavo_msgs::SG100StepGesture>("/sg100/step_gesture");
  kuavo_msgs::SG100StepGesture request;
  request.request.hand_side = 1; request.request.direction = 1;
  bool ok = client.waitForExistence(ros::Duration(3)) && client.call(request) && request.response.success;
  auto waitValues = [&](float kp, float kd, float limit) {
    const auto deadline = ros::WallTime::now() + ros::WallDuration(3);
    while (ros::WallTime::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (count && last.right_hand_kp.size() == 11 && last.right_hand_kd.size() == 11 &&
            last.right_hand_output_limit.size() == 11 &&
            std::abs(last.right_hand_kp[0]-kp)<1e-6f &&
            std::abs(last.right_hand_kd[0]-kd)<1e-6f &&
            std::abs(last.right_hand_output_limit[0]-limit)<1e-6f &&
            last.left_enable_mask == 0 && last.right_enable_mask == 0x7ff) return true;
      }
      ros::WallDuration(0.02).sleep();
    }
    return false;
  };
  ok = ok && waitValues(10, 0.2f, 0.6f);
  std::printf("%s deployment parameters reach position command\n", ok ? "PASS" : "FAIL");
  // This is the same parameter-server update performed by an explicit Quest
  // launch argument, after the controller's bridge is already running.
  nh.setParam("sg100_kp", 2.0);
  nh.setParam("sg100_kd", 0.04);
  nh.setParam("sg100_output_limit", 0.35);
  bool updated = waitValues(2, 0.04f, 0.35f);
  std::printf("%s late Quest overrides reach running bridge\n", updated ? "PASS" : "FAIL");
  ok = ok && updated;
  nh.setParam("sg100_output_limit", 5.0);
  ros::WallDuration(0.2).sleep();
  const unsigned before = count;
  ros::WallDuration(0.2).sleep();
  const bool rejected = count == before;
  std::printf("%s invalid values suppress publication\n", rejected ? "PASS" : "FAIL");
  bridge.stop(); spinner.stop(); ros::shutdown();
  return ok && rejected ? 0 : 1;
}
