#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <chrono>
#include <cmath>

// Translation only: dev's driver owns the sole SG100HandBridge, its services,
// command filtering and hardware safety. Never instantiate a second bridge.
class MapperTriggerForwarder {
 public:
  explicit MapperTriggerForwarder(ros::NodeHandle& nh)
      : pub_(nh.advertise<std_msgs::Float32MultiArray>("/sg100/trigger", 1)) {
    sub_ = nh.subscribe("/sg100/mapper_triggers", 1,
        &MapperTriggerForwarder::receive, this, ros::TransportHints().tcpNoDelay());
    timer_ = nh.createWallTimer(ros::WallDuration(0.025),
        &MapperTriggerForwarder::expire, this);
  }
 private:
  using Clock = std::chrono::steady_clock;
  void revoke() {
    std_msgs::Float32MultiArray msg;  // dev contract: empty means input lost
    pub_.publish(msg);
    valid_ = false;
  }
  void receive(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    const auto& v = msg->vector;
    const double age = (ros::Time::now() - msg->header.stamp).toSec();
    if (v.z != 1.0 || !std::isfinite(v.x) || !std::isfinite(v.y) ||
        v.x < 0 || v.x > 1 || v.y < 0 || v.y > 1 ||
        msg->header.stamp.isZero() || age < -0.05 || age > 0.15 ||
        msg->header.stamp <= last_stamp_) {
      revoke();
      return;
    }
    last_stamp_ = msg->header.stamp;
    received_ = Clock::now();
    valid_ = true;
    std_msgs::Float32MultiArray out;
    out.data = {static_cast<float>(v.x), static_cast<float>(v.y)};
    pub_.publish(out);
  }
  void expire(const ros::WallTimerEvent&) {
    if (valid_ && std::chrono::duration<double>(Clock::now()-received_).count() > 0.15)
      revoke();
  }
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::WallTimer timer_;
  ros::Time last_stamp_;
  Clock::time_point received_{};
  bool valid_{false};
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "sg100_mapper_bridge");
  ros::NodeHandle nh;
  // Keep subscriptions and their callback owner alive until spin() returns.
  MapperTriggerForwarder forwarder(nh);
  (void)forwarder;  // Intentional lifetime owner; also understood by older analyzers.
  ros::spin();
  return 0;
}
