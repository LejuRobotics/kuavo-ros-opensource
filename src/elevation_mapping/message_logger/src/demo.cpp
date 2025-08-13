// This header must be included first as it potentially defines MELO_USE_COUT.
// For MELO_USE_SENTRY we've solved it better by defining it in the CMakeLists.txt.

#include <string>

#include "message_logger/message_logger.hpp"

#ifndef MELO_USE_COUT
#ifndef ROS2_BUILD
#include <ros/node_handle.h>
#else /* ROS2_BUILD */
#include <rclcpp/node.hpp>
#endif /* ROS2_BUILD */
#endif

#ifdef MELO_USE_COUT
int main(int /*argc*/, char** /*argv*/) {
#else
// NOLINTNEXTLINE(bugprone-exception-escape)
int main(int argc, char** argv) {
#ifndef ROS2_BUILD
  ros::init(argc, argv, "demo");
  ros::start();
#else  /* ROS2_BUILD */
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("demo");
  message_logger::log::LoggerManager::setLogger(node->get_logger());
  message_logger::log::LoggerClockManager::setClock(*node->get_clock());
#endif /* ROS2_BUILD */
#endif

  MELO_DEBUG("debug %i", 1)
  MELO_DEBUG_STREAM("debug stream " << std::to_string(2))
  MELO_INFO_STREAM("info stream " << std::to_string(3))
  MELO_INFO_FP("info fp %i", 1)
  MELO_INFO_STREAM_FP("info fp " << std::to_string(2))
  MELO_INFO_ONCE("info once %i", 3)
  MELO_INFO_STREAM_ONCE("info stream once " << std::to_string(4))
  MELO_WARN_THROTTLE(1.0, "warn throttle %i", 1)
  MELO_WARN_THROTTLE_STREAM(1.0, "warn throttle stream " << std::to_string(2))
  // If enabled, an error or fatal message will be sent to Sentry.
  MELO_ERROR("error")

#ifndef MELO_USE_COUT
#ifndef ROS2_BUILD
  ros::waitForShutdown();
  ros::shutdown();
#else  /* ROS2_BUILD */
  rclcpp::spin(node);
  rclcpp::shutdown();
  std::cout << "NOde shutdown..." << std::endl;
#endif /* ROS2_BUILD */
#endif
  return 0;
}
