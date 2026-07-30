#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <kuavo_msgs/SetIncrementalArmTrajLink.h>

#include "kuavo_common/common/arm_traj_shm.h"

namespace humanoid_controller {

/// 人形 / 轮臂统一的手臂轨迹接收（对齐 !3723）：
/// ROS `/kuavo_arm_traj` + ArmTraj SHM，service 切换 transport。
class ArmTrajReceiver {
 public:
  using TrajSink = std::function<void(const double* pos_rad, const double* vel_rad,
                                      const double* tau, int n, uint64_t stamp_nsec)>;

  ArmTrajReceiver() = default;
  ~ArmTrajReceiver();

  ArmTrajReceiver(const ArmTrajReceiver&) = delete;
  ArmTrajReceiver& operator=(const ArmTrajReceiver&) = delete;

  /// @param set_link_service 人形: /humanoid_controller/...  轮臂: /humanoid_wheel/...
  /// @param default_transport 人形常用 KUAVO_ARM_TRAJ；轮臂常用 NONE（仍订阅 ROS）
  void init(ros::NodeHandle& nh, int arm_num, TrajSink sink,
            const std::string& set_link_service =
                "/humanoid_controller/set_incremental_arm_traj_link",
            int8_t default_transport =
                kuavo_msgs::SetIncrementalArmTrajLink::Request::TRANSPORT_KUAVO_ARM_TRAJ);

  bool setLink(int8_t transport, std::string* message = nullptr);
  bool handleSetLink(kuavo_msgs::SetIncrementalArmTrajLink::Request& req,
                     kuavo_msgs::SetIncrementalArmTrajLink::Response& res);
  /// 仅供 traj CallbackQueue 回调调用（退订必须在 spinner 线程）
  void unsubOnTrajQueue();

 private:
  enum class ArmTrajSource { kRosTopic, kIncrementalShm };

  void rosCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void shmThread();
  void ensureRosSub(bool enable);
  void publishStatus();
  bool shouldAcceptFrom(ArmTrajSource source) const;
  void ingest(ArmTrajSource source, const double* pos_rad, const double* vel_rad,
              const double* tau, int count, uint64_t stamp_nsec);

  int arm_num_ = 0;
  TrajSink sink_;

  ros::NodeHandle nh_;
  ros::NodeHandle traj_nh_;
  ros::CallbackQueue traj_callback_queue_;
  std::unique_ptr<ros::AsyncSpinner> traj_spinner_;
  ros::Subscriber ros_sub_;
  std::atomic<bool> pending_unsub_{false};
  ros::ServiceServer srv_;
  ros::Publisher using_shm_pub_;
  ros::Publisher transport_pub_;
  std::mutex ros_sub_mutex_;
  std::mutex status_pub_mutex_;
  std::string set_link_service_;

  std::unique_ptr<kuavo_common::ArmTrajShmManager> shm_;
  std::thread shm_thread_;
  std::atomic<bool> shm_running_{false};
  bool shm_capable_{false};
  double shm_stale_sec_{0.3};
  std::atomic<int8_t> transport_{
      kuavo_msgs::SetIncrementalArmTrajLink::Request::TRANSPORT_NONE};
  std::chrono::steady_clock::time_point last_shm_recv_wall_{};
  bool has_last_shm_recv_{false};
};

}  // namespace humanoid_controller
