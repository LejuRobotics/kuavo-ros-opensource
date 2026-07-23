#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <kuavo_msgs/SetIncrementalArmTrajLink.h>

#include "kuavo_common/common/arm_traj_shm.h"

namespace HighlyDynamic {

/// VR/IK 侧 ArmTraj SHM Writer，与 humanoid_controllers::ArmTrajReceiver 成对。
/// 职责：Writer 生命周期、通知 WBC setLink、可选侧车 service；不含 mode 策略。
class ArmTrajWriter {
 public:
  ArmTrajWriter() = default;
  ~ArmTrajWriter();

  ArmTrajWriter(const ArmTrajWriter&) = delete;
  ArmTrajWriter& operator=(const ArmTrajWriter&) = delete;

  /// @param controller_set_link_service 人形 /humanoid_controller/... 或轮臂 /humanoid_wheel/...
  /// @param sidecar_service 可选侧车，空则不 advertise
  void init(ros::NodeHandle& nh, const std::string& controller_set_link_service,
            const std::string& sidecar_service = "");

  /// 退链 + 释放 SHM（析构也会调用）
  void shutdown();

  bool setTransport(int8_t transport);
  bool writeIfActive(uint32_t num_joints, const double* position_rad, const double* velocity_rad,
                     const double* effort, uint64_t stamp_nsec);
  bool isShmActive() const;

 private:
  bool handleSidecar(kuavo_msgs::SetIncrementalArmTrajLink::Request& req,
                     kuavo_msgs::SetIncrementalArmTrajLink::Response& res);

  ros::NodeHandle nh_;
  std::string controller_service_;
  ros::ServiceClient client_;
  ros::ServiceServer sidecar_;
  std::unique_ptr<kuavo_common::ArmTrajShmManager> shm_;
  std::atomic<int8_t> transport_{
      kuavo_msgs::SetIncrementalArmTrajLink::Request::TRANSPORT_NONE};
  bool inited_{false};
};

}  // namespace HighlyDynamic
