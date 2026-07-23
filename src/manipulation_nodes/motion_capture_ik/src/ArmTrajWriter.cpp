#include "motion_capture_ik/ArmTrajWriter.h"

namespace HighlyDynamic {

ArmTrajWriter::~ArmTrajWriter() {
  shutdown();
}

void ArmTrajWriter::init(ros::NodeHandle& nh, const std::string& controller_set_link_service,
                         const std::string& sidecar_service) {
  nh_ = nh;
  controller_service_ = controller_set_link_service;
  client_ = nh_.serviceClient<kuavo_msgs::SetIncrementalArmTrajLink>(controller_service_);
  if (!sidecar_service.empty()) {
    sidecar_ = nh_.advertiseService(sidecar_service, &ArmTrajWriter::handleSidecar, this);
  }
  inited_ = true;
  ROS_INFO("[ArmTrajWriter] init controller_service=%s sidecar=%s", controller_service_.c_str(),
           sidecar_service.empty() ? "(none)" : sidecar_service.c_str());
}

void ArmTrajWriter::shutdown() {
  if (!inited_) {
    return;
  }
  setTransport(kuavo_msgs::SetIncrementalArmTrajLink::Request::TRANSPORT_NONE);
  if (shm_) {
    shm_->cleanup();
    shm_.reset();
  }
  sidecar_ = ros::ServiceServer();
  inited_ = false;
}

bool ArmTrajWriter::isShmActive() const {
  using Request = kuavo_msgs::SetIncrementalArmTrajLink::Request;
  return transport_.load(std::memory_order_acquire) == Request::TRANSPORT_SHM && shm_ != nullptr;
}

bool ArmTrajWriter::writeIfActive(uint32_t num_joints, const double* position_rad,
                                  const double* velocity_rad, const double* effort,
                                  uint64_t stamp_nsec) {
  if (!isShmActive()) {
    return false;
  }
  return shm_->writeTrajRad(num_joints, position_rad, velocity_rad, effort, stamp_nsec);
}

bool ArmTrajWriter::setTransport(int8_t transport) {
  using Request = kuavo_msgs::SetIncrementalArmTrajLink::Request;
  if (!inited_) {
    ROS_WARN("[ArmTrajWriter] setTransport before init");
    return false;
  }
  if (transport != Request::TRANSPORT_NONE && transport != Request::TRANSPORT_SHM &&
      transport != Request::TRANSPORT_KUAVO_ARM_TRAJ) {
    ROS_WARN("[ArmTrajWriter] Invalid transport: %d", static_cast<int>(transport));
    return false;
  }

  if (!client_.exists()) {
    client_ = nh_.serviceClient<kuavo_msgs::SetIncrementalArmTrajLink>(controller_service_);
  }

  const bool block_until_ready = (transport == Request::TRANSPORT_SHM);
  bool service_ready = false;
  if (block_until_ready) {
    while (ros::ok()) {
      if (client_.waitForExistence(ros::Duration(1.0))) {
        service_ready = true;
        break;
      }
      ROS_WARN("[ArmTrajWriter] Waiting for %s ...", controller_service_.c_str());
    }
  } else {
    service_ready = client_.waitForExistence(ros::Duration(1.0));
  }
  if (!ros::ok() || !service_ready) {
    ROS_WARN("[ArmTrajWriter] Service %s unavailable", controller_service_.c_str());
    return false;
  }

  // Writer 先就绪并清残留，再通知 Receiver，避免旧 seq 触发 stale
  if (transport == Request::TRANSPORT_SHM) {
    if (!shm_) {
      shm_ = std::make_unique<kuavo_common::ArmTrajShmManager>();
      if (!shm_->initialize(kuavo_common::ArmTrajShmManager::Role::Writer)) {
        ROS_ERROR("[ArmTrajWriter] Failed to initialize SHM writer");
        shm_.reset();
        return false;
      }
    } else {
      shm_->invalidate();
    }
  }

  kuavo_msgs::SetIncrementalArmTrajLink srv;
  srv.request.transport = transport;
  if (!client_.call(srv) || !srv.response.success) {
    ROS_WARN("[ArmTrajWriter] setLink failed: %s", srv.response.message.c_str());
    if (transport == Request::TRANSPORT_SHM && shm_) {
      shm_->cleanup();
      shm_.reset();
    }
    return false;
  }
  ROS_INFO("[ArmTrajWriter] setLink ok: %s", srv.response.message.c_str());

  if (transport != Request::TRANSPORT_SHM && shm_) {
    shm_->cleanup();
    shm_.reset();
  }

  transport_.store(transport, std::memory_order_release);
  return true;
}

bool ArmTrajWriter::handleSidecar(kuavo_msgs::SetIncrementalArmTrajLink::Request& req,
                                  kuavo_msgs::SetIncrementalArmTrajLink::Response& res) {
  res.success = setTransport(req.transport);
  if (res.success) {
    using Request = kuavo_msgs::SetIncrementalArmTrajLink::Request;
    switch (req.transport) {
      case Request::TRANSPORT_SHM:
        res.message = "incremental arm traj transport set to SHM";
        break;
      case Request::TRANSPORT_KUAVO_ARM_TRAJ:
        res.message = "incremental arm traj transport set to /kuavo_arm_traj";
        break;
      default:
        res.message = "incremental arm traj transport disabled";
        break;
    }
  } else {
    res.message = "failed to set incremental arm traj transport";
  }
  return true;
}

}  // namespace HighlyDynamic
