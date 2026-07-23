#include "humanoid_controllers/ArmTrajReceiver.h"

#include <algorithm>
#include <cmath>
#include <thread>

#include <boost/make_shared.hpp>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

namespace humanoid_controller {

namespace {
using Request = kuavo_msgs::SetIncrementalArmTrajLink::Request;

/// 把退订投递到 traj CallbackQueue，避免 service 线程跨线程碰 Subscriber
class TrajUnsubCallback : public ros::CallbackInterface {
 public:
  explicit TrajUnsubCallback(ArmTrajReceiver* self) : self_(self) {}
  CallResult call() override {
    if (self_) {
      self_->unsubOnTrajQueue();
    }
    return Success;
  }

 private:
  ArmTrajReceiver* self_;
};
}  // namespace

ArmTrajReceiver::~ArmTrajReceiver() {
  pending_unsub_.store(false, std::memory_order_release);
  shm_running_.store(false, std::memory_order_release);
  if (shm_thread_.joinable()) {
    shm_thread_.join();
  }
  if (traj_spinner_) {
    traj_spinner_->stop();
    traj_spinner_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(ros_sub_mutex_);
    if (ros_sub_) {
      ros_sub_.shutdown();
    }
  }
  if (srv_) {
    srv_.shutdown();
  }
  if (shm_) {
    shm_->cleanup();
    shm_.reset();
  }
}

void ArmTrajReceiver::init(ros::NodeHandle& nh, int arm_num, TrajSink sink,
                           const std::string& set_link_service, int8_t default_transport) {
  nh_ = nh;
  arm_num_ = std::max(0, arm_num);
  sink_ = std::move(sink);
  set_link_service_ = set_link_service;
  nh.param("arm_traj_shm_stale_sec", shm_stale_sec_, 0.3);
  nh.param("/vr_ik/arm_traj_shm_stale_timeout_sec", shm_stale_sec_, shm_stale_sec_);

  traj_nh_ = ros::NodeHandle(nh_);
  traj_nh_.setCallbackQueue(&traj_callback_queue_);
  traj_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &traj_callback_queue_);
  traj_spinner_->start();

  // 测试/诊断订这一对；不双发 /kuavo_arm_traj/*
  using_shm_pub_ =
      nh_.advertise<std_msgs::Float64>("/ik_debug/arm_traj_receive/using_shm", 10, true);
  transport_pub_ =
      nh_.advertise<std_msgs::Float64>("/ik_debug/arm_traj_receive/transport", 10, true);

  shm_ = std::make_unique<kuavo_common::ArmTrajShmManager>();
  shm_capable_ = shm_->initialize(kuavo_common::ArmTrajShmManager::Role::Reader);
  if (!shm_capable_) {
    ROS_WARN("[ArmTrajReceiver] ArmTraj SHM reader init failed; TRANSPORT_SHM unavailable");
    shm_.reset();
  }

  srv_ = nh_.advertiseService(set_link_service_, &ArmTrajReceiver::handleSetLink, this);

  if (default_transport != Request::TRANSPORT_NONE &&
      default_transport != Request::TRANSPORT_KUAVO_ARM_TRAJ) {
    ROS_WARN("[ArmTrajReceiver] invalid default_transport=%d, use KUAVO_ARM_TRAJ",
             static_cast<int>(default_transport));
    default_transport = Request::TRANSPORT_KUAVO_ARM_TRAJ;
  }
  transport_.store(default_transport, std::memory_order_release);
  ensureRosSub(true);

  if (shm_capable_) {
    shm_running_.store(true, std::memory_order_release);
    shm_thread_ = std::thread(&ArmTrajReceiver::shmThread, this);
  }

  publishStatus();
  ROS_INFO("[ArmTrajReceiver] init arm_num=%d service=%s shm_capable=%d default_transport=%d",
           arm_num_, set_link_service_.c_str(), shm_capable_ ? 1 : 0,
           static_cast<int>(transport_.load(std::memory_order_acquire)));
}

void ArmTrajReceiver::publishStatus() {
  std::lock_guard<std::mutex> lock(status_pub_mutex_);
  std_msgs::Float64 using_shm_msg;
  std_msgs::Float64 transport_msg;
  const int8_t transport = transport_.load(std::memory_order_acquire);
  using_shm_msg.data = (transport == Request::TRANSPORT_SHM) ? 1.0 : 0.0;
  transport_msg.data = static_cast<double>(transport);
  using_shm_pub_.publish(using_shm_msg);
  transport_pub_.publish(transport_msg);
}

void ArmTrajReceiver::ensureRosSub(bool enable) {
  if (enable) {
    pending_unsub_.store(false, std::memory_order_release);
    std::lock_guard<std::mutex> lock(ros_sub_mutex_);
    if (!ros_sub_) {
      ros_sub_ = traj_nh_.subscribe("/kuavo_arm_traj", 10, &ArmTrajReceiver::rosCallback, this,
                                    ros::TransportHints().tcpNoDelay());
      ROS_INFO("[ArmTrajReceiver] Subscribed /kuavo_arm_traj");
    }
    return;
  }

  {
    std::lock_guard<std::mutex> lock(ros_sub_mutex_);
    if (!ros_sub_) {
      return;
    }
  }
  pending_unsub_.store(true, std::memory_order_release);
  traj_callback_queue_.addCallback(boost::make_shared<TrajUnsubCallback>(this));
}

void ArmTrajReceiver::unsubOnTrajQueue() {
  if (!pending_unsub_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  std::lock_guard<std::mutex> lock(ros_sub_mutex_);
  if (!ros_sub_) {
    return;
  }
  ros_sub_.shutdown();
  ros_sub_ = ros::Subscriber();
  ROS_INFO("[ArmTrajReceiver] Unsubscribed /kuavo_arm_traj (on traj spinner; SHM active)");
}

bool ArmTrajReceiver::shouldAcceptFrom(ArmTrajSource source) const {
  const int8_t transport = transport_.load(std::memory_order_acquire);
  if (transport == Request::TRANSPORT_SHM) {
    return source == ArmTrajSource::kIncrementalShm;
  }
  if (transport == Request::TRANSPORT_NONE || transport == Request::TRANSPORT_KUAVO_ARM_TRAJ) {
    return source == ArmTrajSource::kRosTopic;
  }
  return false;
}

void ArmTrajReceiver::ingest(ArmTrajSource source, const double* pos_rad, const double* vel_rad,
                             const double* tau, int count, uint64_t stamp_nsec) {
  if (!shouldAcceptFrom(source) || !sink_ || !pos_rad || count <= 0) {
    return;
  }
  sink_(pos_rad, vel_rad, tau, count, stamp_nsec);
}

void ArmTrajReceiver::rosCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (!msg || !shouldAcceptFrom(ArmTrajSource::kRosTopic)) {
    return;
  }
  if (static_cast<int>(msg->position.size()) < arm_num_) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajReceiver] ROS traj size %zu < arm_num %d",
                      msg->position.size(), arm_num_);
    return;
  }

  std::vector<double> pos(arm_num_, 0.0);
  std::vector<double> vel(arm_num_, 0.0);
  std::vector<double> effort(arm_num_, 0.0);
  for (int i = 0; i < arm_num_; ++i) {
    pos[i] = msg->position[i] * M_PI / 180.0;
  }
  if (static_cast<int>(msg->velocity.size()) >= arm_num_) {
    for (int i = 0; i < arm_num_; ++i) {
      vel[i] = msg->velocity[i] * M_PI / 180.0;
    }
  }
  if (static_cast<int>(msg->effort.size()) >= arm_num_) {
    for (int i = 0; i < arm_num_; ++i) {
      effort[i] = msg->effort[i];
    }
  }
  const uint64_t stamp_nsec =
      static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(msg->header.stamp.nsec);
  ingest(ArmTrajSource::kRosTopic, pos.data(), vel.data(), effort.data(), arm_num_, stamp_nsec);
}

void ArmTrajReceiver::shmThread() {
  kuavo_common::ArmTrajShmData data{};
  auto last_status = std::chrono::steady_clock::now();
  while (shm_running_.load(std::memory_order_acquire) && ros::ok()) {
    const auto now_wall = std::chrono::steady_clock::now();
    bool got_update = false;

    if (transport_.load(std::memory_order_acquire) == Request::TRANSPORT_SHM && shm_) {
      if (shm_->readIfUpdated(data)) {
        got_update = true;
        if (static_cast<int>(data.num_joints) < arm_num_) {
          ROS_WARN_THROTTLE(1.0, "[ArmTrajReceiver] SHM joints %u < arm_num %d", data.num_joints,
                            arm_num_);
        } else {
          last_shm_recv_wall_ = now_wall;
          has_last_shm_recv_ = true;
          ingest(ArmTrajSource::kIncrementalShm, data.position, data.velocity, data.effort,
                 arm_num_, data.stamp_nsec);
        }
      }
    }

    // SHM 超时只告警，不自动切链（禁止 fallback）
    if (transport_.load(std::memory_order_acquire) == Request::TRANSPORT_SHM &&
        has_last_shm_recv_) {
      const double age =
          std::chrono::duration<double>(now_wall - last_shm_recv_wall_).count();
      if (age > shm_stale_sec_) {
        ROS_ERROR_THROTTLE(1.0,
                           "[ArmTrajReceiver] SHM stale (%.3fs > %.3fs), hold last (no fallback)",
                           age, shm_stale_sec_);
      }
    }

    if (!got_update) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    if (std::chrono::duration<double>(now_wall - last_status).count() >= 1.0) {
      publishStatus();
      last_status = now_wall;
    }
  }
}

bool ArmTrajReceiver::setLink(int8_t transport, std::string* message) {
  auto set_msg = [&](const std::string& text) {
    if (message) {
      *message = text;
    }
  };

  if (transport != Request::TRANSPORT_NONE && transport != Request::TRANSPORT_SHM &&
      transport != Request::TRANSPORT_KUAVO_ARM_TRAJ) {
    set_msg("invalid transport");
    return false;
  }

  if (transport == Request::TRANSPORT_SHM) {
    if (!shm_capable_ || !shm_) {
      set_msg("incremental arm traj SHM link not available");
      return false;
    }
    transport_.store(Request::TRANSPORT_SHM, std::memory_order_release);
    has_last_shm_recv_ = false;
    ensureRosSub(false);
    set_msg("incremental arm traj transport set to SHM");
    return true;
  }

  transport_.store(transport, std::memory_order_release);
  has_last_shm_recv_ = false;
  ensureRosSub(true);
  if (transport == Request::TRANSPORT_KUAVO_ARM_TRAJ) {
    set_msg("incremental arm traj transport set to /kuavo_arm_traj");
  } else {
    set_msg("incremental arm traj transport disabled");
  }
  return true;
}

bool ArmTrajReceiver::handleSetLink(kuavo_msgs::SetIncrementalArmTrajLink::Request& req,
                                    kuavo_msgs::SetIncrementalArmTrajLink::Response& res) {
  res.success = setLink(req.transport, &res.message);
  if (res.success) {
    ROS_INFO("[ArmTrajReceiver] %s", res.message.c_str());
  } else {
    ROS_WARN("[ArmTrajReceiver] setLink failed: %s", res.message.c_str());
  }
  return true;
}

}  // namespace humanoid_controller
