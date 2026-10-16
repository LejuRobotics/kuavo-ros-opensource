#include "wheel_arm_abs_vel/humanoid_controllers/ExecutionProducerLease.h"
#include "wheel_arm_abs_vel/humanoid_controllers/ClockedReferenceFreshness.h"
#include "wheel_arm_abs_vel/humanoid_controllers/ExecutionClockIdentity.h"
#include "../velocity_ik/async_diagnostics.h"
#include "../velocity_ik/runtime_timing_probe.h"
#include "../velocity_ik/runtime_thread_partition.h"
#include "../velocity_ik/hold_diagnostics.h"
#include <iomanip>
#include "wheel_arm_abs_vel/humanoid_controllers/WholeBodyReferenceShmRos.h"
#include "../velocity_ik/elbow_feedforward.h"
#include "../velocity_ik/channel_observation.h"
#include "../velocity_ik/startup_readiness.h"
#include "../velocity_ik/continuous_reference.h"
#include "../velocity_ik/frame_prediction.h"
#include "../velocity_ik/fixed_prepare_transaction.h"
#include "../velocity_ik/sample_freshness.h"
#include "../velocity_ik/bounded_prediction.h"
#include "../velocity_ik/stop_reserve.h"
#include "../velocity_ik/exact_stop.h"
#include "../velocity_ik/operating_envelope.h"
#include "../velocity_ik/temporal_cost.h"
#include "../velocity_ik/joint_jerk.h"
#include "../velocity_ik/fixed_bandwidth_reference.h"
#include "../velocity_ik/execution_governor.h"
#include "../velocity_ik/teleop_continuity.h"
#include <nlohmann/json.hpp>
#include "wheel_arm_abs_vel/motion_capture_ik/runtime/E2eDeadline.h"
#include <algorithm>
#include <atomic>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "../velocity_ik/model_pinocchio.h"
#include "../velocity_ik/weighted_qp.h"
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <kuavo_msgs/sensorsData.h>

#include "wheel_arm_abs_vel/TeleopTcpTarget.h"
#include "wheel_arm_abs_vel/VelocityIkFeasibilityDebug.h"
#include "wheel_arm_abs_vel/VelocityIkInputDebug.h"
#include "wheel_arm_abs_vel/VelocityIkShadow.h"
#include "wheel_arm_abs_vel/motion_capture_ik/mapper_status.h"
#include "wheel_arm_abs_vel/WholeBodyIkReference.h"
#include "motion_capture_ik/json.hpp"
#include "wheel_arm_abs_vel/motion_capture_ik/whole_body_dof_layout.h"
#include "wheel_arm_abs_vel/motion_capture_ik/whole_body_motion_policy.h"
#include "wheel_arm_abs_vel/motion_capture_ik/whole_body_recruitment.h"

namespace {

using motion_capture_ik::whole_body_ik::DofLayout;
using motion_capture_ik::whole_body_ik::DofGroup;
using motion_capture_ik::whole_body_ik::ArmPressure;
using motion_capture_ik::whole_body_ik::Recruitment;
using motion_capture_ik::whole_body_ik::calculateArmPressure;
using motion_capture_ik::whole_body_ik::risingPressure;
using motion_capture_ik::whole_body_ik::kArmCount;
using motion_capture_ik::whole_body_ik::kBaseDof;
using motion_capture_ik::whole_body_ik::kDofLayoutVersion;
using motion_capture_ik::whole_body_ik::kDualArmDof;
using motion_capture_ik::whole_body_ik::kJointsPerArm;
using motion_capture_ik::whole_body_ik::kLowerBodyDof;

using DecisionVector = Eigen::VectorXd;
using Vector7d = Eigen::Matrix<double, kJointsPerArm, 1>;
using DecisionMatrix = Eigen::MatrixXd;
using DecisionJacobian3d =
    Eigen::Matrix<double, 3, Eigen::Dynamic>;
using Matrix3x7d = Eigen::Matrix<double, 3, kJointsPerArm>;
using Matrix6x7d = Eigen::Matrix<double, 6, kJointsPerArm>;
constexpr int kTorsoConstraintCount = 4;
// Sensor/decision lower-body order is [knee, leg, waist_pitch, waist_yaw].
// Behavioural admission follows physical drivability from the top down.
constexpr std::array<int, kLowerBodyDof> kLowerRecruitmentOrder{3, 2, 1, 0};

enum class ShadowState : int {
  kWaitingForInput = 0,
  kTracking = 1,
  kStaleBraking = 2,
  kHold = 3,
  kModeBlocked = 4,
  kQpError = 5,
  kSafetyBraking = 6,
  kRecovering = 7,
  kPreparing = 8,
};

enum class TrackingSourceMode : int {
  kAuto = 0,
  kController = 1,
  kHand = 2,
};

enum class FeasibilityMode : int {
  kOff = wheel_arm_abs_vel::VelocityIkFeasibilityDebug::MODE_OFF,
  kShadow = wheel_arm_abs_vel::VelocityIkFeasibilityDebug::MODE_SHADOW,
  kActive = wheel_arm_abs_vel::VelocityIkFeasibilityDebug::MODE_ACTIVE,
};

bool allFinite(const Eigen::Vector3d& value) {
  return value.array().isFinite().all();
}

Eigen::Vector3d rotationLog(const Eigen::Matrix3d& rotation) {
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || !allFinite(angle_axis.axis())) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.axis() * angle_axis.angle();
}

Eigen::Vector3d orientationError(const Eigen::Quaterniond& target,
                                 const Eigen::Quaterniond& current) {
  Eigen::Quaterniond delta = target * current.conjugate();
  if (delta.w() < 0.0) {
    delta.coeffs() *= -1.0;
  }
  delta.normalize();
  return rotationLog(delta.toRotationMatrix());
}

template <typename Derived>
double maxAbs(const Eigen::MatrixBase<Derived>& value) {
  return value.size() == 0 ? 0.0 : value.cwiseAbs().maxCoeff();
}

}  // namespace

class PinocchioVelocityIk {
 public:
  PinocchioVelocityIk() : nh_(), pnh_("~") {
    initializeDecisionStorage();
    loadParameters();
    buildModel();
    // Verified reachable families: physical 18-D tracking plus four torso
    // rows; 4-D lower-body supervisor plus the same four row identities.
    weighted_qp_.prepareWorkspaces({{18, kTorsoConstraintCount},
                                    {20, kTorsoConstraintCount + 2},
                                    {22, kTorsoConstraintCount + 4},
                                    {kLowerBodyDof, kTorsoConstraintCount}});
    diagnostic_context_=plant_->CreateDefaultContext();
    diagnostic_q_full_=plant_->GetPositions(*diagnostic_context_);
    pnh_.param("diagnostics_async", diagnostics_async_, true);
    pnh_.setParam("ik_backend", "PINOCCHIO_VELOCITY_IK_WEIGHTED_QP_V1");
    pnh_.setParam("qp_backend", "ProxQP");
    std::cout << "MODEL=PINOCCHIO IK_LEVEL=VELOCITY QP=WEIGHTED_SINGLE SOLVER=PROXQP ELBOW=XYZ_SOFT SEW=OFF HQP=OFF SQP=OFF REFERENCE=D_STYLE_TRAPEZOIDAL" << std::endl;

#ifndef VELOCITY_IK_OFFLINE_TEST
    continuity_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
        "/whole_body_velocity_ik/diagnostics/continuity_v1", 2);
    confirm_service_ = pnh_.advertiseService("confirm_reacquire",
        &PinocchioVelocityIk::confirmReacquire, this);
    terminal_stop_sub_ = nh_.subscribe<std_msgs::Bool>(
        "/stop_robot", 1, [this](const std_msgs::Bool::ConstPtr& msg) {
          if (!msg || !msg->data) return;
          double captured_ns = 0.;
          {
            // Serialize the terminal latch with formal publication. A false
            // event or a later target cannot reactivate this process.
            std::lock_guard<std::mutex> lock(terminal_publication_mutex_);
            if (terminal_stop_requested_.exchange(true)) return;
            captured_ns = monotonicNs();
          }
          ROS_WARN("[VelocityIk] terminal stop latched monotonic_ns=%.0f; "
                   "formal publication disabled; restart required", captured_ns);
        }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    reference_transport_.configure(nh_,formal_reference_topic_,true);
    pnh_.setParam("reference_transport_active",reference_transport_.shm()?"shm":"ros");
    if (!reference_transport_.shm())
      execution_owner_sub_ = nh_.subscribe<wheel_arm_abs_vel::WholeBodyIkReference>(
          formal_reference_topic_ + "/execution_owner",2,
          [this](const wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr& msg) {
            acceptExecutionFeedback(msg);
          },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
    target_sub_ = nh_.subscribe(target_topic_, 1, &PinocchioVelocityIk::targetCallback, this, ros::TransportHints().tcpNoDelay());
    robot_state_sub_ = nh_.subscribe(
        robot_state_topic_, 2,
        &PinocchioVelocityIk::robotStateCallback, this,
        ros::TransportHints().tcpNoDelay());
    if (enable_base_motion_) {
      base_state_sub_ = nh_.subscribe(base_state_topic_, 2, &PinocchioVelocityIk::baseStateCallback, this);
    } else {
      ROS_INFO("[VelocityIkShadow] Fixed-base mode does not require an odometry publisher; base x/y/yaw remain hard locked at the reference origin");
    }
    mode_sub_ = nh_.subscribe(mode_topic_, 2, &PinocchioVelocityIk::modeCallback, this);

    cycle_timing_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/cycle_timing", 32);
    bool runtime_probe_enabled=true;
    pnh_.param("runtime_probe_enabled",runtime_probe_enabled,true);
    if(runtime_probe_enabled) {
      bool audit=false, detail_enabled=true;
      int stride=10;
      pnh_.param("communication_full_rate_audit",audit,false);
      pnh_.param("runtime_probe_detail_stride",stride,10);
      pnh_.param("runtime_probe_detail_enabled",detail_enabled,true);
      if(audit) { stride=1; detail_enabled=true; }
      if(stride<1)throw std::runtime_error("runtime_probe_detail_stride must be positive");
      const auto probe_pub=nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_+"/ik_runtime_probe_v1",32);
      const auto summary_pub=nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_+"/ik_runtime_window_v1",8);
      runtime_probe_.start([probe_pub,summary_pub,stride,detail_enabled,
                           window=velocity_ik::runtime_probe::WindowStats{}]
                          (const velocity_ik::runtime_probe::Record& r) mutable {
        window.merge(r.window);  // Includes every cycle, before detail decimation.
        if(!window.cycles)return;
        if(window.cycles<static_cast<uint64_t>(stride) && r.size!=0)return;
        std_msgs::Float64MultiArray summary;
        summary.data={1.,r.serial,r.session,r.generation,r.reference,double(window.cycles),
            window.wall_max,window.cpu_max,window.voluntary,window.involuntary,
            window.minor,window.major,double(window.overflow),r.dropped};
        summary_pub.publish(summary); window={};
        // No detail allocation/fill on skipped cycles or with details disabled.
        if(!detail_enabled || r.size==0)return;
        std_msgs::Float64MultiArray msg;
        msg.data.reserve(10+7*r.size);
        msg.data={1.,r.pid,r.tid,r.serial,r.session,r.generation,r.reference,r.dropped,double(r.size),r.overflow?1.:0.};
        for(size_t i=0;i<r.size;++i) {
          const auto& p=r.points[i];
          msg.data.insert(msg.data.end(),{p.id,p.wall,p.cpu,p.voluntary,p.involuntary,p.minor,p.major});
        }
        probe_pub.publish(msg);
      });
    }

    qp_stats_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/qp_stats", 4);
    execution_governor_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
        output_prefix_ + "/execution_governor_v1", 8);
    if (envelope_diagnostics_enabled_)
      envelope_diagnostics_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
          output_prefix_ + "/operating_diagnostics_v1", 8);
    if(joint_jerk_enabled_)joint_jerk_pub_=nh_.advertise<std_msgs::Float64MultiArray>(
        output_prefix_+"/joint_jerk_v1",8);
    q_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/q", 2);
    dq_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/dq", 2);
    ddq_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/ddq", 2);
    task_dq_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/task_dq", 2);
    status_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/status", 2);
    task_error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/task_error", 2);
    channel_observation_pub_=nh_.advertise<std_msgs::Float64MultiArray>("/whole_body_velocity_ik/diagnostics/channel_observation",10);
    target_ff_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/target_twist_ff", 2);
    adaptive_reference_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/adaptive_reference_v1", 2);
    reference_dynamics_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/reference_dynamics_v1", 2);
    innovation_reference_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/innovation_reference_v1", 2);
    startup_readiness_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_prefix_ + "/startup_readiness_v1", 2);
    reconstructed_target_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/whole_body_velocity_ik/diagnostics/reconstructed_target", 10);
    if (publish_input_debug_) {
      input_debug_pub_ = nh_.advertise<wheel_arm_abs_vel::VelocityIkInputDebug>(
          output_prefix_ + "/input_debug", 10);
    }
    if (feasibility_mode_ != FeasibilityMode::kOff) {
      feasibility_debug_pub_ =
          nh_.advertise<wheel_arm_abs_vel::VelocityIkFeasibilityDebug>(
              output_prefix_ + "/feasibility_debug", 2);
    }
    reference_pub_ = nh_.advertise<wheel_arm_abs_vel::VelocityIkShadow>(output_prefix_ + "/reference", 2);
    mapper_status_pub_ = nh_.advertise<wheel_arm_abs_vel::VelocityIkMapperStatus>(
        output_prefix_ + "/mapper_status", 2);
    if (enable_takeover_) {
      formal_reference_pub_ =
          nh_.advertise<wheel_arm_abs_vel::WholeBodyIkReference>(
              reference_transport_.shm()?formal_reference_topic_+"/shm_mirror":formal_reference_topic_, 2);
    }
    ROS_INFO_STREAM("[VelocityIkShadow] ready: " << solver_rate_hz_ << " Hz reference supervisor, target=" << target_topic_
                                                  << ", robot_state=" << robot_state_topic_
                                                  << "[" << sensor_arm_start_index_ << ":"
                                                  << sensor_arm_start_index_ + kDualArmDof << "]"
                                                  << ", full_layout_dof="
                                                  << dof_layout_.fullDofCount()
                                                  << ", decision_dof="
                                                  << decisionDofCount()
                                                  << ", base_state="
                                                  << base_state_topic_
                                                  << ", world_frame="
                                                  << world_frame_name_
                                                  << ", session_frame="
                                                  << session_frame_name_
                                                  << ", output=" << output_prefix_
                                                  << ", tracking_source_mode="
                                                  << tracking_source_mode_name_
                                                  << ", feasibility_mode="
                                                  << feasibility_mode_name_
                                                  << ", takeover="
                                                  << (enable_takeover_ ? "enabled" : "disabled")
                                                  << ", formal_reference="
                                                  << formal_reference_topic_);
#endif
  }

  ~PinocchioVelocityIk() {
    stopFeasibilityWorker();
    runtime_probe_.stop();
    diagnostics_.stop();
  }

  void run() {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    startFeasibilityWorker();
    velocity_ik::configureIkThread();
    ros::WallRate rate(solver_rate_hz_);
    kuavo_e2e::Deadline deadline(0);
    uint64_t runtime_probe_serial=0;
    while (ros::ok()) {
      velocity_ik::runtime_probe::Scope runtime_scope(runtime_probe_,++runtime_probe_serial);
      if (deadline.enabled()) deadline.wait();
      velocity_ik::runtime_probe::point(2);
      publishOnce();
      velocity_ik::runtime_probe::point(94);
      runtime_scope.reference(reference_sequence_);
      if (!deadline.enabled()) rate.sleep();
      velocity_ik::runtime_probe::point(96);
    }
    stopFeasibilityWorker();
  }

 private:
  friend struct VelocityIkOfflineTest;
  struct ArmTarget {
    Eigen::Vector3d hand_position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond hand_orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d elbow_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d hand_linear_ff{Eigen::Vector3d::Zero()};
    Eigen::Vector3d hand_angular_ff{Eigen::Vector3d::Zero()};
    Eigen::Vector3d elbow_linear_ff{Eigen::Vector3d::Zero()};
    bool elbow_valid{false};
  };

  struct WorkspaceProjection {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d velocity_map{Eigen::Matrix3d::Identity()};
    double raw_distance{0.0};
    double feasible_distance{0.0};
    double projection_distance{0.0};
  };

  struct HumanBodySample {
    Eigen::Vector3d root_position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond root_orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d chest_position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond chest_orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d head_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d left_shoulder_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d right_shoulder_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d root_linear_ff{Eigen::Vector3d::Zero()};
    Eigen::Vector3d chest_linear_ff{Eigen::Vector3d::Zero()};
    Eigen::Vector3d chest_angular_ff{Eigen::Vector3d::Zero()};
    std::string frame_id;
    uint64_t coordinate_epoch{0};
    bool root_valid{false};
    bool chest_valid{false};
    bool head_valid{false};
    bool left_shoulder_valid{false};
    bool right_shoulder_valid{false};
    bool high_confidence{false};

    bool torsoValid() const {
      // Optical confidence is metadata, not torso admission or continuity.
      return chest_valid;
    }
  };

  struct BodyIntent {
    Eigen::Vector2d root_translation{Eigen::Vector2d::Zero()};
    Eigen::Vector2d desired_base_translation{Eigen::Vector2d::Zero()};
    Eigen::Vector4d desired_torso_feature{Eigen::Vector4d::Zero()};
    Eigen::Vector3d mirror_translation{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d mirror_rotation{Eigen::Matrix3d::Identity()};
    Eigen::Vector2d desired_base_velocity{Eigen::Vector2d::Zero()};
    Eigen::Vector4d desired_torso_velocity{Eigen::Vector4d::Zero()};
    double root_translation_confidence{0.0};
    // relative_hand_motion is diagnostic only. Reach assistance is selected
    // from the robot arm's geometric reach/joint/singularity pressure, never
    // from how far the human moved since pressing the trigger.
    double relative_hand_motion{0.0};
    double reach_assist_demand{0.0};
    bool torso_valid{false};
  };

  struct TargetSnapshot {
    bool estimated_twists_valid=false;
    // Estimated acquisition instant in the local monotonic clock, one per
    // controlled hand and shared torso. Invalid/duplicate frames never renew it.
    std::array<double, 3> observation_ns{{0., 0., 0.}};
    uint64_t observation_sequence=0;
    std::array<ArmTarget, kArmCount> arm;
    std::array<velocity_ik::ChannelObservation, kArmCount> hand_observation, elbow_observation;
    uint64_t body_observation_updates=0;
    double body_observation_host_ns=0, body_observation_dt=0;
    bool body_observation_changed=false;
    // The normal hand task is expressed in the robot torso frame.  A separate
    // world target exists only for explicit geometric reach assistance; it is
    // never used by ordinary Root/Chest body mirroring.
    std::array<ArmTarget, kArmCount> assist_arm_world;
    HumanBodySample body;
    BodyIntent body_intent;
    Eigen::Matrix3d body_mirror_rotation_world{Eigen::Matrix3d::Identity()};
    ros::WallTime receive_time;
    ros::Time source_stamp;
    double source_relative_age{0.0};
    ros::WallTime feedforward_sample_receive_time;
    ros::Time feedforward_sample_stamp;
    ros::WallTime body_feedforward_sample_receive_time;
    ros::Time body_feedforward_sample_stamp;
    std::string frame_id;
    uint8_t source{wheel_arm_abs_vel::TeleopTcpTarget::SOURCE_UNKNOWN};
    uint64_t session_id{0};
    uint64_t generation{0};
    uint8_t absolute_state{0};
    uint8_t control_state{wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_INACTIVE};
    bool valid{false};
    bool arm_target_torso_relative{false};
    bool feedforward_sample_valid{false};
    bool body_feedforward_sample_valid{false};
    uint64_t sequence{0};
    uint64_t unique_sequence{0};
    double receive_monotonic_ns=0;
    uint32_t source_frame_seq=0;
  };

  struct SessionBodyAnchor {
    HumanBodySample human;
    std::array<Eigen::Vector3d, kArmCount> neutral_hand_position{
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector4d robot_torso_feature{Eigen::Vector4d::Zero()};
    Eigen::Vector3d robot_torso_pivot_session{Eigen::Vector3d::Zero()};
    Eigen::Isometry3d robot_torso_pose_session{Eigen::Isometry3d::Identity()};
    Eigen::Vector2d accumulated_root_translation{Eigen::Vector2d::Zero()};
    Eigen::Vector4d accumulated_torso_delta{Eigen::Vector4d::Zero()};
    uint64_t session_id{0};
    uint64_t generation{0};
    uint64_t coordinate_epoch{0};
    bool valid{false};
  };

  struct SolverRequest {
    bool catchup_correction=false;
    double acquisition_blend=1.;
    double input_budget=1.;
    TargetSnapshot target;
    DecisionVector reference_q;
    DecisionVector reference_dq;
    DecisionVector reference_ddq;
    // FK/tasks stay at reference_q. Measured state is used by the execution
    // governor, never substituted into the continuous reference trajectory.
    DecisionVector measured_q;
    DecisionVector measured_dq;
    std::array<double, kArmCount> arm_tracking_error{0.0, 0.0};
    bool measured_state_valid{false};
    std::array<double, kArmCount> governor_risk{0., 0.};
    uint64_t base_reference_seq{0};
    double cycle_dt{0.002};
    double target_age{std::numeric_limits<double>::infinity()};
    double feedforward_age{std::numeric_limits<double>::infinity()};
    double body_feedforward_age{std::numeric_limits<double>::infinity()};
    double lower_body_recruitment{0.0};
    std::array<double, kLowerBodyDof> lower_joint_recruitment{
        0.0, 0.0, 0.0, 0.0};
    double base_recruitment{0.0};
    std::array<double, kBaseDof> base_position_lower{0.0, 0.0, 0.0};
    std::array<double, kBaseDof> base_position_upper{0.0, 0.0, 0.0};
    bool base_session_bounds_valid{false};
    bool tracking{false};
  };

  struct FeasibilityRequest {
    TargetSnapshot target;
    DecisionVector reference_q;
    uint64_t reference_seq{0};
    ros::WallTime enqueue_time;
    bool tracking_active{false};
  };

  struct SessionAnchor {
    Eigen::Vector2d translation{Eigen::Vector2d::Zero()};
    double yaw{0.0};
    uint64_t session_id{0};
    uint64_t generation{0};
    uint64_t base_coordinate_epoch{0};
    bool valid{false};
  };

  struct SessionTargetAlignment {
    // Per-arm rigid calibration from the frozen, chest-neutral Quest/session
    // mapping frame into the robot's shoulder-bearing torso frame.  Capturing
    // and applying the complete transform in the same frame avoids the former
    // bootstrap-time double rotation of world-coordinate offsets.
    std::array<Eigen::Isometry3d, kArmCount> torso_from_source{
        Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
    uint64_t session_id{0};
    uint64_t generation{0};
    bool valid{false};
  };

  struct ReferenceHistorySample {
    ros::WallTime stamp;
    DecisionVector q;
  };

  struct SolverCandidate {
    bool joint_jerk_active=false, joint_jerk_retry=false;
    bool diagnostics_due=false;
    int joint_jerk_first_code=0;
    // Per arm row: decision index, requested J, previous/next a, actual J,
    // excess, safety-relaxed flag, accepted lower/upper velocity.
    std::array<double,126> joint_jerk_diagnostics{};
    std::array<double, kArmCount> governor_risk{0., 0.};
    std::vector<double> governor_diagnostics;
    double continuity_relaxation=0.;
    // Separate diagnostic ABI; never copied into the execution reference.
    std::vector<double> operating_diagnostics;
    // Final next-cycle velocity produced inside the common q/dq/ddq domain.
    // The publisher performs only a freshness/feasibility check; it does not
    // reshape this task with another trajectory state.
    DecisionVector task_dq;
    DecisionVector primary_dq;
    DecisionVector secondary_dq;
    DecisionMatrix primary_task_jacobian;
    DecisionVector primary_task_velocity;
    DecisionMatrix body_task_jacobian;
    DecisionVector body_task_velocity;
    // Exact dynamic domain used by the synchronous solve. A candidate is
    // committed only against this certificate; commit must not rebuild a
    // subtly different q/dq/ddq or torso domain.
    DecisionVector certified_dynamic_lower;
    DecisionVector certified_dynamic_upper;
    DecisionMatrix certified_constraint_matrix;
    DecisionVector certified_constraint_lower;
    DecisionVector certified_constraint_upper;
    bool dynamic_certificate_valid{false};
    std::array<double, kArmCount> sigma_min{0.0, 0.0};
    std::array<double, kArmCount> translation_sigma{0.0, 0.0};
    std::array<bool, kArmCount> singularity_escape_constraint_active{
        false, false};
    bool singularity_escape_constraint_fallback{false};
    std::array<double, kArmCount> primary_regularization{0.0, 0.0};
    std::array<double, kArmCount> orientation_task_scale{1.0, 1.0};
    std::array<double, kArmCount> position_task_weight_scale{1.0, 1.0};
    std::array<ArmPressure, kArmCount> arm_pressure{};
    Recruitment recruitment_target{};
    std::array<double, kLowerBodyDof> lower_joint_recruitment_target{
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kLowerBodyDof> lower_joint_directional_effectiveness{
        0.0, 0.0, 0.0, 0.0};
    double geometric_reach_demand{0.0};
    double arm_only_task_residual{0.0};
    double arm_lower_task_residual{0.0};
    double lower_body_saturation{0.0};
    double primary_task_scale{1.0};
    std::array<double, kArmCount> translation_task_scale{1.0, 1.0};
    double solver_lower_body_recruitment{0.0};
    std::array<double, kLowerBodyDof> solver_lower_joint_recruitment{
        0.0, 0.0, 0.0, 0.0};
    double solver_base_recruitment{0.0};
    int primary_task_rank{0};
    int nullspace_dimension{0};
    double nullspace_residual{0.0};
    double primary_task_leakage{0.0};
    int secondary_return_code{-1100};
    std::array<double, 6> task_error{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ros::WallTime finish_time;
    uint64_t base_reference_seq{0};
    uint64_t session_id{0};
    uint64_t generation{0};
    uint64_t target_seq{0};
    double solve_time_us{0.0};
    double torso_viability_relaxation{0.0};
    double torso_task_scale{0.0};
    double base_intent_task_scale{0.0};
    double torso_task_error{0.0};
    double base_intent_error{0.0};
    bool root_translation_intent_active{false};
    bool emergency_acceleration_used{false};
    bool solver_valid{false};
    int return_code{0};
  };

  struct HardBounds {
    DecisionVector lower;
    DecisionVector upper;
    DecisionVector position_velocity_lower, position_velocity_upper;
    DecisionVector stopping_lower, stopping_upper;
    bool emergency_acceleration_used{false};
    bool valid{false};
  };

  struct LinearConstraints {
    DecisionMatrix matrix;
    DecisionVector lower;
    DecisionVector upper;
    std::array<double, kTorsoConstraintCount> value{0.0, 0.0, 0.0,
                                                    0.0};
    std::array<double, kTorsoConstraintCount> position_lower{
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kTorsoConstraintCount> position_upper{
        0.0, 0.0, 0.0, 0.0};
    bool valid{false};
  };

  struct SingularityConstraintSet {
    LinearConstraints linear;
    std::array<bool, kArmCount> active{false, false};
  };

  int decisionDofCount() const { return dof_layout_.decisionDofCount(); }

  int armDecisionIndex(int arm, int joint) const {
    return dof_layout_.armDecisionIndex(arm, joint);
  }

  int lowerBodyDecisionIndex(int joint) const {
    return lower_body_decision_indices_.at(static_cast<std::size_t>(joint));
  }

  int baseDecisionIndex(int dof) const {
    return base_decision_indices_.at(static_cast<std::size_t>(dof));
  }

  int baseDofForDecisionIndex(int decision_index) const {
    for (int dof = 0; dof < kBaseDof; ++dof) {
      if (baseDecisionIndex(dof) == decision_index) {
        return dof;
      }
    }
    throw std::out_of_range("decision index does not belong to the base");
  }

  int lowerBodyJointForDecisionIndex(int decision_index) const {
    for (int joint = 0; joint < kLowerBodyDof; ++joint) {
      if (lowerBodyDecisionIndex(joint) == decision_index) {
        return joint;
      }
    }
    throw std::out_of_range(
        "decision index does not belong to the lower body");
  }

  bool isDecisionGroup(int decision_index, DofGroup group) const {
    return dof_layout_.decisionSpec(decision_index).group == group;
  }

  void lockBaseVelocity(DecisionVector* velocity) const {
    for (int dof = 0; dof < kBaseDof; ++dof) {
      (*velocity)(baseDecisionIndex(dof)) = 0.0;
    }
  }

  void synchronizeExogenousBasePosition(
      const DecisionVector& measured_q, DecisionVector* reference_q) const {
    for (int dof = 0; dof < kBaseDof; ++dof) {
      (*reference_q)(baseDecisionIndex(dof)) =
          measured_q(baseDecisionIndex(dof));
    }
  }

  std::array<double, 4> groupVelocityNorm(
      const DecisionVector& velocity) const {
    std::array<double, 4> squared_norm{0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < decisionDofCount(); ++i) {
      const auto group = static_cast<std::size_t>(
          dof_layout_.decisionSpec(i).group);
      squared_norm.at(group) += velocity(i) * velocity(i);
    }
    for (double& value : squared_norm) {
      value = std::sqrt(value);
    }
    return squared_norm;
  }

  void populateBaseSessionBounds(SolverRequest* request) const {
    request->base_session_bounds_valid = session_anchor_.valid;
    if (!request->base_session_bounds_valid) {
      return;
    }
    const std::array<double, kBaseDof> center{
        session_anchor_.translation.x(), session_anchor_.translation.y(),
        session_anchor_.yaw};
    const std::array<double, kBaseDof> half_range{
        base_session_x_half_range_, base_session_y_half_range_,
        base_session_yaw_half_range_};
    for (int dof = 0; dof < kBaseDof; ++dof) {
      request->base_position_lower.at(dof) =
          center.at(dof) - half_range.at(dof);
      request->base_position_upper.at(dof) =
          center.at(dof) + half_range.at(dof);
    }
  }

  Matrix3x7d selectArmColumns(const DecisionJacobian3d& jacobian,
                              int arm) const {
    Matrix3x7d selected;
    for (int joint = 0; joint < kJointsPerArm; ++joint) {
      selected.col(joint) = jacobian.col(armDecisionIndex(arm, joint));
    }
    return selected;
  }

  void addArmValues(int arm, const Vector7d& value,
                    DecisionVector* target) const {
    for (int joint = 0; joint < kJointsPerArm; ++joint) {
      (*target)(armDecisionIndex(arm, joint)) += value(joint);
    }
  }

  void initializeDecisionStorage() {
    const int dof = decisionDofCount();
    const std::vector<int> base_indices =
        dof_layout_.decisionIndices(DofGroup::kBase);
    if (base_indices.size() != static_cast<std::size_t>(kBaseDof)) {
      throw std::logic_error(
          "whole-body stage must enable three planar base DoFs");
    }
    std::copy(base_indices.begin(), base_indices.end(),
              base_decision_indices_.begin());
    const std::vector<int> lower_body_indices =
        dof_layout_.decisionIndices(DofGroup::kLowerBody);
    if (lower_body_indices.size() !=
        static_cast<std::size_t>(kLowerBodyDof)) {
      throw std::logic_error(
          "lower-body stage must enable four lower-body joints");
    }
    std::copy(lower_body_indices.begin(), lower_body_indices.end(),
              lower_body_decision_indices_.begin());
    decision_position_index_.assign(dof, -1);
    decision_velocity_index_.assign(dof, -1);
    decision_sensor_index_.assign(dof, -1);
    measured_q_ = DecisionVector::Zero(dof);
    measured_dq_ = DecisionVector::Zero(dof);
    reference_q_ = DecisionVector::Zero(dof);
    reference_dq_ = DecisionVector::Zero(dof);
    reference_ddq_ = DecisionVector::Zero(dof);
    position_lower_ = DecisionVector::Constant(dof, -M_PI);
    position_upper_ = DecisionVector::Constant(dof, M_PI);
    joint_safe_range_ratio_ = DecisionVector::Constant(dof, 0.80);
    joint_safe_center_requested_ = DecisionVector::Zero(dof);
    joint_safe_center_ = DecisionVector::Zero(dof);
    joint_safe_lower_ = DecisionVector::Zero(dof);
    joint_safe_upper_ = DecisionVector::Zero(dof);
    posture_position_ = DecisionVector::Zero(dof);
    session_posture_position_ = DecisionVector::Zero(dof);
    preparation_position_ = DecisionVector::Zero(dof);
    startup_preparation_position_ = DecisionVector::Zero(dof);
    velocity_limit_ = DecisionVector::Constant(dof, 4.0);
    acceleration_limit_ = DecisionVector::Constant(dof, 50.0);
  }

  std::vector<double> buildDecisionDefault(
      const std::array<double, kBaseDof>& base_values,
      const std::array<double, kLowerBodyDof>& lower_body_values,
      const std::array<double, kJointsPerArm>& arm_values) const {
    std::vector<double> result(
        static_cast<std::size_t>(decisionDofCount()),
        std::numeric_limits<double>::quiet_NaN());
    for (int dof = 0; dof < kBaseDof; ++dof) {
      result.at(static_cast<std::size_t>(baseDecisionIndex(dof))) =
          base_values.at(static_cast<std::size_t>(dof));
    }
    for (int joint = 0; joint < kLowerBodyDof; ++joint) {
      result.at(static_cast<std::size_t>(lowerBodyDecisionIndex(joint))) =
          lower_body_values.at(static_cast<std::size_t>(joint));
    }
    for (int arm = 0; arm < kArmCount; ++arm) {
      for (int joint = 0; joint < kJointsPerArm; ++joint) {
        result.at(static_cast<std::size_t>(armDecisionIndex(arm, joint))) =
            arm_values.at(static_cast<std::size_t>(joint));
      }
    }
    if (!std::all_of(result.begin(), result.end(),
                     [](double value) { return std::isfinite(value); })) {
      throw std::logic_error(
          "group defaults do not cover the active decision layout");
    }
    return result;
  }

  template <typename Message>
  void populateDecisionLayout(Message* message) const {
    message->dof_layout_version = kDofLayoutVersion;
    message->dof_names = dof_layout_.decisionNames();
    message->dof_groups = dof_layout_.decisionGroups();
  }

  void resizeFeasibilityDecisionFields(
      wheel_arm_abs_vel::VelocityIkFeasibilityDebug* message) const {
    const std::size_t dof = static_cast<std::size_t>(decisionDofCount());
    message->joint_lower_margin.resize(dof);
    message->joint_upper_margin.resize(dof);
    message->joint_margin_ratio.resize(dof);
    message->joint_safe_center.resize(dof);
    message->joint_safe_lower.resize(dof);
    message->joint_safe_upper.resize(dof);
    message->joint_safe_range_ratio.resize(dof);
    message->joint_safe_margin_ratio.resize(dof);
    message->joint_outward_velocity_scale.resize(dof);
    message->singular_escape_direction.resize(dof);
    message->limit_barrier_direction.resize(dof);
  }

  void resizeReferenceDecisionFields(
      wheel_arm_abs_vel::VelocityIkShadow* message) const {
    const std::size_t dof = static_cast<std::size_t>(decisionDofCount());
    message->q.resize(dof);
    message->measured_q.resize(dof);
    message->dq.resize(dof);
    message->ddq.resize(dof);
    message->task_dq.resize(dof);
    message->primary_dq.resize(dof);
    message->secondary_dq.resize(dof);
    message->jerk_slack.resize(dof);
    message->boundary_brake.resize(dof);
    message->operational_boundary_brake.resize(dof);
  }

  void loadParameters() {
    pnh_.param("robot_version", robot_version_, 63);
    pnh_.param("ctrl_arm_idx", control_arm_index_, 2);
    pnh_.param("solver_rate_hz", solver_rate_hz_, 500.0);
    pnh_.param("target_topic", target_topic_, std::string("/mm/teleop_tcp_target"));
    pnh_.param("robot_state_topic", robot_state_topic_, std::string("/sensors_data_raw"));
    pnh_.param("base_state_topic", base_state_topic_, std::string("/odom"));
    pnh_.param("world_frame", world_frame_name_, std::string("odom"));
    pnh_.param("session_frame", session_frame_name_,
               std::string("ik_session_base"));
    pnh_.param("strict_base_state_frame", strict_base_state_frame_, true);
    pnh_.param("allow_legacy_base_link_session_frame",
               allow_legacy_base_link_session_frame_, true);
    pnh_.param("sensor_arm_start_index", sensor_arm_start_index_, -1);
    pnh_.param("mode_topic", mode_topic_, std::string("/humanoid_wheel/arm_control_mode_status"));
    pnh_.param("output_prefix", output_prefix_, std::string("/velocity_ik_shadow"));
    pnh_.param("enable_takeover", enable_takeover_, false);
    pnh_.param("formal_reference_topic", formal_reference_topic_,
               std::string("/whole_body_velocity_ik/reference"));
    pnh_.param("require_external_arm_mode", require_external_arm_mode_, false);
    pnh_.param("required_arm_mode", required_arm_mode_, 2);
    pnh_.param("strict_target_frame", strict_target_frame_, true);
    pnh_.param("tracking_source_mode", tracking_source_mode_name_, std::string("controller"));
    std::transform(
        tracking_source_mode_name_.begin(), tracking_source_mode_name_.end(),
        tracking_source_mode_name_.begin(),
        [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    if (tracking_source_mode_name_ == "auto") {
      tracking_source_mode_ = TrackingSourceMode::kAuto;
    } else if (tracking_source_mode_name_ == "controller") {
      tracking_source_mode_ = TrackingSourceMode::kController;
    } else if (tracking_source_mode_name_ == "hand" ||
               tracking_source_mode_name_ == "hand_tracking") {
      tracking_source_mode_ = TrackingSourceMode::kHand;
      tracking_source_mode_name_ = "hand";
    } else {
      throw std::runtime_error(
          "~tracking_source_mode must be one of auto|controller|hand, got " +
          tracking_source_mode_name_);
    }
    pnh_.param("publish_input_debug", publish_input_debug_, false);
    pnh_.param("feasibility_mode", feasibility_mode_name_, std::string("active"));
    std::transform(
        feasibility_mode_name_.begin(), feasibility_mode_name_.end(),
        feasibility_mode_name_.begin(),
        [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    if (feasibility_mode_name_ == "off") {
      feasibility_mode_ = FeasibilityMode::kOff;
    } else if (feasibility_mode_name_ == "shadow") {
      feasibility_mode_ = FeasibilityMode::kShadow;
    } else if (feasibility_mode_name_ == "active") {
      feasibility_mode_ = FeasibilityMode::kActive;
    } else {
      throw std::runtime_error(
          "~feasibility_mode must be one of off|shadow|active, got " +
          feasibility_mode_name_);
    }
    pnh_.param("nominal_arm_reach", nominal_arm_reach_, 0.70937088);
    pnh_.param("selected_reach_reserve", selected_reach_reserve_, 0.03);
    pnh_.param("workspace_projection_smoothing",
               workspace_projection_smoothing_, 0.005);
    pnh_.param("feasibility_characteristic_length",
               feasibility_characteristic_length_, 0.30);
    pnh_.param("feasibility_singularity_approach_threshold",
               feasibility_singularity_approach_threshold_, 0.035);
    pnh_.param("feasibility_singularity_escape_threshold",
               feasibility_singularity_escape_threshold_, 0.015);
    pnh_.param("enable_singularity_escape_constraints",
               enable_singularity_escape_constraints_, true);
    pnh_.param("translation_singularity_activation_sigma",
               translation_singularity_activation_sigma_, 0.060);
    pnh_.param("translation_singularity_safe_sigma",
               translation_singularity_safe_sigma_, 0.035);
    pnh_.param("translation_singularity_escape_sigma",
               translation_singularity_escape_sigma_, 0.015);
    pnh_.param("translation_singularity_gain",
               translation_singularity_gain_, 3.0);
    pnh_.param("singularity_gradient_step",
               singularity_gradient_step_, 0.002);
    pnh_.param("singularity_gradient_min_norm",
               singularity_gradient_min_norm_, 0.001);
    pnh_.param("natural_escape_speed", natural_escape_speed_, 0.40);
    pnh_.param("feasibility_joint_limit_approach_ratio",
               feasibility_joint_limit_approach_ratio_, 0.15);
    pnh_.param("feasibility_joint_limit_escape_ratio",
               feasibility_joint_limit_escape_ratio_, 0.05);
    pnh_.param("feasibility_escape_lookahead", feasibility_escape_lookahead_, 0.02);
    pnh_.param("feasibility_escape_joint_margin_weight",
               feasibility_escape_joint_margin_weight_, 0.05);
    pnh_.param("joint_limit_barrier_weight", joint_limit_barrier_weight_, 0.40);
    pnh_.param("joint_limit_barrier_gain", joint_limit_barrier_gain_, 0.80);
    pnh_.param("joint_safe_range_ratio", joint_safe_range_ratio_default_, 0.80);
    pnh_.param("lower_body_safe_range_ratio",
               lower_body_safe_range_ratio_default_, 0.60);
    pnh_.param("use_legacy_shoulder_pitch_cap",
               use_legacy_shoulder_pitch_cap_, false);
    pnh_.param("legacy_shoulder_pitch_upper",
               legacy_shoulder_pitch_upper_, 0.3);
    std::vector<double> reserve_candidates;
    if (!pnh_.getParam("reach_reserve_candidates", reserve_candidates) ||
        reserve_candidates.size() != feasibility_reach_reserve_candidates_.size()) {
      reserve_candidates = {0.03, 0.04, 0.05};
    }
    for (std::size_t i = 0; i < feasibility_reach_reserve_candidates_.size(); ++i) {
      feasibility_reach_reserve_candidates_.at(i) =
          std::max(0.0, reserve_candidates.at(i));
    }
    pnh_.param("duplicate_position_epsilon", duplicate_position_epsilon_, 1.0e-10);
    pnh_.param("duplicate_orientation_epsilon", duplicate_orientation_epsilon_, 1.0e-10);
    pnh_.param("left_controlled_frame", left_controlled_frame_, std::string("left_teleop_tcp"));
    pnh_.param("right_controlled_frame", right_controlled_frame_, std::string("right_teleop_tcp"));
    pnh_.param("body_tracking_frame", body_tracking_frame_name_,
               std::string("quest_tracking_robot_axes"));
    left_tcp_translation_ = loadVector3Parameter("left_tcp_translation", Eigen::Vector3d::Zero());
    right_tcp_translation_ = loadVector3Parameter("right_tcp_translation", Eigen::Vector3d::Zero());
    left_tcp_rpy_ = loadVector3Parameter("left_tcp_rpy", Eigen::Vector3d::Zero());
    right_tcp_rpy_ = loadVector3Parameter("right_tcp_rpy", Eigen::Vector3d::Zero());

    pnh_.param("target_timeout", target_timeout_, 0.15);
    pnh_.param("automatic_hold_enabled", automatic_hold_enabled_, true);
    ROS_INFO("[RuntimeHoldPolicy] automatic_hold_enabled=%s; startup readiness, explicit stop, hard bounds, feedback and execution ownership remain required",
        automatic_hold_enabled_ ? "true" : "false");
    pnh_.param("continuity/hold_on_timeout", input_continuity_.config.hold_on_timeout, true);
    pnh_.param("continuity/grace", input_continuity_.config.grace, .060);
    pnh_.param("continuity/brake", input_continuity_.config.brake, .150);
    pnh_.param("continuity/lost", input_continuity_.config.lost, .500);
    pnh_.param("continuity/recovery", input_continuity_.config.recovery, .100);
    pnh_.param("continuity/recovery_frames", input_continuity_.config.recovery_frames, 3);
    pnh_.param("acquisition/joint_speed", acquisition_joint_speed_, .80);
    pnh_.param("acquisition/linear_speed", acquisition_linear_speed_, .40);
    pnh_.param("acquisition/angular_speed", acquisition_angular_speed_, 1.20);
    pnh_.param("acquisition/acceleration", acquisition_acceleration_, 3.00);
    pnh_.param("acquisition/position_tolerance", acquisition_position_tolerance_, .03);
    pnh_.param("acquisition/rotation_tolerance", acquisition_rotation_tolerance_, .15);
    pnh_.param("acquisition/settle", acquisition_.settle, .10);
    pnh_.param("acquisition/release", acquisition_.release, 1.);
    if (!input_continuity_.config.valid()) throw std::runtime_error("invalid continuity timing");
    for (double x : {acquisition_joint_speed_, acquisition_linear_speed_,
         acquisition_angular_speed_, acquisition_acceleration_, acquisition_position_tolerance_,
         acquisition_rotation_tolerance_, acquisition_.settle, acquisition_.release})
      if (!std::isfinite(x) || x<=0) throw std::runtime_error("invalid acquisition limit");
    target_timeout_=input_continuity_.config.brake;
    pnh_.param("robot_state_timeout", robot_state_timeout_, 0.10);
    pnh_.param("base_state_timeout", base_state_timeout_, 0.10);
    pnh_.param("base_position_jump_threshold",
               base_position_jump_threshold_, 0.20);
    pnh_.param("base_yaw_jump_threshold",
               base_yaw_jump_threshold_, 0.35);
    pnh_.param("base_session_x_half_range",
               base_session_x_half_range_, 0.30);
    pnh_.param("base_session_y_half_range",
               base_session_y_half_range_, 0.30);
    pnh_.param("base_session_yaw_half_range",
               base_session_yaw_half_range_, 0.35);
    pnh_.param("base_tracking_translation_limit",
               base_tracking_translation_limit_, 0.15);
    pnh_.param("base_tracking_yaw_limit", base_tracking_yaw_limit_, 0.20);
    pnh_.param("feedforward_filter_time_constant", ff_filter_time_constant_, 0.030);
    pnh_.param("continuous_reference_enabled", continuous_reference_enabled_, true);
    // Startup-only mode. The joint reference remains the only q/v/a owner.
    pnh_.param<std::string>("orientation_reference_mode", orientation_reference_mode_, "innovation");
    if (orientation_reference_mode_ != "innovation" && orientation_reference_mode_ != "direct_task" && orientation_reference_mode_ != "fixed_bandwidth")
      throw std::runtime_error("orientation_reference_mode must be innovation, direct_task or fixed_bandwidth");
    pnh_.param("hand_position_cutoff_hz", hand_position_cutoff_hz_, 4.);
    pnh_.param("hand_orientation_cutoff_hz", hand_orientation_cutoff_hz_, 3.);
    for(double hz : {hand_position_cutoff_hz_,hand_orientation_cutoff_hz_})
      if(!std::isfinite(hz) || hz<.5 || hz>10.)
        throw std::runtime_error("fixed hand cutoff must be finite and in [0.5,10] Hz");
    if(orientation_reference_mode_=="fixed_bandwidth" && !continuous_reference_enabled_)
      throw std::runtime_error("fixed_bandwidth requires continuous_reference_enabled");
    ROS_INFO("[VelocityIk] fixed reference cutoffs (-3dB): position %.2f Hz, orientation %.2f Hz",hand_position_cutoff_hz_,hand_orientation_cutoff_hz_);
    pnh_.param("hand_reference_frequency", hand_reference_frequency_, 12.0);
    pnh_.param("hand_orientation_reference_frequency", hand_orientation_reference_frequency_, 6.0);
    pnh_.param("torso_reference_frequency", torso_reference_frequency_, 1.2);
    ROS_INFO("[VelocityIk] orientation mode=%s; position %.2f Hz; innovation fallback orientation %.2f Hz",
             orientation_reference_mode_.c_str(),hand_reference_frequency_,hand_orientation_reference_frequency_);
    if (!std::isfinite(hand_reference_frequency_) || hand_reference_frequency_<4. || hand_reference_frequency_>20. ||
        !std::isfinite(hand_orientation_reference_frequency_) || hand_orientation_reference_frequency_<1. || hand_orientation_reference_frequency_>20. ||
        !std::isfinite(torso_reference_frequency_) || torso_reference_frequency_<.5 || torso_reference_frequency_>5.)
      throw std::runtime_error("invalid continuous reference frequency");
    pnh_.param("adaptive_reference/enabled", adaptive_reference_enabled_, true);
    auto& adaptive=adaptive_reference_settings_;
    pnh_.param("innovation_reference/enabled",adaptive.innovation.enabled,true);
    const auto read_innovation=[&](const std::string& name,auto& limits) {
      const std::string prefix="innovation_reference/"+name+"_";
      pnh_.param(prefix+"frequency",limits.frequency,limits.frequency);
      pnh_.param(prefix+"damping",limits.damping,limits.damping);
      pnh_.param(prefix+"speed",limits.speed,limits.speed);
      pnh_.param(prefix+"acceleration",limits.acceleration,limits.acceleration);
      pnh_.param(prefix+"jerk",limits.jerk,limits.jerk);
    };
    read_innovation("position",adaptive.innovation.position);
    adaptive.innovation.orientation.frequency=3.;
    read_innovation("orientation",adaptive.innovation.orientation);
    pnh_.param("adaptive_reference/min_frequency", adaptive.min_frequency, 1.0);
    pnh_.param("adaptive_reference/speed_time_constant", adaptive.speed_time_constant, .020);
    pnh_.param("adaptive_reference/attack_time_constant", adaptive.attack_time_constant, .010);
    pnh_.param("adaptive_reference/release_time_constant", adaptive.release_time_constant, .200);
    pnh_.param("adaptive_reference/position_error_start", adaptive.position.error_start, .004);
    pnh_.param("adaptive_reference/position_error_full", adaptive.position.error_full, .012);
    pnh_.param("adaptive_reference/position_speed_start", adaptive.position.speed_start, .030);
    pnh_.param("adaptive_reference/position_speed_full", adaptive.position.speed_full, .120);
    pnh_.param("adaptive_reference/orientation_error_start", adaptive.orientation.error_start, .00872664625997165);
    pnh_.param("adaptive_reference/orientation_error_full", adaptive.orientation.error_full, .026179938779915);
    pnh_.param("adaptive_reference/orientation_speed_start", adaptive.orientation.speed_start, .05235987755983);
    pnh_.param("adaptive_reference/orientation_speed_full", adaptive.orientation.speed_full, .261799387799149);
    pnh_.param("adaptive_reference/bumpless_enabled", adaptive.bumpless_enabled, true);
    pnh_.param("adaptive_reference/position_parameter_jerk", adaptive.position_parameter_jerk, 300.);
    pnh_.param("adaptive_reference/orientation_parameter_jerk", adaptive.orientation_parameter_jerk, 300.);
    pnh_.param("adaptive_reference/active_parameter_jerk_multiplier", adaptive.active_parameter_jerk_multiplier, 8.);
    auto& prediction=frame_prediction_settings_;
    pnh_.param("reference_prediction/min_hold", prediction.min_hold, .040);
    pnh_.param("reference_prediction/max_hold", prediction.max_hold, .060);
    pnh_.param("reference_prediction/decay", prediction.decay, .030);
    pnh_.param("reference_prediction/max_alignment_age", prediction.max_alignment_age, .060);
    if (!prediction.valid()) throw std::runtime_error("invalid reference prediction settings");
    if (!adaptive.valid(hand_reference_frequency_) || !adaptive.valid(hand_orientation_reference_frequency_))
      throw std::runtime_error("invalid adaptive continuous reference settings");
    ROS_INFO("[VelocityIk] continuous reference=%d adaptive=%d hand=%.2f..%.2f Hz torso=%.2f Hz",
             continuous_reference_enabled_,adaptive_reference_enabled_,adaptive.min_frequency,
             hand_reference_frequency_,torso_reference_frequency_);
    ROS_INFO("[VelocityIk] innovation reference=%d: motion lag compensated; correction limits %.3f m/s %.3f rad/s (not total motion caps)",
             adaptive.innovation.enabled,adaptive.innovation.position.speed,adaptive.innovation.orientation.speed);
    pnh_.param("feedforward_decay_time_constant", ff_decay_time_constant_, 0.050);
    pnh_.param("feedforward_hold_time", ff_hold_time_, 0.025);

    pnh_.param("enable_body_mirroring", enable_body_mirroring_, true);
    pnh_.param("enable_base_motion", enable_base_motion_, false);
    if (enable_base_motion_) {
      throw std::runtime_error("This 18-DoF controller requires enable_base_motion=false");
    }
    pnh_.param("enable_root_translation_intent",
               enable_root_translation_intent_, true);
    pnh_.param("enable_root_yaw_intent",
               enable_root_yaw_intent_, false);
    if (!enable_base_motion_) {
      enable_root_translation_intent_ = false;
      enable_root_yaw_intent_ = false;
      ROS_WARN("[VelocityIkShadow] Base x/y/yaw are HARD LOCKED; tracking uses 18 active DoFs (waist + arms)");
    }
    pnh_.param("root_translation_deadband",
               root_translation_deadband_, 0.035);
    pnh_.param("root_translation_full_distance",
               root_translation_full_distance_, 0.10);
    pnh_.param("root_translation_coherence_start",
               root_translation_coherence_start_, 0.03);
    pnh_.param("root_translation_coherence_full",
               root_translation_coherence_full_, 0.12);
    pnh_.param("root_translation_active_confidence",
               root_translation_active_confidence_, 0.20);
    pnh_.param("root_translation_release_error",
               root_translation_release_error_, 0.01);
    pnh_.param("root_translation_gain", root_translation_gain_, 1.0);
    pnh_.param("torso_forward_gain", torso_forward_gain_, 0.50);
    pnh_.param("torso_height_gain", torso_height_gain_, 0.50);
    pnh_.param("torso_pitch_gain", torso_pitch_gain_, 1.0);
    pnh_.param("torso_yaw_gain", torso_yaw_gain_, 0.7);
    pnh_.param("torso_height_tracking_gain", torso_tracking_gain_(0), 4.0);
    pnh_.param("torso_pitch_tracking_gain", torso_tracking_gain_(1), 5.0);
    pnh_.param("torso_yaw_tracking_gain", torso_tracking_gain_(2), 8.0);
    pnh_.param("torso_intent_weight", torso_intent_weight_, 5.0);
    pnh_.param("base_intent_gain", base_intent_gain_, 2.0);
    pnh_.param("base_intent_weight", base_intent_weight_, 5.0);
    pnh_.param("body_torso_deadband", body_torso_deadband_, 0.015);
    pnh_.param("body_torso_full_error", body_torso_full_error_, 0.15);

    pnh_.param("hand_position_gain", hand_position_gain_, 8.0);
    pnh_.param("hand_orientation_gain", hand_orientation_gain_, 6.0);
    pnh_.param("elbow_position_gain", elbow_position_gain_, 5.0);
    pnh_.param("hand_position_weight", hand_position_weight_, 12.0);
    pnh_.param("hand_orientation_weight", hand_orientation_weight_, 6.0);
    // The elbow is a redundancy preference only.  A low default prevents the
    // human elbow hint from pulling the primary 6-D TCP task away from target.
    pnh_.param("elbow_position_weight", elbow_position_weight_, 0.15);
    pnh_.param("posture_weight", posture_weight_, 0.010);
    pnh_.param("posture_gain", posture_gain_, 0.20);
    pnh_.param("lower_body_posture_weight",
               lower_body_posture_weight_, 0.25);
    pnh_.param("lower_body_posture_gain", lower_body_posture_gain_, 0.20);
    // TCP is still above elbow/posture objectives, but its inverse must not be
    // numerically undamped.  Joint proximity is handled per joint below; only
    // kinematic singularity raises this whole-arm numerical regularization.
    pnh_.param("primary_regularization_weight",
               primary_regularization_weight_, 0.0001);
    pnh_.param("primary_regularization_max_weight",
               primary_regularization_max_weight_, 0.002);
    pnh_.param("lower_body_primary_regularization_weight",
               lower_body_primary_regularization_weight_, 1.0);
    pnh_.param("lower_body_recruited_regularization_weight",
               lower_body_recruited_regularization_weight_, 0.010);
    if (!pnh_.getParam("base_primary_regularization_weight",
                       base_primary_regularization_weight_)) {
      // Temporary compatibility for step-3 launch/configuration snapshots.
      pnh_.param("base_locked_primary_regularization_weight",
                 base_primary_regularization_weight_, 1.0);
    }
    pnh_.param("base_recruited_regularization_weight",
               base_recruited_regularization_weight_, 0.10);
    pnh_.param("lower_body_recruited_posture_weight",
               lower_body_recruited_posture_weight_, 0.05);
    pnh_.param("nullspace_residual_tolerance",
               nullspace_residual_tolerance_, 1.0e-8);
    pnh_.param("global_nullspace_relative_tolerance",
               global_nullspace_relative_tolerance_, 1.0e-6);
    pnh_.param("secondary_correction_regularization",
               secondary_correction_regularization_, 1.0e-6);
    pnh_.param("base_damping", base_damping_, 1.0e-4);
    pnh_.param("singularity_damping", singularity_damping_, 0.12);
    pnh_.param("singularity_soft_threshold", singularity_soft_threshold_, 0.10);
    pnh_.param("singularity_hard_threshold", singularity_hard_threshold_, 0.025);
    pnh_.param("enable_continuous_recruitment",
               enable_continuous_recruitment_, true);
    pnh_.param("geometric_probe_speed", geometric_probe_speed_, 0.10);
    pnh_.param("recruitment_reach_start_distance",
               recruitment_reach_start_distance_,
               std::max(0.05, nominal_arm_reach_ - 0.10));
    pnh_.param("recruitment_reach_full_distance",
               recruitment_reach_full_distance_,
               nominal_arm_reach_ - selected_reach_reserve_);
    pnh_.param("recruitment_singularity_start",
               recruitment_singularity_start_,
               feasibility_singularity_approach_threshold_);
    pnh_.param("recruitment_singularity_full",
               recruitment_singularity_full_,
               feasibility_singularity_escape_threshold_);
    pnh_.param("recruitment_joint_margin_start",
               recruitment_joint_margin_start_, 0.25);
    pnh_.param("recruitment_joint_margin_full",
               recruitment_joint_margin_full_, 0.05);
    pnh_.param("lower_body_recruitment_rise_rate",
               lower_body_recruitment_rise_rate_, 2.0);
    pnh_.param("lower_body_recruitment_fall_rate",
               lower_body_recruitment_fall_rate_, 0.50);
    pnh_.param("base_recruitment_rise_rate",
               base_recruitment_rise_rate_, 1.0);
    pnh_.param("base_recruitment_fall_rate",
               base_recruitment_fall_rate_, 1.0);
    pnh_.param("base_admission_dwell_time",
               base_admission_dwell_time_, 0.40);
    pnh_.param("base_admission_on_threshold",
               base_admission_on_threshold_, 0.10);
    pnh_.param("base_admission_off_threshold",
               base_admission_off_threshold_, 0.03);
    pnh_.param("lower_directional_effectiveness_start",
               lower_directional_effectiveness_start_, 0.02);
    pnh_.param("lower_directional_effectiveness_full",
               lower_directional_effectiveness_full_, 0.15);
    pnh_.param("preparation_gain", preparation_gain_, 2.0);
    pnh_.param("preparation_handoff_duration", preparation_handoff_duration_, 0.02);
    if (!std::isfinite(preparation_handoff_duration_) ||
        preparation_handoff_duration_ < 0.0 || preparation_handoff_duration_ > 0.02)
      throw std::runtime_error("invalid preparation handoff duration");
    pnh_.param("preparation_arm_velocity_limit",
               preparation_arm_velocity_limit_, 0.60);
    pnh_.param("preparation_position_tolerance",
               preparation_position_tolerance_, 0.025);
    pnh_.param("preparation_measured_position_tolerance",
               preparation_measured_position_tolerance_, 0.060);
    pnh_.param("preparation_lower_position_tolerance", preparation_lower_position_tolerance_, 0.008);
    pnh_.param("preparation_lower_measured_position_tolerance", preparation_lower_measured_position_tolerance_, 0.015);
    pnh_.param("preparation_velocity_tolerance",
               preparation_velocity_tolerance_, 0.030);
    pnh_.param("preparation_settle_cycles",
               preparation_settle_cycles_required_, 25);
    auto& startup = startup_readiness_.settings;
    for (auto item : {std::make_pair("window", &startup.window),
         {"median_window", &startup.median_window}, {"arm_position", &startup.arm_position},
         {"lower_position", &startup.lower_position}, {"arm_speed", &startup.arm_speed},
         {"lower_speed", &startup.lower_speed}, {"arm_raw_speed", &startup.arm_raw_speed},
         {"lower_raw_speed", &startup.lower_raw_speed}, {"raw_dwell", &startup.raw_dwell},
         {"extreme_speed", &startup.extreme_speed}})
      pnh_.param(std::string("startup_readiness/")+item.first, *item.second, *item.second);
    startup.validate();
    // ACTIVE acquires the current human pose directly from measured q.
    // No preparatory wrist-dependent target recapture is allowed.
    enable_natural_hang_bootstrap_ = false;
    pnh_.param("natural_hang_arm_threshold",
               natural_hang_arm_threshold_, 0.08);
    auto load_lower_array = [this](
        const std::string& name,
        std::array<double, kLowerBodyDof>* values) {
      std::vector<double> configured;
      if (!pnh_.getParam(name, configured)) {
        return;
      }
      if (configured.size() != values->size() ||
          !std::all_of(configured.begin(), configured.end(),
                       [](double value) { return std::isfinite(value); })) {
        throw std::runtime_error("~" + name + " must contain four finite values");
      }
      std::copy(configured.begin(), configured.end(), values->begin());
    };
    load_lower_array("lower_joint_stage_start", &lower_joint_stage_start_);
    load_lower_array("lower_joint_stage_full", &lower_joint_stage_full_);
    load_lower_array("lower_joint_effort_multiplier",
                     &lower_joint_effort_multiplier_);
    load_lower_array("lower_joint_cost_velocity_scale", &lower_joint_cost_velocity_scale_);
    load_lower_array("preparation_lower_position", &preparation_lower_position_requested_);
    load_lower_array("torso_workspace_lower_offset", &torso_workspace_lower_offset_);
    load_lower_array("torso_workspace_upper_offset", &torso_workspace_upper_offset_);
    load_lower_array("lower_joint_velocity_cap", &lower_joint_velocity_cap_);
    load_lower_array("lower_joint_acceleration_cap",
                     &lower_joint_acceleration_cap_);
    std::vector<double> preparation_arm_position;
    if (pnh_.getParam("preparation_arm_position", preparation_arm_position)) {
      if (preparation_arm_position.size() != kDualArmDof ||
          !std::all_of(preparation_arm_position.begin(),
                       preparation_arm_position.end(),
                       [](double value) { return std::isfinite(value); })) {
        throw std::runtime_error(
            "~preparation_arm_position must contain fourteen finite radians");
      }
      preparation_arm_position_requested_ = preparation_arm_position;
    }
    pnh_.param("cascade_lower_residual_start",
               cascade_lower_residual_start_, 0.015);
    pnh_.param("cascade_lower_residual_full",
               cascade_lower_residual_full_, 0.060);
    pnh_.param("cascade_base_residual_start",
               cascade_base_residual_start_, 0.020);
    pnh_.param("cascade_base_residual_full",
               cascade_base_residual_full_, 0.080);
    pnh_.param("cascade_base_tracking_error_start",
               cascade_base_tracking_error_start_, 0.040);
    pnh_.param("cascade_base_tracking_error_full",
               cascade_base_tracking_error_full_, 0.100);
    pnh_.param("lower_body_return_start",
               lower_body_return_start_, 0.015);
    pnh_.param("lower_body_return_full",
               lower_body_return_full_, 0.10);
    pnh_.param("minimum_translation_progress_ratio",
               minimum_translation_progress_ratio_, 0.02);

    pnh_.param("cascade_stage_dwell_time",
               cascade_stage_dwell_time_, 0.35);
    pnh_.param("cascade_stage_admission_threshold",
               cascade_stage_admission_threshold_, 0.15);
    pnh_.param("cascade_stage_release_dwell_time",
               cascade_stage_release_dwell_time_, 0.50);
    pnh_.param("cascade_stage_release_threshold",
               cascade_stage_release_threshold_, 0.05);
    pnh_.param("cascade_minimum_active_recruitment",
               cascade_minimum_active_recruitment_, 0.20);
    pnh_.param("cascade_directional_skip_threshold",
               cascade_directional_skip_threshold_, 0.01);
    pnh_.param("arm_reference_tracking_hard_limit",
               arm_reference_tracking_hard_limit_, 0.30);
    pnh_.param("lower_reference_tracking_hard_limit",
               lower_reference_tracking_hard_limit_, 0.20);
    pnh_.param("reference_tracking_fault_dwell",
               reference_tracking_fault_dwell_, 0.10);
    pnh_.param("reference_tracking_delay_compensation",
               reference_tracking_delay_compensation_, 0.025);
    pnh_.param("reference_tracking_delay_window",
               reference_tracking_delay_window_, 0.035);
    pnh_.param("arm_reference_tracking_qp_limit",
               arm_reference_tracking_qp_limit_, 0.12);
    pnh_.param("lower_reference_tracking_qp_limit",
               lower_reference_tracking_qp_limit_, 0.08);
    pnh_.param("reference_tracking_recovery_arm_limit",
               reference_tracking_recovery_arm_limit_, 0.08);
    pnh_.param("reference_tracking_recovery_lower_limit",
               reference_tracking_recovery_lower_limit_, 0.05);
    pnh_.param("reference_tracking_recovery_dwell",
               reference_tracking_recovery_dwell_, 0.40);
    pnh_.param("kinematic_prediction_horizon",
               kinematic_prediction_horizon_, 0.015);
    pnh_.param("orientation_tracking_full_error",
               orientation_tracking_full_error_, 0.06);
    pnh_.param("orientation_tracking_zero_error",
               orientation_tracking_zero_error_, 0.16);
    pnh_.param("orientation_joint_margin_zero",
               orientation_joint_margin_zero_, 0.02);
    pnh_.param("orientation_joint_margin_full",
               orientation_joint_margin_full_, 0.15);

    pnh_.param("position_limit_margin", position_limit_margin_, 0.01);
    pnh_.param("boundary_brake_guard", boundary_brake_guard_, 0.02);
    pnh_.param("arm_boundary_execution_delay_proximal",
               arm_boundary_execution_delay_proximal_, 0.020);
    pnh_.param("arm_boundary_execution_delay_wrist",
               arm_boundary_execution_delay_wrist_, 0.045);
    pnh_.param("emergency_acceleration_scale", emergency_acceleration_scale_, 2.0);
    pnh_.param("candidate_timeout", candidate_timeout_, 0.0045);
    pnh_.param("max_candidate_reference_lag_cycles",
               max_candidate_reference_lag_cycles_, 2);
    pnh_.param("max_candidate_hold_cycles", max_candidate_hold_cycles_, 3);
    pnh_.param("hold_velocity_threshold", hold_velocity_threshold_, 1.0e-3);
    pnh_.param("hold_acceleration_threshold", hold_acceleration_threshold_, 2.0e-2);
    // Legacy qpOASES CPU limit is intentionally not read: ProxQP uses max_iter=100.
    pnh_.setParam("qp_cpu_time_limit_status", "UNUSED_BY_PROXQP");
    pnh_.param("qp_max_working_set_recalculations", qp_max_working_set_recalculations_, 80);
    pnh_.param("diagnostic_rate_hz", diagnostic_rate_hz_, 100.0);
    pnh_.param("formal_reference_mirror_rate_hz", formal_reference_mirror_rate_hz_, diagnostic_rate_hz_);
    pnh_.param("torso_frame", torso_frame_name_,
               std::string("waist_yaw_link"));
    pnh_.param("torso_x_half_range", torso_x_half_range_, 0.18);
    pnh_.param("torso_z_half_range", torso_z_half_range_, 0.12);
    pnh_.param("torso_pitch_half_range", torso_pitch_half_range_,
               20.0 * M_PI / 180.0);
    pnh_.param("torso_yaw_half_range", torso_yaw_half_range_,
               45.0 * M_PI / 180.0);

    const std::vector<double> velocity_default =
        buildDecisionDefault(
            {0.15, 0.15, 0.30},
            {0.30, 0.45, 0.80, 2.00},
            {4.0, 4.0, 4.0, 4.0, 8.0, 8.0, 8.0});
    const std::vector<double> acceleration_default =
        buildDecisionDefault(
            {0.40, 0.40, 0.80},
            {1.00, 1.50, 3.00, 8.00},
            {50.0, 50.0, 50.0, 50.0, 100.0, 100.0, 100.0});
    const std::vector<double> safe_ratio_default = buildDecisionDefault(
        {1.0, 1.0, 1.0},
        {lower_body_safe_range_ratio_default_,
         lower_body_safe_range_ratio_default_,
         lower_body_safe_range_ratio_default_,
         lower_body_safe_range_ratio_default_},
        {joint_safe_range_ratio_default_, joint_safe_range_ratio_default_,
         joint_safe_range_ratio_default_, joint_safe_range_ratio_default_,
         joint_safe_range_ratio_default_, joint_safe_range_ratio_default_,
         joint_safe_range_ratio_default_});
    // Fixed v63 natural arm posture: shoulder pitch 15 deg and elbow -30 deg.
    // A fixed center is intentional; deriving it from the enable pose would
    // make the operating limits depend on where a teleoperation session starts.
    const std::vector<double> safe_center_default =
        buildDecisionDefault(
            {0.0, 0.0, 0.0},
            {14.27 * M_PI / 180.0, -25.27 * M_PI / 180.0,
             11.80 * M_PI / 180.0, 0.0},
            {M_PI / 12.0, 0.0, 0.0, -M_PI / 6.0, 0.0, 0.0, 0.0});
    velocity_limit_ = loadVectorParameter("joint_velocity_limits", velocity_default);
    acceleration_limit_ = loadVectorParameter("joint_acceleration_limits", acceleration_default);
    braking_limit_ = loadVectorParameter("joint_braking_limits", acceleration_default);
    outward_acceleration_limit_ = loadVectorParameter("joint_delay_outward_acceleration_limits", acceleration_default);
    joint_jerk_enabled_=false;
    if(pnh_.hasParam("joint_jerk/enabled") && !pnh_.getParam("joint_jerk/enabled",joint_jerk_enabled_))
      throw std::runtime_error("joint_jerk/enabled must be a boolean");
    joint_jerk_limits_=DecisionVector::Zero(decisionDofCount());
    for(int arm=0;arm<kArmCount;++arm)for(int joint=0;joint<kJointsPerArm;++joint) {
      const std::string name="zarm_"+std::string(arm==0?"l":"r")+std::to_string(joint+1)+"_joint";
      const int i=dof_layout_.decisionIndexByName(name);
      if(i!=armDecisionIndex(arm,joint))throw std::runtime_error("joint jerk arm layout mismatch");
      double limit=joint<4?25000.:50000.;
      const std::string key="joint_jerk/limits/"+name;
      if(pnh_.hasParam(key) && !pnh_.getParam(key,limit))
        throw std::runtime_error("joint jerk limit must be numeric: "+key);
      if(!std::isfinite(limit)||limit<=0.)throw std::runtime_error("joint jerk limits must be finite and positive");
      joint_jerk_limits_(i)=limit;
    }
    bool j3_profile = false;
    nh_.param("/j3_overshoot_profile/enabled", j3_profile, false);
    if (j3_profile) {
      // Resolve by canonical names. An invalid/partial profile fails at startup.
      for (const std::string name : {"zarm_l3_joint", "zarm_r3_joint"}) {
        const int i = dof_layout_.decisionIndexByName(name);
        if (i != (name == "zarm_l3_joint" ? 9 : 16))
          throw std::runtime_error("J3 profile requires the canonical 21-D layout");
        const std::string key = "/j3_overshoot_profile/joint_dynamic_overrides/" + name + "/";
        auto read = [&](const std::string& field) {
          double value = 0.;
          if (!nh_.getParam(key + field, value) || !std::isfinite(value) || value <= 0.)
            throw std::runtime_error("Invalid J3 dynamic profile: " + key + field);
          return value;
        };
        velocity_limit_(i) = read("velocity");
        acceleration_limit_(i) = read("acceleration");
        braking_limit_(i) = read("braking");
        outward_acceleration_limit_(i) = read("outward");
        joint_jerk_limits_(i) = read("jerk");
        if (braking_limit_(i) != acceleration_limit_(i) ||
            outward_acceleration_limit_(i) != acceleration_limit_(i))
          throw std::runtime_error("J3 acceleration/braking/outward contract must agree");
      }
    }
    pnh_.param("separate_stopping_contract", separate_stopping_contract_, false);
    pnh_.param("operating_envelope/enabled", operating_envelope_enabled_, false);
    pnh_.param("temporal_cost/enabled", temporal_cost_enabled_, false);
    pnh_.param("operating_diagnostics/enabled", envelope_diagnostics_enabled_, false);
    bool communication_audit = false;
    pnh_.param("communication_full_rate_audit", communication_audit, false);
    pnh_.param("operating_diagnostics/stride", envelope_diagnostics_stride_, 5);
    if (communication_audit) envelope_diagnostics_stride_ = 1;
    if (envelope_diagnostics_stride_ < 1)
      throw std::runtime_error("operating_diagnostics/stride must be positive");
    const auto zeros = std::vector<double>(decisionDofCount(), 0.);
    // Strict validation: a mistyped experiment must not silently use defaults.
    operating_extra_position_ = loadStrictNonnegativeVector(
        "operating_envelope/extra_position", zeros, false);
    operating_extra_time_ = loadStrictNonnegativeVector(
        "operating_envelope/extra_time", zeros, false);
    const auto aw = loadStrictNonnegativeVector("temporal_cost/acceleration_weight", zeros, false);
    const auto dw = loadStrictNonnegativeVector("temporal_cost/delta_acceleration_weight", zeros, false);
    const auto as = loadStrictNonnegativeVector("temporal_cost/acceleration_scale", acceleration_default, true);
    const auto ds = loadStrictNonnegativeVector("temporal_cost/delta_acceleration_scale", acceleration_default, true);
    temporal_cost_.resize(decisionDofCount());
    for (int i = 0; i < decisionDofCount(); ++i)
      temporal_cost_[i] = {aw(i), dw(i), as(i), ds(i)};
    pnh_.param("execution_governor/enabled", execution_governor_enabled_, false);
    const std::array<std::string, kArmCount> governed_names{{"zarm_l3_joint", "zarm_r3_joint"}};
    for (int arm=0;arm<kArmCount;++arm) {
      governor_indices_[arm] = dof_layout_.decisionIndexByName(governed_names[arm]);
      auto& p = governor_config_[arm];
      const auto prefix = "execution_governor/" + governed_names[arm] + "/";
      pnh_.param(prefix+"error_deadband", p.error_deadband, p.error_deadband);
      pnh_.param(prefix+"error_full", p.error_full, p.error_full);
      pnh_.param(prefix+"prediction_time", p.prediction_time, p.prediction_time);
      pnh_.param(prefix+"penalty_ratio", p.penalty_ratio, p.penalty_ratio);
      pnh_.param(prefix+"smoothing_ratio", p.smoothing_ratio, p.smoothing_ratio);
      pnh_.param(prefix+"attack_time", p.attack_time, p.attack_time);
      pnh_.param(prefix+"release_time", p.release_time, p.release_time);
      if (!velocity_ik::validExecutionGovernorConfig(p))
        throw std::runtime_error("invalid execution governor config: " + governed_names[arm]);
    }
    pnh_.param("qp_snapshot_directory", qp_snapshot_directory_, std::string{});
    pnh_.param("host_target_reconstruction", host_target_reconstruction_, false);
    pnh_.param("target_adoption_lead", target_adoption_lead_, 0.001);
    if (!std::isfinite(target_adoption_lead_) || target_adoption_lead_ < 0 || target_adoption_lead_ > .003)
      throw std::runtime_error("target adoption lead must be in [0,3] ms");
    joint_safe_range_ratio_ = loadVectorParameter(
        "joint_safe_range_ratios", safe_ratio_default);
    joint_safe_center_requested_ = loadFiniteVectorParameter(
        "joint_safe_center", safe_center_default);

    solver_rate_hz_ = std::max(1.0, solver_rate_hz_);
    dt_ = 1.0 / solver_rate_hz_;
    candidate_timeout_ = std::max(candidate_timeout_, 1.5 * dt_);
    max_candidate_reference_lag_cycles_ =
        std::clamp(max_candidate_reference_lag_cycles_, 0, 4);
    max_candidate_hold_cycles_ =
        std::clamp(max_candidate_hold_cycles_, 0, 10);
    reference_tracking_delay_compensation_ = std::clamp(
        reference_tracking_delay_compensation_, 0.0, 0.10);
    reference_tracking_delay_window_ = std::clamp(
        reference_tracking_delay_window_, 0.0, 0.10);
    const auto diagnosticStride = [this](double rate) {
      return rate <= 0.0 ? 1 : std::max(1, static_cast<int>(
          std::round(solver_rate_hz_ / std::max(1.0, std::isfinite(rate) ? rate : 100.0))));
    };
    diagnostic_stride_ = diagnosticStride(diagnostic_rate_hz_);
    formal_reference_mirror_stride_ = diagnosticStride(formal_reference_mirror_rate_hz_);
    duplicate_position_epsilon_ = std::max(0.0, duplicate_position_epsilon_);
    duplicate_orientation_epsilon_ = std::max(0.0, duplicate_orientation_epsilon_);
    nominal_arm_reach_ = std::max(0.05, nominal_arm_reach_);
    selected_reach_reserve_ =
        std::clamp(selected_reach_reserve_, 0.0, nominal_arm_reach_ - 0.01);
    workspace_projection_smoothing_ =
        std::clamp(workspace_projection_smoothing_, 1.0e-5, 0.05);
    feasibility_characteristic_length_ =
        std::max(1.0e-3, feasibility_characteristic_length_);
    feasibility_singularity_escape_threshold_ =
        std::max(0.0, feasibility_singularity_escape_threshold_);
    feasibility_singularity_approach_threshold_ = std::max(
        feasibility_singularity_escape_threshold_ + 1.0e-6,
        feasibility_singularity_approach_threshold_);
    translation_singularity_escape_sigma_ =
        std::max(0.0, translation_singularity_escape_sigma_);
    translation_singularity_safe_sigma_ = std::max(
        translation_singularity_escape_sigma_ + 1.0e-6,
        translation_singularity_safe_sigma_);
    translation_singularity_activation_sigma_ = std::max(
        translation_singularity_safe_sigma_ + 1.0e-6,
        translation_singularity_activation_sigma_);
    translation_singularity_gain_ =
        std::max(0.0, translation_singularity_gain_);
    singularity_gradient_step_ =
        std::max(1.0e-5, singularity_gradient_step_);
    singularity_gradient_min_norm_ =
        std::max(0.0, singularity_gradient_min_norm_);
    natural_escape_speed_ = std::max(0.0, natural_escape_speed_);
    feasibility_joint_limit_escape_ratio_ =
        std::clamp(feasibility_joint_limit_escape_ratio_, 0.0, 0.49);
    feasibility_joint_limit_approach_ratio_ = std::clamp(
        std::max(feasibility_joint_limit_approach_ratio_,
                 feasibility_joint_limit_escape_ratio_ + 1.0e-6),
        0.0, 0.50);
    feasibility_escape_lookahead_ =
        std::max(1.0e-4, feasibility_escape_lookahead_);
    feasibility_escape_joint_margin_weight_ =
        std::max(0.0, feasibility_escape_joint_margin_weight_);
    joint_limit_barrier_weight_ = std::max(0.0, joint_limit_barrier_weight_);
    joint_limit_barrier_gain_ = std::max(0.0, joint_limit_barrier_gain_);
    joint_safe_range_ratio_default_ = std::clamp(
        joint_safe_range_ratio_default_, 0.10, 1.0);
    lower_body_safe_range_ratio_default_ = std::clamp(
        lower_body_safe_range_ratio_default_, 0.10, 1.0);
    for (int i = 0; i < decisionDofCount(); ++i) {
      joint_safe_range_ratio_(i) = std::clamp(
          joint_safe_range_ratio_(i), 0.10, 1.0);
    }
    primary_regularization_weight_ =
        std::max(1.0e-6, primary_regularization_weight_);
    primary_regularization_max_weight_ = std::max(
        primary_regularization_weight_, primary_regularization_max_weight_);
    lower_body_posture_weight_ = std::max(0.0, lower_body_posture_weight_);
    lower_body_recruited_posture_weight_ =
        std::max(0.0, lower_body_recruited_posture_weight_);
    lower_body_recruited_regularization_weight_ =
        std::max(1.0e-9, lower_body_recruited_regularization_weight_);
    base_recruited_regularization_weight_ =
        std::max(1.0e-9, base_recruited_regularization_weight_);
    lower_body_posture_gain_ = std::max(0.0, lower_body_posture_gain_);
    lower_body_primary_regularization_weight_ =
        std::max(1.0e-6, lower_body_primary_regularization_weight_);
    base_primary_regularization_weight_ =
        std::max(1.0e-6, base_primary_regularization_weight_);
    lower_body_recruited_regularization_weight_ = std::min(
        lower_body_recruited_regularization_weight_,
        lower_body_primary_regularization_weight_);
    base_recruited_regularization_weight_ = std::min(
        base_recruited_regularization_weight_,
        base_primary_regularization_weight_);
    lower_body_recruited_posture_weight_ = std::min(
        lower_body_recruited_posture_weight_, lower_body_posture_weight_);
    emergency_acceleration_scale_ =
        std::clamp(emergency_acceleration_scale_, 1.0, 4.0);
    arm_boundary_execution_delay_proximal_ = std::clamp(
        arm_boundary_execution_delay_proximal_, 0.0, 0.20);
    arm_boundary_execution_delay_wrist_ = std::clamp(
        arm_boundary_execution_delay_wrist_, 0.0, 0.20);
    nullspace_residual_tolerance_ =
        std::max(1.0e-12, nullspace_residual_tolerance_);
    global_nullspace_relative_tolerance_ =
        std::clamp(global_nullspace_relative_tolerance_, 1.0e-12, 0.10);
    secondary_correction_regularization_ = std::max(
        1.0e-12, secondary_correction_regularization_);
    torso_x_half_range_ = std::max(0.01, torso_x_half_range_);
    torso_z_half_range_ = std::max(0.01, torso_z_half_range_);
    torso_pitch_half_range_ = std::max(1.0e-3, torso_pitch_half_range_);
    torso_yaw_half_range_ = std::max(1.0e-3, torso_yaw_half_range_);
    base_state_timeout_ = std::max(dt_, base_state_timeout_);
    base_position_jump_threshold_ =
        std::max(0.01, base_position_jump_threshold_);
    base_yaw_jump_threshold_ =
        std::clamp(base_yaw_jump_threshold_, 0.01, M_PI);
    base_session_x_half_range_ =
        std::max(0.01, base_session_x_half_range_);
    base_session_y_half_range_ =
        std::max(0.01, base_session_y_half_range_);
    base_session_yaw_half_range_ =
        std::clamp(base_session_yaw_half_range_, 0.01, M_PI);
    base_tracking_translation_limit_ =
        std::max(0.01, base_tracking_translation_limit_);
    base_tracking_yaw_limit_ =
        std::clamp(base_tracking_yaw_limit_, 0.01, M_PI);
    if (body_tracking_frame_name_.empty()) {
      throw std::runtime_error("~body_tracking_frame must be non-empty");
    }
    root_translation_deadband_ = std::max(
        0.0, root_translation_deadband_);
    root_translation_full_distance_ = std::max(
        root_translation_deadband_ + 1.0e-6,
        root_translation_full_distance_);
    root_translation_coherence_start_ = std::max(
        0.0, root_translation_coherence_start_);
    root_translation_coherence_full_ = std::max(
        root_translation_coherence_start_ + 1.0e-6,
        root_translation_coherence_full_);
    root_translation_active_confidence_ = std::clamp(
        root_translation_active_confidence_, 0.0, 1.0);
    root_translation_release_error_ = std::max(
        0.0, root_translation_release_error_);
    root_translation_gain_ = std::max(0.0, root_translation_gain_);
    torso_forward_gain_ = std::max(0.0, torso_forward_gain_);
    torso_height_gain_ = std::max(0.0, torso_height_gain_);
    torso_pitch_gain_ = std::max(0.0, torso_pitch_gain_);
    torso_yaw_gain_ = std::max(0.0, torso_yaw_gain_);
    if (!torso_tracking_gain_.array().isFinite().all() ||
        (torso_tracking_gain_.array() <= 0.0).any()) {
      throw std::runtime_error("torso tracking gains must be finite and positive");
    }
    torso_intent_weight_ = std::max(0.0, torso_intent_weight_);
    base_intent_gain_ = std::max(0.0, base_intent_gain_);
    base_intent_weight_ = std::max(0.0, base_intent_weight_);
    body_torso_deadband_ = std::max(0.0, body_torso_deadband_);
    body_torso_full_error_ = std::max(
        body_torso_deadband_ + 1.0e-6, body_torso_full_error_);
    const bool recruitment_thresholds_valid =
        recruitment_reach_start_distance_ <
            recruitment_reach_full_distance_ &&
        recruitment_singularity_start_ >
            recruitment_singularity_full_ &&
        recruitment_joint_margin_start_ >
            recruitment_joint_margin_full_;
    if (!recruitment_thresholds_valid) {
      throw std::runtime_error(
          "invalid continuous-recruitment thresholds: reach must rise, "
          "singularity and joint margin must fall");
    }
    lower_body_recruitment_rise_rate_ =
        std::max(1.0e-3, lower_body_recruitment_rise_rate_);
    geometric_probe_speed_ = std::max(1.0e-3, geometric_probe_speed_);
    lower_body_recruitment_fall_rate_ =
        std::max(1.0e-3, lower_body_recruitment_fall_rate_);
    base_recruitment_rise_rate_ =
        std::max(1.0e-3, base_recruitment_rise_rate_);
    base_recruitment_fall_rate_ =
        std::max(1.0e-3, base_recruitment_fall_rate_);
    const auto validate_rising_interval = [](double start, double full,
                                             const char* name) {
      if (!std::isfinite(start) || !std::isfinite(full) || start < 0.0 ||
          start >= full) {
        throw std::runtime_error(std::string("invalid ") + name +
                                 " interval");
      }
    };
    base_admission_dwell_time_ = std::max(0.0, base_admission_dwell_time_);
    base_admission_on_threshold_ = std::clamp(
        base_admission_on_threshold_, 1.0e-6, 1.0);
    base_admission_off_threshold_ = std::clamp(
        base_admission_off_threshold_, 0.0,
        base_admission_on_threshold_ - 1.0e-6);
    validate_rising_interval(
        lower_directional_effectiveness_start_,
        lower_directional_effectiveness_full_,
        "lower directional effectiveness");
    for (int joint = 0; joint < kLowerBodyDof; ++joint) {
      if (lower_joint_stage_start_.at(joint) < 0.0 ||
          lower_joint_stage_full_.at(joint) > 1.0 ||
          lower_joint_stage_start_.at(joint) >=
              lower_joint_stage_full_.at(joint)) {
        throw std::runtime_error(
            "lower joint stage intervals must be ordered inside [0,1]");
      }
      lower_joint_effort_multiplier_.at(joint) = std::max(
          1.0, lower_joint_effort_multiplier_.at(joint));
      // Validate normalization separately: changing drive caps cannot change cost.
      motion_capture_ik::whole_body_ik::driveRegularization(
          lower_joint_effort_multiplier_.at(joint),
          lower_joint_cost_velocity_scale_.at(joint));
      if (torso_workspace_lower_offset_.at(joint) > 0.0 ||
          torso_workspace_upper_offset_.at(joint) < 0.0 ||
          torso_workspace_lower_offset_.at(joint) >= torso_workspace_upper_offset_.at(joint)) {
        throw std::runtime_error("torso workspace must contain the explicit ready pose");
      }
      lower_joint_velocity_cap_.at(joint) = std::max(
          1.0e-3, lower_joint_velocity_cap_.at(joint));
      lower_joint_acceleration_cap_.at(joint) = std::max(
          1.0e-3, lower_joint_acceleration_cap_.at(joint));
    }
    preparation_gain_ = std::max(0.1, preparation_gain_);
    preparation_arm_velocity_limit_ = std::max(
        0.05, preparation_arm_velocity_limit_);
    preparation_position_tolerance_ = std::max(
        1.0e-3, preparation_position_tolerance_);
    preparation_measured_position_tolerance_ = std::max(
        preparation_position_tolerance_,
        preparation_measured_position_tolerance_);
    if (!std::isfinite(preparation_lower_position_tolerance_) ||
        !std::isfinite(preparation_lower_measured_position_tolerance_) ||
        preparation_lower_position_tolerance_ <= 0.0 ||
        preparation_lower_measured_position_tolerance_ < preparation_lower_position_tolerance_) {
      throw std::runtime_error("invalid lower-body ready settling tolerances");
    }
    preparation_velocity_tolerance_ = std::max(
        1.0e-3, preparation_velocity_tolerance_);
    preparation_settle_cycles_required_ = std::max(
        1, preparation_settle_cycles_required_);
    natural_hang_arm_threshold_ = std::max(
        0.01, natural_hang_arm_threshold_);
    validate_rising_interval(
        cascade_lower_residual_start_, cascade_lower_residual_full_,
        "lower cascade residual");
    validate_rising_interval(
        cascade_base_residual_start_, cascade_base_residual_full_,
        "base cascade residual");
    validate_rising_interval(
        cascade_base_tracking_error_start_,
        cascade_base_tracking_error_full_,
        "base cascade tracking error");
    validate_rising_interval(
        lower_body_return_start_, lower_body_return_full_,
        "lower-body return");
    minimum_translation_progress_ratio_ = std::clamp(
        minimum_translation_progress_ratio_, 0.0, 1.0);
    cascade_stage_dwell_time_ = std::max(0.0, cascade_stage_dwell_time_);
    cascade_stage_admission_threshold_ = std::clamp(
        cascade_stage_admission_threshold_, 1.0e-4, 1.0);
    cascade_stage_release_dwell_time_ = std::max(
        0.0, cascade_stage_release_dwell_time_);
    cascade_stage_release_threshold_ = std::clamp(
        cascade_stage_release_threshold_, 0.0,
        cascade_stage_admission_threshold_ - 1.0e-6);
    cascade_minimum_active_recruitment_ = std::clamp(
        cascade_minimum_active_recruitment_, 1.0e-3, 1.0);
    cascade_directional_skip_threshold_ = std::clamp(
        cascade_directional_skip_threshold_, 0.0, 1.0);
    arm_reference_tracking_hard_limit_ = std::max(
        0.01, arm_reference_tracking_hard_limit_);
    lower_reference_tracking_hard_limit_ = std::max(
        0.01, lower_reference_tracking_hard_limit_);
    reference_tracking_fault_dwell_ = std::max(
        0.0, reference_tracking_fault_dwell_);
    arm_reference_tracking_qp_limit_ = std::clamp(
        arm_reference_tracking_qp_limit_, 0.01,
        arm_reference_tracking_hard_limit_);
    lower_reference_tracking_qp_limit_ = std::clamp(
        lower_reference_tracking_qp_limit_, 0.01,
        lower_reference_tracking_hard_limit_);
    reference_tracking_recovery_arm_limit_ = std::clamp(
        reference_tracking_recovery_arm_limit_, 0.005,
        arm_reference_tracking_qp_limit_);
    reference_tracking_recovery_lower_limit_ = std::clamp(
        reference_tracking_recovery_lower_limit_, 0.005,
        lower_reference_tracking_qp_limit_);
    reference_tracking_recovery_dwell_ = std::max(
        0.0, reference_tracking_recovery_dwell_);
    kinematic_prediction_horizon_ = std::clamp(
        kinematic_prediction_horizon_, 0.0, 0.05);
    orientation_tracking_full_error_ = std::max(
        0.0, orientation_tracking_full_error_);
    orientation_tracking_zero_error_ = std::max(
        orientation_tracking_full_error_ + 1.0e-6,
        orientation_tracking_zero_error_);
    orientation_joint_margin_zero_ = std::clamp(
        orientation_joint_margin_zero_, 0.0, 0.49);
    orientation_joint_margin_full_ = std::clamp(
        std::max(orientation_joint_margin_full_,
                 orientation_joint_margin_zero_ + 1.0e-6),
        0.0, 0.50);
    if (world_frame_name_.empty() || session_frame_name_.empty()) {
      throw std::runtime_error(
          "~world_frame and ~session_frame must both be non-empty");
    }
  }

  DecisionVector loadVectorParameter(const std::string& name,
                                     const std::vector<double>& fallback) {
    if (fallback.size() != static_cast<std::size_t>(decisionDofCount())) {
      throw std::logic_error("default parameter size does not match decision layout");
    }
    std::vector<double> values;
    if (!pnh_.getParam(name, values)) {
      values = fallback;
    }
    if (values.size() != static_cast<std::size_t>(decisionDofCount())) {
      ROS_WARN_STREAM("[VelocityIkShadow] ~" << name << " must contain "
                      << decisionDofCount() << " values for layout v"
                      << kDofLayoutVersion << "; using defaults");
      values = fallback;
    }
    DecisionVector result(decisionDofCount());
    for (int i = 0; i < decisionDofCount(); ++i) {
      result(i) = std::max(1.0e-6, std::abs(values.at(i)));
    }
    return result;
  }

  DecisionVector loadStrictNonnegativeVector(
      const std::string& name, const std::vector<double>& fallback, bool positive) {
    std::vector<double> values = fallback;
    if (pnh_.hasParam(name) && !pnh_.getParam(name, values))
      throw std::runtime_error(name + " must be a numeric vector");
    if (values.size() != static_cast<size_t>(decisionDofCount()) ||
        !std::all_of(values.begin(), values.end(), [positive](double x) {
          return std::isfinite(x) && (positive ? x > 0. : x >= 0.);
        })) throw std::runtime_error(name + " has invalid size/value");
    return Eigen::Map<const DecisionVector>(values.data(), values.size());
  }

  DecisionVector loadFiniteVectorParameter(
      const std::string& name, const std::vector<double>& fallback) {
    if (fallback.size() != static_cast<std::size_t>(decisionDofCount())) {
      throw std::logic_error("default parameter size does not match decision layout");
    }
    std::vector<double> values;
    if (!pnh_.getParam(name, values)) {
      values = fallback;
    }
    const bool valid =
        values.size() == static_cast<std::size_t>(decisionDofCount()) &&
        std::all_of(values.begin(), values.end(),
                    [](double value) { return std::isfinite(value); });
    if (!valid) {
      ROS_WARN_STREAM("[VelocityIkShadow] ~" << name
                      << " must contain " << decisionDofCount()
                      << " finite values for layout v" << kDofLayoutVersion
                      << "; using defaults");
      values = fallback;
    }
    DecisionVector result(decisionDofCount());
    for (int i = 0; i < decisionDofCount(); ++i) {
      result(i) = values.at(i);
    }
    return result;
  }

  Eigen::Vector3d loadVector3Parameter(const std::string& name,
                                       const Eigen::Vector3d& fallback) {
    std::vector<double> values;
    if (!pnh_.getParam(name, values)) {
      return fallback;
    }
    if (values.size() != 3 ||
        !std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })) {
      ROS_WARN_STREAM("[VelocityIkShadow] ~" << name << " must contain 3 finite values; using defaults");
      return fallback;
    }
    return Eigen::Vector3d(values.at(0), values.at(1), values.at(2));
  }

  static Eigen::Matrix3d rotationFromRpy(const Eigen::Vector3d& rpy) {
    return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  }

  void buildModel() {
    const std::string assets_path = ros::package::getPath("kuavo_assets");
    if (assets_path.empty()) {
      throw std::runtime_error("kuavo_assets package is not available");
    }

    const std::string version = std::to_string(robot_version_ == 15 ? 14 : robot_version_);
    const std::string config_path = assets_path + "/config/kuavo_v" + version + "/kuavo.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
      throw std::runtime_error("cannot open robot config: " + config_path);
    }
    nlohmann::json config;
    config_file >> config;
    const std::string private_config = ros::package::getPath("wheel_arm_abs_vel") +
        "/config/model_s" + version + ".json";
    std::ifstream overlay_file(private_config);
    if (!overlay_file.is_open())
      throw std::runtime_error("Missing absolute model contract: " + private_config);
    nlohmann::json overlay;
    overlay_file >> overlay;
    for (auto it = overlay.begin(); it != overlay.end(); ++it) {
      if (it.key().rfind("velocity_ik_", 0) != 0)
        throw std::runtime_error("Non-absolute key in model contract: " + it.key());
      config[it.key()] = it.value();
    }

    const int configured_arm_dof = config.at("NUM_ARM_JOINT").get<int>();
    if (configured_arm_dof != kDualArmDof) {
      throw std::runtime_error(
          "whole-body velocity IK requires two seven-DoF arms");
    }
    const int configured_joint_count = config.at("NUM_JOINT").get<int>();
    const int configured_head_dof = config.value("NUM_HEAD_JOINT", 0);
    if (sensor_arm_start_index_ < 0) {
      // Joint feedback is ordered as lower body/wheels, waist, arms, head.
      sensor_arm_start_index_ = configured_joint_count - configured_head_dof - configured_arm_dof;
    }
    if (sensor_arm_start_index_ < 0 ||
        sensor_arm_start_index_ + configured_arm_dof > configured_joint_count) {
      throw std::runtime_error("invalid sensor arm range for robot configuration");
    }
    if (sensor_arm_start_index_ < kLowerBodyDof) {
      throw std::runtime_error(
          "whole-body velocity IK requires four lower-body sensor joints before "
          "the arm range");
    }
    const auto frame_names = config.at("end_frames_name_ik").get<std::vector<std::string>>();
    if (frame_names.size() < 5) {
      throw std::runtime_error("end_frames_name_ik must contain base, two hands and two elbows");
    }
    const double eef_z_offset = config.value("eef_z_offset", 0.0);

    std::string model_relative;
    if (config.contains("velocity_ik_urdf")) {
      model_relative = config.at("velocity_ik_urdf").get<std::string>();
    } else if (config.contains("arm_urdf")) {
      model_relative = config.at("arm_urdf").get<std::string>();
    } else {
      model_relative = "biped_s" + version + "/urdf/drake/biped_v3_arm.urdf";
    }
    const std::string model_path = assets_path + "/models/" + model_relative;

    const auto fixed_joints = config.value(
        "velocity_ik_fixed_joints", std::vector<std::string>{});
    plant_ = std::make_unique<velocity_ik::ModelPinocchio>(
        model_path, frame_names.at(0), fixed_joints);
    if (config.contains("velocity_ik_urdf") &&
        (plant_->num_positions() != decisionDofCount() ||
         plant_->num_velocities() != decisionDofCount())) {
      throw std::runtime_error("Whole-body URDF reduction does not match the control layout");
    }
    pnh_.setParam("resolved_model_urdf", model_path);
    pnh_.setParam("resolved_model_fixed_joints", fixed_joints);
    ROS_INFO_STREAM("[PinocchioVelocityIk] URDF: " << model_path
        << ", nq=" << plant_->num_positions() << ", nv=" << plant_->num_velocities());

    const std::array<std::string, kArmCount> shadow_tcp_names{
        "velocity_ik_shadow_tcp_left", "velocity_ik_shadow_tcp_right"};
    const std::array<Eigen::Vector3d, kArmCount> tcp_translation{
        left_tcp_translation_, right_tcp_translation_};
    const std::array<Eigen::Vector3d, kArmCount> tcp_rpy{left_tcp_rpy_, right_tcp_rpy_};
    for (int arm = 0; arm < kArmCount; ++arm) {
      Eigen::Vector3d translation = tcp_translation.at(arm);
      translation.z() += eef_z_offset;
      plant_->addTcp(shadow_tcp_names.at(arm), frame_names.at(arm + 1),
                     translation, rotationFromRpy(tcp_rpy.at(arm)));
      hand_frame_names_.at(arm) = shadow_tcp_names.at(arm);
      elbow_frame_names_.at(arm) = frame_names.at(arm + 3);
      shoulder_frame_names_.at(arm) =
          arm == 0 ? "zarm_l1_link" : "zarm_r1_link";
    }
    base_frame_name_ = frame_names.at(0);
    plant_->Finalize();
    plant_context_ = plant_->CreateDefaultContext();
    feasibility_context_ = plant_->CreateDefaultContext();
    safety_context_ = plant_->CreateDefaultContext();
    supervisor_context_ = plant_->CreateDefaultContext();
    capture_context_ = plant_->CreateDefaultContext();

    q_full_ = plant_->GetPositions(*plant_context_);
    feasibility_q_full_ = plant_->GetPositions(*feasibility_context_);
    safety_q_full_ = plant_->GetPositions(*safety_context_);
    supervisor_q_full_ = plant_->GetPositions(*supervisor_context_);
    capture_q_full_ = plant_->GetPositions(*capture_context_);
    for (int decision_index = 0; decision_index < decisionDofCount();
         ++decision_index) {
      const auto& spec = dof_layout_.decisionSpec(decision_index);
      if (spec.model_joint_name.empty()) {
        throw std::runtime_error(
            spec.name + " has no Pinocchio joint in the active decision layout");
      }
      const auto& joint = plant_->GetJointByName(spec.model_joint_name);
      if (joint.num_positions() != 1 || joint.num_velocities() != 1) {
        throw std::runtime_error(
            spec.model_joint_name + " is not a one-DoF joint");
      }
      decision_position_index_.at(decision_index) = joint.position_start();
      decision_velocity_index_.at(decision_index) = joint.velocity_start();
      position_lower_(decision_index) = joint.position_lower_limits()(0);
      position_upper_(decision_index) = joint.position_upper_limits()(0);
      // A model-source correction must not silently widen the teleop domain.
      // Intersect the explicit operating box with the canonical URDF limits.
      if (config.contains("velocity_ik_position_limits") &&
          config.at("velocity_ik_position_limits").contains(spec.model_joint_name)) {
        const auto bounds = config.at("velocity_ik_position_limits")
            .at(spec.model_joint_name).get<std::vector<double>>();
        if (bounds.size() != 2 || !std::isfinite(bounds[0]) ||
            !std::isfinite(bounds[1]) || bounds[0] >= bounds[1]) {
          throw std::runtime_error("Invalid velocity IK position limits for " + spec.model_joint_name);
        }
        position_lower_(decision_index) = std::max(position_lower_(decision_index), bounds[0]);
        position_upper_(decision_index) = std::min(position_upper_(decision_index), bounds[1]);
        if (position_lower_(decision_index) >= position_upper_(decision_index))
          throw std::runtime_error("Empty velocity IK position range for " + spec.model_joint_name);
      }
    }
    for (int joint = 0; joint < kLowerBodyDof; ++joint) {
      decision_sensor_index_.at(lowerBodyDecisionIndex(joint)) = joint;
    }
    for (int arm = 0; arm < kArmCount; ++arm) {
      for (int joint = 0; joint < kJointsPerArm; ++joint) {
        decision_sensor_index_.at(armDecisionIndex(arm, joint)) =
            sensor_arm_start_index_ + arm * kJointsPerArm + joint;
      }
    }

    const auto peak_velocity_rad_s =
        config.at("joint_peak_velocity_limits").get<std::vector<double>>();
    if (peak_velocity_rad_s.size() < kLowerBodyDof) {
      throw std::runtime_error("missing lower-joint peak protection limits");
    }
    // Hardware checks consume this field in rad/s. Explicit ROS operating
    // limits also remain below that ceiling; no degree conversion here.
    for (int joint = 0; joint < kLowerBodyDof; ++joint) {
      const int index = lowerBodyDecisionIndex(joint);
      velocity_limit_(index) = motion_capture_ik::whole_body_ik::effectiveDriveVelocity(
          velocity_limit_(index), lower_joint_velocity_cap_.at(joint),
          peak_velocity_rad_s.at(joint));
      acceleration_limit_(index) = std::min(
          acceleration_limit_(index),
          lower_joint_acceleration_cap_.at(joint));
    }
    if (!pnh_.hasParam("joint_safe_center") &&
        config.contains("leg_calibration_safe_pose")) {
      const auto safe_pose_degrees =
          config.at("leg_calibration_safe_pose").get<std::vector<double>>();
      if (safe_pose_degrees.size() ==
          static_cast<std::size_t>(kLowerBodyDof)) {
        for (int joint = 0; joint < kLowerBodyDof; ++joint) {
          joint_safe_center_requested_(lowerBodyDecisionIndex(joint)) =
              safe_pose_degrees.at(joint) * M_PI / 180.0;
        }
      }
    }

    // By default velocity IK uses every joint's own URDF limits.  The old
    // shoulder-pitch cap remains an explicit rollback switch only; launch sets
    // it false, so v63 normally keeps its physical 0.6 rad upper limit.
    if (use_legacy_shoulder_pitch_cap_) {
      const int left_shoulder_pitch = armDecisionIndex(0, 0);
      const int right_shoulder_pitch = armDecisionIndex(1, 0);
      position_upper_(left_shoulder_pitch) = std::min(
          position_upper_(left_shoulder_pitch), legacy_shoulder_pitch_upper_);
      position_upper_(right_shoulder_pitch) = std::min(
          position_upper_(right_shoulder_pitch), legacy_shoulder_pitch_upper_);
      ROS_WARN_STREAM(
          "[VelocityIkShadow] legacy shoulder-pitch hard cap enabled: "
          << legacy_shoulder_pitch_upper_ << " rad");
    }
    // Keep the URDF interval as the absolute mechanical boundary, and derive
    // an inner operating envelope around a fixed natural posture.  The inner
    // envelope is deliberately not used to clamp q: outside it the controller
    // may still use the remaining physical buffer while it slows only the
    // outward component and always permits an inward escape.
    const DecisionVector physical_guard_lower =
        position_lower_.array() + position_limit_margin_ + boundary_brake_guard_;
    const DecisionVector physical_guard_upper =
        position_upper_.array() - position_limit_margin_ - boundary_brake_guard_;
    for (int i = 0; i < decisionDofCount(); ++i) {
      if (physical_guard_lower(i) >= physical_guard_upper(i)) {
        throw std::runtime_error(
            "joint physical range is too small for the configured safety guard");
      }
      joint_safe_center_(i) = std::clamp(
          joint_safe_center_requested_(i), physical_guard_lower(i),
          physical_guard_upper(i));
      if (std::abs(joint_safe_center_(i) - joint_safe_center_requested_(i)) >
          1.0e-12) {
        ROS_WARN_STREAM(
            "[VelocityIkShadow] joint " << i + 1
            << " safe center clamped from " << joint_safe_center_requested_(i)
            << " to " << joint_safe_center_(i));
      }
      joint_safe_lower_(i) = std::max(
          physical_guard_lower(i),
          joint_safe_center_(i) + joint_safe_range_ratio_(i) *
              (position_lower_(i) - joint_safe_center_(i)));
      joint_safe_upper_(i) = std::min(
          physical_guard_upper(i),
          joint_safe_center_(i) + joint_safe_range_ratio_(i) *
              (position_upper_(i) - joint_safe_center_(i)));
      if (joint_safe_lower_(i) >= joint_safe_center_(i) ||
          joint_safe_upper_(i) <= joint_safe_center_(i)) {
        throw std::runtime_error(
            "invalid joint operating envelope after applying safe range ratio");
      }
    }
    posture_position_ = joint_safe_center_;
    session_posture_position_ = posture_position_;
    preparation_position_ = joint_safe_center_;
    for (int joint = 0; joint < kLowerBodyDof; ++joint) {
      const int index = lowerBodyDecisionIndex(joint);
      const double ready = preparation_lower_position_requested_.at(joint);
      // Invalid endpoints are configuration errors, never silently clamped
      // into a different (raised) posture.
      motion_capture_ik::whole_body_ik::readyJointEnvelope(
          position_lower_(index), position_upper_(index), ready,
          position_limit_margin_, boundary_brake_guard_);
      preparation_position_(index) = ready;
    }
    for (int arm = 0; arm < kArmCount; ++arm) {
      for (int joint = 0; joint < kJointsPerArm; ++joint) {
        const int index = armDecisionIndex(arm, joint);
        const double requested = preparation_arm_position_requested_.at(
            static_cast<std::size_t>(arm * kJointsPerArm + joint));
        preparation_position_(index) = std::clamp(
            requested, joint_safe_lower_(index), joint_safe_upper_(index));
      }
    }
    startup_preparation_position_ = preparation_position_;
    setPlantDecisionPositions(preparation_position_);
    const auto natural_torso_pose =
        plant_->GetFrameByName(torso_frame_name_)
            .CalcPose(*plant_context_,
                      plant_->GetFrameByName(base_frame_name_));
    torso_x_center_ = natural_torso_pose.translation().x();
    torso_z_center_ = natural_torso_pose.translation().z();
    torso_pitch_center_ =
        preparation_position_(lowerBodyDecisionIndex(0)) +
        preparation_position_(lowerBodyDecisionIndex(1)) +
        preparation_position_(lowerBodyDecisionIndex(2));
    torso_yaw_center_ = preparation_position_(lowerBodyDecisionIndex(3));
    const std::array<double, 4> ready_feature{
        torso_x_center_, torso_z_center_, torso_pitch_center_, torso_yaw_center_};
    for (int i = 0; i < 4; ++i) {
      torso_workspace_lower_.at(i) = ready_feature.at(i) + torso_workspace_lower_offset_.at(i);
      torso_workspace_upper_.at(i) = ready_feature.at(i) + torso_workspace_upper_offset_.at(i);
    }
    pnh_.setParam("resolved_torso_workspace_lower",
        std::vector<double>(torso_workspace_lower_.begin(), torso_workspace_lower_.end()));
    pnh_.setParam("resolved_torso_workspace_upper",
        std::vector<double>(torso_workspace_upper_.begin(), torso_workspace_upper_.end()));
    pnh_.setParam("resolved_joint_velocity_limits",
        std::vector<double>(velocity_limit_.data(), velocity_limit_.data() + velocity_limit_.size()));
    pnh_.setParam("resolved_preparation_position",
        std::vector<double>(preparation_position_.data(), preparation_position_.data() + preparation_position_.size()));
    ROS_INFO_STREAM("[VelocityIkShadow] explicit low-ready waist q="
        << preparation_position_.segment(kBaseDof, kLowerBodyDof).transpose()
        << "; operating waist velocity (rad/s)="
        << velocity_limit_.segment(kBaseDof, kLowerBodyDof).transpose());

    ROS_INFO_STREAM("[VelocityIkShadow] Pinocchio model: " << model_path << ", nq=" << plant_->num_positions()
                                                        << ", nv=" << plant_->num_velocities()
                                                        << ", full layout=" << dof_layout_.fullDofCount()
                                                        << ", active decision=" << decisionDofCount()
                                                        << ", sensor arm range=[" << sensor_arm_start_index_
                                                        << ", " << sensor_arm_start_index_ + kDualArmDof << ")");
    ROS_INFO_STREAM(
        "[VelocityIkShadow] torso constraint center [x,z,pitch,yaw]="
        << torso_x_center_ << "," << torso_z_center_ << ","
        << torso_pitch_center_ << "," << torso_yaw_center_);
    ROS_INFO_STREAM(
        "[VelocityIkShadow] active decision names: "
        << [&]() {
             std::string names;
             for (const auto& name : dof_layout_.decisionNames()) {
               if (!names.empty()) {
                 names += ",";
               }
               names += name;
             }
             return names;
           }());
    ROS_INFO_STREAM(
        "[VelocityIkShadow] joint safe range ratios: "
        << joint_safe_range_ratio_.transpose());
    ROS_INFO_STREAM(
        "[VelocityIkShadow] joint safe lower: "
        << joint_safe_lower_.transpose());
    ROS_INFO_STREAM(
        "[VelocityIkShadow] joint safe upper: "
        << joint_safe_upper_.transpose());
  }

  void publishInputDebug(const wheel_arm_abs_vel::TeleopTcpTarget& input,
                         uint8_t decision, bool accepted, bool duplicate,
                         double packet_dt, double source_stamp_dt,
                         double unique_sample_dt, double feedforward_sample_age,
                         double max_position_delta, double max_orientation_delta,
                         std::unique_lock<std::mutex>* data_lock) {
    if (!publish_input_debug_) {
      data_lock->unlock();
      return;
    }
    wheel_arm_abs_vel::VelocityIkInputDebug debug;
    debug.header.seq = input_debug_sequence_++;
    debug.header.stamp = ros::Time::now();
    debug.header.frame_id = input.header.frame_id;
    debug.source_stamp = input.header.stamp;
    debug.source = input.source;
    debug.decision = decision;
    debug.accepted = accepted;
    debug.duplicate = duplicate;
    debug.packet_dt = packet_dt;
    debug.source_stamp_dt = source_stamp_dt;
    debug.unique_sample_dt = unique_sample_dt;
    debug.feedforward_sample_age = feedforward_sample_age;
    debug.max_position_delta = max_position_delta;
    debug.max_orientation_delta = max_orientation_delta;
    debug.received_count = received_input_count_;
    debug.accepted_count = accepted_input_count_;
    debug.unique_count = unique_input_count_;
    debug.duplicate_count = duplicate_input_count_;
    debug.rejected_source_count = rejected_source_count_;
    debug.rejected_invalid_count = rejected_invalid_count_;
    std_msgs::Float64MultiArray observations;
    observations.data={1.,double(input.header.seq),input.header.stamp.toSec(),
        target_.receive_monotonic_ns,double(input.session_id),double(input.generation),
        double(input.control_state),accepted?1.:0.};
    const auto append=[&](const velocity_ik::ChannelObservation& c) {
      auto& v=observations.data;
      v.insert(v.end(),{c.valid?1.:0.,c.changed?1.:0.,c.source_time,c.update_host_ns,
          c.difference_dt,double(c.updates),c.linear.x(),c.linear.y(),c.linear.z(),
          c.angular.x(),c.angular.y(),c.angular.z(),0.});
    };
    for(int arm=0;arm<kArmCount;++arm) {
      append(target_.hand_observation.at(arm));append(target_.elbow_observation.at(arm));
    }
    velocity_ik::ChannelObservation body;
    body.valid=target_.body_feedforward_sample_valid;
    body.changed=target_.body_observation_changed;
    body.source_time=target_.body_feedforward_sample_stamp.toSec();
    body.update_host_ns=target_.body_observation_host_ns;
    body.difference_dt=target_.body_observation_dt;
    body.updates=target_.body_observation_updates;
    body.linear=target_.body.chest_linear_ff;body.angular=target_.body.chest_angular_ff;
    append(body);
    data_lock->unlock();
    const auto publisher=input_debug_pub_,channel_publisher=channel_observation_pub_;
    diagnostics_.submit([publisher,channel_publisher,debug,observations]{
      publisher.publish(debug);channel_publisher.publish(observations);
    });
  }

  bool sourceAllowed(uint8_t source) const {
    if (tracking_source_mode_ == TrackingSourceMode::kAuto) {
      return source == wheel_arm_abs_vel::TeleopTcpTarget::SOURCE_QUEST_CONTROLLER ||
          source == wheel_arm_abs_vel::TeleopTcpTarget::SOURCE_QUEST_HAND_TRACKING;
    }
    if (tracking_source_mode_ == TrackingSourceMode::kController) {
      return source == wheel_arm_abs_vel::TeleopTcpTarget::SOURCE_QUEST_CONTROLLER;
    }
    return source == wheel_arm_abs_vel::TeleopTcpTarget::SOURCE_QUEST_HAND_TRACKING;
  }

  void targetCallback(const wheel_arm_abs_vel::TeleopTcpTarget::ConstPtr& message) {
    TargetSnapshot next;
    next.receive_time = ros::WallTime::now();
    next.receive_monotonic_ns = monotonicNs();
#ifdef VELOCITY_IK_OFFLINE_TEST
    if(offline_clock_valid_) {
      next.receive_time.fromSec(offline_receive_ros_);
      next.receive_monotonic_ns=offline_receive_ns_;
    }
#endif
    next.source_frame_seq = message->header.seq;
    next.source_stamp = message->header.stamp;
    const bool timing_valid = std::isfinite(message->source_relative_age) &&
        std::isfinite(message->mapper_processing_time) &&
        message->source_relative_age >= 0.0 && message->mapper_processing_time >= 0.0;
    const double transport_age = velocity_ik::transportAge(
        message->mapper_publish_stamp.toNSec(),
#ifdef VELOCITY_IK_OFFLINE_TEST
        offline_clock_valid_ ? uint64_t(offline_receive_ros_*1.e9) :
#endif
        ros::Time::now().toNSec());
    next.source_relative_age = timing_valid
        ? message->source_relative_age + message->mapper_processing_time + transport_age
        : std::numeric_limits<double>::infinity();
    next.frame_id = message->header.frame_id;
    next.source = message->source;
    next.session_id = message->session_id;
    next.generation = message->generation;
    next.absolute_state = message->absolute_state;
    next.control_state = message->control_state;

    const std::array<const geometry_msgs::Pose*, kArmCount> pose_input{
        &message->left_tcp_pose, &message->right_tcp_pose};
    const std::array<const geometry_msgs::Point*, kArmCount> elbow_input{
        &message->left_elbow_hint, &message->right_elbow_hint};
    const std::array<bool, kArmCount> pose_valid{
        static_cast<bool>(message->left_valid), static_cast<bool>(message->right_valid)};
    const std::array<bool, kArmCount> elbow_valid{
        static_cast<bool>(message->left_elbow_hint_valid),
        static_cast<bool>(message->right_elbow_hint_valid)};
    const bool explicit_session_frame =
        message->header.frame_id == session_frame_name_;
    const bool legacy_frozen_base_frame =
        allow_legacy_base_link_session_frame_ &&
        message->header.frame_id == base_frame_name_;
    const bool frame_valid = !strict_target_frame_ ||
        message->header.frame_id == "quest_chest_robot_axes" ||
        explicit_session_frame || legacy_frozen_base_frame;
    const bool controlled_frames_valid =
        message->left_controlled_frame == left_controlled_frame_ &&
        message->right_controlled_frame == right_controlled_frame_;
    const bool lifecycle_valid =
        message->control_state ==
            wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_ARMED ||
        message->control_state ==
            wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_ACTIVE ||
        message->control_state ==
            wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_HOLD ||
        message->control_state ==
            wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_PREPARE;
    const bool contract_valid = frame_valid && controlled_frames_valid &&
        message->control_valid && lifecycle_valid;
    bool valid = contract_valid;
    std::array<bool, kArmCount> hand_valid{{false, false}};
    for (int arm = 0; arm < kArmCount; ++arm) {
      const auto& pose = *pose_input.at(arm);
      const auto& elbow = *elbow_input.at(arm);
      next.arm.at(arm).hand_position =
          Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
      next.arm.at(arm).hand_orientation = Eigen::Quaterniond(
          pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
      next.arm.at(arm).elbow_position = Eigen::Vector3d(elbow.x, elbow.y, elbow.z);
      next.arm.at(arm).elbow_valid = elbow_valid.at(arm) &&
          allFinite(next.arm.at(arm).elbow_position);
      const bool this_pose_valid = pose_valid.at(arm) &&
          allFinite(next.arm.at(arm).hand_position) &&
          next.arm.at(arm).hand_orientation.coeffs().array().isFinite().all() &&
          next.arm.at(arm).hand_orientation.norm() > 0.5;
      hand_valid[arm]=this_pose_valid;
      if (armIsControlled(arm)) {
        valid = valid && this_pose_valid;
      }
      if (next.arm.at(arm).hand_orientation.norm() > 1.0e-12) {
        next.arm.at(arm).hand_orientation.normalize();
      }
    }

    next.body.frame_id = message->body_tracking_frame;
    next.body.coordinate_epoch = message->body_coordinate_epoch;
    next.body.high_confidence = message->body_high_confidence;
    const auto read_body_pose = [](
        const geometry_msgs::Pose& pose, bool declared_valid,
        Eigen::Vector3d* position, Eigen::Quaterniond* orientation) {
      *position = Eigen::Vector3d(
          pose.position.x, pose.position.y, pose.position.z);
      *orientation = Eigen::Quaterniond(
          pose.orientation.w, pose.orientation.x,
          pose.orientation.y, pose.orientation.z);
      const bool valid = declared_valid && allFinite(*position) &&
          orientation->coeffs().array().isFinite().all() &&
          orientation->norm() > 0.5;
      if (valid) {
        orientation->normalize();
      } else {
        *position = Eigen::Vector3d::Zero();
        *orientation = Eigen::Quaterniond::Identity();
      }
      return valid;
    };
    next.body.root_valid = read_body_pose(
        message->root_pose, message->root_valid,
        &next.body.root_position, &next.body.root_orientation);
    next.body.chest_valid = read_body_pose(
        message->chest_pose, message->chest_valid,
        &next.body.chest_position, &next.body.chest_orientation);
    Eigen::Quaterniond unused_orientation = Eigen::Quaterniond::Identity();
    next.body.head_valid = read_body_pose(
        message->head_pose, message->head_valid,
        &next.body.head_position, &unused_orientation);
    next.body.left_shoulder_valid = read_body_pose(
        message->left_shoulder_pose, message->left_shoulder_valid,
        &next.body.left_shoulder_position, &unused_orientation);
    next.body.right_shoulder_valid = read_body_pose(
        message->right_shoulder_pose, message->right_shoulder_valid,
        &next.body.right_shoulder_position, &unused_orientation);
    if (next.body.frame_id != body_tracking_frame_name_) {
      next.body.chest_valid = false;
    }

    valid = valid && next.body.torsoValid() &&
        std::isfinite(next.source_relative_age) && next.source_relative_age <= target_timeout_;
    const bool source_allowed = sourceAllowed(message->source);
    std::unique_lock<std::mutex> lock(data_mutex_);
    // A transaction identity change does not make an older source observation
    // new. Reject it before resetting channel histories or PREPARE admission.
    // Same-stamp lifecycle commands remain admissible; /stop_robot is separate.
    if (source_allowed && next.source == target_.source &&
        !next.source_stamp.isZero() && next.source_stamp < target_.source_stamp &&
        (next.session_id != target_.session_id ||
         next.generation != target_.generation)) {
      ++rejected_invalid_count_;
      return;
    }
    if (target_.valid && target_.control_state==wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_ACTIVE &&
        contract_valid && source_allowed && next.session_id==target_.session_id && next.generation==target_.generation &&
        (next.source!=target_.source || next.frame_id!=target_.frame_id ||
         next.body.coordinate_epoch!=target_.body.coordinate_epoch)) force_confirmation_=true;
    const bool same_contract = target_.valid && next.source==target_.source &&
        next.session_id==target_.session_id && next.generation==target_.generation &&
        next.frame_id==target_.frame_id && next.body.coordinate_epoch==target_.body.coordinate_epoch;
    const bool new_observation = !next.source_stamp.isZero() &&
        (!same_contract || next.source_stamp>target_.source_stamp);
    if (same_contract) next.observation_ns=target_.observation_ns;
    if (contract_valid && source_allowed &&
        (!std::isfinite(next.source_relative_age) || next.source_relative_age>input_continuity_.config.brake)) {
      ++rejected_invalid_count_;
      source_interrupted_=true;
      return; // A delayed/bad-timing packet cannot destroy a recent good pose.
    }
    if (same_contract && contract_valid && next.control_state==target_.control_state &&
        (!new_observation || !std::isfinite(next.source_relative_age))) {
      ++duplicate_input_count_;
      return; // Reordered/replayed packets cannot renew observations or FF.
    }
    // Explicit lifecycle/authority loss is never hidden by a pose cache.
    // Shared torso is a dependency of both hands. A prolonged single-channel
    // outage therefore conservatively brakes the coupled whole body.
    if (contract_valid && source_allowed && std::isfinite(next.source_relative_age) &&
        next.source_relative_age<=input_continuity_.config.brake) {
      if (same_contract) next.observation_ns=target_.observation_ns;
      const double observed_ns=next.receive_monotonic_ns-next.source_relative_age*1e9;
      for (int arm=0; arm<kArmCount; ++arm) {
        if (new_observation && hand_valid[arm]) next.observation_ns[arm]=observed_ns;
        else if (same_contract && target_.observation_ns[arm]>0.) {
          next.arm[arm]=target_.arm[arm];
          next.arm[arm].elbow_valid=false;
          hand_valid[arm]=true;
        }
      }
      if (new_observation && next.body.torsoValid()) next.observation_ns[2]=observed_ns;
      else if (same_contract && target_.observation_ns[2]>0.) next.body=target_.body;
      valid=next.body.torsoValid() && next.observation_ns[2]>0.;
      for (int arm=0;arm<kArmCount;++arm)
        if (armIsControlled(arm)) valid &= hand_valid[arm] && next.observation_ns[arm]>0.;
      const bool all_new=valid && new_observation && next.observation_ns[2]==observed_ns &&
          (!armIsControlled(0) || next.observation_ns[0]==observed_ns) &&
          (!armIsControlled(1) || next.observation_ns[1]==observed_ns);
      next.observation_sequence=target_.observation_sequence+(all_new ? 1 : 0);
    }
    ++received_input_count_;
    // Admission requires a valid configured-source pose once. Continuation of
    // that fixed transaction uses fresh lifecycle commands, not body/hand FK.
    fixed_prepare_transaction_.observe(
        next.session_id, next.generation, next.body.coordinate_epoch,
        next.source_frame_seq, next.source_stamp.toSec(), next.receive_monotonic_ns,
        next.source_relative_age, target_timeout_,
        next.control_state == wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_PREPARE,
        frame_valid && controlled_frames_valid && message->control_valid &&
            (next.source == wheel_arm_abs_vel::TeleopTcpTarget::SOURCE_QUEST_CONTROLLER ||
             next.source == wheel_arm_abs_vel::TeleopTcpTarget::SOURCE_QUEST_HAND_TRACKING),
        valid && source_allowed);

    double packet_dt = 0.0;
    if (!last_input_receive_time_.isZero()) {
      packet_dt = (next.receive_time - last_input_receive_time_).toSec();
    }
    double source_stamp_dt = 0.0;
    if (!next.source_stamp.isZero() && !last_input_source_stamp_.isZero() &&
        next.source_stamp > last_input_source_stamp_) {
      source_stamp_dt = (next.source_stamp - last_input_source_stamp_).toSec();
    }
    last_input_receive_time_ = next.receive_time;
    last_input_source_stamp_ = next.source_stamp;

    double max_position_delta = -1.0;
    double max_orientation_delta = -1.0;
    double max_body_position_delta = -1.0;
    double max_body_orientation_delta = -1.0;
    if (valid && target_.valid) {
      max_position_delta = 0.0;
      max_orientation_delta = 0.0;
      for (int arm = 0; arm < kArmCount; ++arm) {
        Eigen::Quaterniond& q_new = next.arm.at(arm).hand_orientation;
        const Eigen::Quaterniond& q_old = target_.arm.at(arm).hand_orientation;
        if (q_new.coeffs().dot(q_old.coeffs()) < 0.0) {
          q_new.coeffs() *= -1.0;
        }
        max_position_delta = std::max(
            max_position_delta,
            (next.arm.at(arm).hand_position - target_.arm.at(arm).hand_position).norm());
        max_orientation_delta = std::max(
            max_orientation_delta,
            rotationLog((q_new * q_old.conjugate()).toRotationMatrix()).norm());
        if (next.arm.at(arm).elbow_valid != target_.arm.at(arm).elbow_valid) {
          max_position_delta = std::numeric_limits<double>::infinity();
        } else if (next.arm.at(arm).elbow_valid) {
          max_position_delta = std::max(
              max_position_delta,
              (next.arm.at(arm).elbow_position - target_.arm.at(arm).elbow_position).norm());
        }
      }
    }
    if (valid && target_.valid && next.body.torsoValid() &&
        target_.body.torsoValid() &&
        next.body.coordinate_epoch == target_.body.coordinate_epoch) {
      max_body_position_delta = std::max(
          (next.body.root_position - target_.body.root_position).norm(),
          (next.body.chest_position - target_.body.chest_position).norm());
      Eigen::Quaterniond chest_orientation = next.body.chest_orientation;
      if (chest_orientation.coeffs().dot(
              target_.body.chest_orientation.coeffs()) < 0.0) {
        chest_orientation.coeffs() *= -1.0;
        next.body.chest_orientation = chest_orientation;
      }
      max_body_orientation_delta = rotationLog(
          (chest_orientation * target_.body.chest_orientation.conjugate())
              .toRotationMatrix()).norm();
    }

    const auto current_feedforward_age = [&]() {
      return target_.feedforward_sample_valid
          ? std::max(0.0, (next.receive_time - target_.feedforward_sample_receive_time).toSec())
          : -1.0;
    };
    if (!source_allowed) {
      ++rejected_source_count_;
      // Ignore a wrong-source packet; it cannot renew the configured source.
      source_interrupted_ = true;
      publishInputDebug(
          *message, wheel_arm_abs_vel::VelocityIkInputDebug::REJECTED_SOURCE,
          false, false, packet_dt, source_stamp_dt, 0.0,
          current_feedforward_age(), max_position_delta, max_orientation_delta, &lock);
      return;
    }

    if (!valid) {
      ++rejected_invalid_count_;
      target_source_active_ = false;
      source_interrupted_ = true;
      next.sequence = ++target_sequence_counter_;
      next.valid = false;
      target_ = next;
      publishInputDebug(
          *message, wheel_arm_abs_vel::VelocityIkInputDebug::REJECTED_INVALID,
          false, false, packet_dt, source_stamp_dt, 0.0, -1.0,
          max_position_delta, max_orientation_delta, &lock);
      return;
    }

    next.sequence = ++target_sequence_counter_;
    // Controller grip and optical Palm are distinct source frames.  Fixed
    // modes reject the other source; auto remains diagnostic-only.  Even in
    // auto, never differentiate feedforward across a source handoff.
    const bool source_changed = target_.valid && next.source != target_.source;
    const bool coordinate_contract_changed = target_.valid &&
        (next.session_id != target_.session_id ||
         next.generation != target_.generation ||
         next.frame_id != target_.frame_id ||
         next.body.coordinate_epoch != target_.body.coordinate_epoch ||
         next.source_stamp < target_.source_stamp ||
         next.control_state != target_.control_state);
    const bool has_continuous_sample = target_.valid &&
        target_.feedforward_sample_valid && !source_interrupted_ &&
        !source_changed && !coordinate_contract_changed &&
        (next.receive_monotonic_ns-target_.receive_monotonic_ns)*1e-9<=input_continuity_.config.grace;
    const bool duplicate = has_continuous_sample &&
        max_position_delta <= duplicate_position_epsilon_ &&
        max_orientation_delta <= duplicate_orientation_epsilon_;
    double unique_sample_dt = 0.0;
    uint8_t decision = wheel_arm_abs_vel::VelocityIkInputDebug::ACCEPTED_INITIAL;

    // Keep the aggregate debug counters for compatibility; they no longer
    // control either hand or elbow estimator.
    if (duplicate) {
      decision = wheel_arm_abs_vel::VelocityIkInputDebug::ACCEPTED_DUPLICATE;
      next.feedforward_sample_receive_time = target_.feedforward_sample_receive_time;
      next.feedforward_sample_stamp = target_.feedforward_sample_stamp;
      next.unique_sequence = target_.unique_sequence;
      ++duplicate_input_count_;
    } else {
      if (has_continuous_sample) {
        decision = wheel_arm_abs_vel::VelocityIkInputDebug::ACCEPTED_UNIQUE;
        unique_sample_dt = next.source_stamp > target_.feedforward_sample_stamp &&
            !target_.feedforward_sample_stamp.isZero()
            ? (next.source_stamp-target_.feedforward_sample_stamp).toSec()
            : (next.receive_time-target_.feedforward_sample_receive_time).toSec();
      }
      next.feedforward_sample_receive_time=next.receive_time;
      next.feedforward_sample_stamp=next.source_stamp;
      next.unique_sequence=++unique_sequence_counter_;
      ++unique_input_count_;
    }
    next.feedforward_sample_valid=true;
    for (int arm=0;arm<kArmCount;++arm) {
      auto& a=next.arm.at(arm);
      const bool channel_new=next.observation_ns[arm]>target_.observation_ns[arm];
      next.hand_observation.at(arm)=velocity_ik::observeChannel(
          target_.hand_observation.at(arm),a.hand_position,a.hand_orientation,
          true,has_continuous_sample &&
          (next.observation_ns[arm]-target_.observation_ns[arm])*1e-9<=input_continuity_.config.grace,
          next.source_stamp.toSec(),next.receive_monotonic_ns,
          ff_filter_time_constant_,duplicate_position_epsilon_,duplicate_orientation_epsilon_);
      if (!channel_new && same_contract) next.hand_observation[arm]=target_.hand_observation[arm];
      next.elbow_observation.at(arm)=velocity_ik::observeChannel(
          target_.elbow_observation.at(arm),a.elbow_position,Eigen::Quaterniond::Identity(),
          a.elbow_valid,has_continuous_sample,next.source_stamp.toSec(),next.receive_monotonic_ns,
          ff_filter_time_constant_,duplicate_position_epsilon_,duplicate_orientation_epsilon_);
      a.hand_linear_ff=next.hand_observation.at(arm).linear;
      a.hand_angular_ff=next.hand_observation.at(arm).angular;
      a.elbow_linear_ff=next.elbow_observation.at(arm).linear;
    }

    // Root/Chest have their own uniqueness and velocity observation.  A pure
    // body motion intentionally leaves the chest-relative hands unchanged, so
    // sharing the arm duplicate decision discarded all body feedforward.
    const bool has_continuous_body_sample = target_.valid &&
        target_.body_feedforward_sample_valid && !source_interrupted_ &&
        !source_changed && !coordinate_contract_changed &&
        next.body.torsoValid() && target_.body.torsoValid() &&
        next.body.coordinate_epoch == target_.body.coordinate_epoch &&
        (next.observation_ns[2]-target_.observation_ns[2])*1e-9<=input_continuity_.config.grace;
    const bool body_duplicate = has_continuous_body_sample &&
        max_body_position_delta >= 0.0 &&
        max_body_position_delta <= duplicate_position_epsilon_ &&
        max_body_orientation_delta >= 0.0 &&
        max_body_orientation_delta <= duplicate_orientation_epsilon_;
    next.body_observation_changed=!body_duplicate;
    next.body_observation_updates=target_.body_observation_updates+(body_duplicate?0:1);
    next.body_observation_host_ns=body_duplicate ? target_.body_observation_host_ns : next.receive_monotonic_ns;
    if (body_duplicate) {
      next.body.root_linear_ff = target_.body.root_linear_ff;
      next.body.chest_linear_ff = target_.body.chest_linear_ff;
      next.body.chest_angular_ff = target_.body.chest_angular_ff;
      next.body_feedforward_sample_receive_time =
          target_.body_feedforward_sample_receive_time;
      next.body_feedforward_sample_stamp =
          target_.body_feedforward_sample_stamp;
      next.body_feedforward_sample_valid = true;
    } else {
      double body_sample_dt = 0.0;
      if (has_continuous_body_sample) {
        if (!next.source_stamp.isZero() &&
            !target_.body_feedforward_sample_stamp.isZero() &&
            next.source_stamp > target_.body_feedforward_sample_stamp) {
          body_sample_dt =
              (next.source_stamp -
               target_.body_feedforward_sample_stamp).toSec();
        } else {
          body_sample_dt =
              (next.receive_time -
               target_.body_feedforward_sample_receive_time).toSec();
        }
      }
      if (has_continuous_body_sample && body_sample_dt >= 0.003 &&
          body_sample_dt <= 0.100) {
        const double alpha = std::clamp(
            body_sample_dt /
                std::max(body_sample_dt + ff_filter_time_constant_, 1.0e-6),
            0.0, 1.0);
        const Eigen::Vector3d raw_root_linear =
            (next.body.root_position - target_.body.root_position) /
            body_sample_dt;
        const Eigen::Vector3d raw_chest_linear =
            (next.body.chest_position - target_.body.chest_position) /
            body_sample_dt;
        const Eigen::Vector3d raw_chest_angular = rotationLog(
            (next.body.chest_orientation *
             target_.body.chest_orientation.conjugate())
                .toRotationMatrix()) / body_sample_dt;
        next.body.root_linear_ff =
            (1.0 - alpha) * target_.body.root_linear_ff +
            alpha * raw_root_linear;
        next.body.chest_linear_ff =
            (1.0 - alpha) * target_.body.chest_linear_ff +
            alpha * raw_chest_linear;
        next.body.chest_angular_ff =
            (1.0 - alpha) * target_.body.chest_angular_ff +
            alpha * raw_chest_angular;
      }
      next.body_observation_dt = body_sample_dt;
      next.body_feedforward_sample_receive_time = next.receive_time;
      next.body_feedforward_sample_stamp = next.source_stamp;
      next.body_feedforward_sample_valid = next.body.torsoValid();
    }

    // Explicit estimated twists have the same source stamp and pose frames.
    // They REPLACE legacy differences, including equal-content NEW samples.
    next.estimated_twists_valid=message->estimated_twists_valid;
    if (same_contract && target_.estimated_twists_valid &&
        next.observation_ns==target_.observation_ns)
      next.estimated_twists_valid=true; // brief occlusion keeps the admitted path
    if (message->estimated_twists_valid) {
      const auto vector_components=[](const geometry_msgs::Vector3& v){return Eigen::Vector3d(v.x,v.y,v.z);};
      const std::array<geometry_msgs::Twist,2> twists{message->left_tcp_twist,message->right_tcp_twist};
      const std::array<geometry_msgs::Vector3,2> elbows{message->left_elbow_velocity,message->right_elbow_velocity};
      bool finite=vector_components(message->chest_twist.linear).allFinite() && vector_components(message->chest_twist.angular).allFinite() && vector_components(message->root_velocity).allFinite();
      for(int arm=0;arm<kArmCount;++arm)
        finite=finite && vector_components(twists[arm].linear).allFinite() && vector_components(twists[arm].angular).allFinite() && vector_components(elbows[arm]).allFinite();
      if(!finite) { ++rejected_invalid_count_; source_interrupted_=true; return; }
      for(int arm=0;arm<kArmCount;++arm) {
        if(next.observation_ns[arm]<=target_.observation_ns[arm] && same_contract) continue;
        auto& a=next.arm[arm];
        a.hand_linear_ff=vector_components(twists[arm].linear); a.hand_angular_ff=vector_components(twists[arm].angular);
        a.elbow_linear_ff=vector_components(elbows[arm]);
        auto& h=next.hand_observation[arm];
        h.position=a.hand_position;h.orientation=a.hand_orientation;
        h.linear=a.hand_linear_ff;h.angular=a.hand_angular_ff;h.valid=true;h.changed=true;
        h.update_host_ns=next.observation_ns[arm];h.source_time=next.source_stamp.toSec();
        h.updates=target_.hand_observation[arm].updates+1;
        auto& e=next.elbow_observation[arm];
        e.position=a.elbow_position;e.linear=a.elbow_linear_ff;e.valid=a.elbow_valid;
        e.update_host_ns=h.update_host_ns;e.source_time=h.source_time;
      }
      if(next.observation_ns[2]>target_.observation_ns[2] || !same_contract) {
        next.body.root_linear_ff=vector_components(message->root_velocity);
        next.body.chest_linear_ff=vector_components(message->chest_twist.linear);
        next.body.chest_angular_ff=vector_components(message->chest_twist.angular);
        next.body_observation_host_ns=next.observation_ns[2];
        next.body_observation_changed=true;
        next.body_observation_updates=target_.body_observation_updates+1;
        next.body_feedforward_sample_receive_time=next.receive_time;
      }
    }
    next.valid = true;
    target_source_active_ = true;
    source_interrupted_ = false;
    ++accepted_input_count_;
    target_ = next;
    const double feedforward_sample_age = duplicate
        ? std::max(0.0, (next.receive_time - next.feedforward_sample_receive_time).toSec())
        : 0.0;
    publishInputDebug(
        *message, decision, true, duplicate, packet_dt, source_stamp_dt,
        unique_sample_dt, feedforward_sample_age,
        max_position_delta, max_orientation_delta, &lock);
  }

  void robotStateCallback(const kuavo_msgs::sensorsData::ConstPtr& message) {
    const auto& joint_data = message->joint_data;
    const auto maximum_sensor_index = std::max_element(
        decision_sensor_index_.begin(), decision_sensor_index_.end());
    if (maximum_sensor_index == decision_sensor_index_.end() ||
        *maximum_sensor_index < 0) {
      ROS_ERROR_THROTTLE(
          2.0, "[VelocityIkShadow] active decision layout has no sensor mapping");
      return;
    }
    const std::size_t required_size =
        static_cast<std::size_t>(*maximum_sensor_index + 1);
    if (joint_data.joint_q.size() < required_size || joint_data.joint_v.size() < required_size) {
      ROS_WARN_THROTTLE(
          2.0,
          "[VelocityIkShadow] %s requires at least %zu q/v values, received q=%zu v=%zu",
          robot_state_topic_.c_str(), required_size,
          joint_data.joint_q.size(), joint_data.joint_v.size());
      return;
    }
    DecisionVector q = DecisionVector::Zero(decisionDofCount());
    DecisionVector dq = DecisionVector::Zero(decisionDofCount());
    for (int i = 0; i < decisionDofCount(); ++i) {
      if (decision_sensor_index_.at(i) < 0) {
        continue;
      }
      const std::size_t source_index =
          static_cast<std::size_t>(decision_sensor_index_.at(i));
      q(i) = joint_data.joint_q.at(source_index);
      dq(i) = joint_data.joint_v.at(source_index);
      if (!std::isfinite(q(i)) || !std::isfinite(dq(i))) {
        ROS_WARN_THROTTLE(
            2.0,
            "[VelocityIkShadow] %s mechanism q/v contains non-finite data",
            robot_state_topic_.c_str());
        return;
      }
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!robot_measurement_freshness_.observe(message->header.stamp.toNSec(),
            ros::Time::now().toNSec(), static_cast<int64_t>(monotonicNs()), robot_state_timeout_)) {
      ROS_WARN_THROTTLE(2.0, "[VelocityIk] stale/repeated/future joint measurement rejected");
      return;
    }
    for (int i = 0; i < decisionDofCount(); ++i) {
      if (decision_sensor_index_.at(i) >= 0) {
        measured_q_(i) = q(i);
        measured_dq_(i) = dq(i);
      }
    }
    robot_state_receive_time_ = ros::WallTime::now();
    sensor_source_stamp_=message->header.stamp;
    sensor_source_seq_=message->header.seq;
    sensor_receive_ns_=monotonicNs();
    robot_state_valid_ = true;
  }

  static double shortestAngularDelta(double from, double to) {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  void baseStateCallback(const nav_msgs::Odometry::ConstPtr& message) {
    if (strict_base_state_frame_ &&
        message->header.frame_id != world_frame_name_) {
      ROS_WARN_THROTTLE(
          2.0,
          "[VelocityIkShadow] base state frame must be %s, received %s",
          world_frame_name_.c_str(), message->header.frame_id.c_str());
      return;
    }
    if (strict_base_state_frame_ &&
        message->child_frame_id != base_frame_name_) {
      ROS_WARN_THROTTLE(
          2.0,
          "[VelocityIkShadow] base state child frame must be %s because twist is interpreted in the base frame, received %s",
          base_frame_name_.c_str(), message->child_frame_id.c_str());
      return;
    }
    Eigen::Quaterniond orientation(
        message->pose.pose.orientation.w,
        message->pose.pose.orientation.x,
        message->pose.pose.orientation.y,
        message->pose.pose.orientation.z);
    const Eigen::Vector2d position(
        message->pose.pose.position.x, message->pose.pose.position.y);
    const Eigen::Vector2d velocity_body(
        message->twist.twist.linear.x,
        message->twist.twist.linear.y);
    const double yaw_rate = message->twist.twist.angular.z;
    if (!position.array().isFinite().all() ||
        !velocity_body.array().isFinite().all() ||
        !orientation.coeffs().array().isFinite().all() ||
        orientation.norm() < 0.5 || !std::isfinite(yaw_rate)) {
      ROS_WARN_THROTTLE(
          2.0, "[VelocityIkShadow] %s contains invalid planar base state",
          base_state_topic_.c_str());
      return;
    }
    orientation.normalize();
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
    const double wrapped_yaw = std::atan2(rotation(1, 0), rotation(0, 0));

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!base_measurement_freshness_.observe(message->header.stamp.toNSec(),
            ros::Time::now().toNSec(), static_cast<int64_t>(monotonicNs()), base_state_timeout_)) {
      ROS_WARN_THROTTLE(2.0, "[VelocityIk] stale/repeated/future base measurement rejected");
      return;
    }
    bool coordinate_jump = false;
    if (base_yaw_initialized_) {
      const double yaw_delta =
          shortestAngularDelta(last_base_wrapped_yaw_, wrapped_yaw);
      coordinate_jump =
          (position - last_base_position_).norm() >
              base_position_jump_threshold_ ||
          std::abs(yaw_delta) > base_yaw_jump_threshold_;
      if (coordinate_jump) {
        ++base_coordinate_epoch_;
        continuous_base_yaw_ = wrapped_yaw;
        base_coordinate_jump_detected_ = true;
      } else {
        continuous_base_yaw_ += yaw_delta;
      }
    } else {
      continuous_base_yaw_ = wrapped_yaw;
      base_yaw_initialized_ = true;
    }
    last_base_wrapped_yaw_ = wrapped_yaw;
    last_base_position_ = position;

    const double cosine = std::cos(continuous_base_yaw_);
    const double sine = std::sin(continuous_base_yaw_);
    const Eigen::Vector2d velocity_world(
        cosine * velocity_body.x() - sine * velocity_body.y(),
        sine * velocity_body.x() + cosine * velocity_body.y());
    measured_q_(baseDecisionIndex(0)) = position.x();
    measured_q_(baseDecisionIndex(1)) = position.y();
    measured_q_(baseDecisionIndex(2)) = continuous_base_yaw_;
    measured_dq_(baseDecisionIndex(0)) = velocity_world.x();
    measured_dq_(baseDecisionIndex(1)) = velocity_world.y();
    measured_dq_(baseDecisionIndex(2)) = yaw_rate;
    base_state_receive_time_ = ros::WallTime::now();
    base_state_valid_ = true;
    if (coordinate_jump) {
      ROS_ERROR_THROTTLE(
          1.0,
          "[VelocityIkShadow] base coordinate discontinuity detected; "
          "current generation remains unanchored");
    }
  }

  void modeCallback(const std_msgs::Float64MultiArray::ConstPtr& message) {
    if (message->data.size() < 3) {
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_arm_mode_ = static_cast<int>(std::lround(message->data.at(0)));
    target_arm_mode_ = static_cast<int>(std::lround(message->data.at(1)));
    arm_mode_transitioning_ = std::lround(message->data.at(2)) != 0;
    arm_mode_known_ = true;
  }

  void setPlantDecisionPositionsInContext(
      velocity_ik::ModelContext* context, Eigen::VectorXd* q_full,
      const DecisionVector& q) const {
    if (q.size() != decisionDofCount()) {
      throw std::invalid_argument(
          "decision position size does not match active DoF layout");
    }
    for (int i = 0; i < decisionDofCount(); ++i) {
      (*q_full)(decision_position_index_.at(i)) = q(i);
    }
    plant_->SetPositions(context, *q_full);
  }

  DecisionJacobian3d frameTranslationJacobian(const std::string& frame_name) const {
    return frameTranslationJacobianInContext(*plant_context_, frame_name);
  }

  DecisionJacobian3d frameAngularJacobian(const std::string& frame_name) const {
    return frameAngularJacobianInContext(*plant_context_, frame_name);
  }

  void setPlantDecisionPositions(const DecisionVector& q) {
    setPlantDecisionPositionsInContext(plant_context_.get(), &q_full_, q);
  }

  DecisionJacobian3d frameTranslationJacobianInContext(
      const velocity_ik::ModelContext& context,
      const std::string& frame_name) const {
    Eigen::MatrixXd full(3, plant_->num_velocities());
    plant_->translationJacobian(
        context,
        plant_->GetFrameByName(frame_name), Eigen::Vector3d::Zero(),
        plant_->world_frame(), plant_->world_frame(), &full);
    DecisionJacobian3d selected(3, decisionDofCount());
    for (int i = 0; i < decisionDofCount(); ++i) {
      selected.col(i) = full.col(decision_velocity_index_.at(i));
    }
    return selected;
  }

  DecisionJacobian3d frameAngularJacobianInContext(
      const velocity_ik::ModelContext& context,
      const std::string& frame_name) const {
    Eigen::MatrixXd full(6, plant_->num_velocities());
    plant_->spatialJacobian(
        context,
        plant_->GetFrameByName(frame_name), Eigen::Vector3d::Zero(),
        plant_->world_frame(), plant_->world_frame(), &full);
    DecisionJacobian3d selected(3, decisionDofCount());
    for (int i = 0; i < decisionDofCount(); ++i) {
      selected.col(i) = full.topRows<3>().col(decision_velocity_index_.at(i));
    }
    return selected;
  }

  DecisionJacobian3d frameTranslationJacobianRelativeInContext(
      const velocity_ik::ModelContext& context,
      const std::string& frame_name,
      const std::string& reference_frame_name) const {
    Eigen::MatrixXd full(6, plant_->num_velocities());
    const auto& frame = plant_->GetFrameByName(frame_name);
    const auto& reference_frame =
        plant_->GetFrameByName(reference_frame_name);
    plant_->spatialJacobian(
        context,
        frame, Eigen::Vector3d::Zero(), reference_frame, reference_frame,
        &full);
    DecisionJacobian3d selected(3, decisionDofCount());
    for (int index = 0; index < decisionDofCount(); ++index) {
      selected.col(index) =
          full.bottomRows<3>().col(decision_velocity_index_.at(index));
    }
    return selected;
  }

  DecisionJacobian3d frameAngularJacobianRelativeInContext(
      const velocity_ik::ModelContext& context,
      const std::string& frame_name,
      const std::string& reference_frame_name) const {
    Eigen::MatrixXd full(6, plant_->num_velocities());
    const auto& frame = plant_->GetFrameByName(frame_name);
    const auto& reference_frame =
        plant_->GetFrameByName(reference_frame_name);
    plant_->spatialJacobian(
        context,
        frame, Eigen::Vector3d::Zero(), reference_frame, reference_frame,
        &full);
    DecisionJacobian3d selected(3, decisionDofCount());
    for (int index = 0; index < decisionDofCount(); ++index) {
      selected.col(index) =
          full.topRows<3>().col(decision_velocity_index_.at(index));
    }
    return selected;
  }

  DecisionJacobian3d frameTranslationJacobianRelative(
      const std::string& frame_name,
      const std::string& reference_frame_name) const {
    return frameTranslationJacobianRelativeInContext(
        *plant_context_, frame_name, reference_frame_name);
  }

  DecisionJacobian3d frameAngularJacobianRelative(
      const std::string& frame_name,
      const std::string& reference_frame_name) const {
    return frameAngularJacobianRelativeInContext(
        *plant_context_, frame_name, reference_frame_name);
  }

  void addLeastSquaresCost(const DecisionJacobian3d& jacobian, const Eigen::Vector3d& command,
                           double weight, DecisionMatrix* hessian, DecisionVector* gradient) const {
    if (weight <= 0.0) {
      return;
    }
    hessian->noalias() += 2.0 * weight * jacobian.transpose() * jacobian;
    gradient->noalias() -= 2.0 * weight * jacobian.transpose() * command;
  }

  void addDynamicLeastSquaresCost(
      const DecisionMatrix& jacobian, const DecisionVector& command,
      double weight, DecisionMatrix* hessian,
      DecisionVector* gradient) const {
    if (weight <= 0.0 || jacobian.rows() == 0 ||
        jacobian.cols() != decisionDofCount() ||
        command.size() != jacobian.rows()) {
      return;
    }
    hessian->noalias() +=
        2.0 * weight * jacobian.transpose() * jacobian;
    gradient->noalias() -=
        2.0 * weight * jacobian.transpose() * command;
  }

  bool calculateBodyTaskTerms(
      const SolverRequest& request, const DecisionVector& kinematic_q,
      DecisionMatrix* task_jacobian, DecisionVector* task_velocity,
      DecisionMatrix* task_hessian, DecisionVector* task_gradient,
      double* torso_error, double* base_error, bool* root_translation_active) {
    task_jacobian->resize(0, decisionDofCount());
    task_velocity->resize(0);
    *torso_error = 0.0;
    *base_error = 0.0;
    *root_translation_active = false;
    if (!request.target.body_intent.torso_valid) {
      return true;
    }
    // The s63 URDF has three parallel +Y pitch axes, then a local +Z yaw.
    // Translation uses the actual Pinocchio Jacobian; the angular feature
    // derivatives below are exact for this verified serial arrangement.
    const Eigen::Vector4d current =
        torsoFeatureInContext(kinematic_q, plant_context_.get(), &q_full_);
    const auto& torso = plant_->GetFrameByName(torso_frame_name_);
    const auto& base = plant_->GetFrameByName(base_frame_name_);
    Eigen::MatrixXd spatial(6, plant_->num_velocities());
    plant_->spatialJacobian(
        *plant_context_,
        torso, Eigen::Vector3d::Zero(), base, base, &spatial);
    task_jacobian->setZero(3, decisionDofCount());
    for (int index : dof_layout_.decisionIndices(DofGroup::kLowerBody)) {
      (*task_jacobian)(0, index) = spatial(5, decision_velocity_index_.at(index));
    }
    for (int j = 0; j < 3; ++j) {
      (*task_jacobian)(1, lowerBodyDecisionIndex(j)) = 1.0;
    }
    (*task_jacobian)(2, lowerBodyDecisionIndex(3)) = 1.0;
    const Eigen::Vector3d error =
        (request.target.body_intent.desired_torso_feature - current).tail<3>();
    double ff_scale = 1.0;
    if (!(continuous_reference_enabled_ && request.target.estimated_twists_valid) && request.body_feedforward_age > ff_hold_time_) {
      ff_scale = std::exp(-(request.body_feedforward_age - ff_hold_time_) /
                          std::max(ff_decay_time_constant_, 1.0e-6));
    }
    Eigen::Vector3d correction=torso_tracking_gain_.cwiseProduct(error);
    if (request.catchup_correction) {
      correction(0)=std::clamp(correction(0),-acquisition_linear_speed_,acquisition_linear_speed_);
      for (int axis=1;axis<3;++axis)
        correction(axis)=std::clamp(correction(axis),-acquisition_angular_speed_,acquisition_angular_speed_);
    }
    *task_velocity = ff_scale * request.target.body_intent.desired_torso_velocity.tail<3>() + correction;
    for (int axis=0;axis<3;++axis) {
      const double normal=std::abs((*task_velocity)(axis));
      const double slow=axis==0 ? acquisition_linear_speed_ : acquisition_angular_speed_;
      const double cap=request.input_budget*((1.-request.acquisition_blend)*std::min(slow,normal)+request.acquisition_blend*normal);
      (*task_velocity)(axis)=std::clamp((*task_velocity)(axis),-cap,cap);
    }
    *torso_error = error.norm();
    DecisionMatrix scaled = *task_jacobian;
    DecisionVector command = *task_velocity;
    scaled.bottomRows(2) *= feasibility_characteristic_length_;
    command.tail(2) *= feasibility_characteristic_length_;
    addDynamicLeastSquaresCost(scaled, command, torso_intent_weight_,
                               task_hessian, task_gradient);
    return task_jacobian->array().isFinite().all() &&
           task_velocity->array().isFinite().all();
  }

  LinearConstraints calculateTorsoVelocityConstraintsInContext(
      const SolverRequest& request,
      velocity_ik::ModelContext* context,
      Eigen::VectorXd* q_full) const {
    LinearConstraints result;
    result.matrix = DecisionMatrix::Zero(kTorsoConstraintCount,
                                         decisionDofCount());
    result.lower = DecisionVector::Zero(kTorsoConstraintCount);
    result.upper = DecisionVector::Zero(kTorsoConstraintCount);
    if (request.reference_q.size() != decisionDofCount() ||
        request.reference_dq.size() != decisionDofCount() ||
        !request.reference_q.array().isFinite().all() ||
        !request.reference_dq.array().isFinite().all() ||
        !std::isfinite(request.cycle_dt) || request.cycle_dt <= 0.0) {
      return result;
    }

    setPlantDecisionPositionsInContext(context, q_full, request.reference_q);
    const auto& torso_frame = plant_->GetFrameByName(torso_frame_name_);
    const auto& base_frame = plant_->GetFrameByName(base_frame_name_);
    const auto torso_pose = torso_frame.CalcPose(*context, base_frame);
    Eigen::MatrixXd relative_spatial_jacobian(
        6, plant_->num_velocities());
    plant_->spatialJacobian(
        *context,
        torso_frame, Eigen::Vector3d::Zero(), base_frame, base_frame,
        &relative_spatial_jacobian);
    DecisionJacobian3d translation_jacobian(
        3, decisionDofCount());
    for (int index = 0; index < decisionDofCount(); ++index) {
      translation_jacobian.col(index) =
          relative_spatial_jacobian.bottomRows<3>().col(
              decision_velocity_index_.at(index));
    }
    result.matrix.row(0) = translation_jacobian.row(0);
    result.matrix.row(1) = translation_jacobian.row(2);
    result.matrix(2, lowerBodyDecisionIndex(0)) = 1.0;
    result.matrix(2, lowerBodyDecisionIndex(1)) = 1.0;
    result.matrix(2, lowerBodyDecisionIndex(2)) = 1.0;
    result.matrix(3, lowerBodyDecisionIndex(3)) = 1.0;

    result.value.at(0) = torso_pose.translation().x();
    result.value.at(1) = torso_pose.translation().z();
    result.value.at(2) =
        request.reference_q(lowerBodyDecisionIndex(0)) +
        request.reference_q(lowerBodyDecisionIndex(1)) +
        request.reference_q(lowerBodyDecisionIndex(2));
    result.value.at(3) =
        request.reference_q(lowerBodyDecisionIndex(3));
    result.position_lower = torso_workspace_lower_;
    result.position_upper = torso_workspace_upper_;

    const DecisionVector current_rate =
        result.matrix * request.reference_dq;
    for (int constraint = 0; constraint < kTorsoConstraintCount;
         ++constraint) {
      // If a session starts outside the preferred coupled workspace, retain
      // the current boundary so that inward recovery remains feasible while
      // any further outward motion is forbidden.
      if (result.value.at(constraint) <
          result.position_lower.at(constraint)) {
        result.position_lower.at(constraint) = result.value.at(constraint);
      }
      if (result.value.at(constraint) >
          result.position_upper.at(constraint)) {
        result.position_upper.at(constraint) = result.value.at(constraint);
      }
      result.lower(constraint) =
          2.0 * (result.position_lower.at(constraint) -
                 result.value.at(constraint)) /
              request.cycle_dt -
          current_rate(constraint);
      result.upper(constraint) =
          2.0 * (result.position_upper.at(constraint) -
                 result.value.at(constraint)) /
              request.cycle_dt -
          current_rate(constraint);
      if (!std::isfinite(result.lower(constraint)) ||
          !std::isfinite(result.upper(constraint)) ||
          result.lower(constraint) > result.upper(constraint) + 1.0e-10) {
        return result;
      }
    }
    result.valid = result.matrix.array().isFinite().all();
    return result;
  }

  LinearConstraints makeConstraintsRecursivelyFeasible(
      const LinearConstraints& preferred,
      const DecisionVector& dynamic_lower,
      const DecisionVector& dynamic_upper,
      double* maximum_relaxation) const {
    LinearConstraints result = preferred;
    *maximum_relaxation = 0.0;
    if (!result.valid ||
        result.matrix.cols() != decisionDofCount() ||
        dynamic_lower.size() != decisionDofCount() ||
        dynamic_upper.size() != decisionDofCount() ||
        (dynamic_lower.array() > dynamic_upper.array()).any()) {
      result.valid = false;
      return result;
    }
    // Torso x/z/pitch/yaw are preferred operating envelopes.  The closest
    // dynamically reachable braking point must always remain feasible; URDF
    // joint limits and stopping-distance bounds stay hard in dynamic_lower/
    // dynamic_upper.  Expanding only around this point turns a contradictory
    // comfort envelope into a recoverable soft boundary instead of a QP
    // outage.
    const DecisionVector braking_baseline =
        DecisionVector::Zero(decisionDofCount())
            .cwiseMax(dynamic_lower)
            .cwiseMin(dynamic_upper);
    const DecisionVector baseline_rate =
        result.matrix * braking_baseline;
    for (Eigen::Index row = 0; row < baseline_rate.size(); ++row) {
      const double relaxation = std::max(
          result.lower(row) - baseline_rate(row),
          baseline_rate(row) - result.upper(row));
      *maximum_relaxation = std::max(*maximum_relaxation, relaxation);
      result.lower(row) = std::min(result.lower(row), baseline_rate(row));
      result.upper(row) = std::max(result.upper(row), baseline_rate(row));
    }
    result.valid = result.matrix.array().isFinite().all() &&
        result.lower.array().isFinite().all() &&
        result.upper.array().isFinite().all() &&
        (result.lower.array() <= result.upper.array()).all();
    return result;
  }

  bool armIsControlled(int arm) const {
    return control_arm_index_ == 2 || control_arm_index_ == arm;
  }

  static geometry_msgs::Point toPoint(const Eigen::Vector3d& value) {
    geometry_msgs::Point point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
  }

  static geometry_msgs::Quaternion toQuaternion(
      const Eigen::Quaterniond& input) {
    Eigen::Quaterniond value = input;
    if (value.norm() > 1.0e-12) {
      value.normalize();
    } else {
      value = Eigen::Quaterniond::Identity();
    }
    geometry_msgs::Quaternion quaternion;
    quaternion.x = value.x();
    quaternion.y = value.y();
    quaternion.z = value.z();
    quaternion.w = value.w();
    return quaternion;
  }

  static geometry_msgs::Pose toPose(const Eigen::Vector3d& position,
                                    const Eigen::Quaterniond& orientation) {
    geometry_msgs::Pose pose;
    pose.position = toPoint(position);
    pose.orientation = toQuaternion(orientation);
    return pose;
  }

  static ArmTarget transformArmTarget(const Eigen::Isometry3d& transform,
                                      const ArmTarget& source) {
    ArmTarget result = source;
    result.hand_position = transform * source.hand_position;
    result.hand_orientation = Eigen::Quaterniond(transform.linear()) *
        source.hand_orientation;
    result.hand_orientation.normalize();
    result.elbow_position = transform * source.elbow_position;
    result.hand_linear_ff = transform.linear() * source.hand_linear_ff;
    result.hand_angular_ff = transform.linear() * source.hand_angular_ff;
    result.elbow_linear_ff = transform.linear() * source.elbow_linear_ff;
    return result;
  }

  Eigen::Isometry3d torsoPoseInWorldInContext(
      const DecisionVector& q, velocity_ik::ModelContext* context,
      Eigen::VectorXd* q_full) const {
    setPlantDecisionPositionsInContext(context, q_full, q);
    const auto pose = plant_->GetFrameByName(torso_frame_name_)
        .CalcPoseInWorld(*context);
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = pose.rotation().matrix();
    result.translation() = pose.translation();
    return result;
  }

  geometry_msgs::Pose planarBasePose(const DecisionVector& q) const {
    const Eigen::Vector3d position(
        q(baseDecisionIndex(0)), q(baseDecisionIndex(1)), 0.0);
    const Eigen::Quaterniond orientation(
        Eigen::AngleAxisd(q(baseDecisionIndex(2)),
                          Eigen::Vector3d::UnitZ()));
    return toPose(position, orientation);
  }

  geometry_msgs::Pose sessionAnchorPose() const {
    const Eigen::Vector3d position(
        session_anchor_.translation.x(),
        session_anchor_.translation.y(), 0.0);
    const Eigen::Quaterniond orientation(
        Eigen::AngleAxisd(session_anchor_.yaw,
                          Eigen::Vector3d::UnitZ()));
    return toPose(position, orientation);
  }

  Eigen::Vector4d torsoFeatureInContext(
      const DecisionVector& q, velocity_ik::ModelContext* context,
      Eigen::VectorXd* q_full) const {
    setPlantDecisionPositionsInContext(context, q_full, q);
    const auto torso_pose = plant_->GetFrameByName(torso_frame_name_)
        .CalcPose(*context, plant_->GetFrameByName(base_frame_name_));
    Eigen::Vector4d feature;
    feature << torso_pose.translation().x(), torso_pose.translation().z(),
        q(lowerBodyDecisionIndex(0)) + q(lowerBodyDecisionIndex(1)) +
            q(lowerBodyDecisionIndex(2)),
        q(lowerBodyDecisionIndex(3));
    return feature;
  }

  bool captureSessionBodyAnchor(const TargetSnapshot& target,
                                const DecisionVector& measured_q,
                                bool preserve_resolved_intent) {
    if (!enable_body_mirroring_ || !target.body.torsoValid() ||
        target.body.frame_id != body_tracking_frame_name_ ||
        measured_q.size() != decisionDofCount() ||
        !measured_q.array().isFinite().all() || !session_anchor_.valid) {
      session_body_anchor_.valid = false;
      return false;
    }
    const SessionBodyAnchor previous = session_body_anchor_;
    SessionBodyAnchor next;
    next.human = target.body;
    next.session_id = target.session_id;
    next.generation = target.generation;
    next.coordinate_epoch = target.body.coordinate_epoch;
    next.robot_torso_feature =
        preserve_resolved_intent && previous.valid
            ? previous.robot_torso_feature
            : torsoFeatureInContext(
                  measured_q, capture_context_.get(), &capture_q_full_);
    if (preserve_resolved_intent && previous.valid &&
        last_body_intent_.torso_valid) {
      next.accumulated_root_translation =
          last_body_intent_.root_translation;
      next.accumulated_torso_delta =
          last_body_intent_.desired_torso_feature -
          next.robot_torso_feature;
      next.robot_torso_pivot_session =
          previous.robot_torso_pivot_session;
      next.robot_torso_pose_session =
          previous.robot_torso_pose_session;
      next.neutral_hand_position = previous.neutral_hand_position;
    } else {
      next.accumulated_root_translation.setZero();
      next.accumulated_torso_delta.setZero();
      setPlantDecisionPositionsInContext(
          capture_context_.get(), &capture_q_full_, measured_q);
      const Eigen::Vector3d torso_world =
          plant_->GetFrameByName(torso_frame_name_)
              .CalcPoseInWorld(*capture_context_)
              .translation();
      const auto torso_pose_world =
          plant_->GetFrameByName(torso_frame_name_)
              .CalcPoseInWorld(*capture_context_);
      const Eigen::Matrix3d world_from_session =
          Eigen::AngleAxisd(session_anchor_.yaw,
                            Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      const Eigen::Vector3d session_origin_world(
          session_anchor_.translation.x(),
          session_anchor_.translation.y(), 0.0);
      next.robot_torso_pivot_session =
          world_from_session.transpose() *
          (torso_world - session_origin_world);
      next.robot_torso_pose_session = Eigen::Isometry3d::Identity();
      next.robot_torso_pose_session.linear() =
          world_from_session.transpose() *
          torso_pose_world.rotation().matrix();
      next.robot_torso_pose_session.translation() =
          next.robot_torso_pivot_session;
      for (int arm = 0; arm < kArmCount; ++arm) {
        next.neutral_hand_position.at(arm) =
            target.arm.at(arm).hand_position;
      }
    }
    next.valid = true;
    session_body_anchor_ = next;
    return true;
  }

  BodyIntent calculateBodyIntent(const TargetSnapshot& target) const {
    BodyIntent intent;
    if (!enable_body_mirroring_ || !session_body_anchor_.valid ||
        !target.body.torsoValid() ||
        target.session_id != session_body_anchor_.session_id ||
        target.generation != session_body_anchor_.generation ||
        target.body.coordinate_epoch != session_body_anchor_.coordinate_epoch) {
      return intent;
    }
    const auto& anchor = session_body_anchor_.human;
    // Anatomical chest axes, identical to Quest's fixed chest calibration.
    // Never derive spatial axes from either wrist or from Root orientation.
    const Eigen::Matrix3d chest_calibration =
        rotationFromRpy(Eigen::Vector3d(M_PI / 2.0, 0.0, M_PI / 2.0));
    const Eigen::Matrix3d rotation =
        target.body.chest_orientation.toRotationMatrix() * chest_calibration.transpose();
    const Eigen::Matrix3d anchor_rotation =
        anchor.chest_orientation.toRotationMatrix() * chest_calibration.transpose();
    const auto pitch = [](const Eigen::Matrix3d& r) {
      return std::atan2(-r(2, 0), std::hypot(r(0, 0), r(1, 0)));
    };
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const double anchor_yaw =
        std::atan2(anchor_rotation(1, 0), anchor_rotation(0, 0));
    Eigen::Vector4d delta = session_body_anchor_.accumulated_torso_delta;
    // No horizontal locomotion or hand-error recruitment in fixed-base IK.
    delta(1) += torso_height_gain_ *
        (target.body.chest_position.z() - anchor.chest_position.z());
    delta(2) += torso_pitch_gain_ * (pitch(rotation) - pitch(anchor_rotation));
    delta(3) += torso_yaw_gain_ * shortestAngularDelta(anchor_yaw, yaw);
    intent.desired_torso_feature = session_body_anchor_.robot_torso_feature + delta;
    const auto& omega = target.body.chest_angular_ff;
    const double pitch_rate = -std::sin(yaw) * omega.x() + std::cos(yaw) * omega.y();
    const double yaw_rate = omega.z() +
        (std::cos(yaw) * omega.x() + std::sin(yaw) * omega.y()) *
        std::tan(std::clamp(pitch(rotation), -1.45, 1.45));
    intent.desired_torso_velocity <<
        0.0, torso_height_gain_ * target.body.chest_linear_ff.z(),
        torso_pitch_gain_ * pitch_rate, torso_yaw_gain_ * yaw_rate;
    intent.desired_base_translation = session_anchor_.translation;
    intent.desired_base_velocity.setZero();
    intent.torso_valid = true;
    // Legacy recruitment/Root fields stay zero and are diagnostic only.
    return intent;
  }

  TargetSnapshot resolveSessionTargetInWorld(
      const TargetSnapshot& session_target,
      uint64_t base_coordinate_epoch,
      const BodyIntent& body_intent) const {
    TargetSnapshot result = session_target;
    if (!session_anchor_.valid ||
        session_anchor_.session_id != session_target.session_id ||
        session_anchor_.generation != session_target.generation ||
        session_anchor_.base_coordinate_epoch != base_coordinate_epoch) {
      result.valid = false;
      result.frame_id = world_frame_name_;
      result.arm = {};
      return result;
    }
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(session_anchor_.yaw,
                          Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    const Eigen::Vector3d translation(
        session_anchor_.translation.x(),
        session_anchor_.translation.y(), 0.0);
    const Eigen::Quaterniond anchor_orientation(rotation);
    // Diagnostic conversion only. Normal hand control remains torso-relative;
    // applying desired body motion here made the arms absorb torso/base lag.
    for (int arm = 0; arm < kArmCount; ++arm) {
      auto& target = result.arm.at(arm);
      target.hand_position =
          rotation * target.hand_position + translation;
      target.hand_orientation =
          anchor_orientation * target.hand_orientation;
      target.hand_orientation.normalize();
      target.elbow_position =
          rotation * target.elbow_position + translation;
      target.hand_linear_ff = rotation * target.hand_linear_ff;
      target.hand_angular_ff = rotation * target.hand_angular_ff;
      target.elbow_linear_ff = rotation * target.elbow_linear_ff;
    }
    result.body_intent = body_intent;
    result.body_intent.desired_base_translation =
        body_intent.desired_base_translation;
    result.body_mirror_rotation_world = Eigen::Matrix3d::Identity();
    result.frame_id = world_frame_name_;
    result.arm_target_torso_relative = false;
    return result;
  }

  TargetSnapshot applySessionTargetAlignment(
      const TargetSnapshot& session_target, const BodyIntent& body_intent) const {
    TargetSnapshot result = session_target;
    if (!session_target.valid || !session_target_alignment_.valid ||
        session_target_alignment_.session_id != session_target.session_id ||
        session_target_alignment_.generation != session_target.generation) {
      result.valid = false;
      return result;
    }
    for (int arm = 0; arm < kArmCount; ++arm) {
      result.arm.at(arm) = transformArmTarget(
          session_target_alignment_.torso_from_source.at(arm),
          session_target.arm.at(arm));
    }
    result.body_intent = body_intent;
    result.frame_id = torso_frame_name_;
    result.arm_target_torso_relative = true;
    return result;
  }

  TargetSnapshot torsoRelativeTargetInWorld(
      const TargetSnapshot& torso_target,
      const DecisionVector& q) {
    TargetSnapshot result = torso_target;
    if (!torso_target.valid || !torso_target.arm_target_torso_relative ||
        q.size() != decisionDofCount() || !q.array().isFinite().all()) {
      result.valid = false;
      return result;
    }
    const Eigen::Isometry3d world_from_torso = torsoPoseInWorldInContext(
        q, capture_context_.get(), &capture_q_full_);
    for (int arm = 0; arm < kArmCount; ++arm) {
      result.arm.at(arm) = transformArmTarget(
          world_from_torso, torso_target.arm.at(arm));
    }
    result.frame_id = world_frame_name_;
    result.arm_target_torso_relative = false;
    return result;
  }

  bool captureSessionTargetAlignment(
      const TargetSnapshot& session_target, const DecisionVector& measured_q,
      uint64_t base_coordinate_epoch) {
    session_target_alignment_.valid = false;
    if (!session_target.valid || !session_anchor_.valid ||
        session_anchor_.base_coordinate_epoch != base_coordinate_epoch ||
        measured_q.size() != decisionDofCount() ||
        !measured_q.array().isFinite().all()) {
      return false;
    }
    // Python has already removed the complete chest motion and applied the
    // common Quest -> robot axes and fixed Palm -> TCP tool calibration.
    // Only translate its configured shoulder anchors into the URDF torso.
    // This depends on robot geometry, NEVER on the hand pose at enable.
    double shoulder_x = 0.0, shoulder_z = 0.42, shoulder_y = 0.15;
    nh_.param("/quest3/base_chest_offset_x", shoulder_x, shoulder_x);
    nh_.param("/quest3/base_height_offset", shoulder_z, shoulder_z);
    nh_.param("/quest3/shoulder_width", shoulder_y, shoulder_y);
    setPlantDecisionPositionsInContext(capture_context_.get(), &capture_q_full_, measured_q);
    const auto& torso = plant_->GetFrameByName(torso_frame_name_);
    for (int arm = 0; arm < kArmCount; ++arm) {
      auto& alignment = session_target_alignment_.torso_from_source.at(arm);
      alignment = Eigen::Isometry3d::Identity();
      alignment.translation() =
          plant_->GetFrameByName(shoulder_frame_names_.at(arm))
              .CalcPose(*capture_context_, torso).translation() -
          Eigen::Vector3d(shoulder_x, arm == 0 ? shoulder_y : -shoulder_y, shoulder_z);
    }
    session_target_alignment_.session_id = session_target.session_id;
    session_target_alignment_.generation = session_target.generation;
    session_target_alignment_.valid = true;
    return true;
  }

  static double stableSigmoid(double value) {
    if (value >= 0.0) {
      const double decay = std::exp(-value);
      return 1.0 / (1.0 + decay);
    }
    const double growth = std::exp(value);
    return growth / (1.0 + growth);
  }

  WorkspaceProjection calculateWorkspaceProjection(
      const Eigen::Vector3d& raw_position,
      const Eigen::Vector3d& shoulder_position, double reach_reserve) const {
    WorkspaceProjection result;
    result.position = raw_position;
    const Eigen::Vector3d shoulder_to_target =
        raw_position - shoulder_position;
    result.raw_distance = shoulder_to_target.norm();
    result.feasible_distance = result.raw_distance;
    if (!std::isfinite(result.raw_distance) || result.raw_distance < 1.0e-9) {
      return result;
    }

    const double safe_radius =
        std::max(0.01, nominal_arm_reach_ - std::max(0.0, reach_reserve));
    const double smoothing = workspace_projection_smoothing_;
    // Smooth minimum of raw_distance and safe_radius.  It is spatial and
    // memoryless, so it adds no streaming/interpolation delay.  Its analytic
    // derivative also projects velocity feedforward consistently.
    const double separation = std::abs(result.raw_distance - safe_radius);
    const double smooth_minimum =
        std::min(result.raw_distance, safe_radius) -
        smoothing * std::log1p(std::exp(-separation / smoothing));
    result.feasible_distance =
        std::clamp(smooth_minimum, 0.0, result.raw_distance);
    result.projection_distance =
        result.raw_distance - result.feasible_distance;

    const Eigen::Vector3d radial_direction =
        shoulder_to_target / result.raw_distance;
    result.position = shoulder_position +
                      result.feasible_distance * radial_direction;
    const double radial_derivative = stableSigmoid(
        (safe_radius - result.raw_distance) / smoothing);
    const Eigen::Matrix3d radial_projector =
        radial_direction * radial_direction.transpose();
    result.velocity_map =
        radial_derivative * radial_projector +
        (result.feasible_distance / result.raw_distance) *
            (Eigen::Matrix3d::Identity() - radial_projector);
    return result;
  }

  double jointSafeMarginRatio(const DecisionVector& q, int index) const {
    const double lower_span = std::max(
        joint_safe_center_(index) - joint_safe_lower_(index), 1.0e-9);
    const double upper_span = std::max(
        joint_safe_upper_(index) - joint_safe_center_(index), 1.0e-9);
    const double lower_ratio =
        (q(index) - joint_safe_lower_(index)) / lower_span;
    const double upper_ratio =
        (joint_safe_upper_(index) - q(index)) / upper_span;
    return std::min(lower_ratio, upper_ratio);
  }

  static double smoothStep01(double value) {
    value = std::clamp(value, 0.0, 1.0);
    return value * value * (3.0 - 2.0 * value);
  }

  double lowerOutwardVelocityScale(double q, int index) const {
    if (q >= joint_safe_lower_(index)) {
      return 1.0;
    }
    const double hard_lower =
        position_lower_(index) + position_limit_margin_ +
        boundary_brake_guard_;
    const double soft_width =
        std::max(joint_safe_lower_(index) - hard_lower, 1.0e-6);
    return smoothStep01((q - hard_lower) / soft_width);
  }

  double upperOutwardVelocityScale(double q, int index) const {
    if (q <= joint_safe_upper_(index)) {
      return 1.0;
    }
    const double hard_upper =
        position_upper_(index) - position_limit_margin_ -
        boundary_brake_guard_;
    const double soft_width =
        std::max(hard_upper - joint_safe_upper_(index), 1.0e-6);
    return smoothStep01((hard_upper - q) / soft_width);
  }

  double jointOutwardVelocityScale(double q, int index) const {
    return std::min(lowerOutwardVelocityScale(q, index),
                    upperOutwardVelocityScale(q, index));
  }

  double jointLimitBarrierDirection(const DecisionVector& q, int index) const {
    const double lower_span = std::max(
        joint_safe_center_(index) - joint_safe_lower_(index), 1.0e-9);
    const double upper_span = std::max(
        joint_safe_upper_(index) - joint_safe_center_(index), 1.0e-9);
    const double lower_ratio =
        (q(index) - joint_safe_lower_(index)) / lower_span;
    const double upper_ratio =
        (joint_safe_upper_(index) - q(index)) / upper_span;
    const double activation_ratio =
        std::max(feasibility_joint_limit_approach_ratio_, 1.0e-9);
    double direction = 0.0;
    if (lower_ratio < feasibility_joint_limit_approach_ratio_) {
      const double activation = std::clamp(
          (feasibility_joint_limit_approach_ratio_ - lower_ratio) /
              activation_ratio,
          0.0, 1.0);
      direction += activation * activation;
    }
    if (upper_ratio < feasibility_joint_limit_approach_ratio_) {
      const double activation = std::clamp(
          (feasibility_joint_limit_approach_ratio_ - upper_ratio) /
              activation_ratio,
          0.0, 1.0);
      direction -= activation * activation;
    }
    return direction;
  }

  double minimumJointMarginRatio(const DecisionVector& q, int arm) const {
    double minimum_ratio = std::numeric_limits<double>::infinity();
    for (int joint = 0; joint < kJointsPerArm; ++joint) {
      const int index = armDecisionIndex(arm, joint);
      const double range =
          std::max(position_upper_(index) - position_lower_(index), 1.0e-9);
      minimum_ratio = std::min(
          minimum_ratio,
          std::min(q(index) - position_lower_(index),
                   position_upper_(index) - q(index)) /
              range);
    }
    return minimum_ratio;
  }

  double scaledSigmaForArm(const DecisionVector& q, int arm) {
    setPlantDecisionPositionsInContext(feasibility_context_.get(),
                                  &feasibility_q_full_, q);
    const Matrix3x7d translation_jacobian =
        selectArmColumns(
            frameTranslationJacobianInContext(
                *feasibility_context_, hand_frame_names_.at(arm)),
            arm);
    const Matrix3x7d angular_jacobian =
        selectArmColumns(
            frameAngularJacobianInContext(
                *feasibility_context_, hand_frame_names_.at(arm)),
            arm);
    Matrix6x7d scaled_jacobian;
    scaled_jacobian.topRows<3>() = translation_jacobian;
    scaled_jacobian.bottomRows<3>() =
        feasibility_characteristic_length_ * angular_jacobian;
    const Eigen::JacobiSVD<Matrix6x7d> svd(scaled_jacobian);
    return svd.singularValues().minCoeff();
  }

  wheel_arm_abs_vel::VelocityIkFeasibilityDebug calculateFeasibilityDebug(
      const FeasibilityRequest& request) {
    wheel_arm_abs_vel::VelocityIkFeasibilityDebug message;
    populateDecisionLayout(&message);
    resizeFeasibilityDecisionFields(&message);
    message.header.seq = feasibility_debug_sequence_++;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = request.target.arm_target_torso_relative
        ? torso_frame_name_ : world_frame_name_;
    message.source_stamp = request.target.source_stamp;
    message.target_seq = request.target.sequence;
    message.session_id = request.target.session_id;
    message.generation = request.target.generation;
    message.reference_seq = request.reference_seq;
    message.control_state = request.target.control_state;
    message.mode = static_cast<uint8_t>(feasibility_mode_);
    message.tracking_active = request.tracking_active;
    // The shoulder sphere is retained as a reach diagnostic only.  Projecting
    // before the 21-D solve would erase exactly the error that should recruit
    // the lower body and, at higher pressure, the planar base.
    message.workspace_projection_applied = false;
    message.joint_limit_barrier_applied =
        feasibility_mode_ == FeasibilityMode::kActive;
    message.operational_joint_limit_applied =
        feasibility_mode_ == FeasibilityMode::kActive;
    message.sample_age =
        std::max(0.0, (ros::WallTime::now() - request.enqueue_time).toSec());
    message.characteristic_length = feasibility_characteristic_length_;
    message.selected_reach_reserve = selected_reach_reserve_;
    for (std::size_t reserve = 0;
         reserve < feasibility_reach_reserve_candidates_.size(); ++reserve) {
      message.reach_reserve_candidates.at(reserve) =
          feasibility_reach_reserve_candidates_.at(reserve);
    }

    bool valid = request.target.valid &&
                 request.reference_q.array().isFinite().all();
    for (int index = 0; index < decisionDofCount(); ++index) {
      const double range = std::max(
          position_upper_(index) - position_lower_(index), 1.0e-9);
      const double lower_margin =
          request.reference_q(index) - position_lower_(index);
      const double upper_margin =
          position_upper_(index) - request.reference_q(index);
      message.joint_lower_margin.at(index) = lower_margin;
      message.joint_upper_margin.at(index) = upper_margin;
      message.joint_margin_ratio.at(index) =
          std::min(lower_margin, upper_margin) / range;
      message.joint_safe_center.at(index) = joint_safe_center_(index);
      message.joint_safe_lower.at(index) = joint_safe_lower_(index);
      message.joint_safe_upper.at(index) = joint_safe_upper_(index);
      message.joint_safe_range_ratio.at(index) =
          joint_safe_range_ratio_(index);
      message.joint_safe_margin_ratio.at(index) =
          jointSafeMarginRatio(request.reference_q, index);
      message.joint_outward_velocity_scale.at(index) =
          jointOutwardVelocityScale(request.reference_q(index), index);
      message.limit_barrier_direction.at(index) =
          jointLimitBarrierDirection(request.reference_q, index);
    }
    for (int arm = 0; arm < kArmCount; ++arm) {
      message.arm_controlled.at(arm) = armIsControlled(arm);
      message.nominal_arm_reach.at(arm) = nominal_arm_reach_;

      setPlantDecisionPositionsInContext(feasibility_context_.get(),
                                    &feasibility_q_full_, request.reference_q);
      const auto& hand_frame =
          plant_->GetFrameByName(hand_frame_names_.at(arm));
      const auto& shoulder_frame =
          plant_->GetFrameByName(shoulder_frame_names_.at(arm));
      const auto& torso_frame = plant_->GetFrameByName(torso_frame_name_);
      const auto hand_pose = hand_frame.CalcPose(
          *feasibility_context_, torso_frame);
      const Eigen::Vector3d shoulder_position =
          shoulder_frame.CalcPose(
              *feasibility_context_, torso_frame).translation();
      const Eigen::Vector3d raw_position =
          request.target.arm.at(arm).hand_position;
      const Eigen::Quaterniond raw_orientation =
          request.target.arm.at(arm).hand_orientation;
      const WorkspaceProjection projection = calculateWorkspaceProjection(
          raw_position, shoulder_position, selected_reach_reserve_);

      message.raw_tcp_pose.at(arm) =
          toPose(raw_position, raw_orientation);
      message.feasible_tcp_pose.at(arm) =
          toPose(projection.position, raw_orientation);
      message.reference_tcp_pose.at(arm) = toPose(
          hand_pose.translation(),
          Eigen::Quaterniond(hand_pose.rotation().matrix()));
      message.shoulder_position.at(arm) = toPoint(shoulder_position);
      message.shoulder_to_raw_target.at(arm) = projection.raw_distance;
      message.shoulder_to_feasible_target.at(arm) =
          projection.feasible_distance;
      message.shoulder_to_reference_tcp.at(arm) =
          (hand_pose.translation() - shoulder_position).norm();
      message.raw_reach_reserve.at(arm) =
          nominal_arm_reach_ - projection.raw_distance;
      message.selected_projection_distance.at(arm) =
          projection.projection_distance;
      message.projection_active.at(arm) =
          projection.projection_distance > 1.0e-5;
      for (std::size_t reserve = 0;
           reserve < feasibility_reach_reserve_candidates_.size(); ++reserve) {
        const WorkspaceProjection candidate_projection =
            calculateWorkspaceProjection(
                raw_position, shoulder_position,
                feasibility_reach_reserve_candidates_.at(reserve));
        message.projection_distance_by_reserve.at(
            arm * feasibility_reach_reserve_candidates_.size() + reserve) =
            candidate_projection.projection_distance;
      }

      const Matrix3x7d translation_jacobian =
          selectArmColumns(
              frameTranslationJacobianRelativeInContext(
                  *feasibility_context_, hand_frame_names_.at(arm),
                  torso_frame_name_),
              arm);
      const Matrix3x7d angular_jacobian =
          selectArmColumns(
              frameAngularJacobianRelativeInContext(
                  *feasibility_context_, hand_frame_names_.at(arm),
                  torso_frame_name_),
              arm);
      Matrix6x7d unscaled_jacobian;
      unscaled_jacobian.topRows<3>() = translation_jacobian;
      unscaled_jacobian.bottomRows<3>() = angular_jacobian;
      Matrix6x7d scaled_jacobian;
      scaled_jacobian.topRows<3>() = translation_jacobian;
      scaled_jacobian.bottomRows<3>() =
          feasibility_characteristic_length_ * angular_jacobian;

      const Eigen::JacobiSVD<Matrix3x7d> translation_svd(
          translation_jacobian);
      const Eigen::JacobiSVD<Matrix6x7d> unscaled_svd(
          unscaled_jacobian);
      const Eigen::JacobiSVD<Matrix6x7d> scaled_svd(
          scaled_jacobian, Eigen::ComputeFullV);
      message.sigma_translation.at(arm) =
          translation_svd.singularValues().minCoeff();
      message.sigma_unscaled_6d.at(arm) =
          unscaled_svd.singularValues().minCoeff();
      message.sigma_scaled_6d.at(arm) =
          scaled_svd.singularValues().minCoeff();
      if (message.sigma_scaled_6d.at(arm) <=
          feasibility_singularity_escape_threshold_) {
        message.singularity_state.at(arm) =
            wheel_arm_abs_vel::VelocityIkFeasibilityDebug::STATE_ESCAPE;
      } else if (message.sigma_scaled_6d.at(arm) <=
                 feasibility_singularity_approach_threshold_) {
        message.singularity_state.at(arm) =
            wheel_arm_abs_vel::VelocityIkFeasibilityDebug::STATE_APPROACH;
      } else {
        message.singularity_state.at(arm) =
            wheel_arm_abs_vel::VelocityIkFeasibilityDebug::STATE_NORMAL;
      }

      double minimum_margin_ratio = std::numeric_limits<double>::infinity();
      double minimum_safe_margin_ratio =
          std::numeric_limits<double>::infinity();
      double minimum_outward_velocity_scale = 1.0;
      for (int joint = 0; joint < kJointsPerArm; ++joint) {
        const int index = armDecisionIndex(arm, joint);
        const double range = std::max(
            position_upper_(index) - position_lower_(index), 1.0e-9);
        const double lower_margin =
            request.reference_q(index) - position_lower_(index);
        const double upper_margin =
            position_upper_(index) - request.reference_q(index);
        const double lower_ratio = lower_margin / range;
        const double upper_ratio = upper_margin / range;
        const double margin_ratio = std::min(lower_ratio, upper_ratio);
        message.joint_lower_margin.at(index) = lower_margin;
        message.joint_upper_margin.at(index) = upper_margin;
        message.joint_margin_ratio.at(index) = margin_ratio;
        minimum_margin_ratio = std::min(minimum_margin_ratio, margin_ratio);

        const double safe_margin_ratio =
            jointSafeMarginRatio(request.reference_q, index);
        message.joint_safe_center.at(index) = joint_safe_center_(index);
        message.joint_safe_lower.at(index) = joint_safe_lower_(index);
        message.joint_safe_upper.at(index) = joint_safe_upper_(index);
        message.joint_safe_range_ratio.at(index) =
            joint_safe_range_ratio_(index);
        message.joint_safe_margin_ratio.at(index) = safe_margin_ratio;
        const double outward_velocity_scale =
            jointOutwardVelocityScale(request.reference_q(index), index);
        message.joint_outward_velocity_scale.at(index) =
            outward_velocity_scale;
        minimum_outward_velocity_scale = std::min(
            minimum_outward_velocity_scale, outward_velocity_scale);
        minimum_safe_margin_ratio = std::min(
            minimum_safe_margin_ratio, safe_margin_ratio);

        message.limit_barrier_direction.at(index) =
            jointLimitBarrierDirection(request.reference_q, index);
      }
      message.minimum_joint_margin_ratio.at(arm) = minimum_margin_ratio;
      if (minimum_margin_ratio <= feasibility_joint_limit_escape_ratio_) {
        message.joint_limit_state.at(arm) =
            wheel_arm_abs_vel::VelocityIkFeasibilityDebug::STATE_ESCAPE;
      } else if (minimum_margin_ratio <=
                 feasibility_joint_limit_approach_ratio_) {
        message.joint_limit_state.at(arm) =
            wheel_arm_abs_vel::VelocityIkFeasibilityDebug::STATE_APPROACH;
      } else {
        message.joint_limit_state.at(arm) =
            wheel_arm_abs_vel::VelocityIkFeasibilityDebug::STATE_NORMAL;
      }
      message.minimum_safe_margin_ratio.at(arm) =
          minimum_safe_margin_ratio;
      if (minimum_outward_velocity_scale <= 0.05) {
        message.operational_limit_state.at(arm) =
            wheel_arm_abs_vel::VelocityIkFeasibilityDebug::STATE_ESCAPE;
      } else if (minimum_outward_velocity_scale < 1.0 - 1.0e-10) {
        message.operational_limit_state.at(arm) =
            wheel_arm_abs_vel::VelocityIkFeasibilityDebug::STATE_APPROACH;
      } else {
        message.operational_limit_state.at(arm) =
            wheel_arm_abs_vel::VelocityIkFeasibilityDebug::STATE_NORMAL;
      }

      const Eigen::VectorXd full_v = scaled_svd.matrixV().col(5);
      Eigen::Matrix<double, kJointsPerArm, 1> minimum_direction = full_v;
      if (!minimum_direction.array().isFinite().all() ||
          minimum_direction.norm() < 1.0e-12) {
        minimum_direction.setZero();
      } else {
        minimum_direction.normalize();
      }
      DecisionVector q_plus = request.reference_q;
      DecisionVector q_minus = request.reference_q;
      addArmValues(
          arm, feasibility_escape_lookahead_ * minimum_direction, &q_plus);
      addArmValues(
          arm, -feasibility_escape_lookahead_ * minimum_direction, &q_minus);
      q_plus = q_plus.cwiseMax(position_lower_).cwiseMin(position_upper_);
      q_minus = q_minus.cwiseMax(position_lower_).cwiseMin(position_upper_);
      const double score_plus =
          scaledSigmaForArm(q_plus, arm) +
          feasibility_escape_joint_margin_weight_ *
              minimumJointMarginRatio(q_plus, arm);
      const double score_minus =
          scaledSigmaForArm(q_minus, arm) +
          feasibility_escape_joint_margin_weight_ *
              minimumJointMarginRatio(q_minus, arm);
      message.escape_score_plus.at(arm) = score_plus;
      message.escape_score_minus.at(arm) = score_minus;
      const double direction_sign = score_plus >= score_minus ? 1.0 : -1.0;
      for (int joint = 0; joint < kJointsPerArm; ++joint) {
        message.singular_escape_direction.at(
            armDecisionIndex(arm, joint)) =
            direction_sign * minimum_direction(joint);
      }

      valid = valid && allFinite(raw_position) &&
              std::isfinite(projection.raw_distance) &&
              std::isfinite(message.sigma_translation.at(arm)) &&
              std::isfinite(message.sigma_scaled_6d.at(arm)) &&
              std::isfinite(message.sigma_unscaled_6d.at(arm)) &&
              std::isfinite(minimum_margin_ratio) &&
              std::isfinite(score_plus) && std::isfinite(score_minus);
    }
    SolverRequest torso_request;
    torso_request.reference_q = request.reference_q;
    torso_request.reference_dq =
        DecisionVector::Zero(decisionDofCount());
    torso_request.cycle_dt = dt_;
    const LinearConstraints torso_constraints =
        calculateTorsoVelocityConstraintsInContext(
            torso_request, feasibility_context_.get(), &feasibility_q_full_);
    message.torso_frame = torso_frame_name_;
    message.torso_constraint_reference_frame = base_frame_name_;
    const auto torso_pose = plant_->GetFrameByName(torso_frame_name_)
                                .CalcPoseInWorld(*feasibility_context_);
    message.torso_pose = toPose(
        torso_pose.translation(),
        Eigen::Quaterniond(torso_pose.rotation().matrix()));
    message.torso_constraints_valid = torso_constraints.valid;
    for (int constraint = 0; constraint < kTorsoConstraintCount;
         ++constraint) {
      message.torso_constraint_value.at(constraint) =
          torso_constraints.value.at(constraint);
      message.torso_constraint_lower.at(constraint) =
          torso_constraints.position_lower.at(constraint);
      message.torso_constraint_upper.at(constraint) =
          torso_constraints.position_upper.at(constraint);
      message.torso_lower_margin.at(constraint) =
          torso_constraints.value.at(constraint) -
          torso_constraints.position_lower.at(constraint);
      message.torso_upper_margin.at(constraint) =
          torso_constraints.position_upper.at(constraint) -
          torso_constraints.value.at(constraint);
    }
    valid = valid && torso_constraints.valid;
    message.valid = valid;
    return message;
  }

  bool calculateFullPoseTaskTerms(
      const TargetSnapshot& target, const DecisionVector& q, double ff_scale,
      double acquisition_blend, double input_budget,
      bool catchup_correction,
      DecisionMatrix* hessian, DecisionVector* gradient,
      DecisionMatrix* jacobian, DecisionVector* command,
      std::array<double, 6>* task_error,
      std::array<double, kArmCount>* sigma_min,
      std::array<double, kArmCount>* translation_sigma) {
    if (!target.arm_target_torso_relative) {
      return false;
    }
    const int arms = (armIsControlled(0) ? 1 : 0) + (armIsControlled(1) ? 1 : 0);
    jacobian->setZero(6 * arms, decisionDofCount());
    command->setZero(6 * arms);
    setPlantDecisionPositions(q);
    const auto& torso = plant_->GetFrameByName(torso_frame_name_);
    int row = 0;
    for (int arm = 0; arm < kArmCount; ++arm) {
      if (!armIsControlled(arm)) continue;
      const auto pose = plant_->GetFrameByName(hand_frame_names_.at(arm))
          .CalcPose(*plant_context_, torso);
      const auto& goal = target.arm.at(arm);
      const Eigen::Vector3d ep = goal.hand_position - pose.translation();
      const Eigen::Vector3d er = rotationLog(
          goal.hand_orientation.toRotationMatrix() *
          pose.rotation().matrix().transpose());
      const DecisionJacobian3d jp = frameTranslationJacobianRelative(
          hand_frame_names_.at(arm), torso_frame_name_);
      const DecisionJacobian3d jr = frameAngularJacobianRelative(
          hand_frame_names_.at(arm), torso_frame_name_);
      const auto budget_twist=[&](Eigen::Vector3d velocity,double slow) -> Eigen::Vector3d {
        const double norm=velocity.norm();
        const double cap=input_budget*((1.-acquisition_blend)*std::min(slow,norm)+acquisition_blend*norm);
        if (norm>cap && norm>1e-12) velocity*=cap/norm;
        return velocity;
      };
      const auto bounded_correction=[&](Eigen::Vector3d velocity,double cap) -> Eigen::Vector3d {
        if (catchup_correction && velocity.norm()>cap) velocity*=cap/velocity.norm();
        return velocity;
      };
      const Eigen::Vector3d vp = budget_twist(ff_scale * goal.hand_linear_ff + bounded_correction(hand_position_gain_ * ep,acquisition_linear_speed_),acquisition_linear_speed_);
      const Eigen::Vector3d vr = budget_twist(ff_scale * goal.hand_angular_ff + bounded_correction(hand_orientation_gain_ * er,acquisition_angular_speed_),acquisition_angular_speed_);
      jacobian->block(row, 0, 3, decisionDofCount()) = jp;
      jacobian->block(row + 3, 0, 3, decisionDofCount()) = jr;
      command->segment<3>(row) = vp;
      command->segment<3>(row + 3) = vr;
      addLeastSquaresCost(jp, vp, hand_position_weight_, hessian, gradient);
      addLeastSquaresCost(
          feasibility_characteristic_length_ * jr,
          feasibility_characteristic_length_ * vr,
          hand_orientation_weight_, hessian, gradient);
      task_error->at(arm * 3) = ep.norm();
      task_error->at(arm * 3 + 1) = er.norm();
      Matrix6x7d arm_jacobian;
      arm_jacobian.topRows<3>() = selectArmColumns(jp, arm);
      arm_jacobian.bottomRows<3>() =
          feasibility_characteristic_length_ * selectArmColumns(jr, arm);
      sigma_min->at(arm) = Eigen::JacobiSVD<Matrix6x7d>(arm_jacobian)
          .singularValues().minCoeff();
      translation_sigma->at(arm) = Eigen::JacobiSVD<Matrix3x7d>(
          selectArmColumns(jp, arm)).singularValues().minCoeff();
      if (goal.elbow_valid) {
        const Eigen::Vector3d ee = goal.elbow_position -
            plant_->GetFrameByName(elbow_frame_names_.at(arm))
                .CalcPose(*plant_context_, torso).translation();
        task_error->at(arm * 3 + 2) = ee.norm();
        addLeastSquaresCost(
            frameTranslationJacobianRelative(elbow_frame_names_.at(arm), torso_frame_name_),
            budget_twist(ff_scale * goal.elbow_linear_ff + bounded_correction(elbow_position_gain_ * ee,acquisition_linear_speed_),acquisition_linear_speed_),
            elbow_position_weight_, hessian, gradient);
      }
      row += 6;
    }
    return jacobian->array().isFinite().all() && command->array().isFinite().all();
  }

  double primaryRegularizationForArm(
      const std::array<double, kArmCount>& sigma_min, int arm) const {
    const double singularity_activation = std::clamp(
        (singularity_soft_threshold_ - sigma_min.at(arm)) /
            std::max(singularity_soft_threshold_ -
                         singularity_hard_threshold_,
                     1.0e-6),
        0.0, 1.0);

    return primary_regularization_weight_ +
           (primary_regularization_max_weight_ -
            primary_regularization_weight_) *
               singularity_activation * singularity_activation;
  }

  bool calculatePositionVelocityBounds(
      const SolverRequest& request, bool apply_operational_envelope,
      DecisionVector* lower, DecisionVector* upper) const {
    *lower = -velocity_limit_;
    *upper = velocity_limit_;
    DecisionVector safe_lower = position_lower_.array() + position_limit_margin_;
    DecisionVector safe_upper = position_upper_.array() - position_limit_margin_;
    for (int i = 0; i < decisionDofCount(); ++i) {
      const bool base = isDecisionGroup(i, DofGroup::kBase);
      const bool lower_body = isDecisionGroup(i, DofGroup::kLowerBody);
      if (base && !enable_base_motion_) {
        // Fixed-base operation is an equality in every task/braking QP.
        // Root intent, recruitment and nullspace tasks cannot override it.
        (*lower)(i) = 0.0;
        (*upper)(i) = 0.0;
        continue;
      }
      if (lower_body) {
        const auto envelope = motion_capture_ik::whole_body_ik::readyJointEnvelope(
            position_lower_(i), position_upper_(i), preparation_position_(i),
            position_limit_margin_, boundary_brake_guard_);
        safe_lower(i) = envelope.lower;
        safe_upper(i) = envelope.upper;
      }
      double recruitment = 1.0;
      if (base) {
        recruitment = request.base_session_bounds_valid
            ? std::clamp(request.base_recruitment, 0.0, 1.0)
            : 0.0;
      } else if (lower_body) {
        recruitment = std::clamp(
            request.lower_joint_recruitment.at(
                lowerBodyJointForDecisionIndex(i)),
            0.0, 1.0);
      }
      if (recruitment >= 0.5) {
        (*lower)(i) = -velocity_limit_(i);
        (*upper)(i) = velocity_limit_(i);
      } else {
        const double braking_delta =
            acceleration_limit_(i) * request.cycle_dt;
        double braking_velocity = std::clamp(
            0.0, request.reference_dq(i) - braking_delta,
            request.reference_dq(i) + braking_delta);
        if (request.reference_dq(i) * braking_velocity <= 0.0) {
          braking_velocity = 0.0;
        }
        (*lower)(i) = braking_velocity;
        (*upper)(i) = braking_velocity;
      }

      if (base) {
        if (!request.base_session_bounds_valid) {
          continue;
        }
        const int base_dof = baseDofForDecisionIndex(i);
        safe_lower(i) = request.base_position_lower.at(base_dof) +
                        position_limit_margin_;
        safe_upper(i) = request.base_position_upper.at(base_dof) -
                        position_limit_margin_;
      }
      if (request.reference_q(i) < safe_lower(i)) {
        safe_lower(i) = base
            ? request.base_position_lower.at(
                  baseDofForDecisionIndex(i))
            : position_lower_(i);
      }
      if (request.reference_q(i) > safe_upper(i)) {
        safe_upper(i) = base
            ? request.base_position_upper.at(
                  baseDofForDecisionIndex(i))
            : position_upper_(i);
      }
      const double position_dq_lower =
          2.0 * (safe_lower(i) - request.reference_q(i)) / request.cycle_dt - request.reference_dq(i);
      const double position_dq_upper =
          2.0 * (safe_upper(i) - request.reference_q(i)) / request.cycle_dt - request.reference_dq(i);
      (*lower)(i) = std::max((*lower)(i), position_dq_lower);
      (*upper)(i) = std::min((*upper)(i), position_dq_upper);

      // The configured 80% envelope starts a per-joint soft region.  Only the
      // velocity farther toward that joint's physical limit is attenuated;
      // all other joints and the inward direction remain fully available.
      if (apply_operational_envelope &&
          feasibility_mode_ == FeasibilityMode::kActive && !base) {
        const int arm = dof_layout_.armForDecisionIndex(i);
        const bool decision_controlled = arm < 0 || armIsControlled(arm);
        if (decision_controlled) {
          const double lower_scale =
              lowerOutwardVelocityScale(request.reference_q(i), i);
          const double upper_scale =
              upperOutwardVelocityScale(request.reference_q(i), i);
          (*lower)(i) = std::max(
              (*lower)(i), -velocity_limit_(i) * lower_scale);
          (*upper)(i) = std::min(
              (*upper)(i), velocity_limit_(i) * upper_scale);
        }
      }
      if ((*lower)(i) > (*upper)(i) + 1.0e-10) {
        return false;
      }
    }
    return true;
  }

  HardBounds calculateHardBounds(const SolverRequest& request) const {
    HardBounds result;
    result.lower = DecisionVector::Zero(decisionDofCount());
    result.upper = DecisionVector::Zero(decisionDofCount());
    DecisionVector safety_lower;
    DecisionVector safety_upper;
    if (!calculatePositionVelocityBounds(
            request, false, &safety_lower, &safety_upper)) {
      return result;
    }
    if (envelope_diagnostics_enabled_) {
      result.position_velocity_lower = safety_lower;
      result.position_velocity_upper = safety_upper;
    }

    DecisionVector physical_lower_position =
        position_lower_.array() + position_limit_margin_;
    DecisionVector physical_upper_position =
        position_upper_.array() - position_limit_margin_;
    for (int i = 0; i < decisionDofCount(); ++i) {
      const bool base = isDecisionGroup(i, DofGroup::kBase);
      double lower_stopping_guard = boundary_brake_guard_;
      double upper_stopping_guard = boundary_brake_guard_;
      if (isDecisionGroup(i, DofGroup::kLowerBody)) {
        const auto envelope = motion_capture_ik::whole_body_ik::readyJointEnvelope(
            position_lower_(i), position_upper_(i), preparation_position_(i),
            position_limit_margin_, boundary_brake_guard_);
        physical_lower_position(i) = envelope.lower;
        physical_upper_position(i) = envelope.upper;
        lower_stopping_guard = envelope.lower_stopping_guard;
        upper_stopping_guard = envelope.upper_stopping_guard;
      }
      if (base && request.base_session_bounds_valid) {
        const int base_dof = baseDofForDecisionIndex(i);
        physical_lower_position(i) =
            request.base_position_lower.at(base_dof) +
            position_limit_margin_;
        physical_upper_position(i) =
            request.base_position_upper.at(base_dof) -
            position_limit_margin_;
      }
      if (request.reference_q(i) < physical_lower_position(i)) {
        physical_lower_position(i) =
            base && request.base_session_bounds_valid
                ? request.base_position_lower.at(
                      baseDofForDecisionIndex(i))
                : position_lower_(i);
      }
      if (request.reference_q(i) > physical_upper_position(i)) {
        physical_upper_position(i) =
            base && request.base_session_bounds_valid
                ? request.base_position_upper.at(
                      baseDofForDecisionIndex(i))
                : position_upper_(i);
      }

      const int arm = dof_layout_.armForDecisionIndex(i);
      int arm_joint = -1;
      if (arm >= 0) {
        for (int joint = 0; joint < kJointsPerArm; ++joint) {
          if (armDecisionIndex(arm, joint) == i) {
            arm_joint = joint;
            break;
          }
        }
      }
      const double execution_delay = arm_joint < 0
          ? 0.0
          : (arm_joint < 4 ? arm_boundary_execution_delay_proximal_
                           : arm_boundary_execution_delay_wrist_);

      const auto tighten_to_position_envelope =
          [&](double lower_position, double upper_position,
              double lower_guard, double upper_guard) {
            const auto stopping_guard_for_velocity =
                [&](double next_velocity, double fixed_stopping_guard) {
                  // The delay reserve belongs to dq[k+1], not dq[k]. If a
                  // candidate accelerates toward a limit, accounting only for
                  // the old velocity makes the reserve jump on the next tick
                  // and destroys recursive feasibility.
                  if (separate_stopping_contract_) {
                    // The complete distance (including braking) is placed in
                    // this reserve. The legacy quadratic below is then zero.
                    return fixed_stopping_guard + velocity_ik::discreteStopDistance(
                        std::abs(next_velocity), execution_delay,
                        outward_acceleration_limit_(i), braking_limit_(i),
                        request.cycle_dt);
                  }
                  return fixed_stopping_guard +
                      execution_delay * std::abs(next_velocity) +
                      0.5 * acceleration_limit_(i) * execution_delay *
                          execution_delay;
                };
            const auto upper_state_is_stoppable =
                [&](double next_velocity) {
                  if (velocity_ik::trapezoidSegmentBounds(request.reference_q(i),
                          request.reference_dq(i), next_velocity,
                          request.cycle_dt).second > upper_position + 1.0e-12)
                    return false;
                  if (next_velocity <= 0.0) {
                    return true;
                  }
                  const double next_position = request.reference_q(i) +
                      0.5 * request.cycle_dt *
                          (request.reference_dq(i) + next_velocity);
                  const double available_distance =
                      upper_position - next_position -
                      stopping_guard_for_velocity(next_velocity, upper_guard);
                  if (separate_stopping_contract_) return available_distance + 1.0e-12 >= 0.0;
                  return available_distance + 1.0e-12 >=
                      velocity_ik::discreteStopDistance(next_velocity, 0.0, 0.0,
                          std::max(acceleration_limit_(i), 1.0e-6), request.cycle_dt);
                };
            if (!upper_state_is_stoppable(safety_upper(i))) {
              double infeasible_upper = safety_upper(i);
              double feasible_lower = safety_lower(i);
              if (!upper_state_is_stoppable(feasible_lower)) {
                safety_upper(i) = feasible_lower;
              } else {
                feasible_lower = velocity_ik::certifiedUpperZeroSeed(
                    feasible_lower, infeasible_upper, upper_state_is_stoppable(0.0));
                for (int iteration = 0; iteration < 40; ++iteration) {
                  const double middle =
                      0.5 * (feasible_lower + infeasible_upper);
                  if (upper_state_is_stoppable(middle)) {
                    feasible_lower = middle;
                  } else {
                    infeasible_upper = middle;
                  }
                }
                safety_upper(i) = feasible_lower;
              }
            }
            const auto lower_state_is_stoppable =
                [&](double next_velocity) {
                  if (velocity_ik::trapezoidSegmentBounds(request.reference_q(i),
                          request.reference_dq(i), next_velocity,
                          request.cycle_dt).first < lower_position - 1.0e-12)
                    return false;
                  if (next_velocity >= 0.0) {
                    return true;
                  }
                  const double next_position = request.reference_q(i) +
                      0.5 * request.cycle_dt *
                          (request.reference_dq(i) + next_velocity);
                  const double available_distance =
                      next_position - lower_position -
                      stopping_guard_for_velocity(next_velocity, lower_guard);
                  if (separate_stopping_contract_) return available_distance + 1.0e-12 >= 0.0;
                  return available_distance + 1.0e-12 >=
                      velocity_ik::discreteStopDistance(-next_velocity, 0.0, 0.0,
                          std::max(acceleration_limit_(i), 1.0e-6), request.cycle_dt);
                };
            if (!lower_state_is_stoppable(safety_lower(i))) {
              double infeasible_lower = safety_lower(i);
              double feasible_upper = safety_upper(i);
              if (!lower_state_is_stoppable(feasible_upper)) {
                safety_lower(i) = feasible_upper;
              } else {
                feasible_upper = velocity_ik::certifiedLowerZeroSeed(
                    infeasible_lower, feasible_upper, lower_state_is_stoppable(0.0));
                for (int iteration = 0; iteration < 40; ++iteration) {
                  const double middle =
                      0.5 * (infeasible_lower + feasible_upper);
                  if (lower_state_is_stoppable(middle)) {
                    feasible_upper = middle;
                  } else {
                    infeasible_lower = middle;
                  }
                }
                safety_lower(i) = feasible_upper;
              }
            }
          };

      // URDF bounds are the final non-negotiable guard. The per-candidate
      // delay reserve above changes only the feasible joint velocity; it does
      // not filter or rate-limit the Cartesian target.
      tighten_to_position_envelope(
          physical_lower_position(i), physical_upper_position(i),
          lower_stopping_guard, upper_stopping_guard);

      // The 80% envelope is deliberately absent here: it is a soft operating
      // preference, not a stopping boundary.  Predictive hard braking is tied
      // only to the guarded URDF interval above.
    }

    if (envelope_diagnostics_enabled_) {
      result.stopping_lower = safety_lower;
      result.stopping_upper = safety_upper;
    }
    for (int i = 0; i < decisionDofCount(); ++i) {
      // This is the sole normal-path hard temporal constraint. It is imposed in
      // the same QP that allocates the Cartesian task, so a slow waist/base
      // joint cannot be clipped after the solver has chosen a task direction.
      const double velocity_delta =
          acceleration_limit_(i) * request.cycle_dt;
      result.lower(i) = std::max(
          safety_lower(i), request.reference_dq(i) - velocity_delta);
      result.upper(i) = std::min(
          safety_upper(i), request.reference_dq(i) + velocity_delta);

      // A measured plant can enter the nominal stopping envelope between two
      // samples (tracking lag, contact or estimator latency).  In that state
      // rejecting the complete QP and latching a global fault is the least
      // safe response: it discards the only command that can point the joint
      // back inward.  Open only the minimum local emergency acceleration
      // interval needed to intersect the already-computed hard position and
      // velocity safety box.  The hard box still removes every outward
      // command, so this extra authority cannot be spent on Cartesian task
      // progress.
      if (result.lower(i) > result.upper(i)) {
        const double emergency_delta = emergency_acceleration_scale_ *
            acceleration_limit_(i) * request.cycle_dt;
        const double emergency_lower = std::max(
            safety_lower(i), request.reference_dq(i) - emergency_delta);
        const double emergency_upper = std::min(
            safety_upper(i), request.reference_dq(i) + emergency_delta);
        // At a stopping singleton, sampling/timestamp quantization can put
        // the two analytically identical interval edges a few 1e-5 rad/s
        // apart.  Accept only this scale-relative numerical seam and clamp it
        // back into the hard safety box; larger gaps remain a real fault.
        const double emergency_seam_tolerance =
            std::max(1.0e-10, 1.0e-3 * emergency_delta);
        if (emergency_lower <=
            emergency_upper + emergency_seam_tolerance) {
          const double seam_value = std::clamp(
              0.5 * (emergency_lower + emergency_upper),
              safety_lower(i), safety_upper(i));
          result.lower(i) = emergency_lower <= emergency_upper
              ? emergency_lower
              : seam_value;
          result.upper(i) = emergency_lower <= emergency_upper
              ? emergency_upper
              : seam_value;
          result.emergency_acceleration_used = true;
        }
      }

      // Measured/reference tracking is intentionally not fed back into this
      // normal-path feasible set.  The plant state is delayed relative to the
      // command; making it a moving q boundary makes the command chase its own
      // delayed response and can create wrist oscillation.  The solver owns one
      // trajectory state constrained by q/dq/ddq here.  A delay-aware output
      // watchdog independently brakes only a genuine sustained execution
      // failure.
      // At the stopping boundary the analytically tightened velocity and the
      // maximum one-cycle braking velocity describe the same singleton.  The
      // former is obtained by bisection, however, so their floating-point
      // representations can cross by a few ulps.  Treat that numerical seam
      // as the intended singleton instead of turning a healthy ACTIVE session
      // into a latched dynamic fault.
      const double interval_roundoff_tolerance = 1.0e-8 * std::max(
          {1.0, std::abs(result.lower(i)), std::abs(result.upper(i))});
      if (result.lower(i) > result.upper(i) &&
          result.lower(i) - result.upper(i) <=
              interval_roundoff_tolerance) {
        const double singleton =
            0.5 * (result.lower(i) + result.upper(i));
        result.lower(i) = singleton;
        result.upper(i) = singleton;
      }
      if (result.lower(i) > result.upper(i) + 1.0e-10) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[VelocityIkShadow] dynamic interval empty at dof=%d (%s): "
            "q=%.9f dq=%.9f safety=[%.9f,%.9f] "
            "dynamic=[%.9f,%.9f]",
            i, dof_layout_.decisionSpec(i).name.c_str(),
            request.reference_q(i), request.reference_dq(i),
            safety_lower(i), safety_upper(i), result.lower(i),
            result.upper(i));
        return result;
      }
    }
    result.valid = true;
    return result;
  }

  bool solveGeneralQp(
      const DecisionMatrix& task_hessian,
      const DecisionVector& task_gradient,
      const DecisionVector& lower, const DecisionVector& upper,
      const DecisionMatrix& constraint_matrix,
      const DecisionVector& constraint_lower,
      const DecisionVector& constraint_upper,
      DecisionVector* solution, int* return_code) const {
    velocity_ik::WeightedQp::Stats stats;
    const bool valid = weighted_qp_.solve(task_hessian, task_gradient, lower, upper,
                              constraint_matrix, constraint_lower, constraint_upper,
                              solution, return_code, &stats);
    std_msgs::Float64MultiArray message;
    message.data={ros::Time::now().toSec(), double(task_gradient.size()), double(constraint_matrix.rows()),
                  double(*return_code), double(stats.native_status), double(stats.iterations),
                  stats.primal, stats.dual, stats.solve_us, stats.violation,
                  stats.stationarity, stats.complementarity};
    message.data.insert(message.data.end(),{stats.update_us,stats.check_us,stats.total_us,stats.begin_ns,stats.end_ns,double(stats.acceptance_status),stats.accepted?1.0:0.0});
    message.data.insert(message.data.end(),{stats.lock_wait_us,stats.prepare_us,
                                            stats.workspace_created?1.0:0.0});
    last_qp_finish_ns_=stats.end_ns;
#ifndef VELOCITY_IK_OFFLINE_TEST
    const auto publisher=qp_stats_pub_;
    diagnostics_.submit([publisher,message]{publisher.publish(message);});
#endif
    return valid;
  }

  void resetReference(const DecisionVector& measured_q, const DecisionVector& measured_dq) {
    governor_risk_.fill(0.);
    reference_q_ = measured_q.cwiseMax(position_lower_).cwiseMin(position_upper_);
    session_posture_position_ = reference_q_;
    // The captured low posture is valid. ACTIVE body tracking has no spring
    // toward calibration/ready q; PREPARE alone commands that explicit pose.
    // Caller must have established short-window measured standstill. Raw
    // instantaneous encoder-speed noise is not a reason to reject that window.
    if (!measured_dq.array().isFinite().all() || !startup_readiness_.ready) {
      throw std::logic_error("resetReference requires measured standstill");
    }
    reference_dq_.setZero();
    reference_ddq_.setZero();
    reference_initialized_ = true;
    fallback_active_ = true;
    tracking_ramp_scale_ = 0.0;
    reference_history_.clear();
  }

  void recordReferenceHistory(const ros::WallTime& stamp) {
    if (!reference_initialized_ ||
        reference_q_.size() != decisionDofCount() ||
        !reference_q_.array().isFinite().all()) {
      reference_history_.clear();
      return;
    }
    reference_history_.push_back({stamp, reference_q_});
    const double maximum_age = std::max(
        0.20, reference_tracking_delay_compensation_ +
                  reference_tracking_delay_window_ + 0.05);
    while (!reference_history_.empty() &&
           (stamp - reference_history_.front().stamp).toSec() > maximum_age) {
      reference_history_.pop_front();
    }
  }

  bool referenceHistoryReady(const ros::WallTime& now) const {
    if (reference_history_.empty()) {
      return false;
    }
    const double maximum_delay = std::min(
        0.15, reference_tracking_delay_compensation_ +
                  reference_tracking_delay_window_);
    return (now - reference_history_.front().stamp).toSec() >= maximum_delay;
  }

  double minimumDelayedReferenceError(
      const ros::WallTime& now, int index, double measured_position,
      double* matched_q = nullptr, double* matched_wall_s = nullptr) const {
    if (matched_q) *matched_q = std::numeric_limits<double>::quiet_NaN();
    if (matched_wall_s) *matched_wall_s = std::numeric_limits<double>::quiet_NaN();
    const double minimum_delay = std::max(
        0.0, reference_tracking_delay_compensation_ -
                 reference_tracking_delay_window_);
    const double maximum_delay = std::min(
        0.15, reference_tracking_delay_compensation_ +
                  reference_tracking_delay_window_);
    double minimum_error = std::numeric_limits<double>::infinity();
    for (auto iterator = reference_history_.rbegin();
         iterator != reference_history_.rend(); ++iterator) {
      const double age = (now - iterator->stamp).toSec();
      if (age + 1.0e-9 < minimum_delay) {
        continue;
      }
      if (age > maximum_delay + 1.0e-9) {
        break;
      }
      if (iterator->q.size() == decisionDofCount() &&
          std::isfinite(iterator->q(index))) {
        if (std::abs(iterator->q(index) - measured_position) < minimum_error) {
          if (matched_q) *matched_q = iterator->q(index);
          if (matched_wall_s) *matched_wall_s = iterator->stamp.toSec();
        }
        minimum_error = std::min(
            minimum_error,
            std::abs(iterator->q(index) - measured_position));
      }
    }
    return minimum_error;
  }

  void startFeasibilityWorker() {
    if (feasibility_mode_ == FeasibilityMode::kOff) {
      return;
    }
    std::lock_guard<std::mutex> lock(feasibility_mutex_);
    if (feasibility_running_) {
      return;
    }
    feasibility_stop_requested_ = false;
    feasibility_running_ = true;
    feasibility_thread_ =
        std::thread(&PinocchioVelocityIk::feasibilityWorkerLoop, this);
  }

  void stopFeasibilityWorker() {
    {
      std::lock_guard<std::mutex> lock(feasibility_mutex_);
      if (!feasibility_running_) {
        return;
      }
      feasibility_stop_requested_ = true;
      feasibility_request_pending_ = false;
    }
    feasibility_condition_.notify_all();
    if (feasibility_thread_.joinable()) {
      feasibility_thread_.join();
    }
    std::lock_guard<std::mutex> lock(feasibility_mutex_);
    feasibility_running_ = false;
  }

  void queueFeasibilityRequest(const FeasibilityRequest& request) {
    if (feasibility_mode_ == FeasibilityMode::kOff) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(feasibility_mutex_);
      feasibility_request_ = request;
      feasibility_request_pending_ = true;
    }
    feasibility_condition_.notify_one();
  }

  void feasibilityWorkerLoop() {
    while (true) {
      FeasibilityRequest request;
      {
        std::unique_lock<std::mutex> lock(feasibility_mutex_);
        feasibility_condition_.wait(lock, [this]() {
          return feasibility_stop_requested_ || feasibility_request_pending_;
        });
        if (feasibility_stop_requested_) {
          return;
        }
        request = feasibility_request_;
        feasibility_request_pending_ = false;
      }
      try {
        feasibility_debug_pub_.publish(calculateFeasibilityDebug(request));
      } catch (const std::exception& error) {
        // Feasibility runs on a diagnostic worker.  A diagnostic failure
        // must never stop the 500 Hz reference supervisor or its QP.
        ROS_ERROR_THROTTLE(
            1.0, "[VelocityIkShadow] feasibility shadow failed: %s",
            error.what());
      } catch (...) {
        ROS_ERROR_THROTTLE(
            1.0, "[VelocityIkShadow] feasibility shadow failed: unknown error");
      }
    }
  }

  velocity_ik::OperatingJoint operatingJoint(const SolverRequest& request, int i) const {
    velocity_ik::OperatingJoint p;
    p.q = request.reference_q(i); p.v = request.reference_dq(i); p.dt = request.cycle_dt;
    p.lower = position_lower_(i) + position_limit_margin_;
    p.upper = position_upper_(i) - position_limit_margin_;
    p.lower_guard = p.upper_guard = boundary_brake_guard_;
    if (isDecisionGroup(i, DofGroup::kLowerBody)) {
      const auto ready = motion_capture_ik::whole_body_ik::readyJointEnvelope(
          position_lower_(i), position_upper_(i), preparation_position_(i),
          position_limit_margin_, boundary_brake_guard_);
      p.lower = ready.lower; p.upper = ready.upper;
      p.lower_guard = ready.lower_stopping_guard; p.upper_guard = ready.upper_stopping_guard;
      p.lower_comfort = preparation_position_(i) > position_lower_(i) +
          position_limit_margin_ + boundary_brake_guard_;
      p.upper_comfort = preparation_position_(i) < position_upper_(i) -
          position_limit_margin_ - boundary_brake_guard_;
    }
    // Exactly the same recovery envelope and delay contract as hard bounds.
    if (p.q < p.lower) p.lower = position_lower_(i);
    if (p.q > p.upper) p.upper = position_upper_(i);
    const int arm = dof_layout_.armForDecisionIndex(i);
    if (arm >= 0) {
      for (int joint = 0; joint < kJointsPerArm; ++joint)
        if (armDecisionIndex(arm, joint) == i)
          p.delay = joint < 4 ? arm_boundary_execution_delay_proximal_ : arm_boundary_execution_delay_wrist_;
    }
    p.vmax = velocity_limit_(i); p.acceleration = acceleration_limit_(i);
    p.braking = braking_limit_(i); p.outward = outward_acceleration_limit_(i);
    p.separate_stop = separate_stopping_contract_;
    if(arm>=0 && joint_jerk_enabled_) {
      p.current_acceleration=request.reference_ddq(i);
      p.jerk=joint_jerk_limits_(i);
    }
    p.extra_position = operating_extra_position_(i); p.extra_time = operating_extra_time_(i);
    p.lower_scale = lowerOutwardVelocityScale(p.q, i);
    p.upper_scale = upperOutwardVelocityScale(p.q, i);
    return p;
  }

  // Bound attribution is geometric, NOT a claim about a nonzero QP dual.
  int hardBoundReason(const HardBounds& h, const SolverRequest& r, int i, bool upper) const {
    const double edge = upper ? h.upper(i) : h.lower(i);
    const auto near = [edge](double x) { return std::abs(edge - x) <= 2.e-8; };
    int bits = near(upper ? velocity_limit_(i) : -velocity_limit_(i)) ? 1 : 0;
    const double pv = upper ? h.position_velocity_upper(i) : h.position_velocity_lower(i);
    const double stop = upper ? h.stopping_upper(i) : h.stopping_lower(i);
    if (near(pv) && std::abs(pv) < velocity_limit_(i) - 2.e-8) bits |= 2;
    if (near(stop) && std::abs(stop - pv) > 2.e-8) bits |= 4;
    if (near(r.reference_dq(i) + (upper ? 1. : -1.) * acceleration_limit_(i) * r.cycle_dt)) bits |= 8;
    if (std::abs(edge - r.reference_dq(i)) > acceleration_limit_(i) * r.cycle_dt + 2.e-8) bits |= 16;
    return bits;
  }

  SolverCandidate solveRequest(const SolverRequest& request) {
    velocity_ik::runtime_probe::point(41);
    const auto start = std::chrono::steady_clock::now();
    SolverCandidate candidate;
    const int n = decisionDofCount();
    candidate.task_dq = candidate.primary_dq = candidate.secondary_dq = DecisionVector::Zero(n);
    candidate.base_reference_seq = request.base_reference_seq;
    // This candidate identity is also used at publication after adoption.
    candidate.diagnostics_due = request.base_reference_seq % envelope_diagnostics_stride_ == 0;
    candidate.session_id = request.target.session_id;
    candidate.generation = request.target.generation;
    candidate.target_seq = request.target.sequence;
    candidate.solver_lower_joint_recruitment.fill(1.0);
    candidate.solver_lower_body_recruitment = 1.0;
    candidate.solver_base_recruitment = 0.0;
    candidate.position_task_weight_scale.fill(1.0);
    candidate.orientation_task_scale.fill(0.0);
    candidate.translation_task_scale.fill(0.0);
    candidate.secondary_return_code = 0;  // No secondary QP exists.
    candidate.return_code = -1002;
    if (request.tracking) {
      DecisionMatrix h = DecisionMatrix::Zero(n, n);
      DecisionVector g = DecisionVector::Zero(n);
      DecisionMatrix hands_j, body_j;
      DecisionVector hands_v, body_v;
      double ff_scale = 1.0;
      if (!(continuous_reference_enabled_ && request.target.estimated_twists_valid) &&
          request.feedforward_age > ff_hold_time_) {
        ff_scale = std::exp(-(request.feedforward_age - ff_hold_time_) /
                            std::max(ff_decay_time_constant_, 1.0e-6));
      }
      // FK, constraints, integration and WBC all use ONE command trajectory.
      const bool tasks_valid = calculateFullPoseTaskTerms(
          request.target, request.reference_q, ff_scale,
          request.acquisition_blend, request.input_budget,
          request.catchup_correction,
          &h, &g, &hands_j, &hands_v, &candidate.task_error,
          &candidate.sigma_min, &candidate.translation_sigma) &&
          calculateBodyTaskTerms(
              request, request.reference_q, &body_j, &body_v, &h, &g,
              &candidate.torso_task_error, &candidate.base_intent_error,
              &candidate.root_translation_intent_active);
      for (int i = 0; i < n; ++i) {
        const int arm = dof_layout_.armForDecisionIndex(i);
        double weight = 1.0e-6;
        double preferred_velocity = 0.0;
        if (arm >= 0) {
          weight = primaryRegularizationForArm(candidate.sigma_min, arm);
          candidate.primary_regularization.at(arm) = weight;
          // Local soft avoidance only; never mute an entire hand task.
          const double barrier = jointLimitBarrierDirection(request.reference_q, i);
          g(i) -= 2.0e-4 * barrier;
        } else if (isDecisionGroup(i, DofGroup::kLowerBody)) {
          const int joint = lowerBodyJointForDecisionIndex(i);
          weight = motion_capture_ik::whole_body_ik::driveRegularization(
              lower_joint_effort_multiplier_.at(joint),
              lower_joint_cost_velocity_scale_.at(joint));
          // Minimum-effort redundancy resolution, not a ready-pose spring.
        }
        h(i, i) += 2.0 * weight;
        g(i) -= 2.0 * weight * preferred_velocity;
      }
      bool temporal_valid = true;
      if (temporal_cost_enabled_) {
        for (int i = 0; i < n; ++i) {
          if (isDecisionGroup(i, DofGroup::kBase)) continue;
          temporal_valid &= velocity_ik::addTemporalCost(temporal_cost_[i], request.cycle_dt,
              request.reference_dq(i), request.reference_ddq(i), &h(i, i), &g(i));
        }
      }
      last_task_us_=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
      velocity_ik::runtime_probe::point(42);
      const auto bounds_begin=std::chrono::steady_clock::now();
        velocity_ik::runtime_probe::point(43);
      const HardBounds bounds = calculateHardBounds(request);
      candidate.emergency_acceleration_used = bounds.emergency_acceleration_used;
      DecisionVector effective_lower = bounds.lower, effective_upper = bounds.upper;
      // Hard physical bounds take precedence. Preserve every feasible jerk
      // interval BEFORE applying comfort, acquisition or input preferences.
      std::vector<velocity_ik::JointJerkInterval> hard_jerk(n);
      bool jerk_valid = true;
      candidate.joint_jerk_active = joint_jerk_enabled_;
      if (bounds.valid && candidate.joint_jerk_active) {
        for (int arm=0; arm<kArmCount; ++arm) {
          if (!armIsControlled(arm)) continue;
          for (int joint=0; joint<kJointsPerArm; ++joint) {
            const int i=armDecisionIndex(arm,joint);
            hard_jerk[i]=velocity_ik::jointJerkInterval(bounds.lower(i),bounds.upper(i),
                request.reference_dq(i),request.reference_ddq(i),request.cycle_dt,
                joint_jerk_limits_(i));
            jerk_valid &= hard_jerk[i].valid;
            if (hard_jerk[i].valid) {
              effective_lower(i)=hard_jerk[i].lower;
              effective_upper(i)=hard_jerk[i].upper;
            }
          }
        }
      }
      std::vector<velocity_ik::OperatingInterval> operating;
      bool operating_valid = jerk_valid;
      const bool collect = envelope_diagnostics_enabled_ &&
          candidate.diagnostics_due;
      if (bounds.valid && (operating_envelope_enabled_ || collect)) {
        operating.resize(n);
        for (int i = 0; i < n; ++i) {
          if (isDecisionGroup(i, DofGroup::kBase)) {
            operating[i] = velocity_ik::feasibleOperatingInterval(bounds.lower(i), bounds.upper(i),
                                                                 bounds.lower(i), bounds.upper(i));
            continue;
          }
          operating[i] = velocity_ik::operatingInterval(operatingJoint(request, i), effective_lower(i), effective_upper(i));
          if (operating_envelope_enabled_) {
            operating_valid &= operating[i].valid;
            effective_lower(i) = operating[i].lower;
            effective_upper(i) = operating[i].upper;
          }
        }
      }
      const bool restricted=request.acquisition_blend<1. || request.input_budget<1.;
      if (bounds.valid && operating_valid && restricted) {
        // Intersect behavioral budgets inside the QP. Always retain the
        // current hard-domain braking endpoint: lowering a budget at speed
        // must not demand an impossible instantaneous velocity change.
        const double blend=std::clamp(request.acquisition_blend,0.,1.);
        for (int i=0;i<n;++i) {
          if (isDecisionGroup(i,DofGroup::kBase)) continue;
          const double cap=request.input_budget*((1.-blend)*std::min(velocity_limit_(i),acquisition_joint_speed_)+blend*velocity_limit_(i));
          const auto speed=velocity_ik::feasibleOperatingInterval(effective_lower(i),effective_upper(i),-cap,cap);
          const double accel=(1.-blend)*std::min(acceleration_limit_(i),acquisition_acceleration_)+blend*acceleration_limit_(i);
          const auto rate=velocity_ik::feasibleOperatingInterval(speed.lower,speed.upper,
              request.reference_dq(i)-accel*request.cycle_dt,
              request.reference_dq(i)+accel*request.cycle_dt);
          operating_valid &= speed.valid && rate.valid;
          effective_lower(i)=rate.lower; effective_upper(i)=rate.upper;
          candidate.continuity_relaxation=std::max(candidate.continuity_relaxation,
              std::max({speed.lower_relaxation,speed.upper_relaxation,rate.lower_relaxation,rate.upper_relaxation}));
        }
        // No all-joint worst-case Jacobian shrink: that budget reduced actual
        // hardware motion to ~0.02 rad/s and coupled both hands unnecessarily.
        // Initial offset correction is bounded in the task; each joint keeps
        // its own feasible velocity/acceleration budget and all hard guards.
      }
      if (tasks_valid && bounds.valid && temporal_valid && operating_valid) {
        // Comfort budgets preserve the closest-to-zero point of the hard
        // AND jerk domain. Torso preferences admit that same reachable
        // braking baseline; physical joint bounds are never expanded.
        LinearConstraints torso = makeConstraintsRecursivelyFeasible(
            calculateTorsoVelocityConstraintsInContext(
                request, plant_context_.get(), &q_full_),
            effective_lower, effective_upper, &candidate.torso_viability_relaxation);
        last_bounds_us_=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-bounds_begin).count();
        velocity_ik::runtime_probe::point(44);
        if (torso.valid) {
          // Eliminate the three fixed base variables: the actual QP is 18-D,
          // while the wire ABI and reference consumer remain 21-D.
          std::vector<int> active;
          for (int i = 0; i < n; ++i) {
            if (!isDecisionGroup(i, DofGroup::kBase)) active.push_back(i);
          }
          const int na = static_cast<int>(active.size());
          DecisionMatrix ha(na, na), aa(torso.matrix.rows(), na);
          DecisionVector ga(na), lo(na), hi(na), va;
          for (int i = 0; i < na; ++i) {
            ga(i) = g(active[i]);
            lo(i) = effective_lower(active[i]);
            hi(i) = effective_upper(active[i]);
            aa.col(i) = torso.matrix.col(active[i]);
            for (int j = 0; j < na; ++j) ha(i, j) = h(active[i], active[j]);
          }
          DecisionVector constraint_lower=torso.lower, constraint_upper=torso.upper;
          std::vector<velocity_ik::ExecutionGovernorRow> governor_rows;
          std::vector<int> governor_arms;
          if (execution_governor_enabled_ && request.measured_state_valid &&
              request.measured_q.size()==n && request.measured_dq.size()==n &&
              request.measured_q.allFinite() && request.measured_dq.allFinite()) {
            for (int arm=0;arm<kArmCount;++arm) {
              if (!armIsControlled(arm)) continue;
              const int i=governor_indices_[arm];
              const auto found=std::find(active.begin(),active.end(),i);
              if (found==active.end()) continue;
              const int ai=std::distance(active.begin(),found);
              auto r=velocity_ik::executionGovernorRow(governor_config_[arm], ai,
                  request.reference_q(i),request.reference_dq(i),request.reference_ddq(i),
                  request.measured_q(i),request.measured_dq(i),request.cycle_dt,
                  request.governor_risk[arm],std::max(ha(ai,ai),1e-6));
              // A smoothing preference must not request acceleration beyond
              // the already feasible interval (especially during braking).
              r.smooth_target=std::clamp(r.smooth_target,lo(ai),hi(ai));
              candidate.governor_risk[arm]=r.risk;
              governor_rows.push_back(r); governor_arms.push_back(arm);
            }
          }
          // Exact transparent fast path: if the entire physical velocity box
          // is already inside the soft tube, solve the original 18-D QP.
          // Avoid even numerical changes from inactive auxiliary variables.
          const bool governor_qp_active=std::any_of(governor_rows.begin(),governor_rows.end(),
              [&](const auto& r) { return r.smoothing_curvature>0. ||
                  r.lower>lo(r.index) || r.upper<hi(r.index); });
          if (governor_qp_active)
            velocity_ik::augmentExecutionGovernorQp(governor_rows,&ha,&ga,&lo,&hi,
                &aa,&constraint_lower,&constraint_upper);
          std::array<int,14> jerk_indices{};
          if(candidate.joint_jerk_active) {
            // Inactive rows carry no measurements; row index -1 distinguishes them.
            if (candidate.diagnostics_due) {
              candidate.joint_jerk_diagnostics.fill(std::numeric_limits<double>::quiet_NaN());
              for(int row=0;row<14;++row)candidate.joint_jerk_diagnostics[9*row]=-1.;
            }
            for(int arm=0;arm<kArmCount;++arm)for(int joint=0;joint<kJointsPerArm;++joint) {
              const int row=arm*kJointsPerArm+joint,i=armDecisionIndex(arm,joint);
              const auto found=std::find(active.begin(),active.end(),i);
              jerk_indices[row]=found==active.end()?-1:std::distance(active.begin(),found);
              if(!armIsControlled(arm)||jerk_indices[row]<0)continue;
              // Relaxation reports a conflict with the original physical
              // box only. Later behavioral budgets cannot cause jerk excess.
              if (candidate.diagnostics_due) candidate.joint_jerk_diagnostics[9*row+6]=hard_jerk[i].safety_relaxed;
            }
          }
          if(jerk_valid)candidate.solver_valid=solveGeneralQp(
              ha,ga,lo,hi,aa,constraint_lower,constraint_upper,&va,&candidate.return_code);
          else candidate.return_code=-1012;
#ifdef VELOCITY_IK_OFFLINE_TEST
          if(candidate.joint_jerk_active && offline_reject_jerk_solve_) {
            candidate.solver_valid=false;candidate.return_code=-1999;
          }
#endif
          if(candidate.joint_jerk_active && jerk_valid && !candidate.solver_valid) {
            // One bounded numerical retry of the SAME certified problem.
            // Restoring a pre-jerk comfort box here would reintroduce exactly
            // the acceleration discontinuity this ordering prevents.
            candidate.joint_jerk_retry=true;candidate.joint_jerk_first_code=candidate.return_code;
            candidate.solver_valid=solveGeneralQp(
                ha,ga,lo,hi,aa,constraint_lower,constraint_upper,&va,&candidate.return_code);
          }
          if(candidate.joint_jerk_active && candidate.solver_valid) {
            for(int arm=0;arm<kArmCount;++arm)for(int joint=0;joint<kJointsPerArm;++joint) {
              const int row=arm*kJointsPerArm+joint,i=armDecisionIndex(arm,joint),ai=jerk_indices[row];
              if(!armIsControlled(arm)||ai<0)continue;
              if (candidate.diagnostics_due) {
              const double a=(va(ai)-request.reference_dq(i))/request.cycle_dt;
              const double j=(a-request.reference_ddq(i))/request.cycle_dt;
              auto& d=candidate.joint_jerk_diagnostics;const int b=9*row;
              d[b]=i;d[b+1]=joint_jerk_limits_(i);d[b+2]=request.reference_ddq(i);
              d[b+3]=a;d[b+4]=j;d[b+5]=std::max(0.,std::abs(j)-d[b+1]);
              d[b+7]=lo(ai);d[b+8]=hi(ai);
              }
              // The commit certificate uses precisely the accepted QP box.
              effective_lower(i)=lo(ai);effective_upper(i)=hi(ai);
            }
          }
          for (std::size_t j=0;j<governor_rows.size();++j) {
            const auto& r=governor_rows[j]; const int arm=governor_arms[j];
            const double u=candidate.solver_valid ? va(r.index) : std::numeric_limits<double>::quiet_NaN();
            const double sp=candidate.solver_valid ? (governor_qp_active ? va(na+2*j) : 0.) : u;
            const double sn=candidate.solver_valid ? (governor_qp_active ? va(na+2*j+1) : 0.) : u;
            const double predicted=r.error+governor_config_[arm].prediction_time*
                (u-request.measured_dq(governor_indices_[arm]));
            const std::array<double,18> row{{double(governor_indices_[arm]),r.error,
                r.relative_velocity,r.predicted_error,r.lower,r.upper,r.risk,r.smoothing,
                u,sp,sn,predicted,governor_qp_active?r.penalty_curvature:0.,r.smoothing_curvature,
                lo(r.index),hi(r.index),r.smooth_target,double(arm)}};
            candidate.governor_diagnostics.insert(candidate.governor_diagnostics.end(),row.begin(),row.end());
          }
          if (candidate.solver_valid && !qp_snapshot_directory_.empty() &&
              (++qp_snapshot_counter_ % 250 == 0) && qp_snapshot_counter_ <= 100000) {
            const std::string path = qp_snapshot_directory_ + "/qp_" + std::to_string(qp_snapshot_counter_) + ".json";
            const auto q = request.reference_q, v = request.reference_dq;
            const double stamp = request.target.source_stamp.toSec();
            const auto seq = request.target.source_frame_seq;
            const auto stats = weighted_qp_.stats();
            diagnostics_.submit([path, ha, ga, lo, hi, aa, constraint_lower, constraint_upper, va, q, v, stamp, seq, stats, hands_j, hands_v] {
              auto vector_components = [](const auto& x) { return std::vector<double>(x.data(), x.data() + x.size()); };
              nlohmann::json j{{"source_stamp",stamp},{"source_seq",seq},{"q",vector_components(q)},{"v",vector_components(v)},
                {"H_colmajor",vector_components(ha)},{"g",vector_components(ga)},{"lo",vector_components(lo)},{"hi",vector_components(hi)},
                {"C_colmajor",vector_components(aa)},{"C_rows",aa.rows()},{"cl",vector_components(constraint_lower)},{"cu",vector_components(constraint_upper)},
                {"solution",vector_components(va)},{"hand_J_colmajor",vector_components(hands_j)},{"hand_rows",hands_j.rows()},
                {"hand_command",vector_components(hands_v)},{"independent_KKT",{stats.violation,stats.stationarity,stats.complementarity}}};
              std::ofstream(path) << j.dump();
            });
          }
          if (candidate.solver_valid) {
            for (int i = 0; i < na; ++i) candidate.task_dq(active[i]) = va(i);
            candidate.primary_dq = candidate.task_dq;
            candidate.primary_task_jacobian.resize(hands_j.rows() + body_j.rows(), n);
            candidate.primary_task_jacobian.topRows(hands_j.rows()) = hands_j;
            candidate.primary_task_jacobian.bottomRows(body_j.rows()) = body_j;
            candidate.primary_task_velocity.resize(hands_v.size() + body_v.size());
            candidate.primary_task_velocity.head(hands_v.size()) = hands_v;
            candidate.primary_task_velocity.tail(body_v.size()) = body_v;
            candidate.body_task_jacobian = body_j;
            candidate.body_task_velocity = body_v;
            const auto progress = [](const DecisionVector& desired, const DecisionVector& actual) {
              // Zero request has no defined progress; report zero, not success.
              return desired.squaredNorm() > 1.0e-12
                  ? std::clamp(actual.dot(desired) / desired.squaredNorm(), 0.0, 1.0)
                  : 0.0;
            };
            int row = 0;
            for (int arm = 0; arm < kArmCount; ++arm) {
              if (!armIsControlled(arm)) continue;
              candidate.translation_task_scale.at(arm) = progress(
                  hands_v.segment(row, 3),
                  hands_j.middleRows(row, 3) * candidate.task_dq);
              candidate.orientation_task_scale.at(arm) = progress(
                  hands_v.segment(row + 3, 3),
                  hands_j.middleRows(row + 3, 3) * candidate.task_dq);
              row += 6;
            }
            if (body_v.size()) candidate.torso_task_scale =
                progress(body_v, body_j * candidate.task_dq);
            candidate.primary_task_scale = progress(
                candidate.primary_task_velocity,
                candidate.primary_task_jacobian * candidate.task_dq);
            // These fields are telemetry only and never resize a task.
            candidate.certified_dynamic_lower = effective_lower;
            candidate.certified_dynamic_upper = effective_upper;
            candidate.certified_constraint_matrix = torso.matrix;
            candidate.certified_constraint_lower = torso.lower;
            candidate.certified_constraint_upper = torso.upper;
            candidate.dynamic_certificate_valid = true;
          }
        } else {
          candidate.return_code = -1005;
        }
      } else {
        candidate.return_code = !temporal_valid ? -1010 :
            (!operating_valid ? -1011 : (tasks_valid ? -1001 : -1002));
      }
      if (collect) {
        // Header is completed AFTER adoption/ownership checks. Rows describe
        // this solve only; invalid/missing data is NaN, never plausible zeros.
        const double nan = std::numeric_limits<double>::quiet_NaN();
        auto& d = candidate.operating_diagnostics;
        d.assign(28 + 32 * n, nan);
        d[0] = 1; d[1] = n; d[2] = 32; d[5] = request.base_reference_seq;
        d[6] = request.target.session_id; d[7] = request.target.generation;
        d[8] = request.target.sequence; d[9] = request.target.source_stamp.toSec();
        d[10] = request.cycle_dt; d[14] = candidate.solver_valid;
        d[15] = candidate.return_code; d[18] = operating_envelope_enabled_;
        d[19] = temporal_cost_enabled_; d[20] = bounds.valid;
        d[21] = candidate.torso_viability_relaxation;
        if (candidate.solver_valid) {
          d[22] = (hands_j * candidate.task_dq - hands_v).norm();
          d[23] = (body_j * candidate.task_dq - body_v).norm();
        }
        for (int i = 0; i < n && bounds.valid; ++i) {
          const auto& o = operating[i];
          const auto p = operatingJoint(request, i);
          const double u = candidate.solver_valid ? candidate.task_dq(i) : nan;
          const double a = (u - request.reference_dq(i)) / request.cycle_dt;
          const double da = a - request.reference_ddq(i);
          const int binding = candidate.solver_valid ?
              ((std::abs(u - effective_lower(i)) <= 2.e-8 ? 1 : 0) |
               (std::abs(u - effective_upper(i)) <= 2.e-8 ? 2 : 0)) : 0;
          const std::array<double, 32> row{{request.reference_q(i), request.reference_dq(i),
              request.reference_ddq(i), u, bounds.lower(i), bounds.upper(i), o.soft_lower,
              o.soft_upper, effective_lower(i), effective_upper(i), o.baseline,
              o.static_lower, o.static_upper, o.predictive_lower, o.predictive_upper,
              o.lower_relaxation, o.upper_relaxation, double(binding), a, da,
              a / acceleration_limit_(i), da / temporal_cost_[i].delta_acceleration_scale,
              temporal_cost_enabled_ ? velocity_ik::temporalAccelerationCost(temporal_cost_[i], a) : 0.,
              temporal_cost_enabled_ ? velocity_ik::temporalDeltaAccelerationCost(temporal_cost_[i], da) : 0.,
              velocity_ik::operatingStopMarginDiagnostic(p, u, false, false),
              velocity_ik::operatingStopMarginDiagnostic(p, u, true, false),
              velocity_ik::operatingStopMarginDiagnostic(p, u, false, true),
              velocity_ik::operatingStopMarginDiagnostic(p, u, true, true),
              double(hardBoundReason(bounds, request, i, false)),
              double(hardBoundReason(bounds, request, i, true)), h(i, i),
              double((p.lower_comfort ? 0 : 1) | (p.upper_comfort ? 0 : 2))}};
          std::copy(row.begin(), row.end(), d.begin() + 28 + 32 * i);
        }
      }
    }
    candidate.finish_time = ros::WallTime::now();
    candidate.solve_time_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - start).count();
    if (!candidate.operating_diagnostics.empty()) {
      candidate.operating_diagnostics[16] = candidate.solve_time_us;
      candidate.operating_diagnostics[17] = candidate.solver_valid ? last_bounds_us_ :
          std::numeric_limits<double>::quiet_NaN();
    }
    return candidate;
  }

  DecisionVector calculateEmergencyBrakingVelocity(
      const SolverRequest& request) const {
    DecisionVector result = request.reference_dq;
    for (int i = 0; i < decisionDofCount(); ++i) {
      const double maximum_acceleration =
          emergency_acceleration_scale_ * acceleration_limit_(i);
      const double desired_acceleration = std::clamp(
          -request.reference_dq(i) / request.cycle_dt,
          -maximum_acceleration, maximum_acceleration);
      double next_velocity = request.reference_dq(i) +
          request.cycle_dt * desired_acceleration;
      if (request.reference_dq(i) * next_velocity <= 0.0) {
        next_velocity = 0.0;
      }
      result(i) = std::clamp(
          next_velocity, -velocity_limit_(i), velocity_limit_(i));
    }
    return result;
  }

  DecisionVector calculateSupervisedVelocity(const SolverRequest& request,
                                        const DecisionVector& desired_velocity,
                                        bool stabilize_joint_target,
                                        const DecisionMatrix& primary_task_jacobian,
                                        const DecisionVector& primary_task_velocity,
                                        DecisionVector* jerk_slack,
                                        DecisionVector* boundary_brake,
                                        DecisionVector* operational_boundary_brake,
                                        bool* emergency_acceleration_used,
                                        double* task_scale,
                                        double* task_scaling_residual,
                                        DecisionVector* task_scaling_baseline,
                                        DecisionVector* task_scaling_unscaled,
                                        bool* task_scaling_qp_valid,
                                        double* torso_viability_relaxation) {
    HardBounds bounds = calculateHardBounds(request);
    boundary_brake->setZero();
    operational_boundary_brake->setZero();
    *emergency_acceleration_used = bounds.emergency_acceleration_used;
    *task_scaling_qp_valid = false;
    *torso_viability_relaxation = 0.0;
    task_scaling_baseline->resize(0);
    task_scaling_unscaled->resize(0);

    (void)stabilize_joint_target;
    DecisionVector nominal_velocity(decisionDofCount());
    jerk_slack->setZero();
    for (int i = 0; i < decisionDofCount(); ++i) {
      const double target_velocity = std::clamp(
          desired_velocity(i), -velocity_limit_(i), velocity_limit_(i));
      nominal_velocity(i) = target_velocity;
    }

    const LinearConstraints torso_constraints =
        calculateTorsoVelocityConstraintsInContext(
            request, supervisor_context_.get(), &supervisor_q_full_);
    // Zero velocity is not necessarily reachable within one acceleration
    // step. Use the closest-to-zero point of the dynamic box as a guaranteed
    // braking baseline. Coupled torso limits are preferred
    // workspace limits (URDF limits remain hard), so admit exactly the
    // unavoidable baseline rate when one-step dynamics cannot satisfy the
    // nominal coupled interval.  This makes every supervisor cycle recursively
    // feasible without opening any individual position/velocity/dynamic bound.
    DecisionVector projection_lower = bounds.lower;
    DecisionVector projection_upper = bounds.upper;
    if (bounds.valid) {
      // Keep the optimization away from the exact hard boundary.  The guard
      // is relative for narrow lower-body/base acceleration intervals and capped for
      // wider arm intervals; it never consumes more than 40% of the interval.
      for (int i = 0; i < decisionDofCount(); ++i) {
        const double guard = std::min(
            1.0e-4, 0.20 * (bounds.upper(i) - bounds.lower(i)));
        projection_lower(i) += guard;
        projection_upper(i) -= guard;
      }
    }
    DecisionVector braking_baseline = DecisionVector::Zero(decisionDofCount());
    if (bounds.valid) {
      braking_baseline = braking_baseline.cwiseMax(projection_lower).cwiseMin(
          projection_upper);
    }
    LinearConstraints viable_torso_constraints = torso_constraints;
    if (bounds.valid && viable_torso_constraints.valid &&
        viable_torso_constraints.matrix.cols() == decisionDofCount()) {
      const DecisionVector baseline_rate =
          viable_torso_constraints.matrix * braking_baseline;
      for (Eigen::Index row = 0; row < baseline_rate.size(); ++row) {
        *torso_viability_relaxation = std::max(
            *torso_viability_relaxation,
            std::max(viable_torso_constraints.lower(row) -
                         baseline_rate(row),
                     baseline_rate(row) -
                         viable_torso_constraints.upper(row)));
        viable_torso_constraints.lower(row) = std::min(
            viable_torso_constraints.lower(row), baseline_rate(row));
        viable_torso_constraints.upper(row) = std::max(
            viable_torso_constraints.upper(row), baseline_rate(row));
      }
    }
    const bool has_primary_task =
        primary_task_velocity.size() > 0 &&
        primary_task_jacobian.rows() == primary_task_velocity.size() &&
        primary_task_jacobian.cols() == decisionDofCount();

    // This helper is used only for PREPARE/HOLD/fault braking. Normal tracking
    // is solved directly inside the same dynamic feasible domain and bypasses
    // this projection completely.
    DecisionVector result = braking_baseline;
    bool projection_valid = bounds.valid && viable_torso_constraints.valid;
    bool torso_projection_qp_valid = projection_valid;
    if (projection_valid) {
      DecisionVector box_target =
          nominal_velocity.cwiseMax(projection_lower).cwiseMin(
              projection_upper);
      // Coupled torso constraints involve only the four lower-body joints.
      // Project those four coordinates locally; arms and base keep their own
      // independently followable commands.  The previous single 21-D ray
      // scalar let one waist constraint collapse both arm commands to zero.
      DecisionMatrix lower_constraint_matrix(
          viable_torso_constraints.matrix.rows(), kLowerBodyDof);
      DecisionVector fixed_rate =
          viable_torso_constraints.matrix * box_target;
      DecisionVector lower_target(kLowerBodyDof);
      DecisionVector lower_bound(kLowerBodyDof);
      DecisionVector upper_bound(kLowerBodyDof);
      for (int joint = 0; joint < kLowerBodyDof; ++joint) {
        const int index = lowerBodyDecisionIndex(joint);
        lower_constraint_matrix.col(joint) =
            viable_torso_constraints.matrix.col(index);
        fixed_rate.noalias() -=
            viable_torso_constraints.matrix.col(index) * box_target(index);
        lower_target(joint) = box_target(index);
        lower_bound(joint) = projection_lower(index);
        upper_bound(joint) = projection_upper(index);
      }
      const DecisionMatrix projection_hessian =
          2.0 * DecisionMatrix::Identity(kLowerBodyDof, kLowerBodyDof);
      const DecisionVector projection_gradient = -2.0 * lower_target;
      const DecisionVector constraint_lower =
          viable_torso_constraints.lower - fixed_rate;
      const DecisionVector constraint_upper =
          viable_torso_constraints.upper - fixed_rate;
      DecisionVector projected_lower_body;
      int projection_return_code = -1200;
      torso_projection_qp_valid = solveGeneralQp(
          projection_hessian, projection_gradient, lower_bound, upper_bound,
          lower_constraint_matrix, constraint_lower, constraint_upper,
          &projected_lower_body, &projection_return_code);
      if (torso_projection_qp_valid) {
        for (int joint = 0; joint < kLowerBodyDof; ++joint) {
          box_target(lowerBodyDecisionIndex(joint)) =
              projected_lower_body(joint);
        }
      } else {
        // Numerical trouble in the optional four-variable preferred-workspace
        // projection is local: use the already proven-feasible braking point
        // for those four joints, while preserving arm/base progress.  A waist
        // QP must never latch a global 21-D braking fault.
        ROS_WARN_THROTTLE(
            1.0,
            "[VelocityIkShadow] lower-body torso projection QP failed "
            "(code=%d); braking lower body only",
            projection_return_code);
        for (int joint = 0; joint < kLowerBodyDof; ++joint) {
          const int index = lowerBodyDecisionIndex(joint);
          box_target(index) = braking_baseline(index);
        }
      }
      result = box_target.cwiseMax(bounds.lower).cwiseMin(bounds.upper);
    }
    *task_scale = 1.0;
    *task_scaling_residual = 0.0;
    *task_scaling_qp_valid = projection_valid && torso_projection_qp_valid;
    if (has_primary_task) {
      *task_scaling_baseline =
          primary_task_jacobian * braking_baseline;
      *task_scaling_unscaled =
          primary_task_jacobian * nominal_velocity;
      const DecisionVector achieved = primary_task_jacobian * result;
      const DecisionVector task_direction =
          *task_scaling_unscaled - *task_scaling_baseline;
      const double direction_norm_squared = task_direction.squaredNorm();
      if (direction_norm_squared > 1.0e-12) {
        *task_scale = std::clamp(
            (achieved - *task_scaling_baseline).dot(task_direction) /
                direction_norm_squared,
            0.0, 1.0);
      }
      *task_scaling_residual = maxAbs(
          achieved - (*task_scaling_baseline +
                      *task_scale * task_direction));
    }
    if (!projection_valid) {
      ROS_ERROR_THROTTLE(
          1.0,
          "[VelocityIkShadow] dynamic supervisor became infeasible; "
          "latching emergency braking until a new session/generation");
      dynamic_supervisor_valid_ = false;
      dynamic_fault_latched_ = true;
      result = calculateEmergencyBrakingVelocity(request);
      *emergency_acceleration_used = true;
    }
    for (int i = 0; i < decisionDofCount(); ++i) {
      if (std::abs(result(i) - nominal_velocity(i)) > 1.0e-10) {
        (*boundary_brake)(i) = 1.0;
      }
      const double next_ddq =
          (result(i) - request.reference_dq(i)) / request.cycle_dt;
      if (std::abs(next_ddq) > acceleration_limit_(i) + 1.0e-8) {
        *emergency_acceleration_used = true;
      }
    }
    return result;
  }

  bool automaticHoldEnabled() const {
    return automatic_hold_enabled_ || !takeover_started_;
  }

  bool baseTrackingRequiresHold(double translation_error, double yaw_error) const {
    return automaticHoldEnabled() &&
        (translation_error > base_tracking_translation_limit_ ||
         yaw_error > base_tracking_yaw_limit_);
  }

  bool executionTrackingRequiresHold() const {
    // Arm servo lag is diagnostic only: fast reversals can exceed the old
    // 0.30-rad historical-envelope threshold without an actuator fault.
    // Under the post-startup no-auto-HOLD policy lower-body lag is also
    // diagnostic. Hard-domain, feedback, emergency-stop and owner lease checks
    // are independent and remain in force.
    return automaticHoldEnabled() &&
        lower_reference_tracking_error_ > lower_reference_tracking_hard_limit_;
  }

  bool prepareTargetForTracking(TargetSnapshot& world_target,const TargetSnapshot& target,
      double tick_start_ns,double target_age,double body_feedforward_age,
      bool contract_active,bool source_target_fresh) {
    const bool estimated_reference=continuous_reference_enabled_ && target.estimated_twists_valid;
    // Apply each channel's own finite hold/decay exactly once. This is
    // required by independent histories; all constants and reconstruction
    // horizons remain unchanged in F1.
    for(int arm=0;arm<kArmCount;++arm) {
      if (estimated_reference) continue; // The new-frame silence clock owns decay exactly once.
      auto& a=world_target.arm.at(arm);
      const double h=velocity_ik::channelDecay(target.hand_observation.at(arm),tick_start_ns,ff_hold_time_,ff_decay_time_constant_);
      const double e=velocity_ik::channelDecay(target.elbow_observation.at(arm),tick_start_ns,ff_hold_time_,ff_decay_time_constant_);
      a.hand_linear_ff*=h;a.hand_angular_ff*=h;a.elbow_linear_ff*=e;
    }

    // C2 changes only the prediction epoch to each channel content update.
    // Packet receive time remains the latency endpoint. C1 independent caps,
    // the lead, and all FF/hold/decay constants remain unchanged.
    bool reconstructed = false;
    const double reconstruction_age = (tick_start_ns-target.receive_monotonic_ns)*1.0e-9 + target_adoption_lead_;
    if (target_age>input_continuity_.config.grace) {
      if (!estimated_reference) for (auto& arm : world_target.arm) {
          arm.hand_linear_ff.setZero(); arm.hand_angular_ff.setZero(); arm.elbow_linear_ff.setZero();
        }
      world_target.body_intent.desired_torso_velocity.setZero();
    }
    const double ff_budget=acquisition_.blend()*input_continuity_.budget;
    for (auto& arm : world_target.arm) {
      // Estimated references keep one consistent p/v trajectory; acquisition
      // and stale-input authority still constrain the task in budget_twist.
      if (!estimated_reference) {
        arm.hand_linear_ff*=ff_budget; arm.hand_angular_ff*=ff_budget; arm.elbow_linear_ff*=ff_budget;
      }
    }
    world_target.body_intent.desired_torso_velocity*=ff_budget;
    if (orientation_reference_mode_ != "fixed_bandwidth" && host_target_reconstruction_ && !(continuous_reference_enabled_ && target.estimated_twists_valid) && target_age<=input_continuity_.config.grace &&
        contract_active && source_target_fresh &&
        std::isfinite(reconstruction_age) && reconstruction_age >= 0.) {
      for (int arm=0; arm<kArmCount; ++arm) {
        auto& a=world_target.arm.at(arm);
        const double hand_age=velocity_ik::channelPredictionAge(target.hand_observation.at(arm),tick_start_ns,target_adoption_lead_);
        const double elbow_age=velocity_ik::channelPredictionAge(target.elbow_observation.at(arm),tick_start_ns,target_adoption_lead_);
        a.hand_position += velocity_ik::boundedPredictionStep(a.hand_linear_ff, hand_age, .020);
        // Also bypass legacy/non-estimated host orientation reconstruction.
        if (orientation_reference_mode_ != "direct_task") {
          const Eigen::Vector3d r = velocity_ik::boundedPredictionStep(a.hand_angular_ff, hand_age, .060);
          if (r.norm()>1e-12) a.hand_orientation = Eigen::Quaterniond(Eigen::AngleAxisd(r.norm(),r.normalized()))*a.hand_orientation;
        }
        if (a.elbow_valid) a.elbow_position += velocity_ik::boundedPredictionStep(a.elbow_linear_ff, elbow_age, .020);
      }
      const double body_decay = body_feedforward_age <= ff_hold_time_ ? 1.0 :
          std::exp(-(body_feedforward_age-ff_hold_time_)/std::max(ff_decay_time_constant_,1e-6));
      world_target.body_intent.desired_torso_velocity *= body_decay;
      world_target.body_intent.desired_torso_feature += velocity_ik::boundedBodyPredictionStep(
          world_target.body_intent.desired_torso_velocity,
          std::max(0.,(tick_start_ns-target.body_observation_host_ns)*1e-9)+target_adoption_lead_);
      reconstructed = true;
    }

    return reconstructed;
  }

  void shapeContinuousTarget(TargetSnapshot& world_target,const TargetSnapshot& target,
      double tick_start_ns,double cycle_dt) {
      if(continuous_reference_enabled_ && (target.estimated_twists_valid || orientation_reference_mode_=="fixed_bandwidth")) {
        // Latency alignment is distinct from silence since a new observation.
        // Normal 72Hz frames must not repeatedly brake a 500Hz reference.
        for(int arm=0;arm<kArmCount;++arm) {
          auto& a=world_target.arm[arm];
          auto& prediction=hand_prediction_[arm];
          prediction.update(target.hand_observation[arm].update_host_ns,
              target.receive_monotonic_ns,target.hand_observation[arm].source_time,
              tick_start_ns,frame_prediction_settings_);
          const double prediction_rate=prediction.rate,age=prediction.age;
          hand_innovation_audit_[arm].update(prediction,frame_prediction_settings_,tick_start_ns,
              a.hand_position,a.hand_orientation,a.hand_linear_ff,a.hand_angular_ff);
          if(orientation_reference_mode_=="fixed_bandwidth") {
            velocity_ik::stepFixedVector(hand_reference_[arm],a.hand_position,cycle_dt,hand_position_cutoff_hz_);
            velocity_ik::stepFixedRotation(hand_rotation_reference_[arm],a.hand_orientation,cycle_dt,hand_orientation_cutoff_hz_);
            a.hand_position=hand_reference_[arm].p;
            a.hand_linear_ff=hand_reference_[arm].v;
            a.hand_orientation=hand_rotation_reference_[arm].q;
            a.hand_angular_ff=hand_rotation_reference_[arm].v;
          } else {
          Eigen::Vector3d p=a.hand_position+age*a.hand_linear_ff;
          hand_reference_[arm].stepAdaptive(p,prediction_rate*a.hand_linear_ff,cycle_dt,
              hand_reference_frequency_,adaptive_reference_enabled_,adaptive_reference_settings_);
          a.hand_position=hand_reference_[arm].p;a.hand_linear_ff=hand_reference_[arm].v;
          if (orientation_reference_mode_ == "direct_task") {
            // Keep the mapped/session-aligned observation. The existing task
            // adds K_R*Log(R_obs*R(q_k)^T) once. No integrated R_ref or age lead.
            a.hand_angular_ff *= prediction_rate;
          } else {
            Eigen::Quaterniond q=a.hand_orientation;
            const Eigen::Vector3d r=age*a.hand_angular_ff;
            if(r.norm()>1e-12) q=Eigen::Quaterniond(Eigen::AngleAxisd(r.norm(),r.normalized()))*q;
            hand_rotation_reference_[arm].stepAdaptive(q,prediction_rate*a.hand_angular_ff,cycle_dt,
                hand_orientation_reference_frequency_,adaptive_reference_enabled_,adaptive_reference_settings_);
            a.hand_orientation=hand_rotation_reference_[arm].q;a.hand_angular_ff=hand_rotation_reference_[arm].v;
          }
          }
          if(a.elbow_valid) {
            auto& ep=elbow_prediction_[arm];
            ep.update(target.elbow_observation[arm].update_host_ns,
                target.receive_monotonic_ns,target.elbow_observation[arm].source_time,
                tick_start_ns,frame_prediction_settings_);
            elbow_innovation_audit_[arm].update(ep,frame_prediction_settings_,tick_start_ns,
                a.elbow_position,Eigen::Quaterniond::Identity(),a.elbow_linear_ff,Eigen::Vector3d::Zero());
            if(orientation_reference_mode_=="fixed_bandwidth")
              velocity_ik::stepFixedVector(elbow_reference_[arm],a.elbow_position,cycle_dt,hand_position_cutoff_hz_);
            else elbow_reference_[arm].stepAdaptive(a.elbow_position+ep.age*a.elbow_linear_ff,
                ep.rate*a.elbow_linear_ff,cycle_dt,hand_reference_frequency_,
                adaptive_reference_enabled_,adaptive_reference_settings_);
            a.elbow_position=elbow_reference_[arm].p;a.elbow_linear_ff=elbow_reference_[arm].v;
          } else {
            elbow_reference_[arm].valid=false;
            elbow_prediction_[arm].reset();
            elbow_innovation_audit_[arm]=velocity_ik::ReferenceInnovationAudit{};
          }
        }
        if(world_target.body_intent.torso_valid) {
          auto& body=world_target.body_intent;
          if(!torso_reference_.valid)
            torso_reference_.reset(torsoFeatureInContext(reference_q_,plant_context_.get(),&q_full_).tail<3>());
          Eigen::Vector3d p=body.desired_torso_feature.tail<3>();
          p.z()=torso_reference_.p.z()+shortestAngularDelta(torso_reference_.p.z(),p.z());
          // Deliberately low-bandwidth execution, NOT the chest measurement
          // used for hand geometry. Raw body FF must not bypass this shaper.
          torso_reference_.step(p,Eigen::Vector3d::Zero(),cycle_dt,torso_reference_frequency_);
          body.desired_torso_feature.tail<3>()=torso_reference_.p;
          body.desired_torso_velocity.tail<3>()=torso_reference_.v;
        }
      }
  }

  void acceptExecutionFeedback(const wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr& msg) {
          if (!msg || msg->producer_instance != execution_instance_ ||
              msg->q.size() != static_cast<std::size_t>(decisionDofCount()) ||
              msg->dq.size() != msg->q.size() || msg->ddq.size() != msg->q.size()) return;
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (execution_feedback_freshness_.accept(msg->execution_clock_domain,
                  msg->execution_owner_instance, 0, 0, msg->owner_feedback_seq,
                  msg->generated_monotonic_ns, velocity_ik_execution::executionSteadyNs(),
                  msg->header.stamp.toNSec(), ros::Time::now().toNSec(), 50000000) !=
              velocity_ik_execution::ClockedReferenceFreshness::Result::NewSample) return;
          execution_feedback_ = msg;
          execution_feedback_receive_ns_ = monotonicNs();

  }

  void publishOnce() {
    velocity_ik::runtime_probe::point(10);
    if(reference_transport_.shm()){
      wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr feedback;
      if(reference_transport_.readOwner(feedback))acceptExecutionFeedback(feedback);
    }
    velocity_ik::runtime_probe::point(11);
    if (terminal_stop_requested_.load(std::memory_order_acquire)) return;
    const double tick_start_ns=monotonicNs();
    velocity_ik::runtime_probe::point(12);
    last_task_us_=last_bounds_us_=last_qp_finish_ns_=0;
    const ros::WallTime now = ros::WallTime::now();
    const double cycle_dt = dt_;
    double wall_cycle_dt = dt_;
    if (!last_cycle_time_.isZero()) {
      const double measured_cycle_dt = (now - last_cycle_time_).toSec();
      if (std::isfinite(measured_cycle_dt) && measured_cycle_dt > 0.0) {
        wall_cycle_dt = measured_cycle_dt;
      }
    }
    last_cycle_time_ = now;
    TargetSnapshot target;
    velocity_ik::FixedPrepareTransaction fixed_prepare_transaction;
    DecisionVector measured_q;
    DecisionVector measured_dq;
    bool robot_valid = false;
    bool base_valid = false;
    bool base_coordinate_jump = false;
    uint64_t base_coordinate_epoch = 0;
    bool mode_known = false;
    int current_mode = 0;
    int target_mode = 0;
    bool mode_transitioning = false;
    bool target_source_active = false;
    double robot_measurement_age;
    double base_measurement_age;
    double sensor_stamp=0,sensor_receive_ns=0;uint32_t sensor_seq=0;
    velocity_ik::runtime_probe::point(20);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      velocity_ik::runtime_probe::point(21);
      target = target_;
      fixed_prepare_transaction = fixed_prepare_transaction_;
      measured_q = measured_q_;
      measured_dq = measured_dq_;
      robot_valid = robot_state_valid_;
      // Fixed-base operation has no mobile-base state to qualify. Its three
      // base decision variables are hard equality-constrained to zero, so an
      // absent /odom publisher must not block the 18 actuated DoFs.
      base_valid = !enable_base_motion_ || base_state_valid_;
      base_coordinate_jump = base_coordinate_jump_detected_;
      base_coordinate_epoch = base_coordinate_epoch_;
      mode_known = arm_mode_known_;
      current_mode = current_arm_mode_;
      target_mode = target_arm_mode_;
      mode_transitioning = arm_mode_transitioning_;
      target_source_active = target_source_active_;
      const auto measurement_now_ns = static_cast<int64_t>(monotonicNs());
      robot_measurement_age = robot_measurement_freshness_.age(measurement_now_ns);
      base_measurement_age = enable_base_motion_
          ? base_measurement_freshness_.age(measurement_now_ns)
          : 0.0;
      sensor_stamp=sensor_source_stamp_.toSec();sensor_seq=sensor_source_seq_;sensor_receive_ns=sensor_receive_ns_;
    }

    velocity_ik::runtime_probe::point(22);
    wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr execution_feedback;
    velocity_ik::runtime_probe::point(23);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      velocity_ik::runtime_probe::point(24);
      execution_feedback = execution_feedback_;
    }
    velocity_ik::runtime_probe::point(25);
    if (enable_takeover_ && execution_feedback &&
        execution_feedback->owner_feedback_seq != execution_feedback_applied_seq_ &&
        velocity_ik_execution::executionSteadyNs() - execution_feedback->generated_monotonic_ns <= 50000000) {
      execution_feedback_applied_seq_ = execution_feedback->owner_feedback_seq;
      const auto& f = *execution_feedback;
      const DecisionVector q = Eigen::Map<const DecisionVector>(f.q.data(), f.q.size());
      const DecisionVector v = Eigen::Map<const DecisionVector>(f.dq.data(), f.dq.size());
      const DecisionVector a = Eigen::Map<const DecisionVector>(f.ddq.data(), f.ddq.size());
      const bool stopped = q.allFinite() && v.allFinite() && a.allFinite() &&
          maxAbs(v) == 0.0 && maxAbs(a) == 0.0 &&
          (q.array() >= position_lower_.array() - 1e-10).all() &&
          (q.array() <= position_upper_.array() + 1e-10).all();
      const bool matches = stopped && reference_initialized_ &&
          maxAbs(reference_dq_) == 0.0 && maxAbs(reference_ddq_) == 0.0 &&
          maxAbs(q - reference_q_) <= 1e-10;
      using Lease = velocity_ik_execution::ExecutionProducerLease;
      Lease::Feedback feedback;
      feedback.producer=f.producer_instance;feedback.owner=f.execution_owner_instance;
      feedback.epoch=f.execution_epoch;feedback.anchor=f.recovery_anchor_seq;
      feedback.generated_ns=f.generated_monotonic_ns;feedback.lease_until_ns=f.producer_lease_deadline_ns;
      feedback.valid=f.valid;
      feedback.phase=f.authority_state == wheel_arm_abs_vel::WholeBodyIkReference::AUTHORITY_ACTIVE
          ? Lease::Phase::Active : (f.authority_state == wheel_arm_abs_vel::WholeBodyIkReference::AUTHORITY_HOLD
          ? Lease::Phase::Hold : Lease::Phase::Braking);
      const auto result = execution_lease_.observe(feedback, stopped, matches);
      if (result == Lease::Result::InstallAnchor) {
        reference_q_=q;reference_dq_=v;reference_ddq_=a;
        reference_initialized_=true;reference_history_.clear();
      } else if (result == Lease::Result::OwnerRestart) {
        ROS_ERROR("[VelocityIk] execution owner instance changed; motion inhibited; coordinated standstill restart required");
      }
    }
    execution_epoch_=execution_lease_.epoch();
    execution_anchor_seq_=execution_lease_.anchor();
    execution_waiting_=enable_takeover_ && (execution_lease_.initialPending() ||
        execution_lease_.waiting() || execution_lease_.restartRequired());

    const double target_read_ns=monotonicNs();
    velocity_ik::runtime_probe::point(26);
    if(velocity_ik::runtime_probe::current()) velocity_ik::runtime_probe::current()->identity(target.session_id,target.generation,reference_sequence_);
    const double robot_age = robot_valid ? robot_measurement_age : std::numeric_limits<double>::infinity();
    const double base_state_age = base_valid
        ? base_measurement_age
        : std::numeric_limits<double>::infinity();
    double target_age = target.observation_ns[2]>0.
        ? (target_read_ns-target.observation_ns[2])*1e-9
        : std::numeric_limits<double>::infinity();
    for (int arm=0;arm<kArmCount;++arm) if (armIsControlled(arm))
      target_age=std::max(target_age,target.observation_ns[arm]>0.
          ? (target_read_ns-target.observation_ns[arm])*1e-9
          : std::numeric_limits<double>::infinity());
    const double feedforward_age = target.feedforward_sample_valid
        ? (now - target.feedforward_sample_receive_time).toSec()
        : std::numeric_limits<double>::infinity();
    const double body_feedforward_age = target.body_feedforward_sample_valid
        ? (now - target.body_feedforward_sample_receive_time).toSec()
        : std::numeric_limits<double>::infinity();
    const bool robot_fresh = robot_valid && robot_age <= robot_state_timeout_ &&
        base_valid && base_state_age <= base_state_timeout_;
    const bool source_target_fresh = target.valid && target_source_active && target_age >= 0.0 &&
                                     target_age <= target_timeout_;
    // Disable automatic pauses only after the first startup takeover.
    // Preserve actual age and cached pose; do not extrapolate stale velocity.
    const bool automatic_hold_enabled = automaticHoldEnabled();
    const bool source_target_admissible = target.valid && target_source_active &&
        input_continuity_.admitsAge(target_age, automatic_hold_enabled);
    const bool mode_ok = !require_external_arm_mode_ ||
                         (mode_known && current_mode == required_arm_mode_ &&
                          target_mode == required_arm_mode_ && !mode_transitioning);
    const bool contract_active =
        target.control_state == wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_ACTIVE;
    const bool contract_prepare =
        target.control_state ==
        wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_PREPARE;
    const bool stopped_for_recovery = robot_fresh && reference_initialized_ &&
        maxAbs(reference_dq_)<=hold_velocity_threshold_ &&
        maxAbs(measured_dq)<=preparation_velocity_tolerance_;
    if (force_confirmation_.exchange(false)) input_continuity_.confirmation_required=true;
    const bool confirm=confirm_requested_.exchange(false);
    if (!contract_active) input_continuity_.suspend();
    else input_continuity_.update(target_read_ns*1e-9,target_age,target.observation_sequence,
        target.valid && target_source_active,
        confirm && mode_ok && !dynamic_fault_latched_ && !reference_tracking_fault_latched_,
        stopped_for_recovery, automatic_hold_enabled);
    confirm_ready_=input_continuity_.confirmation_required && input_continuity_.stable &&
        stopped_for_recovery && contract_active && mode_ok &&
        !dynamic_fault_latched_ && !reference_tracking_fault_latched_;

    const bool new_coordinate_contract = source_target_fresh &&
        (!session_anchor_.valid ||
         session_anchor_.session_id != target.session_id ||
         session_anchor_.generation != target.generation);
    if (new_coordinate_contract && robot_fresh) {
      acquisition_.reset();
      governor_risk_.fill(0.);
      // X+A changes the input transaction, not the owner of the executing
      // trajectory. Under retained healthy mode-2 ownership, preserve q/dq/ddq
      // even across PREPARE/ARMED transitions. Recapturing measured q here
      // injected the arm's static tracking error as a command step on resume.
      const bool retain_owned_reference = reference_initialized_ && enable_takeover_ &&
          mode_ok && session_anchor_.valid &&
          session_anchor_.base_coordinate_epoch == base_coordinate_epoch &&
          !dynamic_fault_latched_ && !reference_tracking_fault_latched_;
      const DecisionVector& base_anchor_q = retain_owned_reference ? reference_q_ : measured_q;
      session_anchor_.translation = Eigen::Vector2d(
          base_anchor_q(baseDecisionIndex(0)),
          base_anchor_q(baseDecisionIndex(1)));
      session_anchor_.yaw = base_anchor_q(baseDecisionIndex(2));
      session_anchor_.session_id = target.session_id;
      session_anchor_.generation = target.generation;
      session_anchor_.base_coordinate_epoch = base_coordinate_epoch;
      session_anchor_.valid = true;
      session_body_anchor_ = SessionBodyAnchor{};
      last_body_intent_ = BodyIntent{};
      captureSessionBodyAnchor(target, measured_q, false);
      if (!captureSessionTargetAlignment(
              target, measured_q, base_coordinate_epoch)) {
        session_anchor_.valid = false;
        ROS_ERROR_THROTTLE(
            1.0,
          "[VelocityIkShadow] failed to initialize the direct Cartesian session alignment");
      }
      base_coordinate_jump = false;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        base_coordinate_jump_detected_ = false;
      }
      reference_initialized_ = retain_owned_reference;
      fallback_active_ = true;
      tracking_ramp_scale_ = 0.0;
      active_lower_body_recruitment_ = 0.0;
      active_reach_assist_demand_ = 0.0;
      active_lower_joint_recruitment_.fill(0.0);
      lower_joint_stage_active_.fill(false);
      lower_joint_admission_elapsed_.fill(0.0);
      lower_joint_release_elapsed_.fill(0.0);
      cascade_stage_ = 0;
      active_base_recruitment_ = 0.0;
      base_admission_elapsed_ = 0.0;
      base_release_elapsed_ = 0.0;
      base_admission_active_ = false;
      preparation_active_ = false;
      preparation_complete_ = false;
      preparation_initial_error_ = 0.0;
      preparation_progress_ = 0.0;
      preparation_settle_cycles_ = 0;
      dynamic_supervisor_valid_ = true;
      dynamic_fault_latched_ = false;
      reference_tracking_soft_limited_ = false;
      reference_tracking_fault_latched_ = false;
      reference_tracking_fault_elapsed_ = 0.0;
      reference_tracking_recovery_elapsed_ = 0.0;
      arm_reference_tracking_error_ = 0.0;
      lower_reference_tracking_error_ = 0.0;
      arm_reference_tracking_error_by_arm_.fill(0.0);
      bool natural_hang = true;
      for (int arm = 0; arm < kArmCount; ++arm) {
        for (int joint = 0; joint < kJointsPerArm; ++joint) {
          natural_hang = natural_hang &&
              std::abs(measured_q(armDecisionIndex(arm, joint))) <=
                  natural_hang_arm_threshold_;
        }
      }
      startup_bootstrap_active_ = enable_natural_hang_bootstrap_ &&
          target.control_state ==
              wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_ARMED &&
          natural_hang;
      if (startup_bootstrap_active_) {
        // The startup bootstrap exists only to move the arms away from the
        // vertical natural-hang singularity.  Do not pull a healthy lower
        // body all the way to the fixed safe centre: that used its deliberately
        // small ACTIVE velocity budgets and delayed the first usable Cartesian
        // command by more than eight seconds.  A lower joint already inside
        // its operating envelope stays where it is; a boundary start moves
        // only to the nearest envelope edge.  Explicit CONTROL_PREPARE still
        // returns all controlled joints to the configured ready posture.
        startup_preparation_position_ = preparation_position_;
        for (int joint = 0; joint < kLowerBodyDof; ++joint) {
          const int index = lowerBodyDecisionIndex(joint);
          startup_preparation_position_(index) = std::clamp(
              measured_q(index), joint_safe_lower_(index),
              joint_safe_upper_(index));
          session_posture_position_(index) =
              startup_preparation_position_(index);
        }
      }
      ROS_INFO_STREAM(
          "[VelocityIkShadow] anchored session " << target.session_id
          << " generation " << target.generation << " at ["
          << session_anchor_.translation.transpose() << ","
          << session_anchor_.yaw << "] in " << world_frame_name_
          << "; direct Cartesian alignment="
          << (session_target_alignment_.valid ? "ready" : "invalid")
          << ", natural-hang bootstrap="
          << (startup_bootstrap_active_ ? "pending" : "not-required"));
    }
    if (session_anchor_.valid && target.body.torsoValid() &&
        target.session_id == session_anchor_.session_id &&
        target.generation == session_anchor_.generation &&
        (!session_body_anchor_.valid ||
         target.body.coordinate_epoch !=
             session_body_anchor_.coordinate_epoch)) {
      const bool preserve_body_target = session_body_anchor_.valid;
      if (captureSessionBodyAnchor(
              target, measured_q, preserve_body_target)) {
        ROS_WARN_STREAM(
            "[VelocityIkShadow] anchored body-intent epoch "
            << target.body.coordinate_epoch
            << (preserve_body_target
                    ? " while preserving the resolved body target"
                    : " after body tracking became valid"));
      }
    }
    BodyIntent body_intent = calculateBodyIntent(target);
    if (body_intent.torso_valid) {
      last_body_intent_ = body_intent;
    }
    const TargetSnapshot raw_world_target =
        resolveSessionTargetInWorld(
            target, base_coordinate_epoch, body_intent);
    const TargetSnapshot aligned_torso_target =
        applySessionTargetAlignment(target, body_intent);
    const TargetSnapshot aligned_world_target =
        torsoRelativeTargetInWorld(aligned_torso_target, measured_q);
    const bool target_fresh = source_target_admissible &&
        raw_world_target.valid && aligned_torso_target.valid &&
        aligned_world_target.valid;

    ShadowState state = ShadowState::kWaitingForInput;
    bool reference_valid = reference_initialized_;
    bool raw_solver_valid = false;
    bool candidate_accepted = false;
    bool emergency_acceleration_used = false;
    int qp_return_code = -1003;
    double solve_time_us = 0.0;
    double candidate_age = -1.0;
    uint64_t solver_base_seq = 0;
    uint8_t candidate_decision =
        wheel_arm_abs_vel::VelocityIkShadow::CANDIDATE_UNAVAILABLE;
    uint64_t candidate_session_id = 0;
    uint64_t candidate_generation = 0;
    uint64_t candidate_target_seq = 0;
    uint32_t candidate_reference_lag = 0;
    uint64_t candidate_target_lag = 0;
    std::array<double, kArmCount> sigma_min{0.0, 0.0};
    std::array<double, kArmCount> translation_sigma{0.0, 0.0};
    std::array<bool, kArmCount> singularity_escape_constraint_active{
        false, false};
    bool singularity_escape_constraint_fallback = false;
    std::array<double, kArmCount> primary_regularization{0.0, 0.0};
    std::array<double, kArmCount> orientation_task_scale{1.0, 1.0};
    std::array<double, kArmCount> position_task_weight_scale{1.0, 1.0};
    std::array<ArmPressure, kArmCount> arm_pressure{};
    Recruitment recruitment_target{};
    std::array<double, kLowerBodyDof> lower_joint_recruitment_target{
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kLowerBodyDof> lower_joint_directional_effectiveness{
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kLowerBodyDof> solver_lower_joint_recruitment{
        0.0, 0.0, 0.0, 0.0};
    double geometric_reach_demand = 0.0;
    double solver_lower_body_recruitment = 0.0;
    double solver_base_recruitment = 0.0;
    double arm_only_task_residual = 0.0;
    double arm_lower_task_residual = 0.0;
    double lower_body_saturation = 0.0;
    double primary_task_scale = 1.0;
    std::array<double, kArmCount> translation_task_scale{1.0, 1.0};
    int primary_task_rank = 0;
    int nullspace_dimension = 0;
    double nullspace_residual = 0.0;
    double primary_task_leakage = 0.0;
    double task_scale = 0.0;
    double task_scaling_residual = 0.0;
    DecisionVector task_scaling_baseline;
    DecisionVector task_scaling_unscaled;
    bool task_scaling_qp_valid = false;
    double torso_viability_relaxation = 0.0;
    double torso_task_scale = 0.0;
    double base_intent_task_scale = 0.0;
    double torso_task_error = 0.0;
    double base_intent_error = 0.0;
    bool root_translation_intent_active = false;
    int secondary_return_code = -1100;
    std::array<double, 6> task_error{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    DecisionVector task_dq = DecisionVector::Zero(decisionDofCount());
    DecisionVector primary_dq = DecisionVector::Zero(decisionDofCount());
    DecisionVector secondary_dq = DecisionVector::Zero(decisionDofCount());
    DecisionMatrix primary_task_jacobian;
    DecisionVector primary_task_velocity;
    DecisionVector jerk_slack = DecisionVector::Zero(decisionDofCount());
    DecisionVector boundary_brake = DecisionVector::Zero(decisionDofCount());
    DecisionVector operational_boundary_brake =
        DecisionVector::Zero(decisionDofCount());

    // Before the first trackable target arrives, the reference is only a
    // non-valid mirror of the latest measured state.  This prevents startup
    // motion from opening a q offset while the VR stream is not ready.  Once
    // initialized, reference_initialized_ is never cleared: later target
    // dropouts therefore brake the internal trajectory instead of snapping it
    // back to the measured robot state.
    const bool contract_prepared =
        target.control_state ==
            wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_ARMED ||
        contract_active || contract_prepare;
    // First acquisition uses unique sensor samples, not 50 all-joint raw-speed
    // comparisons. PREPARE completion and recovery remain unchanged.
    if (!takeover_started_) {
      velocity_ik::StartupReadiness::Values q{}, v{};
      for (int i=0;i<decisionDofCount();++i) {q[i]=measured_q(i);v[i]=measured_dq(i);}
      startup_readiness_.update(sensor_stamp,q,v,robot_fresh && base_valid);
      if (output_sequence_%5==0) {
        const auto& s=startup_readiness_;
        publishStdVector(startup_readiness_pub_, {1.,ros::Time::now().toSec(),
            double(s.ready),s.span,double(s.joint),s.position_range,s.speed_rms,
            s.raw_max,double(s.reason)});
      }
      if (mode_ok && (contract_active || contract_prepare) && !startup_readiness_.ready)
        ROS_INFO_THROTTLE(1., "[StartupReadiness] waiting: reason=%d joint=%d range=%.6f rms=%.6f window=%.3f",
            startup_readiness_.reason,startup_readiness_.joint,startup_readiness_.position_range,
            startup_readiness_.speed_rms,startup_readiness_.span);
    }
    if (!reference_initialized_ && robot_fresh && target_fresh && mode_ok &&
        (contract_active || contract_prepare) &&
        startup_readiness_.ready) {
      resetReference(measured_q, measured_dq);
      takeover_started_ = true;
      reference_valid = true;
    }
    if (!takeover_started_ && reference_initialized_ && execution_waiting_ &&
        execution_anchor_seq_ &&
        startup_readiness_.ready) {
      // A restarted producer receives the stopped owner state itself.
      // It must not rebase that handoff to measured q.
      takeover_started_ = true;
    }
    if (!reference_initialized_) {
      if (robot_valid && base_valid) {
        reference_q_ = measured_q.cwiseMax(position_lower_).cwiseMin(position_upper_);
        reference_dq_ = measured_dq.cwiseMax(-velocity_limit_).cwiseMin(velocity_limit_);
        lockBaseVelocity(&reference_dq_);
        reference_ddq_.setZero();
      } else {
        reference_q_.setZero();
        reference_dq_.setZero();
        reference_ddq_.setZero();
      }
    }
    if (reference_initialized_ && robot_fresh && !enable_takeover_) {
      // Step 4 still has no command authority.  Base position is exogenous
      // odometry state, while dq is now an explicitly diagnostic, continuously
      // recruited suggestion.  Formal takeover replaces this per-cycle
      // position synchronization with one 21-D reference integration path.
      synchronizeExogenousBasePosition(measured_q, &reference_q_);
    }

    arm_reference_tracking_error_ = 0.0;
    lower_reference_tracking_error_ = 0.0;
    arm_reference_tracking_error_by_arm_.fill(0.0);
    reference_tracking_scale_ = 1.0;
    const bool execution_watchdog_enabled =
        reference_initialized_ && robot_fresh && enable_takeover_ &&
        !execution_waiting_ && contract_active && !startup_bootstrap_active_;
    const bool tracking_history_ready = referenceHistoryReady(now);
    uint32_t numeric_recovery_mask = 64; // not evaluated by the watchdog this tick
    double diagnostic_recovery_good_s = 0.0;
    if (execution_watchdog_enabled && tracking_history_ready) {
      // Different arm joints have different transport/servo delay, and the
      // delay itself changes under load. Compare measured q against the
      // actual reference samples published over the admissible delay window.
      // The previous quadratic extrapolation of current q/dq/ddq generated
      // fictitious errors during fast reversals. Arm errors remain telemetry;
      // only lower-body execution error participates in this hold watchdog.
      for (int arm = 0; arm < kArmCount; ++arm) {
        for (int joint = 0; joint < kJointsPerArm; ++joint) {
          const int index = armDecisionIndex(arm, joint);
          const double tracking_error = minimumDelayedReferenceError(
              now, index, measured_q(index));
          arm_reference_tracking_error_by_arm_.at(arm) = std::max(
              arm_reference_tracking_error_by_arm_.at(arm), tracking_error);
          arm_reference_tracking_error_ = std::max(
              arm_reference_tracking_error_, tracking_error);
        }
      }
      for (int joint = 0; joint < kLowerBodyDof; ++joint) {
          const int index = lowerBodyDecisionIndex(joint);
          lower_reference_tracking_error_ = std::max(
              lower_reference_tracking_error_,
              minimumDelayedReferenceError(now, index, measured_q(index)));
      }
      // Normal servo phase error is health telemetry, never Cartesian
      // backpressure. Only a persistent hard execution fault can brake the
      // session; one joint cannot continuously slow both hands.
      reference_tracking_scale_ = 1.0;
      reference_tracking_soft_limited_ =
          arm_reference_tracking_error_ >
              orientation_tracking_full_error_ ||
          lower_reference_tracking_error_ >
              lower_reference_tracking_qp_limit_;
      const bool hard_tracking_error = executionTrackingRequiresHold();
      reference_tracking_fault_elapsed_ = hard_tracking_error
          ? reference_tracking_fault_elapsed_ + cycle_dt
          : 0.0;
      if (hard_tracking_error && reference_tracking_fault_elapsed_ >=
          reference_tracking_fault_dwell_) {
        if (!reference_tracking_fault_latched_) {
          // Capture the actual minimum-error history sample, not the current
          // reference or an unrelated topic row. Formatting runs off the tick.
          for (int index = 0; index < decisionDofCount(); ++index) {
            const int sensor_index = decision_sensor_index_.at(index);
            if (sensor_index < 0 || sensor_index >= sensor_arm_start_index_) continue;
            const double limit = lower_reference_tracking_hard_limit_;
            double closest_q, closest_wall_s;
            const double qm = measured_q(index);
            const double error = minimumDelayedReferenceError(now,index,qm,&closest_q,&closest_wall_s);
            if (!(error > limit)) continue;
            const auto name = dof_layout_.decisionSpec(index).name;
            const double elapsed = reference_tracking_fault_elapsed_;
            const double dwell = reference_tracking_fault_dwell_;
            const double wall_s = now.toSec();
            diagnostics_.submit([name,index,sensor_index,qm,closest_q,closest_wall_s,error,limit,elapsed,dwell,wall_s]{
              ROS_WARN_STREAM(std::setprecision(17)<<"[HoldReason] tracking joint="<<name<<" reference_index="<<index
                  <<" sensor/cmd/wbc_index="<<sensor_index<<" q_meas="<<qm
                  <<" closest_history_q="<<closest_q<<" history_wall_s="<<closest_wall_s
                  <<" event_wall_s="<<wall_s<<" error="<<error<<" limit="<<limit
                  <<" aggregate_hard_elapsed="<<elapsed<<" dwell="<<dwell);
            });
          }
          ROS_ERROR_STREAM(
              "[VelocityIkShadow] lower-body measured/reference tracking fault latched: "
              "arm=" << arm_reference_tracking_error_
              << " rad, lower=" << lower_reference_tracking_error_
              << " rad; braking until the plant catches the reference or a "
                 "new session/generation is requested");
        }
        reference_tracking_fault_latched_ = true;
      }
      if (reference_tracking_fault_latched_) {
        numeric_recovery_mask = velocity_ik::recoveryReasons(
            true, // arm lag is diagnostic, including while recovering a lower-body fault
            lower_reference_tracking_error_ <= reference_tracking_recovery_lower_limit_,
            maxAbs(reference_dq_) <= hold_velocity_threshold_,
            maxAbs(measured_dq) <= preparation_velocity_tolerance_,true,true);
        const bool recovered =
            lower_reference_tracking_error_ <=
                reference_tracking_recovery_lower_limit_ &&
            maxAbs(reference_dq_) <= hold_velocity_threshold_ &&
            maxAbs(measured_dq) <= preparation_velocity_tolerance_;
        reference_tracking_recovery_elapsed_ = recovered
            ? reference_tracking_recovery_elapsed_ + cycle_dt
            : 0.0;
        diagnostic_recovery_good_s = reference_tracking_recovery_elapsed_;
        if (reference_tracking_recovery_elapsed_ >=
            reference_tracking_recovery_dwell_) {
          // The fault source is ordinary reference tracking saturation, not
          // an actuator/communication fault bit.  Once the measured plant has
          // caught the dynamically braked command for a sustained interval,
          // retain the stopped command and resume the same valid session.
          // Keep the stopped reference state; rebasing q to feedback
          // would introduce an unnecessary position discontinuity.
          reference_tracking_fault_latched_ = false;
          reference_tracking_fault_elapsed_ = 0.0;
          reference_tracking_recovery_elapsed_ = 0.0;
          ROS_WARN(
              "[VelocityIkShadow] measured/reference tracking recovered; "
              "retained the stopped reference and resumed the current session");
        }
      } else {
        reference_tracking_recovery_elapsed_ = 0.0;
      }
    } else {
      reference_tracking_scale_ = 1.0;
      reference_tracking_soft_limited_ = false;
      reference_tracking_fault_elapsed_ = 0.0;
      reference_tracking_recovery_elapsed_ = 0.0;
    }

    // Legacy publishers retain the direct target path. Estimated packets use
    // continuous references below, without a Cartesian speed cap or FIFO.
    TargetSnapshot world_target = aligned_torso_target;
    const double reconstruction_age = (tick_start_ns-target.receive_monotonic_ns)*1.0e-9 + target_adoption_lead_;
    const bool reconstructed = prepareTargetForTracking(world_target,target,tick_start_ns,
        target_age,body_feedforward_age,contract_active,source_target_fresh);

    const Eigen::Vector2d reference_base_translation(
        reference_q_(baseDecisionIndex(0)),
        reference_q_(baseDecisionIndex(1)));
    const Eigen::Vector2d measured_base_translation(
        measured_q(baseDecisionIndex(0)), measured_q(baseDecisionIndex(1)));
    const double base_translation_tracking_error =
        (reference_base_translation - measured_base_translation).norm();
    const double base_yaw_tracking_error = std::abs(shortestAngularDelta(
        measured_q(baseDecisionIndex(2)),
        reference_q_(baseDecisionIndex(2))));
    const bool base_reference_tracking_ok = !enable_takeover_ ||
        !reference_initialized_ ||
        !baseTrackingRequiresHold(base_translation_tracking_error, base_yaw_tracking_error);
    if (!base_reference_tracking_ok) {
      ROS_ERROR_THROTTLE(
          1.0,
          "[VelocityIkShadow] base reference tracking exceeded takeover limits; braking all 21 DoFs");
    }
    const bool startup_preparing = reference_initialized_ && robot_fresh &&
        target_fresh && mode_ok && contract_active &&
        startup_bootstrap_active_ && base_reference_tracking_ok;
    if (enable_takeover_ && !execution_lease_.canAdvance(
            velocity_ik_execution::executionSteadyNs(), static_cast<int64_t>(cycle_dt * 1e9))) {
      execution_waiting_=true;execution_anchor_seq_=execution_lease_.anchor();
    }
    const bool tracking = !execution_waiting_ && takeover_started_ && reference_initialized_ && robot_fresh &&
        target_fresh && mode_ok && contract_active &&
        input_continuity_.canTrack() &&
        base_reference_tracking_ok &&
        !startup_bootstrap_active_ &&
        !reference_tracking_fault_latched_ && !dynamic_fault_latched_;
    const std::array<uint64_t,4> reference_key{{target.session_id,target.generation,target.body.coordinate_epoch,static_cast<uint64_t>(target.source)}};
    if (!tracking || !target.estimated_twists_valid || reference_key!=continuous_reference_key_) {
      continuous_reference_key_=reference_key;
      for(auto& r:hand_reference_) r.valid=false;
      for(auto& r:hand_rotation_reference_) r.valid=false;
      for(auto& r:elbow_reference_) r.valid=false;
      for(auto& p:hand_prediction_) p.reset();
      for(auto& p:elbow_prediction_) p.reset();
      hand_innovation_audit_.fill(velocity_ik::ReferenceInnovationAudit{});
      elbow_innovation_audit_.fill(velocity_ik::ReferenceInnovationAudit{});
      torso_reference_.valid=false;
    }
    if (!tracking) {
      acquisition_.reset(); governor_risk_.fill(0.);
    }
    const bool fixed_prepare_fresh = contract_prepare && session_anchor_.valid &&
        session_anchor_.session_id == target.session_id &&
        session_anchor_.generation == target.generation &&
        session_anchor_.base_coordinate_epoch == base_coordinate_epoch &&
        fixed_prepare_transaction.fresh(target.session_id, target.generation,
                                        target.body.coordinate_epoch, target_read_ns);
    const bool preparing = !execution_waiting_ && takeover_started_ && reference_initialized_ && robot_fresh &&
        mode_ok && (fixed_prepare_fresh || startup_preparing) &&
        base_reference_tracking_ok && !reference_tracking_fault_latched_ &&
        !dynamic_fault_latched_;
    if (contract_prepare && takeover_started_ && !fixed_prepare_fresh) {
      ROS_WARN_THROTTLE(1.0,
          "[VelocityIk] fixed PREPARE waiting for an admitted fresh command with matching session/epoch; pose loss alone does not authorize it");
    }
    SolverCandidate candidate;
    bool candidate_matches = false;
    if (tracking) {
      shapeContinuousTarget(world_target,target,tick_start_ns,cycle_dt);
      // Solve and commit against one immutable 500 Hz snapshot.  The previous
      // asynchronous pipeline solved around q/dq/ddq at tick k and validated that
      // answer against a separately rebuilt domain at tick k+1.  A 2 ms
      // acceleration box is intentionally narrow, so even a sub-millisecond
      // result could alternate between ACCEPTED and BRAKING.  Keeping the
      // solve on the reference owner's clock removes that second temporal
      // owner and makes the solver constraints the commit constraints.
      SolverRequest request;
      // Bound feedback even after acquisition: a later target discontinuity
      // must not become an unbounded proportional chase. Live FF is separate.
      request.catchup_correction=true;
      request.acquisition_blend=acquisition_.blend();
      request.input_budget=input_continuity_.budget;
      request.target = world_target;
      // One reference state defines both the task and the committed command.
      request.reference_q = reference_q_;
      request.reference_dq = reference_dq_;
      request.reference_ddq = reference_ddq_;
      request.measured_q = measured_q;
      request.measured_dq = measured_dq;
      request.arm_tracking_error = arm_reference_tracking_error_by_arm_;
      request.measured_state_valid = robot_fresh;
      request.governor_risk = governor_risk_;
      request.base_reference_seq = reference_sequence_;
      request.cycle_dt = cycle_dt;
      request.target_age = target_age;
      request.feedforward_age = 0.0;  // per-channel decay already applied
      request.body_feedforward_age = reconstructed ? 0.0 :
          std::max(0.0, body_feedforward_age);
      request.lower_body_recruitment = 1.0;
      request.lower_joint_recruitment = {1.0, 1.0, 1.0, 1.0};
      request.base_recruitment = 0.0;
      populateBaseSessionBounds(&request);
      request.tracking = true;

      velocity_ik::runtime_probe::point(40);
      candidate = solveRequest(request);
      velocity_ik::runtime_probe::point(49);
      candidate_age = std::max(0.0,(ros::WallTime::now()-candidate.finish_time).toSec());
      candidate_session_id = candidate.session_id;
      candidate_generation = candidate.generation;
      candidate_target_seq = candidate.target_seq;
      solver_base_seq = candidate.base_reference_seq;
      candidate_reference_lag = 0;
      candidate_target_lag = 0;
      candidate_matches = true;
      candidate_decision =
          wheel_arm_abs_vel::VelocityIkShadow::CANDIDATE_ACCEPTED;
    }
    if (candidate_matches) {
      raw_solver_valid = candidate.solver_valid;
      qp_return_code = candidate.return_code;
      solve_time_us = candidate.solve_time_us;
      solver_base_seq = candidate.base_reference_seq;
      sigma_min = candidate.sigma_min;
      translation_sigma = candidate.translation_sigma;
      singularity_escape_constraint_active =
          candidate.singularity_escape_constraint_active;
      singularity_escape_constraint_fallback =
          candidate.singularity_escape_constraint_fallback;
      primary_regularization = candidate.primary_regularization;
      orientation_task_scale = candidate.orientation_task_scale;
      position_task_weight_scale =
          candidate.position_task_weight_scale;
      arm_pressure = candidate.arm_pressure;
      recruitment_target = candidate.recruitment_target;
      lower_joint_recruitment_target =
          candidate.lower_joint_recruitment_target;
      lower_joint_directional_effectiveness =
          candidate.lower_joint_directional_effectiveness;
      geometric_reach_demand = candidate.geometric_reach_demand;
      solver_lower_body_recruitment =
          candidate.solver_lower_body_recruitment;
      solver_lower_joint_recruitment =
          candidate.solver_lower_joint_recruitment;
      solver_base_recruitment = candidate.solver_base_recruitment;
      arm_only_task_residual = candidate.arm_only_task_residual;
      arm_lower_task_residual = candidate.arm_lower_task_residual;
      lower_body_saturation = candidate.lower_body_saturation;
      primary_task_scale = candidate.primary_task_scale;
      translation_task_scale = candidate.translation_task_scale;
      nullspace_residual = candidate.nullspace_residual;
      primary_task_leakage = candidate.primary_task_leakage;
      primary_task_rank = candidate.primary_task_rank;
      nullspace_dimension = candidate.nullspace_dimension;
      secondary_return_code = candidate.secondary_return_code;
      task_error = candidate.task_error;
      task_dq = candidate.task_dq;
      primary_dq = candidate.primary_dq;
      secondary_dq = candidate.secondary_dq;
      torso_viability_relaxation =
          candidate.torso_viability_relaxation;
      torso_task_scale = candidate.torso_task_scale;
      base_intent_task_scale = candidate.base_intent_task_scale;
      torso_task_error = candidate.torso_task_error;
      base_intent_error = candidate.base_intent_error;
      root_translation_intent_active =
          candidate.root_translation_intent_active;
      emergency_acceleration_used =
          candidate.emergency_acceleration_used;
      primary_task_jacobian = candidate.primary_task_jacobian;
      primary_task_velocity = candidate.primary_task_velocity;
    }

    // Waist tasks are explicit, not recruited by TCP error or pressure.
    active_reach_assist_demand_ = 0.0;
    active_lower_body_recruitment_ = tracking ? 1.0 : 0.0;
    active_lower_joint_recruitment_.fill(active_lower_body_recruitment_);
    active_base_recruitment_ = 0.0;
    const double supervisor_lower_body_recruitment = 1.0;
    const double supervisor_base_recruitment = 1.0;

    SolverRequest safety_request;
    // Braking starts from exactly the same command state as the normal solve.
    safety_request.reference_q = reference_q_;
    safety_request.reference_dq = reference_dq_;
    safety_request.reference_ddq = reference_ddq_;
    safety_request.measured_q = measured_q;
    safety_request.measured_dq = measured_dq;
    safety_request.arm_tracking_error = arm_reference_tracking_error_by_arm_;
    safety_request.measured_state_valid = robot_fresh;
    safety_request.cycle_dt = cycle_dt;
    safety_request.lower_body_recruitment =
        supervisor_lower_body_recruitment;
    safety_request.lower_joint_recruitment = {1.0, 1.0, 1.0, 1.0};
    safety_request.base_recruitment =
        supervisor_base_recruitment;
    populateBaseSessionBounds(&safety_request);
    DecisionVector next_dq = reference_dq_;
    bool preparation_commanded = false;
    bool tracking_command_held = false;

    if (preparing) {
      const DecisionVector& preparation_target = startup_preparing
          ? startup_preparation_position_ : preparation_position_;
      if (!preparation_active_) {
        // Input generation changes do not erase the executing trajectory.
        // Capture only on entry; repeated PREPARE packets must not restart it.
        preparation_entry_dq_ = reference_dq_;
        preparation_entry_ddq_ = reference_ddq_;
        preparation_entry_elapsed_ = 0.0;
        preparation_active_ = true;
        preparation_complete_ = false;
        preparation_settle_cycles_ = 0;
        preparation_initial_error_ = 0.0;
        for (int index :
             dof_layout_.decisionIndices(DofGroup::kLowerBody)) {
          preparation_initial_error_ = std::max(
              preparation_initial_error_,
              std::abs(preparation_target(index) - reference_q_(index)));
        }
        for (int arm = 0; arm < kArmCount; ++arm) {
          for (int joint = 0; joint < kJointsPerArm; ++joint) {
            const int index = armDecisionIndex(arm, joint);
            preparation_initial_error_ = std::max(
                preparation_initial_error_,
                std::abs(preparation_target(index) - reference_q_(index)));
          }
        }
        preparation_initial_error_ = std::max(
            preparation_initial_error_, preparation_position_tolerance_);
        ROS_INFO(
            "[VelocityIkShadow] whole-body preparation trajectory started");
      }

      DecisionVector preparation_velocity =
          DecisionVector::Zero(decisionDofCount());
      double reference_position_error = 0.0;
      double measured_position_error = 0.0;
      double lower_reference_error = 0.0;
      double lower_measured_error = 0.0;
      double selected_velocity = 0.0;
      const auto set_preparation_joint = [&](int index, double limit) {
        const double reference_error =
            preparation_target(index) - reference_q_(index);
        preparation_velocity(index) = std::clamp(
            preparation_gain_ * reference_error, -limit, limit);
        reference_position_error = std::max(
            reference_position_error, std::abs(reference_error));
        measured_position_error = std::max(
            measured_position_error,
            std::abs(preparation_target(index) - measured_q(index)));
        if (isDecisionGroup(index, DofGroup::kLowerBody)) {
          lower_reference_error = std::max(lower_reference_error, std::abs(reference_error));
          lower_measured_error = std::max(lower_measured_error,
              std::abs(preparation_target(index) - measured_q(index)));
        }
        selected_velocity = std::max(
            selected_velocity, std::abs(reference_dq_(index)));
        selected_velocity = std::max(selected_velocity, std::abs(measured_dq(index)));
      };
      for (int joint = 0; joint < kLowerBodyDof; ++joint) {
        const int index = lowerBodyDecisionIndex(joint);
        set_preparation_joint(
            index, std::min(velocity_limit_(index),
                            lower_joint_velocity_cap_.at(joint)));
      }
      for (int arm = 0; arm < kArmCount; ++arm) {
        for (int joint = 0; joint < kJointsPerArm; ++joint) {
          const int index = armDecisionIndex(arm, joint);
          set_preparation_joint(
              index, std::min(velocity_limit_(index),
                              preparation_arm_velocity_limit_));
        }
      }
      if (contract_prepare && preparation_handoff_duration_ > 0.0 &&
          preparation_entry_elapsed_ < preparation_handoff_duration_) {
        // Join the old velocity and acceleration to the live joint target.
        // The quintic weight has zero first/second derivatives at both ends.
        // This is an entry segment, not a filter on normal tracking or output:
        // q and ddq still come solely from the common committed-v integrator.
        const double t = std::min(preparation_entry_elapsed_ + cycle_dt,
                                  preparation_handoff_duration_);
        const double x = t / preparation_handoff_duration_;
        const double blend = x*x*x*(10.0 + x*(-15.0 + 6.0*x));
        preparation_velocity =
            (1.0-blend)*(preparation_entry_dq_ + t*preparation_entry_ddq_) +
            blend*preparation_velocity;
        preparation_entry_elapsed_ = t;
      }
      // Hard position/velocity/acceleration and stop authority retain priority
      // over the preferred handoff segment, including unavoidable braking.
      next_dq = calculateSupervisedVelocity(
          safety_request, preparation_velocity, true,
          DecisionMatrix(), DecisionVector(),
          &jerk_slack, &boundary_brake,
          &operational_boundary_brake,
          &emergency_acceleration_used, &task_scale,
          &task_scaling_residual, &task_scaling_baseline,
          &task_scaling_unscaled,
          &task_scaling_qp_valid, &torso_viability_relaxation);
      preparation_commanded = true;
      preparation_progress_ = std::clamp(
          1.0 - reference_position_error /
                    std::max(preparation_initial_error_, 1.0e-6),
          0.0, 1.0);
      const bool settled =
          lower_reference_error <= preparation_lower_position_tolerance_ &&
          lower_measured_error <= preparation_lower_measured_position_tolerance_ &&
          reference_position_error <= preparation_position_tolerance_ &&
          measured_position_error <=
              preparation_measured_position_tolerance_ &&
          selected_velocity <= preparation_velocity_tolerance_;
      preparation_settle_cycles_ = settled
          ? preparation_settle_cycles_ + 1
          : 0;
      if (preparation_settle_cycles_ >=
          preparation_settle_cycles_required_) {
        if (!preparation_complete_) {
          ROS_INFO(
              "[VelocityIkShadow] whole-body preparation trajectory complete");
        }
        preparation_complete_ = true;
        preparation_progress_ = 1.0;
        if (startup_preparing && startup_bootstrap_active_) {
          // The fixed ready posture has removed the natural-hang
          // singularity.  Re-seed the executable Cartesian target at the FK
          // of this continuous reference, then begin the ordinary absolute
          // human-target acquisition on the following cycle.
          if (captureSessionTargetAlignment(
                  target, reference_q_, base_coordinate_epoch)) {
            // The body/hand neutral sample must describe the same instant as
            // the post-bootstrap TCP alignment. Real operators keep moving
            // while the robot raises its naturally hanging arms; retaining
            // the earlier ARMED body anchor falsely classified that ordinary
            // motion as full reach assistance on the first TRACKING tick.
            if (target.body.torsoValid()) {
              captureSessionBodyAnchor(target, reference_q_, false);
              last_body_intent_ = BodyIntent{};
            } else {
              session_body_anchor_ = SessionBodyAnchor{};
            }
            active_lower_body_recruitment_ = 0.0;
            active_reach_assist_demand_ = 0.0;
            reference_history_.clear();
            active_lower_joint_recruitment_.fill(0.0);
            lower_joint_stage_active_.fill(false);
            lower_joint_admission_elapsed_.fill(0.0);
            lower_joint_release_elapsed_.fill(0.0);
            cascade_stage_ = 0;
            active_base_recruitment_ = 0.0;
            base_admission_elapsed_ = 0.0;
            base_release_elapsed_ = 0.0;
            base_admission_active_ = false;
            // ARMED was captured while the arms were hanging vertically.
            // That pose is intentionally only the zero-motion enable state;
            // it must not remain the ACTIVE nullspace posture after the
            // bootstrap.  Otherwise the secondary objective continuously
            // tries to straighten both elbows while the primary Cartesian
            // task is moving toward the human hands, which creates exactly
            // the slow, internally twisting branch observed in closed loop.
            // Rebase the arm posture at the dynamically reached ready pose.
            // The startup path preserves the measured lower-body posture as
            // its session neutral, and the base has no ACTIVE return
            // objective.
            for (int arm = 0; arm < kArmCount; ++arm) {
              for (int joint = 0; joint < kJointsPerArm; ++joint) {
                const int index = armDecisionIndex(arm, joint);
                session_posture_position_(index) = reference_q_(index);
              }
            }
            startup_bootstrap_active_ = false;
            fallback_active_ = true;
            tracking_ramp_scale_ = 0.0;
            ROS_INFO(
                "[VelocityIkShadow] natural-hang bootstrap complete; "
                "starting direct absolute TCP tracking");
          } else {
            ROS_ERROR_THROTTLE(
                1.0,
                "[VelocityIkShadow] failed to re-seed Cartesian alignment "
                "after natural-hang bootstrap");
          }
        }
      }
      state = preparation_complete_ ? ShadowState::kHold
                                    : ShadowState::kPreparing;
      fallback_active_ = true;
      consecutive_solver_failures_ = 0;
    } else if (!contract_prepare) {
      preparation_active_ = false;
      preparation_complete_ = false;
      preparation_settle_cycles_ = 0;
      preparation_initial_error_ = 0.0;
      preparation_progress_ = 0.0;
    }

    const auto velocity_is_currently_feasible =
        [&](const DecisionVector& velocity) {
          const HardBounds current_bounds = calculateHardBounds(safety_request);
          const LinearConstraints preferred_current_torso =
              calculateTorsoVelocityConstraintsInContext(
                  safety_request, supervisor_context_.get(),
                  &supervisor_q_full_);
          double current_torso_relaxation = 0.0;
          const LinearConstraints current_torso =
              current_bounds.valid
                  ? makeConstraintsRecursivelyFeasible(
                        preferred_current_torso, current_bounds.lower,
                        current_bounds.upper, &current_torso_relaxation)
                  : preferred_current_torso;
          bool feasible = current_bounds.valid && current_torso.valid &&
              velocity.size() == decisionDofCount() &&
              velocity.array().isFinite().all() &&
              (velocity.array() >=
               current_bounds.lower.array() - 1.0e-8).all() &&
              (velocity.array() <=
               current_bounds.upper.array() + 1.0e-8).all();
          if (feasible && current_torso.matrix.rows() > 0) {
            const DecisionVector torso_rate = current_torso.matrix * velocity;
            feasible =
                (torso_rate.array() >=
                 current_torso.lower.array() - 1.0e-8).all() &&
                (torso_rate.array() <=
                 current_torso.upper.array() + 1.0e-8).all();
          }
          return feasible;
        };

    const auto velocity_matches_candidate_certificate =
        [&](const SolverCandidate& certified_candidate) {
          const DecisionVector& velocity = certified_candidate.task_dq;
          bool feasible = certified_candidate.dynamic_certificate_valid &&
              velocity.size() == decisionDofCount() &&
              certified_candidate.certified_dynamic_lower.size() ==
                  decisionDofCount() &&
              certified_candidate.certified_dynamic_upper.size() ==
                  decisionDofCount() &&
              velocity.array().isFinite().all() &&
              (velocity.array() >=
               certified_candidate.certified_dynamic_lower.array() -
                   1.0e-8).all() &&
              (velocity.array() <=
               certified_candidate.certified_dynamic_upper.array() +
                   1.0e-8).all();
          if (feasible &&
              certified_candidate.certified_constraint_matrix.rows() > 0) {
            const DecisionVector rate =
                certified_candidate.certified_constraint_matrix * velocity;
            feasible =
                rate.size() ==
                    certified_candidate.certified_constraint_lower.size() &&
                rate.size() ==
                    certified_candidate.certified_constraint_upper.size() &&
                (rate.array() >=
                 certified_candidate.certified_constraint_lower.array() -
                     1.0e-8).all() &&
                (rate.array() <=
                 certified_candidate.certified_constraint_upper.array() +
                     1.0e-8).all();
          }
          return feasible;
        };

    if (!preparing && tracking && candidate_matches && candidate.solver_valid) {
      const bool candidate_certified =
          velocity_matches_candidate_certificate(candidate);
      if (candidate_certified) {
        // The candidate was solved in the complete q/dq/ddq feasible set.
        // It is the final normal command: no Cartesian state, global scale,
        // critical damping, jerk follower or per-joint post-clamp follows.
        // A recovered solve is usable immediately; an artificial recovery
        // ramp would be another Cartesian governor under a different name.
        next_dq = candidate.task_dq;
        candidate_accepted = true;
        candidate_hold_cycles_ = 0;
        fallback_active_ = false;
        consecutive_solver_failures_ = 0;
        task_scale = candidate.primary_task_scale;
        task_scaling_qp_valid = true;
        state = ShadowState::kTracking;
      } else if (velocity_is_currently_feasible(reference_dq_) &&
                 candidate_hold_cycles_ < max_candidate_hold_cycles_) {
        // A numerically failed current solve may retain the already certified
        // velocity for at most a few ticks. This bounded zero-order hold is a
        // solver-failure fallback, not a reference trajectory or motion filter.
        next_dq = reference_dq_;
        tracking_command_held = true;
        ++candidate_hold_cycles_;
        fallback_active_ = false;
        task_scale = candidate.primary_task_scale;
        task_scaling_qp_valid = true;
        state = ShadowState::kTracking;
      } else {
        // If a candidate ever fails its own dynamic certificate, the safe
        // response is the current-domain braking baseline. This is a normal
        // task-progress result, so ownership remains ACTIVE; entering the
        // lifecycle SAFETY_BRAKING state here would make ReferenceManager
        // drop a healthy session and create the very stop/restart pulse we
        // are trying to avoid.
        fallback_active_ = true;
        ++consecutive_solver_failures_;
        // A failure episode spends its hold budget once. Braking cannot
        // re-arm it; only a newly certified candidate may do that.
        candidate_hold_cycles_ = max_candidate_hold_cycles_;
        state = ShadowState::kTracking;
        ROS_ERROR_THROTTLE(
            1.0,
            "[VelocityIkShadow] solver returned an invalid dynamic-domain "
            "certificate; applying one-cycle feasible braking while "
            "retaining ACTIVE ownership");
      }
    } else if (!preparing && tracking) {
      if (velocity_is_currently_feasible(reference_dq_) &&
          candidate_hold_cycles_ < max_candidate_hold_cycles_) {
        next_dq = reference_dq_;
        tracking_command_held = true;
        ++candidate_hold_cycles_;
        task_scale = primary_task_scale;
        task_scaling_qp_valid = true;
        state = ShadowState::kTracking;
      } else {
        fallback_active_ = true;
        ++consecutive_solver_failures_;
        // A failure episode spends its hold budget once. Braking cannot
        // re-arm it; only a newly certified candidate may do that.
        candidate_hold_cycles_ = max_candidate_hold_cycles_;
        // A solver scheduling miss is not an execution fault. The common
        // hard-domain braking solve below keeps the command safe while the
        // formal reference remains continuously ACTIVE.
        state = ShadowState::kTracking;
      }
    }

    if (!tracking && !preparing) {
      fallback_active_ = true;
      candidate_hold_cycles_ = max_candidate_hold_cycles_;
      tracking_ramp_scale_ = 0.0;
      consecutive_solver_failures_ = 0;
      if (!reference_initialized_) {
        state = target_fresh && !mode_ok ? ShadowState::kModeBlocked
                                         : ShadowState::kWaitingForInput;
      } else if (!world_target.valid) {
        state = ShadowState::kWaitingForInput;
      } else if (!mode_ok) {
        state = ShadowState::kModeBlocked;
      } else if (!base_reference_tracking_ok) {
        state = ShadowState::kSafetyBraking;
      } else {
        state = ShadowState::kStaleBraking;
      }
    }

    if (!execution_waiting_ && reference_initialized_ && !candidate_accepted &&
        !tracking_command_held &&
        !preparation_commanded) {
      next_dq = calculateSupervisedVelocity(
          safety_request, DecisionVector::Zero(decisionDofCount()), true,
          DecisionMatrix(), DecisionVector(),
          &jerk_slack, &boundary_brake,
          &operational_boundary_brake,
          &emergency_acceleration_used, &task_scale,
          &task_scaling_residual, &task_scaling_baseline,
          &task_scaling_unscaled,
          &task_scaling_qp_valid, &torso_viability_relaxation);
      if (tracking) {
        // This cycle applied a feasible brake, not the candidate task.  The
        // legacy supervisor helper reports scale=1 for its zero command;
        // overwrite it so bag analysis cannot mistake a braking pulse for
        // perfect Cartesian progress.
        task_scale = 0.0;
        task_scaling_qp_valid = false;
      }
      const DecisionVector next_ddq = (next_dq - reference_dq_) / cycle_dt;
      if (!tracking && maxAbs(next_dq) < hold_velocity_threshold_ &&
          maxAbs(next_ddq) < hold_acceleration_threshold_) {
        state = ShadowState::kHold;
      }
    }

    if (reference_tracking_fault_latched_ || dynamic_fault_latched_) {
      state = ShadowState::kSafetyBraking;
      candidate_accepted = false;
      fallback_active_ = true;
    }

    // Recheck immediately before committing: solver/runtime delay may exhaust
    // a previously valid grant. Expiry never grants producer ownership.
    if (enable_takeover_ && !execution_lease_.canAdvance(
            velocity_ik_execution::executionSteadyNs(), static_cast<int64_t>(cycle_dt * 1e9))) {
      execution_waiting_=true;execution_anchor_seq_=execution_lease_.anchor();
    }
    DecisionVector output_ddq = DecisionVector::Zero(decisionDofCount());
    if (execution_waiting_) {
      // WBC exclusively owns integration until it acknowledges our exact
      // stopped-state echo. Do not compute an independent stopping trajectory.
      reference_valid = execution_anchor_seq_ != 0 ||
          (execution_lease_.initialPending() && reference_initialized_ && takeover_started_);
      // Initial ownership is negotiated using the already verified standstill
      // seed. No first segment is integrated before the exact seed is echoed.
      if (reference_valid) {
        output_ddq = reference_ddq_;
        ++reference_sequence_;
      }
      state = reference_valid ? ShadowState::kHold : ShadowState::kStaleBraking;
      candidate_accepted = false;
      tracking_command_held = false;
      preparation_commanded = false;
    } else if (reference_initialized_) {
      const DecisionVector previous_command_dq = reference_dq_;
      output_ddq = (next_dq - previous_command_dq) / cycle_dt;
      reference_q_ += 0.5 * cycle_dt * (previous_command_dq + next_dq);
      reference_dq_ = next_dq;
      reference_ddq_ = output_ddq;
      // A material overrun is a fault, not a silent position-only clip that
      // would break q/dq/ddq consistency and hide a second trajectory.
      const bool integrated_position_valid =
          (reference_q_.array() >= position_lower_.array() - 1.0e-7).all() &&
          (reference_q_.array() <= position_upper_.array() + 1.0e-7).all();
      if (!integrated_position_valid) {
        dynamic_fault_latched_ = true;
        dynamic_supervisor_valid_ = false;
        candidate_accepted = false;
        fallback_active_ = true;
        state = ShadowState::kSafetyBraking;
        ROS_ERROR_THROTTLE(1.0, "[VelocityIk] integrated reference exceeded URDF bounds");
      }
      if (robot_fresh && !enable_takeover_) {
        // Do not publish a fictitious integrated base pose while the shadow
        // process has no actuator ownership.  dq remains the solver's proposed
        // base motion and q remains the observed world pose.
        synchronizeExogenousBasePosition(measured_q, &reference_q_);
      }
      ++reference_sequence_;
      reference_valid = integrated_position_valid && reference_q_.array().isFinite().all() &&
                        reference_dq_.array().isFinite().all() &&
                        reference_ddq_.array().isFinite().all();
    } else {
      reference_valid = false;
    }

    // Only an adopted command can advance the governor's short risk memory.
    // Rejected/stale/prepare paths cannot leave a slow-motion latch behind.
    if (tracking && candidate_accepted && reference_valid)
      governor_risk_=candidate.governor_risk;
    else governor_risk_.fill(0.);

    // Track initial convergence using both plant and reference FK for diagnostics.
    // The startup joint/FF ramp is independent of alignment; feedback stays bounded.
    if (tracking && candidate_accepted) {
      setPlantDecisionPositions(measured_q);
      bool aligned=true;
      const auto& torso=plant_->GetFrameByName(torso_frame_name_);
      for (int arm=0;arm<kArmCount;++arm) if (armIsControlled(arm)) {
        const auto pose=plant_->GetFrameByName(hand_frame_names_[arm]).CalcPose(*plant_context_,torso);
        aligned &= (world_target.arm[arm].hand_position-pose.translation()).norm()<=acquisition_position_tolerance_ &&
            rotationLog(world_target.arm[arm].hand_orientation.toRotationMatrix()*pose.rotation().matrix().transpose()).norm()<=acquisition_rotation_tolerance_ &&
            candidate.task_error[arm*3]<=acquisition_position_tolerance_ &&
            candidate.task_error[arm*3+1]<=acquisition_rotation_tolerance_;
      }
      aligned &= arm_reference_tracking_error_<=reference_tracking_recovery_arm_limit_ &&
          lower_reference_tracking_error_<=reference_tracking_recovery_lower_limit_;
      if (world_target.body_intent.torso_valid) {
        for (const DecisionVector* posture : {&measured_q, &reference_q_}) {
          const Eigen::Vector4d error=world_target.body_intent.desired_torso_feature-
              torsoFeatureInContext(*posture,plant_context_.get(),&q_full_);
          aligned &= std::abs(error(1))<=acquisition_position_tolerance_ &&
              error.tail<2>().cwiseAbs().maxCoeff()<=acquisition_rotation_tolerance_;
        }
      }
      acquisition_.update(cycle_dt,aligned,input_continuity_.phase==velocity_ik::InputPhase::Fresh);
    }
    tracking_ramp_scale_ = tracking ? acquisition_.blend() : 0.0;
#ifndef VELOCITY_IK_OFFLINE_TEST
    publishStdVector(continuity_pub_, {1.,target_read_ns*1e-9,double(reference_sequence_),
        double(input_continuity_.phase),target_age,input_continuity_.budget,
        double(input_continuity_.confirmation_required),double(input_continuity_.stable),
        double(confirm_ready_.load()),acquisition_.blend(),double(tracking),
        double(target.observation_sequence),double(candidate_accepted),
        double(reference_tracking_fault_latched_),double(dynamic_fault_latched_),candidate.continuity_relaxation,
        double(acquisition_.catchup)});
#endif

    publishFormalReference(
        world_target, base_state_age, state, reference_valid,
        emergency_acceleration_used || dynamic_fault_latched_ ||
            reference_tracking_fault_latched_,
        task_scale, output_ddq);
    const double formal_end_ns=monotonicNs();
    diagnostic_publish_due_ = ++diagnostic_counter_ >= diagnostic_stride_;
    if (diagnostic_publish_due_) diagnostic_counter_ = 0;
    publishVector(q_pub_, reference_q_);
    publishVector(dq_pub_, reference_dq_);
    publishVector(ddq_pub_, output_ddq);
    publishVector(task_dq_pub_, task_dq);
    const uint8_t applied_command_mode = preparation_commanded
        ? wheel_arm_abs_vel::VelocityIkShadow::APPLIED_PREPARATION
        : (candidate_accepted
               ? wheel_arm_abs_vel::VelocityIkShadow::APPLIED_NEW_SOLUTION
               : (tracking_command_held
                      ? wheel_arm_abs_vel::VelocityIkShadow::
                            APPLIED_ZERO_ORDER_HOLD
                      : (reference_initialized_
                             ? wheel_arm_abs_vel::VelocityIkShadow::
                                   APPLIED_BRAKING
                             : wheel_arm_abs_vel::VelocityIkShadow::
                                   APPLIED_NONE)));
    // Diagnostics named raw_world_tcp_pose represent the ungoverned robot
    // target after the one-time human/robot alignment.  Publishing the
    // pre-alignment controller coordinates here added a constant body-scale
    // offset and made target-to-robot latency diagnostics meaningless.
    publishReference(aligned_torso_target, aligned_world_target, target.frame_id,
                     target_age, base_state_age,
                     base_coordinate_jump, base_coordinate_epoch, measured_q,
                     cycle_dt, solve_time_us, state, reference_valid,
                     raw_solver_valid, qp_return_code,
                     emergency_acceleration_used, candidate_age, solver_base_seq,
                     candidate_decision, applied_command_mode,
                     candidate_session_id,
                     candidate_generation, candidate_target_seq,
                     candidate_reference_lag, candidate_target_lag,
                     reference_dq_, output_ddq, task_dq,
                     primary_dq, secondary_dq,
                     jerk_slack, boundary_brake, operational_boundary_brake,
                     primary_task_jacobian, primary_task_velocity,
                     task_scale, task_scaling_residual,
                     task_scaling_baseline,
                     task_scaling_unscaled,
                     task_scaling_qp_valid,
                     torso_viability_relaxation, torso_task_scale,
                     base_intent_task_scale, torso_task_error,
                     base_intent_error,
                     root_translation_intent_active,
                     sigma_min, translation_sigma,
                     singularity_escape_constraint_active,
                     singularity_escape_constraint_fallback,
                     primary_regularization,
                     arm_pressure, recruitment_target,
                     lower_joint_recruitment_target,
                     lower_joint_directional_effectiveness,
                     geometric_reach_demand,
                     solver_lower_body_recruitment,
                     solver_lower_joint_recruitment,
                     solver_base_recruitment,
                     arm_only_task_residual,
                     arm_lower_task_residual,
                     lower_body_saturation,
                     primary_task_scale,
                     translation_task_scale,
                     supervisor_lower_body_recruitment,
                     supervisor_base_recruitment,
                     orientation_task_scale,
                     position_task_weight_scale, task_error,
                     primary_task_rank, nullspace_dimension,
                     nullspace_residual, primary_task_leakage,
                     secondary_return_code);
    recordReferenceHistory(now);

    if (diagnostic_publish_due_) {
      std::vector<double> status{
          1.0,
          state == ShadowState::kTracking ? 1.0 : 0.0,
          static_cast<double>(static_cast<int>(state)),
          reference_valid ? 1.0 : 0.0,
          raw_solver_valid ? 1.0 : 0.0,
          fallback_active_ ? 1.0 : 0.0,
          std::isfinite(target_age) ? 1000.0 * target_age : -1.0,
          solve_time_us,
          candidate_age >= 0.0 ? 1000.0 * candidate_age : -1.0,
          sigma_min.at(0),
          sigma_min.at(1),
          std::max(task_error.at(0), task_error.at(3)),
          std::max(task_error.at(1), task_error.at(4)),
          std::max(task_error.at(2), task_error.at(5)),
          maxAbs(reference_dq_),
          maxAbs(output_ddq),
          robot_valid ? maxAbs(reference_q_ - measured_q) : 0.0,
          static_cast<double>(qp_return_code),
          mode_known ? static_cast<double>(current_mode) : -1.0,
          mode_transitioning ? 1.0 : 0.0,
          0.0,
          1000.0 * wall_cycle_dt,
          static_cast<double>(consecutive_solver_failures_),
          maxAbs(jerk_slack),
          boundary_brake.sum(),
          emergency_acceleration_used ? 1.0 : 0.0,
          candidate_accepted ? 1.0 : 0.0,
          tracking_ramp_scale_,
          maxAbs(task_dq - reference_dq_),
          maxAbs(secondary_dq),
          nullspace_residual,
          primary_task_leakage,
          primary_regularization.at(0),
          primary_regularization.at(1),
          operational_boundary_brake.sum(),
          orientation_task_scale.at(0),
          orientation_task_scale.at(1),
          position_task_weight_scale.at(0),
          position_task_weight_scale.at(1),
          recruitment_target.pressure,
          recruitment_target.lower_body,
          recruitment_target.base,
          active_lower_body_recruitment_,
          active_base_recruitment_,
          task_scale,
          task_scaling_residual,
          task_scaling_qp_valid ? 1.0 : 0.0,
          torso_viability_relaxation,
          arm_only_task_residual,
          arm_lower_task_residual,
          lower_body_saturation,
          primary_task_scale};
      publishStdVector(status_pub_, status);
      publishStdVector(task_error_pub_, std::vector<double>(task_error.begin(), task_error.end()));

      std::vector<double> target_ff;
      target_ff.reserve(18);
      for (int arm = 0; arm < kArmCount; ++arm) {
        for (int i = 0; i < 3; ++i) target_ff.push_back(world_target.arm.at(arm).hand_linear_ff(i));
        for (int i = 0; i < 3; ++i) target_ff.push_back(world_target.arm.at(arm).hand_angular_ff(i));
        for (int i = 0; i < 3; ++i) target_ff.push_back(world_target.arm.at(arm).elbow_linear_ff(i));
      }
      publishStdVector(target_ff_pub_, target_ff);
      if (host_target_reconstruction_ || (continuous_reference_enabled_ && target.estimated_twists_valid)) {
        std::vector<double> p{double(target.session_id),double(target.generation),double(target.source_frame_seq),reconstruction_age,reconstructed?1.0:0.0};
        for(const auto& a:world_target.arm){for(int i=0;i<3;++i)p.push_back(a.hand_position[i]);for(int i=0;i<4;++i)p.push_back(a.hand_orientation.coeffs()[i]);for(int i=0;i<3;++i)p.push_back(a.elbow_position[i]);}
        publishStdVector(reconstructed_target_pub_,p);
      }
      // Schema 1: wall time, transaction and actual shaper state. See docs.
      const bool reference_active=tracking && continuous_reference_enabled_ && (target.estimated_twists_valid || orientation_reference_mode_=="fixed_bandwidth");
      std::vector<double> adaptive_diag{1.,now.toSec(),double(target.session_id),
          double(target.generation),double(target.source_frame_seq),double(tracking),
          double(reference_active),double(reference_active && adaptive_reference_enabled_ && orientation_reference_mode_!="fixed_bandwidth"),
          hand_reference_frequency_,adaptive_reference_settings_.min_frequency};
      for(int arm=0;arm<kArmCount;++arm) {
        const auto append=[&adaptive_diag,reference_active](const auto& reference) {
          const auto& g=reference.gate;
          adaptive_diag.insert(adaptive_diag.end(),{g.activity,g.frequency,g.error,g.speed,
              g.guide,double(reference_active && reference.valid)});
        };
        append(hand_reference_[arm]);append(hand_rotation_reference_[arm]);append(elbow_reference_[arm]);
      }
      publishStdVector(adaptive_reference_pub_,adaptive_diag);
      // Schema1: header[version,time,session,generation,source_seq,active],
      // per arm: hand and elbow [alignment,silence,prediction_age,rate,hold,valid],
      // then hand-position/rotation/elbow [effective_hz,guide,delta_a,limited].
      std::vector<double> dynamics{1.,now.toSec(),double(target.session_id),
          double(target.generation),double(target.source_frame_seq),double(reference_active)};
      for(int arm=0;arm<kArmCount;++arm) {
        for(const auto* p:{&hand_prediction_[arm],&elbow_prediction_[arm]})
          dynamics.insert(dynamics.end(),{p->alignment_age,p->silence,p->age,p->rate,p->hold,double(reference_active && p->valid)});
        const auto append=[&](const auto& ref) {
          const auto& tr=ref.transition;
          dynamics.insert(dynamics.end(),{tr.frequency,tr.guide,tr.acceleration_change,double(tr.limited)});
        };
        append(hand_reference_[arm]);append(hand_rotation_reference_[arm]);append(elbow_reference_[arm]);
      }
      publishStdVector(reference_dynamics_pub_,dynamics);
      // Schema documented in docs/innovation_reference_v1.md. Diagnostics
      // never drive the control state. Legacy topics/layouts stay unchanged.
      const bool split_active=reference_active && adaptive_reference_settings_.innovation.enabled && orientation_reference_mode_!="fixed_bandwidth";
      std::vector<double> innovation_diag{1.,now.toSec(),double(target.session_id),double(target.generation),
          double(target.source_frame_seq),double(split_active),cycle_dt};
      const auto append_split=[&](const auto& ref) {
        const auto& x=ref.innovation;
        innovation_diag.push_back(double(split_active && ref.valid && x.valid));
        for(const auto* vector_components:{&x.motion.v,&x.motion.a,&x.correction.v,&x.correction.a,&x.residual,&x.modeled_lag})
          for(int i=0;i<3;++i)innovation_diag.push_back((*vector_components)[i]);
        innovation_diag.push_back(double(x.correction.limited));
      };
      for(int arm=0;arm<kArmCount;++arm) {
        append_split(hand_reference_[arm]);append_split(hand_rotation_reference_[arm]);append_split(elbow_reference_[arm]);
        for(const auto* audit:{&hand_innovation_audit_[arm],&elbow_innovation_audit_[arm]}) {
          innovation_diag.push_back(double(split_active && audit->valid));
          innovation_diag.push_back(audit->update_ns);innovation_diag.push_back(audit->anchor.source_stamp);
          for(const auto* vector_components:{&audit->dp,&audit->dr,&audit->dv,&audit->dw})
            for(int i=0;i<3;++i)innovation_diag.push_back((*vector_components)[i]);
        }
      }
      publishStdVector(innovation_reference_pub_,innovation_diag);

      if (feasibility_mode_ != FeasibilityMode::kOff &&
          reference_initialized_ && world_target.valid) {
        FeasibilityRequest feasibility_request;
        feasibility_request.target = world_target;
        feasibility_request.reference_q = reference_q_;
        feasibility_request.reference_seq = reference_sequence_;
        feasibility_request.enqueue_time = now;
        feasibility_request.tracking_active = tracking;
        queueFeasibilityRequest(feasibility_request);
      }
    }

    if (reference_sequence_ % envelope_diagnostics_stride_ == 0) {
      std_msgs::Float64MultiArray d;
      d.data={1.,double(output_sequence_-1),double(reference_sequence_),
          double(target.session_id),double(target.generation),double(state),
          double(execution_governor_enabled_),double(candidate_accepted),
          double(reference_valid),double(candidate.governor_diagnostics.size()/18),18.,
          cycle_dt,double(robot_fresh)};
      d.data.insert(d.data.end(),candidate.governor_diagnostics.begin(),candidate.governor_diagnostics.end());
      const auto publisher=execution_governor_pub_;
      diagnostics_.submit([publisher,d]{publisher.publish(d);});
    }
    if (envelope_diagnostics_enabled_ && (!candidate.operating_diagnostics.empty() ||
        (!tracking && reference_sequence_ % envelope_diagnostics_stride_ == 0))) {
      std_msgs::Float64MultiArray diagnostic;
      diagnostic.data = std::move(candidate.operating_diagnostics);
      if (diagnostic.data.empty()) {
        diagnostic.data.assign(28, std::numeric_limits<double>::quiet_NaN());
        diagnostic.data[0] = 1; diagnostic.data[1] = 0; diagnostic.data[2] = 32;
        diagnostic.data[6] = target.session_id; diagnostic.data[7] = target.generation;
        diagnostic.data[8] = target.sequence; diagnostic.data[9] = target.source_stamp.toSec();
        diagnostic.data[10] = cycle_dt;
        diagnostic.data[18] = operating_envelope_enabled_; diagnostic.data[19] = temporal_cost_enabled_;
      }
      diagnostic.data[3] = output_sequence_ - 1; diagnostic.data[4] = reference_sequence_;
      diagnostic.data[11] = candidate_accepted; diagnostic.data[12] = applied_command_mode;
      diagnostic.data[13] = static_cast<int>(state); diagnostic.data[24] = diagnostics_.dropped();
      diagnostic.data[25] = reference_valid; diagnostic.data[26] = wall_cycle_dt;
      diagnostic.data[27] = execution_waiting_;
      const auto publisher = envelope_diagnostics_pub_;
      diagnostics_.submit([publisher, diagnostic]{ publisher.publish(diagnostic); });
    }
    if(joint_jerk_enabled_ && candidate.diagnostics_due) {
      std_msgs::Float64MultiArray d;
      d.data={1.,double(output_sequence_-1),double(reference_sequence_),double(target.session_id),
          double(target.generation),double(applied_command_mode),double(candidate_accepted),
          double(candidate.joint_jerk_active),double(candidate.joint_jerk_retry),
          double(candidate.joint_jerk_first_code),cycle_dt,14.,9.};
      d.data.insert(d.data.end(),candidate.joint_jerk_diagnostics.begin(),candidate.joint_jerk_diagnostics.end());
      const auto publisher=joint_jerk_pub_;diagnostics_.submit([publisher,d]{publisher.publish(d);});
    }
    std_msgs::Float64MultiArray timing;
    // schema=1; durations and *_ns use CLOCK_MONOTONIC, sensor/source stamps keep their own domains.
    timing.data={1.0,double(output_sequence_-1),double(reference_sequence_),double(target.session_id),double(target.generation),double(target.sequence),double(target.source_frame_seq),target.source_stamp.toSec(),target.receive_monotonic_ns,tick_start_ns,target_read_ns,formal_end_ns,monotonicNs(),wall_cycle_dt,cycle_dt,sensor_stamp,double(sensor_seq),sensor_receive_ns,double(current_mode),double(target_mode),mode_transitioning?1.0:0.0,robot_fresh?1.0:0.0,source_target_fresh?1.0:0.0,mode_ok?1.0:0.0,double(applied_command_mode),last_task_us_,last_bounds_us_,double(diagnostics_.dropped()),last_qp_finish_ns_,last_formal_publish_begin_ns_,last_formal_publish_end_ns_,reconstructed?1.0:0.0,reconstruction_age};
    // Append-only ownership diagnostics: 33 reason (0 shadow, 1 initial
    // grant pending, 2 granted, 3 stop-first, 4 owner restart required),
    // 34 granted steady deadline, 35 accepted echo sequence, 36 epoch,
    // 37 anchor, 38 waiting, 39 latest verified echo generation time.
    const int execution_reason = !enable_takeover_ ? 0 :
        (execution_lease_.restartRequired() ? 4 :
        (execution_lease_.initialPending() ? 1 : (execution_waiting_ ? 3 : 2)));
    timing.data.insert(timing.data.end(), {double(execution_reason),
        double(execution_lease_.leaseUntil()), double(execution_feedback_applied_seq_),
        double(execution_epoch_), double(execution_anchor_seq_),
        execution_waiting_ ? 1.0 : 0.0,
        execution_feedback ? double(execution_feedback->generated_monotonic_ns) : 0.0});
    if (execution_reason != execution_reason_previous_) {
      execution_reason_previous_ = execution_reason;
      diagnostics_.submit([execution_reason]{
        const char* names[]={"SHADOW","WAIT_INITIAL_GRANT","GRANTED","STOP_FIRST_WAIT","OWNER_RESTART_REQUIRED"};
        ROS_INFO("[VelocityIk] execution ownership: %s", names[execution_reason]);
      });
    }
    const auto timing_publisher=cycle_timing_pub_;
    // Parallel explanations of EXISTING predicates; never admission inputs.
    // SourceUnavailable includes invalid/inactive and is not claimed to be a
    // transport timeout. Confidence is diagnostic only; its legacy bit stays clear.
    const uint32_t hold_mask = velocity_ik::holdReasons(
        true, source_target_admissible,
        reference_tracking_fault_latched_, dynamic_fault_latched_,
        preparation_complete_ && contract_prepare, execution_waiting_, mode_ok,
        robot_fresh, contract_active || contract_prepare);
    const bool same_session_owner = session_anchor_.valid &&
        session_anchor_.session_id == target.session_id &&
        session_anchor_.generation == target.generation &&
        session_anchor_.base_coordinate_epoch == base_coordinate_epoch &&
        !execution_waiting_ && mode_ok;
    const uint32_t recovery_mask = numeric_recovery_mask | velocity_ik::recoveryReasons(
        true,true,true,true,target_fresh,same_session_owner);
    if (hold_mask != hold_reason_previous_ ||
        (reference_tracking_fault_latched_ && recovery_mask != recovery_reason_previous_) ||
        static_cast<int>(state) != hold_state_previous_) {
      hold_reason_previous_=hold_mask; recovery_reason_previous_=recovery_mask;
      hold_state_previous_=static_cast<int>(state);
      const int diagnostic_state=static_cast<int>(state);
      const auto session=target.session_id, generation=target.generation;
      const double good_s=diagnostic_recovery_good_s, age=target_age;
      const double wall_s=now.toSec();
      diagnostics_.submit([hold_mask,recovery_mask,diagnostic_state,session,generation,good_s,age,wall_s]{
        ROS_INFO("[HoldReason] wall_s=%.9f state=%d session=%llu generation=%llu reasons=0x%x recovery_blocked=0x%x consecutive_good_s=%.6f source_age_s=%.6f; reasons:1=reserved_confidence,2=source_unavailable,4=tracking,8=dynamic,16=prepare_complete,32=owner_lease,64=mode,128=robot_stale,256=inactive; recovery:1=arm,2=lower,4=ref_speed,8=measured_speed,16=source,32=session_owner (last two gate resume, not fault-latch clearing)",
            wall_s,diagnostic_state,static_cast<unsigned long long>(session),static_cast<unsigned long long>(generation),hold_mask,recovery_mask,good_s,age);
      });
    }
    diagnostics_.submit([timing_publisher,timing]{timing_publisher.publish(timing);});

    // No asynchronous solver request is queued here.  The current tick was
    // solved and committed above from the same state snapshot.
  }

  void publishFormalReference(
      const TargetSnapshot& target, double base_state_age, ShadowState state,
      bool reference_valid, bool emergency_dynamics_used, double task_scale,
      const DecisionVector& ddq) {
    std::lock_guard<std::mutex> terminal_lock(terminal_publication_mutex_);
    if (terminal_stop_requested_.load(std::memory_order_acquire) ||
        !enable_takeover_ || !formal_reference_pub_) {
      return;
    }
    wheel_arm_abs_vel::WholeBodyIkReference message;
    populateDecisionLayout(&message);
    message.header.seq = formal_output_sequence_++;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = world_frame_name_;
    message.session_id = target.session_id;
    message.generation = target.generation;
    message.reference_seq = reference_sequence_;
    message.execution_clock_domain = execution_clock_domain_;
    message.producer_instance = execution_instance_;
    message.producer_lease_deadline_ns = execution_lease_.leaseUntil();
    message.execution_owner_instance = execution_lease_.owner();
    message.generated_monotonic_ns = velocity_ik_execution::executionSteadyNs();
    message.execution_epoch = execution_epoch_;
    message.recovery_anchor_seq = execution_anchor_seq_;
    message.stopping_acceleration.assign(braking_limit_.data(), braking_limit_.data() + braking_limit_.size());
    message.position_lower.assign(position_lower_.data(), position_lower_.data() + position_lower_.size());
    message.position_upper.assign(position_upper_.data(), position_upper_.data() + position_upper_.size());
    message.takeover_requested = true;
    const bool base_feedback_fresh =
        !enable_base_motion_ ||
        (std::isfinite(base_state_age) &&
         base_state_age <= base_state_timeout_);
    message.valid = reference_valid && session_anchor_.valid &&
                    base_feedback_fresh;
    const bool lifecycle_armed =
        target.control_state ==
        wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_ARMED;
    message.takeover_active = reference_initialized_ && takeover_started_ && !lifecycle_armed;
    if (!reference_initialized_ || lifecycle_armed) {
      message.authority_state =
          wheel_arm_abs_vel::WholeBodyIkReference::AUTHORITY_ARMED;
    } else if (state == ShadowState::kTracking) {
      message.authority_state =
          wheel_arm_abs_vel::WholeBodyIkReference::AUTHORITY_ACTIVE;
    } else if (state == ShadowState::kPreparing) {
      message.authority_state =
          wheel_arm_abs_vel::WholeBodyIkReference::AUTHORITY_ACTIVE;
    } else if (state == ShadowState::kHold) {
      message.authority_state =
          wheel_arm_abs_vel::WholeBodyIkReference::AUTHORITY_HOLD;
    } else {
      message.authority_state =
          wheel_arm_abs_vel::WholeBodyIkReference::AUTHORITY_BRAKING;
    }
    message.world_frame = world_frame_name_;
    message.session_frame = session_frame_name_;
    message.session_anchor_valid = session_anchor_.valid;
    message.base_feedback_valid = base_feedback_fresh;
    message.q.resize(static_cast<std::size_t>(decisionDofCount()));
    message.dq.resize(static_cast<std::size_t>(decisionDofCount()));
    message.ddq.resize(static_cast<std::size_t>(decisionDofCount()));
    for (int i = 0; i < decisionDofCount(); ++i) {
      message.q.at(static_cast<std::size_t>(i)) = reference_q_(i);
      message.dq.at(static_cast<std::size_t>(i)) = reference_dq_(i);
      message.ddq.at(static_cast<std::size_t>(i)) = ddq(i);
    }
    message.task_scale = task_scale;
    message.safety_braking =
        state == ShadowState::kStaleBraking ||
        state == ShadowState::kSafetyBraking ||
        state == ShadowState::kModeBlocked ||
        state == ShadowState::kQpError;
    message.emergency_dynamics_used = emergency_dynamics_used;
    velocity_ik::runtime_probe::point(80);
    last_formal_publish_begin_ns_=monotonicNs();
    if(reference_transport_.shm()){
      if(!reference_transport_.writeReference(message)){
        ROS_ERROR_THROTTLE(1.,"[WholeBodyReferenceTransport] reference SHM write rejected");
        return; // no ROS fallback; existing owner's freshness/stop contract applies
      }
      // Recording mirror is asynchronous and is never an execution input.
      if (message.header.seq % formal_reference_mirror_stride_ == 0 &&
          formal_reference_pub_.getNumSubscribers() > 0) {
        const auto publisher=formal_reference_pub_;
        diagnostics_.submit([publisher,message]{publisher.publish(message);});
      }
    }else formal_reference_pub_.publish(message);
    last_formal_publish_end_ns_=monotonicNs();
    velocity_ik::runtime_probe::point(81);
  }

  void publishVector(const ros::Publisher& publisher, const DecisionVector& value) const {
    if (!diagnostic_publish_due_ || publisher.getNumSubscribers() == 0) return;
    std_msgs::Float64MultiArray message;
    message.data.resize(value.size());
    for (Eigen::Index i = 0; i < value.size(); ++i) {
      message.data.at(i) = value(i);
    }
    diagnostics_.submit([publisher,message]{publisher.publish(message);});
  }

  bool confirmReacquire(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    response.success=confirm_ready_.load() && !terminal_stop_requested_.load();
    if (response.success) confirm_requested_=true;
    response.message=response.success
        ? "Confirmation queued; control tick rechecks fresh input, standstill and authority before slow acquisition"
        : "Not ready: require latched VR loss, stable ACTIVE input, valid mode and stopped robot";
    return true;
  }

  void publishReference(const TargetSnapshot& target,
                        const TargetSnapshot& raw_world_target,
                        const std::string& source_target_frame,
                        double target_age, double base_state_age,
                        bool base_coordinate_jump,
                        uint64_t base_coordinate_epoch,
                        const DecisionVector& measured_q,
                        double cycle_dt,
                        double solve_time_us, ShadowState state, bool reference_valid,
                        bool solver_valid, int qp_return_code,
                        bool emergency_acceleration_used,
                        double candidate_age, uint64_t solver_base_seq,
                        uint8_t candidate_decision,
                        uint8_t applied_command_mode,
                        uint64_t candidate_session_id,
                        uint64_t candidate_generation,
                        uint64_t candidate_target_seq,
                        uint32_t candidate_reference_lag,
                        uint64_t candidate_target_lag,
                        const DecisionVector& dq, const DecisionVector& ddq,
                        const DecisionVector& task_dq,
                        const DecisionVector& primary_dq,
                        const DecisionVector& secondary_dq,
                        const DecisionVector& jerk_slack,
                        const DecisionVector& boundary_brake,
                        const DecisionVector& operational_boundary_brake,
                        const DecisionMatrix& primary_task_jacobian,
                        const DecisionVector& primary_task_velocity,
                        double task_scale,
                        double task_scaling_residual,
                        const DecisionVector& task_scaling_baseline,
                        const DecisionVector& task_scaling_unscaled,
                        bool task_scaling_qp_valid,
                        double torso_viability_relaxation,
                        double torso_task_scale,
                        double base_intent_task_scale,
                        double torso_task_error,
                        double base_intent_error,
                        bool root_translation_intent_active,
                        const std::array<double, kArmCount>& sigma_min,
                        const std::array<double, kArmCount>& translation_sigma,
                        const std::array<bool, kArmCount>&
                            singularity_escape_constraint_active,
                        bool singularity_escape_constraint_fallback,
                        const std::array<double, kArmCount>& primary_regularization,
                        const std::array<ArmPressure, kArmCount>& arm_pressure,
                        const Recruitment& recruitment_target,
                        const std::array<double, kLowerBodyDof>&
                            lower_joint_recruitment_target,
                        const std::array<double, kLowerBodyDof>&
                            lower_joint_directional_effectiveness,
                        double geometric_reach_demand,
                        double solver_lower_body_recruitment,
                        const std::array<double, kLowerBodyDof>&
                            solver_lower_joint_recruitment,
                        double solver_base_recruitment,
                        double arm_only_task_residual,
                        double arm_lower_task_residual,
                        double lower_body_saturation,
                        double primary_task_scale,
                        const std::array<double, kArmCount>&
                            translation_task_scale,
                        double supervisor_lower_body_recruitment,
                        double supervisor_base_recruitment,
                        const std::array<double, kArmCount>& orientation_task_scale,
                        const std::array<double, kArmCount>& position_task_weight_scale,
                        const std::array<double, 6>& task_error,
                        int primary_task_rank,
                        int nullspace_dimension,
                        double nullspace_residual,
                        double primary_task_leakage,
                        int secondary_return_code) {
    if (!diagnostic_publish_due_ ||
        (reference_pub_.getNumSubscribers() == 0 && mapper_status_pub_.getNumSubscribers() == 0)) return;
    wheel_arm_abs_vel::VelocityIkShadow message;
    populateDecisionLayout(&message);
    resizeReferenceDecisionFields(&message);
    message.header.seq = output_sequence_++;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = world_frame_name_;
    message.source_stamp = target.source_stamp;
    message.target_seq = target.sequence;
    message.session_id = target.session_id;
    message.generation = target.generation;
    message.control_state = target.control_state;
    message.target_age = std::isfinite(target_age) ? target_age : -1.0;
    message.cycle_dt = cycle_dt;
    message.solve_time_us = solve_time_us;
    message.state = static_cast<uint8_t>(state);
    message.valid = reference_valid;
    message.solver_valid = solver_valid;
    message.primary_qp_return_code = qp_return_code;
    message.fallback_active = fallback_active_;
    message.emergency_acceleration_used = emergency_acceleration_used;
    message.takeover_active = enable_takeover_ && reference_initialized_;
    message.consecutive_solver_failures = consecutive_solver_failures_;
    message.solver_base_seq = solver_base_seq;
    message.candidate_age = candidate_age;
    message.candidate_decision = candidate_decision;
    message.applied_command_mode = applied_command_mode;
    message.candidate_session_id = candidate_session_id;
    message.candidate_generation = candidate_generation;
    message.candidate_target_seq = candidate_target_seq;
    message.candidate_reference_lag = candidate_reference_lag;
    message.candidate_target_lag = candidate_target_lag;
    message.world_frame = world_frame_name_;
    message.source_target_frame = source_target_frame;
    message.session_frame = session_frame_name_;
    message.session_anchor_valid = session_anchor_.valid && target.valid;
    message.anchor_session_id = session_anchor_.session_id;
    message.anchor_generation = session_anchor_.generation;
    message.anchor_base_coordinate_epoch =
        session_anchor_.base_coordinate_epoch;
    message.base_feedback_valid =
        !enable_base_motion_ ||
        (std::isfinite(base_state_age) &&
         base_state_age <= base_state_timeout_);
    message.base_state_age =
        std::isfinite(base_state_age) ? base_state_age : -1.0;
    message.base_coordinate_jump_detected = base_coordinate_jump;
    message.base_coordinate_epoch = base_coordinate_epoch;
    message.base_recruitment_locked =
        !enable_base_motion_ || !enable_continuous_recruitment_;
    message.base_reference_position_from_feedback = !enable_takeover_;
    message.measured_base_pose = planarBasePose(measured_q);
    message.reference_base_pose = planarBasePose(reference_q_);
    message.session_anchor_pose = sessionAnchorPose();
    message.body_intent_valid = target.body_intent.torso_valid;
    message.body_tracking_frame = target.body.frame_id;
    message.body_coordinate_epoch = target.body.coordinate_epoch;
    message.raw_root_pose = toPose(
        target.body.root_position, target.body.root_orientation);
    message.raw_chest_pose = toPose(
        target.body.chest_position, target.body.chest_orientation);
    message.root_translation_intent.at(0) =
        target.body_intent.root_translation.x();
    message.root_translation_intent.at(1) =
        target.body_intent.root_translation.y();
    message.root_translation_confidence =
        target.body_intent.root_translation_confidence;
    message.root_translation_intent_active =
        root_translation_intent_active;
    message.relative_hand_motion =
        target.body_intent.relative_hand_motion;
    message.reach_assist_demand =
        target.body_intent.reach_assist_demand;
    for (int index = 0; index < kTorsoConstraintCount; ++index) {
      message.desired_torso_feature.at(index) =
          target.body_intent.desired_torso_feature(index);
    }
    message.torso_task_error = torso_task_error;
    message.torso_task_scale = torso_task_scale;
    message.base_intent_error = base_intent_error;
    message.base_intent_task_scale = base_intent_task_scale;
    const bool reach_assist = enable_base_motion_ &&
        (geometric_reach_demand > 1.0e-3 ||
         arm_lower_task_residual > cascade_base_residual_start_);
    message.base_recruitment_reason =
        root_translation_intent_active && reach_assist
            ? wheel_arm_abs_vel::VelocityIkShadow::BASE_REASON_BOTH
            : (root_translation_intent_active
                   ? wheel_arm_abs_vel::VelocityIkShadow::
                         BASE_REASON_ROOT_INTENT
                   : (reach_assist
                          ? wheel_arm_abs_vel::VelocityIkShadow::
                                BASE_REASON_REACH_ASSIST
                          : wheel_arm_abs_vel::VelocityIkShadow::
                                BASE_REASON_NONE));
    message.task_contract_version = 2;
    message.active_dof_count = 18;
    message.reach_assist_blend = 0.0;
    message.qp_reference_q.resize(decisionDofCount());
    for (int k = 0; k < decisionDofCount(); ++k) {
      message.qp_reference_q.at(k) = reference_q_(k) -
          cycle_dt * reference_dq_(k) + 0.5 * cycle_dt * cycle_dt * reference_ddq_(k);
    }
    message.cartesian_governor_initialized =
        false;
    const Eigen::Vector2d measured_base(
        measured_q(baseDecisionIndex(0)),
        measured_q(baseDecisionIndex(1)));
    const Eigen::Vector2d reference_base(
        reference_q_(baseDecisionIndex(0)),
        reference_q_(baseDecisionIndex(1)));
    message.base_translation_tracking_error =
        (reference_base - measured_base).norm();
    message.base_yaw_tracking_error = std::abs(shortestAngularDelta(
        measured_q(baseDecisionIndex(2)),
        reference_q_(baseDecisionIndex(2))));
    const std::array<double, kBaseDof> half_range{
        base_session_x_half_range_, base_session_y_half_range_,
        base_session_yaw_half_range_};
    const std::array<double, kBaseDof> anchor_value{
        session_anchor_.translation.x(), session_anchor_.translation.y(),
        session_anchor_.yaw};
    for (int dof = 0; dof < kBaseDof; ++dof) {
      const double value = reference_q_(baseDecisionIndex(dof));
      message.base_session_lower_margin.at(dof) =
          value - (anchor_value.at(dof) - half_range.at(dof));
      message.base_session_upper_margin.at(dof) =
          anchor_value.at(dof) + half_range.at(dof) - value;
    }
    message.primary_task_rank =
        static_cast<uint32_t>(std::max(0, primary_task_rank));
    message.nullspace_dimension =
        static_cast<uint32_t>(std::max(0, nullspace_dimension));
    message.nullspace_residual = nullspace_residual;
    message.primary_task_leakage = primary_task_leakage;
    message.secondary_qp_return_code = secondary_return_code;
    const auto group_velocity_norm = groupVelocityNorm(dq);
    for (std::size_t group = 0; group < group_velocity_norm.size(); ++group) {
      message.group_velocity_norm.at(group) =
          group_velocity_norm.at(group);
    }
    message.continuous_recruitment_enabled =
        enable_continuous_recruitment_;
    message.workspace_projection_applied_to_primary = false;
    message.whole_body_pressure = recruitment_target.pressure;
    message.lower_body_recruitment_target =
        recruitment_target.lower_body;
    message.base_recruitment_target = recruitment_target.base;
    message.lower_body_recruitment =
        active_lower_body_recruitment_;
    message.base_recruitment = active_base_recruitment_;
    message.solver_lower_body_recruitment =
        solver_lower_body_recruitment;
    message.solver_base_recruitment = solver_base_recruitment;
    message.arm_only_task_residual = arm_only_task_residual;
    message.arm_lower_task_residual = arm_lower_task_residual;
    message.lower_body_saturation = lower_body_saturation;
    message.primary_task_scale = primary_task_scale;
    for (int arm = 0; arm < kArmCount; ++arm) {
      message.translation_task_scale.at(arm) =
          translation_task_scale.at(arm);
    }
    for (int joint = 0; joint < kLowerBodyDof; ++joint) {
      message.lower_joint_recruitment_target.at(joint) =
          lower_joint_recruitment_target.at(joint);
      message.lower_joint_recruitment.at(joint) =
          active_lower_joint_recruitment_.at(joint);
      message.solver_lower_joint_recruitment.at(joint) =
          solver_lower_joint_recruitment.at(joint);
      message.lower_joint_directional_effectiveness.at(joint) =
          lower_joint_directional_effectiveness.at(joint);
    }
    message.geometric_reach_demand = geometric_reach_demand;
    message.base_admission_dwell = base_admission_elapsed_;
    message.base_admission_active = base_admission_active_;
    message.cascade_stage = static_cast<uint8_t>(std::clamp(
        cascade_stage_, 0, kLowerBodyDof + 1));
    for (int joint = 0; joint < kLowerBodyDof; ++joint) {
      message.lower_joint_stage_active.at(joint) =
          lower_joint_stage_active_.at(joint);
    }
    message.preparation_active = preparation_active_;
    message.preparation_complete = preparation_complete_;
    message.preparation_progress = preparation_progress_;
    message.startup_bootstrap_active = startup_bootstrap_active_;
    message.supervisor_lower_body_recruitment =
        supervisor_lower_body_recruitment;
    message.supervisor_base_recruitment =
        supervisor_base_recruitment;
    message.dynamic_supervisor_valid = dynamic_supervisor_valid_;
    message.dynamic_fault_latched = dynamic_fault_latched_;
    message.arm_reference_tracking_error =
        arm_reference_tracking_error_;
    for (int arm = 0; arm < kArmCount; ++arm) {
      message.arm_reference_tracking_error_by_arm.at(arm) =
          arm_reference_tracking_error_by_arm_.at(arm);
    }
    message.lower_reference_tracking_error =
        lower_reference_tracking_error_;
    message.reference_tracking_scale = reference_tracking_scale_;
    message.reference_tracking_soft_limited =
        reference_tracking_soft_limited_;
    message.reference_tracking_fault_latched =
        reference_tracking_fault_latched_;
    message.reference_tracking_recovery_elapsed =
        reference_tracking_recovery_elapsed_;
    message.singularity_escape_constraint_fallback =
        singularity_escape_constraint_fallback;
    message.task_scale = task_scale;
    message.task_scaling_active = primary_task_velocity.size() > 0 &&
                                  task_scale < 1.0 - 1.0e-6;
    message.task_scaling_qp_valid = task_scaling_qp_valid;
    message.torso_viability_relaxed = torso_viability_relaxation > 1.0e-10;
    message.torso_viability_relaxation = torso_viability_relaxation;
    message.task_scaling_residual = task_scaling_residual;
    const DecisionVector achieved_task_velocity =
        primary_task_jacobian.cols() == dq.size()
            ? primary_task_jacobian * dq
            : DecisionVector();
    const std::size_t task_count = static_cast<std::size_t>(
        primary_task_velocity.size());
    message.unscaled_task_twist.resize(task_count);
    message.requested_task_twist.resize(task_count);
    message.task_scaling_baseline_twist.resize(task_count);
    message.scaled_task_twist.resize(task_count);
    message.achieved_task_twist.resize(task_count);
    for (std::size_t i = 0; i < task_count; ++i) {
      const double unscaled =
          task_scaling_unscaled.size() == primary_task_velocity.size()
              ? task_scaling_unscaled(i)
              : primary_task_velocity(i);
      message.unscaled_task_twist.at(i) = unscaled;
      message.requested_task_twist.at(i) = primary_task_velocity(i);
      const double baseline =
          task_scaling_baseline.size() == primary_task_velocity.size()
              ? task_scaling_baseline(i)
              : 0.0;
      message.task_scaling_baseline_twist.at(i) = baseline;
      message.scaled_task_twist.at(i) = baseline + task_scale *
          (unscaled - baseline);
      message.achieved_task_twist.at(i) =
          achieved_task_velocity.size() == primary_task_velocity.size()
              ? achieved_task_velocity(i)
              : 0.0;
    }
    for (int i = 0; i < decisionDofCount(); ++i) {
      message.q.at(i) = reference_q_(i);
      message.measured_q.at(i) = measured_q(i);
      message.dq.at(i) = dq(i);
      message.ddq.at(i) = ddq(i);
      message.task_dq.at(i) = task_dq(i);
      message.primary_dq.at(i) = primary_dq(i);
      message.secondary_dq.at(i) = secondary_dq(i);
      message.jerk_slack.at(i) = jerk_slack(i);
      message.boundary_brake.at(i) = boundary_brake(i);
      message.operational_boundary_brake.at(i) =
          operational_boundary_brake(i);
    }
    for (int arm = 0; arm < kArmCount; ++arm) {
      message.reach_pressure.at(arm) = arm_pressure.at(arm).reach;
      message.singularity_pressure.at(arm) =
          arm_pressure.at(arm).singularity;
      message.joint_pressure.at(arm) = arm_pressure.at(arm).joint;
      message.arm_pressure.at(arm) = arm_pressure.at(arm).total;
      message.sigma_min.at(arm) = sigma_min.at(arm);
      message.translation_sigma.at(arm) = translation_sigma.at(arm);
      message.singularity_escape_constraint_active.at(arm) =
          singularity_escape_constraint_active.at(arm);
      message.primary_regularization.at(arm) =
          primary_regularization.at(arm);
      message.orientation_task_scale.at(arm) =
          orientation_task_scale.at(arm);
      message.position_task_weight_scale.at(arm) =
          position_task_weight_scale.at(arm);
    }
    for (int i = 0; i < 6; ++i) {
      message.task_error.at(i) = task_error.at(i);
    }
    const DecisionVector snapshot_q=reference_q_;
    auto finish=[this,message=std::move(message),target,raw_world_target,measured_q,snapshot_q]() mutable {
    const Eigen::Vector4d measured_torso = torsoFeatureInContext(
        measured_q, diagnostic_context_.get(), &diagnostic_q_full_);
    for (int arm = 0; arm < kArmCount; ++arm) {
      message.torso_relative_tcp_pose.at(arm) = toPose(
          target.arm.at(arm).hand_position, target.arm.at(arm).hand_orientation);
      message.raw_world_tcp_pose.at(arm) = toPose(
          raw_world_target.arm.at(arm).hand_position,
          raw_world_target.arm.at(arm).hand_orientation);
      message.current_torso_world_tcp_pose.at(arm) = message.raw_world_tcp_pose.at(arm);
      message.resolved_world_tcp_pose.at(arm) = message.raw_world_tcp_pose.at(arm);
      message.reach_assist_world_tcp_pose.at(arm) = message.raw_world_tcp_pose.at(arm);
      const auto measured_pose = plant_->GetFrameByName(hand_frame_names_.at(arm))
          .CalcPose(*diagnostic_context_, plant_->GetFrameByName(torso_frame_name_));
      message.measured_torso_relative_tcp_pose.at(arm) = toPose(
          measured_pose.translation(), Eigen::Quaterniond(measured_pose.rotation().matrix()));
      message.measured_task_error.at(3 * arm) =
          (target.arm.at(arm).hand_position - measured_pose.translation()).norm();
      message.measured_task_error.at(3 * arm + 1) = rotationLog(
          target.arm.at(arm).hand_orientation.toRotationMatrix() *
          measured_pose.rotation().matrix().transpose()).norm();
      const auto measured_elbow=plant_->GetFrameByName(elbow_frame_names_.at(arm))
          .CalcPose(*diagnostic_context_,plant_->GetFrameByName(torso_frame_name_));
      message.measured_task_error.at(3*arm+2)=target.valid&&target.arm.at(arm).elbow_valid
          ?(target.arm.at(arm).elbow_position-measured_elbow.translation()).norm()
          :std::numeric_limits<double>::quiet_NaN();
      message.hand_task_active.at(arm) = armIsControlled(arm) && target.valid;
      message.hand_orientation_weight.at(arm) =
          message.hand_task_active.at(arm) ? hand_orientation_weight_ : 0.0;
      message.cartesian_governor_linear_speed.at(arm) =
          target.arm.at(arm).hand_linear_ff.norm();
      message.cartesian_governor_angular_speed.at(arm) =
          target.arm.at(arm).hand_angular_ff.norm();
    }
    const Eigen::Vector4d reference_torso = torsoFeatureInContext(
        snapshot_q, diagnostic_context_.get(), &diagnostic_q_full_);
    for (int arm = 0; arm < kArmCount; ++arm) {
      const auto pose = plant_->GetFrameByName(hand_frame_names_.at(arm))
          .CalcPose(*diagnostic_context_, plant_->GetFrameByName(torso_frame_name_));
      message.reference_torso_relative_tcp_pose.at(arm) = toPose(
          pose.translation(), Eigen::Quaterniond(pose.rotation().matrix()));
    }
    for (int k = 0; k < kTorsoConstraintCount; ++k) {
      message.measured_torso_feature.at(k) = measured_torso(k);
      message.reference_torso_feature.at(k) = reference_torso(k);
    }
      if (mapper_status_pub_.getNumSubscribers() > 0) {
        mapper_status_pub_.publish(wheel_arm_abs_vel::makeMapperStatus(message));
      }
      reference_pub_.publish(message);
    };
    if(diagnostics_async_)diagnostics_.submit(std::move(finish));else finish();
  }

  void publishStdVector(const ros::Publisher& publisher, const std::vector<double>& value) const {
    std_msgs::Float64MultiArray message;
    message.data = value;
    diagnostics_.submit([publisher,message]{publisher.publish(message);});
  }

  static double monotonicNs(){return std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  velocity_ik::AsyncDiagnostics diagnostics_;
  bool diagnostics_async_=true;
  uint32_t hold_reason_previous_=~uint32_t(0), recovery_reason_previous_=~uint32_t(0);
  int hold_state_previous_=-1;
  double last_formal_publish_begin_ns_=0.0,last_formal_publish_end_ns_=0.0;
  velocity_ik::runtime_probe::Collector runtime_probe_;
  ros::Publisher cycle_timing_pub_;
  ros::Publisher continuity_pub_;
  ros::ServiceServer confirm_service_;
  std::atomic<bool> confirm_requested_{false}, confirm_ready_{false}, force_confirmation_{false};
  velocity_ik::InputContinuity input_continuity_;
  velocity_ik::Acquisition acquisition_;
  double acquisition_joint_speed_=.80, acquisition_linear_speed_=.40;
  double acquisition_angular_speed_=1.20, acquisition_acceleration_=3.00;
  double acquisition_position_tolerance_=.03, acquisition_rotation_tolerance_=.15;
  ros::Time sensor_source_stamp_;
  velocity_ik::MeasurementFreshness robot_measurement_freshness_;
  velocity_ik::MeasurementFreshness base_measurement_freshness_;
  uint32_t sensor_source_seq_=0;
  double sensor_receive_ns_=0,last_task_us_=0,last_bounds_us_=0;
  mutable double last_qp_finish_ns_=0;
  std::unique_ptr<velocity_ik::ModelContext> diagnostic_context_;
  Eigen::VectorXd diagnostic_q_full_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber target_sub_;
  ros::Subscriber robot_state_sub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber mode_sub_;
  ros::Publisher q_pub_;
  ros::Publisher dq_pub_;
  ros::Publisher ddq_pub_;
  ros::Publisher task_dq_pub_;
  ros::Publisher status_pub_;
  ros::Publisher task_error_pub_;
  ros::Publisher target_ff_pub_,channel_observation_pub_,adaptive_reference_pub_;
  ros::Publisher reference_dynamics_pub_;
  ros::Publisher innovation_reference_pub_;
  ros::Publisher startup_readiness_pub_;
  velocity_ik::StartupReadiness startup_readiness_;
  ros::Publisher input_debug_pub_;
  ros::Publisher feasibility_debug_pub_;
  ros::Publisher reference_pub_;
  ros::Publisher mapper_status_pub_;
  ros::Publisher formal_reference_pub_;

  DofLayout dof_layout_{DofLayout::WholeBodyStage()};
  ros::Publisher qp_stats_pub_;
  ros::Publisher envelope_diagnostics_pub_;
  ros::Publisher joint_jerk_pub_;
  ros::Publisher execution_governor_pub_;
  bool execution_governor_enabled_{false};
  std::array<int,kArmCount> governor_indices_{{-1,-1}};
  std::array<velocity_ik::ExecutionGovernorConfig,kArmCount> governor_config_;
  std::array<double,kArmCount> governor_risk_{{0.,0.}};
  bool operating_envelope_enabled_{false}, temporal_cost_enabled_{false};
  bool envelope_diagnostics_enabled_{false};
  int envelope_diagnostics_stride_{1};
  DecisionVector operating_extra_position_, operating_extra_time_;
  std::vector<velocity_ik::TemporalCost> temporal_cost_;
  bool joint_jerk_enabled_=false;
  DecisionVector joint_jerk_limits_;
#ifdef VELOCITY_IK_OFFLINE_TEST
  bool offline_clock_valid_=false;
  double offline_receive_ros_=0.,offline_receive_ns_=0.;
  bool offline_reject_jerk_solve_=false;
#endif
  mutable velocity_ik::WeightedQp weighted_qp_;
  std::unique_ptr<velocity_ik::ModelPinocchio> plant_;
  std::unique_ptr<velocity_ik::ModelContext> plant_context_;
  std::unique_ptr<velocity_ik::ModelContext> feasibility_context_;
  std::unique_ptr<velocity_ik::ModelContext> safety_context_;
  std::unique_ptr<velocity_ik::ModelContext> supervisor_context_;
  std::unique_ptr<velocity_ik::ModelContext> capture_context_;
  Eigen::VectorXd q_full_;
  Eigen::VectorXd feasibility_q_full_;
  Eigen::VectorXd safety_q_full_;
  Eigen::VectorXd supervisor_q_full_;
  Eigen::VectorXd capture_q_full_;
  std::vector<int> decision_position_index_;
  std::vector<int> decision_velocity_index_;
  std::vector<int> decision_sensor_index_;
  std::array<int, kBaseDof> base_decision_indices_{};
  std::array<int, kLowerBodyDof> lower_body_decision_indices_{};
  std::array<std::string, kArmCount> hand_frame_names_;
  std::array<std::string, kArmCount> elbow_frame_names_;
  std::array<std::string, kArmCount> shoulder_frame_names_;
  std::string base_frame_name_;
  std::string torso_frame_name_{"waist_yaw_link"};
  double torso_x_center_{0.0};
  double torso_z_center_{0.0};
  double torso_pitch_center_{0.0};
  double torso_yaw_center_{0.0};
  double torso_x_half_range_{0.18};
  double torso_z_half_range_{0.12};
  double torso_pitch_half_range_{20.0 * M_PI / 180.0};
  double torso_yaw_half_range_{45.0 * M_PI / 180.0};

  std::mutex data_mutex_;
  TargetSnapshot target_;
  velocity_ik::FixedPrepareTransaction fixed_prepare_transaction_;
  uint64_t target_sequence_counter_{0};
  uint64_t unique_sequence_counter_{0};
  ros::WallTime last_input_receive_time_;
  ros::Time last_input_source_stamp_;
  bool target_source_active_{false};
  bool source_interrupted_{false};
  uint64_t received_input_count_{0};
  uint64_t accepted_input_count_{0};
  uint64_t unique_input_count_{0};
  uint64_t duplicate_input_count_{0};
  uint64_t rejected_source_count_{0};
  uint64_t rejected_invalid_count_{0};
  uint32_t input_debug_sequence_{0};
  DecisionVector measured_q_;
  DecisionVector measured_dq_;
  // Sole temporal state of the dynamic QP. It advances only from committed
  // qdot commands and is separate from the measured-anchored q reference sent
  // to WBC during velocity tracking.
  ros::WallTime robot_state_receive_time_;
  bool robot_state_valid_{false};
  ros::WallTime base_state_receive_time_;
  bool base_state_valid_{false};
  bool base_yaw_initialized_{false};
  double last_base_wrapped_yaw_{0.0};
  double continuous_base_yaw_{0.0};
  Eigen::Vector2d last_base_position_{Eigen::Vector2d::Zero()};
  uint64_t base_coordinate_epoch_{0};
  bool base_coordinate_jump_detected_{false};
  bool arm_mode_known_{false};
  int current_arm_mode_{0};
  int target_arm_mode_{0};
  bool arm_mode_transitioning_{false};

  std::mutex feasibility_mutex_;
  std::condition_variable feasibility_condition_;
  std::thread feasibility_thread_;
  FeasibilityRequest feasibility_request_;
  bool feasibility_request_pending_{false};
  bool feasibility_stop_requested_{false};
  bool feasibility_running_{false};

  SessionAnchor session_anchor_;
  SessionBodyAnchor session_body_anchor_;
  BodyIntent last_body_intent_;
  // Frozen once per (session_id, generation).  The input is absolute in this
  // frame; the one-time alignment makes trigger enable bumpless without
  // introducing any continuously integrated Cartesian reference.
  SessionTargetAlignment session_target_alignment_;

  DecisionVector reference_q_;
  DecisionVector reference_dq_;
  DecisionVector reference_ddq_;
  DecisionVector position_lower_;
  DecisionVector position_upper_;
  DecisionVector joint_safe_range_ratio_;
  DecisionVector joint_safe_center_requested_;
  DecisionVector joint_safe_center_;
  DecisionVector joint_safe_lower_;
  DecisionVector joint_safe_upper_;
  DecisionVector posture_position_;
  // Per-session nullspace return pose.  This is deliberately separate from
  // posture_position_, which remains the fixed safety/escape center.
  DecisionVector session_posture_position_;
  DecisionVector preparation_position_;
  // Session-specific startup target: fixed ready arms plus the nearest safe
  // lower-body posture, avoiding an unrelated move to every safe centre.
  DecisionVector startup_preparation_position_;
  std::vector<double> preparation_arm_position_requested_{
      0.2617993878, 0.0, 0.0, -1.20, 0.0, 0.0, 0.0,
      0.2617993878, 0.0, 0.0, -1.20, 0.0, 0.0, 0.0};
  DecisionVector velocity_limit_;
  DecisionVector acceleration_limit_;
  bool host_target_reconstruction_{false};
  double target_adoption_lead_{.001};
  ros::Publisher reconstructed_target_pub_;
  DecisionVector braking_limit_, outward_acceleration_limit_;
  bool separate_stopping_contract_{false};
  std::string qp_snapshot_directory_;
  uint64_t qp_snapshot_counter_{0};
  bool reference_initialized_{false};
  bool fallback_active_{true};
  uint32_t consecutive_solver_failures_{0};
  int candidate_hold_cycles_{0};
  ros::Subscriber terminal_stop_sub_;
  std::atomic<bool> terminal_stop_requested_{false};
  std::mutex terminal_publication_mutex_;
  kuavo_reference_shm::RosTransport reference_transport_;
  ros::Subscriber execution_owner_sub_;
  wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr execution_feedback_;
  double execution_feedback_receive_ns_{0};
  const std::string execution_clock_domain_{velocity_ik_execution::executionClockDomain()};
  const std::string execution_instance_{velocity_ik_execution::executionInstance()};
  velocity_ik_execution::ClockedReferenceFreshness execution_feedback_freshness_{execution_clock_domain_};
  velocity_ik_execution::ExecutionProducerLease execution_lease_{execution_instance_};
  uint64_t execution_feedback_applied_seq_{0};
  int execution_reason_previous_{-1};
  uint64_t execution_epoch_{0}, execution_anchor_seq_{0};
  bool execution_waiting_{false}, takeover_started_{false};
  double tracking_ramp_scale_{0.0};
  double active_lower_body_recruitment_{0.0};
  // One-cycle delayed geometric admission signal.  It is produced by the
  // arm-only capability probe and joint/singularity pressure, never by human
  // hand displacement or target speed.
  double active_reach_assist_demand_{0.0};
  std::array<double, kLowerBodyDof> active_lower_joint_recruitment_{
      0.0, 0.0, 0.0, 0.0};
  std::array<bool, kLowerBodyDof> lower_joint_stage_active_{
      false, false, false, false};
  std::array<double, kLowerBodyDof> lower_joint_admission_elapsed_{
      0.0, 0.0, 0.0, 0.0};
  std::array<double, kLowerBodyDof> lower_joint_release_elapsed_{
      0.0, 0.0, 0.0, 0.0};
  int cascade_stage_{0};
  double active_base_recruitment_{0.0};
  double base_admission_elapsed_{0.0};
  double base_release_elapsed_{0.0};
  bool base_admission_active_{false};
  bool enable_natural_hang_bootstrap_{true};
  bool startup_bootstrap_active_{false};
  double natural_hang_arm_threshold_{0.08};
  bool preparation_active_{false};
  DecisionVector preparation_entry_dq_, preparation_entry_ddq_;
  double preparation_entry_elapsed_{0.0};
  double preparation_handoff_duration_{0.02};
  bool preparation_complete_{false};
  double preparation_initial_error_{0.0};
  double preparation_progress_{0.0};
  bool dynamic_supervisor_valid_{true};
  bool dynamic_fault_latched_{false};
  bool reference_tracking_soft_limited_{false};
  bool reference_tracking_fault_latched_{false};
  double reference_tracking_fault_elapsed_{0.0};
  double reference_tracking_recovery_elapsed_{0.0};
  double arm_reference_tracking_error_{0.0};
  double lower_reference_tracking_error_{0.0};
  std::array<double, kArmCount> arm_reference_tracking_error_by_arm_{
      0.0, 0.0};
  double reference_tracking_scale_{1.0};
  std::deque<ReferenceHistorySample> reference_history_;
  uint64_t reference_sequence_{0};
  ros::WallTime last_cycle_time_;
  uint32_t output_sequence_{0};
  uint32_t formal_output_sequence_{0};

  int robot_version_{63};
  int control_arm_index_{2};
  double solver_rate_hz_{500.0};
  double dt_{0.002};
  std::string target_topic_;
  std::string robot_state_topic_;
  std::string base_state_topic_{"/odom"};
  std::string world_frame_name_{"odom"};
  std::string session_frame_name_{"ik_session_base"};
  bool strict_base_state_frame_{true};
  bool allow_legacy_base_link_session_frame_{true};
  int sensor_arm_start_index_{-1};
  std::string mode_topic_;
  std::string output_prefix_;
  std::string formal_reference_topic_{"/whole_body_velocity_ik/reference"};
  bool enable_takeover_{false};
  bool require_external_arm_mode_{false};
  int required_arm_mode_{2};
  bool strict_target_frame_{true};
  TrackingSourceMode tracking_source_mode_{TrackingSourceMode::kController};
  std::string tracking_source_mode_name_{"controller"};
  bool publish_input_debug_{false};
  FeasibilityMode feasibility_mode_{FeasibilityMode::kActive};
  std::string feasibility_mode_name_{"active"};
  double nominal_arm_reach_{0.70937088};
  double selected_reach_reserve_{0.03};
  double workspace_projection_smoothing_{0.005};
  std::array<double, 3> feasibility_reach_reserve_candidates_{
      0.03, 0.04, 0.05};
  double feasibility_characteristic_length_{0.30};
  double feasibility_singularity_approach_threshold_{0.035};
  double feasibility_singularity_escape_threshold_{0.015};
  bool enable_singularity_escape_constraints_{true};
  double translation_singularity_activation_sigma_{0.060};
  double translation_singularity_safe_sigma_{0.035};
  double translation_singularity_escape_sigma_{0.015};
  double translation_singularity_gain_{3.0};
  double singularity_gradient_step_{0.002};
  double singularity_gradient_min_norm_{0.001};
  double natural_escape_speed_{0.40};
  double feasibility_joint_limit_approach_ratio_{0.15};
  double feasibility_joint_limit_escape_ratio_{0.05};
  double feasibility_escape_lookahead_{0.02};
  double feasibility_escape_joint_margin_weight_{0.05};
  double joint_limit_barrier_weight_{0.40};
  double joint_limit_barrier_gain_{0.80};
  double joint_safe_range_ratio_default_{0.80};
  double lower_body_safe_range_ratio_default_{0.60};
  bool use_legacy_shoulder_pitch_cap_{false};
  double legacy_shoulder_pitch_upper_{0.3};
  uint32_t feasibility_debug_sequence_{0};
  double duplicate_position_epsilon_{1.0e-10};
  double duplicate_orientation_epsilon_{1.0e-10};
  std::string left_controlled_frame_{"left_teleop_tcp"};
  std::string right_controlled_frame_{"right_teleop_tcp"};
  std::string body_tracking_frame_name_{"quest_tracking_robot_axes"};
  Eigen::Vector3d left_tcp_translation_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_tcp_translation_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d left_tcp_rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_tcp_rpy_{Eigen::Vector3d::Zero()};
  double target_timeout_{0.12};
  double robot_state_timeout_{0.10};
  double base_state_timeout_{0.10};
  double base_position_jump_threshold_{0.20};
  double base_yaw_jump_threshold_{0.35};
  double base_session_x_half_range_{0.30};
  double base_session_y_half_range_{0.30};
  double base_session_yaw_half_range_{0.35};
  double base_tracking_translation_limit_{0.15};
  double base_tracking_yaw_limit_{0.20};
  double ff_filter_time_constant_{0.030};
  double ff_decay_time_constant_{0.050};
  double ff_hold_time_{0.025};
  bool enable_body_mirroring_{true};
  bool enable_base_motion_{false};
  bool enable_root_translation_intent_{true};
  bool enable_root_yaw_intent_{false};
  double root_translation_deadband_{0.035};
  double root_translation_full_distance_{0.10};
  double root_translation_coherence_start_{0.03};
  double root_translation_coherence_full_{0.12};
  double root_translation_active_confidence_{0.20};
  double root_translation_release_error_{0.01};
  double root_translation_gain_{1.0};
  double torso_forward_gain_{0.50};
  double torso_height_gain_{0.50};
  double torso_pitch_gain_{0.75};
  double torso_yaw_gain_{0.7};
  Eigen::Vector3d torso_tracking_gain_{4.0, 5.0, 8.0};
  double torso_intent_weight_{5.0};
  double base_intent_gain_{2.0};
  double base_intent_weight_{5.0};
  double body_torso_deadband_{0.015};
  double body_torso_full_error_{0.15};
  double hand_position_gain_{4.0};
  double hand_orientation_gain_{6.0};
  double elbow_position_gain_{5.0};
  double hand_position_weight_{12.0};
  double hand_orientation_weight_{6.0};
  double elbow_position_weight_{0.15};
  double posture_weight_{0.010};
  double posture_gain_{0.20};
  double lower_body_posture_weight_{0.25};
  double lower_body_recruited_posture_weight_{0.05};
  double lower_body_posture_gain_{0.20};
  double primary_regularization_weight_{0.010};
  double primary_regularization_max_weight_{0.10};
  double lower_body_primary_regularization_weight_{1.0};
  double lower_body_recruited_regularization_weight_{0.010};
  double base_primary_regularization_weight_{1.0};
  double base_recruited_regularization_weight_{0.10};
  double nullspace_residual_tolerance_{1.0e-8};
  double global_nullspace_relative_tolerance_{1.0e-6};
  double secondary_correction_regularization_{1.0e-6};
  double base_damping_{1.0e-4};
  double singularity_damping_{0.12};
  double singularity_soft_threshold_{0.10};
  double singularity_hard_threshold_{0.025};
  bool enable_continuous_recruitment_{true};
  double geometric_probe_speed_{0.10};
  double recruitment_reach_start_distance_{0.60937088};
  double recruitment_reach_full_distance_{0.67937088};
  double recruitment_singularity_start_{0.035};
  double recruitment_singularity_full_{0.015};
  double recruitment_joint_margin_start_{0.25};
  double recruitment_joint_margin_full_{0.05};
  double lower_body_recruitment_rise_rate_{2.0};
  double lower_body_recruitment_fall_rate_{0.50};
  double base_recruitment_rise_rate_{1.0};
  double base_recruitment_fall_rate_{1.0};
  double base_admission_dwell_time_{0.40};
  double base_admission_on_threshold_{0.10};
  double base_admission_off_threshold_{0.03};
  std::array<double, kLowerBodyDof> lower_joint_stage_start_{
      0.90, 0.72, 0.48, 0.25};
  std::array<double, kLowerBodyDof> lower_joint_stage_full_{
      1.00, 0.94, 0.78, 0.55};
  std::array<double, kLowerBodyDof> lower_joint_effort_multiplier_{
      6.0, 3.0, 2.0, 1.0};
  std::array<double, kLowerBodyDof> lower_joint_cost_velocity_scale_{1.0, 1.0, 1.0, 1.0};
  std::array<double, kLowerBodyDof> preparation_lower_position_requested_{0.0, 0.0, 0.0, 0.0};
  // Relative to FK of the configured low ready pose, never the calibration
  // center or a moving measured pose. These are simulation workspace bounds.
  std::array<double, 4> torso_workspace_lower_offset_{-0.18, -0.22, -M_PI/6.0, -M_PI/2.0};
  std::array<double, 4> torso_workspace_upper_offset_{0.40, 0.34, 5.0*M_PI/18.0, M_PI/2.0};
  std::array<double, 4> torso_workspace_lower_{};
  std::array<double, 4> torso_workspace_upper_{};
  std::array<double, kLowerBodyDof> lower_joint_velocity_cap_{
      0.30, 0.45, 0.80, 2.00};
  std::array<double, kLowerBodyDof> lower_joint_acceleration_cap_{
      1.00, 1.50, 3.00, 8.00};
  double lower_directional_effectiveness_start_{0.02};
  double lower_directional_effectiveness_full_{0.15};
  double preparation_gain_{2.0};
  double preparation_arm_velocity_limit_{0.60};
  double preparation_position_tolerance_{0.025};
  double preparation_measured_position_tolerance_{0.060};
  double preparation_lower_position_tolerance_{0.008};
  double preparation_lower_measured_position_tolerance_{0.015};
  double preparation_velocity_tolerance_{0.030};
  int preparation_settle_cycles_required_{25};
  int preparation_settle_cycles_{0};
  double cascade_lower_residual_start_{0.015};
  double cascade_lower_residual_full_{0.060};
  double cascade_base_residual_start_{0.020};
  double cascade_base_residual_full_{0.080};
  double cascade_base_tracking_error_start_{0.040};
  double cascade_base_tracking_error_full_{0.100};
  bool continuous_reference_enabled_{true};
  std::array<uint64_t,4> continuous_reference_key_{{0,0,0,0}};
  std::string orientation_reference_mode_{"innovation"};
  double hand_position_cutoff_hz_{4.},hand_orientation_cutoff_hz_{3.};
  double hand_reference_frequency_{12.},hand_orientation_reference_frequency_{6.},torso_reference_frequency_{1.2};
  bool adaptive_reference_enabled_{true};
  velocity_ik::AdaptiveReferenceSettings adaptive_reference_settings_;
  velocity_ik::FramePredictionSettings frame_prediction_settings_;
  std::array<velocity_ik::FramePrediction,kArmCount> hand_prediction_,elbow_prediction_;
  std::array<velocity_ik::ReferenceInnovationAudit,kArmCount> hand_innovation_audit_,elbow_innovation_audit_;
  std::array<velocity_ik::AdaptiveVectorReference,kArmCount> hand_reference_,elbow_reference_;
  std::array<velocity_ik::AdaptiveRotationReference,kArmCount> hand_rotation_reference_;
  velocity_ik::ContinuousVector torso_reference_;
  double lower_body_return_start_{0.015};
  double lower_body_return_full_{0.10};
  double minimum_translation_progress_ratio_{0.02};
  double cascade_stage_dwell_time_{0.35};
  double cascade_stage_admission_threshold_{0.15};
  double cascade_stage_release_dwell_time_{0.50};
  double cascade_stage_release_threshold_{0.05};
  double cascade_minimum_active_recruitment_{0.20};
  double cascade_directional_skip_threshold_{0.01};
  double arm_reference_tracking_hard_limit_{0.30};
  double lower_reference_tracking_hard_limit_{0.20};
  double reference_tracking_fault_dwell_{0.10};
  double reference_tracking_delay_compensation_{0.025};
  double reference_tracking_delay_window_{0.035};
  double arm_reference_tracking_qp_limit_{0.12};
  double lower_reference_tracking_qp_limit_{0.08};
  double reference_tracking_recovery_arm_limit_{0.08};
  double reference_tracking_recovery_lower_limit_{0.05};
  double reference_tracking_recovery_dwell_{0.40};
  double kinematic_prediction_horizon_{0.015};
  double orientation_tracking_full_error_{0.06};
  double orientation_tracking_zero_error_{0.16};
  double orientation_joint_margin_zero_{0.02};
  double orientation_joint_margin_full_{0.15};
  double position_limit_margin_{0.01};
  double boundary_brake_guard_{0.02};
  double arm_boundary_execution_delay_proximal_{0.020};
  double arm_boundary_execution_delay_wrist_{0.045};
  double emergency_acceleration_scale_{2.0};
  double candidate_timeout_{0.0045};
  int max_candidate_reference_lag_cycles_{2};
  bool automatic_hold_enabled_{true};
  int max_candidate_hold_cycles_{3};
  double hold_velocity_threshold_{1.0e-3};
  double hold_acceleration_threshold_{2.0e-2};

  int qp_max_working_set_recalculations_{80};
  double formal_reference_mirror_rate_hz_{100.0};
  int formal_reference_mirror_stride_{5};
  bool diagnostic_publish_due_{false};
  double diagnostic_rate_hz_{100.0};
  int diagnostic_stride_{5};
  int diagnostic_counter_{0};
};

#ifndef VELOCITY_IK_OFFLINE_TEST
int main(int argc, char** argv) {
  ros::init(argc, argv, "absolute_velocity_ik_shadow");
  try {
    PinocchioVelocityIk node;
    node.run();
  } catch (const std::exception& error) {
    ROS_FATAL("[VelocityIkShadow] startup failed: %s", error.what());
    // A constructor failure can happen before rosout establishes its
    // connection.  Keep the same reason visible in roslaunch's screen output.
    std::cerr << "[VelocityIkShadow] startup failed: " << error.what()
              << std::endl;
    return 1;
  }
  return 0;
}
#endif
