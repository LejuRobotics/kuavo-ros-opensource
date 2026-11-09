// Actual node regression; no controller, CAN or robot publishers are started.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"

struct VelocityIkOfflineTest {
  static void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
  }
  static void run(PinocchioVelocityIk& n, const std::string& kind) {
    n.use_controller_startup_arm_pose_ = true;
    n.offline_capture_messages_ = true;
    n.publishOnce();
    require(!n.controller_startup_arm_pose_applied_ && !n.reference_initialized_,
            "IK acquired authority before receiving its controller pose");
    auto pose = boost::make_shared<sensor_msgs::JointState>();
    pose->header.frame_id = "robot_version/" + std::to_string(n.robot_version_);
    pose->position.assign(14, 0.);
    for (int j = 0; j < 14; ++j) {
      const int index = n.armDecisionIndex(j/7, j%7);
      pose->name.push_back(n.dof_layout_.decisionNames().at(index));
    }
    for (int arm = 0; arm < 2; ++arm) {
      pose->position[7*arm] = kind == "qibeng" ? .5236 : .2618;
      pose->position[7*arm+3] = kind == "qibeng" ? -1.57 : -.5236;
    }
    if (kind == "custom") {
      pose->position[3] = -1.0;
      pose->position[10] = -.9;
    }
    const auto valid = *pose;
    pose->header.frame_id = "robot_version/invalid";
    n.controllerStartupArmPoseCallback(pose);
    require(!n.controller_startup_arm_pose_, "wrong robot version accepted");
    *pose = valid; pose->name[0] = pose->name[7];
    n.controllerStartupArmPoseCallback(pose);
    require(!n.controller_startup_arm_pose_, "wrong joint ordering accepted");
    *pose = valid; pose->position.pop_back();
    n.controllerStartupArmPoseCallback(pose);
    require(!n.controller_startup_arm_pose_, "partial pose accepted");
    *pose = valid; pose->position[3] = std::numeric_limits<double>::quiet_NaN();
    n.controllerStartupArmPoseCallback(pose);
    require(!n.controller_startup_arm_pose_, "nonfinite pose accepted");
    *pose = valid; pose->position[3] = -10.;
    n.controllerStartupArmPoseCallback(pose);
    require(!n.controller_startup_arm_pose_, "unsafe endpoint accepted");
    *pose = valid;
    n.controllerStartupArmPoseCallback(pose);
    require(bool(n.controller_startup_arm_pose_), "valid startup pose rejected");
    n.publishOnce();
    require(n.controller_startup_arm_pose_applied_, "startup pose not applied");
    for (int j = 0; j < 14; ++j) {
      const int index = n.armDecisionIndex(j/7, j%7);
      require(n.preparation_position_(index) == valid.position[j], "PREPARE changed the initial arm endpoint");
      require(n.startup_preparation_position_(index) == valid.position[j], "startup and reset arm endpoints differ");
    }
    const auto old = n.preparation_position_;
    auto changed = boost::make_shared<sensor_msgs::JointState>(valid);
    changed->position[3] = -.8;
    n.controllerStartupArmPoseCallback(changed);
    n.publishOnce();
    require(maxAbs(n.preparation_position_-old) == 0., "late configuration changed the latched endpoint");
    // Exercise the real X+A PREPARE path with ideal measured feedback, starting
    // from the generic arms. This must reach the controller's qibeng/custom
    // endpoint rather than the launch file's generic -30 degree elbows.
    n.reference_q_ = n.preparation_position_;
    for (int arm = 0; arm < 2; ++arm) {
      n.reference_q_(n.armDecisionIndex(arm, 0)) = .2618;
      n.reference_q_(n.armDecisionIndex(arm, 3)) = -.5236;
    }
    n.reference_dq_.setZero(); n.reference_ddq_.setZero();
    n.reference_initialized_ = n.takeover_started_ = n.enable_takeover_ = true;
    n.robot_state_valid_ = n.arm_mode_known_ = true;
    n.current_arm_mode_ = n.target_arm_mode_ = 2;
    n.arm_mode_transitioning_ = false;
    n.session_anchor_.valid = true;
    n.session_anchor_.session_id = n.session_anchor_.generation = 1;
    n.session_anchor_.base_coordinate_epoch = 0;
    using Lease = velocity_ik_execution::ExecutionProducerLease;
    Lease::Feedback feedback;
    feedback.producer = n.execution_instance_; feedback.owner = "offline-owner";
    feedback.valid = true; feedback.phase = Lease::Phase::Active;
    feedback.generated_ns = velocity_ik_execution::executionSteadyNs();
    feedback.lease_until_ns = feedback.generated_ns + 3600000000000LL;
    n.execution_lease_.observe(feedback, true, true);
    using Target = wheel_arm_abs_vel::TeleopTcpTarget;
    auto target = boost::make_shared<Target>();
    target->header.frame_id = "quest_chest_robot_axes";
    target->left_controlled_frame = n.left_controlled_frame_;
    target->right_controlled_frame = n.right_controlled_frame_;
    target->control_state = Target::CONTROL_PREPARE;
    target->control_valid = target->left_valid = target->right_valid = true;
    target->estimated_twists_valid = true;
    target->left_tcp_pose.orientation.w = target->right_tcp_pose.orientation.w = 1.;
    target->source = Target::SOURCE_QUEST_CONTROLLER;
    target->session_id = 1; target->generation = 2;
    target->body_tracking_frame = n.body_tracking_frame_name_;
    target->body_high_confidence = target->chest_valid = true;
    target->chest_pose.orientation.w = 1.;
    for (int k = 0; k < 2000; ++k) {
      n.measured_q_ = n.reference_q_; n.measured_dq_ = n.reference_dq_;
      const auto now = ros::Time::now();
      n.robot_measurement_freshness_.observe(now.toNSec(), now.toNSec(), n.monotonicNs(), n.robot_state_timeout_);
      target->header.seq = k+1; target->header.stamp.fromSec(100.+k*.002);
      target->mapper_publish_stamp = now;
      n.targetCallback(target);
      n.publishOnce();
      require(!n.dynamic_fault_latched_, "dynamic fault during PREPARE");
    }
    require(n.preparation_complete_, "PREPARE did not settle at the controller pose");
    for (int j = 0; j < 14; ++j) {
      const int index = n.armDecisionIndex(j/7, j%7);
      require(std::abs(n.reference_q_(index)-valid.position[j]) < .03,
              "executed PREPARE returned to the wrong arm posture");
    }
    std::cout << "PASS model=" << n.robot_version_ << " pose=" << kind << '\n';
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "absolute_velocity_ik", ros::init_options::NoSigintHandler);
  try {
    PinocchioVelocityIk node;
    VelocityIkOfflineTest::run(node, argc > 1 ? argv[1] : "qibeng");
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
