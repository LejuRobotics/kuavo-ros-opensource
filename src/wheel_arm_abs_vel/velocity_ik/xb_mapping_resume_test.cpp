// Exercise the production session-transition path without control transport.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include <iostream>

struct VelocityIkOfflineTest {
  using Msg = wheel_arm_abs_vel::TeleopTcpTarget;
  using Target = PinocchioVelocityIk::TargetSnapshot;
  static void require(bool ok, const std::string& why) {
    if (!ok) throw std::runtime_error(why);
  }
  static void tick(PinocchioVelocityIk& n, Target& t) {
    const auto now = ros::Time::now();
    t.observation_ns.fill(n.monotonicNs());
    ++t.observation_sequence;
    t.receive_time = ros::WallTime::now();
    n.target_ = t;
    n.target_source_active_ = true;
    n.robot_measurement_freshness_.observe(now.toNSec(), now.toNSec(),
        n.monotonicNs(), n.robot_state_timeout_);
    n.publishOnce();
  }
  static void run(PinocchioVelocityIk& n, const std::string& scenario) {
    // Mode 0 prevents any solve/output takeover. The coordinate-contract path
    // still executes as in the real ARMED -> ACTIVE mode handshake.
    n.enable_takeover_ = false;
    n.robot_state_valid_ = n.arm_mode_known_ = true;
    n.current_arm_mode_ = n.target_arm_mode_ = 0;
    n.measured_q_ = n.preparation_position_;
    n.measured_dq_.setZero();
    n.reference_q_ = n.measured_q_;
    n.reference_dq_.setZero(); n.reference_ddq_.setZero();
    Target t;
    t.valid = true; t.source = Msg::SOURCE_QUEST_CONTROLLER;
    t.session_id = 2; t.generation = 2; t.control_state = Msg::CONTROL_ACTIVE;
    t.frame_id = "quest_chest_robot_axes";
    t.body.frame_id = n.body_tracking_frame_name_;
    t.body.chest_valid = true; t.body.coordinate_epoch = 1;
    const auto calibration = n.rotationFromRpy(Eigen::Vector3d(M_PI/2.,0.,M_PI/2.));
    t.body.chest_orientation = Eigen::Quaterniond(calibration);
    t.body.chest_position = Eigen::Vector3d(0.,0.,1.3);
    for (auto& arm : t.arm) arm.hand_orientation = Eigen::Quaterniond::Identity();
    tick(n,t);
    require(n.session_body_anchor_.valid, "first session not calibrated");
    const auto original_anchor = n.session_body_anchor_;
    t.body.chest_position.z() += .12;
    t.body.chest_orientation = Eigen::Quaterniond(
        n.rotationFromRpy(Eigen::Vector3d(0.,.18,.12))*calibration);
    const auto expected = n.calculateBodyIntent(t).desired_torso_feature;
    require(std::abs(expected(1)-original_anchor.robot_torso_feature(1))>.01,
        "fixture does not exercise changed human pose");
    t.control_state = Msg::CONTROL_HOLD; t.valid = false; ++t.generation;
    tick(n,t);
    if (scenario == "prepare") {
      t.control_state = Msg::CONTROL_PREPARE; t.valid = true;
      tick(n,t);
    }
    if (scenario == "source") t.source = Msg::SOURCE_QUEST_HAND_TRACKING;
    if (scenario == "epoch") ++t.body.coordinate_epoch;
    if (scenario == "base_epoch") ++n.base_coordinate_epoch_;
    if (scenario == "fault") n.reference_tracking_fault_latched_ = true;
    if (scenario == "unknown") t.source = Msg::SOURCE_UNKNOWN;
    // Changed robot posture while paused must not redefine the calibration.
    const int torso_joint = n.lowerBodyDecisionIndex(2);
    n.measured_q_(torso_joint) = std::clamp(n.measured_q_(torso_joint)+.04,
        n.joint_safe_lower_(torso_joint),n.joint_safe_upper_(torso_joint));
    t.valid = true; ++t.session_id; ++t.generation;
    t.control_state = Msg::CONTROL_ARMED;
    tick(n,t);
    auto actual = n.calculateBodyIntent(t);
    require(actual.torso_valid, "resumed body target invalid");
    if (scenario == "resume") {
      const double error = (actual.desired_torso_feature-expected).cwiseAbs().maxCoeff();
      std::cout << "same_pose_mapping_error=" << error << '\n';
      require(error < 1e-12, "X+B recaptured body mapping instead of retaining original calibration");
      t.control_state = Msg::CONTROL_ACTIVE; tick(n,t);
      // Repeated stop/resume must not accumulate calibration drift.
      for (int i=0;i<3;++i) {
        t.control_state=Msg::CONTROL_HOLD;t.valid=false;++t.generation;tick(n,t);
        t.control_state=Msg::CONTROL_ACTIVE;t.valid=true;++t.session_id;++t.generation;tick(n,t);
        require((n.calculateBodyIntent(t).desired_torso_feature-expected).norm()<1e-12,
            "repeated resume drift");
      }
      // Absolute map remains responsive, with the original configured gain.
      t.body.chest_position.z() += .08; tick(n,t);
      require(std::abs(n.calculateBodyIntent(t).desired_torso_feature(1)-expected(1)-
          .08*n.torso_height_gain_)<1e-12, "absolute height gain changed");
    } else {
      require((n.session_body_anchor_.human.chest_position-t.body.chest_position).norm()<1e-12,
          "new calibration required after " + scenario);
      require((actual.desired_torso_feature-expected).norm()>.01,
          "fixture failed to distinguish recapture after " + scenario);
    }
    require(n.reference_dq_.norm()==0. && n.reference_ddq_.norm()==0.,
        "mapping transition introduced reference velocity/acceleration");
    std::cout << "PASS " << scenario << '\n';
  }
};
int main(int argc,char**argv) {
  ros::init(argc,argv,"xb_mapping_resume_test",ros::init_options::NoSigintHandler);
  ros::NodeHandle p("~");
  p.setParam("robot_version",62);
  p.setParam("enable_natural_hang_bootstrap",false);
  try {
    for (const std::string scenario : {"resume","prepare","source","epoch","base_epoch","fault","unknown"}) {
      PinocchioVelocityIk n;
      VelocityIkOfflineTest::run(n,scenario);
    }
  } catch(const std::exception& e) { std::cerr << "FAIL " << e.what() << '\n'; return 1; }
}
