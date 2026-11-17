// Run with a private ROS master and the production 400062 launch parameters.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include <iostream>
#include <sys/shm.h>
#include <unistd.h>

struct VelocityIkOfflineTest {
  using Node = PinocchioVelocityIk;

  static void check(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
  }

  static void lowerBodyHandoff(Node& node) {
    using Bridge = velocity_ik::LowerBodyGripHandoff;
    Bridge bridge;
    const Eigen::Vector3d anchor(.1, .8, .2), goal(.3, 1., .4);
    check(Bridge::checkedDuration(.1)==1. && Bridge::checkedDuration(6.)==1. &&
          Bridge::checkedDuration(std::numeric_limits<double>::quiet_NaN())==1. &&
          Bridge::checkedDuration(.2)==.2 && Bridge::checkedDuration(5.)==5., "resume duration contract");
    bridge.step(1,1,true,anchor,goal,1.,1.);
    check((bridge.step(1,1,false,anchor,goal,2.,1.)-anchor).norm()<1.e-12,"grip resume first frame jumps");
    check((bridge.step(1,1,false,anchor,goal,2.5,1.)-(anchor+goal)*.5).norm()<1.e-12,"dev midpoint mismatch");
    check((bridge.step(1,1,false,anchor,goal,3.,1.)-goal).norm()<1.e-12,"resume did not reach live target");
    // A second hand joining while already following is not another resume.
    check((bridge.step(1,1,false,anchor,goal,3.1,1.)-goal).norm()<1.e-12,"other hand restarted bridge");
    const Eigen::Vector3d stopped(.15,.85,.25);
    bridge.step(1,1,true,stopped,goal,3.2,1.);
    check((bridge.step(1,1,false,stopped,goal,3.3,1.)-stopped).norm()<1.e-12,"rapid re-grip reused old anchor");
    check((bridge.step(1,2,false,stopped,goal,3.4,1.)-goal).norm()<1.e-12,"old generation retained bridge");
    bridge.reset();
    check((bridge.step(1,2,false,stopped,goal,3.5,1.)-goal).norm()<1.e-12,"reset retained bridge");

    // Call the actual production shaper: absolute is unchanged; incremental
    // x/height/pitch resumes from command FK and yaw is never grip-blended.
    node.reference_q_ = node.preparation_position_;
    const auto feature=node.torsoFeatureInContext(node.reference_q_,node.plant_context_.get(),&node.q_full_);
    for (bool incremental : {false,true}) {
      node.incremental_geometry_=incremental;
      node.lower_body_grip_handoff_.reset();node.torso_reference_={};
      Node::TargetSnapshot t;
      t.session_id=1;t.generation=5;
      t.torso_control_enabled=true;t.body_intent.torso_valid=true;
      t.body_intent.desired_torso_feature=feature;
      t.body_intent.desired_torso_velocity.setZero();
      t.torso_position_hold=true;
      auto shaped=t;node.shapeContinuousTarget(shaped,t,1.e9,.002);
      t.torso_position_hold=false;
      t.body_intent.desired_torso_feature.head<3>().array()+=.05;
      t.body_intent.desired_torso_feature[3]+=.1;
      shaped=t;node.shapeContinuousTarget(shaped,t,2.e9,.002);
      check(std::abs(shaped.body_intent.desired_torso_feature[0]-feature[0]-(incremental?0.:.05))<1.e-12,
            "production shaper missing bridge or changed absolute mode");
      for(int k=1;k<=500;++k){
        shaped=t;node.shapeContinuousTarget(shaped,t,2.e9+k*2.e6,.002);
        check(shaped.body_intent.desired_torso_feature.allFinite(),"invalid resumed torso feature");
        if(k==250 && incremental)
          check(std::abs(shaped.body_intent.desired_torso_feature[0]-feature[0]-.025)<1.e-12,"production midpoint mismatch");
      }
      check(std::abs(shaped.body_intent.desired_torso_feature[0]-feature[0]-.05)<1.e-12,"production endpoint mismatch");
      check(shaped.body_intent.desired_torso_feature[3]>feature[3]+.01,"grip handoff blocked yaw");
    }
    std::cout<<"PASS production lower-body re-grip bridge: first frame, 1 s curve, yaw, generations, re-grip, absolute isolation\n";
  }

  static void reachableRetraction(Node& node) {
    auto start = node.preparation_position_.eval();
    auto goal_q = start;
    const std::array<double, 7> from{{-.8, 0., 0., -.079, 0., 0., 0.}};
    const std::array<double, 7> to{{-.3, 0., 0., -1., 0., 0., .421}};
    for (int arm=0; arm<2; ++arm) for (int joint=0; joint<7; ++joint) {
      start[node.armDecisionIndex(arm,joint)] = from[joint];
      goal_q[node.armDecisionIndex(arm,joint)] = to[joint];
    }
    Node::SolverRequest request;
    request.target.valid = request.target.arm_target_torso_relative = true;
    request.tracking = request.measured_state_valid = request.catchup_correction = true;
    request.acquisition_blend = request.input_budget = 1.;
    request.cycle_dt = .002;
    node.incremental_geometry_ = true;
    node.setPlantDecisionPositions(goal_q);
    const auto& torso = node.plant_->GetFrameByName(node.torso_frame_name_);
    for (int arm=0; arm<2; ++arm) {
      const auto pose = node.plant_->GetFrameByName(node.hand_frame_names_[arm])
          .CalcPose(*node.plant_context_,torso);
      auto& goal = request.target.arm[arm];
      goal.hand_position = pose.translation();
      goal.hand_orientation = Eigen::Quaterniond(pose.rotation().matrix());
      goal.elbow_valid = true;
      goal.elbow_position = node.plant_->GetFrameByName(node.elbow_frame_names_[arm])
          .CalcPose(*node.plant_context_,torso).translation();
    }
    request.reference_q = start;
    request.reference_dq = request.reference_ddq = Eigen::VectorXd::Zero(start.size());
    for (int tick=0; tick<2500; ++tick) {
      request.measured_q = request.reference_q;
      request.measured_dq = request.reference_dq;
      const auto result = node.solveRequest(request);
      check(result.solver_valid && result.dynamic_certificate_valid,"retraction solve invalid");
      check((result.task_dq-result.certified_dynamic_lower).minCoeff()>=-1.e-6 &&
            (result.certified_dynamic_upper-result.task_dq).minCoeff()>=-1.e-6,
            "retraction exceeded certified motion bounds");
      request.reference_ddq = (result.task_dq-request.reference_dq)/request.cycle_dt;
      request.reference_q += .5*request.cycle_dt*(result.task_dq+request.reference_dq);
      request.reference_dq = result.task_dq;
    }
    node.setPlantDecisionPositions(request.reference_q);
    double worst = 0.;
    for (int arm=0; arm<2; ++arm) {
      const auto pose = node.plant_->GetFrameByName(node.hand_frame_names_[arm])
          .CalcPose(*node.plant_context_,torso);
      worst = std::max(worst,(pose.translation()-request.target.arm[arm].hand_position).norm());
      check(worst<.002,"reachable target stuck near extension");
      check(Eigen::Quaterniond(pose.rotation().matrix()).angularDistance(
          request.target.arm[arm].hand_orientation)<.01,"retraction lost reachable orientation");
    }
    std::cout << "PASS reachable retraction, both arms, error_mm=" << 1000.*worst << "\n";
  }

  static void run(Node& node, int key) {
    // The radius is a nonlinear function of J4. Verify its finite-step
    // retraction term against actual FK, including the URDF elbow offset.
    for (int sample=0;sample<6;++sample) {
      auto q=node.preparation_position_.eval();
      for(int a=0;a<2;++a)for(int j=0;j<7;++j) {
        const int i=node.armDecisionIndex(a,j);
        q[i]=std::clamp(q[i]+.12*std::sin(1.+sample+3*a+j),
                       node.position_lower_[i]+.1,node.position_upper_[i]-.1);
      }
      for(int a=0;a<2;++a) {
        const auto& torso=node.plant_->GetFrameByName(node.torso_frame_name_);
        const auto& wrist=node.plant_->GetFrameByName(a==0?"zarm_l6_link":"zarm_r6_link");
        const auto& tcp=node.plant_->GetFrameByName(node.hand_frame_names_[a]);
        const int index=node.armDecisionIndex(a,3);
        q[index]=node.position_upper_[index]-.08-.15*sample;
        auto goal_q=q;goal_q[index]=std::max(node.position_lower_[index]+.1,q[index]-.6);
        node.setPlantDecisionPositions(goal_q);
        const auto desired=tcp.CalcPose(*node.plant_context_,torso);
        Node::ArmTarget goal;
        goal.hand_position=desired.translation();
        goal.hand_orientation=Eigen::Quaterniond(desired.rotation().matrix());
        node.setPlantDecisionPositions(q);
        DecisionJacobian3d jac;Eigen::Vector3d error;
        node.incremental_geometry_=true;
        const bool active=node.incrementalRetractionTask(goal,q,a,&jac,&error);
        if(sample==0) check(active,"missing extended-arm retraction task");
        if(active) {
          check(std::abs(error[0]/jac(0,index)-(goal_q[index]-q[index]))<1.e-6,
                "elbow secant does not match full-model FK");
          check(error[0]<0. && jac(0,index)>0.,"retraction points toward extension boundary");
        }
        // Existing torso-relative wrist Jacobian must agree with FK; this
        // checks the model/frame convention independently of the secant task.
        const auto exact=node.frameTranslationJacobianRelative(a==0?"zarm_l6_link":"zarm_r6_link",node.torso_frame_name_);
        for(int i=0;i<q.size();++i) {
          auto plus=q,minus=q;plus[i]+=1.e-6;minus[i]-=1.e-6;
          node.setPlantDecisionPositions(plus);const Eigen::Vector3d p=wrist.CalcPose(*node.plant_context_,torso).translation();
          node.setPlantDecisionPositions(minus);const Eigen::Vector3d m=wrist.CalcPose(*node.plant_context_,torso).translation();
          check(((p-m)/(2.e-6)-exact.col(i)).norm()<2.e-6,"wrist Jacobian/frame mismatch");
        }
        node.setPlantDecisionPositions(q);
        node.incremental_geometry_=false;
        check(!node.incrementalRetractionTask(goal,q,a,&jac,&error),"absolute mode changed");
        node.incremental_geometry_=true;
        const Eigen::Vector3d held_wrist=wrist.CalcPose(*node.plant_context_,torso).translation();
        goal.hand_orientation=Eigen::Quaterniond(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ()));
        goal.hand_position=held_wrist-goal.hand_orientation*wrist.CalcPose(*node.plant_context_,tcp).translation();
        check(!node.incrementalRetractionTask(goal,q,a,&jac,&error),"pure tool rotation triggered retraction");
      }
    }
    std::cout<<"PASS incremental retraction: full-model secant, inward direction, tool lever, Jacobian finite differences, absolute isolation\n";
    reachableRetraction(node);
    const std::string topic = "/incremental_replay/reference";
    node.nh_.setParam(topic + "/transport", "shm");
    node.nh_.setParam(topic + "/shm_reference_key", key);
    node.nh_.setParam(topic + "/shm_owner_key", key + 1);
    node.reference_transport_.configure(node.nh_, topic, true);
    kuavo_reference_shm::RosTransport reader;
    reader.configure(node.nh_, topic, false);
    node.enable_takeover_ = node.reference_initialized_ = node.takeover_started_ = true;
    node.session_anchor_.valid = true;
    node.formal_reference_pub_ = node.nh_.advertise<wheel_arm_abs_vel::WholeBodyIkReference>(
        "/incremental_replay/mirror", 1);
    const double dt = .002;
    auto seed = node.preparation_position_.eval();
    node.setPlantDecisionPositions(seed);
    std::array<Node::ArmTarget, 2> origin;
    for (int arm = 0; arm < 2; ++arm) {
      const auto fk = node.plant_->GetFrameByName(node.hand_frame_names_[arm]).CalcPose(
          *node.plant_context_, node.plant_->GetFrameByName(node.torso_frame_name_));
      origin[arm].hand_position = fk.translation();
      origin[arm].hand_orientation = Eigen::Quaterniond(fk.rotation().matrix());
    }
    for (bool incremental : {false, true}) {
      node.incremental_geometry_=incremental;
      node.reference_q_ = seed;
      node.reference_dq_ = Eigen::VectorXd::Zero(seed.size());
      node.reference_ddq_ = node.reference_dq_;
      node.last_shaped_clutch_.fill(0);
      node.last_shaped_pause_.fill(0);
      Node::SolverRequest r;
      r.reference_q = seed;
      r.reference_dq = Eigen::VectorXd::Zero(seed.size());
      r.reference_ddq = r.reference_dq;
      r.tracking = r.measured_state_valid = true;
      r.target.valid = r.target.arm_target_torso_relative = true;
      r.acquisition_blend = r.input_budget = 1.;
      r.cycle_dt = dt;
      r.target.clutch_seq.fill(incremental ? 1 : 0);
      node.hand_reference_ = {};
      node.hand_rotation_reference_ = {};
      std::vector<double> times;
      unsigned valid = 0;
      double jerk = 0., error2 = 0., overshoot = 0.;
      std::vector<double> intervals;
      uint64_t first_stamp = 0, last_stamp = 0;
      const auto start = std::chrono::steady_clock::now();
      for (int tick = 0; tick < 1250; ++tick) {
        std::this_thread::sleep_until(start + std::chrono::microseconds(tick * 2000));
        r.base_reference_seq = tick;
        r.target.session_id = r.target.generation = 1;
        r.target.control_state = wheel_arm_abs_vel::IncrementalTeleopTcpTarget::CONTROL_ACTIVE;
        r.measured_q = r.reference_q;
        r.measured_dq = r.reference_dq;
        r.target.arm = origin;
        // Same 120-Hz held observation for both modes; the shared kernel owns
        // all 500-Hz interpolation/integration between those observations.
        const double input_time = std::floor(tick * dt * 120.) / 120.;
        const double distance = .05 * std::clamp(input_time - .5, 0., 1.);
        for (auto& arm : r.target.arm) arm.hand_position.x() += distance;
        auto shaped = r.target;
        node.shapeContinuousTarget(shaped, r.target, 0., dt);
        r.target = shaped;
        auto result = node.solveRequest(r);
        times.push_back(result.solve_time_us);
        if (!result.solver_valid) continue;
        ++valid;
        const auto acceleration = ((result.task_dq - r.reference_dq) / dt).eval();
        jerk = std::max(jerk, ((acceleration - r.reference_ddq) / dt).cwiseAbs().maxCoeff());
        r.reference_q += .5 * dt * (result.task_dq + r.reference_dq);
        r.reference_dq = result.task_dq;
        r.reference_ddq = acceleration;
        node.reference_q_ = r.reference_q;
        node.reference_dq_ = r.reference_dq;
        node.diagnostic_tick_ = tick;
        ++node.reference_sequence_;
        node.publishFormalReference(r.target, 0., ShadowState::kTracking, true, false, 1., acceleration);
        wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr packet;
        check(reader.readReference(packet), "formal SHM commit missing");
        check(packet->q.size() == static_cast<size_t>(seed.size()), "wrong SHM dimensions");
        for (int i = 0; i < seed.size(); ++i) {
          check(packet->q[i] == r.reference_q[i] && packet->dq[i] == r.reference_dq[i] &&
                packet->ddq[i] == acceleration[i], "incoherent SHM reference");
        }
        const auto stamp = packet->generated_monotonic_ns;
        if (!first_stamp) first_stamp = stamp;
        if (last_stamp) intervals.push_back((stamp - last_stamp) * 1.e-6);
        last_stamp = stamp;
        node.setPlantDecisionPositions(r.reference_q);
        const auto fk = node.plant_->GetFrameByName(node.hand_frame_names_[0]).CalcPose(
            *node.plant_context_, node.plant_->GetFrameByName(node.torso_frame_name_));
        const Eigen::Vector3d motion = fk.translation() - origin[0].hand_position;
        error2 += (motion - distance * Eigen::Vector3d::UnitX()).squaredNorm();
        if (tick >= 750) overshoot = std::max(overshoot, motion.x() - distance);
      }
      check(!intervals.empty(), "no SHM intervals");
      std::sort(intervals.begin(), intervals.end());
      std::sort(times.begin(), times.end());
      std::cout << "BENCH " << (incremental ? "incremental" : "absolute")
                << " samples=" << times.size() << " valid=" << valid
                << " p50_us=" << times[times.size()/2] << " p95_us=" << times[times.size()*95/100]
                << " shm_hz=" << 1.e9 * intervals.size() / (last_stamp - first_stamp)
                << " shm_interval_p99_ms=" << intervals[intervals.size()*99/100]
                << " jerk=" << jerk << " rms_mm=" << 1000*std::sqrt(error2/std::max(1u,valid))
                << " overshoot_mm=" << 1000*overshoot << std::endl;
      check(valid == times.size(), "invalid QP");
    }
    // Replay same-stamp button traffic into the real target subscriber.
    using Message = wheel_arm_abs_vel::IncrementalTeleopTcpTarget;
    Message::Ptr m(new Message);
    m->header.frame_id = "quest_chest_robot_axes";
    m->header.stamp = ros::Time::now();
    m->left_controlled_frame = node.left_controlled_frame_;
    m->right_controlled_frame = node.right_controlled_frame_;
    m->body_tracking_frame = node.body_tracking_frame_name_;
    m->body_coordinate_epoch = m->session_id = m->generation = 1;
    m->control_state = Message::CONTROL_ACTIVE;
    m->control_valid = m->left_valid = m->right_valid = m->estimated_twists_valid = true;
    m->chest_valid = true;
    m->source = Message::SOURCE_QUEST_CONTROLLER;
    m->chest_pose.orientation.w = m->left_tcp_pose.orientation.w = m->right_tcp_pose.orientation.w = 1.;
    m->mapper_publish_stamp = ros::Time::now();
    node.targetCallback(m);
    check(node.target_.valid, "initial target was rejected");
    const auto observation = node.target_.observation_ns;
    m->clutch_seq[0] = 2;
    m->clutch_hold[0] = true;
    node.targetCallback(m);
    check(node.target_.clutch_hold[0] && !node.target_.clutch_hold[1], "left hold not admitted");
    check(node.target_.observation_ns == observation, "clutch edge refreshed observations");
    m->clutch_seq[0] = 1;
    m->clutch_hold[0] = false;
    node.targetCallback(m);
    check(node.target_.clutch_hold[0], "stale operation unlocked left arm");
    m->clutch_seq[0] = 3;
    node.targetCallback(m);
    check(!node.target_.clutch_hold[0], "reengagement was rejected");
    const auto pitch_only = Node::torsoJointPermissions(true, false, false, true);
    check(pitch_only == std::array<double,4>{{0.,0.,1.,1.}}, "knee/leg hold changed waist axes");
    const auto prepare = Node::torsoJointPermissions(false, true, true, true);
    check(prepare == std::array<double,4>{{1.,1.,1.,1.}}, "PREPARE retained incremental locks");
    // A held Mapper goal may be far ahead of the executed command. A clutch
    // handoff must start from command FK, independent of measured lag/old goal.
    node.reference_q_ = seed;
    node.setPlantDecisionPositions(seed);
    Node::TargetSnapshot handoff;
    // Production passes the original source snapshot separately from the
    // torso-aligned target. Equal flags here would hide a world/torso reset.
    handoff.arm_target_torso_relative = false;
    handoff.arm = origin;
    handoff.clutch_seq.fill(30);
    handoff.clutch_hold[0] = true;
    handoff.arm[0].hand_position.x() += .2;
    auto held = handoff;
    held.arm_target_torso_relative = true;
    node.shapeContinuousTarget(held, handoff, 0., dt);
    check((held.arm[0].hand_position-origin[0].hand_position).norm()<1.e-12,
          "held reference inherited unachieved Mapper goal");
    check(held.arm[0].hand_linear_ff.norm()<1.e-12,"hold released reference residual velocity");
    handoff.clutch_hold[0] = false;
    ++handoff.clutch_seq[0];
    auto resumed=handoff;
    resumed.arm_target_torso_relative = true;
    node.shapeContinuousTarget(resumed,handoff,0.,dt);
    check((resumed.arm[0].hand_position-origin[0].hand_position).norm()<.001,
          "re-engagement reset to unachieved goal");
    Node::SolverRequest r;
    r.reference_q = seed; r.reference_dq = Eigen::VectorXd::Zero(seed.size()); r.reference_ddq = r.reference_dq;
    r.cycle_dt = dt; r.target.clutch_hold[0] = true;
    Eigen::VectorXd lo, hi;
    check(node.calculatePositionVelocityBounds(r, false, &lo, &hi), "infeasible clutch");
    for (int joint = 0; joint < 7; ++joint) {
      const int left = node.armDecisionIndex(0,joint), right = node.armDecisionIndex(1,joint);
      check(std::abs(lo[left])+std::abs(hi[left]) < 1e-8, "left hold failed");
      check(hi[right] > lo[right], "left clutch locked right");
    }
    // Non-identity torso rotation catches orientation-frame errors as well as
    // the torso/world translation error. Exercise pause and clutch transitions
    // without changing their existing admission conditions or filter settings.
    for (bool torso_relative : {false, true}) {
      node.reference_q_ = seed;
      node.reference_q_[node.lowerBodyDecisionIndex(2)] += .1;
      node.reference_q_[node.lowerBodyDecisionIndex(3)] += .3;
      node.setPlantDecisionPositions(node.reference_q_);
      Node::TargetSnapshot source;
      source.arm_target_torso_relative = false;
      source.clutch_seq.fill(100);
      const auto& frame = torso_relative
          ? node.plant_->GetFrameByName(node.torso_frame_name_) : node.plant_->world_frame();
      for (int arm=0; arm<2; ++arm) {
        const auto fk=node.plant_->GetFrameByName(node.hand_frame_names_[arm]).CalcPose(*node.plant_context_,frame);
        source.arm[arm].hand_position=fk.translation();
        source.arm[arm].hand_orientation=Eigen::Quaterniond(fk.rotation().matrix());
      }
      for (int phase=0; phase<4; ++phase) {
        source.arm_hold = phase==1;
        source.clutch_hold[0] = phase==2;
        ++source.arm_operation_seq;
        ++source.clutch_seq[0]; ++source.clutch_seq[1];
        auto aligned=source;
        aligned.arm_target_torso_relative=torso_relative;
        node.shapeContinuousTarget(aligned,source,0.,dt);
        for (int arm=0; arm<2; ++arm) {
          check((aligned.arm[arm].hand_position-source.arm[arm].hand_position).norm()<1.e-12,
                "stationary operation changed the reference position frame");
          check(aligned.arm[arm].hand_orientation.angularDistance(source.arm[arm].hand_orientation)<1.e-12,
                "stationary operation changed the reference orientation frame");
          check(aligned.arm[arm].hand_linear_ff.norm()<1.e-12 && aligned.arm[arm].hand_angular_ff.norm()<1.e-12,
                "stationary operation released frame-change feedforward");
        }
      }
    }
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "absolute_velocity_ik");
  const int key = 0x610000 + getpid()*2;
  if (shmget(key, 1, 0) >= 0 || shmget(key+1, 1, 0) >= 0) return 2;
  int status = 0;
  try { PinocchioVelocityIk node; VelocityIkOfflineTest::run(node, key); VelocityIkOfflineTest::lowerBodyHandoff(node); }
  catch (const std::exception& e) { std::cerr << e.what() << std::endl; status = 1; }
  for (int k : {key, key+1}) {
    int id = shmget(k, 1, 0600);
    if (id >= 0) shmctl(id, IPC_RMID, nullptr);
  }
  return status;
}
