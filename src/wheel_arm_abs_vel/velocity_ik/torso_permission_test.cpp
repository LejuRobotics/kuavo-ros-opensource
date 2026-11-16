// Production callback, publishOnce and QP with ideal feedback; no robot transport.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include <iostream>
#include <ros/serialization.h>
struct VelocityIkOfflineTest {
 using Node=PinocchioVelocityIk;
 using Msg=wheel_arm_abs_vel::TeleopTcpTarget;
 static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
 static double lowerMax(Node& n,const Eigen::VectorXd& x){double e=0.;for(int i:n.dof_layout_.decisionIndices(DofGroup::kLowerBody))e=std::max(e,std::abs(x(i)));return e;}
 static double armMax(Node& n,const Eigen::VectorXd& x){double e=0.;for(int arm=0;arm<2;++arm)for(int j=0;j<7;++j)e=std::max(e,std::abs(x(n.armDecisionIndex(arm,j))));return e;}
 template<class M> static boost::shared_ptr<M> readBagMessage(std::ifstream& input) {
  uint32_t size=0;input.read(reinterpret_cast<char*>(&size),sizeof(size));
  require(input && size>0 && size<1000000,"bad offline bag message length");
  std::vector<uint8_t> bytes(size);input.read(reinterpret_cast<char*>(bytes.data()),size);
  require(bool(input),"truncated offline bag message");
  ros::serialization::IStream stream(bytes.data(),size);
  auto message=boost::make_shared<M>();ros::serialization::deserialize(stream,*message);return message;
 }
 static void replayBag(Node& n,const std::string& path,double lag) {
  std::ifstream input(path,std::ios::binary);
  auto sensor=readBagMessage<kuavo_msgs::sensorsData>(input);
  std::vector<std::pair<double,Msg::Ptr>> frames;double time;
  while(input.read(reinterpret_cast<char*>(&time),sizeof(time)))
   frames.emplace_back(time,readBagMessage<Msg>(input));
  require(!frames.empty() && n.enable_takeover_,"empty replay or disabled reference authority");
  sensor->header.stamp=ros::Time::now();n.robotStateCallback(sensor);
  require(n.robot_state_valid_,"invalid recorded sensor state");
  n.reference_q_=n.measured_q_.cwiseMax(n.position_lower_).cwiseMin(n.position_upper_);
  n.reference_dq_.setZero();n.reference_ddq_.setZero();
  for(int a=0;a<2;++a)for(int j=0;j<7;++j)
   n.preparation_position_[n.armDecisionIndex(a,j)]=j==0?.2618:j==3?-.5236:0.;
  n.reference_initialized_=n.takeover_started_=n.arm_mode_known_=n.base_state_valid_=true;
  n.current_arm_mode_=n.target_arm_mode_=2;n.arm_mode_transitioning_=false;n.session_anchor_.valid=true;
  n.offline_capture_messages_=true;n.diagnostics_async_=false;n.publish_input_debug_=false;
  using Lease=velocity_ik_execution::ExecutionProducerLease;
  Lease::Feedback lease;lease.producer=n.execution_instance_;lease.owner="private-bag-replay";
  lease.valid=true;lease.phase=Lease::Phase::Active;
  DecisionVector feedback=n.reference_q_.eval(),feedback_v=n.reference_dq_.eval();
  const auto start=std::chrono::steady_clock::now();size_t frame=0;
  const auto initial=n.reference_q_.eval();
  auto frozen_min=initial,frozen_max=initial;
  bool sampled_frozen=false;int completed_returns=0;bool last_prepared=false;
  const std::string csv=path+".lag"+std::to_string(lag)+".csv";
  std::ofstream output(csv);output<<std::setprecision(12);
  output<<"time,phase,return_phase,prepared,session,generation,max_arm_speed,max_arm_delta,model_gap_m\n";
  for(int k=0;k<=std::ceil(frames.back().first/n.dt_);++k) {
   const double seconds=k*n.dt_;
   std::this_thread::sleep_until(start+std::chrono::microseconds(2000*k));
   const auto now=ros::Time::now();const auto ns=n.monotonicNs();
   n.measured_q_=feedback;n.measured_dq_=feedback_v;n.sensor_source_stamp_=now;n.sensor_receive_ns_=ns;
   n.robot_measurement_freshness_.observe(now.toNSec(),now.toNSec(),ns,n.robot_state_timeout_);
   n.base_measurement_freshness_.observe(now.toNSec(),now.toNSec(),ns,n.base_state_timeout_);
   lease.generated_ns=velocity_ik_execution::executionSteadyNs();lease.lease_until_ns=lease.generated_ns+1000000000LL;
   n.execution_lease_.observe(lease,true,true);
   while(frame<frames.size() && frames[frame].first<=seconds) {
    auto message=frames[frame++].second;
    const auto processing=message->mapper_publish_stamp-message->mapper_receive_stamp;
    message->mapper_publish_stamp=now;message->mapper_receive_stamp=now-processing;n.targetCallback(message);
   }
   const auto old_q=n.reference_q_.eval(),old_v=n.reference_dq_.eval();n.publishOnce();
   require(!n.dynamic_fault_latched_ && !n.reference_tracking_fault_latched_,"fault during bag replay");
   require(maxAbs(n.reference_q_-old_q-.001*(old_v+n.reference_dq_))<1e-8,"bag replay integration mismatch");
   if(seconds>=1.2 && seconds<=12.8) {
    if(!sampled_frozen){frozen_min=frozen_max=n.reference_q_;sampled_frozen=true;}
    frozen_min=frozen_min.cwiseMin(n.reference_q_);frozen_max=frozen_max.cwiseMax(n.reference_q_);
   }
   if(n.preparation_complete_ && !last_prepared)++completed_returns;
   last_prepared=n.preparation_complete_;
   if(lag>0.) {
    const auto next=(feedback+(-std::expm1(-n.dt_/lag))*(n.reference_q_-feedback)).eval();
    feedback_v=(next-feedback)/n.dt_;feedback=next;
   } else {feedback=n.reference_q_;feedback_v=n.reference_dq_;}
   if(k%2==0) {
    n.setPlantDecisionPositions(n.reference_q_);std::array<std::array<double,7>,2> zero{};
    const double gap=n.unlock_geometry_.clearance(*n.plant_,*n.plant_context_,n.torso_frame_name_,zero);
    output<<seconds<<','<<int(n.unlock_phase_)<<','<<int(n.return_phase_)<<','<<n.preparation_complete_<<','
          <<n.target_.session_id<<','<<n.target_.generation<<','<<n.maxArmAbs(n.reference_dq_)<<','
          <<n.maxArmAbs(n.reference_q_-initial)<<','<<gap<<'\n';
   }
  }
  const double range=n.maxArmAbs(frozen_max-frozen_min);
  require(sampled_frozen && range>.01,"recorded frozen interval still has no arm motion");
  std::cout<<"PASS BAG_REPLAY frames="<<frame<<" lag="<<lag<<" formerly_frozen_joint_range_rad="
           <<range<<" completed_returns="<<completed_returns<<" csv="<<csv<<'\n';
 }
 static void boundarySeams(Node& n){
  for(int index=0;index<n.decisionDofCount();++index) {
   if(n.isDecisionGroup(index,DofGroup::kBase))continue;
   for(bool upper:{false,true}) {
    Node::SolverRequest r;r.reference_q=n.preparation_position_;
    r.reference_dq=Eigen::VectorXd::Zero(n.decisionDofCount());r.reference_ddq=r.reference_dq;
    r.cycle_dt=.002;r.lower_joint_recruitment.fill(0.);r.lower_body_recruitment=0.;
    r.target.arm_hold=true;
    r.reference_q(index)=(upper?n.position_upper_(index):n.position_lower_(index))+(upper?1.:-1.)*1e-11;
    r.reference_dq(index)=(upper?1.:-1.)*5e-10;
    Eigen::VectorXd lo,hi;
    require(n.calculatePositionVelocityBounds(r,false,&lo,&hi),"numerical endpoint seam prevents lock");
    const auto bounds=n.calculateHardBounds(r);
    require(bounds.valid,"numerical endpoint seam prevents hard hold");
    require(std::abs(bounds.lower(index))<1e-8 && std::abs(bounds.upper(index))<1e-8,"seam introduced motion");
    r.reference_q(index)=(upper?n.position_upper_(index):n.position_lower_(index))+(upper?1.:-1.)*1e-5;
    require(!n.calculatePositionVelocityBounds(r,false,&lo,&hi),"material boundary violation silently accepted");
   }
  }
  std::cout<<"PASS arm/lower boundary numerical seams and material overrun rejection\n";
 }
 static void run(Node& n,const std::string& scenario){
  n.reference_q_=n.preparation_position_;n.reference_dq_.setZero();n.reference_ddq_.setZero();
  // Locked means current pose, not the configured ready pose.
  const int knee=n.lowerBodyDecisionIndex(1);
  n.reference_q_(knee)=std::clamp(n.reference_q_(knee)+.03,n.joint_safe_lower_(knee),n.joint_safe_upper_(knee));
  const bool full_reset=scenario.rfind("reset_",0)==0;
  if(full_reset) {
   for(int i:n.dof_layout_.decisionIndices(DofGroup::kLowerBody)) {
    const double direction=n.preparation_position_(i)+.04<n.joint_safe_upper_(i)?1.:-1.;
    n.reference_q_(i)=n.preparation_position_(i)+direction*.04;
   }
   for(int arm=0;arm<2;++arm) {
    const int i=n.armDecisionIndex(arm,1);
    n.reference_q_(i)=std::clamp(n.reference_q_(i)+.08,n.joint_safe_lower_(i),n.joint_safe_upper_(i));
   }
  }
  n.reference_initialized_=n.takeover_started_=n.enable_takeover_=true;
  n.robot_state_valid_=n.arm_mode_known_=true;n.current_arm_mode_=n.target_arm_mode_=2;n.arm_mode_transitioning_=false;
  n.session_anchor_.valid=true;n.session_anchor_.session_id=1;n.session_anchor_.generation=1;n.session_anchor_.base_coordinate_epoch=0;
  if(scenario=="tracking_lag") {
   // Reproduce the right-elbow command/feedback gap from the 18:56 bag.
   // The fixture has no hardware transport and never enables a motor.
   using Phase=Node::UnlockPhase;
   const int elbow=n.armDecisionIndex(1,3);
   n.reference_q_[elbow]=-2.400351035323815;
   auto measured=n.reference_q_.eval();measured[elbow]=-2.271305822813991;
   const auto zero=DecisionVector::Zero(n.decisionDofCount()).eval();
   const auto original_q=n.reference_q_.eval();
   require(n.maxArmAbs(measured-n.reference_q_)>.10,"fixture lost the recorded tracking error");
   auto raw=n.target_,live=raw;live.arm_target_torso_relative=true;
   n.unlock_key_valid_=true;n.unlock_session_=raw.session_id;n.unlock_generation_=raw.generation;
   n.unlock_epoch_=raw.body.coordinate_epoch;n.unlock_base_epoch_=0;
   n.unlock_arm_operation_seq_=raw.arm_operation_seq;n.ready_follow_guard_=false;
   n.unlock_total_elapsed_=0.;n.unlock_elapsed_=.1;
   n.unlock_plan_.valid=true;n.unlock_plan_.points={n.reference_q_,n.reference_q_};
   n.unlock_plan_.points.back()[elbow]+=.05;n.unlock_plan_.durations={1.};
   n.unlock_phase_=Phase::Executing;
   n.updateUnlockLifecycle(raw,live,0,true,false,false,measured,zero,.002);
   require(n.unlock_phase_==Phase::Executing,"tracking error blocked planned acquisition");
   n.unlock_phase_=Phase::Joining;n.unlock_join_elapsed_=0.;
   n.updateUnlockLifecycle(raw,live,0,true,false,false,measured,zero,.002);
   require(n.unlock_phase_==Phase::Joining,"tracking error blocked live handoff");
   n.cancelReturnPath();
   const auto velocity=n.returnPathVelocity(n.preparation_position_,measured,zero,.002);
   require(n.return_phase_==Phase::Executing && velocity[elbow]>0.,
           "tracking error prevented X+A return from starting");
   require(std::abs(velocity[elbow])<=.002*std::min(n.unlock_acceleration_,n.acceleration_limit_[elbow])+1e-10,
           "return bypassed the existing acceleration limit");
   raw.arm_hold=true;
   n.updateUnlockLifecycle(raw,live,0,false,false,false,measured,zero,.002);
   require(n.unlock_phase_==Phase::Suspended,"X+B no longer interrupts acquisition");
   require(maxAbs(n.reference_q_-original_q)<1e-12,"lag handling stepped the position reference");
   std::cout<<"PASS recorded 0.129045-rad right-elbow lag: acquisition, handoff, X+A return; X+B and acceleration limit retained\n";
   return;
  }
  n.diagnostics_async_=false;n.publish_input_debug_=false;
  n.reference_pub_=n.nh_.advertise<wheel_arm_abs_vel::VelocityIkShadow>("/offline/reference",1);
  using Lease=velocity_ik_execution::ExecutionProducerLease;
  Lease::Feedback f;f.producer=n.execution_instance_;f.owner="torso-test";f.valid=true;f.phase=Lease::Phase::Active;
  f.generated_ns=velocity_ik_execution::executionSteadyNs();f.lease_until_ns=f.generated_ns+3600000000000LL;n.execution_lease_.observe(f,true,true);
  Msg::Ptr m(new Msg);m->header.frame_id="quest_chest_robot_axes";
  m->left_controlled_frame=n.left_controlled_frame_;m->right_controlled_frame=n.right_controlled_frame_;
  m->control_state=Msg::CONTROL_ACTIVE;m->control_valid=m->left_valid=m->right_valid=true;
  m->estimated_twists_valid=true;  // synthetic pose stream carries a valid zero twist
  m->source=Msg::SOURCE_QUEST_CONTROLLER;m->session_id=2;m->generation=2;
  m->body_tracking_frame=n.body_tracking_frame_name_;m->body_high_confidence=true;m->chest_valid=true;
  const Eigen::Quaterniond rot(n.rotationFromRpy(Eigen::Vector3d(M_PI/2.,0.,M_PI/2.)));
  m->chest_pose.orientation.x=rot.x();m->chest_pose.orientation.y=rot.y();m->chest_pose.orientation.z=rot.z();m->chest_pose.orientation.w=rot.w();
  m->chest_pose.position.z=1.3;m->body_coordinate_epoch=1;
  n.setPlantDecisionPositions(n.reference_q_);
  const auto& torso=n.plant_->GetFrameByName(n.torso_frame_name_);
  for(int arm=0;arm<2;++arm){
   const auto pose=n.plant_->GetFrameByName(n.hand_frame_names_[arm]).CalcPose(*n.plant_context_,torso);
   auto& p=arm?m->right_tcp_pose:m->left_tcp_pose;
   p.position.x=pose.translation().x();p.position.y=pose.translation().y();p.position.z=pose.translation().z();
   const Eigen::Quaterniond q(pose.rotation().matrix());p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
  }
  int tick=0;
  bool publish_target=true;
  auto step=[&]{
   ros::WallDuration(.002).sleep();
   n.measured_q_=n.reference_q_;n.measured_dq_=n.reference_dq_;
   const auto now=ros::Time::now();n.robot_measurement_freshness_.observe(now.toNSec(),now.toNSec(),n.monotonicNs(),n.robot_state_timeout_);
   m->header.seq=++tick;m->header.stamp.fromSec(100.+tick*.002);m->mapper_publish_stamp=now;
   if(publish_target)n.targetCallback(m);const auto q=n.reference_q_.eval(),v=n.reference_dq_.eval();n.publishOnce();
   if(n.dynamic_fault_latched_) {
    Node::SolverRequest probe;probe.reference_q=q;probe.reference_dq=v;probe.reference_ddq=n.reference_ddq_;
    probe.cycle_dt=.002;probe.lower_joint_recruitment.fill(m->torso_control_enabled?1.:0.);probe.lower_body_recruitment=m->torso_control_enabled?1.:0.;
    Eigen::VectorXd lo,hi;const bool pv=n.calculatePositionVelocityBounds(probe,false,&lo,&hi);
    const auto bounds=n.calculateHardBounds(probe);
    const auto torso_bounds=n.calculateTorsoVelocityConstraintsInContext(probe,n.supervisor_context_.get(),&n.supervisor_q_full_);
    std::cerr<<"fault tick="<<tick<<" posvel="<<pv<<" hard="<<bounds.valid<<" torso="<<torso_bounds.valid<<" q="<<q.transpose()<<" v="<<v.transpose()<<"\nlo="<<lo.transpose()<<"\nhi="<<hi.transpose()<<'\n';
   }
   require(!n.dynamic_fault_latched_,"dynamic fault");require(!n.reference_tracking_fault_latched_,"tracking fault");
   require(maxAbs(n.reference_q_-q-.001*(v+n.reference_dq_))<1e-8,"integration mismatch");
  };
  if (scenario == "xa_close_return") {
    // The first seed is the frozen command in the 17:39:17 bag, with
    // 8.84 mm model clearance. The second is the former 6 mm rejection
    // fixture. Both must now return under the ordinary joint supervisor.
    const auto origin=n.reference_q_.eval();
    const std::array<std::array<double,14>,3> starts{{
      {.3042870315,-.0502438563,-.0627412575,-.5906617701,.0076753392,.0355962400,-.0293800792,
       .2969517155,-.0703973723,-.0469681500,-.5865662654,-.0649012927,-.0010582971,.0234696190},
      {.200735,.242907,-.678298,-1.362924,-.398447,0.,0., .2618,0.,0.,-.5236,0.,0.,0.},
      {-1.45380,-.05724,-.51234,-.48408,.97566,.14056,-.07684,
       -1.52354,.04390,.71313,-.44316,-.81000,-.02453,.11893}
    }};
    for(int a=0;a<2;++a)for(int j=0;j<7;++j)
      n.preparation_position_[n.armDecisionIndex(a,j)]=j==0?.2618:j==3?-.5236:0.;
    for(size_t sample=0;sample<starts.size();++sample) {
      n.reference_q_=origin;n.reference_dq_.setZero();n.reference_ddq_.setZero();
      // Each recorded pose is a separate starting condition. Reset the
      // completed return before replacing the simulated robot posture.
      n.cancelReturnPath();
      n.preparation_active_=false;n.preparation_settle_cycles_=0;
      for(int a=0;a<2;++a)for(int j=0;j<7;++j)
        n.reference_q_[n.armDecisionIndex(a,j)]=starts[sample][7*a+j];
      if(sample<2) {
        n.setPlantDecisionPositions(n.reference_q_);
        std::array<std::array<double,7>,2> zero{};
        require(n.unlock_geometry_.clearance(*n.plant_,*n.plant_context_,n.torso_frame_name_,zero)<.01,
                "near-body regression no longer represents the rejected pose");
      }
      m->control_state=Msg::CONTROL_PREPARE;++m->generation;
      n.preparation_complete_=false;
      const auto start=n.reference_q_.eval();
      int first_motion=-1;
      for(int i=0;i<2500 && !n.preparation_complete_;++i) {
        step();
        if(first_motion<0 && armMax(n,n.reference_q_-start)>1e-6)first_motion=i;
      }
      require(first_motion>=0 && first_motion<30,"near-body return did not begin promptly");
      require(n.preparation_complete_,"one X+A must reach ready from the recorded near-body pose");
      require(armMax(n,n.reference_q_-n.preparation_position_)<=n.preparation_position_tolerance_,
              "PREPARE reported completion away from ready");
      std::cout<<"PASS near-body return seed="<<sample<<" first_motion_s="<<.002*first_motion<<'\n';
    }
    return;
  }
  if (scenario=="xa_torso_handoff") {
    using Phase=Node::UnlockPhase;
    m->torso_control_enabled=true;
    auto chest=[&](double height,double pitch,double yaw) {
      m->chest_pose.position.z=height;
      const Eigen::Quaterniond orientation=
          Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()) * rot;
      m->chest_pose.orientation=n.toPose(Eigen::Vector3d::Zero(),orientation).orientation;
    };
    for(int i=0;i<500 && n.unlock_phase_!=Phase::Joining;++i)step();
    require(n.unlock_phase_==Phase::Joining && !n.arm_only_join_, "expected whole-body unlock");
    chest(1.27,-.06,.10);
    const auto start=n.reference_q_.eval();
    for(int i=0;i<1500 && n.unlock_phase_!=Phase::Done;++i)step();
    require(n.unlock_phase_==Phase::Done && n.torso_control_requested_, "torso did not take over");
    const auto feature=n.torsoFeatureInContext(start,n.plant_context_.get(),&n.q_full_);
    require((n.last_body_intent_.desired_torso_feature-feature).norm()<1e-8,
            "torso took over with displacement accumulated during arm entry");
    for(int i=0;i<160;++i)step();
    require(lowerMax(n,n.reference_q_-start)<1e-8, "stationary chest caused lower-body catch-up");

    chest(1.25,-.09,.16);
    for(int i=0;i<300;++i)step();
    const Eigen::Vector3d delta(-.02*n.torso_height_gain_,-.03*n.torso_pitch_gain_,
                                .06*n.torso_yaw_gain_);
    require((n.last_body_intent_.desired_torso_feature.tail<3>()-feature.tail<3>()-delta).norm()<1e-8,
            "live height, pitch or yaw lost its configured gain");
    const auto tracked=n.torsoFeatureInContext(n.reference_q_,n.plant_context_.get(),&n.q_full_);
    require(tracked[1]<feature[1]-.001 && tracked[2]<feature[2]-.005 && tracked[3]>feature[3]+.005,
            "live torso did not track the new chest motion");

    m->arm_hold=true;++m->arm_operation_seq;
    for(int i=0;i<100;++i)step();
    m->arm_hold=false;++m->arm_operation_seq;
    for(int i=0;i<1500;++i) {
      step();
      if(n.arm_only_join_ && n.unlock_phase_==Phase::Done)break;
    }
    require(n.arm_only_join_ && n.unlock_phase_==Phase::Done, "X+B resume did not complete");
    require((n.last_body_intent_.desired_torso_feature.tail<3>()-feature.tail<3>()-delta).norm()<1e-8,
            "X+B arm-only handoff reset the torso target");
    std::cout<<"PASS X+A torso handoff, stationary chest, live height/pitch/yaw and X+B resume\n";
    return;
  }
  if (scenario=="follow_entry") {
   const auto ready=n.reference_q_.eval();
   std::array<Eigen::Vector3d,2> hand_goal;
   std::array<Eigen::Quaterniond,2> rotation_goal;
   auto goal=[&](double offset) {
    auto q=ready;
    for(int a=0;a<2;++a) {
     q[n.armDecisionIndex(a,0)]-=offset;
     q[n.armDecisionIndex(a,3)]-=std::abs(offset)*.7;
     for (int k : {0,3}) {
      const int j=n.armDecisionIndex(a,k);
      require(q[j]>n.position_lower_[j]+n.position_limit_margin_+n.boundary_brake_guard_ &&
              q[j]<n.position_upper_[j]-n.position_limit_margin_-n.boundary_brake_guard_,
              "requested test pose exceeds operating travel");
     }
    }
    n.setPlantDecisionPositions(q);
    for(int a=0;a<2;++a) {
     const auto pose=n.plant_->GetFrameByName(n.hand_frame_names_[a]).CalcPose(*n.plant_context_,torso);
     hand_goal[a]=pose.translation();
     rotation_goal[a]=Eigen::Quaterniond(pose.rotation().matrix());
     const Eigen::Vector3d origin=n.plant_->GetFrameByName(n.shoulder_frame_names_[a])
         .CalcPose(*n.plant_context_,torso).translation()-Eigen::Vector3d(0.,a==0?.15:-.15,.42);
     auto& p=a?m->right_tcp_pose:m->left_tcp_pose;
     p=n.toPose(hand_goal[a]-origin,rotation_goal[a]);
    }
   };
   auto error=[&]() {
    n.setPlantDecisionPositions(n.reference_q_);
    Eigen::Vector2d e=Eigen::Vector2d::Zero();
    for(int a=0;a<2;++a) {
     const auto actual=n.plant_->GetFrameByName(n.hand_frame_names_[a]).CalcPose(*n.plant_context_,torso);
     e.x()=std::max(e.x(),(actual.translation()-hand_goal[a]).norm());
     e.y()=std::max(e.y(),rotation_goal[a].angularDistance(Eigen::Quaterniond(actual.rotation().matrix())));
    }
    return e;
   };
   for(int round=0;round<4;++round) {
    if(round) {
     m->arm_hold=true;++m->arm_operation_seq;
     for(int i=0;i<100;++i)step();
     const auto paused=n.reference_q_.eval();
     goal(round==1 ? -.20 : (round==2 ? .80 : -.26));
     for(int i=0;i<40;++i)step();
     require(armMax(n,n.reference_q_-paused)<1e-8,"moved during operator pause");
     m->arm_hold=false;++m->arm_operation_seq;
    } else goal(.25);
    const auto origin=n.reference_q_.eval();
    const double initial=error().x();
    int entry_ticks=0, first_motion=-1;
    double previous=0.;
    for(int i=0;i<1800;++i) {
     if(i==65)goal(round==1 ? -.25 : (round==2 ? .85 : (round==3 ? -.29 : .33)));  // live endpoint, no new operation
     step();
     if(!n.entry_key_valid_)continue;
     require(n.entry_progress_>=previous,"VR frames restarted interpolation");
     previous=n.entry_progress_;
     ++entry_ticks;
     if(first_motion<0 && armMax(n,n.reference_q_-origin)>1e-6)first_motion=entry_ticks;
     if(!n.entry_active_)break;
    }
    require(entry_ticks>1 && entry_ticks<900,"no bounded entry or handoff");
    require(first_motion>0 && first_motion<30,"entry added a standstill dwell");
    const auto handoff=error();
    std::cout<<"entry "<<round<<": "<<.002*entry_ticks<<" s; first motion "
             <<.002*first_motion<<" s; TCP error "<<initial<<" -> "<<handoff.transpose()<<std::endl;
    require(handoff.x()<.035 && handoff.y()<.14,"entry handed off far from reachable live goal");
    for(int i=0;i<180;++i)step();
    require(!n.entry_active_ && n.entry_progress_==1.,"ordinary follow restarted interpolation");
    require(error().x()<.025 && error().y()<.10,"did not track live endpoint");
    require(lowerMax(n,n.reference_q_-ready)<1e-8,"arm acquisition recruited lower body");
   }
   return;
  }
  if(full_reset) {
   m->control_state=Msg::CONTROL_PREPARE;
   const auto initial=n.reference_q_.eval();
   for(int k=0;k<6000 && !n.preparation_complete_;++k) {
    if(k==80 && scenario=="reset_hold")m->control_state=Msg::CONTROL_HOLD;
    if(k==80 && scenario=="reset_stale")publish_target=false;
    step();
    require(n.reference_q_.allFinite() && n.reference_dq_.allFinite(),"nonfinite reset trajectory");
    if(k>=700 && scenario!="reset_complete")break;
   }
   require(!n.target_.torso_control_enabled,"reset enabled human torso following");
   if(scenario=="reset_complete") {
    require(n.preparation_complete_,"full reset did not complete");
    for(int i:n.dof_layout_.decisionIndices(DofGroup::kLowerBody)) {
     require(std::abs(n.reference_q_(i)-n.preparation_position_(i))<=n.preparation_lower_position_tolerance_,"lower joint not reset");
     require(std::abs(n.reference_q_(i)-initial(i))>.02,"lower joint skipped");
    }
    require(armMax(n,n.reference_q_-n.preparation_position_)<=n.preparation_position_tolerance_,"arms not reset");
   } else {
    require(!n.preparation_complete_,"cancelled reset falsely completed");
    require(maxAbs(n.reference_dq_)<1e-8,"cancelled reset failed to brake");
    const auto held=n.reference_q_.eval();
    for(int k=0;k<100;++k)step();
    require(maxAbs(n.reference_q_-held)<1e-8,"cancelled reset drifted");
   }
   std::cout<<"PASS "<<scenario<<" all four lower joints and both arms, ticks="<<tick<<'\n';
   return;
  }
  step();const auto locked=n.reference_q_.eval();
  m->left_tcp_pose.position.x+=.10;m->right_tcp_pose.position.x+=.10;m->chest_pose.position.z+=.15;
  for(int k=0;k<600;++k)step();
  require(lowerMax(n,n.reference_q_-locked)<1e-10,"X+A follow moved locked folding joints");
  require(maxAbs(n.reference_q_-locked)>1e-4,"fixture did not move arms");
  require(!n.calculateBodyIntent(n.target_).torso_valid,"locked body intent remained active");
  std::cout<<"PASS locked follow: arms move, lower displacement="<<lowerMax(n,n.reference_q_-locked)<<'\n';
  m->torso_control_enabled=true;const auto before=n.reference_q_.eval();step();
  require(n.torso_control_requested_,"permission did not activate");
  const auto feature=n.torsoFeatureInContext(before,n.plant_context_.get(),&n.q_full_);
  require((n.calculateBodyIntent(n.target_).desired_torso_feature-feature).norm()<1e-9,"enable jumps to accumulated human displacement");
  m->chest_pose.position.z-=.10;
  for(int k=0;k<300;++k)step();
  require(lowerMax(n,n.reference_q_-before)>1e-5,"enabled torso did not respond");
  std::cout<<"PASS enabled torso responds, lower displacement="<<lowerMax(n,n.reference_q_-before)<<'\n';
  // X+B pauses only arms while the same ACTIVE owner continues torso motion.
  m->left_tcp_pose.position.x+=.12;m->right_tcp_pose.position.x+=.12;
  for(int k=0;k<40;++k)step();
  require(armMax(n,n.reference_dq_)>1e-5,"arm pause fixture was not moving");
  const auto old_follow=boost::make_shared<Msg>(*m);
  const auto body_anchor=n.session_body_anchor_.human.chest_position.eval();
  const auto hand_alignment=n.session_target_alignment_.torso_from_source;
  const auto observation=n.target_.observation_ns;
  m->arm_hold=true;++m->arm_operation_seq;
  n.targetCallback(m); // same skeleton stamp: apply operation, not fake freshness
  require(n.target_.arm_hold,"same-stamp arm pause was discarded as duplicate");
  require(n.target_.observation_ns==observation,"operation refreshed stale skeleton");
  for(int k=0;k<200;++k)step();
  require(armMax(n,n.reference_dq_)<1e-9,"arm-only pause did not brake arms");
  const auto held=n.reference_q_.eval();
  m->left_tcp_pose.position.x+=.15;m->right_tcp_pose.position.x+=.15;
  m->chest_pose.position.z-=.08;
  for(int k=0;k<300;++k)step();
  require(armMax(n,n.reference_q_-held)<1e-8,"paused arms followed live targets");
  require(lowerMax(n,n.reference_q_-held)>1e-5,"X+B also stopped torso");
  require(n.torso_control_requested_,"arm pause revoked torso permission");
  old_follow->header.stamp=m->header.stamp;
  n.targetCallback(old_follow);
  require(n.target_.arm_hold && n.target_.arm_operation_seq==m->arm_operation_seq,
          "old arm follow packet undid pause");
  std::cout<<"PASS X+B: arms held, torso moved="<<lowerMax(n,n.reference_q_-held)<<"; stale operation rejected\n";
  if(scenario=="stop" || scenario=="stale") {
    if(scenario=="stop")m->control_state=Msg::CONTROL_HOLD;
    else m->left_valid=m->right_valid=m->chest_valid=m->body_high_confidence=false;
    for(int k=0;k<700;++k)step();
    require(maxAbs(n.reference_dq_)<1e-8,"stop/lost input failed to stop whole body");
    const auto stopped_all=n.reference_q_.eval();
    m->chest_pose.position.z+=.15;m->left_tcp_pose.position.x+=.10;
    for(int k=0;k<100;++k)step();
    require(maxAbs(n.reference_q_-stopped_all)<1e-8,"stopped whole body drifted");
    std::cout<<"PASS global "<<scenario<<" still stops arms and torso; ticks="<<tick<<'\n';
    return;
  }
  m->arm_hold=false;++m->arm_operation_seq;
  for(int k=0;k<200;++k)step();
  require(armMax(n,n.reference_q_-held)>1e-5,"X+B resume did not restore arm follow");
  require((n.session_body_anchor_.human.chest_position-body_anchor).norm()<1e-12,
          "arm resume recalibrated live torso");
  for(int arm=0;arm<2;++arm)
    require((n.session_target_alignment_.torso_from_source[arm].matrix()-hand_alignment[arm].matrix()).norm()<1e-12,
            "arm resume changed absolute hand calibration");
  require(n.torso_control_requested_,"arm resume revoked torso");
  std::cout<<"PASS X+B resumes absolute arm mapping without changing torso anchor\n";
  m->arm_hold=true;++m->arm_operation_seq;
  for(int k=0;k<200;++k)step();
  require(armMax(n,n.reference_dq_)<1e-9,"second arm pause failed");
  const auto paused_again=n.reference_q_.eval();
  m->torso_control_enabled=false;
  for(int k=0;k<300;++k)step();
  std::cout<<"disable lower max velocity="<<lowerMax(n,n.reference_dq_)<<" dq="<<n.reference_dq_.transpose()<<'\n';
  // Use the production QP certificate tolerance for the combined 18-joint
  // equality. Do not mistake sub-nanoradian numerical residuals for motion.
  require(lowerMax(n,n.reference_dq_)<1e-8,"disable did not stop lower velocity");
  const auto stopped=n.reference_q_.eval();m->chest_pose.position.z+=.30;
  for(int k=0;k<150;++k)step();
  require(lowerMax(n,n.reference_q_-stopped)<1e-8,"disabled torso drifted");
  require(armMax(n,n.reference_q_-paused_again)<1e-8,"disabling torso resumed paused arms");
  m->arm_hold=false;++m->arm_operation_seq;
  m->control_state=Msg::CONTROL_PREPARE;++m->generation;
  const double initial_lower_error=lowerMax(n,n.reference_q_-n.preparation_position_);
  require(initial_lower_error>.015,"PREPARE fixture did not displace folding joints");
  for(int k=0;k<6000 && !n.preparation_complete_;++k)step();
  require(n.preparation_complete_,"whole-body PREPARE did not complete");
  const double lower_error=lowerMax(n,n.reference_q_-n.preparation_position_);
  require(lower_error<=n.preparation_lower_position_tolerance_,"PREPARE skipped lower ready pose");
  require(armMax(n,n.reference_q_-n.preparation_position_)<=n.preparation_position_tolerance_,
          "PREPARE skipped arm ready pose");
  require(lowerMax(n,n.reference_q_-stopped)>.005,"PREPARE did not move folding joints");
  require(!n.target_.torso_control_enabled,"PREPARE required torso follow permission");
  std::cout<<"PASS X+A full-body PREPARE with torso disabled: initial lower error="
           <<initial_lower_error<<" final="<<lower_error<<'\n';
  // Reordered permission from an old operation cannot re-enable the live one.
  auto old=boost::make_shared<Msg>(*m);old->generation=2;old->torso_control_enabled=true;
  n.targetCallback(old);require(!n.target_.torso_control_enabled,"old generation enabled torso");
  std::cout<<"PASS stale-generation permission rejected; ticks="<<tick<<'\n';
 }
};
int main(int argc,char**argv){ros::init(argc,argv,"torso_permission_test",ros::init_options::NoSigintHandler);
 ros::NodeHandle p("~");p.setParam("robot_version",argc>1?std::stoi(argv[1]):62);p.setParam("enable_natural_hang_bootstrap",false);
 try{PinocchioVelocityIk n;VelocityIkOfflineTest::boundarySeams(n);
  if(argc>4 && std::string(argv[2])=="bag_replay")VelocityIkOfflineTest::replayBag(n,argv[3],std::stod(argv[4]));
  else VelocityIkOfflineTest::run(n,argc>2?argv[2]:"normal");
 }catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
