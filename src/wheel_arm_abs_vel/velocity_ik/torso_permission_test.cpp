// Production callback, publishOnce and QP with ideal feedback; no robot transport.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include <iostream>
struct VelocityIkOfflineTest {
 using Node=PinocchioVelocityIk;
 using Msg=wheel_arm_abs_vel::TeleopTcpTarget;
 static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
 static double lowerMax(Node& n,const Eigen::VectorXd& x){double e=0.;for(int i:n.dof_layout_.decisionIndices(DofGroup::kLowerBody))e=std::max(e,std::abs(x(i)));return e;}
 static double armMax(Node& n,const Eigen::VectorXd& x){double e=0.;for(int arm=0;arm<2;++arm)for(int j=0;j<7;++j)e=std::max(e,std::abs(x(n.armDecisionIndex(arm,j))));return e;}
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
  n.diagnostics_async_=false;n.publish_input_debug_=false;
  n.reference_pub_=n.nh_.advertise<wheel_arm_abs_vel::VelocityIkShadow>("/offline/reference",1);
  using Lease=velocity_ik_execution::ExecutionProducerLease;
  Lease::Feedback f;f.producer=n.execution_instance_;f.owner="torso-test";f.valid=true;f.phase=Lease::Phase::Active;
  f.generated_ns=velocity_ik_execution::executionSteadyNs();f.lease_until_ns=f.generated_ns+3600000000000LL;n.execution_lease_.observe(f,true,true);
  Msg::Ptr m(new Msg);m->header.frame_id="quest_chest_robot_axes";
  m->left_controlled_frame=n.left_controlled_frame_;m->right_controlled_frame=n.right_controlled_frame_;
  m->control_state=Msg::CONTROL_ACTIVE;m->control_valid=m->left_valid=m->right_valid=true;
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
 try{PinocchioVelocityIk n;VelocityIkOfflineTest::boundarySeams(n);VelocityIkOfflineTest::run(n,argc>2?argv[2]:"normal");}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
