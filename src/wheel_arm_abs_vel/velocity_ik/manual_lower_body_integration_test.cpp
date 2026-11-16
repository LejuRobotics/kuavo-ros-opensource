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
   if(!n.isDecisionGroup(index,DofGroup::kLowerBody))continue;
   for(bool upper:{false,true}) {
    Node::SolverRequest r;r.reference_q=n.preparation_position_;
    r.reference_dq=Eigen::VectorXd::Zero(n.decisionDofCount());r.reference_ddq=r.reference_dq;
    r.cycle_dt=.002;r.lower_joint_recruitment.fill(0.);r.lower_body_recruitment=0.;
    r.reference_q(index)=(upper?n.position_upper_(index):n.position_lower_(index))+(upper?1.:-1.)*1e-11;
    r.reference_dq(index)=(upper?1.:-1.)*5e-10;
    Eigen::VectorXd lo,hi;
    require(n.calculatePositionVelocityBounds(r,false,&lo,&hi),"numerical endpoint seam prevents lock");
    const auto bounds=n.calculateHardBounds(r);
    require(bounds.valid,"numerical endpoint seam prevents hard hold");
    require(std::abs(bounds.lower(index))<1e-8 && std::abs(bounds.upper(index))<1e-8,"seam introduced motion");
    r.lower_joint_recruitment.fill(1.);r.lower_body_recruitment=1.;
    const auto released=n.calculateHardBounds(r);
    require(released.valid,"numerical endpoint seam prevents unlock");
    require(upper?released.upper(index)<=0.:released.lower(index)>=0.,"unlock permits outward boundary motion");
    require(upper?std::abs(released.upper(index))<1e-7:std::abs(released.lower(index))<1e-7,"unlock forces finite jump");
    r.reference_q(index)=(upper?n.position_upper_(index):n.position_lower_(index))+(upper?1.:-1.)*1e-5;
    require(!n.calculateHardBounds(r).valid,"material unlock boundary violation silently accepted");
    r.lower_joint_recruitment.fill(0.);r.lower_body_recruitment=0.;
    require(!n.calculatePositionVelocityBounds(r,false,&lo,&hi),"material boundary violation silently accepted");
   }
  }
  std::cout<<"PASS lower boundary numerical seams and material overrun rejection\n";
 }
 static void run(Node& n,const std::string& scenario){
  n.reference_q_=n.preparation_position_;n.reference_dq_.setZero();n.reference_ddq_.setZero();
  // Locked means current pose, not the configured ready pose.
  const int knee=n.lowerBodyDecisionIndex(1);
  n.reference_q_(knee)=std::clamp(n.reference_q_(knee)+.03,n.joint_safe_lower_(knee),n.joint_safe_upper_(knee));
  n.reference_initialized_=n.takeover_started_=n.enable_takeover_=true;
  n.robot_state_valid_=n.arm_mode_known_=true;n.current_arm_mode_=n.target_arm_mode_=2;n.arm_mode_transitioning_=false;
  n.session_anchor_.valid=true;n.session_anchor_.session_id=1;n.session_anchor_.generation=1;n.session_anchor_.base_coordinate_epoch=0;
  n.offline_capture_messages_=true;
  n.diagnostics_async_=false;n.publish_input_debug_=false;
  n.reference_pub_=n.nh_.advertise<wheel_arm_abs_vel::VelocityIkShadow>("/offline/reference",1);
  using Lease=velocity_ik_execution::ExecutionProducerLease;
  Lease::Feedback f;f.producer=n.execution_instance_;f.owner="torso-test";f.valid=true;f.phase=Lease::Phase::Active;
  f.generated_ns=velocity_ik_execution::executionSteadyNs();f.lease_until_ns=f.generated_ns+3600000000000LL;n.execution_lease_.observe(f,true,true);
  Msg::Ptr m(new Msg);m->header.frame_id="quest_chest_robot_axes";
  m->left_controlled_frame=n.left_controlled_frame_;m->right_controlled_frame=n.right_controlled_frame_;
  m->control_state=Msg::CONTROL_ACTIVE;m->control_valid=m->left_valid=m->right_valid=true;
  m->source=Msg::SOURCE_QUEST_CONTROLLER;m->session_id=2;m->generation=2;
  m->estimated_twists_valid=true; // beta requires the Mapper pose + twist contract
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
  bool fresh_target=true;
  auto step=[&]{
   ros::WallDuration(.002).sleep();
   n.measured_q_=n.reference_q_;n.measured_dq_=n.reference_dq_;
   const auto now=ros::Time::now();n.robot_measurement_freshness_.observe(now.toNSec(),now.toNSec(),n.monotonicNs(),n.robot_state_timeout_);
   m->header.seq=++tick;m->header.stamp=now;m->mapper_receive_stamp=now;m->mapper_publish_stamp=now;
   if(fresh_target)n.targetCallback(m);const auto q=n.reference_q_.eval(),v=n.reference_dq_.eval();n.publishOnce();
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
  m->arm_hold=true; ++m->arm_operation_seq;
  for(int k=0;k<10;++k)step();
  using Service=wheel_arm_abs_vel::ManualLowerBody;
  auto query=[&]{Service::Request req;Service::Response res;req.command=req.QUERY;n.manualLowerBodyService(req,res);return res;};
  auto set=[&](double offset){Service::Request req;Service::Response res;req.command=req.SET_TARGET;
    for(int j=0;j<4;++j)req.position[j]=n.reference_q_(n.lowerBodyDecisionIndex(j));
    req.position[0]+=offset;req.position[1]-=offset;
    n.manualLowerBodyService(req,res);require(res.accepted,res.message.c_str());return res;};
  if(!query().ready) {
    const auto st=query();
    std::cerr<<"manual admission: "<<st.message<<" takeover="<<n.takeover_started_
      <<" waiting="<<n.execution_waiting_<<" target_valid="<<n.manual_lower_.snapshot.allowed<<"\n";
  }
  require(query().ready,"manual interface not ready");
  Service::Request bad;Service::Response rejected;bad.command=bad.SET_TARGET;bad.position[0]=1e6;
  n.manualLowerBodyService(bad,rejected);require(!rejected.accepted,"out-of-range goal accepted");
  const auto initial=n.reference_q_.eval();set(.12);
  double peak_v=0.,peak_a=0.;bool done=false;
  for(int k=0;k<4500;++k){step();peak_v=std::max(peak_v,lowerMax(n,n.reference_dq_));peak_a=std::max(peak_a,lowerMax(n,n.reference_ddq_));
    require(armMax(n,n.reference_q_-initial)<1e-9,"manual adjustment moved arm joints");
    if(query().state==Service::Response::SETTLED){done=true;break;}}
  require(done,"manual target did not settle");
  require(peak_v<=velocity_ik::ManualLowerBody::kMaxVelocity+1e-7,"manual speed cap exceeded");
  require(peak_a<=velocity_ik::ManualLowerBody::kMaxAcceleration+1e-6,"manual acceleration cap exceeded");
  require(std::abs(n.reference_q_(n.lowerBodyDecisionIndex(0))-initial(n.lowerBodyDecisionIndex(0))-.12)<.001,"wrong knee goal");
  require(std::abs(n.reference_q_(n.lowerBodyDecisionIndex(1))-initial(n.lowerBodyDecisionIndex(1))+.12)<.001,"wrong leg goal");
  for(int k=0;k<50;++k)step();
  const auto held=n.reference_q_.eval();m->arm_hold=false;++m->arm_operation_seq;
  m->left_tcp_pose.position.x+=.04;m->right_tcp_pose.position.x+=.04;
  for(int k=0;k<500;++k)step();
  require(lowerMax(n,n.reference_q_-held)<1e-9,"X+B resumed to the old lower posture");
  require(armMax(n,n.reference_q_-held)>1e-4,"X+B did not resume arms");
  require(!query().ready,"unpaused manual admission allowed");
  std::cout<<"PASS manual target, stationary arms, limits, X+B new-pose resume: vmax="<<peak_v<<" amax="<<peak_a<<std::endl;
  m->arm_hold=true;++m->arm_operation_seq;for(int k=0;k<200;++k)step();
  auto active=set(.08);for(int k=0;k<100;++k)step();
  Service::Request cancel;Service::Response cr;cancel.command=cancel.CANCEL;cancel.request_id=active.request_id;
  n.manualLowerBodyService(cancel,cr);require(cr.accepted,"cancel rejected");
  for(int k=0;k<400;++k)step();
  require(query().state==Service::Response::ABORTED,"cancel did not stop");
  require(lowerMax(n,n.reference_dq_)<1e-8,"cancel left moving lower joints");
  const auto cancelled=n.reference_q_.eval();for(int k=0;k<100;++k)step();
  require(lowerMax(n,n.reference_q_-cancelled)<1e-9,"cancel auto-resumed");
  std::cout<<"PASS cancellation brakes and holds"<<std::endl;
  set(.05);for(int k=0;k<50;++k)step();
  ++m->arm_operation_seq; // even same hold bit invalidates the pending operation
  for(int k=0;k<200;++k)step();
  require(query().state==Service::Response::ABORTED,"operator change did not revoke adjustment");
  std::cout<<"PASS operator change revokes pending adjustment"<<std::endl;
  set(.05);for(int k=0;k<50;++k)step();
  fresh_target=false;
  for(int k=0;k<500;++k)step();
  require(query().state==Service::Response::ABORTED,"stale VR did not abort adjustment");
  require(lowerMax(n,n.reference_dq_)<1e-8,"stale VR did not brake");
  const auto stale_stopped=n.reference_q_.eval();fresh_target=true;
  for(int k=0;k<300;++k)step();
  require(query().state==Service::Response::ABORTED,"VR recovery retried adjustment");
  require(lowerMax(n,n.reference_q_-stale_stopped)<1e-9,"VR recovery moved lower joints");
  std::cout<<"PASS stale VR stops and recovery does not retry"<<std::endl;
 }
};
int main(int argc,char**argv){ros::init(argc,argv,"manual_lower_body_test",ros::init_options::NoSigintHandler);
 ros::NodeHandle p("~");p.setParam("robot_version",argc>1?std::stoi(argv[1]):62);p.setParam("enable_natural_hang_bootstrap",false);
 try{PinocchioVelocityIk n;VelocityIkOfflineTest::run(n,"normal");}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
