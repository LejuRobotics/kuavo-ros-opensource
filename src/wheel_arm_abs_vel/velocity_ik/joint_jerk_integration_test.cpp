#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
struct VelocityIkOfflineTest {
 using N=PinocchioVelocityIk;
 static void need(bool ok,const char* what){if(!ok)throw std::runtime_error(what);}
 static void checkReferences(N& n) {
  const double now=1.e12;
  N::TargetSnapshot target;target.valid=true;target.estimated_twists_valid=true;
  target.receive_monotonic_ns=now;
  for(int arm=0;arm<2;++arm) {
   target.hand_observation[arm].update_host_ns=now;
   target.hand_observation[arm].source_time=1.;
   target.arm[arm].hand_orientation=velocity_ik::innovationExp(Eigen::Vector3d(.4,-.3,.2));
   target.arm[arm].hand_angular_ff=Eigen::Vector3d(.8,-.5,.3);
  }
  const auto old_mode=n.orientation_reference_mode_;
  const auto old_continuous=n.continuous_reference_enabled_;
  const auto old_reconstruction=n.host_target_reconstruction_;
  n.orientation_reference_mode_="direct_task";n.continuous_reference_enabled_=true;
  for(int k=0;k<100;++k) {
   // Fresh frames first; then a genuine source gap. Same code for both hands.
   if(k<30 && k%7==0) {
    target.receive_monotonic_ns=now+k*2.e6;
    for(auto& h:target.hand_observation){h.update_host_ns=target.receive_monotonic_ns;h.source_time=1.+k*.002;}
   }
   auto world=target;n.shapeContinuousTarget(world,target,now+k*2.e6,.002);
   for(int arm=0;arm<2;++arm) {
    need(world.arm[arm].hand_orientation.angularDistance(target.arm[arm].hand_orientation)<1.e-12,"direct target was extrapolated/integrated");
    need((world.arm[arm].hand_angular_ff-n.hand_prediction_[arm].rate*target.arm[arm].hand_angular_ff).norm()<1.e-12,"direct FF changed outside silence budget");
    need(!n.hand_rotation_reference_[arm].valid,"unused rotation reference became active");
    if(k<30)need(std::abs(n.hand_prediction_[arm].rate-1.)<1.e-12,"normal VR interval decayed");
    if(k==99)need(n.hand_prediction_[arm].rate<.1,"stale FF did not decay");
   }
  }
  // Test the legacy host reconstruction combination, previously uncovered.
  n.continuous_reference_enabled_=false;n.host_target_reconstruction_=true;
  target.estimated_twists_valid=false;target.receive_monotonic_ns=now;
  for(auto& h:target.hand_observation){h.update_host_ns=now;h.source_time=1.;}
  auto world=target;n.prepareTargetForTracking(world,target,now+20.e6,.02,0.,true,true);
  for(int arm=0;arm<2;++arm)
   need(world.arm[arm].hand_orientation.angularDistance(target.arm[arm].hand_orientation)<1.e-12,"legacy host path bypassed direct mode");
  // Explicit rollback keeps the existing 6/3Hz continuous reference.
  n.orientation_reference_mode_="innovation";n.continuous_reference_enabled_=true;
  target.estimated_twists_valid=true;
  for(int arm=0;arm<2;++arm)n.hand_prediction_[arm].reset();
  world=target;n.shapeContinuousTarget(world,target,now,.002);
  for(int arm=0;arm<2;++arm)need(n.hand_rotation_reference_[arm].valid,"innovation rollback inactive");
  n.orientation_reference_mode_=old_mode;n.continuous_reference_enabled_=old_continuous;n.host_target_reconstruction_=old_reconstruction;
  std::cout<<"PASS direct reference both arms, normal frames, silence, legacy host and innovation rollback"<<std::endl;
 }
 static void run(N& n) {
  checkReferences(n);
  n.envelope_diagnostics_stride_=1;
  n.joint_jerk_enabled_=true;n.temporal_cost_enabled_=n.operating_envelope_enabled_=true;
  const int nd=n.decisionDofCount();auto q=n.preparation_position_;
  Eigen::VectorXd v=Eigen::VectorXd::Zero(nd),a=v;const auto seed=q;
  int relaxed=0,governor_active=0;
  for(int k=0;k<1200;++k) {
   N::SolverRequest r;r.reference_q=q;r.reference_dq=v;r.reference_ddq=a;
   r.tracking=true;r.cycle_dt=k%3==0?.001:k%3==1?.002:.003;r.base_reference_seq=k;
   r.feedforward_age=r.body_feedforward_age=0.;r.lower_body_recruitment=1.;r.lower_joint_recruitment.fill(1.);
   r.measured_q=q;r.measured_dq=v;r.measured_state_valid=true;r.governor_risk=n.governor_risk_;
   n.execution_governor_enabled_=k>=400;
   if(k>=400&&k<800)for(int arm=0;arm<2;++arm) {
    r.measured_q(n.governor_indices_[arm])-=.60;r.measured_dq(n.governor_indices_[arm])-=.3;
   }
   auto goal=seed;for(int arm=0;arm<2;++arm)goal(n.armDecisionIndex(arm,0))+=k<400?.08:k<800?-.08:0.;
   n.setPlantDecisionPositions(goal);const auto& torso=n.plant_->GetFrameByName(n.torso_frame_name_);
   r.target.valid=r.target.arm_target_torso_relative=true;
   for(int arm=0;arm<2;++arm){auto pose=n.plant_->GetFrameByName(n.hand_frame_names_[arm]).CalcPose(*n.plant_context_,torso);r.target.arm[arm].hand_position=pose.translation();r.target.arm[arm].hand_orientation=Eigen::Quaterniond(pose.rotation().matrix());}
   r.target.body_intent.torso_valid=true;r.target.body_intent.desired_torso_feature=n.torsoFeatureInContext(seed,n.plant_context_.get(),&n.q_full_);
   n.offline_reject_jerk_solve_=k==700;auto c=n.solveRequest(r);n.offline_reject_jerk_solve_=false;
   need(c.solver_valid&&c.dynamic_certificate_valid,"actual QP failed");need(c.joint_jerk_active,"normal jerk path inactive");
   if(k==700)need(c.joint_jerk_retry&&c.joint_jerk_first_code==-1999,"safety retry not observed");
   const auto hard=n.calculateHardBounds(r);need(hard.valid,"original hard bounds invalid");
   need((c.certified_dynamic_lower.array()>=hard.lower.array()-1e-10).all()&&(c.certified_dynamic_upper.array()<=hard.upper.array()+1e-10).all(),"jerk widened original safety box");
   for(int j=0;j<14;++j){const auto& d=c.joint_jerk_diagnostics;need(d[9*j]==n.armDecisionIndex(j/7,j%7),"arm index mismatch");if(d[9*j+6]==0.)need(std::abs(d[9*j+4])<=d[9*j+1]+.03,"unreported jerk violation");else ++relaxed;}
   for(size_t j=0;j<c.governor_diagnostics.size();j+=18)governor_active+=c.governor_diagnostics[j+12]>0.;
   auto u=c.task_dq;Eigen::VectorXd na=(u-v)/r.cycle_dt;q+=.5*r.cycle_dt*(v+u);v=u;a=na;n.governor_risk_=c.governor_risk;
   need((q.array()>=n.position_lower_.array()-1e-8).all()&&(q.array()<=n.position_upper_.array()+1e-8).all(),"integrated position outside safety bounds");
  }
  need(governor_active>0,"augmented governor QP was not covered");
  std::cout<<"PASS actual normal/governor QP 1200 variable-step ticks, forced safety retry, relaxed_axis_ticks="<<relaxed<<" governor_active="<<governor_active<<'\n';
 }
};
int main(int argc,char**argv){ros::init(argc,argv,"joint_jerk_test",ros::init_options::NoSigintHandler);try{PinocchioVelocityIk n;if(argc>1&&std::string(argv[1])=="--validate-params")return 0;VelocityIkOfflineTest::run(n);}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
