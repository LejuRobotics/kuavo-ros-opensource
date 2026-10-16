#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
using J=nlohmann::json;
J vec(const Eigen::VectorXd& x){return std::vector<double>(x.data(),x.data()+x.size());}
struct VelocityIkOfflineTest {
static void run(PinocchioVelocityIk& n,const J& seed,std::ostream& out,const std::string& scenario) {
 auto read=[&](const char* k){auto x=seed[k].get<std::vector<double>>();return Eigen::VectorXd(Eigen::Map<Eigen::VectorXd>(x.data(),x.size()));};
 // Recorded 2026-09-22 PREPARE boundary. Ideal feedback is a fixture,
 // not a physical robot replay; all output integration is production code.
 n.reference_q_=read("q");n.reference_dq_=read("v");n.reference_ddq_=read("a");
 if (scenario=="disabled") n.preparation_handoff_duration_=0.0;
 if (scenario=="stationary") {n.reference_dq_.setZero();n.reference_ddq_.setZero();}
 n.reference_initialized_=n.takeover_started_=n.enable_takeover_=true;
 n.robot_state_valid_=n.arm_mode_known_=true;n.current_arm_mode_=n.target_arm_mode_=2;n.arm_mode_transitioning_=false;
 n.session_anchor_.valid=true;n.session_anchor_.session_id=2;n.session_anchor_.generation=2;n.session_anchor_.base_coordinate_epoch=0;
 n.diagnostics_async_=false;n.publish_input_debug_=false;
 n.reference_pub_=n.nh_.advertise<wheel_arm_abs_vel::VelocityIkShadow>("/offline/reference",1);
 using Lease=velocity_ik_execution::ExecutionProducerLease;
 Lease::Feedback f;f.producer=n.execution_instance_;f.owner="offline-owner";f.valid=true;f.phase=Lease::Phase::Active;
 f.generated_ns=velocity_ik_execution::executionSteadyNs();f.lease_until_ns=f.generated_ns+3600000000000LL;
 n.execution_lease_.observe(f,true,true);
 using Msg=wheel_arm_abs_vel::TeleopTcpTarget;
 Msg::Ptr m(new Msg);m->header.frame_id="quest_chest_robot_axes";
 m->left_controlled_frame=n.left_controlled_frame_;m->right_controlled_frame=n.right_controlled_frame_;
 m->control_state=Msg::CONTROL_PREPARE;m->control_valid=true;m->left_valid=m->right_valid=true;
 m->left_tcp_pose.orientation.w=m->right_tcp_pose.orientation.w=1.;
 m->source=Msg::SOURCE_QUEST_CONTROLLER;m->session_id=2;m->generation=3;
 m->body_tracking_frame=n.body_tracking_frame_name_;m->body_high_confidence=true;
 m->chest_valid=true;m->chest_pose.orientation.w=1.;m->body_coordinate_epoch=1;
 for(int k=0;k<2500;++k) {
   n.measured_q_=n.reference_q_;n.measured_dq_=n.reference_dq_;
   auto now=ros::Time::now();n.robot_measurement_freshness_.observe(now.toNSec(),now.toNSec(),n.monotonicNs(),n.robot_state_timeout_);
   m->header.seq=k+1;m->header.stamp.fromSec(100.+k*.002);m->mapper_publish_stamp=now;
   if (scenario=="cancel" && k>=10) {m->control_state=Msg::CONTROL_HOLD;m->control_valid=false;}
   if (scenario=="reenter" && k==10) {m->control_state=Msg::CONTROL_HOLD;m->control_valid=false;}
   if (scenario=="reenter" && k==20) {m->control_state=Msg::CONTROL_PREPARE;m->control_valid=true;m->generation=4;}
   if (scenario=="resume" && k>=2000) {m->control_state=Msg::CONTROL_ACTIVE;m->control_valid=true;m->session_id=3;m->generation=4;}
   n.targetCallback(m);
   if (scenario=="stale" && k==12) {
     auto old=boost::make_shared<Msg>(*m);old->generation=2;old->header.stamp.fromSec(99.);old->mapper_publish_stamp=now;
     n.targetCallback(old);
     if(n.target_.generation!=3) throw std::runtime_error("stale generation replaced live target");
   }
   if (scenario=="terminal" && k==10) n.terminal_stop_requested_=true;
   auto q=n.reference_q_,v=n.reference_dq_,a=n.reference_ddq_;
   n.publishOnce();
   if (scenario=="resume" && k>=1990) ros::WallDuration(.002).sleep();
   if (maxAbs(n.reference_q_-q-.001*(v+n.reference_dq_))>1e-9 &&
       !(scenario=="terminal" && k>=10)) throw std::runtime_error("q/v integral mismatch");
   if (scenario=="terminal" && k>=10 && maxAbs(n.reference_q_-q)>0.)
     throw std::runtime_error("terminal stop allowed integration");
   if (scenario=="cancel" && k>=10 && n.preparation_active_)
     throw std::runtime_error("cancel left PREPARE active");
   if (scenario=="recorded" && k==0 && maxAbs(n.reference_ddq_-a)>25.)
     throw std::runtime_error("recorded PREPARE acceleration pulse not closed");
   if (n.dynamic_fault_latched_) throw std::runtime_error("dynamic fault in entry fixture");
   if (scenario=="recorded" && k<50 && maxAbs(n.reference_dq_)>3.20)
     throw std::runtime_error("handoff raised speed beyond accepted fixture margin");
   if (scenario=="resume" && k==2499 && n.tracking_ramp_scale_<0.5)
     throw std::runtime_error("resume never reacquired tracking");
   out<<J({{"k",k},{"q",vec(n.reference_q_)},{"v",vec(n.reference_dq_)},{"a",vec(n.reference_ddq_)},
   {"q_residual",maxAbs(n.reference_q_-q-.001*(v+n.reference_dq_))},
   {"delta_a",maxAbs(n.reference_ddq_-a)},{"preparing",n.preparation_active_},{"complete",n.preparation_complete_},
   {"fault",n.dynamic_fault_latched_},{"valid",n.target_.valid},{"generation",n.target_.generation},{"tracking_blend",n.tracking_ramp_scale_}}).dump()<<'\n';
 }
}
};
int main(int argc,char**argv){ros::init(argc,argv,"operating_replay",ros::init_options::NoSigintHandler);J j;std::ifstream(argv[1])>>j;PinocchioVelocityIk n;std::ofstream o(argv[2]);VelocityIkOfflineTest::run(n,j["prepare"]["before"],o,argc>3?argv[3]:"recorded");}
