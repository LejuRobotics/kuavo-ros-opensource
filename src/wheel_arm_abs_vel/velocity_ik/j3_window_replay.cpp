// Offline replay of recorded solved-target snapshots through the REAL node QP.
// No ROS subscriptions, control publishers, CAN, SHM attachment or robot process.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include "wheel_arm_abs_vel/humanoid_controllers/ExecutionReferenceOwner.h"
#include <fstream>
#include <sstream>
struct ReplayReference {
 Eigen::VectorXd q,dq,ddq,stopping_acceleration,position_lower,position_upper;
 uint64_t reference_seq=0,execution_epoch=0,recovery_anchor_seq=0;
 bool safety_braking=false,takeover_active=true;
 std::string producer_instance="offline-ik",execution_owner_instance="offline-owner";
 int64_t producer_lease_deadline_ns=0;
};
struct VelocityIkOfflineTest {
 using N=PinocchioVelocityIk;
 static void need(bool x,const std::string& text){if(!x)throw std::runtime_error(text);}
 static void run(N& n,const char* source,const char* output,bool recorded_feedback) {
  std::ifstream input(source);need(bool(input),"missing replay input");
  std::ofstream out(output);need(bool(out),"cannot open output");
  std::vector<std::vector<double>> ticks;std::string line;
  while(std::getline(input,line)) {
   std::replace(line.begin(),line.end(),',',' ');std::istringstream stream(line);
   std::vector<double> row;double x;while(stream>>x)row.push_back(x);
   need(row.size()==102,"replay input must have 102 columns");ticks.push_back(row);
  }
  need(!ticks.empty(),"empty replay");
  Eigen::VectorXd q=Eigen::Map<const Eigen::VectorXd>(ticks[0].data()+1,21);
  Eigen::VectorXd v=Eigen::VectorXd::Zero(21),a=v;
  velocity_ik_execution::ExecutionReferenceOwner<ReplayReference> owner("offline-owner");
  bool owned=true;int diag_count=0,jerk_relaxed=0,emergency_count=0;double vmax=0.,amax=0.,jmax=0.;
  int j3_jerk_relaxed=0;double jmax_unrelaxed=0.;const double dt=.002;
  for(size_t k=0;k<ticks.size();++k) {
   const auto& row=ticks[k];
   N::SolverRequest r;r.reference_q=q;r.reference_dq=v;r.reference_ddq=a;
   r.tracking=true;r.cycle_dt=dt;r.base_reference_seq=k;
   r.feedforward_age=r.body_feedforward_age=0.;
   r.lower_body_recruitment=1.;r.lower_joint_recruitment.fill(1.);
   r.measured_q=recorded_feedback?Eigen::VectorXd(Eigen::Map<const Eigen::VectorXd>(row.data()+22,21)):q;
   r.measured_dq=recorded_feedback?Eigen::VectorXd(Eigen::Map<const Eigen::VectorXd>(row.data()+43,21)):v;
   r.measured_state_valid=true;r.governor_risk=n.governor_risk_;
   auto& target=r.target;target.valid=target.arm_target_torso_relative=true;
   target.session_id=1;target.generation=1;target.sequence=k;target.source_stamp.fromSec(1+k*dt);
   for(int arm=0;arm<2;++arm) {
    const double* pose=row.data()+64+10*arm;const double* ff=row.data()+84+9*arm;
    auto& t=target.arm[arm];
    t.hand_position=Eigen::Map<const Eigen::Vector3d>(pose);
    t.hand_orientation=Eigen::Quaterniond(pose[6],pose[3],pose[4],pose[5]).normalized();
    t.elbow_position=Eigen::Map<const Eigen::Vector3d>(pose+7);t.elbow_valid=true;
    t.hand_linear_ff=Eigen::Map<const Eigen::Vector3d>(ff);
    t.hand_angular_ff=Eigen::Map<const Eigen::Vector3d>(ff+3);
    t.elbow_linear_ff=Eigen::Map<const Eigen::Vector3d>(ff+6);
   }
   // The extraction lacks the shaped body-intent field. Reconstruct its torso
   // feature from recorded lower-body reference FK, keeping waist motion present.
   const Eigen::VectorXd recorded_q=Eigen::Map<const Eigen::VectorXd>(row.data()+1,21);
   target.body_intent.torso_valid=true;
   target.body_intent.desired_torso_feature=n.torsoFeatureInContext(recorded_q,n.plant_context_.get(),&n.q_full_);
   const auto c=n.solveRequest(r);const auto hard=n.calculateHardBounds(r);
   need(c.solver_valid && c.dynamic_certificate_valid && hard.valid,
        "QP/certificate failure at "+std::to_string(row[0])+" code "+std::to_string(c.return_code));
   if(c.emergency_acceleration_used) {
    ++emergency_count;
    if(emergency_count<6) {
     std::cerr<<"emergency tick "<<row[0]<<" normal interval gaps:";
     for(int i=0;i<21;++i) {
      const double lo=std::max(hard.stopping_lower(i),v(i)-n.acceleration_limit_(i)*dt);
      const double hi=std::min(hard.stopping_upper(i),v(i)+n.acceleration_limit_(i)*dt);
      if(lo>hi)std::cerr<<" axis="<<i<<" gap="<<std::setprecision(17)<<lo-hi;
     }
     std::cerr<<'\n';
    }
   }
   need((c.task_dq.array()>=c.certified_dynamic_lower.array()-2e-8).all() &&
        (c.task_dq.array()<=c.certified_dynamic_upper.array()+2e-8).all(),"velocity certificate violation");
   const auto na=((c.task_dq-v)/dt).eval();
   const auto nq=(q+.5*dt*(v+c.task_dq)).eval();
   need((nq.array()>=n.position_lower_.array()-1e-7).all() &&
        (nq.array()<=n.position_upper_.array()+1e-7).all(),"position violation");
   need(c.diagnostics_due==(k%size_t(n.envelope_diagnostics_stride_)==0),"candidate identity gate mismatch");
   if(c.diagnostics_due) {
    ++diag_count;need(c.operating_diagnostics.size()==700,"missing full diagnostic");
    for(int j=0;j<14;++j)jerk_relaxed+=c.joint_jerk_diagnostics[9*j+6]>0.;
   } else need(c.operating_diagnostics.empty(),"diagnostic constructed on skipped cycle");
   ReplayReference ref;ref.q=nq;ref.dq=c.task_dq;ref.ddq=na;
   ref.stopping_acceleration=n.braking_limit_;ref.position_lower=n.position_lower_;ref.position_upper=n.position_upper_;
   need(ref.stopping_acceleration.allFinite() && (ref.stopping_acceleration.array()>0.).all(),"CDM stopping contract");
   ref.reference_seq=k+1;ref.producer_lease_deadline_ns=1000000000LL+k*2000000LL+50000000LL;
   ReplayReference accepted;
   need(owner.resolve(ref,true,owned,1000000000LL+k*2000000LL,50000000LL,dt,accepted),"owner rejected");
   need(!owner.degrading() && !accepted.safety_braking,"unexpected owner braking");
   need((accepted.q-ref.q).norm()==0 && (accepted.stopping_acceleration-ref.stopping_acceleration).norm()==0,"owner altered contract");
   std::array<bool,2> j3_relaxed{};
   for (int arm=0;arm<2;++arm) {
    const int i=arm==0?9:16;
    const auto jerk=velocity_ik::jointJerkInterval(hard.lower(i),hard.upper(i),v(i),a(i),dt,n.joint_jerk_limits_(i));
    need(jerk.valid,"invalid J3 jerk interval");
    j3_relaxed[arm]=jerk.safety_relaxed;
    const double actual_jerk=std::abs((na(i)-a(i))/dt);
    if (!jerk.safety_relaxed) {
     need(actual_jerk<=n.joint_jerk_limits_(i)+.03,"unreported J3 jerk violation");
     if(row[0]>=0. && row[0]<=10.)jmax_unrelaxed=std::max(jmax_unrelaxed,actual_jerk);
    } else if(row[0]>=0. && row[0]<=10.) ++j3_jerk_relaxed;
   }
   if(row[0]>=0. && row[0]<=10.) {
    for(int i:{9,16}) {
     vmax=std::max(vmax,std::abs(c.task_dq(i)));amax=std::max(amax,std::abs(na(i)));
     jmax=std::max(jmax,std::abs((na(i)-a(i))/dt));
     need(std::abs(c.task_dq(i))<=n.velocity_limit_(i)+2e-8,"J3 speed violation");
     need(std::abs(na(i))<=n.acceleration_limit_(i)+2e-6,"J3 acceleration violation");
    }
   }
   auto arr=[](const auto& x){return std::vector<double>(x.data(),x.data()+x.size());};
   out<<nlohmann::json{{"t",row[0]},{"q",arr(nq)},{"v",arr(c.task_dq)},{"a",arr(na)},
        {"braking",arr(n.braking_limit_)},{"qp",c.return_code},{"owner_phase",0},
        {"safety_braking",accepted.safety_braking},{"emergency_dynamics_used",c.emergency_acceleration_used},{"diagnostics_due",c.diagnostics_due},{"j3_jerk_safety_relaxed",j3_relaxed}}.dump()<<'\n';
   q=nq;v=c.task_dq;a=na;n.governor_risk_=c.governor_risk;
  }
  nlohmann::json result{{"ticks",ticks.size()},{"recorded_feedback",recorded_feedback},
      {"j3_velocity_max",vmax},{"j3_acceleration_max",amax},{"j3_jerk_max",jmax},{"j3_jerk_unrelaxed_max",jmax_unrelaxed},{"j3_jerk_safety_relaxed_axis_ticks",j3_jerk_relaxed},
      {"diagnostic_rows",diag_count},{"diagnosed_jerk_relaxations",jerk_relaxed},
      {"owner_rejections",0},{"unexpected_safety_braking",0},{"qp_failures",0},{"emergency_dynamics_ticks",emergency_count},
      {"initial_velocity","zero; one second recorded pre-window lead-in"},
      {"body_intent","reconstructed from recorded lower-body reference FK; not full lifecycle replay"}};
  std::ofstream(std::string(output)+".summary.json")<<result.dump(2)<<'\n';std::cout<<result.dump(2)<<'\n';
 }
};
int main(int argc,char** argv) {
 ros::init(argc,argv,"absolute_velocity_ik",ros::init_options::NoSigintHandler);
 try {if(argc<3)throw std::runtime_error("usage: replay input.csv output.jsonl [recorded]");
  PinocchioVelocityIk n;VelocityIkOfflineTest::run(n,argv[1],argv[2],argc>3&&std::string(argv[3])=="recorded");
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;
}
