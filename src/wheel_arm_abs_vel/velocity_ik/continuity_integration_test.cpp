// Actual callback + actual QP; production publishers/subscribers compiled out.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"

struct VelocityIkOfflineTest {
  using Node=PinocchioVelocityIk;
  static void require(bool ok,const char* what) { if(!ok) throw std::runtime_error(what); }
  static void inputs(Node& node) {
    // Use the production watchdog predicate, not a duplicate test policy.
    node.lower_reference_tracking_error_=0.;
    for (double arm_error : {0., .33855, .482271, .523218, .642541, .653305, 2.}) {
      node.arm_reference_tracking_error_=arm_error;
      require(!node.executionTrackingRequiresHold(),"arm lag reintroduced motion hold");
    }
    node.lower_reference_tracking_error_=node.lower_reference_tracking_hard_limit_+.01;
    require(node.executionTrackingRequiresHold(),"lower-body execution protection removed");
    node.arm_reference_tracking_error_=node.lower_reference_tracking_error_=0.;
    node.publish_input_debug_=false;
    using Msg=wheel_arm_abs_vel::TeleopTcpTarget;
    Msg::Ptr m(new Msg);
    m->header.stamp.fromSec(10.); m->header.frame_id="quest_chest_robot_axes";
    m->left_controlled_frame=node.left_controlled_frame_; m->right_controlled_frame=node.right_controlled_frame_;
    m->control_state=Msg::CONTROL_ACTIVE; m->control_valid=true;
    m->left_valid=m->right_valid=true;
    m->left_tcp_pose.orientation.w=m->right_tcp_pose.orientation.w=1.;
    m->source=Msg::SOURCE_QUEST_CONTROLLER; m->session_id=1; m->generation=1;
    m->body_tracking_frame=node.body_tracking_frame_name_;
    m->body_high_confidence=true; m->chest_valid=true; m->chest_pose.orientation.w=1.;
    const auto send=[&] { m->mapper_publish_stamp=ros::Time::now(); node.targetCallback(m); };
    send(); require(node.target_.valid,"initial observation rejected");
    const auto initial=node.target_;
    send(); require(node.target_.observation_ns==initial.observation_ns,"duplicate renewed pose");
    m->header.stamp.fromSec(10.02); send();
    require(node.target_.observation_sequence>initial.observation_sequence,"stationary new frame rejected");
    const auto before=node.target_;
    m->source_relative_age=1.; m->header.stamp.fromSec(10.03); send();
    require(node.target_.valid && node.target_.observation_ns==before.observation_ns,"stale packet destroyed recent pose");
    m->source_relative_age=0.;
    m->left_valid=false; m->header.stamp.fromSec(10.04); send();
    require(node.target_.valid,"one invalid hand destroyed cached target");
    require(node.target_.observation_ns[0]==before.observation_ns[0],"invalid hand renewed age");
    require(node.target_.observation_ns[1]>before.observation_ns[1],"valid right hand not updated");
    m->chest_valid=false; m->header.stamp.fromSec(10.06); send();
    require(node.target_.valid,"single invalid torso destroyed cache");
    m->source=Msg::SOURCE_QUEST_HAND_TRACKING; m->header.stamp.fromSec(10.08); send();
    require(node.target_source_active_,"wrong-source packet immediately stopped configured source");
    m->source=Msg::SOURCE_QUEST_CONTROLLER;
    m->control_state=Msg::CONTROL_HOLD; m->control_valid=false; send();
    require(!node.target_.valid,"explicit HOLD ignored");
    m->control_state=Msg::CONTROL_ACTIVE; m->control_valid=true;
    m->generation=2; m->header.stamp.fromSec(10.1); send();
    require(!node.target_.valid,"invalid new generation borrowed old cache");
    std::cout<<"PASS actual input callback cache, duplicates, stationary frames, wrong source, explicit stop and epoch isolation\n";
  }
  static void solves(Node& node, bool release=false, bool distant=false) {
    node.execution_governor_enabled_=true;
    node.operating_envelope_enabled_=node.temporal_cost_enabled_=true;
    const int n=node.decisionDofCount();
    Eigen::VectorXd q=node.preparation_position_, v=Eigen::VectorXd::Zero(n), a=v;
    const auto seed=q;
    velocity_ik::Acquisition acquisition;
    double max_joint=0.,max_linear=0.,max_angular=0.,max_time=0.,max_transparent_delta=0.;
    for(int k=0;k<2000;++k) {
      Node::SolverRequest r;
      r.reference_q=q; r.reference_dq=v; r.reference_ddq=a; r.tracking=true;
      r.measured_q=q; r.measured_dq=v; r.measured_state_valid=true;
      r.cycle_dt=.002; r.feedforward_age=r.body_feedforward_age=0.;
      r.lower_joint_recruitment.fill(1.); r.lower_body_recruitment=1.;
      r.base_reference_seq=k; r.acquisition_blend=(release || distant) ? acquisition.blend() : 0.;
      r.catchup_correction=true;
      // Recovering/degrading at nonzero speed also exercises budget feasibility.
      r.input_budget=!release && k>=1000 && k<1100 ? (1100.-k)/100. : 1.;
      auto goal=seed;
      for(int arm=0;arm<2;++arm) goal(node.armDecisionIndex(arm,0))+=release ? .04 : .5;
      node.setPlantDecisionPositions(goal);
      const auto& torso=node.plant_->GetFrameByName(node.torso_frame_name_);
      r.target.valid=r.target.arm_target_torso_relative=true;
      for(int arm=0;arm<2;++arm) {
        auto& target=r.target.arm[arm];
        const auto pose=node.plant_->GetFrameByName(node.hand_frame_names_[arm]).CalcPose(*node.plant_context_,torso);
        target.hand_position=pose.translation(); target.hand_orientation=Eigen::Quaterniond(pose.rotation().matrix());
        // Intentionally large/unreachable target during the final interval.
        if(!release && k>1400) target.hand_position.x()+=1.;
        if(distant && k>=500) target.hand_linear_ff.y()=.20;
      }
      r.target.body_intent.torso_valid=true;
      r.target.body_intent.desired_torso_feature=node.torsoFeatureInContext(seed,node.plant_context_.get(),&node.q_full_);
      if(!release) r.target.body_intent.desired_torso_feature(3)+=.05;
      const auto c=node.solveRequest(r);
      if(k%100==0) {
        node.execution_governor_enabled_=false;
        const auto baseline=node.solveRequest(r);
        node.execution_governor_enabled_=true;
        max_transparent_delta=std::max(max_transparent_delta,
            (baseline.task_dq-c.task_dq).cwiseAbs().maxCoeff());
        if(!baseline.solver_valid || !c.solver_valid ||
            (baseline.task_dq-c.task_dq).cwiseAbs().maxCoeff()>=1e-5)
          std::cerr<<"healthy comparison tick="<<k<<" base_code="<<baseline.return_code
            <<" governor_code="<<c.return_code<<" max_delta="
            <<(baseline.task_dq-c.task_dq).cwiseAbs().maxCoeff()<<'\n';
        require(baseline.solver_valid && c.solver_valid &&
            (baseline.task_dq-c.task_dq).cwiseAbs().maxCoeff()<1e-5,
            "healthy full IK changed when governor enabled");
        for(std::size_t offset=0;offset<c.governor_diagnostics.size();offset+=18)
          require(c.governor_diagnostics[offset+12]==0. && c.governor_diagnostics[offset+13]==0.,
              "healthy governor did not use exact original-QP fast path");
      }
      if(!c.solver_valid) {
        const auto s=node.weighted_qp_.stats();
        std::cerr<<"tick="<<k<<" code="<<c.return_code<<" native="<<s.native_status
          <<" violation="<<s.violation<<" stationarity="<<s.stationarity<<" comp="<<s.complementarity<<'\n';
        const auto b=node.calculateHardBounds(r);
        std::cerr<<std::setprecision(17)<<"q "<<q.transpose()<<"\nv "<<v.transpose()
            <<"\nlo "<<b.lower.transpose()<<"\nhi "<<b.upper.transpose()<<'\n';
        throw std::runtime_error("actual acquisition QP failed");
      }
      require(c.dynamic_certificate_valid,"missing certificate");
      require((c.task_dq.array()>=c.certified_dynamic_lower.array()-2e-8).all() &&
          (c.task_dq.array()<=c.certified_dynamic_upper.array()+2e-8).all(),"joint certificate violation");
      const Eigen::VectorXd rates=c.certified_constraint_matrix*c.task_dq;
      require((rates.array()>=c.certified_constraint_lower.array()-2e-8).all() &&
          (rates.array()<=c.certified_constraint_upper.array()+2e-8).all(),"TCP/torso certificate violation");
      max_joint=std::max(max_joint,c.task_dq.cwiseAbs().maxCoeff());
      node.setPlantDecisionPositions(q);
      for(int arm=0;arm<2;++arm) {
        max_linear=std::max(max_linear,(node.frameTranslationJacobian(node.hand_frame_names_[arm])*c.task_dq).norm());
        max_angular=std::max(max_angular,(node.frameAngularJacobian(node.hand_frame_names_[arm])*c.task_dq).norm());
      }
      const auto u=c.task_dq;
      if(k==0) require(u.cwiseAbs().maxCoeff()<=node.acquisition_acceleration_*r.cycle_dt+2e-8,
          "first command exceeded startup acceleration budget");
      if(distant && k>=501) {
        require(r.acquisition_blend==1.,"distant target stalled joint/FF startup release");
        if(r.input_budget==1.) {
          node.setPlantDecisionPositions(q);
          const auto pose=node.plant_->GetFrameByName(node.hand_frame_names_[0]).CalcPose(*node.plant_context_,torso);
          Eigen::Vector3d correction=node.hand_position_gain_*(r.target.arm[0].hand_position-pose.translation());
          if(correction.norm()>node.acquisition_linear_speed_) correction*=node.acquisition_linear_speed_/correction.norm();
          require((c.primary_task_velocity.head<3>()-r.target.arm[0].hand_linear_ff-correction).norm()<1e-8,
              "catchup incorrectly throttled valid live feedforward");
        }
      }
      acquisition.update(r.cycle_dt,c.task_error[0]<.03 && c.task_error[1]<.15 &&
          c.task_error[3]<.03 && c.task_error[4]<.15,r.input_budget==1.);
      a=(u-v)/r.cycle_dt; q+=.5*r.cycle_dt*(v+u); v=u;
      require(q.allFinite() && (q.array()>=node.position_lower_.array()-1e-7).all() &&
          (q.array()<=node.position_upper_.array()+1e-7).all(),"position bounds violation");
      max_time=std::max(max_time,c.solve_time_us);
    }
    if (!release && !distant) {
      require(max_joint<=node.acquisition_joint_speed_+2e-7,"acquisition joint cap");
    } else require(acquisition.progress==1.,"aligned acquisition did not release");
    std::cout<<"PASS actual QP 2000 ticks release="<<release<<" distant="<<distant<<": joint="<<max_joint<<" linear="<<max_linear<<" angular="<<max_angular<<" max_solve_us="<<max_time<<" transparent_max_delta="<<max_transparent_delta<<'\n';
  }
  static void governor(Node& node) {
    const int n=node.decisionDofCount();
    Node::SolverRequest r;
    r.reference_q=node.preparation_position_;
    r.reference_dq=Eigen::VectorXd::Zero(n);r.reference_ddq=r.reference_dq;
    r.measured_q=r.reference_q;r.measured_dq=r.reference_dq;
    r.tracking=true;r.cycle_dt=.002;
    r.feedforward_age=r.body_feedforward_age=0.;r.lower_joint_recruitment.fill(1.);
    r.target.valid=r.target.arm_target_torso_relative=true;
    auto goal=r.reference_q;
    for(int arm=0;arm<2;++arm) goal(node.governor_indices_[arm])+=.20;
    node.setPlantDecisionPositions(goal);
    const auto& torso=node.plant_->GetFrameByName(node.torso_frame_name_);
    for(int arm=0;arm<2;++arm) {
      const auto pose=node.plant_->GetFrameByName(node.hand_frame_names_[arm]).CalcPose(*node.plant_context_,torso);
      r.target.arm[arm].hand_position=pose.translation();
      r.target.arm[arm].hand_orientation=Eigen::Quaterniond(pose.rotation().matrix());
    }
    node.execution_governor_enabled_=true;
    double peak=0.,max_slack=0.;
    for(int k=0;k<600;++k) {
      // Includes severe plant overshoot, stale feedback bypass and both signs.
      r.measured_state_valid=k%17!=0;
      for(int arm=0;arm<2;++arm) {
        const int i=node.governor_indices_[arm];
        r.measured_q(i)=r.reference_q(i)-1.8*std::sin(.03*k+arm);
        r.measured_dq(i)=8.*std::cos(.03*k+arm);
      }
      const auto c=node.solveRequest(r);
      require(c.solver_valid && c.dynamic_certificate_valid,"governed full IK failed");
      require(c.governor_diagnostics.size()==(r.measured_state_valid?36:0),"governor feedback validity bypass");
      require((c.task_dq.array()>=c.certified_dynamic_lower.array()-2e-8).all() &&
          (c.task_dq.array()<=c.certified_dynamic_upper.array()+2e-8).all(),"governor broke hard bounds");
      const auto rates=(c.certified_constraint_matrix*c.task_dq).eval();
      require((rates.array()>=c.certified_constraint_lower.array()-2e-8).all() &&
          (rates.array()<=c.certified_constraint_upper.array()+2e-8).all(),"governor broke torso certificate");
      for(std::size_t offset=0;offset<c.governor_diagnostics.size();offset+=18)
        max_slack=std::max(max_slack,std::max(c.governor_diagnostics[offset+9],c.governor_diagnostics[offset+10]));
      r.governor_risk=c.governor_risk;
      peak=std::max(peak,c.solve_time_us);
    }
    require(max_slack>1e-6,"test did not exercise active soft constraints");
    for(int arm=0;arm<2;++arm) {
      node.control_arm_index_=arm;r.measured_state_valid=true;
      const int i=node.governor_indices_[arm];
      r.measured_q(i)=r.reference_q(i)-1.;r.measured_dq(i)=-8.;
      const auto c=node.solveRequest(r);
      require(c.solver_valid && c.governor_diagnostics.size()==18 &&
          c.governor_diagnostics[17]==arm,"single-arm governor workspace/mapping failed");
    }
    node.control_arm_index_=2;
    node.governor_risk_.fill(1.);
    velocity_ik::StartupReadiness::Values startup_q{}, startup_v{};
    for(int i=0;i<n;++i)startup_q[i]=node.preparation_position_(i);
    for(int k=0;k<100;++k)node.startup_readiness_.update(1.+.001*k,startup_q,startup_v,true);
    node.resetReference(node.preparation_position_,Eigen::VectorXd::Zero(n));
    require(node.governor_risk_[0]==0. && node.governor_risk_[1]==0.,"reference reset retained risk");
    std::cout<<"PASS actual governed QP 600 adversarial snapshots; max_slack="<<max_slack
      <<" max_solve_us="<<peak<<" (not a physical closed-loop simulation)\n";
  }
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"continuity_test",ros::init_options::NoSigintHandler);
  try { PinocchioVelocityIk node; VelocityIkOfflineTest::inputs(node); VelocityIkOfflineTest::solves(node); VelocityIkOfflineTest::solves(node,true); VelocityIkOfflineTest::solves(node,false,true); VelocityIkOfflineTest::governor(node); }
  catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
  return 0;
}
