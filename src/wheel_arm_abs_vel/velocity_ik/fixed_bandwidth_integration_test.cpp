// Actual production reference and Cartesian task construction; no actuator IO.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include <cassert>
using Json=nlohmann::json;
struct VelocityIkOfflineTest {
 using N=PinocchioVelocityIk;
 static std::pair<double,double> signal(int kind,double t) {
  if(kind==0)return {0.,0.};
  if(kind==1)return {.1*t,.1};
  if(kind==2)return {.1*std::min(t,1.),t<1.?.1:0.};
  if(kind==3)return {t<1.?.1*t:.2-.1*t,t<1.?.1:-.1};
  return {.08*std::sin(2.*M_PI*.7*t),.08*2.*M_PI*.7*std::cos(2.*M_PI*.7*t)};
 }
 static void run(N& n) {
  assert(n.hand_position_cutoff_hz_==5. && n.hand_orientation_cutoff_hz_==4.);
  assert(n.hand_position_gain_==12. && n.hand_position_weight_==12.);
  assert(n.hand_orientation_gain_==8. && n.hand_orientation_weight_==6.);
  assert(n.elbow_position_weight_==.15);
  assert(n.acquisition_linear_speed_==.4 && n.acquisition_angular_speed_==1.2);
  for(int arm=0;arm<2;++arm)assert(n.joint_jerk_limits_[n.armDecisionIndex(arm,3)]==25000.);
  Eigen::VectorXd q=n.preparation_position_;
  n.setPlantDecisionPositions(q);
  std::array<Eigen::Vector3d,2> origin;std::array<Eigen::Quaterniond,2> rotation;
  const auto& torso=n.plant_->GetFrameByName(n.torso_frame_name_);
  for(int a=0;a<2;++a){auto x=n.plant_->GetFrameByName(n.hand_frame_names_[a]).CalcPose(*n.plant_context_,torso);origin[a]=x.translation();rotation[a]=Eigen::Quaterniond(x.rotation().matrix());}
  Json results=Json::array();
  for(int rate: {72,120,0})for(int kind=0;kind<5;++kind) {
   for(auto& x:n.hand_reference_)x.valid=false;for(auto& x:n.elbow_reference_)x.valid=false;for(auto& x:n.hand_rotation_reference_)x.valid=false;
   N::TargetSnapshot input;input.valid=true;input.estimated_twists_valid=true;input.arm_target_torso_relative=true;
   double next=0.,sum_p=0.,sum_r=0.,sum_v=0.,sum_w=0.,residual=0.,rotation_residual=0.,task_residual=0.;int samples=0,packets=0;
   Eigen::Vector3d settled_v=Eigen::Vector3d::Zero(),settled_w=Eigen::Vector3d::Zero();int settled_count=0;
   auto previous=input;const double dt=.002,epoch=1.e12;Eigen::Vector3d axis(1.,2.,3.);axis.normalize();
   for(int k=0;k<2000;++k) {
    const double t=k*dt;
    if(t+1e-12>=next) {
     const auto z=signal(kind,t);
     input.observation_ns[0]=input.observation_ns[1]=epoch+t*1e9;
     input.source_frame_seq=++packets;
     for(int a=0;a<2;++a){auto& x=input.arm[a];x.hand_position=origin[a]+Eigen::Vector3d(z.first,0,0);x.hand_linear_ff=Eigen::Vector3d(z.second,0,0);x.hand_orientation=Eigen::Quaterniond(Eigen::AngleAxisd(2.*z.first,axis))*rotation[a];if(packets%2)x.hand_orientation.coeffs()*=-1.;x.hand_angular_ff=2.*z.second*axis;x.elbow_valid=true;x.elbow_position=Eigen::Vector3d(.1+z.first,.2,.3);x.elbow_linear_ff.setConstant(300.);}
     next+=rate?1./rate:std::array<double,5>{{.007,.014,.009,.019,.011}}[packets%5];
    }
    const auto hands_before=n.hand_reference_,elbows_before=n.elbow_reference_;
    const auto rotations_before=n.hand_rotation_reference_;
    auto world=input;n.shapeContinuousTarget(world,input,epoch+t*1e9,dt);
    // Observation age and an arbitrary reported twist must not bypass the
    // fixed shaper or create the removed 60 ms extrapolation.
    n.hand_reference_=hands_before;n.elbow_reference_=elbows_before;
    n.hand_rotation_reference_=rotations_before;
    auto counterfactual=input;
    for(auto& arm:counterfactual.arm){arm.hand_linear_ff.setConstant(100.);arm.hand_angular_ff.setConstant(-100.);}
    n.shapeContinuousTarget(counterfactual,input,epoch+(t+.060)*1e9,dt);
    for(int arm=0;arm<2;++arm){
      assert((counterfactual.arm[arm].hand_position-world.arm[arm].hand_position).norm()<1e-14);
      assert((counterfactual.arm[arm].hand_angular_ff-world.arm[arm].hand_angular_ff).norm()<1e-14);
    }
    DecisionMatrix h=DecisionMatrix::Zero(q.size(),q.size()),j;DecisionVector g=DecisionVector::Zero(q.size()),cmd;
    std::array<double,6> error{};std::array<double,2> sigma{},translation{};
    assert(n.calculateFullPoseTaskTerms(world,q,1.,1.,1.,false,&h,&g,&j,&cmd,&error,&sigma,&translation));
    int row=0;const auto truth=signal(kind,t);
    for(int a=0;a<2;++a){const auto& x=world.arm[a];
     assert((x.hand_linear_ff-n.hand_reference_[a].v).norm()<1e-14);
     assert((x.hand_angular_ff-n.hand_rotation_reference_[a].v).norm()<1e-14);
     assert((x.elbow_linear_ff-n.elbow_reference_[a].v).norm()<1e-14);
     assert((x.elbow_position-n.elbow_reference_[a].p).norm()<1e-14);
     if(n.armIsControlled(a)){
      task_residual=std::max(task_residual,(cmd.segment<3>(row)-(x.hand_linear_ff+n.hand_position_gain_*(x.hand_position-origin[a]))).norm());
      task_residual=std::max(task_residual,(cmd.segment<3>(row+3)-(x.hand_angular_ff+n.hand_orientation_gain_*rotationLog(x.hand_orientation.toRotationMatrix()*rotation[a].toRotationMatrix().transpose()))).norm());row+=6;
     }
     if(k>1){residual=std::max(residual,(x.hand_position-previous.arm[a].hand_position-.5*dt*(x.hand_linear_ff+previous.arm[a].hand_linear_ff)).norm());rotation_residual=std::max(rotation_residual,(rotationLog(x.hand_orientation.toRotationMatrix()*previous.arm[a].hand_orientation.toRotationMatrix().transpose())-.5*dt*(x.hand_angular_ff+previous.arm[a].hand_angular_ff)).norm());}
     if(k>250){sum_p+=std::pow((x.hand_position-origin[a]).x()-truth.first,2);sum_r+=std::pow(x.hand_orientation.angularDistance(Eigen::Quaterniond(Eigen::AngleAxisd(2.*truth.first,axis))*rotation[a]),2);sum_v+=std::pow(x.hand_linear_ff.x()-truth.second,2);sum_w+=(x.hand_angular_ff-2.*truth.second*axis).squaredNorm();++samples;}
    }
    if(k>=1750){settled_v+=world.arm[0].hand_linear_ff;settled_w+=world.arm[0].hand_angular_ff;++settled_count;}
    previous=world;
   }
   assert(task_residual<1e-10);assert(residual<2e-5);assert(rotation_residual<4e-5);
   if(kind==0 || kind==2)assert(previous.arm[0].hand_linear_ff.norm()<1e-9 && previous.arm[0].hand_angular_ff.norm()<1e-9);
   // Sample-and-hold input produces a bounded sub-frame velocity ripple.
   // Compare steady mean slope, not one arbitrary 500 Hz phase.
   if(kind==1 || kind==3){assert(std::abs(settled_v.x()/settled_count-signal(kind,4.).second)<.002);assert((settled_w/settled_count-2.*signal(kind,4.).second*axis).norm()<.004);}
   results.push_back({{"rate",rate},{"kind",kind},{"packets",packets},{"position_rms",std::sqrt(sum_p/samples)},{"rotation_rms",std::sqrt(sum_r/samples)},{"velocity_rms",std::sqrt(sum_v/samples)},{"omega_rms",std::sqrt(sum_w/samples)},{"integral_residual_max",residual},{"rotation_integral_residual_max",rotation_residual},{"qp_task_residual_max",task_residual}});
  }
  std::cout<<"PRODUCTION_REFERENCE_RESULTS "<<results.dump()<<'\n';
 }
};
int main(int argc,char**argv){ros::init(argc,argv,"absolute_velocity_ik",ros::init_options::NoSigintHandler);try{PinocchioVelocityIk n;VelocityIkOfflineTest::run(n);}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
