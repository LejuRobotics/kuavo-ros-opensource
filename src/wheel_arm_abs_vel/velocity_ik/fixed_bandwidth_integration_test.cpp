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
 static void run(N& n,const Json& seed) {
  assert(n.hand_position_cutoff_hz_==20. && n.hand_orientation_cutoff_hz_==16.);
  Eigen::VectorXd q(n.decisionDofCount());for(int i=0;i<q.size();++i)q[i]=seed[i][0];
  n.setPlantDecisionPositions(q);
  std::array<Eigen::Vector3d,2> origin;std::array<Eigen::Quaterniond,2> rotation;
  const auto& torso=n.plant_->GetFrameByName(n.torso_frame_name_);
  for(int a=0;a<2;++a){auto x=n.plant_->GetFrameByName(n.hand_frame_names_[a]).CalcPose(*n.plant_context_,torso);origin[a]=x.translation();rotation[a]=Eigen::Quaterniond(x.rotation().matrix());}
  Json results=Json::array();
  for(int rate: {72,120,0})for(int kind=0;kind<5;++kind) {
   for(auto& x:n.hand_reference_)x.valid=false;for(auto& x:n.hand_rotation_reference_)x.valid=false;
   N::TargetSnapshot input;input.valid=true;input.estimated_twists_valid=true;input.arm_target_torso_relative=true;
   double next=0.,sum_p=0.,sum_r=0.,sum_v=0.,sum_w=0.,residual=0.,rotation_residual=0.,task_residual=0.;int samples=0,packets=0;
   auto previous=input;const double dt=.002,epoch=1.e12;Eigen::Vector3d axis(1.,2.,3.);axis.normalize();
   for(int k=0;k<2000;++k) {
    const double t=k*dt;
    if(t+1e-12>=next) {
     const auto z=signal(kind,t);
     input.observation_ns[0]=input.observation_ns[1]=epoch+t*1e9;
     input.source_frame_seq=++packets;
     for(int a=0;a<2;++a){auto& x=input.arm[a];x.hand_position=origin[a]+Eigen::Vector3d(z.first,0,0);x.hand_linear_ff=Eigen::Vector3d(z.second,0,0);x.hand_orientation=Eigen::Quaterniond(Eigen::AngleAxisd(2.*z.first,axis))*rotation[a];if(packets%2)x.hand_orientation.coeffs()*=-1.;x.hand_angular_ff=2.*z.second*axis;x.elbow_position=Eigen::Vector3d(.1,.2,.3);x.elbow_linear_ff.setConstant(300.);}
     next+=rate?1./rate:std::array<double,5>{{.007,.014,.009,.019,.011}}[packets%5];
    }
    auto world=input;n.shapeContinuousTarget(world,input,epoch+t*1e9,dt);
    DecisionMatrix h=DecisionMatrix::Zero(q.size(),q.size()),j;DecisionVector g=DecisionVector::Zero(q.size()),cmd;
    std::array<double,6> error{};std::array<double,2> sigma{},translation{};
    assert(n.calculateFullPoseTaskTerms(world,q,1.,1.,1.,false,&h,&g,&j,&cmd,&error,&sigma,&translation));
    int row=0;const auto truth=signal(kind,t);
    for(int a=0;a<2;++a){const auto& x=world.arm[a];
     assert((x.hand_linear_ff-n.hand_reference_[a].v).norm()<1e-14);
     assert((x.hand_angular_ff-n.hand_rotation_reference_[a].v).norm()<1e-14);
     assert(x.elbow_linear_ff.isZero() && x.elbow_position==input.arm[a].elbow_position);
     if(n.armIsControlled(a)){
      task_residual=std::max(task_residual,(cmd.segment<3>(row)-(x.hand_linear_ff+n.hand_position_gain_*(x.hand_position-origin[a]))).norm());
      task_residual=std::max(task_residual,(cmd.segment<3>(row+3)-(x.hand_angular_ff+n.hand_orientation_gain_*rotationLog(x.hand_orientation.toRotationMatrix()*rotation[a].toRotationMatrix().transpose()))).norm());row+=6;
     }
     if(k>1){residual=std::max(residual,(x.hand_position-previous.arm[a].hand_position-.5*dt*(x.hand_linear_ff+previous.arm[a].hand_linear_ff)).norm());rotation_residual=std::max(rotation_residual,(rotationLog(x.hand_orientation.toRotationMatrix()*previous.arm[a].hand_orientation.toRotationMatrix().transpose())-.5*dt*(x.hand_angular_ff+previous.arm[a].hand_angular_ff)).norm());}
     if(k>250){sum_p+=std::pow((x.hand_position-origin[a]).x()-truth.first,2);sum_r+=std::pow(x.hand_orientation.angularDistance(Eigen::Quaterniond(Eigen::AngleAxisd(2.*truth.first,axis))*rotation[a]),2);sum_v+=std::pow(x.hand_linear_ff.x()-truth.second,2);sum_w+=(x.hand_angular_ff-2.*truth.second*axis).squaredNorm();++samples;}
    }
    previous=world;
   }
   assert(task_residual<1e-10);assert(residual<2e-5);assert(rotation_residual<4e-5);
   if(kind==0 || kind==2)assert(previous.arm[0].hand_linear_ff.norm()<1e-9 && previous.arm[0].hand_angular_ff.norm()<1e-9);
   if(kind==1 || kind==3){assert(std::abs(previous.arm[0].hand_linear_ff.x()-signal(kind,4.).second)<.002);assert((previous.arm[0].hand_angular_ff-2.*signal(kind,4.).second*axis).norm()<.004);}
   results.push_back({{"rate",rate},{"kind",kind},{"packets",packets},{"position_rms",std::sqrt(sum_p/samples)},{"rotation_rms",std::sqrt(sum_r/samples)},{"velocity_rms",std::sqrt(sum_v/samples)},{"omega_rms",std::sqrt(sum_w/samples)},{"integral_residual_max",residual},{"rotation_integral_residual_max",rotation_residual},{"qp_task_residual_max",task_residual}});
  }
  std::cout<<"PRODUCTION_REFERENCE_RESULTS "<<results.dump()<<'\n';
 }
};
int main(int argc,char**argv){ros::init(argc,argv,"operating_replay",ros::init_options::NoSigintHandler);try{Json input;std::ifstream(argv[1])>>input;PinocchioVelocityIk n;VelocityIkOfflineTest::run(n,input["segments"][0]["seed"]);}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
