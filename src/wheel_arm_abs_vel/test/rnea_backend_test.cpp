#include "wheel_arm_abs_vel/humanoid_wheel_wbc/RneaBackend.h"
#include <humanoid_wheel_interface/FactoryFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <Eigen/Cholesky>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <chrono>
#include <algorithm>

using namespace ocs2;
using namespace ocs2::mobile_manipulator;
namespace {
int checks=0;
void require(bool condition,const char* name) {
  if(!condition) throw std::runtime_error(name);
  ++checks;
}
void near(const vector_t& a,const vector_t& b,const char* name) {
  require(a.size()==b.size() && (a-b).norm()<1e-8*(1+b.norm()),name);
}
template<class F> void rejects(F f,const char* name) {
  bool rejected=false;
  try {f();} catch(const std::invalid_argument&) {rejected=true;}
  require(rejected,name);
}
}

int main(int argc,char** argv) {
 try {
  if(argc!=2) throw std::invalid_argument("usage: rnea_backend_test REPOSITORY_ROOT");
  const std::string root=argv[1];
  const std::string task=root+"/src/humanoid-wheel-control/humanoid_wheel_interface/config/kuavo_s62/task.info";
  std::vector<std::string> removed;
  loadData::loadStdVector<std::string>(task,"model_information.removeJoints",removed,false);
  const auto type=ManipulatorModelType::WheelWorldBasedMobileManipulator;
  auto interface=createPinocchioInterface(root+"/src/kuavo_assets/models/biped_s62/urdf/biped_s62.urdf",type,removed);
  auto info=createManipulatorModelInfo(interface,type,"base_link","waist_yaw_link",{});
  const auto& model=interface.getModel(); auto& data=interface.getData();
  require(model.nq==21 && model.nv==21 && info.armDim==18,"actual s62 dimensions");
  RneaSettings s;
  s.torqueLimit=vector_t::Constant(18,1e6);
  s.torqueRateLimit=vector_t::Constant(18,1e12);
  s.accelerationLimit=vector_t::Constant(18,1e4);
  s.jerkLimit=vector_t::Constant(18,1e12);
  s.dynamicRampSeconds=0.001;
  SystemObservation m;
  m.state=vector_t::Zero(21);m.input=vector_t::Zero(21);
  m.state.segment(3,4)<<-0.2,0.4,-0.2,0.1;
  DynamicsReference r;
  r.q=m.state;r.v=m.input;r.acceleration=vector_t::Zero(18);

  s.mode="gravity";
  RneaBackend gravity(interface,info,s);
  auto result=gravity.update(r,m,0.002,0.001);
  require(result.valid,"gravity output valid");
  vector_t g=pinocchio::computeGeneralizedGravity(model,data,m.state);
  near(result.torque,g.tail(18),"gravity matches Pinocchio");
  // A deliberately remote target cannot inject an extra software PD term.
  r.q.tail(18).array()+=0.3;r.v.tail(18).array()+=0.2;
  near(gravity.update(r,m,0.002,0.001).torque,g.tail(18),"no hidden reference PD");
  auto moved=m;moved.state.head(3)<<2,-3,1.2;moved.input.head(3)<<1,2,3;
  near(gravity.update(r,moved,0.002,0.001).torque,g.tail(18),"base frozen explicitly");
  s.mode="bias";
  m.input.tail(18).setLinSpaced(18,-0.3,0.3);
  RneaBackend bias(interface,info,s);
  const vector_t expectedBias=pinocchio::rnea(model,data,m.state,m.input,vector_t::Zero(21));
  near(bias.update(r,m,0.002,0).torque,expectedBias.tail(18),"bias matches RNEA");
  s.mode="full";
  r.acceleration.setLinSpaced(18,-0.4,0.4);r.accelerationValid=true;
  vector_t a=vector_t::Zero(21);a.tail(18)=r.acceleration;
  RneaBackend full(interface,info,s);
  const vector_t expectedFull=pinocchio::rnea(model,data,m.state,m.input,a);
  near(full.update(r,m,0.002,0).torque,expectedFull.tail(18),"full matches RNEA");
  pinocchio::crba(model,data,m.state);
  matrix_t M=data.M.selfadjointView<Eigen::Upper>();
  near(expectedFull,M*a+expectedBias,"RNEA equals M*a+nle, full physical coupling");
  // Loss of ddq preserves bias instead of zeroing torque.
  r.accelerationValid=false;
  near(full.update(r,m,0.002,0).torque,expectedBias.tail(18),"invalid ddq falls back to bias");
  require(!full.update(r,m,0.002,0.2).valid,"stale state rejected");
  require(!full.update(r,m,0.002,-0.1).valid,"future state rejected");
  require(!full.update(r,m,0.2,0).valid,"long cycle rejected");
  require(!full.update(r,m,0.0,0).valid,"zero dt rejected");
  auto bad=m;bad.input[4]=std::numeric_limits<double>::quiet_NaN();
  require(!full.update(r,bad,0.002,0).valid,"NaN measurement rejected");
  auto badRef=r;badRef.q[4]=std::numeric_limits<double>::infinity();
  require(!full.update(badRef,m,0.002,0).valid,"nonfinite reference rejected");
  badRef=r;badRef.accelerationValid=true;badRef.acceleration.resize(1);
  require(!full.update(badRef,m,0.002,0).valid,"wrong ddq dimension rejected");
  auto badInfo=info;std::swap(badInfo.dofNames[0],badInfo.dofNames[1]);
  rejects([&]{RneaBackend x(interface,badInfo,s);},"joint permutation rejected");
  auto badSettings=s;badSettings.mode="typo";
  rejects([&]{RneaBackend x(interface,info,badSettings);},"invalid mode rejected");
  badSettings=s;badSettings.torqueLimit[0]=0;
  rejects([&]{RneaBackend x(interface,info,badSettings);},"invalid limits rejected");
  badSettings=s;badSettings.maxMeasurementAge=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{RneaBackend x(interface,info,badSettings);},"nonfinite setting rejected");

  // Test bounded transition and active limits independently of formula equality.
  s.torqueLimit.setConstant(2.0);s.torqueRateLimit.setConstant(10.0);
  s.jerkLimit.setConstant(100.0);s.dynamicRampSeconds=0.2;
  RneaBackend limited(interface,info,s);
  auto previous=limited.update(r,m,0.002,0);
  require(previous.valid && previous.torqueLimited,"torque saturation reported");
  for(int i=0;i<300;++i) {
    r.accelerationValid=i%2==0;r.acceleration.setConstant((i % 2) ?100:-100);
    auto next=limited.update(r,m,0.002,0);
    require(next.valid && next.torque.cwiseAbs().maxCoeff()<=2.0+1e-9,"absolute torque bound");
    require((next.torque-previous.torque).cwiseAbs().maxCoeff()<=0.020000001,"slew bound incl ddq validity chatter");
    previous=next;
  }
  require(previous.saturationSeconds>0.1,"persistent saturation observable");

  // Recover the applied acceleration from torque at a fixed, stationary pose.
  // This checks the final acceleration AFTER startup ramp, not just a filter's
  // internal state; ramp-after-slew would violate this contract.
  auto smoothSettings=s;
  smoothSettings.torqueLimit.setConstant(1e6);
  smoothSettings.torqueRateLimit.setConstant(1e12);
  smoothSettings.accelerationLimit.setConstant(2.0);
  smoothSettings.jerkLimit.setConstant(1.0);
  RneaBackend smooth(interface,info,smoothSettings);
  auto still=m;still.input.setZero();
  auto smoothReference=r;smoothReference.acceleration.setConstant(100.0);
  vector_t previousA=vector_t::Zero(18);
  const matrix_t jointM=M.bottomRightCorner(18,18);
  for(int i=0;i<300;++i) {
    smoothReference.accelerationValid=i<150;
    const auto next=smooth.update(smoothReference,still,0.002,0);
    require(next.valid,"ramped full output valid");
    const vector_t recovered=jointM.ldlt().solve(next.rawTorque-g.tail(18));
    require(recovered.cwiseAbs().maxCoeff()<=2.0+1e-8,"final acceleration bound");
    require((recovered-previousA).cwiseAbs().maxCoeff()<=0.00200001,"final jerk bound during ramp and loss of ddq");
    previousA=recovered;
  }

  // Model Data must be private to the backend.
  data.tau.setConstant(123.0);
  limited.update(r,m,0.002,0);
  require((data.tau.array()==123.0).all(),"Pinocchio Data not shared");
  std::vector<double> times;
  for(int i=0;i<12000;++i) {
    const auto start=std::chrono::steady_clock::now();
    require(limited.update(r,m,0.002,0).valid,"long-run finite output");
    times.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count());
  }
  std::sort(times.begin(),times.end());
  std::cout<<"PASS "<<checks<<" checks, actual s62 model, 12000 cycles; wall us median="
           <<times[6000]<<" p99="<<times[11880]<<" max="<<times.back()<<" (container, not robot RT guarantee)\n";
  return 0;
 } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
