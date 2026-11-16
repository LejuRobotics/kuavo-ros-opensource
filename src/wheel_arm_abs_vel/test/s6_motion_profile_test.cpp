#include <wheel_arm_abs_vel/motion_capture_ik/S6MotionProfile.h>
#include <cassert>
#include <limits>
#include <map>
#include <vector>
#include <iostream>
using velocity_ik::S6MotionProfile;
struct Params {
 std::map<std::string,double> values;
 bool getParam(const std::string& key,double& value) const {
   const auto it=values.find(key); if(it==values.end())return false;
   value=it->second;return true;
 }
};
template<class F> void rejects(F f) { bool threw=false;try {f();}catch(const std::runtime_error&){threw=true;}assert(threw); }
int main() {
 Params params{{{"kp",250.},{"j3_kd",18.}}};
 S6MotionProfile defaults;int k=0;
 for(int j:{3,5,6,7}) {
  const auto prefix="j"+std::to_string(j)+"/";
  params.values[prefix+"velocity"]=defaults.limits[k].velocity;
  params.values[prefix+"acceleration"]=defaults.limits[k].acceleration;
  params.values[prefix+"jerk"]=defaults.limits[k++].jerk;
 }
 const auto index=[](const std::string& name){return 7+(name[5]=='r'?7:0)+(name[6]-'1');};
 for(int version:{60,61,62,63,200062,300062,400062,400063}) {
  const auto p=velocity_ik::readS6MotionProfile(params,version);
  std::vector<double> v(21,8.),a(21,100.),b=a,o=a,jerk(21,50000.);
  p.applyDynamics(v,a,b,o,jerk,index);
  assert(v[9]==2.2&&v[16]==2.2&&a[9]==18.&&jerk[9]==2000.);
  for(int start:{7,14}) {
   assert(v[start+4]==4.5&&a[start+4]==45.&&jerk[start+4]==4000.);
   for(int j:{5,6})assert(v[start+j]==5.5&&a[start+j]==55.&&jerk[start+j]==6000.);
  }
  assert(a==b&&a==o);assert(a[0]==100.&&a[7]==100.&&v[7]==8.);
  v[9]=1.;a[9]=5.;b[9]=4.;o[9]=3.;jerk[9]=500.;
  p.applyDynamics(v,a,b,o,jerk,index);
  assert(v[9]==1.&&a[9]==3.&&b[9]==3.&&o[9]==3.&&jerk[9]==500.);
  const auto before=a;
  rejects([&]{p.applyDynamics(v,a,b,o,jerk,[](const std::string&){return 0;});});assert(a==before);
 }
 for(int version:{0,-1})rejects([&]{velocity_ik::readS6MotionProfile(params,version);});
 auto missing=params;missing.values.erase("j6/jerk");rejects([&]{velocity_ik::readS6MotionProfile(missing,63);});
 for(double bad:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
  auto malformed=params;malformed.values["j3/acceleration"]=bad;rejects([&]{velocity_ik::readS6MotionProfile(malformed,400063);});
 }
 auto gains=params;gains.values["kp"]=300.;rejects([&]{velocity_ik::readS6MotionProfile(gains,62);});
 std::cout<<"PASS: eight wheel models, canonical names, shared braking bounds, conservative overrides, malformed profiles\n";
}
