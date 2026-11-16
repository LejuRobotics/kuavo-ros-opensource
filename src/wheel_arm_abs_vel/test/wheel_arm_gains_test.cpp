#include <wheel_arm_abs_vel/common/wheel_arm_gains.h>
#include <cassert>
#include <vector>
#include <iostream>
struct Settings {
  std::vector<int> joint_kp, joint_kd, vr_joint_kp, vr_joint_kd;
  std::vector<double> ruiwo_kp, ruiwo_kd, vr_ruiwo_kp, vr_ruiwo_kd;
};
template<class F> void rejects(F f) {
  bool rejected=false; try { f(); } catch (const std::runtime_error&) { rejected=true; }
  assert(rejected);
}
int main() {
  for (bool mixed : {false, true}) for (bool empty_vr : {false, true}) {
    std::vector<int> drivers(20, 1); // EC=0, RUIWO=1
    for (int i=0; i<4; ++i) drivers[i]=0;
    if (mixed) drivers[4]=drivers[11]=0; // 60/61: J1 is EtherCAT.
    const int ec=mixed?6:4, rw=20-ec;
    Settings s{{}, {}, {}, {}, {}, {}, {}, {}};
    s.joint_kp.assign(ec, 777); s.joint_kd.assign(ec, 888);
    s.ruiwo_kp.assign(rw, 99); s.ruiwo_kd.assign(rw, 9);
    if (!empty_vr) {
      s.vr_joint_kp.assign(ec, 666); s.vr_joint_kd.assign(ec, 555);
      s.vr_ruiwo_kp.assign(rw, 88); s.vr_ruiwo_kd.assign(rw, 8);
    }
    HighlyDynamic::applyWheelArmGains(s,drivers,4,14,0,1);
    const double kp[]={250,210,250,180,120,120,120};
    const double kd[]={20,10,18,10,6,6,6};
    for (bool vr : {false,true}) {
      int ei=0,ri=0;
      for (int i=0;i<20;++i) {
        const double p=drivers[i]==0?(vr?s.vr_joint_kp[ei]:s.joint_kp[ei]):
                                       (vr?s.vr_ruiwo_kp[ri]:s.ruiwo_kp[ri]);
        const double d=drivers[i]==0?(vr?s.vr_joint_kd[ei]:s.joint_kd[ei]):
                                       (vr?s.vr_ruiwo_kd[ri]:s.ruiwo_kd[ri]);
        if(i>=4 && i<18) {assert(p==kp[(i-4)%7]);assert(d==kd[(i-4)%7]);}
        else if(i<4) {assert(p==((vr&&!empty_vr)?666:777));assert(d==((vr&&!empty_vr)?555:888));}
        else {assert(p==((vr&&!empty_vr)?88:99));assert(d==((vr&&!empty_vr)?8:9));}
        if(drivers[i]==0) ++ei; else ++ri;
      }
    }
    auto invalid=s; invalid.vr_ruiwo_kd.pop_back(); const auto before=invalid;
    rejects([&]{HighlyDynamic::applyWheelArmGains(invalid,drivers,4,14,0,1);});
    assert(invalid.joint_kp==before.joint_kp && invalid.ruiwo_kd==before.ruiwo_kd);
    rejects([&]{HighlyDynamic::applyWheelArmGains(s,drivers,4,12,0,1);});
    rejects([&]{HighlyDynamic::applyWheelArmGains(s,drivers,-1,14,0,1);});
    auto bad_driver=drivers; bad_driver[4]=2;
    rejects([&]{HighlyDynamic::applyWheelArmGains(s,bad_driver,4,14,0,1);});
  }
  std::cout << "PASS: full arm gains, mixed drivers, normal/VR selection, head/torso preservation, invalid layouts\n";
}
