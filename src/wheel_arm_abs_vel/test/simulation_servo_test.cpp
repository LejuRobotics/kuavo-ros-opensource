#include "wheel_arm_abs_vel/SimulationServo.h"
#include <cassert>
#include <fstream>
#include <unistd.h>
int main() {
  char path[]="/tmp/abs-servo-XXXXXX";
  int fd=mkstemp(path); assert(fd>=0); close(fd);
  std::ofstream(path)<<R"(<mujoco><option timestep="0.002" gravity="0 0 0"/>
    <worldbody><body><joint name="j" type="hinge" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.01" mass="0.1"/>
    </body></worldbody><actuator><motor joint="j" gear="1" ctrllimited="true" ctrlrange="-2 2"/></actuator></mujoco>)";
  char error[1024];mjModel* m=mj_loadXML(path,nullptr,error,sizeof(error));unlink(path);assert(m);
  mjData* d=mj_makeData(m);
  AbsoluteSimulationServo servo;
  servo.configureModel(m,d,{0},{120},{6});
  kuavo_msgs::jointCmd c;c.joint_q={0.2};c.joint_v={0};c.control_modes={2};
  for(int i=0;i<10000;++i) {
    std::vector<double> u={0};assert(servo.encode(c,u));d->ctrl[0]=u[0];mj_step(m,d);
    assert(std::isfinite(d->qpos[0]));assert(std::abs(d->actuator_force[0])<=2.000001);
  }
  assert(std::abs(d->qpos[0]-0.2)<1e-5);assert(std::abs(d->qvel[0])<1e-5);
  c.joint_q[0]=-0.2;
  for(int i=0;i<10000;++i){std::vector<double> u={0};assert(servo.encode(c,u));d->ctrl[0]=u[0];mj_step(m,d);}
  assert(std::abs(d->qpos[0]+0.2)<1e-5);
  c.joint_q[0]=NAN;std::vector<double> u={0};assert(!servo.encode(c,u));
  c.joint_q[0]=0;c.control_modes[0]=1;assert(!servo.encode(c,u));
  mj_deleteData(d);mj_deleteModel(m);
  return 0;
}
