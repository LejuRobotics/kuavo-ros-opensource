"""Execute the production conveyor force function against real MuJoCo contacts."""
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[3]


def test_conveyor_moves_only_on_contact_and_cannot_pass_through_a_wall(tmp_path):
    source = (ROOT / "src/mujoco/src/mujoco_node.cc").read_text()
    function = source[source.index("void applyTask2ConveyorForce()\n{"):
                      source.index("\n}\n", source.index("void applyTask2ConveyorForce()\n{")) + 3]
    harness = r"""
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <cstring>
#define ROS_INFO(...) ((void)0)
mjModel *m;
mjData *d;
struct State {
  std::string name = "box";
  int body_id, qpos_addr = 0, dof_addr = 0;
  bool held = false;
  std::array<mjtNum,3> conveyor_force{{0,0,0}};
};
std::vector<State> bimanual_contact_followers;
int task2_conveyor_body_id, task2_conveyor_belt_geom_id;
bool task2_conveyor_started = false, task2_conveyor_completed = false;
std::atomic<bool> task2_conveyor_enabled{true};
struct Node {
  bool getParam(const char*,std::string &value) {value="box";return true;}
  void setParam(const char*,bool) {}
} node, *g_nh_ptr = &node;
mjtNum numericScalarOrDefault(const char *name,mjtNum fallback) {
  return std::string(name)=="task2_conveyor_stop_local_x" ? 0.65 : fallback;
}
Eigen::Matrix3d contactFollowerBodyRotation(int id) {
  return Eigen::Map<Eigen::Matrix<mjtNum,3,3,Eigen::RowMajor>>(d->xmat+9*id);
}
"""
    main = r"""
void load(bool wall) {
  std::string xml=R"(<mujoco><option timestep="0.002" integrator="implicitfast"/>
  <worldbody><body name="belt"><geom name="belt" type="box" size="1 .5 .05"
  friction="1.2 .01 .001" solref=".01 1" solimp=".95 .99 .001"/></body>
  <body name="box" pos="-.5 0 .11"><freejoint/><geom type="box" size=".32 .22 .06"
  mass="1" friction="1 .01 .001" solref=".01 1" solimp=".95 .99 .001"/></body>)";
  if(wall) xml+=R"(<geom name="wall" type="box" pos=".5 0 .3" size=".03 .5 .3"
  solref=".01 1" solimp=".95 .99 .001"/>)";
  xml+="</worldbody></mujoco>";
  mjVFS vfs; mj_defaultVFS(&vfs);
  mj_makeEmptyFileVFS(&vfs,"test.xml",xml.size());
  std::memcpy(vfs.filedata[0],xml.data(),xml.size());
  char error[1024];
  m=mj_loadXML("test.xml",&vfs,error,sizeof(error));
  if(!m){std::cerr<<error;std::abort();}
  mj_deleteVFS(&vfs);d=mj_makeData(m);
  task2_conveyor_body_id=mj_name2id(m,mjOBJ_BODY,"belt");
  task2_conveyor_belt_geom_id=mj_name2id(m,mjOBJ_GEOM,"belt");
  bimanual_contact_followers.clear();
  State state; state.body_id=mj_name2id(m,mjOBJ_BODY,"box");
  bimanual_contact_followers.push_back(state);
  task2_conveyor_enabled=true;
  task2_conveyor_started=task2_conveyor_completed=false;
  mj_forward(m,d);
}
void step(int count) {
  for(int i=0;i<count;++i) {
    std::vector<mjtNum> q(d->qpos,d->qpos+m->nq),v(d->qvel,d->qvel+m->nv);
    applyTask2ConveyorForce();
    assert(std::equal(q.begin(),q.end(),d->qpos));
    assert(std::equal(v.begin(),v.end(),d->qvel));
    mj_step(m,d);
  }
}
void close(){mj_deleteData(d);mj_deleteModel(m);}
int main(){
  std::cout<<std::unitbuf;
  load(false);
  step(10000);
  std::cout<<"free conveyor x="<<d->qpos[0]<<"\n";
  assert(task2_conveyor_completed && std::abs(d->qpos[0]-.65)<.02);
  close();

  load(true);
  step(10000);
  std::cout<<"blocked conveyor x="<<d->qpos[0]<<"\n";
  assert(!task2_conveyor_completed && d->qpos[0]<.153 && d->qpos[0]>.12);
  for(int i=0;i<d->ncon;++i) assert(d->contact[i].dist>-.002);

  // Disabling removes only conveyor force, retaining another force source.
  d->qfrc_applied[0]+=3;
  task2_conveyor_enabled=false;
  applyTask2ConveyorForce();
  assert(std::abs(d->qfrc_applied[0]-3)<1e-10);
  close();

  load(false);
  d->qpos[2]=1;
  mj_forward(m,d);
  applyTask2ConveyorForce();
  assert(d->qfrc_applied[0]==0 && !task2_conveyor_started);
  close();

  load(false);
  for(int i=0;i<100;++i) mj_step(m,d);
  bimanual_contact_followers[0].held=true;
  applyTask2ConveyorForce();
  assert(d->qfrc_applied[0]==0 && !task2_conveyor_started);
  close();
}
"""
    cpp = tmp_path / "conveyor.cpp"
    cpp.write_text(harness + function + main)
    module = ROOT / "src/mujoco"
    library = module / "lib/x86_64/libmujoco.so"
    exe = tmp_path / "conveyor"
    subprocess.run([
        "g++", "-std=c++14", "-O2", "-I/usr/include/eigen3",
        "-I" + str(module / "include"), str(cpp),
        str(library),
        "-Wl,-rpath," + str(library.parent), "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)
