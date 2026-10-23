"""Run the C++ Task1/3 geometry observer against real MuJoCo contacts."""
import subprocess
from pathlib import Path

def test_mesh_contact_observer(tmp_path):
    mujoco = Path(__file__).resolve().parents[3] / "src/mujoco"
    xml=tmp_path/"scene.xml"
    xml.write_text('''<mujoco><option timestep=".002"/>
    <asset><mesh name="cube" vertex="-.02 -.02 -.02 .02 -.02 -.02 -.02 .02 -.02 .02 .02 -.02 -.02 -.02 .02 .02 -.02 .02 -.02 .02 .02 .02 .02 .02"/></asset>
    <worldbody>
      <geom name="source" type="box" pos="0 0 .5" size=".5 .5 .1"/>
      <geom name="target" type="box" pos="2 0 .7" size=".2 .3 .1"/>
      <geom name="target_bin_front_wall" type="box" pos="1.805 0 .85" size=".005 .3 .05"/>
      <body name="object" pos="0 0 .62"><freejoint/><geom type="mesh" mesh="cube" mass=".05"/></body>
    </worldbody></mujoco>''')
    cpp=tmp_path/"observer.cpp"
    cpp.write_text(r'''
#include "mujoco_cpp/task13_score_observation.h"
#include <cassert>
#include <vector>
int main(int argc, char** argv) {
  char error[1024]; auto* m=mj_loadXML(argv[1],nullptr,error,1024); assert(m);
  auto* d=mj_makeData(m);
  int body=mj_name2id(m,mjOBJ_BODY,"object"), source=mj_name2id(m,mjOBJ_GEOM,"source"),
      target=mj_name2id(m,mjOBJ_GEOM,"target");
  auto place=[&](double x,double z) {
    mj_resetData(m,d); d->qpos[0]=x;d->qpos[2]=z;mj_forward(m,d);
  };
  auto observe=[&](bool bin=false) {
    std::vector<mjtNum> q(d->qpos,d->qpos+m->nq),v(d->qvel,d->qvel+m->nv);
    auto flags=task13_score::observe(m,d,body,source,target,bin);
    assert(std::equal(q.begin(),q.end(),d->qpos));
    assert(std::equal(v.begin(),v.end(),d->qvel));return flags;
  };
  place(0,.62);for(int i=0;i<200;i++)mj_step(m,d);
  assert(observe()&8);assert(!(observe()&1));
  place(0,.621);assert(observe()&1); // one millimetre is sufficient
  place(.8,.2);assert(!(observe()&1)); // fell off table
  place(2,3);assert(observe()&2);assert(!(observe()&4));assert(!(observe(true)&4));
  place(2,.84);assert(observe(true)&4); // entered bin, no settling wait
  place(2,.91);assert(observe(true)&4); // bottom inside, top above rim (stacked)
  place(2,.921);assert(!(observe(true)&4)); // wholly above rim
  place(2,.82);for(int i=0;i<200;i++)mj_step(m,d);
  assert(observe()&4);
  place(2,.5);assert(!(observe()&2));assert(!(observe()&4));
  // Move the table; scoring must follow it.
  m->geom_pos[3*target]=2.1;
  place(1.85,1);assert(!(observe()&2));
  place(2.1,1);assert(observe()&2);
  place(2.1,.82);for(int i=0;i<200;i++)mj_step(m,d);assert(observe()&4);
  mj_deleteData(d);mj_deleteModel(m);
}''')
    lib=mujoco/"lib/x86_64"
    executable=tmp_path/"observer"
    subprocess.run(["g++","-std=c++14",str(cpp),"-I"+str(mujoco/"include"),
        "-L"+str(lib),"-Wl,-rpath,"+str(lib),"-lmujoco","-o",str(executable)],
        check=True,capture_output=True)
    subprocess.run([str(executable),str(xml)],check=True,capture_output=True)
