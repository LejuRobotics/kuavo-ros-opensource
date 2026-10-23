"""Execute the actual C++ observer against MuJoCo collision/force data."""
import subprocess
from pathlib import Path


def test_geometry_and_contact_score_observation(tmp_path):
    root = Path(__file__).resolve().parents[3]
    mujoco = root / "src/mujoco"
    xml = tmp_path / "scene.xml"
    xml.write_text('''<mujoco><option timestep="0.002"/>
      <worldbody>
        <geom name="table" type="box" pos="0 0 .5" size="1 1 .1"/>
        <geom name="belt" type="box" pos="2.5 0 .4" size=".7 1 .1"/>
        <body name="box" pos="-.2 0 .65"><freejoint/>
          <geom type="box" pos=".2 0 0" size=".05 .05 .05" mass="1"/>
        </body>
      </worldbody></mujoco>''')
    raised = tmp_path / "raised.xml"
    raised.write_text(xml.read_text().replace('pos="2.5 0 .4"', 'pos="2.5 0 1.4"'))
    rotated = tmp_path / "rotated.xml"
    rotated.write_text(raised.read_text().replace('name="belt"',
        'name="belt" quat="0.7071067812 0 0 0.7071067812"'))
    cpp = tmp_path / "observe.cpp"
    cpp.write_text(r'''
#include "mujoco_cpp/task2_score_observation.h"
#include <cassert>
#include <vector>
int main(int argc, char** argv) {
  char error[1024];
  mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  assert(m);
  mjData* d = mj_makeData(m);
  int body = mj_name2id(m, mjOBJ_BODY, "box");
  int table = mj_name2id(m, mjOBJ_GEOM, "table");
  int belt = mj_name2id(m, mjOBJ_GEOM, "belt");
  auto place = [&](double x, double y, double z) {
    mj_resetData(m, d);
    d->qpos[0] = x - .2; d->qpos[1] = y; d->qpos[2] = z;
    mj_forward(m, d);
  };
  auto observe = [&]() {
    std::vector<mjtNum> before(d->qpos, d->qpos + m->nq);
    std::vector<mjtNum> velocity(d->qvel, d->qvel + m->nv);
    auto state = task2_score::observe(m, d, body, table, belt);
    assert(std::equal(before.begin(), before.end(), d->qpos));
    assert(std::equal(velocity.begin(), velocity.end(), d->qvel));
    return state;
  };
  place(0, 0, .65);
  for (int i=0; i<200; ++i) mj_step(m,d);
  assert((observe() & 8) && !(observe() & 1));
  place(0, 0, .6501);  // Only 0.1 mm off the table: no minimum lift gate.
  assert(observe() & 1);
  place(1.5, 0, .2);  // Fell off, below tabletop: not an upward lift.
  assert(!(observe() & 1));
  for (double z : {-3.0, .55, 10.0}) {
    place(2.5, 0, z);
    assert(observe() & 2);  // Arrival ignores height.
  }
  place(3.19, 0, 2);  // Body origin != box geometric centre.
  assert(observe() & 2);
  place(3.21, 0, 2);
  assert(!(observe() & 2));
  for (double x : {1.9, 2.5, 3.1}) {
    place(x, 0, .55);
    for (int i=0; i<200; ++i) mj_step(m,d);
    assert(observe() & 4);
  }
  place(1.76, 0, .4); // Contact with conveyor side, not top.
  assert(d->ncon > 0);
  assert(!(observe() & 4));
  place(2.5, 0, .26); // Underside contact cannot score placement.
  assert(!(observe() & 4));
  // Raise the entire belt: placement follows actual geometry, not absolute Z.
  mj_deleteData(d); mj_deleteModel(m);
  m = mj_loadXML(argv[2], nullptr, error, sizeof(error)); assert(m);
  d = mj_makeData(m);
  place(2.5, 0, 1.55);
  for (int i=0; i<200; ++i) mj_step(m,d);
  assert(observe() & 4);
  // A rotated belt uses its own axes, not a hardcoded world rectangle.
  place(2.5, .85, 3);
  assert(observe() & 2);
  mj_deleteData(d); mj_deleteModel(m);
  m = mj_loadXML(argv[3], nullptr, error, sizeof(error)); assert(m);
  d = mj_makeData(m);
  place(2.5, .85, 3);
  assert(!(observe() & 2));
  mj_deleteData(d); mj_deleteModel(m);
}
''')
    executable = tmp_path / "observe"
    lib = mujoco / "lib/x86_64"
    subprocess.run(["g++", "-std=c++14", str(cpp), "-I"+str(mujoco/"include"),
                    "-L"+str(lib), "-Wl,-rpath,"+str(lib), "-lmujoco",
                    "-o", str(executable)], check=True, capture_output=True)
    subprocess.run([str(executable), str(xml), str(raised), str(rotated)], check=True, capture_output=True)
