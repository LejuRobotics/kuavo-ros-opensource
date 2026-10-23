#pragma once

#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

// Read-only Task 2 observations: lifted=1, over belt=2, belt support=4,
// table support=8. No joint, body, actuator or contact state is changed.
namespace task2_score {
inline std::array<mjtNum, 3> axis(const mjData* d, int geom, int col) {
  const mjtNum* rotation = d->geom_xmat + 9 * geom;
  return {{rotation[col], rotation[3 + col], rotation[6 + col]}};
}
inline mjtNum dot(const mjtNum* a, const std::array<mjtNum, 3>& b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
inline mjtNum radius(const mjModel* m, const mjData* d, int geom,
                    const std::array<mjtNum, 3>& direction) {
  mjtNum result = 0;
  for (int i = 0; i < 3; ++i) {
    const auto local_axis = axis(d, geom, i);
    result += m->geom_size[3*geom+i] * std::abs(dot(local_axis.data(), direction));
  }
  return result;
}
inline std::uint8_t observe(const mjModel* m, const mjData* d,
                            int body, int table, int belt) {
  if (body < 0 || table < 0 || belt < 0) return 0;
  const auto up = axis(d, table, 2);
  const mjtNum table_top = dot(d->geom_xpos + 3*table, up) + m->geom_size[3*table+2];
  mjtNum clearance = std::numeric_limits<mjtNum>::infinity();
  std::array<mjtNum, 3> lower{{clearance, clearance, clearance}};
  std::array<mjtNum, 3> upper{{-clearance, -clearance, -clearance}};
  bool found = false;
  for (int geom = m->body_geomadr[body];
       geom < m->body_geomadr[body] + m->body_geomnum[body]; ++geom) {
    if (!(m->geom_contype[geom] || m->geom_conaffinity[geom])) continue;
    // Task 2's collision shells are boxes. Unsupported geometry must not score.
    if (m->geom_type[geom] != mjGEOM_BOX) return 0;
    found = true;
    clearance = std::min(clearance,
        dot(d->geom_xpos + 3*geom, up) - radius(m, d, geom, up) - table_top);
    for (int i = 0; i < 3; ++i) {
      std::array<mjtNum, 3> direction{{0, 0, 0}};
      direction[i] = 1;
      const mjtNum extent = radius(m, d, geom, direction);
      lower[i] = std::min(lower[i], d->geom_xpos[3*geom+i] - extent);
      upper[i] = std::max(upper[i], d->geom_xpos[3*geom+i] + extent);
    }
  }
  if (!found) return 0;
  bool table_support = false, belt_support = false;
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact& contact = d->contact[i];
    const bool box_first = m->geom_bodyid[contact.geom1] == body;
    const bool box_second = m->geom_bodyid[contact.geom2] == body;
    if (!box_first && !box_second) continue;
    const int surface = box_first ? contact.geom2 : contact.geom1;
    if (surface != table && surface != belt) continue;
    mjtNum force[6];
    mj_contactForce(m, d, i, force);
    if (force[0] <= 0) continue;
    const auto surface_up = axis(d, surface, 2);
    // Contact normal points from geom1 to geom2; require support toward the
    // box from the surface's upper face, never a side/underside collision.
    const mjtNum normal_up = dot(contact.frame, surface_up) * (box_second ? 1 : -1);
    if (normal_up <= 0.5) continue;
    if (surface == table) table_support = true;
    if (surface == belt) belt_support = true;
  }
  std::array<mjtNum, 3> offset;
  for (int i = 0; i < 3; ++i)
    offset[i] = (lower[i] + upper[i]) / 2 - d->geom_xpos[3*belt+i];
  const bool over_belt =
      std::abs(dot(offset.data(), axis(d, belt, 0))) <= m->geom_size[3*belt] &&
      std::abs(dot(offset.data(), axis(d, belt, 1))) <= m->geom_size[3*belt+1];
  return ((!table_support && clearance > 0) ? 1 : 0) |
         (over_belt ? 2 : 0) | (belt_support ? 4 : 0) | (table_support ? 8 : 0);
}
}  // namespace task2_score
