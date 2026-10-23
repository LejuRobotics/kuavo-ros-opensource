#pragma once
#include "mujoco_cpp/task2_score_observation.h"

// Read-only Task 1/3 flags: lifted=1, above target=2, placed=4,
// source support=8. Target surfaces use current model geometry.
namespace task13_score {
inline bool supported(const mjModel* m, const mjData* d, int body, int surface) {
  for (int i = 0; i < d->ncon; ++i) {
    const auto& c = d->contact[i];
    bool first = m->geom_bodyid[c.geom1] == body;
    bool second = m->geom_bodyid[c.geom2] == body;
    if ((!first && !second) || (first ? c.geom2 : c.geom1) != surface) continue;
    mjtNum force[6];
    mj_contactForce(m, d, i, force);
    if (force[0] > 0 && task2_score::dot(c.frame, task2_score::axis(d, surface, 2))
        * (second ? 1 : -1) > .5) return true;
  }
  return false;
}
inline std::uint8_t observe(const mjModel* m, const mjData* d, int body,
                            int source, int target, bool bin) {
  if (body < 0 || source < 0 || target < 0) return 0;
  const auto up = task2_score::axis(d, source, 2);
  const mjtNum surface = task2_score::dot(d->geom_xpos+3*source, up)
                         + m->geom_size[3*source+2];
  mjtNum bottom = std::numeric_limits<mjtNum>::infinity();
  mjtNum low_z = bottom, high_z = -bottom;
  std::array<mjtNum, 2> low_xy{{bottom, bottom}}, high_xy{{-bottom, -bottom}};
  for (int geom = m->body_geomadr[body];
       geom < m->body_geomadr[body]+m->body_geomnum[body]; ++geom) {
    if (!(m->geom_contype[geom] || m->geom_conaffinity[geom])) continue;
    // Both tasks use convex collision meshes. Use their actual vertices,
    // including rotation, so tipping cannot turn body-origin height into lift.
    if (m->geom_type[geom] != mjGEOM_MESH) return 0;
    const int mesh = m->geom_dataid[geom];
    for (int v = m->mesh_vertadr[mesh]; v < m->mesh_vertadr[mesh]+m->mesh_vertnum[mesh]; ++v) {
      mjtNum point[3];
      for (int a = 0; a < 3; ++a) {
        point[a] = d->geom_xpos[3*geom+a];
        for (int b = 0; b < 3; ++b)
          point[a] += d->geom_xmat[9*geom+3*a+b] * m->mesh_vert[3*v+b];
      }
      bottom = std::min(bottom, task2_score::dot(point, up)-surface);
      for (int a = 0; a < 2; ++a) {
        low_xy[a] = std::min(low_xy[a], point[a]);
        high_xy[a] = std::max(high_xy[a], point[a]);
      }
      low_z = std::min(low_z, point[2]);
      high_z = std::max(high_z, point[2]);
    }
  }
  if (!std::isfinite(bottom)) return 0;
  const bool source_support = supported(m, d, body, source);
  const bool target_support = supported(m, d, body, target);
  mjtNum offset[3];
  for (int a = 0; a < 3; ++a)
    offset[a] = (a < 2 ? (low_xy[a]+high_xy[a])/2 : (low_z+high_z)/2)
                - d->geom_xpos[3*target+a];
  const auto tx = task2_score::axis(d, target, 0);
  const auto ty = task2_score::axis(d, target, 1);
  const mjtNum inset = bin ? .01 : 0;
  const bool inside =
    std::abs(task2_score::dot(offset, tx)) <= m->geom_size[3*target]-inset &&
    std::abs(task2_score::dot(offset, ty)) <= m->geom_size[3*target+1]-inset;
  const mjtNum target_top = d->geom_xpos[3*target+2] + m->geom_size[3*target+2];
  const bool above = inside && low_z >= target_top-.002;
  bool placed = inside && target_support;
  if (bin) {
    const int wall = mj_name2id(m, mjOBJ_GEOM, "target_bin_front_wall");
    const mjtNum rim = d->geom_xpos[3*wall+2] + m->geom_size[3*wall+2];
    // Entered the container volume, including a block stacked above another.
    // Its bottom must cross the rim; hovering entirely above does not count.
    placed = inside && low_z >= target_top-.005 && low_z < rim-.0005;
  }
  // 0.5 mm rejects solver separation noise; this is not a task lift height.
  return ((!source_support && bottom > .0005) ? 1 : 0)
      | (above ? 2 : 0) | (placed ? 4 : 0) | (source_support ? 8 : 0);
}
}  // namespace task13_score
