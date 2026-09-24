#!/usr/bin/env python3
"""Static and short-dynamics audit for the Task 3 scene and hand path."""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.hand_postures import internal_expansion_pose


SCENE = SRC_DIR / (
    "data_challenge_simulator/models/biped_s400062/xml/"
    "task3.xml")
ACTIVE_FINGERS = ("index", "middle", "little")
EXPECTED_OUTER_RADIUS = 0.045
EXPECTED_RING_HEIGHT = 0.060
EXPECTED_COMPACT_JOINTS = {
    "r_index_j2": 1.55,
    "r_middle_j1": 1.65,
    "r_little_j1": 3.0,
    "r_little_j2": 1.8,
}
EXPECTED_EXPANDED_JOINTS = {
    "r_index_j2": 1.0,
    "r_middle_j1": 1.1,
    "r_little_j1": 3.0,
    "r_little_j2": 1.0,
}


def object_id(model, kind, name):
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise RuntimeError("Missing model object: {}".format(name))
    return result


def numeric_value(model, name):
    numeric_id = object_id(model, mujoco.mjtObj.mjOBJ_NUMERIC, name)
    return float(model.numeric_data[model.numeric_adr[numeric_id]])


def apply_pose(model, data, pose):
    for name, value in pose.items():
        joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        lower, upper = model.jnt_range[joint_id]
        if not lower <= value <= upper:
            raise RuntimeError("{}={} violates [{}, {}]".format(
                name, value, lower, upper))
        data.qpos[model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(model, data)


def fingertip_xy(model, data, expansion):
    apply_pose(model, data, internal_expansion_pose(expansion, "r"))
    hand_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "r_hand_base")
    hand_rotation = data.xmat[hand_id].reshape(3, 3)
    hand_origin = data.xpos[hand_id]
    points = []
    for finger in ACTIVE_FINGERS:
        geom_id = object_id(
            model, mujoco.mjtObj.mjOBJ_GEOM,
            "r_{}_fingertip_collision".format(finger))
        local = hand_rotation.T @ (data.geom_xpos[geom_id] - hand_origin)
        points.append(local[:2])
    return np.asarray(points)


def support_radius(points):
    """Return the radius of the smallest circle containing three pad centers."""
    candidates = []
    for first in range(3):
        for second in range(first + 1, 3):
            center = 0.5 * (points[first] + points[second])
            radius = 0.5 * np.linalg.norm(points[first] - points[second])
            if np.all(np.linalg.norm(points - center, axis=1) <= radius + 1e-8):
                candidates.append(float(radius))
    twice_offsets = 2.0 * (points[1:] - points[0])
    if abs(np.linalg.det(twice_offsets)) > 1e-10:
        rhs = np.sum(points[1:] ** 2, axis=1) - np.sum(points[0] ** 2)
        center = np.linalg.solve(twice_offsets, rhs)
        candidates.append(float(np.max(np.linalg.norm(points - center, axis=1))))
    if not candidates:
        raise RuntimeError("Cannot construct three-finger support circle")
    return min(candidates)


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    ring_body = object_id(
        model, mujoco.mjtObj.mjOBJ_BODY, "task3_hollow_cylinder")
    ring_joint = object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT,
        "task3_hollow_cylinder_freejoint")
    if model.jnt_type[ring_joint] != mujoco.mjtJoint.mjJNT_FREE:
        raise RuntimeError("Task 3 cylinder is not a free body")
    segment_count = sum(
        model.geom_bodyid[geom_id] == ring_body and
        (mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "").startswith(
                "task3_ring_segment_")
        for geom_id in range(model.ngeom))
    if segment_count != 24:
        raise RuntimeError("Expected 24 convex ring segments, got {}".format(
            segment_count))

    inner_radius = numeric_value(model, "task3_inner_radius")
    ring_height = numeric_value(model, "task3_ring_height")
    if not math.isclose(ring_height, EXPECTED_RING_HEIGHT, abs_tol=1e-9):
        raise RuntimeError("Unexpected ring height: {}".format(ring_height))
    ray_geom = np.asarray([-1], dtype=np.int32)
    ray_normal = np.zeros(3, dtype=np.float64)
    collision_group = np.asarray([0, 0, 0, 1, 0, 0], dtype=np.uint8)
    bore_midpoint = data.xpos[ring_body] + np.asarray(
        [0.0, 0.0, 0.5 * ring_height])
    bore_hit = mujoco.mj_ray(
        model, data, bore_midpoint, np.asarray([1.0, 0.0, 0.0]),
        collision_group, True, -1, ray_geom, ray_normal)
    if not math.isclose(bore_hit, inner_radius, abs_tol=0.001):
        raise RuntimeError(
            "Bore ray did not hit the inner wall at its declared radius: {}".format(
                bore_hit))
    outer_ray_offset = 0.100
    outer_hit = mujoco.mj_ray(
        model, data,
        data.xpos[ring_body] + np.asarray(
            [outer_ray_offset, 0.0, 0.5 * ring_height]),
        np.asarray([-1.0, 0.0, 0.0]), collision_group, True, -1,
        ray_geom, ray_normal)
    measured_outer_radius = outer_ray_offset - outer_hit
    if not math.isclose(
            measured_outer_radius, EXPECTED_OUTER_RADIUS, abs_tol=0.001):
        raise RuntimeError(
            "Unexpected outer radius: {}".format(measured_outer_radius))
    compact_pose = internal_expansion_pose(0.0, "r")
    expanded_pose = internal_expansion_pose(1.0, "r")
    for joint_name, expected in EXPECTED_COMPACT_JOINTS.items():
        if not math.isclose(compact_pose[joint_name], expected, abs_tol=1e-9):
            raise RuntimeError("Unexpected compact target for {}".format(joint_name))
    for joint_name, expected in EXPECTED_EXPANDED_JOINTS.items():
        if not math.isclose(expanded_pose[joint_name], expected, abs_tol=1e-9):
            raise RuntimeError("Unexpected expanded target for {}".format(joint_name))

    compact_points = fingertip_xy(model, data, 0.0)
    expanded_points = fingertip_xy(model, data, 1.0)
    compact_radius = support_radius(compact_points)
    expanded_radius = support_radius(expanded_points)
    if not compact_radius < inner_radius < expanded_radius:
        raise RuntimeError(
            "Invalid support radii: compact={}, expanded={}, bore={}".format(
                compact_radius, expanded_radius, inner_radius))
    if expanded_radius - compact_radius < 0.003:
        raise RuntimeError("Three-finger stroke is too small")
    compact_distances = sorted(
        np.linalg.norm(compact_points[first] - compact_points[second])
        for first, second in ((0, 1), (1, 2), (0, 2)))
    expanded_distances = sorted(
        np.linalg.norm(expanded_points[first] - expanded_points[second])
        for first, second in ((0, 1), (1, 2), (0, 2)))
    if any(after <= before for before, after in zip(
            compact_distances, expanded_distances)):
        raise RuntimeError("Not all three fingertip pairs move outward")

    source_top = object_id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "task3_source_table_top")
    target_top = object_id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "task3_destination_table_top")
    source_height = data.geom_xpos[source_top, 2] + model.geom_size[source_top, 2]
    target_height = data.geom_xpos[target_top, 2] + model.geom_size[target_top, 2]
    source_center = data.geom_xpos[source_top]
    target_center = data.geom_xpos[target_top]
    ring_center = data.xpos[ring_body]
    if not (
            np.allclose(source_center[:2], (0.62, 0.0), atol=1e-6) and
            np.allclose(target_center[:2], (0.62, 0.85), atol=1e-6) and
            np.allclose(ring_center[:2], (0.62, 0.0), atol=1e-6)):
        raise RuntimeError("Unexpected Task 3 table or ring XY placement")
    if not (
            math.isclose(model.geom_size[source_top, 0], 0.20, abs_tol=1e-6) and
            math.isclose(model.geom_size[target_top, 0], 0.20, abs_tol=1e-6)):
        raise RuntimeError("Task 3 table length is not 0.40 m")
    if not math.isclose(source_height, 0.90, abs_tol=1e-6):
        raise RuntimeError("Unexpected source tabletop height")
    if not math.isclose(target_height, 1.10, abs_tol=1e-6):
        raise RuntimeError("Unexpected destination tabletop height")

    # Let the free annulus settle on the source table and reject explosions or
    # fall-through caused by a malformed convex decomposition.
    for _ in range(1000):
        mujoco.mj_step(model, data)
    ring_z = float(data.xpos[ring_body, 2])
    if not source_height - 0.005 <= ring_z <= source_height + 0.010:
        raise RuntimeError("Ring did not remain on the source table: z={}".format(
            ring_z))

    print("scene={}".format(SCENE))
    print("ring_segments={}".format(segment_count))
    print("ring_mass_kg={:.4f}".format(model.body_mass[ring_body]))
    print("inner_radius_mm={:.1f}".format(inner_radius * 1000.0))
    print("measured_bore_radius_mm={:.1f}".format(bore_hit * 1000.0))
    print("measured_outer_radius_mm={:.1f}".format(
        measured_outer_radius * 1000.0))
    print("ring_height_mm={:.1f}".format(ring_height * 1000.0))
    print("compact_support_radius_mm={:.1f}".format(compact_radius * 1000.0))
    print("expanded_support_radius_mm={:.1f}".format(expanded_radius * 1000.0))
    print("source_table_height_m={:.3f}".format(source_height))
    print("destination_table_height_m={:.3f}".format(target_height))
    print("table_world_x_range_m=[{:.3f}, {:.3f}]".format(
        source_center[0] - model.geom_size[source_top, 0],
        source_center[0] + model.geom_size[source_top, 0]))
    print("ring_world_x_m={:.3f}".format(ring_center[0]))
    print("settled_ring_bottom_z_m={:.4f}".format(ring_z))


if __name__ == "__main__":
    main()
