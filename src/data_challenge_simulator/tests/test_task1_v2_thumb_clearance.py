"""Offline contact sweep for the Task1 V2 pre-descent hand sequence."""

import math
from pathlib import Path
import sys

import mujoco
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

from utils.hand_postures import single_gripper_pose
from utils.scene1_v2_right_arm_ik import Scene1V2RightArmIK

BASE_WORLD = (-0.20, 0.20, 0.0)
PIVOT = (0.62, -0.196, 0.62)
HANDLE_LOCAL = (-0.32, 0.0, 0.08)
TANGENT_OFFSET = 0.026
TRACKING_BIAS = (0.0, 0.0, 0.010)
APPROACH_HEIGHT = 0.08
READY = (-0.9, -0.265, 1.0, -0.8, 0.58, -0.4, 0.35)


def lever_target(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    local_x, _, local_z = HANDLE_LOCAL
    handle_x = PIVOT[0] + cosine * local_x + sine * local_z
    handle_z = PIVOT[2] - sine * local_x + cosine * local_z
    tangent_x = -sine * local_x + cosine * local_z
    tangent_z = -cosine * local_x - sine * local_z
    norm = math.hypot(tangent_x, tangent_z)
    return np.array((
        handle_x - TANGENT_OFFSET * tangent_x / norm,
        PIVOT[1],
        handle_z - TANGENT_OFFSET * tangent_z / norm,
    ))


def command_target(angle):
    return lever_target(angle) - np.asarray(TRACKING_BIAS)


def set_pose(ik, arm, hand):
    ik.data.qpos[:] = ik.model.qpos0
    ik.data.qpos[ik.base_qpos_address:ik.base_qpos_address + 3] += BASE_WORLD
    ik.data.qpos[ik.qpos_addresses] = arm
    ik.data.qpos[ik.hand_qpos_addresses] = hand
    mujoco.mj_forward(ik.model, ik.data)


def thumb_handle_contacts(ik):
    handle = mujoco.mj_name2id(
        ik.model, mujoco.mjtObj.mjOBJ_GEOM, "lever_handle_collision")
    thumb = mujoco.mj_name2id(
        ik.model, mujoco.mjtObj.mjOBJ_GEOM, "r_thumb_fingertip_collision")
    assert handle >= 0 and thumb >= 0
    return sum(
        {int(ik.data.contact[i].geom1), int(ik.data.contact[i].geom2)}
        == {handle, thumb}
        for i in range(ik.data.ncon))


def thumb_hand_contacts(ik):
    thumb_body = mujoco.mj_name2id(
        ik.model, mujoco.mjtObj.mjOBJ_BODY, "r_thumb_link3")
    assert thumb_body >= 0
    thumb_geoms = {
        geom for geom, body in enumerate(ik.model.geom_bodyid)
        if int(body) == thumb_body
    }
    right_hand_geoms = {
        geom for geom, body in enumerate(ik.model.geom_bodyid)
        if ik.model.body(body).name.startswith("r_")
    }
    return [
        (int(ik.data.contact[i].geom1), int(ik.data.contact[i].geom2))
        for i in range(ik.data.ncon)
        if ((int(ik.data.contact[i].geom1) in thumb_geoms
             and int(ik.data.contact[i].geom2) in right_hand_geoms)
            or (int(ik.data.contact[i].geom2) in thumb_geoms
                and int(ik.data.contact[i].geom1) in right_hand_geoms))
    ]


def test_thumb_never_contacts_handle_during_nominal_sequence():
    ik = Scene1V2RightArmIK()
    ik.set_base_translation_world(BASE_WORLD)
    contact = command_target(0.0)
    approach = contact.copy()
    approach[2] += APPROACH_HEIGHT
    contact_q = np.asarray(ik.solve_lever(
        contact, READY, joint_limit_margin_fraction=0.10))
    rotation0 = ik.eef_rotation_world(contact_q)
    approach_q = np.asarray(ik.solve_lever(
        approach, READY, joint_limit_margin_fraction=0.10))
    contact_q = np.asarray(ik.solve_lever(
        contact, approach_q, target_rotation_world=rotation0,
        joint_limit_margin_fraction=0.10))

    names = (
        "r_thumb_j1", "r_thumb_j2", "r_thumb_j3",
        "r_index_j1", "r_index_j2", "r_index_j3",
        "r_middle_j1", "r_middle_j2",
        "r_little_j1", "r_little_j2", "r_little_j3",
    )
    opened = single_gripper_pose(0.0, "r")
    initial = np.array([opened[name] for name in names])
    initialized = initial.copy()
    initialized[:2] = 0.0
    initialized[5] = 0.0
    assert initial[2] == 1.5
    assert initialized[2] == initial[2]
    hook = initialized.copy()
    hook[3:6] = (0.0, 0.0, 1.5)
    hook[6:8] = (0.0, 1.5)
    hook[8:11] = (0.0, 0.0, 1.3)

    samples = []
    for phase in np.linspace(0.0, 1.0, 61):
        samples.append(("hand_init", approach_q,
                        initial + phase * (initialized - initial)))
    for phase in np.linspace(0.0, 1.0, 81):
        samples.append(("descent", approach_q + phase * (contact_q - approach_q),
                        initialized))
    for phase in np.linspace(0.0, 1.0, 61):
        samples.append(("hook", contact_q,
                        initialized + phase * (hook - initialized)))

    seed = contact_q
    for angle_deg in np.linspace(0.0, 30.0, 13):
        angle = math.radians(float(angle_deg))
        seed = ik.solve_lever(
            command_target(angle), seed,
            target_rotation_world=ik.rotate_about_world_y(rotation0, angle),
            joint_limit_margin_fraction=0.02)
        samples.append(("path_{:.1f}".format(angle_deg), seed, hook))

    collisions = []
    self_collisions = []
    for label, arm, hand in samples:
        set_pose(ik, arm, hand)
        if thumb_handle_contacts(ik):
            collisions.append(label)
        if thumb_hand_contacts(ik):
            self_collisions.append(label)
    assert not collisions, "thumb contacted lever_handle_collision at {}".format(collisions)
    assert not self_collisions, "thumb contacted another hand part at {}".format(
        self_collisions)
