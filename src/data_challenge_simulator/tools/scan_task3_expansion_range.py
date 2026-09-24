#!/usr/bin/env python3
"""Scan the Task 3 partial-insertion internal-grasp expansion range.

This is a hand/ring geometry gate, not an arm IK test.  It places the real
24-segment collision ring at a measured candidate pose relative to
``r_hand_base`` and scans the canonical ``internal_expansion_pose`` path.

A sample is feasible when:

* the configured number of distinct active fingers are within the geometric
  inner-wall detection gap;
* thumb and other robot geoms do not contact the ring;
* counterfactual MuJoCo contact penetration stays below the selected limit.

Active-pad end-rim contacts are reported separately.  They match the current
runtime follower semantics and are allowed by default; use
``--reject-rim-contacts`` for the stricter physical-clearance interpretation.
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.hand_postures import internal_expansion_pose


DEFAULT_SCENE = (
    SRC_DIR / "data_challenge_simulator/models/biped_s400062/xml/"
    "task3.xml"
)
ACTIVE_FINGERS = ("index", "middle", "little")

# Local hand/ring candidate found by the partial-insertion geometry search.
# The ring axis and origin are expressed in the r_hand_base frame.  Keeping
# this transform explicit makes the test independent of the current arm pose.
CANDIDATE_RING_AXIS_HAND = np.array(
    (0.95444174, -0.27950385, 0.10449199), dtype=float)
CANDIDATE_RING_LINE_HAND = np.array(
    (-0.04633579, -0.12641128, 0.08510096), dtype=float)
CANDIDATE_RING_BOTTOM_AXIS_M = 0.02149011


@dataclass
class ExpansionSample:
    expansion: float
    feasible: bool
    nearby_fingers: list
    valid_fingers: list
    surface_gaps_mm: dict
    nearby_heights_mm: dict
    maximum_penetration_mm: float
    robot_ring_contacts: int
    active_rim_contacts: int
    thumb_contacts: int
    forbidden_robot_contacts: list
    contact_heights_mm: dict


def required_id(model, object_type, name):
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise RuntimeError("Missing model object: {}".format(name))
    return object_id


def numeric_value(model, name):
    numeric_id = required_id(model, mujoco.mjtObj.mjOBJ_NUMERIC, name)
    return float(model.numeric_data[model.numeric_adr[numeric_id]])


def descendant_geom_ids(model, root_body_id):
    body_ids = {root_body_id}
    changed = True
    while changed:
        changed = False
        for body_id in range(1, model.nbody):
            if (body_id not in body_ids and
                    model.body_parentid[body_id] in body_ids):
                body_ids.add(body_id)
                changed = True
    return {
        geom_id for geom_id in range(model.ngeom)
        if model.geom_bodyid[geom_id] in body_ids
    }


def expansion_values(minimum, maximum, step):
    count = int(math.floor((maximum - minimum) / step + 1e-9))
    values = [minimum + index * step for index in range(count + 1)]
    if not values or values[-1] < maximum - 1e-9:
        values.append(maximum)
    return values


def contiguous_ranges(samples, step):
    feasible = [sample.expansion for sample in samples if sample.feasible]
    if not feasible:
        return []
    ranges = []
    start = previous = feasible[0]
    for value in feasible[1:]:
        if value - previous > 1.5 * step:
            ranges.append((start, previous))
            start = value
        previous = value
    ranges.append((start, previous))
    return ranges


class Task3ExpansionScanner:
    def __init__(self, args):
        self.args = args
        self.scene_path = Path(args.scene).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)

        self.hand_body_id = required_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "r_hand_base")
        base_body_id = required_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.robot_geom_ids = descendant_geom_ids(self.model, base_body_id)
        self.pad_geom_to_finger = {
            required_id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM,
                "r_{}_fingertip_collision".format(finger)): finger
            for finger in ("thumb",) + ACTIVE_FINGERS
        }
        self.active_pad_ids = {
            geom_id for geom_id, finger in self.pad_geom_to_finger.items()
            if finger in ACTIVE_FINGERS
        }
        ring_body_id = required_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY,
            "task3_hollow_cylinder")
        self.ring_body_id = ring_body_id
        self.ring_geom_ids = {
            geom_id for geom_id in range(self.model.ngeom)
            if (self.model.geom_bodyid[geom_id] == ring_body_id and
                (mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM,
                    geom_id) or "").startswith("task3_ring_segment_"))
        }
        ring_joint_id = required_id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT,
            "task3_hollow_cylinder_freejoint")
        self.ring_qpos_address = self.model.jnt_qposadr[ring_joint_id]
        self.inner_radius = numeric_value(
            self.model, "task3_inner_radius")
        self.contact_tolerance = numeric_value(
            self.model, "task3_inner_contact_tolerance")
        self.ring_height = numeric_value(
            self.model, "task3_ring_height")
        if self.args.detection_gap_mm is None:
            self.args.detection_gap_mm = 1000.0 * numeric_value(
                self.model, "task3_grasp_detection_gap")
        if self.args.required_inner_fingers is None:
            self.args.required_inner_fingers = int(round(numeric_value(
                self.model, "task3_required_inner_fingers")))
        self.release_lost_fingers = int(round(numeric_value(
            self.model, "task3_release_lost_fingers")))
        self.hand_joint_addresses = {
            joint_name: self.model.jnt_qposadr[required_id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)]
            for joint_name in internal_expansion_pose(0.0, "r")
        }
        self.set_candidate_ring_pose()

    def set_candidate_ring_pose(self):
        axis = CANDIDATE_RING_AXIS_HAND.copy()
        axis /= np.linalg.norm(axis)
        reference = np.array((0.0, 0.0, 1.0))
        ring_x = np.cross(axis, reference)
        ring_x /= np.linalg.norm(ring_x)
        ring_y = np.cross(axis, ring_x)
        yaw = math.radians(self.args.ring_yaw_deg)
        yaw_x = math.cos(yaw) * ring_x + math.sin(yaw) * ring_y
        yaw_y = -math.sin(yaw) * ring_x + math.cos(yaw) * ring_y

        hand_rotation = self.data.xmat[self.hand_body_id].reshape(3, 3)
        hand_position = self.data.xpos[self.hand_body_id]
        ring_rotation = hand_rotation @ np.column_stack(
            (yaw_x, yaw_y, axis))
        ring_quaternion = np.empty(4)
        mujoco.mju_mat2Quat(ring_quaternion, ring_rotation.ravel())
        ring_bottom_hand = (
            CANDIDATE_RING_LINE_HAND +
            CANDIDATE_RING_BOTTOM_AXIS_M * axis)
        ring_position = hand_position + hand_rotation @ ring_bottom_hand

        address = self.ring_qpos_address
        self.data.qpos[address:address + 3] = ring_position
        self.data.qpos[address + 3:address + 7] = ring_quaternion
        self.ring_position = ring_position
        self.ring_quaternion = ring_quaternion

    def set_expansion(self, expansion):
        for joint_name, value in internal_expansion_pose(
                expansion, "r").items():
            self.data.qpos[self.hand_joint_addresses[joint_name]] = value
        address = self.ring_qpos_address
        self.data.qpos[address:address + 3] = self.ring_position
        self.data.qpos[address + 3:address + 7] = self.ring_quaternion
        mujoco.mj_forward(self.model, self.data)

    def scan_sample(self, expansion):
        self.set_expansion(expansion)
        ring_rotation = self.data.xmat[self.ring_body_id].reshape(3, 3)
        ring_position = self.data.xpos[self.ring_body_id]
        nearby_heights = {finger: [] for finger in ACTIVE_FINGERS}
        surface_gaps = {}
        detection_gap = self.args.detection_gap_mm / 1000.0
        ring_axis_world = ring_rotation[:, 2]
        for pad_geom_id in self.active_pad_ids:
            finger = self.pad_geom_to_finger[pad_geom_id]
            center_world = self.data.geom_xpos[pad_geom_id]
            center_local = ring_rotation.T @ (center_world - ring_position)
            center_radius = math.hypot(center_local[0], center_local[1])
            if (center_radius < 1e-9 or
                    center_radius >
                    self.inner_radius + self.contact_tolerance):
                continue
            radial_local = np.array((
                center_local[0] / center_radius,
                center_local[1] / center_radius,
                0.0))
            radial_world = ring_rotation @ radial_local
            pad_rotation = self.data.geom_xmat[pad_geom_id].reshape(3, 3)
            pad_size = self.model.geom_size[pad_geom_id]
            radial_extent = float(np.sum(
                pad_size * np.abs(pad_rotation.T @ radial_world)))
            axial_extent = float(np.sum(
                pad_size * np.abs(pad_rotation.T @ ring_axis_world)))
            surface_gap = (
                self.inner_radius - center_radius - radial_extent)
            overlaps_valid_height = (
                center_local[2] + axial_extent >= self.contact_tolerance and
                center_local[2] - axial_extent <=
                self.ring_height - self.contact_tolerance)
            if surface_gap < detection_gap and overlaps_valid_height:
                nearby_heights[finger].append(float(np.clip(
                    center_local[2], self.contact_tolerance,
                    self.ring_height - self.contact_tolerance)))
                surface_gaps[finger] = 1000.0 * surface_gap

        valid_heights = {finger: [] for finger in ACTIVE_FINGERS}
        maximum_penetration = 0.0
        robot_ring_contacts = 0
        active_rim_contacts = 0
        thumb_contacts = 0
        forbidden_robot_contacts = set()

        for contact in self.data.contact:
            if contact.geom1 in self.ring_geom_ids:
                other_geom_id = contact.geom2
            elif contact.geom2 in self.ring_geom_ids:
                other_geom_id = contact.geom1
            else:
                continue
            if other_geom_id not in self.robot_geom_ids:
                continue

            robot_ring_contacts += 1
            maximum_penetration = max(
                maximum_penetration, max(0.0, -float(contact.dist)))
            finger = self.pad_geom_to_finger.get(other_geom_id)
            if finger is None:
                name = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM,
                    other_geom_id)
                forbidden_robot_contacts.add(name or "<unnamed>")
                continue
            if finger == "thumb":
                thumb_contacts += 1
                continue

            local = ring_rotation.T @ (
                np.asarray(contact.pos) - ring_position)
            radial = math.hypot(local[0], local[1])
            within_axial_band = (
                self.contact_tolerance <= local[2] <=
                self.ring_height - self.contact_tolerance)
            on_inner_wall = (
                radial <= self.inner_radius + self.contact_tolerance and
                within_axial_band)
            if on_inner_wall:
                valid_heights[finger].append(float(local[2]))
            elif not within_axial_band:
                active_rim_contacts += 1

        valid_fingers = sorted(
            finger for finger, heights in valid_heights.items() if heights)
        nearby_fingers = sorted(
            finger for finger, heights in nearby_heights.items() if heights)
        feasible = (
            len(nearby_fingers) >= self.args.required_inner_fingers and
            maximum_penetration <= self.args.max_penetration_mm / 1000.0 and
            thumb_contacts == 0 and
            not forbidden_robot_contacts and
            (not self.args.reject_rim_contacts or
             active_rim_contacts == 0))
        return ExpansionSample(
            expansion=float(expansion),
            feasible=feasible,
            nearby_fingers=nearby_fingers,
            valid_fingers=valid_fingers,
            surface_gaps_mm=surface_gaps,
            nearby_heights_mm={
                finger: [1000.0 * height for height in heights]
                for finger, heights in nearby_heights.items()
                if heights
            },
            maximum_penetration_mm=1000.0 * maximum_penetration,
            robot_ring_contacts=robot_ring_contacts,
            active_rim_contacts=active_rim_contacts,
            thumb_contacts=thumb_contacts,
            forbidden_robot_contacts=sorted(forbidden_robot_contacts),
            contact_heights_mm={
                finger: [1000.0 * height for height in heights]
                for finger, heights in valid_heights.items()
                if heights
            },
        )


def build_parser():
    parser = argparse.ArgumentParser(
        description="Scan the Task 3 partial internal-grasp expansion range")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--expansion-min", type=float, default=0.0)
    parser.add_argument("--expansion-max", type=float, default=0.25)
    parser.add_argument("--expansion-step", type=float, default=0.001)
    parser.add_argument(
        "--detection-gap-mm", type=float,
        help="Override the scene task3_grasp_detection_gap numeric")
    parser.add_argument(
        "--required-inner-fingers", type=int,
        help="Override the scene task3_required_inner_fingers numeric")
    parser.add_argument("--max-penetration-mm", type=float, default=0.0)
    parser.add_argument("--ring-yaw-deg", type=float, default=0.0)
    parser.add_argument("--reject-rim-contacts", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser


def validate_args(parser, args):
    if not 0.0 <= args.expansion_min <= 1.0:
        parser.error("--expansion-min must be in [0, 1]")
    if not 0.0 <= args.expansion_max <= 1.0:
        parser.error("--expansion-max must be in [0, 1]")
    if args.expansion_max < args.expansion_min:
        parser.error("--expansion-max must be >= --expansion-min")
    if args.expansion_step <= 0.0:
        parser.error("--expansion-step must be positive")
    if args.max_penetration_mm < 0.0:
        parser.error("--max-penetration-mm must be non-negative")
    if args.detection_gap_mm is not None and args.detection_gap_mm <= 0.0:
        parser.error("--detection-gap-mm must be positive")
    if (args.required_inner_fingers is not None and
            not 1 <= args.required_inner_fingers <= len(ACTIVE_FINGERS)):
        parser.error("--required-inner-fingers must be in [1, 3]")


def main():
    parser = build_parser()
    args = parser.parse_args()
    validate_args(parser, args)
    scanner = Task3ExpansionScanner(args)

    compact = scanner.scan_sample(0.0)
    samples = [
        scanner.scan_sample(value)
        for value in expansion_values(
            args.expansion_min, args.expansion_max, args.expansion_step)
    ]
    ranges = contiguous_ranges(samples, args.expansion_step)
    release_remaining_limit = (
        len(ACTIVE_FINGERS) - scanner.release_lost_fingers)
    release_transitions = [
        (lower, upper)
        for lower, upper in zip(samples, samples[1:])
        if len(lower.nearby_fingers) <= release_remaining_limit and
        len(upper.nearby_fingers) > release_remaining_limit
    ]

    print("scene={}".format(scanner.scene_path))
    print(
        "candidate_axis_hand=[{:.6f},{:.6f},{:.6f}] ring_yaw_deg={:.3f}".format(
            *CANDIDATE_RING_AXIS_HAND, args.ring_yaw_deg))
    print(
        "compact_contact_free={} compact_contacts={} "
        "compact_penetration_mm={:.3f}".format(
            not compact.valid_fingers and
            compact.thumb_contacts == 0 and
            not compact.forbidden_robot_contacts and
            compact.robot_ring_contacts == 0,
            compact.robot_ring_contacts,
            compact.maximum_penetration_mm))
    print(
        "samples={} feasible={} required_inner_fingers={} "
        "detection_gap_mm={:.3f} max_penetration_mm={:.3f} "
        "reject_rim_contacts={}".format(
            len(samples), sum(sample.feasible for sample in samples),
            args.required_inner_fingers, args.detection_gap_mm,
            args.max_penetration_mm, args.reject_rim_contacts))
    for start, end in ranges:
        print("feasible_expansion=[{:.6f}, {:.6f}]".format(start, end))
    for lower, upper in release_transitions:
        print(
            "two_finger_release_transition=[{:.6f}, {:.6f}] "
            "remaining_fingers=[{}, {}]".format(
                lower.expansion, upper.expansion,
                len(lower.nearby_fingers), len(upper.nearby_fingers)))

    boundary_samples = []
    for sample in samples:
        if sample.feasible or args.verbose:
            boundary_samples.append(sample)
            if args.verbose:
                print(
                    "expansion={:.6f} feasible={} nearby={} contacts={} "
                    "penetration_mm={:.3f} rim_contacts={}".format(
                        sample.expansion, sample.feasible,
                        ",".join(sample.nearby_fingers) or "none",
                        ",".join(sample.valid_fingers) or "none",
                        sample.maximum_penetration_mm,
                        sample.active_rim_contacts))
    if not args.verbose and boundary_samples:
        first = boundary_samples[0]
        last = boundary_samples[-1]
        for label, sample in (("first", first), ("last", last)):
            print(
                "{}_feasible expansion={:.6f} nearby={} gaps_mm={} "
                "penetration_mm={:.3f} rim_contacts={} nearby_heights_mm={}".format(
                    label, sample.expansion,
                    sample.nearby_fingers, sample.surface_gaps_mm,
                    sample.maximum_penetration_mm,
                    sample.active_rim_contacts,
                    sample.nearby_heights_mm))

    if args.output:
        payload = {
            "scene": str(scanner.scene_path),
            "parameters": {
                key: str(value) if isinstance(value, Path) else value
                for key, value in vars(args).items()
                if key != "output"
            },
            "compact": asdict(compact),
            "feasible_ranges": [list(item) for item in ranges],
            "release_transitions": [
                [lower.expansion, upper.expansion]
                for lower, upper in release_transitions
            ],
            "samples": [asdict(sample) for sample in samples],
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        print("output={}".format(args.output.resolve()))

    compact_contact_free = (
        not compact.valid_fingers and
        compact.thumb_contacts == 0 and
        not compact.forbidden_robot_contacts and
        compact.robot_ring_contacts == 0)
    return 0 if compact_contact_free and ranges else 2


if __name__ == "__main__":
    sys.exit(main())
