#!/usr/bin/env python3
"""Preview the Task 3 compact-to-internal-expansion hand path."""

import argparse
import sys
import time
from pathlib import Path

import mujoco


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.hand_postures import internal_expansion_pose


SCENES = {
    "hand": SRC_DIR / "kuavo_assets/models/biped_s400062/xml/scene_hand_posture.xml",
    "task3": SRC_DIR / "data_challenge_simulator/models/biped_s400062/xml/task3.xml",
}


def apply_pose(model, data, pose):
    for name, value in pose.items():
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError("Joint is missing from model: {}".format(name))
        lower, upper = model.jnt_range[joint_id]
        if not lower <= value <= upper:
            raise RuntimeError(
                "{}={} is outside [{}, {}]".format(
                    name, value, lower, upper))
        data.qpos[model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(model, data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--expansion", type=float, required=True,
        help="0=compact insertion, 1=three-finger inner-wall support")
    parser.add_argument("--side", choices=("l", "r"), default="r")
    parser.add_argument("--scene", choices=tuple(SCENES), default="hand")
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()

    pose = internal_expansion_pose(args.expansion, args.side)
    model = mujoco.MjModel.from_xml_path(str(SCENES[args.scene]))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    apply_pose(model, data, pose)

    print("scene={}".format(SCENES[args.scene]))
    print("expansion={:.3f}".format(args.expansion))
    for name, value in pose.items():
        print("  {:20s} {:.6f}".format(name, value))
    if args.no_viewer:
        return

    from mujoco import viewer as mujoco_viewer
    with mujoco_viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
