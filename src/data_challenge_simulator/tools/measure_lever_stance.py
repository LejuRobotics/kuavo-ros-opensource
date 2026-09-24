#!/usr/bin/env python3
"""Measure fixed chassis stances for the unchanged Scene 1 lever motion."""

import argparse
import contextlib
import io
import itertools
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from utils.scene1_left_arm_ik import Scene1LeftArmIK


LEVER_PIVOT_WORLD = (0.56, 0.28, 0.92)
LEVER_HOOK_TANGENT_OFFSET = 0.026
LEVER_TRACKING_BIAS_WORLD = (0.0, 0.0, 0.010)


def lever_target(radius, angle):
    geometric_target = (
        LEVER_PIVOT_WORLD[0] - radius * math.sin(angle)
        + LEVER_HOOK_TANGENT_OFFSET * math.cos(angle),
        LEVER_PIVOT_WORLD[1],
        LEVER_PIVOT_WORLD[2] - radius * math.cos(angle)
        - LEVER_HOOK_TANGENT_OFFSET * math.sin(angle),
    )
    return tuple(
        value - bias
        for value, bias in zip(geometric_target, LEVER_TRACKING_BIAS_WORLD))


def evaluate(candidate):
    (radius, base_x, base_y, base_yaw_deg, step_deg, completion_deg,
     required_margin) = candidate
    ik = Scene1LeftArmIK()
    ik.set_base_pose_world(
        (base_x, base_y, 0.0), math.radians(base_yaw_deg))
    seed = ik.READY
    worst_margin = 1.0
    limiting_angle = None
    limiting_joint = None
    limiting_value = None
    max_position_error = 0.0
    max_row_axis_error = 0.0

    for angle_deg in np.arange(0.0, completion_deg + 0.5 * step_deg, step_deg):
        target = lever_target(radius, math.radians(float(angle_deg)))
        try:
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                solution = np.asarray(ik.solve_lever(
                    target, seed,
                    joint_limit_margin_fraction=required_margin))
        except RuntimeError:
            return None

        measured, _, measured_rotation = ik._forward(solution)
        max_position_error = max(
            max_position_error,
            float(np.linalg.norm(measured - np.asarray(target))))
        row_axis_error = math.acos(np.clip(
            np.dot(measured_rotation[:, 0], (0.0, -1.0, 0.0)),
            -1.0, 1.0))
        max_row_axis_error = max(max_row_axis_error, row_axis_error)
        span = ik.upper - ik.lower
        normalized_margin = np.minimum(
            (solution - ik.lower) / span,
            (ik.upper - solution) / span,
        )
        index = int(np.argmin(normalized_margin))
        if normalized_margin[index] < worst_margin:
            worst_margin = float(normalized_margin[index])
            limiting_angle = float(angle_deg)
            limiting_joint = index + 1
            limiting_value = float(solution[index])
        seed = solution

    return {
        "radius": radius,
        "base_x": base_x,
        "base_y": base_y,
        "base_yaw_deg": base_yaw_deg,
        "margin": worst_margin,
        "limiting_angle_deg": limiting_angle,
        "limiting_joint": limiting_joint,
        "limiting_value": limiting_value,
        "max_position_error": max_position_error,
        "max_row_axis_error": max_row_axis_error,
    }


def comma_floats(text):
    return [float(value) for value in text.split(",")]


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Grid-search chassis x/y/yaw and rank each stance by the minimum "
            "normalized joint-limit margin over the unchanged lever arc."))
    parser.add_argument("--radii", type=comma_floats, default=[0.10])
    parser.add_argument("--base-x", type=comma_floats, default=[-0.10, 0.0, 0.10])
    parser.add_argument("--base-y", type=comma_floats, default=[0.05, 0.15, 0.25])
    parser.add_argument("--base-yaw-deg", type=comma_floats,
                        default=[-45.0, -30.0, -15.0, 0.0])
    parser.add_argument("--step-deg", type=float, default=15.0)
    parser.add_argument("--completion-deg", type=float, default=45.0)
    parser.add_argument("--top", type=int, default=5)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--required-margin", type=float, default=0.10)
    args = parser.parse_args()

    candidates = [
        values + (args.step_deg, args.completion_deg, args.required_margin)
        for values in itertools.product(
            args.radii, args.base_x, args.base_y, args.base_yaw_deg)
    ]
    with ProcessPoolExecutor(max_workers=args.jobs) as executor:
        results = [result for result in executor.map(evaluate, candidates)
                   if result is not None]

    for radius in args.radii:
        ranked = sorted(
            (result for result in results if result["radius"] == radius),
            key=lambda result: (
                result["margin"], -result["max_position_error"]),
            reverse=True,
        )
        print("radius={:.3f} m feasible={}/{}".format(
            radius, len(ranked), len(candidates) // len(args.radii)))
        for result in ranked[:args.top]:
            print(
                "  base=({:+.3f}, {:+.3f}, {:+.1f} deg) "
                "min_margin={:.1f}% joint=l{} angle={:.1f} deg "
                "max_position_error={:.2f} mm max_row_axis_error={:.2f} deg".format(
                    result["base_x"], result["base_y"],
                    result["base_yaw_deg"], 100.0 * result["margin"],
                    result["limiting_joint"],
                    result["limiting_angle_deg"],
                    1000.0 * result["max_position_error"],
                    math.degrees(result["max_row_axis_error"]),
                ))


if __name__ == "__main__":
    main()
