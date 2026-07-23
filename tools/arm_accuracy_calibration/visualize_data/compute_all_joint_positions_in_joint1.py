#!/usr/bin/env python3
import argparse
import csv
import re
from pathlib import Path

import numpy as np


FILE_PATTERN = re.compile(r"^T_joint(\d+)_in_joint(\d+)_fitted\.csv$")


def load_matrix(csv_file: Path) -> np.ndarray:
    with csv_file.open("r", newline="") as f:
        reader = csv.DictReader(f)
        row = next(reader, None)
        if row is None:
            raise ValueError(f"Empty csv file: {csv_file}")

    matrix = np.zeros((4, 4), dtype=float)
    for r in range(4):
        for c in range(4):
            key = f"m{r}{c}"
            if key not in row:
                raise KeyError(f"Missing key '{key}' in {csv_file}")
            matrix[r, c] = float(row[key])
    return matrix


def collect_transforms(input_dir: Path):
    transforms = {}
    for csv_file in sorted(input_dir.glob("T_joint*_in_joint*_fitted.csv")):
        match = FILE_PATTERN.match(csv_file.name)
        if not match:
            continue
        child_joint = int(match.group(1))
        parent_joint = int(match.group(2))
        transforms[(child_joint, parent_joint)] = load_matrix(csv_file)
    return transforms


def compute_positions_in_joint1(transforms):
    if (2, 1) not in transforms:
        raise ValueError("Missing required transform: T_joint2_in_joint1_fitted.csv")

    positions = {}
    cumulative = np.eye(4, dtype=float)
    parent = 1
    child = 2

    while (child, parent) in transforms:
        cumulative = cumulative @ transforms[(child, parent)]
        positions[child] = cumulative[:3, 3].copy()
        parent = child
        child += 1

    return positions


def write_positions_csv(output_csv: Path, positions):
    with output_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["joint", "x", "y", "z"])
        for joint_idx in sorted(positions.keys()):
            x, y, z = positions[joint_idx]
            writer.writerow([joint_idx, x, y, z])


def print_positions_table(positions):
    headers = ["joint", "x", "y", "z"]
    rows = []
    for joint_idx in sorted(positions.keys()):
        x, y, z = positions[joint_idx]
        rows.append([f"joint{joint_idx}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])

    col_widths = [len(h) for h in headers]
    for row in rows:
        for i, value in enumerate(row):
            col_widths[i] = max(col_widths[i], len(value))

    border = "+" + "+".join("-" * (w + 2) for w in col_widths) + "+"
    header_line = (
        "| "
        + " | ".join(headers[i].ljust(col_widths[i]) for i in range(len(headers)))
        + " |"
    )

    print(border)
    print(header_line)
    print(border)
    for row in rows:
        line = "| " + " | ".join(row[i].ljust(col_widths[i]) for i in range(len(row))) + " |"
        print(line)
    print(border)


def main():
    parser = argparse.ArgumentParser(
        description="Multiply fitted transforms and get all joint positions in joint1 frame."
    )
    parser.add_argument(
        "--target-dir",
        type=Path,
        required=True,
        help="Directory containing T_jointX_in_jointY_fitted.csv files (required).",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=None,
        help="Output csv path. Default: <input-dir>/joint_positions_in_joint1.csv",
    )
    args = parser.parse_args()

    input_dir = args.target_dir.resolve()
    if not input_dir.is_dir():
        raise NotADirectoryError(f"Target directory does not exist: {input_dir}")

    output_csv = (
        args.output_csv.resolve()
        if args.output_csv is not None
        else input_dir / "joint_positions_in_joint1.csv"
    )

    transforms = collect_transforms(input_dir)
    positions = compute_positions_in_joint1(transforms)
    write_positions_csv(output_csv, positions)

    print(f"Input dir: {input_dir}")
    print(f"Output csv: {output_csv}")
    print("joint in joint1 positions:")
    print_positions_table(positions)


if __name__ == "__main__":
    main()
