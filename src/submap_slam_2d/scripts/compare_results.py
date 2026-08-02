#!/usr/bin/env python3
"""Generate trajectory summaries and an overlay from exported CSV files."""

import argparse
import csv
import math
from pathlib import Path


def load(path: Path):
    if not path.exists():
        return []
    with path.open(newline="") as stream:
        return [(float(row["x"]), float(row["y"])) for row in csv.DictReader(stream)]


def metrics(points):
    distance = sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])
    )
    closure = math.hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1]) if points else 0.0
    return distance, closure


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    slam = load(args.output_dir / "trajectory_slam_toolbox.csv")
    submap = load(args.output_dir / "trajectory_submap_slam.csv")
    lines = ["No ground-truth accuracy is claimed."]
    for name, points in (("slam_toolbox", slam), ("submap_slam", submap)):
        distance, closure = metrics(points)
        lines.append(f"{name}_total_travel_m={distance:.6f}")
        lines.append(f"{name}_return_to_start_m={closure:.6f}")
    (args.output_dir / "summary.txt").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
