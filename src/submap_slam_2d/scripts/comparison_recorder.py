#!/usr/bin/env python3
"""Record parallel slam_toolbox/submap outputs without claiming ground truth."""

import csv
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


class Recorder(Node):
    def __init__(self, output_dir: Path):
        super().__init__("submap_slam_comparison_recorder")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.slam_points = []
        self.submap_points = []
        self.slam_map = None
        self.submap_map = None
        self.diagnostics = []
        self.loop_events = []
        self.optimization_events = []
        self.last_loop_count = 0
        self.last_optimization_cost = None
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._slam_map, map_qos)
        self.create_subscription(
            OccupancyGrid, "/submap_slam/map", self._submap_map, map_qos
        )
        self.create_subscription(
            RosPath, "/submap_slam/trajectory", self._submap_path, map_qos
        )
        self.create_subscription(
            DiagnosticArray, "/submap_slam/diagnostics", self._diagnostic, 10
        )
        self.create_timer(0.1, self._sample_slam_pose)

    def _slam_map(self, message):
        self.slam_map = message

    def _submap_map(self, message):
        self.submap_map = message

    def _submap_path(self, message):
        self.submap_points = [
            (
                pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9,
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in message.poses
        ]

    def _sample_slam_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(), Duration(seconds=0.02)
            )
        except TransformException:
            return
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        point = (
            stamp,
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        if not self.slam_points or point[0] > self.slam_points[-1][0]:
            self.slam_points.append(point)

    def _diagnostic(self, message):
        if not message.status:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        values = {item.key: item.value for item in message.status[0].values}
        values["stamp"] = stamp
        self.diagnostics.append(values)
        loops = int(values.get("loop_closures_accepted", "0"))
        if loops > self.last_loop_count:
            self.loop_events.append((stamp, loops))
            self.last_loop_count = loops
        cost = values.get("last_optimization_final_cost")
        if cost is not None and cost != self.last_optimization_cost:
            self.optimization_events.append((stamp, cost))
            self.last_optimization_cost = cost

    @staticmethod
    def _write_trajectory(path, points):
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("stamp", "x", "y"))
            writer.writerows(points)

    @staticmethod
    def _write_map(path, message):
        if message is None or not message.data:
            path.write_bytes(b"P5\n1 1\n255\n\xcd")
            return
        values = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        image = np.full(values.shape, 205, dtype=np.uint8)
        image[values >= 65] = 0
        image[(values >= 0) & (values < 25)] = 254
        image = np.flipud(image)
        with path.open("wb") as stream:
            stream.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode())
            stream.write(image.tobytes())

    def _write_overlay(self):
        all_points = self.slam_points + self.submap_points
        canvas = np.full((900, 900, 3), 245, dtype=np.uint8)
        if not all_points:
            cv2.imwrite(str(self.output_dir / "trajectory_overlay.png"), canvas)
            return
        xs = [point[1] for point in all_points]
        ys = [point[2] for point in all_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        scale = 820.0 / max(max_x - min_x, max_y - min_y, 0.1)

        def pixels(points):
            return np.asarray(
                [
                    (int(40 + (x - min_x) * scale), int(860 - (y - min_y) * scale))
                    for _, x, y in points
                ],
                dtype=np.int32,
            )

        for points, color in ((self.slam_points, (220, 80, 30)), (self.submap_points, (30, 150, 40))):
            line = pixels(points)
            if len(line) > 1:
                cv2.polylines(canvas, [line], False, color, 2, cv2.LINE_AA)
        cv2.putText(canvas, "slam_toolbox", (30, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 80, 30), 2)
        cv2.putText(canvas, "submap_slam", (220, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (30, 150, 40), 2)
        cv2.imwrite(str(self.output_dir / "trajectory_overlay.png"), canvas)

    @staticmethod
    def _metrics(points):
        travelled = sum(
            math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(points, points[1:])
        )
        closure = math.hypot(points[-1][1] - points[0][1], points[-1][2] - points[0][2]) if points else 0.0
        return travelled, closure

    def save(self):
        self._write_trajectory(self.output_dir / "trajectory_slam_toolbox.csv", self.slam_points)
        self._write_trajectory(self.output_dir / "trajectory_submap_slam.csv", self.submap_points)
        self._write_map(self.output_dir / "slam_toolbox_map.pgm", self.slam_map)
        self._write_map(self.output_dir / "submap_slam_map.pgm", self.submap_map)
        keys = sorted({key for row in self.diagnostics for key in row})
        with (self.output_dir / "diagnostics_submap_slam.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.diagnostics)
        for name, rows in (("loop_closures.csv", self.loop_events), ("optimization_events.csv", self.optimization_events)):
            with (self.output_dir / name).open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("stamp", "value"))
                writer.writerows(rows)
        summary = ["No ground-truth accuracy is claimed.", f"loop_closures={self.last_loop_count}"]
        for name, points in (("slam_toolbox", self.slam_points), ("submap_slam", self.submap_points)):
            travelled, closure = self._metrics(points)
            summary.extend((f"{name}_total_travel_m={travelled:.6f}", f"{name}_return_to_start_m={closure:.6f}"))
        (self.output_dir / "summary.txt").write_text("\n".join(summary) + "\n")
        self._write_overlay()


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: comparison_recorder.py OUTPUT_DIR")
    output_dir = Path(sys.argv[1])
    rclpy.init()
    node = Recorder(output_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
