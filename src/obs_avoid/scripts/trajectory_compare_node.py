#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TrajectoryCompareNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_compare_node")

        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("odom_topic", "/mavros/local_position/odom")
        self.declare_parameter("ground_truth_topic", "/ground_truth/odom")
        self.declare_parameter("use_ground_truth", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("append_distance_threshold", 0.05)
        self.declare_parameter("max_path_points", 20000)

        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.gt_topic = self.get_parameter("ground_truth_topic").get_parameter_value().string_value
        self.use_ground_truth = self.get_parameter("use_ground_truth").get_parameter_value().bool_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.publish_hz = max(1.0, self.get_parameter("publish_hz").get_parameter_value().double_value)
        self.dist_thresh = max(
            0.0, self.get_parameter("append_distance_threshold").get_parameter_value().double_value
        )
        self.max_path_points = max(
            100, self.get_parameter("max_path_points").get_parameter_value().integer_value
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_tf_warn_ns = 0

        self.odom_path = Path()
        self.odom_path.header.frame_id = self.odom_frame
        self.map_path = Path()
        self.map_path.header.frame_id = self.map_frame
        self.gt_path = Path()
        self.gt_path.header.frame_id = self.odom_frame

        self.odom_path_pub = self.create_publisher(Path, "/mapping/debug/path_odom", 10)
        self.map_path_pub = self.create_publisher(Path, "/mapping/debug/path_map", 10)
        self.gt_path_pub = self.create_publisher(Path, "/mapping/debug/path_ground_truth", 10)

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        if self.use_ground_truth:
            self.create_subscription(Odometry, self.gt_topic, self.gt_cb, 20)

        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

        self.get_logger().info(
            "trajectory_compare_node started. odom_topic='%s' map_frame='%s' base_frame='%s' gt=%s('%s')"
            % (
                self.odom_topic,
                self.map_frame,
                self.base_frame,
                "on" if self.use_ground_truth else "off",
                self.gt_topic,
            )
        )

    @staticmethod
    def pose_distance(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def maybe_append(self, path: Path, pose: PoseStamped) -> None:
        if path.poses:
            if self.pose_distance(path.poses[-1], pose) < self.dist_thresh:
                return
        path.poses.append(pose)
        if len(path.poses) > self.max_path_points:
            drop_n = len(path.poses) - self.max_path_points
            del path.poses[:drop_n]

    def odom_cb(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = self.odom_frame
        pose.pose = msg.pose.pose
        self.maybe_append(self.odom_path, pose)

    def gt_cb(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = self.odom_frame
        pose.pose = msg.pose.pose
        self.maybe_append(self.gt_path, pose)

    def on_timer(self) -> None:
        now = self.get_clock().now().to_msg()

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            pose = PoseStamped()
            pose.header.stamp = tf.header.stamp
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            self.maybe_append(self.map_path, pose)
        except TransformException as ex:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_warn_ns > 2_000_000_000:
                self.last_tf_warn_ns = now_ns
                self.get_logger().warn(f"map->base TF lookup failed: {ex}")

        self.odom_path.header.stamp = now
        self.map_path.header.stamp = now
        self.gt_path.header.stamp = now

        self.odom_path_pub.publish(self.odom_path)
        self.map_path_pub.publish(self.map_path)
        if self.use_ground_truth:
            self.gt_path_pub.publish(self.gt_path)


def main() -> None:
    rclpy.init()
    node = TrajectoryCompareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
