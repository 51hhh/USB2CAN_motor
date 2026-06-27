#!/usr/bin/env python3
"""Republish a live Odometry stream under safe RViz-only frames/topics."""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster


class LiveOdomAdapter(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("live_odom_adapter")
        self.args = args
        self.path_poses = []

        reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if args.qos_reliability == "best_effort"
            else ReliabilityPolicy.RELIABLE
        )
        qos = QoSProfile(
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=args.qos_depth,
        )

        self.odom_pub = self.create_publisher(Odometry, args.output_odom, qos)
        self.path_pub = self.create_publisher(Path, args.path_topic, qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, args.input_odom, self.on_odom, qos)

        self.get_logger().info(
            "live odom adapter: %s -> %s, frame %s -> %s",
            args.input_odom,
            args.output_odom,
            args.frame_id,
            args.child_frame_id,
        )

    def on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        if self.args.stamp_policy == "now":
            out.header.stamp = self.get_clock().now().to_msg()
        elif out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()

        out.header.frame_id = self.args.frame_id
        out.child_frame_id = self.args.child_frame_id
        self.odom_pub.publish(out)

        pose = PoseStamped()
        pose.header = out.header
        pose.pose = out.pose.pose
        self.path_poses.append(pose)
        if len(self.path_poses) > self.args.max_path_poses:
            del self.path_poses[: len(self.path_poses) - self.args.max_path_poses]

        path = Path()
        path.header = out.header
        path.poses = self.path_poses
        self.path_pub.publish(path)

        if self.args.publish_tf:
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = out.child_frame_id
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main() -> int:
    parser = argparse.ArgumentParser(description="Live Odometry adapter for safe RViz viewing")
    parser.add_argument("--input-odom", default="/odom", help="Input odometry topic")
    parser.add_argument("--output-odom", default="/remote_view/odom", help="Output odometry topic")
    parser.add_argument("--path-topic", default="/remote_view/path", help="Generated path topic")
    parser.add_argument("--frame-id", default="remote_odom", help="Output odometry frame")
    parser.add_argument("--child-frame-id", default="remote_base_link", help="Output child frame")
    parser.add_argument("--max-path-poses", type=int, default=5000, help="Maximum path poses")
    parser.add_argument("--stamp-policy", choices=["preserve", "now"], default="preserve")
    parser.add_argument("--qos-reliability", choices=["reliable", "best_effort"], default="reliable")
    parser.add_argument("--qos-depth", type=int, default=50)
    parser.add_argument("--no-tf", dest="publish_tf", action="store_false", help="Do not publish TF")
    parser.set_defaults(publish_tf=True)
    args = parser.parse_args()

    rclpy.init()
    node = LiveOdomAdapter(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
