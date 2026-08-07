#!/usr/bin/env python3
"""
Dummy obstacle publisher for testing VisualAvoidanceController without a
real YOLO/Aurora pipeline.

Publishes a `geometry_msgs/PoseArray` to `/obstacle_info` containing a
single obstacle in the robot base_link frame, so the controller can be
verified end-to-end before the YOLO+depth node is built.

Usage:
    # spawn an obstacle 0.8 m ahead, dead-center
    python3 dummy_obstacle_publisher.py \
        --ros-args -p x:=0.0 -p y:=0.8 -p cycle_period:=0.0

    # cycle obstacles (cone-region tests):
    #   t=0  -> (0, 0.6)   center, near
    #   t=4  -> (0.3, 0.5) right of center, near
    #   t=8  -> (-0.3, 0.5) left of center, near
    #   t=12 -> empty
    python3 dummy_obstacle_publisher.py \
        --ros-args -p cycle_period:=4.0

Parameters:
    x              (double, default 0.0) -- obstacle x in base_link frame
    y              (double, default 0.8) -- obstacle y in base_link frame (front)
    cycle_period   (double, default 0.0) -- if > 0, cycle through presets every N sec
    rate_hz        (double, default 5.0) -- publish rate
    frame_id       (string, default "base_link")
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


PRESETS = [
    # (x, y, label)
    (0.0, 0.6, "center near"),
    (0.3, 0.5, "right-of-center near"),
    (-0.3, 0.5, "left-of-center near"),
    (0.0, 1e3, "no obstacle (very far)"),  # past y_threshold -> no avoidance
]


class DummyObstaclePublisher(Node):
    def __init__(self):
        super().__init__("dummy_obstacle_publisher")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.8)
        self.declare_parameter("cycle_period", 0.0)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("frame_id", "base_link")

        self.x = self.get_parameter("x").value
        self.y = self.get_parameter("y").value
        self.cycle_period = self.get_parameter("cycle_period").value
        self.rate_hz = self.get_parameter("rate_hz").value
        self.frame_id = self.get_parameter("frame_id").value

        self.pub = self.create_publisher(PoseArray, "/obstacle_info", 10)

        self.start_time = time.time()
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self.tick)

        self.get_logger().info(
            f"dummy_obstacle_publisher started: x={self.x}, y={self.y}, "
            f"cycle_period={self.cycle_period}s, rate={self.rate_hz}Hz"
        )

    def tick(self):
        if self.cycle_period > 0.0:
            elapsed = time.time() - self.start_time
            slot = int(elapsed / self.cycle_period) % len(PRESETS)
            x, y, label = PRESETS[slot]
        else:
            x, y = self.x, self.y
            label = "static"

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Only publish a non-empty array when the obstacle is "in front of" us
        # (y < 3.0 matches the YOLO.cpp convention from the senior code).
        if y < 3.0 and y > 0.0:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0.0
            p.orientation.w = 1.0
            msg.poses.append(p)

        self.pub.publish(msg)

        # Log only on preset transitions, not every tick.
        if self.cycle_period > 0.0:
            now_secs = time.time() - self.start_time
            # Log when we cross into a new preset slot (whole-second multiple).
            if int(now_secs) % int(self.cycle_period) == 0 and \
               int((now_secs + 0.1)) % int(self.cycle_period) == 0:
                pass  # avoid log spam; comment out the test below
            # Print once per slot change.
            slot_whole = int(now_secs)
            if slot_whole != getattr(self, "_last_logged_slot", -1):
                self._last_logged_slot = slot_whole
                self.get_logger().info(f"preset: {label} (x={x:.2f}, y={y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = DummyObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
