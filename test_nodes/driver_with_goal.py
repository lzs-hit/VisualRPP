#!/usr/bin/env python3
"""
Test driver for VisualAvoidanceController state machine.

What this does:
  1. Sends a /follow_path goal to controller_server with a 5m straight path
     (so the controller starts ticking our plugin at 10Hz).
  2. Publishes cycling obstacles to /obstacle_info every 0.5s, switching
     presets every 3s (no obstacle -> center -> right -> left -> repeat).
  3. Subscribes to /cmd_vel and logs when the linear.x deviates from the
     expected NORMAL value of 0.5 m/s (i.e., whenever the plugin switches
     into AVOIDING or RECOVERY).

What you should see:
  - controller_server log: "VisualAvoidanceController 'VisualRPP' ready"
  - driver log: "Sent FollowPath goal with 5 poses"
  - Every ~6s, when the obstacle enters the cone: the plugin logs
    "[VisualAvoidance] NORMAL -> AVOIDING ..." in the controller_server log
  - AVOIDING state emits cmd_vel with angular.z != 0
  - After obstacle leaves: "[VisualAvoidance] AVOIDING -> RECOVERY"
  - After recovery timeout: "[VisualAvoidance] RECOVERY -> NORMAL"

Usage (with the smoke test already running):
  T1: ros2 launch visual_avoidance_controller plugin_only_smoke_test.launch.py
  T2: python3 driver_with_goal.py
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, TwistStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


# (x, y, label) -- cycling every 3 seconds.
# y=1e3 effectively disables avoidance (past y_threshold).
PRESETS = [
    (0.0, 1e3, "no obstacle (very far)"),
    (0.0, 0.6, "center near"),
    (0.3, 0.5, "right-of-center near"),
    (-0.3, 0.5, "left-of-center near"),
]
PRESET_PERIOD_S = 3.0
NORMAL_LINEAR_VEL = 0.5  # must match desired_linear_vel in plugin params


class TestDriver(Node):
    def __init__(self):
        super().__init__("test_driver")

        self._action_client = ActionClient(self, FollowPath, "/follow_path")
        self._obs_pub = self.create_publisher(PoseArray, "/obstacle_info", 10)
        self._cmd_sub = self.create_subscription(
            TwistStamped, "/cmd_vel", self._cmd_callback, 10)

        self._start_time = time.time()
        self._last_linear = None
        self._last_omega = None
        self._cmd_count = 0

        # Send the goal once, after a 2-second delay (so controller_server
        # is fully active by then).
        self._goal_timer = self.create_timer(2.0, self._send_goal_once)

        # Publish obstacles every 0.2 s (faster than the cone-zone tests need,
        # but cheap and gives smoother state transitions).
        self._obs_timer = self.create_timer(0.2, self._publish_obstacle)

        self.get_logger().info("test_driver ready, will send FollowPath goal in 2s")

    def _send_goal_once(self):
        # Cancel this one-shot timer.
        self._goal_timer.cancel()

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "/follow_path action server not available -- "
                "is the controller_server running?")
            return

        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()

        # 5 poses in a straight line, 1 m apart, heading +x.
        for i in range(5):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(i) * 1.0
            p.pose.position.y = 0.0
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "VisualRPP"
        goal.goal_checker_id = "goal_checker"

        self._action_client.send_goal_async(goal)
        self.get_logger().info(
            f"Sent FollowPath goal with {len(path.poses)} poses "
            f"(straight line, 5 m, controller=VisualRPP)")

    def _publish_obstacle(self):
        elapsed = time.time() - self._start_time
        slot = int(elapsed / PRESET_PERIOD_S) % len(PRESETS)
        x, y, label = PRESETS[slot]

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        if y < 3.0 and y > 0.0:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0.0
            p.orientation.w = 1.0
            msg.poses.append(p)

        self._obs_pub.publish(msg)

        # Log on preset transitions, not every tick.
        slot_whole = int(elapsed)
        if slot_whole != getattr(self, "_last_logged_slot", -1):
            self._last_logged_slot = slot_whole
            self.get_logger().info(f"preset: {label} (x={x:.2f}, y={y:.2f})")

    def _cmd_callback(self, msg: TwistStamped):
        self._cmd_count += 1
        self._last_linear = msg.twist.linear.x
        self._last_omega = msg.twist.angular.z

        # Log only when linear or angular velocity changes significantly,
        # so we can see state transitions without flooding the log.
        prev_lin = getattr(self, "_logged_linear", None)
        prev_omega = getattr(self, "_logged_omega", None)
        lin_changed = (prev_lin is None or
                       abs(self._last_linear - prev_lin) > 0.05)
        omega_changed = (prev_omega is None or
                         abs(self._last_omega - prev_omega) > 0.1)
        if lin_changed or omega_changed:
            self.get_logger().info(
                f"/cmd_vel #{self._cmd_count}: linear.x={self._last_linear:.2f}, "
                f"angular.z={self._last_omega:+.2f}")
            self._logged_linear = self._last_linear
            self._logged_omega = self._last_omega


def main(args=None):
    rclpy.init(args=args)
    node = TestDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
