# plugin_only_smoke_test.launch.py
#
# A truly minimal launch that only spins up controller_server + a
# local_costmap (no global costmap, no planner, no AMCL, no lidar, no
# map server, no robot). The sole purpose is to verify that our
# VisualAvoidanceController plugin:
#   - is discoverable by pluginlib (already shown by `ros2 plugin list`)
#   - can be loaded into a controller_server without errors
#   - reads its parameters successfully (you'll see the INFO log "ready:")
#
# If controller_server starts and the log shows "VisualAvoidanceController
# 'VisualRPP' ready", the plugin is working. If it crashes (e.g.
# "pluginlib Exception: cannot load plugin"), you'll see why here
# without the noise of a full nav2 bringup.
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   source /root/ros2_ws/install/setup.bash
#   ros2 launch visual_avoidance_controller plugin_only_smoke_test.launch.py

import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Static TF so controller_server is not angry at TF lookup failures.
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    # )

    # Lifecycle manager that only manages the controller_server + its costmap.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_smoke",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "bond_timeout": 4.0,
            "node_names": ["/controller_server"],
            "attempt_respawn_reconnection": True,
        }],
    )

    # controller_server alone, configured to load VisualRPP. No global
    # costmap, no local costmap even -- the plugin should instantiate
    # and configure() without touching costmaps, because that work is
    # deferred until activate().
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "controller_frequency": 10.0,
            "controller_plugins": ["VisualRPP"],
            "goal_checker_plugins": ["goal_checker"],
            "goal_checker": {
                "plugin": "nav2_controller::SimpleGoalChecker",
                "xy_goal_tolerance": 0.2,
                "yaw_goal_tolerance": 6.28,
                "stateful": True,
            },
            "VisualRPP": {
                "plugin": "visual_avoidance_controller::VisualAvoidanceController",
                "desired_linear_vel": 0.5,
                "lookahead_dist": 0.4,
                "min_lookahead_dist": 0.4,
                "max_lookahead_dist": 0.8,
                "use_velocity_scaled_lookahead_dist": False,
                "transform_tolerance": 0.1,
                "use_rotate_to_heading": False,
                "allow_reversing": False,
                # visual avoidance params (use defaults)
                "obstacle_topic":          "/obstacle_info",
                "cone_zone_a":             75.0,
                "cone_zone_b":             20.0,
                "cone_zone_c":             12.0,
                "avoid_R":                 0.35,
                "avoid_bigR":              0.45,
                "y_threshold":             0.75,
                "offset_x_first_half":    -0.12,
                "offset_x_second_half":   -0.08,
                "recovery_duration":       0.58,
                "avoid_linear_vel":        0.4,
                "max_angular_vel":         1.0,
                "enable_visual_avoidance": True,
            },
        }],
    )

    return LaunchDescription([
        # static_tf,
        controller_server,
        lifecycle_manager,
    ])
