#!/usr/bin/env python3
"""
rpp_visual.launch.py - Competition-grade full nav2 bringup with VisualAvoidance.

This is the visual_avoidance_controller extension of your existing
nav_pkg/launch/rpp.launch.py. It starts every node the original launch does
(plus the VisualRPP controller plugin via the visual_rpp.yaml params) and
adds three toggle arguments.

Quick start:
  # 1) Default -- vision avoidance ON, no dummy obstacles (use real YOLO).
  ros2 launch visual_avoidance_controller rpp_visual.launch.py \
      enable_yolo_bridge:=true

  # 2) Revert to the stock RPP controller (no vision avoidance).
  ros2 launch visual_avoidance_controller rpp_visual.launch.py \
      use_visual_avoidance:=false

  # 3) Use the dummy obstacle publisher for testing without a real YOLO node.
  ros2 launch visual_avoidance_controller rpp_visual.launch.py \
      enable_dummy_obstacle:=true

  # 4) Override the params file entirely (point at any nav2 yaml).
  ros2 launch visual_avoidance_controller rpp_visual.launch.py \
      nav_params_file:=/path/to/your.yaml

Launch arguments:
  use_visual_avoidance (bool, default 'true')
      true:  pass visual_rpp.yaml (VisualRPP controller) to nav2
      false: pass rpp.yaml       (standard RegulatedPurePursuitController)

  enable_dummy_obstacle (bool, default 'false')
      true:  start dummy_obstacle_publisher.py so the avoidance state machine
             has data to react to even without a real YOLO node.
      false: assume YOLO (or a real detection node) will publish to
             /obstacle_info; don't start the dummy.

  enable_yolo_bridge (bool, default 'false')
      true:  Include obstacle_info_bridge's launch which starts dnn_node
             (BPU YOLO) + the new bridge node. Populates /obstacle_info
             from real Aurora 930 RGB + depth.
      false: don't start YOLO/BPU. Use this if the YOLO model isn't on
             disk yet, or if you want to test the nav2 path with
             enable_dummy_obstacle:=true only.
      Note: exactly ONE of {enable_dummy_obstacle, enable_yolo_bridge}
            should be true. The dummy is for tests; the YOLO bridge is
            for the real competition.

  nav_params_file (string, default = visual_rpp.yaml)
      Full path to the nav2 yaml params file. Override to use a custom
      yaml or to test against stock rpp.yaml.

  yolo_workconfig_file (string, default = /root/dev_ws/models/obstacle_yolov8.json)
      Used only when enable_yolo_bridge:=true. See obstacle_info_bridge
      package README for how to obtain a real .hbm model.
"""

import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ─── Launch args ─────────────────────────────────────────────────────
    declare_use_visual = DeclareLaunchArgument(
        'use_visual_avoidance',
        default_value='true',
        description=(
            'true = use VisualRPP plugin (visual_rpp.yaml);  '
            'false = fall back to stock RPP (rpp.yaml).'))

    declare_enable_dummy = DeclareLaunchArgument(
        'enable_dummy_obstacle',
        default_value='false',
        description=(
            'true = start dummy_obstacle_publisher.py so the state machine '
            'has data to react to;  '
            'false = assume a real detector publishes to /obstacle_info.'))

    declare_enable_yolo = DeclareLaunchArgument(
        'enable_yolo_bridge',
        default_value='false',
        description=(
            'true = include obstacle_info_bridge launch (dnn_node BPU YOLO '
            '+ new bridge node) which feeds /obstacle_info from real '
            'Aurora 930 RGB + depth.  false = skip YOLO (use this for tests).'))

    declare_nav_params = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(
            get_package_share_directory('visual_avoidance_controller'),
            'config', 'visual_rpp.yaml'),
        description='nav2 yaml params file (default: visual_rpp.yaml).')

    declare_yolo_workconfig = DeclareLaunchArgument(
        'yolo_workconfig_file',
        default_value='/root/dev_ws/models/obstacle_yolov8.json',
        description=(
            'Path to dnn_node JSON config. Used only when '
            'enable_yolo_bridge:=true.'))

    use_visual_avoidance = LaunchConfiguration('use_visual_avoidance')
    enable_dummy_obstacle = LaunchConfiguration('enable_dummy_obstacle')
    enable_yolo_bridge = LaunchConfiguration('enable_yolo_bridge')
    nav_params_file = LaunchConfiguration('nav_params_file')
    yolo_workconfig_file = LaunchConfiguration('yolo_workconfig_file')

    # ─── Workspace paths (mirrors nav_pkg/launch/rpp.launch.py) ─────────
    nav_pkg_dir = get_package_share_directory('nav_pkg')
    visual_pkg_dir = get_package_share_directory('visual_avoidance_controller')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    lidar_launch_dir = (
        '/root/lidar_ws/install/lslidar_driver/share/lslidar_driver/launch'
    )
    base_launch_dir = (
        '/userdata/dev_ws/install/origincar_base/share/origincar_base/launch'
    )
    deptrum_dir = get_package_share_directory('deptrum-ros-driver-aurora930')

    qrcode_launch_path = (
        '/root/qrcode_decoding/src/launch/qr.launch.py'
    )
    vlm_launch_path = '/root/hobot_llamacpp/launch/vlm_bringup.launch.py'

    # ─── Map file (used by bringup_launch.py) ────────────────────────────
    map_file = os.path.join(nav_pkg_dir, 'maps', 'keepout.yaml')

    # ─── Sensor drivers (in launch order) ───────────────────────────────
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_launch_dir, 'lsn10_launch.py')))

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_launch_dir, 'origincar_bringup.launch.py')))

    deptrum_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(deptrum_dir, 'launch', 'aurora930_launch.py')))

    qrcode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(qrcode_launch_path))

    vlm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vlm_launch_path))
    delayed_vlm_pose_node = TimerAction(period=5.0, actions=[vlm_launch])

    # ─── Scan deskew (your custom node from nav_pkg) ────────────────────
    scan_deskew_node = launch_ros.actions.Node(
        package='nav_pkg',
        executable='scan_deskew_pc',
        name='scan_deskew',
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_deskewed',
            'output_cloud_topic': '/scan_deskewed_pc',
            'laser_offset_x': 0.08,
            'laser_offset_y': 0.0,
            'stamp_is_last': True,
            'publish_cloud': True,
        }],
    )

    # ─── Keepout filter (mask + filter-info + lifecycle) ────────────────
    keepout_filter_info_cmd = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'type': 0,                              # 0 = keepout filter
            'filter_info_topic': '/keepout_costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )

    keepout_mask_path = os.path.join(nav_pkg_dir, 'maps', 'mask.yaml')
    keepout_mask_cmd = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_mask_publisher',
        output='screen',
        parameters=[{'yaml_filename': keepout_mask_path,
                     'publish_period_map': 0.0}],
        remappings=[('map', '/keepout_filter_mask')],
    )

    keepout_lifecycle_mgr = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'costmap_filter_info_server',
                'keepout_mask_publisher',
            ],
        }],
    )

    map_server_node = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav_params_file, {'yaml_filename': map_file}],
    )

    # ─── nav2 bringup (AMCL + planner + controller + BT + vel smoother) ──
    navigation_cmd_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav_params_file,
            'map': map_file,
        }.items(),
    )

    unified_lifecycle_mgr = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                '/map_server',
                '/global_costmap/global_costmap',
                '/local_costmap/local_costmap',
            ],
            'bond_timeout': 4.0,
            'attempt_respawn_reconnection': True,
        }],
    )

    # ─── Initial-pose / re-localization (opencv_scan_matcher) ────────────
    reloc_node = launch_ros.actions.Node(
        package='opencv_scan_matcher',
        executable='reloc',
        name='reloc_node',
        output='screen',
        parameters=[nav_params_file],
    )

    # ─── ★ Optional dummy obstacle publisher (only for testing) ─────────
    # When enable_dummy_obstacle:=true and YOLO isn't ready yet, this node
    # publishes a cycling PoseArray to /obstacle_info so the avoidance state
    # machine has data to react to.
    dummy_obstacle = launch_ros.actions.Node(
        package='visual_avoidance_controller',
        executable='dummy_obstacle_publisher.py',
        name='dummy_obstacle_publisher',
        output='screen',
        condition=IfCondition(enable_dummy_obstacle),
        parameters=[{
            'x': 0.0,
            'y': 0.7,
            'cycle_period': 5.0,
            'rate_hz': 5.0,
            'frame_id': 'base_link',
        }],
    )

    # ─── ★ Optional YOLO/BPU bridge (competition default) ───────────────
    # When enable_yolo_bridge:=true, include the obstacle_info_bridge launch
    # which starts dnn_node (BPU YOLO) + the new bridge node. This populates
    # /obstacle_info from real Aurora 930 RGB + depth.
    yolo_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('obstacle_info_bridge'),
                'launch', 'bridge.launch.py')),
        condition=IfCondition(enable_yolo_bridge),
        launch_arguments={
            'dnn_workconfig_file': yolo_workconfig_file,
        }.items(),
    )

    return LaunchDescription([
        # ── args ──
        declare_use_visual,
        declare_enable_dummy,
        declare_enable_yolo,
        declare_nav_params,
        declare_yolo_workconfig,

        # ── sensor drivers ──
        lidar_launch,
        base_launch,
        deptrum_launch,
        qrcode_launch,
        # delayed_vlm_pose_node,
        scan_deskew_node,

        # ── keepout filter ──
        keepout_filter_info_cmd,
        keepout_mask_cmd,
        keepout_lifecycle_mgr,

        map_server_node,

        # ── nav2 ──
        navigation_cmd_amcl,
        unified_lifecycle_mgr,
        reloc_node,

        # ── optional obstacle sources (off by default) ──
        dummy_obstacle,
        yolo_bridge,
    ])
