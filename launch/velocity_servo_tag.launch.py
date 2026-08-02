#!/usr/bin/env python3
"""
velocity_servo_tag.launch.py

启动视觉伺服上层节点：

    1. apriltag_detector
    2. velocity_mapper_node

本文件不启动：

    1. Franka真实硬件
    2. controller_manager
    3. velocity_command_node

这些底层组件由fr3_hardware.launch.py负责启动。
"""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # =========================================================
    # 默认参数文件
    # =========================================================

    package_share = get_package_share_directory(
        "velocity_servo_tag"
    )

    default_params_file = os.path.join(
        package_share,
        "config",
        "velocity_servo_tag.yaml",
    )

    # =========================================================
    # Launch参数
    # =========================================================

    params_file = LaunchConfiguration(
        "params_file"
    )

    dry_run = LaunchConfiguration(
        "dry_run"
    )

    start_detector = LaunchConfiguration(
        "start_detector"
    )

    start_mapper = LaunchConfiguration(
        "start_mapper"
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description=(
            "Unified velocity_servo_tag YAML file."
        ),
    )

    declare_dry_run = DeclareLaunchArgument(
        "dry_run",
        default_value="true",
        description=(
            "Suppress nonzero joint velocity commands when true; "
            "safety zero targets are still published."
        ),
    )

    declare_start_detector = DeclareLaunchArgument(
        "start_detector",
        default_value="true",
        description=(
            "Start the USB AprilTag detector."
        ),
    )

    declare_start_mapper = DeclareLaunchArgument(
        "start_mapper",
        default_value="true",
        description=(
            "Start the Cartesian-to-joint "
            "velocity mapper."
        ),
    )

    # =========================================================
    # AprilTag检测节点
    # =========================================================

    detector_node = Node(
        package="velocity_servo_tag",
        executable="apriltag_detector",
        name="apriltag_detector",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(
            start_detector
        ),
        parameters=[
            params_file
        ],
    )

    # =========================================================
    # 相机速度到关节速度映射节点
    # =========================================================

    velocity_mapper_node = Node(
        package="velocity_servo_tag",
        executable="velocity_mapper_node",
        name="velocity_mapper_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(
            start_mapper
        ),
        parameters=[
            params_file,
            {
                "dry_run": ParameterValue(
                    dry_run,
                    value_type=bool,
                )
            },
        ],
    )

    # =========================================================
    # 返回LaunchDescription
    # =========================================================

    return LaunchDescription(
        [
            declare_params_file,
            declare_dry_run,
            declare_start_detector,
            declare_start_mapper,

            detector_node,
            velocity_mapper_node,
        ]
    )
