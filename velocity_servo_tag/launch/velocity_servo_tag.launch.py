#!/usr/bin/env python3
"""
velocity_servo_tag.launch.py

功能：
    启动Stage 1双目AprilTag视觉节点。

输入：
    params_file：统一ROS 2 YAML参数文件。
    start_vision：是否启动vision_double_node。

输出：
    /vision_double/stereo_features
        velocity_servo_tag_interfaces/msg/StereoFeatures

调用：
    ros2 launch velocity_servo_tag velocity_servo_tag.launch.py

方法：
    使用YAML配置左右USB相机和AprilTag检测参数。本Launch不启动
    MATLAB/Simulink、Franka硬件、焦距反馈节点或
    velocity_command_node。
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


def generate_launch_description():
    """创建Stage 1双目视觉LaunchDescription。"""

    package_share = get_package_share_directory(
        "velocity_servo_tag"
    )
    default_params_file = os.path.join(
        package_share,
        "config",
        "velocity_servo_tag.yaml",
    )

    params_file = LaunchConfiguration(
        "params_file"
    )
    start_vision = LaunchConfiguration(
        "start_vision"
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description=(
            "Unified velocity_servo_tag YAML file."
        ),
    )
    declare_start_vision = DeclareLaunchArgument(
        "start_vision",
        default_value="true",
        description=(
            "Start the dual-camera AprilTag node."
        ),
    )

    vision_double_node = Node(
        package="velocity_servo_tag",
        executable="vision_double_node",
        name="vision_double_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_vision),
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            declare_params_file,
            declare_start_vision,
            vision_double_node,
        ]
    )
