#!/usr/bin/env python3
"""
vision_double.launch.py

功能：
    使用统一YAML启动Stage 1双目AprilTag视觉节点。

输入：
    params_file：ROS 2参数文件路径。

输出：
    /vision_double/stereo_features
        velocity_servo_tag_interfaces/msg/StereoFeatures

调用：
    ros2 launch velocity_servo_tag vision_double.launch.py

方法：
    从已安装包的config目录查找默认YAML，并启动vision_double_node。
    焦距反馈由独立节点发布，本Launch不发布Zoom占位消息。
"""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """创建双目视觉节点的LaunchDescription。"""

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

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description=(
            "Unified velocity_servo_tag YAML file."
        ),
    )

    vision_double_node = Node(
        package="velocity_servo_tag",
        executable="vision_double_node",
        name="vision_double_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            declare_params_file,
            vision_double_node,
        ]
    )
