#!/usr/bin/env python3
"""
apriltag_detector.launch.py

独立启动 AprilTag USB 相机检测节点。

默认使用 velocity_servo_tag/config/velocity_servo_tag.yaml。

启动：
    ros2 launch velocity_servo_tag apriltag_detector.launch.py

指定参数文件：
    ros2 launch velocity_servo_tag apriltag_detector.launch.py \
        params_file:=/path/to/velocity_servo_tag.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =========================================================
    # Launch 参数
    # =========================================================

    params_file = LaunchConfiguration("params_file")

    # =========================================================
    # 默认参数文件
    # =========================================================

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("velocity_servo_tag"),
            "config",
            "velocity_servo_tag.yaml",
        ]
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="AprilTag detector ROS 2 parameter YAML file.",
    )

    # =========================================================
    # AprilTag detector
    #
    # 使用 ROS 2 标准 Node action 启动 setup.py 注册的可执行程序。
    # =========================================================

    apriltag_detector = Node(
        package="velocity_servo_tag",
        executable="apriltag_detector",
        name="apriltag_detector",
        parameters=[params_file],
        output="screen",
    )

    # =========================================================
    # 返回
    # =========================================================

    return LaunchDescription(
        [
            declare_params_file,
            apriltag_detector,
        ]
    )
