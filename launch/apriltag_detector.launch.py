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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    # 使用 python3 -m 启动，而不是 launch_ros.actions.Node，
    # 这样不依赖 setup.py 中是否已经注册 console_scripts。
    # 与当前可直接运行方式保持一致：
    # python3 -m velocity_servo_tag.vision.apriltag_detector
    # =========================================================

    apriltag_detector = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "velocity_servo_tag.vision.apriltag_detector",
            "--ros-args",
            "--params-file",
            params_file,
        ],
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
