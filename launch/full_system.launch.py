#!/usr/bin/env python3
"""
full_system.launch.py

启动指令：
ros2 launch velocity_servo_tag full_system.launch.py \
  start_hardware:=false \
  dry_run:=true \
  command_mode:=zero

完整系统启动入口：

AprilTag detector
    → Simulink
    → velocity_mapper_node
    → velocity_command_node
    → 底层关节速度控制器
    → Franka硬件

默认安全设置：

    start_hardware:=false
    dry_run:=true
    command_mode:=zero

实机控制必须同时显式设置：

    start_hardware:=true
    dry_run:=false
    command_mode:=topic
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =========================================================
    # Launch参数
    # =========================================================

    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_rviz = LaunchConfiguration("use_rviz")

    start_hardware = LaunchConfiguration(
        "start_hardware"
    )
    start_detector = LaunchConfiguration(
        "start_detector"
    )

    dry_run = LaunchConfiguration("dry_run")
    command_mode = LaunchConfiguration(
        "command_mode"
    )

    max_velocity_scale = LaunchConfiguration(
        "max_velocity_scale"
    )
    params_file = LaunchConfiguration(
        "params_file"
    )

    # =========================================================
    # 参数声明
    # =========================================================

    declare_robot_ip = DeclareLaunchArgument(
        "robot_ip",
        default_value="172.16.0.2",
        description="Franka FR3 IP address.",
    )

    declare_load_gripper = DeclareLaunchArgument(
        "load_gripper",
        default_value="false",
        description="Load Franka Hand gripper.",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Start RViz2.",
    )

    declare_start_hardware = DeclareLaunchArgument(
        "start_hardware",
        default_value="false",
        description=(
            "Start the real Franka hardware stack."
        ),
    )

    declare_start_detector = DeclareLaunchArgument(
        "start_detector",
        default_value="true",
        description=(
            "Start the USB AprilTag detector."
        ),
    )

    declare_dry_run = DeclareLaunchArgument(
        "dry_run",
        default_value="true",
        description=(
            "Suppress nonzero mapper joint commands when true; "
            "safety zero targets are still published."
        ),
    )

    declare_command_mode = DeclareLaunchArgument(
        "command_mode",
        default_value="zero",
        description=(
            "Bottom command mode: zero or topic."
        ),
    )

    declare_max_velocity_scale = (
        DeclareLaunchArgument(
            "max_velocity_scale",
            default_value="0.10",
            description=(
                "Final velocity limit as a fraction "
                "of FR3 hardware limits."
            ),
        )
    )

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare(
                "velocity_servo_tag"
            ),
            "config",
            "velocity_servo_tag.yaml",
        ]
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description=(
            "Unified velocity_servo_tag YAML file."
        ),
    )

    # =========================================================
    # Franka硬件子系统
    # =========================================================

    hardware_launch_file = PathJoinSubstitution(
        [
            FindPackageShare(
                "velocity_servo_tag"
            ),
            "launch",
            "fr3_hardware.launch.py",
        ]
    )

    hardware_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            hardware_launch_file
        ),
        condition=IfCondition(
            start_hardware
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "load_gripper": load_gripper,
            "use_rviz": use_rviz,
            "command_mode": command_mode,
            "max_velocity_scale":
                max_velocity_scale,
            "params_file": params_file,
        }.items(),
    )

    # =========================================================
    # AprilTag和速度映射子系统
    # =========================================================

    servo_launch_file = PathJoinSubstitution(
        [
            FindPackageShare(
                "velocity_servo_tag"
            ),
            "launch",
            "velocity_servo_tag.launch.py",
        ]
    )

    servo_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            servo_launch_file
        ),
        launch_arguments={
            "params_file": params_file,
            "dry_run": dry_run,
            "start_detector": start_detector,
        }.items(),
    )

    # =========================================================
    # 返回完整系统
    # =========================================================

    return LaunchDescription(
        [
            declare_robot_ip,
            declare_load_gripper,
            declare_use_rviz,
            declare_start_hardware,
            declare_start_detector,
            declare_dry_run,
            declare_command_mode,
            declare_max_velocity_scale,
            declare_params_file,

            hardware_system,
            servo_system,
        ]
    )
