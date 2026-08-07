#!/usr/bin/env python3
"""
detector_handware.launch.py

q_dot 直出版本总启动入口。

启动关系：

    apriltag_detector.launch.py
        ↓
    AprilTag detector
        ↓
    /apriltag_detector/target_position
        ↓
    Simulink
        ↓
    /simulink/target_joints_velocities
        ↓
    hardware.launch.py
        ↓
    joint_velocity_example_controller
        ↓
    Franka FR3

说明：
    1. 本 launch 不启动 Simulink。
    2. 本 launch 不启动 velocity_mapper_node。
    3. 本 launch 不启动 velocity_command_node。
    4. Simulink 直接输出 7 维 q_dot 到底层关节速度控制器。

默认安全启动：

ros2 launch velocity_servo_tag detector_handware.launch.py \
    start_hardware:=false \
    start_detector:=true

此时：
    - 启动 AprilTag detector
    - 不连接 Franka 实机

实机启动：

ros2 launch velocity_servo_tag detector_handware.launch.py \
    start_hardware:=true \
    start_detector:=true \
    robot_ip:=172.16.0.2 \
    use_rviz:=false
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
    # Launch 参数
    # =========================================================

    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_rviz = LaunchConfiguration("use_rviz")

    start_hardware = LaunchConfiguration("start_hardware")
    start_detector = LaunchConfiguration("start_detector")

    params_file = LaunchConfiguration("params_file")

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
        description="Start the real Franka hardware stack.",
    )

    declare_start_detector = DeclareLaunchArgument(
        "start_detector",
        default_value="true",
        description="Start the USB AprilTag detector.",
    )

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
        description="AprilTag detector parameter YAML file.",
    )

    # =========================================================
    # AprilTag detector
    # =========================================================

    detector_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("velocity_servo_tag"),
            "launch",
            "apriltag_detector.launch.py",
        ]
    )

    detector_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            detector_launch_file
        ),
        condition=IfCondition(start_detector),
        launch_arguments={
            "params_file": params_file,
        }.items(),
    )

    # =========================================================
    # Franka 底层硬件
    # =========================================================

    hardware_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("velocity_servo_tag"),
            "launch",
            "hardware.launch.py",
        ]
    )

    hardware_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            hardware_launch_file
        ),
        condition=IfCondition(start_hardware),
        launch_arguments={
            "robot_ip": robot_ip,
            "load_gripper": load_gripper,
            "use_rviz": use_rviz,
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
            declare_params_file,

            detector_system,
            hardware_system,
        ]
    )
