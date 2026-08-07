#!/usr/bin/env python3
"""
hardware.launch.py

q_dot 直出版本的 Franka FR3 底层启动文件。

只负责：

    Franka FR3 hardware
    → controller_manager
    → joint_state_broadcaster
    → franka_robot_state_broadcaster
    → joint_velocity_example_controller

不启动：

    velocity_mapper_node
    velocity_command_node
    AprilTag detector
    Simulink

Simulink 直接向 joint_velocity_example_controller 的速度命令 topic
发布 7 维 q_dot。

启动示例：

ros2 launch velocity_servo_tag hardware.launch.py \
    robot_ip:=172.16.0.2 \
    use_rviz:=false
"""

import os

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
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # 路径
    # =========================================================

    package_share = get_package_share_directory(
        "velocity_servo_tag"
    )

    controllers_yaml = os.path.join(
        package_share,
        "config",
        "controllers.yaml",
    )

    franka_bringup_share = get_package_share_directory(
        "franka_bringup"
    )

    franka_launch_file = os.path.join(
        franka_bringup_share,
        "launch",
        "franka.launch.py",
    )

    # =========================================================
    # Launch 参数
    # =========================================================

    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_rviz = LaunchConfiguration("use_rviz")

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

    # =========================================================
    # Franka 实机硬件
    # =========================================================

    franka_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            franka_launch_file
        ),
        launch_arguments={
            "robot_type": "fr3",
            "robot_ip": robot_ip,
            "load_gripper": load_gripper,
            "use_fake_hardware": "false",
            "controllers_yaml": controllers_yaml,
        }.items(),
    )

    # =========================================================
    # 关节速度控制器
    # =========================================================
    #
    # joint_state_broadcaster 和
    # franka_robot_state_broadcaster
    # 由 franka.launch.py 按 controllers.yaml 启动。
    #
    # 这里只额外启动我们需要的关节速度控制器。
    # =========================================================

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="velocity_controller_spawner",
        arguments=[
            "joint_velocity_example_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    # 等待 Franka hardware 和 controller_manager 初始化。
    delayed_velocity_controller = TimerAction(
        period=5.0,
        actions=[
            velocity_controller_spawner,
        ],
    )

    # =========================================================
    # RViz
    # =========================================================

    rviz_config = os.path.join(
        get_package_share_directory(
            "franka_description"
        ),
        "rviz",
        "visualize_franka.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "--display-config",
            rviz_config,
            "-f",
            "world",
        ],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # =========================================================
    # LaunchDescription
    # =========================================================

    return LaunchDescription(
        [
            declare_robot_ip,
            declare_load_gripper,
            declare_use_rviz,

            franka_bringup,
            delayed_velocity_controller,
            rviz_node,
        ]
    )
