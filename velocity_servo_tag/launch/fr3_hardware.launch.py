#!/usr/bin/env python3
"""
fr3_hardware.launch.py

功能：
    1. 启动Franka FR3真实硬件驱动。
    2. 启动controller_manager。
    3. 加载Franka状态广播器。
    4. 加载关节速度控制器。
    5. 启动本包velocity_command_node。

默认安全行为：
    command_mode:=zero

此时即使硬件已连接，也只向机器人发送七维零速度。
"""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # =========================================================
    # 本包文件路径
    # =========================================================

    package_share = get_package_share_directory(
        "velocity_servo_tag"
    )

    controllers_yaml = os.path.join(
        package_share,
        "config",
        "controllers.yaml",
    )

    default_params_file = os.path.join(
        package_share,
        "config",
        "velocity_servo_tag.yaml",
    )

    franka_bringup_share = (
        get_package_share_directory(
            "franka_bringup"
        )
    )

    franka_launch_file = os.path.join(
        franka_bringup_share,
        "launch",
        "franka.launch.py",
    )

    # =========================================================
    # Launch参数
    # =========================================================

    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_rviz = LaunchConfiguration("use_rviz")
    command_mode = LaunchConfiguration("command_mode")
    max_velocity_scale = LaunchConfiguration(
        "max_velocity_scale"
    )
    params_file = LaunchConfiguration("params_file")

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

    declare_command_mode = DeclareLaunchArgument(
        "command_mode",
        default_value="zero",
        description=(
            "Joint command mode: zero or topic. "
            "Default is zero for safety."
        ),
    )

    declare_max_velocity_scale = DeclareLaunchArgument(
        "max_velocity_scale",
        default_value="0.50",
        description=(
            "Final joint velocity limit as a fraction "
            "of FR3 hardware limits."
        ),
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description=(
            "Unified velocity_servo_tag parameter file."
        ),
    )

    # =========================================================
    # Franka硬件启动
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
            "controllers_yaml":
                controllers_yaml,
        }.items(),
    )

    # =========================================================
    # Controller spawner
    # =========================================================

    # joint_state_broadcaster由franka.launch.py启动，
    # 这里不能重复启动。

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


    delayed_velocity_controller = TimerAction(
        period=5.0,
        actions=[
            velocity_controller_spawner
        ],
    )

    # =========================================================
    # 速度命令桥接节点
    # =========================================================

    velocity_command_node = Node(
        package="velocity_servo_tag",
        executable="velocity_command_node",
        name="velocity_command_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            params_file,
            {
                "mode": ParameterValue(
                    command_mode,
                    value_type=str,
                ),
                "max_velocity_scale": ParameterValue(
                    max_velocity_scale,
                    value_type=float,
                ),
            },
        ],
    )

    # 等待底层控制器启动后，再启动命令节点。
    delayed_velocity_command_node = TimerAction(
        period=7.0,
        actions=[
            velocity_command_node
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
            declare_command_mode,
            declare_max_velocity_scale,
            declare_params_file,

            franka_bringup,
            delayed_velocity_controller,
            delayed_velocity_command_node,
            rviz_node,
        ]
    )
