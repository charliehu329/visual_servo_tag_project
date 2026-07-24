#!/usr/bin/env python3
"""
full_system.launch.py

功能：
    组合Stage 1双目视觉节点、可选Franka硬件、底层关节速度控制器
    和velocity_command_node，作为当前系统的统一ROS 2启动入口。

输入：
    start_vision：是否启动vision_double_node。
    start_hardware：是否启动真实FR3硬件和velocity_command_node。
    command_mode：velocity_command_node使用zero或topic模式。
    robot_ip、load_gripper、use_rviz、max_velocity_scale、params_file。

输出：
    视觉Topic、FR3状态Topic和底层关节速度控制器命令。

调用：
    安全通信测试：
    ros2 launch velocity_servo_tag full_system.launch.py \
      start_hardware:=false start_vision:=true command_mode:=zero

方法：
    包含velocity_servo_tag.launch.py和fr3_hardware.launch.py。
    默认不启动真实硬件，并保持zero模式。MATLAB/Simulink仍需单独运行。
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
    """创建Stage 1完整ROS 2系统LaunchDescription。"""

    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_rviz = LaunchConfiguration("use_rviz")
    start_hardware = LaunchConfiguration(
        "start_hardware"
    )
    start_vision = LaunchConfiguration(
        "start_vision"
    )
    command_mode = LaunchConfiguration(
        "command_mode"
    )
    max_velocity_scale = LaunchConfiguration(
        "max_velocity_scale"
    )
    params_file = LaunchConfiguration(
        "params_file"
    )

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
    declare_start_vision = DeclareLaunchArgument(
        "start_vision",
        default_value="true",
        description=(
            "Start the dual-camera AprilTag node."
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
            default_value="0.50",
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
        condition=IfCondition(start_hardware),
        launch_arguments={
            "robot_ip": robot_ip,
            "load_gripper": load_gripper,
            "use_rviz": use_rviz,
            "command_mode": command_mode,
            "max_velocity_scale": (
                max_velocity_scale
            ),
            "params_file": params_file,
        }.items(),
    )

    vision_launch_file = PathJoinSubstitution(
        [
            FindPackageShare(
                "velocity_servo_tag"
            ),
            "launch",
            "velocity_servo_tag.launch.py",
        ]
    )
    vision_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            vision_launch_file
        ),
        launch_arguments={
            "params_file": params_file,
            "start_vision": start_vision,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_robot_ip,
            declare_load_gripper,
            declare_use_rviz,
            declare_start_hardware,
            declare_start_vision,
            declare_command_mode,
            declare_max_velocity_scale,
            declare_params_file,
            hardware_system,
            vision_system,
        ]
    )
