#!/usr/bin/env python3
"""
极简视觉速度映射节点。

功能：
1. 读取FR3当前7维关节角。
2. 接收相机坐标系速度 V_c=[vx,vy,vz,wx,wy,wz]。
3. 根据手眼矩阵将V_c转换为末端速度V_e。
4. 计算Jacobian并用伪逆得到任务关节速度。
5. 当关节进入自身范围外侧40%时，在Jacobian零空间内推动关节回中。
6. 发布7维关节速度给velocity_command_node。

说明：
- 中间60%的关节范围不干预。
- 不做速度限幅、加速度限幅、超时检查或平滑处理。
- 最终安全限制仍由velocity_command_node完成。
"""

import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from velocity_servo_tag.robot_kinematics import FrankaKinematics


JOINT_CENTER_START_RATIO = 0.60
JOINT_CENTER_GAIN = 0.20


class VelocityMapperNode(Node):

    def __init__(self):
        super().__init__("velocity_mapper_node")

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("end_effector_frame", "fr3_hand_tcp")
        self.declare_parameter(
            "T_end_effector_camera",
            [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        )
        self.declare_parameter("dry_run", False)
        self.declare_parameter(
            "joint_state_topic",
            "/franka/joint_states",
        )
        self.declare_parameter(
            "visual_velocity_topic",
            "/simulink/camera_velocity",
        )
        self.declare_parameter(
            "command_topic",
            "/velocity_mapper_node/target_joints_velocities",
        )

        urdf_path = str(
            self.get_parameter("urdf_path").value
        )

        if not urdf_path:
            urdf_path = os.path.join(
                get_package_share_directory("velocity_servo_tag"),
                "config",
                "urdf",
                "fr3.urdf",
            )

        end_effector_frame = str(
            self.get_parameter("end_effector_frame").value
        )

        self.T_e_c = np.asarray(
            self.get_parameter(
                "T_end_effector_camera"
            ).value,
            dtype=float,
        ).reshape(4, 4)

        self.dry_run = bool(
            self.get_parameter("dry_run").value
        )

        self.q_min, self.q_max = self.read_joint_limits(
            urdf_path
        )
        self.q_mid = 0.5 * (self.q_min + self.q_max)
        self.q_half_range = 0.5 * (
            self.q_max - self.q_min
        )

        self.kinematics = FrankaKinematics(
            urdf_path=urdf_path,
            end_effector_frame=end_effector_frame,
        )

        self.q = None

        self.create_subscription(
            JointState,
            str(
                self.get_parameter(
                    "joint_state_topic"
                ).value
            ),
            self.joint_state_callback,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            str(
                self.get_parameter(
                    "visual_velocity_topic"
                ).value
            ),
            self.visual_velocity_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            str(
                self.get_parameter(
                    "command_topic"
                ).value
            ),
            10,
        )

    @staticmethod
    def read_joint_limits(urdf_path):
        root = ET.parse(urdf_path).getroot()

        q_min = []
        q_max = []

        for joint_index in range(1, 8):
            joint = root.find(
                f".//joint[@name='fr3_joint{joint_index}']"
            )
            limit = joint.find("limit")

            q_min.append(float(limit.attrib["lower"]))
            q_max.append(float(limit.attrib["upper"]))

        return (
            np.asarray(q_min, dtype=float),
            np.asarray(q_max, dtype=float),
        )

    def joint_state_callback(self, msg):
        joint = dict(zip(msg.name, msg.position))

        self.q = np.asarray(
            [
                joint["fr3_joint1"],
                joint["fr3_joint2"],
                joint["fr3_joint3"],
                joint["fr3_joint4"],
                joint["fr3_joint5"],
                joint["fr3_joint6"],
                joint["fr3_joint7"],
            ],
            dtype=float,
        )

    def visual_velocity_callback(self, msg):
        if self.q is None:
            return

        V_c = np.asarray(msg.data, dtype=float)

        R_e_c = self.T_e_c[:3, :3]
        t_e_c = self.T_e_c[:3, 3]

        t_skew = np.asarray(
            [
                [0.0, -t_e_c[2], t_e_c[1]],
                [t_e_c[2], 0.0, -t_e_c[0]],
                [-t_e_c[1], t_e_c[0], 0.0],
            ]
        )

        adjoint_e_c = np.zeros((6, 6))
        adjoint_e_c[:3, :3] = R_e_c
        adjoint_e_c[:3, 3:] = t_skew @ R_e_c
        adjoint_e_c[3:, 3:] = R_e_c

        V_e = adjoint_e_c @ V_c

        J = np.asarray(
            self.kinematics.compute_jacobian(self.q),
            dtype=float,
        )

        J_pinv = np.linalg.pinv(J)

        # 主任务：完成相机速度。
        q_dot_task = J_pinv @ V_e

        # 归一化关节位置：
        # 0表示关节中心，±1表示上下限。
        q_normalized = (
            (self.q - self.q_mid)
            / self.q_half_range
        )

        # 中间60%不干预，进入外侧40%后逐渐增强。
        activation = np.clip(
            (
                np.abs(q_normalized)
                - JOINT_CENTER_START_RATIO
            )
            / (1.0 - JOINT_CENTER_START_RATIO),
            0.0,
            1.0,
        )

        q_dot_center = (
            -JOINT_CENTER_GAIN
            * activation**2
            * np.clip(q_normalized, -1.0, 1.0)
        )

        # 零空间投影：尽量不影响相机速度主任务。
        null_projector = (
            np.eye(7)
            - J_pinv @ J
        )

        q_dot = (
            q_dot_task
            + null_projector @ q_dot_center
        )

        if not self.dry_run:
            msg_out = Float64MultiArray()
            msg_out.data = q_dot.tolist()
            self.publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()