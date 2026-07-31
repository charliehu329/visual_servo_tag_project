#!/usr/bin/env python3

"""
极简单目视觉速度映射节点。

功能：
1. 读取FR3当前7维关节角。
2. 接收相机坐标系速度：
   V_c = [vx, vy, vz, wx, wy, wz]。
3. 根据手眼变换T_end_effector_camera，
   将相机速度转换为末端执行器速度。
4. 根据当前关节角计算末端Jacobian。
5. 使用Jacobian伪逆计算7维关节速度：
   q_dot = pinv(J) @ V_e。
6. 将q_dot发布给velocity_command_node。

说明：
- 本节点只做坐标变换和速度映射。
- 不做速度限幅、加速度限幅、超时检查或平滑处理。
- 最终安全限制由velocity_command_node完成。
- dry_run=true时只计算，不发布关节速度。
"""

import os
import numpy as np
import rclpy

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from velocity_servo_tag.robot_kinematics import FrankaKinematics


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

        self.kinematics = FrankaKinematics(
            urdf_path=urdf_path,
            end_effector_frame=end_effector_frame,
        )

        self.q = None

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.joint_state_callback,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("visual_velocity_topic").value),
            self.visual_velocity_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("command_topic").value),
            10,
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
        J = self.kinematics.compute_jacobian(self.q)
        q_dot = np.linalg.pinv(J) @ V_e

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