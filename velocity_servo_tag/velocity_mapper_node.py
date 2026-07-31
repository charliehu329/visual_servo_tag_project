#!/usr/bin/env python3
"""
视觉速度到FR3关节速度的极简映射节点。

功能：
1. 读取FR3当前7维关节角。
2. 接收相机坐标系速度 V_c=[vx,vy,vz,wx,wy,wz]。
3. 根据手眼矩阵将V_c转换为末端坐标系速度V_e。
4. 计算机器人Jacobian。
5. 根据最小奇异值自动选择阻尼，使用阻尼最小二乘伪逆求任务关节速度。
6. 当关节进入自身范围外侧20%时，在Jacobian零空间内推动关节回中。
7. 发布7维关节速度给velocity_command_node。

说明：
- JOINT_CENTER_START_RATIO=0.80：
  中间80%的关节范围不干预，进入外侧20%后逐渐回中。
- 本节点不做最终速度限幅、加速度限幅、命令超时或平滑处理。
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


# ============================================================
# 用户可调参数
# ============================================================

# 0表示只要偏离关节中点就回中；
# 0.80表示中间80%不干预，进入外侧20%后开始回中。
JOINT_CENTER_START_RATIO = 0.80

# 关节回中增益，单位约为rad/s。
JOINT_CENTER_GAIN = 0.03

# 奇异性指标打印周期，单位秒。
SINGULARITY_LOG_PERIOD_SEC = 1.0

# 根据最小奇异值选择阻尼。
SIGMA_MIN_CRITICAL = 0.03
SIGMA_MIN_WARNING = 0.08

DAMPING_CRITICAL = 0.08
DAMPING_WARNING = 0.03
DAMPING_NORMAL = 0.005


class VelocityMapperNode(Node):

    def __init__(self):
        super().__init__("velocity_mapper_node")

        if not 0.0 <= JOINT_CENTER_START_RATIO < 1.0:
            raise ValueError(
                "JOINT_CENTER_START_RATIO必须满足0.0 <= ratio < 1.0"
            )

        self.declare_parameter("urdf_path", "")
        self.declare_parameter(
            "end_effector_frame",
            "fr3_hand_tcp",
        )
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
                get_package_share_directory(
                    "velocity_servo_tag"
                ),
                "config",
                "urdf",
                "fr3.urdf",
            )

        end_effector_frame = str(
            self.get_parameter(
                "end_effector_frame"
            ).value
        )

        transform_values = self.get_parameter(
            "T_end_effector_camera"
        ).value

        if len(transform_values) != 16:
            raise ValueError(
                "T_end_effector_camera必须包含16个数"
            )

        self.T_e_c = np.asarray(
            transform_values,
            dtype=float,
        ).reshape(4, 4)

        if not np.all(np.isfinite(self.T_e_c)):
            raise ValueError(
                "T_end_effector_camera包含NaN或Inf"
            )

        self.dry_run = bool(
            self.get_parameter("dry_run").value
        )

        self.q_min, self.q_max = (
            self.read_joint_limits(urdf_path)
        )

        self.q_mid = (
            0.5 * (self.q_min + self.q_max)
        )

        self.q_half_range = (
            0.5 * (self.q_max - self.q_min)
        )

        if np.any(self.q_half_range <= 0.0):
            raise ValueError(
                "URDF中的关节上下限无效"
            )

        self.kinematics = FrankaKinematics(
            urdf_path=urdf_path,
            end_effector_frame=end_effector_frame,
        )

        self.q = None

        joint_state_topic = str(
            self.get_parameter(
                "joint_state_topic"
            ).value
        )

        visual_velocity_topic = str(
            self.get_parameter(
                "visual_velocity_topic"
            ).value
        )

        command_topic = str(
            self.get_parameter(
                "command_topic"
            ).value
        )

        self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            visual_velocity_topic,
            self.visual_velocity_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            command_topic,
            10,
        )

        self.last_singularity_log_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            "VelocityMapperNode started | "
            f"joint_center_start_ratio="
            f"{JOINT_CENTER_START_RATIO:.2f} | "
            f"joint_center_gain="
            f"{JOINT_CENTER_GAIN:.3f} | "
            f"dry_run={self.dry_run}"
        )

    @staticmethod
    def read_joint_limits(urdf_path):
        root = ET.parse(urdf_path).getroot()

        q_min = []
        q_max = []

        for joint_index in range(1, 8):
            joint_name = (
                f"fr3_joint{joint_index}"
            )

            joint = root.find(
                f".//joint[@name='{joint_name}']"
            )

            if joint is None:
                raise ValueError(
                    f"URDF中未找到{joint_name}"
                )

            limit = joint.find("limit")

            if limit is None:
                raise ValueError(
                    f"{joint_name}没有limit字段"
                )

            if (
                "lower" not in limit.attrib
                or "upper" not in limit.attrib
            ):
                raise ValueError(
                    f"{joint_name}缺少上下限"
                )

            q_min.append(
                float(limit.attrib["lower"])
            )
            q_max.append(
                float(limit.attrib["upper"])
            )

        return (
            np.asarray(q_min, dtype=float),
            np.asarray(q_max, dtype=float),
        )

    def joint_state_callback(self, msg):
        joint = dict(
            zip(msg.name, msg.position)
        )

        required_joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        if not all(
            name in joint
            for name in required_joint_names
        ):
            return

        q = np.asarray(
            [
                joint[name]
                for name in required_joint_names
            ],
            dtype=float,
        )

        if q.shape != (7,):
            return

        if not np.all(np.isfinite(q)):
            return

        self.q = q

    @staticmethod
    def select_damping(sigma_min):
        if sigma_min < SIGMA_MIN_CRITICAL:
            return DAMPING_CRITICAL

        if sigma_min < SIGMA_MIN_WARNING:
            return DAMPING_WARNING

        return DAMPING_NORMAL

    @staticmethod
    def calculate_damped_pseudoinverse(
        jacobian,
        damping,
    ):
        """
        阻尼最小二乘伪逆：

            J# = J^T (J J^T + lambda^2 I)^(-1)

        这里不显式计算逆矩阵，而使用solve提高数值稳定性。
        """
        task_dimension = jacobian.shape[0]

        regularized_matrix = (
            jacobian @ jacobian.T
            + damping**2
            * np.eye(task_dimension)
        )

        return np.linalg.solve(
            regularized_matrix,
            jacobian,
        ).T

    def visual_velocity_callback(self, msg):
        if self.q is None:
            return

        V_c = np.asarray(
            msg.data,
            dtype=float,
        ).reshape(-1)

        if V_c.size != 6:
            self.get_logger().warning(
                "camera_velocity必须包含6个元素，"
                f"当前收到{V_c.size}个"
            )
            return

        if not np.all(np.isfinite(V_c)):
            self.get_logger().warning(
                "camera_velocity包含NaN或Inf，"
                "本次命令已忽略"
            )
            return

        R_e_c = self.T_e_c[:3, :3]
        t_e_c = self.T_e_c[:3, 3]

        t_skew = np.asarray(
            [
                [
                    0.0,
                    -t_e_c[2],
                    t_e_c[1],
                ],
                [
                    t_e_c[2],
                    0.0,
                    -t_e_c[0],
                ],
                [
                    -t_e_c[1],
                    t_e_c[0],
                    0.0,
                ],
            ],
            dtype=float,
        )

        adjoint_e_c = np.zeros(
            (6, 6),
            dtype=float,
        )

        adjoint_e_c[:3, :3] = R_e_c
        adjoint_e_c[:3, 3:] = (
            t_skew @ R_e_c
        )
        adjoint_e_c[3:, 3:] = R_e_c

        V_e = adjoint_e_c @ V_c

        J = np.asarray(
            self.kinematics.compute_jacobian(
                self.q
            ),
            dtype=float,
        )

        if J.shape != (6, 7):
            self.get_logger().error(
                "Jacobian尺寸错误，"
                f"期望(6,7)，实际{J.shape}"
            )
            return

        if not np.all(np.isfinite(J)):
            self.get_logger().error(
                "Jacobian包含NaN或Inf，"
                "本次命令已忽略"
            )
            return

        try:
            singular_values = np.linalg.svd(
                J,
                compute_uv=False,
            )
        except np.linalg.LinAlgError:
            self.get_logger().error(
                "Jacobian奇异值分解失败，"
                "本次命令已忽略"
            )
            return

        sigma_max = float(
            singular_values[0]
        )
        sigma_min = float(
            singular_values[-1]
        )

        condition_number = (
            sigma_max
            / max(sigma_min, 1e-9)
        )

        damping = self.select_damping(
            sigma_min
        )

        current_time = (
            self.get_clock().now()
        )

        elapsed_sec = (
            current_time
            - self.last_singularity_log_time
        ).nanoseconds * 1e-9

        if (
            elapsed_sec
            >= SINGULARITY_LOG_PERIOD_SEC
        ):
            self.get_logger().info(
                f"sigma_min={sigma_min:.6f}, "
                f"condition={condition_number:.2f}, "
                f"damping={damping:.4f}"
            )

            self.last_singularity_log_time = (
                current_time
            )

        try:
            J_pinv = (
                self.calculate_damped_pseudoinverse(
                    J,
                    damping,
                )
            )
        except np.linalg.LinAlgError:
            self.get_logger().error(
                "阻尼伪逆求解失败，"
                "本次命令已忽略"
            )
            return

        # 主任务：完成相机速度。
        q_dot_task = J_pinv @ V_e

        # 归一化关节位置：
        # 0表示关节中点，±1表示上下限。
        q_normalized = (
            (self.q - self.q_mid)
            / self.q_half_range
        )

        # 中间80%不干预，进入外侧20%后逐渐增强。
        activation = np.clip(
            (
                np.abs(q_normalized)
                - JOINT_CENTER_START_RATIO
            )
            / (
                1.0
                - JOINT_CENTER_START_RATIO
            ),
            0.0,
            1.0,
        )

        q_dot_center = (
            -JOINT_CENTER_GAIN
            * activation**2
            * np.clip(
                q_normalized,
                -1.0,
                1.0,
            )
        )

        # 使用当前阻尼伪逆构造广义零空间投影。
        #
        # 注意：
        # 使用阻尼伪逆时，该矩阵不是严格正交投影，
        # 但接近奇异位形时通常比普通pinv更稳定。
        null_projector = (
            np.eye(J.shape[1])
            - J_pinv @ J
        )

        q_dot_center_projected = (
            null_projector
            @ q_dot_center
        )

        q_dot = (
            q_dot_task
            + q_dot_center_projected
        )

        if not np.all(np.isfinite(q_dot)):
            self.get_logger().error(
                "计算得到的关节速度包含NaN或Inf，"
                "本次命令已忽略"
            )
            return

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
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()