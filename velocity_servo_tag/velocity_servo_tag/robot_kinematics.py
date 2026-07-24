#!/usr/bin/env python3
"""
robot_kinematics.py

功能：
    根据当前关节角 q 计算 Franka FR3 机械臂的雅可比矩阵 J(q)。

接口：
    FrankaKinematics(urdf_path, end_effector_frame="fr3_hand_tcp")
    compute_jacobian(q)

输入：
    urdf_path: FR3 的 URDF 文件路径，例如 /tmp/fr3.urdf

    end_effector_frame: 末端坐标系名称
        默认使用 "fr3_hand_tcp"

    q: 7维关节角 [q1, q2, q3, q4, q5, q6, q7]
       shape = (7,)

输出：
    J: 当前雅可比矩阵
       shape = (6, 7)
"""

import numpy as np
import pinocchio as pin


class FrankaKinematics:
    def __init__(self, urdf_path, end_effector_frame="fr3_hand_tcp"):
        self.urdf_path = urdf_path
        self.end_effector_frame = end_effector_frame

        # 读取urdf模型
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        if not self.model.existFrame(self.end_effector_frame):
            raise ValueError(
                f"Frame '{self.end_effector_frame}' not found in URDF."
            )

        self.frame_id = self.model.getFrameId(self.end_effector_frame)

        self.arm_joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        self.arm_q_indices = []
        self.arm_v_indices = []

        for joint_name in self.arm_joint_names:
            if not self.model.existJointName(joint_name):
                raise ValueError(f"Joint '{joint_name}' not found in URDF.")

            joint_id = self.model.getJointId(joint_name)
            joint_model = self.model.joints[joint_id]

            self.arm_q_indices.append(joint_model.idx_q)
            self.arm_v_indices.append(joint_model.idx_v)

    def compute_jacobian(self, q):
        """
        Compute 6x7 Jacobian of FR3 end-effector.

        Parameters
        ----------
        q : array-like, shape (7,)
            Current arm joint positions.

        Returns
        -------
        J : np.ndarray, shape (6, 7)
            End-effector Jacobian.
        """

        q = np.asarray(q, dtype=float).reshape(7)

        q_full = pin.neutral(self.model)

        for i, idx_q in enumerate(self.arm_q_indices):
            q_full[idx_q] = q[i]

        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        J_full = pin.computeFrameJacobian(
            self.model,
            self.data,
            q_full,
            self.frame_id,
            pin.ReferenceFrame.LOCAL,
        )

        J_arm = J_full[:, self.arm_v_indices]

        return J_arm