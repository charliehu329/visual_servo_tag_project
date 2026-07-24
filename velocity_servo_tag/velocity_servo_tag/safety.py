#!/usr/bin/env python3
"""
safety.py

功能：
    提供 Franka 控制中可复用的安全处理函数。
    主要用于对关节速度和末端速度进行检查与限幅。

接口：
    check_finite_vector(vector, name)
    limit_joint_velocity(q_dot, max_abs)
    limit_cartesian_velocity(V_e, max_linear, max_angular)

输入：
    q_dot:
        7维关节速度 [dq1, dq2, dq3, dq4, dq5, dq6, dq7]，单位 rad/s。

    V_e:
        6维末端速度 [vx, vy, vz, wx, wy, wz]。
        vx, vy, vz 单位 m/s。
        wx, wy, wz 单位 rad/s。

    max_abs:
        单个关节最大速度绝对值，单位 rad/s。

    max_linear:
        末端最大线速度绝对值，单位 m/s。

    max_angular:
        末端最大角速度绝对值，单位 rad/s。

输出：
    limit_joint_velocity:
        返回限幅后的 q_dot_safe。

    limit_cartesian_velocity:
        返回限幅后的 V_e_safe。

方法：
    关节速度使用同比例缩放限幅，保持 7 维速度方向。
    笛卡尔速度使用 np.clip 逐元素限幅。
"""

import numpy as np


def check_finite_vector(vector, name):
    """
    检查向量中是否存在 nan 或 inf。
    """

    vector = np.asarray(vector, dtype=float)

    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} contains nan or inf.")

    return vector


def limit_joint_velocity(q_dot, max_abs):
    """
    对 7 维关节速度同比例缩放限幅。
    """

    q_dot = np.asarray(q_dot, dtype=float).reshape(-1)

    if q_dot.shape[0] != 7:
        raise ValueError(
            f"q_dot must have 7 elements, but got {q_dot.shape[0]}."
        )

    check_finite_vector(q_dot, "q_dot")

    max_abs = np.asarray(max_abs, dtype=float)

    if max_abs.ndim == 0:
        max_abs = np.full(7, float(max_abs))
    else:
        max_abs = max_abs.reshape(-1)

    if (
        max_abs.shape != (7,)
        or not np.all(np.isfinite(max_abs))
        or np.any(max_abs <= 0.0)
    ):
        raise ValueError(
            "max_abs must be a positive scalar or 7 positive values."
        )

    # 同比例缩放整组关节速度，避免逐关节裁剪改变末端运动方向。
    ratios = np.abs(q_dot) / max_abs
    scale = max(1.0, float(np.max(ratios)))
    q_dot_safe = q_dot / scale

    return q_dot_safe


def limit_cartesian_velocity(V_e, max_linear, max_angular):
    """
    对 6 维末端速度进行限幅。
    """

    V_e = np.asarray(V_e, dtype=float).reshape(-1)

    if V_e.shape[0] != 6:
        raise ValueError(
            f"V_e must have 6 elements, but got {V_e.shape[0]}."
        )

    check_finite_vector(V_e, "V_e")

    if max_linear <= 0.0:
        raise ValueError("max_linear must be positive.")

    if max_angular <= 0.0:
        raise ValueError("max_angular must be positive.")

    V_e_safe = V_e.copy()

    # 前三个是线速度 [vx, vy, vz]
    V_e_safe[0:3] = np.clip(V_e_safe[0:3], -max_linear, max_linear)

    # 后三个是角速度 [wx, wy, wz]
    V_e_safe[3:6] = np.clip(V_e_safe[3:6], -max_angular, max_angular)

    return V_e_safe
