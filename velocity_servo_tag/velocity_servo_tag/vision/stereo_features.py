#!/usr/bin/env python3
"""
stereo_features.py

功能：
    提供双目 AprilTag 新帧消息所需的纯算法工具，不访问相机，
    也不依赖 ROS 2。

输入：
    AprilTag 四个像素角点、当前 uint32 帧序号和纳秒时间戳。

输出：
    AprilTag 像素尺度、回绕后的下一帧序号，以及 ROS 2 Time
    使用的秒和纳秒字段。

调用：
    compute_tag_scale(corners)
    next_frame_sequence(current_sequence)
    split_timestamp_ns(timestamp_ns)

方法：
    使用鞋带公式计算四角多边形像素面积，并取面积平方根作为尺度。
    帧序号按 uint32 回绕；整数纳秒时间戳通过 divmod 拆分。
"""

from dataclasses import dataclass
import math

import numpy as np


UINT32_MAX = (1 << 32) - 1
NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class CameraFeature:
    """一侧相机最近处理完成的真实图像结果。"""

    sequence: int = 0
    capture_stamp_ns: int = 0
    valid: bool = False
    u: float = 0.0
    v: float = 0.0
    scale: float = 0.0


def compute_tag_scale(corners):
    """
    根据四个像素角点计算 sqrt(area)，单位为 pixel。
    """

    points = np.asarray(
        corners,
        dtype=float,
    )

    if points.shape != (4, 2):
        raise ValueError(
            "corners must have shape (4, 2)."
        )

    if not np.all(np.isfinite(points)):
        raise ValueError(
            "corners contain nan or inf."
        )

    x_coordinates = points[:, 0]
    y_coordinates = points[:, 1]

    area = 0.5 * abs(
        float(
            np.dot(
                x_coordinates,
                np.roll(y_coordinates, -1),
            )
            - np.dot(
                y_coordinates,
                np.roll(x_coordinates, -1),
            )
        )
    )

    if area <= 0.0:
        raise ValueError(
            "corners describe a zero-area polygon."
        )

    return math.sqrt(area)


def next_frame_sequence(current_sequence):
    """返回按uint32自然回绕的下一帧序号。"""

    if isinstance(current_sequence, bool):
        raise ValueError(
            "current_sequence must be an integer."
        )

    sequence = int(current_sequence)

    if sequence != current_sequence:
        raise ValueError(
            "current_sequence must be an integer."
        )

    if sequence < 0 or sequence > UINT32_MAX:
        raise ValueError(
            "current_sequence is outside uint32 range."
        )

    return (sequence + 1) & UINT32_MAX


def split_timestamp_ns(timestamp_ns):
    """将非负整数纳秒时间戳拆为ROS 2 Time字段。"""

    if isinstance(timestamp_ns, bool):
        raise ValueError(
            "timestamp_ns must be an integer."
        )

    timestamp = int(timestamp_ns)

    if timestamp != timestamp_ns:
        raise ValueError(
            "timestamp_ns must be an integer."
        )

    if timestamp < 0:
        raise ValueError(
            "timestamp_ns must be non-negative."
        )

    seconds, nanoseconds = divmod(
        timestamp,
        NANOSECONDS_PER_SECOND,
    )

    return int(seconds), int(nanoseconds)
