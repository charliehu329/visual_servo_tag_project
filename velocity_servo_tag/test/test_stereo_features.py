#!/usr/bin/env python3
"""
test_stereo_features.py

功能：
    验证双目AprilTag像素尺度、uint32帧序号和纳秒时间戳拆分。

输入：
    人工构造的四角像素坐标、帧序号和纳秒时间戳。

输出：
    unittest测试结果。

调用：
    python3 -m unittest discover -v -s test -p 'test_stereo_features.py'

方法：
    使用已知正方形面积和边界值，对纯算法函数进行无相机、无ROS测试。
"""

import unittest

import numpy as np

from velocity_servo_tag.vision.stereo_features import (
    UINT32_MAX,
    compute_tag_scale,
    next_frame_sequence,
    split_timestamp_ns,
)


class StereoFeaturesTest(unittest.TestCase):
    """双目特征纯算法单元测试。"""

    def test_compute_tag_scale_for_square(self):
        """边长20 pixel的正方形应返回scale=20。"""

        corners = np.asarray(
            [
                [10.0, 20.0],
                [30.0, 20.0],
                [30.0, 40.0],
                [10.0, 40.0],
            ]
        )

        self.assertAlmostEqual(
            compute_tag_scale(corners),
            20.0,
        )

    def test_compute_tag_scale_rejects_zero_area(self):
        """共线角点不能形成有效AprilTag像素面积。"""

        corners = np.asarray(
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [2.0, 0.0],
                [3.0, 0.0],
            ]
        )

        with self.assertRaises(ValueError):
            compute_tag_scale(corners)

    def test_next_frame_sequence_increments(self):
        """普通帧序号应增加1。"""

        self.assertEqual(
            next_frame_sequence(41),
            42,
        )

    def test_next_frame_sequence_wraps_uint32(self):
        """uint32最大值的下一帧应回绕到0。"""

        self.assertEqual(
            next_frame_sequence(UINT32_MAX),
            0,
        )

    def test_next_frame_sequence_rejects_invalid_range(
        self,
    ):
        """负数和超过uint32范围的序号必须被拒绝。"""

        with self.assertRaises(ValueError):
            next_frame_sequence(-1)

        with self.assertRaises(ValueError):
            next_frame_sequence(UINT32_MAX + 1)

    def test_split_timestamp_ns(self):
        """整数纳秒时间戳应正确拆分为秒和纳秒。"""

        self.assertEqual(
            split_timestamp_ns(12_345_678_901),
            (12, 345_678_901),
        )

    def test_split_timestamp_rejects_negative(self):
        """负时间戳必须被拒绝。"""

        with self.assertRaises(ValueError):
            split_timestamp_ns(-1)


if __name__ == "__main__":
    unittest.main()
