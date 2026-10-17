#!/usr/bin/env python3
"""
功能：由同一目标在两台相机中的像素坐标计算三维位置和光轴深度。
输入：YAML 标定参数、原始图像像素 uv1/uv2，要求来自同一时刻的同一点。
输出：第一台相机坐标系中的 [X, Y, Z]，单位 m；不可靠时返回 None。
约定：X 向右、Y 向下、Z 向前；第二相机坐标 = R @ 第一相机坐标 + T。
说明：示例 YAML 是假设的双目标定参数，不代表实际相机标定。
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


class StereoDepthEstimator:
    """预先缓存双目投影矩阵，对单对像素点进行稀疏三角化。"""

    def __init__(self, parameters):
        """
        功能：加载并检查双目内参、畸变、外参和质量阈值。
        输入：parameters 为 YAML 中 stereo_depth 配置字典。
        输出：无返回值；配置非法时抛出 ValueError。
        """

        self.camera_matrices = []
        self.distortions = []
        for name in ("camera_1", "camera_2"):
            camera = parameters[name]
            matrix = np.asarray(camera["matrix"], dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(camera["distortion"], dtype=np.float64).reshape(-1)
            if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)):
                raise ValueError("相机内参和畸变必须为有限数值。")
            if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
                raise ValueError("相机焦距必须大于 0。")
            if not np.allclose(matrix[2], [0, 0, 1]) or matrix[0, 1] != 0 or matrix[1, 0] != 0:
                raise ValueError("相机矩阵必须采用无斜切的标准针孔内参形式。")
            if distortion.size not in (4, 5, 8, 12, 14):
                raise ValueError("畸变参数数量必须为 4、5、8、12 或 14。")
            self.camera_matrices.append(matrix)
            self.distortions.append(distortion)

        self.rotation = np.asarray(parameters["rotation_1_to_2"], dtype=np.float64).reshape(3, 3)
        self.translation = np.asarray(parameters["translation_1_to_2_m"], dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(self.rotation)) or not np.all(np.isfinite(self.translation)):
            raise ValueError("外参必须为有限数值。")
        if not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(self.rotation), 1.0):
            raise ValueError("rotation_1_to_2 必须为有效旋转矩阵。")
        if np.linalg.norm(self.translation) <= 0:
            raise ValueError("双目基线不能为 0。")

        self.min_depth_m = float(parameters["min_depth_m"])
        self.max_depth_m = float(parameters["max_depth_m"])
        self.max_reprojection_error_px = float(parameters["max_reprojection_error_px"])
        self.min_ray_angle_deg = float(parameters["min_ray_angle_deg"])
        values = [self.min_depth_m, self.max_depth_m, self.max_reprojection_error_px, self.min_ray_angle_deg]
        if not np.all(np.isfinite(values)) or not 0 < self.min_depth_m < self.max_depth_m:
            raise ValueError("深度范围必须为有限正数，且最小值小于最大值。")
        if self.max_reprojection_error_px <= 0 or not 0 < self.min_ray_angle_deg < 90:
            raise ValueError("重投影阈值必须大于 0，最小射线夹角必须在 (0, 90) 度内。")

        # 三角化使用去畸变后的归一化坐标，因此投影矩阵只包含外参，不再乘内参 K。
        # 以第一相机为参考系：P1 = [I | 0]，P2 = [R | T]；T 的米制单位决定输出单位。
        self.projection_1 = np.column_stack((np.eye(3), np.zeros(3)))
        self.projection_2 = np.column_stack((self.rotation, self.translation))
        # projectPoints 接收 Rodrigues 旋转向量，预先转换以供后续重投影检查使用。
        self.rotation_vector_2 = cv2.Rodrigues(self.rotation)[0]
        self.last_invalid_reason = None

    @classmethod
    def from_yaml(cls, path):
        """
        功能：从 YAML 创建深度计算对象；文件只需在初始化时读取。
        输入：path 为 YAML 文件路径。
        输出：StereoDepthEstimator 实例。
        """

        with Path(path).open("r", encoding="utf-8") as stream:
            return cls(yaml.safe_load(stream)["stereo_depth"])

    def triangulate(self, uv1, uv2):
        """
        功能：去畸变、三角化并检查射线夹角、正深度及重投影误差。
        输入：uv1、uv2 为原始图像对应像素 [u, v]；两路均有效且已配对。
        输出：第一相机坐标系的 numpy 数组 [X, Y, Z]，单位 m；失败为 None。
              失败原因保存在 last_invalid_reason，第三个数是光轴深度。
        """

        self.last_invalid_reason = None
        pixels = [np.asarray(uv, dtype=np.float64) for uv in (uv1, uv2)]
        if any(point.shape != (2,) or not np.all(np.isfinite(point)) for point in pixels):
            self.last_invalid_reason = "像素坐标必须为两个有限数值。"
            return None
        # 不指定输出投影矩阵时，undistortPoints 同时去除镜头畸变和内参影响，
        # 得到归一化成像坐标 (x, y)，对应相机坐标系中的视线方向 [x, y, 1]。
        normalized = [
            cv2.undistortPoints(point.reshape(1, 1, 2), matrix, distortion).reshape(2, 1)
            for point, matrix, distortion in zip(pixels, self.camera_matrices, self.distortions)
        ]
        ray1 = np.r_[normalized[0].ravel(), 1.0]
        # R 将第一相机坐标变换到第二相机；用 R.T 将第二条视线转回第一相机系。
        # 此处只转换方向，不转换视线起点，因此不需要加平移 T。
        ray2 = self.rotation.T @ np.r_[normalized[1].ravel(), 1.0]
        cosine = np.dot(ray1, ray2) / (np.linalg.norm(ray1) * np.linalg.norm(ray2))
        # 浮点误差可能使余弦略超出 [-1, 1]；截断后再求夹角，避免 arccos 产生 NaN。
        angle = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
        # 视线接近平行时，微小像素误差会引起很大的深度变化，提前拒绝此类结果。
        if not np.isfinite(angle) or angle < self.min_ray_angle_deg:
            self.last_invalid_reason = "两条射线夹角过小，深度不可靠。"
            return None

        # 线性三角化：由两路观测和投影矩阵求齐次三维点 [Xh, Yh, Zh, W]。
        homogeneous = cv2.triangulatePoints(
            self.projection_1, self.projection_2, normalized[0], normalized[1]
        ).ravel()
        # W 接近 0 表示点趋向无穷远，不能安全转换为有限的三维坐标。
        if not np.all(np.isfinite(homogeneous)) or abs(homogeneous[3]) < np.finfo(float).eps:
            self.last_invalid_reason = "三角化结果无效或位于无穷远。"
            return None
        # 齐次坐标除以 W，恢复第一相机系中的米制坐标；Z 是光轴深度而非欧氏距离。
        xyz = homogeneous[:3] / homogeneous[3]
        xyz2 = self.rotation @ xyz + self.translation
        # 检查点位于两台相机前方；第一相机的正深度由配置的正值深度范围保证。
        if not np.all(np.isfinite(xyz)) or xyz2[2] <= 0 or not self.min_depth_m <= xyz[2] <= self.max_depth_m:
            self.last_invalid_reason = "目标位于相机后方或超出允许深度范围。"
            return None

        # 将三维点分别投回两张原始图像（包含内参和畸变），与输入像素比较。
        # 两路误差都必须过关，用于过滤不符合双目几何关系的匹配或标定偏差。
        for pixel, matrix, distortion, rotation, translation in zip(
            pixels, self.camera_matrices, self.distortions,
            (np.zeros(3), self.rotation_vector_2), (np.zeros(3), self.translation)
        ):
            projected = cv2.projectPoints(xyz.reshape(1, 3), rotation, translation, matrix, distortion)[0].reshape(2)
            error = np.linalg.norm(projected - pixel)  # 原始图像上的二维欧氏距离，单位为像素。
            if not np.isfinite(error) or error > self.max_reprojection_error_px:
                self.last_invalid_reason = "重投影误差过大，两路像素或标定不一致。"
                return None
        return xyz


def main():
    """
    功能：命令行读取标定文件和两路像素，输出一次三维位置与深度。
    输入：--config YAML路径、--uv1 u1 v1、--uv2 u2 v2。
    输出：终端 JSON，包含 valid、xyz_m、depth_m 或失败原因。
    """

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--uv1", nargs=2, type=float, required=True)
    parser.add_argument("--uv2", nargs=2, type=float, required=True)
    args = parser.parse_args()
    estimator = StereoDepthEstimator.from_yaml(args.config)
    xyz = estimator.triangulate(args.uv1, args.uv2)
    result = {"valid": xyz is not None}
    if xyz is None:
        result["reason"] = estimator.last_invalid_reason
    else:
        result.update(xyz_m=xyz.tolist(), depth_m=float(xyz[2]))
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
