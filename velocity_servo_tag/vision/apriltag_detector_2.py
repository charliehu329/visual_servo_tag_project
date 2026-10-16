#!/usr/bin/env python3
"""
apriltag_detector.py

功能：同轮采集两台 USB 相机的 RGB 图像，分别检测 AprilTag 中心并发布。
输入：launch 加载 config/velocity_servo_tag.yaml 中的 ROS 2 参数。
      camera_* 为第一路配置，camera_2_* 为第二路配置；检测和滤波对象独立。
      camera_pair_max_delta_ms 为两次 grab 完成时间差上限，单位 ms。
输出：target_position_topic，std_msgs/msg/Float64MultiArray：
      [valid1, valid2, u1, v1, u2, v2]。
      valid=1 表示对应相机读取成功且检测有效，否则为 0，坐标也为 0。
      超过抓帧时间差阈值时两路均无效；该阈值不保证硬件曝光同步。
      仅第一路可以显示预览；第二路不显示预览。
接口：AprilTagDetector.detect(image, require_depth=False) 返回 [u, v, 0] 或 None。
      require_depth=True 保留检测器原有几何深度接口，节点不发布深度。
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray

from velocity_servo_tag.vision.camera import USBCamera


class AprilTagDetector:
    """
    使用 pupil_apriltags 检测指定 AprilTag 的检测器。
    """

    def __init__(
        self,
        tag_family="tag36h11",
        target_tag_id=0,
        detector_threads=2,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
        uv_filter_alpha=0.4,
        camera_fx=1.0,
        camera_fy=1.0,
        camera_cx=0.0,
        camera_cy=0.0,
        tag_size_m=0.12,
        depth_filter_alpha=0.4,
        depth_min_m=0.05,
        depth_max_m=5.0,
        max_edge_ratio=1.8,
    ):
        """
        初始化 AprilTag 检测参数。
        """

        try:
            from pupil_apriltags import Detector
        except ImportError as error:
            raise RuntimeError(
                "未安装 pupil_apriltags。请先执行："
                "python3 -m pip install pupil-apriltags"
            ) from error

        self.tag_family = str(tag_family)
        self.target_tag_id = int(target_tag_id)
        self.detector_threads = int(detector_threads)
        self.quad_decimate = float(quad_decimate)
        self.quad_sigma = float(quad_sigma)
        self.refine_edges = bool(refine_edges)
        self.decode_sharpening = float(
            decode_sharpening
        )
        self.uv_filter_alpha = float(
            uv_filter_alpha
        )
        self.camera_params = (
            float(camera_fx),
            float(camera_fy),
            float(camera_cx),
            float(camera_cy),
        )
        self.tag_size_m = float(tag_size_m)
        self.depth_filter_alpha = float(
            depth_filter_alpha
        )
        self.depth_min_m = float(depth_min_m)
        self.depth_max_m = float(depth_max_m)

        # 四边形边长比上限 max(edge)/min(edge)，用于剔除被裁切、遮挡
        # 或误检的四边形（正常倾斜通常不超过 1.3）。0 表示不检查。
        self.max_edge_ratio = float(max_edge_ratio)

        # 供节点打日志使用的诊断量。
        self.last_drop_reason = None
        self.last_edge_pixel = None
        self.last_edge_ratio = None

        self.validate_parameters()

        self.detector = Detector(
            families=self.tag_family,
            nthreads=self.detector_threads,
            quad_decimate=self.quad_decimate,
            quad_sigma=self.quad_sigma,
            refine_edges=int(self.refine_edges),
            decode_sharpening=(
                self.decode_sharpening
            ),
            debug=0,
        )

        # 最近一次有效检测信息，供显示窗口使用。
        self.last_corners = None
        self.last_raw_center = None
        self.last_filtered_center = None
        self.last_raw_depth = None
        self.last_filtered_depth = None

    def set_camera_params(self, fx, fy, cx, cy):
        """Update pose-estimation intrinsics after a zoom/focus command."""
        values = tuple(float(value) for value in (fx, fy, cx, cy))
        if not all(np.isfinite(values)) or values[0] <= 0.0 or values[1] <= 0.0:
            raise ValueError("dynamic camera intrinsics are invalid")
        self.camera_params = values

    def validate_parameters(self):
        """
        检查检测参数是否合法。
        """

        if not self.tag_family:
            raise ValueError(
                "tag_family 不能为空。"
            )

        if self.target_tag_id < 0:
            raise ValueError(
                "target_tag_id 必须大于或等于 0。"
            )

        if self.detector_threads <= 0:
            raise ValueError(
                "detector_threads 必须大于 0。"
            )

        if self.quad_decimate < 1.0:
            raise ValueError(
                "quad_decimate 必须大于或等于 1.0。"
            )

        if self.quad_sigma < 0.0:
            raise ValueError(
                "quad_sigma 不能小于 0。"
            )

        if self.decode_sharpening < 0.0:
            raise ValueError(
                "decode_sharpening 不能小于 0。"
            )

        if not 0.0 < self.uv_filter_alpha <= 1.0:
            raise ValueError(
                "uv_filter_alpha 必须在 (0, 1] 范围内。"
            )

        if not all(np.isfinite(self.camera_params)):
            raise ValueError("相机内参必须为有限数值。")

        if self.camera_params[0] <= 0.0 or self.camera_params[1] <= 0.0:
            raise ValueError("camera_fx 和 camera_fy 必须大于 0。")

        if self.tag_size_m <= 0.0:
            raise ValueError("tag_size_m 必须大于 0。")

        if not 0.0 < self.depth_filter_alpha <= 1.0:
            raise ValueError(
                "depth_filter_alpha 必须在 (0, 1] 范围内。"
            )

        if not (
            0.0 < self.depth_min_m < self.depth_max_m
        ):
            raise ValueError(
                "depth_min_m 和 depth_max_m 范围不合法。"
            )

        if self.max_edge_ratio < 0.0:
            raise ValueError(
                "max_edge_ratio 不能小于 0（0 表示不检查）。"
            )

    def reset_tracking_state(self):
        """
        清除目标丢失前保存的检测和滤波状态。
        """

        self.last_corners = None
        self.last_raw_center = None
        self.last_filtered_center = None
        self.last_raw_depth = None
        self.last_filtered_depth = None
        self.last_edge_pixel = None
        self.last_edge_ratio = None

    def filter_depth(self, raw_depth):
        """对 AprilTag 深度进行一阶低通滤波。"""

        raw_depth = float(raw_depth)

        if self.last_filtered_depth is None:
            filtered_depth = raw_depth
        else:
            alpha = self.depth_filter_alpha
            filtered_depth = (
                alpha * raw_depth +
                (1.0 - alpha) * self.last_filtered_depth
            )

        self.last_raw_depth = raw_depth
        self.last_filtered_depth = filtered_depth
        return float(filtered_depth)

    def filter_center(
        self,
        raw_center
    ):
        """
        对标签中心进行一阶低通滤波。
        """

        raw_center = np.asarray(
            raw_center,
            dtype=float
        )

        if self.last_filtered_center is None:
            filtered_center = raw_center.copy()
        else:
            alpha = self.uv_filter_alpha

            filtered_center = (
                alpha * raw_center +
                (1.0 - alpha) *
                self.last_filtered_center
            )

        self.last_filtered_center = filtered_center

        return filtered_center.copy()

    def detect(
        self,
        image,
        require_depth=True
    ):
        """
        功能：检测 RGB 图像中的指定 AprilTag，并独立滤波。
        输入：image 为 RGB 图像；require_depth 表示是否检查和计算深度。
        输出：成功返回 [u, v, depth_m]，失败返回 None；关闭深度时 depth_m 为 0。
        """

        if image is None:
            self.last_drop_reason = "camera image unavailable"
            self.reset_tracking_state()
            return None

        if (
            image.ndim != 3 or
            image.shape[2] != 3
        ):
            self.last_drop_reason = "unexpected image shape"
            self.reset_tracking_state()
            return None

        gray_image = cv2.cvtColor(
            image,
            cv2.COLOR_RGB2GRAY
        )

        # pupil_apriltags 要求输入连续的 uint8 灰度图。
        gray_image = np.ascontiguousarray(
            gray_image,
            dtype=np.uint8
        )

        detections = self.detector.detect(
            gray_image,
            estimate_tag_pose=False
        )

        target_detections = [
            detection
            for detection in detections
            if int(detection.tag_id) ==
            self.target_tag_id
        ]

        if not target_detections:
            # 正常情况（视野里没有标签），不作为异常记录。
            self.last_drop_reason = None
            self.reset_tracking_state()
            return None

        # 正常情况下同一 ID 只出现一次。
        # 如果出现多个结果，选择解码质量最高的一个。
        target = max(
            target_detections,
            key=lambda detection: float(
                detection.decision_margin
            )
        )

        raw_center = np.asarray(
            target.center,
            dtype=float
        )

        corners = np.asarray(
            target.corners,
            dtype=float
        )

        if (
            raw_center.shape != (2,) or
            corners.shape != (4, 2) or
            not np.all(np.isfinite(raw_center)) or
            not np.all(np.isfinite(corners))
        ):
            self.last_drop_reason = "invalid detection geometry"
            self.reset_tracking_state()
            return None

        # 几何深度：Z = fx * 标签物理边长 / 标签像素边长。
        # corners 按顺序绕标签一周，相邻角点构成 4 条边。
        edge_pixels = np.linalg.norm(
            np.roll(corners, -1, axis=0) - corners,
            axis=1
        )
        edge_pixel = float(np.mean(edge_pixels))
        edge_ratio = float(
            np.max(edge_pixels) /
            max(float(np.min(edge_pixels)), 1e-9)
        )

        if not np.isfinite(edge_pixel) or edge_pixel <= 1.0:
            self.last_drop_reason = (
                f"tag edge too small ({edge_pixel:.1f} px)"
            )
            self.reset_tracking_state()
            return None

        if (
            self.max_edge_ratio > 0.0 and
            edge_ratio > self.max_edge_ratio
        ):
            self.last_drop_reason = (
                "quad distorted, edge ratio "
                f"{edge_ratio:.2f} > {self.max_edge_ratio:.2f}"
            )
            self.reset_tracking_state()
            return None

        # camera_params[0] 即 fx，单位 pixel。
        raw_depth = (
            float(self.camera_params[0]) *
            self.tag_size_m /
            edge_pixel
        )

        if require_depth and not (
            self.depth_min_m <= raw_depth <= self.depth_max_m
        ):
            self.last_drop_reason = (
                f"depth {raw_depth:.3f} m outside "
                f"[{self.depth_min_m:.3f}, "
                f"{self.depth_max_m:.3f}] m"
            )
            self.reset_tracking_state()
            return None

        self.last_drop_reason = None
        self.last_edge_pixel = edge_pixel
        self.last_edge_ratio = edge_ratio

        filtered_center = self.filter_center(
            raw_center
        )
        filtered_depth = self.filter_depth(raw_depth) if require_depth else 0.0

        self.last_raw_center = raw_center.copy()
        self.last_corners = corners.copy()

        feature = np.asarray(
            [
                float(filtered_center[0]),
                float(filtered_center[1]),
                float(filtered_depth),
            ],
            dtype=float
        )

        return feature


class AprilTagDetectorNode(Node):
    """
    读取 USB 相机并发布 AprilTag 中心位置的 ROS 2 节点。
    """

    def __init__(self):
        """
        功能：从 ROS 2 参数初始化两路相机、检测器和发布器。
        输入：无显式参数；配置由 launch 加载的 YAML 提供。
        输出：无返回值；建立相机对象和采集定时器。
        """

        super().__init__("apriltag_detector")

        # =====================================================
        # 声明参数
        # =====================================================

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 60.0)
        self.declare_parameter("camera_2_enabled", False)
        self.declare_parameter("camera_2_index", 0)
        self.declare_parameter("camera_2_width", 640)
        self.declare_parameter("camera_2_height", 480)
        self.declare_parameter("camera_2_fps", 60.0)

        self.declare_parameter(
            "tag_family",
            "tag36h11"
        )
        self.declare_parameter("target_tag_id", 0)

        self.declare_parameter("detector_threads", 2)
        self.declare_parameter("quad_decimate", 1.0)
        self.declare_parameter("quad_sigma", 0.0)
        self.declare_parameter("refine_edges", True)
        self.declare_parameter(
            "decode_sharpening",
            0.25
        )
        self.declare_parameter("uv_filter_alpha", 0.4)

        self.declare_parameter("camera_fx", 2057.0)
        self.declare_parameter("camera_fy", 2054.0)
        self.declare_parameter("camera_cx", 1000.0)
        self.declare_parameter("camera_cy", 592.0)
        self.declare_parameter("tag_size_m", 0.12)
        self.declare_parameter("depth_filter_alpha", 0.4)
        self.declare_parameter("depth_min_m", 0.25)
        self.declare_parameter("depth_max_m", 0.80)
        self.declare_parameter("max_edge_ratio", 1.8)

        self.declare_parameter(
            "target_position_topic",
            "/apriltag_detector/target_position"
        )
        self.declare_parameter("use_dynamic_camera_info", True)
        self.declare_parameter("camera_info_topic", "/lens/camera_info")
        self.declare_parameter("show_window", True)

        # =====================================================
        # 读取参数
        # =====================================================

        self.camera_index = int(
            self.get_parameter(
                "camera_index"
            ).value
        )
        self.camera_width = int(
            self.get_parameter(
                "camera_width"
            ).value
        )
        self.camera_height = int(
            self.get_parameter(
                "camera_height"
            ).value
        )
        self.camera_fps = float(
            self.get_parameter(
                "camera_fps"
            ).value
        )

        self.camera_2_enabled = bool(
            self.get_parameter("camera_2_enabled").value
        )
        self.camera_2_index = int(
            self.get_parameter("camera_2_index").value
        )
        self.camera_2_width = int(
            self.get_parameter("camera_2_width").value
        )
        self.camera_2_height = int(
            self.get_parameter("camera_2_height").value
        )
        self.camera_2_fps = float(
            self.get_parameter("camera_2_fps").value
        )

        self.tag_family = str(
            self.get_parameter(
                "tag_family"
            ).value
        )
        self.target_tag_id = int(
            self.get_parameter(
                "target_tag_id"
            ).value
        )

        self.detector_threads = int(
            self.get_parameter(
                "detector_threads"
            ).value
        )
        self.quad_decimate = float(
            self.get_parameter(
                "quad_decimate"
            ).value
        )
        self.quad_sigma = float(
            self.get_parameter(
                "quad_sigma"
            ).value
        )
        self.refine_edges = bool(
            self.get_parameter(
                "refine_edges"
            ).value
        )
        self.decode_sharpening = float(
            self.get_parameter(
                "decode_sharpening"
            ).value
        )
        self.uv_filter_alpha = float(
            self.get_parameter(
                "uv_filter_alpha"
            ).value
        )

        self.camera_fx = float(
            self.get_parameter("camera_fx").value
        )
        self.camera_fy = float(
            self.get_parameter("camera_fy").value
        )
        self.camera_cx = float(
            self.get_parameter("camera_cx").value
        )
        self.camera_cy = float(
            self.get_parameter("camera_cy").value
        )
        self.tag_size_m = float(
            self.get_parameter("tag_size_m").value
        )
        self.depth_filter_alpha = float(
            self.get_parameter("depth_filter_alpha").value
        )
        self.depth_min_m = float(
            self.get_parameter("depth_min_m").value
        )
        self.depth_max_m = float(
            self.get_parameter("depth_max_m").value
        )
        self.max_edge_ratio = float(
            self.get_parameter("max_edge_ratio").value
        )

        self.target_position_topic = str(
            self.get_parameter(
                "target_position_topic"
            ).value
        )
        self.use_dynamic_camera_info = bool(
            self.get_parameter("use_dynamic_camera_info").value
        )
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.show_window = bool(
            self.get_parameter(
                "show_window"
            ).value
        )

        self.declare_parameter("camera_pair_max_delta_ms", 60.0)
        self.camera_pair_max_delta_ms = float(
            self.get_parameter("camera_pair_max_delta_ms").value
        )
        self.declare_parameter("camera_fps_log_interval_s", 20.0)
        self.camera_fps_log_interval_s = float(
            self.get_parameter("camera_fps_log_interval_s").value
        )
        # 第二路检测参数独立读取，避免两台相机共享滤波状态。
        self.camera_2_detector_parameters = {}
        for name in (
            "tag_family", "target_tag_id", "detector_threads",
            "quad_decimate", "quad_sigma", "refine_edges",
            "decode_sharpening", "uv_filter_alpha", "camera_fx",
            "camera_fy", "camera_cx", "camera_cy", "tag_size_m",
            "depth_filter_alpha", "depth_min_m", "depth_max_m", "max_edge_ratio",
        ):
            parameter_name = "camera_2_" + name.removeprefix("camera_")
            self.declare_parameter(parameter_name, getattr(self, name))
            self.camera_2_detector_parameters[name] = self.get_parameter(
                parameter_name
            ).value

        self.validate_parameters()

        # =====================================================
        # 初始化相机和检测器
        # =====================================================

        self.detector = AprilTagDetector(
            tag_family=self.tag_family,
            target_tag_id=self.target_tag_id,
            detector_threads=self.detector_threads,
            quad_decimate=self.quad_decimate,
            quad_sigma=self.quad_sigma,
            refine_edges=self.refine_edges,
            decode_sharpening=(
                self.decode_sharpening
            ),
            uv_filter_alpha=self.uv_filter_alpha,
            camera_fx=self.camera_fx,
            camera_fy=self.camera_fy,
            camera_cx=self.camera_cx,
            camera_cy=self.camera_cy,
            tag_size_m=self.tag_size_m,
            depth_filter_alpha=self.depth_filter_alpha,
            depth_min_m=self.depth_min_m,
            depth_max_m=self.depth_max_m,
            max_edge_ratio=self.max_edge_ratio,
        )

        self.detector_2 = None
        if self.camera_2_enabled:
            self.detector_2 = AprilTagDetector(**self.camera_2_detector_parameters)

        self.camera = USBCamera(
            camera_index=self.camera_index,
            width=self.camera_width,
            height=self.camera_height,
            fps=int(round(self.camera_fps))
        )
        self.camera_2 = None
        self.camera_2_image = None
        self.camera_2_timer = None
        if self.camera_2_enabled:
            try:
                self.camera_2 = USBCamera(
                    camera_index=self.camera_2_index,
                    width=self.camera_2_width,
                    height=self.camera_2_height,
                    fps=int(round(self.camera_2_fps))
                )
            except Exception:
                self.camera.release()
                super().destroy_node()
                raise

        # =====================================================
        # ROS 通信和运行状态
        # =====================================================

        self.position_publisher = self.create_publisher(
            Float64MultiArray,
            self.target_position_topic,
            1
        )
        self.camera_info_subscription = None
        if self.use_dynamic_camera_info:
            self.camera_info_subscription = self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.camera_info_callback,
                10,
            )

        self.target_was_detected = False
        self.last_detection_time = None
        self.last_detection_feature = None
        self.last_pixel_velocity = None
        # 检测相机实时读取的频率
        # =====================================================
        # 分别统计两路成功取得 RGB 图像的频率，与检测成功率无关。
        self.successful_frame_count = 0
        self.successful_frame_count_2 = 0
        self.fps_measurement_start = time.monotonic()
        # =====================================================

        self.camera_timer = self.create_timer(
            1.0 / self.camera_fps,
            self.camera_callback
        )

        self.get_logger().info(
            "AprilTagDetectorNode started. "
            f"camera={self.camera_index}, "
            f"requested_image={self.camera_width}x"
            f"{self.camera_height}@{self.camera_fps:.1f} Hz, "
            f"family={self.tag_family}, "
            f"target_id={self.target_tag_id}, "
            f"tag_size={self.tag_size_m:.4f} m, "
            f"topic={self.target_position_topic}."
        )

    def camera_info_callback(self, message):
        """Use the current zoom-dependent K matrix for AprilTag pose depth."""
        if len(message.k) != 9:
            self.get_logger().error("CameraInfo K must contain nine values")
            return
        try:
            self.detector.set_camera_params(
                message.k[0], message.k[4], message.k[2], message.k[5]
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))


    def validate_parameters(self):
        """
        功能：检查两路相机和 ROS 2 节点参数是否合法。
        输入：无显式参数；读取节点已保存的配置。
        输出：合法时无返回值；非法时抛出 ValueError。
        """

        if not np.isfinite(self.camera_pair_max_delta_ms) or self.camera_pair_max_delta_ms <= 0.0:
            raise ValueError("camera_pair_max_delta_ms 必须为大于 0 的有限数值。")
        if not np.isfinite(self.camera_fps_log_interval_s) or self.camera_fps_log_interval_s <= 0.0:
            raise ValueError("camera_fps_log_interval_s 必须为大于 0 的有限数值。")
        if not np.isfinite(self.camera_fps) or self.camera_fps < 0.5:
            raise ValueError("camera_fps 必须为不小于 0.5 的有限数值。")
        if self.camera_2_enabled:
            if self.camera_2_fps != self.camera_fps:
                raise ValueError("双相机采集时 camera_fps 和 camera_2_fps 必须相同。")
            if self.camera_2_index < 0:
                raise ValueError("camera_2_index 不能小于 0。")
            if self.camera_2_index == self.camera_index:
                raise ValueError("两台相机的设备编号不能相同。")
            if self.camera_2_width <= 0 or self.camera_2_height <= 0:
                raise ValueError("第二台相机的图像宽度和高度必须大于 0。")
            if not np.isfinite(self.camera_2_fps) or self.camera_2_fps < 0.5:
                raise ValueError("camera_2_fps 必须为不小于 0.5 的有限数值。")

        if self.camera_index < 0:
            raise ValueError(
                "camera_index 不能小于 0。"
            )

        if self.camera_width <= 0:
            raise ValueError(
                "camera_width 必须大于 0。"
            )

        if self.camera_height <= 0:
            raise ValueError(
                "camera_height 必须大于 0。"
            )

        if self.camera_fps <= 0.0:
            raise ValueError(
                "camera_fps 必须大于 0。"
            )

        if self.camera_fx <= 0.0 or self.camera_fy <= 0.0:
            raise ValueError("camera_fx 和 camera_fy 必须大于 0。")

        if self.tag_size_m <= 0.0:
            raise ValueError("tag_size_m 必须大于 0。")

        if not 0.0 < self.depth_filter_alpha <= 1.0:
            raise ValueError(
                "depth_filter_alpha 必须在 (0, 1] 范围内。"
            )

        if not 0.0 < self.depth_min_m < self.depth_max_m:
            raise ValueError("深度范围参数不合法。")

        if self.max_edge_ratio < 0.0:
            raise ValueError(
                "max_edge_ratio 不能小于 0（0 表示不检查）。"
            )

        if not self.target_position_topic:
            raise ValueError(
                "target_position_topic 不能为空。"
            )

    def publish_target(self, feature, feature_2=None):
        """
        功能：发布两路目标的有效标志和像素坐标。
        输入：feature、feature_2 为各自检测结果；None 表示该路无效。
        输出：无返回值；发布 [valid1, valid2, u1, v1, u2, v2]。
        """

        message = Float64MultiArray()
        message.data = [
            1.0 if feature is not None else 0.0,
            1.0 if feature_2 is not None else 0.0,
            float(feature[0]) if feature is not None else 0.0,
            float(feature[1]) if feature is not None else 0.0,
            float(feature_2[0]) if feature_2 is not None else 0.0,
            float(feature_2[1]) if feature_2 is not None else 0.0,
        ]
        self.position_publisher.publish(message)

    def publish_invalid_target(self):
        """
        功能：发布双路无效消息。
        输入：无。
        输出：无返回值；发布六个 0。
        """

        self.publish_target(None, None)

    def draw_detection(
        self,
        image,
        feature,
        pixel_velocity=None
    ):
        """
        功能：在第一路预览中绘制标签边框和像素中心。
        输入：image 为 RGB 图像，feature 为检测结果或 None；pixel_velocity 为可选速度。
        输出：无返回值；更新第一路窗口，按 q 停止节点。
        """

        display_image = cv2.cvtColor(
            image,
            cv2.COLOR_RGB2BGR
        )

        if feature is None:
            cv2.putText(
                display_image,
                "Target not detected",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2
            )
        else:
            corners = np.rint(
                self.detector.last_corners
            ).astype(np.int32)

            for corner_index in range(4):
                start_point = tuple(
                    int(value)
                    for value in corners[
                        corner_index
                    ]
                )
                end_point = tuple(
                    int(value)
                    for value in corners[
                        (corner_index + 1) % 4
                    ]
                )

                cv2.line(
                    display_image,
                    start_point,
                    end_point,
                    (0, 255, 0),
                    2
                )

            center = (
                int(round(feature[0])),
                int(round(feature[1]))
            )

            cv2.circle(
                display_image,
                center,
                4,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                display_image,
                (
                    f"id={self.target_tag_id}, "
                    f"u={feature[0]:.1f}, "
                    f"v={feature[1]:.1f}"
                ),
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            if pixel_velocity is not None:
                cv2.putText(
                    display_image,
                    (
                        f"du/dt={pixel_velocity[0]:.1f} px/s, "
                        f"dv/dt={pixel_velocity[1]:.1f} px/s"
                    ),
                    (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 0),
                    2
                )

        cv2.imshow(
            "AprilTag Detector",
            display_image
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            self.publish_invalid_target()
            rclpy.shutdown()

    def log_camera_fps(self, image, image_2):
        """
        功能：按 YAML 配置周期统计并打印两台相机成功取图的实际 FPS。
        输入：image、image_2 为两路 RGB 图像，None 表示对应一路取图失败。
        输出：无返回值；到达周期时打印日志并重置计数和统计起点。
        """

        if image is not None:
            self.successful_frame_count += 1
        if image_2 is not None:
            self.successful_frame_count_2 += 1

        measurement_now = time.monotonic()
        elapsed = measurement_now - self.fps_measurement_start
        if elapsed >= self.camera_fps_log_interval_s:
            self.get_logger().info(
                f"Successful camera-read FPS: camera1={self.successful_frame_count / elapsed:.2f}, "
                f"camera2={self.successful_frame_count_2 / elapsed:.2f} "
                f"(enabled={self.camera_2_enabled})."
            )
            self.successful_frame_count = 0
            self.successful_frame_count_2 = 0
            self.fps_measurement_start = measurement_now

    def camera_callback(self):
        """
        功能：同轮抓取两路图像，统计 FPS、检查时间差，分别检测并统一发布。
        输入：无显式参数；使用相机对象和 YAML 时间差阈值。
        输出：无返回值；发布 [valid1, valid2, u1, v1, u2, v2]。
              抓帧完成时间差超过阈值时，两路均无效；只预览第一路。
        """

        grabbed = self.camera.grab()
        first_time = time.monotonic()
        grabbed_2 = False
        second_time = None
        if self.camera_2 is not None:
            grabbed_2 = self.camera_2.grab()
            second_time = time.monotonic()

        # 两路均先抓帧，再解码；这里衡量软件抓帧完成时间，不是曝光时间。
        image = self.camera.retrieve() if grabbed else None
        self.camera_2_image = (
            self.camera_2.retrieve() if grabbed_2 else None
        )
        # 采集成功即计数，即使随后因同步超限或未检测到标签而丢弃。
        self.log_camera_fps(image, self.camera_2_image)
        self.last_camera_pair_delta_ms = None
        if grabbed and grabbed_2:
            self.last_camera_pair_delta_ms = abs(second_time - first_time) * 1000.0
            if self.last_camera_pair_delta_ms > self.camera_pair_max_delta_ms:
                self.detector.reset_tracking_state()
                self.detector_2.reset_tracking_state()
                self.publish_invalid_target()
                self.get_logger().warn(
                    f"Camera pair dropped: grab delta={self.last_camera_pair_delta_ms:.2f} ms "
                    f"> {self.camera_pair_max_delta_ms:.2f} ms.",
                    throttle_duration_sec=1.0
                )
                return

        feature = self.detector.detect(image, require_depth=False)
        feature_2 = None
        if self.detector_2 is not None:
            feature_2 = self.detector_2.detect(
                self.camera_2_image, require_depth=False
            )
        self.publish_target(feature, feature_2)

        if self.show_window and image is not None:
            self.draw_detection(image, feature)

    def destroy_node(self):
        """
        功能：停止采集，释放两路相机和窗口，然后销毁 ROS 2 节点。
        输入：无。
        输出：父类 destroy_node() 的返回值。
        """

        self.camera_timer.cancel()
        if self.camera_2_timer is not None:
            self.camera_2_timer.cancel()

        if rclpy.ok():
            self.publish_invalid_target()

        if self.camera is not None:
            self.camera.release()
        if self.camera_2 is not None:
            self.camera_2.release()
        self.camera_2_image = None

        if self.show_window:
            cv2.destroyAllWindows()

        return super().destroy_node()


def main(args=None):
    """
    ROS 2 节点运行入口。
    """

    rclpy.init(args=args)

    node = None

    try:
        node = AprilTagDetectorNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
