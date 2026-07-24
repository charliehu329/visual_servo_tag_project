#!/usr/bin/env python3
"""
apriltag_detector.py

概括：
    创建 AprilTagDetector 检测器和 AprilTagDetectorNode ROS 2 节点。
    节点读取普通 USB 相机图像，使用 pupil_apriltags 检测指定的
    tag36h11 标签，并发布标签中心的像素坐标，供 Simulink 中的
    基础单目 IBVS 控制器使用。

输入：
    USB 相机 RGB 图像，numpy.ndarray，格式为 (H, W, 3)。

    ROS 2 yaml 参数：
        camera_index：
            USB 相机编号。
        camera_width、camera_height：
            相机图像分辨率，单位 pixel。
        camera_fps：
            目标相机帧率和检测定时器频率，单位 Hz。
        tag_family：
            AprilTag 标签族，默认 tag36h11。
        target_tag_id：
            需要跟踪的标签 ID，默认 0。
        detector_threads：
            pupil_apriltags 使用的检测线程数。
        quad_decimate：
            四边形检测降采样倍数。1.0 表示不降采样。
        quad_sigma：
            四边形检测前的高斯模糊标准差。
        refine_edges：
            是否对检测到的标签边缘进行细化。
        decode_sharpening：
            标签编码区域的锐化强度。
        uv_filter_alpha：
            标签中心低通滤波系数，范围 (0, 1]。
            1.0 表示不进行时间滤波。
        target_position_topic：
            检测结果发布 topic。
        show_window：
            是否显示实时检测窗口。

输出：
    ROS topic：
        /apriltag_detector/target_position

    消息类型：
        std_msgs/msg/Float64MultiArray

    数据顺序：
        [valid, u, v]

        valid：
            1.0 表示检测有效，0.0 表示未检测到目标。
        u、v：
            标签中心的像素坐标，单位 pixel。

接口：
    AprilTagDetector.detect(image)
        检测成功时返回 [u, v]。
        检测失败时返回 None。

    AprilTagDetectorNode
        按 camera_fps 读取相机并持续发布检测结果。

方法：
    1. 从 USB 相机读取 RGB 图像。
    2. 将 RGB 图像转换为灰度图。
    3. 使用 pupil_apriltags 检测 tag36h11 标签。
    4. 只保留 target_tag_id 指定的目标。
    5. 对目标中心 u、v 进行一阶低通滤波。
    6. 发布 [valid, u, v]。
    7. 在预览图像中绘制标签边框、中心和检测信息。

备注：
    1. 当前版本只为基础单目 IBVS 发布 u、v，不计算深度、XYZ、
       R/t 位姿矩阵或相机到机械臂的坐标变换。
    2. 当前实验使用 tag36h11、ID 0，推荐打印标签最外侧正方形
       实际边长为 12 cm，以提高约 1 m 距离下快速运动时的检测稳定性。
    3. 未检测到目标或相机读取失败时发布 [0.0, 0.0, 0.0]。
       Simulink 必须先检查 valid；valid=0.0 时必须输出六维零速度。
    4. 本节点只使用 pupil_apriltags，不提供其他检测后端回退。
       如果依赖未安装，节点会在启动时直接给出安装提示。
    5. 运行本节点时不能同时启动原有 visual_servo_law，否则两个节点
       会同时占用 USB 相机，并同时向控制链路发送视觉信息。
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
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

    def reset_tracking_state(self):
        """
        清除目标丢失前保存的检测和滤波状态。
        """

        self.last_corners = None
        self.last_raw_center = None
        self.last_filtered_center = None

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
        image
    ):
        """
        检测 RGB 图像中的指定 AprilTag。

        检测成功时返回 [u, v]，失败时返回 None。
        """

        if image is None:
            self.reset_tracking_state()
            return None

        if (
            image.ndim != 3 or
            image.shape[2] != 3
        ):
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
            self.reset_tracking_state()
            return None

        filtered_center = self.filter_center(
            raw_center
        )

        self.last_raw_center = raw_center.copy()
        self.last_corners = corners.copy()

        feature = np.asarray(
            [
                float(filtered_center[0]),
                float(filtered_center[1]),
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
        从 ROS 2 参数初始化相机、检测器和发布器。
        """

        super().__init__("apriltag_detector")

        # =====================================================
        # 声明参数
        # =====================================================

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 60.0)

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

        self.declare_parameter(
            "target_position_topic",
            "/apriltag_detector/target_position"
        )
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

        self.target_position_topic = str(
            self.get_parameter(
                "target_position_topic"
            ).value
        )
        self.show_window = bool(
            self.get_parameter(
                "show_window"
            ).value
        )

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
        )

        self.camera = USBCamera(
            camera_index=self.camera_index,
            width=self.camera_width,
            height=self.camera_height,
            fps=int(round(self.camera_fps))
        )

        # =====================================================
        # ROS 通信和运行状态
        # =====================================================

        self.position_publisher = self.create_publisher(
            Float64MultiArray,
            self.target_position_topic,
            1
        )

        self.target_was_detected = False
        # 检测相机实时读取的频率
        # =====================================================
        # 统计 camera.read() 成功返回图像的实际频率。
        self.successful_frame_count = 0
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
            f"topic={self.target_position_topic}."
        )


    def validate_parameters(self):
        """
        检查相机和 ROS 2 节点参数是否合法。
        """

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

        if not self.target_position_topic:
            raise ValueError(
                "target_position_topic 不能为空。"
            )

    def publish_target(
        self,
        feature
    ):
        """
        发布一次有效的 AprilTag 中心位置。
        """

        message = Float64MultiArray()
        message.data = [
            1.0,
            float(feature[0]),
            float(feature[1]),
        ]

        self.position_publisher.publish(
            message
        )

    def publish_invalid_target(self):
        """
        发布一次目标无效消息。
        """

        message = Float64MultiArray()
        message.data = [
            0.0,
            0.0,
            0.0,
        ]

        self.position_publisher.publish(
            message
        )

    def draw_detection(
        self,
        image,
        feature
    ):
        """
        绘制标签边框、中心和当前检测信息。
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

        cv2.imshow(
            "AprilTag Detector",
            display_image
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            self.publish_invalid_target()
            rclpy.shutdown()

    def camera_callback(self):
        """
        读取一帧图像、检测目标并发布结果。
        """

        image = self.camera.read()

        if image is None:
            self.detector.reset_tracking_state()
            self.publish_invalid_target()

            if self.target_was_detected:
                self.get_logger().warn(
                    "Camera image is unavailable. "
                    "Publishing invalid target."
                )

            self.target_was_detected = False
            return

        self.successful_frame_count += 1
        fps_measurement_now = time.monotonic()
        fps_elapsed = (
            fps_measurement_now -
            self.fps_measurement_start
        )

        if fps_elapsed >= 20.0:
            actual_camera_fps = (
                self.successful_frame_count /
                fps_elapsed
            )

            self.get_logger().info(
                "Successful camera-read FPS: "
                f"{actual_camera_fps:.2f}"
            )

            self.successful_frame_count = 0
            self.fps_measurement_start = (
                fps_measurement_now
            )

        feature = self.detector.detect(
            image
        )

        if feature is None:
            self.publish_invalid_target()

            if self.target_was_detected:
                self.get_logger().warn(
                    "Target lost. "
                    "Publishing invalid target."
                )

            self.target_was_detected = False
        else:
            self.publish_target(
                feature
            )

            if not self.target_was_detected:
                self.get_logger().info(
                    "Target detected."
                )

            self.target_was_detected = True

        if self.show_window:
            self.draw_detection(
                image,
                feature
            )

    def destroy_node(self):
        """
        释放相机和 OpenCV 窗口，然后销毁 ROS 2 节点。
        """

        if rclpy.ok():
            self.publish_invalid_target()

        if self.camera is not None:
            self.camera.release()

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
