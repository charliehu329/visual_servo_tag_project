#!/usr/bin/env python3
"""
camera.py

功能：
    使用OpenCV调用普通USB摄像头。
    打开摄像头、读取实时图像，并将OpenCV默认的BGR图像
    转换为RGB图像，供绿色小球检测器使用。

接口：
    USBCamera.read()
    USBCamera.is_opened()
    USBCamera.release()

输入：
    camera_index:
        摄像头设备编号。

        0通常对应：
            /dev/video0

        1通常对应：
            /dev/video1

    width:
        图像宽度，单位pixel。

    height:
        图像高度，单位pixel。

    fps:
        期望采集帧率，单位Hz。

输出：
    read():
        读取成功时返回RGB图像：

        image:
            numpy.ndarray
            shape = (height, width, 3)

        读取失败时返回None。
"""

import cv2


class USBCamera:
    """
    普通USB摄像头读取类。
    """

    def __init__(
        self,
        camera_index=3, #默认相机ID,在创建实例的时候传入新的ID
        width=640,
        height=480,
        fps=30
    ):
        """
        打开USB摄像头并设置采集参数。

        输入：
            camera_index:
                摄像头编号。

            width:
                图像宽度。

            height:
                图像高度。

            fps:
                期望帧率。
        """

        self.camera_index = camera_index

        # Ubuntu下优先使用V4L2后端打开摄像头
        self.capture = cv2.VideoCapture(
            camera_index,
            cv2.CAP_V4L2
        )

        if not self.capture.isOpened():
            self.capture.release()

            # V4L2打开失败时，使用OpenCV默认后端重试
            self.capture = cv2.VideoCapture(
                camera_index
            )

        if not self.capture.isOpened():
            raise RuntimeError(
                f"无法打开USB摄像头，camera_index={camera_index}"
            )
        
        self.capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )

        self.capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            width
        )

        self.capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            height
        )

        self.capture.set(
            cv2.CAP_PROP_FPS,
            fps
        )


        actual_width = int(
            self.capture.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )

        actual_height = int(
            self.capture.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )

        actual_fps = self.capture.get(
            cv2.CAP_PROP_FPS
        )

        print(
            "实际相机配置：",
            actual_width,
            "x",
            actual_height,
            "@",
            actual_fps,
            "FPS"
        )

    def is_opened(self):
        """
        检查摄像头是否已成功打开。

        输出：
            True:
                摄像头已打开。

            False:
                摄像头未打开。
        """

        return self.capture.isOpened()

    def read(self):
        """
        从USB摄像头读取一帧图像。

        输出：
            image_rgb:
                RGB格式图像。

            None:
                图像读取失败。
        """

        success, image_bgr = self.capture.read()

        if not success:
            return None

        # OpenCV默认读取BGR图像
        # 转换为detector需要的RGB图像
        image_rgb = cv2.cvtColor(
            image_bgr,
            cv2.COLOR_BGR2RGB
        )

        return image_rgb

    def release(self):
        """
        释放摄像头资源。
        """

        if self.capture is not None:
            self.capture.release()
