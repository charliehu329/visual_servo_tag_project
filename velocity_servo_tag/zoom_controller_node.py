"""ROS 2 camera-0 fx/fy-to-Z/F controller and dynamic CameraInfo publisher."""

from __future__ import annotations

import math
import os
import re
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from velocity_servo_tag.lens_model import ZoomLensModel


class ZoomControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("zoom_controller_node")
        self._declare_parameters()
        calibration_path = str(self.get_parameter("calibration_file").value)
        if not calibration_path:
            calibration_path = os.path.join(
                get_package_share_directory("velocity_servo_tag"),
                "config", "lens_calibration", "camera0_zoom_v2.json",
            )
        self.model = ZoomLensModel.from_json(calibration_path)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.max_disagreement = float(
            self.get_parameter("max_fx_fy_disagreement_steps").value
        )
        self.zoom_deadband = int(self.get_parameter("zoom_deadband_steps").value)
        self.focus_deadband = int(self.get_parameter("focus_deadband_steps").value)
        self.distance_timeout = float(self.get_parameter("distance_timeout_sec").value)
        self.command_timeout = float(
            self.get_parameter("serial_command_timeout_sec").value
        )
        self.frame_id = str(self.get_parameter("camera_frame_id").value)
        self.bootstrap_distance = float(
            self.get_parameter("bootstrap_distance_m").value
        )
        if not (
            self.model.distance_range[0]
            <= self.bootstrap_distance
            <= self.model.distance_range[1]
        ):
            raise ValueError("bootstrap_distance_m is outside calibration domain")
        self.bootstrap_focus = int(round(
            self.model.predict_control(0.0, self.bootstrap_distance).focus_steps
        ))
        self._last_distance_warning = 0.0

        self.state_publisher = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("state_topic").value), 10
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), 10
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("target_topic").value),
            self._target_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("distance_topic").value),
            self._distance_callback,
            10,
        )
        self.create_service(Trigger, "~/home", self._home_callback)

        self.serial = None
        self._serial_lock = threading.Lock()
        self.homed = self.dry_run
        self.current_zoom: int | None = 0 if self.dry_run else None
        self.current_focus: int | None = (
            self.bootstrap_focus if self.dry_run else None
        )
        self.latest_distance: float | None = None
        self.latest_distance_time: float | None = None
        self._pending = None
        self._condition = threading.Condition()
        self._stopping = False
        if not self.dry_run:
            self._open_serial()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"zoom controller ready: model={self.model.model_id}, dry_run={self.dry_run}, "
            "input=[fx_target_px, fy_target_px]"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("dry_run", True)
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("serial_port", "/dev/serial/by-id/CHANGE_ME")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("serial_command_timeout_sec", 35.0)
        self.declare_parameter("distance_timeout_sec", 0.5)
        self.declare_parameter("bootstrap_distance_m", 0.8)
        self.declare_parameter("zoom_deadband_steps", 3)
        self.declare_parameter("focus_deadband_steps", 3)
        self.declare_parameter("max_fx_fy_disagreement_steps", 80.0)
        self.declare_parameter("target_topic", "/lens/target_intrinsics")
        self.declare_parameter("distance_topic", "/apriltag_detector/target_position")
        self.declare_parameter("state_topic", "/lens/zoom_state")
        self.declare_parameter("camera_info_topic", "/lens/camera_info")
        self.declare_parameter("camera_frame_id", "camera_optical_frame")

    def _open_serial(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("python3-serial is required when dry_run=false") from exc
        self.serial = serial.Serial(
            str(self.get_parameter("serial_port").value),
            int(self.get_parameter("serial_baud").value),
            timeout=0.05,
            write_timeout=2.0,
        )
        time.sleep(2.0)
        self.serial.reset_input_buffer()

    def _distance_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) < 4 or message.data[0] < 0.5:
            return
        distance = float(message.data[3])
        if not math.isfinite(distance):
            return
        if not self.model.distance_range[0] <= distance <= self.model.distance_range[1]:
            self.latest_distance_time = None
            now = time.monotonic()
            if now - self._last_distance_warning >= 2.0:
                self.get_logger().warn(
                    f"distance {distance:.3f} m outside calibrated range "
                    f"{self.model.distance_range}; lens command disabled"
                )
                self._last_distance_warning = now
            return
        self.latest_distance = distance
        self.latest_distance_time = time.monotonic()
        if self.current_zoom is not None:
            self._publish_camera_info(self.current_zoom, distance)

    def _target_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) < 2:
            self.get_logger().error("target must contain [fx_target, fy_target]")
            return
        fx, fy = float(message.data[0]), float(message.data[1])
        if not math.isfinite(fx) or not math.isfinite(fy):
            self.get_logger().error("target fx/fy must be finite")
            return
        now = time.monotonic()
        if (
            self.latest_distance is None
            or self.latest_distance_time is None
            or now - self.latest_distance_time > self.distance_timeout
        ):
            self.get_logger().error("lens target rejected: no recent valid target distance")
            return
        try:
            solution = self.model.solve_fx_fy(
                fx, fy, self.latest_distance, self.max_disagreement
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        target_z = int(round(solution.zoom_steps))
        target_f = int(round(
            self.model.predict_control(target_z, self.latest_distance).focus_steps
        ))
        with self._condition:
            self._pending = (fx, fy, self.latest_distance, target_z, target_f, solution)
            self._condition.notify()

    def _worker_loop(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._pending is not None or self._stopping
                )
                if self._stopping:
                    return
                request = self._pending
                self._pending = None
            fx, fy, distance, target_z, target_f, solution = request
            if not self.homed:
                self.get_logger().error(
                    "lens command rejected: call /zoom_controller_node/home first"
                )
                continue
            try:
                if (
                    self.current_zoom is None
                    or abs(target_z - self.current_zoom) > self.zoom_deadband
                ):
                    if not self.dry_run:
                        self._move_axis("ZPOS", "ZOOM_POS", target_z)
                    self.current_zoom = target_z
                else:
                    target_z = self.current_zoom
                if (
                    self.current_focus is None
                    or abs(target_f - self.current_focus) > self.focus_deadband
                ):
                    if not self.dry_run:
                        self._move_axis("FPOS", "FOCUS_POS", target_f)
                    self.current_focus = target_f
                else:
                    target_f = self.current_focus
            except Exception as exc:
                self.get_logger().error(f"lens motor command failed: {exc}")
                continue
            self._publish_state(fx, fy, distance, target_z, target_f, solution)

    def _move_axis(self, command: str, response_name: str, target: int) -> None:
        self._send_and_wait(
            f"{command} {target}", re.compile(rf"^DONE,{response_name}={target}$")
        )

    def _send_and_wait(self, command: str, pattern: re.Pattern) -> None:
        with self._serial_lock:
            if self.serial is None:
                raise RuntimeError("serial port is not open")
            self.serial.reset_input_buffer()
            self.serial.write((command + "\n").encode("ascii"))
            self.serial.flush()
            deadline = time.monotonic() + self.command_timeout
            while time.monotonic() < deadline:
                line = self.serial.readline().decode("ascii", errors="replace").strip()
                if pattern.match(line):
                    return
                if line.startswith("ERR,"):
                    raise RuntimeError(line)
            raise TimeoutError(f"timeout waiting for {command}")

    def _home_callback(self, request, response):
        del request
        if self.dry_run:
            self.homed = True
            self.current_zoom = 0
            self.current_focus = self.bootstrap_focus
            self._publish_camera_info(0, self.bootstrap_distance)
            response.success = True
            response.message = (
                f"dry-run HOMEZF accepted; bootstrap F={self.bootstrap_focus}"
            )
            return response
        try:
            self._send_and_wait("HOMEZF", re.compile(r"^HOMING_OK,Z=0,F=0$"))
            self._move_axis("FPOS", "FOCUS_POS", self.bootstrap_focus)
            self.homed = True
            self.current_zoom = 0
            self.current_focus = self.bootstrap_focus
            self._publish_camera_info(0, self.bootstrap_distance)
            response.success = True
            response.message = (
                f"HOMEZF complete; bootstrap F={self.bootstrap_focus}"
            )
        except Exception as exc:
            self.homed = False
            response.success = False
            response.message = str(exc)
        return response

    def _publish_camera_info(self, zoom: int, distance: float) -> None:
        camera = self.model.predict_camera(zoom, distance)
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = self.frame_id
        info.width, info.height = self.model.resolution
        info.distortion_model = "plumb_bob"
        info.d = list(camera.distortion)
        info.k = [camera.fx, 0., camera.cx, 0., camera.fy, camera.cy, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [camera.fx, 0., camera.cx, 0., 0., camera.fy, camera.cy, 0., 0., 0., 1., 0.]
        self.camera_info_publisher.publish(info)

    def _publish_state(self, fx, fy, distance, zoom, focus, solution) -> None:
        control = self.model.predict_control(zoom, distance)
        camera = self.model.predict_camera(zoom, distance)
        state = Float64MultiArray()
        state.data = [
            1.0, distance, fx, fy, solution.z_from_fx, solution.z_from_fy,
            float(zoom), float(focus), control.fx, control.fy,
            camera.fx, camera.fy, *camera.distortion,
        ]
        self.state_publisher.publish(state)
        self._publish_camera_info(zoom, distance)
        self.get_logger().info(
            f"target=({fx:.2f},{fy:.2f}) d={distance:.3f} m -> "
            f"ZPOS {zoom}, FPOS {focus}"
        )

    def destroy_node(self):
        with self._condition:
            self._stopping = True
            self._condition.notify()
        if hasattr(self, "_worker"):
            self._worker.join(timeout=2.0)
        if self.serial is not None:
            try:
                self.serial.write(b"STOP\n")
                self.serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZoomControllerNode()
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
