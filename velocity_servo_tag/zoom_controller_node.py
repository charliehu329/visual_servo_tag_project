"""ROS 2 fx/fy-to-zoom controller with safe dry-run and serial modes."""

from __future__ import annotations

import math
import re
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from velocity_servo_tag.lens_model import ZoomIntrinsicsModel


class ZoomControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("zoom_controller_node")
        self._declare_parameters()
        self.model = ZoomIntrinsicsModel(
            self.get_parameter("zoom_samples").value,
            self.get_parameter("fx_samples").value,
            self.get_parameter("fy_samples").value,
            self.get_parameter("cx_samples").value,
            self.get_parameter("cy_samples").value,
            self.get_parameter("distortion_samples_flat").value,
        )
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.max_disagreement = float(
            self.get_parameter("max_fx_fy_disagreement_steps").value
        )
        self.deadband = int(self.get_parameter("zoom_deadband_steps").value)
        self.command_timeout = float(
            self.get_parameter("serial_command_timeout_sec").value
        )
        self.frame_id = str(self.get_parameter("camera_frame_id").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.state_publisher = self.create_publisher(
            Float64MultiArray, self.state_topic, 10
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, self.camera_info_topic, 10
        )
        self.create_subscription(
            Float64MultiArray, self.target_topic, self._target_callback, 10
        )
        self.create_service(Trigger, "~/home", self._home_callback)
        self.serial = None
        self._serial_lock = threading.Lock()
        self.homed = self.dry_run
        self.current_zoom: int | None = None
        self._pending = None
        self._condition = threading.Condition()
        self._stopping = False
        if not self.dry_run:
            self._open_serial()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"ZoomControllerNode started: dry_run={self.dry_run}, "
            f"target={self.target_topic}. Input data=[fx_target, fy_target]."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("dry_run", True)
        self.declare_parameter("serial_port", "/dev/serial/by-id/CHANGE_ME")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("serial_command_timeout_sec", 35.0)
        self.declare_parameter("zoom_deadband_steps", 3)
        self.declare_parameter("max_fx_fy_disagreement_steps", 20.0)
        self.declare_parameter("target_topic", "/lens/target_intrinsics")
        self.declare_parameter("state_topic", "/lens/zoom_state")
        self.declare_parameter("camera_info_topic", "/lens/camera_info")
        self.declare_parameter("camera_frame_id", "camera_optical_frame")
        self.declare_parameter("image_width", 1920)
        self.declare_parameter("image_height", 1080)
        self.declare_parameter("zoom_samples", [0., 400., 800., 1200., 1600., 2000., 2400.])
        self.declare_parameter("fx_samples", [2035.797529, 2331.545143, 2713.737027, 3137.392785, 3684.865723, 4505.996199, 5494.414672])
        self.declare_parameter("fy_samples", [2034.742218, 2331.622073, 2714.807451, 3138.780385, 3683.854745, 4503.318979, 5490.231460])
        self.declare_parameter("cx_samples", [946.313693, 931.606961, 945.418505, 930.987463, 930.964585, 928.012695, 894.846174])
        self.declare_parameter("cy_samples", [749.155224, 766.482076, 774.908282, 766.529524, 757.853146, 756.246991, 746.485232])
        self.declare_parameter("distortion_samples_flat", [-0.258094321,0.261993622,0.000111694,0.002053981,0., -0.235809222,0.428278290,0.001437905,-0.001850212,0., -0.188571553,0.616540501,0.002934832,-0.001131934,0., -0.166386778,0.918454335,0.003322227,-0.003138638,0., -0.115889778,0.972427637,0.002954982,-0.001741346,0., -0.087717949,1.419104638,0.003376276,-0.002862911,0., -0.080995647,1.287880442,0.002501424,-0.005151698,0.])

    def _open_serial(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required when dry_run=false") from exc
        port = str(self.get_parameter("serial_port").value)
        baud = int(self.get_parameter("serial_baud").value)
        self.serial = serial.Serial(port, baud, timeout=0.05, write_timeout=2.0)
        time.sleep(2.0)
        self.serial.reset_input_buffer()

    def _target_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) < 2:
            self.get_logger().error("target must contain [fx_target, fy_target]")
            return
        fx, fy = float(message.data[0]), float(message.data[1])
        if not math.isfinite(fx) or not math.isfinite(fy):
            self.get_logger().error("target fx/fy must be finite")
            return
        try:
            solution = self.model.solve_fx_fy(fx, fy, self.max_disagreement)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        target_z = int(round(solution.zoom_steps))
        with self._condition:
            self._pending = (fx, fy, target_z, solution)
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
            fx, fy, target_z, solution = request
            if not self.homed:
                self.get_logger().error(
                    "zoom command rejected: call /zoom_controller_node/home first"
                )
                continue
            if self.current_zoom is not None and abs(target_z-self.current_zoom) <= self.deadband:
                target_z = self.current_zoom
            elif not self.dry_run:
                try:
                    self._send_and_wait(
                        f"ZPOS {target_z}",
                        re.compile(rf"^DONE,ZOOM_POS={target_z}$"),
                    )
                except Exception as exc:
                    self.get_logger().error(f"ZPOS failed: {exc}")
                    continue
            self.current_zoom = target_z
            actual = self.model.intrinsics(target_z)
            self._publish_state(fx, fy, solution, actual)

    def _send_and_wait(self, command: str, pattern: re.Pattern) -> None:
        with self._serial_lock:
            self._send_and_wait_locked(command, pattern)

    def _send_and_wait_locked(self, command: str, pattern: re.Pattern) -> None:
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
            response.success = True
            response.message = "dry-run home accepted"
            return response
        try:
            self._send_and_wait("HOMEZF", re.compile(r"^HOMING_OK,Z=0,F=0$"))
            self.homed = True
            self.current_zoom = 0
            response.success = True
            response.message = "HOMEZF complete"
        except Exception as exc:
            self.homed = False
            response.success = False
            response.message = str(exc)
        return response

    def _publish_state(self, target_fx, target_fy, solution, actual) -> None:
        state = Float64MultiArray()
        state.data = [
            1.0, target_fx, target_fy,
            solution.z_from_fx, solution.z_from_fy,
            float(self.current_zoom), actual.fx, actual.fy,
            *actual.distortion,
        ]
        self.state_publisher.publish(state)
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = self.frame_id
        info.width = int(self.get_parameter("image_width").value)
        info.height = int(self.get_parameter("image_height").value)
        info.distortion_model = "plumb_bob"
        info.d = list(actual.distortion)
        info.k = [actual.fx,0.,actual.cx, 0.,actual.fy,actual.cy, 0.,0.,1.]
        info.r = [1.,0.,0., 0.,1.,0., 0.,0.,1.]
        info.p = [actual.fx,0.,actual.cx,0., 0.,actual.fy,actual.cy,0., 0.,0.,1.,0.]
        self.camera_info_publisher.publish(info)
        self.get_logger().info(
            f"fx/fy target=({target_fx:.2f},{target_fy:.2f}) -> "
            f"ZPOS {self.current_zoom}, predicted=({actual.fx:.2f},{actual.fy:.2f})"
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
