#!/usr/bin/env python3
"""Map camera-frame velocity commands to safe FR3 joint velocities."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from velocity_servo_tag.mapper_control import (
    camera_velocity_is_zero,
    compute_centering_velocity,
    extract_ordered_joint_positions,
    input_freshness_status,
    limit_nullspace_acceleration,
    limit_nullspace_velocity,
    singularity_gate,
    task_speed_gate,
    validate_camera_velocity,
)
from velocity_servo_tag.robot_kinematics import FrankaKinematics


NUM_JOINTS = 7
JOINT_NAMES = [f"fr3_joint{index}" for index in range(1, 8)]

# Keep the existing main-task damping policy unchanged.
SIGMA_MIN_CRITICAL = 0.03
SIGMA_MIN_WARNING = 0.08
DAMPING_CRITICAL = 0.08
DAMPING_WARNING = 0.03
DAMPING_NORMAL = 0.005

DIAGNOSTIC_LOG_PERIOD_SEC = 1.0
SAFETY_LOG_PERIOD_SEC = 1.0


class VelocityMapperNode(Node):
    """Camera-twist to joint-velocity mapper with independent watchdogs."""

    def __init__(self):
        super().__init__("velocity_mapper_node")

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("end_effector_frame", "fr3_hand_tcp")
        self.declare_parameter(
            "T_end_effector_camera",
            [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        )
        self.declare_parameter("dry_run", False)
        self.declare_parameter("joint_state_topic", "/franka/joint_states")
        self.declare_parameter("visual_velocity_topic", "/simulink/camera_velocity")
        self.declare_parameter(
            "command_topic",
            "/velocity_mapper_node/target_joints_velocities",
        )

        # P0 input watchdog and zero-command parameters.
        self.declare_parameter("joint_state_timeout_sec", 0.10)
        self.declare_parameter("visual_velocity_timeout_sec", 0.10)
        self.declare_parameter("mapper_watchdog_rate_hz", 100.0)
        self.declare_parameter("camera_velocity_zero_epsilon", 1.0e-6)

        # P2 engineered null-space task parameters.
        self.declare_parameter("nullspace_activation_start_ratio", 0.80)
        self.declare_parameter("nullspace_activation_full_ratio", 0.95)
        self.declare_parameter("nullspace_centering_gain", 0.10)
        self.declare_parameter("nullspace_task_speed_fade_start", 0.02)
        self.declare_parameter("nullspace_task_speed_disable", 0.10)
        self.declare_parameter("nullspace_sigma_disable", 0.03)
        self.declare_parameter("nullspace_sigma_full", 0.08)
        self.declare_parameter("nullspace_max_joint_velocity", 0.10)
        self.declare_parameter("nullspace_max_joint_acceleration", 0.30)

        urdf_path = str(self.get_parameter("urdf_path").value)
        if not urdf_path:
            urdf_path = os.path.join(
                get_package_share_directory("velocity_servo_tag"),
                "config",
                "urdf",
                "fr3.urdf",
            )

        end_effector_frame = str(
            self.get_parameter("end_effector_frame").value
        )
        transform_values = self.get_parameter(
            "T_end_effector_camera"
        ).value
        if len(transform_values) != 16:
            raise ValueError("T_end_effector_camera must contain 16 values.")

        self.T_e_c = np.asarray(transform_values, dtype=float).reshape(4, 4)
        if not np.all(np.isfinite(self.T_e_c)):
            raise ValueError("T_end_effector_camera contains NaN or Inf.")

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.joint_state_timeout_sec = self._positive_parameter(
            "joint_state_timeout_sec", 0.10
        )
        self.visual_velocity_timeout_sec = self._positive_parameter(
            "visual_velocity_timeout_sec", 0.10
        )
        self.mapper_watchdog_rate_hz = self._positive_parameter(
            "mapper_watchdog_rate_hz", 100.0
        )
        self.camera_velocity_zero_epsilon = self._positive_parameter(
            "camera_velocity_zero_epsilon", 1.0e-6
        )
        (
            self.nullspace_activation_start_ratio,
            self.nullspace_activation_full_ratio,
        ) = self._ratio_pair(
            "nullspace_activation_start_ratio",
            "nullspace_activation_full_ratio",
            0.80,
            0.95,
            upper_bound=1.0,
        )
        self.nullspace_centering_gain = self._positive_parameter(
            "nullspace_centering_gain", 0.10
        )
        (
            self.nullspace_task_speed_fade_start,
            self.nullspace_task_speed_disable,
        ) = self._ratio_pair(
            "nullspace_task_speed_fade_start",
            "nullspace_task_speed_disable",
            0.02,
            0.10,
            upper_bound=None,
        )
        (
            self.nullspace_sigma_disable,
            self.nullspace_sigma_full,
        ) = self._ratio_pair(
            "nullspace_sigma_disable",
            "nullspace_sigma_full",
            0.03,
            0.08,
            upper_bound=None,
        )
        self.nullspace_max_joint_velocity = self._positive_parameter(
            "nullspace_max_joint_velocity", 0.10
        )
        self.nullspace_max_joint_acceleration = self._positive_parameter(
            "nullspace_max_joint_acceleration", 0.30
        )

        self.q_min, self.q_max = self.read_joint_limits(urdf_path)
        self.q_mid = 0.5 * (self.q_min + self.q_max)
        self.q_half_range = 0.5 * (self.q_max - self.q_min)
        if (
            not np.all(np.isfinite(self.q_half_range))
            or np.any(self.q_half_range <= 0.0)
        ):
            raise ValueError("URDF joint ranges are invalid.")

        self.kinematics = FrankaKinematics(
            urdf_path=urdf_path,
            end_effector_frame=end_effector_frame,
        )

        # The last legal samples are retained for diagnostics, while the
        # validity flags prevent an illegal sample from being used.
        self.last_joint_state_time = None
        self.last_visual_velocity_time = None
        self.has_valid_joint_state = False
        self.has_valid_visual_velocity = False
        self.latest_q = None
        self.latest_camera_velocity = None

        self.previous_nullspace_velocity = np.zeros(NUM_JOINTS, dtype=float)
        self.previous_nullspace_update_time = None
        self.in_safe_state = True
        self.safe_state_reason = "startup"
        self._last_log_times = {}

        joint_state_topic = str(
            self.get_parameter("joint_state_topic").value
        )
        visual_velocity_topic = str(
            self.get_parameter("visual_velocity_topic").value
        )
        command_topic = str(self.get_parameter("command_topic").value)

        self.joint_state_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10,
        )
        self.visual_velocity_subscription = self.create_subscription(
            Float64MultiArray,
            visual_velocity_topic,
            self.visual_velocity_callback,
            10,
        )
        self.publisher = self.create_publisher(
            Float64MultiArray,
            command_topic,
            10,
        )
        self.watchdog_timer = self.create_timer(
            1.0 / self.mapper_watchdog_rate_hz,
            self.watchdog_callback,
        )

        self.get_logger().info(
            "VelocityMapperNode started | "
            f"dry_run={self.dry_run} | "
            f"joint_timeout={self.joint_state_timeout_sec:.3f}s | "
            f"visual_timeout={self.visual_velocity_timeout_sec:.3f}s | "
            f"watchdog={self.mapper_watchdog_rate_hz:.1f}Hz"
        )
        self.get_logger().info(
            "Null-space safety | "
            f"activation={self.nullspace_activation_start_ratio:.2f}->"
            f"{self.nullspace_activation_full_ratio:.2f} | "
            f"task_fade={self.nullspace_task_speed_fade_start:.3f}->"
            f"{self.nullspace_task_speed_disable:.3f}m/s | "
            f"sigma={self.nullspace_sigma_disable:.3f}->"
            f"{self.nullspace_sigma_full:.3f} | "
            f"velocity_limit={self.nullspace_max_joint_velocity:.3f}rad/s | "
            f"acceleration_limit="
            f"{self.nullspace_max_joint_acceleration:.3f}rad/s^2"
        )

    def _positive_parameter(self, name, default):
        value = float(self.get_parameter(name).value)
        if np.isfinite(value) and value > 0.0:
            return value

        self.get_logger().warning(
            f"Invalid {name}={value!r}; using safe default {default}."
        )
        return float(default)

    def _ratio_pair(
        self,
        start_name,
        full_name,
        default_start,
        default_full,
        *,
        upper_bound,
    ):
        start = float(self.get_parameter(start_name).value)
        full = float(self.get_parameter(full_name).value)
        valid = (
            np.isfinite(start)
            and np.isfinite(full)
            and 0.0 <= start < full
            and (upper_bound is None or full <= upper_bound)
        )
        if valid:
            return start, full

        self.get_logger().warning(
            f"Invalid {start_name}/{full_name}=({start!r}, {full!r}); "
            f"using safe defaults ({default_start}, {default_full})."
        )
        return float(default_start), float(default_full)

    @staticmethod
    def read_joint_limits(urdf_path):
        root = ET.parse(urdf_path).getroot()
        q_min = []
        q_max = []

        for joint_name in JOINT_NAMES:
            joint = root.find(f".//joint[@name='{joint_name}']")
            if joint is None:
                raise ValueError(f"URDF does not contain {joint_name}.")

            limit = joint.find("limit")
            if (
                limit is None
                or "lower" not in limit.attrib
                or "upper" not in limit.attrib
            ):
                raise ValueError(f"{joint_name} has no valid position limits.")

            q_min.append(float(limit.attrib["lower"]))
            q_max.append(float(limit.attrib["upper"]))

        return np.asarray(q_min, dtype=float), np.asarray(q_max, dtype=float)

    def _log_throttled(self, key, message, level="warning", period=None):
        if period is None:
            period = SAFETY_LOG_PERIOD_SEC

        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._last_log_times.get(key)
        elapsed = None if last_ns is None else (now_ns - last_ns) * 1.0e-9
        if last_ns is not None and elapsed is not None and 0.0 <= elapsed < period:
            return

        self._last_log_times[key] = now_ns
        getattr(self.get_logger(), level)(message)

    @staticmethod
    def _age_seconds(now, timestamp):
        if timestamp is None:
            return None
        return float((now - timestamp).nanoseconds * 1.0e-9)

    def _input_status(self, now):
        joint_age = self._age_seconds(now, self.last_joint_state_time)
        visual_age = self._age_seconds(now, self.last_visual_velocity_time)
        ready, reason = input_freshness_status(
            has_joint_state=self.has_valid_joint_state,
            joint_state_age=joint_age,
            has_visual_velocity=self.has_valid_visual_velocity,
            visual_velocity_age=visual_age,
            joint_state_timeout=self.joint_state_timeout_sec,
            visual_velocity_timeout=self.visual_velocity_timeout_sec,
        )
        return ready, reason, joint_age, visual_age

    def reset_nullspace_state(self):
        self.previous_nullspace_velocity.fill(0.0)
        self.previous_nullspace_update_time = None

    def publish_joint_velocity(self, velocity, *, force=False):
        # dry_run suppresses motion commands, but safety zero targets are still
        # published so the mapper's output contract remains fail-safe.
        if self.dry_run and not force:
            return

        message = Float64MultiArray()
        message.data = np.asarray(
            velocity,
            dtype=float,
        ).reshape(NUM_JOINTS).tolist()
        self.publisher.publish(message)

    def enter_safe_state(self, reason, message, *, level="warning"):
        """Immediately publish a zero target and clear null-space history."""

        dq_task = np.zeros(NUM_JOINTS, dtype=float)
        dq_ns = np.zeros(NUM_JOINTS, dtype=float)
        dq_output = dq_task + dq_ns
        self.reset_nullspace_state()
        self.in_safe_state = True
        self.safe_state_reason = reason
        self.publish_joint_velocity(dq_output, force=True)
        self._log_throttled(reason, message, level=level)
        return dq_output

    def joint_state_callback(self, message):
        q = extract_ordered_joint_positions(
            message.name,
            message.position,
            JOINT_NAMES,
        )
        if q is None:
            # Keep the last legal q for diagnostics, but prohibit its use.
            self.has_valid_joint_state = False
            self.enter_safe_state(
                "invalid_joint_state",
                "Invalid JointState: expected seven finite FR3 joint positions. "
                "Publishing zero joint velocity.",
            )
            return

        self.latest_q = q
        self.last_joint_state_time = self.get_clock().now()
        self.has_valid_joint_state = True

    @staticmethod
    def select_damping(sigma_min):
        if sigma_min < SIGMA_MIN_CRITICAL:
            return DAMPING_CRITICAL
        if sigma_min < SIGMA_MIN_WARNING:
            return DAMPING_WARNING
        return DAMPING_NORMAL

    @staticmethod
    def calculate_damped_pseudoinverse(jacobian, damping):
        """Keep the existing DLS formula without explicitly taking an inverse."""

        task_dimension = jacobian.shape[0]
        regularized_matrix = (
            jacobian @ jacobian.T
            + damping**2 * np.eye(task_dimension)
        )
        return np.linalg.solve(regularized_matrix, jacobian).T

    def _nullspace_dt(self, now):
        nominal_dt = 1.0 / self.mapper_watchdog_rate_hz
        if self.previous_nullspace_update_time is None:
            dt = nominal_dt
        else:
            dt = self._age_seconds(now, self.previous_nullspace_update_time)

        self.previous_nullspace_update_time = now
        maximum_dt = max(5.0 * nominal_dt, 0.05)
        return dt, nominal_dt, maximum_dt

    def visual_velocity_callback(self, message):
        camera_velocity = validate_camera_velocity(message.data)
        if camera_velocity is None:
            # Keep the last legal command for diagnostics, but prohibit its use.
            self.has_valid_visual_velocity = False
            self.enter_safe_state(
                "invalid_visual_velocity",
                "Invalid camera velocity: expected at least six finite values. "
                "Publishing zero joint velocity.",
            )
            return

        now = self.get_clock().now()
        self.latest_camera_velocity = camera_velocity
        self.last_visual_velocity_time = now
        self.has_valid_visual_velocity = True

        ready, reason, _, _ = self._input_status(now)
        if not ready:
            self._enter_input_safety(reason)
            return

        if camera_velocity_is_zero(
            camera_velocity,
            self.camera_velocity_zero_epsilon,
        ):
            self.enter_safe_state(
                "zero_visual_velocity",
                "Camera velocity is a zero command; main and null-space "
                "velocities are forced to zero.",
                level="info",
            )
            return

        self._compute_and_publish(camera_velocity, now)

    def _enter_input_safety(self, reason):
        messages = {
            "missing_joint_state": (
                "No valid JointState has been received; publishing zero joint velocity."
            ),
            "invalid_joint_state_age": (
                "JointState age is invalid; publishing zero joint velocity."
            ),
            "joint_state_timeout": (
                "JointState timeout; stale joint positions will not be used."
            ),
            "missing_visual_velocity": (
                "No valid visual velocity has been received; publishing zero joint velocity."
            ),
            "invalid_visual_velocity_age": (
                "Visual velocity age is invalid; publishing zero joint velocity."
            ),
            "visual_velocity_timeout": (
                "Visual velocity timeout; publishing zero joint velocity."
            ),
        }
        self.enter_safe_state(reason, messages.get(reason, reason))

    def watchdog_callback(self):
        """Independently force zero when either upstream input stops arriving."""

        now = self.get_clock().now()
        ready, reason, _, _ = self._input_status(now)
        if not ready:
            self._enter_input_safety(reason)
            return

        if self.latest_camera_velocity is None:
            self._enter_input_safety("missing_visual_velocity")
            return

        if camera_velocity_is_zero(
            self.latest_camera_velocity,
            self.camera_velocity_zero_epsilon,
        ):
            self.enter_safe_state(
                "zero_visual_velocity",
                "Camera velocity remains zero; null-space motion remains disabled.",
                level="info",
            )

    def _compute_and_publish(self, camera_velocity, now):
        if self.latest_q is None:
            self._enter_input_safety("missing_joint_state")
            return

        q = self.latest_q.copy()
        if not np.all(np.isfinite(q)):
            self.has_valid_joint_state = False
            self.enter_safe_state(
                "nonfinite_joint_state",
                "Stored joint state is non-finite; publishing zero joint velocity.",
            )
            return

        R_e_c = self.T_e_c[:3, :3]
        t_e_c = self.T_e_c[:3, 3]
        t_skew = np.asarray(
            [
                [0.0, -t_e_c[2], t_e_c[1]],
                [t_e_c[2], 0.0, -t_e_c[0]],
                [-t_e_c[1], t_e_c[0], 0.0],
            ],
            dtype=float,
        )
        adjoint_e_c = np.zeros((6, 6), dtype=float)
        adjoint_e_c[:3, :3] = R_e_c
        adjoint_e_c[:3, 3:] = t_skew @ R_e_c
        adjoint_e_c[3:, 3:] = R_e_c
        V_e = adjoint_e_c @ camera_velocity

        try:
            jacobian = np.asarray(
                self.kinematics.compute_jacobian(q),
                dtype=float,
            )
        except Exception as error:  # Pinocchio may raise several exception types.
            self.enter_safe_state(
                "jacobian_failure",
                f"Jacobian calculation failed ({error}); publishing zero.",
            )
            return

        if jacobian.shape != (6, 7) or not np.all(np.isfinite(jacobian)):
            self.enter_safe_state(
                "invalid_jacobian",
                "Jacobian is not a finite 6x7 matrix; publishing zero.",
            )
            return

        try:
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
        except np.linalg.LinAlgError as error:
            self.enter_safe_state(
                "jacobian_svd_failure",
                f"Jacobian SVD failed ({error}); publishing zero.",
            )
            return

        sigma_max = float(singular_values[0])
        sigma_min = float(singular_values[-1])
        damping = self.select_damping(sigma_min)

        try:
            J_dls = self.calculate_damped_pseudoinverse(jacobian, damping)
        except np.linalg.LinAlgError as error:
            self.enter_safe_state(
                "dls_solve_failure",
                f"DLS solve failed ({error}); publishing zero.",
            )
            return

        dq_task = J_dls @ V_e

        try:
            dq_center_raw, activation, _ = compute_centering_velocity(
                q,
                self.q_mid,
                self.q_half_range,
                self.nullspace_activation_start_ratio,
                self.nullspace_activation_full_ratio,
                self.nullspace_centering_gain,
            )
        except ValueError as error:
            self.enter_safe_state(
                "nullspace_input_failure",
                f"Null-space input is invalid ({error}); publishing zero.",
            )
            return

        # With a damped pseudoinverse this is only an approximate null-space
        # projector: J @ N_dls is not assumed to be exactly zero.
        N_dls = np.eye(NUM_JOINTS) - J_dls @ jacobian

        # The current main model commands camera XY translation only. If Z or
        # angular velocity is enabled later, replace this with a weighted norm
        # rather than mixing m/s and rad/s without scaling.
        task_speed = float(np.linalg.norm(camera_velocity[:2]))
        g_task = task_speed_gate(
            task_speed,
            self.nullspace_task_speed_fade_start,
            self.nullspace_task_speed_disable,
        )
        g_sigma = singularity_gate(
            sigma_min,
            self.nullspace_sigma_disable,
            self.nullspace_sigma_full,
        )
        dq_ns_unlimited = g_task * g_sigma * (N_dls @ dq_center_raw)

        # A disabled gate means exactly zero, without retaining an acceleration-
        # limited residual from a previously active null-space command.
        if (
            g_task <= 0.0
            or g_sigma <= 0.0
            or np.all(activation <= 0.0)
        ):
            dq_ns = np.zeros(NUM_JOINTS, dtype=float)
            self.reset_nullspace_state()
        else:
            try:
                dq_ns_target = limit_nullspace_velocity(
                    dq_ns_unlimited,
                    self.nullspace_max_joint_velocity,
                )
                dt, nominal_dt, maximum_dt = self._nullspace_dt(now)
                dq_ns = limit_nullspace_acceleration(
                    self.previous_nullspace_velocity,
                    dq_ns_target,
                    self.nullspace_max_joint_acceleration,
                    dt,
                    nominal_dt,
                    maximum_dt,
                )
            except ValueError as error:
                self.enter_safe_state(
                    "nullspace_limit_failure",
                    f"Null-space limiter failed ({error}); publishing zero.",
                )
                return

            self.previous_nullspace_velocity = dq_ns.copy()

        dq_output = dq_task + dq_ns
        if (
            dq_task.shape != (NUM_JOINTS,)
            or dq_ns.shape != (NUM_JOINTS,)
            or dq_output.shape != (NUM_JOINTS,)
            or not np.all(np.isfinite(dq_task))
            or not np.all(np.isfinite(dq_ns))
            or not np.all(np.isfinite(dq_output))
        ):
            self.enter_safe_state(
                "nonfinite_output",
                "Non-finite mapper output was forced to seven-dimensional zero.",
            )
            return

        self.publish_joint_velocity(dq_output)
        if self.in_safe_state:
            self.get_logger().info(
                "Fresh valid inputs and successful kinematics restored; "
                "velocity mapping resumed."
            )
        self.in_safe_state = False
        self.safe_state_reason = None

        joint_age = self._age_seconds(now, self.last_joint_state_time)
        visual_age = self._age_seconds(now, self.last_visual_velocity_time)
        condition_number = sigma_max / max(sigma_min, 1.0e-9)
        self._log_throttled(
            "mapper_diagnostics",
            "Mapper diagnostics | "
            f"sigma_min={sigma_min:.6f} | condition={condition_number:.2f} | "
            f"damping={damping:.4f} | ||dq_task||={np.linalg.norm(dq_task):.5f} | "
            f"||dq_ns||={np.linalg.norm(dq_ns):.5f} | g_task={g_task:.3f} | "
            f"g_sigma={g_sigma:.3f} | max_activation={np.max(activation):.3f} | "
            f"joint_age={joint_age:.4f}s | visual_age={visual_age:.4f}s",
            level="debug",
            period=DIAGNOSTIC_LOG_PERIOD_SEC,
        )


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.enter_safe_state(
                "shutdown",
                "Velocity mapper is shutting down; publishing zero.",
                level="info",
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
