#!/usr/bin/env python3
"""
slow_home_recovery.py

Franka FR3 缓慢恢复到安全初始关节姿态。

订阅：
    /franka/joint_states
        sensor_msgs/msg/JointState

发布：
    /simulink/target_joints_velocities
        std_msgs/msg/Float64MultiArray
        [dq1, dq2, dq3, dq4, dq5, dq6, dq7]

本节点不直接发布到底层控制器，而是沿用现有框架：
    slow_home_recovery
        -> /simulink/target_joints_velocities
        -> velocity_command_node
        -> /joint_velocity_example_controller/commands

安全策略：
1. 默认 execute=false，只发布零速度，不移动机器人。
2. 目标不是七关节全零，而是工程中已有的安全姿态：
       [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]
3. 启动后先持续发送零速度，确认关节状态稳定。
4. 速度、加速度和 jerk 均受限制。
5. 根据剩余距离计算制动速度，接近目标自动减速。
6. 关节状态超时、数值异常、存在其他目标速度发布者时立即停止。
7. 到达目标后持续发布零速度。
8. Ctrl+C 时尝试发布多次零速度。

重要：
- 运行本节点前必须停止 Simulink 视觉控制，确保本节点是
  /simulink/target_joints_velocities 的唯一外部发布者。
- 如果机器人当前处于 reflex/error 状态，必须先完成 Franka
  error recovery、硬件重新激活和速度控制器重新激活。
"""

import math
import time
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


NUM_JOINTS = 7

JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

# FR3 官方硬位置范围，单位 rad。
FR3_Q_MIN = np.asarray(
    [-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508],
    dtype=float,
)

FR3_Q_MAX = np.asarray(
    [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508],
    dtype=float,
)

DEFAULT_SAFE_Q = np.asarray(
    [
        0.0,
        -math.pi / 4.0,
        0.0,
        -3.0 * math.pi / 4.0,
        0.0,
        math.pi / 2.0,
        math.pi / 4.0,
    ],
    dtype=float,
)


class SlowHomeRecovery(Node):
    """缓慢、平滑地把 FR3 恢复到预设安全关节姿态。"""

    def __init__(self) -> None:
        super().__init__("slow_home_recovery")

        # -----------------------------------------------------
        # 参数
        # -----------------------------------------------------
        self.declare_parameter("execute", False)

        self.declare_parameter(
            "joint_state_topic",
            "/franka/joint_states",
        )
        self.declare_parameter(
            "output_topic",
            "/simulink/target_joints_velocities",
        )

        self.declare_parameter(
            "target_joint_positions",
            DEFAULT_SAFE_Q.tolist(),
        )

        self.declare_parameter("publish_rate_hz", 120.0)

        # 故意设置得很低，用于错误恢复。
        self.declare_parameter("max_joint_speed", 0.04)
        self.declare_parameter("max_joint_acceleration", 0.08)
        self.declare_parameter("max_joint_jerk", 0.40)

        self.declare_parameter("position_gain", 0.35)
        self.declare_parameter("position_tolerance", 0.008)
        self.declare_parameter("velocity_tolerance", 0.015)

        self.declare_parameter("joint_state_timeout_sec", 0.10)
        self.declare_parameter("initial_zero_hold_sec", 2.0)
        self.declare_parameter("final_zero_hold_sec", 2.0)

        # 目标与硬限位至少保留该余量。
        self.declare_parameter("joint_limit_margin", 0.08)

        self.execute = bool(
            self.get_parameter("execute").value
        )
        self.joint_state_topic = str(
            self.get_parameter("joint_state_topic").value
        )
        self.output_topic = str(
            self.get_parameter("output_topic").value
        )

        self.target_q = np.asarray(
            self.get_parameter(
                "target_joint_positions"
            ).value,
            dtype=float,
        ).reshape(-1)

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.max_joint_speed = float(
            self.get_parameter("max_joint_speed").value
        )
        self.max_joint_acceleration = float(
            self.get_parameter(
                "max_joint_acceleration"
            ).value
        )
        self.max_joint_jerk = float(
            self.get_parameter("max_joint_jerk").value
        )

        self.position_gain = float(
            self.get_parameter("position_gain").value
        )
        self.position_tolerance = float(
            self.get_parameter(
                "position_tolerance"
            ).value
        )
        self.velocity_tolerance = float(
            self.get_parameter(
                "velocity_tolerance"
            ).value
        )

        self.joint_state_timeout_sec = float(
            self.get_parameter(
                "joint_state_timeout_sec"
            ).value
        )
        self.initial_zero_hold_sec = float(
            self.get_parameter(
                "initial_zero_hold_sec"
            ).value
        )
        self.final_zero_hold_sec = float(
            self.get_parameter(
                "final_zero_hold_sec"
            ).value
        )
        self.joint_limit_margin = float(
            self.get_parameter(
                "joint_limit_margin"
            ).value
        )

        self.validate_parameters()

        # -----------------------------------------------------
        # 状态
        # -----------------------------------------------------
        self.current_q: Optional[np.ndarray] = None
        self.current_q_dot: Optional[np.ndarray] = None
        self.last_joint_state_time = None

        self.command_q_dot = np.zeros(NUM_JOINTS, dtype=float)
        self.command_q_ddot = np.zeros(NUM_JOINTS, dtype=float)

        self.last_update_monotonic = time.monotonic()
        self.state = "WAITING_FOR_JOINT_STATE"
        self.state_start_monotonic = time.monotonic()
        self.last_status_print = 0.0
        self.abort_reason: Optional[str] = None

        # -----------------------------------------------------
        # ROS 接口
        # -----------------------------------------------------
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            self.output_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            "SlowHomeRecovery started | "
            f"execute={self.execute} | "
            f"joint_state={self.joint_state_topic} | "
            f"output={self.output_topic}"
        )
        self.get_logger().info(
            "Target safe joint pose [rad]: "
            f"{np.round(self.target_q, 4).tolist()}"
        )
        self.get_logger().info(
            "Limits: "
            f"speed={self.max_joint_speed:.3f} rad/s, "
            f"acceleration={self.max_joint_acceleration:.3f} rad/s^2, "
            f"jerk={self.max_joint_jerk:.3f} rad/s^3"
        )

        if not self.execute:
            self.get_logger().warning(
                "execute=false: dry run only. "
                "The node will publish zero velocity."
            )

    def validate_parameters(self) -> None:
        """检查所有参数以及目标关节姿态。"""

        if self.target_q.shape != (NUM_JOINTS,):
            raise ValueError(
                "target_joint_positions must contain exactly 7 values."
            )

        if not np.all(np.isfinite(self.target_q)):
            raise ValueError(
                "target_joint_positions contains NaN or Inf."
            )

        positive_scalars = {
            "publish_rate_hz": self.publish_rate_hz,
            "max_joint_speed": self.max_joint_speed,
            "max_joint_acceleration":
                self.max_joint_acceleration,
            "max_joint_jerk": self.max_joint_jerk,
            "position_gain": self.position_gain,
            "position_tolerance": self.position_tolerance,
            "velocity_tolerance": self.velocity_tolerance,
            "joint_state_timeout_sec":
                self.joint_state_timeout_sec,
            "joint_limit_margin": self.joint_limit_margin,
        }

        for name, value in positive_scalars.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be positive and finite."
                )

        nonnegative_scalars = {
            "initial_zero_hold_sec":
                self.initial_zero_hold_sec,
            "final_zero_hold_sec":
                self.final_zero_hold_sec,
        }

        for name, value in nonnegative_scalars.items():
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{name} must be nonnegative and finite."
                )

        safe_min = FR3_Q_MIN + self.joint_limit_margin
        safe_max = FR3_Q_MAX - self.joint_limit_margin

        if np.any(self.target_q <= safe_min) or np.any(
            self.target_q >= safe_max
        ):
            raise ValueError(
                "Target joint pose is too close to an FR3 joint limit. "
                f"safe_min={safe_min.tolist()}, "
                f"safe_max={safe_max.tolist()}"
            )

    def joint_state_callback(self, message: JointState) -> None:
        """按 fr3_joint1...fr3_joint7 的顺序提取关节状态。"""

        if len(message.name) != len(message.position):
            self.abort(
                "JointState name and position lengths do not match."
            )
            return

        position_by_name: Dict[str, float] = {
            str(name): float(position)
            for name, position in zip(
                message.name,
                message.position,
            )
        }

        missing_names = [
            name
            for name in JOINT_NAMES
            if name not in position_by_name
        ]

        if missing_names:
            self.abort(
                "JointState is missing joints: "
                + ", ".join(missing_names)
            )
            return

        q = np.asarray(
            [
                position_by_name[name]
                for name in JOINT_NAMES
            ],
            dtype=float,
        )

        velocity_by_name: Dict[str, float] = {}

        if len(message.velocity) == len(message.name):
            velocity_by_name = {
                str(name): float(velocity)
                for name, velocity in zip(
                    message.name,
                    message.velocity,
                )
            }

        if all(
            name in velocity_by_name
            for name in JOINT_NAMES
        ):
            q_dot = np.asarray(
                [
                    velocity_by_name[name]
                    for name in JOINT_NAMES
                ],
                dtype=float,
            )
        else:
            q_dot = np.zeros(NUM_JOINTS, dtype=float)

        if not np.all(np.isfinite(q)) or not np.all(
            np.isfinite(q_dot)
        ):
            self.abort(
                "JointState contains NaN or Inf."
            )
            return

        if np.any(q <= FR3_Q_MIN) or np.any(q >= FR3_Q_MAX):
            self.abort(
                "Measured joint position is outside the FR3 hard limits."
            )
            return

        self.current_q = q
        self.current_q_dot = q_dot
        self.last_joint_state_time = self.get_clock().now()

    def publish_velocity(self, q_dot: np.ndarray) -> None:
        """发布7维目标关节速度。"""

        q_dot = np.asarray(
            q_dot,
            dtype=float,
        ).reshape(NUM_JOINTS)

        if not np.all(np.isfinite(q_dot)):
            q_dot = np.zeros(NUM_JOINTS, dtype=float)

        message = Float64MultiArray()
        message.data = q_dot.tolist()
        self.command_publisher.publish(message)

    def publish_zero(self) -> None:
        """发布零关节速度。"""

        self.command_q_dot.fill(0.0)
        self.command_q_ddot.fill(0.0)
        self.publish_velocity(self.command_q_dot)

    def joint_state_is_fresh(self) -> bool:
        """检查关节状态是否新鲜。"""

        if self.last_joint_state_time is None:
            return False

        age = (
            self.get_clock().now() -
            self.last_joint_state_time
        ).nanoseconds * 1e-9

        return (
            np.isfinite(age)
            and 0.0 <= age <= self.joint_state_timeout_sec
        )

    def other_publishers_exist(self) -> bool:
        """
        检查恢复话题上是否还有其他发布者。

        get_publishers_info_by_topic() 会包含本节点自身，
        因此数量大于1表示存在另一个发布者。
        """

        publisher_info = self.get_publishers_info_by_topic(
            self.output_topic
        )
        return len(publisher_info) > 1

    def transition(self, new_state: str) -> None:
        """切换内部状态机。"""

        self.state = new_state
        self.state_start_monotonic = time.monotonic()
        self.get_logger().info(
            f"Recovery state -> {new_state}"
        )

    def abort(self, reason: str) -> None:
        """进入安全停止状态。"""

        if self.abort_reason == reason:
            return

        self.abort_reason = reason
        self.state = "ABORTED"
        self.command_q_dot.fill(0.0)
        self.command_q_ddot.fill(0.0)

        self.get_logger().error(
            "Recovery aborted: " + reason
        )

    def compute_dt(self) -> float:
        """计算并限制控制周期。"""

        now = time.monotonic()
        nominal_dt = 1.0 / self.publish_rate_hz
        dt = now - self.last_update_monotonic
        self.last_update_monotonic = now

        if not np.isfinite(dt) or dt <= 0.0:
            return nominal_dt

        # 定时器卡顿时不允许一次跨越过大的速度/加速度增量。
        return min(max(dt, 1e-6), nominal_dt)

    def compute_recovery_velocity(
        self,
        dt: float,
    ) -> np.ndarray:
        """计算带制动、加速度限制和 jerk 限制的关节速度。"""

        assert self.current_q is not None

        error = self.target_q - self.current_q
        abs_error = np.abs(error)

        # 比例速度。
        proportional_speed = (
            self.position_gain * abs_error
        )

        # 根据剩余距离计算能够停下来的最大速度：
        # v_stop = sqrt(2*a*distance)
        stopping_speed = np.sqrt(
            2.0 *
            self.max_joint_acceleration *
            abs_error
        )

        desired_speed_magnitude = np.minimum(
            self.max_joint_speed,
            np.minimum(
                proportional_speed,
                stopping_speed,
            ),
        )

        desired_q_dot = (
            np.sign(error) *
            desired_speed_magnitude
        )

        desired_q_dot[
            abs_error <= self.position_tolerance
        ] = 0.0

        # 期望加速度。
        desired_q_ddot = (
            desired_q_dot -
            self.command_q_dot
        ) / dt

        desired_q_ddot = np.clip(
            desired_q_ddot,
            -self.max_joint_acceleration,
            self.max_joint_acceleration,
        )

        # jerk 限制：限制相邻周期加速度变化。
        max_acceleration_change = (
            self.max_joint_jerk * dt
        )

        acceleration_change = np.clip(
            desired_q_ddot -
            self.command_q_ddot,
            -max_acceleration_change,
            max_acceleration_change,
        )

        self.command_q_ddot += acceleration_change

        self.command_q_ddot = np.clip(
            self.command_q_ddot,
            -self.max_joint_acceleration,
            self.max_joint_acceleration,
        )

        self.command_q_dot += (
            self.command_q_ddot * dt
        )

        self.command_q_dot = np.clip(
            self.command_q_dot,
            -self.max_joint_speed,
            self.max_joint_speed,
        )

        # 已经进入容差且速度很小时，明确归零。
        reached_joint = (
            abs_error <= self.position_tolerance
        ) & (
            np.abs(self.command_q_dot) <=
            self.velocity_tolerance
        )

        self.command_q_dot[reached_joint] = 0.0
        self.command_q_ddot[reached_joint] = 0.0

        # 禁止命令继续朝硬限位方向运动。
        lower_guard = (
            self.current_q <=
            FR3_Q_MIN + self.joint_limit_margin
        )
        upper_guard = (
            self.current_q >=
            FR3_Q_MAX - self.joint_limit_margin
        )

        self.command_q_dot[
            lower_guard &
            (self.command_q_dot < 0.0)
        ] = 0.0

        self.command_q_dot[
            upper_guard &
            (self.command_q_dot > 0.0)
        ] = 0.0

        return self.command_q_dot.copy()

    def target_reached(self) -> bool:
        """判断是否到达目标姿态并基本静止。"""

        if (
            self.current_q is None
            or self.current_q_dot is None
        ):
            return False

        position_ok = np.all(
            np.abs(
                self.target_q -
                self.current_q
            ) <= self.position_tolerance
        )

        velocity_ok = np.all(
            np.abs(self.current_q_dot) <=
            self.velocity_tolerance
        )

        return bool(position_ok and velocity_ok)

    def print_status_periodically(self) -> None:
        """每秒打印一次当前姿态和误差。"""

        now = time.monotonic()

        if now - self.last_status_print < 1.0:
            return

        self.last_status_print = now

        if self.current_q is None:
            return

        error = self.target_q - self.current_q

        self.get_logger().info(
            f"state={self.state} | "
            f"q={np.round(self.current_q, 4).tolist()} | "
            f"error={np.round(error, 4).tolist()} | "
            f"q_dot_cmd="
            f"{np.round(self.command_q_dot, 4).tolist()}"
        )

    def timer_callback(self) -> None:
        """恢复状态机。"""

        dt = self.compute_dt()

        # 始终先检查是否有其他外部速度发布者。
        if self.other_publishers_exist():
            self.abort(
                "Another publisher exists on "
                f"{self.output_topic}. Stop Simulink and other "
                "joint-velocity publishers before recovery."
            )

        if self.state == "ABORTED":
            self.publish_zero()
            return

        if not self.joint_state_is_fresh():
            self.publish_zero()

            if self.current_q is not None:
                self.abort(
                    "Joint state timeout."
                )
            return

        assert self.current_q is not None
        assert self.current_q_dot is not None

        if self.state == "WAITING_FOR_JOINT_STATE":
            self.publish_zero()
            self.transition("ZERO_HOLD")
            return

        if self.state == "ZERO_HOLD":
            self.publish_zero()

            elapsed = (
                time.monotonic() -
                self.state_start_monotonic
            )

            # 必须先等待实测速度降下来。
            measured_stationary = np.all(
                np.abs(self.current_q_dot) <=
                self.velocity_tolerance
            )

            if (
                elapsed >= self.initial_zero_hold_sec
                and measured_stationary
            ):
                if self.execute:
                    self.transition("MOVING")
                else:
                    self.transition("DRY_RUN")
            return

        if self.state == "DRY_RUN":
            self.publish_zero()
            self.print_status_periodically()
            return

        if self.state == "MOVING":
            command = self.compute_recovery_velocity(dt)
            self.publish_velocity(command)
            self.print_status_periodically()

            if self.target_reached():
                self.publish_zero()
                self.transition("FINAL_ZERO_HOLD")
            return

        if self.state == "FINAL_ZERO_HOLD":
            self.publish_zero()

            elapsed = (
                time.monotonic() -
                self.state_start_monotonic
            )

            if elapsed >= self.final_zero_hold_sec:
                self.transition("COMPLETED")
                self.get_logger().info(
                    "Safe joint pose reached. "
                    "Zero velocity will continue to be published."
                )
            return

        if self.state == "COMPLETED":
            self.publish_zero()
            return

        self.abort(
            f"Unknown state: {self.state}"
        )
        self.publish_zero()

    def shutdown_safely(self) -> None:
        """退出前重复发送零速度。"""

        self.get_logger().warning(
            "Shutting down. Publishing zero joint velocity."
        )

        for _ in range(10):
            try:
                self.publish_zero()
                time.sleep(0.01)
            except Exception:
                break


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlowHomeRecovery()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_safely()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
