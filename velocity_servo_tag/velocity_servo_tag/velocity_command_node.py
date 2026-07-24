#!/usr/bin/env python3
"""
velocity_command_node.py

功能：
    接收7维目标关节速度，对其进行有效性检查、速度限制、
    加速度限制和通信超时保护，然后发送给 Franka 底层速度控制器。

订阅：
    /simulink/target_joints_velocities
        std_msgs/msg/Float64MultiArray
        [dq1, dq2, dq3, dq4, dq5, dq6, dq7]

发布：
    /joint_velocity_example_controller/commands
        std_msgs/msg/Float64MultiArray

模式：
    zero:
        持续向底层控制器发送零速度，默认安全模式。

    topic:
        接收外部7维关节速度并转发给底层控制器。

安全机制：
    1. 检查命令长度是否为7。
    2. 拒绝包含 NaN 或 Inf 的命令。
    3. 按 FR3 关节速度上限同比例缩放。
    4. 对七维速度增量整组同比例缩放，限制相邻周期的加速度。
    5. 外部命令超时后平滑减速至零。
    6. 节点退出前平滑减速至零。
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


NUM_JOINTS = 7

# FR3硬件关节速度上限，单位 rad/s。
FR3_MAX_JOINT_VELOCITIES = np.asarray(
    [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26],
    dtype=float,
)


class VelocityCommandNode(Node):
    """Franka FR3 关节速度命令桥接节点。"""

    def __init__(self):
        super().__init__("velocity_command_node")

        # =====================================================
        # 参数声明
        # =====================================================

        self.declare_parameter("mode", "zero")
        self.declare_parameter("publish_rate_hz", 120.0)
        self.declare_parameter("max_velocity_scale", 0.1)

        self.declare_parameter(
            "max_joint_accelerations",
            [0.40] * NUM_JOINTS,
        )

        self.declare_parameter(
            "target_velocity_timeout_sec",
            0.15,
        )

        self.declare_parameter(
            "input_topic",
            "/simulink/target_joints_velocities",
        )

        self.declare_parameter(
            "command_topic",
            "/joint_velocity_example_controller/commands",
        )

        # =====================================================
        # 参数读取
        # =====================================================

        self.mode = str(
            self.get_parameter("mode").value
        )

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )

        self.max_velocity_scale = float(
            self.get_parameter("max_velocity_scale").value
        )

        self.max_joint_accelerations = np.asarray(
            self.get_parameter(
                "max_joint_accelerations"
            ).value,
            dtype=float,
        ).reshape(-1)

        self.target_velocity_timeout_sec = float(
            self.get_parameter(
                "target_velocity_timeout_sec"
            ).value
        )

        self.input_topic = str(
            self.get_parameter("input_topic").value
        )

        self.command_topic = str(
            self.get_parameter("command_topic").value
        )

        self.validate_parameters()

        self.max_joint_velocities = (
            FR3_MAX_JOINT_VELOCITIES *
            self.max_velocity_scale
        )

        # =====================================================
        # 状态
        # =====================================================

        # 最近一次收到的合法目标速度。
        self.target_q_dot = np.zeros(
            NUM_JOINTS,
            dtype=float,
        )

        # 当前实际发布的速度。
        self.commanded_q_dot = np.zeros(
            NUM_JOINTS,
            dtype=float,
        )

        self.last_target_time = None
        self.last_update_time = None
        self.stop_reason = None

        # =====================================================
        # ROS接口
        # =====================================================

        # 只订阅输入话题，不再向该话题发布echo消息。
        self.target_subscription = self.create_subscription(
            Float64MultiArray,
            self.input_topic,
            self.target_callback,
            10,
        )

        # 唯一输出：Franka底层控制器命令话题。
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            self.command_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            "VelocityCommandNode started | "
            f"mode={self.mode} | "
            f"rate={self.publish_rate_hz:.1f} Hz | "
            f"input={self.input_topic} | "
            f"output={self.command_topic}"
        )

        self.get_logger().info(
            "Maximum joint velocities: "
            f"{self.max_joint_velocities.tolist()} rad/s"
        )

        self.get_logger().info(
            "Maximum joint accelerations: "
            f"{self.max_joint_accelerations.tolist()} rad/s^2"
        )

        if self.mode == "zero":
            self.get_logger().warning(
                "ZERO mode is active. "
                "Only zero joint velocity will be sent."
            )
        else:
            self.get_logger().info(
                "TOPIC mode is active. "
                "Waiting for external joint velocity commands."
            )

    def validate_parameters(self):
        """检查节点参数是否合法。"""

        if self.mode not in ("zero", "topic"):
            raise ValueError(
                "mode must be 'zero' or 'topic', "
                f"but got '{self.mode}'."
            )

        if (
            not np.isfinite(self.publish_rate_hz)
            or self.publish_rate_hz <= 0.0
        ):
            raise ValueError(
                "publish_rate_hz must be positive and finite."
            )

        if (
            not np.isfinite(self.max_velocity_scale)
            or self.max_velocity_scale <= 0.0
            or self.max_velocity_scale > 1.0
        ):
            raise ValueError(
                "max_velocity_scale must be in (0, 1]."
            )

        if (
            self.max_joint_accelerations.shape !=
            (NUM_JOINTS,)
            or not np.all(
                np.isfinite(
                    self.max_joint_accelerations
                )
            )
            or np.any(
                self.max_joint_accelerations <= 0.0
            )
        ):
            raise ValueError(
                "max_joint_accelerations must contain "
                "7 positive finite values."
            )

        if (
            not np.isfinite(
                self.target_velocity_timeout_sec
            )
            or self.target_velocity_timeout_sec <= 0.0
        ):
            raise ValueError(
                "target_velocity_timeout_sec "
                "must be positive and finite."
            )

        if not self.input_topic:
            raise ValueError(
                "input_topic cannot be empty."
            )

        if not self.command_topic:
            raise ValueError(
                "command_topic cannot be empty."
            )

        if self.input_topic == self.command_topic:
            raise ValueError(
                "input_topic and command_topic "
                "must be different."
            )

    def target_callback(self, message):
        """接收外部7维目标关节速度。"""

        if self.mode != "topic":
            return

        data = np.asarray(
            message.data,
            dtype=float,
        ).reshape(-1)

        if data.shape != (NUM_JOINTS,):
            self.enter_safe_stop(
                "Invalid command length: "
                f"expected 7, got {data.size}."
            )
            return

        if not np.all(np.isfinite(data)):
            self.enter_safe_stop(
                "Joint velocity command contains NaN or Inf."
            )
            return

        self.target_q_dot = (
            self.limit_joint_velocity(data)
        )

        self.last_target_time = (
            self.get_clock().now()
        )

        if self.stop_reason is not None:
            self.get_logger().info(
                "Valid joint velocity command recovered."
            )
            self.stop_reason = None

    def limit_joint_velocity(self, q_dot):
        """同比例缩放7维关节速度，保持速度方向。"""

        q_dot = np.asarray(
            q_dot,
            dtype=float,
        ).reshape(NUM_JOINTS)

        ratios = (
            np.abs(q_dot) /
            self.max_joint_velocities
        )

        scale = max(
            1.0,
            float(np.max(ratios)),
        )

        return q_dot / scale

    def compute_dt(self, now):
        """计算速度变化限制所使用的时间步长。"""

        nominal_dt = (
            1.0 /
            self.publish_rate_hz
        )

        if self.last_update_time is None:
            dt = nominal_dt
        else:
            dt = (
                now -
                self.last_update_time
            ).nanoseconds * 1e-9

            if (
                not np.isfinite(dt)
                or dt <= 0.0
            ):
                dt = nominal_dt

            # 定时器卡顿后不允许一次跨越过大速度增量。
            dt = min(
                float(dt),
                nominal_dt,
            )

        self.last_update_time = now

        return max(
            float(dt),
            1e-6,
        )

    def command_is_fresh(self, now):
        """判断外部目标速度是否仍在超时时间内。"""

        if self.last_target_time is None:
            return False, None

        age_sec = (
            now -
            self.last_target_time
        ).nanoseconds * 1e-9

        age_sec = max(
            0.0,
            float(age_sec),
        )

        return (
            age_sec <=
            self.target_velocity_timeout_sec,
            age_sec,
        )

    def limit_velocity_change(
        self,
        target_q_dot,
        dt,
    ):
        """同比例缩放七维速度增量，限制相邻控制周期的加速度。"""

        target_q_dot = self.limit_joint_velocity(
            target_q_dot
        )

        max_delta = (
            self.max_joint_accelerations *
            float(dt)
        )

        delta = (
            target_q_dot -
            self.commanded_q_dot
        )

        acceleration_ratios = (
            np.abs(delta) /
            max_delta
        )

        acceleration_scale = max(
            1.0,
            float(np.max(acceleration_ratios)),
        )

        delta = delta / acceleration_scale

        new_command = (
            self.commanded_q_dot +
            delta
        )

        new_command = self.limit_joint_velocity(
            new_command
        )

        # 接近零时明确归零，避免残留极小速度。
        if (
            np.allclose(
                target_q_dot,
                0.0,
                atol=1e-12,
            )
            and np.all(
                np.abs(new_command) <= max_delta
            )
        ):
            new_command = np.zeros(
                NUM_JOINTS,
                dtype=float,
            )

        self.commanded_q_dot = new_command

        return new_command.copy()

    def publish_command(self, q_dot):
        """发布7维关节速度到底层控制器。"""

        message = Float64MultiArray()
        message.data = (
            np.asarray(
                q_dot,
                dtype=float,
            )
            .reshape(NUM_JOINTS)
            .tolist()
        )

        self.command_publisher.publish(
            message
        )

    def enter_safe_stop(self, reason):
        """记录安全停止原因，并将目标速度设为零。"""

        self.target_q_dot = np.zeros(
            NUM_JOINTS,
            dtype=float,
        )

        if reason != self.stop_reason:
            self.get_logger().warning(
                f"Safety stop: {reason}"
            )
            self.stop_reason = reason

    def timer_callback(self):
        """周期计算并发布安全关节速度。"""

        now = self.get_clock().now()
        dt = self.compute_dt(now)

        if self.mode == "zero":
            target = np.zeros(
                NUM_JOINTS,
                dtype=float,
            )

        else:
            fresh, age_sec = (
                self.command_is_fresh(now)
            )

            if fresh:
                target = self.target_q_dot.copy()
            else:
                if age_sec is None:
                    reason = (
                        "Waiting for first target "
                        "velocity command."
                    )
                else:
                    reason = (
                        "Target velocity timeout: "
                        f"{age_sec:.3f} s."
                    )

                self.enter_safe_stop(reason)

                target = np.zeros(
                    NUM_JOINTS,
                    dtype=float,
                )

        command = self.limit_velocity_change(
            target_q_dot=target,
            dt=dt,
        )

        self.publish_command(command)

    def ramp_to_zero_before_shutdown(self):
        """节点退出前，在有限时间内平滑减速到零。"""

        if not rclpy.ok():
            return

        nominal_dt = (
            1.0 /
            self.publish_rate_hz
        )

        initial_speed = np.abs(
            self.commanded_q_dot
        )

        required_time = float(
            np.max(
                initial_speed /
                self.max_joint_accelerations
            )
        )

        deadline = (
            time.monotonic() +
            required_time +
            0.20
        )

        zero_target = np.zeros(
            NUM_JOINTS,
            dtype=float,
        )

        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and not np.allclose(
                self.commanded_q_dot,
                0.0,
                atol=1e-12,
            )
        ):
            command = self.limit_velocity_change(
                target_q_dot=zero_target,
                dt=nominal_dt,
            )

            self.publish_command(command)

            time.sleep(nominal_dt)

        # 最后明确发布零速度。
        self.commanded_q_dot = zero_target.copy()
        self.publish_command(zero_target)

        self.get_logger().info(
            "Joint velocity ramped to zero."
        )


def main(args=None):
    """ROS 2节点入口。"""

    rclpy.init(args=args)

    node = None

    try:
        node = VelocityCommandNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info(
                "Keyboard interrupt received."
            )

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Unexpected error: {error}"
            )
        raise

    finally:
        if node is not None:
            if rclpy.ok():
                node.ramp_to_zero_before_shutdown()

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
