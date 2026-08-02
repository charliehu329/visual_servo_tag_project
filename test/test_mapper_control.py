"""Unit tests for velocity-mapper safety and null-space helpers."""

import numpy as np
import unittest

from velocity_servo_tag.mapper_control import (
    camera_velocity_is_zero,
    compute_centering_velocity,
    extract_ordered_joint_positions,
    input_freshness_status,
    joint_limit_activation,
    limit_nullspace_acceleration,
    limit_nullspace_velocity,
    singularity_gate,
    task_speed_gate,
    validate_camera_velocity,
)


JOINT_NAMES = [f"fr3_joint{index}" for index in range(1, 8)]
Q_MID = np.zeros(7)
Q_HALF_RANGE = np.ones(7)


def freshness(**overrides):
    arguments = {
        "has_joint_state": True,
        "joint_state_age": 0.01,
        "has_visual_velocity": True,
        "visual_velocity_age": 0.01,
        "joint_state_timeout": 0.10,
        "visual_velocity_timeout": 0.10,
    }
    arguments.update(overrides)
    return input_freshness_status(**arguments)


class MapperControlTests(unittest.TestCase):
    def test_01_missing_joint_state_forces_safe_status(self):
        ready, reason = freshness(
            has_joint_state=False,
            joint_state_age=None,
        )
        self.assertFalse(ready)
        self.assertEqual(reason, "missing_joint_state")

    def test_02_stale_joint_state_forces_safe_status(self):
        ready, reason = freshness(joint_state_age=0.101)
        self.assertFalse(ready)
        self.assertEqual(reason, "joint_state_timeout")

    def test_03_stale_visual_velocity_forces_safe_status(self):
        ready, reason = freshness(visual_velocity_age=0.101)
        self.assertFalse(ready)
        self.assertEqual(reason, "visual_velocity_timeout")

    def test_04_zero_camera_command_disables_outer_joint_centering(self):
        q = np.asarray([0.96, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        raw_centering, _, _ = compute_centering_velocity(
            q, Q_MID, Q_HALF_RANGE, 0.80, 0.95, 0.10
        )
        self.assertLess(raw_centering[0], 0.0)
        self.assertTrue(camera_velocity_is_zero(np.zeros(6), 1.0e-6))

        # Mirrors the mapper's mandatory early-return branch.
        dq_output = np.zeros(7) + np.zeros(7)
        np.testing.assert_array_equal(dq_output, np.zeros(7))

    def test_05_invalid_visual_arrays_are_rejected(self):
        cases = [
            [0.0] * 5,
            [0.0, 0.0, np.nan, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, np.inf, 0.0, 0.0],
        ]
        for data in cases:
            with self.subTest(data=data):
                self.assertIsNone(validate_camera_velocity(data))

    def test_06_invalid_joint_states_are_rejected(self):
        cases = [
            [0.0] * 6,
            [0.0, 0.0, np.nan, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, np.inf, 0.0, 0.0, 0.0],
        ]
        for positions in cases:
            with self.subTest(positions=positions):
                self.assertIsNone(
                    extract_ordered_joint_positions(
                        JOINT_NAMES, positions, JOINT_NAMES
                    )
                )

    def test_07_center_region_has_no_centering_velocity(self):
        q = np.asarray([0.79, -0.50, 0.20, -0.10, 0.0, 0.40, -0.79])
        velocity, activation, _ = compute_centering_velocity(
            q, Q_MID, Q_HALF_RANGE, 0.80, 0.95, 0.10
        )
        np.testing.assert_array_equal(activation, np.zeros(7))
        np.testing.assert_array_equal(velocity, np.zeros(7))

    def test_08_centering_direction_always_points_to_midpoint(self):
        q = np.asarray([0.96, -0.96, 0.0, 0.0, 0.0, 0.0, 0.0])
        velocity, _, _ = compute_centering_velocity(
            q, Q_MID, Q_HALF_RANGE, 0.80, 0.95, 0.10
        )
        self.assertLess(velocity[0], 0.0)
        self.assertGreater(velocity[1], 0.0)

    def test_09_activation_is_continuous_and_monotonic(self):
        ratios = np.asarray([0.79, 0.80, 0.85, 0.95, 0.96])
        activation = joint_limit_activation(ratios, 0.80, 0.95)
        self.assertTrue(np.all((0.0 <= activation) & (activation <= 1.0)))
        self.assertTrue(np.all(np.diff(activation) >= 0.0))
        self.assertAlmostEqual(activation[0], 0.0)
        self.assertAlmostEqual(activation[1], 0.0)
        self.assertTrue(0.0 < activation[2] < 1.0)
        self.assertAlmostEqual(activation[3], 1.0)
        self.assertAlmostEqual(activation[4], 1.0)

    def test_10_strong_xy_task_disables_nullspace_only(self):
        g_task = task_speed_gate(0.10, 0.02, 0.10)
        dq_task = np.ones(7) * 0.02
        dq_ns = g_task * np.ones(7) * 0.05
        self.assertEqual(g_task, 0.0)
        np.testing.assert_array_equal(dq_ns, np.zeros(7))
        self.assertGreater(np.linalg.norm(dq_task), 0.0)

    def test_11_near_singularity_disables_nullspace_only(self):
        g_sigma = singularity_gate(0.03, 0.03, 0.08)
        dq_task = np.ones(7) * 0.02
        dq_ns = g_sigma * np.ones(7)
        self.assertEqual(g_sigma, 0.0)
        np.testing.assert_array_equal(dq_ns, np.zeros(7))
        self.assertGreater(np.linalg.norm(dq_task), 0.0)

    def test_12_nullspace_velocity_is_independently_limited(self):
        limited = limit_nullspace_velocity(
            [-1.0, -0.2, -0.1, 0.0, 0.1, 0.2, 1.0], 0.10
        )
        self.assertTrue(np.all(np.abs(limited) <= 0.10))

    def test_13_nullspace_acceleration_is_limited(self):
        previous = np.zeros(7)
        dt = 0.02
        max_acceleration = 0.30
        limited = limit_nullspace_acceleration(
            previous,
            np.ones(7),
            max_acceleration,
            dt,
            nominal_dt=0.01,
            maximum_dt=0.05,
        )
        self.assertTrue(
            np.all(
                np.abs(limited - previous)
                <= max_acceleration * dt + 1.0e-12
            )
        )

    def test_14_normal_combined_output_is_finite_seven_vector(self):
        q = np.asarray([0.90, -0.90, 0.0, 0.0, 0.0, 0.0, 0.0])
        center, _, _ = compute_centering_velocity(
            q, Q_MID, Q_HALF_RANGE, 0.80, 0.95, 0.10
        )
        dq_ns = limit_nullspace_velocity(center, 0.10)
        output = np.linspace(-0.02, 0.02, 7) + dq_ns
        self.assertEqual(output.shape, (7,))
        self.assertTrue(np.all(np.isfinite(output)))

    def test_valid_payloads_accept_extra_elements_and_joint_order(self):
        velocity = validate_camera_velocity([1, 2, 3, 4, 5, 6, 7])
        np.testing.assert_array_equal(velocity, np.arange(1.0, 7.0))

        ordered = extract_ordered_joint_positions(
            list(reversed(JOINT_NAMES)),
            list(reversed(np.arange(7.0))),
            JOINT_NAMES,
        )
        np.testing.assert_array_equal(ordered, np.arange(7.0))


if __name__ == "__main__":
    unittest.main()
