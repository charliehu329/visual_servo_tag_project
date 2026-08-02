"""Pure helpers for velocity-mapper safety and null-space control."""

from __future__ import annotations

from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


NUM_JOINTS = 7


def smoothstep01(value):
    """Return the cubic smoothstep of *value* clipped to ``[0, 1]``."""

    value_array = np.clip(np.asarray(value, dtype=float), 0.0, 1.0)
    return 3.0 * value_array**2 - 2.0 * value_array**3


def extract_ordered_joint_positions(
    names: Sequence[str],
    positions: Sequence[float],
    required_names: Sequence[str],
) -> Optional[np.ndarray]:
    """Validate a JointState payload and return positions in FR3 order."""

    if len(positions) < len(required_names):
        return None

    try:
        position_by_name = {
            str(name): float(position)
            for name, position in zip(names, positions)
        }
        ordered = np.asarray(
            [position_by_name[name] for name in required_names],
            dtype=float,
        )
    except (KeyError, TypeError, ValueError):
        return None

    if ordered.shape != (len(required_names),):
        return None

    if not np.all(np.isfinite(ordered)):
        return None

    return ordered


def validate_camera_velocity(data: Iterable[float]) -> Optional[np.ndarray]:
    """Return the first six finite camera-twist values, or ``None``."""

    try:
        values = np.asarray(data, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None

    if values.size < 6:
        return None

    velocity = values[:6].copy()
    if not np.all(np.isfinite(velocity)):
        return None

    return velocity


def input_freshness_status(
    *,
    has_joint_state: bool,
    joint_state_age: Optional[float],
    has_visual_velocity: bool,
    visual_velocity_age: Optional[float],
    joint_state_timeout: float,
    visual_velocity_timeout: float,
) -> Tuple[bool, str]:
    """Evaluate mapper input availability and freshness."""

    if not has_joint_state or joint_state_age is None:
        return False, "missing_joint_state"

    if not np.isfinite(joint_state_age) or joint_state_age < 0.0:
        return False, "invalid_joint_state_age"

    if joint_state_age > joint_state_timeout:
        return False, "joint_state_timeout"

    if not has_visual_velocity or visual_velocity_age is None:
        return False, "missing_visual_velocity"

    if not np.isfinite(visual_velocity_age) or visual_velocity_age < 0.0:
        return False, "invalid_visual_velocity_age"

    if visual_velocity_age > visual_velocity_timeout:
        return False, "visual_velocity_timeout"

    return True, "ready"


def camera_velocity_is_zero(
    camera_velocity: Sequence[float],
    epsilon: float,
) -> bool:
    """Return whether a complete camera twist represents a zero command."""

    velocity = np.asarray(camera_velocity, dtype=float).reshape(-1)
    return bool(
        velocity.size >= 6
        and np.all(np.isfinite(velocity[:6]))
        and np.linalg.norm(velocity[:6]) <= float(epsilon)
    )


def joint_limit_activation(
    normalized_distance: Sequence[float],
    start_ratio: float,
    full_ratio: float,
) -> np.ndarray:
    """Smoothly activate centering between two normalized limit ratios."""

    distance = np.asarray(normalized_distance, dtype=float)
    x = (distance - float(start_ratio)) / (
        float(full_ratio) - float(start_ratio)
    )
    return smoothstep01(x)


def task_speed_gate(
    task_speed: float,
    fade_start: float,
    disable: float,
) -> float:
    """Fade null-space motion out as the XY visual task becomes strong."""

    speed = float(task_speed)
    if speed <= fade_start:
        return 1.0
    if speed >= disable:
        return 0.0

    x = (speed - fade_start) / (disable - fade_start)
    return float(1.0 - smoothstep01(x))


def singularity_gate(
    sigma_min: float,
    disable: float,
    full: float,
) -> float:
    """Fade null-space motion in as the Jacobian moves away from singularity."""

    sigma = float(sigma_min)
    if sigma <= disable:
        return 0.0
    if sigma >= full:
        return 1.0

    x = (sigma - disable) / (full - disable)
    return float(smoothstep01(x))


def compute_centering_velocity(
    q: Sequence[float],
    q_mid: Sequence[float],
    q_half_range: Sequence[float],
    start_ratio: float,
    full_ratio: float,
    gain: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute raw center-seeking velocity and its smooth activation."""

    q_array = np.asarray(q, dtype=float).reshape(NUM_JOINTS)
    midpoint = np.asarray(q_mid, dtype=float).reshape(NUM_JOINTS)
    half_range = np.asarray(q_half_range, dtype=float).reshape(NUM_JOINTS)

    if (
        not np.all(np.isfinite(q_array))
        or not np.all(np.isfinite(midpoint))
        or not np.all(np.isfinite(half_range))
        or np.any(half_range <= 0.0)
    ):
        raise ValueError("Joint positions and ranges must be finite and valid.")

    normalized_offset = (q_array - midpoint) / half_range
    normalized_distance = np.abs(normalized_offset)
    activation = joint_limit_activation(
        normalized_distance,
        start_ratio,
        full_ratio,
    )
    center_velocity = -float(gain) * activation * normalized_offset

    return center_velocity, activation, normalized_offset


def limit_nullspace_velocity(
    velocity: Sequence[float],
    max_joint_velocity: float,
) -> np.ndarray:
    """Apply an independent per-joint velocity limit to a null-space term."""

    velocity_array = np.asarray(velocity, dtype=float).reshape(NUM_JOINTS)
    if not np.all(np.isfinite(velocity_array)):
        raise ValueError("Null-space velocity contains NaN or Inf.")

    limit = float(max_joint_velocity)
    if not np.isfinite(limit) or limit <= 0.0:
        raise ValueError("Null-space velocity limit must be positive.")

    return np.clip(velocity_array, -limit, limit)


def limit_nullspace_acceleration(
    previous_velocity: Sequence[float],
    target_velocity: Sequence[float],
    max_joint_acceleration: float,
    dt: float,
    nominal_dt: float,
    maximum_dt: float,
) -> np.ndarray:
    """Limit per-joint null-space velocity changes using a bounded real dt."""

    previous = np.asarray(previous_velocity, dtype=float).reshape(NUM_JOINTS)
    target = np.asarray(target_velocity, dtype=float).reshape(NUM_JOINTS)

    if not np.all(np.isfinite(previous)) or not np.all(np.isfinite(target)):
        raise ValueError("Null-space velocity contains NaN or Inf.")

    effective_dt = float(dt)
    if not np.isfinite(effective_dt) or effective_dt <= 0.0:
        effective_dt = float(nominal_dt)

    effective_dt = min(effective_dt, float(maximum_dt))
    effective_dt = max(effective_dt, 1.0e-6)

    max_delta = float(max_joint_acceleration) * effective_dt
    delta = np.clip(target - previous, -max_delta, max_delta)
    return previous + delta
