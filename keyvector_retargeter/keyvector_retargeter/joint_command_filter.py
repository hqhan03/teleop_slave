"""Command post-processing utilities for DG-5F joint trajectories."""

from __future__ import annotations

import numpy as np

from .manus_joints import RIGHT_DG_SPREAD_OR_OPPOSITION_JOINTS


def velocity_limits_rad_s(joint_names, max_flex_velocity_deg_s: float, max_spread_velocity_deg_s: float) -> np.ndarray:
    """Return per-joint velocity limits in rad/s."""
    limits = []
    for name in joint_names:
        deg_s = max_spread_velocity_deg_s if name in RIGHT_DG_SPREAD_OR_OPPOSITION_JOINTS else max_flex_velocity_deg_s
        limits.append(np.deg2rad(float(deg_s)))
    return np.asarray(limits, dtype=float)


class PerJointVelocityLimiter:
    """Cap per-step joint motion without suppressing small commands."""

    def __init__(self, joint_names, max_flex_velocity_deg_s: float, max_spread_velocity_deg_s: float, initial_state=None):
        self.joint_names = list(joint_names)
        self.velocity_limits_rad_s = velocity_limits_rad_s(
            self.joint_names,
            max_flex_velocity_deg_s=max_flex_velocity_deg_s,
            max_spread_velocity_deg_s=max_spread_velocity_deg_s,
        )
        self.current_state = None if initial_state is None else np.asarray(initial_state, dtype=float).copy()

    def reset(self, joint_positions) -> None:
        self.current_state = np.asarray(joint_positions, dtype=float).copy()

    def apply(self, target_positions, dt_sec: float | None) -> np.ndarray:
        target = np.asarray(target_positions, dtype=float)
        if self.current_state is None or dt_sec is None:
            self.current_state = target.copy()
            return target.copy()

        dt = max(float(dt_sec), 0.0)
        if dt <= 0.0:
            return self.current_state.copy()

        max_step = self.velocity_limits_rad_s * dt
        delta = np.clip(target - self.current_state, -max_step, max_step)
        self.current_state = self.current_state + delta
        return self.current_state.copy()
