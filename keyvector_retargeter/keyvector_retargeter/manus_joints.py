"""Manus joint-state parsing and DG-5F joint-prior helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .dg5f_model import DG5F_RIGHT_JOINT_NAMES


EXPECTED_MANUS_JOINT_NAMES = (
    'thumb_mcp_spread', 'thumb_mcp_stretch', 'thumb_pip', 'thumb_dip',
    'index_mcp_spread', 'index_mcp_stretch', 'index_pip', 'index_dip',
    'middle_mcp_spread', 'middle_mcp_stretch', 'middle_pip', 'middle_dip',
    'ring_mcp_spread', 'ring_mcp_stretch', 'ring_pip', 'ring_dip',
    'pinky_mcp_spread', 'pinky_mcp_stretch', 'pinky_pip', 'pinky_dip',
)

RIGHT_DG_SPREAD_OR_OPPOSITION_JOINTS = {
    'rj_dg_1_1',
    'rj_dg_1_2',
    'rj_dg_2_1',
    'rj_dg_3_1',
    'rj_dg_4_1',
    'rj_dg_5_1',
    'rj_dg_5_2',
}

_VENDOR_RIGHT_DIR = np.array([
    1.0, -1.0, 1.0, 1.0,
    -1.0, 1.0, 1.0, 1.0,
    -1.0, 1.0, 1.0, 1.0,
    -1.0, 1.0, 1.0, 1.0,
    1.0, -1.0, 1.0, 1.0,
], dtype=float)

_VENDOR_CALIBRATION = np.array([
    1.0, 1.6, 1.3, 1.3,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.0, 1.0,
], dtype=float)


@dataclass(frozen=True)
class ManusJointSample:
    """Canonical 20-DoF Manus joint sample in bridge order."""

    positions_rad: np.ndarray

    def is_finite(self) -> bool:
        return np.isfinite(self.positions_rad).all()


def decode_manus_joint_state_msg(msg) -> ManusJointSample:
    """Decode `/manus/finger_joints` into the expected 20-joint Manus order."""
    if len(msg.position) < len(EXPECTED_MANUS_JOINT_NAMES):
        raise ValueError(
            f'Expected at least {len(EXPECTED_MANUS_JOINT_NAMES)} Manus joint positions, got {len(msg.position)}')

    ordered = None
    if len(msg.name) >= len(EXPECTED_MANUS_JOINT_NAMES):
        name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
        if all(name in name_to_pos for name in EXPECTED_MANUS_JOINT_NAMES):
            ordered = [name_to_pos[name] for name in EXPECTED_MANUS_JOINT_NAMES]

    if ordered is None:
        ordered = list(msg.position[:len(EXPECTED_MANUS_JOINT_NAMES)])

    return ManusJointSample(positions_rad=np.asarray(ordered, dtype=float))


def manus_sample_from_positions(positions_rad) -> ManusJointSample:
    """Construct a ManusJointSample from an already ordered radian vector."""
    positions = np.asarray(positions_rad, dtype=float)
    if positions.shape != (len(EXPECTED_MANUS_JOINT_NAMES),):
        raise ValueError(
            f'Expected Manus joint vector shape {(len(EXPECTED_MANUS_JOINT_NAMES),)}, got {positions.shape}')
    return ManusJointSample(positions_rad=positions)


def manus_joint_prior_basis(sample_or_positions) -> np.ndarray:
    """Compute the vendor-style right-hand DG-5F prior basis from Manus joints."""
    if isinstance(sample_or_positions, ManusJointSample):
        sample = sample_or_positions.positions_rad
    else:
        sample = np.asarray(sample_or_positions, dtype=float)

    if sample.shape != (len(EXPECTED_MANUS_JOINT_NAMES),):
        raise ValueError(
            f'Expected Manus joint vector shape {(len(EXPECTED_MANUS_JOINT_NAMES),)}, got {sample.shape}')

    q_deg = np.rad2deg(sample)
    qd = np.zeros(len(DG5F_RIGHT_JOINT_NAMES), dtype=float)

    qd[0] = math.radians(58.5 - q_deg[1])
    qd[1] = math.radians(q_deg[0] + 20.0)
    qd[2] = math.radians(q_deg[2])
    qd[3] = math.radians(0.5 * (q_deg[2] + q_deg[3]))

    qd[4] = math.radians(q_deg[4])
    qd[5] = math.radians(q_deg[5])
    qd[6] = math.radians(q_deg[6] - 40.0)
    qd[7] = math.radians(q_deg[7])

    qd[8] = math.radians(q_deg[8])
    qd[9] = math.radians(q_deg[9])
    qd[10] = math.radians(q_deg[10] - 30.0)
    qd[11] = math.radians(q_deg[11])

    qd[12] = math.radians(q_deg[12])
    qd[13] = math.radians(q_deg[13])
    qd[14] = math.radians(q_deg[14])
    qd[15] = math.radians(q_deg[15])

    if q_deg[17] > 55.0 and q_deg[18] > 25.0 and q_deg[18] > 20.0:
        qd[16] = math.radians(abs(q_deg[16]) * 2.0)
    else:
        qd[16] = math.radians(abs(q_deg[16]) / 1.5)

    qd[17] = math.radians(q_deg[16])
    qd[18] = math.radians(q_deg[17])
    qd[19] = math.radians(q_deg[18])

    mapped = qd * _VENDOR_CALIBRATION * _VENDOR_RIGHT_DIR
    for idx in range(mapped.shape[0]):
        if idx == 1:
            if mapped[idx] >= 0.0:
                mapped[idx] = 0.0
        elif idx not in (4, 8, 12, 16, 17):
            if mapped[idx] <= 0.0:
                mapped[idx] = 0.0

    return mapped


def apply_joint_prior_calibration(basis_positions: np.ndarray, calibration: dict) -> np.ndarray:
    """Apply the calibrated affine correction to a DG-5F joint-prior basis."""
    basis = np.asarray(basis_positions, dtype=float)
    gains = np.asarray(calibration.get('joint_prior_gains', [1.0] * len(DG5F_RIGHT_JOINT_NAMES)), dtype=float)
    offsets = np.asarray(calibration.get('joint_prior_offsets', [0.0] * len(DG5F_RIGHT_JOINT_NAMES)), dtype=float)
    if gains.shape != basis.shape or offsets.shape != basis.shape:
        raise ValueError('Joint-prior calibration arrays must match the DG-5F joint vector length')
    return gains * basis + offsets


def joint_prior_from_manus_sample(sample_or_positions, calibration: dict, model=None) -> np.ndarray:
    """Compute a calibrated DG-5F joint prior from Manus joint measurements."""
    basis = manus_joint_prior_basis(sample_or_positions)
    calibrated = apply_joint_prior_calibration(basis, calibration)
    if model is not None:
        calibrated = model.clip_to_limits(calibrated)
    return calibrated


def default_joint_prior_weights(
    joint_names,
    flex_weight: float,
    spread_weight: float,
) -> np.ndarray:
    """Return per-joint prior weights using DG-5F joint semantics."""
    weights = []
    for name in joint_names:
        if name in RIGHT_DG_SPREAD_OR_OPPOSITION_JOINTS:
            weights.append(float(spread_weight))
        else:
            weights.append(float(flex_weight))
    return np.asarray(weights, dtype=float)
