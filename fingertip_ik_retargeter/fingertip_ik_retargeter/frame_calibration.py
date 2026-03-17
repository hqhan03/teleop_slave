"""Shared frame-calibration helpers for Manus palm-local -> DG-5F palm-local mapping."""

from __future__ import annotations

import itertools
import os
from typing import Dict

import numpy as np
import yaml


MANUS_FINGER_ORDER = ['thumb', 'index', 'middle', 'ring', 'pinky']
DEFAULT_CALIBRATION_PATH = os.path.expanduser('~/.ros/manus_dg5f_ik_frame.yaml')
DEFAULT_WORKSPACE_AXIS_SCALE = [1.0, 1.0, 1.0]
DEFAULT_FINGER_TARGET_SCALES = [1.0, 1.0, 1.0, 1.0, 1.0]

# Multi-pose calibration targets copied from the existing Tesollo calibration
# flow so the user can match the same intuitive poses with the Manus glove.
DG5F_MULTI_POSE_CALIBRATION = [
    {
        'name': 'flat_open',
        'description': 'FLAT OPEN — extend all fingers, thumb spread wide',
        'joint_positions_deg': [
            77.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            -15.0, 0.0, 0.0, 0.0,
            0.0, -15.0, 0.0, 0.0,
        ],
    },
    {
        'name': 'relaxed_fist',
        'description': 'RELAXED FIST — curl fingers moderately, thumb resting across index',
        'joint_positions_deg': [
            40.0, -90.0, 0.0, 0.0,
            0.0, 80.0, 60.0, 60.0,
            0.0, 80.0, 60.0, 60.0,
            0.0, 75.0, 60.0, 60.0,
            0.0, 0.0, 60.0, 60.0,
        ],
    },
    {
        'name': 'pinch',
        'description': 'PINCH — touch thumb+index tips together, others extended and spread',
        'joint_positions_deg': [
            40.0, -70.0, -20.0, -20.0,
            0.0, 40.0, 30.0, 30.0,
            15.0, 0.0, 0.0, 0.0,
            15.0, 0.0, 0.0, 0.0,
            0.0, 40.0, 0.0, 0.0,
        ],
    },
    {
        'name': 'pointing',
        'description': 'POINTING — extend index only, curl others, thumb tucked beside index',
        'joint_positions_deg': [
            60.0, -30.0, 0.0, 0.0,
            -10.0, 0.0, 0.0, 0.0,
            -10.0, 70.0, 50.0, 50.0,
            -5.0, 70.0, 50.0, 50.0,
            0.0, -5.0, 50.0, 50.0,
        ],
    },
    {
        'name': 'rock_on',
        'description': 'ROCK ON — index+pinky extended, middle+ring curled, thumb mid',
        'joint_positions_deg': [
            40.0, -90.0, 0.0, 0.0,
            10.0, 0.0, 0.0, 0.0,
            0.0, 100.0, 70.0, 70.0,
            15.0, 100.0, 70.0, 70.0,
            0.0, 50.0, 0.0, 0.0,
        ],
    },
]


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm <= 1e-8:
        raise ValueError('Cannot normalize a near-zero vector')
    return vec / norm


def apply_axis_mapping(pos: np.ndarray, axes, signs) -> np.ndarray:
    """Apply a signed permutation axis mapping."""
    arr = np.asarray(pos, dtype=float)
    return np.array([
        float(signs[0]) * arr[int(axes[0])],
        float(signs[1]) * arr[int(axes[1])],
        float(signs[2]) * arr[int(axes[2])],
    ], dtype=float)


def _compute_hand_basis(fingertips: Dict[str, np.ndarray]) -> np.ndarray:
    """Compute an orthonormal palm basis from palm-local fingertip positions."""
    thumb = np.asarray(fingertips['thumb'], dtype=float)
    index = np.asarray(fingertips['index'], dtype=float)
    middle = np.asarray(fingertips['middle'], dtype=float)
    ring = np.asarray(fingertips['ring'], dtype=float)
    pinky = np.asarray(fingertips['pinky'], dtype=float)

    z_axis = normalize((index + middle + ring + pinky) / 4.0)
    y_seed = normalize(thumb - pinky)
    x_axis = normalize(np.cross(y_seed, z_axis))
    y_axis = normalize(np.cross(z_axis, x_axis))
    return np.column_stack((x_axis, y_axis, z_axis))


def compute_signed_permutation(manus_tips: Dict[str, np.ndarray],
                               dg5f_tips: Dict[str, np.ndarray]):
    """Find the best signed-permutation that maps Manus palm axes into DG-5F palm axes."""
    manus_basis = _compute_hand_basis(manus_tips)
    dg5f_basis = _compute_hand_basis(dg5f_tips)
    alignment = dg5f_basis.T @ manus_basis

    best_score = None
    best_axes = None
    best_signs = None
    best_perm_matrix = None

    for axes in itertools.permutations(range(3)):
        for signs in itertools.product([-1.0, 1.0], repeat=3):
            perm = np.zeros((3, 3), dtype=float)
            for row, axis in enumerate(axes):
                perm[row, axis] = signs[row]
            score = float(np.trace(perm @ alignment))
            if best_score is None or score > best_score:
                best_score = score
                best_axes = list(axes)
                best_signs = list(signs)
                best_perm_matrix = perm

    return {
        'axes': best_axes,
        'signs': best_signs,
        'score': float(best_score),
        'matrix': best_perm_matrix,
    }


def refine_signed_permutation(manus_tips: Dict[str, np.ndarray],
                              dg5f_tips: Dict[str, np.ndarray],
                              scale: float):
    """Resolve sign/permutation ambiguity using the labeled fingertip geometry itself."""
    best_error = None
    best_axes = None
    best_signs = None

    for axes in itertools.permutations(range(3)):
        for signs in itertools.product([-1.0, 1.0], repeat=3):
            error = 0.0
            for name in MANUS_FINGER_ORDER:
                mapped = apply_axis_mapping(manus_tips[name], axes, signs) * scale
                error += float(np.sum((mapped - np.asarray(dg5f_tips[name], dtype=float)) ** 2))
            if best_error is None or error < best_error:
                best_error = error
                best_axes = list(axes)
                best_signs = list(signs)

    return {
        'axes': best_axes,
        'signs': best_signs,
        'error': float(best_error),
    }


def compute_hand_scale(manus_tips: Dict[str, np.ndarray],
                       dg5f_tips: Dict[str, np.ndarray]) -> float:
    """Compute a workspace scale using non-thumb fingertip distances from the palm origin."""
    fingers = ['index', 'middle', 'ring', 'pinky']
    manus_dist = np.mean([np.linalg.norm(np.asarray(manus_tips[name], dtype=float)) for name in fingers])
    dg5f_dist = np.mean([np.linalg.norm(np.asarray(dg5f_tips[name], dtype=float)) for name in fingers])
    if manus_dist <= 1e-8:
        raise ValueError('Cannot compute scale from near-zero Manus fingertip distances')
    return float(dg5f_dist / manus_dist)


def compute_calibration(manus_tips: Dict[str, np.ndarray],
                        dg5f_tips: Dict[str, np.ndarray]) -> dict:
    scale = compute_hand_scale(manus_tips, dg5f_tips)
    basis_mapping = compute_signed_permutation(manus_tips, dg5f_tips)
    point_mapping = refine_signed_permutation(manus_tips, dg5f_tips, scale)
    return {
        'input_frame_mode': 'palm_local',
        'manus_to_dg5f_axes': point_mapping['axes'],
        'manus_to_dg5f_signs': point_mapping['signs'],
        'hand_scale': scale,
        'palm_offset': [0.0, 0.0, 0.0],
        'alignment_score': basis_mapping['score'],
        'point_error': point_mapping['error'],
    }


def fit_retarget_scales(
    manus_pose_samples: Dict[str, Dict[str, np.ndarray]],
    dg5f_pose_tips: Dict[str, Dict[str, np.ndarray]],
    axes,
    signs,
    hand_scale: float,
    palm_offset=None,
    iterations: int = 30,
) -> dict:
    """Fit stage-2 retarget gains from multiple matched glove/robot poses.

    The fitted model is:
        target_dg5f = workspace_axis_scale * finger_target_scale[finger]
                      * (hand_scale * axis_remap(manus_tip)) + palm_offset

    Stage 1 already estimates the frame mapping and a global hand scale. This
    second stage learns axis-specific scaling (especially for spread) and a
    per-finger scalar (especially useful for the thumb).
    """
    palm_offset = np.asarray(
        palm_offset if palm_offset is not None else [0.0, 0.0, 0.0],
        dtype=float,
    )

    samples = []
    for pose_name, manus_tips in manus_pose_samples.items():
        if pose_name not in dg5f_pose_tips:
            continue
        for finger_name in MANUS_FINGER_ORDER:
            if finger_name not in manus_tips or finger_name not in dg5f_pose_tips[pose_name]:
                continue
            manus_tip = apply_axis_mapping(manus_tips[finger_name], axes, signs) * float(hand_scale)
            dg5f_tip = np.asarray(dg5f_pose_tips[pose_name][finger_name], dtype=float) - palm_offset
            samples.append((finger_name, manus_tip, dg5f_tip))

    if not samples:
        raise ValueError('No overlapping Manus/DG5F multi-pose samples were provided')

    axis_scale = np.ones(3, dtype=float)
    finger_scale = {name: 1.0 for name in MANUS_FINGER_ORDER}

    for _ in range(max(1, int(iterations))):
        for axis in range(3):
            numerator = 0.0
            denominator = 0.0
            for finger_name, manus_tip, dg5f_tip in samples:
                value = finger_scale[finger_name] * manus_tip[axis]
                numerator += value * dg5f_tip[axis]
                denominator += value * value
            if denominator > 1e-9:
                axis_scale[axis] = np.clip(numerator / denominator, 0.25, 4.0)

        for finger_name in MANUS_FINGER_ORDER:
            numerator = 0.0
            denominator = 0.0
            for sample_finger, manus_tip, dg5f_tip in samples:
                if sample_finger != finger_name:
                    continue
                value = axis_scale * manus_tip
                numerator += float(np.dot(value, dg5f_tip))
                denominator += float(np.dot(value, value))
            if denominator > 1e-9:
                finger_scale[finger_name] = float(np.clip(numerator / denominator, 0.25, 4.0))

        # Remove the global scale ambiguity by normalizing the non-thumb finger
        # scales back around 1.0 and absorbing that factor into the axis scale.
        non_thumb_mean = np.mean([finger_scale[name] for name in MANUS_FINGER_ORDER[1:]])
        if non_thumb_mean > 1e-9:
            axis_scale *= non_thumb_mean
            for name in MANUS_FINGER_ORDER:
                finger_scale[name] /= non_thumb_mean

    residual = 0.0
    point_count = 0
    for finger_name, manus_tip, dg5f_tip in samples:
        predicted = axis_scale * finger_scale[finger_name] * manus_tip
        residual += float(np.sum((predicted - dg5f_tip) ** 2))
        point_count += 3

    return {
        'workspace_axis_scale': axis_scale.tolist(),
        'finger_target_scales': [float(finger_scale[name]) for name in MANUS_FINGER_ORDER],
        'multipose_residual': residual,
        'multipose_rmse': float(np.sqrt(residual / max(1, point_count))),
    }


def load_calibration(path: str = DEFAULT_CALIBRATION_PATH) -> dict | None:
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or None


def save_calibration(calibration: dict, path: str = DEFAULT_CALIBRATION_PATH):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(calibration, handle, sort_keys=False)
