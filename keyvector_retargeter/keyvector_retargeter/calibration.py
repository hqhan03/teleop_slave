"""Calibration helpers for Manus landmark mapping and keyvector scaling."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import os
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
import yaml

from .dg5f_model import DEFAULT_REFERENCE_JOINTS_DEG, DG5F_RIGHT_JOINT_NAMES, DG5FHandModel
from .keyvector_loss import KEYVECTOR_NAMES, beta_dict_from_sequence, beta_sequence_from_dict, compute_keyvectors
from .manus_landmarks import FINGER_ORDER, ManusHandLandmarks
from .manus_joints import EXPECTED_MANUS_JOINT_NAMES, manus_joint_prior_basis


DEFAULT_CALIBRATION_PATH = os.path.expanduser('~/.ros/manus_dg5f_keyvector.yaml')
DEFAULT_WORKSPACE_AXIS_SCALE = [1.0, 1.0, 1.0]

# These stage-2 poses are chosen to be easy to mirror with the glove while
# remaining kinematically meaningful in the DG-5F URDF. The pinch and tripod
# poses were re-fit against URDF forward kinematics so the modeled fingertips
# are actually near contact instead of only looking plausible from the joint
# values alone.
CALIBRATION_POSES = [
    {
        'name': 'open_hand',
        'description': 'Open hand with all fingers extended and comfortably spread.',
        'joint_positions_deg': [0.0] * 20,
    },
    {
        'name': 'wide_spread',
        'description': 'Open hand with the fingers abducted and the thumb opened.',
        'joint_positions_deg': [40.0, 0.0, 0.0, 0.0, -20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 30.0, 20.0, 0.0, 0.0],
    },
    {
        'name': 'thumb_index_pinch',
        'description': 'URDF-validated thumb and index fingertip pinch with the other fingers kept clear.',
        'joint_positions_deg': [21.64, -70.03, 31.75, 23.51, -19.38, 76.47, 33.98, 20.37, 10.0, 5.0, 5.0, 5.0, 10.0, 5.0, 5.0, 5.0, 25.0, 10.0, 5.0, 5.0],
    },
    {
        'name': 'thumb_middle_pinch',
        'description': 'URDF-validated thumb and middle fingertip pinch with the index left clear.',
        'joint_positions_deg': [10.18, -95.14, 37.84, 28.36, -8.0, 8.0, 5.0, 3.0, -0.08, 79.46, 36.09, 20.87, 8.0, 10.0, 6.0, 4.0, 25.0, 10.0, 5.0, 5.0],
    },
    {
        'name': 'thumb_ring_pinch',
        'description': 'URDF-validated thumb and ring fingertip pinch with index, middle, and pinky kept clear.',
        'joint_positions_deg': [9.42, -114.11, 35.91, 26.84, -8.0, 8.0, 5.0, 3.0, 0.0, 12.0, 8.0, 4.0, 8.01, 75.0, 41.15, 24.93, 28.0, 12.0, 8.0, 5.0],
    },
    {
        'name': 'thumb_pinky_pinch',
        'description': 'URDF-validated thumb and pinky fingertip pinch with index, middle, and ring kept clear.',
        'joint_positions_deg': [6.11, -120.75, 33.21, 22.6, -5.0, 8.0, 5.0, 3.0, 0.0, 10.0, 7.0, 4.0, 6.0, 14.0, 10.0, 6.0, 34.52, 83.18, 67.08, 45.99],
    },
    {
        'name': 'tripod_precision_grasp',
        'description': 'A tripod precision grasp with thumb, index, and middle clustered near contact.',
        'joint_positions_deg': [13.33, -85.17, 27.3, 40.46, -31.0, 86.97, 31.35, 17.39, 22.38, 87.05, 32.71, 16.06, 10.0, 15.0, 8.0, 5.0, 24.0, 12.0, 6.0, 5.0],
    },
    {
        'name': 'power_grasp',
        'description': 'A strongly closed power grasp with all fingers driven near full flexion.',
        'joint_positions_deg': [24.76, -116.74, 90.0, 78.51, -13.36, 115.0, 90.0, 90.0, -0.47, 115.0, 90.0, 90.0, 13.09, 110.0, 90.0, 90.0, 56.37, 90.0, 90.0, 90.0],
    },
    {
        'name': 'pinky_engaged',
        'description': 'A grasp with visible pinky participation.',
        'joint_positions_deg': [10.0, -35.0, 0.2 * 180.0 / np.pi, 0.1 * 180.0 / np.pi, -5.0, 20.0, 10.0, 5.0, -5.0, 25.0, 15.0, 10.0, 5.0, 35.0, 20.0, 12.0, 30.0, 35.0, 45.0, 25.0],
    },
]


@dataclass(frozen=True)
class CalibrationPoseCapture:
    """One stage-2 calibration capture."""

    name: str
    description: str
    hand: ManusHandLandmarks
    robot_keypoints: dict[str, dict[str, np.ndarray]]
    robot_joint_positions: np.ndarray
    manus_joint_positions: np.ndarray | None = None


def _normalize(vec: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm <= 1e-10:
        raise ValueError('Cannot normalize near-zero vector')
    return vec / norm


def default_calibration() -> dict:
    return {
        'manus_to_dg5f_axes': [0, 1, 2],
        'manus_to_dg5f_signs': [1.0, 1.0, 1.0],
        'hand_scale': 1.0,
        'palm_offset': [0.0, 0.0, 0.0],
        'workspace_axis_scale': list(DEFAULT_WORKSPACE_AXIS_SCALE),
        'betas': [1.0] * len(KEYVECTOR_NAMES),
        'joint_prior_enabled': True,
        'joint_prior_gains': [1.0] * len(DG5F_RIGHT_JOINT_NAMES),
        'joint_prior_offsets': [0.0] * len(DG5F_RIGHT_JOINT_NAMES),
        'joint_prior_joint_names': list(DG5F_RIGHT_JOINT_NAMES),
        'joint_prior_stage2_success': False,
        'reference_joint_positions_deg': DEFAULT_REFERENCE_JOINTS_DEG.tolist(),
    }


def load_calibration(path: str) -> dict:
    """Load a calibration YAML file, falling back to defaults when missing."""
    calibration = default_calibration()
    expanded = os.path.expanduser(path)
    if not os.path.exists(expanded):
        return calibration

    with open(expanded, 'r', encoding='utf-8') as stream:
        loaded = yaml.safe_load(stream) or {}

    calibration.update(loaded)
    return calibration


def save_calibration(calibration: dict, path: str) -> None:
    """Save a calibration YAML file."""
    expanded = os.path.expanduser(path)
    Path(expanded).parent.mkdir(parents=True, exist_ok=True)
    with open(expanded, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(calibration, stream, sort_keys=False)


def apply_axis_mapping(vec: np.ndarray, axes, signs) -> np.ndarray:
    arr = np.asarray(vec, dtype=float)
    return np.array([
        float(signs[0]) * arr[int(axes[0])],
        float(signs[1]) * arr[int(axes[1])],
        float(signs[2]) * arr[int(axes[2])],
    ], dtype=float)


def transform_point(point: np.ndarray, calibration: dict, workspace_axis_scale=None) -> np.ndarray:
    """Transform one Manus wrist-local point into the DG-5F palm frame."""
    axis_scale = np.asarray(
        calibration['workspace_axis_scale'] if workspace_axis_scale is None else workspace_axis_scale,
        dtype=float,
    )
    mapped = apply_axis_mapping(point, calibration['manus_to_dg5f_axes'], calibration['manus_to_dg5f_signs'])
    mapped = mapped * float(calibration['hand_scale'])
    mapped = mapped * axis_scale
    mapped = mapped + np.asarray(calibration['palm_offset'], dtype=float)
    return mapped


def transform_hand(hand: ManusHandLandmarks, calibration: dict, workspace_axis_scale=None) -> ManusHandLandmarks:
    """Transform all Manus wrist-local landmarks into the DG-5F palm frame."""
    return hand.transform(lambda point: transform_point(point, calibration, workspace_axis_scale=workspace_axis_scale), frame_id='rl_dg_palm')


def _hand_basis_from_loss_keypoints(loss_keypoints: dict[str, dict[str, np.ndarray]]) -> np.ndarray:
    thumb = np.asarray(loss_keypoints['thumb']['tip'], dtype=float)
    index = np.asarray(loss_keypoints['index']['tip'], dtype=float)
    middle = np.asarray(loss_keypoints['middle']['tip'], dtype=float)
    ring = np.asarray(loss_keypoints['ring']['tip'], dtype=float)
    pinky = np.asarray(loss_keypoints['pinky']['tip'], dtype=float)

    z_axis = _normalize((index + middle + ring + pinky) / 4.0)
    y_seed = _normalize(thumb - pinky)
    x_axis = _normalize(np.cross(y_seed, z_axis))
    y_axis = _normalize(np.cross(z_axis, x_axis))
    return np.column_stack((x_axis, y_axis, z_axis))


def _compute_signed_permutation(manus_loss_keypoints, robot_loss_keypoints) -> tuple[list[int], list[float]]:
    manus_basis = _hand_basis_from_loss_keypoints(manus_loss_keypoints)
    robot_basis = _hand_basis_from_loss_keypoints(robot_loss_keypoints)
    alignment = robot_basis.T @ manus_basis

    best_score = None
    best_axes = None
    best_signs = None
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
    return best_axes, best_signs


def compute_stage1_calibration(reference_hand: ManusHandLandmarks, model: DG5FHandModel, reference_joint_positions: np.ndarray | None = None) -> dict:
    """Compute signed axis mapping, global scale, and palm offset from a reference pose sample."""
    if reference_joint_positions is None:
        reference_joint_positions = model.reference_joint_positions

    robot_keypoints = model.get_virtual_keypoints(reference_joint_positions)
    robot_loss = robot_keypoints
    human_loss = reference_hand.loss_keypoints()

    axes, signs = _compute_signed_permutation(human_loss, robot_loss)

    robot_self_lengths = []
    human_self_lengths = []
    for finger in FINGER_ORDER:
        robot_self_lengths.append(np.linalg.norm(robot_loss[finger]['tip'] - robot_loss[finger]['mcp']))
        raw_mcp = human_loss[finger]['mcp']
        raw_tip = human_loss[finger]['tip']
        mapped_mcp = apply_axis_mapping(raw_mcp, axes, signs)
        mapped_tip = apply_axis_mapping(raw_tip, axes, signs)
        human_self_lengths.append(np.linalg.norm(mapped_tip - mapped_mcp))

    robot_mean = float(np.mean(robot_self_lengths))
    human_mean = float(np.mean(human_self_lengths))
    hand_scale = 1.0 if human_mean <= 1e-10 else robot_mean / human_mean

    diffs = []
    for finger in FINGER_ORDER:
        for keypoint in ('mcp', 'pip', 'dip', 'tip'):
            mapped_point = apply_axis_mapping(human_loss[finger][keypoint], axes, signs) * hand_scale
            diffs.append(robot_loss[finger][keypoint] - mapped_point)
    palm_offset = np.mean(np.asarray(diffs, dtype=float), axis=0)

    calibration = default_calibration()
    calibration.update({
        'manus_to_dg5f_axes': axes,
        'manus_to_dg5f_signs': signs,
        'hand_scale': float(hand_scale),
        'palm_offset': palm_offset.tolist(),
        'reference_joint_positions_deg': np.rad2deg(reference_joint_positions).tolist(),
    })
    return calibration


def _fit_joint_prior_affine(captures: list[CalibrationPoseCapture]) -> tuple[np.ndarray, np.ndarray, bool, float]:
    valid_captures = [capture for capture in captures if capture.manus_joint_positions is not None]
    if not valid_captures:
        gains = np.ones(len(DG5F_RIGHT_JOINT_NAMES), dtype=float)
        offsets = np.zeros(len(DG5F_RIGHT_JOINT_NAMES), dtype=float)
        return gains, offsets, False, float('inf')

    basis_matrix = np.asarray([manus_joint_prior_basis(capture.manus_joint_positions) for capture in valid_captures], dtype=float)
    target_matrix = np.asarray([capture.robot_joint_positions for capture in valid_captures], dtype=float)

    gains = np.ones(len(DG5F_RIGHT_JOINT_NAMES), dtype=float)
    offsets = np.zeros(len(DG5F_RIGHT_JOINT_NAMES), dtype=float)
    success = True

    for joint_idx in range(len(DG5F_RIGHT_JOINT_NAMES)):
        x = basis_matrix[:, joint_idx]
        y = target_matrix[:, joint_idx]
        if np.std(x) <= 1e-8:
            gains[joint_idx] = 1.0
            offsets[joint_idx] = float(np.mean(y - x))
            success = False
            continue

        A = np.column_stack((x, np.ones_like(x)))
        gain, offset = np.linalg.lstsq(A, y, rcond=None)[0]
        if not np.isfinite(gain) or not np.isfinite(offset):
            gains[joint_idx] = 1.0
            offsets[joint_idx] = float(np.mean(y - x))
            success = False
            continue

        gains[joint_idx] = float(gain)
        offsets[joint_idx] = float(offset)

    corrected = basis_matrix * gains[None, :] + offsets[None, :]
    rmse = float(np.sqrt(np.mean((corrected - target_matrix) ** 2)))
    return gains, offsets, success, rmse


def fit_stage2_calibration(captures: list[CalibrationPoseCapture], calibration: dict) -> dict:
    """Fit per-keyvector betas and global axis-scale refinement from guided calibration poses."""
    if not captures:
        raise ValueError('No calibration captures provided')

    x0 = np.concatenate([
        np.asarray(calibration.get('workspace_axis_scale', DEFAULT_WORKSPACE_AXIS_SCALE), dtype=float),
        beta_sequence_from_dict(beta_dict_from_sequence(calibration.get('betas', [1.0] * len(KEYVECTOR_NAMES)))),
    ])
    lower = np.concatenate([np.full(3, 0.5, dtype=float), np.full(len(KEYVECTOR_NAMES), 0.1, dtype=float)])
    upper = np.concatenate([np.full(3, 2.0, dtype=float), np.full(len(KEYVECTOR_NAMES), 3.0, dtype=float)])

    def residual(params: np.ndarray) -> np.ndarray:
        axis_scale = params[:3]
        betas = params[3:]
        beta_dict = beta_dict_from_sequence(betas)
        residuals = []
        for capture in captures:
            transformed = transform_hand(capture.hand, calibration, workspace_axis_scale=axis_scale)
            human_vectors = compute_keyvectors(transformed.loss_keypoints())
            robot_vectors = compute_keyvectors(capture.robot_keypoints)
            for name in KEYVECTOR_NAMES:
                residuals.extend((robot_vectors[name] - beta_dict[name] * human_vectors[name]).tolist())
        return np.asarray(residuals, dtype=float)

    result = least_squares(residual, x0, method='trf', bounds=(lower, upper), xtol=1e-5, ftol=1e-5, gtol=1e-5, max_nfev=100)
    updated = dict(calibration)
    updated['workspace_axis_scale'] = result.x[:3].tolist()
    updated['betas'] = result.x[3:].tolist()
    residual_vector = residual(result.x)
    updated['stage2_rmse'] = float(np.sqrt(np.mean(residual_vector ** 2)))
    updated['stage2_success'] = bool(result.success)
    updated['stage2_message'] = result.message
    updated['stage2_poses'] = [capture.name for capture in captures]
    gains, offsets, joint_prior_success, joint_prior_rmse = _fit_joint_prior_affine(captures)
    updated['joint_prior_enabled'] = True
    updated['joint_prior_gains'] = gains.tolist()
    updated['joint_prior_offsets'] = offsets.tolist()
    updated['joint_prior_joint_names'] = list(DG5F_RIGHT_JOINT_NAMES)
    updated['joint_prior_stage2_success'] = bool(joint_prior_success)
    updated['joint_prior_stage2_rmse'] = joint_prior_rmse
    updated['joint_prior_source_joint_names'] = list(EXPECTED_MANUS_JOINT_NAMES)
    return updated
