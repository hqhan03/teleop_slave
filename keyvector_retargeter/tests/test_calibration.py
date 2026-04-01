import numpy as np

from keyvector_retargeter.calibration import CALIBRATION_POSES, CalibrationPoseCapture, default_calibration, fit_stage2_calibration
from keyvector_retargeter.dg5f_model import DG5FHandModel, find_dg5f_urdf
from keyvector_retargeter.manus_joints import manus_joint_prior_basis
from keyvector_retargeter.manus_landmarks import hand_from_loss_keypoints


def _pose_by_name(name: str) -> dict:
    for pose in CALIBRATION_POSES:
        if pose['name'] == name:
            return pose
    raise AssertionError(f'Missing calibration pose {name}')


def _fingertip_distances(model: DG5FHandModel, pose_name: str) -> dict[str, float]:
    pose = _pose_by_name(pose_name)
    q = np.deg2rad(np.asarray(pose['joint_positions_deg'], dtype=float))
    tips = {
        finger: keypoints['tip']
        for finger, keypoints in model.get_virtual_keypoints(q).items()
    }
    return {
        'thumb_index': float(np.linalg.norm(tips['thumb'] - tips['index'])),
        'thumb_middle': float(np.linalg.norm(tips['thumb'] - tips['middle'])),
        'thumb_ring': float(np.linalg.norm(tips['thumb'] - tips['ring'])),
        'thumb_pinky': float(np.linalg.norm(tips['thumb'] - tips['pinky'])),
        'index_middle': float(np.linalg.norm(tips['index'] - tips['middle'])),
    }


def test_stage2_pose_list_includes_tripod_precision_grasp():
    pose_names = [pose['name'] for pose in CALIBRATION_POSES]
    assert 'tripod_precision_grasp' in pose_names
    assert 'thumb_ring_pinch' in pose_names
    assert 'thumb_pinky_pinch' in pose_names


def test_thumb_index_pinch_pose_is_actual_near_contact_in_urdf_fk():
    model = DG5FHandModel(find_dg5f_urdf())
    distances = _fingertip_distances(model, 'thumb_index_pinch')

    assert distances['thumb_index'] < 0.005
    assert distances['thumb_middle'] > 0.10
    assert distances['thumb_ring'] > 0.10
    assert distances['thumb_pinky'] > 0.10


def test_thumb_middle_pinch_pose_is_actual_near_contact_in_urdf_fk():
    model = DG5FHandModel(find_dg5f_urdf())
    distances = _fingertip_distances(model, 'thumb_middle_pinch')

    assert distances['thumb_middle'] < 0.005
    assert distances['thumb_index'] > 0.10
    assert distances['thumb_ring'] > 0.10
    assert distances['thumb_pinky'] > 0.10


def test_thumb_ring_pinch_pose_is_actual_near_contact_in_urdf_fk():
    model = DG5FHandModel(find_dg5f_urdf())
    distances = _fingertip_distances(model, 'thumb_ring_pinch')

    assert distances['thumb_ring'] < 0.005
    assert distances['thumb_index'] > 0.10
    assert distances['thumb_middle'] > 0.10
    assert distances['thumb_pinky'] > 0.10


def test_thumb_pinky_pinch_pose_is_actual_near_contact_in_urdf_fk():
    model = DG5FHandModel(find_dg5f_urdf())
    distances = _fingertip_distances(model, 'thumb_pinky_pinch')

    assert distances['thumb_pinky'] < 0.005
    assert distances['thumb_index'] > 0.10
    assert distances['thumb_middle'] > 0.10
    assert distances['thumb_ring'] > 0.10


def test_tripod_precision_grasp_clusters_thumb_index_and_middle():
    model = DG5FHandModel(find_dg5f_urdf())
    distances = _fingertip_distances(model, 'tripod_precision_grasp')

    assert distances['thumb_index'] < 0.015
    assert distances['thumb_middle'] < 0.015
    assert distances['index_middle'] < 0.02
    assert distances['thumb_ring'] > 0.10
    assert distances['thumb_pinky'] > 0.10


def test_power_grasp_pose_is_more_closed_than_the_previous_medium_grasp():
    model = DG5FHandModel(find_dg5f_urdf())
    old_medium_power_grasp_deg = np.asarray([
        15.0, -60.0, 0.25 * 180.0 / np.pi, 0.2 * 180.0 / np.pi,
        0.0, 70.0, 45.0, 30.0,
        0.0, 70.0, 45.0, 30.0,
        0.0, 65.0, 45.0, 30.0,
        20.0, 20.0, 45.0, 30.0,
    ], dtype=float)
    old_q = np.deg2rad(old_medium_power_grasp_deg)
    old_tips = {
        finger: keypoints['tip']
        for finger, keypoints in model.get_virtual_keypoints(old_q).items()
    }
    old_distances = {
        'thumb_index': float(np.linalg.norm(old_tips['thumb'] - old_tips['index'])),
        'thumb_middle': float(np.linalg.norm(old_tips['thumb'] - old_tips['middle'])),
        'thumb_ring': float(np.linalg.norm(old_tips['thumb'] - old_tips['ring'])),
        'thumb_pinky': float(np.linalg.norm(old_tips['thumb'] - old_tips['pinky'])),
    }
    new_distances = _fingertip_distances(model, 'power_grasp')

    assert new_distances['thumb_index'] < old_distances['thumb_index']
    assert new_distances['thumb_middle'] < old_distances['thumb_middle']
    assert new_distances['thumb_ring'] < old_distances['thumb_ring']
    assert new_distances['thumb_pinky'] < old_distances['thumb_pinky']

    power_grasp = _pose_by_name('power_grasp')
    joint_positions_deg = np.asarray(power_grasp['joint_positions_deg'], dtype=float)
    assert joint_positions_deg[5] == 115.0
    assert joint_positions_deg[6] == 90.0
    assert joint_positions_deg[7] == 90.0
    assert joint_positions_deg[9] == 115.0
    assert joint_positions_deg[10] == 90.0
    assert joint_positions_deg[11] == 90.0
    assert joint_positions_deg[13] == 110.0
    assert joint_positions_deg[14] == 90.0
    assert joint_positions_deg[15] == 90.0
    assert joint_positions_deg[18] == 90.0
    assert joint_positions_deg[19] == 90.0


def test_stage2_joint_prior_fit_reproduces_synthetic_robot_targets():
    model = DG5FHandModel(find_dg5f_urdf())
    calibration = default_calibration()
    sample_sets_deg = [
        [5, 35, 15, 8, 0, 30, 45, 10, 0, 25, 35, 10, 0, 20, 25, 8, 5, 20, 15, 8],
        [10, 50, 25, 15, 5, 45, 60, 20, 2, 40, 55, 18, 2, 35, 45, 15, 8, 35, 28, 15],
        [20, 70, 35, 25, -5, 60, 70, 25, -2, 55, 65, 20, 4, 45, 50, 20, 12, 50, 35, 20],
        [30, 90, 45, 30, -10, 80, 80, 30, -5, 70, 75, 25, 5, 60, 60, 25, 15, 65, 40, 25],
        [15, 60, 40, 20, -15, 55, 50, 15, 10, 30, 40, 10, 8, 25, 35, 10, 15, 30, 18, 10],
        [8, 40, 18, 12, 10, 20, 30, 8, 12, 15, 25, 6, 10, 18, 20, 8, 6, 10, 12, 6],
        [25, 85, 38, 28, 15, 75, 75, 28, 18, 68, 70, 24, 12, 55, 55, 22, 10, 58, 42, 24],
    ]

    captures = []
    for idx, sample_deg in enumerate(sample_sets_deg):
        manus_joint_positions = np.deg2rad(np.asarray(sample_deg, dtype=float))
        robot_joint_positions = model.clip_to_limits(manus_joint_prior_basis(manus_joint_positions))
        robot_keypoints = model.get_virtual_keypoints(robot_joint_positions)
        captures.append(CalibrationPoseCapture(
            name=f'synth_{idx}',
            description='Synthetic stage-2 capture for joint-prior fitting.',
            hand=hand_from_loss_keypoints(robot_keypoints),
            robot_keypoints=robot_keypoints,
            robot_joint_positions=robot_joint_positions,
            manus_joint_positions=manus_joint_positions,
        ))

    updated = fit_stage2_calibration(captures, calibration)

    assert updated['joint_prior_stage2_success']
    gains = np.asarray(updated['joint_prior_gains'], dtype=float)
    offsets = np.asarray(updated['joint_prior_offsets'], dtype=float)
    assert gains.shape == (20,)
    assert offsets.shape == (20,)

    reconstructed = []
    targets = []
    for capture in captures:
        basis = manus_joint_prior_basis(capture.manus_joint_positions)
        reconstructed.append(gains * basis + offsets)
        targets.append(capture.robot_joint_positions)
    reconstructed = np.asarray(reconstructed, dtype=float)
    targets = np.asarray(targets, dtype=float)
    rmse = float(np.sqrt(np.mean((reconstructed - targets) ** 2)))
    assert rmse < 0.03
