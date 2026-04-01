import numpy as np

from keyvector_retargeter.calibration import default_calibration
from keyvector_retargeter.dg5f_model import DG5FHandModel, find_dg5f_urdf
from keyvector_retargeter.keyvector_loss import KEYVECTOR_NAMES
from keyvector_retargeter.keyvector_solver import KeyvectorRetargetSolver
from keyvector_retargeter.manus_landmarks import hand_from_loss_keypoints


def test_solver_recovers_open_reference_pose():
    model = DG5FHandModel(find_dg5f_urdf())
    calibration = default_calibration()
    solver = KeyvectorRetargetSolver(
        model=model,
        calibration=calibration,
        epsilon_contact_m=1e-6,
    )

    hand = hand_from_loss_keypoints(model.get_reference_keypoints())
    result = solver.solve(hand)

    assert result.success
    np.testing.assert_allclose(result.joint_positions, np.zeros(20), atol=1e-4)


def test_solver_follows_synthetic_nonzero_pose():
    model = DG5FHandModel(find_dg5f_urdf())
    calibration = default_calibration()
    target = np.zeros(20, dtype=float)
    target[0] = 0.55
    target[4] = -0.20
    target[12] = 0.20
    target[16] = 0.40
    target[17] = 0.25

    hand = hand_from_loss_keypoints(model.get_virtual_keypoints(target))
    solver = KeyvectorRetargetSolver(
        model=model,
        calibration=calibration,
        epsilon_contact_m=1e-6,
        solver_max_nfev=40,
    )
    result = solver.solve(hand)

    assert result.success
    human_vectors = result.human_vectors
    robot_vectors = result.robot_vectors
    mean_vector_error = np.mean([
        np.linalg.norm(robot_vectors[name] - human_vectors[name])
        for name in KEYVECTOR_NAMES
    ])
    assert mean_vector_error < 0.06
    assert np.all(result.joint_positions >= model.lower_limits - 1e-9)
    assert np.all(result.joint_positions <= model.upper_limits + 1e-9)


def test_solver_joint_prior_improves_small_signal_responsiveness():
    model = DG5FHandModel(find_dg5f_urdf())
    calibration = default_calibration()
    solver = KeyvectorRetargetSolver(
        model=model,
        calibration=calibration,
        epsilon_contact_m=1e-6,
        lambda_smooth=0.0,
        solver_max_nfev=40,
    )

    joint_idx = model.joint_names.index('rj_dg_2_2')

    target_1deg = np.zeros(20, dtype=float)
    target_1deg[joint_idx] = np.deg2rad(1.0)
    hand_1deg = hand_from_loss_keypoints(model.get_virtual_keypoints(target_1deg))
    result_1deg = solver.solve(hand_1deg, q_prior=target_1deg)
    assert result_1deg.success
    assert np.rad2deg(result_1deg.joint_positions[joint_idx]) >= 0.7

    target_5deg = np.zeros(20, dtype=float)
    target_5deg[joint_idx] = np.deg2rad(5.0)
    hand_5deg = hand_from_loss_keypoints(model.get_virtual_keypoints(target_5deg))
    result_5deg = solver.solve(hand_5deg, q_prior=target_5deg)
    assert result_5deg.success
    assert np.rad2deg(result_5deg.joint_positions[joint_idx]) >= 4.0
