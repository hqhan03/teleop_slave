import numpy as np

from keyvector_retargeter.calibration import default_calibration
from keyvector_retargeter.dg5f_model import DG5FHandModel, find_dg5f_urdf
from keyvector_retargeter.keyvector_solver import KeyvectorRetargetSolver
from keyvector_retargeter.manus_landmarks import hand_from_loss_keypoints


def test_dummy_cycle_produces_bounded_command():
    model = DG5FHandModel(find_dg5f_urdf())
    calibration = default_calibration()
    target = np.zeros(20, dtype=float)
    target[1] = -0.5
    target[5] = 0.8
    target[6] = 0.5
    target[7] = 0.3

    solver = KeyvectorRetargetSolver(
        model=model,
        calibration=calibration,
        epsilon_contact_m=1e-6,
        solver_max_nfev=40,
    )
    hand = hand_from_loss_keypoints(model.get_virtual_keypoints(target))
    result = solver.solve(hand)

    assert result.success
    assert np.all(result.joint_positions >= model.lower_limits - 1e-9)
    assert np.all(result.joint_positions <= model.upper_limits + 1e-9)
