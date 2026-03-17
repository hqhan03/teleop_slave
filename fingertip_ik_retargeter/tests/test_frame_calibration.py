import numpy as np

from fingertip_ik_retargeter.frame_calibration import (
    apply_axis_mapping,
    compute_calibration,
    fit_retarget_scales,
)


def _quat_from_axis_angle(axis, angle_rad):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = np.sin(angle_rad / 2.0)
    return np.array([np.cos(angle_rad / 2.0), axis[0] * s, axis[1] * s, axis[2] * s], dtype=float)


def _quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def _quat_mul(a, b):
    return np.array([
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ], dtype=float)


def _rotate_vec_by_inverse_quat(vec, quat_wxyz):
    q_inv = _quat_conjugate(quat_wxyz) / np.dot(quat_wxyz, quat_wxyz)
    vec_q = np.array([0.0, vec[0], vec[1], vec[2]], dtype=float)
    result = _quat_mul(_quat_mul(q_inv, vec_q), quat_wxyz)
    return result[1:]


def _rotate_vec(vec, quat_wxyz):
    vec_q = np.array([0.0, vec[0], vec[1], vec[2]], dtype=float)
    result = _quat_mul(_quat_mul(quat_wxyz, vec_q), _quat_conjugate(quat_wxyz))
    return result[1:]


def test_palm_local_transform_is_world_invariant():
    wrist_world_a = np.array([0.3, -0.1, 0.9], dtype=float)
    wrist_world_b = np.array([-0.7, 0.4, 0.2], dtype=float)
    q_a = _quat_from_axis_angle([0.2, 0.8, 0.5], np.deg2rad(40.0))
    q_b = _quat_from_axis_angle([0.1, -0.4, 0.9], np.deg2rad(70.0))
    local_tip = np.array([0.04, 0.06, 0.18], dtype=float)

    def palm_local(wrist_pos, wrist_quat):
        # Build a world-space tip, then recover it back into the palm frame.
        tip_world = wrist_pos + _rotate_vec(local_tip, wrist_quat)
        return _rotate_vec_by_inverse_quat(tip_world - wrist_pos, wrist_quat)

    recovered_a = palm_local(wrist_world_a, q_a)
    recovered_b = palm_local(wrist_world_b, q_b)

    np.testing.assert_allclose(recovered_a, local_tip, atol=1e-6)
    np.testing.assert_allclose(recovered_b, local_tip, atol=1e-6)


def test_calibration_recovers_signed_permutation_and_scale():
    dg5f_tips = {
        'thumb': np.array([0.02575, 0.1251, 0.0128]),
        'index': np.array([0.01055, 0.0270, 0.1957]),
        'middle': np.array([0.01055, 0.0025, 0.1997]),
        'ring': np.array([0.01055, -0.0220, 0.1917]),
        'pinky': np.array([0.0103, -0.0475, 0.1624]),
    }

    axes = [2, 0, 1]
    signs = [-1.0, 1.0, 1.0]
    scale = 0.72

    def invert_mapping(pos):
        out = np.zeros(3, dtype=float)
        for dg_axis, manus_axis in enumerate(axes):
            out[manus_axis] = pos[dg_axis] / (scale * signs[dg_axis])
        return out

    manus_tips = {name: invert_mapping(pos) for name, pos in dg5f_tips.items()}
    calibration = compute_calibration(manus_tips, dg5f_tips)

    assert abs(calibration['hand_scale'] - scale) < 1e-6

    for name, manus_tip in manus_tips.items():
        mapped = apply_axis_mapping(manus_tip, calibration['manus_to_dg5f_axes'],
                                    calibration['manus_to_dg5f_signs']) * calibration['hand_scale']
        np.testing.assert_allclose(mapped, dg5f_tips[name], atol=1e-6)


def test_multipose_fit_recovers_axis_and_finger_scales():
    dg5f_pose_tips = {
        'flat_open': {
            'thumb': np.array([0.030, 0.115, 0.030]),
            'index': np.array([0.010, 0.035, 0.200]),
            'middle': np.array([0.010, 0.005, 0.205]),
            'ring': np.array([0.010, -0.022, 0.194]),
            'pinky': np.array([0.010, -0.050, 0.168]),
        },
        'pinch': {
            'thumb': np.array([0.020, 0.075, 0.095]),
            'index': np.array([0.012, 0.025, 0.115]),
            'middle': np.array([0.012, 0.010, 0.195]),
            'ring': np.array([0.012, -0.020, 0.185]),
            'pinky': np.array([0.012, -0.048, 0.158]),
        },
    }

    axes = [2, 0, 1]
    signs = [-1.0, 1.0, -1.0]
    hand_scale = 0.9
    workspace_axis_scale = np.array([1.25, 1.55, 0.95], dtype=float)
    finger_target_scales = {
        'thumb': 1.3,
        'index': 0.9,
        'middle': 1.0,
        'ring': 1.1,
        'pinky': 1.0,
    }

    def invert_mapping(pos_dg5f, finger_name):
        arr = np.asarray(pos_dg5f, dtype=float) / (
            hand_scale * workspace_axis_scale * finger_target_scales[finger_name]
        )
        out = np.zeros(3, dtype=float)
        for dg_axis, manus_axis in enumerate(axes):
            out[manus_axis] = arr[dg_axis] / signs[dg_axis]
        return out

    manus_pose_samples = {
        pose_name: {
            finger_name: invert_mapping(tip, finger_name)
            for finger_name, tip in tips.items()
        }
        for pose_name, tips in dg5f_pose_tips.items()
    }

    fit = fit_retarget_scales(
        manus_pose_samples,
        dg5f_pose_tips,
        axes,
        signs,
        hand_scale,
    )

    np.testing.assert_allclose(fit['workspace_axis_scale'], workspace_axis_scale, atol=1e-6)
    np.testing.assert_allclose(
        fit['finger_target_scales'],
        [finger_target_scales[name] for name in ['thumb', 'index', 'middle', 'ring', 'pinky']],
        atol=1e-6,
    )
