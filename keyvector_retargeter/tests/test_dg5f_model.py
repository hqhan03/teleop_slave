import math

import numpy as np

from keyvector_retargeter.dg5f_model import DG5FHandModel, DG5F_RIGHT_JOINT_NAMES, find_dg5f_urdf


def test_joint_order_and_limits():
    model = DG5FHandModel(find_dg5f_urdf())
    assert model.joint_names == DG5F_RIGHT_JOINT_NAMES
    assert math.isclose(model.lower_limits[0], -0.3839724354387525)
    assert math.isclose(model.upper_limits[1], 0.0)
    assert math.isclose(model.lower_limits[5], 0.0)
    assert math.isclose(model.upper_limits[5], 2.007128639793479)


def test_virtual_keypoints_use_fixed_tip_offsets():
    model = DG5FHandModel(find_dg5f_urdf())
    keypoints = model.get_reference_keypoints()

    thumb_tip_len = np.linalg.norm(keypoints['thumb']['tip'] - keypoints['thumb']['dip'])
    index_tip_len = np.linalg.norm(keypoints['index']['tip'] - keypoints['index']['dip'])
    pinky_tip_len = np.linalg.norm(keypoints['pinky']['tip'] - keypoints['pinky']['dip'])

    assert np.isclose(thumb_tip_len, 0.0363, atol=1e-6)
    assert np.isclose(index_tip_len, 0.0255, atol=1e-6)
    assert np.isclose(pinky_tip_len, 0.0363, atol=1e-6)


def test_urdf_joint_axes_match_dg5f_semantics():
    model = DG5FHandModel(find_dg5f_urdf())

    expected_axes = {
        'rj_dg_1_1': [1.0, 0.0, 0.0],
        'rj_dg_1_2': [0.0, 0.0, 1.0],
        'rj_dg_1_3': [1.0, 0.0, 0.0],
        'rj_dg_1_4': [1.0, 0.0, 0.0],
        'rj_dg_2_1': [1.0, 0.0, 0.0],
        'rj_dg_2_2': [0.0, 1.0, 0.0],
        'rj_dg_2_3': [0.0, 1.0, 0.0],
        'rj_dg_2_4': [0.0, 1.0, 0.0],
        'rj_dg_3_1': [1.0, 0.0, 0.0],
        'rj_dg_3_2': [0.0, 1.0, 0.0],
        'rj_dg_3_3': [0.0, 1.0, 0.0],
        'rj_dg_3_4': [0.0, 1.0, 0.0],
        'rj_dg_4_1': [1.0, 0.0, 0.0],
        'rj_dg_4_2': [0.0, 1.0, 0.0],
        'rj_dg_4_3': [0.0, 1.0, 0.0],
        'rj_dg_4_4': [0.0, 1.0, 0.0],
        'rj_dg_5_1': [0.0, 0.0, 1.0],
        'rj_dg_5_2': [1.0, 0.0, 0.0],
        'rj_dg_5_3': [0.0, 1.0, 0.0],
        'rj_dg_5_4': [0.0, 1.0, 0.0],
    }

    for joint_name, expected_axis in expected_axes.items():
        np.testing.assert_allclose(model.joints_by_name[joint_name].axis, expected_axis, atol=1e-9)


def test_fk_changes_tip_positions():
    model = DG5FHandModel(find_dg5f_urdf())
    q_open = np.zeros(20, dtype=float)
    q_closed = np.zeros(20, dtype=float)
    q_closed[5] = 0.7
    q_closed[6] = 0.4
    q_closed[7] = 0.2

    open_tip = model.get_virtual_keypoints(q_open)['index']['tip']
    closed_tip = model.get_virtual_keypoints(q_closed)['index']['tip']
    assert not np.allclose(open_tip, closed_tip)
