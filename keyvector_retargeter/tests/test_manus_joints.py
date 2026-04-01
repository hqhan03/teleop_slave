import numpy as np
from sensor_msgs.msg import JointState

from keyvector_retargeter.manus_joints import (
    EXPECTED_MANUS_JOINT_NAMES,
    decode_manus_joint_state_msg,
    manus_joint_prior_basis,
)


def test_decode_joint_state_uses_expected_names_when_present():
    msg = JointState()
    msg.name = list(reversed(EXPECTED_MANUS_JOINT_NAMES))
    msg.position = [float(idx) for idx in range(len(msg.name))]

    sample = decode_manus_joint_state_msg(msg)

    expected = np.array([msg.position[msg.name.index(name)] for name in EXPECTED_MANUS_JOINT_NAMES], dtype=float)
    np.testing.assert_allclose(sample.positions_rad, expected)


def test_decode_joint_state_falls_back_to_raw_order():
    msg = JointState()
    msg.name = ['unexpected'] * len(EXPECTED_MANUS_JOINT_NAMES)
    msg.position = [0.1 * idx for idx in range(len(EXPECTED_MANUS_JOINT_NAMES))]

    sample = decode_manus_joint_state_msg(msg)

    np.testing.assert_allclose(sample.positions_rad, msg.position)


def test_vendor_joint_prior_basis_returns_finite_dg_vector():
    q_deg = np.array([
        10.0, 55.0, 25.0, 15.0,
        0.0, 45.0, 55.0, 15.0,
        0.0, 40.0, 50.0, 12.0,
        2.0, 35.0, 40.0, 10.0,
        8.0, 30.0, 20.0, 12.0,
    ], dtype=float)

    basis = manus_joint_prior_basis(np.deg2rad(q_deg))

    assert basis.shape == (20,)
    assert np.isfinite(basis).all()
