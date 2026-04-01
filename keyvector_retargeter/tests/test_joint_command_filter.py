import numpy as np

from keyvector_retargeter.joint_command_filter import PerJointVelocityLimiter


JOINT_NAMES = [
    'rj_dg_1_1', 'rj_dg_1_2', 'rj_dg_1_3', 'rj_dg_1_4',
    'rj_dg_2_1', 'rj_dg_2_2', 'rj_dg_2_3', 'rj_dg_2_4',
    'rj_dg_3_1', 'rj_dg_3_2', 'rj_dg_3_3', 'rj_dg_3_4',
    'rj_dg_4_1', 'rj_dg_4_2', 'rj_dg_4_3', 'rj_dg_4_4',
    'rj_dg_5_1', 'rj_dg_5_2', 'rj_dg_5_3', 'rj_dg_5_4',
]


def test_rate_limiter_passes_small_steps_unchanged():
    limiter = PerJointVelocityLimiter(
        JOINT_NAMES,
        max_flex_velocity_deg_s=480.0,
        max_spread_velocity_deg_s=300.0,
        initial_state=np.zeros(20, dtype=float),
    )
    target = np.zeros(20, dtype=float)
    target[5] = np.deg2rad(5.0)

    output = limiter.apply(target, dt_sec=0.1)

    np.testing.assert_allclose(output, target)


def test_rate_limiter_caps_large_steps():
    limiter = PerJointVelocityLimiter(
        JOINT_NAMES,
        max_flex_velocity_deg_s=480.0,
        max_spread_velocity_deg_s=300.0,
        initial_state=np.zeros(20, dtype=float),
    )
    target = np.zeros(20, dtype=float)
    target[5] = np.deg2rad(90.0)

    output = limiter.apply(target, dt_sec=0.05)

    assert np.isclose(np.rad2deg(output[5]), 24.0, atol=1e-6)
