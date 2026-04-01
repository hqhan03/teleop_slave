import numpy as np

from keyvector_retargeter.keyvector_loss import (
    KEYVECTOR_NAMES,
    KEYVECTOR_SPECS,
    SET_S1,
    SET_S2,
    build_target_vectors,
    compute_keyvectors,
    default_beta_dict,
)


def _synthetic_keypoints():
    return {
        'thumb': {'mcp': np.array([0.0, 0.0, 0.0]), 'pip': np.array([0.0, 0.5, 0.0]), 'dip': np.array([0.0, 1.0, 0.0]), 'tip': np.array([0.0, 1.5, 0.0])},
        'index': {'mcp': np.array([1.0, 0.0, 0.0]), 'pip': np.array([1.0, 0.7, 0.0]), 'dip': np.array([1.0, 1.1, 0.0]), 'tip': np.array([1.0, 1.6, 0.0])},
        'middle': {'mcp': np.array([2.0, 0.0, 0.0]), 'pip': np.array([2.0, 0.7, 0.0]), 'dip': np.array([2.0, 1.1, 0.0]), 'tip': np.array([2.0, 1.6, 0.0])},
        'ring': {'mcp': np.array([3.0, 0.0, 0.0]), 'pip': np.array([3.0, 0.7, 0.0]), 'dip': np.array([3.0, 1.1, 0.0]), 'tip': np.array([3.0, 1.6, 0.0])},
        'pinky': {'mcp': np.array([4.0, 0.0, 0.0]), 'pip': np.array([4.0, 0.7, 0.0]), 'dip': np.array([4.0, 1.1, 0.0]), 'tip': np.array([4.0, 1.6, 0.0])},
    }


def test_exact_keyvector_topology():
    assert len(KEYVECTOR_NAMES) == 15
    assert len([spec for spec in KEYVECTOR_SPECS if spec.vector_set == SET_S1]) == 4
    assert len([spec for spec in KEYVECTOR_SPECS if spec.vector_set == SET_S2]) == 6


def test_keyvector_values_and_contact_targets():
    keypoints = _synthetic_keypoints()
    vectors = compute_keyvectors(keypoints)
    np.testing.assert_allclose(vectors['index_mcp_to_index_tip'], [0.0, 1.6, 0.0])
    np.testing.assert_allclose(vectors['index_pip_to_middle_pip'], [1.0, 0.0, 0.0])

    betas = default_beta_dict()
    vectors['index_tip_to_thumb_tip'] = np.array([0.0, 0.001, 0.0])
    target_bundle = build_target_vectors(
        human_vectors=vectors,
        betas=betas,
        epsilon_contact_m=0.015,
        eta_thumb_close_m=0.008,
        eta_interfinger_sep_m=0.018,
    )
    assert np.isclose(target_bundle.weights['index_tip_to_thumb_tip'], 200.0)
    np.testing.assert_allclose(target_bundle.vectors['index_tip_to_thumb_tip'], [0.0, 0.008, 0.0])
