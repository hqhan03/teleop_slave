"""Keyvector topology and loss utilities."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


SET_SELF = 'self'
SET_S1 = 'S1'
SET_S2 = 'S2'
DEFAULT_CONTACT_WEIGHT_S1 = 200.0
DEFAULT_CONTACT_WEIGHT_S2 = 400.0


@dataclass(frozen=True)
class KeyvectorSpec:
    """One keyvector definition."""

    name: str
    origin_finger: str
    origin_keypoint: str
    target_finger: str
    target_keypoint: str
    vector_set: str


KEYVECTOR_SPECS = [
    KeyvectorSpec('thumb_mcp_to_thumb_tip', 'thumb', 'mcp', 'thumb', 'tip', SET_SELF),
    KeyvectorSpec('index_mcp_to_index_tip', 'index', 'mcp', 'index', 'tip', SET_SELF),
    KeyvectorSpec('middle_mcp_to_middle_tip', 'middle', 'mcp', 'middle', 'tip', SET_SELF),
    KeyvectorSpec('ring_mcp_to_ring_tip', 'ring', 'mcp', 'ring', 'tip', SET_SELF),
    KeyvectorSpec('pinky_mcp_to_pinky_tip', 'pinky', 'mcp', 'pinky', 'tip', SET_SELF),
    KeyvectorSpec('index_tip_to_thumb_tip', 'index', 'tip', 'thumb', 'tip', SET_S1),
    KeyvectorSpec('middle_tip_to_thumb_tip', 'middle', 'tip', 'thumb', 'tip', SET_S1),
    KeyvectorSpec('ring_tip_to_thumb_tip', 'ring', 'tip', 'thumb', 'tip', SET_S1),
    KeyvectorSpec('pinky_tip_to_thumb_tip', 'pinky', 'tip', 'thumb', 'tip', SET_S1),
    KeyvectorSpec('index_pip_to_middle_pip', 'index', 'pip', 'middle', 'pip', SET_S2),
    KeyvectorSpec('index_pip_to_ring_pip', 'index', 'pip', 'ring', 'pip', SET_S2),
    KeyvectorSpec('index_pip_to_pinky_dip', 'index', 'pip', 'pinky', 'dip', SET_S2),
    KeyvectorSpec('middle_pip_to_ring_pip', 'middle', 'pip', 'ring', 'pip', SET_S2),
    KeyvectorSpec('middle_pip_to_pinky_dip', 'middle', 'pip', 'pinky', 'dip', SET_S2),
    KeyvectorSpec('ring_pip_to_pinky_dip', 'ring', 'pip', 'pinky', 'dip', SET_S2),
]

KEYVECTOR_NAMES = [spec.name for spec in KEYVECTOR_SPECS]


@dataclass(frozen=True)
class TargetVectorBundle:
    """Target vectors and weights derived from the human hand."""

    vectors: dict[str, np.ndarray]
    weights: dict[str, float]
    distances: dict[str, float]


def compute_keyvector_endpoints(keypoints: dict[str, dict[str, np.ndarray]]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Compute origin and endpoint positions for every named keyvector."""
    endpoints = {}
    for spec in KEYVECTOR_SPECS:
        origin = np.asarray(keypoints[spec.origin_finger][spec.origin_keypoint], dtype=float)
        target = np.asarray(keypoints[spec.target_finger][spec.target_keypoint], dtype=float)
        endpoints[spec.name] = (origin, target)
    return endpoints


def compute_keyvectors(keypoints: dict[str, dict[str, np.ndarray]]) -> dict[str, np.ndarray]:
    """Compute all 15 keyvectors from named finger keypoints."""
    endpoints = compute_keyvector_endpoints(keypoints)
    return {name: target - origin for name, (origin, target) in endpoints.items()}


def default_beta_dict(default_value: float = 1.0) -> dict[str, float]:
    return {name: float(default_value) for name in KEYVECTOR_NAMES}


def beta_dict_from_sequence(values) -> dict[str, float]:
    values = list(values)
    if len(values) != len(KEYVECTOR_NAMES):
        raise ValueError(f'Expected {len(KEYVECTOR_NAMES)} beta values, got {len(values)}')
    return {name: float(val) for name, val in zip(KEYVECTOR_NAMES, values)}


def beta_sequence_from_dict(values: dict[str, float]) -> np.ndarray:
    return np.array([float(values[name]) for name in KEYVECTOR_NAMES], dtype=float)


def build_target_vectors(
    human_vectors: dict[str, np.ndarray],
    betas: dict[str, float],
    epsilon_contact_m: float,
    eta_thumb_close_m: float,
    eta_interfinger_sep_m: float,
    contact_weight_s1: float = DEFAULT_CONTACT_WEIGHT_S1,
    contact_weight_s2: float = DEFAULT_CONTACT_WEIGHT_S2,
) -> TargetVectorBundle:
    """Build target vectors and weights from human keyvectors."""
    targets: dict[str, np.ndarray] = {}
    weights: dict[str, float] = {}
    distances: dict[str, float] = {}

    for spec in KEYVECTOR_SPECS:
        human_vector = np.asarray(human_vectors[spec.name], dtype=float)
        distance = float(np.linalg.norm(human_vector))
        distances[spec.name] = distance

        if distance > 1e-10:
            direction = human_vector / distance
        else:
            direction = np.zeros(3, dtype=float)

        weight = 1.0
        target_magnitude = float(betas[spec.name]) * distance
        if distance <= epsilon_contact_m and spec.vector_set == SET_S1:
            weight = float(contact_weight_s1)
            target_magnitude = float(eta_thumb_close_m)
        elif distance <= epsilon_contact_m and spec.vector_set == SET_S2:
            weight = float(contact_weight_s2)
            target_magnitude = float(eta_interfinger_sep_m)

        targets[spec.name] = direction * target_magnitude
        weights[spec.name] = weight

    return TargetVectorBundle(vectors=targets, weights=weights, distances=distances)
