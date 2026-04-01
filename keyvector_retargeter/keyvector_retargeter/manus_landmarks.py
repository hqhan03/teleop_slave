"""Named Manus hand-landmark utilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np


CANONICAL_MANUS_HAND_FRAME = 'manus_wrist_local'
ACCEPTED_MANUS_HAND_FRAMES = ('', CANONICAL_MANUS_HAND_FRAME)

FINGER_ORDER = ('thumb', 'index', 'middle', 'ring', 'pinky')
LANDMARK_ORDER = ('metacarpal', 'proximal', 'intermediate', 'distal', 'tip')
LOSS_KEYPOINT_ORDER = ('mcp', 'pip', 'dip', 'tip')
LOSS_KEYPOINT_TO_LANDMARK = {
    'mcp': 'proximal',
    'pip': 'intermediate',
    'dip': 'distal',
    'tip': 'tip',
}


@dataclass(frozen=True)
class FingerLandmarks:
    """Named landmarks for one finger."""

    metacarpal: np.ndarray
    proximal: np.ndarray
    intermediate: np.ndarray
    distal: np.ndarray
    tip: np.ndarray

    def loss_keypoints(self) -> dict[str, np.ndarray]:
        return {
            'mcp': self.proximal.copy(),
            'pip': self.intermediate.copy(),
            'dip': self.distal.copy(),
            'tip': self.tip.copy(),
        }

    def transform(self, fn: Callable[[np.ndarray], np.ndarray]) -> 'FingerLandmarks':
        return FingerLandmarks(
            metacarpal=np.asarray(fn(self.metacarpal), dtype=float),
            proximal=np.asarray(fn(self.proximal), dtype=float),
            intermediate=np.asarray(fn(self.intermediate), dtype=float),
            distal=np.asarray(fn(self.distal), dtype=float),
            tip=np.asarray(fn(self.tip), dtype=float),
        )

    def is_all_zero(self) -> bool:
        return not any(np.linalg.norm(point) > 1e-8 for point in (
            self.metacarpal, self.proximal, self.intermediate, self.distal, self.tip))

    def is_finite(self) -> bool:
        return all(np.isfinite(point).all() for point in (
            self.metacarpal, self.proximal, self.intermediate, self.distal, self.tip))


@dataclass(frozen=True)
class ManusHandLandmarks:
    """Named Manus hand landmarks for all five fingers."""

    fingers: dict[str, FingerLandmarks]
    frame_id: str = CANONICAL_MANUS_HAND_FRAME

    def loss_keypoints(self) -> dict[str, dict[str, np.ndarray]]:
        return {finger: self.fingers[finger].loss_keypoints() for finger in FINGER_ORDER}

    def transform(self, fn: Callable[[np.ndarray], np.ndarray], frame_id: str | None = None) -> 'ManusHandLandmarks':
        return ManusHandLandmarks(
            fingers={finger: self.fingers[finger].transform(fn) for finger in FINGER_ORDER},
            frame_id=self.frame_id if frame_id is None else frame_id,
        )

    def is_all_zero(self) -> bool:
        return all(self.fingers[finger].is_all_zero() for finger in FINGER_ORDER)

    def is_finite(self) -> bool:
        return all(self.fingers[finger].is_finite() for finger in FINGER_ORDER)

    def keypoint_names_in_order(self) -> list[str]:
        ordered = []
        for finger in FINGER_ORDER:
            for keypoint in LOSS_KEYPOINT_ORDER:
                ordered.append(f'{finger}_{keypoint}')
        return ordered


def _point_from_pose(pose) -> np.ndarray:
    return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)


def decode_manus_landmarks_msg(msg) -> ManusHandLandmarks:
    """Decode a `/manus/hand_landmarks` PoseArray into a named structure."""
    if len(msg.poses) < len(FINGER_ORDER) * len(LANDMARK_ORDER):
        raise ValueError(f'Expected 25 landmark poses, got {len(msg.poses)}')

    frame_id = msg.header.frame_id or ''
    if frame_id not in ACCEPTED_MANUS_HAND_FRAMES:
        raise ValueError(f'Unexpected Manus landmark frame: {frame_id!r}')

    fingers: dict[str, FingerLandmarks] = {}
    pose_idx = 0
    for finger in FINGER_ORDER:
        points = {}
        for landmark in LANDMARK_ORDER:
            points[landmark] = _point_from_pose(msg.poses[pose_idx])
            pose_idx += 1
        fingers[finger] = FingerLandmarks(
            metacarpal=points['metacarpal'],
            proximal=points['proximal'],
            intermediate=points['intermediate'],
            distal=points['distal'],
            tip=points['tip'],
        )

    return ManusHandLandmarks(fingers=fingers, frame_id=frame_id or CANONICAL_MANUS_HAND_FRAME)


def hand_from_loss_keypoints(loss_keypoints: dict[str, dict[str, np.ndarray]], frame_id: str = CANONICAL_MANUS_HAND_FRAME) -> ManusHandLandmarks:
    """Construct a ManusHandLandmarks sample from mapped MCP/PIP/DIP/tip keypoints."""
    fingers = {}
    for finger in FINGER_ORDER:
        mcp = np.asarray(loss_keypoints[finger]['mcp'], dtype=float)
        pip = np.asarray(loss_keypoints[finger]['pip'], dtype=float)
        dip = np.asarray(loss_keypoints[finger]['dip'], dtype=float)
        tip = np.asarray(loss_keypoints[finger]['tip'], dtype=float)
        metacarpal = mcp - 0.5 * (pip - mcp)
        fingers[finger] = FingerLandmarks(
            metacarpal=metacarpal,
            proximal=mcp,
            intermediate=pip,
            distal=dip,
            tip=tip,
        )
    return ManusHandLandmarks(fingers=fingers, frame_id=frame_id)
