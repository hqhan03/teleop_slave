"""DG-5F URDF parser and forward-kinematics model."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


PALM_LINK_NAME = 'rl_dg_palm'
PALM_FRAME_ID = PALM_LINK_NAME

DG5F_RIGHT_JOINT_NAMES = [
    'rj_dg_1_1', 'rj_dg_1_2', 'rj_dg_1_3', 'rj_dg_1_4',
    'rj_dg_2_1', 'rj_dg_2_2', 'rj_dg_2_3', 'rj_dg_2_4',
    'rj_dg_3_1', 'rj_dg_3_2', 'rj_dg_3_3', 'rj_dg_3_4',
    'rj_dg_4_1', 'rj_dg_4_2', 'rj_dg_4_3', 'rj_dg_4_4',
    'rj_dg_5_1', 'rj_dg_5_2', 'rj_dg_5_3', 'rj_dg_5_4',
]

DEFAULT_REFERENCE_JOINTS_DEG = np.zeros(len(DG5F_RIGHT_JOINT_NAMES), dtype=float)
DEFAULT_REFERENCE_JOINTS_RAD = np.deg2rad(DEFAULT_REFERENCE_JOINTS_DEG)

_DG5F_URDF_FILENAME = os.path.join('delto_m_ros2', 'dg_description', 'urdf', 'dg5f_right.urdf')


def find_dg5f_urdf() -> str:
    """Find the DG-5F right-hand URDF in common workspace locations."""
    packaged_urdf = Path(__file__).resolve().parent / 'data' / 'dg5f_right_local.urdf'
    candidates = [
        str(packaged_urdf),
        os.path.join(os.getcwd(), _DG5F_URDF_FILENAME),
        os.path.join(os.getcwd(), 'install', 'dg_description', 'share', 'dg_description', 'urdf', 'dg5f_right.urdf'),
        os.path.expanduser(os.path.join('~/Desktop/tesollo_manus_teleop', _DG5F_URDF_FILENAME)),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm <= 1e-12:
        return np.eye(3, dtype=float)
    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ], dtype=float)


def _transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


@dataclass(frozen=True)
class JointSpec:
    """URDF joint specification."""

    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None

    @property
    def origin_transform(self) -> np.ndarray:
        return _transform(_rpy_matrix(self.origin_rpy), self.origin_xyz)


@dataclass(frozen=True)
class FKResult:
    """Forward-kinematics result."""

    link_transforms: dict[str, np.ndarray]
    joint_origin_transforms: dict[str, np.ndarray]


class DG5FHandModel:
    """Forward-kinematics model for the DG-5F right hand."""

    FINGER_KEYPOINT_JOINTS = {
        'thumb': {
            'mcp': 'rj_dg_1_2',
            'pip': 'rj_dg_1_3',
            'dip': 'rj_dg_1_4',
            'tip': 'rj_dg_1_tip',
        },
        'index': {
            'mcp': 'rj_dg_2_2',
            'pip': 'rj_dg_2_3',
            'dip': 'rj_dg_2_4',
            'tip': 'rj_dg_2_tip',
        },
        'middle': {
            'mcp': 'rj_dg_3_2',
            'pip': 'rj_dg_3_3',
            'dip': 'rj_dg_3_4',
            'tip': 'rj_dg_3_tip',
        },
        'ring': {
            'mcp': 'rj_dg_4_2',
            'pip': 'rj_dg_4_3',
            'dip': 'rj_dg_4_4',
            'tip': 'rj_dg_4_tip',
        },
        'pinky': {
            'mcp': 'rj_dg_5_2',
            'pip': 'rj_dg_5_3',
            'dip': 'rj_dg_5_4',
            'tip': 'rj_dg_5_tip',
        },
    }

    def __init__(self, urdf_path: str):
        self.urdf_path = str(Path(urdf_path))
        self.joints_by_name, self.children_by_parent = self._parse_urdf(self.urdf_path)
        self.joint_names = list(DG5F_RIGHT_JOINT_NAMES)
        self.name_to_index = {name: idx for idx, name in enumerate(self.joint_names)}
        self.lower_limits = np.array([self.joints_by_name[name].lower for name in self.joint_names], dtype=float)
        self.upper_limits = np.array([self.joints_by_name[name].upper for name in self.joint_names], dtype=float)
        self.reference_joint_positions = np.copy(DEFAULT_REFERENCE_JOINTS_RAD)
        self._validate_model()

    @staticmethod
    def _parse_urdf(urdf_path: str) -> tuple[dict[str, JointSpec], dict[str, list[JointSpec]]]:
        root = ET.parse(urdf_path).getroot()
        joints_by_name: dict[str, JointSpec] = {}
        children_by_parent: dict[str, list[JointSpec]] = {}

        for joint_xml in root.findall('joint'):
            name = joint_xml.attrib.get('name', '')
            if not name.startswith('rj_dg_'):
                continue

            joint_type = joint_xml.attrib['type']
            parent = joint_xml.find('parent').attrib['link']
            child = joint_xml.find('child').attrib['link']
            origin_xml = joint_xml.find('origin')
            axis_xml = joint_xml.find('axis')
            limit_xml = joint_xml.find('limit')

            origin_xyz = np.fromstring(origin_xml.attrib.get('xyz', '0 0 0'), sep=' ', dtype=float)
            origin_rpy = np.fromstring(origin_xml.attrib.get('rpy', '0 0 0'), sep=' ', dtype=float)
            axis = np.fromstring(axis_xml.attrib.get('xyz', '0 0 0'), sep=' ', dtype=float) if axis_xml is not None else np.zeros(3, dtype=float)
            lower = float(limit_xml.attrib['lower']) if limit_xml is not None and 'lower' in limit_xml.attrib else None
            upper = float(limit_xml.attrib['upper']) if limit_xml is not None and 'upper' in limit_xml.attrib else None

            spec = JointSpec(
                name=name,
                joint_type=joint_type,
                parent=parent,
                child=child,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                axis=axis,
                lower=lower,
                upper=upper,
            )
            joints_by_name[name] = spec
            children_by_parent.setdefault(parent, []).append(spec)

        return joints_by_name, children_by_parent

    def _validate_model(self) -> None:
        missing = [name for name in self.joint_names if name not in self.joints_by_name]
        if missing:
            raise RuntimeError(f'Missing DG-5F joints in URDF: {missing}')

        for finger, keypoints in self.FINGER_KEYPOINT_JOINTS.items():
            for joint_name in keypoints.values():
                if joint_name not in self.joints_by_name:
                    raise RuntimeError(f'Missing keypoint joint {joint_name} for finger {finger}')

    def clip_to_limits(self, joint_positions: np.ndarray) -> np.ndarray:
        return np.clip(np.asarray(joint_positions, dtype=float), self.lower_limits, self.upper_limits)

    def joint_limits_deg(self) -> tuple[np.ndarray, np.ndarray]:
        return np.rad2deg(self.lower_limits), np.rad2deg(self.upper_limits)

    def forward_kinematics(self, joint_positions: np.ndarray) -> FKResult:
        q = self.clip_to_limits(joint_positions)
        q_map = {name: q[idx] for idx, name in enumerate(self.joint_names)}

        link_transforms: dict[str, np.ndarray] = {PALM_LINK_NAME: np.eye(4, dtype=float)}
        joint_origin_transforms: dict[str, np.ndarray] = {}

        def visit(link_name: str) -> None:
            parent_tf = link_transforms[link_name]
            for joint in self.children_by_parent.get(link_name, []):
                joint_tf = parent_tf @ joint.origin_transform
                joint_origin_transforms[joint.name] = joint_tf

                if joint.joint_type == 'revolute':
                    motion_tf = _transform(_axis_angle_matrix(joint.axis, q_map.get(joint.name, 0.0)), np.zeros(3, dtype=float))
                    child_tf = joint_tf @ motion_tf
                else:
                    child_tf = joint_tf

                link_transforms[joint.child] = child_tf
                visit(joint.child)

        visit(PALM_LINK_NAME)
        return FKResult(link_transforms=link_transforms, joint_origin_transforms=joint_origin_transforms)

    def get_virtual_keypoints(self, joint_positions: np.ndarray) -> dict[str, dict[str, np.ndarray]]:
        fk = self.forward_kinematics(joint_positions)
        keypoints: dict[str, dict[str, np.ndarray]] = {}
        for finger, mapping in self.FINGER_KEYPOINT_JOINTS.items():
            finger_points: dict[str, np.ndarray] = {}
            for keypoint_name, joint_name in mapping.items():
                finger_points[keypoint_name] = fk.joint_origin_transforms[joint_name][:3, 3].copy()
            keypoints[finger] = finger_points
        return keypoints

    def get_reference_keypoints(self) -> dict[str, dict[str, np.ndarray]]:
        return self.get_virtual_keypoints(self.reference_joint_positions)

    def keypoint_names_in_order(self) -> list[str]:
        ordered = []
        for finger in ('thumb', 'index', 'middle', 'ring', 'pinky'):
            for keypoint in ('mcp', 'pip', 'dip', 'tip'):
                ordered.append(f'{finger}_{keypoint}')
        return ordered
