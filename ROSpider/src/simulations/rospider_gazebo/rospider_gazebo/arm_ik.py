"""Closed-form kinematics for the ROSpider 5-DOF arm, in the base_link frame.

Hiwonder ships its arm IK as arm_kinematics/*.so, which is aarch64-only, and
MoveIt's config sets position_only_ik so it cannot constrain a grasp's
orientation. Everything here is derived from rospider_description's URDF.

Joint axes: joint1 is +Z (yaw), joint2/3/4 are all -Y (a planar 3R arm), joint5
is +Z along its own link (roll). "Pitch" below is the approach angle of the hand:
0 points horizontally forward, +pi/2 points straight down.
"""

import math
from typing import Optional, Sequence

import numpy as np

JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
JOINT_LIMITS = ((-math.pi, math.pi),) + ((-2.09, 2.09),) * 4

# base_link -> joint1 -> ... -> joint5, as (xyz, rpy, axis) straight from
# arm.urdf.xacro. The lateral terms are sub-millimetre but are kept so that
# forward_kinematics is exact and can be used to refine the idealised solution.
_CHAIN = (
    ((0.065006, 0.0, 0.082583), (0.0, 0.0, 0.0), (0.00040398, 0.0, 1.0)),
    ((-2.7885e-05, 4.5188e-04, 0.043027), (0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    ((0.0, -1.5386e-04, 0.08), (0.0, 0.0, 1.2291e-05), (0.0, -1.0, 0.0)),
    ((3.5733e-04, 1.0042e-03, 0.080556), (0.0, -0.014429, 0.0), (0.0, -1.0, 0.0)),
    ((-1.0511e-03, 2.7762e-03, 0.049547), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
)
_END_EFFECTOR_OFFSET = (0.0, 0.0, 0.08)   # link5 -> end_effector_link, fixed

SHOULDER = np.array([0.065006, 0.0, 0.125610])   # joint2 origin in base_link
L1 = 0.080000          # joint2 -> joint3
L2 = 0.080563          # joint3 -> joint4
L3 = 0.129632          # joint4 -> end_effector_link (joint5 adds no length)
MAX_REACH = L1 + L2 + L3
WRIST_REACH = L1 + L2
BASE_LINK_HEIGHT = 0.116091   # base_footprint -> base_link, base.urdf.xacro:10


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_rotation(axis: Sequence[float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _transform(xyz, rpy, axis, angle) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = _rpy(*rpy) @ _axis_rotation(axis, angle)
    m[:3, 3] = xyz
    return m


def forward_kinematics(q: Sequence[float]):
    """End effector origin in base_link, and the hand's approach pitch."""
    m = np.eye(4)
    for (xyz, rpy, axis), angle in zip(_CHAIN, q):
        m = m @ _transform(xyz, rpy, axis, angle)
    offset = np.eye(4)
    offset[:3, 3] = _END_EFFECTOR_OFFSET
    m = m @ offset
    approach = m[:3, 2]     # the hand points along its own +Z
    return m[:3, 3], -math.asin(float(np.clip(approach[2], -1.0, 1.0)))
