"""Closed-form kinematics for the ROSpider 5-DOF arm, in the base_link frame.

Hiwonder ships its arm IK as arm_kinematics/*.so, which is aarch64-only, and
MoveIt's config sets position_only_ik so it cannot constrain a grasp's
orientation. Everything here is derived from rospider_description's URDF.

Joint axes: joint1 is +Z (yaw), joint2/3/4 are all -Y (a planar 3R arm), joint5
is +Z along its own link (roll). "Pitch" below is the approach angle of the hand:
0 points horizontally forward, +pi/2 points straight down.
"""

import math
from typing import NamedTuple, Optional, Sequence

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


class GraspPlan(NamedTuple):
    """An approach pitch plus the two arm poses a grasp needs."""

    pitch: float
    approach: np.ndarray
    grasp: np.ndarray
    margin: float


def reach_limit(z: float) -> float:
    """Horizontal reach from the shoulder axis at base_link height z.

    This is the bare chain, with the hand free to point anywhere. A grasp fixes
    the approach direction and reaches considerably less; use plan_grasp.
    """
    drop = abs(z - SHOULDER[2])
    if drop >= MAX_REACH:
        return 0.0
    return math.sqrt(MAX_REACH * MAX_REACH - drop * drop)


def tool_axis(pitch: float, yaw: float) -> np.ndarray:
    """Unit vector the hand points along, for an approach pitch and a yaw."""
    return np.array([math.cos(pitch) * math.cos(yaw),
                     math.cos(pitch) * math.sin(yaw),
                     -math.sin(pitch)])


def _limit_margin(q: Sequence[float]) -> float:
    return min(min(high - value, value - low)
               for value, (low, high) in zip(q, JOINT_LIMITS))


def _seed_branches(target: np.ndarray, pitch: float, roll: float):
    """Both elbow solutions of the idealised planar model, or None."""
    delta = target - SHOULDER
    q1 = math.atan2(delta[1], delta[0])
    rho = math.hypot(delta[0], delta[1])
    height = delta[2]

    # Angles below are measured from +Z towards the radial direction, so the
    # last link's direction is phi = pi/2 + pitch.
    phi = math.pi / 2.0 + pitch
    wrist_rho = rho - L3 * math.sin(phi)
    wrist_height = height - L3 * math.cos(phi)

    cosine = ((wrist_rho ** 2 + wrist_height ** 2 - L1 * L1 - L2 * L2)
              / (2.0 * L1 * L2))
    if abs(cosine) > 1.0:
        return None

    branches = []
    for sign in (-1.0, 1.0):
        elbow = sign * math.acos(max(-1.0, min(1.0, cosine)))
        shoulder = (math.atan2(wrist_rho, wrist_height)
                    - math.atan2(L2 * math.sin(elbow), L1 + L2 * math.cos(elbow)))
        # URDF joints 2-4 turn about -Y, so their values are the negated
        # about-+Y angles used above.
        branches.append(np.array([q1, -shoulder, -elbow,
                                  -(phi - (shoulder + elbow)), roll]))
    return branches


def _refine(q: np.ndarray, target: np.ndarray, pitch: float) -> np.ndarray:
    """Newton steps on the exact chain, clearing the idealisation's error."""
    q = np.array(q, dtype=float)
    for _ in range(20):
        position, actual = forward_kinematics(q)
        residual = np.array([position[0] - target[0],
                             position[1] - target[1],
                             position[2] - target[2],
                             actual - pitch])
        if np.max(np.abs(residual)) < 1e-10:
            break
        jacobian = np.zeros((4, 4))
        for column in range(4):
            nudged = q.copy()
            nudged[column] += 1e-6
            moved, moved_pitch = forward_kinematics(nudged)
            jacobian[:, column] = np.array([moved[0] - position[0],
                                            moved[1] - position[1],
                                            moved[2] - position[2],
                                            moved_pitch - actual]) / 1e-6
        q[:4] += np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
    return q


def inverse_kinematics(target: Sequence[float], pitch: float,
                       roll: float = 0.0) -> Optional[np.ndarray]:
    """Joint angles reaching target with the hand at the given approach pitch.

    Returns the elbow branch with the larger joint-limit margin. Which branch
    is chosen matters: taking the first valid one leaves the pedestal grasps
    under a degree from a limit, while choosing by margin leaves tens of
    degrees.
    """
    target = np.asarray(target, dtype=float)
    branches = _seed_branches(target, pitch, roll)
    if branches is None:
        return None

    best = None
    for seed in branches:
        q = _refine(seed, target, pitch)
        position, actual = forward_kinematics(q)
        if np.linalg.norm(position - target) > 1e-4:
            continue
        if abs(actual - pitch) > 1e-4:
            continue
        if any(not low <= value <= high
               for value, (low, high) in zip(q, JOINT_LIMITS)):
            continue
        margin = _limit_margin(q)
        if best is None or margin > best[0]:
            best = (margin, q)
    return None if best is None else best[1]


def plan_grasp(target: Sequence[float], pitches_deg: Sequence[float],
               back_off: float, roll: float = 0.0) -> Optional[GraspPlan]:
    """Best approach pitch for which grasp and pre-grasp both solve.

    The pre-grasp backs off along the tool axis rather than straight up: at
    these heights a vertical hover needs more than the L1+L2 sub-chain has.
    """
    target = np.asarray(target, dtype=float)
    yaw = math.atan2(target[1] - SHOULDER[1], target[0] - SHOULDER[0])

    best = None
    for pitch_deg in pitches_deg:
        pitch = math.radians(pitch_deg)
        grasp = inverse_kinematics(target, pitch, roll)
        if grasp is None:
            continue
        approach = inverse_kinematics(
            target - back_off * tool_axis(pitch, yaw), pitch, roll)
        if approach is None:
            continue
        margin = min(_limit_margin(grasp), _limit_margin(approach))
        if best is None or margin > best.margin:
            best = GraspPlan(pitch, approach, grasp, margin)
    return best
