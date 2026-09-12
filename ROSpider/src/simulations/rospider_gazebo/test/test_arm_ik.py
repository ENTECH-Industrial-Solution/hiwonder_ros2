import math

import numpy as np

from rospider_gazebo import arm_ik


def test_zero_pose_matches_urdf_chain():
    position, pitch = arm_ik.forward_kinematics((0.0, 0.0, 0.0, 0.0, 0.0))
    np.testing.assert_allclose(
        position, (0.062415239, 0.004078388, 0.415684349), atol=1e-7)
    assert math.isclose(pitch, -1.5563673267948994, abs_tol=1e-9)


def test_init_pose_tilts_52_degrees_down():
    # controller/config/init_pose.yaml; CLAUDE.md records this pose as "camera
    # tilted 52 deg down at the floor".
    _, pitch = arm_ik.forward_kinematics((0.0, 0.628, -1.927, -1.194, 0.0))
    assert math.isclose(math.degrees(pitch), 52.012, abs_tol=0.01)


def test_constants_match_the_urdf():
    assert math.isclose(arm_ik.MAX_REACH, 0.290195, abs_tol=1e-6)
    assert math.isclose(arm_ik.WRIST_REACH, 0.160563, abs_tol=1e-6)
    assert arm_ik.JOINT_NAMES == (
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5')


CUBE_Z = 0.105 - arm_ik.BASE_LINK_HEIGHT      # cube centre, base_link frame
PITCHES = (90.0, 80.0, 70.0, 60.0, 50.0, 40.0, 30.0)
LAYOUT = {                                    # spec section 5.1
    'red': (0.235, 0.07),
    'green': (0.235, 0.0),
    'blue': (0.235, -0.07),
    'drop': (0.160, 0.0),
}


def test_round_trip_over_the_pedestal_workspace():
    checked = 0
    for x in np.arange(0.16, 0.28, 0.02):
        for y in np.arange(-0.09, 0.091, 0.03):
            for pitch_deg in (60.0, 75.0, 90.0):
                target = np.array([x, y, CUBE_Z])
                pitch = math.radians(pitch_deg)
                q = arm_ik.inverse_kinematics(target, pitch)
                if q is None:
                    continue
                position, actual = arm_ik.forward_kinematics(q)
                assert np.linalg.norm(position - target) < 1e-3
                assert abs(actual - pitch) < math.radians(0.5)
                checked += 1
    assert checked > 50, f'only {checked} poses solved; the workspace moved'


def test_out_of_reach_returns_none():
    far = np.array([arm_ik.SHOULDER[0] + arm_ik.MAX_REACH + 0.02, 0.0, CUBE_Z])
    assert all(arm_ik.inverse_kinematics(far, math.radians(p)) is None
               for p in range(-80, 81, 5))
    assert arm_ik.inverse_kinematics(arm_ik.SHOULDER, math.radians(90)) is None


def test_reach_limit_brackets_the_solvable_set():
    limit = arm_ik.reach_limit(CUBE_Z)
    assert math.isclose(limit, 0.255980, abs_tol=1e-5)
    inside = np.array([arm_ik.SHOULDER[0] + limit - 0.002, 0.0, CUBE_Z])
    outside = np.array([arm_ik.SHOULDER[0] + limit + 0.002, 0.0, CUBE_Z])
    assert any(arm_ik.inverse_kinematics(inside, math.radians(p)) is not None
               for p in range(-80, 81, 1))
    assert all(arm_ik.inverse_kinematics(outside, math.radians(p)) is None
               for p in range(-80, 81, 1))


def test_solutions_respect_joint_limits():
    q = arm_ik.inverse_kinematics((0.235, 0.0, CUBE_Z), math.radians(80))
    assert q is not None
    for value, (low, high) in zip(q, arm_ik.JOINT_LIMITS):
        assert low <= value <= high


def test_scene_layout_is_graspable():
    for name, (x, y) in LAYOUT.items():
        plan = arm_ik.plan_grasp((x, y, CUBE_Z), PITCHES, back_off=0.04)
        assert plan is not None, f'{name} has no workable approach'
        assert plan.margin > math.radians(10.0), (
            f'{name} sits {math.degrees(plan.margin):.1f} deg from a joint limit')
