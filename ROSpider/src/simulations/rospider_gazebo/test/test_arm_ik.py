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
