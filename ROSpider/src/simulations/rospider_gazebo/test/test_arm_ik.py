import math
from pathlib import Path

import numpy as np
import yaml

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


_CONFIG_PATH = Path(__file__).resolve().parent.parent / 'config' / 'pick_place.yaml'


def _load_pick_place_params():
    with open(_CONFIG_PATH) as f:
        return yaml.safe_load(f)['pick_and_place']['ros__parameters']


_PARAMS = _load_pick_place_params()

CUBE_Z = 0.105 - arm_ik.BASE_LINK_HEIGHT      # cube centre, base_link frame

# Read from config/pick_place.yaml rather than hardcoded, so this test
# actually guards what spec section 7.5 assigned it: it must fail if someone
# shortens the pitch list or the back-off distance, not only if they move an
# object out of the workspace.
PITCHES = tuple(_PARAMS['approach_pitches_deg'])
BACK_OFF = float(_PARAMS['approach_distance'])

# Cube layout, keyed by colour, read from pick_place.yaml's `scene` (model
# name -> [x, y, z] spawn pose) -- the same source of truth
# launch/pick_place.launch.py spawns the models from -- rather than a
# hardcoded LAYOUT constant that could silently drift out of step with it.
# See spec section 4.4 and the config's `scene` comment.
LAYOUT = {
    name[len('pick_cube_'):]: (x, y)
    for name, (x, y, _z) in _PARAMS['scene'].items()
    if name.startswith('pick_cube_')
}

# The raw point localize() reports for a cube on the pedestal, in
# base_footprint, before grasp_z_offset is applied. This is a property of the
# camera/cube geometry at look_pose, not a config value, so it can't be read
# out of pick_place.yaml -- it is measured empirically and is stable to 3
# decimal places across colour and across look_pose (see
# config/pick_place.yaml's grasp_z_offset comment and task-6-report.md, Fix
# round 1 verification).
_RAW_LOCALIZED_Z_FOOTPRINT = 0.1191

# The height the node actually commands a grasp descent to, in base_link:
# the raw point above plus the configured grasp_z_offset, exactly as
# _on_localize computes it (point[2] += self.grasp_z_offset). This currently
# lands within 9e-5 m of the cube's geometric centre (0.105 m, base_footprint)
# because grasp_z_offset was tuned to cancel the raw bias -- see Finding 1 --
# but the formula tracks pick_place.yaml rather than hardcoding 0.105.
GRASP_Z = (_RAW_LOCALIZED_Z_FOOTPRINT + _PARAMS['grasp_z_offset']
           - arm_ik.BASE_LINK_HEIGHT)

# Fix round 2: a third STACKED release is not reachable at all (see
# config/pick_place.yaml) -- the arm's 2R sub-chain is too short to back off
# from shoulder height -- so the three cubes now go into a side-by-side row
# of ground-level drop_slots instead. Each slot's z is already the commanded
# release height (the old release_z_offset is baked in, not added again --
# see the config comment), so the only conversion needed here is
# base_footprint -> base_link, exactly as _on_lift computes `drop`.
# get_parameters_by_prefix isn't in play here (this test reads the yaml
# directly via PyYAML, not through rclpy), so drop_slots comes back as a
# plain nested dict; sorting by key recovers placement order.
DROP_POINTS = {
    name: (x, y, z - arm_ik.BASE_LINK_HEIGHT)
    for name, (x, y, z) in sorted(_PARAMS['drop_slots'].items())
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
    # Spec section 7 test 5: cover every cube AND every drop slot at the
    # poses the node actually commands -- the three cube grasps at GRASP_Z
    # (not the plain cube-centre CUBE_Z the node no longer targets directly,
    # though the two are numerically close by design of the grasp_z_offset
    # fix) and the three row drop_slots (Fix round 2), rather than the
    # stacked release heights that no longer exist -- a third stacked
    # release isn't solvable by any pitch at all (see
    # config/pick_place.yaml), which is exactly why the row replaced it.
    points = {name: (x, y, GRASP_Z) for name, (x, y) in LAYOUT.items()}
    points.update(DROP_POINTS)
    for name, point in points.items():
        plan = arm_ik.plan_grasp(point, PITCHES, back_off=BACK_OFF)
        assert plan is not None, f'{name} has no workable approach'
        # The 10 deg margin gate is not loosened for any slot: the row was
        # chosen precisely because every slot clears it with a healthy
        # margin (~14 deg), unlike the stacked arrangement it replaced.
        assert plan.margin > math.radians(10.0), (
            f'{name} sits {math.degrees(plan.margin):.1f} deg from a joint limit')
