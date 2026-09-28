import math

import pytest
from rospider_gazebo import servo_map


def test_init_pulse_is_zero_radians():
    for joint in servo_map.SERVOS:
        assert servo_map.pulse_to_rad(joint, 500) == 0.0


def test_arm_pulses_match_the_robot_init_pose():
    """driver/controller/config/init_pose.yaml, as the URDF's start pose.

    urdf/rospider_gazebo.urdf.xacro starts joint2 at 0.628, joint3 at -1.927
    and joint4 at -1.194 rad, and says those are the robot's init action,
    which is servo 20 at 650, 21 at 40 and 22 at 215. If this drifts, every
    ported demo's look pose points somewhere else than the robot's.
    """
    assert servo_map.pulse_to_rad('joint2', 650) == pytest.approx(0.628, abs=5e-3)
    assert servo_map.pulse_to_rad('joint3', 40) == pytest.approx(-1.927, abs=5e-3)
    assert servo_map.pulse_to_rad('joint4', 215) == pytest.approx(-1.194, abs=5e-3)


def test_horizontal_camera_pulse():
    """arm_pose:=horizontal is joint4 at -0.286 rad; upstream face_track starts at 432."""
    assert servo_map.pulse_to_rad('joint4', 432) == pytest.approx(-0.286, abs=5e-3)


def test_a_flipped_joint_turns_the_other_way():
    # Both coxa servos have their min pulse (1000) above their max (0), so
    # more pulse is less angle; femur_RF is the plain way round. Which legs
    # are flipped is not symmetric -- it is whatever the robot's own
    # servo_controller.yaml says -- so this pins the two cases apart.
    assert servo_map.pulse_to_rad('coxa_LF_joint', 600) < 0
    assert servo_map.pulse_to_rad('coxa_RF_joint', 600) < 0
    assert servo_map.pulse_to_rad('femur_RF_joint', 600) > 0
    assert (servo_map.pulse_to_rad('femur_LF_joint', 600)
            == -servo_map.pulse_to_rad('femur_RF_joint', 600))


def test_full_travel_is_240_degrees():
    span = (servo_map.pulse_to_rad('joint1', 1000)
            - servo_map.pulse_to_rad('joint1', 0))
    assert math.degrees(span) == pytest.approx(240.0)


def test_pulses_are_clamped_to_the_servo_range():
    assert (servo_map.pulse_to_rad('joint1', 5000)
            == servo_map.pulse_to_rad('joint1', 1000))
    assert (servo_map.pulse_to_rad('joint1', -5000)
            == servo_map.pulse_to_rad('joint1', 0))
    assert servo_map.rad_to_pulse('joint1', 100.0) == 1000
    assert servo_map.rad_to_pulse('joint1', -100.0) == 0


def test_round_trip():
    for joint in servo_map.SERVOS:
        for pulse in (120, 300, 500, 700, 880):
            angle = servo_map.pulse_to_rad(joint, pulse)
            assert servo_map.rad_to_pulse(joint, angle) == pytest.approx(pulse)


def test_positions_to_angles_reads_a_hiwonder_call():
    # The look pose upstream color_recognition sends.
    angles = servo_map.positions_to_angles(
        ((19, 500), (20, 750), (21, 200), (22, 150), (23, 500), (24, 700)))
    assert set(angles) == {'joint1', 'joint2', 'joint3', 'joint4', 'joint5',
                           'r_joint'}
    assert angles['joint1'] == 0.0
    assert angles['joint2'] == pytest.approx(math.radians(60.0))
    assert angles['joint4'] == pytest.approx(math.radians(-84.0))


def test_leg_servo_ids_match_the_pose_control_mapping():
    """Upstream pose_control mirrors arms onto ids 5/3/1 and 6/4/2."""
    assert servo_map.joint_name(5) == 'coxa_LF_joint'
    assert servo_map.joint_name(3) == 'femur_LF_joint'
    assert servo_map.joint_name(1) == 'tibla_LF_joint'
    assert servo_map.joint_name(6) == 'coxa_RF_joint'
    assert servo_map.joint_name(4) == 'femur_RF_joint'
    assert servo_map.joint_name(2) == 'tibla_RF_joint'


def test_controller_joint_lists_cover_every_servo():
    listed = set(servo_map.LEG_JOINTS + servo_map.ARM_JOINTS
                 + servo_map.GRIPPER_JOINTS)
    assert listed == set(servo_map.SERVOS)


def test_unknown_names_and_ids_raise():
    with pytest.raises(KeyError):
        servo_map.pulse_to_rad('no_such_joint', 500)
    with pytest.raises(KeyError):
        servo_map.joint_name(99)
