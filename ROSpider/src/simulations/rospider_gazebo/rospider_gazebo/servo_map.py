"""Hiwonder servo pulses <-> simulated joint angles.

Upstream's demos talk to the robot in servo pulses:

    set_servo_position(pub, 1, ((19, 500), (20, 750), (21, 200), ...))

The pulses mean nothing to Gazebo, which takes radians on
/arm_controller/joint_trajectory and /leg_controller/joint_trajectory. On the
real robot the translation happens in driver/servo_controller
(joint_position_controller.py): a servo turns 240 degrees over its 1000 pulse
counts, `init` is the pulse at 0 rad, and a joint whose `min` pulse is larger
than its `max` turns the other way.

SERVOS below is that table, copied from
driver/servo_controller/config/servo_controller.yaml. Keeping the pulses lets
the ported demos carry upstream's numbers unchanged, which is the only way to
compare a simulated pose against the robot's.

Pure Python on purpose: test/test_servo_map.py covers it without ROS.
"""

import math

# joint name -> (servo id, init pulse, min pulse, max pulse). min > max means
# the servo turns against the joint's own positive direction ("flipped").
SERVOS = {
    'coxa_LF_joint': (5, 500, 1000, 0),
    'femur_LF_joint': (3, 500, 1000, 0),
    'tibla_LF_joint': (1, 500, 1000, 0),
    'coxa_LM_joint': (11, 500, 1000, 0),
    'femur_LM_joint': (9, 500, 1000, 0),
    'tibla_LM_joint': (7, 500, 1000, 0),
    'coxa_LR_joint': (17, 500, 1000, 0),
    'femur_LR_joint': (15, 500, 1000, 0),
    'tibla_LR_joint': (13, 500, 1000, 0),
    'coxa_RF_joint': (6, 500, 1000, 0),
    'femur_RF_joint': (4, 500, 0, 1000),
    'tibla_RF_joint': (2, 500, 0, 1000),
    'coxa_RM_joint': (12, 500, 0, 1000),
    'femur_RM_joint': (10, 500, 0, 1000),
    'tibla_RM_joint': (8, 500, 0, 1000),
    'coxa_RR_joint': (18, 500, 1000, 0),
    'femur_RR_joint': (16, 500, 0, 1000),
    'tibla_RR_joint': (14, 500, 0, 1000),
    'joint1': (19, 500, 0, 1000),
    'joint2': (20, 500, 0, 1000),
    'joint3': (21, 500, 0, 1000),
    'joint4': (22, 500, 0, 1000),
    'joint5': (23, 500, 0, 1000),
    'r_joint': (24, 500, 0, 1000),
}

# Joint order of each simulated controller (config/ros2_controllers.yaml).
LEG_JOINTS = tuple(
    f'{part}_{leg}_joint'
    for leg in ('LF', 'LM', 'LR', 'RF', 'RM', 'RR')
    for part in ('coxa', 'femur', 'tibla')
)
ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
GRIPPER_JOINTS = ('r_joint',)

# 240 degrees over 1000 counts, as joint_position_controller.py computes it.
RADIANS_PER_COUNT = 240 / 360 * (math.pi * 2) / 1000

JOINT_OF_ID = {servo[0]: name for name, servo in SERVOS.items()}


def _entry(joint):
    try:
        return SERVOS[joint]
    except KeyError:
        raise KeyError(f'no servo mapped to joint {joint!r}') from None


def joint_name(servo_id):
    """The joint a servo id drives, e.g. 19 -> 'joint1'."""
    try:
        return JOINT_OF_ID[servo_id]
    except KeyError:
        raise KeyError(f'no joint mapped to servo id {servo_id!r}') from None


def pulse_to_rad(joint, pulse):
    """A servo pulse as the joint angle in radians, clamped to the servo's range."""
    _, init, low, high = _entry(joint)
    pulse = min(max(pulse, min(low, high)), max(low, high))
    flipped = low > high
    return (init - pulse if flipped else pulse - init) * RADIANS_PER_COUNT


def rad_to_pulse(joint, angle):
    """A joint angle in radians as a servo pulse, clamped to the servo's range."""
    _, init, low, high = _entry(joint)
    flipped = low > high
    counts = angle / RADIANS_PER_COUNT
    pulse = init - counts if flipped else init + counts
    return min(max(pulse, min(low, high)), max(low, high))


def positions_to_angles(positions):
    """[(servo id, pulse), ...] -> {joint name: angle in radians}.

    The argument is exactly what upstream hands set_servo_position(), so a
    ported demo can keep the call it had.
    """
    return {joint_name(servo_id): pulse_to_rad(joint_name(servo_id), pulse)
            for servo_id, pulse in positions}
