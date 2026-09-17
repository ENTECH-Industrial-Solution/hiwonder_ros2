import numpy as np
import pytest
from rospider_gazebo import gestures


# --------------------------------------------------------------- geometry

def test_vector_2d_angle_signs():
    # Positive turning from +x towards +y (which is downwards in an image).
    assert gestures.vector_2d_angle((1, 0), (0, 1)) == pytest.approx(90.0)
    assert gestures.vector_2d_angle((1, 0), (0, -1)) == pytest.approx(-90.0)
    assert gestures.vector_2d_angle((1, 0), (1, 0)) == pytest.approx(0.0)


def test_vector_2d_angle_survives_a_zero_vector():
    """MediaPipe can put two landmarks on the same pixel; sdk.common divides
    by the product of the norms and raises there."""
    assert gestures.vector_2d_angle((0, 0), (1, 0)) == 0.0


def test_get_angle_is_the_turn_of_a_three_point_path():
    assert gestures.get_angle((0, 0), (1, 0), (2, 0)) == pytest.approx(0.0)
    assert gestures.get_angle((0, 0), (1, 0), (1, 1)) == pytest.approx(90.0)


def test_distance_and_box_center():
    assert gestures.distance((0, 0), (3, 4)) == pytest.approx(5.0)
    assert gestures.box_center((10, 20, 30, 60)) == (20, 40)


def test_landmarks_to_pixels_scales_by_the_image_size():
    class Landmark:
        def __init__(self, x, y):
            self.x, self.y = x, y

    pixels = gestures.landmarks_to_pixels((640, 480),
                                          [Landmark(0.5, 0.25),
                                           Landmark(1.0, 1.0)])
    assert pixels.tolist() == [[320.0, 120.0], [640.0, 480.0]]


# ------------------------------------------------------------ hand angles

def _hand(index_tip):
    """21 landmarks with the index finger laid out and `index_tip` placed."""
    hand = np.zeros((21, 2), dtype=float)
    hand[0] = (0.0, 0.0)          # wrist
    hand[6] = (0.0, -60.0)
    hand[7] = (0.0, -80.0)
    hand[8] = index_tip
    return hand


def test_hand_angle_reads_a_straight_finger_as_near_zero():
    assert gestures.hand_angle(_hand((0.0, -100.0)))[1] == pytest.approx(0.0)


def test_hand_angle_reads_a_curled_finger_as_bent():
    # Fingertip curled back towards the palm: the last bone points the other
    # way from the finger as a whole.
    assert gestures.hand_angle(_hand((0.0, -60.0)))[1] == pytest.approx(180.0)


# ------------------------------------------------------------- gestures

@pytest.mark.parametrize('angles, expected', [
    ([60, 90, 90, 90, 90], 'fist'),
    ([10, 10, 90, 90, 90], 'gun'),
    ([10, 90, 90, 90, 90], 'hand_heart'),
    ([60, 10, 90, 90, 90], 'one'),
    ([60, 10, 10, 90, 90], 'two'),
    ([60, 10, 10, 10, 90], 'three'),
    ([60, 90, 10, 10, 10], 'OK'),
    ([60, 10, 10, 10, 10], 'four'),
    ([10, 10, 10, 10, 10], 'five'),
    ([10, 90, 90, 90, 10], 'six'),
    ([60, 60, 60, 60, 60], 'none'),
])
def test_h_gesture(angles, expected):
    assert gestures.h_gesture(angles) == expected


def test_the_four_gestures_hand_gesture_demo_acts_on_are_distinguishable():
    """scripts/hand_gesture.py fires a move for each of these four, so no two
    of them may fall through to the same branch."""
    named = {gestures.h_gesture(a) for a in ([10, 10, 90, 90, 90],
                                             [10, 90, 90, 90, 90],
                                             [60, 90, 10, 10, 10],
                                             [60, 90, 90, 90, 90])}
    assert named == {'gun', 'hand_heart', 'OK', 'fist'}


# ------------------------------------------------------------ body poses

def _person(wrists, elbows=((480, 120), (160, 120))):
    """33 pose landmarks: only the head, shoulders, elbows and wrists matter.

    Image coordinates, so y grows downwards and "above the head" is a
    smaller y. Index 11/13/15 is one arm, 12/14/16 the other.
    """
    body = np.zeros((33, 2), dtype=float)
    for i in range(7):                    # nose, eyes, ears
        body[i] = (320.0, 140.0)
    body[11], body[12] = (400.0, 200.0), (240.0, 200.0)     # shoulders
    body[13], body[14] = elbows
    body[15], body[16] = wrists
    return body


def test_pentagon_is_hands_over_the_head():
    assert gestures.is_pentagon(_person(((360, 100), (280, 100))))


def test_hands_down_is_not_a_pentagon():
    assert not gestures.is_pentagon(_person(((560, 300), (80, 300)),
                                            elbows=((480, 260), (160, 260))))


def test_wide_hands_are_not_a_pentagon():
    # Wrists further apart than the shoulders are wide.
    assert not gestures.is_pentagon(_person(((600, 100), (40, 100))))


def test_flat_is_both_arms_straight_out():
    flat = _person(((560, 200), (80, 200)), elbows=((480, 200), (160, 200)))
    assert gestures.is_flat(flat, 30)
    assert gestures.is_level(flat)


def test_a_bent_arm_is_not_flat():
    bent = _person(((560, 320), (80, 200)), elbows=((480, 200), (160, 200)))
    assert not gestures.is_flat(bent, 30)


def test_a_tilted_person_is_not_level():
    tilted = _person(((560, 200), (80, 200)), elbows=((480, 200), (160, 200)))
    tilted[11] = (400.0, 300.0)
    assert not gestures.is_level(tilted)


def test_cross_is_a_pentagon_with_the_wrists_swapped():
    assert not gestures.is_cross(_person(((360, 100), (280, 100))))
    assert gestures.is_cross(_person(((280, 100), (360, 100))))
