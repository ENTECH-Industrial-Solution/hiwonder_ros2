import math

import cv2
import numpy as np

from rospider_gazebo import tags


def _camera_matrix():
    """The simulated depth camera: horizontal_fov 1.2 rad at 640x480, so
    fx = 320 / tan(0.6). See urdf/rospider_gazebo.urdf.xacro line 175."""
    fx = 320.0 / math.tan(0.6)
    return np.array([[fx, 0.0, 320.0],
                     [0.0, fx, 240.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def _project(rvec, tvec, size=tags.TAG_SIZE):
    points, _ = cv2.projectPoints(
        tags.object_points(size), rvec, tvec, _camera_matrix(), np.zeros(5))
    return points.reshape(4, 2).astype(np.float32)


def test_generated_tag_round_trips():
    # The guard against a missing quiet zone or the wrong dictionary: without
    # the white border the detector finds nothing at all, and that failure is
    # silent everywhere else.
    image = tags.generate_tag_image(7)
    found = tags.detect_tags(image)
    assert [tag_id for tag_id, _ in found] == [7]


def test_object_points_are_the_ippe_square_order():
    # SOLVEPNP_IPPE_SQUARE requires exactly this order and orientation:
    # top-left, top-right, bottom-right, bottom-left with y up.
    half = tags.TAG_SIZE / 2.0
    np.testing.assert_allclose(
        tags.object_points(),
        [[-half, half, 0.0], [half, half, 0.0],
         [half, -half, 0.0], [-half, -half, 0.0]],
        atol=1e-9)


def test_pose_recovers_known_transform():
    rvec = np.array([[0.0], [0.3], [0.0]])
    tvec = np.array([[0.02], [0.01], [1.0]])
    solved = tags.solve_tag_pose(
        _project(rvec, tvec), _camera_matrix(), np.zeros(5))
    assert solved is not None
    got_rvec, got_tvec, error = solved
    np.testing.assert_allclose(got_tvec.ravel(), tvec.ravel(), atol=1e-3)
    np.testing.assert_allclose(got_rvec.ravel(), rvec.ravel(),
                               atol=math.radians(0.5))
    assert error < 0.1


def test_ambiguity_is_resolved():
    # A planar square has two poses that reproject almost identically. This
    # fails if solvePnP is ever substituted for solvePnPGeneric: solvePnP
    # returns one of the two arbitrarily and the tag frame flips between
    # frames.
    rvec = np.array([[0.0], [math.radians(60.0)], [0.0]])
    tvec = np.array([[0.02], [0.01], [1.0]])
    solved = tags.solve_tag_pose(
        _project(rvec, tvec), _camera_matrix(), np.zeros(5))
    assert solved is not None
    got_rvec, got_tvec, _ = solved
    np.testing.assert_allclose(got_tvec.ravel(), tvec.ravel(), atol=1e-3)
    np.testing.assert_allclose(got_rvec.ravel(), rvec.ravel(),
                               atol=math.radians(0.5))


def test_board_face_matches_the_generated_texture():
    # The board is sized from the texture, not guessed: the black square must
    # come out at exactly TAG_SIZE once the texture is stretched across the
    # board face, or every distance the solver reports is wrong by the ratio.
    image = tags.generate_tag_image(0)
    module_px = image.shape[0] // (10 + 2 * tags.QUIET_MODULES)
    black_square_px = 10 * module_px
    assert math.isclose(
        tags.BOARD_FACE * black_square_px / image.shape[0],
        tags.TAG_SIZE, abs_tol=1e-9)


def test_pedestal_matches_pick_pedestal():
    # models/pick_pedestal/model.sdf is 0.14 x 0.22 x 0.08, which puts its top
    # face at z = 0.08. Matching it is what lets pick_place.yaml's drop_slots
    # z of 0.135 land a cube on a station without re-deriving anything.
    assert tags.PEDESTAL_SIZE == (0.14, 0.22, 0.08)


def test_tag_to_pedestal_top():
    # Tag frame: x = model +y, y = model +z, z = model +x (the tag faces +x).
    # Pedestal top centre is (0, 0, 0.08) in the model frame; the tag origin is
    # (BOARD_OFFSET_X, 0, TAG_CENTRE_HEIGHT).
    np.testing.assert_allclose(
        tags.tag_to_pedestal_top(),
        [0.0,
         tags.PEDESTAL_SIZE[2] - tags.TAG_CENTRE_HEIGHT,
         -tags.BOARD_OFFSET_X],
        atol=1e-9)
