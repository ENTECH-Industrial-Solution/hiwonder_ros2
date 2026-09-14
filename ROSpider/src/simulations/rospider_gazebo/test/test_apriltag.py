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


def test_quaternion_from_rvec_matches_the_rotation_matrix():
    # Checked against cv2.Rodrigues rather than a hand-written expected
    # quaternion: the property that matters is that rotating a vector by the
    # quaternion and by the matrix give the same answer.
    for axis_angle in ([0.0, 0.0, 0.0],
                       [0.3, -0.2, 1.1],
                       [math.pi - 1e-6, 0.0, 0.0],
                       [0.0, math.pi - 1e-6, 0.0],
                       [0.0, 0.0, math.pi - 1e-6]):
        rvec = np.array(axis_angle, dtype=np.float64).reshape(3, 1)
        matrix, _ = cv2.Rodrigues(rvec)
        x, y, z, w = tags.quaternion_from_rvec(rvec)
        assert math.isclose(x * x + y * y + z * z + w * w, 1.0, abs_tol=1e-9)
        from_quat = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        np.testing.assert_allclose(from_quat, matrix, atol=1e-9)


import importlib.util  # noqa: E402  (grouped with the tool-loading helpers)
import pathlib  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402

_PKG_ROOT = pathlib.Path(__file__).resolve().parent.parent


def _load_tool():
    """tools/ is not an installed package; load the script by path."""
    spec = importlib.util.spec_from_file_location(
        'make_tag_textures', _PKG_ROOT / 'tools' / 'make_tag_textures.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _boxes(sdf_text):
    """{visual or collision name: (sx, sy, sz)} for every box in the model."""
    root = ET.fromstring(sdf_text)
    out = {}
    for element in root.iter():
        if element.tag not in ('visual', 'collision'):
            continue
        size = element.find('.//box/size')
        if size is not None:
            out[element.get('name')] = tuple(
                round(float(v), 6) for v in size.text.split())
    return out


def test_committed_station_sdf_matches_geometry():
    # The SDF on disk is generated, but nothing stops someone editing it by
    # hand. If it drifts from tags.py the solver's distances stay right while
    # the rendered tag is the wrong size -- a failure with no visible symptom
    # except systematically wrong poses.
    committed = (_PKG_ROOT / 'models' / 'tag_station_0' / 'model.sdf').read_text()
    assert committed == _load_tool().station_sdf(0)


def test_station_boxes_match_tags_module():
    boxes = _boxes(_load_tool().station_sdf(0))
    assert boxes['pedestal_v'] == tuple(round(v, 6) for v in tags.PEDESTAL_SIZE)
    assert boxes['board_v'] == (round(tags.BOARD_THICKNESS, 6),
                                round(tags.BOARD_FACE, 6),
                                round(tags.BOARD_FACE, 6))
    assert boxes['post_v'] == tuple(round(v, 6) for v in tags.POST_SIZE)


def test_committed_texture_is_detectable():
    image = cv2.imread(
        str(_PKG_ROOT / 'worlds' / 'textures' / 'tag_0.png'),
        cv2.IMREAD_GRAYSCALE)
    assert image is not None, 'worlds/textures/tag_0.png is missing'
    assert [tag_id for tag_id, _ in tags.detect_tags(image)] == [0]


def test_detector_parameters_defaults_match_stock_aruco():
    # The GUI shows these as the starting point, so they must be exactly
    # what detect_tags used before parameters existed at all.
    stock = cv2.aruco.DetectorParameters()
    built = tags.detector_parameters()
    assert built.adaptiveThreshWinSizeMin == stock.adaptiveThreshWinSizeMin
    assert built.adaptiveThreshWinSizeMax == stock.adaptiveThreshWinSizeMax
    assert built.adaptiveThreshWinSizeStep == stock.adaptiveThreshWinSizeStep
    assert built.adaptiveThreshConstant == stock.adaptiveThreshConstant
    assert built.minMarkerPerimeterRate == stock.minMarkerPerimeterRate
    assert built.polygonalApproxAccuracyRate == stock.polygonalApproxAccuracyRate
    assert built.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_NONE


def test_detector_parameters_maps_every_key():
    built = tags.detector_parameters({
        'adaptive_thresh_win_size_min': 5,
        'adaptive_thresh_win_size_max': 41,
        'adaptive_thresh_win_size_step': 4,
        'adaptive_thresh_constant': 9.5,
        'min_marker_perimeter_rate': 0.05,
        'polygonal_approx_accuracy_rate': 0.08,
        'corner_refinement': 'subpix',
    })
    assert built.adaptiveThreshWinSizeMin == 5
    assert built.adaptiveThreshWinSizeMax == 41
    assert built.adaptiveThreshWinSizeStep == 4
    assert built.adaptiveThreshConstant == 9.5
    assert built.minMarkerPerimeterRate == 0.05
    assert built.polygonalApproxAccuracyRate == 0.08
    assert built.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_SUBPIX


def test_detector_parameters_rounds_even_windows_up():
    # aruco's adaptive threshold needs odd windows; an even slider value
    # must not silently become something other than the next odd one.
    built = tags.detector_parameters({'adaptive_thresh_win_size_min': 4,
                                      'adaptive_thresh_win_size_max': 20})
    assert built.adaptiveThreshWinSizeMin == 5
    assert built.adaptiveThreshWinSizeMax == 21


def test_detector_parameters_rejects_unknown_keys():
    import pytest
    with pytest.raises(KeyError):
        tags.detector_parameters({'adaptive_thresh_constant': 7.0,
                                  'made_up': 1})
    with pytest.raises(ValueError):
        tags.detector_parameters({'corner_refinement': 'contour'})


def test_detector_parameters_rejects_min_window_above_max():
    # A min above max makes aruco try zero scales; the tag then just
    # silently stops being found, so this must raise instead.
    import pytest
    with pytest.raises(ValueError):
        tags.detector_parameters({'adaptive_thresh_win_size_min': 41,
                                  'adaptive_thresh_win_size_max': 21})


def test_detect_tags_accepts_explicit_parameters():
    image = tags.generate_tag_image(3)
    params = tags.detector_parameters({'corner_refinement': 'subpix'})
    assert [tag_id for tag_id, _ in tags.detect_tags(image, params)] == [3]
