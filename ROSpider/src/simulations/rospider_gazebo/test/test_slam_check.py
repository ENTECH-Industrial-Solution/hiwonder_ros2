import numpy as np
import pytest
from rospider_gazebo import slam_check
from rospider_gazebo.slam_check import FREE, OCCUPIED, UNKNOWN, GridMap

RES = 0.05
ORIGIN = (-1.2, -2.2)
W, H = 108, 88          # 5.4 x 4.4 m: the 5 x 4 m room plus a margin


def _room(resolution=RES, origin=ORIGIN, width=W, height=H):
    """The exercise room rasterised: 0.1 m wall bands on the wall lines,
    free inside, unknown outside. Same geometry whatever the grid."""
    xs = origin[0] + (np.arange(width) + 0.5) * resolution
    ys = origin[1] + (np.arange(height) + 0.5) * resolution
    x, y = np.meshgrid(xs, ys)

    def near(d):
        return np.abs(d) < 0.05

    inside = (x > -1.05) & (x < 4.05) & (y > -2.05) & (y < 2.05)
    wall = inside & (near(x + 1.0) | near(x - 4.0) | near(y + 2.0) | near(y - 2.0)
                     | (near(x - 1.0) & ((y <= 0.3) | (y >= 0.8))))
    states = np.full(x.shape, UNKNOWN, dtype=np.int8)
    states[inside] = FREE
    states[wall] = OCCUPIED
    return GridMap(states, resolution, origin)


def _downsample(grid, factor, start):
    """Coarser cells the way a SLAM grid fills them: a block is occupied if
    any fine cell is, else free if any is free. `start` offsets the blocks."""
    s = grid.states[start[1]:, start[0]:]
    h, w = (s.shape[0] // factor) * factor, (s.shape[1] // factor) * factor
    blocks = s[:h, :w].reshape(h // factor, factor, w // factor, factor).swapaxes(1, 2)
    out = np.full(blocks.shape[:2], UNKNOWN, dtype=np.int8)
    out[(blocks == FREE).any(axis=(2, 3))] = FREE
    out[(blocks == OCCUPIED).any(axis=(2, 3))] = OCCUPIED
    origin = (grid.origin[0] + start[0] * grid.resolution,
              grid.origin[1] + start[1] * grid.resolution)
    return GridMap(out, grid.resolution * factor, origin)


def _statuses(levels):
    return [level.status for level in levels]


def test_exact_copy_passes_every_level():
    reference = _room()
    assert _statuses(slam_check.evaluate(_room(), reference)) == ['pass'] * 3


def test_half_the_room_fails_coverage_and_skips_detail():
    reference = _room()
    learner = _room()
    x, _ = learner.cell_centres()
    learner.states[x > 1.5] = UNKNOWN
    levels = slam_check.evaluate(learner, reference)
    assert levels[0].status == 'pass'
    assert levels[1].status == 'fail'
    assert levels[1].numbers['coverage'] == pytest.approx(0.5, abs=0.05)
    # Doorway and wall numbers mean nothing on half a map.
    assert levels[2].status == 'skip'
    assert 'ผ่านด่าน 2 ก่อน' in slam_check.format_report(levels)


def test_coarse_cells_close_the_door_and_thicken_the_walls():
    reference = _room()
    # Blocks start one fine cell in, so block edges sit at y = 0.35, 0.6 and
    # 0.85: the block [0.6, 0.85) holds the upper jamb (y = 0.8), so the
    # doorway samples at y 0.6..0.7 are inside an occupied block, as they
    # are in a real 0.25 m SLAM map.
    coarse = _downsample(reference, 5, (1, 1))
    levels = slam_check.evaluate(coarse, reference)
    assert levels[1].status == 'pass'            # still explored
    assert levels[2].status == 'fail'
    assert levels[2].numbers['blocked'] > 0
    assert levels[2].numbers['ratio'] > slam_check.MAX_THICKNESS


def test_doorway_walled_shut_fails_detail():
    reference = _room()
    shut = _room()
    x, y = shut.cell_centres()
    shut.states[(np.abs(x - 1.0) < 0.05) & (y > 0.3) & (y < 0.8)] = OCCUPIED
    levels = slam_check.evaluate(shut, reference)
    assert levels[2].status == 'fail'
    assert levels[2].numbers['blocked'] > 0


def test_one_cell_of_jamb_smear_still_passes():
    """A correct map whose lower jamb reads one cell long (grid alignment)."""
    reference = _room()
    smeared = _room()
    x, y = smeared.cell_centres()
    # In the recorded reference the lower jamb's cells reach y ~0.35; a map
    # whose jamb reads one cell longer covers 0.40..0.45.
    smeared.states[(np.abs(x - 1.0) < 0.05) & (y < 0.45)] = OCCUPIED
    assert slam_check.evaluate(smeared, reference)[2].status == 'pass'


def test_missing_file_hint_covers_saving_and_the_topic():
    levels = slam_check.evaluate(None, _room(), missing=True)
    text = slam_check.format_report(levels)
    assert 'ไม่พบไฟล์แผนที่' in text
    # With the wrong LiDAR topic map_saver_cli fails and writes nothing, so a
    # missing file is also that symptom's report: it must still lead there.
    assert 'ros2 topic list' in text


def test_smeared_fine_map_gets_the_smear_hint_not_the_coarse_one():
    reference = _room()
    smeared = _room()
    x, y = smeared.cell_centres()
    # Walls doubled a few cells over, as when scans are placed off.
    for shift in (0.05, 0.10, 0.15):
        wall = reference.states_at(x - shift, y - shift) == OCCUPIED
        smeared.states[wall & (smeared.states != UNKNOWN)] = OCCUPIED
    text = slam_check.format_report(slam_check.evaluate(smeared, reference))
    assert 'บล็อกหยาบ' not in text
    assert 'ซ้อน' in text


def test_shifted_origin_same_room_still_passes():
    reference = _room()
    shifted = _room(origin=(-1.213, -2.187))
    assert _statuses(slam_check.evaluate(shifted, reference)) == ['pass'] * 3


def test_other_resolution_is_measured_in_metres():
    reference = _room()
    other = _room(resolution=0.07, origin=(-1.2, -2.2), width=78, height=63)
    levels = slam_check.evaluate(other, reference)
    assert levels[1].numbers['coverage'] > 0.95
    assert levels[2].numbers['blocked'] == 0


def test_no_map_fails_level_one_and_skips_the_rest():
    levels = slam_check.evaluate(None, _room())
    assert _statuses(levels) == ['fail', 'skip', 'skip']


def test_empty_map_fails_level_one():
    empty = _room()
    empty.states[:] = UNKNOWN
    assert _statuses(slam_check.evaluate(empty, _room())) == ['fail', 'skip', 'skip']


def test_map_round_trip(tmp_path):
    grid = _room()
    path = tmp_path / 'room.yaml'
    slam_check.save_map(grid, str(path))
    loaded = slam_check.load_map(str(path))
    assert loaded.resolution == pytest.approx(RES)
    assert loaded.origin == pytest.approx(ORIGIN)
    assert np.array_equal(loaded.states, grid.states)


def test_pgm_header_comment_is_skipped(tmp_path):
    # map_saver_cli writes a CREATOR comment line.
    path = tmp_path / 'm.pgm'
    path.write_bytes(b'P5\n# CREATOR: map_saver.cpp 0.050 m/pix\n3 2\n255\n'
                     + bytes([0, 205, 254, 254, 205, 0]))
    pixels = slam_check.read_pgm(str(path))
    assert pixels.shape == (2, 3)
    assert pixels[0].tolist() == [0, 205, 254]


def test_missing_image_raises_map_load_error(tmp_path):
    path = tmp_path / 'm.yaml'
    path.write_text('image: nowhere.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    with pytest.raises(slam_check.MapLoadError):
        slam_check.load_map(str(path))


def test_missing_yaml_raises_map_load_error(tmp_path):
    with pytest.raises(slam_check.MapLoadError):
        slam_check.load_map(str(tmp_path / 'nope.yaml'))


def test_report_hints_name_symptoms_not_parameters():
    reference = _room()
    text = slam_check.format_report(slam_check.evaluate(None, reference))
    assert 'ros2 topic list' in text
    coarse = _downsample(reference, 5, (1, 1))
    text = slam_check.format_report(slam_check.evaluate(coarse, reference))
    for parameter in ('resolution', 'max_laser_range', 'scan_topic'):
        assert parameter not in text
    assert 'ผ่านครบทุกด่าน' in slam_check.format_report(
        slam_check.evaluate(_room(), reference))
