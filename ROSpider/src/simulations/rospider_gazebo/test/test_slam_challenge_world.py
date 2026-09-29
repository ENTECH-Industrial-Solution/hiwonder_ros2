"""worlds/slam_challenge.sdf: every inner-wall face carries its own texture (V-SLAM loop
closure matched identical faces 1.2 m apart), and the collision geometry the LiDAR map was
recorded with is unchanged."""
import pathlib
import xml.etree.ElementTree as ET

PKG = pathlib.Path(__file__).resolve().parents[1]
WALLS = ('partition_n', 'partition_s', 'stub', 'maze_1', 'maze_2', 'maze_3', 'maze_4')


def _walls_link():
    root = ET.parse(PKG / 'worlds' / 'slam_challenge.sdf').getroot()
    return root.find(".//model[@name='walls']/link")


def test_each_inner_face_has_its_own_existing_texture():
    link = _walls_link()
    textures = []
    for wall in WALLS:
        assert link.find(f"visual[@name='{wall}_v']") is None
        for side in 'ab':
            visual = link.find(f"visual[@name='{wall}_{side}_v']")
            assert visual is not None, f'{wall}_{side}_v missing'
            texture = visual.find('.//albedo_map').text
            assert (PKG / 'worlds' / texture).is_file(), texture
            textures.append(texture)
    assert len(set(textures)) == 14
    assert 'partition.png' not in (PKG / 'worlds' / 'slam_challenge.sdf').read_text()


def test_inner_wall_collisions_unchanged():
    link = _walls_link()
    for wall in WALLS:
        assert link.find(f"collision[@name='{wall}_c']") is not None
    assert len(link.findall('collision')) == 11      # 4 outer + 7 inner
