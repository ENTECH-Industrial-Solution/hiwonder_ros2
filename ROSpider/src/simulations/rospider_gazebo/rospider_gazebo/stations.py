"""AprilTag station geometry and its SDF, with no OpenCV in sight.

tags.py re-exports everything here, so callers keep saying tags.TAG_SIZE and
tags.station_sdf(); this module exists because mini_game.launch.py needs
station_sdf() and importing cv2 inside `ros2 launch` sets Qt's plugin path
for every child process, which crashes the Gazebo GUI.

Tag frame convention: x right, y up, z out of the tag face toward the viewer.
"""

import numpy as np

# A 36h11 marker is 10 modules across including its own black border.
TAG_MODULES = 10

# Edge of the black square in metres, excluding the quiet zone. Must match the
# physical board through BOARD_FACE below, or every distance solve_tag_pose
# reports is wrong by the ratio between them.
TAG_SIZE = 0.15

# White margin around the marker, in modules. The detector needs light around
# the tag and finds nothing at all without it -- the most common way a rendered
# tag fails, and it fails silently.
QUIET_MODULES = 2

# The board carries the whole texture, marker plus quiet zone, so its face is
# larger than the tag by exactly that ratio. Derived rather than chosen: a
# hand-picked board size would make the black square some other size and put a
# constant scale error into every pose.
BOARD_FACE = TAG_SIZE * (TAG_MODULES + 2 * QUIET_MODULES) / TAG_MODULES


def board_face(tag_size=TAG_SIZE):
    """Board edge for a tag of `tag_size`; BOARD_FACE is the default."""
    return tag_size * (TAG_MODULES + 2 * QUIET_MODULES) / TAG_MODULES

# Station model, all in the model frame, which faces +x: a robot approaching
# from +x sees the tag.
#   - pedestal, identical to models/pick_pedestal so its top face is at
#     z = 0.08 and pick_place.yaml's drop_slots z of 0.135 lands a cube on it
#   - a post holding the board up
#   - the board, BOARD_FACE square, behind the pedestal
PEDESTAL_SIZE = (0.14, 0.22, 0.08)
BOARD_THICKNESS = 0.01
BOARD_OFFSET_X = -0.12
TAG_CENTRE_HEIGHT = 0.25
POST_SIZE = (0.03, 0.03, TAG_CENTRE_HEIGHT - BOARD_FACE / 2.0)

# The mini game's colour marker: a square panel above the tag board, on the
# same post. Vertical so a level camera reads its colour from metres away.
MARKER_SIZE = 0.20
MARKER_CENTRE_HEIGHT = TAG_CENTRE_HEIGHT + BOARD_FACE / 2.0 + 0.02 + MARKER_SIZE / 2.0


def tag_to_pedestal_top():
    """The pedestal top centre, expressed in the tag frame, metres.

    Unused by the detector; this is the natural home for the number and a
    consumer that wants to place an object on the station's pedestal needs it.

    The model faces +x, so tag x = model +y, tag y = model +z, tag z = model
    +x. The pedestal top centre is (0, 0, PEDESTAL_SIZE[2]) in the model frame
    and the tag origin is (BOARD_OFFSET_X, 0, TAG_CENTRE_HEIGHT).
    """
    return np.array([0.0,
                     PEDESTAL_SIZE[2] - TAG_CENTRE_HEIGHT,
                     -BOARD_OFFSET_X])


def station_sdf(tag_id, marker_rgba=None, tag_size=TAG_SIZE):
    """The SDF text for tag_station_<id>.

    Static, box geometry only, no meshes -- the same shape as
    models/pick_pedestal, and for the same reason: gazebo.launch.py strips
    mesh collisions because the robot's own STLs are 1.5M triangles, and
    scenery that needs the same treatment would be a trap.

    The model faces +x: a robot approaching from +x sees the tag.

    `marker_rgba` adds the mini game's colour panel, MARKER_SIZE square,
    above the tag on a taller post: it says which cube belongs on this
    station, and it is vertical rather than painted on the floor so a level
    camera can read its colour from metres away. None gives exactly the
    committed tag_station_<id> models.

    `tag_size` scales the board (and lifts the marker with it). The mini
    game uses 0.30 m tags: at its 2 m survey distance a 0.15 m tag is 30 px
    wide, three pixels per module, and aruco finds nothing. The same
    texture is stretched; apriltag_detect must be told the size too.
    """
    px, py, pz = PEDESTAL_SIZE
    ox, oy, _ = POST_SIZE
    face = board_face(tag_size)
    oz = TAG_CENTRE_HEIGHT - face / 2.0
    post_x = BOARD_OFFSET_X
    board = f'{BOARD_THICKNESS:.6f} {face:.6f} {face:.6f}'
    marker = ''
    if marker_rgba is not None:
        marker_height = TAG_CENTRE_HEIGHT + face / 2.0 + 0.02 + MARKER_SIZE / 2.0
        oz = marker_height              # the post now carries the panel too
        # The taller post crosses the tag face, and centred on the board it
        # sticks out 1 cm in front of it -- a grey stripe down the middle of
        # the tag that aruco cannot see through. Behind the board instead.
        post_x = BOARD_OFFSET_X - BOARD_THICKNESS / 2.0 - ox / 2.0
        rgba = ' '.join(f'{v:.3f}' for v in marker_rgba)
        # Half-strength emissive on top of the diffuse colour: a lit sign
        # that keeps its hue. A plain diffuse panel facing away from the sun
        # rendered at saturation 32 where the cubes sit at 60-95, and a
        # full-strength emissive one clipped to near white (saturation 20).
        # The marker is meant to be read, not to be the colour puzzle.
        glow = ' '.join(f'{v * 0.5:.3f}' for v in marker_rgba[:3]) + ' 1'
        marker = f'''
      <visual name="marker_v">
        <pose>{BOARD_OFFSET_X:.6f} 0 {marker_height:.6f} 0 0 0</pose>
        <geometry><box><size>{BOARD_THICKNESS:.6f} {MARKER_SIZE:.6f} {MARKER_SIZE:.6f}</size></box></geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
          <emissive>{glow}</emissive>
        </material>
      </visual>'''
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <!-- Generated by tools/make_tag_textures.py. Edit that, not this. -->
  <model name="tag_station_{tag_id}">
    <static>true</static>
    <link name="link">
      <collision name="pedestal_c">
        <pose>0 0 {pz / 2:.6f} 0 0 0</pose>
        <geometry><box><size>{px:.6f} {py:.6f} {pz:.6f}</size></box></geometry>
      </collision>
      <visual name="pedestal_v">
        <pose>0 0 {pz / 2:.6f} 0 0 0</pose>
        <geometry><box><size>{px:.6f} {py:.6f} {pz:.6f}</size></box></geometry>
        <material>
          <ambient>0.6 0.6 0.62 1</ambient>
          <diffuse>0.6 0.6 0.62 1</diffuse>
        </material>
      </visual>
      <collision name="post_c">
        <pose>{post_x:.6f} 0 {oz / 2:.6f} 0 0 0</pose>
        <geometry><box><size>{ox:.6f} {oy:.6f} {oz:.6f}</size></box></geometry>
      </collision>
      <visual name="post_v">
        <pose>{post_x:.6f} 0 {oz / 2:.6f} 0 0 0</pose>
        <geometry><box><size>{ox:.6f} {oy:.6f} {oz:.6f}</size></box></geometry>
        <material>
          <ambient>0.3 0.3 0.32 1</ambient>
          <diffuse>0.3 0.3 0.32 1</diffuse>
        </material>
      </visual>
      <collision name="board_c">
        <pose>{BOARD_OFFSET_X:.6f} 0 {TAG_CENTRE_HEIGHT:.6f} 0 0 0</pose>
        <geometry><box><size>{board}</size></box></geometry>
      </collision>
      <visual name="board_v">
        <pose>{BOARD_OFFSET_X:.6f} 0 {TAG_CENTRE_HEIGHT:.6f} 0 0 0</pose>
        <geometry><box><size>{board}</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal>
            <albedo_map>../../worlds/textures/tag_{tag_id}.png</albedo_map>
            <roughness>0.9</roughness>
            <metalness>0</metalness>
          </metal></pbr>
        </material>
      </visual>{marker}
    </link>
  </model>
</sdf>
'''
