"""Build a sensor_msgs/Image without cv_bridge.cv2_to_imgmsg.

On this dev machine a pip-installed opencv-python (5.0.0) shadows the apt
OpenCV that cv_bridge's C++ extension was compiled against, so cv2_to_imgmsg
raises "KeyError: 16" while looking up the encoding's numpy dtype from the
wrong module's constants. imgmsg_to_cv2, used for incoming images, is
unaffected.

A contiguous bgr8 or mono8 frame needs no cv_bridge machinery to serialize, so
it is built here instead of depending on the shadowed cv2 import resolving
right. Every node in this package that publishes an overlay uses this.
"""

import numpy as np
from sensor_msgs.msg import Image

_CHANNELS = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}


def to_image_msg(frame, header, encoding='bgr8'):
    """A sensor_msgs/Image carrying `frame`, stamped with `header`."""
    if encoding not in _CHANNELS:
        raise ValueError(f'unsupported encoding {encoding!r}; '
                         f'expected one of {sorted(_CHANNELS)}')
    msg = Image()
    msg.header = header
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * _CHANNELS[encoding]
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg
