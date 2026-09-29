"""Reading interfaces/ObjectInfo boxes.

ObjectInfo.box is an unbounded int32[], and this workspace publishes it in
two shapes: [x1, y1, x2, y2] from scripts/color_detect.py and a plain YOLO
model, and eight numbers -- the four corners of an oriented box -- from an
OBB model, which is what Hiwonder's own competition/yolo_node.py emits. Both
have to be read the same way, or an OBB detection silently gives the midpoint
of two corners as its centre.

Pure Python, tested in test/test_detections.py.
"""


def box_centroid(box):
    """(u, v, area) of a detection box, or None if `box` is neither shape."""
    if len(box) == 4:
        x1, y1, x2, y2 = box
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0, abs(x2 - x1) * abs(y2 - y1)
    if len(box) == 8:
        xs, ys = box[0::2], box[1::2]
        return (sum(xs) / 4.0, sum(ys) / 4.0,
                (max(xs) - min(xs)) * (max(ys) - min(ys)))
    return None


def box_corners(box):
    """The axis-aligned (x1, y1, x2, y2) around a box, or None.

    An oriented box is reduced to the rectangle that contains it, which is
    what a demo needs to draw one rectangle over a detection.
    """
    if len(box) == 4:
        x1, y1, x2, y2 = box
        return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)
    if len(box) == 8:
        xs, ys = box[0::2], box[1::2]
        return min(xs), min(ys), max(xs), max(ys)
    return None


def largest(objects):
    """The detection with the biggest box, or None for an empty list."""
    best = None
    for obj in objects:
        centroid = box_centroid(obj.box)
        if centroid is None:
            continue
        if best is None or centroid[2] > best[1][2]:
            best = (obj, centroid)
    return best
