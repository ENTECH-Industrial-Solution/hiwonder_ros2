"""A minimal Wavefront .obj reader for the AR demo.

Ported from example/opencv_example/include/obj_loader.py, kept to the parts
scripts/ar_view.py uses: vertices (optionally carrying a per-vertex colour in
columns 4-6, which is how Hiwonder's models store theirs) and faces.

Stdlib only.
"""


class OBJ:
    """The vertices and faces of one .obj file."""

    def __init__(self, filename, swapyz=False):
        self.vertices = []
        self.normals = []
        self.faces = []

        with open(filename, 'r') as handle:
            for line in handle:
                if line.startswith('#'):
                    continue
                values = line.split()
                if not values:
                    continue
                if values[0] == 'v':
                    vertex = [float(v) for v in values[1:]]
                    if swapyz and len(vertex) > 3:
                        vertex = [vertex[0], vertex[2], vertex[1],
                                  vertex[3], vertex[4], vertex[5]]
                    elif swapyz:
                        vertex = [vertex[0], vertex[2], vertex[1]]
                    self.vertices.append(vertex)
                elif values[0] == 'vn':
                    normal = [float(v) for v in values[1:4]]
                    if swapyz:
                        normal = [normal[0], normal[2], normal[1]]
                    self.normals.append(normal)
                elif values[0] == 'f':
                    face, norms = [], []
                    for value in values[1:]:
                        parts = value.split('/')
                        face.append(int(parts[0]))
                        norms.append(int(parts[2])
                                     if len(parts) >= 3 and parts[2] else 0)
                    self.faces.append((face, norms))
