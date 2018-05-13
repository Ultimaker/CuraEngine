"""
Data structure in slicing
Volumes
 -- layers
    -- parts
        -- outlines
"""


class Storage:
    def __init__(self):
        self._volumes = list()
        self.model_size_x = None
        self.model_size_y = None

    def get_volume(self, index):
        return self._volumes[index]

    def add_volume(self, volume):
        self._volumes.append(volume)


class Volume:
    def __init__(self):
        self._layers = list()

    def get_layer(self, index):
        return self._layers[index]

    def add_layer(self, layer):
        self._layers.append(layer)

    def get_layer_size(self):
        return len(self._layers)


class Layer:
    def __init__(self):
        self._parts = list()

    def get_part(self, index):
        return self._parts[index]

    def get_parts_size(self):
        return len(self._parts)

    def add_part(self, part):
        self._parts.append(part)


class Part:
    def __init__(self):
        self._polygons = list()

    def get_outline(self, index):
        return self._polygons[index]

    def add_outline(self, outline):
        self._polygons.append(outline)

    def get_outlines_size(self):
        return len(self._polygons)


class Polygon:
    def __init__(self):
        self._points_at_x = list()
        self._points_at_y = list()

    def get_point_at_x(self, index):
        return self._points_at_x[index]

    def get_all_points_at_x(self):
        return self._points_at_x

    def get_point_at_y(self, index):
        return self._points_at_y[index]

    def get_all_points_at_y(self):
        return self._points_at_y

    def add_point(self, x, y):
        self._points_at_x.append(x)
        self._points_at_y.append(y)

