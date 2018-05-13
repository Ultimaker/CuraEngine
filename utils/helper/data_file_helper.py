from __future__ import absolute_import, division, print_function, unicode_literals

import sys
import os.path
from model.storage import *

if 2 == sys.version_info[0]:
    text = unicode
else:
    text = str


class DataFileHelper:

    KEY_INDEX_VOLUME = 'volume index:'
    KEY_INDEX_LAYER = 'layer index:'
    KEY_INDEX_PART = 'part index:'
    KEY_INDEX_OUTLINE = 'outline index:'
    KEY_SIZE_MODEL = 'model size:'

    def __init__(self):
        pass

    @staticmethod
    def load_part_file(file_path):
        assert isinstance(file_path, text)
        if not os.path.isfile(file_path):
            return None, "file doesn't exist!"
        storage = Storage()
        # initialize index
        volume_index = None
        layer_index = None
        part_index = None
        outline_index = None

        current_volume = None
        current_layer = None
        currnet_part = None
        model_size_x = None
        model_size_y = None

        with open(file_path, 'r') as part_file:
            for line in part_file:
                if line.startswith(DataFileHelper.KEY_SIZE_MODEL):
                    # read model size
                    str_list = line[len(DataFileHelper.KEY_SIZE_MODEL):].split()
                    num_list = [int(e) for e in str_list]
                    model_size_x = num_list[0]
                    model_size_y = num_list[1]
                elif line.startswith(DataFileHelper.KEY_INDEX_VOLUME):
                    volume_index = int(line[len(DataFileHelper.KEY_INDEX_VOLUME):])
                    current_volume = Volume()
                    storage.add_volume(current_volume)
                elif line.startswith(DataFileHelper.KEY_INDEX_LAYER):
                    layer_index = int(line[len(DataFileHelper.KEY_INDEX_LAYER):])
                    current_layer = Layer()
                    current_volume.add_layer(current_layer)
                elif line.startswith(DataFileHelper.KEY_INDEX_PART):
                    part_index = int(line[len(DataFileHelper.KEY_INDEX_PART):])
                    current_part = Part()
                    current_layer.add_part(current_part)
                elif line.startswith(DataFileHelper.KEY_INDEX_OUTLINE):
                    outline_index = int(line[len(DataFileHelper.KEY_INDEX_OUTLINE):])
                else:
                    # read point list into polygon
                    str_list = line.split()
                    num_list = [int(e) for e in str_list]
                    polygon = Polygon()
                    if len(num_list) % 2 != 0:
                        raise RuntimeError('Invalid number of the point list -- %d!')
                    for point_index in range(int(len(num_list)/2)):
                        polygon.add_point(num_list[2 * point_index], num_list[2 * point_index + 1])
                    current_part.add_outline(polygon)

            storage.model_size_y = model_size_y
            storage.model_size_x = model_size_x

        return storage, 'succeeded in loading part data!'


