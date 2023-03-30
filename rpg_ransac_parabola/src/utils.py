import numpy as np
import enum

class BboxKeys(enum.IntEnum):
    time = 0
    width = 1
    height = 2
    center_x = 3
    center_y = 4
    angle = 5
    image_idx = 6

def load_bboxes(path):
    data = np.genfromtxt(path, dtype="float64")
    data[:,BboxKeys.time] -= data[0,BboxKeys.time]
    data[:,BboxKeys.time] /= 1e9
    return data

def filter_to_max_bboxes(bboxes):
    image_idx_unique = np.unique(bboxes[:, BboxKeys.image_idx])

    max_bboxes = []
    for idx in image_idx_unique:
        sub_data = bboxes[bboxes[:, BboxKeys.image_idx] == idx]
        argmax = compute_area_from_bboxes(sub_data).argmax(-1)
        max_bbox = sub_data[argmax]
        max_bboxes.append(max_bbox)
    bboxes = np.stack(max_bboxes)

    return bboxes

def filter_bboxes_to_interval(bboxes, t0, t1):
    bboxes = bboxes[(t0 < bboxes[:,BboxKeys.time]) & (t1 > bboxes[:,BboxKeys.time])]
    bboxes[:,BboxKeys.time] -= bboxes[0,BboxKeys.time]
    return bboxes

def compute_area_from_bboxes(bboxes):
    return bboxes[:, [BboxKeys.width, BboxKeys.height]].prod(-1)
