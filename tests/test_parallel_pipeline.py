import copy
import numpy as np
from omniavoid.dataset import render_camera
from omniavoid.geometry import Camera
from omniavoid.runtime import process_images
from omniavoid.stereo import StereoPipeline


def test_four_pair_workers_preserve_depth_and_unknown_outputs(cfg):
    images={d['name']:render_camera(Camera.from_dict(d)) for d in cfg['cameras']}
    serial_cfg=copy.deepcopy(cfg)
    serial_cfg['stereo'].update(pair_workers=1,opencv_threads=1)
    parallel_cfg=copy.deepcopy(cfg)
    parallel_cfg['stereo'].update(pair_workers=4,opencv_threads=1)
    serial=StereoPipeline(serial_cfg)
    try:
        a,pa,sa=process_images(serial,serial_cfg,images)
    finally:
        serial.close()
    parallel=StereoPipeline(parallel_cfg)
    try:
        b,pb,sb=process_images(parallel,parallel_cfg,images)
    finally:
        parallel.close()
    assert a['distances_cm']==b['distances_cm']
    assert a['support_counts']==b['support_counts']
    assert np.array_equal(pa,pb)
    assert np.array_equal(sa,sb)
