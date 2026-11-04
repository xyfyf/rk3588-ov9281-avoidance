"""Repeatable full-resolution compute profiling. No camera or FC connection."""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import platform
import time
import cv2
import numpy as np
from omniavoid.config import load_config
from omniavoid.dataset import render_camera
from omniavoid.geometry import Camera
from omniavoid.runtime import process_images
from omniavoid.stereo import StereoPipeline


def measure(cfg, images, iterations):
    pipeline = StereoPipeline(cfg)
    try:
        for _ in range(5):
            process_images(pipeline, cfg, images)
        timings, matching, serialization, coverage = [], [], [], []
        from omniavoid.protocol import Snapshot, obstacle_message
        from pymavlink.dialects.v20 import common as mav
        encoder = mav.MAVLink(None)
        for _ in range(iterations):
            start = time.perf_counter()
            result, _, _ = process_images(pipeline, cfg, images)
            serialized = time.perf_counter()
            message = obstacle_message(Snapshot(time.monotonic_ns(), time.time_ns(), tuple(result['distances_cm'])), cfg['range']['max_m'])
            message.pack(encoder)
            serialization.append((time.perf_counter() - serialized) * 1000)
            timings.append((time.perf_counter() - start) * 1000)
            matching.append(result['stereo_ms'])
            coverage.append(result['valid_sectors'])
        a = np.array(timings)
        return {'iterations':iterations, 'median_ms':float(np.median(a)),
                'p95_ms':float(np.percentile(a,95)), 'p99_ms':float(np.percentile(a,99)),
                'max_ms':float(a.max()), 'mean_compute_rate_hz':float(1000/a.mean()),
                'over_50ms_percent':float(np.mean(a>50)*100),
                'over_100ms_percent':float(np.mean(a>100)*100),
                'stereo_p50_ms':float(np.median(matching)),
                'aggregation_overhead_p50_ms':float(np.median(a-np.array(matching)-np.array(serialization))),
                'packet_encode_p50_ms':float(np.median(serialization)),
                'valid_sectors_min':min(coverage), 'timings_ms':timings}
    finally:
        pipeline.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='configs/rig.example.json')
    parser.add_argument('--iterations', type=int, default=150)
    parser.add_argument('--profiles',nargs='+',default=['1:4','2:2','4:1','1:1'],
                        help='Pairs of camera-pair workers:OpenCV threads; e.g. --profiles 4:1')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.iterations < 20:
        parser.error('At least 20 iterations are required')
    cfg = load_config(args.config)
    print('Rendering full sensor-resolution synthetic inputs; rendering excluded from timing', flush=True)
    images = {d['name']:render_camera(Camera.from_dict(d)) for d in cfg['cameras']}
    profiles = []
    try:
        selected=[tuple(map(int,item.split(':'))) for item in args.profiles]
        if any(len(p)!=2 or min(p)<1 or max(p)>32 for p in selected):
            raise ValueError('Invalid profile')
    except ValueError:
        parser.error('Each profile must be two positive integers <=32, such as 4:1')
    for workers, threads in selected:
        candidate = copy.deepcopy(cfg)
        candidate['stereo'].update(pair_workers=workers,opencv_threads=threads)
        result = measure(candidate, images, args.iterations)
        result.update(pair_workers=workers,opencv_threads=threads)
        profiles.append(result)
        print(json.dumps({k:v for k,v in result.items() if k!='timings_ms'}), flush=True)
    report = {'platform':platform.platform(),'python':platform.python_version(),'opencv':cv2.__version__,
              'machine':platform.machine(), 'sensor_size':[cfg['cameras'][0]['width'],cfg['cameras'][0]['height']],
              'virtual_size':[cfg['stereo']['width'],cfg['stereo']['height']],
              'profiles':profiles,'synthetic_input':True,'gpu_used':False,
              'excludes':['image rendering','camera exposure/readout/transport','disk input','flight-controller response','vehicle dynamics'],
              'warning':'Static image compute benchmark, not a guarantee of control or RK3588 performance. Concurrent PC load is uncontrolled.'}
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    Path(args.output).write_text(json.dumps(report,indent=2),encoding='utf-8')


if __name__ == '__main__':
    main()
